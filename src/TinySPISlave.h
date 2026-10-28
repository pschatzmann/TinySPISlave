/**
 * @file TinySPISlave.h
 * @brief One SPI-slave API for ESP32, STM32, RP2040 and RP2350 (header only).
 *
 * Usage:
 * @code
 *   #include "TinySPISlave.h"
 *   TinySPISlave spiSlave;
 *   void setup() { spiSlave.begin(SCK, MISO, MOSI, CS); }
 *   void loop()  { while (spiSlave.available()) Serial.println(spiSlave.read()); }
 * @endcode
 *
 * The class is a Stream: bytes clocked in by the master are read with
 * read()/available(), bytes the master should clock out are queued with
 * write(). Optionally a callback is invoked once per CS-delimited frame.
 * For register style protocols see TinySPISlaveRegisters.h (RegisterBank).
 *
 * Backends (selected automatically):
 *  - ESP32      : ESP-IDF spi_slave driver (DMA, background task)
 *  - RP2040/2350: PL022 slave mode via pico-sdk (interrupt driven)
 *  - STM32      : STM32duino spi_init() in peripheral mode + LL (interrupt driven)
 */
#pragma once

#include <Arduino.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define TINYSPISLAVE_VERSION "0.1.0"

namespace tinyspi {

/// Called once per completed frame (CS low -> high) with the number of bytes
/// received in it. The bytes themselves are available via read().
typedef void (*FrameCallback)(size_t len, void* ctx);

struct Config {
  int8_t sck = -1;
  int8_t miso = -1;
  int8_t mosi = -1;
  int8_t cs = -1;          ///< chip select is mandatory on all platforms
  uint8_t mode = 0;        ///< SPI mode 0..3 (CPOL/CPHA)
  size_t rxBufferSize = 256;  ///< bytes
  size_t txBufferSize = 256;  ///< bytes
  // --- ESP32 only (ignored elsewhere) ---
  size_t maxFrameSize = 64;   ///< bytes per DMA transaction; longer frames are split
  uint8_t slots = 2;          ///< transactions kept queued (>=2 avoids lost frames)
  uint8_t idleByte = 0x00;    ///< sent when no tx data is queued
  uint8_t spiHost = 2;        ///< 2 = SPI2_HOST (HSPI/FSPI), 3 = SPI3_HOST (VSPI)
};

namespace detail {

/// Lock-free single-producer / single-consumer byte ring (ISR safe).
class ByteRing {
 public:
  ~ByteRing() { free(buf_); }
  bool init(size_t capacity) {
    free(buf_);
    size_ = capacity + 1;
    head_ = 0;
    tail_ = 0;
    buf_ = (uint8_t*)malloc(size_);
    return buf_ != nullptr;
  }
  void release() {
    free(buf_);
    buf_ = nullptr;
    size_ = 0;
    head_ = 0;
    tail_ = 0;
  }
  size_t available() const {
    size_t h = head_, t = tail_;
    return h >= t ? h - t : h + size_ - t;
  }
  size_t space() const { return size_ ? size_ - 1 - available() : 0; }
  bool push(uint8_t b) {
    size_t h = head_;
    size_t n = (h + 1 == size_) ? 0 : h + 1;
    if (!buf_ || n == tail_) return false;
    buf_[h] = b;
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
    head_ = n;
    return true;
  }
  int pop() {
    size_t t = tail_;
    if (!buf_ || t == head_) return -1;
    uint8_t b = buf_[t];
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
    tail_ = (t + 1 == size_) ? 0 : t + 1;
    return b;
  }
  int peek() const {
    size_t t = tail_;
    return (!buf_ || t == head_) ? -1 : buf_[t];
  }

 private:
  uint8_t* buf_ = nullptr;
  size_t size_ = 0;
  volatile size_t head_ = 0;
  volatile size_t tail_ = 0;
};

/// Interrupt-safe critical section (short copies only).
class Critical {
 public:
#if defined(ARDUINO_ARCH_ESP32)
  Critical() { portENTER_CRITICAL(&mux()); }
  ~Critical() { portEXIT_CRITICAL(&mux()); }
 private:
  static portMUX_TYPE& mux() {
    static portMUX_TYPE m = portMUX_INITIALIZER_UNLOCKED;
    return m;
  }
#elif defined(ARDUINO_ARCH_RP2040)
  Critical() : s_(save_and_disable_interrupts()) {}
  ~Critical() { restore_interrupts(s_); }
 private:
  uint32_t s_;
#else
  Critical() : s_(__get_PRIMASK()) { __disable_irq(); }
  ~Critical() { __set_PRIMASK(s_); }
 private:
  uint32_t s_;
#endif
};

/// State shared between the user-facing class and the platform backend.
struct Shared {
  ByteRing rx;  // backend produces, user (or frame hook) consumes
  ByteRing tx;  // user (or frame hook) produces, backend consumes
  volatile size_t frameBytes = 0;
  volatile uint32_t rxOverflow = 0;
  volatile uint32_t frameOverflow = 0;
  FrameCallback cb = nullptr;    // deferred, dispatched by poll()
  void* cbCtx = nullptr;
  FrameCallback hook = nullptr;  // immediate, runs in ISR / backend task
  void* hookCtx = nullptr;

  void pushRx(uint8_t b) {
    if (rx.push(b)) {
      frameBytes = frameBytes + 1;
    } else {
      rxOverflow = rxOverflow + 1;
    }
  }

  /// Backend: the master released CS and all bytes are in the rx ring.
  void frameEnd() {
    size_t n = frameBytes;
    frameBytes = 0;
    if (!n) return;
    if (hook) hook(n, hookCtx);
    if (cb) {
      uint8_t h = lensHead_, nx = (h + 1) % kLens;
      if (nx == lensTail_) {
        frameOverflow = frameOverflow + 1;
      } else {
        lens_[h] = n > 0xFFFF ? 0xFFFF : (uint16_t)n;
        __atomic_thread_fence(__ATOMIC_SEQ_CST);
        lensHead_ = nx;
      }
    }
  }

  /// User side: length of the oldest finished frame, or -1.
  int popFrame() {
    uint8_t t = lensTail_;
    if (t == lensHead_) return -1;
    int n = lens_[t];
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
    lensTail_ = (t + 1) % kLens;
    return n;
  }

  void resetFrames() {
    lensHead_ = 0;
    lensTail_ = 0;
    frameBytes = 0;
  }

 private:
  static const uint8_t kLens = 16;
  uint16_t lens_[kLens];
  volatile uint8_t lensHead_ = 0;
  volatile uint8_t lensTail_ = 0;
};

}  // namespace detail
}  // namespace tinyspi

// ---- platform backend: defines tinyspi::detail::Backend ----
#if defined(ARDUINO_ARCH_ESP32)
#include "TinySPISlaveEsp32.h"
#elif defined(ARDUINO_ARCH_RP2040) && !defined(ARDUINO_ARCH_MBED)
#include "TinySPISlaveRp2040.h"
#elif defined(ARDUINO_ARCH_STM32)
#include "TinySPISlaveStm32.h"
#else
#error "TinySPISlave: unsupported platform (ESP32, STM32, RP2040/RP2350 with arduino-pico)"
#endif

class TinySPISlave : public Stream {
 public:
  typedef tinyspi::Config Config;
  typedef tinyspi::FrameCallback FrameCallback;

  ~TinySPISlave() { end(); }

  bool begin(int sck, int miso, int mosi, int cs, uint8_t mode = 0) {
    Config cfg;
    cfg.sck = sck;
    cfg.miso = miso;
    cfg.mosi = mosi;
    cfg.cs = cs;
    cfg.mode = mode;
    return begin(cfg);
  }

  bool begin(const Config& cfg) {
    end();
    if (cfg.sck < 0 || cfg.miso < 0 || cfg.mosi < 0 || cfg.cs < 0 || cfg.mode > 3) return false;
    if (!s_.rx.init(cfg.rxBufferSize) || !s_.tx.init(cfg.txBufferSize)) {
      end();
      return false;
    }
    s_.resetFrames();
    s_.rxOverflow = 0;
    s_.frameOverflow = 0;
    running_ = backend_.begin(cfg, &s_);
    if (!running_) end();
    return running_;
  }

  void end() {
    if (running_) backend_.end();
    running_ = false;
    s_.rx.release();
    s_.tx.release();
  }

  bool isRunning() const { return running_; }

  /// Per-frame callback, delivered from poll() i.e. in your loop() context.
  /// Up to 16 finished frames are queued between poll() calls.
  void onFrame(FrameCallback cb, void* ctx = nullptr) {
    s_.cb = cb;
    s_.cbCtx = ctx;
  }

  /// Per-frame callback that runs immediately when CS is released: in the
  /// interrupt (RP2040/STM32) or the backend task (ESP32). Keep it short, no
  /// Serial/heap. The rx bytes of the frame are in the rx buffer; if you read()
  /// or write() from the hook, do not do so from loop() as well.
  /// The RegisterMap is built on this.
  void onFrameImmediate(FrameCallback cb, void* ctx = nullptr) {
    s_.hook = cb;
    s_.hookCtx = ctx;
  }

  /// Dispatch queued onFrame() callbacks. Call from loop().
  void poll() {
    int n;
    while ((n = s_.popFrame()) >= 0) {
      if (s_.cb) s_.cb((size_t)n, s_.cbCtx);
    }
  }

  /// Drop queued, not yet shifted-out tx bytes (not those already in the
  /// hardware FIFO: up to 8 on RP2040, 1-4 on STM32). Call from the immediate
  /// hook or while the master is idle.
  void flushTx() { while (s_.tx.pop() >= 0) {} }

  // ---- receive (master -> slave) ----
  int available() override { return (int)s_.rx.available(); }
  int read() override { return s_.rx.pop(); }
  int peek() override { return s_.rx.peek(); }
  size_t read(uint8_t* data, size_t len) {
    size_t n = 0;
    while (n < len) {
      int b = s_.rx.pop();
      if (b < 0) break;
      data[n++] = (uint8_t)b;
    }
    return n;
  }

  // ---- transmit (slave -> master) ----
  using Print::write;
  size_t write(uint8_t b) override {
    if (!running_ || !s_.tx.push(b)) return 0;
    backend_.kickTx();
    return 1;
  }
  size_t write(const uint8_t* data, size_t len) override {
    size_t n = 0;
    if (!running_) return 0;
    while (n < len && s_.tx.push(data[n])) n++;
    backend_.kickTx();
    return n;
  }
  int availableForWrite() override { return (int)s_.tx.space(); }
  void flush() override {}

  /// Bytes dropped because the rx buffer was full.
  uint32_t rxOverflows() const { return s_.rxOverflow; }
  /// Frames dropped from the onFrame() queue because poll() was too slow.
  uint32_t frameOverflows() const { return s_.frameOverflow; }

 private:
  tinyspi::detail::Shared s_;
  tinyspi::detail::Backend backend_;
  bool running_ = false;
};

#include "TinySPISlaveRegisters.h"
