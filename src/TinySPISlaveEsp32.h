// ESP32 backend: ESP-IDF spi_slave driver, DMA, one background task.
#pragma once

#include <driver/gpio.h>
#include <driver/spi_slave.h>
#include <esp_heap_caps.h>

#ifndef TINYSPISLAVE_ESP32_TASK_STACK
#define TINYSPISLAVE_ESP32_TASK_STACK 4096
#endif
#ifndef TINYSPISLAVE_ESP32_TASK_PRIORITY
#define TINYSPISLAVE_ESP32_TASK_PRIORITY 5
#endif

namespace tinyspi {
namespace detail {

class Backend {
 public:
  bool begin(const Config& cfg, Shared* s) {
    s_ = s;
    idle_ = cfg.idleByte;
    slots_ = cfg.slots < 1 ? 1 : (cfg.slots > kMaxSlots ? kMaxSlots : cfg.slots);
    frame_ = (cfg.maxFrameSize + 3) & ~(size_t)3;  // DMA wants 32-bit multiples
    if (frame_ == 0) frame_ = 4;
    host_ = (spi_host_device_t)((cfg.spiHost == 3 ? 3 : 2) - 1);

    for (uint8_t i = 0; i < slots_; i++) {
      tx_[i] = (uint8_t*)heap_caps_malloc(frame_, MALLOC_CAP_DMA);
      rx_[i] = (uint8_t*)heap_caps_malloc(frame_, MALLOC_CAP_DMA);
      if (!tx_[i] || !rx_[i]) {
        freeBuffers();
        return false;
      }
    }

    // Same pull-ups as the ESP-IDF slave examples
    gpio_set_pull_mode((gpio_num_t)cfg.sck, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode((gpio_num_t)cfg.mosi, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode((gpio_num_t)cfg.cs, GPIO_PULLUP_ONLY);

    spi_bus_config_t bus = {};
    bus.mosi_io_num = cfg.mosi;
    bus.miso_io_num = cfg.miso;
    bus.sclk_io_num = cfg.sck;
    bus.quadwp_io_num = -1;
    bus.quadhd_io_num = -1;
    spi_slave_interface_config_t slv = {};
    slv.spics_io_num = cfg.cs;
    slv.mode = cfg.mode;
    slv.queue_size = slots_;
    slv.flags = 0;
    if (spi_slave_initialize(host_, &bus, &slv, SPI_DMA_CH_AUTO) != ESP_OK) {
      freeBuffers();
      return false;
    }

    run_ = true;
    taskDone_ = false;
    if (xTaskCreate(&Backend::taskEntry, "TinySPISlave", TINYSPISLAVE_ESP32_TASK_STACK, this,
                    TINYSPISLAVE_ESP32_TASK_PRIORITY, nullptr) != pdPASS) {
      spi_slave_free(host_);
      freeBuffers();
      return false;
    }
    return true;
  }

  void end() {
    run_ = false;
    while (!taskDone_) delay(1);  // task polls with a 50 ms timeout
    spi_slave_free(host_);
    freeBuffers();
  }

  /// tx data is picked up when the next transaction is queued.
  void kickTx() {}

 private:
  static const uint8_t kMaxSlots = 4;

  static void taskEntry(void* p) {
    static_cast<Backend*>(p)->run();
    vTaskDelete(nullptr);
  }

  void queue(uint8_t i) {
    // Consume-at-queue: bytes not clocked out by a short frame are dropped.
    size_t n = 0;
    while (n < frame_) {
      int b = s_->tx.pop();
      if (b < 0) break;
      tx_[i][n++] = (uint8_t)b;
    }
    memset(tx_[i] + n, idle_, frame_ - n);
    memset(&t_[i], 0, sizeof(t_[i]));
    t_[i].length = frame_ * 8;
    t_[i].tx_buffer = tx_[i];
    t_[i].rx_buffer = rx_[i];
    t_[i].user = (void*)(uintptr_t)i;
    spi_slave_queue_trans(host_, &t_[i], portMAX_DELAY);
  }

  void run() {
    for (uint8_t i = 0; i < slots_; i++) queue(i);
    while (run_) {
      spi_slave_transaction_t* done = nullptr;
      if (spi_slave_get_trans_result(host_, &done, pdMS_TO_TICKS(50)) != ESP_OK) continue;
      uint8_t i = (uint8_t)(uintptr_t)done->user;
      size_t len = done->trans_len / 8;
      if (len > frame_) len = frame_;
      for (size_t k = 0; k < len; k++) s_->pushRx(rx_[i][k]);
      s_->frameEnd();
      queue(i);
    }
    taskDone_ = true;
  }

  void freeBuffers() {
    for (uint8_t i = 0; i < kMaxSlots; i++) {
      heap_caps_free(tx_[i]);
      heap_caps_free(rx_[i]);
      tx_[i] = rx_[i] = nullptr;
    }
  }

  Shared* s_ = nullptr;
  spi_host_device_t host_ = SPI2_HOST;
  uint8_t slots_ = 2;
  uint8_t idle_ = 0;
  size_t frame_ = 64;
  uint8_t* tx_[kMaxSlots] = {};
  uint8_t* rx_[kMaxSlots] = {};
  spi_slave_transaction_t t_[kMaxSlots];
  volatile bool run_ = false;
  volatile bool taskDone_ = true;
};

}  // namespace detail
}  // namespace tinyspi
