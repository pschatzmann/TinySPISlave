# TinySPISlave

Header-only SPI **slave** library with one API for **ESP32, STM32, RP2040 and RP2350**.

## Why this library?

Arduino's `SPI` library only does the master side. SPI slave support exists, but it is
platform-specific: each chip family has its own hardware, its own vendor API
(ESP-IDF `spi_slave`, pico-sdk PL022 slave mode, STM32 HAL/LL) and its own quirks. The existing
Arduino libraries I found each target a single platform, so a sketch that acts as an SPI peripheral
(a sensor-like device, a co-processor, a bridge between two MCUs) has to be rewritten whenever you
move it to another board.

TinySPISlave is a small portability layer that hides those differences behind one `Stream`-style
API (`begin`, `available`, `read`, `write`) with consistent frame handling on every platform. On top
of that it offers a register-map helper, because "expose some registers to an SPI master" is what
most slave sketches end up implementing by hand.

It is header only, has no dependencies beyond the Arduino core, and lets you pick the board
(Raspberry Pi Pico, STM32 Nucleo, ESP32) based on other requirements rather than on SPI slave support.
The platform differences that can't be hidden (pin rules, DMA reply latency on ESP32) are listed
below instead of being papered over.

## Features

* One API for ESP32, STM32, RP2040 and RP2350; the backend is selected automatically
* Header only, no dependencies beyond the Arduino core
* `Stream` interface: `available()`, `read()`, `peek()`, `write()`, `availableForWrite()`
* SPI modes 0-3, 8 bit frames, ring buffers for rx and tx with configurable size
* Interrupt/DMA driven: no polling needed to receive or transmit bytes
* Frame detection via the CS line on every platform
  * `onFrame()` callbacks delivered from `loop()` via `poll()`
  * `onFrameImmediate()` hook that runs right when CS is released
* Register maps (`RegisterMap`)
  * table driven: every register has its own size (1-255 bytes), address and access (RW / RO / WO)
  * maps onto your own variables, structs and arrays, or uses internal storage
  * optional big-endian wire format per register
  * per-register read callback to compute a value on demand
  * `onWrite()` notifications in `loop()` context, atomic `get()` / `set()` for the application
  * configurable command byte layout (read bit, address mask, auto-increment)
* Overflow counters (`rxOverflows()`, `frameOverflows()`) to spot lost data
* Examples for raw byte access, request/response and register maps

```cpp
#include "TinySPISlave.h"
TinySPISlave spiSlave;

void setup() { spiSlave.begin(SCK, MISO, MOSI, CS, /*mode*/ 0); }
void loop() {
  spiSlave.poll();                       // only needed for onFrame() callbacks
  while (spiSlave.available()) Serial.println(spiSlave.read());
  spiSlave.write(0x42);                  // queued for the master's next clocks
}
```

`TinySPISlave` is a `Stream` (`available/read/peek/write`). `onFrame(cb, ctx)` registers an
optional callback fired when CS goes high, with the number of bytes received in that frame
(read them with `read()`). `tinyspi::Config` gives access to buffer sizes and ESP32 options.

## Backends

| Platform | Implementation | Notes |
|---|---|---|
| ESP32 | ESP-IDF `spi_slave` + DMA, one internal task | any pins via GPIO matrix, SPI2/SPI3 host |
| RP2040 / RP2350 | PL022 slave mode (pico-sdk), IRQ driven | arduino-pico core only (not mbed) |
| STM32 | STM32duino `spi_init(..., SPI_PERIPHERAL_C)` + LL IRQ | pins must belong to one SPI instance |

## Platform rules

* **CS is mandatory** on every platform.
* **RP2040/RP2350 pins** are fixed by function select. All four in the same block, and for pin `n`:
  `n%4==0` MOSI (slave RX), `1` CS, `2` SCK, `3` MISO (slave TX); block = `(n>>3)&1`.
  Example: MOSI 16, CS 17, SCK 18, MISO 19. Keep SCK ≲ clk_peri/12 (~10 MHz at defaults).
  Do not use the Arduino `SPI` object on the same block.
* **STM32:** SPI IRQ vectors are defined (weak) by this header. Don't also drive that SPI with the `SPI` library.
* **ESP32** transfers are DMA transactions of `maxFrameSize` bytes (longer frames are split). `slots`
  transactions stay queued, so a reply written now is clocked out up to `slots` frames later, and
  any part of the staged reply that a short frame didn't clock out is dropped.
  On RP2040/STM32 queued tx bytes form a stream: unsent bytes carry over to the next frame.
* When no tx data is queued: ESP32 sends `idleByte`; RP2040/STM32 send whatever the hardware does (typically 0x00).
* **Frames** end at the CS rising edge on every platform (ESP32 transaction end, RP2040 GPIO IRQ,
  STM32 EXTI armed before the pin switches to NSS). `onFrame()` callbacks are queued (16) and run
  from `poll()`; `onFrameImmediate()` runs in the ISR / ESP32 task.
* `rxOverflows()` counts bytes dropped because the rx buffer was full.

## Register maps

`tinyspi::RegisterMap` serves a table of registers, each with its own size, access, byte order and
storage (your variable, or internal). A register can also be computed by a callback on every read.

```cpp
uint16_t id = 0xBEEF; int16_t temp; float gains[3];
tinyspi::RegisterDef defs[] = {                       // ascending addresses
  tinyspi::Reg(0x00, id, tinyspi::ReadOnly),
  tinyspi::Reg(0x01, temp, tinyspi::ReadOnly, true),  // big endian on the wire
  tinyspi::Reg(0x10, gains),                          // 12 byte register
  tinyspi::Reg(0x20, 16),                             // 16 byte, internal storage
  tinyspi::Reg(0x30, 4, readUptime),                  // reply computed by callback
};
tinyspi::RegisterMap regs(defs, sizeof(defs) / sizeof(defs[0]));
// setup: regs.begin(spiSlave);   loop: regs.set(0x01, v); regs.update();
```

Protocol: command byte (`bit7` = read, 7 bit address, configurable via `RegisterProtocol`).
Write: `[cmd][bytes...]`. Read: `[cmd][n]` then a **second frame** in which the master clocks `n` bytes
(a slave can't answer inside the frame carrying the address). Leave a short CS-high gap between the
two (~50 us RP2040/STM32, ~200 us ESP32, where `Config::slots` must be 1). Use `regs.get/set` from
`loop()`; `onWrite(cb)` notifications are delivered by `regs.update()`. See `examples/RegisterMap`.

## Status

Compiles (`arduino-cli`) for ESP32, ESP32-S3, Pico, Pico 2, Nucleo F411RE, G474RE and H743ZI2.
**Not yet verified on hardware**, in particular the STM32 SPI-v2 (H7/U5/H5) path.
