// Echoes every received byte + 1 back to the master. Because the slave can only
// answer while the master clocks, the reply appears in the master's *next*
// transfer (ESP32: up to `slots` frames later). Send dummy bytes to read it.
#include "TinySPISlave.h"

#if defined(ARDUINO_ARCH_ESP32)
const int PIN_SCK = 18, PIN_MISO = 19, PIN_MOSI = 23, PIN_CS = 5;
#elif defined(ARDUINO_ARCH_RP2040)
const int PIN_MOSI = 16, PIN_CS = 17, PIN_SCK = 18, PIN_MISO = 19;
#else
const int PIN_SCK = PA5, PIN_MISO = PA6, PIN_MOSI = PA7, PIN_CS = PA4;
#endif

TinySPISlave spiSlave;

void setup() {
  Serial.begin(115200);
  tinyspi::Config cfg;
  cfg.sck = PIN_SCK; cfg.miso = PIN_MISO; cfg.mosi = PIN_MOSI; cfg.cs = PIN_CS;
  cfg.mode = 0;
  cfg.maxFrameSize = 32;  // ESP32 only
  if (!spiSlave.begin(cfg)) {
    Serial.println("SPI slave init failed");
    while (true) delay(1000);
  }
}

void loop() {
  while (spiSlave.available()) {
    uint8_t b = spiSlave.read();
    spiSlave.write((uint8_t)(b + 1));
  }
}
