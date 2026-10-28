// Prints every byte the SPI master sends, and reports each frame.
// Adjust the pins for your board (see README for per-platform pin rules).
#include "TinySPISlave.h"

#if defined(ARDUINO_ARCH_ESP32)
const int PIN_SCK = 18, PIN_MISO = 19, PIN_MOSI = 23, PIN_CS = 5;
#elif defined(ARDUINO_ARCH_RP2040)
// Slave mode: MOSI = RX pin (GPIO%4==0), CS = GPIO%4==1, SCK = GPIO%4==2, MISO = TX pin (GPIO%4==3)
const int PIN_MOSI = 16, PIN_CS = 17, PIN_SCK = 18, PIN_MISO = 19;
#else  // STM32: Nucleo-64 SPI1 (Arduino D13/D12/D11/D10 style); change to match your board
const int PIN_SCK = PA5, PIN_MISO = PA6, PIN_MOSI = PA7, PIN_CS = PA4;
#endif

TinySPISlave spiSlave;

void onFrame(size_t len, void*) {
  Serial.print("frame of ");
  Serial.print(len);
  Serial.println(" bytes");
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}
  spiSlave.onFrame(onFrame);
  if (!spiSlave.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS, 0)) {
    Serial.println("SPI slave init failed");
    while (true) delay(1000);
  }
}

void loop() {
  spiSlave.poll();  // frame detection on RP2040/STM32 (no-op on ESP32)
  while (spiSlave.available()) {
    Serial.printf("0x%02X\n", spiSlave.read());
  }
}
