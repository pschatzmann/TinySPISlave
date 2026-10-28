// SPI slave with registers of different sizes, described by a table.
//   0x00  ID       uint16  ro   0xBEEF
//   0x01  TEMP     int16   ro   0.01 degC, big endian on the wire
//   0x02  CTRL     uint8   rw   bit0 = LED
//   0x03  COUNTER  uint32  ro   increments every second
//   0x10  GAINS    float[3] rw  12 byte register
//   0x20  NAME     16 byte rw   internal storage
//   0x30  UPTIME   uint32  ro   computed by a callback on every read
//
// Master side (bit7 of the command = read):
//   write CTRL=1 :  CS low, 0x02, 0x01, CS high
//   read TEMP    :  CS low, 0x81, CS high           (read request, size of the register)
//                   pause >= 200 us
//                   CS low, 0x00, 0x00, CS high    (2 bytes arrive on MISO)
//   read 5 bytes starting at COUNTER: CS low, 0x83, 0x05, CS high, pause, 5 dummy bytes
#include "TinySPISlave.h"

#if defined(ARDUINO_ARCH_ESP32)
const int PIN_SCK = 18, PIN_MISO = 19, PIN_MOSI = 23, PIN_CS = 5;
#elif defined(ARDUINO_ARCH_RP2040)
const int PIN_MOSI = 16, PIN_CS = 17, PIN_SCK = 18, PIN_MISO = 19;
#else
const int PIN_SCK = PA5, PIN_MISO = PA6, PIN_MOSI = PA7, PIN_CS = PA4;
#endif

#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

// Runs when the master reads UPTIME (interrupt/task context: keep it short)
void readUptime(uint8_t, void* out, void*) {
  uint32_t ms = millis();
  memcpy(out, &ms, sizeof(ms));
}

enum : uint8_t { REG_ID = 0x00, REG_TEMP = 0x01, REG_CTRL = 0x02, REG_COUNTER = 0x03,
                 REG_GAINS = 0x10, REG_NAME = 0x20, REG_UPTIME = 0x30 };

uint16_t id = 0xBEEF;
int16_t temp;
uint8_t ctrl;
uint32_t counter;
float gains[3] = {1.0f, 0.1f, 0.01f};

using tinyspi::Reg;
tinyspi::RegisterDef defs[] = {  // ascending addresses
    Reg(REG_ID, id, tinyspi::ReadOnly),
    Reg(REG_TEMP, temp, tinyspi::ReadOnly, /*bigEndian*/ true),
    Reg(REG_CTRL, ctrl),
    Reg(REG_COUNTER, counter, tinyspi::ReadOnly),
    Reg(REG_GAINS, gains),
    Reg(REG_NAME, 16),  // storage owned by the map
    Reg(REG_UPTIME, sizeof(uint32_t), readUptime),
};

TinySPISlave spiSlave;
tinyspi::RegisterMap regs(defs, sizeof(defs) / sizeof(defs[0]));

// loop() context, runs after the master wrote a register
void onWrite(uint8_t addr, void*) {
  if (addr == REG_CTRL) digitalWrite(LED_BUILTIN, regs.get<uint8_t>(REG_CTRL) & 1);
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

  tinyspi::Config cfg;
  cfg.sck = PIN_SCK; cfg.miso = PIN_MISO; cfg.mosi = PIN_MOSI; cfg.cs = PIN_CS;
  cfg.slots = 1;  // ESP32: lets the reply go out in the very next frame
  if (!spiSlave.begin(cfg)) {
    Serial.println("SPI slave init failed");
    while (true) delay(1000);
  }

  regs.onWrite(onWrite);
  if (!regs.begin(spiSlave)) {  // default protocol: bit7 = read, 7 bit address
    Serial.println("invalid register table");
    while (true) delay(1000);
  }
}

void loop() {
  static uint32_t last;
  if (millis() - last >= 1000) {
    last = millis();
    regs.set(REG_COUNTER, regs.get<uint32_t>(REG_COUNTER) + 1);
    regs.set<int16_t>(REG_TEMP, 2150 + (int16_t)(millis() / 1000 % 10));
  }
  regs.update();
}
