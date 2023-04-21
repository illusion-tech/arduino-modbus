#include <ModbusRTU.h>

#define RX_PIN 18
#define TX_PIN 17
HardwareSerial S(1);

#define EN_PIN 21
#define REGN 0
#define SLAVE_ID 1

ModbusRTU mb;

uint16_t getRegCb(TRegister *reg, uint16_t val) {
  int regType = reg->address.type;
  uint16_t regAddress = reg->address.address;
  
  // Serial.printf("Get RegType: %d, RegAddress: %d, RegVal: %d\n", regType, regAddress, val);

  return val;
}

void setup() {
  // Serial.begin(115200);

  S.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

  mb.begin(&S, EN_PIN);
  mb.slave(SLAVE_ID);

  mb.addHreg(0, 100);
  mb.addHreg(1, 120);
  mb.addHreg(2, 80);
  mb.addHreg(3, 90);

  mb.addCoil(REGN, false, 4);

  mb.onGet(HREG(REGN), getRegCb);
  mb.onGet(COIL(REGN), getRegCb);
}

void loop() {
  mb.task();
  yield();
}