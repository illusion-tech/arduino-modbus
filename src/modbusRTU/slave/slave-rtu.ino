#include <ModbusRTU.h>
#include <SoftwareSerial.h>

#define REGN 0
#define SLAVE_ID 1

SoftwareSerial S(1, 0);

ModbusRTU mb;

uint16_t getHregCb(TRegister *reg, uint16_t val) {
  // neopixelWrite(8, 0, RGB_BRIGHTNESS, 0);
  Serial.printf("reg val: %d\n", val);

  return val;
}

void setup() {
  Serial.begin(115200);
  S.begin(9600, SWSERIAL_8N1);
  mb.begin(&S, 10);

  mb.slave(SLAVE_ID);

  mb.addHreg(REGN);
  mb.Hreg(REGN, 100);

  if (mb.Coil(0)) {
    mb.removeCoil(0, 10);
  }
  mb.addCoil(0, true, 10);

  mb.onGet(HREG(REGN), getHregCb);
}

void loop() {
  mb.task();
  yield();
}
