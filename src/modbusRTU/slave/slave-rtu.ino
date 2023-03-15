#include <ModbusRTU.h>

#if CONFIG_IDF_TARGET_ESP32S3
#define RX_PIN 3
#define TX_PIN 2
HardwareSerial S(1);

#elif CONFIG_IDF_TARGET_ESP32C3
#include <SoftwareSerial.h>
#define RX_PIN 1
#define TX_PIN 0
SoftwareSerial S;

#endif

#define EN_PIN 1
#define REGN 0
#define SLAVE_ID 1

ModbusRTU mb;

uint16_t getRegCb(TRegister *reg, uint16_t val) {
  int regType = reg->address.type;
  uint16_t regAddress = reg->address.address;
  
  Serial.printf("Get RegType: %d, RegAddress: %d, RegVal: %d\n", regType, regAddress, val);

  return val;
}

void setup() {
  Serial.begin(115200);

#if CONFIG_IDF_TARGET_ESP32S3
  S.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
#elif CONFIG_IDF_TARGET_ESP32C3
  S.begin(9600, SWSERIAL_8N1, RX_PIN, TX_PIN);
#endif

  mb.begin(&S, EN_PIN);
  mb.slave(SLAVE_ID);

  mb.addHreg(REGN, 100);
  mb.addCoil(REGN, true, 5);

  mb.onGet(HREG(REGN), getRegCb);
  mb.onGet(COIL(REGN), getRegCb);
}

void loop() {
  mb.task();
  yield();
}