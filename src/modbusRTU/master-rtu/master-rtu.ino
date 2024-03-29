#include <ModbusRTU.h>

#if CONFIG_IDF_TARGET_ESP32S3
#define RX_PIN 18
#define TX_PIN 17
#define RGB_PIN 48
HardwareSerial S(1);

#elif CONFIG_IDF_TARGET_ESP32C3
#include <SoftwareSerial.h>
#define RX_PIN 1
#define TX_PIN 0
#define RGB_PIN 8
SoftwareSerial S;

#endif

#define EN_PIN 21
#define SLAVE_ID 2

ModbusRTU mb;
Modbus::ResultCode code;

bool cbHandle(Modbus::ResultCode event, uint16_t transactionId, void* data) {
  // Serial.printf_P("Request result: 0x%02X ...\n", event);
  code = event;

  return true;
}

void setup() {
  Serial.begin(115200);
  pinMode(1, OUTPUT);

#if CONFIG_IDF_TARGET_ESP32S3
  S.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
#elif CONFIG_IDF_TARGET_ESP32C3
  S.begin(9600, SWSERIAL_8N1, RX_PIN, TX_PIN);
#endif

  mb.begin(&S, EN_PIN);
  mb.master();
}

bool coils[5];
uint16_t res[1];

void loop() {
  if (!mb.slave()) {
    mb.readHreg(SLAVE_ID, 0, res, 1, cbHandle);
    while (mb.slave()) {
      mb.task();
      delayMicroseconds(100);
    }
    if (code == Modbus::EX_SUCCESS) {
      // neopixelWrite(RGB_PIN, 0, RGB_BRIGHTNESS, 0);
      digitalWrite(1, HIGH);
      // Serial.printf("read hreg value: %d\n", res[0]);
    } else {
      // neopixelWrite(RGB_PIN, RGB_BRIGHTNESS, 0, 0);
      digitalWrite(1, LOW);
    }
    delay(100);
    // neopixelWrite(RGB_PIN, 0, 0, 0);
    digitalWrite(1, LOW);
  }
  
  delay(1000);

  if (!mb.slave()) {
    mb.readCoil(SLAVE_ID, 0, coils, 5, cbHandle);
    while (mb.slave()) {
      mb.task();
      delayMicroseconds(100);
    }
    if (code == Modbus::EX_SUCCESS) {
      // neopixelWrite(RGB_PIN, 0, RGB_BRIGHTNESS, 0);
      digitalWrite(1, HIGH);
      // Serial.printf("coil value: ");
      // for (int i = 0; i < 5; i++) {
      //   Serial.printf("%d ", coils[i]);
      // }
      // Serial.println();
    } else {
      // neopixelWrite(RGB_PIN, RGB_BRIGHTNESS, 0, 0);
      digitalWrite(1, LOW);
    }
    delay(100);
    // neopixelWrite(RGB_PIN, 0, 0, 0);
    digitalWrite(1, LOW);
  }

  delay(1000);
}
