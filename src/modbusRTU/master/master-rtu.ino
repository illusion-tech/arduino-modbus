#include <ModbusRTU.h>
#include <SoftwareSerial.h>

#define BUTTON_PIN 9 // Boot button pin
#define SLAVE_ID 1

SoftwareSerial S(1, 0);

ModbusRTU mb;

bool cbWrite(Modbus::ResultCode event, uint16_t transactionId, void* data) {
  Serial.printf_P("Request result: 0x%02X, Mem: %d\n", event, ESP.getFreeHeap());

  return true;
}

void setup() {
  Serial.begin(115200);
  S.begin(9600, SWSERIAL_8N1);
  mb.begin(&S, 10);
  
  mb.master();

  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), pin_ISR, FALLING);
}

bool coils[10];
uint16_t res[1];

bool isUpdateHreg = false;
// bool isUpdateCoil = false;

void pin_ISR()
{
  Serial.println("Interrupt writeHreg");
  mb.writeHreg(SLAVE_ID, 0, 10);
  isUpdateHreg = true;
  // mb.writeCoil(SLAVE_ID, 0, false);
  // isUpdateCoil = true;
}

void loop() {
  if (!mb.slave() || isUpdateHreg) {
    mb.readHreg(SLAVE_ID, 0, res, 1, cbWrite);
    neopixelWrite(8, RGB_BRIGHTNESS, 0, 0);
    while (mb.slave()) {
      isUpdateHreg = false;
      mb.task();
      delayMicroseconds(100);
    }
    neopixelWrite(8, 0, 0, 0);
    Serial.printf("hreg value: %d\n", res[0]);
  }

  delay(1000);

  if (!mb.slave()) {
    mb.readCoil(SLAVE_ID, 0, coils, 10);
    neopixelWrite(8, RGB_BRIGHTNESS, 0, 0);
    while (mb.slave()) {
      mb.task();
      delayMicroseconds(100);
    }
    neopixelWrite(8, 0, 0, 0);
    Serial.printf("coil value: ");
    for (int i = 0; i < 10; i++) {
      Serial.printf("%d ", coils[i]);
    }
    Serial.println();
  }

  delay(1000);
}
