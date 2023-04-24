#include <ModbusRTU.h>

// 作为从机响应消息给上位机
ModbusRTU serverMb;
#define RX_PIN 18
#define TX_PIN 17
#define EN_PIN 21
#define REGN 0
#define SLAVE_ID 1
HardwareSerial serverSerial(1);

// 作为主机读取传感器数值
ModbusRTU sensorMb;
#define SENSOR_RX_PIN 48
#define SENSOR_TX_PIN 47
#define SENSOR_EN_PIN 38
#define SENSOR_ID 1
HardwareSerial sensorSerial(2);

Modbus::ResultCode code;

xSemaphoreHandle xMutex;

bool cbHandle(Modbus::ResultCode event, uint16_t transactionId, void* data) {
  code = event;

  return true;
}

// 传感器读数
uint16_t sensorVal[1];

// 读取传感器数据
void readSensor() {
  xSemaphoreTake(xMutex, portMAX_DELAY);

  if (!sensorMb.slave()) {
    sensorMb.readHreg(SENSOR_ID, 0, sensorVal, 2, cbHandle);
    while (sensorMb.slave()) {
      sensorMb.task();
      delayMicroseconds(100);
    }
    if (code == Modbus::EX_SUCCESS) {
      serverMb.Hreg(0, sensorVal[0]);
    } else {
      //
    }

    xSemaphoreGive(xMutex);
  }
}

void sensorLoop(void * pvParameters);

void setup() {
  serverSerial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

  sensorSerial.begin(9600, SERIAL_8N1, SENSOR_RX_PIN, SENSOR_TX_PIN);


  serverMb.begin(&serverSerial, EN_PIN);
  serverMb.slave(SLAVE_ID);
  serverMb.addHreg(0, 100);
  serverMb.addHreg(1, 80);
  serverMb.addHreg(2, 120);
  serverMb.addHreg(3, 60);

  serverMb.addCoil(0, true);
  serverMb.addCoil(1, true);
  serverMb.addCoil(2, true);
  serverMb.addCoil(3, true);


  sensorMb.begin(&sensorSerial, SENSOR_EN_PIN);
  sensorMb.master();

  xMutex = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(
                    sensorLoop,   /* Task function. */
                    "ReadSensorTask",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    10,           /* priority of the task */
                    NULL,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 1 */
}

void sensorLoop(void * pvParameters) {
  while (true) {
    delay(100);
    readSensor();
  }
}

void loop() {
  serverMb.task();
  yield();
}
