// 定义Modbus地址和功能码
#define SLAVE_ADDRESS 1
#define READ_HOLDING_REGISTERS 0x03

// 定义控制寄存器地址和长度
#define CONTROL_REG_ADDRESS 0x0000
#define CONTROL_REG_LENGTH 1

// 定义串口通信的RX和TX引脚
#define RX_PIN 18
#define TX_PIN 17

// 定义RS485通信模块的使能引脚
#define ENABLE_PIN 21

// 定义Modbus帧的最大长度
#define MAX_FRAME_LENGTH 256

// 定义Modbus帧的结构体
struct ModbusFrame {
  uint8_t address;
  uint8_t function;
  uint16_t startAddress;
  uint16_t quantity;
  uint8_t crc[2];
};

HardwareSerial S(1);

// 定义控制寄存器
uint16_t controlReg[CONTROL_REG_LENGTH];

void setup() {
  Serial.begin(115200);
  S.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);  // 使能管脚低电平有效
}

void loop() {
  // 如果有数据可读取
  while (S.available()) {
    // 读取数据
    uint8_t buffer[MAX_FRAME_LENGTH];
    uint8_t length = 0;
    while (S.available()) {
      buffer[length++] = S.read();
      if (length >= MAX_FRAME_LENGTH) {
        break;
      }
    }

    Serial.print("收到主机消息:长度为 ");
    Serial.println(length);
    for (int i = 0; i < length; i++) {
      Serial.print(buffer[i], HEX);
      Serial.print(" ");
    }
    Serial.println();

    // 解析Modbus帧
    ModbusFrame frame;
    memcpy(&frame, buffer, sizeof(frame));

    // 如果地址不匹配，忽略此帧
    if (frame.address != SLAVE_ADDRESS) {
      return;
    }

    // 如果功能码不匹配，忽略此帧
    if (frame.function != READ_HOLDING_REGISTERS) {
      return;
    }

    // 如果起始地址不匹配，忽略此帧
    if (frame.startAddress != CONTROL_REG_ADDRESS) {
      return;
    }

    // 如果数量不匹配，忽略此帧
    if (frame.quantity != CONTROL_REG_LENGTH) {
      return;
    }

    // 计算CRC校验码
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < length - 2; i++) {
      crc ^= buffer[i];
      for (int j = 0; j < 8; j++) {
        if (crc & 0x0001) {
          crc >>= 1;
          crc ^= 0xA001;
        } else {
          crc >>= 1;
        }
      }
    }

    // 如果CRC校验码不匹配，忽略此帧
    if (crc != (frame.crc[1] << 8 | frame.crc[0])) {
      return;
    }

    // 如果所有条件都匹配，发送响应帧
    //uint8_t responseLength = 1 + CONTROL_REG_LENGTH * 2 + 2;
    uint8_t responseLength = 8;
    uint8_t response[responseLength];
    response[0] = SLAVE_ADDRESS;
    response[1] = READ_HOLDING_REGISTERS;
    response[2] = CONTROL_REG_LENGTH * 2;
    controlReg[0] = 100;
    for (int i = 0; i < CONTROL_REG_LENGTH; i++) {
      response[3 + i * 2] = controlReg[i] >> 8;
      response[3 + i * 2 + 1] = controlReg[i] & 0xFF;
    }
    crc = 0xFFFF;
    for (int i = 0; i < responseLength - 2; i++) {
      crc ^= response[i];
      for (int j = 0; j < 8; j++) {
        if (crc & 0x0001) {
          crc >>= 1;
          crc ^= 0xA001;
        } else {
          crc >>= 1;
        }
      }
    }
    response[responseLength - 2] = crc & 0xFF;
    response[responseLength - 1] = crc >> 8;

    // 发送响应帧
    digitalWrite(ENABLE_PIN, HIGH);  // 使能RS485发送
    S.write(response, responseLength);
    S.flush();  // 等待发送完成
    Serial.print("发送响应帧:长度为 ");
    Serial.println(responseLength);
    for (int i = 0; i < responseLength; i++) {
      Serial.print(response[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
    digitalWrite(ENABLE_PIN, LOW);  // 禁用RS485发送
  }
}

void serialEvent() {
  // 如果有数据可读取
  while (S.available()) {
    // 读取数据
    uint8_t buffer[MAX_FRAME_LENGTH];
    uint8_t length = 0;
    while (S.available()) {
      buffer[length++] = S.read();
      if (length >= MAX_FRAME_LENGTH) {
        break;
      }
    }

    Serial.print("收到主机消息:长度为 ");
    Serial.println(length);
    for (int i = 0; i < length; i++) {
      Serial.print(buffer[i], HEX);
      Serial.print(" ");
    }
    Serial.println();

    // 解析Modbus帧
    ModbusFrame frame;
    memcpy(&frame, buffer, sizeof(frame));

    // 如果地址不匹配，忽略此帧
    if (frame.address != SLAVE_ADDRESS) {
      return;
    }

    // 如果功能码不匹配，忽略此帧
    if (frame.function != READ_HOLDING_REGISTERS) {
      return;
    }

    // 如果起始地址不匹配，忽略此帧
    if (frame.startAddress != CONTROL_REG_ADDRESS) {
      return;
    }

    // 如果数量不匹配，忽略此帧
    if (frame.quantity != CONTROL_REG_LENGTH) {
      return;
    }

    // 计算CRC校验码
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < length - 2; i++) {
      crc ^= buffer[i];
      for (int j = 0; j < 8; j++) {
        if (crc & 0x0001) {
          crc >>= 1;
          crc ^= 0xA001;
        } else {
          crc >>= 1;
        }
      }
    }

    // 如果CRC校验码不匹配，忽略此帧
    if (crc != (frame.crc[1] << 8 | frame.crc[0])) {
      return;
    }

    // 如果所有条件都匹配，发送响应帧
    uint8_t responseLength = 1 + CONTROL_REG_LENGTH * 2 + 2;
    uint8_t response[responseLength];
    response[0] = SLAVE_ADDRESS;
    response[1] = READ_HOLDING_REGISTERS;
    response[2] = CONTROL_REG_LENGTH * 2;
    for (int i = 0; i < CONTROL_REG_LENGTH; i++) {
      response[3 + i * 2] = controlReg[i] >> 8;
      response[3 + i * 2 + 1] = controlReg[i] & 0xFF;
    }
    crc = 0xFFFF;
    for (int i = 0; i < responseLength - 2; i++) {
      crc ^= response[i];
      for (int j = 0; j < 8; j++) {
        if (crc & 0x0001) {
          crc >>= 1;
          crc ^= 0xA001;
        } else {
          crc >>= 1;
        }
      }
    }
    response[responseLength - 2] = crc & 0xFF;
    response[responseLength - 1] = crc >> 8;

    // 发送响应帧
    digitalWrite(ENABLE_PIN, HIGH);  // 使能RS485发送
    S.write(response, responseLength);
    S.flush();  // 等待发送完成
    Serial.print("发送响应帧:长度为 ");
    Serial.println(responseLength);
    for (int i = 0; i < responseLength; i++) {
      Serial.print(response[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
    digitalWrite(ENABLE_PIN, LOW);  // 禁用RS485发送
  }
}