// Function Codes
enum FunctionCode {
  READ_COILS       = 0x01, // Read Coils (Output) Status
  READ_INPUT_STAT  = 0x02, // Read Input Status (Discrete Inputs)
  READ_REGS        = 0x03, // Read Holding Registers
  READ_INPUT_REGS  = 0x04, // Read Input Registers
  WRITE_COIL       = 0x05, // Write Single Coil (Output)
  WRITE_REG        = 0x06, // Preset Single Register
  DIAGNOSTICS      = 0x08, // Not implemented. Diagnostics (Serial Line only)
  WRITE_COILS      = 0x0F, // Write Multiple Coils (Outputs)
  WRITE_REGS       = 0x10, // Write block of contiguous registers
  READ_FILE_REC    = 0x14, // Read File Record
  WRITE_FILE_REC   = 0x15, // Write File Record
  MASKWRITE_REG    = 0x16, // Mask Write Register
  READWRITE_REGS   = 0x17  // Read/Write Multiple registers
};

// 定义Modbus帧的最大长度
#define MAX_FRAME_LENGTH 256

// 定义从机地址
#define SLAVE_ADDRESS 1

// 定义寄存器长度
#define REG_NUM 3
// 定义寄存器起始地址
#define HREG_ADDRESS 0x0000
#define COIL_ADDRESS 0x0000

// 定义寄存器初始值
uint16_t HREG[REG_NUM] = {100, 80, 50};
bool COIL[REG_NUM] = {false, false, false};

// 定义Modbus帧的结构体
struct ModbusFrame {
  uint8_t address;
  FunctionCode fcode;
  uint16_t startAddress;
  uint16_t quantity;
  uint8_t crc[2];
};

// 定义串口通信的 RX TX 使能 引脚
#define RX_PIN 18
#define TX_PIN 17
#define ENABLE_PIN 21

HardwareSerial S(1);

void setup() {
  Serial.begin(115200);
  S.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);  // 使能管脚低电平有效
}

// 计算CRC校验码
void crc16modbus(uint8_t *buffer, uint16_t length) {
  uint16_t crc16 = 0xFFFF;
  for (int i = 0; i < length; i++) {
    crc16 ^= buffer[i];
    for (int j = 0; j < 8; j++) {
      if (crc16 & 0x0001) {
        crc16 >>= 1;
        crc16 ^= 0xA001;
      } else {
        crc16 >>= 1;
      }
    }
  }
  buffer[length] = crc16 & 0xFF;
  buffer[length + 1] = crc16 >> 8;
}

// 获取 Modbus 请求帧
void getRequestFrame(ModbusFrame *frame, uint8_t *buffer, uint16_t length) {
  frame->address = buffer[0];
  frame->fcode = (FunctionCode)buffer[1];
  frame->startAddress = buffer[2] << 8 | buffer[3];
  frame->quantity = buffer[4] << 8 | buffer[5];
  frame->crc[0] = buffer[length - 2];
  frame->crc[1] = buffer[length - 1];
}

// 获取 Modbus 响应帧
void getResponseFrame(ModbusFrame *frame, uint16_t *response) {
  FunctionCode fcode = (FunctionCode)frame->fcode;

  response[0] = frame->address;
  response[1] = fcode;

  uint8_t byteCount = 0;

  switch (fcode) {
    case FunctionCode::READ_COILS:
    case FunctionCode::READ_INPUT_STAT:
      byteCount = frame->quantity / 8 + (frame->quantity % 8 == 0 ? 0 : 1);
      Serial.print("byteCount:");
      Serial.println(byteCount);

      response[2] = byteCount;
      for (int i = 0; i < byteCount; i++) {
        uint8_t byte = 0;
        for (int j = 0; j < 8; j++) {
          if (COIL[frame->startAddress + i * 8 + j]) {
            byte |= 1 << j;
          }
        }
        response[3 + i] = byte;
      }
      break;
    case FunctionCode::READ_REGS:
    case FunctionCode::READ_INPUT_REGS:
      response[2] = frame->quantity * 2;
      for (int i = 0; i < frame->quantity; i++) {
        response[3 + i * 2] = HREG[frame->startAddress + i] >> 8;
        response[3 + i * 2 + 1] = HREG[frame->startAddress + i] & 0xFF;
      }
      break;
  }

  uint16_t responseLen = 3 + response[2];

  crc16modbus((uint8_t *)response, responseLen);
}

// 发送响应帧
void sendResponseFrame(ModbusFrame *frame) {
  uint16_t response[MAX_FRAME_LENGTH];
  getResponseFrame(frame, response);

  Serial.print("response:");
  for (int i = 0; i < 3 + response[2] + 2; i++) {
    Serial.print(response[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  digitalWrite(ENABLE_PIN, HIGH);  // 使能RS485发送
  for (int i = 0; i < 3 + response[2] + 2; i++) {
    S.write(response[i]);
  }
  S.flush();
  digitalWrite(ENABLE_PIN, LOW);  // 禁用RS485发送
}

void loop() {
  // 如果有数据可读取
  while (S.available()) {
    // 读取数据
    uint8_t buffer[MAX_FRAME_LENGTH];
    uint16_t length = 0;
    while (S.available()) {
      buffer[length++] = S.read();
      if (length >= MAX_FRAME_LENGTH) {
        break;
      }
    }

    if (length == 1 && buffer[0] == 0x00) {
      Serial.println("接收到空帧");
      continue;
    }

    Serial.print("receive:");
    for (int i = 0; i < length; i++) {
      Serial.print(buffer[i], HEX);
      Serial.print(" ");
    }
    Serial.println();

    // 解析Modbus帧
    ModbusFrame frame;
    getRequestFrame(&frame, buffer, length);

    // 如果地址不匹配，忽略此帧
    if (frame.address != SLAVE_ADDRESS) return;

    // 如果起始地址不匹配，忽略此帧
    if (frame.startAddress != HREG_ADDRESS) return;

    // 如果所有条件都匹配，发送响应帧
    sendResponseFrame(&frame);
  }
}
