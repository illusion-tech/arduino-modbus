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
// 定义Modbus帧的结构体
struct ModbusFrame {
  uint8_t address;
  FunctionCode fcode;
  uint16_t startAddress;
  uint16_t quantity;
  uint8_t crc[2];
};

// 定义Modbus帧的最大长度
#define MAX_FRAME_LENGTH 64

// 定义从机地址
#define SLAVE_ADDRESS 1

// 定义寄存器长度
#define REG_NUM 3
// 定义寄存器起始地址
#define HREG_ADDRESS 0x0000
#define COIL_ADDRESS 0x0000

// 定义寄存器初始值
uint16_t HREG[REG_NUM] = {100, 80, 50};
bool COIL[REG_NUM] = {0, 0, 0};
// 定义继电器引脚
uint16_t REPLAY_PINS[REG_NUM] = {2, 4, 5};

// 定义上位机通信的 RX TX 使能 引脚
#define PC_RX_PIN 18
#define PC_TX_PIN 17
#define PC_EN_PIN 21
HardwareSerial pcSerial(1);

// 定义移动app端通信的 RX TX 使能 引脚
#define APP_RX_PIN 48
#define APP_TX_PIN 47
#define APP_EN_PIN 38
HardwareSerial appSerial(2);

void setup() {
  Serial.begin(115200);
  // 初始化串口1
  pcSerial.begin(115200, SERIAL_8N1, PC_RX_PIN, PC_TX_PIN);
  pinMode(PC_EN_PIN, OUTPUT);
  digitalWrite(PC_EN_PIN, LOW);
  // 初始化串口2
  appSerial.begin(115200, SERIAL_8N1, APP_RX_PIN, APP_TX_PIN);
  pinMode(APP_EN_PIN, OUTPUT);
  digitalWrite(APP_EN_PIN, LOW);

  for (int i = 0; i < REG_NUM; i++) {
    pinMode(REPLAY_PINS[i], OUTPUT);
    digitalWrite(REPLAY_PINS[i], LOW);
  }
}

// 计算CRC校验码
uint16_t crc16(uint8_t *buffer, uint16_t length) {
  uint16_t crc, temp, flag;
  crc = 0xFFFF;
  for (uint8_t i = 0; i < length; i++) {
    crc ^= buffer[i];
    for (uint8_t j = 1; j <= 8; j++) {
      flag = crc & 0x0001;
      crc >>= 1;
      if (flag)
        crc ^= 0xA001;
    }
  }
  // Reverse byte order. 
  temp = crc >> 8;
  crc = (crc << 8) | temp;
  crc &= 0xFFFF;
  return crc;
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
void getResponseFrame(ModbusFrame *frame, uint8_t *response) {
  FunctionCode fcode = (FunctionCode)frame->fcode;

  response[0] = frame->address;
  response[1] = fcode;

  uint8_t byteCount = 0;

  switch (fcode) {
    case FunctionCode::READ_COILS:
    case FunctionCode::READ_INPUT_STAT:
      byteCount = frame->quantity / 8 + (frame->quantity % 8 == 0 ? 0 : 1);
      response[2] = byteCount;
      for (int i = 0; i < byteCount; i++) {
        uint8_t byte = 0;
        for (int j = 0; j < 8; j++) {
          if (getCoil(frame->startAddress + i * 8 + j)) {
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
        response[3 + i * 2] = getHreg(frame->startAddress + i) >> 8;
        response[3 + i * 2 + 1] = getHreg(frame->startAddress + i) & 0xFF;
      }
      break;
  }
  uint16_t pduLen = 3 + response[2];
  uint16_t crc = crc16(response, pduLen);
  response[pduLen] = crc >> 8;
  response[pduLen + 1] = crc & 0xFF;
}

// 发送响应帧
void sendResponseFrame(HardwareSerial *serial, uint16_t en_pin, ModbusFrame *frame) {
  uint8_t response[MAX_FRAME_LENGTH];
  getResponseFrame(frame, response);

  Serial.print("response:");
  for (int i = 0; i < 3 + response[2] + 2; i++) {
    Serial.print(response[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  digitalWrite(en_pin, HIGH);  // 使能RS485发送
  for (int i = 0; i < 3 + response[2] + 2; i++) {
    serial->write(response[i]);
  }
  serial->flush();
  digitalWrite(en_pin, LOW);  // 禁用RS485发送
}

/// @brief 设置寄存器的值
/// @param frame 
/// @param buffer 
void setRegister(ModbusFrame *frame, uint8_t *buffer) {
  switch (frame->fcode) {
    case FunctionCode::WRITE_COIL:
      setCoil(frame->startAddress, (buffer[4] << 8 | buffer[5]) == 0xFF00);
      break;
    case FunctionCode::WRITE_COILS:
      for (int i = 0; i < frame->quantity; i++) {
        setCoil(frame->startAddress + i, buffer[7 + i / 8] & (1 << (i % 8)));
      }
      break;
    case FunctionCode::WRITE_REG:
      setHreg(frame->startAddress, buffer[4] << 8 | buffer[5]);
      break;
    case FunctionCode::WRITE_REGS:
      for (int i = 0; i < frame->quantity; i++) {
        setHreg(frame->startAddress + i, buffer[7 + i * 2] << 8 | buffer[8 + i * 2]);
      }
      break;
  }
}
void setCoil(uint16_t address, bool value) {
  COIL[address] = value;
  setRelay(address, value);
}
void setHreg(uint16_t address, uint16_t value) {
  HREG[address] = value;
}
bool getCoil(uint16_t address) {
  return COIL[address];
}
uint16_t getHreg(uint16_t address) {
  return HREG[address];
}

// 根据线圈状态设置继电器引脚输出
void setRelay(uint16_t offset, bool value) {
  digitalWrite(REPLAY_PINS[offset], value);
}

// 处理接收到的 buffer
void handleBuffer(HardwareSerial *serial, uint16_t en_pin, uint8_t *buffer, uint16_t length) {
  // 校验从机地址
  if (buffer[0] != SLAVE_ADDRESS) return;
  // 校验 CRC
  uint16_t crc = crc16(buffer, length - 2);
  if (crc != (buffer[length - 2] << 8 | buffer[length - 1])) return;

  ModbusFrame frame;
  getRequestFrame(&frame, buffer, length);

  FunctionCode fcode = (FunctionCode)frame.fcode;
  switch (fcode) {
    // 读取线圈和保持寄存器，发送响应帧
    case FunctionCode::READ_COILS:
    case FunctionCode::READ_REGS:
      sendResponseFrame(serial, en_pin, &frame);
      break;
    case FunctionCode::WRITE_COIL:
    case FunctionCode::WRITE_COILS:
    case FunctionCode::WRITE_REG:
    case FunctionCode::WRITE_REGS:
      setRegister(&frame, buffer);
      break;
  }
}

// 上位机串口接收缓冲区
uint8_t pc_buffer[MAX_FRAME_LENGTH];
uint16_t pc_buffer_index = 0;

// app串口接收缓冲区
uint8_t app_buffer[MAX_FRAME_LENGTH];
uint16_t app_buffer_index = 0;

void loop() {
  // 接收上位机 Modbus 帧
  if (pcSerial.available()) {
    while (pcSerial.available()) {
      pc_buffer[pc_buffer_index++] = pcSerial.read();
    }
    
    if (pc_buffer_index == 1 && pc_buffer[0] == 0x00) {
      pc_buffer_index = 0;
      return;
    }

    handleBuffer(&pcSerial, PC_EN_PIN, pc_buffer, pc_buffer_index);
    pc_buffer_index = 0;
  }

  // 接收 app Modbus 帧
  if (appSerial.available()) {
    while (appSerial.available()) {
      app_buffer[app_buffer_index++] = appSerial.read();
    }
    
    if (app_buffer_index == 1 && app_buffer[0] == 0x00) {
      app_buffer_index = 0;
      return;
    }

    handleBuffer(&appSerial, APP_EN_PIN, app_buffer, app_buffer_index);
    app_buffer_index = 0;
  }
}
