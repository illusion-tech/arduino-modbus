#include "Arduino.h"
#include <SoftwareSerial.h>
#define RXPin        1  // Serial Receive pin
#define TXPin        0  // Serial Transmit pin
 
//RS485 control
#define SERIAL_COMMUNICATION_CONTROL_PIN 10 // Transmission set pin
#define RS485_TX_PIN_VALUE HIGH
#define RS485_RX_PIN_VALUE LOW

 
SoftwareSerial RS485Serial(RXPin, TXPin); // RX, TX
 
void setup()  {
  Serial.begin(115200);
 
  pinMode(SERIAL_COMMUNICATION_CONTROL_PIN, OUTPUT);
  digitalWrite(SERIAL_COMMUNICATION_CONTROL_PIN, RS485_RX_PIN_VALUE);
  RS485Serial.begin(115200);   // set the data rate
 
  delay(500);
 
  digitalWrite(SERIAL_COMMUNICATION_CONTROL_PIN, RS485_TX_PIN_VALUE); // Now trasmit
  Serial.println("Send data!");
  RS485Serial.print("Hello world!"); // Send message
}
 
void loop() {
      digitalWrite(SERIAL_COMMUNICATION_CONTROL_PIN, RS485_RX_PIN_VALUE);  // Disable RS485 Transmit
 
        if (RS485Serial.available()){
            Serial.println("Response available!");
            Serial.println(RS485Serial.readString());
        }                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               
 
      delay(100);
}