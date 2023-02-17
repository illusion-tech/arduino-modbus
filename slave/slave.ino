#include "Arduino.h"
#include <SoftwareSerial.h>
#define RXPin        1  // Serial Receive pin
#define TXPin        0  // Serial Transmit pin
 
//RS485 control
#define SERIAL_COMMUNICATION_CONTROL_PIN 10 // Transmission set pin
#define RS485_TX_PIN_VALUE HIGH
#define RS485_RX_PIN_VALUE LOW
 
SoftwareSerial RS485Serial(RXPin, TXPin); // RX, TX
 
int byteSend;
 
void setup()  {
  Serial.begin(115200);
 
  pinMode(SERIAL_COMMUNICATION_CONTROL_PIN, OUTPUT);
  digitalWrite(SERIAL_COMMUNICATION_CONTROL_PIN, RS485_RX_PIN_VALUE);
  RS485Serial.begin(115200);   // set the data rate
}
String dataReceived;
bool isDataReceived = false;
void loop() {
    digitalWrite(SERIAL_COMMUNICATION_CONTROL_PIN, RS485_RX_PIN_VALUE);// Init receive
 
    if (RS485Serial.available()){
        dataReceived = RS485Serial.readString();
        Serial.print("Data received ");
        Serial.println(dataReceived);
        isDataReceived = true;
        delay(10);
    }
 
    if (isDataReceived){
        delay(111);
        digitalWrite(SERIAL_COMMUNICATION_CONTROL_PIN, RS485_TX_PIN_VALUE); // Init transmit
        Serial.println("Send response!");
        RS485Serial.print("OK --> ");
        RS485Serial.println(dataReceived);
        isDataReceived = false;
    }
 
    delay(111);
}