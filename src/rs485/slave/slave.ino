#include "Arduino.h"
#include <SoftwareSerial.h>
#define RXPin 1 // Serial Receive pin
#define TXPin 0 // Serial Transmit pin

// RS485 control
#define SERIAL_COMMUNICATION_CONTROL_PIN 10 // Transmission set pin

SoftwareSerial RS485Serial(RXPin, TXPin); // RX, TX

// receive data control
String dataReceived;
bool isDataReceived = false;

void setReceiveMode()
{
    digitalWrite(SERIAL_COMMUNICATION_CONTROL_PIN, LOW);
}

void setTrasmitMode()
{
    digitalWrite(SERIAL_COMMUNICATION_CONTROL_PIN, HIGH);
}

void setup()
{
    pinMode(SERIAL_COMMUNICATION_CONTROL_PIN, OUTPUT);
    Serial.begin(115200);
    RS485Serial.begin(115200);
    setReceiveMode();
}

void loop()
{
    //listen 485 serial data
    if (RS485Serial.available())
    {
        dataReceived = RS485Serial.read();
        Serial.print("Data received: ");
        Serial.println(dataReceived);
        isDataReceived = true;
        delay(10);
    }
    //send resp to master
    if (isDataReceived)
    {
        setTrasmitMode(); // switch transmit mode
        Serial.println("Send response!");
        RS485Serial.println(dataReceived);
        isDataReceived = false;
        setReceiveMode();
    }
}