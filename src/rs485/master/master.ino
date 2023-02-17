#include "Arduino.h"
#include <SoftwareSerial.h>
#define RXPin 1 // Serial Receive pin
#define TXPin 0 // Serial Transmit pin

// RS485 control
#define SERIAL_COMMUNICATION_CONTROL_PIN 10 // Transmission set pin

#define INTERRUPT_BUTTON_PIN 9 // Boot button pin

SoftwareSerial RS485Serial(RXPin, TXPin); // RX, TX

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
    RS485Serial.begin(115200); // set the data rate
    setReceiveMode();
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_BUTTON_PIN), pin_ISR, FALLING);
}

void pin_ISR()
{
    setTrasmitMode(); // Now trasmit
    Serial.println("Send data!");
    RS485Serial.print("Hello world!"); // Send message
    setReceiveMode();                  // Init receive
}

void loop()
{
    if (RS485Serial.available())
    {
        Serial.println("Response available!");
        Serial.println(RS485Serial.read());
    }
    delay(100);
}