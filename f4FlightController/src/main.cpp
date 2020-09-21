#include <Arduino.h>

void setup()
{
  delay(500);
  //Serial.begin(9600);
  SerialUSB.begin();

  pinMode(PC1, OUTPUT);
  pinMode(PC2, OUTPUT);
}

void loop()
{
  digitalWrite(PC1, HIGH);
  digitalWrite(PC2, LOW);
  SerialUSB.println("High");
  delay(500);
  digitalWrite(PC1, LOW);
  digitalWrite(PC2, HIGH);
  SerialUSB.println("Low Test");
  delay(500);
}