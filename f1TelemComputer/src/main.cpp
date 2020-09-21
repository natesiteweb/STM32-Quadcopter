#include <Arduino.h>

void setup() 
{
  pinMode(PA6, OUTPUT);
  pinMode(PA7, OUTPUT);

  Serial.begin(9600);
}

void loop() 
{
  digitalWrite(PA6, HIGH);
  digitalWrite(PA7, LOW);
  Serial.println("High");
  delay(500);
  digitalWrite(PA6, LOW);
  digitalWrite(PA7, HIGH);
  Serial.println("Low Test");
  delay(500);
}