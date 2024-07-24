#include <Arduino.h>

#define encA 8
#define encB 9

void setup()
{
  Serial.begin(115200);

  pinMode(encA, INPUT);
  pinMode(encB, INPUT);
}

void loop()
{
  int enca = digitalRead(encA);
  int encb = digitalRead(encB);

  Serial.print(enca);
  Serial.println(encb);
}
