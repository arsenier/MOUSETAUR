#include <Arduino.h>

#include "Encoder.h"

void setup()
{
  Serial.begin(115200);

  encoderInit();
}

void loop()
{
  encoderTick();

  Serial.print(enc_phi_rad);
  Serial.println();
}
