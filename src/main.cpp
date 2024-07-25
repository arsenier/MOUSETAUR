#include <Arduino.h>

#include "Encoder.h"
#include "Motor.h"

void setup()
{
  Serial.begin(115200);

  encoderInit();
}

void loop()
{
  encoderTick();

  motorTick(2, 1);

  Serial.print(enc_phi_rad);
  Serial.println();
}
