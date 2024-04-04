#include <Arduino.h>
const int DIR = 5;//Droite 14; //GPIO_NUM_13;
const int STEP = 4;//Droite 13; //GPIO_NUM_12;
const int  steps_per_rev = 200;

void setup()
{
  Serial.begin(115200);
  pinMode(STEP, OUTPUT);
  pinMode(DIR, OUTPUT);
  Serial.println("Setup done");
}

void loop()
{
  digitalWrite(DIR, HIGH);
  Serial.println("Spinning Clockwise...");


  for (int i = 0; i < steps_per_rev; i++)
  {
    digitalWrite(STEP, HIGH);
    delayMicroseconds(2000);
    digitalWrite(STEP, LOW);
    delayMicroseconds(2000);
  }
  delay(1000);

  digitalWrite(DIR, LOW);
  Serial.println("Spinning Anti-Clockwise...");

  for (int i = 0; i < steps_per_rev; i++)
  {
    digitalWrite(STEP, HIGH);
    delayMicroseconds(1000);
    digitalWrite(STEP, LOW);
    delayMicroseconds(1000);
  }
  delay(1000);
}