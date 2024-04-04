#include <Arduino.h>
#define LEFT_DIR_PIN GPIO_NUM_5 
#define LEFT_STEP_PIN  GPIO_NUM_4

#define RIGHT_DIR_PIN GPIO_NUM_14 
#define RIGHT_STEP_PIN GPIO_NUM_13
const int  steps_per_rev = 200;

void setup()
{
  Serial.begin(115200);
  pinMode(LEFT_DIR_PIN, OUTPUT); pinMode(RIGHT_DIR_PIN, OUTPUT);
  pinMode(LEFT_STEP_PIN, OUTPUT); pinMode(RIGHT_STEP_PIN, OUTPUT);
  Serial.println("Setup done");
}

void loop()
{
  digitalWrite(LEFT_DIR_PIN, HIGH); digitalWrite(RIGHT_DIR_PIN, HIGH);
  Serial.println("Spinning Clockwise...");


  for (int i = 0; i < steps_per_rev; i++)
  {
    digitalWrite(LEFT_STEP_PIN, HIGH); digitalWrite(RIGHT_STEP_PIN, HIGH);  
    delayMicroseconds(2000);
    digitalWrite(LEFT_STEP_PIN, LOW); digitalWrite(RIGHT_STEP_PIN, LOW);
    delayMicroseconds(2000);
  }
  delay(1000);

  digitalWrite(LEFT_DIR_PIN, LOW); digitalWrite(RIGHT_DIR_PIN, LOW);
  Serial.println("Spinning Anti-Clockwise...");

  for (int i = 0; i < steps_per_rev; i++)
  {
    digitalWrite(LEFT_STEP_PIN, HIGH); digitalWrite(RIGHT_STEP_PIN, HIGH);  
    delayMicroseconds(1000);
    digitalWrite(LEFT_STEP_PIN, LOW); digitalWrite(RIGHT_STEP_PIN, LOW);
    delayMicroseconds(1000);
  }
  delay(1000);
}