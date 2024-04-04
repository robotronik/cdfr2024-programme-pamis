/* Includes ------------------------------------------------------------------*/
#include <Arduino.h>
#include <Wire.h>
#include "pami.h"

Pami pami;
bool runMotors = true;

void setup(){
  Serial.begin(115200);
  pami.init();
  Serial.println("Start");
}

void loop(){
  if (runMotors){
    pami.MoteurDroit.spinSteps(1, 200, 1000);
    }
  
  VL53L7CX_ResultsData * Results = (VL53L7CX_ResultsData *)malloc(sizeof(VL53L7CX_ResultsData));

  pami.getSensorData(Results);
  if (minValue(Results) < THRESHOLD){
    runMotors = false;
    digitalWrite(LED_BUILTIN, HIGH);
  }
  else{
    runMotors = true;
    digitalWrite(LED_BUILTIN, LOW);
  }
}