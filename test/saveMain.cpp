/* Includes ------------------------------------------------------------------*/
#include <Arduino.h>
#include <Wire.h>
#include "pami.h"
#include "soc/rtc_wdt.h"

// Components
Pami pami;

bool runMotors = true;

void gestionMoteurs(void *pvParameters)
{
  for(;;)
  {
    if (runMotors){
      Serial.println("Moteurs");
      pami.goToPos(pami.x_zone,pami.y_zone);
      pami.Moteurs.spinDistance(pami.direction, pami.speed, pami.distance);
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }
}

void gestionCapteur(void *pvParameters){
  VL53L7CX_ResultsData * Results = (VL53L7CX_ResultsData *)malloc(sizeof(VL53L7CX_ResultsData));
  uint8_t NewDataReady = 0;
  uint8_t status;
  uint8_t res = VL53L7CX_RESOLUTION_4X4;

  int8_t i, j, k, l;
  uint8_t zones_per_line;

  uint16_t minDistance;
  TickType_t xLastWakeTime;

  for(;;){
    xLastWakeTime = xTaskGetTickCount();
    status = pami.sensor.vl53l7cx_check_data_ready(&NewDataReady);
    minDistance = INT16_MAX;
    if ((!status) && (NewDataReady != 0)) {
      //Chargement des données dans Results
      status = pami.sensor.vl53l7cx_get_ranging_data(Results);
      
      zones_per_line = (res == VL53L7CX_RESOLUTION_8X8) ? 8 : 4;

      Serial.println("Capteur");
      //Calul distance minimum
      for (j = 0; j < res; j += zones_per_line){
        for (l = 0; l < VL53L7CX_NB_TARGET_PER_ZONE; l++){
          for (k = (zones_per_line - 1); k >= 0; k--){

            //On prend en compte uniquement les zones où le mesure est valide (status = 5 ou 9)
            uint8_t zoneStatus = Results->target_status[(VL53L7CX_NB_TARGET_PER_ZONE * (j+k)) + l];
            if (zoneStatus == 5 || zoneStatus == 9){
              uint16_t distance = Results->distance_mm[(VL53L7CX_NB_TARGET_PER_ZONE * (j+k)) + l];
              if (distance < minDistance){
                minDistance = distance;
              }
            }
          }
        }   
      }

      if (minDistance < THRESHOLD){
        runMotors = false;
        digitalWrite(LED_BUILTIN, HIGH);
      }
      else {
        runMotors = true;
        digitalWrite(LED_BUILTIN, LOW);
      }

    }
  vTaskDelayUntil(&xLastWakeTime,100);
  }
}

void setup()
{
  Serial.begin(115200);
  //pami.connectToWiFi("RaspberryRobotronik", "robotronik");
  pami.init();

  xTaskCreate(gestionMoteurs, "Gestion Moteur", 10000, NULL,  tskIDLE_PRIORITY, NULL);
  xTaskCreate(gestionCapteur, "Gestion Capteur", 10000, NULL, configMAX_PRIORITIES, NULL);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("Setup done");
  while(1){}
}

void loop()
{
}