/* Includes ------------------------------------------------------------------*/
#include <Arduino.h>
#include <Wire.h>
#include "pami.h"
#include "soc/rtc_wdt.h"

// Components
Pami pami;

void gestionMoteurs(void *pvParameters)
{
  TickType_t xLastWakeTime;
  for(;;){
    xLastWakeTime = xTaskGetTickCount();
	  switch(pami.direction){
      case FORWARDS:
        pami.moteur_droit.setDirection(CW);
        pami.moteur_gauche.setDirection(CW);
        pami.x += (cos(pami.orientation) * DIAMETRE_ROUE*M_PI/STEPS_PER_REV);
        pami.y += (sin(pami.orientation) * DIAMETRE_ROUE*M_PI/STEPS_PER_REV);
        break;
      
      case BACKWARDS:
        pami.moteur_droit.setDirection(CCW);
        pami.moteur_gauche.setDirection(CCW);
        pami.x -= (cos(pami.orientation) * DIAMETRE_ROUE*M_PI/STEPS_PER_REV);
        pami.y -= (sin(pami.orientation) * DIAMETRE_ROUE*M_PI/STEPS_PER_REV);
        break;

      case LEFT:
        pami.moteur_droit.setDirection(CCW);
        pami.moteur_gauche.setDirection(CW);
        pami.orientation -= (DIAMETRE_ROUE*M_PI/STEPS_PER_REV) / DISTANCE_CENTRE_POINT_CONTACT_ROUE;
        break;

      case RIGHT:
        pami.moteur_droit.setDirection(CW);
        pami.moteur_gauche.setDirection(CCW);
        pami.orientation += (DIAMETRE_ROUE*M_PI/STEPS_PER_REV) / DISTANCE_CENTRE_POINT_CONTACT_ROUE;
        break;

      case STOP:
        break;
	  }

    if (pami.state == MOVING){
      digitalWrite(RIGHT_STEP_PIN, HIGH);
      digitalWrite(LEFT_STEP_PIN, HIGH);
      vTaskDelay(pdMS_TO_TICKS(2));

      pami.nbStepsToDo--;

      digitalWrite(RIGHT_STEP_PIN, LOW);	
      digitalWrite(LEFT_STEP_PIN, LOW);
      vTaskDelay(pdMS_TO_TICKS(2));
    }
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5));
  }
}

void gestionCapteur(void *pvParameters){
  vTaskDelay(pdMS_TO_TICKS(10));

  uint8_t NewDataReady = 0;
  uint8_t status;

  int8_t i, j, k, l;
  uint8_t zones_per_line;

  uint8_t res = VL53L7CX_RESOLUTION_4X4;
  
  TickType_t xLastWakeTime;

  for(;;){
    xLastWakeTime = xTaskGetTickCount();

    if (pami.state != IDLE){
      VL53L7CX_ResultsData * Results = (VL53L7CX_ResultsData *)malloc(sizeof(VL53L7CX_ResultsData));

      status = pami.sensor.vl53l7cx_check_data_ready(&NewDataReady);
      //Attente de données du capteur
      if ((!status) && (NewDataReady != 0)) {
        pami.closestObstacle = INT16_MAX;
        //Chargement des données dans Results
        status = pami.sensor.vl53l7cx_get_ranging_data(Results);
        
        zones_per_line = (res == VL53L7CX_RESOLUTION_8X8) ? 8 : 4;

        //Serial.println("Capteur");
        //Calul distance minimum
        for (j = 0; j < res; j += zones_per_line){
          for (l = 0; l < VL53L7CX_NB_TARGET_PER_ZONE; l++){
            for (k = (zones_per_line - 1); k >= 0; k--){

              //On prend en compte uniquement les zones où le mesure est valide (status = 5 ou 9)
              uint8_t zoneStatus = Results->target_status[(VL53L7CX_NB_TARGET_PER_ZONE * (j+k)) + l];
              if (zoneStatus == 5 || zoneStatus == 9){
                uint16_t distance = Results->distance_mm[(VL53L7CX_NB_TARGET_PER_ZONE * (j+k)) + l];
                if (distance < pami.closestObstacle){
                  pami.closestObstacle = distance;
                }
              }
            }
          }   
        }

        /*
        Serial.print("Distance minimale: ");
        Serial.print(pami.closestObstacle);
        Serial.println(" mm");
        */
      }
      free(Results);
    }
    vTaskDelayUntil(&xLastWakeTime, configTICK_RATE_HZ/SENSOR_FREQUENCY_HZ);
  }
}

void gestionShutdown(void *pvParameters){
  for(;;){
    char * data = pami.readData(8);
    if (data == "shutdown"){
      pami.shutdown();
    }
  }
}

void strategie(void *pvParameters){
  vTaskDelay(pdMS_TO_TICKS(10));
  Serial.println("Début Strat");

  TickType_t xLastWakeTime;

  for(;;){
    xLastWakeTime = xTaskGetTickCount();
    switch(pami.state){

      case IDLE:
        Serial.println("Idle");
      //if signal top départ
        if (!pami.inZone())
          pami.state = GO_FOR_TARGET;
        break;

      case MOVING:
        //Zone non atteinte
        if (pami.nbStepsToDo == 0 && !pami.inZone()){
          if (pami.nbInstructions != 0){
            pami.executeNextInstruction();
            pami.state = MOVING;
          }
          else{
            pami.state = GO_FOR_TARGET;
          }
        }
        //Zone atteinte
        else if (pami.nbStepsToDo == 0){
          Serial.println("Zone atteinte");
          pami.clearInstructions();
          pami.state = IDLE;
        }

        //Pami en mouvement
        else if (pami.nbStepsToDo > 0 && pami.closestObstacle <= THRESHOLD && (pami.direction == FORWARDS || pami.direction == BACKWARDS)){
          pami.state = AVOID_OBSTACLE;
        }  

        else if (pami.nbStepsToDo > 0){
          pami.state = MOVING;
        }
        break;

      //Pami se dirige vers sa zone
      case GO_FOR_TARGET:
        Serial.println("Go for target");
        pami.goToPos(pami.x_zone, pami.y_zone);
        pami.state = MOVING;
        Serial.println("Moving");
        break;

      //Détection d'obstacle ==> évitement
      case AVOID_OBSTACLE:
        Serial.print("Obstacle detected at: x = "); Serial.print(pami.x); Serial.print(" y = "); Serial.print(pami.y); Serial.print(" Orientation = "); Serial.print(pami.orientation); Serial.println(" rad");
        pami.clearInstructions();
        pami.nbStepsToDo = 0;
        pami.steerRad(LEFT, M_PI/2); 
        pami.moveDist(FORWARDS, 100);
        pami.state = MOVING;
        Serial.println("Moving");
        break;
    }
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5));
  } 
}

void setup()
{
  Serial.begin(115200);
  //pami.connectToWiFi("RaspberryRobotronik", "robotronik");
  
  pami.id = 1;
  pami.init();
  
  xTaskCreatePinnedToCore(gestionMoteurs, "Gestion Moteurs", 100000, NULL,  configMAX_PRIORITIES, NULL,0);
  xTaskCreatePinnedToCore(gestionCapteur, "Gestion Capteur", 10000, NULL, configMAX_PRIORITIES-1, NULL,0);
  xTaskCreatePinnedToCore(strategie, "Stratégie", 100000, NULL, configMAX_PRIORITIES, NULL,1);

  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("Setup done");
  for(;;);
}

void loop()
{
}