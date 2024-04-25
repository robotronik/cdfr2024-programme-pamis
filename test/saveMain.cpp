/* Includes ------------------------------------------------------------------*/
#include <Arduino.h>
#include <Wire.h>
#include "pami.h"
#include "soc/rtc_wdt.h"

// Components
Pami pami;

bool runMotors;
bool dodgeMotors = false;

void gestionMoteurs(void *pvParameters)
{

  for(;;){
	  switch(pami.direction){
      case FORWARDS:
        runMotors = true;
        pami.moteur_droit.setDirection(CW);
        pami.moteur_gauche.setDirection(CW);
        pami.x += (int)(cos(pami.orientation) * DIAMETRE_ROUE*M_PI/STEPS_PER_REV);
        pami.y += (int)(sin(pami.orientation) * DIAMETRE_ROUE*M_PI/STEPS_PER_REV);
        break;
      
      case BACKWARDS:
        runMotors = true;
        pami.moteur_droit.setDirection(CCW);
        pami.moteur_gauche.setDirection(CCW);
        pami.x -= (int)(cos(pami.orientation) * DIAMETRE_ROUE*M_PI/STEPS_PER_REV);
        pami.y -= (int)(sin(pami.orientation) * DIAMETRE_ROUE*M_PI/STEPS_PER_REV);
        break;
      case FORWARDS:
        runMotors = true;
        pami.moteur_droit.setDirection(CW);
        pami.moteur_gauche.setDirection(CW);
        pami.x += (int)(cos(pami.orientation) * DIAMETRE_ROUE*M_PI/STEPS_PER_REV);
        pami.y += (int)(sin(pami.orientation) * DIAMETRE_ROUE*M_PI/STEPS_PER_REV);
        break;
      
      case BACKWARDS:
        runMotors = true;
        pami.moteur_droit.setDirection(CCW);
        pami.moteur_gauche.setDirection(CCW);
        pami.x -= (int)(cos(pami.orientation) * DIAMETRE_ROUE*M_PI/STEPS_PER_REV);
        pami.y -= (int)(sin(pami.orientation) * DIAMETRE_ROUE*M_PI/STEPS_PER_REV);
        break;

      case LEFT:
        runMotors = true;
        pami.moteur_droit.setDirection(CCW);
        pami.moteur_gauche.setDirection(CW);
        pami.orientation -= (DIAMETRE_ROUE*M_PI/STEPS_PER_REV) / DISTANCE_CENTRE_POINT_CONTACT_ROUE;
        break;
      case LEFT:
        runMotors = true;
        pami.moteur_droit.setDirection(CCW);
        pami.moteur_gauche.setDirection(CW);
        pami.orientation -= (DIAMETRE_ROUE*M_PI/STEPS_PER_REV) / DISTANCE_CENTRE_POINT_CONTACT_ROUE;
        break;

      case RIGHT:
        runMotors = true;
        pami.moteur_droit.setDirection(CW);
        pami.moteur_gauche.setDirection(CCW);
        pami.orientation += (DIAMETRE_ROUE*M_PI/STEPS_PER_REV) / DISTANCE_CENTRE_POINT_CONTACT_ROUE;
        break;
      case RIGHT:
        runMotors = true;
        pami.moteur_droit.setDirection(CW);
        pami.moteur_gauche.setDirection(CCW);
        pami.orientation += (DIAMETRE_ROUE*M_PI/STEPS_PER_REV) / DISTANCE_CENTRE_POINT_CONTACT_ROUE;
        break;

      case STOP:
        runMotors = false;
        break;
	  }
      case STOP:
        runMotors = false;
        break;
	  }
    

    if (runMotors){
      digitalWrite(RIGHT_STEP_PIN, HIGH);
      digitalWrite(LEFT_STEP_PIN, HIGH);
      vTaskDelay(pdMS_TO_TICKS(2));
      digitalWrite(RIGHT_STEP_PIN, HIGH);
      digitalWrite(LEFT_STEP_PIN, HIGH);
      vTaskDelay(pdMS_TO_TICKS(2));

      pami.nbStepsDone++;
      pami.nbStepsDone++;

      digitalWrite(RIGHT_STEP_PIN, LOW);	
      digitalWrite(LEFT_STEP_PIN, LOW);
      vTaskDelay(pdMS_TO_TICKS(2));
      digitalWrite(RIGHT_STEP_PIN, LOW);	
      digitalWrite(LEFT_STEP_PIN, LOW);
      vTaskDelay(pdMS_TO_TICKS(2));
    }
    else if(dodgeMotors){
      pami.steerRad(LEFT, M_PI/2);
      pami.moveDist(FORWARDS, 100);
      dodgeMotors = false;
    }

    else{
      vTaskDelay(pdMS_TO_TICKS(1));
    }
  }
}

void gestionCapteur(void *pvParameters){
  vTaskDelay(pdMS_TO_TICKS(10));

  Serial.println("Début gestion capteur");
  
  uint16_t minDistance = INT16_MAX;
  uint8_t NewDataReady = 0;
  uint8_t status;
  uint8_t res = VL53L7CX_RESOLUTION_4X4;

  uint8_t i, j, k, l;
  uint8_t zones_per_line;

  VL53L7CX_ResultsData * Results = (VL53L7CX_ResultsData *)malloc(sizeof(VL53L7CX_ResultsData));
  TickType_t xLastWakeTime;

  for(;;){
    xLastWakeTime = xTaskGetTickCount();
    status = pami.sensor.vl53l7cx_check_data_ready(&NewDataReady);
    //Attente de données du capteur
    if ((!status) && (NewDataReady != 0)) {
      minDistance = INT16_MAX;
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
              if (distance < minDistance){
                minDistance = distance;
              }
            }
          }
        }   
      }
    }
    
    

    if (minDistance < THRESHOLD){
      runMotors = false;
      dodgeMotors = true;
      digitalWrite(LED_BUILTIN, HIGH);
    }
    else {
      runMotors = true;
      dodgeMotors = false;
      digitalWrite(LED_BUILTIN, LOW);
    }

    vTaskDelayUntil(&xLastWakeTime,100);
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

	pami.moveDist(FORWARDS, 100);

  //xTaskCreatePinnedToCore(gestionShutdown, "Gestion Shutdown", 100, NULL, configMAX_PRIORITIES, NULL,0);
  Serial.println("Fin Strat");
  for(;;){
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void setup()
{
  Serial.begin(115200);
  //pami.connectToWiFi("RaspberryRobotronik", "robotronik");
  Wire.begin();
  pami.id = 1;
  pami.init();
  
  xTaskCreate(gestionMoteurs, "Gestion Moteurs", 100000, NULL,  configMAX_PRIORITIES, NULL);
  //xTaskCreate(gestionCapteur, "Gestion Capteur", 100000, NULL, configMAX_PRIORITIES, NULL);
  xTaskCreate(strategie, "Stratégie", 100000, NULL, configMAX_PRIORITIES, NULL);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("Setup done");
  for(;;);
}

void loop()
{
}