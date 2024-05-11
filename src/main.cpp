/* Includes ------------------------------------------------------------------*/
#include <Arduino.h>
#include <Wire.h>
#include "pami.h"
#include "soc/rtc_wdt.h"
#include "AccelStepper.h"

/*
/!\ DEFINE WETHER THIS PROGRAM MUST RUN AS A TEST OR A MATCH AND WITH OR WITHOUT ACTIVE AVOIDANCE /!\
*/

// Components
Pami pami;
// Objects & Variables
WiFiUDP udp;


void gestionMoteur(void *pvParameters){
  vTaskDelay(pdMS_TO_TICKS(10));

  TickType_t xLastWakeTime;

  for(;;){
    //Serial.println("Gestion moteur");
    xLastWakeTime = xTaskGetTickCount();

    if (pami.state != BLOCKED){
      pami.moteur_gauche.run();
      pami.moteur_droit.run();

      if (pami.state == MOVING){
        //Distance parcourue par les roues (mm, >0)
        double distance_parcourue = (pami.moteur_droit.currentPosition()+pami.moteur_gauche.currentPosition())*DIAMETRE_ROUE*M_PI/(2*(double)STEPS_PER_REV);
        double Dtheta = (pami.moteur_gauche.currentPosition()-pami.moteur_droit.currentPosition())*DIAMETRE_ROUE*M_PI/((double)STEPS_PER_REV*DISTANCE_ROUES);

        pami.x = pami.x_last + cos(pami.theta_last)*distance_parcourue;
        pami.y = pami.y_last + sin(pami.theta_last)*distance_parcourue;
      
        pami.theta = normalizeAngle(pami.theta_last + Dtheta);
      }
    }

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
  }
}

void gestionCapteur(void *pvParameters){
  vTaskDelay(pdMS_TO_TICKS(10));

  uint8_t NewDataReady = 0;
  uint8_t status;

  uint8_t i;
  uint8_t  res = SENSOR_RES;
  
  TickType_t xLastWakeTime;
  uint16_t minValue = INT16_MAX;
  double distanceLeftToGo; 
  uint16_t threshold; 

  for(;;){
    xLastWakeTime = xTaskGetTickCount();
    VL53L7CX_ResultsData * Results = (VL53L7CX_ResultsData *)malloc(sizeof(VL53L7CX_ResultsData));

    if ((pami.state == MOVING || pami.state == BLOCKED) && pami.sensorIsActive){

      status = pami.sensor.vl53l7cx_check_data_ready(&NewDataReady);

      //Attente de données du capteur
      if ((!status) && (NewDataReady != 0)) {
        minValue = INT16_MAX;
        //Chargement des données dans Results
        status = pami.sensor.vl53l7cx_get_ranging_data(Results);
    
        //Calul distance minimum
        //On ne regarde que la ligne de "haut" matrice de mesure
        for (i=11; i>7; i--){              
          //On prend en compte uniquement les zones où le mesure est valide (status = 5 ou 9)
          uint8_t zoneStatus = Results->target_status[VL53L7CX_NB_TARGET_PER_ZONE * i];
          if (zoneStatus == 5 || zoneStatus == 9){
            uint16_t distance = Results->distance_mm[VL53L7CX_NB_TARGET_PER_ZONE *i];
            if (distance < minValue){
              minValue = distance;
            }
          } 
        }
        if (minValue <= SENSOR_THRESHOLD){
          digitalWrite(LED_BUILTIN, HIGH);
          pami.obstacleDetected = true;
        }
        else{
          digitalWrite(LED_BUILTIN, LOW);
          pami.obstacleDetected = false;
        }
      }
    }
    free(Results);  
    //Delay according to sensor frequency
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5));
  }
}

time_t addSecondsToTime(time_t timeToAdd, int secondsToAdd) {
    struct tm timeStruct = *localtime(&timeToAdd); // Convertit time_t en struct tm

    timeStruct.tm_sec += secondsToAdd; // Ajoute les secondes
    mktime(&timeStruct); // Normalise la structure tm

    return mktime(&timeStruct); // Convertit la structure tm en time_t
}

void strategie(void *pvParameters){
  //Task parameters
  vTaskDelay(pdMS_TO_TICKS(5));
  TickType_t xLastWakeTime;
  //Connection variables
  unsigned long testID;
  time_t start_time=0;
  time_t end_time=0;
  time_t now = 0;
  int packetSize;
  char packetBuffer[255];
  for(;;){
    time(&now);
    xLastWakeTime = xTaskGetTickCount();
    switch(pami.state){

      case START:
        Serial.println("[STATE] START");

        while (!digitalRead(GPIO_NUM_5)){
          vTaskDelay(pdMS_TO_TICKS(1));
        }

        time(&now);
        start_time = now + 90; //addSecondsToTime(now, 90);
        end_time = start_time + 10;
        
        pami.nextState = WAIT_IDLE;
        Serial.println();
      break;

      case WAIT_IDLE:
        Serial.printf("    |--Start time: %d, current time:%d \r",start_time,now);
        if(start_time<=now){
          Serial.println();
          pami.nextState=IDLE;
          digitalWrite(nENABLE_PIN, LOW);
        }
        break;
        
      case IDLE:
        Serial.println("\n[STATE] Idle");
        if (end_time == 0){
          time(&now);
          end_time = now + 10;
        }

        digitalRead(DS1_PIN) ? pami.couleur = JAUNE : pami.couleur = BLEU;
        pami.id = 1+digitalRead(DS2_PIN)+2*digitalRead(DS3_PIN);

        Serial.printf("    |--PAMI: %s - %d\n",pami.couleur?"Jaune":"Bleu", pami.id);

      #ifdef EVITEMENT_ACTIF
      //if signal top départ
        if (!pami.inZone()){
          pami.moveDist(FORWARDS, 150);
          pami.nextState = STOPPED;
        }
        break;
      #endif
      #ifdef TEST_LINEAR
        pami.moveDist(FORWARDS, 1000);
        pami.nextState = STOPPED;
        break;
      #endif
      #ifdef TEST_ANGULAR
        pami.steerRad(RIGHT, M_PI);
        pami.nextState = STOPPED;
        break;
      #endif
      #ifdef EVITEMENT_PASSIF
        switch (pami.id){
          case 1:
            pami.moveDist(FORWARDS, 500);
            break;
          case 2:
            pami.moveDist(FORWARDS, 190);
            break;
          case 3:
            pami.moveDist(FORWARDS, 190);
          break;
        }

        switch (pami.couleur){
          case JAUNE:
            pami.steerRad(LEFT, M_PI/2);
            break;
          case BLEU:
            pami.steerRad(RIGHT, M_PI/2);
            break;
        }

        switch (pami.id){
          case 1:
            pami.moveDist(FORWARDS, 1000 -(SENSOR_THRESHOLD + 20));
            pami.moveDist(FORWARDS, SENSOR_THRESHOLD + 20);
            break;
          case 3:
            pami.moveDist(FORWARDS, 1020);
            pami.moveDist(FORWARDS, 10);
            break;
          case 2:
            pami.moveDist(FORWARDS, 660);
            switch(pami.couleur){
              case JAUNE:
                pami.steerRad(LEFT, M_PI/2);
                break;
              case BLEU:
                pami.steerRad(RIGHT, M_PI/2);
                break;
            }
            pami.moveDist(FORWARDS, 250);
          break;
        }

      #endif

      pami.nextState = STOPPED;
      break;


      case STOPPED:
        pami.direction = STOP;
        Serial.print("    |--:"); pami.printPos();
        //Zone non atteinte
        //if(!pami.inZone()){
          if(pami.nbInstructions > 0){
            if (pami.nbInstructions == 1) pami.sensorIsActive = false;
            pami.sendNextInstruction();
            pami.nextState = MOVING;

            Serial.println("\t[STATE] Moving");
          }
          else{
            #ifdef EVITEMENT_ACTIF
            digitalWrite(LED_BUILTIN, LOW);
            Serial.print("\n[STATE] Go for target: "); pami.printTarget();
            pami.goToPos(pami.zone.x_center, pami.zone.y_center);
            pami.nextState = STOPPED;
            #else 
            pami.nextState = END;
            #endif
          }
        //}

        /*
        //Zone atteinte
        else if (pami.inZone()) {
          Serial.println("\n[STATE] Zone atteinte"); 
          pami.clearInstructions();
          pami.direction = STOP;
          pami.nextState = END;
        }
        */
      break;

      case BLOCKED: 
        pami.moteur_gauche.setSpeed(0);
        pami.moteur_droit.setSpeed(0);
        if(end_time<=now){
          pami.nextState = END;
        }
        #ifdef EVITEMENT_PASSIF 
        else if (!pami.obstacleDetected){
          for (int i=0; i<NB_MAX_INSTRUCTIONS-1; i++){
            pami.listInstruction[i+1] = pami.listInstruction[i];
          }
          pami.listInstruction[0].dir = pami.currentInstruction.dir;
          pami.listInstruction[0].nbSteps  = pami.currentInstruction.nbSteps- abs(pami.moteur_droit.currentPosition()+pami.moteur_gauche.currentPosition())/2;
          pami.nextState = STOPPED;
        }
        #endif	
        else{
          pami.nextState = BLOCKED;
        }
      break;

      //Pami en mouvement
      case MOVING:
        //Obstacle détecté
        if(end_time<=now){
          pami.nextState = END;
        }
        else if(pami.obstacleDetected && pami.direction == FORWARDS){
          #ifdef EVITEMENT_PASSIF
          pami.nextState = BLOCKED;
          Serial.print("\n[STATE] Obstacle detected at: "); pami.printPos();
          #endif
          #ifdef EVITEMENT_ACTIF
          /*
          pami.steerRad(LEFT, M_PI/2); 
          pami.moveDist(FORWARDS, 150);
          */
          #endif
        }
        else if (!pami.motorsAreRunning()){
          pami.nextState = STOPPED;
        }
        else{
          pami.nextState = MOVING;
        }
      break;

      case END:
        Serial.println("\n[STATE] End");
        pami.clearInstructions();
        pami.direction = STOP;
        pami.nextState = END;
        digitalWrite(nENABLE_PIN, HIGH);

        #ifdef EVITEMENT_ACTIF
        double final_orientation;
        if (pami.zone.type == JARDINIERE){
          Serial.println("\n[Action de fin jardinière]");
          switch(pami.couleur){
            case JAUNE:
              switch(pami.id){
                case 2:
                  final_orientation = M_PI/2;
                  break;
                case 3:
                  final_orientation = M_PI;
                  break;
                default:
                  break;
              }
              break;

            case BLEU:
              switch (pami.id){
              case 2:
                final_orientation = -M_PI/2;
                break;
              case 3:
                final_orientation = -M_PI;
                break;
              default:
                break;
              }
              break;
          }

          double Dtheta = final_orientation - pami.theta;
          Direction dir;
          if (Dtheta > 0){
            dir = LEFT;
          }
          else if (Dtheta < 0){
            dir = RIGHT;
          }
          
          Serial.print("    |--:"); pami.printPos();
          pami.steerRad(dir, Dtheta);
          pami.sendNextInstruction();
          while(pami.motorsAreRunning()){
            vTaskDelay(pdMS_TO_TICKS(5));
          }

          Serial.print("    |--:"); pami.printPos();
          pami.moveDist(FORWARDS, 150);
          pami.sendNextInstruction();
          while(pami.motorsAreRunning()){
            vTaskDelay(pdMS_TO_TICKS(5));
          }
        }
        #endif

        Serial.println("\n========================================== END ==========================================");
        pami.printPos();  
        for(;;){vTaskDelay(pdMS_TO_TICKS(5));}
        break;

    }
    pami.state = pami.nextState;
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20));
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.println("========================================= SETUP =========================================");
  pami.init();

  xTaskCreate(gestionMoteur, "Gestion Moteur", 10000, NULL, configMAX_PRIORITIES, NULL);
  xTaskCreate(gestionCapteur, "Gestion Capteur", 10000, NULL, configMAX_PRIORITIES-1, NULL);
  xTaskCreate(strategie, "Stratégie", 100000, NULL, configMAX_PRIORITIES-2, NULL);

  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("========================================= START =========================================");
}

void loop(){}
  