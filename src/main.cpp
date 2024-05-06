/* Includes ------------------------------------------------------------------*/
#include <Arduino.h>
#include <Wire.h>
#include "pami.h"
#include "soc/rtc_wdt.h"
#include "AccelStepper.h"

#define MATCH

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
    pami.moteur_gauche.run();
    pami.moteur_droit.run();
    switch(pami.direction){
      case FORWARDS:
        pami.x = pami.x_last + abs(pami.moteur_droit.currentPosition())*DIAMETRE_ROUE*M_PI/STEPS_PER_REV*sin(pami.theta_last);
        pami.y = pami.y_last + abs(pami.moteur_droit.currentPosition())*DIAMETRE_ROUE*M_PI/STEPS_PER_REV*cos(pami.theta_last);
        break;
      case BACKWARDS:
        pami.x = pami.x_last - abs(pami.moteur_droit.currentPosition())*DIAMETRE_ROUE*M_PI/STEPS_PER_REV*cos(pami.theta_last);
        pami.y = pami.y_last - abs(pami.moteur_droit.currentPosition())*DIAMETRE_ROUE*M_PI/STEPS_PER_REV*sin(pami.theta_last);
        break;
      case RIGHT:
        pami.theta = pami.theta_last + abs(pami.moteur_droit.currentPosition())*DISTANCE_CENTRE_POINT_CONTACT_ROUE*DIAMETRE_ROUE*M_PI/STEPS_PER_REV;
        pami.theta = fmodf(pami.theta+M_PI, 2*M_PI) - M_PI;
        break;
      case LEFT:
          pami.theta = pami.theta_last - abs(pami.moteur_droit.currentPosition())*DISTANCE_CENTRE_POINT_CONTACT_ROUE*DIAMETRE_ROUE*M_PI/STEPS_PER_REV;
          pami.theta = fmodf(pami.theta+M_PI, 2*M_PI) - M_PI;
          break;
      default:
          break;
    }
    pami.moteur_gauche.currentPosition();
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
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
    //Serial.println("Capteur");
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
  //Task parameters
  vTaskDelay(pdMS_TO_TICKS(5));
  TickType_t xLastWakeTime;
  //Connection variables
  unsigned long testID;
  time_t start_time;
  time_t now;
  int packetSize;
  char packetBuffer[255];
  for(;;){
    time(&now);
    xLastWakeTime = xTaskGetTickCount();
    switch(pami.state){

      case START:
        Serial.println("Pami.state=START");
        pami.connectToWiFi();
        pami.UDPBeginAndSynchro(&udp);
        pami.state=WAIT_INFO;
        Serial.println();

      case WAIT_INFO:
        //Serial.printf("Pami.state=WAIT_INFO \r");
        pami.SendUDPPacket(&udp);
        packetSize = udp.parsePacket();
        if(packetSize){
          if(pami.ReadPacket(&udp,packetBuffer)!=1){
            break;
          }
          else{
            const char delim = ':';
            char * token =strtok(packetBuffer,&delim);
            token[10]='\0';
            if (token != NULL) {
            start_time = (time_t)atoi(token);
            token =strtok(NULL,&delim); // Get the next token
            if (token != NULL) {
                char item=token[0];
                pami.couleur=(Couleur)atoi(&item);
                Serial.printf("Le pami est de couleur : %s",pami.couleur?"Jaune":"Bleu");
                Serial.println();
            }
            }
          pami.state=WAIT_IDLE;
          }
        }
        break;

      case WAIT_IDLE:
        Serial.printf(" Start time: %d, current time:%d \r",start_time,now);
        if(start_time<=now){
          Serial.println();
          pami.state=IDLE;
        }
        break;
        
      case IDLE:
      //if signal top départ
        if (!pami.inZone()){
          Serial.println("[STATE] Idle");
          pami.moveDist(FORWARDS, 100);
          pami.state = MOVING;
        }
        break;

      case MOVING:
        //Pami à l'arrêt  
        if (!pami.isMoving()){
          pami.printPos();
          //Zone non atteinte
          if(!pami.inZone()){
            if(pami.nbInstructions > 0){
              pami.setNextInstruction();
              pami.state = MOVING;
              Serial.println("[STATE] Moving");
            }
            else{
              pami.state = GO_FOR_TARGET;
            }
          }

          //Zone atteinte
          if (pami.inZone()) {
            Serial.println("[STATE] Zone atteinte"); 
            pami.printPos();
            pami.clearInstructions();
            pami.state = IDLE;
          }
        }

        //Pami en mouvement
        else if (pami.isMoving()){
          //Obstacle détecté
          if( pami.closestObstacle <= THRESHOLD && (pami.direction == FORWARDS || pami.direction == BACKWARDS)){
            pami.state = AVOID_OBSTACLE;
          }
          else{
            pami.state = MOVING;
          }
        }
        break;

      //Pami se dirige vers sa zone
      case GO_FOR_TARGET:
        Serial.print("[STATE] Go for target: "); Serial.print("x = "); Serial.print(pami.zone.x_center ); Serial.print(" y = "); Serial.println(pami.zone.y_center);  
        pami.goToPos(pami.zone.x_center, pami.zone.y_center);
        pami.state = MOVING;
        break;

      //Détection d'obstacle ==> évitement
      case AVOID_OBSTACLE:
        Serial.print("[STATE] Obstacle detected at:"); pami.printPos();
        pami.clearInstructions();
        pami.nbStepsToDo = 0;
        pami.steerRad(LEFT, M_PI/2); 
        pami.moveDist(FORWARDS, 100);
        pami.state = MOVING;
        break;

    }
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5));
  }
}

void mouvement(void *pvParameters){
  vTaskDelay(pdMS_TO_TICKS(10));
  Serial.println("Début Mouvement");

  pami.printPos();

  #ifdef TEST_LINEAR
  pami.moveDist(FORWARDS, 430);
  #endif
  #ifdef TEST_ANGULAR
  pami.steerRad(RIGHT, M_PI);
  #endif

  pami.setNextInstruction();
  pami.state = MOVING;
  for(;;){
    pami.moteur_gauche.run();
    pami.moteur_droit.run();
    vTaskDelay(1);
  }
}

void setup()
{
  Serial.begin(115200);

  pami.id = 1;
  pami.couleur = BLEU;
  pami.init();
  
  
  //xTaskCreatePinnedToCore(ReceptionUDP,"Reception Connexion",10000,NULL,configMAX_PRIORITIES,NULL,0);

  #ifdef TEST_LINEAR
  xTaskCreatePinnedToCore(mouvement, "Mouvement", 100000, NULL, tskIDLE_PRIORITY, NULL,0);
  #endif
  #ifdef TEST_ANGULAR
  xTaskCreatePinnedToCore(mouvement, "Mouvement", 100000, NULL, tskIDLE_PRIORITY, NULL,0);
  #endif
  #ifdef MATCH
  xTaskCreatePinnedToCore(gestionCapteur, "Gestion Capteur", 10000, NULL, configMAX_PRIORITIES-1, NULL,0);
  xTaskCreatePinnedToCore(gestionMoteur, "Gestion Moteur", 10000, NULL, configMAX_PRIORITIES, NULL,0);
  xTaskCreatePinnedToCore(strategie, "Stratégie", 100000, NULL, configMAX_PRIORITIES, NULL,1);
  #endif
  digitalWrite(LED_BUILTIN, LOW);
}

void loop(){}
  