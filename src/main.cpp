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

    //Distance parcourue par les roues (mm, >0)
    double distance_parcourue = abs(pami.moteur_droit.currentPosition())*DIAMETRE_ROUE*M_PI/STEPS_PER_REV;

    switch(pami.direction){
      case FORWARDS:
        pami.x = pami.x_last + cos(pami.theta_last)*distance_parcourue;
        pami.y = pami.y_last + sin(pami.theta_last)*distance_parcourue;
        break;
      case BACKWARDS:
        pami.x = pami.x_last - cos(pami.theta_last)*distance_parcourue;
        pami.y = pami.y_last - sin(pami.theta_last)*distance_parcourue;
        break;
      /*
      case RIGHT:
        pami.theta = pami.theta_last - distance_parcourue / DISTANCE_CENTRE_POINT_CONTACT_ROUE;
        pami.theta = normalizeAngle(pami.theta);
        break;
      case LEFT:
        pami.theta = pami.theta_last + distance_parcourue / DISTANCE_CENTRE_POINT_CONTACT_ROUE;
        pami.theta = normalizeAngle(pami.theta);
        break;
      */
      default:
        break;
    }
    
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
  }
}

void gestionCapteur(void *pvParameters){
  vTaskDelay(pdMS_TO_TICKS(10));

  uint8_t NewDataReady = 0;
  uint8_t status;
  VL53L7CX_ResultsData results;

  int8_t i;

  uint8_t res = VL53L7CX_RESOLUTION_8X8;
  
  TickType_t xLastWakeTime;

  for(;;){
    xLastWakeTime = xTaskGetTickCount();

    if (pami.state != IDLE){

      status = pami.sensor.vl53l7cx_check_data_ready(&NewDataReady);
      //Attente de données du capteur
      if ((!status) && (NewDataReady != 0)) {
        pami.closestObstacle = INT16_MAX;
        //Chargement des données dans Results
        status = pami.sensor.vl53l7cx_get_ranging_data(&results);
    
        //Calul distance minimum
        //On ne regarde que la partie "basse" (partie supérieure une fois monté sur le PAMI) de la matrice de mesure
        for (i=0; i<res/2 - 1; i++){              
          //On prend en compte uniquement les zones où le mesure est valide (status = 5 ou 9)
          uint8_t zoneStatus = results.target_status[VL53L7CX_NB_TARGET_PER_ZONE * i];
          if (zoneStatus == 5 || zoneStatus == 9){
            uint16_t distance = results.distance_mm[VL53L7CX_NB_TARGET_PER_ZONE *i];
            if (distance < pami.closestObstacle){
              pami.closestObstacle = distance;
            }
          } 
        }
      }
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
        Serial.println("[STATE] START");
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
          pami.moveDist(FORWARDS, 150);
          pami.state = MOVING;
        }
        break;

      case MOVING:
        //Pami à l'arrêt  
        if (!pami.isMoving()){
          pami.direction = STOP;
          pami.printPos();
          //Zone non atteinte
          if(!pami.inZone()){
            if(pami.nbInstructions > 0){
              pami.sendNextInstruction();
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
            pami.clearInstructions();
            pami.direction = STOP;
            pami.state = END;
          }
        }

        //Pami en mouvement
        else if (pami.isMoving()){
          //Obstacle détecté
          if( pami.closestObstacle <= THRESHOLD && (pami.direction == FORWARDS || pami.direction == BACKWARDS)){
            pami.direction = STOP;
            pami.state = AVOID_OBSTACLE;
          }
          else{
            pami.state = MOVING;
          }
        }
        break;

      //Pami se dirige vers sa zone
      case GO_FOR_TARGET:
        Serial.print("[STATE] Go for target: "); pami.printTarget();
        pami.goToPos(pami.zone.x_center, pami.zone.y_center);
        pami.state = MOVING;
        break;

      //Détection d'obstacle ==> évitement
      case AVOID_OBSTACLE:
        Serial.print("[STATE] Obstacle detected at:"); pami.printPos();
        pami.clearInstructions();
        pami.nbStepsToDo = 0;
        pami.steerRad(LEFT, M_PI/2); 
        pami.moveDist(FORWARDS, 150);
        pami.state = MOVING;
        break;

      case END:
        Serial.println("[STATE] End");
        pami.clearInstructions();
        pami.nbStepsToDo = 0;
        pami.direction = STOP;
        double final_orientation;
        if (pami.zone.type == JARDINIERE){
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
          
          pami.steerRad(dir, Dtheta);
          pami.sendNextInstruction();
          while(pami.isMoving()){
            vTaskDelay(pdMS_TO_TICKS(5));
          }

          pami.moveDist(FORWARDS, 150);
          pami.sendNextInstruction();
          while(pami.isMoving()){
            vTaskDelay(pdMS_TO_TICKS(5));
          }
        }
        for(;;) vTaskDelay(pdMS_TO_TICKS(5));
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

  pami.sendNextInstruction();
  pami.state = MOVING;
  while (pami.isMoving()){
    vTaskDelay(pdMS_TO_TICKS(5));
  }
  pami.printPos();
  for(;;) vTaskDelay(pdMS_TO_TICKS(5));
}

void setup()
{
  Serial.begin(115200);

  pami.id = 1;
  pami.couleur = JAUNE;
  pami.init();

  //xTaskCreatePinnedToCore(ReceptionUDP,"Reception Connexion",10000,NULL,configMAX_PRIORITIES,NULL,0);

  xTaskCreatePinnedToCore(gestionMoteur, "Gestion Moteur", 10000, NULL, configMAX_PRIORITIES, NULL,0);
  #ifdef TEST_LINEAR
  xTaskCreatePinnedToCore(mouvement, "Mouvement", 100000, NULL, tskIDLE_PRIORITY, NULL,0);
  #endif
  #ifdef TEST_ANGULAR
  xTaskCreatePinnedToCore(mouvement, "Mouvement", 100000, NULL, tskIDLE_PRIORITY, NULL,0);
  #endif
  #ifdef MATCH
  xTaskCreatePinnedToCore(gestionCapteur, "Gestion Capteur", 10000, NULL, configMAX_PRIORITIES-1, NULL,0);
  xTaskCreatePinnedToCore(strategie, "Stratégie", 100000, NULL, configMAX_PRIORITIES, NULL,1);
  #endif
  digitalWrite(LED_BUILTIN, LOW);
}

void loop(){}
  