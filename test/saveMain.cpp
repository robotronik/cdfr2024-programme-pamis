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

    if (pami.state == MOVING){
      //Distance parcourue par les roues (mm, >0)
      double distance_parcourue = (pami.moteur_droit.currentPosition()+pami.moteur_gauche.currentPosition())*DIAMETRE_ROUE*M_PI/(2*(double)STEPS_PER_REV);
      double Dtheta = (pami.moteur_droit.currentPosition()-pami.moteur_gauche.currentPosition())*DIAMETRE_ROUE*M_PI/((double)STEPS_PER_REV*DISTANCE_ROUES);

      pami.x = pami.x_last + cos(pami.theta_last)*distance_parcourue;
      pami.y = pami.y_last + sin(pami.theta_last)*distance_parcourue;
    
      pami.theta = normalizeAngle(pami.theta_last + Dtheta);
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
  double minValue = INT16_MAX;

  for(;;){
    xLastWakeTime = xTaskGetTickCount();
    VL53L7CX_ResultsData * Results = (VL53L7CX_ResultsData *)malloc(sizeof(VL53L7CX_ResultsData));

    if (pami.state != IDLE){

      status = pami.sensor.vl53l7cx_check_data_ready(&NewDataReady);
      //Attente de données du capteur
      if ((!status) && (NewDataReady != 0)) {
        minValue = INT16_MAX;
        //Chargement des données dans Results
        status = pami.sensor.vl53l7cx_get_ranging_data(Results);
    
        //Calul distance minimum
        //On ne regarde que la partie "basse" (partie supérieure une fois monté sur le PAMI) de la matrice de mesure
        for (i=0; i<res/2 - 1; i++){              
          //On prend en compte uniquement les zones où le mesure est valide (status = 5 ou 9)
          uint8_t zoneStatus = Results->target_status[VL53L7CX_NB_TARGET_PER_ZONE * i];
          if (zoneStatus == 5 || zoneStatus == 9){
            uint16_t distance = Results->distance_mm[VL53L7CX_NB_TARGET_PER_ZONE *i];
            if (distance < pami.closestObstacle){
              minValue = distance;
            }
          } 
        }
      }
    }
    free(Results);  
    pami.closestObstacle = minValue;
    //Delay according to sensor frequency
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(CONFIG_FREERTOS_HZ/SENSOR_FREQUENCY_HZ));
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
        Serial.println("\n[STATE] Idle");
      #ifdef MATCH
      //if signal top départ
        if (!pami.inZone()){
          pami.moveDist(FORWARDS, 150);
          pami.state = MOVING;
        }
        break;
      #endif
      #ifdef TEST_LINEAR
        pami.moveDist(FORWARDS, 430);
        pami.state = MOVING;
        break;
      #endif
      #ifdef TEST_ANGULAR
        pami.steerRad(RIGHT, M_PI);
        pami.state = MOVING;
        break;
      #endif

      case STOPPED:
        pami.direction = STOP;
        Serial.print("    |--:"); pami.printPos();
        //Zone non atteinte
        if(!pami.inZone()){
          if(pami.nbInstructions > 0){
            pami.sendNextInstruction();
            pami.state = MOVING;
            Serial.println("\t[STATE] Moving");
          }
          else{
            #ifdef MATCH
            digitalWrite(LED_BUILTIN, LOW);
            Serial.print("\n[STATE] Go for target: "); pami.printTarget();
            pami.goToPos(pami.zone.x_center, pami.zone.y_center);
            pami.state = STOPPED;
            #else 
            pami.state = END;
            #endif
          }
        }

        //Zone atteinte
        if (pami.inZone()) {
          Serial.println("\n[STATE] Zone atteinte"); 
          pami.clearInstructions();
          pami.direction = STOP;
          pami.state = END;
        }
      break;

      case BLOCKED:
        if(pami.closestObstacle > THRESHOLD){
          pami.goToPos(pami.zone.x_center, pami.zone.y_center);
          pami.state = STOPPED;
        }
        break;

      //Pami en mouvement
      case MOVING:
        //Obstacle détecté
        if (pami.closestObstacle <= THRESHOLD && (pami.direction == FORWARDS)){
          pami.state = BLOCKED;
          digitalWrite(LED_BUILTIN, HIGH);
          Serial.print("\n[STATE] Obstacle detected at: "); pami.printPos();
          pami.clearInstructions();
          /*
          pami.steerRad(LEFT, M_PI/2); 
          pami.moveDist(FORWARDS, 150);
          */
        }
        if (!pami.motorsAreRunning()){
          pami.state = STOPPED;
        }
        else{
          pami.state = MOVING;
        }
      break;

      case END:
        Serial.println("\n[STATE] End");
        pami.clearInstructions();
        pami.nbStepsToDo = 0;
        pami.direction = STOP;

        #ifdef MATCH
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
        for(;;) vTaskDelay(pdMS_TO_TICKS(5));
        break;

    }
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
  }
}

void setup()
{
  Serial.begin(115200);

  pami.id = 1;
  pami.couleur = BLEU;
  pami.init();


  xTaskCreatePinnedToCore(gestionMoteur, "Gestion Moteur", 10000, NULL, configMAX_PRIORITIES, NULL,0);
  xTaskCreatePinnedToCore(gestionCapteur, "Gestion Capteur", 10000, NULL, configMAX_PRIORITIES-2, NULL,0);
  xTaskCreatePinnedToCore(strategie, "Stratégie", 100000, NULL, configMAX_PRIORITIES-1, NULL,0);

  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("========================================= START =========================================");
}

void loop(){}
  