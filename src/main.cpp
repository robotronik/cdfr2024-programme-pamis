/* Includes ------------------------------------------------------------------*/
#include <Arduino.h>
#include <Wire.h>
#include "pami.h"
#include "soc/rtc_wdt.h"

#define MATCH

// Components
Pami pami;
// Objects & Variables
WiFiUDP udp;



void gestionMoteurs(void *pvParameters)
{
  TickType_t xLastWakeTime;
  for(;;){
    //Serial.println("Moteur");
    xLastWakeTime = xTaskGetTickCount();

    if (pami.state == MOVING){
      switch(pami.direction){
        case FORWARDS:
          pami.moteur_droit.setDirection(CW);
          pami.moteur_gauche.setDirection(CW);
          pami.x += (cos(pami.theta) * DIAMETRE_ROUE*M_PI/STEPS_PER_REV);
          pami.y += (sin(pami.theta) * DIAMETRE_ROUE*M_PI/STEPS_PER_REV);
          break;
        
        case BACKWARDS:
          pami.moteur_droit.setDirection(CCW);
          pami.moteur_gauche.setDirection(CCW);
          pami.x -= (cos(pami.theta) * DIAMETRE_ROUE*M_PI/STEPS_PER_REV);
          pami.y -= (sin(pami.theta) * DIAMETRE_ROUE*M_PI/STEPS_PER_REV);
          break;

        case LEFT:
          pami.moteur_droit.setDirection(CCW);
          pami.moteur_gauche.setDirection(CW);
          pami.theta += (DIAMETRE_ROUE*M_PI/STEPS_PER_REV) / DISTANCE_CENTRE_POINT_CONTACT_ROUE;
          pami.theta = fmod(pami.theta + M_PI, 2 * M_PI) - M_PI;  
          break;

        case RIGHT:
          pami.moteur_droit.setDirection(CW);
          pami.moteur_gauche.setDirection(CCW);
          pami.theta -= (DIAMETRE_ROUE*M_PI/STEPS_PER_REV) / DISTANCE_CENTRE_POINT_CONTACT_ROUE;
          pami.theta = fmod(pami.theta + M_PI, 2 * M_PI) - M_PI; 
          break;

        case STOP:
          break;
      }

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
        Serial.printf("Pami.state=WAIT_INFO \r");
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
          Serial.println("Idle");
          pami.moveDist(FORWARDS, 100);
          pami.executeNextInstruction();
          pami.state = MOVING;
        }
        break;

      case MOVING:
        //Pami à l'arrêt  
        if (pami.nbStepsToDo == 0){
          pami.printPos();
          //Zone non atteinte
          if (!pami.inZone()){
            if (pami.nbInstructions != 0){
              pami.executeNextInstruction();
              pami.state = MOVING;
            }
            else{
              pami.state = GO_FOR_TARGET;
            }
          }

          //Zone atteinte
          else if (pami.inZone()) {
            Serial.println("Zone atteinte"); 
            pami.printPos();
            pami.clearInstructions();
            pami.state = IDLE;
          }
        }

        //Pami en mouvement
        else if (pami.nbStepsToDo != 0){
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
        Serial.print("Go for target: "); Serial.print("x = "); Serial.print(pami.zone.x_center ); Serial.print(" y = "); Serial.println(pami.zone.y_center);  
        pami.goToPos(pami.zone.x_center, pami.zone.y_center);
        pami.state = MOVING;
        pami.executeNextInstruction();
        Serial.println("Moving");
        break;

      //Détection d'obstacle ==> évitement
      case AVOID_OBSTACLE:
        Serial.print("Obstacle detected at:"); pami.printPos();
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

void mouvement(void *pvParameters){
  vTaskDelay(pdMS_TO_TICKS(10));
  Serial.println("Début Mouvement");

  pami.printPos();

  //pami.steerRad(RIGHT,M_PI);
  pami.moveDist(FORWARDS,1000);
  pami.executeNextInstruction();
  pami.state = MOVING;
  for(;;){
    if (pami.nbStepsToDo == 0){
      pami.printPos();
      pami.state = IDLE;  
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void setup()
{
  Serial.begin(115200);

  pami.id = 1;
  pami.couleur = JAUNE;
  pami.init();
  
  Serial.println("Setup done");
  xTaskCreatePinnedToCore(gestionMoteurs, "Gestion Moteurs", 100000, NULL,  configMAX_PRIORITIES, NULL, 0);
  xTaskCreatePinnedToCore(gestionCapteur, "Gestion Capteur", 10000, NULL, configMAX_PRIORITIES-1, NULL, 0);
  #ifdef TEST_MVT
  xTaskCreatePinnedToCore(mouvement, "Mouvement", 10000, NULL, configMAX_PRIORITIES-2, NULL,0);
  #endif
  #ifdef MATCH
  xTaskCreatePinnedToCore(strategie, "Stratégie", 10000, NULL, configMAX_PRIORITIES, NULL, 1);
  #endif
  digitalWrite(LED_BUILTIN, LOW);
}

void loop()
{

}
