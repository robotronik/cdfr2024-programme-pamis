/* Includes ------------------------------------------------------------------*/
#include <Arduino.h>
#include <Wire.h>
#include "pami.h"
#include "soc/rtc_wdt.h"
#include "AccelStepper.h"

#define TEST_MVT

// Components
Pami pami;
// Objects & Variables
char packetBuffer[255];
unsigned int localPort = 9999;
const char *serverip = "raspitronik.local";
unsigned int serverport = 8888;
const char *ssid = "Poulet";
const char *password = "yolespotos2343";

void gestionMoteur(void *pvParameters){
  vTaskDelay(pdMS_TO_TICKS(10));

  TickType_t xLastWakeTime;

  for(;;){
    Serial.println("Gestion moteur");
    xLastWakeTime = xTaskGetTickCount();
    pami.moteur_gauche.run();
    pami.moteur_droit.run();
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
/*
//TODO: récupérer l'équipe jouée
void ReceptionUDP(void *pvParameters){
	WiFiUDP udp;
	time_t start_time;
	time_t now;
	
	char packetBuffer[255];
  	for(;;){
		time(&now);
		int packetSize = udp.parsePacket();
		switch(pami.state){
			case START:
			//if démarrage du WiFi
				// Connect to Wifi network.
 				WiFi.begin(ssid, password);
 				while (WiFi.status() != WL_CONNECTED) {
 					delay(500); Serial.print(F("."));
 				}
 				udp.begin(LOCALPORT);
 				Serial.printf("UDP Client : %s:%i \n", WiFi.localIP().toString().c_str(), LOCALPORT);
  				//init and get the time
  				configTime(GMTOFFSET, DAYLOFFSET, serverip);
				pami.state=WAIT_INFO;
				break;
			case WAIT_INFO:
					if (packetSize) {
							Serial.print(" Received packet from : "); Serial.println(udp.remoteIP());
							int len = udp.read(packetBuffer, 255);
							Serial.printf("Data : %s\n", packetBuffer);
							Serial.println();
							const char delim = ':';
							char * token =strtok(packetBuffer,&delim);
							start_time=atoi(token);
							char * token2 =strtok(packetBuffer,&delim);
							char item=token2[0];
							int team=atoi(&item);
							switch(team){
								case 0:
									pami.couleur=BLEU;
									break;
								case 1:
									pami.couleur=JAUNE;
									break;
							}
							pami.state=WAIT_IDLE;
					}
				break;
				case WAIT_IDLE:
				Serial.printf(" WAIT FOR IDLE %d,%d",start_time,now);
				if(start_time<=now){
					
					pami.state=IDLE;
				}
			break;
		}
			//Serial.print("[Client Connected] "); Serial.println(WiFi.localIP());
			udp.beginPacket(serverip, SERVERPORT);
			char buf[30];
			unsigned long testID = millis();
			sprintf(buf, "ESP32 send millis: %lu", testID);
			udp.printf(buf);
			udp.endPacket();
			vTaskDelay(pdMS_TO_TICKS(100));
	}
}
*/
void strategie(void *pvParameters){
  vTaskDelay(pdMS_TO_TICKS(10));
  Serial.println("Début Strat");

  TickType_t xLastWakeTime;

  for(;;){
    xLastWakeTime = xTaskGetTickCount();

    pami.moteur_gauche.run();
    pami.moteur_droit.run();

    switch(pami.state){

      case IDLE:
      //if signal top départ
        if (!pami.inZone()){
          Serial.println("Idle");
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
              Serial.println("Moving");
            }
            else{
              pami.state = GO_FOR_TARGET;
            }
          }

          //Zone atteinte
          if (pami.inZone()) {
            Serial.println("Zone atteinte"); 
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
        Serial.print("Go for target: "); Serial.print("x = "); Serial.print(pami.zone.x_center ); Serial.print(" y = "); Serial.println(pami.zone.y_center);  
        pami.goToPos(pami.zone.x_center, pami.zone.y_center);
        pami.state = MOVING;
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
  float dist = -500;
  pami.state = MOVING;
  for(;;){
    pami.moteur_gauche.run();
    pami.moteur_droit.run();
    if (!pami.isMoving()){
      Serial.println("Done");
      vTaskDelay(pdMS_TO_TICKS(1000));
      pami.moveDist(BACKWARDS,dist);
      pami.setNextInstruction();
    }
  }
}

void setup()
{
  Serial.begin(115200);

  pami.id = 1;
  pami.couleur = BLEU;
  pami.init();
  
  
  //xTaskCreatePinnedToCore(ReceptionUDP,"Reception Connexion",10000,NULL,configMAX_PRIORITIES,NULL,0);

  #ifdef TEST_MVT
  xTaskCreatePinnedToCore(mouvement, "Mouvement", 100000, NULL, tskIDLE_PRIORITY, NULL,0);
  #endif
  #ifdef MATCH
  xTaskCreatePinnedToCore(gestionCapteur, "Gestion Capteur", 10000, NULL, configMAX_PRIORITIES-1, NULL,0);
  xTaskCreatePinnedToCore(gestionMoteur, "Gestion Moteur", 10000, NULL, configMAX_PRIORITIES, NULL,0);
  xTaskCreatePinnedToCore(strategie, "Stratégie", 100000, NULL, configMAX_PRIORITIES, NULL,1);
  #endif

  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("Setup done");
}

void loop(){}
  