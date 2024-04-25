#include "pami.h"

Pami::Pami() : moteur_droit(RIGHT_DIR_PIN, RIGHT_STEP_PIN, STEPS_PER_REV),
               moteur_gauche(LEFT_DIR_PIN, LEFT_STEP_PIN, STEPS_PER_REV),
               sensor(&Wire, LPN_PIN, I2C_RST_PIN){

}
void Pami::init(){
    // Initialize I2C bus.
    Wire.begin();

    //Initialize RF module
    //radio.init();
    
    // Enable PWREN pin if present
    if (PWREN_PIN >= 0) {
        pinMode(PWREN_PIN, OUTPUT);
        digitalWrite(PWREN_PIN, HIGH);
        delay(10);
    }

    // Configure VL53L7CX component.
    this->sensor.begin();

    this->sensor.init_sensor();
    this->sensor.vl53l7cx_set_ranging_frequency_hz(FREQUENCY_HZ);

    // Start Measurements
    this->sensor.vl53l7cx_start_ranging();

    pinMode(LED_BUILTIN, OUTPUT);

    //Read HW ID
    /*this->id = digitalRead(DS1_PIN)
            + digitalRead(DS2_PIN)*2
            + digitalRead(DS3_PIN)*4;*/

    //Defining coordinates
    // Initially, all the pamis are on the same horizontal axis
    // and have the same orientation
    this->x = -925;
    this->orientation = M_PI/2;
    
    switch(this->id){
        case 1:
        this->y = -405;
        this->x_zone = -775;
        this->y_zone = -1275; 
        break;
        case 2:
        this->y = -315;
        this->x_zone = -550;
        this->y_zone = -925; 
        break;
        case 3:
        this->y = -225;
        this->x_zone = -925;
        this->y_zone = -737.5; 
        break;
        case 4:
        this->y = -135;
        this->x_zone = 775;
        this->y_zone = -1275; 
        break;
        case 5:
        this->y = -45;
        this->x_zone = 0;
        this->y_zone = 1275; 
        break;
    }
    this->nbStepsDone=0;
    this->nbStepsToDo=0;
    this->direction=STOP;

    Serial.println("Setup done pami.init()");
}

void Pami::shutdown(){
    // Disable PWREN pin if present
    if (PWREN_PIN >= 0) {
        digitalWrite(PWREN_PIN, LOW);
    }
    // Stop Measurements
    this->sensor.vl53l7cx_stop_ranging();
    // Shutdown I2C bus.
    Wire.end();
    // Shutdown SPI bus.
    SPI.end();
}

//Communication
/*
WiFiClient Pami::initWiFi(const char* ssid, const char* password){
    WiFi.mode(WIFI_STA); //Optional
    WiFi.begin(ssid, password);
    Serial.println("\nConnecting");

    while(WiFi.status() != WL_CONNECTED){
        Serial.print(".");
        delay(100);
    }

    char* msg;
    sprintf(msg, "Connected to %s with IP: ", WiFi.SSID());
    Serial.print(msg);
    Serial.println(WiFi.localIP());

    WiFiClient client;
    client.localIP() = WiFi.localIP();
    client.remoteIP();
    return client;
}
*/

/*
*Capteur ToF
*/
void Pami::getSensorData(VL53L7CX_ResultsData *Results){
    uint8_t NewDataReady = 0;
    uint8_t status;

    //Attente de données du capteur
    do {
        status = this->sensor.vl53l7cx_check_data_ready(&NewDataReady);
    } while (!NewDataReady);

    if ((!status) && (NewDataReady != 0)) {
        //Chargement des données dans Results
        status = this->sensor.vl53l7cx_get_ranging_data(Results);
    }
}

uint16_t Pami::getMin(){
    uint16_t minDistance = INT16_MAX;
    uint8_t NewDataReady = 0;
    uint8_t status;
    uint8_t res = VL53L7CX_RESOLUTION_4X4;

    uint8_t i, j, k, l;
    uint8_t zones_per_line;

    status = this->sensor.vl53l7cx_check_data_ready(&NewDataReady);
    VL53L7CX_ResultsData * Results = (VL53L7CX_ResultsData *)malloc(sizeof(VL53L7CX_ResultsData));

    //Attente de données du capteur
    if ((!status) && (NewDataReady != 0)) {
      //Chargement des données dans Results
      status = this->sensor.vl53l7cx_get_ranging_data(Results);
      
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

    free(Results);
    return minDistance;
}

/*************
 * Déplacement
 *************/

void Pami::moveDist(Direction dir, int distance_mm){
    this->direction = dir;
    this->nbStepsToDo = abs(distance_mm*STEPS_PER_REV/(M_PI*DIAMETRE_ROUE));
    Serial.println(this->nbStepsToDo);
    this->waitForPos();
}


void Pami::steerRad(Direction dir, float orientation_rad){
    if (dir != LEFT && dir != RIGHT) return;
    this->direction = dir;
    this->nbStepsToDo = abs(orientation_rad*DISTANCE_CENTRE_POINT_CONTACT_ROUE*STEPS_PER_REV/(M_PI*DIAMETRE_ROUE));
    Serial.println(this->nbStepsToDo);
    this->waitForPos();
}

void Pami::goToPos(int x, int y){
    int Dx = x - this->x;
    int Dy = y - this->y;
    float distance;
    char msg[50];

    float orientation_rad = atan2(Dy,Dx);
    this->orientation = orientation_rad;
    if (orientation_rad != 0){
        int dir = (Dx*cos(orientation_rad) + Dy*sin(orientation_rad));
        if (dir > 0) {
            this->direction = RIGHT;
            Serial.print("Turning Right: ");
        }
        else{
            this->direction = LEFT;
            Serial.print("Turning Left: ");
        }
        this->steerRad(this->direction, orientation_rad);
        
    }
    
    Serial.print("Moving Forward: ");
    this->moveDist(FORWARDS, sqrt(Dx*Dx + Dy*Dy));
    
}

void Pami::setPos(int x, int y){
    this->x = x;
    this->y = y;
}

void Pami::waitForPos(){
    while(this->nbStepsDone < this->nbStepsToDo){
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    this->nbStepsDone = 0;
    this->nbStepsToDo = 0;
    this->direction=STOP;
}