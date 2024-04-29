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
    this->sensor.vl53l7cx_set_ranging_frequency_hz(SENSOR_FREQUENCY_HZ);

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
        this->zone.x_center = -775;
        this->zone.y_center = -1275;
        this->zone.x_1 = -1000;
        this->zone.y_1 = -1500;
        this->zone.x_2 = -550;
        this->zone.y_2 = -1050; 
        break;
        case 2:
        this->y = -315;
        this->zone.x_center = -550;
        this->zone.y_center = -925;
        this->zone.x_1 = -775;
        this->zone.y_1 = -1275; 
        this->zone.x_2 = -775;
        this->zone.y_2 = -1275; 
        break;
        case 3:
        this->y = -225;
        this->zone.x_center = -925;
        this->zone.y_center = -737.5; 
        this->zone.x_1 = -775;
        this->zone.y_1 = -1275;
        this->zone.x_2 = -775;
        this->zone.y_2 = -1275; 
        break;
        case 4:
        this->y = -135;
        this->zone.x_center = 775;
        this->zone.y_center = -1275; 
        this->zone.x_1 = -550;
        this->zone.y_1 = -1500; 
        this->zone.x_2 = 1000;
        this->zone.y_2 = -1050; 
        break;
        case 5:
        this->y = -45;
        this->zone.x_center = 0;
        this->zone.y_center = 1275; 
        this->zone.x_1 = -225;
        this->zone.y_1 = 150; 
        this->zone.x_2 = 225;
        this->zone.y_2 = 1500; 
        break;
    }
    this->nbStepsToDo=0;
    this->direction=STOP;
    this->state = IDLE; 
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

/*************
 * Déplacement
 *************/

void Pami::moveDist(Direction dir, int distance_mm){
    int nbSteps = abs(distance_mm*STEPS_PER_REV/(M_PI*DIAMETRE_ROUE));
    this->addInstruction(dir,  nbSteps);
}

void Pami::steerRad(Direction dir, float orientation_rad){
    if (dir != LEFT && dir != RIGHT) return;
    int nbSteps = abs(orientation_rad*DISTANCE_CENTRE_POINT_CONTACT_ROUE*STEPS_PER_REV/(M_PI*DIAMETRE_ROUE));
    this->addInstruction(dir,  nbSteps);
}

void Pami::goToPos(int x, int y){
    int Dx = x - this->x;
    int Dy = y - this->y;
    float distance;

    float orientation_rad = atan2(Dy,Dx);
    this->orientation = orientation_rad;
    if (orientation_rad != 0){
        int dir = (Dx*cos(orientation_rad) + Dy*sin(orientation_rad));
        if (dir > 0) {
            this->direction = RIGHT;
        }
        else{
            this->direction = LEFT;
        }
        this->steerRad(this->direction, orientation_rad);
        
    }
    
    this->moveDist(FORWARDS, sqrt(Dx*Dx + Dy*Dy));
    
}

void Pami::setPos(int x, int y){
    this->x = x;
    this->y = y;
}

bool Pami::inZone(){
    return (this->x == this->zone.x_center && this->y == this->zone.y_center);
}

void Pami::addInstruction(Direction dir, int nbSteps){
    if (this->nbInstructions+1<=NB_MAX_INSTRUCTIONS){
        this->listInstruction[this->nbInstructions].dir = dir;
        this->listInstruction[this->nbInstructions].nbSteps = nbSteps;
        Serial.print("Instruction added: ");
        Serial.print(this->listInstruction[this->nbInstructions].dir);
        Serial.print(" ");
        Serial.println(this->listInstruction[this->nbInstructions].nbSteps);
        this->nbInstructions++;
    }
}

void Pami::clearInstructions(){
    this->nbInstructions = 0;
    for (int i=0; i<NB_MAX_INSTRUCTIONS; i++){
        this->listInstruction[i].dir = STOP;
        this->listInstruction[i].nbSteps = 0;
    }
    Serial.println("Instructions cleared");
}

void Pami::executeNextInstruction(){
    if (this->nbInstructions >= 0){
        this->direction = this->listInstruction[0].dir;
        this->nbStepsToDo = this->listInstruction[0].nbSteps;

        for (int i=0; i<this->nbInstructions; i++){
            this->listInstruction[i] = this->listInstruction[i+1];
        }
        this->nbInstructions--;
        Serial.print("Executing instruction: ");
        Serial.print(this->direction);
        Serial.print(" ");
        Serial.println(this->nbStepsToDo);
    }
}
void Pami::connectToWiFi(const char* ssid,const char* password,const char* serverip,WiFiUDP udp){
    WiFi.mode(WIFI_STA); 
    WiFi.begin(ssid, password);
    Serial.println("\nConnecting");
    while(WiFi.status() != WL_CONNECTED){
        Serial.print(".");
        delay(100);
    }
    Serial.print("\nConnected with IP adress: ");
    Serial.println(WiFi.localIP());
    udp.begin(LOCALPORT);
 	Serial.printf("UDP Client : %s:%i \n", WiFi.localIP().toString().c_str(), LOCALPORT);
    configTime(GMTOFFSET, DAYLOFFSET, serverip);

}
