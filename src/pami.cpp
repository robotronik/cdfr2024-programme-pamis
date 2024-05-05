#include "pami.h"

Pami::Pami() : moteur_gauche(1, LEFT_STEP_PIN, LEFT_DIR_PIN),
               moteur_droit(1, RIGHT_STEP_PIN, RIGHT_DIR_PIN),
               sensor(&Wire, LPN_PIN, I2C_RST_PIN){

}

 //6 zones par couleur
Zone zones_bleues[6] = {
    {1, SERRE, -775, -1275, -1000, -1500, -550, -1050},
    {2, JARDINIERE, -550, -925, -775, -1275, -550, -925},
    {3, JARDINIERE, -925, -737.5, -775, -1275, -925, -737.5},
    {4, SERRE, 775, -1275, 550, -1500, 1000, -1050},
    {5, JARDINIERE, 550, -925, 775, -1275, 550, -925},
    {6, SERRE, 0, 1275, -225, 1500, 225, 1050} //zone non visée
};

Zone zones_jaunes[6] = {
    {1, SERRE, -775, 1275, -1000, 1500, -550, 1050},
    {2, JARDINIERE, -550, 925, -775, 1275, -550, 925},
    {3, JARDINIERE, -925, 737.5, -775, 1275, -925, 737.5},
    {4, SERRE, 775, 1275, 550, 1500, 1000, 1050},
    {5, JARDINIERE, 0, -1275, -225, -150, 225, -1500},
    {6, SERRE, 0, -1275, -225, -1500, 225, -1050}, //zone non visée
};

void Pami::init(){
    // Initialize I2C bus.
    Wire.begin();

    //Initialize WiFi and UDP
    //pami.connectToWiFi(ssid,password,server_ip,udp);
    
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

    // Initialize motors
    this->moteur_gauche.setAcceleration(ACCELERATION); 
    this->moteur_gauche.setMaxSpeed(MAX_SPEED);
    this->moteur_gauche.setSpeed(0);

    this->moteur_droit.setAcceleration(ACCELERATION);
    this->moteur_droit.setMaxSpeed(MAX_SPEED);
    this->moteur_droit.setSpeed(0);

    //Read HW ID
    /*this->id = digitalRead(DS1_PIN)
            + digitalRead(DS2_PIN)*2
            + digitalRead(DS3_PIN)*4;*/

    //Defining coordinates
    // Initially, all the pamis are on the same horizontal axis
    // and have the same orientation
    this->x = -925;
    this->theta = 0;
    
    switch(this->couleur){
        case BLEU:
            this->zone = zones_bleues[this->id-1];
            switch(this->id){
                case 1: 
                    this->y = -405;
                    break;
                case 2: 
                    this->y = -315;
                    break;
                case 3: 
                    this->y = -225;
                    break;
                case 4: 
                    this->y = -135;
                    break;
                case 5: 
                    this->y = -45;
                    break;
            }
            break;
        case JAUNE:
            this->zone = zones_jaunes[this->id];
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

/***********
*Capteur ToF
************/
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

void Pami::steerRad(Direction dir, float Dtheta){
    if (dir != LEFT && dir != RIGHT) return;
    this->moveDist(dir,Dtheta*DISTANCE_CENTRE_POINT_CONTACT_ROUE);
}

void Pami::goToPos(int x_target, int y_target){
    float Dx = x_target - this->x;
    float Dy = y_target - this->y;
    float distance = sqrt(Dx*Dx + Dy*Dy);

    float theta_target = atan2f(Dy,Dx);
    float Dtheta = theta_target - this->theta;
    
    // Normalisation de l'angle entre -pi et pi
    Dtheta = fmodf(Dtheta + M_PI, 2 * M_PI) - M_PI;

    Serial.print("Dx = "); Serial.print(Dx); Serial.print(" mm ");
    Serial.print("Dy = "); Serial.print(Dy); Serial.print(" mm ");
    Serial.print("theta_target = "); Serial.print(Dtheta); Serial.println(" rad");

    if (Dtheta != 0) {
        if (Dtheta > 0) {
            this->direction = LEFT;
        } else {
            this->direction = RIGHT;
        }
        this->steerRad(this->direction, abs(Dtheta));
    }
    
    this->moveDist(FORWARDS, distance);
    
}

void Pami::setPos(int x, int y){
    this->x = x;
    this->y = y;
}

bool Pami::inZone(){
    bool x_in_zone = (this->x >= this->zone.x_1 && this->x <= this->zone.x_2);
    bool y_in_zone = (this->y >= this->zone.y_1 && this->y <= this->zone.y_2);  
    return (x_in_zone && y_in_zone);
}

/**********************
 * Instructions moteurs
***********************/

//Add instruction to the list of instructions
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

//Clear the list of instructions
void Pami::clearInstructions(){
    this->nbInstructions = 0;
    for (int i=0; i<NB_MAX_INSTRUCTIONS; i++){
        this->listInstruction[i].dir = STOP;
        this->listInstruction[i].nbSteps = 0;
    }
    Serial.println("Instructions cleared");
}

//Translate next instruction into stepper commands via AccelStepper API
void Pami::setNextInstruction(){
    if (this->nbInstructions >= 0){
        Instruction nextInstruction = this->listInstruction[0];
        switch(nextInstruction.dir){
            case FORWARDS:
                this->moteur_gauche.move(nextInstruction.nbSteps);
                this->moteur_droit.move(nextInstruction.nbSteps);
                break;
            case BACKWARDS:
                this->moteur_gauche.move(-nextInstruction.nbSteps);
                this->moteur_droit.move(-nextInstruction.nbSteps);
                break;
            case LEFT:
                this->moteur_gauche.move(nextInstruction.nbSteps);
                this->moteur_droit.move(-nextInstruction.nbSteps);
                break;
            case RIGHT:
                this->moteur_gauche.move(-nextInstruction.nbSteps);
                this->moteur_droit.move(nextInstruction.nbSteps);
                break;
            case STOP:
                this->moteur_gauche.stop();
                this->moteur_droit.stop();
                break;
        }

        for (int i=0; i<this->nbInstructions; i++){
            this->listInstruction[i] = this->listInstruction[i+1];
        }

        this->nbInstructions--;
        Serial.print("Executing instruction: ");
        Serial.print(nextInstruction.dir);
        Serial.print(" ");
        Serial.println(nextInstruction.nbSteps);
    }
}


bool Pami::isMoving(){
    return (this->moteur_gauche.distanceToGo() != 0 || this->moteur_droit.distanceToGo() != 0);
}

/**********
 * WiFi
 * ********/
/**********************
 * Fonctions connectivité
***********************/
void Pami::connectToWiFi(){
    WiFi.mode(WIFI_STA); 
    WiFi.begin(SSID,PASSWORD);
    Serial.println("\nConnecting");
    while(WiFi.status() != WL_CONNECTED){
        Serial.print(".");
        delay(100);
    }
    
    Serial.print("\nConnected with IP adress: ");
    Serial.println(WiFi.localIP());

}
void Pami::UDPBeginAndSynchro(WiFiUDP *udp){
    char buf[30];
    (*udp).begin(LOCALPORT);
    Serial.printf("UDP Client : %s:%i \n", WiFi.localIP().toString().c_str(), LOCALPORT);
    
    Serial.print("Synchronizing...");
    while(time(nullptr) <= 100000){
        configTime(GMTOFFSET, DAYLOFFSET, SERVERIP);
        delay(4000);
    }
    Serial.println();
    Serial.printf("Synchronized with : %s \n", SERVERIP);
}
void Pami::SendUDPPacket(WiFiUDP* udp){
    char buf[20];
    (*udp).beginPacket(SERVERIP, SERVERPORT);
    sprintf(buf, "ESP32 send millis");
    (*udp).printf(buf);
    (*udp).endPacket();
}  
int Pami::ReadPacket(WiFiUDP* udp, char* packetBuffer){
    Serial.print(" Received packet from : "); Serial.println((*udp).remoteIP());
    int len = (*udp).read(packetBuffer, 12);
    if(len!=12){
        Serial.println("Packet not received correctly");
        return -1;
    }
    else{
        packetBuffer[len] = '\0'; // Ensure null-termination
        Serial.printf("Data : %s\n", packetBuffer);
        Serial.println();
        return 1;
    }
}
/**********************
 * Fonctions affichage
***********************/
void Pami::printPos(){
    Serial.print("x = "); Serial.print(this->x); Serial.print(" mm ");
    Serial.print("y = "); Serial.print(this->y); Serial.print(" mm ");
    Serial.print("theta = "); Serial.print(this->theta); Serial.println(" rad");
}

void Pami::printLocalTime(struct tm* timeinfo){
    Serial.print("Time: ");
    Serial.print(timeinfo->tm_hour);
    Serial.print(":");
    Serial.print(timeinfo->tm_min);
    Serial.print(":");
    Serial.println(timeinfo->tm_sec);
}   
