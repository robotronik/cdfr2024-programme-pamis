#include "pami.h"

Pami::Pami() : moteur_gauche(1, LEFT_STEP_PIN, LEFT_DIR_PIN),
               moteur_droit(1, RIGHT_STEP_PIN, RIGHT_DIR_PIN),
               sensor(&Wire, 0, 0){

}

 //6 zones par couleur
Zone zones_bleues[6] = {
    {1, SERRE, -775, -1275, -1000, -1500, -550, -1050},  //ok
    {2, JARDINIERE, -387.5, -1275, -550, -1500, -225, -1050},  //ok
    {3, JARDINIERE, -900, -737.5, -1000, -900, -800, -575},  //ok
    {4, SERRE, 775, -1275, 550, -1500, 1000, -1050},  //ok
    {5, JARDINIERE,  387.5, -1275, 550, -1500, 225, -1050},  //ok
    {6, SERRE, 0, 1275, -225, 1500, 225, 1050} //zone non visée
};

Zone zones_jaunes[6] = {
    {1, SERRE, -775, 1275, -1000, 1050, -550, 1500},  //ok
    {2, JARDINIERE, -387.5, 1275, -550, 1050, -225, 1500},  //ok
    {3, JARDINIERE, -900, 737.5, -1000, 575, -800, 900},  //ok
    {4, SERRE, 775, 1275, 550, 1050, 1000, 1500},  //ok
    {5, JARDINIERE, 387.5, 1275, 225, 1050, 550, 1500},  //ok
    {6, SERRE, 0, -1275, -225, -1500, 225, -1050}, //zone non visée
};

void Pami::init(){

    //Indicate that the PAMI is booting
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    pinMode(DS1_PIN, INPUT_PULLDOWN);
    pinMode(DS2_PIN, INPUT_PULLDOWN);
    pinMode(DS3_PIN, INPUT_PULLDOWN);
    pinMode(GPIO_NUM_5, INPUT_PULLUP);
    pinMode(nENABLE_PIN, OUTPUT);
    digitalWrite(nENABLE_PIN,HIGH);


    // Initialize I2C bus.
    Wire.begin();

    //Initialize WiFi and UDP
    //pami.connectToWiFi(ssid,password,server_ip,udp);
    
    // Enable PWREN pin if present
    if (PWREN_PIN != 0){
        pinMode(PWREN_PIN, OUTPUT);
        digitalWrite(PWREN_PIN, LOW);
        delay(10);
    }

    // Configure VL53L7CX component.
    this->sensor.begin();
    Serial.println("VL53L7CX component initialized.");
    this->sensor.init_sensor();
    this->sensor.vl53l7cx_set_ranging_frequency_hz(SENSOR_FREQUENCY_HZ);
    Serial.println("SATEL-VL53L7CX board initialized.");

    // Start Measurements
    this->sensor.vl53l7cx_start_ranging();
    Serial.println("VL53L7CX starting measures.");

    // Initialize motors
    this->moteur_gauche.setAcceleration(ACCELERATION); 
    this->moteur_gauche.setMaxSpeed(MAX_SPEED);
    this->moteur_gauche.setSpeed(0);

    this->moteur_droit.setAcceleration(ACCELERATION);
    this->moteur_droit.setMaxSpeed(MAX_SPEED);
    this->moteur_droit.setSpeed(0);

    Serial.println("Motors setup.");

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
            this->zone = zones_jaunes[this->id-1];
            switch(this->id){
                case 1: 
                    this->y = 405;
                    break;
                case 2: 
                    this->y = 315;
                    break;
                case 3: 
                    this->y = 225;
                    break;
                case 4: 
                    this->y = 135;
                    break;
                case 5: 
                    this->y = 45;
                    break;
            }
            break;
    }
    Serial.print("Start coordinates: ");  this->printPos();
    
    this->direction=STOP;
    this->state = ENTRY_STATE;
    this->sensorIsActive = true;

    digitalWrite(nENABLE_PIN,LOW);
    Serial.println("Setup done");  
}

void Pami::shutdown(){
    // Disable PWREN pin if present
    #ifdef PWRENPIN 
    digitalWrite(PWREN_PIN, LOW);
    #endif

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

//Ajoute l'instruction pour faire tourner les roues d'une valeur de distance_mm, dans le sens de rotation en accord avec la direction du déplacement du pami
void Pami::moveDist(Direction dir, double distance_mm){
    long nbSteps = abs(distance_mm*STEPS_PER_REV/(M_PI*DIAMETRE_ROUE));
    this->addInstruction(dir,  nbSteps);
}

void Pami::steerRad(Direction dir, double Dtheta){
    if (dir != LEFT && dir != RIGHT) return;
    this->Dtheta = Dtheta;
    this->moveDist(dir,Dtheta*DISTANCE_ROUES/2);
}

//Ajoute la série d'instruction rotation+déplacement linéaire nécessaire pour atteindre un point cible 
void Pami::goToPos(double x_target, double y_target){
    double Dx = x_target - this->x;
    double Dy = y_target - this->y;
    double distance = sqrt(Dx*Dx + Dy*Dy);

    double theta_target = atan2f(Dy,Dx);
    double Dtheta = theta_target - this->theta;
    
    // Normalisation de l'angle entre -pi et pi
    Dtheta = normalizeAngle(Dtheta);

    Serial.print("    |");
    Serial.print("--> dist = "); Serial.print(distance); Serial.print(" mm ");
    Serial.print("Dtheta = "); Serial.print(Dtheta); Serial.println(" rad");

    if (Dtheta > 0) {
        this->steerRad(LEFT, Dtheta);
    } else if (Dtheta < 0){
        this->steerRad(RIGHT, Dtheta);
    }
    
    this->moveDist(FORWARDS, distance);
}

void Pami::setPos(int x, int y){
    this->x = x;
    this->y = y;
}

bool Pami::inZone(){
    //x1 <= x <= x2 et y1 <= y <= y2
    bool x_in_zone = (this->x >= this->zone.x_1 && this->x <= this->zone.x_2);
    bool y_in_zone = (this->y >= this->zone.y_1 && this->y <= this->zone.y_2);  
    return (x_in_zone && y_in_zone);
}

/**********************
 * Instructions moteurs
***********************/

//Add instruction to the list of instructions
void Pami::addInstruction(Direction dir, long nbSteps){
    if (this->nbInstructions+1<=NB_MAX_INSTRUCTIONS){
        this->listInstruction[this->nbInstructions].dir = dir;
        this->listInstruction[this->nbInstructions].nbSteps = nbSteps;
        Serial.print("    |"); Serial.print("++> Instruction added: ");
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
    Serial.print("    |"); Serial.println(">>>Instructions cleared<<<");
}

//Send next instructions to steppers via AccelStepper API
void Pami::sendNextInstruction(){
    //Reset current position
    this->moteur_droit.setCurrentPosition(this->moteur_droit.currentPosition());
    this->moteur_gauche.setCurrentPosition(this->moteur_gauche.currentPosition());

    //Save last position (checkpoint)
    this->x_last = this->x;
    this->y_last = this->y;
    this->theta_last = this->theta;

    //Convention de cablage: les 2 moteurs sont cablés "fils noirs vers l'extérieur"
    if (this->nbInstructions >= 0){

        //Deactivate sensor for last instruction
        if (nbInstructions == 1){
            this->sensorIsActive = false;
            Serial.print("    |"); Serial.println("==> Deactivating sensor");
        }
        else this->sensorIsActive = true;

        Instruction nextInstruction = this->listInstruction[0];
        this->direction = nextInstruction.dir;
        long nbSteps; 

        switch(nextInstruction.dir){

            case BACKWARDS:
                nbSteps =  nextInstruction.nbSteps * DELTA_FORWARD;
                this->moteur_gauche.move(-nbSteps);
                this->moteur_droit.move(-nbSteps);
                break;

            case FORWARDS:
                nbSteps =  nextInstruction.nbSteps * DELTA_FORWARD;
                this->moteur_gauche.move(+nbSteps);
                this->moteur_droit.move(+nbSteps);
                break;

            case LEFT:
                nbSteps =  nextInstruction.nbSteps * DELTA_ROTATE;
                this->moteur_gauche.move(+nbSteps);
                this->moteur_droit.move(-nbSteps);
                break;
            
            case RIGHT:
                nbSteps =  nextInstruction.nbSteps * DELTA_ROTATE;
                this->moteur_gauche.move(-nbSteps);
                this->moteur_droit.move(+nbSteps);
                break;
            
            case STOP:
                this->moteur_gauche.setSpeed(0);
                this->moteur_droit.setSpeed(0);
                break;
        }

        for (int i=0; i<this->nbInstructions; i++){
            this->listInstruction[i] = this->listInstruction[i+1];
        }
        this->nbInstructions--;
        Serial.print("    |"); Serial.print("==> Executing instruction: ");
        Serial.print(nextInstruction.dir);
        Serial.print(" ");
        Serial.println(nextInstruction.nbSteps);
        this->currentInstruction = nextInstruction;
    }
}

//Save next instructions |/!\ Current instruction is not saved ! /!\|
int Pami::saveInstructions(Instruction* dst){
    int nbInstructionsSaved = 0;
    if (this->nbInstructions == 0) return nbInstructionsSaved;
    for (int i=0;i<this->nbInstructions; i++){
        nbInstructionsSaved ++;
        dst[i] = this->listInstruction[i];
    }
    Serial.printf("    |>>>Saved %d instructions<<<\n", nbInstructionsSaved);
    return nbInstructionsSaved;
}

bool Pami::motorsAreRunning(){
    return (this->moteur_gauche.distanceToGo() != 0 || this->moteur_droit.distanceToGo() != 0);
}

/**********
 * WiFi
 * ********/
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
    Serial.print("theta = "); Serial.print(fmod(M_PI + this->theta, 2*M_PI) - M_PI); Serial.println(" rad");
}

void Pami::printLocalTime(struct tm* timeinfo){
    Serial.print("Time: ");
    Serial.print(timeinfo->tm_hour);
    Serial.print(":");
    Serial.print(timeinfo->tm_min);
    Serial.print(":");
    Serial.println(timeinfo->tm_sec);
} 


void Pami::printTarget(){
    Serial.print("center = ("); Serial.print(this->zone.x_center); Serial.print(","); Serial.print(this->zone.y_center); Serial.print(") ");
    Serial.print("x1 = ("); Serial.print(this->zone.x_1); Serial.print(","); Serial.print(this->zone.y_1); Serial.print(") ");
    Serial.print("x2 = ("); Serial.print(this->zone.x_2); Serial.print(","); Serial.print(this->zone.y_2); Serial.println(")");
}

//Utilities

//Normalize angle between [-pi,pi]
double normalizeAngle(double angle){
    angle = fmod(angle, 2*M_PI);
    if (angle > M_PI) angle -= 2*M_PI;
    else if (angle < -M_PI) angle += 2*M_PI;
    return angle;
}