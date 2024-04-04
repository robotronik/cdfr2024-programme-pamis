#include "pami.h"

Pami::Pami() : MoteurDroit(RIGHT_DIR_PIN, RIGHT_STEP_PIN, STEPS_PER_REV), 
               MoteurGauche(LEFT_DIR_PIN, LEFT_STEP_PIN, STEPS_PER_REV),
               sensor(&Wire, LPN_PIN, I2C_RST_PIN) {

}
void Pami::init(){
    // Initialize I2C bus.
    Wire.begin();
    
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
    digitalWrite(LED_BUILTIN, HIGH);
        
    // Enable PWREN pin if present
    if (PWREN_PIN >= 0) {
    pinMode(PWREN_PIN, OUTPUT);
    digitalWrite(PWREN_PIN, HIGH);
    delay(10);
  }
  
}
//Communication
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
/*
void Pami::readData(WiFiClient client){
    WiFiClient client;
    if (client.connect(this->ip, this->port)){
        while(client.available()){
            char c = client.read();
            Serial.print(c);
        }
        client.stop();
    }
}

void Pami::sendData(WiFiClient client, char* data){
    if (client.connect(ip, port)){
        client.print(data);
        client.stop();
    }
}*/
//Capteur ToF
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

//Déplacement
void Pami::moveDist(int dir, int speed, int distance_mm){
    this->MoteurDroit.spinDistance(dir,distance_mm,speed);
    this->MoteurGauche.spinDistance(dir,distance_mm,speed);
}

void Pami::steerRad(int dir, int speed, float orientation_rad){
    float distance_mm = orientation_rad * DISTANCE_CENTRE_POINT_CONTACT_ROUE;
    this->MoteurDroit.spinDistance(dir,distance_mm,speed);
    this->MoteurGauche.spinDistance(abs(dir-1),distance_mm,speed);
}

void Pami::goToPos(int x, int y){
    int Dx = x - this->x;
    int Dy = y - this->y;

    float orientation_rad = atan2(Dy,Dx);
    if (orientation_rad != 0){
        int dir = (Dx*cos(orientation_rad) + Dy*sin(orientation_rad));
        if (dir > 0) dir = RIGHT;
        else dir = LEFT;
        this->steerRad(dir, 100, orientation_rad);
    }
    
    this->moveDist(FORWARDS, 100, sqrt(Dx*Dx + Dy*Dy));
}

void Pami::setPos(int x, int y){
    this->x = x;
    this->y = y;
}
