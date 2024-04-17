#include "pami.h"

Pami::Pami() : moteur_droit(DIR_PIN_DROIT, STEP_PIN, STEPS_PER_REV),
               moteur_gauche(DIR_PIN_GAUCHE, STEP_PIN, STEPS_PER_REV),
               sensor(&Wire, LPN_PIN, I2C_RST_PIN),
               radio(0, 0) {

}
void Pami::init(){
    // Initialize I2C bus.
    Wire.begin();

    //Initialize RF module
    radio.init();
    
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
        
    // Enable PWREN pin if present
    if (PWREN_PIN >= 0) {
        pinMode(PWREN_PIN, OUTPUT);
        digitalWrite(PWREN_PIN, HIGH);
        delay(10);
    }

    // Start Measurements
    this->sensor.vl53l7cx_start_ranging();

    pinMode(LED_BUILTIN, OUTPUT);

    //Read HW ID
    this->id = digitalRead(DS1_PIN)
            + digitalRead(DS2_PIN)*2
            + digitalRead(DS3_PIN)*4;

    //Define zone coordinates
    switch(this->id){
        case 1:
        this->x_zone = 0;
        this->y_zone = 0; 
        case 2:
        this->x_zone = 0;
        this->y_zone = 0; 
        case 3:
        this->x_zone = 0;
        this->y_zone = 0; 
        case 4:
        this->x_zone = 0;
        this->y_zone = 0; 
        case 5:
        this->x_zone = 0;
        this->y_zone = 0; 
    }
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

//Send data via SPI
void Pami::sendData(char * data, uint8_t length){
   this->radio.sendPacket(length, (const byte *) data);
}

//Read data via SPI
char * Pami::readData(uint8_t length){
    char* data; 
    this->radio.getPacketReceived(&length, (byte *) data);
    return data;
}

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

void Pami::goToPos(int x, int y){
    int Dx = x - this->x;
    int Dy = y - this->y;

    float orientation_rad = atan2(Dy,Dx);
    if (orientation_rad != 0){
        int dir = (Dx*cos(orientation_rad) + Dy*sin(orientation_rad));
        if (dir > 0) direction = RIGHT;
        else direction = LEFT;
        distance = orientation_rad * DISTANCE_CENTRE_POINT_CONTACT_ROUE;
    }
    direction = FORWARDS;
    distance = 10;
    //distance = sqrt(Dx*Dx + Dy*Dy);
}

void Pami::setPos(int x, int y){
    this->x = x;
    this->y = y;
}
