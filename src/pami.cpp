#include "pami.h"

Pami::Pami() : MoteurDroit(RIGHT_DIR_PIN, RIGHT_STEP_PIN, STEPS_PER_REV), 
               MoteurGauche(LEFT_DIR_PIN, LEFT_STEP_PIN, STEPS_PER_REV),
               sensor(&Wire, LPN_PIN, I2C_RST_PIN) {}

//Communication
IPAddress Pami::connectToWiFi(const char* ssid, const char* password){
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
    return WiFi.localIP();
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
