#ifndef PAMI_H
#define PAMI_H

#include "stepper.h"
#include "datmo.h"
#include "vl53l7cx_class.h"
#include <WiFi.h>
#include "SPI.h"
#include "si4432.h"
#include "math.h"

//Pinout 
#define LPN_PIN GPIO_NUM_26
#define I2C_RST_PIN GPIO_NUM_25
#define PWREN_PIN GPIO_NUM_27

#define LEFT_DIR_PIN GPIO_NUM_5 
#define LEFT_STEP_PIN  GPIO_NUM_4

#define RIGHT_DIR_PIN GPIO_NUM_14 
#define RIGHT_STEP_PIN GPIO_NUM_13

#define DS1_PIN GPIO_NUM_15
#define DS2_PIN GPIO_NUM_35
#define DS3_PIN GPIO_NUM_34

//Caractéristiques géométriques du PAMI
#define DISTANCE_CENTRE_POINT_CONTACT_ROUE 0.1 //Distance entre le centre du PAMI et le point de contact de la roue en projection sur le sol

#define FREQUENCY_HZ 60
#define THRESHOLD 10

//Définition des directions
enum Direction {BACKWARDS, FORWARDS, LEFT, RIGHT, STOP};

class Pami{
    public:
        //Constructeur et initialisation
        Pami();
        void init();
        void shutdown();

        //Communication
        /*
        WiFiClient initWiFi(const char* ssid, const char* password);
        void readDataWiFi(WiFiClient client);
        void sendDataWiFi(WiFiClient client, char* data);
        */
        void sendData(char * data, uint8_t length);
        char * readData(uint8_t length);

        //Captuer ToF
        void getSensorData(VL53L7CX_ResultsData *Results);
        
        //Déplacement
        void moveDist(int dir, int speed, int distance_mm);
        void steerRad(int dir, int speed, float orientation_rad);
        void setPos(int x, int y);
        void goToPos(int x, int y);
        void detectObstacle();

        int id; //N° du PAMI, 1-5
        Stepper moteur_gauche;
        Stepper moteur_droit;   
        VL53L7CX sensor;
        Si4432 radio;

        Direction direction = FORWARDS;
        int speed = 100;
        int distance;

        float x;
        float y;
        float orientation; //Rad
        float x_zone;
        float y_zone;
};

#endif