#ifndef PAMI_H
#define PAMI_H

#include "stepper.h"
#include "vl53l7cx_class.h"
#include <WiFi.h>

//Pinout
#define LEFT_DIR_PIN GPIO_NUM_5
#define LEFT_STEP_PIN GPIO_NUM_4

#define RIGHT_DIR_PIN GPIO_NUM_14
#define RIGHT_STEP_PIN GPIO_NUM_13

#define LPN_PIN GPIO_NUM_26
#define I2C_RST_PIN GPIO_NUM_25
#define PWREN_PIN GPIO_NUM_27



//Caractéristiques géométriques du PAMI
#define DISTANCE_CENTRE_POINT_CONTACT_ROUE 0.1 //Distance entre le centre du PAMI et le point de contact de la roue en projection sur le sol


//Macros déplacement PAMI
#define FORWARDS 1
#define BACKWARDS 0
#define LEFT 0
#define RIGHT 1


class Pami{
    public:
        Pami();

        //Communication
        IPAddress connectToWiFi(const char* ssid, const char* password);
        void readData();
        void sendData(char* data);

        //Déplacement
        void moveDist(int dir, int speed, int distance_mm);
        void steerRad(int dir, int speed, float orientation_rad);
        void setPos(int x, int y);
        void goToPos(int x, int y);
        void detectObstacle();

    private:
        int id; //N° du PAMI, 1-6
        Stepper MoteurDroit;
        Stepper MoteurGauche;
        VL53L7CX sensor;

        int speed;
        int x;
        int y;
};

#endif