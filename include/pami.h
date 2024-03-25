#ifndef PAMI_H
#define PAMI_H

#include "stepper.h"
#include "VL53L7CX_class.h"

//Pinout
#define LEFT_DIR_PIN -1
#define LEFT_STEP_PIN -1

#define RIGHT_DIR_PIN -1
#define RIGHT_STEP_PIN -1

#define LPN_PIN GPIO_NUM_33
#define I2C_RST_PIN GPIO_NUM_35
#define PWREN_PIN GPIO_NUM_32

//Caractéristiques géométriques du PAMI
#define DISTANCE_CENTRE_POINT_CONTACT_ROUE


//Macros déplacement PAMI
#define FORWARDS 8
#define BACKWARDS 2
#define LEFT 4
#define RIGHT 6


class Pami{
    public:
        Pami();
        void moveDist(int dir, int speed, int distance_mm);
        void steerDeg(int dir, int speed, int deg);
        void setPos(int x, int y);
        void goToPos(int x, int y);
        void detectObstacle();

    private:
        int id; //N° du PAMI, 1-6
        Stepper MoteurDroit;
        Stepper MoteurGauche;
        VL53L7CX ToF;

        int speed;
        int x;
        int y;
};

#endif