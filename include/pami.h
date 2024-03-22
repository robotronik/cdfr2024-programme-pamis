#ifndef PAMI_H
#define PAMI

#include "stepper.h"
#include "VL53L7CX_class.h"

//Pinout
#define LEFT_DIR_PIN
#define LEFT_STEP_PIN

#define RIGHT_DIR_PIN
#define RIGHT_STEP_PIN

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
        moveDist(int dir, int speed, int distance_mm);
        steerDeg(int dir, int speed, int deg);
        setPos(int x, int y);
        goToPos(int x, int y);
        detectObstacle();

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