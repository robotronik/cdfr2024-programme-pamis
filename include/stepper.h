#ifndef STEPPER_H
#define STEPPER_H

//Caractéristiques mécaniques de l'ensemble roues + moteurs
#define WHEEL_DIAMETER 75 // mm
#define STEPS_PER_REV 200

#include <Arduino.h>

class Stepper{
    public:
        Stepper(int dir_pin, int step_pin, int steps_per_rev){
            this->dir = dir_pin;
            this->step = step_pin;
            this->steps_per_rev = steps_per_rev;

            pinMode(this->step, OUTPUT);
            pinMode(this->dir, OUTPUT);
        }

        void spinSteps(int dir, int steps, int speed);
        void spinDistance(int dir, int speed, int distance_mm);

    private :
        int dir;
        int step;
        int steps_per_rev;
};

#endif