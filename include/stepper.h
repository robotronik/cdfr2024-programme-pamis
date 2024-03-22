#ifndef STEPPER_H
#define STEPPER_H

//Caractéristiques mécaniques de l'ensemble roues + moteurs
#define WHEEL_DIAMETER 65 // mm
#define STEPS_PER_REV 0

#include <Arduino.h>

class Stepper{
    public:
        Stepper(int dir, int step, int steps_per_rev);

        void spinSteps(int dir, int steps, int speed);
        void spinDistance(int dir, int speed, int distance_mm);
        void calibrate();

    private:
        int dir;
        int step;
        int steps_per_rev;
};

#endif