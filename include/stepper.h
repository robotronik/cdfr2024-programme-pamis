#ifndef STEPPER_H
#define STEPPER_H

#define WHEEL_DIAMETER 65 // mm
#include <Arduino.h>

class Stepper{
    public:
        Stepper(int dir, int step, int steps_per_rev);

        void spinSteps(int dir, int steps, int speed);
        void spinDistance(int dir, int distance_mm, int speed);
        void calibrate();

    private:
        int dir;
        int step;
        int steps_per_rev;
};

#endif