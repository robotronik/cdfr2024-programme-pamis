#include "stepper.h"

void Stepper::setDirection(RotDir dir){
    digitalWrite(this->dir, dir);
}

void Stepper::spinSteps(RotDir dir, int steps, int speed){
    digitalWrite(this->dir, dir);

    for (int i = 0; i < steps; i++)
    {
        digitalWrite(this->step, HIGH);
        delayMicroseconds(1000);
        digitalWrite(this->step, LOW);
        delayMicroseconds(1000);
    }
}
void Stepper::spinDistance(RotDir dir, int speed, int distance_mm){
    int steps = (distance_mm * 360) / (WHEEL_DIAMETER * 3.14159);
    spinSteps(dir, steps, speed);
}

