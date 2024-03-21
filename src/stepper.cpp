#include "stepper.h"

Stepper::Stepper(int dir_pin, int step_pin, int steps_per_rev)
{
    this->dir = dir_pin;
    this->step = step_pin;
    this->steps_per_rev = steps_per_rev;

    pinMode(this->step, OUTPUT);
    pinMode(this->dir, OUTPUT);
}

void Stepper::spinSteps(int dir, int steps, int speed){
    digitalWrite(this->dir, dir);

    for (int i = 0; i < steps; i++)
    {
        digitalWrite(this->step, HIGH);
        delayMicroseconds(speed);
        digitalWrite(this->step, LOW);
        delayMicroseconds(speed);
    }
}
void Stepper::spinDistance(int dir, int distance_mm, int speed){
    int steps = (distance_mm * 360) / (WHEEL_DIAMETER * 3.14159);
    spinSteps(dir, steps, speed);
}

