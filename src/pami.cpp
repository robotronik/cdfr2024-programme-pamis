#include "pami.h"

Pami::Pami(){
    this->MoteurDroit = Stepper(RIGHT_DIR_PIN, RIGHT_STEP_PIN, STEPS_PER_REV);
    this->MoteurGauche = Stepper(LEFT_DIR_PIN, LEFT_STEP_PIN, STEPS_PER_REV);
    this->ToF = VL53L7CX(&Wire, LPN_PIN, I2C_RST_PIN);
}

void Pami::moveDist(int dir, int speed, int distance_mm){
    this->MoteurDroit.spinDistance(dir,distance_mm,speed);
    this->MoteurGauche.spinDistance(dir,distance_mm,speed);
}

void Pami::steerDeg(int dir, int speed, int distance_mm){
    //TO COMPLETE
    /*
    this->MoteurDroit.spinDistance();
    this->MoteurGauche.spinDistance();
    */
}

void Pami::goToPos(int x, int y){
    //TODO
}

void Pami::setPos(int x, int y){
    this.x = x;
    this.y = y;
}
