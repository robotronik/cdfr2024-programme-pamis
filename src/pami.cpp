#include "pami.h"

Pami::Pami(Stepper MoteurGauche, VL53L7CX ToF){

    this.MoteurDroit = new Stepper(RIGHT_DIR_PIN, RIGHT_STEP_PIN, STEPS_PER_REV);
    this->MoteurGauche = new Stepper(LEFT_DIR_PIN, LEFT_STEP_PIN, STEPS_PER_REV);
    //this.ToF = new VL53L7CX(&DEV_I2C, LPN_PIN, I2C_RST_PIN);
}

Pami::moveDist(int dir, int speed, int distance_mm){
    this->MoteurDroit.spinDistance(dir,distance_mm,speed);
    this->MoteurGauche.spinDistance(dir,distance_mm,speed);
}

Pami::steerDeg(int dir, int speed, int distance_mm){
    //TO COMPLETE
    /*
    this->MoteurDroit.spinDistance();
    this->MoteurGauche.spinDistance();
    */
}

Pami::goToPos(int x, int y){
    //TODO
}

Pami::setPos(int x, int y){
    this.x = x;
    this.y = y;
}
