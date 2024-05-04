#include <AccelStepper.h>

// Define stepper motor connections and steps per revolution
#define motorInterfaceType 1 // 1: A4988 or DRV8825, 2: L298N, 3: L293D
#define LEFT_DIR_PIN GPIO_NUM_5 
#define LEFT_STEP_PIN  GPIO_NUM_4

#define RIGHT_DIR_PIN GPIO_NUM_14 
#define RIGHT_STEP_PIN GPIO_NUM_13

#define motorPin1 2
#define motorPin2 3
#define motorPin3 4
#define motorPin4 5

#define STEPS_PER_REV 200

AccelStepper stepperLeft(motorInterfaceType, LEFT_STEP_PIN, LEFT_DIR_PIN);
AccelStepper stepperRight(motorInterfaceType, RIGHT_STEP_PIN, RIGHT_DIR_PIN);

void setup() {
  // Set maximum speed and acceleration
  stepperLeft.setMaxSpeed(500); // Adjust maximum speed as needed
  stepperLeft.setAcceleration(500); // Adjust acceleration as needed
  stepperRight.setMaxSpeed(500); // Adjust maximum speed as needed
  stepperRight.setAcceleration(500); // Adjust acceleration as needed
  
  // Set initial speed and target position
  stepperLeft.setSpeed(0);
  stepperLeft.moveTo(2000); // Target position
  stepperRight.setSpeed(0);
  stepperRight.moveTo(2000); // Target position
  
  Serial.begin(9600);
}

void loop() {
  stepperLeft.run(); // Must be called to allow the stepper to step
  stepperRight.run(); // Must be called to allow the stepper to step
  
  if (stepperLeft.distanceToGo() == 0 && stepperRight.distanceToGo() == 0){
    // Stepper has reached the target position
    delay(1000); // Wait for 1 second before moving again
    
    // Change target position
    stepperLeft.moveTo(-stepperLeft.currentPosition());
    stepperRight.moveTo(-stepperRight.currentPosition());
  }
}
