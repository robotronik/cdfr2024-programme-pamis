#ifndef PAMI_H
#define PAMI_H

#include "stepper.h"
#include "datmo.h"
#include "vl53l7cx_class.h"
#include <WiFi.h>
#include "SPI.h"
#include "math.h"
#include "zone.h"

//Pinout 
#define LPN_PIN GPIO_NUM_26
#define I2C_RST_PIN GPIO_NUM_25
#define PWREN_PIN GPIO_NUM_27

#define LEFT_DIR_PIN GPIO_NUM_5 
#define LEFT_STEP_PIN  GPIO_NUM_4

#define RIGHT_DIR_PIN GPIO_NUM_14 
#define RIGHT_STEP_PIN GPIO_NUM_13

#define DS1_PIN GPIO_NUM_15
#define DS2_PIN GPIO_NUM_35
#define DS3_PIN GPIO_NUM_34

#define NB_MAX_INSTRUCTIONS 10

//Caractéristiques géométriques du PAMI
#define DISTANCE_CENTRE_POINT_CONTACT_ROUE 46 //Distance entre le centre du PAMI et le point de contact de la roue en projection sur le sol
#define DIAMETRE_ROUE 78//mm
#define SENSOR_FREQUENCY_HZ 10
#define THRESHOLD 30//mm


//Caractéristiques connexion WiFi
#define LOCALPORT 9999
#define SERVERPORT 8888
#define GMTOFFSET 3600
#define DAYLOFFSET 3600

//Types énumérés
enum Direction {BACKWARDS, FORWARDS, LEFT, RIGHT, STOP};
enum State {IDLE, GO_FOR_TARGET, AVOID_OBSTACLE, MOVING};
enum Couleur {BLEU, JAUNE}; 

//Commande moteurs
typedef struct _instruction{
    Direction dir;
    int nbSteps;
} Instruction;

class Pami{
    public:
        //Constructeur et initialisation
        Pami();
        void init();
        void shutdown();

        //Communication
        /*
        WiFiClient initWiFi(const char* ssid, const char* password);
        void readDataWiFi(WiFiClient client);
        void sendDataWiFi(WiFiClient client, char* data);
        */
        void sendData(char * data, uint8_t length);
        char * readData(uint8_t length);

        //Capteur ToF
        void getSensorData(VL53L7CX_ResultsData *Results);
        
        //Déplacement
        void moveDist(Direction dir, int distance_mm);
        void steerRad(Direction dir, float orientation_rad);
        void setPos(int x, int y);
        void goToPos(int x, int y);
        bool inZone();
        void addInstruction(Direction dir, int nbSteps);
        void clearInstructions();
        void executeNextInstruction();
        void connectToWiFi(const char* ssid,const char* password,const char* serverip,WiFiUDP udp);
        void printLocalTime(struct tm* timeinfo);


        //Utilities
        void printPos();    

        //N° du PAMI, 1-5
        int id; 
        Couleur couleur;

        Stepper moteur_gauche;
        Stepper moteur_droit;   
        VL53L7CX sensor;

        Direction direction = FORWARDS;
        int speed = 100;
        int nbStepsToDo = 0;

        float x;
        float y;
        float theta; //Rad
        
        Zone zone;

        uint16_t closestObstacle = UINT16_MAX;
        State state;
        Instruction listInstruction[NB_MAX_INSTRUCTIONS];
        int nbInstructions = 0;
};

#endif
