#ifndef PAMI_H
#define PAMI_H
#include <stdio.h>
#include <stdlib.h>
#include "stepper.h"
#include "datmo.h"
#include "vl53l7cx_class.h"
#include <WiFi.h>
#include "SPI.h"

#include <cmath>
#include "zone.h"
#include "AccelStepper.h"

//Pinout 

#define LPN_PIN GPIO_NUM_26
#define I2C_RST_PIN GPIO_NUM_25
#define PWREN_PIN GPIO_NUM_27

#define LEFT_DIR_PIN GPIO_NUM_26
#define LEFT_STEP_PIN  GPIO_NUM_25

#define RIGHT_DIR_PIN GPIO_NUM_33 
#define RIGHT_STEP_PIN GPIO_NUM_32

#define DS1_PIN GPIO_NUM_15
#define DS2_PIN GPIO_NUM_35
#define DS3_PIN GPIO_NUM_34

#define NB_MAX_INSTRUCTIONS 10

//Caractéristiques du capteur ToF   
#define SENSOR_FREQUENCY_HZ 60
#define SENSOR_RES  VL53L7CX_RESOLUTION_4X4; // 4x4 resolution
#define SENSOR_THRESHOLD  120 //M_PI * MAX_SPEED*MAX_SPEED/(STEPS_PER_REV*ACCELERATION) //mm

//Caractéristiques géométriques du PAMI
#define DISTANCE_ROUES 60.0//Distance entres les points de contact des roues (mm)
#define DIAMETRE_ROUE 79.5773//mm

//Caractéristiques moteurs
#define STEPS_PER_REV 200
#define MAX_SPEED 500 //steps/s
#define ACCELERATION 500 //steps/s^2
#define MIN_STEP_TIME_INTERVAL 1000/MAX_SPEED //ms

//Constantes de différence mouvement effectué/mouvement indiqué
#define DELTA_FORWARD 1
#define DELTA_ROTATE 1

//Caractéristiques connexion WiFi
#define SSID "SuperRoutotronik"
#define PASSWORD "J2MRNCKAWP"
#define SERVERIP "raspitronik.local"
#define LOCALPORT 9999
#define SERVERPORT 8888
#define GMTOFFSET 3600
#define DAYLOFFSET 3600

//Types énumérés
enum Direction {BACKWARDS, FORWARDS, LEFT, RIGHT, STOP};
enum State {START,WAIT_INFO,WAIT_IDLE,IDLE, STOPPED, BLOCKED, MOVING, END};
enum Couleur {BLEU, JAUNE}; 

//Commande moteurs
typedef struct _instruction{
    Direction dir;
    long nbSteps;
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
        void connectToWiFi(const char* ssid,const char* password,const char* serverip,WiFiUDP udp);

        //Capteur ToF
        void getSensorData(VL53L7CX_ResultsData *Results);
        
        //Déplacement
        void moveDist(Direction dir, double distance_mm);
        void steerRad(Direction dir, double orientation_rad);
        void setPos(int x, int y);
        void goToPos(double x, double y);
        bool inZone();
        bool motorsAreRunning();
        void addInstruction(Direction dir, long nbSteps);
        void clearInstructions();
        void sendNextInstruction();
        void printTarget();

        //WiFI
        void connectToWiFi();
        void UDPBeginAndSynchro(WiFiUDP *udp);
        void SendUDPPacket(WiFiUDP *udp);
        int ReadPacket(WiFiUDP *udp, char* buf);
        void printLocalTime(struct tm* timeinfo);


        //Utilities
        void printPos();    

        //N° du PAMI, 1-5
        int id; 
        Couleur couleur;

        AccelStepper moteur_gauche = (1, LEFT_STEP_PIN, LEFT_DIR_PIN);
        AccelStepper moteur_droit = (1, RIGHT_STEP_PIN, RIGHT_DIR_PIN);   


        VL53L7CX sensor;

        Direction direction = FORWARDS;
        int speed = 100;
        int nbStepsToDo = 0;

        //Current position
        double x;
        double y;
        double theta; //Rad

        //Last saved position (checkpoint)
        double x_last;
        double y_last;
        double theta_last;
        double Dtheta; 

        
        Zone zone;

        bool obstacleDetected = false;

        State state;
        State nextState;
        Instruction listInstruction[NB_MAX_INSTRUCTIONS];
        Instruction currentInstruction;
        int nbInstructions = 0;
};

double normalizeAngle(double angle);

#endif
