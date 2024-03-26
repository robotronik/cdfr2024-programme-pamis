 /**
 ******************************************************************************
 * @file    VL53L7CX_HelloWorld.ino
 * @author  STMicroelectronics
 * @version V1.0.0
 * @date    16 January 2023
 * @brief   Arduino test application for STMicroelectronics VL53L7CX
 *          proximity sensor satellite based on FlightSense.
 *          This application makes use of C++ classes obtained from the C
 *          components' drivers.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2021 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/*
 * To use these examples you need to connect the VL53L7CX satellite sensor directly to the Nucleo board with wires as explained below:
 * pin 1 (GND) of the VL53L7CX satellite connected to GND of the Nucleo board
 * pin 2 (IOVDD) of the VL53L7CX satellite connected to 3V3 pin of the Nucleo board
 * pin 3 (AVDD) of the VL53L7CX satellite connected to 5V pin of the Nucleo board
 * pin 4 (PWREN) of the VL53L7CX satellite connected to pin A5 of the Nucleo board
 * pin 5 (LPn) of the VL53L7CX satellite connected to pin A3 of the Nucleo board
 * pin 6 (SCL) of the VL53L7CX satellite connected to pin D15 (SCL) of the Nucleo board
 * pin 7 (SDA) of the VL53L7CX satellite connected to pin D14 (SDA) of the Nucleo board
 * pin 8 (I2C_RST) of the VL53L7CX satellite connected to pin A1 of the Nucleo board
 * pin 9 (INT) of the VL53L7CX satellite connected to pin A2 of the Nucleo board
 */

/* Includes ------------------------------------------------------------------*/
#include <Arduino.h>
#include <Wire.h>
#include <vl53l7cx_class.h>

#define DEV_I2C Wire
#define SerialPort Serial

//Pinout 
#define LPN_PIN GPIO_NUM_33
#define I2C_RST_PIN GPIO_NUM_35
#define PWREN_PIN GPIO_NUM_32
#define LED GPIO_NUM_2
#define DIR_PIN GPIO_NUM_26
#define STEP_PIN GPIO_NUM_25

#define FREQUENCY_HZ 60
#define THRESHOLD 10

void print_result(VL53L7CX_ResultsData *Result);
void clear_screen(void);
void handle_cmd(uint8_t cmd);
void display_commands_banner(void);

// Components.
VL53L7CX sensor_vl53l7cx_top(&DEV_I2C, LPN_PIN, I2C_RST_PIN);

bool EnableAmbient = false;
bool EnableSignal = false;
uint8_t res = VL53L7CX_RESOLUTION_4X4;
char report[256];

uint32_t loopTime, calcTime;

/* Setup ---------------------------------------------------------------------*/
void setup()
{

  // Enable PWREN pin if present
  if (PWREN_PIN >= 0) {
    pinMode(PWREN_PIN, OUTPUT);
    digitalWrite(PWREN_PIN, HIGH);
    delay(10);
  }

  pinMode(LED, OUTPUT);
  // Initialize serial for output.
  SerialPort.begin(250000);

  // Initialize I2C bus.
  DEV_I2C.begin();

  // Configure VL53L7CX component.
  sensor_vl53l7cx_top.begin();

  sensor_vl53l7cx_top.init_sensor();
  sensor_vl53l7cx_top.vl53l7cx_set_ranging_frequency_hz(FREQUENCY_HZ);

  // Start Measurements
  sensor_vl53l7cx_top.vl53l7cx_start_ranging();

  //Initialisation du moteur pas à pas
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT); digitalWrite(DIR_PIN, HIGH);
}

void loop()
{
  VL53L7CX_ResultsData * Results = (VL53L7CX_ResultsData *)malloc(sizeof(VL53L7CX_ResultsData));
  uint8_t NewDataReady = 0;
  uint8_t status;

  int8_t i, j, k, l;
  uint8_t zones_per_line;

  uint16_t minDistance = INT16_MAX;

  loopTime = millis();

  //Attente de données du capteur
  do {
    status = sensor_vl53l7cx_top.vl53l7cx_check_data_ready(&NewDataReady);
  } while (!NewDataReady);

  if ((!status) && (NewDataReady != 0)) {
    //Chargement des données dans Results
    status = sensor_vl53l7cx_top.vl53l7cx_get_ranging_data(Results);
    
    zones_per_line = (res == VL53L7CX_RESOLUTION_8X8) ? 8 : 4;

    //Calul distance minimum
    calcTime = millis();
    for (j = 0; j < res; j += zones_per_line){
      for (l = 0; l < VL53L7CX_NB_TARGET_PER_ZONE; l++){
        for (k = (zones_per_line - 1); k >= 0; k--){

          //On prend en compte uniquement les zones où le mesure est valide (status = 5 ou 9)
          uint8_t zoneStatus = Results->target_status[(VL53L7CX_NB_TARGET_PER_ZONE * (j+k)) + l];
          if (zoneStatus == 5 || zoneStatus == 9){
            uint16_t distance = Results->distance_mm[(VL53L7CX_NB_TARGET_PER_ZONE * (j+k)) + l];
            if (distance < minDistance){
              minDistance = distance;
            }
          }
        }
      }
    } 
  }

  if (minDistance < THRESHOLD){
    digitalWrite(LED, HIGH);
    digitalWrite(STEP_PIN, LOW);
  } else {

    //Déplacement du moteur pas à pas
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(1000);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(1000);

    digitalWrite(LED, LOW);

  }

  free(Results);
  /*
  //Affichage du temps d'éxécution de la boucle
  loopTime = millis() - loopTime;
  sprintf(report,"Loop time : %d ms", loopTime);
  Serial.println(report);

  //Affichage du temps de calcul
  calcTime = millis() - calcTime;
  sprintf(report,"Calc time : %d ms", calcTime);
  Serial.println(report);


  //Affichage de la distance minimum
  sprintf(report, "Distance minimum : %d mm", minDistance);
  Serial.println(report);

  Serial.println("------------------------------------------------------");
  */
}
