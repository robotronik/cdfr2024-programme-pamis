#include <WiFi.h>

const char* ssid = "RaspberryRobotronik";
const char* password = "robotronik";

void setup(){
    Serial.begin(115200);
    delay(1000);

    WiFi.mode(WIFI_STA); //Optional
    WiFi.begin(ssid, password);
    Serial.println("\nConnecting");

    while(WiFi.status() != WL_CONNECTED){
        Serial.print(".");
        delay(100);
    }

    Serial.print("\nConnected with IP adress: ");
    Serial.println(WiFi.localIP());
}

void loop(){}