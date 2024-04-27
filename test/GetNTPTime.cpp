#include <WiFi.h>
#include <WiFiUdp.h>
#include "time.h"

#define LED_PIN 2

WiFiUDP udp;
char packetBuffer[255];
unsigned int localPort = 9999;
char *serverip = "192.168.62.92"; // IP Raspberry Pi
unsigned int serverport = 8888;
const char *ssid = "***";
const char *password = "****";
const long  gmtOffset_sec = 3600;
const int   daylightOffset_sec = 3600;
struct tm timeinfo;

void printLocalTime()
{
  
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  Serial.println(mktime(&timeinfo));
}

void setup() {
 	Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
 	// Connect to Wifi network.
 	WiFi.begin(ssid, password);
 	while (WiFi.status() != WL_CONNECTED) {
 			delay(500); Serial.print(F("."));
 	}
 	udp.begin(localPort);
 	Serial.printf("UDP Client : %s:%i \n", WiFi.localIP().toString().c_str(), localPort);
  //init and get the time
  configTime(gmtOffset_sec, daylightOffset_sec, serverip);
}
void loop() {
  
 	int packetSize = udp.parsePacket();
 	if (packetSize) {
 			Serial.print(" Received packet from : "); Serial.println(udp.remoteIP());
 			int len = udp.read(packetBuffer, 255);
      time_t start_time=atoi(packetBuffer);
 			Serial.printf("Data : %s\n", packetBuffer);
 			Serial.println();
      struct tm *start_time_struct=localtime(&start_time);
      Serial.println(start_time_struct, "%A, %B %d %Y %H:%M:%S");
      Serial.println();
      if(start_time<=mktime(&timeinfo)){
        digitalWrite(LED_PIN, HIGH);
      }
      
 	}
 	delay(100);
 	Serial.print("[Client Connected] "); Serial.println(WiFi.localIP());
  printLocalTime();
 	udp.beginPacket(serverip, serverport);
 	char buf[30];
 	unsigned long testID = millis();
 	sprintf(buf, "ESP32 send millis: %lu", testID);
 	udp.printf(buf);
 	udp.endPacket();
}
