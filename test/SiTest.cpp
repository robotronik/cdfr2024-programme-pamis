// Do not remove the include below
#include "SiTest.h"
#include "si4432.h"

#include "define.h"

Si4432 radio(GPIO_NUM_2, 0);

//Data to send
byte dummy[] = {'H', 'E', 'L', 'L', 'O', ' ', 'W', 'O', 'R', 'L', 'D'};
byte TxLen = sizeof(dummy) / sizeof(byte);


//Received data
byte *payload; 
byte *RxLen; 
byte intStat;

unsigned long pTime;
//The setup function is called once at startup of the sketch
void setup() {

	Serial.begin(115200);
	delay(300);
	radio.init();
	radio.setBaudRate(70);
	radio.setFrequency(433);

#ifdef RX
	radio.startListening();
	do {
		payload = (byte*)malloc(MAX_INPUT);
		RxLen = (byte*)malloc(1);
		} 
	while (payload == NULL || RxLen == NULL);

#endif
	pTime = millis();
// Add your initialization code here
}

// The loop function is called in an endless loop
void loop() {
//Add your repeated code here

#ifdef TX
	byte len = 0;
	byte answer[64] = { 0 };

	bool pkg = radio.sendPacket(TxLen, dummy);
  Serial.println("Packet sent");
  delay(1000);
#endif

#ifdef RX
	if (radio.waitForPacket(1000)) {  
		radio.getPacketReceived(RxLen, payload);
    Serial.print(" - PACKET CAME - ");
    Serial.print(*RxLen, DEC);
    Serial.print(" - ");
    Serial.println(millis() - pTime, DEC);

    pTime = millis();
    for (byte i = 0; i < *RxLen; ++i) {
      Serial.print((int) payload[i], HEX);
      Serial.print(" ");
    }
    Serial.println(" ");
  }
  else{
		//Serial.println("No packet this cycle");
  }
#endif
}