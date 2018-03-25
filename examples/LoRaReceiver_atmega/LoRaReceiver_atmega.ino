#include <SPI.h> 
#include <LoRa.h>

#define TX_EN   2
#define RX_EN   3
#define NRESET  6
#define NSS     7
#define DIO0    5

#define BAND    868.10E6
#define PABOOST true

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("LoRa Receiver");

  pinMode(TX_EN,OUTPUT);
  digitalWrite(TX_EN, LOW);
  pinMode(RX_EN,OUTPUT);
  digitalWrite(RX_EN, HIGH);
  pinMode(NRESET,OUTPUT);
  //??
  pinMode(NSS,OUTPUT);
  digitalWrite(NSS, HIGH);
  pinMode(DIO0,INPUT);
  
  LoRa.setPins (NSS,NRESET,DIO0);

  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  LoRa.setSignalBandwidth(62500);
  LoRa.setSpreadingFactor(12);
}

void loop() {
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // received a packet
    Serial.print("Received packet '");

    // read packet
    while (LoRa.available()) {
      Serial.print((char)LoRa.read());
    }

    // print RSSI of packet
    Serial.print("' with RSSI ");
    Serial.println(LoRa.packetRssi());
  }
}
