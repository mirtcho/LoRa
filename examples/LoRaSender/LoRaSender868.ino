#include <SPI.h>
#include <LoRa.h>

int counter = 0;

void setup() {
  Serial2.begin(9600);
  while (!Serial2);

  Serial2.println("LoRa Sender");

  if (!LoRa.begin(868.10E6)) {
    Serial2.println("Starting LoRa failed!");
    while (1);
  }
  //LoRa.enableCrc();
  LoRa.setSignalBandwidth(125e3);//125e3);15.6E3
  LoRa.setSpreadingFactor(11);
  LoRa.setTxPower(7,PA_OUTPUT_PA_BOOST_PIN); //PA_OUTPUT_PA_BOOST_PIN
}


void loop() {
  Serial2.print("Sending packet: ");
  Serial2.println(counter);

  // send packet
  LoRa.beginPacket();
  LoRa.print("5keer test test test test test #");
  LoRa.println(counter,HEX);
  //LoRa.dumpRegisters(Serial2);
  if (LoRa.endPacket()!=0)
  {
    //correct transmitred packed
    counter++;  
  }
  Serial2.println("lora end packet");
  LoRa.setTxPower(7,PA_OUTPUT_PA_BOOST_PIN);//mma works with 7
  delay (15000);
  
}
