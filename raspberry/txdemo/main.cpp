/*******************************************************************************
 *
 * Copyright (c) 2015 Thomas Telkamp
 *
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *
 *******************************************************************************/

#include <string>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <string.h>
#include <iostream>
#include <cstdlib>
#include <sys/time.h>
#include <cstring>

#include <sys/ioctl.h>
#include <net/if.h>

using namespace std;

#include <unistd.h>

#include <wiringPi.h>
#include <wiringPiSPI.h>

typedef bool boolean;
typedef unsigned char byte;

static const int CHANNEL = 0;

byte currentMode = 0x81;

char message[256];
char b64[256];

bool sx1272 = true;

byte receivedbytes;

struct sockaddr_in si_other;
int s, slen=sizeof(si_other);
struct ifreq ifr;

uint32_t cp_nb_rx_rcv;
uint32_t cp_nb_rx_ok;
uint32_t cp_nb_rx_bad;
uint32_t cp_nb_rx_nocrc;
uint32_t cp_up_pkt_fwd;

enum sf_t { SF7=7, SF8, SF9, SF10, SF11, SF12 };

/*******************************************************************************
 *
 * Configure these values!
 *
 *******************************************************************************/

// SX1272 - Raspberry connections
int ssPin = 6; 
int dio0 = 7; 
int RST =0; 
int TxEnable = 29; //mma 
int RxEnable = 27; //mma

// Set spreading factor (SF7 - SF12)
sf_t sf =SF12; //mma  SF7;

// Set center frequency
uint32_t  freq =868100000; //=867.5


// #############################################
// #############################################

#define REG_FIFO                    0x0
#define REG_FIFO_ADDR_PTR           0x0D
#define REG_FIFO_TX_BASE_AD         0x0E
#define REG_FIFO_RX_BASE_AD         0x0F
#define REG_RX_NB_BYTES             0x13
#define REG_OPMODE                  0x01
#define REG_FIFO_RX_CURRENT_ADDR    0x10
#define REG_IRQ_FLAGS               0x12
#define REG_DIO_MAPPING_1           0x40
#define REG_DIO_MAPPING_2           0x41
#define REG_MODEM_CONFIG            0x1D
#define REG_MODEM_CONFIG2           0x1E
#define REG_MODEM_CONFIG3           0x26
#define REG_SYMB_TIMEOUT_LSB  		0x1F
#define REG_PKT_SNR_VALUE			0x19
#define REG_PAYLOAD_LENGTH          0x22
#define REG_IRQ_FLAGS_MASK          0x11
#define REG_MAX_PAYLOAD_LENGTH 		0x23
#define REG_HOP_PERIOD              0x24
#define REG_SYNC_WORD				0x39
#define REG_VERSION	  				0x42
//mma
#define REG_LR_PADAC		 0x4d
#define REG_OCP                  0x0b
#define REG_PA_CONFIG            0x09

//mma
#define REG_MODEM_STAT              0x18
#define REG_DETECTION_OPTIMIZE      0x31
#define REG_DETECTION_TRESHOLD      0x37


#define SX72_MODE_RX_CONTINUOS      0x85
#define SX72_MODE_TX                0x83
#define SX72_MODE_SLEEP             0x80
#define SX72_MODE_STANDBY           0x81

#define PAYLOAD_LENGTH              0x40

// LOW NOISE AMPLIFIER
#define REG_LNA                     0x0C
#define LNA_MAX_GAIN                0x23
#define LNA_OFF_GAIN                0x00
#define LNA_LOW_GAIN		    	0x20

// CONF REG
#define REG1                        0x0A
#define REG2                        0x84

#define SX72_MC2_FSK                0x00
#define SX72_MC2_SF7                0x70
#define SX72_MC2_SF8                0x80
#define SX72_MC2_SF9                0x90
#define SX72_MC2_SF10               0xA0
#define SX72_MC2_SF11               0xB0
#define SX72_MC2_SF12               0xC0

#define SX72_MC1_LOW_DATA_RATE_OPTIMIZE  0x01 // mandated for SF11 and SF12

// FRF
#define        REG_FRF_MSB              0x06
#define        REG_FRF_MID              0x07
#define        REG_FRF_LSB              0x08

#define        FRF_MSB                  0xD9 // 868.1 Mhz
#define        FRF_MID                  0x06
#define        FRF_LSB                  0x66
//mma port lora.cpp 
#define IRQ_TX_DONE_MASK 0x08
// PA config
#define PA_BOOST 0x80
#define PA_OUTPUT_RFO_PIN 0

#define MAX_PKT_LENGTH 255

#define BUFLEN 2048  //Max length of buffer

#define PROTOCOL_VERSION  1
#define PKT_PUSH_DATA 0
#define PKT_PUSH_ACK  1
#define PKT_PULL_DATA 2
#define PKT_PULL_RESP 3
#define PKT_PULL_ACK  4

#define TX_BUFF_SIZE  2048
#define STATUS_SIZE	  1024

void die(const char *s)
{
    perror(s);
    exit(1);
}

void selectreceiver()
{
    digitalWrite(ssPin, LOW);
}

void unselectreceiver()
{
    digitalWrite(ssPin, HIGH);
}

byte readRegister(byte addr)
{
    unsigned char spibuf[2];

    selectreceiver();
    spibuf[0] = addr & 0x7F;
    spibuf[1] = 0x00;
    wiringPiSPIDataRW(CHANNEL, spibuf, 2);
    unselectreceiver();

    return spibuf[1];
}

void writeRegister(byte addr, byte value)
{
    unsigned char spibuf[2];

    spibuf[0] = addr | 0x80;
    spibuf[1] = value;
    selectreceiver();
    wiringPiSPIDataRW(CHANNEL, spibuf, 2);

    unselectreceiver();
}


void LoRa_idle()
{
  writeRegister(REG_OPMODE, SX72_MODE_STANDBY);
                
}

void LoRa_sleep()
{
  writeRegister(REG_OPMODE, SX72_MODE_SLEEP);
}



boolean receivePkt(char *payload)
{

    // clear rxDone
    writeRegister(REG_IRQ_FLAGS, 0x40);

    int irqflags = readRegister(REG_IRQ_FLAGS);
    int i;

    cp_nb_rx_rcv++;
    //printf("\nIRQ_FLAGS msg= %x\n",irqflags); //mma
    //  payload crc: 0x20
    if((irqflags & 0x20) == 0x20)
    {
        printf("CRC error\n");
        writeRegister(REG_IRQ_FLAGS, 0x20);
        return false;
    } else {

        cp_nb_rx_ok++;

        byte currentAddr = readRegister(REG_FIFO_RX_CURRENT_ADDR);
        byte receivedCount = readRegister(REG_RX_NB_BYTES);
        receivedbytes = receivedCount;

        writeRegister(REG_FIFO_ADDR_PTR, currentAddr);

        for(i = 0; i < receivedCount; i++)
        {
            payload[i] = (char)readRegister(REG_FIFO);
        }
        payload[i]=0;//mma set zero for string end
    }
    return true;
}

void SetupLoRa()
{
    
    digitalWrite(RST, HIGH);
    sleep(0.100);
    digitalWrite(RST, LOW);
    sleep(0.100);
    byte version = readRegister(REG_VERSION);

    if (version == 0x22) {
        // sx1272
        printf("SX1272 detected, starting.\n");
        sx1272 = true;
    } else {
        // sx1276?
        digitalWrite(RST, LOW);
        sleep(0.100);
        digitalWrite(RST, HIGH);
        sleep(0.100);
        version = readRegister(REG_VERSION);

        if (version == 0x12) {
            // sx1276
            printf("SX1276 detected, starting.\n");
            sx1272 = false;
        } else {
            printf("Unrecognized transceiver.\n");
            //printf("Version: 0x%x\n",version);
            exit(1);
        }
    }

    writeRegister(REG_OPMODE, SX72_MODE_SLEEP);

    // set frequency
    uint64_t frf = ((uint64_t)freq << 19) / 32000000;
    writeRegister(REG_FRF_MSB, (uint8_t)(frf>>16) );
    writeRegister(REG_FRF_MID, (uint8_t)(frf>> 8) );
    writeRegister(REG_FRF_LSB, (uint8_t)(frf>> 0) );

    //writeRegister(REG_SYNC_WORD, 0x34); // LoRaWAN public sync word   MMA removed use dafault 0x12 as transmiter

    if (sx1272) {
        if (sf == SF11 || sf == SF12) {
            writeRegister(REG_MODEM_CONFIG,0x0B);
            writeRegister(REG_DETECTION_OPTIMIZE, 0xc3);
            writeRegister(REG_DETECTION_TRESHOLD, 0x0a);
        } 
        else {
            writeRegister(REG_MODEM_CONFIG,0x0A);
        }
        writeRegister(REG_MODEM_CONFIG2,(sf<<4) | 0x04);
    } else { //sx1276
        if (sf == SF11 || sf == SF12) {
            writeRegister(REG_MODEM_CONFIG3,0x0C);  //LNA  gain max and optimize low symbol rate
        } else {
            writeRegister(REG_MODEM_CONFIG3,0x04);
        }
        writeRegister(REG_MODEM_CONFIG,0x62); // BW=62.5KHz  was 0x72 BW=125KHz, Coding 4/5, Explicite header mode
        writeRegister(REG_MODEM_CONFIG2,(sf<<4) | 0x07);//mma was 0x4 made timeout biger
        writeRegister(REG_SYMB_TIMEOUT_LSB,0xff);
    }

    if (sf == SF10 || sf == SF11 || sf == SF12) {
        writeRegister(REG_SYMB_TIMEOUT_LSB,0x05);
    } else {
        writeRegister(REG_SYMB_TIMEOUT_LSB,0x08);
    }
    writeRegister(REG_MAX_PAYLOAD_LENGTH,0x80);
    writeRegister(REG_PAYLOAD_LENGTH,PAYLOAD_LENGTH);
    //writeRegister(REG_HOP_PERIOD,0xFF);      //mma disabled 
    writeRegister(REG_FIFO_TX_BASE_AD,0x00); //mma
    writeRegister(REG_FIFO_RX_BASE_AD,0x00); //mma
    writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_BASE_AD));

    // Set Continous Receive Mode
    writeRegister(REG_LNA, LNA_MAX_GAIN);  // max lna gain
    //writeRegister(REG_OPMODE, SX72_MODE_RX_CONTINUOS); //remarked for TX demo
    /* next section till the end copy-pase from LoRa.cpp */
    writeRegister( 0x70, 0x10 ); //pll=75khz default 300khz
    writeRegister(REG_OCP, 0xb); //disable over current protection
    // put in standby mode
    LoRa_idle();
}


void receivepacket() {

    long int SNR;
    int rssicorr;
#if 0
    byte modem_stat = readRegister(REG_MODEM_STAT)&0xf; 
    if ((prev_modem_stat!=modem_stat) ) //mma debug
    {
        printf("REG_MODEM_STAT= %x\n",modem_stat);
    }
    prev_modem_stat = modem_stat;
#endif
    if(digitalRead(dio0) == 1)
    {
	//printf("dio0=1  \n");//mma 
        if(receivePkt(message)) {
            printf("MMa real rx MSG= %s\n",message);//mma
            byte value = readRegister(REG_PKT_SNR_VALUE);
            if( value & 0x80 ) // The SNR sign bit is 1
            {
                // Invert and divide by 4
                value = ( ( ~value + 1 ) & 0xFF ) >> 2;
                SNR = -value;
            }
            else
            {
                // Divide by 4
                SNR = ( value & 0xFF ) >> 2;
            }
            
            if (sx1272) {
                rssicorr = 139;
            } else {
                rssicorr = 157;
            }

            printf("Packet RSSI: %d, ",readRegister(0x1A)-rssicorr);
            printf("RSSI: %d, ",readRegister(0x1B)-rssicorr);
            printf("SNR: %li, ",SNR);
            printf("Length: %i ",(int)receivedbytes);
            printf("Freq offset MSB=%d Mid=%d LSB=%d  ",readRegister(0x28),readRegister(0x29),readRegister(0x2A) );
            printf("FIFO_RX_CURR_ADDR=%x ",readRegister(REG_FIFO_RX_CURRENT_ADDR));
            printf("\n");
            fflush(stdout);

        } // received a message

    } // dio0=1
}

/****************************************/
/* mma section copy paste from lora.cpp */
/* verify before use                    */
/****************************************/

size_t LoRa_write(const uint8_t *buffer, size_t size)
{
  int currentLength = readRegister(REG_PAYLOAD_LENGTH);
  // check size
  if ((currentLength + size) > MAX_PKT_LENGTH) {
    size = MAX_PKT_LENGTH - currentLength;
  }

  // write data
  for (size_t i = 0; i < size; i++) {
    writeRegister(REG_FIFO, buffer[i]);
  }

  // update length
  writeRegister(REG_PAYLOAD_LENGTH, currentLength + size);

  return size;
}

void LoRa_setTxPower(int level, int outputPin)
{
  if (PA_OUTPUT_RFO_PIN == outputPin) {
    // RFO
    if (level < 0) {
      level = 0;
    } else if (level > 14) {
      level = 14;
    }

    writeRegister(REG_PA_CONFIG, 0x70 | level);
  } else {
    // PA BOOST
    if (level < 2) {
      level = 2;
    } else if (level > 17) {
      level = 17;
    }

    writeRegister(REG_PA_CONFIG, PA_BOOST | (level - 2));
    writeRegister(REG_LR_PADAC, 0x87);//mma enable 20dbm
  }
}

int BeginPacket()
{
  // put in standby mode
  LoRa_idle();

  writeRegister(REG_MODEM_CONFIG, readRegister(REG_MODEM_CONFIG) & 0xfe);  //explicit header mode

  // reset FIFO address and paload length
  writeRegister(REG_FIFO_ADDR_PTR, 0);
  writeRegister(REG_PAYLOAD_LENGTH, 0);

  return 1;
}

int EndPacket()
{
  int8_t tx_timeout=16;
  digitalWrite (RxEnable, LOW);
  digitalWrite (TxEnable, HIGH);
  sleep (0.3); //3msec
  //mma clear irq before tx
  writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
  // put in TX mode
  printf ("start TX\n");
  writeRegister(REG_OPMODE, SX72_MODE_TX);

  // wait for TX done
  while( ((readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0)&&(tx_timeout>0))
  {
	  sleep (1);
	  tx_timeout--;
  }
  //printf("before sleep\n");
  //sleep(3);
  //printf("after\n");
  // clear IRQ's
  writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);

  digitalWrite (TxEnable, LOW);
  sleep (0.001);
  digitalWrite (RxEnable, HIGH);
  printf("tx_timeout =%d\n",tx_timeout);
  return tx_timeout;
} 

void transmit_packet (int packet_nr)
{
  char lora_msg_str[30];

  int n=sprintf (lora_msg_str, "Greetings raspberry pi=%d\n", packet_nr);
  BeginPacket ();
  LoRa_write((uint8_t *)lora_msg_str, n);
  EndPacket();
}

int main () {
    printf("LoRa TX_DEMO program \n\n");
    wiringPiSetup () ;
    pinMode(ssPin, OUTPUT);
    pinMode(dio0, INPUT);
    pinMode(RST, OUTPUT);


    pinMode(TxEnable, OUTPUT); //mma
    digitalWrite(TxEnable, LOW);
    pinMode(RxEnable, OUTPUT); //mma
    digitalWrite(RxEnable, HIGH);// mma set rx mode    
    //int fd = 
    wiringPiSPISetup(CHANNEL, 500000);
    //cout << "Init result: " << fd << endl;

    SetupLoRa();
    LoRa_setTxPower(17,PA_OUTPUT_RFO_PIN+1);//pa_boost_pin use 7

    int msg_count=0;
    while(1) {

        transmit_packet(msg_count);
        msg_count++;
	//printf("LoRa message nr= %d\n",msg_count);
        sleep (5);
   }

    return (0);

}

