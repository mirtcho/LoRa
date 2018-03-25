# LoRaSender
# transmit loar demo 

CC=g++
CFLAGS=-c -Wall
LIBS=-lwiringPi

all: tx_demo

tx_demo: LoRa.o LoRaSender.o
	$(CC) LoRa.o LoRaSender.o $(LIBS) -o Tx_demo

LoRa.o: LoRa.cpp
	$(CC) $(CFLAGS) LoRa.cpp

LoRaSender.o: LoRaSender.c
	$(CC) $(CFLAGS) LoRaSender.c

clean:
	rm *.o tx_demo	
