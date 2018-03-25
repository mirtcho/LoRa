# Tx_demo
# transmit demo 

CC=g++
CFLAGS=-c -Wall
LIBS=-lwiringPi

all: tx_demo

tx_demo: main.o
	$(CC) main.o $(LIBS) -o tx_demo

main.o: main.cpp
	$(CC) $(CFLAGS) main.cpp


clean:
	rm *.o tx_demo
