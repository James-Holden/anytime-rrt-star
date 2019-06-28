CC = g++
CFLAGS = -g -Wall -std=c++11

all: planner World.o Node.o Point.o clean

planner: planner World.o Node.o Point.o
	$(CC) $(CFLAGS) -o planner planner.cpp World.o Node.o Point.o  

World.o: World.cpp World.h Node.o Point.o 
	$(CC) $(CFLAGS) -c World.cpp 

Node.o: Node.cpp Node.h Point.o 
	$(CC) $(CFLAGS) -c Node.cpp

Point.o: Point.cpp Point.h
	$(CC) $(CFLAGS) -c Point.cpp

clean: 
	$(RM) all *.o *~
