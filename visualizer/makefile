CC = g++
CFLAGS = -g -Wall -std=c++11


all: visualizer visualizer.o clean

visualizer: visualizer.o
	$(CC) visualizer.o -o visualizer -lsfml-graphics -lsfml-window -lsfml-system

visualizer.o: visualizer.cpp  
	$(CC) $(CFLAGS) -c visualizer.cpp

clean: 
	$(RM) all *.o *~
