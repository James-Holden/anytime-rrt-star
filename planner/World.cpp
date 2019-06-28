#include <random>
#include "World.h"
#include "Node.h"
#include "Point.h"
#include <chrono>

World::World(std::vector<char> input, int numRows, int numColumns){
	world = input;
	width = numColumns;
	height = numRows; 
	// obtain a seed from the timer
	typedef std::chrono::high_resolution_clock myclock;
	myclock::time_point beginning = myclock::now();
	myclock::duration d = myclock::now() - beginning;
	unsigned seed = d.count();
	generator.seed(seed);
}

//returns valid sample point, checks for goal 1 in 20 cases 
Point World::getValidPoint(){
	std::uniform_real_distribution<double> xDistribution(0.0, width); 
	std::uniform_real_distribution<double> yDistribution(0.0, height);
	char worldItem = '#';
	double samplex; 
	double sampley; 
	while(worldItem == '#'){
		samplex = xDistribution(generator);
		sampley = yDistribution(generator);
		int xCoord = int(floor(samplex));
		int yCoord = int(floor(sampley));
		worldItem = world[yCoord*width+xCoord];
	}
	Point p; 
	p.x = samplex;
	p.y = sampley; 
	return p;
}

//returns goal 1 in 20 times 
Node* World::getNodeSample(int bias, Point goalPoint){
	std::uniform_int_distribution<int> rand(0,20);
	int roll = rand(generator);
	if(roll == bias){
		return new Node(NULL, goalPoint);
	} 
	else{
		return new Node(NULL, getValidPoint());
	}
}

void World::print(){
	for(char c : world){
		std::cout << c;
	}
	std::cout << std::endl; 
}
