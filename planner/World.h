#ifndef WORLD_H
#define WORLD_H
#include "Point.h"
#include "Node.h"
//represents search space of agent 
class World{
private: 
public:
	std::default_random_engine generator;
	std::vector<char> world; //1D vector: # = blocked _ = open 
	int width; 
	int height;

	World(std::vector<char> input, int numRows, int numColumns);

	//returns valid sample point, checks for goal 1 in 20 cases 
	Point getValidPoint();

	//returns goal 1 in 20 times 
	Node* getNodeSample(int bias, Point goalPoint);

	void print();
};
#endif