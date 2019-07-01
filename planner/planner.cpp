#include <iostream>
#include <fstream> 
#include <string>
#include <vector>
#include <random>
#include <cmath>
#include <math.h> 
#include <algorithm> 
#include "Node.h"
#include "Point.h"
#include "World.h"


//DDA algorithm to check if line intersects blocked area 
//returns true if clear, false if collision  
bool validTrajectory(Node* nodeA, Node* nodeB, World world){
	// //std::cout << "validTrajectory()" << nodeA->x << ' ' << nodeA->y << " to " << nodeB->x << ' ' << nodeB->y << std::endl; 
	// double dx = abs(nodeA->x - nodeB->x);
	// double dy = abs(nodeA->y - nodeB->y); 
	// // int steps = abs(dx) > abs(dy) ? abs(dx) : abs(dy);
	// double steps = 500;  
	// // calculate increment in x & y for each steps 
	// double Xinc = dx / steps; 
	// double Yinc = dy / steps; 
	// // Put pixel for each step 
	// double curX = nodeA->x;
	// double curY = nodeA->y; 
	// for( int i = 0; i <= steps; i++ ){ 
	// 	if(world.world[(int(curY)*world.width)+int(curX)] == '#'){
	// 		//std::cout << "COLLISON" << std::endl;
	// 		return 0; 
	// 	}
	// 	curX += Xinc;//increment in x at each step 
	// 	curY += Yinc;//increment in y at each step 
	// } 
	// return 1; 

  double step,x,y;
  double x1 = nodeA->x;
  double y1 = nodeA->y;
  double x2 = nodeB->x;
  double y2 = nodeB->y; 
  double dx = (x2 - x1);
  double dy = (y2 - y1);
  if(abs(dx) >= abs(dy))
    step = abs(dx)*100;
  else
    step = abs(dy)*100;
  dx = dx / step;
  dy = dy / step;
  x = x1;
  y = y1;
  int i = 1;
  while(i <= step) {
    x = x + dx;
    y = y + dy;
    i = i + 1;
	if(world.world[(int(y)*world.width)+int(x)] == '#'){
		//std::cout << "COLLISON" << std::endl;
		return 0; 
	}
  }
  return 1; 
}

//takes two nodes and returns 2d euclidean distance between them 
double distance(Node* nodeA, Node* nodeB){
	double diffX = nodeA->x - nodeB->x;
	double diffY = nodeA->y - nodeB->y;
	return hypot(diffX, diffY);
}

//return the nearest existing node to a given node in the world 
//simple euclidean distence per pythagorean theorm used by distance() 
//O(n) compares all nodes and saves returns closest 
Node* nearestNeighbor(Node* node, std::vector<Node*> neighborhood){
	Node* nearest;
	double nearestDist = INFINITY;  
	for(Node* n : neighborhood){
		double dist = distance(node, n);
		if(dist < nearestDist){
			nearest = n; 
			nearestDist = dist;
		}
	}
	return nearest;
}

//given environment, start, and goal: find path from start to goal avoiding obsiticles
//returns vector of node* in which last is goal 
std::vector<Node*> rrt(World world, Point start, Point goalPoint){
	std::vector<Node*> nodes;
	Node* root = new Node(NULL, start);
	nodes.push_back(root); 
	while(1){
		Node* sample = world.getNodeSample(20, goalPoint);
		Node* nearest = nearestNeighbor(sample, nodes);
		if(validTrajectory(sample, nearest, world)){
			sample->parent = nearest;  
			nodes.push_back(sample);					
			if(sample->x == goalPoint.x && sample->y == goalPoint.y){
				return nodes;
			}
		}
	}
}

//single goal path planning with simple RRT 
std::vector<Node*> planner(World world, Point start, Point goal){
	return rrt(world, start, goal);
	//rrtstar(world, start, goal, root);
}

//print solution trajectory: series of x y points preceeded by number of points in solution
//last node is always goal, keep referring up and then reverse to be in order 
void printSolution(std::vector<Node*> tree){
	std::vector<Node*> solutionTrajectory;
	Node* cur = tree.back(); 
	tree.pop_back();//goal node is on the back 
	do{
		solutionTrajectory.push_back(cur);
		cur = cur->parent;
	}while(cur->parent);
	std::reverse(solutionTrajectory.begin(), solutionTrajectory.end());
	std::cout << solutionTrajectory.size()+1 << '\n';
	std::cout << tree.front()->location.x << ' ' << tree.front()->y << std::endl;
	for(Node* n : solutionTrajectory){
		std::cout << n->x << ' ' << n->y << '\n';
	}
}

//print entire explore tree in line segments: x1 y1 x2 y2 preceeded by number of lines 
void printTree(std::vector<Node*> tree){
	std::cout << tree.size()-1 << '\n'; 
	for(Node* n : tree){
		if(n->parent){ //root wont segfault
			std::cout << n->x << ' ' << n->y << ' ' << n->parent->x << ' ' << n->parent->y << '\n';
		}
	}
}

//planner takes world on stdin and performs planner 
int main(int argc, char* argv[]){

	//header info  
	char width[10];
	char height[10];
	std::cin.getline(width, 10);
	std::cin.getline(height, 10);
	int numColumns = std::stoi(width);
	int numRows = std::stoi(height);
	
	//read world into 1d: # = blocked, _ = open 
	std::vector<char> input;
	for(int i = 0; i < numRows*numColumns; i++){
		char cur; 
		std::cin >> cur; 
		input.push_back(cur);
	}


	//setup for sampler
	World world = World(input, numRows, numColumns);
	Point start = world.getValidPoint();
	Point goal = world.getValidPoint();

	std::cout << world.width << '\n' << world.height << std::endl;
	world.print(); 
	std::cout << start.x << '\n' << start.y << std::endl;
	std::cout << goal.x << '\n' << goal.y << std::endl;

	std::vector<Node*> tree = planner(world, start, goal);
	printSolution(tree);
	printTree(tree); 
	return 0;
}
