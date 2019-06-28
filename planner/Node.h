#ifndef NODE_H
#define NODE_H

#include <iostream>
#include "Point.h"

class Node{
private: 
public:
	Node* parent; 
	double x; 
	double y; 
	Point location;
	Node(Node* parent_, Point location_);
	Node();
	void print();
};

#endif