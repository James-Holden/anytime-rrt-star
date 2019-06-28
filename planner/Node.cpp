#include "Node.h"
#include <iostream>
		
Node::Node(Node* parent_, Point location_){
	parent = parent_;
	location = location_; 
	x = location_.x; 
	y = location.y; 
}

Node::Node(){

}

void Node::print(){
	std::cout << location.x << ' ' << location.y << parent << std::endl;// ' ' << parent->location.x << parent->location.y << '\n';   
}