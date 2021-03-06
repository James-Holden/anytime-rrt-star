#include <SFML/Graphics.hpp>
#include <iostream>
#include <string>
#include <stdio.h>
#include <string.h>
#include <tuple>
#include <unistd.h>
#include <fstream>

int scaler = 75; //scale up size from model to graphics window

std::vector<sf::RectangleShape> buildWorld(int numRows, int numColumns){
    std::vector<char> world; 
    for(int i = 0; i < numRows*numColumns; i++){
		char cur; 
		std::cin >> cur; 
		world.push_back(cur);
	}
    std::vector<sf::RectangleShape> blocks; 
    for(int i = 0; i < numColumns; i++){
        for(int j = 0; j < numRows; j++){
            if(world[j*numColumns+i] == '_'){
                sf::RectangleShape rectangle(sf::Vector2f(scaler, scaler));
                rectangle.setPosition(i*scaler, j*scaler);
                rectangle.setFillColor(sf::Color::White);
                blocks.push_back(rectangle);
            }
        }
    }
    return blocks; 
}

std::vector<sf::VertexArray> buildSoltuionLines(std::ostream* myfile){
    std::vector<double> solutionPointXs; 
    std::vector<double> solutionPointYs; 
    //parse solution lines 
    char duration[20];  
    char soltuionLength[20];  
  
    char charSolutionPoints[10]; 
    std::cin.getline(charSolutionPoints, 10);
    int numSolutionPoints = std::stoi(charSolutionPoints); 
    if(numSolutionPoints){
        std::cin.getline(duration, 10);
        std::cin.getline(soltuionLength, 10);
        *myfile << duration << ',' << soltuionLength <<'\n';
    }
    
    for (int i = 0; i < numSolutionPoints; i++){
        char solutionPoint[20]; 
        std::cin.getline(solutionPoint, 20);
        char* split; 
        split = strtok(solutionPoint, " ");
        solutionPointXs.push_back(std::stod(split));
        split = strtok(NULL, " ");
        solutionPointYs.push_back(std::stod(split));
    }

    //create solution lines 
    std::vector<sf::VertexArray> soltuionLines;
    for (int i = 0; i < numSolutionPoints-1; i++){
        sf::VertexArray line(sf::Lines, 2);
        line[0].position = sf::Vector2f(solutionPointXs[i]*scaler, solutionPointYs[i]*scaler);
        line[0].color = sf::Color::Black;
        line[1].position = sf::Vector2f(solutionPointXs[i+1]*scaler, solutionPointYs[i+1]*scaler);
        line[1].color = sf::Color::Black;
        soltuionLines.push_back(line);
    }

    return soltuionLines; 
}

std::vector<sf::VertexArray> buildExploredLines(){
    std::vector<sf::VertexArray> exploredLines; 
    //parse explored lines 
    char charExploredLines[10]; //number of lines 
    std::cin.getline(charExploredLines, 10);
    int numExploredLines = std::stoi(charExploredLines);
    for (int i = 0; i < numExploredLines; i++){
        char thisLine[40]; 
        std::cin.getline(thisLine, 40);
        char* split; 
        split = strtok(thisLine, " ");
        double x1 = std::stod(split); 
        split = strtok(NULL, " ");
        double y1 = std::stod(split); 
        split = strtok(NULL, " ");
        double x2 = std::stod(split); 
        split = strtok(NULL, " ");
        double y2 = std::stod(split); 

        sf::VertexArray line(sf::Lines, 2);
        line[0].position = sf::Vector2f(x1*scaler, y1*scaler);
        line[0].color = sf::Color::Green;
        line[1].position = sf::Vector2f(x2*scaler, y2*scaler);
        line[1].color = sf::Color::Green;
        exploredLines.push_back(line);
    }
    return exploredLines; 
}


void displayScene(sf::RenderWindow* window, int numRows, int numColumns, std::ostream* myfile){
    
    //create world blocks 
    std::vector<sf::RectangleShape> blocks = buildWorld(numRows, numColumns); 

    char decoy[10]; 
    char startx[10];
    char starty[10];
    char goalx[10];
    char goaly[10];
    std::cin.getline(decoy, 10);
    std::cin.getline(startx, 10);
	std::cin.getline(starty, 10);
    std::cin.getline(goalx, 10);
	std::cin.getline(goaly, 10);
    double sx = std::stod(startx);
    double sy = std::stod(starty);
    double gx = std::stod(goalx);
    double gy = std::stod(goaly);

    //create start/finish dots 
    sf::CircleShape startCircle(5);
    startCircle.setFillColor(sf::Color::Green);
    startCircle.setPosition(sx*scaler-5, sy*scaler-5);
    sf::CircleShape goalCircle(5);
    goalCircle.setFillColor(sf::Color::Red);
    goalCircle.setPosition(gx*scaler-5, gy*scaler-5);

    std::vector<sf::VertexArray> exploredLines = buildExploredLines(); 
    std::vector<sf::VertexArray> solutionLines = buildSoltuionLines(myfile);
    
    while (window->isOpen()){
        sf::Event event;

        while (window->pollEvent(event)){
            if (event.type == sf::Event::Closed)
                window->close();
        }
        window->clear();
        for(sf::RectangleShape block : blocks){
            window->draw(block);
        }
        window->draw(startCircle);
        window->draw(goalCircle);
        for(sf::VertexArray line : exploredLines){
            window->draw(line); 
        }
        for(sf::VertexArray line : solutionLines){
           window->draw(line);
        }
        window->display();
        usleep(1000);
        return;
    }
}

int main(){

    std::ofstream myfile;
    myfile.open ("results.csv");

    //header info  
	char width[10];
	char height[10];
	std::cin.getline(width, 10);
	std::cin.getline(height, 10);
	int numColumns = std::stoi(width);
	int numRows = std::stoi(height);

    sf::RenderWindow window(sf::VideoMode(numColumns*scaler, numRows*scaler), "RRT Visualizer");
    while(std::cin){
        //try
        // {
        displayScene(&window, numRows, numColumns, &myfile);
        // }
        // catch(const std::exception& e)
        // {
        //     //std::cerr << e.what() << '\n';
        // }
        
    }

    return 0;
}
