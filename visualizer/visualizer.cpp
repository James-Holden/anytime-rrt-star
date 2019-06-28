#include <SFML/Graphics.hpp>
#include <iostream>
#include <string>
#include <stdio.h>
#include <string.h>
#include <tuple>


int main()
{
    //header info  
	char width[10];
	char height[10];
	std::cin.getline(width, 10);
	std::cin.getline(height, 10);
	int numColumns = std::stoi(width);
	int numRows = std::stoi(height);
	
	//read world into 1d: # = blocked, _ = open 
	std::vector<char> world;
	for(int i = 0; i < numRows*numColumns; i++){
		char cur; 
		std::cin >> cur; 
		world.push_back(cur);
	}

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
    
    //create world blocks 
    std::vector<sf::RectangleShape> blocks; 
    for(int i = 0; i < numColumns; i++){
        for(int j = 0; j < numRows; j++){
            if(world[j*numColumns+i] == '_'){
                sf::RectangleShape rectangle(sf::Vector2f(50, 50));
                rectangle.setPosition(i*50, j*50);
                rectangle.setFillColor(sf::Color::White);
                blocks.push_back(rectangle);
            }
        }
    }

    //create start/finish dots 
    sf::CircleShape startCircle(10);
    startCircle.setFillColor(sf::Color::Green);
    startCircle.setPosition(sx*50-10, sy*50-10);
    sf::CircleShape goalCircle(10);
    goalCircle.setFillColor(sf::Color::Red);
    goalCircle.setPosition(gx*50-10, gy*50-10);

    std::vector<double> solutionPointXs; 
    std::vector<double> solutionPointYs; 

    //parse solution lines 
    std::vector<char[20]> points;  
    char charSolutionPoints[10]; 
    std::cin.getline(charSolutionPoints, 10);
    int numSolutionPoints = std::stoi(charSolutionPoints); 
    for (int i = 0; i < numSolutionPoints; i++)
    {
        char solutionPoint[20]; 
        std::cin.getline(solutionPoint, 20);
        char* split; 
        split = strtok(solutionPoint, " ");
        solutionPointXs.push_back(std::stod(split));
        // std::cout << split << std::endl;
        split = strtok(NULL, " ");
        solutionPointYs.push_back(std::stod(split));

        // std::cout << split << std::endl;
    }
    //create solution lines 
    std::vector<sf::VertexArray> soltuionLines;
    for (int i = 0; i < numSolutionPoints-1; i++)
    {
        std::cout << '!' << solutionPointXs[i] << ' ' << solutionPointYs[i] << std::endl;
        std::cout << '!' << solutionPointXs[i+1] << ' ' << solutionPointYs[i+1] << std::endl;
    
        sf::VertexArray line(sf::Lines, 2);
        line[0].position = sf::Vector2f(solutionPointXs[i]*50, solutionPointYs[i]*50);
        line[0].color = sf::Color::Black;
        line[1].position = sf::Vector2f(solutionPointXs[i+1]*50, solutionPointYs[i+1]*50);
        line[1].color = sf::Color::Black;
        soltuionLines.push_back(line);
    }
    

    std::vector<sf::VertexArray> exploredLines; 
    //parse explored lines 
    char charExploredLines[10]; //number of lines 
    std::cin.getline(charExploredLines, 10);
    int numExploredLines = std::stoi(charExploredLines);
    for (int i = 0; i < numExploredLines; i++)
    {
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

        std::cout << x1 << ' ' << y1 << ' ' << x2 << ' ' << y2 << std::endl;

        sf::VertexArray line(sf::Lines, 2);
        line[0].position = sf::Vector2f(x1*50, y1*50);
        line[0].color = sf::Color::Green;
        line[1].position = sf::Vector2f(x2*50, y2*50);
        line[1].color = sf::Color::Green;
        exploredLines.push_back(line);

    }
    





    sf::RenderWindow window(sf::VideoMode(numColumns*50, numRows*50), "RRT Visualizer");

    while (window.isOpen()){

        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        window.clear();
        for(sf::RectangleShape block : blocks){
            window.draw(block);
        }
        window.draw(startCircle);
        window.draw(goalCircle);
        for(sf::VertexArray var : exploredLines){
            window.draw(var); 
        }
        for(sf::VertexArray var : soltuionLines){
            window.draw(var);
        }

        window.display();

    }

    return 0;
}
