#ifndef STUDENTWORLD_H_
#define STUDENTWORLD_H_

#include "GameWorld.h"
#include "GameConstants.h"
#include <string>
#include "GraphObject.h"


#include <iostream>
#include "Level.h"
#include <vector>


using namespace std;

// Students:  Add code to this file, StudentWorld.cpp, Actor.h, and Actor.cpp

class StudentWorld : public GameWorld
{
public:

	Level lev;										//store the loaded level here
	vector<GraphObject*> graphobject_pt_vec;		//store all the graph objects here
	int** world;									//2D matrix of the world on which the A* path is computed

	virtual ~StudentWorld();
	StudentWorld(std::string assetDir);
	virtual int init();
	void init_graph_objects_and_world(Level& L);
	void display_world();							// display content of world matrix in console	
	void compute_path();							
	void compute_a_star_path(int** w);
	virtual int move();
	virtual void cleanUp();


private:
};

#endif // STUDENTWORLD_H_
