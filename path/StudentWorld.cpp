#include "StudentWorld.h"
#include <string>
#include <map>
#include <queue>
#include <functional>
using namespace std;

GameWorld* createStudentWorld(string assetDir)
{
	return new StudentWorld(assetDir);
}



StudentWorld::~StudentWorld()
	{
		// delete all GraphObjects
		vector<GraphObject*>::iterator it;
		for (it = graphobject_pt_vec.begin(); it!=graphobject_pt_vec.end(); ++it)
			delete *it;

		// delete world
		if (world)
			for (int x=0; x<VIEW_WIDTH; ++x)
				delete [] world[x];
			delete [] world;
	}


StudentWorld::StudentWorld(std::string assetDir):GameWorld(assetDir), lev(assetDirectory())
{
}

int StudentWorld::init()
{
	//Level lev(assetDirectory());
	Level::LoadResult result = lev.loadLevel("level0x.dat");
 
	if (result == Level::load_fail_file_not_found) 
		cout << "Could not find level00.dat data file\n";
	else if (result == Level::load_fail_bad_format)
		cout << "Your level was improperly formatted\n";
	else if (result == Level::load_success)
	{
		cout << "Successfully loaded level\n";
		cout << "Initialize Graph Objects"<<endl;
		init_graph_objects_and_world(lev);
	}
	return GWSTATUS_CONTINUE_GAME;
}


void StudentWorld::init_graph_objects_and_world(Level& L)
{
	// create world matrix
	world = new int*[VIEW_WIDTH];
	for (int x=0; x<VIEW_WIDTH; ++x)
		world[x] = new int[VIEW_HEIGHT];

	// initialize content
	for (int y=0; y<VIEW_HEIGHT; ++y)
		for (int x=0; x<VIEW_WIDTH; ++x)
		{
			// store each map element as a GraphObject
			// only need to handle player_starting_point (@), wall (#) and exit (x)

			GraphObject* go=NULL;
			Level::MazeEntry ge = L.getContentsOf(x,y);

			switch (ge)
			{
				case Level::player:
					go = new GraphObject(IID_PLAYER, x,y);
					world[x][y] = IID_PLAYER;
					break;
				case Level::wall:
					go = new GraphObject(IID_WALL, x,y);
					world[x][y] = IID_WALL;
					break;
				case Level::exit:
					go = new GraphObject(IID_EXIT, x,y);
					world[x][y] = IID_EXIT;
					break;
				default:
					world[x][y] = IID_EMPTY;
					break;		// ignore all other elements
			}

			if (go)
			{
				go->setVisible(true);
				graphobject_pt_vec.push_back(go);
			}
		}

	display_world();
}


// display content of world matrix in console
void StudentWorld::display_world()
{
	cout<<"World:"<<endl;

	for (int y=0; y<VIEW_HEIGHT; ++y)
	{
		for (int x=0; x<VIEW_WIDTH; ++x)
			cout<<world[x][VIEW_HEIGHT-1-y]<<" ";		
		cout<<endl;
	}
}


// !implement the A* pathfinding algorithm here
void StudentWorld::compute_path()
{
	cout<<"Compute path..."<<endl;

	// !implement the A* pathfinding function below; a world's entry that belongs to the resulting path should be assigned IID_PATH
	compute_a_star_path(world);	

	// !dummy code to demonstrate how to use IID_PATH to denote an entry that belongs to the path; YOU SHOULD COMMENT THIS OUT
	/*
  world[8][13] = IID_PATH;	
	world[8][12] = IID_PATH;	
	world[8][11] = IID_PATH;	
	world[8][10] = IID_PATH;	
	world[8][9] = IID_PATH;	
	world[8][8] = IID_PATH;	
	world[8][7] = IID_PATH;	
	world[8][6] = IID_PATH;	
	world[8][5] = IID_PATH;	
	world[8][4] = IID_PATH;	
	world[8][3] = IID_PATH;	
	world[7][3] = IID_PATH;	
	world[7][2] = IID_PATH;	
  */

	// show the path
	for (int y=0; y<VIEW_HEIGHT; ++y)
		for (int x=0; x<VIEW_WIDTH; ++x)
		{
			if (world[x][y] == IID_PATH)
			{
				GraphObject* go = new GraphObject(IID_PATH, x,y);
				go->setVisible(true);
				graphobject_pt_vec.push_back(go);
			}
		}

	return;
}


void StudentWorld::compute_a_star_path(int** w)
{
  // !your implementation here
  struct GridPos {
    int x,y;
    bool operator<(const GridPos& rhs) const
    {
      return (x == rhs.x ? y < rhs.y : x < rhs.x);
    }
    bool equals(const GridPos& rhs) const
    {
      return (x == rhs.x && y == rhs.y);
    }
  };
  typedef pair<int,GridPos> FrontierEntry;

  GridPos startPos, endPos;
  map<GridPos,GridPos> cameFrom;
  map<GridPos,int> currCost;
  priority_queue<FrontierEntry,vector<FrontierEntry>,greater<FrontierEntry>> frontier;
  
  // find start and end
  bool startFound, endFound;
  int x, y;
  startFound = endFound = false;
  x = y = 1;
  // while start end end are not found
  while (!(startFound && endFound))
  {
    // check if the current position is a start or end and assign and confirm
    if (!startFound && w[x][y] == IID_PLAYER)
    {
      startPos.x = x;
      startPos.y = y;
      startFound = true;
      cout << "Start Position: [" << startPos.x << ", " << startPos.y << "]\n";
    }
    else if (!endFound && w[x][y] == IID_EXIT)
    {
      endPos.x = x;
      endPos.y = y;
      endFound = true;
      cout << "End Position: [" << endPos.x << ", " << endPos.y << "]\n";
    }

    // check the next entry
    if (++y == 14)  // increment to the next column and see if it hits the end
    {
      x++;     // goto next row
      y = 1;  // goto next start of row
    }

  }
  
  // insert necessary values
  GridPos tempGP;
  FrontierEntry tempFE;

  tempGP.x = tempGP.y = 0;
  tempFE.first = 0;
  tempFE.second = startPos;

  cameFrom[startPos] = tempGP;
  currCost[startPos] = 0;
  frontier.push(tempFE);

  // actual A* algo
  while (!frontier.empty())
  {
    // get values
    GridPos currPos = frontier.top().second;
    int associatedCost = frontier.top().first;
    frontier.pop();

    // check if at goal
    if (currPos.equals(endPos))
      break;

    // search neighbors
    GridPos neighbors [4];  // 0 - RIGHT, 1 - DOWN, 2 - LEFT, 3 - UP
    neighbors[0] = currPos;
    neighbors[0].x++;
    neighbors[2] = neighbors[0];
    neighbors[2].x -= 2;
    neighbors[1] = neighbors[2];
    neighbors[1].x++;
    neighbors[1].y--;
    neighbors[3] = neighbors[1];
    neighbors[3].y += 2;

    for (GridPos next : neighbors)
    {
      if (w[next.x][next.y] != IID_WALL)
      {
        int newCost = associatedCost+1;
        if (currCost.find(next) == currCost.end() || newCost < currCost[next])
        {
          currCost[next] = newCost;
          int priority = newCost + abs(endPos.x - next.x) + abs(endPos.y - next.y);
          tempFE.first = priority;
          tempFE.second = next;
          frontier.push(tempFE);
          cameFrom[next] = currPos;
        }
      }
    }
  }

  // trace
  GridPos tracer = cameFrom[endPos];
  while (!tracer.equals(startPos))
  {
    w[tracer.x][tracer.y] = IID_PATH;
    tracer = cameFrom[tracer];
  }
}



int StudentWorld::move()
{
	//decLives();
	return GWSTATUS_CONTINUE_GAME;
	//return GWSTATUS_PLAYER_DIED;
}


void StudentWorld::cleanUp()
{
}
