#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <set>
#include <algorithm>
using namespace std;
#include "API.h"
#define MAZE_SIZE 16

// Direction constants
const int dx[] = { 0,  0, -1,  1};  // x changes for Up, Down, Left, Right
const int dy[] = {-1,  1,  0,  0};  // y changes for Up, Down, Left, Right

const int INF = 1000;  // A large number to represent infinity

class Cell {
public:
    int x, y;
    bool visited;
    bool wallUp, wallRight, wallDown, wallLeft;

    // A* variables
    int gCost, hCost, fCost;  
    Cell* parent; // Pointer to track the shortest path

    // Default constructor
    Cell() : x(0), y(0), visited(false), 
             wallUp(false), wallRight(false), wallDown(false), wallLeft(false), 
             gCost(0), hCost(0), fCost(0), parent(nullptr) {}

    // Constructor with coordinates
    Cell(int x, int y) : x(x), y(y), visited(false), 
                         wallUp(false), wallRight(false), wallDown(false), wallLeft(false), 
                         gCost(0), hCost(0), fCost(0), parent(nullptr) {}

    bool hasWall(int direction) const {
        switch (direction) {
            case 0: return wallUp;
            case 1: return wallRight;
            case 2: return wallDown;
            case 3: return wallLeft;
            default: return false;
        }
    }
};

// Robot variables
int robotX = 0, robotY = 0;
int robotDirection = 0;  // Direction the robot is facing (0 = Up, 1 = Right, 2 = Down, 3 = Left)

// Maze setup
vector<vector<Cell>> maze(MAZE_SIZE, vector<Cell>(MAZE_SIZE));

// Log function for debugging and tracking movements
void log(const string& text) {
    cerr << text << endl;
}

int main() {
    API::turnRight();
    API::moveForward();
    log("Amogus");
    return 0;
} 

