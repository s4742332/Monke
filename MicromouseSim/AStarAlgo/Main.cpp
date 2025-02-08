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
void exploreMaze() {
    // Check the front wall
    if (API::wallFront()) {
        switch (robotDirection) {
            case 0: maze[robotX][robotY].wallUp = true; break;    // Facing Up
            case 1: maze[robotX][robotY].wallRight = true; break; // Facing Right
            case 2: maze[robotX][robotY].wallDown = true; break;  // Facing Down
            case 3: maze[robotX][robotY].wallLeft = true; break;  // Facing Left
        }
    }

    // Check the right wall
    if (API::wallRight()) {
        switch (robotDirection) {
            case 0: maze[robotX][robotY].wallRight = true; break; // Facing Up
            case 1: maze[robotX][robotY].wallDown = true; break;  // Facing Right
            case 2: maze[robotX][robotY].wallLeft = true; break;  // Facing Down
            case 3: maze[robotX][robotY].wallUp = true; break;    // Facing Left
        }
    }

    // Check the left wall
    if (API::wallLeft()) {
        switch (robotDirection) {
            case 0: maze[robotX][robotY].wallLeft = true; break;   // Facing Up
            case 1: maze[robotX][robotY].wallUp = true; break;     // Facing Right
            case 2: maze[robotX][robotY].wallRight = true; break;  // Facing Down
            case 3: maze[robotX][robotY].wallDown = true; break;   // Facing Left
        }
    }

    // Mark this cell as visited
    maze[robotX][robotY].visited = true;
}

void moveRobot(int direction) {
    // Check if there's no wall in front (based on the current direction)
    if (!maze[robotX][robotY].hasWall(direction)) {
        // Call the API to move the robot forward
        API::moveForward();
        
        // Update the robot's position based on the direction it is facing
        robotX += dx[direction];
        robotY += dy[direction];

        // Log the robot's new position
        log("Robot moved to (" + to_string(robotX) + ", " + to_string(robotY) + ")");
        
        // Update the maze with new sensor data after moving
        exploreMaze();
    }
    else {
        log("Cannot move, wall detected!");
    }
}

void turnRobot(int newDirection) {
    // Calculate the difference in direction (normalize it to a value between 0 and 3)
    int diff = (newDirection - robotDirection + 4) % 4;  // This ensures a positive difference
    
    // Turn logic based on the calculated difference (diff)
    if (diff == 1) {  // Turn right once (90 degrees clockwise)
        API::turnRight();   // Turn the robot 90 degrees to the right
        robotDirection = (robotDirection + 1) % 4;  // Update the robot's direction
    }
    else if (diff == 2) {  // Turn around (180 degrees)
        API::turnRight();   // First 90 degrees clockwise
        API::turnRight();   // Another 90 degrees to complete the 180 degree turn
        robotDirection = (robotDirection + 2) % 4;  // Update the robot's direction to reflect the turn
    }
    else if (diff == 3) {  // Turn left once (90 degrees counterclockwise)
        log("Turning left");
        API::turnLeft();    // Turn the robot 90 degrees to the left
        robotDirection = (robotDirection + 3) % 4;  // Update the robot's direction
    }

    // Log the new direction after the turn
    log("Robot turned to direction " + to_string(robotDirection));
}


int main() {
    turnRobot(3);
    moveRobot(1);
}

