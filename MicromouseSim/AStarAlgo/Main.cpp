#include <iostream>
#include <vector>
#include <list>
#include <cmath>
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

    void setWall(int direction) {
        switch (direction) {
            case 0: wallUp = true; break;
            case 1: wallRight = true; break;
            case 2: wallDown = true; break;
            case 3: wallLeft = true; break;
        }
    }

    void removeWall(int direction) {
        switch (direction) {
            case 0: wallUp = false; break;
            case 1: wallRight = false; break;
            case 2: wallDown = false; break;
            case 3: wallLeft = false; break;
        }
    }
};

// Robot variables
int robotX = 0, robotY = 0;
int robotDirection = 0;  // Direction the robot is facing (0 = Up, 1 = Right, 2 = Down, 3 = Left)

// Maze setup
vector<vector<Cell>> maze(MAZE_SIZE, vector<Cell>(MAZE_SIZE));

// Log function for debugging and tracking movements
void log(const std::string& text) {
    std::cerr << text << std::endl;
}

// Explore the maze based on the robot's current position and sensor data
void exploreMaze() {
    if (API::wallFront()) {
        if (robotDirection == 0) maze[robotX][robotY].wallUp = true;
        else if (robotDirection == 1) maze[robotX][robotY].wallRight = true;
        else if (robotDirection == 2) maze[robotX][robotY].wallDown = true;
        else if (robotDirection == 3) maze[robotX][robotY].wallLeft = true;
    }

    if (API::wallRight()) {
        if (robotDirection == 0) maze[robotX][robotY].wallRight = true;
        else if (robotDirection == 1) maze[robotX][robotY].wallDown = true;
        else if (robotDirection == 2) maze[robotX][robotY].wallLeft = true;
        else if (robotDirection == 3) maze[robotX][robotY].wallUp = true;
    }

    if (API::wallLeft()) {
        if (robotDirection == 0) maze[robotX][robotY].wallLeft = true;
        else if (robotDirection == 1) maze[robotX][robotY].wallUp = true;
        else if (robotDirection == 2) maze[robotX][robotY].wallRight = true;
        else if (robotDirection == 3) maze[robotX][robotY].wallDown = true;
    }
}

// Function to move the robot based on the direction it is facing
void moveRobot(int direction) {
    if (direction == 0) {
        API::moveForward();
        robotY -= 1;  // Move Up
    } else if (direction == 1) {
        API::moveForward();
        robotX += 1;  // Move Right
    } else if (direction == 2) {
        API::moveForward();
        robotY += 1;  // Move Down
    } else if (direction == 3) {
        API::moveForward();
        robotX -= 1;  // Move Left
    }
    log("Robot moved to (" + to_string(robotX) + ", " + to_string(robotY) + ")");
}

// Function to turn the robot to a specific direction
void turnRobot(int newDirection) {
    while (robotDirection != newDirection) {
        API::turnRight();
        robotDirection = (robotDirection + 1) % 4;
    }
    log("Robot turned to direction " + to_string(robotDirection));
}

// A* Search Incrementally: Updates the path while moving
void aStarSearchIncremental(int startX, int startY, int goalX, int goalY) {
    vector<vector<bool>> openList(MAZE_SIZE, vector<bool>(MAZE_SIZE, false));  // Open list
    vector<vector<int>> gCost(MAZE_SIZE, vector<int>(MAZE_SIZE, INF));    // Cost to reach each cell
    vector<vector<Cell*>> parent(MAZE_SIZE, vector<Cell*>(MAZE_SIZE, nullptr));  // Parent cell pointers

    // Initialize start position
    gCost[startX][startY] = 0;
    openList[startX][startY] = true;

    // Priority Queue: open list sorted by fCost
    list<Cell*> openSet;
    openSet.push_back(&maze[startX][startY]);

    while (!openSet.empty()) {
        Cell* current = openSet.front();
        openSet.pop_front();

        // Check if goal is reached
        if (current->x == goalX && current->y == goalY) {
            vector<pair<int, int>> path;
            Cell* temp = current;
            while (temp != nullptr) {
                path.push_back({temp->x, temp->y});
                temp = temp->parent;
            }

            // Manually reverse the path
            vector<pair<int, int>> reversedPath;
            for (int i = path.size() - 1; i >= 0; --i) {
                reversedPath.push_back(path[i]);
            }

            // Move robot along the path
            for (auto& p : reversedPath) {
                int targetX = p.first;
                int targetY = p.second;
                int direction = -1;

                // Determine direction to move
                if (targetX == robotX && targetY == robotY - 1) direction = 0;  // Up
                else if (targetX == robotX + 1 && targetY == robotY) direction = 1;  // Right
                else if (targetX == robotX && targetY == robotY + 1) direction = 2;  // Down
                else if (targetX == robotX - 1 && targetY == robotY) direction = 3;  // Left

                if (direction != -1) {
                    turnRobot(direction);
                    moveRobot(direction);
                    exploreMaze();  // Update maze with sensor data after moving
                }
            }
            return;
        }

        // Explore neighbors
        for (int dir = 0; dir < 4; ++dir) {
            int neighborX = current->x + dx[dir];
            int neighborY = current->y + dy[dir];

            if (neighborX >= 0 && neighborX < MAZE_SIZE && neighborY >= 0 && neighborY < MAZE_SIZE && !current->hasWall(dir)) {
                int newGCost = gCost[current->x][current->y] + 1;
                if (newGCost < gCost[neighborX][neighborY]) {
                    gCost[neighborX][neighborY] = newGCost;
                    openList[neighborX][neighborY] = true;
                    parent[neighborX][neighborY] = current;

                    int hCost = abs(neighborX - goalX) + abs(neighborY - goalY);
                    int fCost = newGCost + hCost;

                    Cell* neighborCell = &maze[neighborX][neighborY];
                    openSet.push_back(neighborCell);
                }
            }
        }
    }
}

// Function to explore and move towards the goal incrementally
void exploreAndMoveToGoal(int goalX, int goalY) {
    while (robotX != goalX || robotY != goalY) {
        exploreMaze();  // Update maze with current sensor data
        aStarSearchIncremental(robotX, robotY, goalX, goalY);  // Perform incremental A* search
    }
}

int main() {
    // Define some goals
    vector<pair<int, int>> goals = {{7, 7}, {7, 8}, {8, 7}, {8, 8}};

    // Move towards the first goal
    exploreAndMoveToGoal(goals[0].first, goals[0].second);

    return 0;
}
