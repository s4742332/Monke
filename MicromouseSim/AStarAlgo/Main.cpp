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
int hasMoved = 0;

class Cell {
public:
    int x, y;
    bool visited;
    bool wallUp, wallRight, wallDown, wallLeft;

    // A* variables
    int gCost, hCost, fCost;
    Cell* parent;

    Cell() : x(0), y(0), visited(false), wallUp(false), wallRight(false),
             wallDown(false), wallLeft(false), gCost(0), hCost(0), fCost(0), parent(nullptr) {}

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

bool isInBounds(int x, int y) {
    return (x >= 0 && x < MAZE_SIZE && y >= 0 && y < MAZE_SIZE);
}

int calculateHeuristic(int x, int y, int goalX, int goalY) {
    // Use Manhattan distance as heuristic
    int heuristic = abs(goalX - x) + abs(goalY - y);
    return heuristic;
}

void exploreMaze() {
    log("Scanning (" + to_string(robotX) + ", " + to_string(robotY) + ")");
    
    // Check if there is a wall in front of the robot.
    if (API::wallFront()) {
        if (robotDirection == 0) maze[robotX][robotY].wallUp = true;
        else if (robotDirection == 1) maze[robotX][robotY].wallRight = true;
        else if (robotDirection == 2) maze[robotX][robotY].wallDown = true;
        else if (robotDirection == 3) maze[robotX][robotY].wallLeft = true;
        log("Wall detected in front.");
    }

    if (API::wallRight()) {
        if (robotDirection == 0) maze[robotX][robotY].wallRight = true;
        else if (robotDirection == 1) maze[robotX][robotY].wallDown = true;
        else if (robotDirection == 2) maze[robotX][robotY].wallLeft = true;
        else if (robotDirection == 3) maze[robotX][robotY].wallUp = true;
        log("Wall detected to the right.");
    }

    if (API::wallLeft()) {
        if (robotDirection == 0) maze[robotX][robotY].wallLeft = true;
        else if (robotDirection == 1) maze[robotX][robotY].wallUp = true;
        else if (robotDirection == 2) maze[robotX][robotY].wallRight = true;
        else if (robotDirection == 3) maze[robotX][robotY].wallDown = true;
        log("Wall detected to the left.");
    }
}

void turnRobot(int newDirection) {
    if (newDirection == -1)
    {
        log("Robot does not need to turn");
        return;
    }
    log("continuing");
    int diff = (newDirection - robotDirection + 4) % 4;
    log("Turning robot from direction " + to_string(robotDirection) + " to direction " + to_string(newDirection));

    if (diff == 1) {  // Turn right once (90 degrees clockwise)
        API::turnRight();
        robotDirection = (robotDirection + 1) % 4;
    }
    else if (diff == 2) {  // Turn around (180 degrees)
        API::turnRight();
        API::turnRight();
        robotDirection = (robotDirection + 2) % 4;
    }
    else if (diff == 3) {  // Turn left once (90 degrees counterclockwise)
        API::turnLeft();
        robotDirection = (robotDirection + 3) % 4;
    }
    hasMoved = 1;
    log("Robot turned to direction " + to_string(robotDirection));
}

void moveRobot(int targetX, int targetY) {
    // We need to move the robot to the target cell at (targetX, targetY).
    // We need to calculate the direction from current position to target position
    int targetDirection = -1;
    if (targetX > robotX) targetDirection = 1; // Move Right
    else if (targetX < robotX) targetDirection = 3; // Move Left
    else if (targetY > robotY) targetDirection = 2; // Move Down
    else if (targetY < robotY) targetDirection = 0; // Move Up

    // Only turn the robot if it is not facing the target direction
    if (robotDirection != targetDirection) {
        log("Target direction is " + to_string(targetDirection));
        turnRobot(targetDirection);
    }

    if (hasMoved != 0){
        // Move the robot forward
        API::moveForward();
        log("Moved to (" + to_string(targetX) + ", " + to_string(targetY) + ")");
    }
    else
    {
        log("Moved to (" + to_string(targetX) + ", " + to_string(targetY) + ")");
    }
    // Update robot's position
    robotX = targetX;
    robotY = targetY;

    // After moving, explore the maze again to update walls
    exploreMaze();
    
}

struct CompareCells {
    bool operator()(const Cell& a, const Cell& b) const {
        return a.fCost > b.fCost;  // Min-heap priority queue, prioritize lower fCost
    }
};

vector<Cell> aStarSearch(int startX, int startY, int goalX, int goalY) {
    priority_queue<Cell, vector<Cell>, CompareCells> openList;
    set<pair<int, int>> closedList;
    vector<Cell> path;

    // Set up start cell
    Cell startCell;
    startCell.x = startX;
    startCell.y = startY;
    startCell.gCost = 0;
    startCell.hCost = calculateHeuristic(startX, startY, goalX, goalY);
    startCell.fCost = startCell.gCost + startCell.hCost;
    openList.push(startCell);

    while (!openList.empty()) {
        Cell currentCell = openList.top();
        openList.pop();

        if (currentCell.x == goalX && currentCell.y == goalY) {
            log("Goal reached at (" + to_string(goalX) + ", " + to_string(goalY) + ")");
            while (currentCell.parent != nullptr) {
                path.push_back(currentCell);
                currentCell = *currentCell.parent;
            }
            reverse(path.begin(), path.end());
            return path;
        }

        closedList.insert({currentCell.x, currentCell.y});
        log("Visiting cell (" + to_string(currentCell.x) + ", " + to_string(currentCell.y) + ")");

        // Move the robot to the current cell
        moveRobot(currentCell.x, currentCell.y);
        log("Called moveRobot");

        // Explore the neighbors
        for (int i = 0; i < 4; ++i) {
            int newX = currentCell.x + dx[i];
            int newY = currentCell.y + dy[i];

            if (newX < 0 || newX >= MAZE_SIZE || newY < 0 || newY >= MAZE_SIZE || maze[newX][newY].hasWall(i)) {
                continue;
            }

            if (closedList.find({newX, newY}) != closedList.end()) {
                continue;
            }

            int newGCost = currentCell.gCost + 1;
            int newHCost = calculateHeuristic(newX, newY, goalX, goalY);
            int newFCost = newGCost + newHCost;

            Cell neighbor;
            neighbor.x = newX;
            neighbor.y = newY;
            neighbor.gCost = newGCost;
            neighbor.hCost = newHCost;
            neighbor.fCost = newFCost;
            neighbor.parent = new Cell(currentCell);

            openList.push(neighbor);
        }
    }

    log("No path found.");
    return path;
}

int main() {
    // Example of starting the exploration
    log("Program started.");
    exploreMaze();  // Initially explore the maze
    vector<Cell> path = aStarSearch(0, 0, 7, 7);  // Goal at (7, 7)
    
    // Optionally, add movement along the path
    log("Path found with " + to_string(path.size()) + " cells.");
    for (const Cell& step : path) {
        // Calculate the direction the robot should be facing
        int targetDirection = -1;
        if (step.x > robotX) targetDirection = 1; // Move Right
        else if (step.x < robotX) targetDirection = 3; // Move Left
        else if (step.y > robotY) targetDirection = 2; // Move Down
        else if (step.y < robotY) targetDirection = 0; // Move Up

        // Turn robot towards the target direction
        if (robotDirection != targetDirection) {
            turnRobot(targetDirection);
        }

        // Move robot to the next step in the path
        moveRobot(step.x, step.y); // This will update the position and check for walls
        log("Moved to (" + to_string(step.x) + ", " + to_string(step.y) + ")");
    }

    return 0;
}
