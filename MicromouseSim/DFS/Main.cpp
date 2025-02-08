#include <iostream>
#include <vector>
#include <stack>
#include <queue>
#include <cmath>
using namespace std;
#include "API.h"
#define MAZE_SIZE 16

class Cell {
public:
    int x, y;
    bool visited;
    bool wallUp, wallRight, wallDown, wallLeft; // Use booleans for walls

    // Default constructor
    Cell() : x(0), y(0), visited(false), wallUp(false), wallRight(false), wallDown(false), wallLeft(false) {}

    // Constructor with coordinates
    Cell(int x, int y) : x(x), y(y), visited(false), wallUp(false), wallRight(false), wallDown(false), wallLeft(false) {}

    // Function to check if there's a wall in a specific direction
    bool hasWall(int direction) const {
        switch (direction) {
            case 0: return wallUp;   // Up direction
            case 1: return wallRight; // Right direction
            case 2: return wallDown;  // Down direction
            case 3: return wallLeft;  // Left direction
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

int robotX = 0, robotY = 0;
int robotDirection = 0;  // Direction the robot is facing (0 = Up, 1 = Right, 2 = Down, 3 = Left)

vector<vector<Cell>> maze(MAZE_SIZE, vector<Cell>(MAZE_SIZE));

const int dx[] = { 0,  0, -1,  1};  // x changes for Up, Down, Left, Right
const int dy[] = {-1,  1,  0,  0};  // y changes for Up, Down, Left, Right


void log(const std::string& text) {
    std::cerr << text << std::endl;
}

// Function to explore the maze and update the walls of the current cell
void exploreMaze() {
    // Determine walls based on the robot's current direction
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
// DFS to explore the maze and reach goals
// DFS to explore the maze and reach goals
void DFS(vector<pair<int, int>>& goals) {
    stack<pair<int, int>> stack;
    stack.push({robotX, robotY});
    maze[robotX][robotY].visited = true;

    while (!stack.empty()) {
        auto current = stack.top();
        robotX = current.first;
        robotY = current.second;

        for (auto& goal : goals) {
            if (robotX == goal.first && robotY == goal.second) {
                log("Goal reached at (" + to_string(robotX) + ", " + to_string(robotY) + ")!");
                return;
            }
        }

        exploreMaze();
        bool moved = false;

        // Try moving in priority order: Front, Right, Left
        for (int i = 0; i < 3; ++i) {
            int newX = robotX;
            int newY = robotY;

            if (i == 0) { // Move front
                if (robotDirection == 0) newY++;
                else if (robotDirection == 1) newX++;
                else if (robotDirection == 2) newY--;
                else if (robotDirection == 3) newX--;
            } 
            else if (i == 1) { // Move right
                if (robotDirection == 0) newX++;
                else if (robotDirection == 1) newY--;
                else if (robotDirection == 2) newX--;
                else if (robotDirection == 3) newY++;
            } 
            else if (i == 2) { // Move left
                if (robotDirection == 0) newX--;
                else if (robotDirection == 1) newY++;
                else if (robotDirection == 2) newX++;
                else if (robotDirection == 3) newY--;
            }

            if (newX >= 0 && newX < MAZE_SIZE && newY >= 0 && newY < MAZE_SIZE && !maze[newX][newY].visited) {
                bool canMove = false;

                if (i == 0 && !API::wallFront()) canMove = true;
                else if (i == 1 && !API::wallRight()) canMove = true;
                else if (i == 2 && !API::wallLeft()) canMove = true;

                if (canMove) {
                    if (i == 1) {
                        API::turnRight();
                        robotDirection = (robotDirection + 1) % 4;
                        log("Turned right. Now facing: " + to_string(robotDirection));
                    } 
                    else if (i == 2) {
                        API::turnLeft();
                        robotDirection = (robotDirection + 3) % 4;
                        log("Turned left. Now facing: " + to_string(robotDirection));
                    }

                    API::moveForward();
                    log("Moved to (" + to_string(newX) + ", " + to_string(newY) + ")");
                    maze[newX][newY].visited = true;
                    stack.push({newX, newY});
                    moved = true;
                    break;
                }
            }
        }

        if (!moved) { 
            log("Dead end reached at (" + to_string(robotX) + ", " + to_string(robotY) + "), backtracking...");

            stack.pop();  // Remove the current dead-end cell from stack

            if (!stack.empty()) {  
                auto backtrackTarget = stack.top();
                int backX = backtrackTarget.first;
                int backY = backtrackTarget.second;

                int dx = backX - robotX;
                int dy = backY - robotY;

                int newDirection = robotDirection;

                if (dx == 1) newDirection = 1; // Right
                else if (dx == -1) newDirection = 3; // Left
                else if (dy == 1) newDirection = 0; // Up
                else if (dy == -1) newDirection = 2; // Down

                while (robotDirection != newDirection) {
                    API::turnRight();
                    robotDirection = (robotDirection + 1) % 4;
                }

                API::moveForward();
                log("Backtracked to (" + to_string(backX) + ", " + to_string(backY) + ")");
            } 
            else {
                log("No moves left, DFS complete.");
            }
        }
    }
}
int main() {
    // Set goals (in this case, we place 4 goals in the maze)
    vector<pair<int, int>> goals = {{7, 7}, {7, 8}, {8, 7}, {8, 8}};
    // Call DFS to explore and search for the goals
    DFS(goals);

    return 0;
}
