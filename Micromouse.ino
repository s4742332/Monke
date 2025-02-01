#include <Stepper.h>
#define MAZE_SIZE 9  // Define maze dimensions

// Directions: North, East, South, West (dx, dy)
int dx[4] = {-1, 0, 1, 0}; // x-offsets (North, East, South, West)
int dy[4] = {0, 1, 0, -1}; // y-offsets (North, East, South, West)

struct Point {
  int x, y;
};

class Cell {
  public:
    bool northWall, eastWall, southWall, westWall;
    bool visited;

    // Constructor: Initializes all walls and sets visited to false
    Cell() {
      northWall = eastWall = southWall = westWall = true;
      visited = false;
    }

    // Remove wall in a specific direction
    void removeWall(char direction) {
      switch (direction) {
        case 'N': northWall = false; break;
        case 'E': eastWall = false; break;
        case 'S': southWall = false; break;
        case 'W': westWall = false; break;
      }
    }

    // Mark the cell as visited
    void markVisited() {
      visited = true;
    }
};

class Maze {
  private:
    Cell grid[MAZE_SIZE][MAZE_SIZE]; // 2D array of cells
    int robotX, robotY;  // Robot's current position

  public:
    // Constructor: Initialize robot and set initial position
    Maze() {
      robotX = 0;
      robotY = 0;
      grid[robotX][robotY].markVisited();  // Mark starting cell as visited
    }

    // Check if a move to a new cell is valid (not out of bounds and not blocked by walls)
    bool isValidMove(int x1, int y1, int x2, int y2) {
      if (x2 < 0 || x2 >= MAZE_SIZE || y2 < 0 || y2 >= MAZE_SIZE) return false; // Out of bounds
      if (grid[x2][y2].visited) return false; // Already visited

      // Check for walls between adjacent cells
      if (x1 == x2) {
        return (y1 < y2) ? !grid[x1][y1].eastWall : !grid[x1][y1].westWall;
      } else if (y1 == y2) {
        return (x1 < x2) ? !grid[x1][y1].southWall : !grid[x1][y1].northWall;
      }
      return false;  // Not adjacent
    }

    // Depth-First Search (DFS) Algorithm to find an exit
    bool dfs(int x, int y) {
      // Mark the current cell as visited
      grid[x][y].visited = true;

      // Check if the current cell is an exit (on the boundary with an open side)
      if ((x == 0 && !grid[x][y].northWall) || 
          (y == 0 && !grid[x][y].westWall) || 
          (x == MAZE_SIZE - 1 && !grid[x][y].southWall) || 
          (y == MAZE_SIZE - 1 && !grid[x][y].eastWall)) {
        Serial.print("Exit found at: ");
        Serial.print(x);
        Serial.print(", ");
        Serial.println(y);
        return true; // Exit found, return success
      }

      // Explore all four directions (North, East, South, West)
      for (int i = 0; i < 4; i++) {
        int nx = x + dx[i];
        int ny = y + dy[i];

        if (isValidMove(x, y, nx, ny)) {  // If the move is valid
          Serial.print("Moving to: ");
          Serial.print(nx);
          Serial.print(", ");
          Serial.println(ny);

          if (dfs(nx, ny)) {  // Recursively search further
            return true;  // If exit is found, return true
          }
        }
      }

      // If no valid moves, backtrack
      Serial.print("Backtracking from: ");
      Serial.print(x);
      Serial.print(", ");
      Serial.println(y);
      return false; // No exit found along this path
    }

    // Start solving the maze with DFS from the robot's starting position
    void solveMaze() {
      Serial.println("Starting DFS search for an exit...");
      if (!dfs(robotX, robotY)) {
        Serial.println("No exit found.");
      }
    }
};
// Define the number of steps per revolution for the 28BYJ-48 motor
const int stepsPerRevolution = 2038;  // Full revolution for the 28BYJ-48 motor in full-step mode

// Define the motor connections
const int motorPin1 = 8;  
const int motorPin2 = 9;  
const int motorPin3 = 10; 
const int motorPin4 = 11; 

// Create a Stepper object
Stepper myStepper(stepsPerRevolution, motorPin1, motorPin2, motorPin3, motorPin4);

void setup() {
  // Set the motor speed (RPM)
  myStepper.setSpeed(10);  // Set a lower speed to reduce RPM (adjust to your needs)
}

void loop() {
  // Rotate the motor one direction
  myStepper.step(stepsPerRevolution);  // Rotate 1 full revolution
}
