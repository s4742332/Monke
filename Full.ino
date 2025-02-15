// Libraries
#include <Stepper.h>

// Set-up Motors
  const int STEPS = 2048; // Full revolution for the 28BYJ-48 motor in full-step mode
  const int MaxSpeed = 255; 
  const int Stop = 0;
  Stepper Leftmotor(STEPS, 8, 9, 10, 11);
  Stepper Rightmotor(STEPS, 4, 5, 6, 7);

// Ultrasonic Set-up
  const int trigPin = 9; 
  const float echo[3] = {10, 11, 12}; // L, R, F echo pins
  float duration[3], distance[3]; // L, R, F duration and distance

//Indicators Set-up
  const float Lights[3] = {3, 4, 5}; //Pins for Lights, L, R, F

//Walls
  const int Walld = 5; //distance (cm) in which a Wall is definitely in front/surrounding the mouse
  float Walls[3]; //L, R, F in that order, boolean inside 

//Set-Up Algorithim
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
// Maze Set-p
void setup() {
  // Set the motor speed (RPM)
  Leftmotor.setSpeed(10);  // Set a lower speed to reduce RPM (adjust to your needs)
  Rightmotor.setSpeed(10);  // Set a lower speed to reduce RPM (adjust to your needs)

  //Set-up pinmodes
  for (byte i = 0; i=2; i++) {
    pinMode(Lights[i], OUTPUT); //set L, R, F indicators
    pinMode(echo[i], INPUT); //set L, R, F ultrasonic echopins
  }
  pinMode(trigPin, OUTPUT); // Trigger pin set-up
  Serial.begin(9600); //begin terminal
}

void loop() {
// Rotate the motor one direction
  Leftmotor.step(STEPS);  // Rotate 1 full revolution
  Rightmotor.step(STEPS);  // Rotate 1 full revolution

// Get Sensor reading
  digitalWrite(trigPin, LOW); //Clear the signal
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

// Durations
  duration[0] = pulseIn(echo[0], HIGH);
  duration[1] = pulseIn(echo[1], HIGH);
  duration[2] = pulseIn(echo[2], HIGH);

// Calc distance
  distance[0] = (duration[0]*.0343)/2;
  distance[1] = (duration[1]*.0343)/2;
  distance[2] = (duration[2]*.0343)/2;

//Print Results to terminal
  Serial.println("Left: ");  Serial.print(distance[0]);
  Serial.println("Right: "); Serial.print(distance[1]);
  Serial.println("Front: "); Serial.print(distance[2]);

//Indicate if Wall
  for (byte i = 0; i =3; i++) { // for each ultrasonic: L, R, F
    if (distance[i] <= Walld) { // check if the the distance is close to us or not
      analogWrite(Lights[i], HIGH); // Associated indicator light 
      Walls[i] = true; //Wall is there
      Serial.println(Walls[i]);
    }
    else {
      analogWrite(Lights[i], LOW);
      Walls[i] = false; // No Wall
      Serial.println(Walls[i]);
    }
  } 

//Decide directino to spin
if (Walls[0] == true && Walls[1] == true && Walls[2] == false) { //No Wall in front 
  Leftmotor.step(STEPS);
  Rightmotor.step(STEPS);
  Serial.println("Can go forward");
}
else if (Walls[0] == true && Walls[1] == false && Walls[2] == false) {Serial.println("Can go forward, right");}
else if (Walls[0] == false && Walls[1] == true && Walls[2] == false) {Serial.println("Can go forward, left");}
else if (Walls[0] == false && Walls[1] == false && Walls[2] == false) {Serial.println("Can go forward, left, right");}
else if (Walls[0] == false && Walls[1] == false && Walls[2] == true) {Serial.println("Can go left, right");}
else if (Walls[0] == true && Walls[1] == true && Walls[2] == true) {Serial.println("Can only go backkward");}
  
}
