#define ENCA_M1 27 // Encoder pin A for Motor 1
#define ENCB_M1 26 // Encoder pin B for Motor 1
#define PWM_PIN_M1 15 // Motor 1 PWM pin
#define IN1_PIN_M1 0 // Motor 1 direction pin 1
#define IN2_PIN_M1 2 // Motor 1 direction pin 2

#define ENCA_M2 19 // Encoder pin A for Motor 2
#define ENCB_M2 21 // Encoder pin B for Motor 2
#define PWM_PIN_M2 17 // Motor 2 PWM pin
#define IN1_PIN_M2 4 // Motor 2 direction pin 1
#define IN2_PIN_M2 16 // Motor 2 direction pin 2

#define IR_SENSOR_1_PIN 33 // IR sensor 1 pin
#define IR_SENSOR_2_PIN 32 // IR sensor 2 pin
#define IR_SENSOR_3_PIN 23 // IR sensor 3 pin
#define IR_SENSOR_4_PIN 22 // IR sensor 4 pin

#define M1 1 // Motor 1 identifier
#define M2 2 // Motor 2 identifier

volatile int encoderPosM1 = 0; // Encoder position for Motor 1
volatile int encoderPosM2 = 0;

#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <string.h>

// #include "C:\Users\Hitesh\Desktop\Mickey_\API.h"

#define min(a,b) ((a) < (b) ? (a) : (b))
#define MAZE_MAX_SIZE 8
#define INF 9999
#define NORTH 0
#define EAST 1
#define SOUTH 2
#define WEST 3

int orient = NORTH;

void runMotor(int motor, int pwm) {
  if(pwm >= 0){
    if(motor == M1){
      digitalWrite(IN1_PIN_M1, LOW);
      digitalWrite(IN2_PIN_M1, HIGH);
      analogWrite(PWM_PIN_M1, pwm);
    }if (motor == M2) {
    digitalWrite(IN1_PIN_M2, HIGH);
    digitalWrite(IN2_PIN_M2, LOW);
    analogWrite(PWM_PIN_M2, pwm); // Adjust PWM value as needed for motor speed
    }
  }
  else if(pwm < 0){
    if(motor == M1){
      digitalWrite(IN1_PIN_M1, HIGH);
      digitalWrite(IN2_PIN_M1,LOW);
      analogWrite(PWM_PIN_M1, -pwm);
    }if(motor == M2){
      digitalWrite(IN1_PIN_M2, LOW);
      digitalWrite(IN2_PIN_M2,HIGH);
      analogWrite(PWM_PIN_M2, -pwm);
    }
  }
}


typedef struct Cell{
    int x, y;
}Cell;

/*
Queue Data Structure:
    - items: Array of Cells
    - front: Index of the front element
    - rear: Index of the rear element
*/
void API_moveForward(){
  runMotor(M1,100);
  runMotor(M2,100);
  delay(1000);
}

void API_turnLeft(){
  runMotor(M1,100);
  runMotor(M2,0);
  delay(1000);
}

void API_turnRight(){
  runMotor(M1,0);
  runMotor(M2,100);
  delay(1000);
}

bool API_wallFront(bool ir2, bool ir3){
  bool wallf = ir2 || ir3;
  return wallf;
}

bool API_wallLeft(bool ir1){
  return ir1;
}

bool API_wallRight(bool ir4){
  return ir4;
}

/*
Coordinates in Actual Array (i,j)

(0,8),(1,8),(2,8),(3,8),(4,8),(5,8),(6,8),(7,8),(8,8)
(0,7),(1,7),(2,7),(3,7),(4,7),(5,7),(6,7),(7,7),(8,7)
(0,6),(1,6),(2,6),(3,6),(4,6),(5,6),(6,6),(7,6),(8,6)
(0,5),(1,5),(2,5),(3,5),(4,5),(5,5),(6,5),(7,5),(8,5)
(0,4),(1,4),(2,4),(3,4),(4,4),(5,4),(6,4),(7,4),(8,4)
(0,3),(1,3),(2,3),(3,3),(4,3),(5,3),(6,3),(7,3),(8,3)
(0,2),(1,2),(2,2),(3,2),(4,2),(5,2),(6,2),(7,2),(8,2)
(0,1),(1,1),(2,1),(3,1),(4,1),(5,1),(6,1),(7,1),(8,1)
(0,0),(1,0),(2,0),(3,0),(4,0),(5,0),(6,0),(7,0),(8,0)

*/




typedef struct {
    Cell prev;
    int dist;
} PathInfo;

PathInfo path_info[MAZE_MAX_SIZE][MAZE_MAX_SIZE];

Cell destination_cells[4];
int num_destinations = 0;

Cell start = {0, 0};


void updateEncoderM1() {
  // This function updates the encoder position for Motor 1
  int encA = digitalRead(ENCA_M1);
  int encB = digitalRead(ENCB_M1);

  encoderPosM1 += (encA == encB) ? 1 : -1;
}

void updateEncoderM2() {
  // This function updates the encoder position for Motor 2
  int encA = digitalRead(ENCA_M2);
  int encB = digitalRead(ENCB_M2);

  encoderPosM2 += (encA == encB) ? 1 : -1;
}

int flood_vals[MAZE_MAX_SIZE][MAZE_MAX_SIZE] = {0};
int vertical_walls[MAZE_MAX_SIZE - 1][MAZE_MAX_SIZE] = {0};
int horizontal_walls[MAZE_MAX_SIZE][MAZE_MAX_SIZE - 1] = {0};
int visitedMaze[MAZE_MAX_SIZE][MAZE_MAX_SIZE] = {0};

Cell current = start;
bool isDeadEnd = 0; 

void setup() {
  Serial.begin(9600);

  // Set IR sensor pins as inputs
  pinMode(IR_SENSOR_1_PIN, INPUT);
  pinMode(IR_SENSOR_2_PIN, INPUT);
  pinMode(IR_SENSOR_3_PIN, INPUT);
  pinMode(IR_SENSOR_4_PIN, INPUT);

  // Set motor 1 control pins as outputs
  pinMode(PWM_PIN_M1, OUTPUT);
  pinMode(IN1_PIN_M1, OUTPUT);
  pinMode(IN2_PIN_M1, OUTPUT);

  // Set motor 2 control pins as outputs
  pinMode(PWM_PIN_M2, OUTPUT);
  pinMode(IN1_PIN_M2, OUTPUT);
  pinMode(IN2_PIN_M2, OUTPUT);

  // Attach encoder interrupts
  attachInterrupt(digitalPinToInterrupt(ENCA_M1), updateEncoderM1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCA_M2), updateEncoderM2, CHANGE);

  Cell start = {0, 0};
  if(MAZE_MAX_SIZE%2){
      destination_cells[num_destinations].x = MAZE_MAX_SIZE/2;
      destination_cells[num_destinations].y = MAZE_MAX_SIZE/2;
      num_destinations++;
  }
  else{
      destination_cells[num_destinations].x = MAZE_MAX_SIZE/2;
      destination_cells[num_destinations].y = MAZE_MAX_SIZE/2;
      num_destinations++;
      destination_cells[num_destinations].x = MAZE_MAX_SIZE/2 - 1;
      destination_cells[num_destinations].y = MAZE_MAX_SIZE/2;
      num_destinations++;
      destination_cells[num_destinations].x = MAZE_MAX_SIZE/2;
      destination_cells[num_destinations].y = MAZE_MAX_SIZE/2 - 1;
      num_destinations++;
      destination_cells[num_destinations].x = MAZE_MAX_SIZE/2 - 1;
      destination_cells[num_destinations].y = MAZE_MAX_SIZE/2 - 1;
      num_destinations++;
  }
  
    if(MAZE_MAX_SIZE % 2){
        for(int i = 0; i < MAZE_MAX_SIZE; i++) {
            for(int j = 0; j < MAZE_MAX_SIZE; j++) {
                flood_vals[i][j] = abs(MAZE_MAX_SIZE / 2 - i) + abs(MAZE_MAX_SIZE / 2 - j);
            }
        }
    }
    else{
        for(int i = 0; i < MAZE_MAX_SIZE; i++) {
            for(int j = 0; j < MAZE_MAX_SIZE; j++) {
                flood_vals[i][j] = abs(MAZE_MAX_SIZE / 2 - i) + abs(MAZE_MAX_SIZE / 2 - j);
                flood_vals[i][j] = min(flood_vals[i][j], abs(MAZE_MAX_SIZE / 2 - i) + abs(MAZE_MAX_SIZE / 2 - j - 1));
                flood_vals[i][j] = min(flood_vals[i][j], abs(MAZE_MAX_SIZE / 2 - i - 1) + abs(MAZE_MAX_SIZE / 2 - j));
                flood_vals[i][j] = min(flood_vals[i][j], abs(MAZE_MAX_SIZE / 2 - i - 1) + abs(MAZE_MAX_SIZE / 2 - j - 1));
            }
        }
    }
    for(int i = 0; i < MAZE_MAX_SIZE; i++){
        for(int j = 0; j < MAZE_MAX_SIZE - 1; j++){
            vertical_walls[i][j] = 0;
        }
    }
    for(int i = 0; i < MAZE_MAX_SIZE - 1; i++){
        for(int j = 0; j < MAZE_MAX_SIZE; j++){
            horizontal_walls[i][j] = 0;
        }
    }
    // showText();
 

    // API_setColor(current.x, current.y, 'R');  //For simulation
    // if(MAZE_MAX_SIZE%2){
    //     API_setColor(MAZE_MAX_SIZE/2, MAZE_MAX_SIZE/2, 'B');
    // }
    // else{
    //     API_setColor(MAZE_MAX_SIZE/2, MAZE_MAX_SIZE/2, 'B');
    //     API_setColor(MAZE_MAX_SIZE/2 - 1, MAZE_MAX_SIZE/2, 'B');
    //     API_setColor(MAZE_MAX_SIZE/2, MAZE_MAX_SIZE/2 - 1, 'B');
    //     API_setColor(MAZE_MAX_SIZE/2 - 1, MAZE_MAX_SIZE/2 - 1, 'B');
}
    
void loop(){

  bool ir1 = !digitalRead(IR_SENSOR_1_PIN);
  bool ir2 = !digitalRead(IR_SENSOR_2_PIN);
  bool ir3 = !digitalRead(IR_SENSOR_3_PIN);
  bool ir4 = !digitalRead(IR_SENSOR_4_PIN);
  int count = 0;
  int reachable[4];
  reachable[0] = (current.y<MAZE_MAX_SIZE-1)?!horizontal_walls[current.x][current.y]:0;
  reachable[1] = (current.x<MAZE_MAX_SIZE-1)?!vertical_walls[current.x][current.y]:0;
  reachable[2] = (current.y>0)?!horizontal_walls[current.x][current.y - 1]:0;
  reachable[3] = (current.x>0)?!vertical_walls[current.x - 1][current.y]:0;
  for(int i = 0; i < 4; i++){
      if(reachable[i]){
          count++;
      }
  }
  isDeadEnd = (count == 1);
  switch(orient){
        case NORTH:
            if(API_wallFront(ir2,ir3)){
                horizontal_walls[start.x][start.y] = 1;
            }if(API_wallRight(ir4)){
                vertical_walls[start.x][start.y] = 1;
            }if(API_wallLeft(ir1)){
                if(start.x>0)
                    vertical_walls[start.x - 1][start.y] = 1;
            }break;
        case EAST:
            if(API_wallFront(ir2,ir3)){
                vertical_walls[start.x][start.y] = 1;
            }if(API_wallRight(ir4)){
                if(start.y>0)
                    horizontal_walls[start.x][start.y-1] = 1;
            }if(API_wallLeft(ir1)){
                horizontal_walls[start.x][start.y] = 1;
            }break;
        case SOUTH:
            if(API_wallFront(ir2,ir3)){
                if(start.y>0)
                    horizontal_walls[start.x][start.y-1] = 1;
            }if(API_wallRight(ir4)){
                if(start.x>0)
                    vertical_walls[start.x - 1][start.y] = 1;
            }if(API_wallLeft(ir1)){
                vertical_walls[start.x][start.y] = 1;
            }break;
        case WEST:
            if(API_wallFront(ir2,ir3)){
                if(start.x>0)
                    vertical_walls[start.x - 1][start.y] = 1;
            }if(API_wallRight(ir4)){
                horizontal_walls[start.x][start.y] = 1;
            }if(API_wallLeft(ir1)){
                if(start.y>0)
                    horizontal_walls[start.x][start.y-1] = 1;
            }break;
        default: break;
    }
    if(current.x == destination_cells[0].x && current.y == destination_cells[0].y && flood_vals[current.x][current.y] == flood_vals[destination_cells[0].x][destination_cells[0].y]){
        flood_vals[current.x][current.y] = 0;
        return;
    }
    for (int i = 0; i < MAZE_MAX_SIZE; i++) {
        for (int j = 0; j < MAZE_MAX_SIZE; j++) {
            flood_vals[i][j] = INF;
        }
    }

    for (int d = 0; d < num_destinations; d++) {
        Cell dest_cell = destination_cells[d];
        flood_vals[dest_cell.x][dest_cell.y] = 0;
    }

    // Perform flood fill algorithm
    bool changed = true;
    while (changed) {
        changed = false;
        for (int i = 0; i < MAZE_MAX_SIZE; i++) {
            for (int j = 0; j < MAZE_MAX_SIZE; j++) {
                if (flood_vals[i][j] != INF) {
                    Cell current = {i, j};
                    Cell* neighbours = (Cell*)malloc(4*sizeof(Cell));
                    neighbours[0].x = current.x;
                    neighbours[0].y = current.y + 1;

                    neighbours[1].x = current.x + 1;
                    neighbours[1].y = current.y;

                    neighbours[2].x = current.x;
                    neighbours[2].y = current.y - 1;

                    neighbours[3].x = current.x - 1;
                    neighbours[3].y = current.y;
;
                    for (int orient = 0; orient < 4; orient++) {
                        Cell neighbour = neighbours[orient];
                        bool isWall;
                        switch(orient) {
                            case NORTH:
                               isWall = horizontal_walls[current.x][current.y];
                            case EAST:
                               isWall = vertical_walls[current.x][current.y];
                            case SOUTH:
                               isWall = horizontal_walls[current.x][current.y - 1];
                            case WEST:
                               isWall = vertical_walls[current.x - 1][current.y];
                            default:
                               isWall = 1;
                        }
                        if (neighbour.x >= 0 && neighbour.x < MAZE_MAX_SIZE &&
                            neighbour.y >= 0 && neighbour.y < MAZE_MAX_SIZE &&
                            !isWall) {
                            int new_dist = flood_vals[current.x][current.y] + 1;
                            if (new_dist < flood_vals[neighbour.x][neighbour.y]) {
                                flood_vals[neighbour.x][neighbour.y] = new_dist;
                                path_info[neighbour.x][neighbour.y].prev = current; // Update previous cell
                                path_info[neighbour.x][neighbour.y].dist = new_dist;
                                changed = true;
                            }
                        }
                    }
                    free(neighbours); // Free the allocated memory for neighbours
                }
            }
        }
    }
    while(flood_vals[current.x][current.y] != 0){
        // showText();
        // API_setColor(current.x, current.y, color);
            Cell* neighbours = (Cell*)malloc(4*sizeof(Cell));
            neighbours[0].x = current.x;
            neighbours[0].y = current.y + 1;

            neighbours[1].x = current.x + 1;
            neighbours[1].y = current.y;

            neighbours[2].x = current.x;
            neighbours[2].y = current.y - 1;

            neighbours[3].x = current.x - 1;
            neighbours[3].y = current.y;

        bool all_greater = true;
        int min_flood_val = flood_vals[current.x][current.y];
        visitedMaze[current.x][current.y] = 1;
        int dir_to_move = -1;
        int* reachable = (int*)malloc(4*sizeof(int));
        reachable[0] = (current.y<MAZE_MAX_SIZE-1)?!horizontal_walls[current.x][current.y]:0;
        reachable[1] = (current.x<MAZE_MAX_SIZE-1)?!vertical_walls[current.x][current.y]:0;
        reachable[2] = (current.y>0)?!horizontal_walls[current.x][current.y - 1]:0;
        reachable[3] = (current.x>0)?!vertical_walls[current.x - 1][current.y]:0;
        for(int i = 0; i < 4; i++){
            if(reachable[i]){
                if(flood_vals[neighbours[i].x][neighbours[i].y] < flood_vals[current.x][current.y]){
                    all_greater = false;
                    if(flood_vals[neighbours[i].x][neighbours[i].y] < min_flood_val){
                        min_flood_val = flood_vals[neighbours[i].x][neighbours[i].y];
                        dir_to_move = i;
                    }
                }
            }
        }
        switch(orient){
            case NORTH:
                switch(dir_to_move)
                {
                    case NORTH: API_moveForward(); current.y++; break;
                    case EAST: API_turnRight(); orient = EAST; API_moveForward(); current.x++; break;
                    case WEST: API_turnLeft(); orient = WEST; API_moveForward(); current.x--; break;
                    case SOUTH: API_turnRight(); API_turnRight(); orient = SOUTH; API_moveForward(); current.y--; break;
                    default: break;
                }break;
            case EAST:
                switch(dir_to_move)
                {
                    case EAST: API_moveForward(); current.x++; break;
                    case SOUTH: API_turnRight(); orient = SOUTH; API_moveForward(); current.y--; break;
                    case NORTH: API_turnLeft(); orient = NORTH; API_moveForward(); current.y++; break;
                    case WEST: API_turnRight(); API_turnRight(); orient = WEST; API_moveForward(); current.x--; break;
                    default: break;
                }break;
            case SOUTH:
                switch(dir_to_move)
                {
                    case SOUTH: API_moveForward(); current.y--; break;
                    case WEST: API_turnRight(); orient = WEST; API_moveForward(); current.x--; break;
                    case EAST: API_turnLeft(); orient = EAST; API_moveForward(); current.x++; break;
                    case NORTH: API_turnRight(); API_turnRight(); orient = NORTH; API_moveForward(); current.y++; break;
                    default: break;
                }break;
            case WEST:
                switch(dir_to_move)
                {
                    case WEST: API_moveForward(); current.x--; break;
                    case NORTH: API_turnRight(); orient = NORTH; API_moveForward(); current.y++; break;
                    case SOUTH: API_turnLeft(); orient = SOUTH; API_moveForward(); current.y--; break;
                    case EAST: API_turnRight(); API_turnRight(); orient = EAST; API_moveForward(); current.x++; break;
                    default: break;
                }break;
            default: break;
        }   
        switch(orient){
        case NORTH:
            if(API_wallFront(ir2,ir3)){
                horizontal_walls[start.x][start.y] = 1;
            }if(API_wallRight(ir4)){
                vertical_walls[start.x][start.y] = 1;
            }if(API_wallLeft(ir1)){
                if(start.x>0)
                    vertical_walls[start.x - 1][start.y] = 1;
            }break;
        case EAST:
            if(API_wallFront(ir2,ir3)){
                vertical_walls[start.x][start.y] = 1;
            }if(API_wallRight(ir4)){
                if(start.y>0)
                    horizontal_walls[start.x][start.y-1] = 1;
            }if(API_wallLeft(ir1)){
                horizontal_walls[start.x][start.y] = 1;
            }break;
        case SOUTH:
            if(API_wallFront(ir2,ir3)){
                if(start.y>0)
                    horizontal_walls[start.x][start.y-1] = 1;
            }if(API_wallRight(ir4)){
                if(start.x>0)
                    vertical_walls[start.x - 1][start.y] = 1;
            }if(API_wallLeft(ir1)){
                vertical_walls[start.x][start.y] = 1;
            }break;
        case WEST:
            if(API_wallFront(ir2,ir3)){
                if(start.x>0)
                    vertical_walls[start.x - 1][start.y] = 1;
            }if(API_wallRight(ir4)){
                horizontal_walls[start.x][start.y] = 1;
            }if(API_wallLeft(ir1)){
                if(start.y>0)
                    horizontal_walls[start.x][start.y-1] = 1;
            }break;
        default: break;
    }
        if(current.x == destination_cells[0].x && current.y == destination_cells[0].y && flood_vals[current.x][current.y] == flood_vals[destination_cells[0].x][destination_cells[0].y]){
        flood_vals[current.x][current.y] = 0;
        return;
    }
    for (int i = 0; i < MAZE_MAX_SIZE; i++) {
        for (int j = 0; j < MAZE_MAX_SIZE; j++) {
            flood_vals[i][j] = INF;
        }
    }

    for (int d = 0; d < num_destinations; d++) {
        Cell dest_cell = destination_cells[d];
        flood_vals[dest_cell.x][dest_cell.y] = 0;
    }

    // Perform flood fill algorithm
    bool changed = true;
    while (changed) {
        changed = false;
        for (int i = 0; i < MAZE_MAX_SIZE; i++) {
            for (int j = 0; j < MAZE_MAX_SIZE; j++) {
                if (flood_vals[i][j] != INF) {
                    Cell current = {i, j};
                    Cell* neighbours = (Cell*)malloc(4*sizeof(Cell));
                    neighbours[0].x = current.x;
                    neighbours[0].y = current.y + 1;

                    neighbours[1].x = current.x + 1;
                    neighbours[1].y = current.y;

                    neighbours[2].x = current.x;
                    neighbours[2].y = current.y - 1;

                    neighbours[3].x = current.x - 1;
                    neighbours[3].y = current.y;

                    for (int orient = 0; orient < 4; orient++) {
                        Cell neighbour = neighbours[orient];
                        bool isWall;
                        switch(orient) {
                            case NORTH:
                               isWall = horizontal_walls[current.x][current.y];
                            case EAST:
                               isWall = vertical_walls[current.x][current.y];
                            case SOUTH:
                               isWall = horizontal_walls[current.x][current.y - 1];
                            case WEST:
                               isWall = vertical_walls[current.x - 1][current.y];
                            default:
                               isWall = 1;
                        }
                        if (neighbour.x >= 0 && neighbour.x < MAZE_MAX_SIZE &&
                            neighbour.y >= 0 && neighbour.y < MAZE_MAX_SIZE &&
                            !isWall) {
                            int new_dist = flood_vals[current.x][current.y] + 1;
                            if (new_dist < flood_vals[neighbour.x][neighbour.y]) {
                                flood_vals[neighbour.x][neighbour.y] = new_dist;
                                path_info[neighbour.x][neighbour.y].prev = current; // Update previous cell
                                path_info[neighbour.x][neighbour.y].dist = new_dist;
                                changed = true;
                            }
                        }
                    }
                    free(neighbours); // Free the allocated memory for neighbours
                }
            }
        }
    }
    }
}


