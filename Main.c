#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <string.h>

#include "API.h"

#define min(a,b) ((a) < (b) ? (a) : (b))
#define MAZE_MAX_SIZE 16
#define INF 9999
#define NORTH 0
#define EAST 1
#define SOUTH 2
#define WEST 3

int orient = NORTH;

typedef struct Cell{
    int x, y;
}Cell;

/*
Queue Data Structure:
    - items: Array of Cells
    - front: Index of the front element
    - rear: Index of the rear element
*/

typedef struct CircularQueue {
    Cell items[MAZE_MAX_SIZE * MAZE_MAX_SIZE];
    int front, rear;
}cirque;

void initializeQueue(cirque *q) {
    q->front = -1;
    q->rear = -1;
}

bool isEmpty(cirque *q) {
    return q->front == -1;
}

bool isFull(cirque *q) {
    return (q->front == 0 && q->rear == MAZE_MAX_SIZE - 1) || (q->front == q->rear + 1);
}

void enqueue(cirque *q, Cell item) {
    if (isFull(q)) {
        return;
    }
    if (q->front == -1) {
        q->front = 0;
    }
    q->rear = (q->rear + 1) % MAZE_MAX_SIZE;
    q->items[q->rear] = item;
}

Cell dequeue(cirque *q) {
    Cell item;
    if (isEmpty(q)) {
        return item;
    }
    item = q->items[q->front];
    if (q->front == q->rear) {
        q->front = -1;
        q->rear = -1;
    } else {
        q->front = (q->front + 1) % MAZE_MAX_SIZE;
    }
    return item;
}
/*
Coordinates in Actual Array (i,j)

(0,12),(1,12),(2,12),(3,12),(4,12),(5,12),(6,12),(7,12),(8,12),(9,12),(10,12),(11,12),(12,12)
(0,11),(1,11),(2,11),(3,11),(4,11),(5,11),(6,11),(7,11),(8,11),(9,11),(10,11),(11,11),(12,11)
(0,10),(1,10),(2,10),(3,10),(4,10),(5,10),(6,10),(7,10),(8,10),(9,10),(10,10),(11,10),(12,10)
(0,9),(1,9),(2,9),(3,9),(4,9),(5,9),(6,9),(7,9),(8,9),(9,9),(10,9),(11,9),(12,9)
(0,8),(1,8),(2,8),(3,8),(4,8),(5,8),(6,8),(7,8),(8,8),(9,8),(10,8),(11,8),(12,8)
(0,7),(1,7),(2,7),(3,7),(4,7),(5,7),(6,7),(7,7),(8,7),(9,7),(10,7),(11,7),(12,7)
(0,6),(1,6),(2,6),(3,6),(4,6),(5,6),(6,6),(7,6),(8,6),(9,6),(10,6),(11,6),(12,6)
(0,5),(1,5),(2,5),(3,5),(4,5),(5,5),(6,5),(7,5),(8,5),(9,5),(10,5),(11,5),(12,5)
(0,4),(1,4),(2,4),(3,4),(4,4),(5,4),(6,4),(7,4),(8,4),(9,4),(10,4),(11,4),(12,4)
(0,3),(1,3),(2,3),(3,3),(4,3),(5,3),(6,3),(7,3),(8,3),(9,3),(10,3),(11,3),(12,3)
(0,2),(1,2),(2,2),(3,2),(4,2),(5,2),(6,2),(7,2),(8,2),(9,2),(10,2),(11,2),(12,2)
(0,1),(1,1),(2,1),(3,1),(4,1),(5,1),(6,1),(7,1),(8,1),(9,1),(10,1),(11,1),(12,1)
(0,0),(1,0),(2,0),(3,0),(4,0),(5,0),(6,0),(7,0),(8,0),(9,0),(10,0),(11,0),(12,0)

*/

int flood_vals[MAZE_MAX_SIZE][MAZE_MAX_SIZE] = {0};
int vertical_walls[MAZE_MAX_SIZE - 1][MAZE_MAX_SIZE] = {0};
int horizontal_walls[MAZE_MAX_SIZE][MAZE_MAX_SIZE - 1] = {0};
int visitedMaze[MAZE_MAX_SIZE][MAZE_MAX_SIZE] = {0};
cirque q;

void initializeFloodVals() { // Initialize the flood_vals array assuming there is no wall
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

}

void log_it(const char *string) {
    fprintf(stderr, "%s\n", string);
}

int isWall(Cell cell, int orient) {
    switch(orient) {
        case NORTH:
            return horizontal_walls[cell.x][cell.y];
        case EAST:
            return vertical_walls[cell.x][cell.y];
        case SOUTH:
            return horizontal_walls[cell.x][cell.y - 1];
        case WEST:
            return vertical_walls[cell.x - 1][cell.y];
        default:
            return 1;
    }
    
}
// vertical_walls[i][j] == 1 --> there is a wall between cell (i,j), (i+1,j) i.e. on the right side of cell (i,j)
// i.e. (i,j)|(i+1,j)

// horizontal_walls[i][j] == 1 --> there is a wall between cell (i,j) and (i,j+1) i.e. on the top side of cell (i,j)
// i.e. (i,j+1)
//       ______
//       (i,j)

void updateWalls(Cell start, int orient){
    switch(orient){
        case NORTH:
            if(API_wallFront()){
                horizontal_walls[start.x][start.y] = 1;
            }if(API_wallRight()){
                vertical_walls[start.x][start.y] = 1;
            }if(API_wallLeft()){
                if(start.x>0)
                    vertical_walls[start.x - 1][start.y] = 1;
            }break;
        case EAST:
            if(API_wallFront()){
                vertical_walls[start.x][start.y] = 1;
            }if(API_wallRight()){
                if(start.y>0)
                    horizontal_walls[start.x][start.y-1] = 1;
            }if(API_wallLeft()){
                horizontal_walls[start.x][start.y] = 1;
            }break;
        case SOUTH:
            if(API_wallFront()){
                if(start.y>0)
                    horizontal_walls[start.x][start.y-1] = 1;
            }if(API_wallRight()){
                if(start.x>0)
                    vertical_walls[start.x - 1][start.y] = 1;
            }if(API_wallLeft()){
                vertical_walls[start.x][start.y] = 1;
            }break;
        case WEST:
            if(API_wallFront()){
                if(start.x>0)
                    vertical_walls[start.x - 1][start.y] = 1;
            }if(API_wallRight()){
                horizontal_walls[start.x][start.y] = 1;
            }if(API_wallLeft()){
                if(start.y>0)
                    horizontal_walls[start.x][start.y-1] = 1;
            }break;
        default: break;
    }
    return;
}

void initializeWalls(){
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

    
}

Cell* getNeighbours(Cell current){ // Returns the neighbours of the current cell in the order North, East, South, West
    Cell* neighbours = (Cell*)malloc(4*sizeof(Cell));
    neighbours[0].x = current.x;
    neighbours[0].y = current.y + 1;

    neighbours[1].x = current.x + 1;
    neighbours[1].y = current.y;

    neighbours[2].x = current.x;
    neighbours[2].y = current.y - 1;

    neighbours[3].x = current.x - 1;
    neighbours[3].y = current.y;

    return neighbours;
}

void showText(){ // To Display the flood_vals array on the maze in simulator
    for(int i = 0; i < MAZE_MAX_SIZE; i++){
        for(int j = 0; j < MAZE_MAX_SIZE; j++){
            char text[10];
            sprintf(text, "%d", flood_vals[i][j]);
            API_setText(i, j, text);
        }
    }
}


int isDeadEnd(Cell current){
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
    return count == 1;
}

typedef struct {
    Cell prev;
    int dist;
} PathInfo;

PathInfo path_info[MAZE_MAX_SIZE][MAZE_MAX_SIZE];

void printShortestPathDebug(Cell start, Cell destination) {
    log_it("Shortest path info:");
    log_it("====================");
    log_it("Destination Cell:");
    fprintf(stderr, "x: %d, y: %d\n", destination.x, destination.y);
    log_it("Shortest Path:");

    Cell current = destination;
    while (!(current.x == start.x && current.y == start.y)) {
        fprintf(stderr, "(x: %d, y: %d)\n", current.x, current.y);
        current = path_info[current.x][current.y].prev;
    }
    fprintf(stderr, "(x: %d, y: %d)\n", current.x, current.y);

    log_it("====================");
}



void updateFloodVals(Cell current, Cell* destination_cells, int num_destinations) {
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
                    Cell* neighbours = getNeighbours(current);
                    for (int orient = 0; orient < 4; orient++) {
                        Cell neighbour = neighbours[orient];
                        if (neighbour.x >= 0 && neighbour.x < MAZE_MAX_SIZE &&
                            neighbour.y >= 0 && neighbour.y < MAZE_MAX_SIZE &&
                            !isWall(current, orient)) {
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
    char text[20];
    sprintf(text,"previous neighbour: %d %d", path_info[current.x][current.y].prev.x, path_info[current.x][current.y].prev.y);
    log_it(text);
}

Cell destination_cells[4];
int num_destinations = 0;

Cell start = {0, 0};

/// @brief  
Cell MazeSolver(Cell start, Cell* destination_cells, int num_destinations, char color){
    log_it("Starting the maze solver");
    initializeQueue(&q);
    initializeWalls();
    initializeFloodVals();
    showText();
    Cell current = start;
    API_setColor(current.x, current.y, 'R');
    if(MAZE_MAX_SIZE%2){
        API_setColor(MAZE_MAX_SIZE/2, MAZE_MAX_SIZE/2, 'B');
    }
    else{
        API_setColor(MAZE_MAX_SIZE/2, MAZE_MAX_SIZE/2, 'B');
        API_setColor(MAZE_MAX_SIZE/2 - 1, MAZE_MAX_SIZE/2, 'B');
        API_setColor(MAZE_MAX_SIZE/2, MAZE_MAX_SIZE/2 - 1, 'B');
        API_setColor(MAZE_MAX_SIZE/2 - 1, MAZE_MAX_SIZE/2 - 1, 'B');
    }
    
    updateWalls(current, orient);
    updateFloodVals(current, destination_cells, num_destinations);
    while(flood_vals[current.x][current.y] != 0){
        showText();
        API_setColor(current.x, current.y, color);
        Cell* neighbours= getNeighbours(current);
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
        updateWalls(current, orient);
        updateFloodVals(current,destination_cells, num_destinations);
    }
    return current;
}

int main(){
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
    Cell current = MazeSolver(start, destination_cells, num_destinations, 'G');
    
    // destination_cells[0] = start;
    // start = current;
    // char text[10];
    // sprintf(text, "%d", orient);
    // log_it(text);
    // num_destinations = 1;
    printShortestPathDebug(start, current);
    // // orient = (orient + 2) % 4;
    // current = MazeSolver(start, destination_cells, num_destinations, 'B');
    // API_turnRight();
    // API_turnRight();
    // while(current.x != MAZE_MAX_SIZE/2 || current.y != MAZE_MAX_SIZE/2){
    //     API_setColor(current.x, current.y, 'Y');
    //     Cell prev = path_info[current.x][current.y].prev;
    //     API_setColor(prev.x, prev.y, 'Y');
    //     current = prev;
    // }
    return 0;
}
