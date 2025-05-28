#ifndef PATHFIND_H
#define PATHFIND_H

// Maximum grid dimensions
#define MAX_HEIGHT 30
#define MAX_WIDTH 20

#include <cstdint> // Used for uint8_t, uint16_t
#include <queue>   // priority_queue used for sorting nodes
#include <vector>
#include <cmath>   // Used for min(), max() and abs()
using namespace std;

// Possible states for each node in the grid 
enum State : uint8_t {
    CLOSED = 0,  // Node has been evaluated and closed
    OPEN = 1,    // Node is available for evaluation
    WALKABLE,    // Node is free to walk on
    START,       // Starting position for pathfinding
    FINISH,      // End position for pathfinding
    WALL,        // Node that cannot be traversed
    PATH         // Part of the final path
};

// Struct representing a 2D map position
typedef struct Position {
    uint8_t x;
    uint8_t y;
} pos;

// Class representing a single Node on the grid
class Node {
private:
    pos position;    // Coordinates of the node
    State state;     // State of the node
    Node* parent;    // Pointer to parent node for reconstructing the path
    uint16_t h, g, f; // A* pathfinding cost values: h - distance from end to this node, g - distance from start to this node, f = g + h

public:
    // Constructor with default position 
    Node(int x = 0, int y = 0);

    // Accessor methods
    State get_state() const;
    pos get_position() const;
    Node* get_parent() const;
    uint16_t get_f() const;
    uint16_t get_h() const;
    uint16_t get_g() const;

    // Checks if node is WALKABLE or FINISH
    // Used in algorithm
    bool isWalkable() const;

    // Calculates g, h and f costs
    // More detail on how this works is in pathfind.cpp
    void calculate_f(Node*, Node*);
    void calculate_g(Node*);
    void calculate_h(Node*);

    // Setter methods
    void set_parent(Node*);
    void set_open();
    void set_closed();
    void set_start();
    void set_finish();
    void set_wall();
    void set_path();
    void set_walkable();
};

// Class that stores nodes in a matrix
// Strictly declare as global object, because it uses around 12KB of memory and will cause stack overflow on ESP32 otherwise
// Max height and width exist to prevent stack overflow
class Grid {
private:
    Node map[MAX_HEIGHT][MAX_WIDTH];
    int width, height;

public:
    // Constructor that sets width and height, defaults to max values
    Grid(int w = MAX_WIDTH, int h = MAX_HEIGHT);

    bool serial_debugging(pos start, pos end, HardwareSerial& serial = Serial);
    void serial_print_grid(HardwareSerial& serial = Serial);

    // Statically creates the grid for memory optimization
    void construct_grid();

    // Used for debugging on PC
    // Also needs to be uncommented in pathfind.cpp
    // friend ostream& operator<<(ostream& os, const Grid& grid); @Only for PC debugging

    // Retrieves the node at x,y position from struct position
    // Returns nullptr if grid limits are exceeded
    Node* get_node(int, int);

    // Retrieves the nodes around a given node
    // It returns a vector since some nodes will be nullptr and therefore array size won't always be the same
    vector<Node*> get_neighbours(Node* node);
};

// A* pathfinding algorithm optimized for ESP32 use
// Returns a vector of positions {x, y}
vector<pos> pathfind(Grid&, pos, pos);

// Called in pathfind function for creating the path
vector<pos> reconstruct_path(Node*);

#endif
