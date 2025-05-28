#include "pathfind.h"


Node::Node(int x, int y){
    position.x = x;
    position.y = y;
    state = WALKABLE;
    parent = nullptr;
    h = 0; g = 0; f = 0;
}

State Node::get_state() const{return state;}
pos Node::get_position() const{return {position.x, position.y};}
Node* Node::get_parent() const{return parent;}
uint16_t Node::get_f() const{return f;}
uint16_t Node::get_h() const{return h;}
uint16_t Node::get_g() const{return g;}

bool Node::isWalkable() const{
    if(state == WALKABLE || state == FINISH){
        return true;
    }
    return false;
}

void Node::calculate_g(Node *start){
    uint16_t dx = abs(position.x - start->get_position().x);    // Octile calculation of g and h cost is much faster than calculating hypotenuse from
    uint16_t dy = abs(position.y - start->get_position().y);    // node to start or finish.
    g = 10 * (dx + dy) - 6 * min(dx, dy);                       // It takes that distance between nodes is D=10 and diagonaly that it is D2=14
}                                                               // This way we avoid floating points (1.41) and the use of sqrt() 

void Node::calculate_h(Node *end){                              // Formula is: D*(dx+dy) + (D2 - 2*D) * min(dx, dy)
    uint16_t dx = abs(position.x - end->get_position().x);      // min(dx, dy) is maximal distance we can walk diagonaly
    uint16_t dy = abs(position.y - end->get_position().y);      // D*(dx+dy) would be the total distance if we walked without diagonales
    h = 10 * (dx + dy) - 6 * min(dx, dy);                       // More memory and procces efficient
}

void Node::calculate_f(Node *start, Node *end){
    calculate_g(start);
    calculate_h(end);
    f = g + h;
}

void Node::set_parent(Node *a){parent = a;}
void Node::set_open(){state = OPEN;}
void Node::set_closed(){state = CLOSED;}
void Node::set_start(){state = START;}
void Node::set_finish(){state = FINISH;}
void Node::set_wall(){state = WALL;}
void Node::set_path(){state = PATH;}
void Node::set_walkable(){state = WALKABLE;}


void Grid::construct_grid(){
    for(int y = 0; y < height; y++){
        for(int x = 0; x < width; x++){
            map[y][x] = Node(x, y);
        }
    }
}

Grid::Grid(int w, int h){           // Makes sure that maximum height and width is not execeded
    if (w > MAX_WIDTH){
        width = MAX_WIDTH;
    }else{
        width = w;
    }

    if(h > MAX_HEIGHT){
        height = MAX_HEIGHT;
    }else{
        height = h;
    }
    Grid::construct_grid();
}

/*ostream& operator<<(ostream& os, const Grid &grid){
    os << "LEGEND:" << endl << endl;
    os << "\033[31mS - start position" << endl;
    os << "F - final position" << endl;
    os << "0 - open node" << endl;
    os << "X - checked node" << endl;
    os << "# - wall" << endl;
    os << "P - PATH" << endl;
    os << "? - error in creating the map\033[32m" << endl << endl;

    cout << "   0  1  2  3  4  5  6  7  8  9  10 11 12 13 14 15 16 17 18 19" << endl;

    for(int i = 0; i < grid.height; i++){
        if(i < 10){
            cout << i << "  ";
        }else{
            cout << i << " ";
        }
        for(int j = 0; j < grid.width; j++){
            switch(grid.map[i][j].get_state()){
                case START: os << "\033[33mS  \033[32m"; break;
                case FINISH: os << "\033[35mF  \033[32m"; break;
                case WALKABLE: os << "\033[34mW  \033[32m"; break;
                case OPEN: os << "\033[97m0  \033[32m"; break;
                case CLOSED: os << "\033[31mX  \033[32m"; break;
                case WALL: os << "\033[30m#  \033[32m"; break;
                case PATH: os << "\033[92mP  \033[32m"; break;
                default: os << "\033[91m?  \033[32m"; break;
            }
        }
        os << endl;
    }

    os << "\033[0m" << endl;

    return os;
}*/

bool Grid::serial_debugging(pos start, pos end, HardwareSerial& serial){
    if(!serial){
        return false;
    }

    serial.println("----------------------------------");
    serial.println(" Debugging grid for pathfinding!");
    serial.println("----------------------------------");
    serial.println("\nLegend:");
    serial.println("S - starting node");
    serial.println("F - end node");
    serial.println("# - wall");
    serial.println("W - walkable node");
    serial.println("P - path node");
    serial.println("X - closed node after evaluation");
    serial.println("O - open node for evaluation");
    serial.println("\n\n");

    serial_print_grid(serial);
    
    Node *start_node = get_node(start.x, start.y);
    Node *end_node = get_node(end.x, end.y);

    serial.println("[INFO] Exploring the path.\n\n");

    unsigned int start_time = millis();
    vector<pos> path = pathfind(*this, start_node->get_position(), end_node->get_position());
    unsigned int duration = millis() - start_time;

    serial_print_grid(serial);

    if(path.empty()){
        serial.println("[ERROR] Couldn't find the path\n\n");

        serial.println("----------------------------------");
        serial.println(" End of debugging grid");
        serial.println("----------------------------------\n\n");

        return false;
    }

    serial.println("[OK] Found the path\n");
    serial.print("[OK] Time to find the path: ");
    serial.print(duration); serial.println("milliseconds");
    serial.print("[INFO] Size of grid in bytes: ");
    serial.println(sizeof(Grid));
    serial.println("\nPath(x,y):");

    for(int i = 0; i < path.size(); i++){
        serial.print("Node "); serial.print(i+1);
        serial.print(" (");serial.print(path[i].x);
        serial.print(path[i].y);serial.println(")\n\n");
    }

    serial.println("----------------------------------");
    serial.println(" End of debugging");
    serial.println("----------------------------------\n\n");

    return true;
}

void Grid::serial_print_grid(HardwareSerial& serial){
    for(int y = 0; y < height; y++){
        serial.println(" ");

        for(int x = 0; x < width; x++){
            State state = get_node(x, y)->get_state();

            switch(state){
                case WALL: serial.print("# "); break;
                case START: serial.print("S "); break;
                case FINISH: serial.print("F "); break;
                case WALKABLE: serial.print("W "); break;
                case OPEN: serial.print("O "); break;
                case CLOSED: serial.print("X "); break;
                case PATH: serial.print("P "); break;
                default: serial.print("? "); break;
            }
        }
    }
    serial.println("\n\n");
}

Node* Grid::get_node(int x, int y){
    if((x >= 0 && x < width) && (y >= 0 && y < height)){
        return &map[y][x];    
    }
    return nullptr;
}

vector<Node*> Grid::get_neighbours(Node *node){
    vector<Node*> neighbours;
    pos p = node->get_position();

    int dx[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
    int dy[8] = {-1, -1, -1, 0, 0, 1, 1, 1};

    for(int i = 0; i < 8; i++){
        Node* neighbour = get_node(p.x + dx[i], p.y + dy[i]);

        if(neighbour == nullptr) continue;

        neighbours.push_back(neighbour);

    }

    return neighbours; 
}

// Struct that sets compare rules for priority queue
// Sorts for the lowest f cost
// If it happens that 2 f costs are the same, then it cheks h cost
struct compareF{
    bool operator()(const Node *a, const Node *b){
        if(a->get_f() == b->get_f()){
            return a->get_h() > b->get_h();
        }
        return a->get_f() > b->get_f();
    }
};

vector<pos> reconstruct_path(Node* node){
    vector<pos> path;
    Node *current_node = node;
    Node *parrent = current_node->get_parent();
    current_node->set_path();

    while(parrent != nullptr){
        path.push_back(current_node->get_position());
        current_node = parrent;
        current_node->set_path();
        parrent = current_node->get_parent();
    }

    return path;
}

vector<pos> pathfind(Grid& grid, pos start, pos end){
    priority_queue<Node*, vector<Node*>, compareF> open_list;

    Node* start_node = grid.get_node(start.x, start.y);
    Node* end_node = grid.get_node(end.x, end.y);

    start_node->set_start();
    end_node->set_finish();
    start_node->calculate_f(start_node, end_node);

    open_list.push(start_node);
    start_node->set_open();

    Node* current_node;
    while(!open_list.empty()){
        current_node = open_list.top();

        if(current_node == end_node){
            return reconstruct_path(current_node);
        }

        current_node->set_closed();
        open_list.pop();

        vector<Node*> neighbours = grid.get_neighbours(current_node);

        for(Node *neighbour : neighbours){
            if(!neighbour->isWalkable()){
                continue;
            }

            neighbour->calculate_f(start_node, end_node);
            neighbour->set_parent(current_node);
            open_list.push(neighbour);
            neighbour->set_open();
        }

    }

    return {};
}