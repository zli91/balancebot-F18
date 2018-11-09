#include "astar.h"


// constructors
Node::Node() {}

Node::Node(int node_ID_in, vector<double> pos_in, vector<int> neighbors_in){
    node_ID = node_ID_in;
    pos = pos_in;
    neighbors = neighbors_in;
}

void Node::set_ID(int ID_in){
    node_ID = ID_in;
}

int Node::get_ID() const {
    return node_ID;
}

void Node::set_pos(vector<double> pos_in) {
    pos = pos_in;
}

vector<double> Node::get_pos() const {
    return pos;
}

void Node::set_neighbors(vector<int> neighbor_in){
    neighbors = neighbor_in;
}

vector<int> Node::get_neighbors() const {
    return neighbors;
}

//Node& Node::operator=(Node &node_rhs) {
//    node_ID = node_rhs.node_ID;
//    pos = node_rhs.pos;
//    neighbors = node_rhs.neighbors;
//    return *this;
//}


// define a type of pairs: pair<priority, Node>
bool PNcomp::operator() (Priority_Node pn1, Priority_Node pn2) {
    return pn1.first > pn2.first;
}

// functions
inline bool Priority_Queue::empty() const {
    return PQ.empty();
}

inline void Priority_Queue::add(Node node, double f) {
    PQ.emplace(f, node);
}

// get the top node in the priority queue and pop the top node
Node Priority_Queue::get(){
    Node top_node = PQ.top().second;
    PQ.pop();
    return top_node;
}


double edge_cost(const Node& current, const Node& next_node){
    vector<double> next_node_pos = next_node.get_pos();
    vector<double> current_pos = current.get_pos();

    double dx = next_node_pos[0] - current_pos[0];
    double dy = next_node_pos[1] - current_pos[1];
    double dz = next_node_pos[2] - current_pos[2];

    return sqrt(dx*dx + dy*dy + dz*dz);

}


double heuristic(const Node& next_node, const Node& goal_node){
    vector<double> next_node_pos = next_node.get_pos();
    vector<double> goal_node_pos = goal_node.get_pos();

    double dx = next_node_pos[0] - goal_node_pos[0];
    double dy = next_node_pos[1] - goal_node_pos[1];
    double dz = next_node_pos[2] - goal_node_pos[2];

    return sqrt(dx*dx + dy*dy + dz*dz);

}


vector<int> reconstruct_path(map<int, int>& came_from, int& current_node_ID, const int& start_ID) {
    // reconstruct the waypoints
    // output: a vector of node IDs of waypoints

    vector<int> total_path;

    while(current_node_ID != start_ID){
        total_path.push_back(current_node_ID);
        current_node_ID = came_from[current_node_ID];
    }
    total_path.push_back(start_ID);
    reverse(total_path.begin(), total_path.end());
    return total_path;
}

// INPUT: start_node, goal_node, node_map, neighbor_type, heuristic_type
// OUTPUT: a vector containing the IDs of all the waypoints
vector<int> astar(const Node& start_node, const Node& goal_node, map<int, Node>& node_map, vector<int>& obst_id){
    // start_node: Node object, start node
    // goal_node: Node object, goal
    // neighbor type
    // heuristic type not implemented yet

    cout << "starting astar..." << endl;
    int start_ID = start_node.get_ID();
    int goal_ID = goal_node.get_ID();

    // total path (optimal path that will be returned) in node ID
    vector<int> total_path;

    // Nodes that have been explored () in node ID
    vector<int> explored;

    // frontier nodes in Priority_Queue
    Priority_Queue frontier;
    frontier.add(start_node, 0);

    // the most efficient previous steps in Node ID
    map<int, int> came_from;
    came_from[start_ID] = 0;

    // cost so far, g(x)
    map<int, double> cost_so_far;
    cost_so_far[start_ID] = 0;

    // total cost, f(x) = g(x) + h(x)
    map<int, double> total_cost;

    // initialize obstacles
    double inf = 10000.0;
    for (int i = 0; i < (int)obst_id.size(); ++i){
        cost_so_far[obst_id[i]] = inf;
    }
//    cost_so_far[414] = inf;
//    cost_so_far[415] = inf;
//    cost_so_far[416] = inf;
//    cost_so_far[391] = inf;
//    cost_so_far[366] = inf;
//    cost_so_far[413] = inf;
//    cost_so_far[412] = inf;
//    cost_so_far[411] = inf;
//    cost_so_far[341] = inf;
//    cost_so_far[316] = inf;
//    cost_so_far[291] = inf;
//
    // loop
    while (!frontier.empty()){
        // get first entry of the frontier and pop it out from the frontier
        const Node current = frontier.get();
        int current_node_ID = current.get_ID();

        // add this node to explored
        explored.push_back(current_node_ID);

        if (current_node_ID == goal_ID){
            //
            total_path = reconstruct_path(came_from, current_node_ID, start_ID);
            break;
        }

        // get neighbors ID of current node
        vector<int> neighbor_IDs = current.get_neighbors();

        for (int next : neighbor_IDs){
            const Node next_node = node_map[next];
            double new_cost = cost_so_far[current_node_ID] + edge_cost(current, next_node);

            if (cost_so_far.find(next) == cost_so_far.end() || new_cost < cost_so_far[next]){
//                cout << cost_so_far.find(next)->first << " first " << cost_so_far.end()->first << endl;
//                cout << cost_so_far.find(next)->second << " second " << cost_so_far.end()->second << endl;
//                cout << "cost so far[next] = " << cost_so_far[next] << endl;
                // collision checker
                if (cost_so_far[next] >= inf) {
//                    cout << next << " g = " << cost_so_far[next] << endl;
                    continue;
                }
                // if no collision happened
                cost_so_far[next] = new_cost;
                double f = new_cost + 1.1*heuristic(next_node, goal_node);
                frontier.add(next_node, f);
                came_from[next] = current_node_ID;
            }
        }
    }
    cout << "size of explored = " << explored.size() << endl;
//    cout << cost_so_far[18] << endl;
    return total_path;
}