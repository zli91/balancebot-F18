#ifndef ASTAR_H
#define ASTAR_H

#include <iostream>
#include <iomanip>
#include <map>
#include <set>
#include <array>
#include <vector>
#include <utility>
#include <queue>
#include <tuple>
#include <algorithm>
#include <cstdlib>
#include <cmath>

using namespace std;

class Node{
private:
    int node_ID;
    vector<double> pos;
    vector<int> neighbors; // neighbors in ID

public:

    //Constructors
    Node();

    Node(int node_ID_in, vector<double> pos_in, vector<int> neighbors_in);

    void set_ID(int ID_in);

    int get_ID() const;

    void set_pos(vector<double> pos_in);

    vector<double> get_pos() const;

    void set_neighbors(vector<int> neighbor_in);

    vector<int> get_neighbors() const;

//    Node& operator= (Node& node_rhs);
};

typedef pair<double, Node> Priority_Node;
struct PNcomp {
    bool operator() (Priority_Node pn1, Priority_Node pn2);
};

struct Priority_Queue {
    priority_queue<Priority_Node, vector<Priority_Node>, PNcomp> PQ;

    // functions
    inline bool empty() const;

    // add a pair into the queue with priority f
    inline void add(Node node, double f);

    // get the top node in the priority queue and pop the top node
    Node get();
};

double edge_cost(const Node& current, const Node& next_node);

double heuristic(const Node& next_node, const Node& goal_node);

vector<int> reconstruct_path(map<int, int>& came_from, int& current_node_ID, const int& start_ID);

vector<int> astar(const Node& start_node, const Node& goal_node, map<int, Node>& node_map, vector<int>& obst_id);



#endif