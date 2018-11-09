#include "astar.h"
#include <iostream>
#include <fstream>

template<typename T>
ostream& operator<< (ostream& out, const vector<T>& v) {
    for (auto it = v.begin(); it != v.end(); ++it){
        out << *it << " ";
    }
    return out;
}

// eliminate neighbors outside of the map
vector<int> elim_out(vector<int>& vec_in, int& num_points){
    vector<int> neighbors;
    for (auto it = vec_in.begin(); it != vec_in.end(); ++it){
        if (*it >= 0 && *it <= num_points * num_points - 1){
            neighbors.push_back(*it);
        }
    }
    return neighbors;
}

// distance of two points
double dis(vector<double>& point1, vector<double>& point2){
    double x1 = point1[0];
    double y1 = point1[1];
    double x2 = point2[0];
    double y2 = point2[1];
    return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
}

// isIn
bool isIn(vector<int>& vec, int& num){
    for (int i = 0; i < (int)vec.size(); ++i){
        if (vec[i] == num)
            return true;
    }
    return false;
}


bool collision_check(map<int, Node>& node_map, int& behind, int ahead, vector<vector<double>>& gates, double& circle_thre, double& elip_thre){
    // return true if there is a collision
    // vector from behind to ahead
    Node behind_node = node_map[behind];
    Node ahead_node = node_map[ahead];
    vector<double> behind_pos = behind_node.get_pos();
    vector<double> ahead_pos = ahead_node.get_pos();
    double len = dis(behind_pos, ahead_pos);
    vector<double> direction_normal;
    direction_normal.push_back((ahead_pos[0]-behind_pos[0])/len);
    direction_normal.push_back((ahead_pos[1]-behind_pos[1])/len);
    double dl = 0.05; // check every 0.02 m
    int num = (int)(len/dl + 1);
    vector<double> current_pos = behind_pos;
//    double current_x = behind_pos[0];
//    double current_y = behind_pos[1];

    for (int i = 0; i < num; ++i){
        for (int k = 0; k < (int)gates.size(); ++k){ // loop over gates
            vector<double> gate = gates[k];
            vector<double> left_gate_pos;
            vector<double> right_gate_pos;
            left_gate_pos.push_back(gate[0]);
            left_gate_pos.push_back(gate[1]);
            right_gate_pos.push_back(gate[2]);
            right_gate_pos.push_back(gate[3]);
            if (dis(current_pos, left_gate_pos) < circle_thre-0.01|| dis(current_pos, right_gate_pos) < circle_thre-0.01 || dis(current_pos, right_gate_pos)+dis(current_pos, left_gate_pos)<elip_thre) {
                return true; // there is a collision !!!!!!
            }
        }
        current_pos[0] += dl*direction_normal[0];
        current_pos[1] += dl*direction_normal[1];
    }
    return false;
}


vector<vector<int>>SmoothPath(map<int, Node>& node_map, vector<vector<int>>& path_in, vector<vector<double>>& gates,
                              double& circle_thre, double& elip_thre){
    vector<vector<int>> result;
    for (int k = 0; k < (int) path_in.size(); ++k){
        vector<int> path = path_in[k];
        vector<int> smooth_path;  // new smoothed path
        if (path.size() <=2){
            smooth_path = path;
            break;
        }
        auto behind = path.begin();
        auto ahead = path.begin()+1;

        // always add the start
        smooth_path.push_back(*behind);

        while(ahead != path.end()){
            ++ahead;
            if(collision_check(node_map, *behind, *ahead, gates, circle_thre, elip_thre)) {
                auto next = ahead - 1;  // next to push
                smooth_path.push_back(*next);
                behind = next;
            }
            if (*ahead == path.back()) {
                smooth_path.push_back(*ahead);
                break;
            }


        }

        result.push_back(smooth_path);

        //clear
        smooth_path.clear();

    }
    return result;


}


int main(){
    // num_points*num_points points L test
    // number of points per side
    int num_points = 60;

    // spacing between two connected points
    double space = 3.65/(num_points-1);

    // initialize position and neighbors of each node
    vector<vector<double>> node_pos;
    vector<vector<int>> node_neighbors;
    vector<double> temp_node_pos;           // temp node pos
    vector<int> temp_node_neighbors;        // temp node neighbors
    int temp_node_ID(0);
//    // initialize Node objects
//    vector<Node> nodes; //
    // node map
    map<int, Node> node_map;
    for (int j = 0; j < num_points; ++j)        // loop over y direction
        for (int i = 0; i < num_points; ++i){   // loop over x direction
            temp_node_pos.push_back(-1.8+i*space);   // node coordinates !! NEED to modify according to the lab field
            temp_node_pos.push_back(-1.8+j*space);
            temp_node_pos.push_back(0);
            node_pos.push_back(temp_node_pos);
            temp_node_ID = num_points*j+i;
            if (temp_node_ID % num_points == 0){
                temp_node_neighbors.push_back(temp_node_ID+1);
                temp_node_neighbors.push_back(temp_node_ID+num_points);
                temp_node_neighbors.push_back(temp_node_ID-num_points);
                temp_node_neighbors.push_back(temp_node_ID+num_points+1);
                temp_node_neighbors.push_back(temp_node_ID-num_points+1);
            } else if (temp_node_ID % num_points == num_points-1){
                temp_node_neighbors.push_back(temp_node_ID-1);
                temp_node_neighbors.push_back(temp_node_ID-num_points);
                temp_node_neighbors.push_back(temp_node_ID+num_points);
                temp_node_neighbors.push_back(temp_node_ID+num_points-1);
                temp_node_neighbors.push_back(temp_node_ID-num_points-1);
            } else {
                temp_node_neighbors.push_back(temp_node_ID-1);
                temp_node_neighbors.push_back(temp_node_ID+1);
                temp_node_neighbors.push_back(temp_node_ID+num_points);
                temp_node_neighbors.push_back(temp_node_ID-num_points);
                temp_node_neighbors.push_back(temp_node_ID+num_points+1);
                temp_node_neighbors.push_back(temp_node_ID-num_points+1);
                temp_node_neighbors.push_back(temp_node_ID+num_points-1);
                temp_node_neighbors.push_back(temp_node_ID-num_points-1);
            }
//            for (auto it = temp_node_neighbors.begin(); it != temp_node_neighbors.end(); ++it) {
//                if (*it < 0 || *it > num_points * num_points - 1) {
//                    temp_node_neighbors.erase(it);
////                    break;
//                }
//            }
            // eliminate neighbors that are not on map;
            temp_node_neighbors = elim_out(temp_node_neighbors, num_points);

//            cout << temp_node_neighbors << endl;
            node_neighbors.push_back(temp_node_neighbors);

            // assign value to Node object;
            Node temp_node(temp_node_ID, temp_node_pos, temp_node_neighbors);
//            nodes.push_back(temp_node);

            // assign values to node map
            node_map.emplace(temp_node_ID, temp_node);

            // clear
            temp_node_neighbors.clear();
            temp_node_pos.clear();
        }


    // load gates coordinates
    vector<vector<double>> gates;    //x_left, y_left, x_right, y_right for each gate, four gates in total
    fstream gates_file ("/home/yueshen/Desktop/BalancebotPlan/cmake-build-debug/gates.txt");
    double coordinate;
    int count = 0;
    vector<double> temp_gate;
    while (gates_file >> coordinate){
        cout << "before "<< endl;
        int idx = count/4;
        cout << "ind = " << idx << " " << coordinate << endl;
        temp_gate.push_back(coordinate);
//        gates[idx].push_back(coordinate);
        cout << "after" << endl;
        ++count;
        if (count%4 == 0){
            gates.push_back(temp_gate);
            temp_gate.clear();
        }
    }
    for (int k = 0; k < 4; ++k)
        cout << "gates coordinate " << k <<": "<< gates[k] << endl;

//    double gate1[] = {1.2, 1.2, 1.2, 0.6};
//    double gate2[] = {2.4, 0.6, 2.4, 0};
//    double gate3[] = {3.0, 1.2, 2.4, 1.2};
//    double gate4[] = {1.8, 1.8, 1.2, 1.8};
//
//
//    gates.push_back(vector<double> (gate1, gate1+sizeof(gate1)/sizeof(double)));
//    gates.push_back(vector<double> (gate2, gate2+sizeof(gate2)/sizeof(double)));
//    gates.push_back(vector<double> (gate3, gate3+sizeof(gate3)/sizeof(double)));
//    gates.push_back(vector<double> (gate4, gate4+sizeof(gate4)/sizeof(double)));


    // generate goals and initials
    vector<vector<double>> goals;  // x, y for each gate, four gates in total  4*2
    vector<vector<double>> initials; // x, y for each gate, four gates in total  4*2
    for (int i = 0; i < 4; ++i){
        double xl = gates[i][0];
        double yl = gates[i][1];
        double xr = gates[i][2];
        double yr = gates[i][3];
        vector<double> pgate_normal;
        double gate_len = sqrt((yl-yr)*(yl-yr) + (xr-xl)*(xr-xl));
        pgate_normal.push_back((-yr+yl)/gate_len);
        pgate_normal.push_back((-xl+xr)/gate_len);

        vector<double> tmp_goal;
        vector<double> tmp_initial;

        tmp_goal.push_back((xl+xr)/2 - 0.25*pgate_normal[0]);
        tmp_goal.push_back((yl+yr)/2 - 0.25*pgate_normal[1]);

        tmp_initial.push_back((xl+xr)/2 + 0.25*pgate_normal[0]);
        tmp_initial.push_back((yl+yr)/2 + 0.25*pgate_normal[1]);

        goals.push_back(tmp_goal);
        initials.push_back(tmp_initial);

        // clear
        tmp_goal.clear();
        tmp_initial.clear();

    }

    vector<int> goals_id (4, 0);  // four gates in total
    for (int k = 0; k < 4; ++k) {
        double min_dis (10);
        for (int i = 0; i < num_points; ++i)
            for (int j = 0; j < num_points; ++j) {
                Node current = node_map[i*num_points + j];
                vector<double> current_pos = current.get_pos();
//                cout << current_pos << endl;
                if(dis(current_pos, goals[k]) < min_dis) {
                        min_dis = dis(current_pos, goals[k]);
                        goals_id[k] = current.get_ID();
                }

            }
    }

    vector<int> initials_id(4, 0); // four gates in total
    for (int k = 0; k < 4; ++k) {
        double min_dis (10);
        for (int i = 0; i < num_points; ++i)
            for (int j = 0; j < num_points; ++j) {
                Node current = node_map[i*num_points + j];
                vector<double> current_pos = current.get_pos();
//                cout << current_pos << endl;
                if(dis(current_pos, initials[k]) < min_dis) {
                    min_dis = dis(current_pos, initials[k]);
                    initials_id[k] = current.get_ID();
                }

            }
    }

    for (int i = 0; i < 4; ++i){
        cout << "gate goal id = " << goals_id[i] << " gate pos = " << goals[i] << endl;
        cout << "gate initial id = " << initials_id[i] << " gate pos = " << initials[i]<< endl;
    }



    // find obstacle ID from obstacle
    // check circles with r = 0.25 centered at the two poles
    // check if it lies in an elipse
    // define obstacles' ID

    vector<int> obst_id; //(ob_id, ob_id + sizeof(ob_id)/sizeof(int));
    double circle_thre = 0.25;
    double elip_thre = 0.73;
    for (int i = 0; i < num_points; ++i)
        for (int j = 0; j < num_points; ++j){
            for (int k = 0; k < 4; ++k) {
                double xl = gates[k][0];
                double yl = gates[k][1];
                double xr = gates[k][2];
                double yr = gates[k][3];
                vector<double> gateleft;
                gateleft.push_back(xl);
                gateleft.push_back(yl);
                vector<double> gateright;
                gateright.push_back(xr);
                gateright.push_back(yr);

                Node current = node_map[i*num_points + j];
                vector<double> current_pos = current.get_pos();
                if (dis(current_pos, gateleft) < circle_thre || dis(current_pos, gateright) < circle_thre || (dis(current_pos, gateleft)+dis(current_pos, gateright)) < elip_thre){
                    obst_id.push_back(current.get_ID());
                    break;
                }

                // clear
                gateleft.clear();
                gateright.clear();
            }

        }




    // print neighbors for checking
    cout << "checking all nodes... "<< endl;
    for (int i = 0; i < num_points; ++i){
        cout << "nodes ID: " << node_map[i].get_ID() << " ";
        cout << "nodes postions: (" << node_map[i].get_pos() << ")";
        cout << "nodes neighbors: " << node_map[i].get_neighbors() << endl;
        cout << endl;
    }

    // astar result path
    vector<vector<int>> total_path_astar;
    for (int i = 0; i < 4; ++i){
        total_path_astar.push_back(astar(node_map[initials_id[i]], node_map[goals_id[(i+1)%4]], node_map, obst_id));
    }
    // smoothing the path result
    vector<vector<int>> total_path_smoothed = SmoothPath(node_map, total_path_astar, gates, circle_thre, elip_thre);






    // print solution path
    cout << "solution path: "<< endl;
    cout << total_path_astar << endl;

    // visualize map
    for (int i = num_points-1; i >=0; --i) {
        for (int j = 0; j < num_points; ++j) {
//            cout << "nodes ID: " << node_map[i * num_points + j].get_ID() << " ";
//            cout << "nodes postions: (" << node_map[i * num_points + j].get_pos() << ")";
            int id = node_map[i * num_points + j].get_ID();
            if(isIn(obst_id, id))
                cout << " " << " ";
            else if (isIn(total_path_astar[0], id))
                cout << "1" << " ";
            else if (isIn(total_path_astar[1], id))
                cout << "2" << " ";
            else if (isIn(total_path_astar[2], id))
                cout << "3" << " ";
            else if (isIn(total_path_astar[3], id))
                cout << "4" << " ";
            else
                cout << "o" << " ";
        }
        cout << endl;
    }
    cout << endl;
    cout << "##################### SMOOTHED PATH ########################" << endl;

    for (int i = num_points-1; i >=0; --i) {
        for (int j = 0; j < num_points; ++j) {
//            cout << "nodes ID: " << node_map[i * num_points + j].get_ID() << " ";
//            cout << "nodes postions: (" << node_map[i * num_points + j].get_pos() << ")";
            int id = node_map[i * num_points + j].get_ID();
            if(isIn(obst_id, id))
                cout << " " << " ";
            else if (isIn(total_path_smoothed[0], id))
                cout << "1" << " ";
            else if (isIn(total_path_smoothed[1], id))
                cout << "2" << " ";
            else if (isIn(total_path_smoothed[2], id))
                cout << "3" << " ";
            else if (isIn(total_path_smoothed[3], id))
                cout << "4" << " ";
            else
                cout << "o" << " ";
        }
        cout << endl;
    }
    cout << endl;

    // write to file
    int num_waypoints = 0;
    for (int k = 0; k < (int) total_path_smoothed.size(); ++k){
        num_waypoints += (int) total_path_smoothed[k].size();
    }

    ofstream waypoints_file ("./waypoints.txt");
    if (waypoints_file.is_open()){
        waypoints_file << num_waypoints << endl;
        vector<double> current_pos = node_map[total_path_smoothed[3].back()].get_pos();
        waypoints_file << current_pos[0] << " " << current_pos[1] << endl;
        for (int k = 0; k < 4; ++k)
            for (int i = 0; i < (int) total_path_smoothed[k].size(); ++i){
                if (k == 3 && i == (int) total_path_smoothed[k].size()-1){
                    break;
                } else {
                    vector<double> current_pos = node_map[total_path_smoothed[k][i]].get_pos();
                    waypoints_file << current_pos[0] << " " << current_pos[1] << endl;
                }
            }
        waypoints_file.close();
    }

    return 0;
}