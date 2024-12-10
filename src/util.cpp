#include "gurobi_c++.h"
#include <iterator>
#include "definitions.h"
#include "algorithms.h"
#include <cmath>
#include <algorithm>
#include <vector>

#include "util.h"

double util::get_distance(int x1, int y1, int x2, int y2) {
    double delta_x = x2 - x1;
    double delta_y = y2 - y1;

    return sqrt(delta_x * delta_x + delta_y * delta_y);
}


bool util::check_intersection(vector<int> flight1, vector<int> flight2) {   
    for (int i:flight1){
        if (  find(flight2.begin(), flight2.end(), i) != flight2.end()  ){ // intersect false
            return true;
        }
    }
    return false;
}

vector<int> util::largest_nonoverlap_delivery(vector<int> launches, vector<int> rendezvouses) {
    vector<int> p;  // >= -1
    for (int i = 0; i < launches.size(); i++) {
        int id = -10;
        for (int j = 0; j < launches.size(); j++) {
            if (i != j) {
                if (rendezvouses[j] >= launches[i]) {
                    id = j - 1;
                    if (id == i) {
                        id = id - 1;
                    }
                    p.push_back(id);
                    id = -10;
                    break;
                } else {
                    id = j;
                }
            }
        }
        if (id >= 0) {
            p.push_back(id);
        }
    }

    return p;
}
