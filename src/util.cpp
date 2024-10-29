#include <cmath>
#include <algorithm>
#include <vector>

#include "util.h"

double util::get_distance(int x1, int y1, int x2, int y2) {
    double delta_x = x2 - x1;
    double delta_y = y2 - y1;

    return sqrt(delta_x * delta_x + delta_y * delta_y);
}

bool util::check_intersection(vector<int> flight1, vector<int> flight2){
    vector<int> temp;
    set_intersection(flight1.begin(), flight1.end(),
                          flight2.begin(), flight2.end(),
                          back_inserter(temp));
    return !temp.empty();
}