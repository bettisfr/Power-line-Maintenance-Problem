#ifndef UTIL_H
#define UTIL_H

#include <vector>
#include <iostream>

using namespace std;

class util {

public:
    static double get_distance(double, double, double, double);

    static bool check_intersection(const vector<int> &, vector<int>);

    static vector<int> largest_non_overlap_delivery(vector<int> launches, vector<int> rendezvouses);
};


#endif //UTIL_H
