#ifndef CODE_UTIL_H
#define CODE_UTIL_H

#include <vector>
#include <iostream>

using namespace std;

class util {

public:
    static double get_distance(double, double, double, double);

    static bool check_intersection(const vector<int> &, vector<int>);

    static vector<int> largest_nonoverlap_delivery(vector<int>, vector<int>);
};


#endif //CODE_UTIL_H
