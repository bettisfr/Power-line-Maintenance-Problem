#ifndef CODE_UTIL_H
#define CODE_UTIL_H

#include <vector>
#include <iostream>

using namespace std;

class util {

public:
    static double get_distance(int, int, int, int);

    static bool check_intersection(vector<int>, vector<int>);
};


#endif //CODE_UTIL_H
