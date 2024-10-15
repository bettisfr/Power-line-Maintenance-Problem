#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#include <iostream>
#include <iomanip>
#include <vector>

using namespace std;

struct solution {
    // General info to be saved
    double total_reward = -1;
    double total_cost = -1;

    double running_time = -1;

    friend ostream &operator<<(ostream &os, const solution &out) {
        os << "Total reward: " << out.total_reward << endl;
        os << "Total costs: " << out.total_cost << endl;
        os << "Running Time: " << out.running_time << endl;

        return os;
    }
};

#endif //DEFINITIONS_H
