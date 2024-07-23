#ifndef TOSN_DEFINITIONS_H
#define TOSN_DEFINITIONS_H

#include <iostream>
#include <iomanip>
#include <vector>

using namespace std;

typedef tuple<double, double> point;

struct solution {
    // General info to be saved
    double total_reward = -1;
    double total_cost = -1;

    double running_time = -1;

    friend ostream &operator<<(ostream &os, const solution &out) {
        os << "Total reward: " << out.total_reward << endl;
        os << "Tours costs: " << out.total_cost << endl;
        os << "Running Time: " << out.running_time << endl;

        return os;
    }
};

#endif // TOSN_DEFINITIONS_H
