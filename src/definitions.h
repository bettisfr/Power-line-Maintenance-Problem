#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#include <iostream>
#include <iomanip>
#include <vector>

using namespace std;

struct solution {
    // General info to be saved
    double total_profit = -1;
    vector<vector<int>> selected_intervals;
    double total_energy_cost = -1;

    double running_time = -1;

    friend ostream &operator<<(ostream &os, const solution &out) {
        os << "Total profit: " << out.total_profit << endl;
        os << "Total energy costs: " << out.total_energy_cost << endl;
        os << "Selected intervals: " << endl;
        for (auto s : out.selected_intervals) {
            for (auto e : s) {
                os << e << ", ";
            }
            os << endl;
        }
        os << "Running Time: " << out.running_time << endl;

        return os;
    }
};

#endif //DEFINITIONS_H
