
#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#include <iostream>
#include <iomanip>
#include <vector>

using namespace std;

struct solution {
    // General info to be saved
    int total_profit = -1;
    double total_energy_cost = -1;
    vector<vector<int>> intervals;
    vector<int> profits;
    vector<double> energies;
    vector<int> loads;

    double running_time = -1;

    friend ostream &operator<<(ostream &os, const solution &out) {
        os << "Total profit=" << out.total_profit << endl;
        os << "Total energy=" << out.total_energy_cost << endl;
        os << "Total flights=" << out.intervals.size() << endl;
        int i = 0;
        for (const auto &s: out.intervals) {
            os << "Flight=" << i << " -> ";
            for (auto e: s) {
                os << "[" << e << "] ";
            }
            os << "Profit=" << out.profits[i] << ", Energy=" << out.energies[i] << ", Load=" << out.loads[i] << endl;

            i++;
        }
        os << "Running Time: " << out.running_time << endl;

        return os;
    }
};

#endif //DEFINITIONS_H
