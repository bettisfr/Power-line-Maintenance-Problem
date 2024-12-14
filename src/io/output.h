#ifndef OUTPUT_H
#define OUTPUT_H

#include <iostream>
#include <ctime>
#include <cstdlib>
#include <map>
#include <fstream>
#include <cmath>
#include <vector>

#include "input.h"

using namespace std;

struct solution {
    // General info to be saved
    int total_profit = -1;
    double total_energy = -1;
    vector<vector<int>> total_flights;
    vector<int> profits;
    vector<double> energies;
    vector<int> weights;

    double running_time = -1;

    friend ostream &operator<<(ostream &os, const solution &out) {
        os << "Total profit=" << out.total_profit << endl;
        os << "Total energy=" << out.total_energy << endl;
        os << "Total flights=" << out.total_flights.size() << endl;
        int i = 0;
        for (const auto &s: out.total_flights) {
            os << "Flight=" << i << " -> ";
            for (auto e: s) {
                os << "[" << e << "] ";
            }
            os << "profit=" << out.profits[i] << ", energy=" << out.energies[i] << ", weight=" << out.weights[i]
               << endl;

            i++;
        }
        os << "Running time: " << out.running_time << endl;

        return os;
    }
};

template<typename T>
pair<double, double> calculate_avg_std(const vector<T> &values) {
    if (values.empty()) {
        return {0.0, 0.0};
    }

    T sum = static_cast<T>(0);
    for (const T &value: values) {
        sum += value;
    }

    double average = static_cast<double>(sum) / values.size();

    if (values.size() == 1) {
        return {average, 0.0}; // Standard deviation is 0 for a single value
    }

    T sum_squared_diff = static_cast<T>(0);
    for (const T &value: values) {
        T diff = value - static_cast<T>(average);
        sum_squared_diff += diff * diff;
    }

    double std_dev = sqrt(static_cast<double>(sum_squared_diff) / (values.size() - 1.0));

    return {average, std_dev};
}

void save_output(const input &, const vector<solution> &);

#endif //OUTPUT_H
