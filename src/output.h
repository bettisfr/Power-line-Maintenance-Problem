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
#include "definitions.h"

using namespace std;

// Template function to calculate the average of a vector of any numeric type
template <typename T>
double calculate_avg(const vector<T>& values) {
    if (values.empty()) {
        return static_cast<T>(0);
    }

    T sum = static_cast<T>(0);
    for (const T& value : values) {
        sum += value;
    }

    return sum / static_cast<T>(values.size());
}

// Template function to calculate the standard deviation of a vector of any numeric type
template <typename T>
double calculate_std(const vector<T>& values, double average) {
    if (values.size() <= 1) {
        return static_cast<T>(0);
    }

    T sum_squared_diff = static_cast<T>(0);
    for (const T& value : values) {
        T diff = value - average;
        sum_squared_diff += diff * diff;
    }

    return sqrt(sum_squared_diff / (static_cast<T>(values.size()) - static_cast<T>(1)));
}

void save_output(const input &, const vector<solution> &);

#endif //OUTPUT_H
