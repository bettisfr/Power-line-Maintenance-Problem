#ifndef ALGORITHMS_H
#define ALGORITHMS_H

#include <iostream>
#include <vector>
#include <tuple>
#include <functional>
#include <cmath>
#include <unordered_set>

#include "input.h"
#include "deployment.h"

using namespace std;

class algorithms {

private:
    constexpr static const double epsilon = 0.01;

    deployment *dep;

    vector<function<solution(algorithms &)>> algorithm_functions = {
            &algorithms::opt_ilp,
            &algorithms::heuristic_1,
            &algorithms::heuristic_2
    };

public:
    explicit algorithms(deployment *);

    double get_distance(int, int, int, int);
    tuple<vector<vector<int>>, vector<double>> compute_all_flights();
    double compute_energy_cost(vector<int>);
    int compute_profit(vector<int>);
    int compute_load(vector<int>);
    bool check_intersection(vector<int>, vector<int>);

    solution run_experiment(int);

    solution opt_ilp();

    solution heuristic_1();

    solution heuristic_2();
};

#endif //ALGORITHMS_H
