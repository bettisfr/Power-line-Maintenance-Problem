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
            &algorithms::heuristic_2,
            &algorithms::Bin_S,
            &algorithms::knapsack_opt,
            &algorithms::col_s,
    };

    tuple<vector<vector<int>>, vector<double>> compute_all_flights();

    int compute_profit(const vector<int>&);
    int compute_load(const vector<int>&);
    double compute_energy(const vector<int> &delivery_ids);

    tuple<int, int>compute_LR(const vector<int>&);
    vector<int> largest_nonoverlap_delivery(vector<int>, vector<int>);
    vector<int> weighted_interval(vector<int>, vector<int>, vector<int>,
                                        vector<int>, vector<int>);
    int compute_opt(int, vector<int>, vector<int>, vector<int>,
                                        vector<int>, vector<int>);
    vector<int> find_solution(int, vector<int>, vector<int>, 
                                      vector<int>, vector<int>,
                                      vector<int>, vector<int>);

public:
    explicit algorithms(deployment *);
    solution run_experiment(int);

    /// Opt
    solution opt_ilp();
    solution knapsack_opt();

    ///APX
    solution Bin_S();
    solution col_s();

    /// Heuristics
    solution heuristic_1();
    solution heuristic_2();

};

#endif //ALGORITHMS_H
