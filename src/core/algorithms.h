#ifndef ALGORITHMS_H
#define ALGORITHMS_H

#include <iostream>
#include <vector>
#include <tuple>
#include <functional>
#include <cmath>
#include <unordered_set>

#include "../io/input.h"
#include "../io/output.h"
#include "deployment.h"

using namespace std;

class algorithms {

private:
    deployment dep;
    solution sol;

    vector<function<solution(algorithms &)>> algorithm_functions = {
            &algorithms::opt_multi, // 0
            &algorithms::bin_packing, // 1
            &algorithms::knapsack, // 2
            &algorithms::coloring, // 3
            &algorithms::greedy_profit, // 4
            &algorithms::greedy_energy, // 5
            &algorithms::greedy_profit_energy, // 6
            &algorithms::greedy_profit_load, // 7
            &algorithms::opt_single, // 8
    };

    solution ilp_solver(tuple<vector<vector<int>>, vector<double>, vector<int>, vector<int>>);

    solution flight_selection_in_heu(vector<vector<int>>, vector<double>, vector<int>, vector<int>, vector<int>);

public:
    explicit algorithms(const deployment&);

    solution run_experiment(int);

    solution opt_multi();

    solution opt_single();

    solution bin_packing();

    solution knapsack();

    solution greedy_profit();

    solution greedy_energy();

    solution greedy_profit_energy();

    solution greedy_profit_load();

    solution coloring();
};

#endif //ALGORITHMS_H
