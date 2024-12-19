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
            &algorithms::opt_ilp, // 0
            &algorithms::bin_packing, // 1
            &algorithms::knapsack, // 2
            &algorithms::coloring, // 3
            &algorithms::greedy_profit, // 4
            &algorithms::greedy_energy, // 5
            &algorithms::greedy_profit_energy, // 6
            &algorithms::greedy_profit_load, // 7
            &algorithms::max_profit_extended, // 8
    };

    solution greedy_profit_helper(vector<vector<int>>, vector<double>);

    solution greedy_profit_ul();

    solution greedy_profit_al();

    solution greedy_energy_helper(vector<vector<int>>, vector<double>);

    solution greedy_energy_ul();

    solution greedy_energy_al();

    solution flight_selection_in_heu(vector<vector<int>>, vector<double>, vector<int>, vector<int>, vector<int>);

    solution greedy_profit_energy_helper(vector<vector<int>>, vector<double>);

    solution greedy_profit_energy_ul();

    solution greedy_profit_energy_al();

    solution greedy_profit_load_helper(vector<vector<int>>, vector<double>);

    solution greedy_profit_load_ul();

    solution greedy_profit_load_al();

    solution bin_packing_helper();

    solution bin_packing_ul();

    solution bin_packing_al();

    solution knapsack_opt_helper();

    solution knapsack_opt_ul();

    solution knapsack_heu_al();

    solution coloring_helper();

    solution coloring_ul();

    solution coloring_al();

    bool if_flight_extends(const vector<int> &, int, double);

public:
    explicit algorithms(const deployment&);

    solution run_experiment(int);

    solution opt_ilp();

    solution bin_packing();

    solution knapsack();

    solution greedy_profit();

    solution greedy_energy();

    solution greedy_profit_energy();

    solution greedy_profit_load();

    solution coloring();

    solution max_profit_extended();

};

#endif //ALGORITHMS_H
