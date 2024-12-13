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
//    constexpr static const double epsilon = 0.01;

    deployment *dep;

    vector<function<solution(algorithms &)>> algorithm_functions = {
            &algorithms::opt_ilp, // 0
            &algorithms::bin_packing, // 1
            &algorithms::knapsack, // 2
            &algorithms::coloring, // 3
            &algorithms::greedy_reward, // 4
            &algorithms::greedy_energy, // 5
            &algorithms::greedy_reward_energy, // 6
            &algorithms::greedy_reward_load, // 7
            &algorithms::max_profit_extended, // 8
    };

    tuple<int, int> compute_LR(const vector<int> &);

    vector<int>
    weighted_interval(const vector<int> &, const vector<int> &, const vector<int> &, vector<int>, const vector<int> &);

    int compute_opt(int, const vector<int> &, const vector<int> &, vector<int>, vector<int>, vector<int>);

    vector<int> find_solution(int, const vector<int> &, const vector<int> &, vector<int>, vector<int>, vector<int>, vector<int>);

    solution opt_ilp_helper(vector<vector<int>> &, vector<double> &);

    solution opt_ilp_ul();

    solution opt_ilp_al();

    static bool check_correct_interval(const vector<vector<int>> &, vector<int>, vector<int>, int, int);

    solution greedy_reward_helper(vector<vector<int>>, vector<double>);

    solution greedy_reward_ul();

    solution greedy_reward_al();

    solution greedy_energy_helper(vector<vector<int>>, vector<double>);

    solution greedy_energy_ul();

    solution greedy_energy_al();

    solution flight_selection_in_heu(vector<vector<int>>, vector<double>, vector<int>, vector<int>, vector<int>);

    solution greedy_reward_energy_helper(vector<vector<int>>, vector<double>);

    solution greedy_reward_energy_ul();

    solution greedy_reward_energy_al();

    solution greedy_reward_load_helper(vector<vector<int>>, vector<double>);

    solution greedy_reward_load_ul();

    solution greedy_reward_load_al();

    tuple<vector<vector<int>>, vector<double>, vector<int>, vector<int>, vector<int>> sorting_with_rendezvouses_in_apx();

    solution bin_packing_helper();

    solution bin_packing_ul();

    solution bin_packing_al();

    solution knapsack_opt_helper();

    solution knapsack_opt_ul();

    solution knapsack_heu_al();

    solution coloring_helper();

    solution coloring_ul();

    solution coloring_al();

    bool if_flight_extends(vector<int>, int, double);

public:
    explicit algorithms(deployment *);

    solution run_experiment(int);

    solution opt_ilp();

    solution bin_packing();

    solution knapsack();

    solution greedy_reward();

    solution greedy_energy();

    solution greedy_reward_energy();

    solution greedy_reward_load();

    solution coloring();

    solution max_profit_extended();

};

#endif //ALGORITHMS_H
