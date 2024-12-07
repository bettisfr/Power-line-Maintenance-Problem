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
            &algorithms::opt_ilp_unit_load,
            &algorithms::opt_ilp_arbitrary_load,
            &algorithms::bin_s_unit_load,
            &algorithms::bin_s_arbitrary_load,
            &algorithms::knapsack_opt_unit_load,
            &algorithms::knapsack_heu_arbitrary_load,
            &algorithms::col_s,
            //&algorithms::col_s_unit_load,
            //&algorithms::col_s_arbitrary_load,
            &algorithms::greedy_reward_selection_unit_load,
            &algorithms::greedy_reward_selection_arbitrary_load,
            &algorithms::greedy_energy_selection_unit_load,
            &algorithms::greedy_energy_selection_arbitrary_load,
            &algorithms::greedy_reward_energy_selection_unit_load,
            &algorithms::greedy_reward_energy_selection_arbitrary_load,
            &algorithms::greedy_reward_load_selection_unit_load,
            &algorithms::greedy_reward_load_selection_arbitrary_load,
    };

    tuple<int, int> compute_LR(const vector<int> &);

    vector<int> weighted_interval(const vector<int> &, const vector<int> &, const vector<int> &,
                                  vector<int>, const vector<int> &);

    int compute_opt(int, const vector<int> &, const vector<int> &, vector<int>,
                    vector<int>, vector<int>);

    vector<int> find_solution(int, const vector<int> &, const vector<int> &,
                              vector<int>, vector<int>,
                              vector<int>, vector<int>);

    solution opt_ilp_helper(vector<vector<int>>&, vector<double>&);

    bool getOverlap(int, int, int, int);

    bool check_correct_interval(vector<vector<int>>, vector<int>, vector<int>, int, int);

    solution greedy_reward_selection_helper(vector<vector<int>>, vector<double>);

    solution greedy_energy_selection_helper(vector<vector<int>>, vector<double>);

    solution flight_selectin_in_heu(vector<vector<int>>,vector<double>,
                                            vector<int>, vector<int>, vector<int>);
    
    solution greedy_reward_energy_selection_helper(vector<vector<int>>, vector<double>);

    solution greedy_reward_load_selection_helper(vector<vector<int>>, vector<double>);

    tuple<vector<vector<int>>, vector<double>, vector<int>, vector<int>, vector<int>> sorting_with_rendezvouses_in_apx(string);

    solution bin_s_helper(string);

    solution knapsack_opt_helper(string);

    solution col_s_helper(string);

public:
    explicit algorithms(deployment *);

    solution run_experiment(int);    
    /// Opt
    solution opt_ilp_unit_load();
    solution opt_ilp_arbitrary_load();
    solution knapsack_opt_unit_load();
    ///APX
    solution bin_s_unit_load();
    solution bin_s_arbitrary_load(); 
    solution col_s();
    //solution col_s_unit_load();
    //heuristics
    solution knapsack_heu_arbitrary_load();
    //solution col_s_arbitrary_load();
    solution greedy_reward_selection_unit_load();
    solution greedy_reward_selection_arbitrary_load();
    solution greedy_energy_selection_unit_load();
    solution greedy_energy_selection_arbitrary_load();
    solution greedy_reward_energy_selection_unit_load();
    solution greedy_reward_energy_selection_arbitrary_load();
    solution greedy_reward_load_selection_unit_load();
    solution greedy_reward_load_selection_arbitrary_load();
};

#endif //ALGORITHMS_H
