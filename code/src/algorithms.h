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
            &algorithms::knapsack_multidrone,
            &algorithms::greedy_submodular_multidrone,
            &algorithms::generalize_bin_packing,
            &algorithms::coloring_multidrone,
            &algorithms::greedy_weight_selection,
            &algorithms::greedy_ending_selection,
            &algorithms::greedy_reward_selection,
    };

public:
    explicit algorithms(deployment *);

    solution run_experiment(int);

    solution opt_ilp();

    solution knapsack_multidrone();

    solution greedy_submodular_multidrone();

    solution generalize_bin_packing();

    solution coloring_multidrone();

    solution greedy_weight_selection();

    solution greedy_ending_selection();

    solution greedy_reward_selection();

};

#endif //ALGORITHMS_H
