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

    vector<function<solution(algorithms &)>> algorithm_functions;

    solution ilp_solver(tuple<vector<vector<int>>, vector<double>, vector<int>, vector<int>>);

    solution flight_selection_in_heu(vector<vector<int>>, vector<double>, vector<int>, vector<double>, vector<double>);

public:
    explicit algorithms(deployment );

    solution run_experiment(int);

    solution no_alg();

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
