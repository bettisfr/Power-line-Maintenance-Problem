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
            &algorithms::bin_s,
            &algorithms::knapsack_opt,
            &algorithms::col_s,
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

public:
    explicit algorithms(deployment *);

    solution run_experiment(int);

    

    /// Opt
    solution opt_ilp_unit_load();
    solution opt_ilp_arbitrary_load();
    solution knapsack_opt();
    ///APX
    solution bin_s();
    solution col_s();
};

#endif //ALGORITHMS_H
