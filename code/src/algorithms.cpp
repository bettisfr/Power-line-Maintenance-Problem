#include "algorithms.h"
#include <vector>
#include <unordered_set>

algorithms::algorithms(deployment *m_dep) {
    dep = m_dep;
}

// Useless (and stupid) method, but it was nice to use a vector of pointers to methods :-D
solution algorithms::run_experiment(int algorithm) {
    int index = algorithm;
    solution out;

    if (index >= 0 && index <= 7) {
        out = algorithm_functions[index](*this);
    } else {
        cerr << "Invalid algorithm index." << endl;
    }

    return out;
}

solution algorithms::opt_ilp() {
    return solution();
}

solution algorithms::knapsack_multidrone() {
    return solution();
}

solution algorithms::greedy_submodular_multidrone() {
    return solution();
}

solution algorithms::generalize_bin_packing() {
    return solution();
}

solution algorithms::coloring_multidrone() {
    return solution();
}

solution algorithms::greedy_weight_selection() {
    return solution();
}

solution algorithms::greedy_ending_selection() {
    return solution();
}

solution algorithms::greedy_reward_selection() {
    return solution();
}




