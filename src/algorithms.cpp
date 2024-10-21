#include "algorithms.h"
#include "definitions.h"
#include <vector>
#include <algorithm>
#include <cmath>

algorithms::algorithms(deployment *m_dep) {
    dep = m_dep;
}

// Useless (and stupid) method, but it was nice to use a vector of pointers to methods :-D
solution algorithms::run_experiment(int algorithm) {
    int index = algorithm;
    solution out;

    if (index >= 0 && index <= 2) {
        out = algorithm_functions[index](*this);
    } else {
        cerr << "Invalid algorithm index." << endl;
    }

    return out;
}

solution algorithms::opt_ilp() {
    return solution();
}

solution algorithms::heuristic_1() {
    solution sol;

    // Use here the important parameters
    int n = dep->get_num_deliveries();
    vector<int> launches = dep->get_launches();
    vector<int> rendezvouses = dep->get_rendezvouses();
    vector<int> profits = dep->get_profits();
    vector<int> loads = dep->get_loads();
    int B = dep->get_drone_battery();
    int L = dep->get_drone_load();

    // Example of how to fill the solution
    vector<vector<int>> selected;

    vector<int> s1;
    s1.push_back(3);
    selected.push_back(s1);

    vector<int> s2;
    s2.push_back(1);
    s2.push_back(4);
    s2.push_back(5);
    selected.push_back(s2);

    vector<int> s3;
    s3.push_back(8);
    selected.push_back(s3);

    sol.selected_intervals = selected;
    sol.total_energy_cost = 199;
    sol.total_profit = 144;

    return sol;
}

solution algorithms::heuristic_2() {
    return solution();
}
