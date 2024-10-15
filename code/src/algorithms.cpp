#include "algorithms.h"
#include "definitions.h"
#include <vector>
#include <algorithm>

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

solution algorithms::greedy_weight_selection(vector<vector<int>> intervals, int drone=1, int budget=5000) {
    int total_reward = 0;
    vector<vector<int>> intervals_copy = intervals;//dalla funzione ricevo una lista
    vector<solution> solution;//poi conterrà un task che altro non è che un input. input è costituito da partenza, arrivo, costo e premio
    //a questi valori si aggiungeranno al termine della funzione il costo totale e il premio che però risulteranno uguali a quelli dell'input essendo unico l'input
    //unico dubbio. input in teoria prevede anche partenza e arrivo. qui invece solution no, solo reward e costo... come faccio?dovrò aggiungerlo in solution?
    //da chiedere, per ora magari non lo stampo ma semplicemente gli assegnerò singolarmente i 2 valori

    vector<int> batteries;

    int cost = 0;

    for(int i = 0; i < drone; i ++ ) {
        batteries.push_back(budget); //inserisce un elemento in fondo
        struct solution append;//perché devo riindicar struct
        solution.push_back(append);
    }
    std::sort(intervals_copy.begin(), intervals_copy.end(),[](const vector<int>& a, const vector<int>& b) {return a[2] < b[2]} );

    return solution();
}

solution algorithms::greedy_ending_selection() {
    return solution();
}

solution algorithms::greedy_reward_selection() {
    return solution();
}
