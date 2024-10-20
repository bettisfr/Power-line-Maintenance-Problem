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
    vector<vector<vector<int>>> sol;
    int total_cost=0;
    vector<int> batteries;

    int cost = 0;

    for(int i = 0; i < drone; i ++ ) {
        batteries.push_back(budget); //inserisce un elemento in fondo
        vector<vector<int>> append;
        sol.push_back(append);
    }

   //per ordinare in base al costo
    for(int i = 0; i<intervals_copy.size(); i++){
        vector<int> min = intervals_copy.at(i);
        for(int j = i; j< intervals_copy.size(); j++){
            vector<int> second = intervals_copy.at(j);
            if(second.at(2)< min.at((2))){
                vector<int> tmp = min;
                min = second;
                intervals_copy.at(j) = tmp;
            }
        }
        intervals_copy.at(i)=min;

    }



    while(intervals_copy.size() != 0 && [batteries]() {//lambda che restituisce la somma delle batterie x vedere se !=0
        int sum=batteries.at(0);
        for (int i = 1; i<batteries.size(); i++) {
            sum += batteries.at(i);
        }
        return sum;
    }() != 0) {
        int b=[batteries]() {//lambda che restituisce l'indice della batteria massima
            int max=batteries.at(0);
            int index_max=0;
            for (int i=1; i<batteries.size(); i++){
                if(batteries.at(i)>max){
                    max=batteries.at(i);
                    index_max = i;
                }
            }
            return index_max;
        }();

        vector<int> task = intervals_copy.front();
        intervals_copy.erase(intervals_copy.begin());
        if(sol.at(b).size() == 0) {
            if(task.at(2)<=batteries.at(b)) {
                sol[b].push_back(task);
                total_reward += task[3];
                total_cost += task[2];
                batteries[b] -= task[2];
            }else{
                if(task.at(2) <= batteries.at(b) && check_correct_interval(sol.at(b), task)){
                    sol.at(b).push_back(task);
                    total_reward += task.at(3);
                    batteries[b] -= task.at(2);
                    total_cost += task.at(2) ;
                }
            }
        }
    }
    solution result;
    result.total_reward = total_reward;
    result.total_cost = cost;
    cout<<"premio totale:"<<result.total_reward<<" costo totale:"<<result.total_cost<<endl;
    for(int i=0; i<sol.size();i++){
        for(int j=0; j<sol.at(i).size(); j++){
            for(int z=0; z<sol.at(i).at(j).size(); z++){
                cout << sol.at(i).at(j).at(z)<<endl;
            }

        }
    }
    return result;
}

solution algorithms::greedy_ending_selection() {

    return solution();
}

solution algorithms::greedy_reward_selection() {
    return solution();
}

bool algorithms::check_correct_interval(vector<vector<int>>&list_of_intervals, vector<int> interval) {//solution, task
    std::sort(list_of_intervals.begin(), list_of_intervals.end(),[](const vector<int>& a, const vector<int>& b) {return a[1] < b[1];} );
    for(int i =0 ; i<list_of_intervals.size(); i++ ){
        if(getOverLap(list_of_intervals.at(i), interval) !=0){
            return false;
        }
    }
    return true;
}

int algorithms::getOverLap(vector<int> a, vector<int> b) {
    if(max(0, min(a[1],b[1])-max(a[0],b[0]))==0){
        return true;
    }
    else
        return false;
}

