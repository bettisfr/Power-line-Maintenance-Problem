#ifndef DEPLOYMENT_H
#define DEPLOYMENT_H

#include <iostream>
#include <vector>
#include <random>
#include <numeric>
#include <algorithm>
#include <set>

#include "../io/input.h"

using namespace std;

class deployment {

private:
    // Delivery parameters
    int num_deliveries;
    vector<int> launches;
    vector<int> rendezvouses;
    vector<int> profits;
    vector<int> weights;
    vector<int> delivery_points;

    // Drone's parameters
    int drone_battery;
    int drone_load;

    double height;
    double energy_unit_cost;
    bool unit_weight;
    int solution_space;

public:
    [[nodiscard]] bool is_unit_weight() const;

    explicit deployment(const input &);

    [[nodiscard]] int get_num_deliveries() const;

    [[nodiscard]] const vector<int> &get_launches() const;

    [[nodiscard]] const vector<int> &get_rendezvouses() const;

    [[nodiscard]] const vector<int> &get_profits() const;

    [[nodiscard]] const vector<int> &get_weights() const;

    [[nodiscard]] const vector<int> &get_delivery_points() const;

    [[nodiscard]] int get_drone_battery() const;

    [[nodiscard]] int get_drone_load() const;

    [[nodiscard]] int get_solution_space() const;

    tuple<vector<vector<int>>, vector<double>, vector<int>, vector<int>> compute_solution_space();

    static bool check_intersection(const vector<int> &, vector<int>);

    static vector<int> largest_non_overlap_delivery(vector<int> launches, vector<int> rendezvouses);

    tuple<int, int> compute_LR(const vector<int> &);

    int compute_profit(const vector<int> &);

    int compute_load(const vector<int> &);

    tuple<vector<vector<int>>, vector<double>, vector<int>, vector<int>, vector<int>> sorting_with_rendezvouses_in_apx();

    static bool check_correct_interval(const vector<vector<int>> &, vector<int>, vector<int>, int, int);

    static vector<int>
    weighted_interval(const vector<int> &, const vector<int> &, const vector<int> &, vector<int>, const vector<int> &);

    static int compute_opt(int, const vector<int> &, const vector<int> &, vector<int>, vector<int>, vector<int>);

    static vector<int>
    find_solution(int, const vector<int> &, const vector<int> &, vector<int>, vector<int>, vector<int>, vector<int>);

    tuple<vector<vector<int>>, vector<double>, vector<int>, vector<int>> compute_all_flights_unitary_weight();

    tuple<vector<vector<int>>, vector<double>, vector<int>, vector<int>> compute_all_flights_arbitrary_weight();

    double compute_energy(const vector<int> &delivery_ids);

    void find_subsets(vector<int> &v, int idx, vector<int> &subset, set<vector<int>> &result);

    vector<vector<int>> compute_all_subsets(vector<int> &);

    vector<int> compute_flight_using_knapsack(vector<int>, int);

    tuple<vector<vector<int>>, vector<double>, vector<int>, vector<int>> compute_all_flights_using_knapsack();

    tuple<vector<vector<int>>, vector<double>, vector<int>, vector<int>> compute_all_flights_new();

    //tuple<vector<vector<int>>, vector<double>> compute_all_flights_arbitrary_weight_limited();

    friend ostream &operator<<(ostream &os, const deployment &dep) {
        for (int i = 0; i < dep.num_deliveries; i++) {
            os << i << "-> " << dep.delivery_points[i]
                 << ": [" << dep.launches[i]
                 << ", " << dep.rendezvouses[i]
                 << "], profit=" << dep.profits[i]
                 << ", weight=" << dep.weights[i] << endl;
        }

        return os;
    }
};

#endif //DEPLOYMENT_H
