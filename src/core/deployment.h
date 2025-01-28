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
    vector<double> launches;
    vector<double> rendezvouses;
    vector<int> profits;
    vector<int> weights;
    vector<double> delivery_points;

    // Drone's parameters
    int drone_battery;
    int drone_load;

    double height;
    double distance;
    double energy_unit_cost;
    bool unit_weight;
    int solution_space;
    double energy_per_delivery;

public:
    [[nodiscard]] bool is_unit_weight() const;

    explicit deployment(const input &);

    [[nodiscard]] int get_num_deliveries() const;

    [[nodiscard]] const vector<double> &get_launches() const;

    [[nodiscard]] const vector<double> &get_rendezvouses() const;

    [[nodiscard]] const vector<int> &get_profits() const;

    [[nodiscard]] const vector<int> &get_weights() const;

    [[nodiscard]] const vector<double> &get_delivery_points() const;

    [[nodiscard]] int get_drone_battery() const;

    [[nodiscard]] int get_drone_load() const;

    [[nodiscard]] int get_solution_space() const;

    tuple<vector<vector<int>>, vector<double>, vector<int>, vector<int>> compute_solution_space();

    //static bool check_intersection(const vector<int> &, vector<int>);

    static vector<int> largest_non_overlap_delivery(vector<double>, vector<double>);

    tuple<double, double> compute_LR(const vector<int> &);

    int compute_profit(const vector<int> &);

    int compute_load(const vector<int> &);

    tuple<vector<vector<int>>, vector<double>, vector<int>, vector<double>, vector<double>> sorting_with_rendezvouses_in_apx();

    static bool check_correct_interval(const vector<vector<int>> &, vector<double>, vector<double>, double, double);

    static vector<int>
    weighted_interval(const vector<double> &, const vector<double> &, const vector<int> &, vector<int>, const vector<int> &);

    static int compute_opt(int, const vector<double> &, const vector<double> &, vector<int>, vector<int>, vector<int>);

    static vector<int>
    find_solution(int, const vector<double> &, const vector<double> &, vector<int>, vector<int>, vector<int>, vector<int>);

    tuple<vector<vector<int>>, vector<double>, vector<int>, vector<int>> compute_all_flights_equal_weight();

    tuple<vector<vector<int>>, vector<double>, vector<int>, vector<int>> compute_individual_deliveries();

    //tuple<vector<vector<int>>, vector<double>, vector<int>, vector<int>> compute_all_flights_arbitrary_weight();

    double compute_energy(const vector<int> &);

    void find_subsets(vector<int> &, int, vector<int> &, set<vector<int>> &);

    vector<vector<int>> compute_all_subsets(vector<int> &);

    vector<int> compute_flight_using_knapsack(vector<int>, int);

    tuple<vector<vector<int>>, vector<double>, vector<int>, vector<int>> compute_all_flights_using_knapsack();

    tuple<vector<vector<int>>, vector<double>, vector<int>, vector<int>> compute_all_flights();

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
