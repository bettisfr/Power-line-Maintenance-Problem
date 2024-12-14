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

    int compute_profit(const vector<int> &);

    int compute_load(const vector<int> &);

    tuple<vector<vector<int>>, vector<double>> compute_all_flights_unitary_weight(const vector<int> &, const int &);

    tuple<vector<vector<int>>, vector<double>> compute_all_flights_arbitrary_weight();

    double compute_energy(const vector<int> &delivery_ids);

    void find_subsets(vector<int> &v, int idx, vector<int> &subset, set<vector<int>> &result);

    vector<vector<int>> compute_all_subsets(vector<int> &);

    tuple<vector<vector<int>>, vector<double>> compute_all_flights_arbitrary_weight_limited();

};

#endif //DEPLOYMENT_H
