#ifndef DEPLOYMENT_H
#define DEPLOYMENT_H

#include <iostream>
#include <vector>
#include <random>
#include <numeric>

#include "input.h"
#include "definitions.h"

using namespace std;

class deployment {

private:
    // Delivery parameters
    int num_deliveries;
    vector<int> launches;
    vector<int> rendezvouses;
    vector<int> profits;
    vector<int> loads;
    vector<int> delivery_points;

    // Drone's parameters
    int drone_battery;
    int drone_load;

    int height;
    int energy_per_flight;

public:
    explicit deployment(const input &);
    double get_distance(int, int, int, int);
    vector<vector<int>> compute_all_flights();
    double compute_energy_cost(vector<int>);
    int get_num_deliveries() const;
    const vector<int> &get_launches() const;
    const vector<int> &get_rendezvouses() const;
    const vector<int> &get_profits() const;
    const vector<int> &get_loads() const;
    const vector<int> &get_delivery_points() const;
    int get_drone_battery() const;
    int get_drone_load() const;
    int get_height() const;
    int get_energy_per_flight() const;
};

#endif //DEPLOYMENT_H
