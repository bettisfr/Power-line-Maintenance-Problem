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

    // Drone's parameters
    int drone_battery;
    int drone_load;

public:
    explicit deployment(const input &);
    int get_num_deliveries() const;
    const vector<int> &get_launches() const;
    const vector<int> &get_rendezvouses() const;
    const vector<int> &get_profits() const;
    const vector<int> &get_loads() const;
    int get_drone_battery() const;
    int get_drone_load() const;
};

#endif //DEPLOYMENT_H
