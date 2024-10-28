#include <random>
#include <iostream>
#include "deployment.h"
#include "algorithms.h"

using namespace std;

double numpy(mt19937& g) {
    int a = g() >> 5;
    int b = g() >> 6;
    double value = (a * 67108864.0 + b) / 9007199254740992.0;
    return value;
}

deployment::deployment(const input &par) {
    // creating random instance by using par.seed
    int seed = par.seed;
    mt19937 g(seed);

    num_deliveries = par.num_deliveries;
    int max_len_road = par.max_len_road;
    int max_interval_len = par.max_interval_len;
    int max_profit = par.max_profit;
    int max_load = par.max_load;
    
    height = par.height;
    energy_per_flight = par.energy_per_flight;

    drone_battery = par.battery;
    drone_load = par.load;

    for (int i = 0; i < num_deliveries; i++) {
        int departure = static_cast<int>(numpy(g) * max_len_road);
        int arrival = static_cast<int>(departure + numpy(g) * max_interval_len);
        if (arrival == departure) {
            arrival++;
        }
        if (arrival > max_len_road) {
            arrival = max_len_road;
        }

        int profit = static_cast<int>(numpy(g) * max_profit) + 1;
        int load = static_cast<int>(numpy(g) * max_load) + 1;

        int x = arrival - departure;
        int delivery_location = departure + static_cast<int>(numpy(g) * x) + 1;

        delivery_points.push_back(delivery_location);
        launches.push_back(departure);
        rendezvouses.push_back(arrival);
        profits.push_back(profit);
        loads.push_back(load);

        cout << i << "-> " << delivery_location
             << ": [" << departure
             << ", " << arrival
             << "], profit=" << profit
             << ", load=" << load << endl;
    }

    cout << endl;
}

int deployment::get_num_deliveries() const {
    return num_deliveries;
}

const vector<int> &deployment::get_launches() const {
    return launches;
}

const vector<int> &deployment::get_rendezvouses() const {
    return rendezvouses;
}

const vector<int> &deployment::get_profits() const {
    return profits;
}

const vector<int> &deployment::get_loads() const {
    return loads;
}
const vector<int> &deployment::get_delivery_points() const {
    return delivery_points;
}
int deployment::get_drone_battery() const {
    return drone_battery;
}

int deployment::get_drone_load() const {
    return drone_load;
}

int deployment::get_height() const {
    return height;
}

int deployment::get_energy_per_flight() const {
    return energy_per_flight;
}
