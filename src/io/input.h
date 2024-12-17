#ifndef INPUT_H
#define INPUT_H

#include <iostream>
#include <ctime>
#include <cstdlib>
#include <map>
#include <fstream>
#include <sstream>
#include <vector>

using namespace std;

// Default values
struct input {
    // 0: No prints; 1: Prints
    int log = 1;

    // 0: No save to CSV; 1: Save to CSV
    int save = 0;

    // Application parameters
    int seed = 0;

    // 0: Default values; 1: From cfg file; 2 From command line
    int experiment = 0;

    // opt_ilp 0
    // bin_packing 1
    // knapsack 2
    // coloring 3
    // greedy_reward 4
    // greedy_energy 5
    // greedy_reward_energy 6
    // greedy_reward_load 7
    // max_profit_extended 8
    int algorithm = 8;

    // Number or random instances to be performed (when doing plots)
    int iterations = 1;

    // Name of the experiment (just a string to be used when loading/saving)
    string exp_name = "default";

    // Number of deliveries
    int num_deliveries = 15;

    // Maximum length of the road
    int max_len_road = 100;

    // Maximum length of an interval
    int max_interval_len = 15;

    // Maximum value for a delivery
    int max_profit = 10;

    // Maximum weight for a delivery
    int max_weight = 1;

    // Drone's energy drone_battery
    int drone_battery = 5000;

    // Drone's drone_load capacity
    int drone_load = 8;

    // Height of the deliveries
    double height = 0.5;

    // Energy consumption of a drone per distance
    double energy_unit_cost = 200;
};

void print_parameters(const input &);

void save_parameters(const input &);

input load_parameters(input &);

input read_parameters(input &, int, char *[]);

#endif //INPUT_H
