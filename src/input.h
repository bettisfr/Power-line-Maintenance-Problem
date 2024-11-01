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
    // 0: No save to CSV; 1: Save to CSV
    int save = 0;

    // Application parameters
    int seed = 0;

    // 0: Default values; 1: From cfg file; 2 From command line
    int experiment = 0;

    // 0: opt_ilp,
    // 1: heuristic_1,
    // 2: heuristic_2,
    // 3: Bin_S,
    // 4: knapsack,
    // 5: col-s,
    int algorithm = 4;

    // Number or random instances to be performed (when doing plots)
    int iterations = 1;

    // Name of the experiment (just a string to be used when loading/saving)
    string exp_name = "default";

    // Number of deliveries
    int num_deliveries = 10;

    // Maximum length of the road
    int max_len_road = 1000;

    // Maximum length of an interval
    int max_interval_len = 100;

    // Maximum value for a delivery
    int max_profit = 15;

    // Maximum drone_load for a delivery
    int max_load = 1;

    // Drone's energy drone_battery
    int battery = 60000;

    // Drone's drone_load capacity
    int load = 4;

    // Height of the deliveries
    int height = 20;

    //Energy consumption of a drone per distance 
    int energy_per_flight = 150;
};

void print_parameters(const input &);

void save_parameters(const input &);

input load_parameters(input &);

input read_parameters(input &, int, char *[]);

#endif //INPUT_H
