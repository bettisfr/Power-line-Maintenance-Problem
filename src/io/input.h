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
    // 360
    // 2100
    int seed = 2100;

    // 0: Default values; 1: From cfg file; 2 From command line
    int experiment = 0;

    // opt_multi - OPT - 0
    // bin_packing - 1
    // knapsack - KNA - 2
    // coloring - COL - 3
    // greedy_max_profit 4
    // greedy_max_energy 5
    // greedy_max_profit/energy - GMP/E - 6
    // greedy_max_profit_load 7
    // opt_single - OPTs - 8
    int algorithm = 0;

    // Number or random instances to be performed (when doing plots)
    int iterations = 1;                                         

    // Name of the experiment (just a string to be used when loading/saving)
    string exp_name = "default";

    // Number of deliveries
    int num_deliveries = 10;

    // Maximum length of the road (in km)
    int max_len_road = 100;

    // Maximum length of an interval (in km)
    double max_interval_len = 8;

    // Maximum value for a delivery
    int max_profit = 10;

    // Maximum weight for a delivery (in kg)
    int max_weight = 5;

    // How the solution space is computed
    // 0 - no additional cost - exhaustively (optimal),
    // 1 - no additional cost - knapsack (optimal),
    // 2 - no additional cost - sorting (not optimal) [optimal if costs are unitary],
    // 3 - with additional cost - exhaustively (optimal),
    // 4 - with additional cost - DP (optimal)
    int solution_space = 3;

    // Drone's energy drone_battery (in kJ)
    int drone_battery = 5000;

    // Drone's drone_load capacity (in kg)
    int drone_load = 10;

    // Height of the deliveries (in km)
    double height = 0.05;

    // Distance from the road (in km)
    double distance = 0.5;

    // Energy consumption of a drone per distance (normally it is 150 J/m, so 150 kJ/km) (in kJ)
    double energy_unit_cost = 150;

    // Energy consumption to drop the delivery at the delivery location (assumes 100 seconds at 300 J/s) (in kJ)
    double energy_per_delivery = 30;

    // If the trellises are located at a regular distance from each other (1), otherwise random (0)
    int regularly_spaced = 0;

    // Starting point of deliveries, if regularly_spaced = 1 (in km)
    double deliveries_starting_point = 0.5;
    double error = 0.05;

    // For Zipf distribution
    // 0 - uniform
    // 1 - logarithmic
    // 2 - exponential
    double exponent = 0;
};

bool check_parameters(const input &);

void print_parameters(const input &);

void save_parameters(const input &);

input load_parameters(input &);

input read_parameters(input &, int, char *[]);

#endif //INPUT_H
