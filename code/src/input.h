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
    // 1: knapsack_multidrone,
    // 2: greedy_submodular_multidrone,
    // 3: generalize_bin_packing,
    // 4: coloring_multidrone,
    // 5: greedy_weight_selection,
    // 6: greedy_ending_selection,
    // 7: greedy_reward_selection,
    int algorithm = 5;

    // Number or random instances to be performed (when doing plots)
    int iterations = 1;

    // Name of the experiment (just a string to be used when loading/saving)
    string exp_name = "default";

    // Number of drones
    int num_drones = 1;

    // Number of deliveries
    int num_deliveries = 5;
};

void print_parameters(const input &);

void save_parameters(const input &);

input load_parameters(input &);

input read_parameters(input &, int, char *[]);

#endif //INPUT_H
