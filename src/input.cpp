#include "input.h"

void print_parameters(const input &par) {
    cout << "Save=" << par.save << endl;
    cout << "Seed=" << par.seed << endl;
    cout << "Experiment name=" << par.exp_name << endl;

    map<int, string> experiment_str = {
            {0, "Default parameters"},
            {1, "Loaded parameters from config file"},
            {2, "Read parameters from command line"},
    };

    map<int, string> algorithm_str = {
            {0, "ILP (OPT-unit-load)"},
            {1, "ILP (OPT-arbitrary-load)"},
            {2, "bin_s_ul (APX)"},
            {3, "bin_s (HEU)"},
            {4, "knapsack_opt_ul"},
            {5, "knapsack_heu (HEU)"},
            {6, "col_s (OPT)"},
            // {6, "col_s_unit_load (OPT)"},
            // {7, "col_s_arbitrary_load (HEU)"},
            {8, "greedy_reward_ul (HEU)"},
            {9, "greedy_reward (HEU)"},
            {10, "greedy_energy_ul (HEU)"},
            {11, "greedy_energy (HEU)"},
            {12, "greedy_reward_energy_ul (HEU)"},
            {13, "greedy_reward_energy (HEU)"},
            {14, "greedy_reward_load_ul (HEU)"},
            {15, "greedy_reward_load (HEU)"},
    };
    /// change the list

    cout << "Experiment=" << par.experiment << " (" << experiment_str[par.experiment] << ")" << endl << endl;

    cout << "Number of deliveries = " << par.num_deliveries << endl;
    cout << "Maximum length of the road = " << par.max_len_road << endl;
    cout << "Maximum length of an interval = " << par.max_interval_len << endl;
    cout << "Maximum value for a delivery = " << par.max_profit << endl;
    cout << "Maximum drone_load for a delivery = " << par.max_load << endl;
    cout << "Drone's drone_battery capacity = " << par.battery << endl;
    cout << "Drone's drone_load capacity = " << par.load << endl;
    cout << "Height of the deliveries = " << par.height << endl;
    cout << "Energy consumption of a drone per distance = " << par.energy_per_flight << endl;
    cout << "Algorithm=" << algorithm_str[par.algorithm] << endl;
    cout << "Iterations=" << par.iterations << endl;

    cout << endl << endl;
}

void save_parameters(const input &par) {
    string cfg_filename = "input/" + par.exp_name + ".cfg";
    ofstream file_cfg(cfg_filename);

    file_cfg << "save=" << par.save << endl;
    file_cfg << "seed=" << par.seed << endl;
    file_cfg << "num_deliveries=" << par.num_deliveries << endl;
    file_cfg << "max_len_road=" << par.max_len_road << endl;
    file_cfg << "max_interval_len=" << par.max_interval_len << endl;
    file_cfg << "max_profit=" << par.max_profit << endl;
    file_cfg << "max_load=" << par.max_load << endl;
    file_cfg << "drone_battery=" << par.battery << endl;
    file_cfg << "drone_load=" << par.load << endl;
    file_cfg << "height=" << par.height << endl;
    file_cfg << "energy_per_flight=" << par.energy_per_flight << endl;
    file_cfg << "algorithm=" << par.algorithm << endl;
    file_cfg << "iterations=" << par.iterations << endl;

    file_cfg << endl;

    file_cfg.close();
}

input load_parameters(input &par) {
    string cfg_filename = "input/" + par.exp_name + ".cfg";
    ifstream file_cfg(cfg_filename);

    // Check if the file exists and can be opened
    if (!file_cfg.is_open()) {
        cerr << "Error opening config file: " << cfg_filename << endl;
        exit(-1);
    }

    string line;
    while (getline(file_cfg, line)) {
        stringstream lineStream(line);
        string key;
        string value;

        if (getline(lineStream, key, '=') && lineStream >> value) {
            if (key == "save") {
                par.save = stoi(value);
            } else if (key == "seed") {
                par.seed = stoi(value);
            } else if (key == "num_deliveries") {
                par.num_deliveries = stoi(value);
            } else if (key == "algorithm") {
                par.algorithm = stoi(value);
            } else if (key == "iterations") {
                par.iterations = stoi(value);
            } else if (key == "max_len_road") {
                par.max_len_road = stoi(value);
            } else if (key == "max_interval_len") {
                par.max_interval_len = stoi(value);
            } else if (key == "max_profit") {
                par.max_profit = stoi(value);
            } else if (key == "max_load") {
                par.max_load = stoi(value);
            } else if (key == "drone_battery") {
                par.battery = stoi(value);
            } else if (key == "drone_load") {
                par.load = stoi(value);
            } else if (key == "height") {
                par.height = stoi(value);
            } else if (key == "energy_per_flight") {
                par.energy_per_flight = stoi(value);
            }
        }
    }

    file_cfg.close();

    return par;
}

input read_parameters(input &par, int argc, char *argv[]) {
    // Iterate through command line arguments
    // Start from 2 to skip program name and --params
    for (int i = 2; i < argc; ++i) {
        string arg = argv[i];

        // Check if the argument is a flag (e.g., starts with '-')
        if (!arg.empty() && arg[0] == '-') {
            // Process the flag or option accordingly
            if (arg == "-exp_name") {
                par.exp_name = argv[i + 1];
            } else if (arg == "-save") {
                par.save = stoi(argv[i + 1]);
            } else if (arg == "-seed") {
                par.seed = stoi(argv[i + 1]);
            } else if (arg == "-num_deliveries") {
                par.num_deliveries = stoi(argv[i + 1]);
            } else if (arg == "-algorithm") {
                par.algorithm = stoi(argv[i + 1]);
            } else if (arg == "-iterations") {
                par.iterations = stoi(argv[i + 1]);
            } else if (arg == "-max_len_road") {
                par.max_len_road = stoi(argv[i + 1]);
            } else if (arg == "-max_interval_len") {
                par.max_interval_len = stoi(argv[i + 1]);
            } else if (arg == "-max_profit") {
                par.max_profit = stoi(argv[i + 1]);
            } else if (arg == "-max_load") {
                par.max_load = stoi(argv[i + 1]);
            } else if (arg == "-drone_battery") {
                par.battery = stoi(argv[i + 1]);
            } else if (arg == "-drone_load") {
                par.load = stoi(argv[i + 1]);
            } else if (arg == "-height") {
                par.height = stoi(argv[i + 1]);
            } else if (arg == "-energy_per_flight") {
                par.energy_per_flight = stoi(argv[i + 1]);
            } else {
                cerr << "Unknown option: " << arg << endl;
            }
        }
    }

    return par;
}
