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
            {0, "opt ilp (OPT)"},
            {1, "knapsack multidrone (KNA)"},
            {2, "greedy submodular multidrone (MR)"},
            {3, "generalize bin packing (BIN)"},
            {4, "coloring multidrone (COL)"},
            {5, "greedy weight selection (GSw)"},
            {6, "greedy ending selection (GeRt)"},
            {7, "greedy reward selection (GLp)"},
    };

    cout << "Experiment=" << par.experiment << " (" << experiment_str[par.experiment] << ")" << endl << endl;

    cout << "Number of drones=" << par.num_drones << endl;
    cout << "Number of deliveries=" << par.num_deliveries << endl;

    cout << "Algorithm=" << algorithm_str[par.algorithm] << endl;
    cout << "Iterations=" << par.iterations << endl;

    cout << endl << endl;
}

void save_parameters(const input &par) {
    string cfg_filename = "input/" + par.exp_name + ".cfg";
    ofstream file_cfg(cfg_filename);

    file_cfg << "save=" << par.save << endl;
    file_cfg << "seed=" << par.seed << endl;
    file_cfg << "num_drones=" << par.num_drones << endl;
    file_cfg << "num_deliveries=" << par.num_deliveries << endl;
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
            } else if (key == "num_drones") {
                par.num_drones = stoi(value);
            } else if (key == "num_deliveries") {
                par.num_deliveries = stoi(value);
            } else if (key == "algorithm") {
                par.algorithm = stoi(value);
            } else if (key == "iterations") {
                par.iterations = stoi(value);
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
            } else if (arg == "-num_drones") {
                par.num_drones = stoi(argv[i + 1]);
            } else if (arg == "-num_deliveries") {
                par.num_deliveries = stoi(argv[i + 1]);
            } else if (arg == "-algorithm") {
                par.algorithm = stoi(argv[i + 1]);
            } else if (arg == "-iterations") {
                par.iterations = stoi(argv[i + 1]);
            } else {
                cerr << "Unknown option: " << arg << endl;
            }
        }
    }

    return par;
}
