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
            {0, "OPT (ILP)"},
            {1, "bin packing"},
            {2, "knapsack"},
            {3, "coloring"},
            {4, "Gmax profit"},
            {5, "Gmax energy"},
            {6, "Gmax profit/energy"},
            {7, "Gmax profit/load"},
            {8, "Gmax profit+neigh"},
    };

    map<int, string> solution_space_str = {
            {0, "No additional cost - Exhaustively"},
            {1, "No additional cost - Knapsack"},
            {2, "No additional cost - Same weight"},
            {3, "Additional cost - Exhaustively"},
            {4, "Additional cost - DP"},
    };

    cout << "Experiment=" << par.experiment << " (" << experiment_str[par.experiment] << ")" << endl << endl;

    cout << "Number of deliveries=" << par.num_deliveries << endl;
    cout << "Maximum length of the road=" << par.max_len_road << endl;
    cout << "Maximum length of an interval=" << par.max_interval_len << endl;
    cout << "Maximum value for a delivery=" << par.max_profit << endl;
    cout << "Maximum weight for a delivery=" << par.max_weight << endl;
    cout << "Drone's energy battery=" << par.drone_battery << endl;
    cout << "Drone's load capacity=" << par.drone_load << endl;
    cout << "Height of the deliveries=" << par.height << endl;
    cout << "Distance from the road=" << par.distance << endl;
    cout << "Energy unit cost per distance=" << par.energy_unit_cost << endl;
    cout << "Energy per delivery=" << par.energy_per_delivery << endl;
    cout << "Deliveries are regularly spaced=" << par.regularly_spaced << endl;
    cout << "Starting location of deliveries (regularly_spaced)=" << par.deliveries_starting_point << endl;
    cout << "Error in deliveries locations (regularly_spaced)=" << par.error << endl;
    cout << "Algorithm=" << algorithm_str[par.algorithm] << endl;
    cout << "Solution space=" << solution_space_str[par.solution_space] << endl;
    cout << "Iterations=" << par.iterations << endl;

    cout << endl << endl;
}

void save_parameters(const input &par) {
    const string cfg_filename = "input/" + par.exp_name + ".cfg";
    ofstream file_cfg(cfg_filename);

    file_cfg << "log=" << par.log << endl;
    file_cfg << "save=" << par.save << endl;
    file_cfg << "seed=" << par.seed << endl;
    file_cfg << "num_deliveries=" << par.num_deliveries << endl;
    file_cfg << "max_len_road=" << par.max_len_road << endl;
    file_cfg << "max_interval_len=" << par.max_interval_len << endl;
    file_cfg << "max_profit=" << par.max_profit << endl;
    file_cfg << "max_weight=" << par.max_weight << endl;
    file_cfg << "drone_battery=" << par.drone_battery << endl;
    file_cfg << "drone_load=" << par.drone_load << endl;
    file_cfg << "height=" << par.height << endl;
    file_cfg << "distance=" << par.distance << endl;
    file_cfg << "energy_unit_cost=" << par.energy_unit_cost << endl;
    file_cfg << "energy_per_delivery=" << par.energy_per_delivery << endl;
    file_cfg << "algorithm=" << par.algorithm << endl;
    file_cfg << "iterations=" << par.iterations << endl;
    file_cfg << "solution_space=" << par.solution_space << endl;
    file_cfg << "regularly_spaced=" << par.regularly_spaced << endl;
    file_cfg << "deliveries_starting_point=" << par.deliveries_starting_point << endl;
    file_cfg << "error=" << par.error << endl;
    file_cfg << "exponent=" << par.exponent << endl;
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
            if (key == "log") {
                par.log = stoi(value);
            } else if (key == "exp_name") {
                par.exp_name = value;
            } else if (key == "save") {
                par.save = stoi(value);
            } else if (key == "save") {
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
                par.max_interval_len = stod(value);
            } else if (key == "max_profit") {
                par.max_profit = stoi(value);
            } else if (key == "max_weight") {
                par.max_weight = stoi(value);
            } else if (key == "drone_battery") {
                par.drone_battery = stoi(value);
            } else if (key == "drone_load") {
                par.drone_load = stoi(value);
            } else if (key == "height") {
                par.height = stod(value);
            } else if (key == "distance") {
                par.distance = stod(value);
            } else if (key == "energy_unit_cost") {
                par.energy_unit_cost = stod(value);
            } else if (key == "energy_per_delivery") {
                par.energy_per_delivery = stod(value);
            } else if (key == "solution_space") {
                par.solution_space = stoi(value);
            } else if (key == "regularly_spaced") {
                par.regularly_spaced = stoi(value);
            } else if (key == "deliveries_starting_point") {
                par.deliveries_starting_point = stod(value);
            } else if (key == "error") {
                par.error = stod(value);
            } else if (key == "exponent") {
                par.exponent = stoi(value);
            }
        }
    }

    file_cfg.close();

    return par;
}

input read_parameters(input &par, const int argc, char *argv[]) {
    // Iterate through command line arguments
    // Start from 2 to skip program name and --params
    for (int i = 2; i < argc; ++i) {
        string arg = argv[i];

        // Check if the argument is a flag (e.g., starts with '-')
        if (!arg.empty() && arg[0] == '-') {
            // Process the flag or option accordingly
            if (arg == "-exp_name") {
                par.exp_name = argv[i + 1];
            } else if (arg == "-log") {
                par.log = stoi(argv[i + 1]);
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
                par.max_interval_len = stod(argv[i + 1]);
            } else if (arg == "-max_profit") {
                par.max_profit = stoi(argv[i + 1]);
            } else if (arg == "-max_weight") {
                par.max_weight = stoi(argv[i + 1]);
            } else if (arg == "-drone_battery") {
                par.drone_battery = stoi(argv[i + 1]);
            } else if (arg == "-drone_load") {
                par.drone_load = stoi(argv[i + 1]);
            } else if (arg == "-height") {
                par.height = stod(argv[i + 1]);
            } else if (arg == "-distance") {
                par.distance = stod(argv[i + 1]);
            } else if (arg == "-energy_unit_cost") {
                par.energy_unit_cost = stod(argv[i + 1]);
            } else if (arg == "-energy_per_delivery") {
                par.energy_per_delivery = stod(argv[i + 1]);
            } else if (arg == "-solution_space") {
                par.solution_space = stoi(argv[i + 1]);
            } else if (arg == "-regularly_spaced") {
                par.regularly_spaced = stoi(argv[i + 1]);
            } else if (arg == "-deliveries_starting_point") {
                par.deliveries_starting_point = stod(argv[i + 1]);
            } else if (arg == "-error") {
                par.error = stod(argv[i + 1]);
            } else if (arg == "-exponent") {
                par.exponent = stoi(argv[i + 1]);
            } else {
                cerr << "Unknown option: " << arg << endl;
            }
        }
    }

    return par;
}

bool check_parameters(const input &par) {
    bool error = false;

    if (par.seed < 0)
        cerr << "Error: seed cannot be negative" << endl, error = true;

    if (par.algorithm < 0 || par.algorithm > 9)
        cerr << "Error: algorithm index out of range (0-9)" << endl, error = true;

    if (par.iterations <= 0)
        cerr << "Error: iterations must be > 0" << endl, error = true;

    if (par.num_deliveries <= 0)
        cerr << "Error: num_deliveries must be > 0" << endl, error = true;

    if (par.max_len_road <= 0)
        cerr << "Error: max_len_road must be > 0" << endl, error = true;

    if (par.max_interval_len <= 0.0 || par.max_interval_len > par.max_len_road)
        cerr << "Error: max_interval_len must be > 0 and <= max_len_road" << endl, error = true;

    if (par.max_profit <= 0)
        cerr << "Error: max_profit must be > 0" << endl, error = true;

    if (par.max_weight <= 0)
        cerr << "Error: max_weight must be > 0" << endl, error = true;

    if (par.solution_space < 0 || par.solution_space > 4)
        cerr << "Error: solution_space must be in [0, 4]" << endl, error = true;

    if (par.drone_battery <= 0)
        cerr << "Error: drone_battery must be > 0" << endl, error = true;

    if (par.drone_load <= 0)
        cerr << "Error: drone_load must be > 0" << endl, error = true;

    if (par.height < 0)
        cerr << "Error: height cannot be negative" << endl, error = true;

    if (par.distance < 0)
        cerr << "Error: distance cannot be negative" << endl, error = true;

    if (par.energy_unit_cost <= 0)
        cerr << "Error: energy_unit_cost must be > 0" << endl, error = true;

    if (par.energy_per_delivery < 0)
        cerr << "Error: energy_per_delivery cannot be negative" << endl, error = true;

    if (par.regularly_spaced != 0 && par.regularly_spaced != 1)
        cerr << "Error: regularly_spaced must be 0 or 1" << endl, error = true;

    if (par.deliveries_starting_point < 0)
        cerr << "Error: deliveries_starting_point cannot be negative" << endl, error = true;

    if (par.error < 0.0 || par.error >= 1.0)
        cerr << "Error: error must be in [0, 1)" << endl, error = true;

    if (par.exponent < 0 || par.exponent > 2)
        cerr << "Error: exponent must be 0 (uniform), 1 (log), or 2 (exp)" << endl, error = true;

    return error;
}

