#include <iostream>
#include <iomanip>
#include <chrono>

#include "io/input.h"
#include "io/output.h"
#include "core/algorithms.h"

using namespace std;
using namespace chrono;

void run_experiment(input &par) {
    if (par.log == 1) {
        print_parameters(par);
    }

    vector<solution> outputs;
    for (int i = 0; i < par.iterations; i++) {
        // Deployment creation with respect to the input parameters
        deployment dep(par);
        if (par.log == 1) {
            cout << dep << endl;
        }

        cout << "Iteration: " << (i + 1) << "/" << par.iterations << endl;

        algorithms alg(dep);

        auto start_time = high_resolution_clock::now();

        solution out = alg.run_experiment(par.algorithm);

        auto end_time = high_resolution_clock::now();
        auto duration = duration_cast<milliseconds>(end_time - start_time);
        out.running_time = static_cast<double>(duration.count()) / 1e+3;

        if (par.log == 1) {
            cout << out << endl;
        }

        outputs.push_back(out);
    }

    if (par.save == 1) {
        save_output(par, outputs);
    }
}

int main(int argc, char **argv) {
    // Set global precision for printing
    cout << fixed << setprecision(2);

    input par;

    if (argc > 1) {
        string option = argv[1];

        // Choose the behavior based on the command line argument
        if (option == "--file") {
            if (argc > 2) {
                // Read from config file
                cout << "Reading parameters from file" << endl;
                par.experiment = 1;
                par.exp_name = argv[2];
                par = load_parameters(par);
            } else {
                cerr << "Error: File name not provided." << endl;
                exit(-1);
            }
        } else if (option == "--params") {
            // Read from command line parameters
            par.experiment = 2;
            cout << "Reading command line parameters" << endl;
            read_parameters(par, argc, argv);
        } else {
            cerr << "Unknown option: " << option << endl;
            exit(-1);
        }
    } else {
        par.experiment = 0;
        cout << "Loading default parameters" << endl;
    }

    run_experiment(par);

    return 0;
}
