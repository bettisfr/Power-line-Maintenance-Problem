#include <iostream>
#include <iomanip>
#include <chrono>

#include "io/input.h"
#include "io/output.h"
#include "core/algorithms.h"

using namespace std;
using namespace chrono;

void run_experiment(input&);
void run_test(input&);

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

    // Fake iteration numbers to execute this test part!
    if (par.iterations == -1) {
        run_test(par);
    } else {
        run_experiment(par);
    }

    return 0;
}

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

        // Print iteration number on the same line
        cout << "\rIteration: " << (i + 1) << "/" << par.iterations << flush;

        algorithms alg(dep);

        auto start_time = high_resolution_clock::now();

        solution out = alg.run_experiment(par.algorithm);

        if (out.total_energy > par.drone_battery) {
            cerr << "Error: Used energy is=" << out.total_energy << " while total budget is=" << par.drone_battery << endl;
	        exit(-1);
        }

        auto end_time = high_resolution_clock::now();
        auto duration = duration_cast<milliseconds>(end_time - start_time);
        out.running_time = static_cast<double>(duration.count()) / 1e+3;

        if (par.log == 1) {
            cout << endl << out << endl;
        }

        outputs.push_back(out);
    }

    // Print a newline after the iterations are done to avoid overwriting the last line
    cout << endl;

    if (par.save == 1) {
        save_output(par, outputs);
    }
}

void run_test(input &par) {
//    if (par.log == 1) {
//        print_parameters(par);
//    }

    int max_seed = 10000;
    for (int i = 361; i < max_seed; i++) {
        cout << "Seed: " << (i) << "/" << max_seed << endl;

        vector<solution> solutions;
        vector<int> vals = {3, 4};
        for (int j : vals) {
            par.solution_space = j;
            par.seed = i;

            // WARNING: if using this, you must remove "static" on line 21 in deployment.cpp
            // static mt19937 g(seed);                          <<<---------------- EYE!!!!!!
            // When done with this, please readd static         <<<---------------- EYE!!!!!!
            deployment dep(par);  /////////////////////////////////////////

        //    if (par.log == 1) {
        //        cout << dep << endl;
        //    }

            algorithms alg(dep);

            solution out = alg.run_experiment(par.algorithm);
            solutions.push_back(out);

//            if (par.log == 1) {
//                cout << out << endl;
//            }
        }

        if (solutions[0].total_profit != solutions[1].total_profit) {
            cerr << "Error - Found counterexample" << endl;

            cout << "Exhaustive=" << solutions[0] << endl;
            cout << "DP=" << solutions[1] << endl;

            exit(-1);
        }
    }
}
