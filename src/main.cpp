#include <iostream>
#include <chrono>

#include "input.h"
#include "output.h"
#include "algorithms.h"

using namespace std;

void run_experiment(input &par) {
    print_parameters(par);

    vector<solution> outputs;
    for (int i = 0; i < par.iterations; i++) {
        // Deployment creation with respect to the input parameters
        deployment dep(par);

        cout << "Iteration: " << (i + 1) << "/" << par.iterations << endl;

        algorithms alg(&dep);

        auto start_time = chrono::high_resolution_clock::now();

        solution out = alg.run_experiment(par.algorithm);

        auto end_time = chrono::high_resolution_clock::now();
        auto duration = chrono::duration_cast<chrono::milliseconds>(end_time - start_time);
        out.running_time = static_cast<double>(duration.count()) / 1e+3;

        cout << "|--------------------------|" << endl;
        cout << out << endl;

        outputs.push_back(out);
    }

    if (par.save == 1) {
        save_output(par, outputs);
    }
}

int main(int argc, char **argv) {
    // Set global precision for cout
    cout << fixed << setprecision(2);

    input par;

    if (argc > 1) {
        string option = argv[1];

        // Choose the behavior based on the command line argument
        if (option == "--file") {
            if (argc > 2) {
                // Read from config file
                cout << "Reading parameters from file" << endl << endl;
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
            cout << "Reading command line parameters" << endl << endl;
            read_parameters(par, argc, argv);
        } else {
            cerr << "Unknown option: " << option << endl;
            exit(-1);
        }
    } else {
        par.experiment = 0;
        cout << "Loading default parameters" << endl << endl;
    }

    run_experiment(par);

    return 0;
}
