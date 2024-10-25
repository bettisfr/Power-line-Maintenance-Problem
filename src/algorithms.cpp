#include "algorithms.h"
#include "definitions.h"
#include <vector>
#include <algorithm>
#include <cmath>

#include "gurobi_c++.h"

algorithms::algorithms(deployment *m_dep) {
    dep = m_dep;
}

// Useless (and stupid) method, but it was nice to use a vector of pointers to methods :-D
solution algorithms::run_experiment(int algorithm) {
    int index = algorithm;
    solution out;

    if (index >= 0 && index <= 2) {
        out = algorithm_functions[index](*this);
    } else {
        cerr << "Invalid algorithm index." << endl;
    }

    return out;
}


solution algorithms::opt_ilp() {
    solution sol;

    try {

        // Create an environment
        GRBEnv env = GRBEnv(true);
//        env.set("LogFile", "mip1.log");
        env.start();

        // Create an empty model
        GRBModel model = GRBModel(env);

        // Create variables
        GRBVar x = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "x");
        GRBVar y = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "y");
        GRBVar z = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "z");

        // Set objective: maximize x + y + 2 z
        model.setObjective(x + y + 2 * z, GRB_MAXIMIZE);

        // Add constraint: x + 2 y + 3 z <= 4
        model.addConstr(x + 2 * y + 3 * z <= 4, "c0");

        // Add constraint: x + y >= 1
        model.addConstr(x + y >= 1, "c1");

        // Optimize model
        model.optimize();

        cout << x.get(GRB_StringAttr_VarName) << " "
             << x.get(GRB_DoubleAttr_X) << endl;
        cout << y.get(GRB_StringAttr_VarName) << " "
             << y.get(GRB_DoubleAttr_X) << endl;
        cout << z.get(GRB_StringAttr_VarName) << " "
             << z.get(GRB_DoubleAttr_X) << endl;

        cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << endl;

    } catch(GRBException e) {
        cout << "Error code = " << e.getErrorCode() << endl;
        cout << e.getMessage() << endl;
    } catch(...) {
        cout << "Exception during optimization" << endl;
    }

    return sol;
}

solution algorithms::heuristic_1() {
    solution sol;

    // Use here the important parameters
    int n = dep->get_num_deliveries();
    vector<int> launches = dep->get_launches();
    vector<int> rendezvouses = dep->get_rendezvouses();
    vector<int> profits = dep->get_profits();
    vector<int> loads = dep->get_loads();
    int B = dep->get_drone_battery();
    int L = dep->get_drone_load();

    // Example of how to fill the solution
    vector<vector<int>> selected;

    vector<int> s1;
    s1.push_back(3);
    selected.push_back(s1);

    vector<int> s2;
    s2.push_back(1);
    s2.push_back(4);
    s2.push_back(5);
    selected.push_back(s2);

    vector<int> s3;
    s3.push_back(8);
    selected.push_back(s3);

    sol.selected_intervals = selected;
    sol.total_energy_cost = 199;
    sol.total_profit = 144;

    return sol;
}

solution algorithms::heuristic_2() {
    return solution();
}
