#include "algorithms.h"
#include "definitions.h"
#include <vector>
#include <algorithm>
#include <cmath>
#include <iterator>

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

int algorithms::compute_profit(vector<int>delivery_ids) {
    int profit = 0;
    vector<int> profits = dep->get_profits();
    for (auto id:delivery_ids){
        profit += profits[id];
    }

    return profit;
}

int algorithms::compute_load(vector<int>delivery_ids) {
    int load = 0;
    vector<int> loads = dep->get_loads();
    for (auto id:delivery_ids){
        load += loads[id];
    }

    return load;
}

double algorithms::get_distance(int x1, int y1, int x2, int y2) {
    double delta_x = x2 - x1;
    double delta_y = y2 - y1;

    return sqrt(delta_x * delta_x + delta_y * delta_y);
}

double algorithms::compute_energy_cost(vector<int>delivery_ids){
    vector<int> delivery_points = dep->get_delivery_points();
    vector<int> launches = dep->get_launches();
    vector<int> rendezvouses = dep->get_rendezvouses();
    int height = dep->get_height();
        int energy_per_flight = dep->get_energy_per_flight();

    vector<int>delivery_locations;
    vector<int>launch_points;
    vector<int>rendezvous_points;

    for (auto id:delivery_ids){
        delivery_locations.push_back(delivery_points[id]);
        launch_points.push_back(launches[id]);
        rendezvous_points.push_back(rendezvouses[id]);
    }
    
    int L = *min_element(launch_points.begin(), launch_points.end());
    int R = *max_element(rendezvous_points.begin(), rendezvous_points.end());

    int L_delivery = *min_element(delivery_locations.begin(), delivery_locations.end());
    int R_delivery = *max_element(delivery_locations.begin(), delivery_locations.end());

    double a = get_distance(L, 0, L_delivery, height);
    double b = R_delivery - L_delivery;
    double c = get_distance(R_delivery, height, R, 0);

    return (a + b + c) * energy_per_flight;
}


tuple<vector<vector<int>>, vector<double>> algorithms::compute_all_flights(){
    // For any lunch L and rendezvous point R, compute the set of deliveris such that their
    // lunch and rendezvous point lies in [L, R]
    vector<int> launches = dep->get_launches();
    vector<int> rendezvouses = dep->get_rendezvouses();
    int energy_per_flight = dep->get_energy_per_flight();
    int drone_battery = dep->get_drone_battery();
    vector<int> delivery_points = dep->get_delivery_points();
    int drone_load = dep->get_drone_load();

    vector<vector<int>> all_flights;
    vector<double> energy_costs;

    for(int i=0; i<launches.size(); i++){
        int L = launches[i];
        for(int j=0; j<rendezvouses.size(); j++){
            int R = rendezvouses[j];
            vector<int>flight;
            if(R > L && rendezvouses[i] <= R && launches[j] >= L){  // R >= L
                if((R - L)*energy_per_flight <= drone_battery){
                    for(int k=0; k<delivery_points.size(); k++){
                        if(L <= launches[k] && launches[k] <= R && L <= rendezvouses[k] && rendezvouses[k] <= R){
                            flight.push_back(k);
                        }
                        
                        if (flight.size() > drone_load){
                            flight.clear();
                            break;
                        }
                    }
                    if(flight.size() >= 1 && flight.size() <= drone_load){
                        double energy = compute_energy_cost(flight);
                        if(energy <= drone_battery){
                            all_flights.push_back(flight);
                            energy_costs.push_back(energy);
                            // cout << "L: " << L << " , " << "R: " << R << " -- ";
                            // cout << " energy: " << energy << " , delivery: ";
                            // for (auto i:flight){
                            //         cout << i << ", ";
                            // }
                            // cout << endl;
                        }
                    }
                }
            }
        }
    }
    return {all_flights, energy_costs};
}

bool algorithms::check_intersection(vector<int> flight1, vector<int> flight2){
    vector<int> temp;
    set_intersection(flight1.begin(), flight1.end(), 
                          flight2.begin(), flight2.end(),
                          back_inserter(temp));
    return !temp.empty();
}


solution algorithms::opt_ilp() {
    auto sets = compute_all_flights();
    vector<vector<int>> all_flights = get<0>(sets);
    vector<double> energy_costs = get<1>(sets);                         

    int X = all_flights.size();
    int num_deliveries = dep->get_num_deliveries();
    int drone_load = dep->get_drone_load();

    int drone_battery = dep->get_drone_battery();

    vector<int> flights_load;
    vector<int> flights_proft;
    for (auto f:all_flights){
        int load = compute_load(f);
        flights_load.push_back(load);

        int profit = compute_profit(f);
        flights_proft.push_back(profit);
    }
    
    solution sol;

    try {
        // Create an environment
        GRBEnv env = GRBEnv(true);
        // env.set("LogFile", "mip1.log");
        env.start();

        // Create an empty model
        GRBModel model = GRBModel(env);

        // Create variables
        // For flights       
        GRBVar x[X];
        for (int i = 0; i < X; i++){
            ostringstream vname ;
            vname << "x" << i;
            x[i] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, vname.str());
        }

        // For deliveris 
        GRBVar y[num_deliveries];
        for (int j = 0; j < num_deliveries; j++){
            ostringstream yname ;
            yname << "y" << j;
            y[j] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, yname.str());
        }
        
        model.update();

        // (3) in the paper
        GRBLinExpr sum_y;
        for (int j = 0; j < num_deliveries; j++){
            sum_y = 0;
            for (int i = 0; i < X; i++){
                if (find(all_flights[i].begin(), all_flights[i].end(), j) != all_flights[i].end()){
                    sum_y += y[j];
                }
            }
            model.addConstr(sum_y <= 1);
        }

        // (4)
        for (int i = 0; i < X; i++){
            for (int k = 0; k < X; k++){
                if (i != k){
                    if (check_intersection(all_flights[i], all_flights[k])){
                        model.addConstr(x[i] + x[k] <= 1);
                    }
                }         
            }
        }

        // (5), this is already satisfied
        // for (int i = 0; i < X; i++){
        //     model.addConstr(flights_load[i] * x[i] <= drone_load);
        // }

        // (6)
        GRBLinExpr sum_energy = 0;
        for (int i = 0; i < X; i++){
            sum_energy += energy_costs[i] * x[i];
        }
        model.addConstr(sum_energy <= drone_battery);

        // (1)
        GRBLinExpr sum_profit = 0;
        for (int i = 0; i < X; i++){
            sum_profit += flights_proft[i] * x[i];
        }

        model.update();
        model.setObjective(sum_profit, GRB_MAXIMIZE);
        model.optimize();

        cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << endl;

        for (int i = 0; i < X; i++){
            cout << x[i].get(GRB_StringAttr_VarName) << " " << x[i].get(GRB_DoubleAttr_X) << endl;
        }

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
