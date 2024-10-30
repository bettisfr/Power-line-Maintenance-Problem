#include "algorithms.h"
#include "definitions.h"
#include <vector>
#include <algorithm>
#include <cmath>
#include <iterator>

#include "gurobi_c++.h"
#include "util.h"

algorithms::algorithms(deployment *m_dep) {
    dep = m_dep;
}

// Useless (and stupid) method, but it was nice to use a vector of pointers to methods :-D
solution algorithms::run_experiment(int algorithm) {
    int index = algorithm;
    solution out;

    if (index >= 0 && index <= 3) {
        out = algorithm_functions[index](*this);
    } else {
        cerr << "Invalid algorithm index." << endl;
    }

    return out;
}

int algorithms::compute_profit(const vector<int>&delivery_ids) {
    int profit = 0;
    vector<int> profits = dep->get_profits();
    for (auto id:delivery_ids){
        profit += profits[id];
    }

    return profit;
}

int algorithms::compute_load(const vector<int>&delivery_ids) {
    int load = 0;
    vector<int> loads = dep->get_loads();
    for (auto id:delivery_ids){
        load += loads[id];
    }

    return load;
}

double algorithms::compute_energy(const vector<int>&delivery_ids){
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

    double a = util::get_distance(L, 0, L_delivery, height);
    double b = R_delivery - L_delivery;
    double c = util::get_distance(R_delivery, height, R, 0);

    return (a + b + c) * energy_per_flight;
}


tuple<vector<vector<int>>, vector<double>> algorithms::compute_all_flights(){
    // For any lunch L and rendezvous point R, compute the set of deliveries such that their
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
                    if(!flight.empty() && flight.size() <= drone_load){
                        double energy = compute_energy(flight);
                        if(energy <= drone_battery){
                            all_flights.push_back(flight);
                            energy_costs.push_back(energy);
                        }
                    }
                }
            }
        }
    }
    return {all_flights, energy_costs};
}


solution algorithms::opt_ilp() {
    auto sets = compute_all_flights();
    vector<vector<int>> all_flights = get<0>(sets);
    vector<double> energy_costs = get<1>(sets);                         

    int X = all_flights.size();
    int num_deliveries = dep->get_num_deliveries();
    int drone_load = dep->get_drone_load();

    int B = dep->get_drone_battery();

    vector<int> flights_load;
    vector<int> flights_proft;
    for (const auto& f:all_flights){
        int load = compute_load(f);
        flights_load.push_back(load);

        int profit = compute_profit(f);
        flights_proft.push_back(profit);
    }

    // for (int i=0;i<all_flights.size(); i++){
    //     for (auto j:all_flights[i]){
    //         cout << j << ", ";
    //     }
    //     cout << " load: " << flights_load[i] << " Energy: " << energy_costs[i] 
    //                       << " p: "<< flights_proft[i]<< endl;
    // }                    
    
    solution sol;

    try {
        GRBEnv env = GRBEnv(true);
        // env.set("LogFile", "mip1.log");
        env.start();
        GRBModel model = GRBModel(env);
 
        // Variables for flights       
        GRBVar x[X];
        for (int i = 0; i < X; i++){
            ostringstream vname ;
            vname << "x" << i;
            x[i] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, vname.str());
        }

        // Variables for deliveris 
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
                    if (util::check_intersection(all_flights[i], all_flights[k])){
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
        model.addConstr(sum_energy <= B);

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

/////////// Bin_S /////////
tuple<int, int> algorithms::compute_LR(const vector<int>&flight){
    vector<int> launches = dep->get_launches();
    vector<int> rendezvouses = dep->get_rendezvouses();

    vector<int>launch_points;
    vector<int>rendezvous_points;

    for (auto id:flight){
        launch_points.push_back(launches[id]);
        rendezvous_points.push_back(rendezvouses[id]);
    }
    
    int L = *min_element(launch_points.begin(), launch_points.end());
    int R = *max_element(rendezvous_points.begin(), rendezvous_points.end());

    return {L, R};
}

vector<int> algorithms::largest_nonoverlap_delivery(vector<int> lunches, vector<int> rendezvouses){
    vector<int> p;  // >= -1
    for (int i = 0; i < lunches.size(); i++){
        int id = -10;
        for (int j = 0; j < lunches.size(); j++){
            if (i != j){
                if (rendezvouses[j] >= lunches[i]){
                    id = j - 1;
                    if (id == i){
                        id = id - 1;
                    }
                    p.push_back(id);
                    id = -10;
                    break;
                } else {
                    id = j;
                }
            }
        }
        if (id >= 0){
            p.push_back(id);
        }
    }
    
    return p;
}

int algorithms::compute_opt(int id, vector<int> lunches, vector<int> rendezvouses, 
                                      vector<int> profits, vector<int> opt,
                                      vector<int> p){

    if (id == -1){
        return 0;
    } else if(id >= 0 && id < opt.size()){
        return opt[id];
    } else {
        return max(profits[id] + compute_opt(p[id], lunches, rendezvouses, profits, opt, p),
                                  compute_opt(id-1, lunches, rendezvouses, profits, opt, p));
    }                        
}


vector<int> algorithms::weighted_interval(vector<int> lunches, vector<int> rendezvouses, 
                                      vector<int> profits, vector<int> opt,
                                      vector<int> p){

        for (int i = 0; i < lunches.size(); i++){
            int opt_i = compute_opt(i, lunches, rendezvouses,  profits, opt, p);
            opt.push_back(opt_i);
        }
        
    return opt;                                 
}

vector<int> algorithms::find_solution(int j, vector<int> lunches, vector<int> rendezvouses, 
                                      vector<int> profits, vector<int> opt,
                                      vector<int> p, vector<int> O){

    if (j == -1){
        return O;
    } else {
        if (profits[j] + opt[p[j]] >= opt[j-1]){
            O.push_back(j);
            return find_solution(p[j], lunches, rendezvouses, profits, opt, p, O);
        } else {
            return find_solution(j-1, lunches, rendezvouses, profits, opt, p, O);
        }
    }
    
}

solution algorithms::Bin_S() {
    solution sol;

    // compute all flights and energy, sort them
    auto sets = compute_all_flights();
    vector<vector<int>> all_flights_temp = get<0>(sets);
    vector<double> energy_costs_temp = get<1>(sets); 

    //vector<int> delivery_points_temp = dep->get_delivery_points();
    vector<int> lunches_temp;
    vector<int> rendezvouses_temp;

    for (auto flight:all_flights_temp){
        auto points = compute_LR(flight);
        lunches_temp.push_back(get<0>(points));
        rendezvouses_temp.push_back(get<1>(points));
    }
    
    vector<vector<int>> all_flights;
    vector<double> energy_costs;
    vector<int> profits;
    vector<int> lunches;
    vector<int> rendezvouses;

    // sort according to rendezvouses_temp
    vector<pair<int, int> > Ri; 
    for (int i = 0; i < all_flights_temp.size(); i++){
        Ri.push_back({rendezvouses_temp[i], i});
    }
    
    sort(Ri.begin(), Ri.end());

    for (auto it:Ri){
        all_flights.push_back(all_flights_temp[it.second]);
        energy_costs.push_back(energy_costs_temp[it.second]);
        lunches.push_back(lunches_temp[it.second]);
        rendezvouses.push_back(rendezvouses_temp[it.second]);
    } 

    for (auto flight:all_flights){
        int profit = compute_profit(flight);
        profits.push_back(profit);
    }
    
    // compute p
    vector<int>p = largest_nonoverlap_delivery(lunches, rendezvouses);

    // for (int i=0;i<all_flights.size(); i++){
    //     for (auto j:all_flights[i]){
    //         cout << j << ", ";
    //     }
    //     cout << " L: " << lunches[i] << " R: " << rendezvouses[i] << " p: " << p[i] << endl;
    // }    

    vector<int> opt; // values
    vector<int> pp = weighted_interval(lunches, rendezvouses, profits, opt, p);
    // for (auto i:pp){
    //     cout << i << " , " << endl;
    // }
    
    vector<int> O;  // ids
    vector<int> opt_flights;
    opt_flights = find_solution(lunches.size()-1, lunches, rendezvouses, profits, pp, p, O);

    // for (auto f:opt_flights){
    //     cout << f << " , ";
    // }
   
   // bin packing


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
