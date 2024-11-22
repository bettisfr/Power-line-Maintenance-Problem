#include "algorithms.h"
#include "definitions.h"
#include <vector>
#include <algorithm>
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

    if (index >= 0 && index <= 5) {
        out = algorithm_functions[index](*this);
    } else {
        cerr << "Invalid algorithm index." << endl;
    }

    return out;
}

int algorithms::compute_profit(const vector<int> &delivery_ids) {
    int profit = 0;
    vector<int> profits = dep->get_profits();
    for (auto id: delivery_ids) {
        profit += profits[id];
    }

    return profit;
}

int algorithms::compute_load(const vector<int> &delivery_ids) {
    int load = 0;
    vector<int> loads = dep->get_loads();
    for (auto id: delivery_ids) {
        load += loads[id];
    }

    return load;
}

double algorithms::compute_energy(const vector<int> &delivery_ids) {
    vector<int> delivery_points = dep->get_delivery_points();
    vector<int> launches = dep->get_launches();
    vector<int> rendezvouses = dep->get_rendezvouses();
    int height = dep->get_height();
    int energy_per_flight = dep->get_energy_per_flight();

    vector<int> delivery_locations;
    vector<int> launch_points;
    vector<int> rendezvous_points;

    for (auto id: delivery_ids) {
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


tuple<vector<vector<int>>, vector<double>> algorithms::compute_all_flights() {
    // For any launch L and rendezvous point R, compute the set of deliveries such that their
    // launch and rendezvous point lies in [L, R]
    vector<int> launches = dep->get_launches();
    vector<int> rendezvouses = dep->get_rendezvouses();
    int energy_per_flight = dep->get_energy_per_flight();
    int drone_battery = dep->get_drone_battery();
    vector<int> delivery_points = dep->get_delivery_points();
    int drone_load = dep->get_drone_load();

    vector<vector<int>> all_flights;
    vector<double> energy_costs;

    for (int i = 0; i < launches.size(); i++) {
        int L = launches[i];
        for (int j = 0; j < rendezvouses.size(); j++) {
            int R = rendezvouses[j];
            vector<int> flight;
            if (R > L && rendezvouses[i] <= R && launches[j] >= L) {  // R >= L
                if ((R - L) * energy_per_flight <= drone_battery) {
                    for (int k = 0; k < delivery_points.size(); k++) {
                        if (L <= launches[k] && launches[k] <= R && L <= rendezvouses[k] && rendezvouses[k] <= R) {
                            flight.push_back(k);
                        }

                        if (flight.size() > drone_load) {
                            flight.clear();
                            break;
                        }
                    }
                    if (!flight.empty() && flight.size() <= drone_load) {
                        double energy = compute_energy(flight);
                        if (energy <= drone_battery) {
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

    int X = static_cast<int>(all_flights.size());
    int num_deliveries = dep->get_num_deliveries();
//    int drone_load = dep->get_drone_load();

    int B = dep->get_drone_battery();

    vector<int> flights_load;
    vector<int> flights_profit;
    for (const auto &f: all_flights) {
        int load = compute_load(f);
        flights_load.push_back(load);

        int profit = compute_profit(f);
        flights_profit.push_back(profit);
    }

    // for (int i=0;i<all_flights.size(); i++){
    //     for (auto j:all_flights[i]){
    //         cout << j << ", ";
    //     }
    //     cout << " load: " << flights_load[i] << " Energy: " << energy_costs[i] 
    //                       << " p: "<< flights_profit[i]<< endl;
    // }                    

    solution sol;

    try {
        GRBEnv env = GRBEnv(true);
        // env.set("LogFile", "mip1.log");
        env.start();
        GRBModel model = GRBModel(env);

        // Variables for flights       
        GRBVar x[X];
        for (int i = 0; i < X; i++) {
            ostringstream v_name;
            v_name << "x" << i;
            x[i] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, v_name.str());
        }

        // Variables for deliveries
        GRBVar y[num_deliveries];
        for (int j = 0; j < num_deliveries; j++) {
            ostringstream y_name;
            y_name << "y" << j;
            y[j] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, y_name.str());
        }

        model.update();

        // (3) in the paper
        GRBLinExpr sum_y;
        for (int j = 0; j < num_deliveries; j++) {
            sum_y = 0;
            for (int i = 0; i < X; i++) {
                if (find(all_flights[i].begin(), all_flights[i].end(), j) != all_flights[i].end()) {
                    sum_y += y[j];
                }
            }
            model.addConstr(sum_y <= 1);
        }

        // (4)
        for (int i = 0; i < X; i++) {
            for (int k = 0; k < X; k++) {
                if (i != k) {
                    if (util::check_intersection(all_flights[i], all_flights[k])) {
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
        for (int i = 0; i < X; i++) {
            sum_energy += energy_costs[i] * x[i];
        }
        model.addConstr(sum_energy <= B);

        // (1)
        GRBLinExpr sum_profit = 0;
        for (int i = 0; i < X; i++) {
            sum_profit += flights_profit[i] * x[i];
        }

        model.update();
        model.setObjective(sum_profit, GRB_MAXIMIZE);
        model.optimize();

        cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << endl;

        for (int i = 0; i < X; i++) {
            cout << x[i].get(GRB_StringAttr_VarName) << " " << x[i].get(GRB_DoubleAttr_X) << endl;
        }

    } catch (GRBException& e) {
        cout << "Error code = " << e.getErrorCode() << endl;
        cout << e.getMessage() << endl;
    } catch (...) {
        cout << "Exception during optimization" << endl;
    }

    return sol;
}

/////////// bin_s /////////
tuple<int, int> algorithms::compute_LR(const vector<int> &flight) {
    vector<int> launches = dep->get_launches();
    vector<int> rendezvouses = dep->get_rendezvouses();

    vector<int> launch_points;
    vector<int> rendezvous_points;

    for (auto id: flight) {
        launch_points.push_back(launches[id]);
        rendezvous_points.push_back(rendezvouses[id]);
    }

    int L = *min_element(launch_points.begin(), launch_points.end());
    int R = *max_element(rendezvous_points.begin(), rendezvous_points.end());

    return {L, R};
}

int algorithms::compute_opt(int id, const vector<int>& launches, const vector<int>& rendezvouses,
                            vector<int> profits, vector<int> opt,
                            vector<int> p) {

    if (id == -1) {
        return 0;
    } else if (id >= 0 && id < opt.size()) {
        return opt[id];
    } else {
        return max(profits[id] + compute_opt(p[id], launches, rendezvouses, profits, opt, p),
                   compute_opt(id - 1, launches, rendezvouses, profits, opt, p));
    }
}


vector<int> algorithms::weighted_interval(const vector<int>& launches, const vector<int>& rendezvouses,
                                          const vector<int>& profits, vector<int> opt,
                                          const vector<int>& p) {

    for (int i = 0; i < launches.size(); i++) {
        int opt_i = compute_opt(i, launches, rendezvouses, profits, opt, p);
        opt.push_back(opt_i);
    }

    return opt;
}

vector<int> algorithms::find_solution(int j, const vector<int>& launches, const vector<int>& rendezvouses,
                                      vector<int> profits, vector<int> opt,
                                      vector<int> p, vector<int> O) {

    if (j == -1) {
        return O;
    } else {
        if (profits[j] + opt[p[j]] >= opt[j - 1]) {
            O.push_back(j);
            return find_solution(p[j], launches, rendezvouses, profits, opt, p, O);
        } else {
            return find_solution(j - 1, launches, rendezvouses, profits, opt, p, O);
        }
    }
}

solution algorithms::bin_s() {
    solution sol;

    int B = dep->get_drone_battery();
    // compute all flights and energy, then sort them
    auto sets = compute_all_flights();
    vector<vector<int>> all_flights_temp = get<0>(sets);
    vector<double> energy_costs_temp = get<1>(sets);

    vector<int> launches_temp;
    vector<int> rendezvouses_temp;

    for (const auto& flight: all_flights_temp) {
        auto points = compute_LR(flight);
        launches_temp.push_back(get<0>(points));
        rendezvouses_temp.push_back(get<1>(points));
    }

    vector<vector<int>> all_flights;
    vector<double> energy_costs;
    vector<int> profits;
    vector<int> launches;
    vector<int> rendezvouses;

    // sort according to rendezvouses_temp
    vector<pair<int, int> > Ri;
    for (int i = 0; i < all_flights_temp.size(); i++) {
        Ri.emplace_back(rendezvouses_temp[i], i);
    }

    sort(Ri.begin(), Ri.end());

    for (auto it: Ri) {
        all_flights.push_back(all_flights_temp[it.second]);
        energy_costs.push_back(energy_costs_temp[it.second]);
        launches.push_back(launches_temp[it.second]);
        rendezvouses.push_back(rendezvouses_temp[it.second]);
    }

    for (const auto& flight: all_flights) {
        int profit = compute_profit(flight);
        profits.push_back(profit);
    }

    // compute p
    vector<int> p = util::largest_nonoverlap_delivery(launches, rendezvouses);

    // for (int i=0;i<all_flights.size(); i++){
    //     for (auto j:all_flights[i]){
    //         cout << j << ", ";
    //     }
    //     cout << " L: " << launches[i] << " R: " << rendezvouses[i] << " p: " << p[i] << endl;
    // }    

    vector<int> opt; // profits
    vector<int> pp = weighted_interval(launches, rendezvouses, profits, opt, p);

    vector<int> O;  // ids
    vector<int> opt_flights;
    opt_flights = find_solution(static_cast<int>(launches.size() - 1), launches, rendezvouses, profits, pp, p, O);
    reverse(opt_flights.begin(), opt_flights.end());

    // cout << "opt_flights: ";
    // for (auto f:opt_flights){
    //     cout << f << " , ";
    // }
    // cout << endl;

    // bin packing
    vector<vector<int>> bin_sol;
    vector<int> reward(opt_flights.size(), 0);
    vector<int> cost(opt_flights.size(), 0);

    for (int i = 0; i < opt_flights.size(); i++) {
        bin_sol.emplace_back();
    }

    int bins = 1;
    for (int k : opt_flights) {
        bool assigned = false;
        for (int j = 0; j < bins; j++) {
            if (cost[j] + energy_costs[k] <= B && !assigned) {
                assigned = true;
                bin_sol[j].push_back(k);
                reward[j] = reward[j] + profits[k];
                cost[j] += energy_costs[k]; // FIXME: cost is "int", "energy_cost" is double...
            }
        }

        if (!assigned) {
            bins++;
            assigned = true;
            bin_sol.emplace_back();
            reward.push_back(0);
            cost.push_back(0);
            bin_sol[bins].push_back(k);
            reward[bins] += profits[k];
            cost[bins] += energy_costs[k];
        }
    }

    // for (int i = 0; i < bin_sol.size(); i++){
    //     for (auto j:bin_sol[i]){
    //         cout << j << " , ";
    //     }
    //     cout << " reward: " << reward[i] << " cost: " << cost[i] << endl;
    // }

    int opt_id = distance(reward.begin(), max_element(reward.begin(), reward.end()));

    vector<vector<int>> selected;
    if (bin_sol.size() > 0) {
        for (auto flight_id: bin_sol[opt_id]) {
            selected.push_back(all_flights[flight_id]);
        }

        sol.total_energy_cost = cost[opt_id];
        sol.total_profit = reward[opt_id];
    }

//    sol.total_energy_cost = cost[opt_id];
//    sol.total_profit = reward[opt_id];

    return sol;
}

//// Knapsack

solution algorithms::knapsack_opt() {
    cout << "knapsack" << endl;
    // Variables
    solution sol;
    int B = dep->get_drone_battery();
    // compute all flights and energy, then sort them
    auto sets = compute_all_flights();
    vector<vector<int>> all_flights_temp = get<0>(sets);
    vector<double> energy_costs_temp = get<1>(sets);

    vector<int> launches_temp;
    vector<int> rendezvouses_temp;

    vector<vector<int>> all_flights;
    vector<double> energy_costs;
    vector<int> profits;
    vector<int> launches;
    vector<int> rendezvouses;

    vector<pair<int, int> > Ri;

    vector<int> predecessors;

    // Logic
    for (const auto& flight: all_flights_temp) {
        auto points = compute_LR(flight);
        launches_temp.push_back(get<0>(points));
        rendezvouses_temp.push_back(get<1>(points));
    }

    // sort according to rendezvouses_temp
    for (int i = 0; i < all_flights_temp.size(); i++) {
        Ri.emplace_back(rendezvouses_temp[i], i);
    }

    sort(Ri.begin(), Ri.end());

    for (auto it: Ri) {
        all_flights.push_back(all_flights_temp[it.second]);
        energy_costs.push_back(energy_costs_temp[it.second]);
        launches.push_back(launches_temp[it.second]);
        rendezvouses.push_back(rendezvouses_temp[it.second]);
    }


    for (const auto& flight: all_flights) {
        int profit = compute_profit(flight);
        profits.push_back(profit);
    }

    for (int i=0; i < all_flights.size(); i++) {
        cout << "i: " << i << " profit: " << profits[i] << " cost: " << energy_costs[i] << " launch: " << launches[i] << " rendevouz: " << rendezvouses[i] << endl;
    }

    // compute predecessors
    predecessors = util::largest_nonoverlap_delivery(launches, rendezvouses);

    int numFlights = all_flights.size();
    vector<vector<int>> opt_reward(numFlights + 1, vector<int>(B + 1, 0));
    vector<vector<int>> opt_costs(numFlights + 1, vector<int>(B + 1, 0));
    vector<vector<vector<int>>> opt_intervals(numFlights + 1, vector<vector<int>>(B + 1, vector<int>(1, 0)));

    int i_reward = 0;
    int i_cost = 0;

    int pred_reward = 0;
    int pred_cost = 0;

    int row_reward = 0;
    int row_cost = 0;

    for (int i = 1; i <= numFlights; i++)
    {
        for (int b = 0; b <= B; b++)
        {
            if (energy_costs[i - 1] <= b)
            {
                // current item
                i_reward = profits[i - 1];
                i_cost = energy_costs[i - 1];
                //last compatible item
                pred_reward = opt_reward[predecessors[i - 1] + 1][b - energy_costs[i-1]];
                pred_cost = opt_costs[predecessors[i - 1] + 1][b - energy_costs[i-1]];
                //best at previous row
                row_reward = opt_reward[i-1][b];
                row_cost = opt_costs[i-1][b];
                if (i_reward + pred_reward > row_reward)
                {
                    opt_reward[i][b] = i_reward + pred_reward;
                    opt_costs[i][b] = i_cost + pred_cost;
                    cout << i << " " << b << " " << i_reward + pred_reward << endl;
                    opt_intervals[i][b] = opt_intervals[predecessors[i - 1] + 1][b - energy_costs[i - 1]];
                    opt_intervals[i][b].push_back((i - 1));
                }
                else
                {
                    opt_reward[i][b] = row_reward;
                    opt_costs[i][b] = row_cost;
                    opt_intervals[i][b] = opt_intervals[i-1][b];
                }
            }
            else
            {
                opt_reward[i][b] = row_reward;
                opt_costs[i][b] = row_cost;
                opt_intervals[i][b] = opt_intervals[i-1][b];
            }
        }
    }

    cout << " OPT reward = " << opt_reward[numFlights][B] << endl;
    cout << " OPT cost = " << opt_costs[numFlights][B] << endl;
    for (int i = 1; i <= opt_intervals[numFlights][B].size(); i++)
    {
        cout << " OPT Intervals = " << opt_intervals[numFlights][B][i] << endl;
    }
    return solution();
}

///

//// Col-S
solution algorithms::col_s() {
    return solution();
}

///

