#include "algorithms.h"
#include <utility>
#include <vector>
#include <algorithm>
#include <iterator>
#include <iomanip>
#include <thread>

#include "gurobi_c++.h"
#include "../util/util.h"

using namespace std;

algorithms::algorithms(const deployment& m_dep) : dep(m_dep) {
}

// Useless (and stupid) method, but it was nice to use a vector of pointers to methods :-D
solution algorithms::run_experiment(int algorithm) {
    int index = algorithm;
    solution out;

    if (index >= 0 && index <= 7) {
        out = algorithm_functions[index](*this);
    } else {
        cerr << "Invalid algorithm index." << endl;
    }

    return out;
}

solution algorithms::opt_ilp() {
    // Pre-processing
    auto [all_flights, energy_costs, profits, loads] = dep.compute_solution_space();

    int X = static_cast<int>(all_flights.size());
    int num_deliveries = dep.get_num_deliveries();
    int B = dep.get_drone_battery();
    // vector<int> flights_load;

    vector<int> flights_profit;
    for (const auto &f: all_flights) {
        // int drone_load = dep.compute_load(f);
        // flights_load.push_back(drone_load);
        int profit = dep.compute_profit(f);
        flights_profit.push_back(profit);
    }

    try {
        GRBEnv env = GRBEnv(true);
        // env.set("LogFile", "mip1.log");
        env.set("OutputFlag", "0");

        int maxThreads = static_cast<int>(thread::hardware_concurrency());
        env.set(GRB_IntParam_Threads, maxThreads);

        env.start();
        GRBModel model = GRBModel(env);

        // Disable log output
        model.set(GRB_IntParam_OutputFlag, 0);

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

        // constr (3)
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

        // constr (4)
        for (int i = 0; i < X; i++) {
            for (int k = 0; k < X; k++) {
                if (i != k) {
                    if (!deployment::check_correct_interval({all_flights[i]},
                                        {get<0>(dep.compute_LR(all_flights[i]))}, {get<1>(dep.compute_LR(all_flights[i]))},
                                            get<0>(dep.compute_LR(all_flights[k])), get<1>(dep.compute_LR(all_flights[k])))) {
                        model.addConstr(x[i] + x[k] <= 1);
                    }
                }
            }
        }

        // constr (5), this is already satisfied
        // for (int i = 0; i < X; i++){
        //     model.addConstr(flights_load[i] * x[i] <= drone_load);
        // }

        // constr (6)
        GRBLinExpr sum_energy = 0;
        for (int i = 0; i < X; i++) {
            sum_energy += energy_costs[i] * x[i];
        }
        model.addConstr(sum_energy <= B);

        // for obj function (1)
        GRBLinExpr sum_profit = 0;
        for (int i = 0; i < X; i++) {
            sum_profit += flights_profit[i] * x[i];
        }

        model.update();
        model.setObjective(sum_profit, GRB_MAXIMIZE);
        model.optimize();

        // cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << endl;

        vector<vector<int>> selected_intervals;
        vector<int> sel_int_profits;
        vector<double> sel_int_energies;
        vector<int> sel_int_weights;

        double total_cost = 0.0;
        int total_profit = 0;
        for (int i = 0; i < X; i++) {
            // cout << x[i].get(GRB_StringAttr_VarName) << " " << x[i].get(GRB_DoubleAttr_X) << endl;
            if (x[i].get(GRB_DoubleAttr_X) > 0) {
                selected_intervals.push_back(all_flights[i]);
                total_cost = total_cost + energy_costs[i];
                total_profit = total_profit + flights_profit[i];

                int tmp_profit = 0;
                int tmp_weight = 0;
                for (int j: all_flights[i]) {
                    tmp_profit += dep.get_profits()[j];
                    tmp_weight += dep.get_weights()[j];
                }

                sel_int_profits.push_back(tmp_profit);
                sel_int_weights.push_back(tmp_weight);
                sel_int_energies.push_back(total_cost);
            }
        }
        sol.total_flights = selected_intervals;
        sol.total_energy = total_cost;
        sol.total_profit = total_profit;
        sol.profits = sel_int_profits;
        sol.weights = sel_int_weights;
        sol.energies = sel_int_energies;

    } catch (GRBException &e) {
        cout << "Error code = " << e.getErrorCode() << endl;
        cout << e.getMessage() << endl;
    } catch (...) {
        cout << "Exception during optimization" << endl;
    }

    return sol;
}

solution algorithms::bin_packing() {
    int B = dep.get_drone_battery();

    auto parameters = dep.sorting_with_rendezvouses_in_apx();
    vector<vector<int>> all_flights = get<0>(parameters);
    vector<double> energy_costs = get<1>(parameters);
    vector<int> profits = get<2>(parameters);
    vector<int> launches = get<3>(parameters);
    vector<int> rendezvouses = get<4>(parameters);

    // compute p
    vector<int> p = deployment::largest_non_overlap_delivery(launches, rendezvouses);

    vector<int> opt; // profits
    vector<int> pp = deployment::weighted_interval(launches, rendezvouses, profits, opt, p);

    vector<int> O;  // ids
    vector<int> opt_flights;
    opt_flights = deployment::find_solution(static_cast<int>(launches.size() - 1), launches, rendezvouses, profits, pp, p, O);
    reverse(opt_flights.begin(), opt_flights.end());

    // bin packing
    vector<vector<int>> bin_sol;
    vector<int> reward(opt_flights.size(), 0);
    vector<double> cost(opt_flights.size(), 0.0);

    for (int i = 0; i < opt_flights.size(); i++) {
        bin_sol.emplace_back();
    }

    int bins = 1;
    for (int k: opt_flights) {
        bool assigned = false;
        for (int j = 0; j < bins; j++) {
            if (cost[j] + energy_costs[k] <= B && !assigned) {
                assigned = true;
                bin_sol[j].push_back(k);
                reward[j] = reward[j] + profits[k];
                cost[j] += energy_costs[k];
            }
        }

        if (!assigned) {
            bins++;
//            assigned = true;
            bin_sol.emplace_back();
            reward.push_back(0);
            cost.push_back(0);
            bin_sol[bins].push_back(k);
            reward[bins] += profits[k];
            cost[bins] += energy_costs[k];
        }
    }

    int opt_id = static_cast<int>(distance(reward.begin(), max_element(reward.begin(), reward.end())));

    vector<vector<int>> selected_intervals;
    vector<int> sel_int_profits;
    vector<double> sel_int_energies;
    vector<int> sel_int_weights;

    if (!bin_sol.empty()) {
        for (auto flight_id: bin_sol[opt_id]) {
            selected_intervals.push_back(all_flights[flight_id]);
            int tmp_profit = 0;
            int tmp_weight = 0;
            for (int j: all_flights[flight_id]) {
                tmp_profit += dep.get_profits()[j];
                tmp_weight += dep.get_weights()[j];
            }

            sel_int_profits.push_back(tmp_profit);
            sel_int_weights.push_back(tmp_weight);
            sel_int_energies.push_back(dep.compute_energy(all_flights[flight_id]));
        }

        sol.total_energy = cost[opt_id];
        sol.total_profit = reward[opt_id];

        sol.total_flights = selected_intervals;
        sol.profits = sel_int_profits;
        sol.weights = sel_int_weights;
        sol.energies = sel_int_energies;
    }

    return sol;
}


solution algorithms::knapsack() {
    int B = dep.get_drone_battery();

    auto parameters = dep.sorting_with_rendezvouses_in_apx();
    vector<vector<int>> all_flights = get<0>(parameters);
    vector<double> energy_costs = get<1>(parameters);
    vector<int> profits = get<2>(parameters);
    vector<int> launches = get<3>(parameters);
    vector<int> rendezvouses = get<4>(parameters);

    // for (int i=0; i < all_flights.size(); i++) {
    //     cout << "i: " << i << " profit: " << profits[i] << " cost: " << energy_costs[i] << " launch: " << launches[i] << " rendezvous: " << rendezvouses[i] << endl;
    // }

    //dummy value
    launches.insert(launches.begin(), -1);
    rendezvouses.insert(rendezvouses.begin(), -1);
    profits.insert(profits.begin(), 0);
    energy_costs.insert(energy_costs.begin(), 0);

    // compute predecessors
    vector<int> predecessors = deployment::largest_non_overlap_delivery(launches, rendezvouses);

    // for (int i = 0; i < predecessors.size(); i++) {
    //     cout << "element: " << i << " compatible with " << predecessors[i] << endl;
    // }
    auto numFlights = all_flights.size();
    vector<vector<int>> opt_reward(numFlights + 1, vector<int>(B + 1, 0));
    vector<vector<int>> opt_costs(numFlights + 1, vector<int>(B + 1, 0));
    vector<vector<vector<int>>> opt_intervals(numFlights + 1, vector<vector<int>>(B + 1, vector<int>(1, 0)));

    int i_reward = 0;
    int i_cost = 0;

    int pred_reward = 0;
    int pred_cost = 0;

    int row_reward = 0;
    int row_cost = 0;

    for (int i = 1; i <= numFlights; i++) {
        for (int b = 0; b <= B; b++) {
            //best at previous row
            row_reward = opt_reward[i - 1][b];
            row_cost = opt_costs[i - 1][b];

            i_cost = static_cast<int>(energy_costs[i]) + 1;
            if (i_cost <= b) {
                // current item
                i_reward = profits[i];
                //last compatible item
                pred_reward = opt_reward[predecessors[i]][b - i_cost];
                pred_cost = opt_costs[predecessors[i]][b - i_cost];
                // cout << "previous sol: " << row_reward << " " << row_cost << endl;
                if (i_reward + pred_reward > row_reward) {
                    opt_reward[i][b] = i_reward + pred_reward;
                    opt_costs[i][b] = i_cost + pred_cost;
                    opt_intervals[i][b] = opt_intervals[predecessors[i]][b - i_cost];
                    opt_intervals[i][b].push_back((i));
                } else {
                    opt_reward[i][b] = row_reward;
                    opt_costs[i][b] = row_cost;
                    opt_intervals[i][b] = opt_intervals[i - 1][b];
                }
            } else {
                opt_reward[i][b] = row_reward;
                opt_costs[i][b] = row_cost;
                opt_intervals[i][b] = opt_intervals[i - 1][b];
            }
            // just for print and debug
            // std::stringstream result;
            // std::copy(opt_intervals[i][b].begin(), opt_intervals[i][b].end(), std::ostream_iterator<int>(result, " "));
            // cout << "OPT[" << i << "][" << b << "] = {r: " << opt_reward[i][b] << ", c: " << opt_costs[i][b] << ", int: " << result.str().c_str() << endl;
        }
    }

    vector<vector<int>> selected_intervals;
    vector<int> sel_int_profits;
    vector<double> sel_int_energies;
    vector<int> sel_int_weights;

    // cout << " OPT reward = " << opt_reward[numFlights][B] << endl;
    // cout << " OPT cost = " << opt_costs[numFlights][B] << endl;

    opt_intervals[numFlights][B].erase(
            remove(opt_intervals[numFlights][B].begin(), opt_intervals[numFlights][B].end(), 0),
            opt_intervals[numFlights][B].end()
    );
    for (int i: opt_intervals[numFlights][B]) {
        selected_intervals.push_back(all_flights[i - 1]);

        int tmp_profit = 0;
        int tmp_weight = 0;
        for (int j: all_flights[i - 1]) {
            tmp_profit += dep.get_profits()[j];
            tmp_weight += dep.get_weights()[j];
        }

        sel_int_profits.push_back(tmp_profit);
        sel_int_weights.push_back(tmp_weight);
        sel_int_energies.push_back(dep.compute_energy(all_flights[i - 1]));
    }

    sol.total_profit = opt_reward[numFlights][B];
    sol.total_energy = opt_costs[numFlights][B];
    sol.total_flights = selected_intervals;
    sol.profits = sel_int_profits;
    sol.weights = sel_int_weights;
    sol.energies = sel_int_energies;

    return sol;
}


solution algorithms::coloring() {
    int B = dep.get_drone_battery();

    auto parameters = dep.sorting_with_rendezvouses_in_apx();
    vector<vector<int>> all_flights = get<0>(parameters);
    vector<double> energy_costs = get<1>(parameters);
    vector<int> profits = get<2>(parameters);
    vector<int> launches = get<3>(parameters);
    vector<int> rendezvouses = get<4>(parameters);

    // for (int i=0; i<rendezvouses.size(); i++) {
    //     cout << "[" << launches[i] << "," << rendezvouses[i] << "]" << endl;
    // }

    vector<vector<vector<int>>> colors;
    auto n = rendezvouses.size();

    // initialize the colors which is at most n, the number of total_flights
    for (int i = 0; i < n; i++) {
        colors.emplace_back();
    }

    vector<int> last_interval;
    vector<int> current_interval;
    bool compatible;
    int index = 0;

    for (int i = 0; i < n; i++) {
        current_interval.clear();
        current_interval.push_back(launches[i]);
        current_interval.push_back(rendezvouses[i]);
        //index
        current_interval.push_back(i);

        index = 0;
        compatible = false;
        while (not compatible) {
            if (empty(colors[index])) {
                colors[index].push_back(current_interval);
                compatible = true;
            } else {
                last_interval = colors[index].back();
                int max_start = max(last_interval[0], current_interval[0]);
                int min_end = min(current_interval[1], last_interval[1]);
                if (max_start > min_end) {
                    colors[index].push_back(current_interval);
                    compatible = true;
                }
                index++;
            }
        }
    }
    // solve single colors by fractional knapsack

    // compute ratios
    vector<vector<double>> color_ratios;
    vector<vector<int>> color = {{1}};
    int max_profit = 0;
    double max_cost = 0;
    vector<int> max_selection;
    int profit = 0;
    double cost = 0;
    vector<int> selection;
    index = 0;
    int curr_index = 0;

    while (index < colors.size()) {
        color = colors[index];
        if (color.empty()) {
            break;
        }
        color_ratios.clear();
        // cout << "#Color: " << index << endl;
        for (auto &i: color) {
            // cout << "interval: " << color[i][2] <<" [" << color[i][0] << "," << color[i][1] << "] . profit: " << profits[color[i][2]] << " cost: " << energy_costs[color[i][2]] << " ratio: " <<profits[color[i][2]]/energy_costs[color[i][2]] << endl;
            color_ratios.push_back({(profits[i[2]] / energy_costs[i[2]]), static_cast<double>(i[2])});
        }
        cost = 0;
        profit = 0;
        selection.clear();
        sort(color_ratios.begin(), color_ratios.end());
        for (int i = static_cast<int>(color_ratios.size()) - 1; i >= 0; i--) {
            curr_index = static_cast<int>(color_ratios[i][1]);
            if (cost + energy_costs[curr_index] <= B) {
                cost += energy_costs[curr_index];
                profit += profits[curr_index];
                selection.push_back(curr_index);
            }
        }
        // cout << "profit: " << profit << " cost: " << cost << endl;
        if (profit > max_profit) {
            max_profit = profit;
            max_selection = selection;
            max_cost = cost;
        }
        index++;
    }

    vector<vector<int>> selected_intervals;
    vector<int> sel_int_profits;
    vector<double> sel_int_energies;
    vector<int> sel_int_weights;

    for (int i: max_selection) {
        selected_intervals.push_back(all_flights[i]);

        int tmp_profit = 0;
        int tmp_weight = 0;
        for (int j: all_flights[i]) {
            tmp_profit += dep.get_profits()[j];
            tmp_weight += dep.get_weights()[j];
        }

        sel_int_profits.push_back(tmp_profit);
        sel_int_weights.push_back(tmp_weight);
        sel_int_energies.push_back(dep.compute_energy(all_flights[i]));
    }

    sol.total_profit = max_profit;
    sol.total_energy = max_cost;
    sol.total_flights = selected_intervals;
    sol.profits = sel_int_profits;
    sol.weights = sel_int_weights;
    sol.energies = sel_int_energies;

    return sol;
}


/////////////   Heuristics   ////////////////
solution algorithms::flight_selection_in_heu(vector<vector<int>> all_flights, vector<double> energy_costs,
                                             vector<int> profits, vector<int> launches, vector<int> rendezvouses) {
    vector<vector<int>> selected_intervals;
    vector<int> sel_int_profits;
    vector<double> sel_int_energies;
    vector<int> sel_int_weights;
    vector<int> launches_result;
    vector<int> rendezvouses_result;

    int total_profit = 0;
    double total_energy = 0.0;
    double B = dep.get_drone_battery();

    while (!all_flights.empty() && B != 0) {
        vector<int> task = all_flights[0];
        int L = launches[0];
        int R = rendezvouses[0];

        if (energy_costs[0] <= B &&
            (selected_intervals.empty() ||
             deployment::check_correct_interval(selected_intervals, launches_result, rendezvouses_result, L, R))) {
            selected_intervals.push_back(task);
            launches_result.push_back(L);
            rendezvouses_result.push_back(R);
            total_profit += profits[0];
            B -= energy_costs[0];
            total_energy += energy_costs[0];

            int tmp_profit = 0;
            int tmp_weight = 0;
            for (int j: all_flights[0]) {
                tmp_profit += dep.get_profits()[j];
                tmp_weight += dep.get_weights()[j];
            }

            sel_int_profits.push_back(tmp_profit);
            sel_int_weights.push_back(tmp_weight);
            sel_int_energies.push_back(dep.compute_energy(all_flights[0]));
        }

//        if (selected_intervals.empty()) {
//            if (energy_costs[0] <= B) {
//                selected_intervals.push_back(task);
//                launches_result.push_back(L);
//                rendezvouses_result.push_back(R);
//                total_profit += profits[0];
//                B -= energy_costs[0];
//                total_energy += energy_costs[0];
//            }
//        } else {
//            if (energy_costs[0] <= B &&
//                check_correct_interval(selected_intervals, launches_result, rendezvouses_result, L, R)) {
//                selected_intervals.push_back(task);
//                total_profit += profits[0];
//                launches_result.push_back(L);
//                rendezvouses_result.push_back(R);
//                B -= energy_costs[0];
//                total_energy += energy_costs[0];
//            }
//        }
        // remove task
        all_flights.erase(all_flights.begin());
        energy_costs.erase(energy_costs.begin());
        profits.erase(profits.begin());
        launches.erase(launches.begin());
        rendezvouses.erase(rendezvouses.begin());
    }

//    cout << total_profit << endl;

    sol.total_profit = total_profit;
    sol.total_energy = total_energy;
    sol.total_flights = selected_intervals;

    sol.profits = sel_int_profits;
    sol.weights = sel_int_weights;
    sol.energies = sel_int_energies;

    return sol;
}

solution algorithms::greedy_profit() {
    auto [all_flights_temp, energy_costs_temp, profits_temp, loads_temp] = dep.compute_solution_space();

    // sort according to profits_temp (decreasing order)
    vector<pair<int, int> > Ri;
    for (int i = 0; i < all_flights_temp.size(); i++) {
        Ri.emplace_back(dep.compute_profit(all_flights_temp[i]), i);
    }
    sort(Ri.begin(), Ri.end(), greater<>());

    vector<vector<int>> all_flights;
    vector<double> energy_costs;
    vector<int> profits;
    vector<int> launches;
    vector<int> rendezvouses;

    for (auto [value, i]: Ri) {
        all_flights.push_back(all_flights_temp[i]);
        auto [l, r] = dep.compute_LR(all_flights_temp[i]);
        launches.push_back(l);
        rendezvouses.push_back(r);
        energy_costs.push_back(energy_costs_temp[i]);
        profits.push_back(profits_temp[i]);
    }

    return flight_selection_in_heu(all_flights, energy_costs, profits, launches, rendezvouses);
}

solution algorithms::greedy_energy() {
    auto [all_flights_temp, energy_costs_temp, profits_temp, loads_temp] = dep.compute_solution_space();

    // sort according to energy_costs_temp
    vector<pair<double, int> > Ri;
    for (int i = 0; i < all_flights_temp.size(); i++) {
        Ri.emplace_back(energy_costs_temp[i], i);
    }
    sort(Ri.begin(), Ri.end());

    vector<vector<int>> all_flights;
    vector<double> energy_costs;
    vector<int> profits;
    vector<int> launches;
    vector<int> rendezvouses;

    for (auto [value, i]: Ri) {
        all_flights.push_back(all_flights_temp[i]);
        auto [l, r] = dep.compute_LR(all_flights_temp[i]);
        launches.push_back(l);
        rendezvouses.push_back(r);
        energy_costs.push_back(energy_costs_temp[i]);
        profits.push_back(dep.compute_profit(all_flights_temp[i]));
    }

    return flight_selection_in_heu(all_flights, energy_costs, profits, launches, rendezvouses);
}

solution algorithms::greedy_profit_energy() {
    auto [all_flights_temp, energy_costs_temp, profits_temp, loads_temp] = dep.compute_solution_space();

    vector<pair<double, int>> Ri;
    for (int i = 0; i < all_flights_temp.size(); i++) {
        double ratio = static_cast<double>(profits_temp[i]) / energy_costs_temp[i];
        Ri.emplace_back(ratio, i);
    }
    sort(Ri.begin(), Ri.end(), greater<>());

    vector<vector<int>> all_flights;
    vector<double> energy_costs;
    vector<int> profits;
    vector<int> launches;
    vector<int> rendezvouses;

    for (auto [value, i]: Ri) {
        all_flights.push_back(all_flights_temp[i]);
        auto [l, r] = dep.compute_LR(all_flights_temp[i]);
        launches.push_back(l);
        rendezvouses.push_back(r);
        energy_costs.push_back(energy_costs_temp[i]);
        profits.push_back(profits_temp[i]);
    }

    return flight_selection_in_heu(all_flights, energy_costs, profits, launches, rendezvouses);
}

solution algorithms::greedy_profit_load() {
    auto [all_flights_temp, energy_costs_temp, profits_temp, loads_temp] = dep.compute_solution_space();

    vector<pair<double, int>> Ri;
    for (int i = 0; i < all_flights_temp.size(); i++) {
        double ratio = static_cast<double>(profits_temp[i]) / loads_temp[i];
        Ri.emplace_back(ratio, i);
    }
    sort(Ri.begin(), Ri.end(), greater<>());

    vector<vector<int>> all_flights;
    vector<double> energy_costs;
    vector<int> profits;
    vector<int> launches;
    vector<int> rendezvouses;

    for (auto [value, i]: Ri) {
        all_flights.push_back(all_flights_temp[i]);
        auto [l, r] = dep.compute_LR(all_flights_temp[i]);
        launches.push_back(l);
        rendezvouses.push_back(r);
        energy_costs.push_back(energy_costs_temp[i]);
        profits.push_back(profits_temp[i]);
    }

    return flight_selection_in_heu(all_flights, energy_costs, profits, launches, rendezvouses);
}

//bool algorithms::if_flight_extends(const vector<int> &flight, int delivery, double remaining_energy) {
//    // compute energy add load by adding delivery to flight
//    vector<int> extended_flight = flight;
//    extended_flight.push_back(delivery);
//    double energy_cost = dep.compute_energy(extended_flight);
//    int load = dep.compute_load(extended_flight);
//
//    if (energy_cost <= remaining_energy && load <= dep.get_drone_load()) {
//        return true;
//    } else {
//        return false;
//    }
//}

//solution algorithms::max_profit_extended() {
//    //auto set = dep.compute_all_flights_using_knapsack();
//
//    // vector<vector<int>> flightss;
//    // for (auto f: flightss){
//    //     for (auto i:f){
//    //         cout << i << " ";
//    //     }
//    //     cout << endl;
//
//    // }
//
//    // A: sort deliveries based on profit
//    vector<int> A;
//    vector<int> profits = dep.get_profits();
//    vector<pair<double, int> > Ri;
//    for (int i = 0; i < profits.size(); i++) {
//        Ri.emplace_back(profits[i], i);
//    }
//
//    sort(Ri.begin(), Ri.end(), greater<>());
//
//    for (auto [value, i]: Ri) {
//        A.push_back(i);
//    }
//
//    // C: sort deliveries based on delivery_points
//    vector<int> C;
//    vector<int> delivery_points = dep.get_delivery_points();
//    vector<pair<double, int> > pi;
//    for (int i = 0; i < profits.size(); i++) {
//        pi.emplace_back(delivery_points[i], i);
//    }
//
//    sort(pi.begin(), pi.end());
//
//    for (auto [value, i]: pi) {
//        C.push_back(i);
//    }
//
//    vector<int> loads = dep.get_weights();
//
//    double B = dep.get_drone_battery();
//    vector<vector<int>> flights;
//    double cost = 0.0;
//
//    while (!A.empty() && cost < B) {   // visited is not enough: && visited < n
//        int delivery_maxP = A[0]; // id of delivery
//        double cost_maxP = dep.compute_energy({delivery_maxP});
//        if (cost_maxP <= B - cost) { // single delivery (delivery_maxP), load satisfied
//            // there is a flight
//            vector<int> flight;
//            flight.push_back(delivery_maxP);
//
//            // find index of delivery_maxP in C
//            auto index = find(C.begin(), C.end(), delivery_maxP);
//            int index_maxP_C = static_cast<int>(index - C.begin());
//
//            bool go_left = true;
//            bool go_right = true;
//
//            int left_increment = 1;
//            int right_increment = 1;
//
//            while (go_left || go_right) { // till we can extend
//                int left_index = index_maxP_C - left_increment;
//                int right_index = index_maxP_C + right_increment;
//
//                // update go_left and right based on index
//                if (left_index < 0) {
//                    go_left = false;
//                }
//
//                if (right_index > C.size() - 1) {
//                    go_right = false;
//                }
//
//                if (go_left && go_right) { // 1
//                    int left_delivery = C[left_index];
//                    int right_delivery = C[right_index];
//
//                    // check if flight extends from left and right, then 4 case
//                    bool from_left = if_flight_extends(flight, left_delivery, B - cost);
//                    bool from_right = if_flight_extends(flight, right_delivery, B - cost);
//
//                    if (from_left && from_right) { // 1:1 both can be extended
//                        // select the most profitable one
//                        if (profits[left_delivery] >= profits[right_delivery]) {
//                            left_increment++;
//                            flight.push_back(left_delivery);
//                        } else {
//                            right_increment++;
//                            flight.push_back(right_delivery);
//                        }
//                    } else if (from_left && !from_right) {  // 1:2
//                        left_increment++;
//                        flight.push_back(left_delivery);
//                        go_right = false;
//                    } else if (!from_left && from_right) {  // 1:3
//                        right_increment++;
//                        flight.push_back(right_delivery);
//                        go_left = false;
//                    } else {  // 1:4    if (from_left == false && from_right == false)
//                        go_left = false;
//                        go_right = false;
//                    }
//                } else if (go_left) { // 2
//                    int left_delivery = C[left_index];
//                    bool from_left = if_flight_extends(flight, left_delivery, B - cost);
//                    if (from_left) {
//                        left_increment++;
//                        flight.push_back(left_delivery);
//                    } else {
//                        go_left = false;
//                    }
//                } else if (go_right) { // 3
//                    int right_delivery = C[right_index];
//                    bool from_right = if_flight_extends(flight, right_delivery, B - cost);
//                    if (from_right) {
//                        right_increment++;
//                        flight.push_back(right_delivery);
//                    } else {
//                        go_right = false;
//                    }
//                }
//
//            }// second while
//
//            // after while, go_left == go_right = false, then update A, C, cost
//            flights.push_back(flight);
//            cost = cost + dep.compute_energy(flight);
//            // remove flight from A and C and all the deliveries intersect with flight
//            for (int f: flight) {
//                A.erase(find(A.begin(), A.end(), f));
//                C.erase(find(C.begin(), C.end(), f));
//            }
//
//            auto points = dep.compute_LR(flight);
//            vector<int> launches = dep.get_launches();
//            vector<int> rendezvouses = dep.get_rendezvouses();
//
//            vector<int> delivery_temp;
//            for (int i: A) {
//                if (!deployment::check_correct_interval({flight}, {get<0>(points)}, {get<1>(points)}, launches[i],
//                                                        rendezvouses[i])) {
//                    delivery_temp.push_back(i);
//                }
//            }
//
//            for (int f: delivery_temp) {
//                A.erase(find(A.begin(), A.end(), f));
//                C.erase(find(C.begin(), C.end(), f));
//            }
//
//        } else {
//            // remove delivery_maxP from A and C
//            A.erase(find(A.begin(), A.end(), delivery_maxP));
//            C.erase(find(C.begin(), C.end(), delivery_maxP));
//        } // first if-else
//
//    } // first while
//
//    vector<int> sel_int_profits;
//    vector<double> sel_int_energies;
//    vector<int> sel_int_loads;
//
//    int total_profit = 0;
//    double total_energy = 0.0;
//
//    for (const auto &f: flights) {
//        int tmp_profit = 0;
//        double tmp_energy_cost = 0.0;
//        int tmp_load = 0;
//        for (int j: f) {
//            tmp_profit += dep.get_profits()[j];
//            tmp_load += dep.get_weights()[j];
//        }
//
//        sel_int_profits.push_back(tmp_profit);
//        total_profit += tmp_profit;
//        sel_int_loads.push_back(tmp_load);
//        tmp_energy_cost = dep.compute_energy(f);
//        sel_int_energies.push_back(tmp_energy_cost);
//        total_energy += tmp_energy_cost;
//    }
//
//    sol.total_profit = total_profit;
//    sol.total_energy = total_energy;
//    sol.total_flights = flights;
//    sol.profits = sel_int_profits;
//    sol.weights = sel_int_loads;
//    sol.energies = sel_int_energies;
//
//    return sol;
//}