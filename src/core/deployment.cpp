#include <random>
#include <iostream>
#include <set>
#include "deployment.h"
#include "../util/util.h"

using namespace std;

double numpy(mt19937 &g) {
    int a = static_cast<int>(g() >> 5);
    int b = static_cast<int>(g() >> 6);
    double value = (a * 67108864.0 + b) / 9007199254740992.0;
    return value;
}

deployment::deployment(const input &par) {
    // creating random instance by using par.seed
    int seed = par.seed;
    // This must be static otherwise during the next iteration "g" will be recreated, while if static it remains alive
    static mt19937 g(seed);

    int max_len_road = par.max_len_road;
    int max_interval_len = par.max_interval_len;
    int max_profit = par.max_profit;
    int max_load = par.max_weight;

    num_deliveries = par.num_deliveries;
    unit_weight = (max_load == 1);

    height = par.height;
    energy_unit_cost = par.energy_unit_cost;

    drone_battery = par.drone_battery;
    drone_load = par.drone_load;

    solution_space = par.solution_space;

    for (int i = 0; i < num_deliveries; i++) {
        int departure = static_cast<int>(numpy(g) * max_len_road);
        int arrival = static_cast<int>(departure + numpy(g) * max_interval_len);
        if (arrival == departure) {
            arrival++;
        }
        if (arrival > max_len_road) {
            arrival = max_len_road;
        }

        int profit = static_cast<int>(numpy(g) * max_profit) + 1;
        int weight = static_cast<int>(numpy(g) * max_load) + 1;

        int x = arrival - departure;
        int delivery_location = departure + static_cast<int>(numpy(g) * x) + 1;

        delivery_points.push_back(delivery_location);
        launches.push_back(departure);
        rendezvouses.push_back(arrival);
        profits.push_back(profit);
        weights.push_back(weight);
    }
}

tuple<int, int> deployment::compute_LR(const vector<int> &flight) {
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

tuple<vector<vector<int>>, vector<double>, vector<int>, vector<int>, vector<int>> deployment::sorting_with_rendezvouses_in_apx() {
    // compute all flights and energy, then sort them
    vector<int> deliveries_id;
    for (int i = 0; i < num_deliveries; i++) {
        deliveries_id.push_back(i);
    }

    auto sets = compute_all_flights_unitary_weight();

    vector<vector<int>> all_flights_temp = get<0>(sets);
    vector<double> energy_costs_temp = get<1>(sets);

    vector<int> launches_temp;
    vector<int> rendezvouses_temp;

    for (const auto &flight: all_flights_temp) {
        auto points = compute_LR(flight);
        launches_temp.push_back(get<0>(points));
        rendezvouses_temp.push_back(get<1>(points));
    }

    // sort according to rendezvouses_temp
    vector<pair<int, int> > Ri;
    for (int i = 0; i < all_flights_temp.size(); i++) {
        Ri.emplace_back(rendezvouses_temp[i], i);
    }

    sort(Ri.begin(), Ri.end());

    vector<vector<int>> all_flights;
    vector<double> energy_costs;
    vector<int> new_profits;
    vector<int> new_launches;
    vector<int> new_rendezvouses;

    for (auto it: Ri) {
        all_flights.push_back(all_flights_temp[it.second]);
        energy_costs.push_back(energy_costs_temp[it.second]);
        new_launches.push_back(launches_temp[it.second]);
        new_rendezvouses.push_back(rendezvouses_temp[it.second]);
        new_profits.push_back(compute_profit(all_flights_temp[it.second]));
    }

    return {all_flights, energy_costs, new_profits, new_launches, new_rendezvouses};
}

bool deployment::check_correct_interval(const vector<vector<int>> &flights, vector<int> launches_flights,
                                        vector<int> rendezvouses_flights, int L, int R) {
    // no intersection: true
    for (int i = 0; i < flights.size(); i++) {
        if (L <= launches_flights[i] && launches_flights[i] <= R) {
            return false;
        }

        if ((launches_flights[i] <= L && L <= rendezvouses_flights[i]) ||
            (launches_flights[i] <= R && R <= rendezvouses_flights[i])) {
            return false;
        }

        if (L <= rendezvouses_flights[i] && rendezvouses_flights[i] <= R) {
            return false;
        }
    }

    return true;
}

int deployment::compute_opt(int id, const vector<int> &launches, const vector<int> &rendezvouses,
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


vector<int> deployment::weighted_interval(const vector<int> &launches, const vector<int> &rendezvouses,
                                          const vector<int> &profits, vector<int> opt,
                                          const vector<int> &p) {

    for (int i = 0; i < launches.size(); i++) {
        int opt_i = compute_opt(i, launches, rendezvouses, profits, opt, p);
        opt.push_back(opt_i);
    }

    return opt;
}

vector<int> deployment::find_solution(int j, const vector<int> &launches, const vector<int> &rendezvouses,
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

// to find all unique subsets
void deployment::find_subsets(vector<int> &v, int idx, vector<int> &subset, set<vector<int>> &result) {
    if (!subset.empty())
        result.insert(subset);

    for (int j = idx; j < v.size(); j++) {
        subset.push_back(v[j]);
        find_subsets(v, j + 1, subset, result);
        // Backtrack to drop the element
        subset.pop_back();
    }
}

bool deployment::check_intersection(const vector<int> &flight1, vector<int> flight2) {
    for (int i: flight1) {
        if (find(flight2.begin(), flight2.end(), i) != flight2.end()) { // if intersection
            return true;
        }
    }
    return false;
}

vector<int> deployment::largest_non_overlap_delivery(vector<int> launches, vector<int> rendezvouses) {
    vector<int> p;  // >= -1
    for (int i = 0; i < launches.size(); i++) {
        int id = -10;
        for (int j = 0; j < launches.size(); j++) {
            if (i != j) {
                if (rendezvouses[j] >= launches[i]) {
                    id = j - 1;
                    if (id == i) {
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
        if (id >= 0) {
            p.push_back(id);
        }
    }

    return p;
}

// return all unique subsets
vector<vector<int> > deployment::compute_all_subsets(vector<int> &v) {
    set<vector<int> > result;
    vector<int> subset;
    find_subsets(v, 0, subset, result);
    vector<vector<int> > res;
    for (const auto &tmp: result)
        res.push_back(tmp);

    return res;
}

tuple<vector<vector<int>>, vector<double>, vector<int>, vector<int>> deployment::compute_all_flights_arbitrary_weight() {
    // consider all deliveries and compute all possible flights
    vector<int> ids;
    for (int i = 0; i < launches.size(); i++) {
        ids.push_back(i);
    }

    vector<vector<int> > all_subsets = compute_all_subsets(ids);
    // for each subset in all_subsets, check drone_load and energy
    vector<vector<int>> all_flights;
    vector<double> energy_flight;

    for (const auto &flight: all_subsets) {
        double energy = compute_energy(flight);
        int load = compute_load(flight);
        if (energy <= drone_battery && load <= drone_load) {
            all_flights.push_back(flight);
            energy_flight.push_back(energy);
        }
    }

    vector<int> profits_flight;
    vector<int> loads_flight;

    for (const auto &flight: all_flights) {
        profits_flight.push_back(compute_profit(flight));
        loads_flight.push_back(compute_load(flight));
    }

    return {all_flights, energy_flight, profits_flight, loads_flight};
}

// TODO
tuple<vector<vector<int>>, vector<double>, vector<int>, vector<int>> deployment::compute_all_flights_new() {
    vector<vector<int>> all_flights;
    vector<double> energy_flight;
    vector<int> loads_flight;

    vector<int> ids;
    for (int i = 0; i < num_deliveries; i++){
        ids.push_back(i);
    }
    
    for (int id_i: ids) {
        int L = launches[id_i];
        for (int id_j: ids) {
            int R = rendezvouses[id_j];
            if (R > L && rendezvouses[id_i] <= R && launches[id_j] >= L) {                  
                vector<int> flight;
                if (id_i == id_j) {
                    flight.push_back(id_i);
                } else {
                    flight.push_back(id_i);
                    flight.push_back(id_j);
                }
                // check [L, R] is energy and drone_load feasible
                double energy_L_R = compute_energy(flight);
                int load_flight = compute_load(flight);

                if (energy_L_R <= drone_battery && load_flight <= drone_load) {
                    all_flights.push_back(flight);
                    energy_flight.push_back(energy_L_R);
                    loads_flight.push_back(load_flight);

                    // compute all deliveries between L and R 
                    vector<int> deliveries_L_R;
                    for (int id_k: ids) {
                        if (id_k != id_i && id_k != id_j && L <= launches[id_k] && launches[id_k] <= R &&
                            L <= rendezvouses[id_k] && rendezvouses[id_k] <= R &&
                            delivery_points[id_k] >= delivery_points[id_i] &&
                            delivery_points[id_k] <= delivery_points[id_j]) {
                            deliveries_L_R.push_back(id_k);
                        }
                    }

                    if (!deliveries_L_R.empty()) {                        
                        vector<int> flights_L_R; // after computing, update indices based on deliveries_L_R
                        // using deliveries_L_R[indices]
                        // compute all flights for deliveries_L_R, then for each flight check <= drone_load - load_flight
                        vector<int> indices;
                        for (int i = 0; i < deliveries_L_R.size(); i++) {
                            indices.push_back(i);
                        }
                        vector<vector<int> > all_subsets = compute_all_subsets(indices);

                        for (auto f : all_subsets){
                            vector<int> flight_temp;
                            for (int index : f){
                                flight_temp.push_back(deliveries_L_R[index]);
                            }
                            for (int i : flight){
                                flight_temp.push_back(i);
                            }

                            int load_flight_temp = compute_load(flight_temp);
                            if (load_flight_temp <= drone_load - load_flight){
                                all_flights.push_back(flight_temp);
                                energy_flight.push_back(energy_L_R);
                                loads_flight.push_back(load_flight_temp);
                            }
                        }
                    } 
                }
            }
        }
    }

    vector<int> profits_flight;
    for (const auto &flight: all_flights) {
        profits_flight.push_back(compute_profit(flight));
        //loads_flight.push_back(compute_load(flight));
    }

    return {all_flights, energy_flight, profits_flight, loads_flight};
}




tuple<vector<vector<int>>, vector<double>, vector<int>, vector<int>> deployment::compute_all_flights_unitary_weight() {
    set<int> unique_loads;
    for (int l: weights) {
        unique_loads.insert(l);
    }
   
    // for each load in unique_loads compute flights
    map<int, vector<int>> load_flight;

    for (auto l: unique_loads) {
        load_flight[l] = vector<int>();
    }

    for (int i = 0; i < weights.size(); i++) {
        load_flight[weights[i]].push_back(i);
    }

    vector<vector<int>> all_flights;
    vector<double> energy_flight;

    for (const auto &f: load_flight) {
        int total_load = floor(drone_load / f.first);

        vector<int> deliveries_id = f.second;

        vector<pair<int, int>> profit_id;
        for (int i: deliveries_id) {
            profit_id.emplace_back(profits[i], i);
        }

        sort(profit_id.begin(), profit_id.end());

        vector<int> ids;
        for (auto j: profit_id) {
            ids.push_back(j.second);
        }
        reverse(ids.begin(), ids.end());

        for (int id_i: ids) {
            int L = launches[id_i];
            for (int id_j: ids) {
                int R = rendezvouses[id_j];
                if (R > L && rendezvouses[id_i] <= R && launches[id_j] >= L) {  // id_i != id_j &&                     
                    vector<int> flight;
                    if (id_i == id_j) {
                        flight.push_back(id_i);
                    } else {
                        flight.push_back(id_i);
                        flight.push_back(id_j);
                    }
                    // check [L, R] is energy and drone_load feasible
                    double energy_L_R = compute_energy(flight);
                    if (energy_L_R <= drone_battery && flight.size() <= total_load) {
                        // add all flights of size <= L
                        all_flights.push_back(flight);
                        energy_flight.push_back(energy_L_R);
                        for (int id_k: ids) {
                            if (id_k != id_i && id_k != id_j && L <= launches[id_k] && launches[id_k] <= R &&
                                L <= rendezvouses[id_k] && rendezvouses[id_k] <= R &&
                                delivery_points[id_k] >= delivery_points[id_i] &&
                                delivery_points[id_k] <= delivery_points[id_j] && flight.size() < total_load) {
                                flight.push_back(id_k);
                                all_flights.push_back(flight);
                                energy_flight.push_back(energy_L_R);
                            }
                        }
                    }
                }
            }
        }
    }

    vector<int> profits_flight;
    vector<int> loads_flight;

    for (const auto &flight: all_flights) {
        profits_flight.push_back(compute_profit(flight));
        loads_flight.push_back(compute_load(flight));
    }

    return {all_flights, energy_flight, profits_flight, loads_flight};
}

tuple<vector<vector<int>>, vector<double>, vector<int>, vector<int>> deployment::compute_all_flights_using_knapsack() {
    vector<vector<int>> all_flights;
    vector<double> energy_flight;

    vector<int> ids;
    for (int i = 0; i < num_deliveries; i++){
        ids.push_back(i);
    }
    
    for (int id_i: ids) {
        int L = launches[id_i];
        for (int id_j: ids) {
            int R = rendezvouses[id_j];
            if (R > L && rendezvouses[id_i] <= R && launches[id_j] >= L) {                  
                vector<int> flight;
                if (id_i == id_j) {
                    flight.push_back(id_i);
                } else {
                    flight.push_back(id_i);
                    flight.push_back(id_j);
                }
                // check [L, R] is energy and drone_load feasible
                double energy_L_R = compute_energy(flight);
                int load_flight = compute_load(flight);

                if (energy_L_R <= drone_battery && load_flight <= drone_load) {
                    // run knapsack for all deliveries between L and R
                    vector<int> deliveries_L_R;
                    for (int id_k: ids) {
                        if (id_k != id_i && id_k != id_j && L <= launches[id_k] && launches[id_k] <= R &&
                            L <= rendezvouses[id_k] && rendezvouses[id_k] <= R &&
                            delivery_points[id_k] >= delivery_points[id_i] &&
                            delivery_points[id_k] <= delivery_points[id_j]) {
                            deliveries_L_R.push_back(id_k);
                        }
                    }

                    all_flights.push_back(flight);
                    energy_flight.push_back(energy_L_R);

                    if (!deliveries_L_R.empty()) {
                        vector<int> flight_knapsack = compute_flight_using_knapsack(deliveries_L_R, drone_load - load_flight);
                        for (int f : flight_knapsack){
                            flight.push_back(f);
                            all_flights.push_back(flight);
                            energy_flight.push_back(energy_L_R);
                        }
                    } 
                }
            }
        }
    }

    vector<int> profits_flight;
    vector<int> loads_flight;

    for (const auto &flight: all_flights) {
        profits_flight.push_back(compute_profit(flight));
        loads_flight.push_back(compute_load(flight));
    }

    return {all_flights, energy_flight, profits_flight, loads_flight};
}


// from chatgpt
vector<int> deployment::compute_flight_using_knapsack(vector<int> deliveries_id, int total_load) {
    vector<int> weights_temp;
    vector<int> values;

    for (int i:deliveries_id){
        weights_temp.push_back(weights[i]);
        values.push_back(profits[i]);
    }

    int n = static_cast<int>(deliveries_id.size());
    vector<int> selectedItems;

    // Create a DP table where dp[i][w] will store the maximum value that can be attained with weight w and first i items
    vector<vector<int>> dp(n + 1, vector<int>(total_load + 1, 0));

    // Build the DP table in a bottom-up manner
    for (int i = 1; i <= n; ++i) {
        for (int w = 1; w <= total_load; ++w) {
            // If the current item can be included in the knapsack
            if (weights_temp[i - 1] <= w) {
                // Take the maximum of either including or excluding the item
                dp[i][w] = max(dp[i - 1][w], values[i - 1] + dp[i - 1][w - weights_temp[i - 1]]);
            } else {
                // If the item cannot be included, take the value from the row above (exclude the item)
                dp[i][w] = dp[i - 1][w];
            }
        }
    }

    // To find the selected items, we backtrack through the DP table
    int w = total_load;
    for (int i = n; i > 0; --i) {
        if (dp[i][w] != dp[i - 1][w]) {  // Item i was included
            selectedItems.push_back(i - 1);  // Store the index of the selected item (0-based index)
            w -= weights_temp[i - 1];  // Reduce the remaining weight
        }
    }

    // Return the maximum value found in the bottom-right corner of the DP table
    //cout << "max_profit: " << dp[n][total_load] << endl;

    reverse(selectedItems.begin(), selectedItems.begin());
    vector<int> flight;
    for (int i : selectedItems){
        flight.push_back(deliveries_id[i]);
    }
    
    return flight;
}


double deployment::compute_energy(const vector<int> &delivery_ids) {
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

    return (a + b + c) * energy_unit_cost;
}

int deployment::compute_profit(const vector<int> &delivery_ids) {
    int profit = 0;
    for (auto id: delivery_ids) {
        profit += profits[id];
    }

    return profit;
}

int deployment::compute_load(const vector<int> &delivery_ids) {
    int load = 0;
    for (auto id: delivery_ids) {
        load += weights[id];
    }

    return load;
}

int deployment::get_num_deliveries() const {
    return num_deliveries;
}

const vector<int> &deployment::get_launches() const {
    return launches;
}

const vector<int> &deployment::get_rendezvouses() const {
    return rendezvouses;
}

const vector<int> &deployment::get_profits() const {
    return profits;
}

const vector<int> &deployment::get_weights() const {
    return weights;
}

const vector<int> &deployment::get_delivery_points() const {
    return delivery_points;
}

int deployment::get_drone_battery() const {
    return drone_battery;
}

int deployment::get_drone_load() const {
    return drone_load;
}

bool deployment::is_unit_weight() const {
    return unit_weight;
}

int deployment::get_solution_space() const {
    return solution_space;
}

tuple<vector<vector<int>>, vector<double>, vector<int>, vector<int>> deployment::compute_solution_space() {
    return is_unit_weight()
                ? compute_all_flights_unitary_weight()
                : (get_solution_space() == 0
                   ? compute_all_flights_arbitrary_weight() //compute_all_flights_new()  ?????
                   : compute_all_flights_using_knapsack());
}
