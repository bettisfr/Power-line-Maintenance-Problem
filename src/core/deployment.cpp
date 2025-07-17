#include <random>
#include <iostream>
#include <set>
#include <algorithm>
#include "deployment.h"
#include "../util/util.h"

using namespace std;

double numpy(mt19937 &g) {
    const int a = static_cast<int>(g() >> 5);
    const int b = static_cast<int>(g() >> 6);
    const double value = (a * 67108864.0 + b) / 9007199254740992.0;
    return value;
}

deployment::deployment(const input &par) {
    // creating a random instance by using par.seed
    int seed = par.seed;
    // This must be static, otherwise during the next iteration "g" will be recreated, while if static it remains alive
    static mt19937 g(seed);

    int max_len_road = par.max_len_road;            // for random
    double max_interval_len = par.max_interval_len;     
    int max_profit = par.max_profit;
    int max_load = par.max_weight;
    int regularly_spaced = par.regularly_spaced;

    double deliveries_starting_point = par.deliveries_starting_point;
    double error = par.error;
    double exponent = par.exponent;

    uniform_real_distribution uniform_dist(-error, error);

    num_deliveries = par.num_deliveries;
    unit_weight = max_load == 1;

    height = par.height;
    distance = par.distance;
    energy_unit_cost = par.energy_unit_cost;
    energy_per_delivery = par.energy_per_delivery;

    drone_battery = par.drone_battery;
    drone_load = par.drone_load;

    solution_space = par.solution_space;

    for (int i = 1; i <= num_deliveries; i++) {

        double delivery_location = 0.0;
        if (regularly_spaced == 1) {
            delivery_location = i * deliveries_starting_point + uniform_dist(g);
        } else {        // random
            delivery_location = numpy(g) * max_len_road;
        }

        // max_interval_len is too much when regularly_spaced == 1
        double departure = delivery_location - numpy(g) * max_interval_len;
        if (departure < 0) {
            departure = 0;
        }

        double arrival = delivery_location + numpy(g) * (max_interval_len - (delivery_location - departure));
        
        if (arrival == departure) {
            arrival++;
        }
        if (regularly_spaced == 0 && arrival > max_len_road) {
            arrival = max_len_road;
        }

        // int profit = static_cast<int>(numpy(g) * max_profit) + 1; // use Zip
        int weight = static_cast<int>(numpy(g) * max_load) + 1;

        delivery_points.push_back(delivery_location);
        launches.push_back(departure);
        rendezvouses.push_back(arrival);
        // profits.push_back(profit);
        weights.push_back(weight);
    }

    // Zipf distribution for profits
    double H_Ns = 0.0;
    vector<double> probs;

    // Compute normalization constant for Zipf
    for (int i = 1; i <= max_profit; i++)
        H_Ns += 1.0 / pow(i, exponent);

    for (int i = 1; i <= max_profit; i++)
        probs.push_back(1.0 / pow(i, exponent) / H_Ns);

    if (exponent == 0.0) {
        // Uniform distribution case: same as before
        for (int i = 0; i < num_deliveries; i++) {
            discrete_distribution discrete_dist(probs.begin(), probs.end());
            int profit = discrete_dist(g) + 1;
            profits.push_back(profit);
        }
    } else {
        // Skewed case with interpolation
        int num_samples = num_deliveries / 5;
        vector<int> anchor_positions;
        vector<int> anchor_values;

        // 1. Compute anchor positions (equally spaced)
        for (int i = 0; i < num_samples; i++) {
            int pos = i * (num_deliveries - 1) / (num_samples - 1);
            anchor_positions.push_back(pos);
        }

        // 2. Generate anchor values from Zipf
        discrete_distribution discrete_dist(probs.begin(), probs.end());
        for (int i = 0; i < num_samples; i++) {
            int profit = discrete_dist(g) + 1;
            anchor_values.push_back(profit);
        }

        // 3. Fill profits via interpolation
        profits.resize(num_deliveries);
        for (int seg = 0; seg < num_samples - 1; seg++) {
            int pos_start = anchor_positions[seg];
            int pos_end = anchor_positions[seg + 1];
            int val_start = anchor_values[seg];
            int val_end = anchor_values[seg + 1];
            int len = pos_end - pos_start;
            for (int i = 0; i <= len; i++) {
                double t = static_cast<double>(i) / len;
                int interpolated = static_cast<int>(round((1 - t) * val_start + t * val_end));
                profits[pos_start + i] = interpolated;
            }
        }
    }

    //////////////////////////////
    // // creating a random instance by using par.seed
    // int seed = par.seed;
    // // This must be static otherwise during the next iteration "g" will be recreated, while if static it remains alive
    // static mt19937 g(seed);

    // int max_len_road = par.max_len_road;
    // double max_interval_len = par.max_interval_len;
    // int max_profit = par.max_profit;
    // int max_load = par.max_weight;

    // num_deliveries = par.num_deliveries;
    // unit_weight = (max_load == 1);

    // height = par.height;
    // distance = par.distance;
    // energy_unit_cost = par.energy_unit_cost;
    // energy_per_delivery = par.energy_per_delivery;

    // drone_battery = par.drone_battery;
    // drone_load = par.drone_load;

    // solution_space = par.solution_space;

    // for (int i = 0; i < num_deliveries; i++) {
    //     double departure = numpy(g) * max_len_road;
    //     double arrival = departure + numpy(g) * max_interval_len;
    //     if (arrival == departure) {
    //         arrival++;
    //     }
    //     if (arrival > max_len_road) {
    //         arrival = max_len_road;
    //     }

    //     int profit = static_cast<int>(numpy(g) * max_profit) + 1;
    //     int weight = static_cast<int>(numpy(g) * max_load) + 1;

    //     double x = arrival - departure;
    //     double delivery_location = departure + (numpy(g) * x) + 1;

    //     // for single delivery
    //     delivery_points.push_back(delivery_location);
    //     launches.push_back(departure);
    //     rendezvouses.push_back(arrival);
    //     profits.push_back(profit);
    //     weights.push_back(weight);
    // }
}


tuple<vector<vector<int>>, vector<double>, vector<int>, vector<int>> deployment::compute_individual_deliveries() {
    vector<vector<int>> all_flights;
    vector<double> energy_flight;

    for (int i = 0; i < num_deliveries; i++){
        all_flights.push_back({i});
    }

    for (const auto& f : all_flights){
        energy_flight.push_back(compute_energy(f));
    }

    return {all_flights, energy_flight, profits, weights};
}


tuple<double, double> deployment::compute_LR(const vector<int> &flight) const {
    const int total = static_cast<int>(flight.size());

    double L;
    double R;
    if (total == 1) {
        L = launches[flight[0]];
        R = rendezvouses[flight[0]];
    } else {
        L = launches[flight[0]];
        R = rendezvouses[flight[1]];
    }

    return {L, R};
}

tuple<vector<vector<int>>, vector<double>, vector<int>, vector<double>, vector<double>> deployment::sorting_with_rendezvouses_in_apx() {
    auto [all_flights_temp, energy_costs_temp, profits_temp, loads_temp] = compute_solution_space();

    vector<double> launches_temp;
    vector<double> rendezvouses_temp;

    for (const auto &flight: all_flights_temp) {
        auto points = compute_LR(flight);
        launches_temp.push_back(get<0>(points));
        rendezvouses_temp.push_back(get<1>(points));
    }

    // sort, according to rendezvouses_temp
    vector<pair<int, int> > Ri;
    for (int i = 0; i < all_flights_temp.size(); i++) {
        Ri.emplace_back(rendezvouses_temp[i], i);
    }

    sort(Ri.begin(), Ri.end());

    vector<vector<int>> all_flights;
    vector<double> energy_costs;
    vector<int> new_profits;
    vector<double> new_launches;
    vector<double> new_rendezvouses;

    for (auto [fst, snd]: Ri) {
        all_flights.push_back(all_flights_temp[snd]);
        energy_costs.push_back(energy_costs_temp[snd]);
        new_launches.push_back(launches_temp[snd]);
        new_rendezvouses.push_back(rendezvouses_temp[snd]);
        new_profits.push_back(profits_temp[snd]);
    }

    return {all_flights, energy_costs, new_profits, new_launches, new_rendezvouses};
}

bool deployment::check_correct_interval(const vector<vector<int>> &flights,
                                        const vector<double> &launches_flights,
                                        const vector<double> &rendezvouses_flights,
                                        const double L, const double R) {
    for (size_t i = 0; i < flights.size(); ++i) {
        // If intervals [launch, rendezvous] and [L, R] overlap
        if (!(rendezvouses_flights[i] < L || launches_flights[i] > R)) {
            return false;
        }
    }
    return true;
}


int deployment::compute_opt(const int id, const vector<double> &launches, const vector<double> &rendezvouses,
                            const vector<int>& profits, const vector<int>& opt,
                            const vector<int>& p) {

    if (id == -1) {
        return 0;
    }
    if (id >= 0 && id < opt.size()) {
        return opt[id];
    }
    return max(profits[id] + compute_opt(p[id], launches, rendezvouses, profits, opt, p),
               compute_opt(id - 1, launches, rendezvouses, profits, opt, p));
}


vector<int> deployment::weighted_interval(const vector<double> &launches, const vector<double> &rendezvouses,
                                          const vector<int> &profits, vector<int> opt,
                                          const vector<int> &p) {                     

    for (int i = 0; i < launches.size(); i++) {
        int opt_i = compute_opt(i, launches, rendezvouses, profits, opt, p);
        opt.push_back(opt_i);
    }

    return opt;
}

vector<int> deployment::find_solution(int j, const vector<double> &launches, const vector<double> &rendezvouses,
                                      const vector<int>& profits, const vector<int>& opt,
                                      const vector<int>& p, vector<int> O) {
    if (j == -1) {
        return O;
    }
    if (profits[j] + opt[p[j]] >= opt[j - 1]) {
        O.push_back(j);
        return find_solution(p[j], launches, rendezvouses, profits, opt, p, O);
    }
    return find_solution(j - 1, launches, rendezvouses, profits, opt, p, O);
}


vector<int> deployment::largest_non_overlap_delivery(const vector<double>& launches, const vector<double>& rendezvouses) {
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
                }
                id = j;
            }
        }
        if (id >= 0) {
            p.push_back(id);
        }
    }

    return p;
}

// find all unique subsets
void deployment::find_subsets(vector<int> &v, const int idx, vector<int> &subset, set<vector<int>> &result) {
    if (!subset.empty())
        result.insert(subset);

    for (int j = idx; j < v.size(); j++) {
        subset.push_back(v[j]);
        find_subsets(v, j + 1, subset, result);
        // Backtrack to drop the element
        subset.pop_back();
    }
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


tuple<vector<vector<int>>, vector<double>, vector<int>, vector<int>> deployment::compute_all_flights_extra_cost_DP() const {
    vector<vector<int>> all_flights;
    vector<double> energy_flight;
    vector<int> loads_flight;

    vector<int> ids;
    for (int i = 0; i < num_deliveries; i++){
        ids.push_back(i);
    }

    for (int id_i: ids) {
        const double L = launches[id_i];  // x-coordinate
        for (int id_j: ids) {
            if (double R = rendezvouses[id_j]; R > L) {
                vector<int> flight;
                if (id_i == id_j) {
                    flight.push_back(id_i);
                } else {
                    flight.push_back(id_i);
                    flight.push_back(id_j);
                }
                // line 6
                if (compute_energy_extra_cost(flight) <= drone_battery && compute_load(flight) <= drone_load){
                    all_flights.push_back(flight);

                    // cout << "id_i: " << id_i << " id_j: " << id_j << endl;
                    // compute all deliveries between L and R 
                    vector<int> deliveries_L_R;
                    for (int id_k: ids) {
                        if (id_k != id_i && id_k != id_j && 
                            delivery_points[id_k] >= delivery_points[id_i] &&
                            delivery_points[id_k] <= delivery_points[id_j]) {
                            deliveries_L_R.push_back(id_k);
                        }
                    }

                    // cout << " deliveries_L_R: " ;
                    // for (int a: deliveries_L_R){
                    //     cout << a << " ";
                    // } cout << endl;

                    const int n = static_cast<int>(deliveries_L_R.size());

                    int available_load = 0;
                    if (id_i == id_j) {
                        available_load = drone_load - weights[id_i];
                    } else {
                        available_load = drone_load - weights[id_i] - weights[id_j];
                    }

                    vector dp(n + 1, vector(available_load + 1, vector(n + 1, 0)));
                 
                    for (int i = 0; i <= n; i++){ // deliveries start from 1
                        for (int w = 0; w <= available_load; w++){
                            for (int t = 1; t <= n; t++){
                                if (compute_energy(flight) + (t+2) * energy_per_delivery <= drone_battery){
                                    if (t == 1){ // can be removed
                                        if (i == 0 || w == 0){
                                            dp[i][w][t] = 0;
                                        } else if (w >= weights[deliveries_L_R[i-1]]){
                                            dp[i][w][t] = max(profits[deliveries_L_R[i-1]], dp[i-1][w][t]);
                                        } else if(w < weights[deliveries_L_R[i-1]]){
                                            dp[i][w][t] = dp[i-1][w][t];
                                        }
                                    } else { // t >= 2
                                        if (i == 0 || w == 0){
                                            dp[i][w][t] = 0;
                                        } else if (w >= weights[deliveries_L_R[i-1]]){
                                            dp[i][w][t] = max(dp[i-1][w - weights[deliveries_L_R[i-1]]][t-1] + profits[deliveries_L_R[i-1]],
                                                                dp[i-1][w][t]);
                                        } else if(w < weights[deliveries_L_R[i-1]]){
                                            dp[i][w][t] = dp[i-1][w][t];
                                        }
                                    }  
                                }
                            }
                        }                       
                    }

                    for (int t = n; t > 0; t--){
                        vector<int> selectedItems;
                        int w = available_load;
                        int j = n;

                        for (int tt = t; tt > 0; tt--){
                            if (dp[j][w][tt] > 0){
                                for (int i = j; i > 0; i--) {
                                    if (dp[i][w][tt] != dp[i - 1][w][tt]) {
                                        if (find(selectedItems.begin(), selectedItems.end(), deliveries_L_R[i-1]) == selectedItems.end()){
                                            selectedItems.push_back(deliveries_L_R[i-1]);
                                        }
                                        w -= weights[deliveries_L_R[i-1]];
                                        j = i-1;
                                        break;
                                    }
                                }
                            }
                        }

                        // update
                        vector<int> flight_temp;
                        flight_temp = flight;
                        for (int i : selectedItems){
                            flight_temp.push_back(i);
                        }
                        // Use 'set' for all_flights ???? to avoid using 'find'
                        if (find(all_flights.begin(), all_flights.end(), flight_temp) == all_flights.end()){
                            if (compute_energy_extra_cost(flight_temp) <= drone_battery){
                                all_flights.push_back(flight_temp);
                            }
                        }
                    }            
                }
            }
        }
    }

    vector<int> profits_flight;

    for (const auto& f:all_flights){
        energy_flight.push_back(compute_energy_extra_cost(f));
        loads_flight.push_back(compute_load(f));
        profits_flight.push_back(compute_profit(f));

        // for (auto i : f){
        //     cout << i << " ";
        // }
        // cout << " energy: " << compute_energy_extra_cost(f) << " load: " << compute_load(f) << " profit: " << compute_profit(f) << endl;    
    }
    
    return {all_flights, energy_flight, profits_flight, loads_flight};
}

tuple<vector<vector<int>>, vector<double>, vector<int>, vector<int>> deployment::compute_all_flights() const {
    vector<vector<int>> all_flights;
    vector<double> energy_flight;
    vector<int> loads_flight;

    vector<int> ids;
    for (int i = 0; i < num_deliveries; i++){
        ids.push_back(i);
    }

    int type = get_solution_space();

    for (int id_i: ids) {
        double L = launches[id_i];
        for (int id_j: ids) {
            double R = rendezvouses[id_j];
            if (R > L) {
                vector<int> flight;
                if (id_i == id_j) {
                    flight.push_back(id_i);
                } else {
                    flight.push_back(id_i);
                    flight.push_back(id_j);
                }
                double energy_L_R = (type == 0) ? compute_energy(flight) : compute_energy_extra_cost(flight);
                int load_flight = compute_load(flight);

                if (energy_L_R <= drone_battery && load_flight <= drone_load) {
                    all_flights.push_back(flight);
                    energy_flight.push_back(energy_L_R);
                    loads_flight.push_back(load_flight);

                    vector<int> deliveries_L_R;
                    for (int id_k: ids) {
                        if (id_k != id_i && id_k != id_j &&
                            delivery_points[id_k] >= delivery_points[id_i] &&
                            delivery_points[id_k] <= delivery_points[id_j]) {
                            deliveries_L_R.push_back(id_k);
                        }
                    }

                    if (!deliveries_L_R.empty()) {
                        vector<int> indices;
                        for (int i = 0; i < deliveries_L_R.size(); i++) {
                            indices.push_back(i);
                        }
                        vector<vector<int>> all_subsets = compute_all_subsets(indices);

                        for (const auto& f : all_subsets){
                            vector<int> flight_temp = flight;

                            for (const int index : f){
                                flight_temp.push_back(deliveries_L_R[index]);
                            }

                            int load_flight_temp = compute_load(flight_temp);
                            double energy_flight_temp = (type == 0) ? compute_energy(flight_temp) : compute_energy_extra_cost(flight_temp);

                            if (load_flight_temp <= drone_load && energy_flight_temp <= drone_battery){
                                all_flights.push_back(flight_temp);
                                energy_flight.push_back(energy_flight_temp);
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
    }

    return {all_flights, energy_flight, profits_flight, loads_flight};
}


tuple<vector<vector<int>>, vector<double>, vector<int>, vector<int>> deployment::compute_all_flights_equal_weight() {
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
        const int total_load = floor(drone_load / f.first);

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
            const double L = launches[id_i];
            for (int id_j: ids) {
                double R = rendezvouses[id_j];
                if (R > L) {
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
                        // consider only one flight with max profit
                        for (int id_k: ids) {
                            if (id_k != id_i && id_k != id_j && 
                                delivery_points[id_k] >= delivery_points[id_i] &&
                                delivery_points[id_k] <= delivery_points[id_j] && flight.size() < total_load ) {
                                
                                flight.push_back(id_k);
                            }
                        }                        
                        all_flights.push_back(flight);
                        energy_flight.push_back(energy_L_R);
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
        const double L = launches[id_i];
        for (int id_j: ids) {
            const double R = rendezvouses[id_j];
            if (R > L) {            
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
                        if (id_k != id_i && id_k != id_j &&
                            delivery_points[id_k] >= delivery_points[id_i] &&
                            delivery_points[id_k] <= delivery_points[id_j]) {
                            deliveries_L_R.push_back(id_k);
                        }
                    }

                    vector<int> flight_knapsack;
                    if (!deliveries_L_R.empty()) {
                        const int available_load = drone_load - load_flight;
                        flight_knapsack = compute_flight_using_knapsack(deliveries_L_R, available_load);
                    } 

                    for (int f : flight_knapsack){
                        flight.push_back(f);
                    }

                    all_flights.push_back(flight);
                    energy_flight.push_back(energy_L_R);
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

vector<int> deployment::compute_flight_using_knapsack(const vector<int>& deliveries_id, const int total_load) {
    vector<int> weights_temp;
    vector<int> values;

    for (int i:deliveries_id){
        weights_temp.push_back(weights[i]);
        values.push_back(profits[i]);
    }

    const int n = static_cast<int>(deliveries_id.size());
    vector<int> selectedItems;

    // Create a DP table where dp[i][w] will store the maximum value that can be attained with weight w and first i items
    vector dp(n + 1, vector(total_load + 1, 0));

    for (int i = 1; i <= n; ++i) {
        for (int w = 1; w <= total_load; ++w) {
            // "i" can be included in the knapsack
            if (weights_temp[i - 1] <= w) {
                dp[i][w] = max(dp[i - 1][w], values[i - 1] + dp[i - 1][w - weights_temp[i - 1]]);
            } else {
                // "i" cannot be included
                dp[i][w] = dp[i - 1][w];
            }
        }
    }

    // To find the selected items, we backtrack through the DP table
    int w = total_load;
    for (int i = n; i > 0; --i) {
        if (dp[i][w] != dp[i - 1][w]) {  // Item "i" was included
            selectedItems.push_back(i - 1);  // Store the index of the selected item (0-based index)
            w -= weights_temp[i - 1];  // Reduce the remaining weight
        }
    }

    reverse(selectedItems.begin(), selectedItems.begin());
    vector<int> flight_temp;
    for (const int i : selectedItems){
        flight_temp.push_back(deliveries_id[i]);
    }

    // sort flight_temp based on profits
    vector<pair<int, int> > Ri;
    for (int i : flight_temp) {
        Ri.emplace_back(profits[i], i);
    }

    sort(Ri.begin(), Ri.end()), greater<>();

    vector<int> flight;
    for (auto [value, i]: Ri) {
        flight.push_back(i);
    }
    
    return flight;
}

// all_flights: {first_delivery, last_delivery, .... }
double deployment::compute_energy(const vector<int> &delivery_ids) const {
    int first_delivery;
    int last_delivery;

    const int total = static_cast<int>(delivery_ids.size());

    if (total == 1) {
        first_delivery = delivery_ids[0];
        last_delivery = delivery_ids[0];
    } else {
        first_delivery = delivery_ids[0];
        last_delivery = delivery_ids[1];
    }

    const double l = launches[first_delivery];
    const double r = rendezvouses[last_delivery];

//    double a = util::get_2D_distance(l, 0, delivery_points[first_delivery], height);
    const double a = util::get_3D_distance(l, 0, 0, delivery_points[first_delivery], height, distance);
    const double b = abs(delivery_points[first_delivery] - delivery_points[last_delivery]);
//    double c = util::get_2D_distance(delivery_points[last_delivery], height, r, 0);
    const double c = util::get_3D_distance(delivery_points[last_delivery], height, distance, r, 0, 0);

    return (a + b + c) * energy_unit_cost + total * energy_per_delivery;
}


double deployment::compute_energy_extra_cost(const vector<int> &delivery_ids) const {
    return compute_energy(delivery_ids) + energy_per_delivery * static_cast<int>(delivery_ids.size());
}

int deployment::compute_profit(const vector<int> &delivery_ids) const {
    int profit = 0;
    for (const auto id: delivery_ids) {
        profit += profits[id];
    }

    return profit;
}

int deployment::compute_load(const vector<int> &delivery_ids) const {
    int load = 0;
    for (const auto id: delivery_ids) {
        load += weights[id];
    }

    return load;
}

int deployment::get_num_deliveries() const {
    return num_deliveries;
}

const vector<double> &deployment::get_launches() const {
    return launches;
}

const vector<double> &deployment::get_rendezvouses() const {
    return rendezvouses;
}

const vector<int> &deployment::get_profits() const {
    return profits;
}

const vector<int> &deployment::get_weights() const {
    return weights;
}

const vector<double> &deployment::get_delivery_points() const {
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

int deployment::compute_n_prime(const vector<int>& flight) const {
    // If a single mission, return 1
    if (flight.size() == 1) {
        return 1;
    }

    // If multiple mission, return n'+2
    double del_L = numeric_limits<double>::max();
    double del_R = numeric_limits<double>::lowest();

    for (int idx : flight) {
        del_L = min(del_L, delivery_points[idx]);
        del_R = max(del_R, delivery_points[idx]);
    }

    int count = 0;
    for (const double point : delivery_points) {
        if (point >= del_L && point <= del_R) {
            count++;
        }
    }

    return count;
}

tuple<vector<vector<int>>, vector<double>, vector<int>, vector<int>> deployment::compute_solution_space() {
    // if (is_unit_weight()) {
    //     // It works also with unitary weights
    //     return compute_all_flights_equal_weight();
    // }
    const int type = get_solution_space();
    // cout << "type: " << type << endl;
    if (type == 0 or type == 3) {
        // Exhaustively, optimal
        return compute_all_flights();
    }
    if (type == 1) {
        // Knapsack, optimal
        return compute_all_flights_using_knapsack();
    }
    if (type == 2) {
        // It works also with non-unitary weights, suboptimal
        return compute_all_flights_equal_weight();
    }
    if (type == 4) {
        // DP additional cost, optimal
        return compute_all_flights_extra_cost_DP();
    }

    cerr << "Error in compute_solution_space" << endl;
    exit(1);
}
