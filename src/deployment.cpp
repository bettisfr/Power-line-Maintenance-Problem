#include <random>
#include <iostream>
#include "deployment.h"
#include "algorithms.h"

using namespace std;

double numpy(mt19937& g) {
    int a = g() >> 5;
    int b = g() >> 6;
    double value = (a * 67108864.0 + b) / 9007199254740992.0;
    return value;
}

deployment::deployment(const input &par) {
    // creating random instance by using par.seed
    int seed = par.seed;
    mt19937 g(seed);

    int num_delivery = par.num_deliveries;
    int max_len_road = par.max_len_road;
    int max_interval_len = par.max_interval_len;
    int max_profit = par.max_profit;
    int max_load = par.max_load;
    
    height = par.height;
    energy_per_flight = par.energy_per_flight;

    drone_battery = par.battery;
    drone_load = par.load;

    for (int i = 0; i < num_delivery; i++) {
        int departure = static_cast<int>(numpy(g) * max_len_road);
        int arrival = static_cast<int>(departure + numpy(g) * max_interval_len);
        if (arrival == departure) {
            arrival++;
        }
        if (arrival > max_len_road) {
            arrival = max_len_road;
        }

        int profit = static_cast<int>(numpy(g) * max_profit) + 1;
        int load = static_cast<int>(numpy(g) * max_load) + 1;

        int x = arrival - departure;
        int delivery_location = departure + static_cast<int>(numpy(g) * x) + 1;

        delivery_points.push_back(delivery_location);
        launches.push_back(departure);
        rendezvouses.push_back(arrival);
        profits.push_back(profit);
        loads.push_back(load);

        cout << i << "-> " << delivery_location
             << ": [" << departure
             << ", " << arrival
             << "], profit=" << profit
             << ", load=" << load << endl;
    }

    cout << endl;

    //vector<vector<int>> flights = compute_all_flights();
}

double deployment::get_distance(int x1, int y1, int x2, int y2) {
    double delta_x = x2 - x1;
    double delta_y = y2 - y1;

    return sqrt(delta_x * delta_x + delta_y * delta_y);
}

double deployment::compute_energy_cost(vector<int>delivery_ids){
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

vector<vector<int>> deployment::compute_all_flights(){
    // For any lunch L and rendezvous point R, compute the set of deliveris such that their
    // lunch and rendezvous point lies in [L, R]
    vector<vector<int>> all_flights;
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
                            cout << "L: " << L << " , " << "R: " << R << " -- ";
                            cout << " energy: " << energy << " , delivery: ";
                            for (auto i:flight){
                                    cout << i << ", ";
                            }
                            cout << endl;
                        }
                    }
                }
            }
        }
    }

    return all_flights;
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

const vector<int> &deployment::get_loads() const {
    return loads;
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

int deployment::get_height() const {
    return height;
}

int deployment::get_energy_per_flight() const {
    return energy_per_flight;
}
