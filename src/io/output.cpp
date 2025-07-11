#include "output.h"
#include "../util/util.h"

void save_output(const input &par, const vector<solution> &results) {
    string filename = "output/" + par.exp_name + ".csv";
    ofstream file(filename, ofstream::out | ofstream::trunc);

    // Initialize averages and standard deviations
    double total_profit_avg, total_profit_std;
    double total_energy_avg, total_energy_std;
    double total_flights_avg, total_flights_std;
    double running_time_avg, running_time_std;
    double all_flights_size_avg, all_flights_size_std;
    double deliveries_per_flight_avg, deliveries_per_flight_std;
    double avg_energy_per_flight_avg, avg_energy_per_flight_std;
    double avg_weight_per_flight_avg, avg_weight_per_flight_std;
    double n_prime_avg, n_prime_std;

    // Calculate averages and standard deviations
    if (!results.empty()) {
        vector<double> total_profits;
        vector<double> total_energy;
        vector<int> total_flights;
        vector<double> running_times;
        vector<int> all_flights_size;
        vector<double> avg_deliveries_per_flight;
        vector<double> avg_energy_per_flight;
        vector<double> avg_weight_per_flight;
        vector<double> avg_n_prime;

        for (const auto &out : results) {
            total_profits.push_back(out.total_profit);
            total_energy.push_back(out.total_energy);
            total_flights.push_back(static_cast<int>(out.total_flights.size()));
            running_times.push_back(out.running_time);
            all_flights_size.push_back(out.all_flights_size);

            // Compute avg deliveries per flight
            if (!out.deliveries_per_flight.empty()) {
                double sum = 0.0;
                for (int d : out.deliveries_per_flight) {
                    sum += d;
                }
                avg_deliveries_per_flight.push_back(sum / out.deliveries_per_flight.size());
            } else {
                avg_deliveries_per_flight.push_back(0.0);
            }

            if (!out.total_flights.empty()) {
                avg_energy_per_flight.push_back(out.total_energy / out.total_flights.size());

                int total_weight = 0;
                for (int w : out.weights) {
                    total_weight += w;
                }
                avg_weight_per_flight.push_back(static_cast<double>(total_weight) / out.total_flights.size());
            } else {
                avg_energy_per_flight.push_back(0.0);
                avg_weight_per_flight.push_back(0.0);
            }

            // Compute avg n_prime per solution (if any)
            if (!out.n_prime.empty()) {
                double sum = 0.0;
                for (int np : out.n_prime) {
                    sum += np;
                }
                avg_n_prime.push_back(sum / out.n_prime.size());
            } else {
                avg_n_prime.push_back(0.0);
            }
        }

        tie(total_profit_avg, total_profit_std) = util::calculate_avg_std(total_profits);
        tie(total_energy_avg, total_energy_std) = util::calculate_avg_std(total_energy);
        tie(total_flights_avg, total_flights_std) = util::calculate_avg_std(total_flights);
        tie(running_time_avg, running_time_std) = util::calculate_avg_std(running_times);
        tie(all_flights_size_avg, all_flights_size_std) = util::calculate_avg_std(all_flights_size);
        tie(deliveries_per_flight_avg, deliveries_per_flight_std) = util::calculate_avg_std(avg_deliveries_per_flight);
        tie(avg_energy_per_flight_avg, avg_energy_per_flight_std) = util::calculate_avg_std(avg_energy_per_flight);
        tie(avg_weight_per_flight_avg, avg_weight_per_flight_std) = util::calculate_avg_std(avg_weight_per_flight);
        tie(n_prime_avg, n_prime_std) = util::calculate_avg_std(avg_n_prime);
    }

    if (file.is_open()) {
        file
            << "seed,num_deliveries,max_len_road,max_interval_len,max_profit,max_weight,"
            << "drone_battery,drone_load,height,distance,algorithm,solution_space,iterations,"
            << "energy_unit_cost,energy_per_delivery,"
            << "regularly_spaced,deliveries_starting_point,error,exponent,"
            << "total_profit_avg,total_profit_std,"
            << "total_energy_avg,total_energy_std,"
            << "total_flights_avg,total_flights_std,"
            // << "running_time_avg,running_time_std,"
            << "all_flights_size_avg,all_flights_size_std,"
            << "deliveries_per_flight_avg,deliveries_per_flight_std,"
            << "avg_energy_per_flight_avg,avg_energy_per_flight_std,"
            << "avg_weight_per_flight_avg,avg_weight_per_flight_std,"
            << "n_prime_avg,n_prime_std"
            << endl;

        file
            << par.seed << ","
            << par.num_deliveries << ","
            << par.max_len_road << ","
            << par.max_interval_len << ","
            << par.max_profit << ","
            << par.max_weight << ","
            << par.drone_battery << ","
            << par.drone_load << ","
            << par.height << ","
            << par.distance << ","
            << par.algorithm << ","
            << par.solution_space << ","
            << par.iterations << ","
            << par.energy_unit_cost << ","
            << par.energy_per_delivery << ","
            << par.regularly_spaced << ","
            << par.deliveries_starting_point << ","
            << par.error << ","
            << par.exponent << ","
            << total_profit_avg << "," << total_profit_std << ","
            << total_energy_avg << "," << total_energy_std << ","
            << total_flights_avg << "," << total_flights_std << ","
            // << running_time_avg << "," << running_time_std << ","
            << all_flights_size_avg << "," << all_flights_size_std << ","
            << deliveries_per_flight_avg << "," << deliveries_per_flight_std << ","
            << avg_energy_per_flight_avg << "," << avg_energy_per_flight_std << ","
            << avg_weight_per_flight_avg << "," << avg_weight_per_flight_std << ","
            << n_prime_avg << "," << n_prime_std
            << endl;

        file.close();
        cout << "Output saved to: " << filename << endl;
    } else {
        cerr << "Error: Unable to open file " << filename << endl;
    }
}
