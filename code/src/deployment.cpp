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
    // mapping par to private variables

    // creating random instance by using par.seed
    int seed = par.seed;
    mt19937 g(seed);

    int num_delivery = par.num_deliveries;
    int max_len_road = 1000;//non presente in par
    int max_interval_len = 100;
    int max_cost = 10;
    int max_reward = 15;

    //solution=
    vector<vector<int>> input;
    for (int i = 0; i < num_delivery; i++) {
        int departure = numpy(g) * max_len_road;
        int arrival = departure + numpy(g) * max_interval_len;
        if (arrival > max_len_road) {
            arrival = max_len_road;
        }

        int cost = numpy(g) * max_cost;
        int reward = numpy(g) * max_reward;

        vector<int> delivery = {departure, arrival, cost, reward};
        input.push_back(delivery);
    }

    for (const auto& row : input) {
        int departure = row[0];
        int arrival = row[1];
        int cost = row[2];
        int reward = row[3];

        cout << "departure: " << departure
        << ", arrival: " << arrival
        << ", cost: " << cost
        << ", reward: " << reward << endl;
    }

    algorithms algo(this);//ad algo passo l'istanza corrente; se ne creassi un'altra algo non potrebbe accedere ai dati di questa istanza
    algo.greedy_weight_selection(input, 1, 5000);//qui mi chiede 3 parametri
}

int deployment::get_num_drones() {
    return num_drones;
}
