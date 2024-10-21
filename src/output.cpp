#include "output.h"

void save_output(const input &par, vector<solution> &results) {
    string filename = "output/" + par.exp_name + ".csv";
    ofstream file(filename, ofstream::out | ofstream::trunc);

    // to implement
}