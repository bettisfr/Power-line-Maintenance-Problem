#ifndef DEPLOYMENT_H
#define DEPLOYMENT_H

#include <iostream>
#include <vector>
#include <random>
#include <numeric>

#include "input.h"
#include "definitions.h"

using namespace std;

class deployment {

private:
    // drones, budgets, etc
    int num_drones;

public:
    explicit deployment(const input &);

    // get/set
    int get_num_drones();

};

#endif //DEPLOYMENT_H
