#include <cmath>
#include <algorithm>

#include "util.h"

double util::get_distance(double x1, double y1, double x2, double y2) {
    double delta_x = x2 - x1;
    double delta_y = y2 - y1;

    return sqrt(delta_x * delta_x + delta_y * delta_y);
}
