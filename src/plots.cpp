#include <cmath>
#include <matplot/matplot.h>

using namespace matplot;

void test_plot1() {
    // Generate x values from 0 to 90 with step 10
    std::vector<double> x = iota(0, 10, 90);
    std::vector<double> y = {20, 30, 45, 40, 60, 65, 80, 75, 95, 90};
    std::vector<double> err(y.size(), 10.); // Error values for each point

    // Plot error bars
    errorbar(x, y, err);

    // Set axis limits
    axis({0, 100, 0, 110});

    // Save plot to file
    save("output/test.pdf");
}
