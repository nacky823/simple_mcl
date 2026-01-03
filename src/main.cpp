#include <iostream>

#include "simple_mcl/random.hpp"
#include "simple_mcl/types.hpp"

int main() {
    simple_mcl::Pose pose{1.0, 2.0, 0.1};
    std::cout << "Hello MCL: pose=(" << pose.x << ", " << pose.y << ", " << pose.theta << ")\n";

    simple_mcl::Random rng(42);
    const int n = 10000;
    double sum = 0.0;
    double sum_sq = 0.0;
    for (int i = 0; i < n; ++i) {
        double x = rng.normal(0.0, 1.0);
        sum += x;
        sum_sq += x * x;
    }

    double mean = sum / n;
    double var = sum_sq / n - mean * mean;
    std::cout << "normal(0,1) sample mean=" << mean << " var=" << var << "\n";

    return 0;
}
