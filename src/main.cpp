#include <iostream>

#include "simple_mcl/particle.hpp"
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

    const double kPi = 3.141592653589793;
    simple_mcl::Pose min_pose{0.0, 0.0, -kPi};
    simple_mcl::Pose max_pose{10.0, 10.0, kPi};
    std::vector<simple_mcl::Particle> particles =
        simple_mcl::initializeUniformParticles(5, min_pose, max_pose, rng);

    for (auto &p : particles) {
        p.weight = rng.uniform(0.0, 1.0);
    }
    simple_mcl::normalizeWeights(&particles);

    double wsum = 0.0;
    for (const auto &p : particles) {
        wsum += p.weight;
    }
    std::cout << "particles=" << particles.size() << " weight_sum=" << wsum << "\n";
    if (!particles.empty()) {
        const auto &p0 = particles.front();
        std::cout << "p0=(" << p0.pose.x << ", " << p0.pose.y << ", " << p0.pose.theta << ")\n";
    }

    return 0;
}
