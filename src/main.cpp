#include <fstream>
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

    simple_mcl::Pose estimate = simple_mcl::estimatePoseWeightedMean(particles);
    std::cout << "estimate=(" << estimate.x << ", " << estimate.y << ", " << estimate.theta << ")\n";

    std::ofstream log("simple_mcl_log.csv");
    if (!log) {
        std::cerr << "failed to open simple_mcl_log.csv\n";
        return 1;
    }
    log << "step,truth_x,measurement,est_x,est_y,est_theta\n";
    double truth_x = 2.0;
    const double dx = 0.5;
    const double motion_noise = 0.1;
    const double sensor_std = 0.2;
    for (int step = 0; step < 5; ++step) {
        simple_mcl::apply1DMotion(&particles, dx, motion_noise, rng);
        truth_x += dx;
        double measurement = truth_x + rng.normal(0.0, sensor_std);
        simple_mcl::updateWeights1D(&particles, measurement, sensor_std);
        simple_mcl::normalizeWeights(&particles);
        simple_mcl::Pose est = simple_mcl::estimatePoseWeightedMean(particles);
        log << step << "," << truth_x << "," << measurement << ","
            << est.x << "," << est.y << "," << est.theta << "\n";
    }
    std::cout << "wrote simple_mcl_log.csv\n";

    return 0;
}
