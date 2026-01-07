#include <cmath>
#include <fstream>
#include <iostream>
#include <string>

#include <opencv2/highgui.hpp>

#include "simple_mcl/particle.hpp"
#include "simple_mcl/random.hpp"
#include "simple_mcl/types.hpp"
#include "simple_mcl/visualization.hpp"

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
    const double world_min = 0.0;
    const double world_max = 15.0;
    const double display_min = 0.0;
    const double display_max = 15.0;
    simple_mcl::Pose min_pose{world_min, world_min, -kPi};
    simple_mcl::Pose max_pose{world_max, world_max, kPi};
    std::vector<simple_mcl::Particle> particles =
        simple_mcl::initializeUniformParticles(300, min_pose, max_pose, rng);

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

    simple_mcl::Pose truth{7.5, 4.0, 0.0};
    std::ofstream log("simple_mcl_log.csv");
    if (!log) {
        std::cerr << "failed to open simple_mcl_log.csv\n";
        return 1;
    }
    simple_mcl::Visualizer viz(display_min, display_max, display_min, display_max, 30, 600, 600);

    log << "step,truth_x,truth_y,truth_theta,meas_0,meas_1,meas_2,est_x,est_y,est_theta\n";
    simple_mcl::OdomDelta u{0.1, 0.5, 0.05};
    std::vector<simple_mcl::Landmark> landmarks{
        {2.0, 4.0},
        {13.0, 4.0},
        {7.5, 13.0},
    };
    const double rot1_std = 0.05;
    const double trans_std = 0.1;
    const double rot2_std = 0.05;
    const double sensor_std = 0.4;
    const int steps = 200;
    const double dt = 0.1;
    for (int step = 0; step < steps; ++step) {
        simple_mcl::applyOdometryMotion(&particles, u, rot1_std, trans_std, rot2_std, rng);
        simple_mcl::Pose next_truth = truth;
        next_truth.x += u.trans * std::cos(truth.theta + u.rot1);
        next_truth.y += u.trans * std::sin(truth.theta + u.rot1);
        next_truth.theta = simple_mcl::normalizeAngle(truth.theta + u.rot1 + u.rot2);
        if (next_truth.x >= world_min && next_truth.x <= world_max &&
            next_truth.y >= world_min && next_truth.y <= world_max) {
            truth = next_truth;
        }
        std::vector<double> measurements;
        measurements.reserve(landmarks.size());
        for (const auto &lm : landmarks) {
            double dx = truth.x - lm.x;
            double dy = truth.y - lm.y;
            double range = std::sqrt(dx * dx + dy * dy);
            measurements.push_back(range + rng.normal(0.0, sensor_std));
        }
        simple_mcl::updateWeightsLandmarks(&particles, landmarks, measurements, sensor_std);
        simple_mcl::normalizeWeights(&particles);
        particles = simple_mcl::resampleMultinomial(particles, rng);
        simple_mcl::Pose est = simple_mcl::estimatePoseWeightedMean(particles);
        log << step << "," << truth.x << "," << truth.y << "," << truth.theta;
        for (double m : measurements) {
            log << "," << m;
        }
        log << "," << est.x << "," << est.y << "," << est.theta << "\n";

        double time_sec = dt * static_cast<double>(step);
        cv::Mat frame = viz.drawFrame(particles, truth, est, landmarks, time_sec);
        static bool window_ready = false;
        if (!window_ready) {
            cv::namedWindow("simple_mcl", cv::WINDOW_NORMAL);
            cv::resizeWindow("simple_mcl", 800, 800);
            window_ready = true;
        }
        cv::imshow("simple_mcl", frame);
        int key = cv::waitKey(240);
        if (key == 27) break;
    }
    std::cout << "wrote simple_mcl_log.csv\n";

    return 0;
}
