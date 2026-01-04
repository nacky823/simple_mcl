#pragma once

#include <cmath>
#include <vector>

#include "simple_mcl/random.hpp"
#include "simple_mcl/types.hpp"

namespace simple_mcl {

struct Particle {
    Pose pose;
    double weight{1.0};
};

inline std::vector<Particle> initializeUniformParticles(
    std::size_t count, const Pose &min_pose, const Pose &max_pose, Random &rng)
{
    std::vector<Particle> particles;
    particles.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
        Pose pose;
        pose.x = rng.uniform(min_pose.x, max_pose.x);
        pose.y = rng.uniform(min_pose.y, max_pose.y);
        pose.theta = rng.uniform(min_pose.theta, max_pose.theta);
        particles.push_back(Particle{pose, 1.0});
    }
    return particles;
}

inline void normalizeWeights(std::vector<Particle> *particles)
{
    if (!particles) return;

    double sum = 0.0;
    for (const auto &p : *particles) {
        sum += p.weight;
    }
    if (sum <= 0.0) return;

    for (auto &p : *particles) {
        p.weight /= sum;
    }
}

inline Pose estimatePoseWeightedMean(const std::vector<Particle> &particles)
{
    Pose estimate;
    if (particles.empty()) return estimate;

    double sum_w = 0.0;
    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_sin = 0.0;
    double sum_cos = 0.0;
    for (const auto &p : particles) {
        sum_w += p.weight;
        sum_x += p.weight * p.pose.x;
        sum_y += p.weight * p.pose.y;
        sum_sin += p.weight * std::sin(p.pose.theta);
        sum_cos += p.weight * std::cos(p.pose.theta);
    }
    if (sum_w <= 0.0) return estimate;

    estimate.x = sum_x / sum_w;
    estimate.y = sum_y / sum_w;
    estimate.theta = std::atan2(sum_sin, sum_cos);
    return estimate;
}

inline void apply1DMotion(
    std::vector<Particle> *particles, double delta, double noise_std, Random &rng)
{
    if (!particles) return;
    for (auto &p : *particles) {
        double noisy = delta + rng.normal(0.0, noise_std);
        p.pose.x += noisy;
    }
}

inline void applyOdometryMotion(
    std::vector<Particle> *particles, const OdomDelta &u,
    double rot1_std, double trans_std, double rot2_std, Random &rng)
{
    if (!particles) return;
    for (auto &p : *particles) {
        double rot1 = u.rot1 + rng.normal(0.0, rot1_std);
        double trans = u.trans + rng.normal(0.0, trans_std);
        double rot2 = u.rot2 + rng.normal(0.0, rot2_std);

        p.pose.x += trans * std::cos(p.pose.theta + rot1);
        p.pose.y += trans * std::sin(p.pose.theta + rot1);
        p.pose.theta = normalizeAngle(p.pose.theta + rot1 + rot2);
    }
}

inline double gaussianPdf(double x, double mean, double stddev)
{
    const double kPi = 3.141592653589793;
    const double var = stddev * stddev;
    if (var <= 0.0) return 0.0;

    const double diff = x - mean;
    const double norm = 1.0 / std::sqrt(2.0 * kPi * var);
    return norm * std::exp(-(diff * diff) / (2.0 * var));
}

inline void updateWeights1D(
    std::vector<Particle> *particles, double measurement, double sensor_std)
{
    if (!particles) return;
    for (auto &p : *particles) {
        double pz = gaussianPdf(measurement, p.pose.x, sensor_std);
        p.weight *= pz;
    }
}

inline std::vector<Particle> resampleMultinomial(
    const std::vector<Particle> &particles, Random &rng)
{
    std::vector<Particle> out;
    if (particles.empty()) return out;

    out.reserve(particles.size());
    std::vector<double> cdf;
    cdf.reserve(particles.size());
    double accum = 0.0;
    for (const auto &p : particles) {
        accum += p.weight;
        cdf.push_back(accum);
    }
    if (accum <= 0.0) return out;

    for (std::size_t i = 0; i < particles.size(); ++i) {
        double r = rng.uniform(0.0, accum);
        std::size_t idx = 0;
        while (idx < cdf.size() && r > cdf[idx]) {
            ++idx;
        }
        if (idx >= particles.size()) idx = particles.size() - 1;
        out.push_back(particles[idx]);
        out.back().weight = 1.0;
    }
    return out;
}

}  // namespace simple_mcl
