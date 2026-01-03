#pragma once

#include <random>

namespace simple_mcl {

class Random {
public:
    explicit Random(unsigned int seed = std::random_device{}()) : rng_(seed) {}

    double uniform(double min, double max) {
        std::uniform_real_distribution<double> dist(min, max);
        return dist(rng_);
    }

    double normal(double mean, double stddev) {
        std::normal_distribution<double> dist(mean, stddev);
        return dist(rng_);
    }

private:
    std::mt19937 rng_;
};

}  // namespace simple_mcl
