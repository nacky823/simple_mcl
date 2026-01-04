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

    std::size_t uniformIndex(std::size_t max_exclusive) {
        if (max_exclusive == 0) return 0;
        std::uniform_int_distribution<std::size_t> dist(0, max_exclusive - 1);
        return dist(rng_);
    }

private:
    std::mt19937 rng_;
};

}  // namespace simple_mcl
