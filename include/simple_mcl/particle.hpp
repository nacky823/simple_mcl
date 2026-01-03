#pragma once

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

}  // namespace simple_mcl
