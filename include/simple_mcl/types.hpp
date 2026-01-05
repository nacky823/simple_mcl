#pragma once

namespace simple_mcl {

struct Pose {
    double x{0.0};
    double y{0.0};
    double theta{0.0};
};

struct OdomDelta {
    double rot1{0.0};
    double trans{0.0};
    double rot2{0.0};
};

struct Landmark {
    double x{0.0};
    double y{0.0};
};

inline double normalizeAngle(double theta)
{
    const double kPi = 3.141592653589793;
    while (theta > kPi) theta -= 2.0 * kPi;
    while (theta < -kPi) theta += 2.0 * kPi;
    return theta;
}

}  // namespace simple_mcl
