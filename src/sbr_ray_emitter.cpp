// sbr_ray_emitter.cpp — Fibonacci Sphere 射线发射
#include "sbr/sbr_ray_emitter.h"
#include "sbr/sbr_math.h"
#include <cmath>

namespace sbr {

std::vector<Vec3> GenerateFibonacciRays(int N) {
    std::vector<Vec3> rays;
    rays.reserve(N);

    if (N <= 0) return rays;
    if (N == 1) {
        rays.push_back(MakeVec3(0.0, 1.0, 0.0));
        return rays;
    }

    // Golden ratio angle: phi = pi * (3 - sqrt(5))
    const double phi = kPi * (3.0 - std::sqrt(5.0));

    for (int i = 0; i < N; ++i) {
        // y from +1 to -1 (uniform in cos(theta))
        double y = 1.0 - (static_cast<double>(i) / static_cast<double>(N - 1)) * 2.0;
        double radius = std::sqrt(1.0 - y * y);
        double theta = phi * static_cast<double>(i);

        rays.push_back(MakeVec3(
            std::cos(theta) * radius,
            y,
            std::sin(theta) * radius));
    }

    return rays;
}

} // namespace sbr
