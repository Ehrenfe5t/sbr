// smoke_los.cpp — LoS 自由空间冒烟测试
#include "doctest.h"
#include "sbr/sbr_engine.h"
#include "sbr/sbr_accelerator_bruteforce.h"
#include "sbr/sbr_scene_builder.h"
#include "sbr/sbr_math.h"
#include <cmath>

using namespace sbr;

TEST_CASE("Smoke: LoS free space — 近距离命中") {
    // 空场景, Tx→Rx 沿 Y 轴 (Fibonacci 首条射线方向恰好是 +Y)
    SceneBuilder sb;
    Scene scene = sb.Build();

    BruteForceAccelerator accel(scene);
    SbrEngine engine(std::make_unique<BruteForceAccelerator>(scene));

    SbrConfig config;
    config.trace_profile        = "DebugValidation";
    config.ray_count            = 100;           // small count for speed
    config.max_ray_depth        = 0;
    config.max_reflection_count  = 0;
    config.rx_sphere_radius_m   = 1.0;           // generous sphere
    config.ray_power_threshold_dB = -120.0;

    NumericToleranceConfig tol;
    Point3 txPoint = MakeVec3(0, 0, 0);
    // Rx 沿 Y 轴 2m — Fibonacci 射线 #0 方向正好是 (0,1,0)
    std::vector<Point3> rxPoints = { MakeVec3(0, 2, 0) };
    MaterialDatabase matDb;

    GeometricPathSet result = engine.RunPointToPoint(
        scene, matDb, config, txPoint, rxPoints, tol);

    // 第一条 Fibonacci 射线 (0,1,0) 直接经过 Rx → 命中
    CHECK(result.paths.size() > 0);

    if (!result.paths.empty()) {
        const auto& path = result.paths[0];
        CHECK(path.valid);
        CHECK(path.is_los);
        CHECK(path.nodes.size() == 2);  // Tx + Rx
        CHECK(path.nodes[0].interaction_type == InteractionType::Tx);
        CHECK(path.nodes[1].interaction_type == InteractionType::Rx);
        // 总长度 ≈ 2.0m (sphere radius tolerance)
        CHECK(std::fabs(path.total_length - 2.0) < config.rx_sphere_radius_m + 0.1);
    }
}

TEST_CASE("Smoke: LoS — ray_count 增加命中数单调不降") {
    SceneBuilder sb;
    Scene scene = sb.Build();
    BruteForceAccelerator accel(scene);
    SbrEngine engine(std::make_unique<BruteForceAccelerator>(scene));

    SbrConfig config;
    config.trace_profile        = "DebugValidation";
    config.max_ray_depth        = 0;
    config.max_reflection_count  = 0;
    config.rx_sphere_radius_m   = 0.5;
    config.ray_power_threshold_dB = -120.0;

    NumericToleranceConfig tol;
    Point3 txPoint = MakeVec3(0, 0, 0);
    std::vector<Point3> rxPoints = { MakeVec3(0, 2, 0) };
    MaterialDatabase matDb;

    int prevHits = -1;
    for (int n : {100, 500, 2000}) {
        config.ray_count = n;
        GeometricPathSet result = engine.RunPointToPoint(
            scene, matDb, config, txPoint, rxPoints, tol);
        int hits = static_cast<int>(result.paths.size());
        CHECK(hits >= prevHits);
        prevHits = hits;
    }
}
