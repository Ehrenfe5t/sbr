// smoke_pec_reflection.cpp — PEC 单反射冒烟测试
#include "doctest.h"
#include "sbr/sbr_engine.h"
#include "sbr/sbr_accelerator_bruteforce.h"
#include "sbr/sbr_scene_builder.h"
#include "sbr/sbr_math.h"
#include <cmath>

using namespace sbr;

// 构造水平地面 (z=0, 法线 +Z, 10x10m)
static Scene MakeGround() {
    SceneBuilder sb;
    sb.AddVertex(-5, -5, 0);
    sb.AddVertex( 5, -5, 0);
    sb.AddVertex( 5,  5, 0);
    sb.AddVertex(-5,  5, 0);
    sb.AddFace(0, 1, 2);
    sb.AddFace(0, 2, 3);
    return sb.Build();
}

TEST_CASE("Smoke: PEC 单反射 — 地面反射路径存在") {
    // 地面在 z=0, Tx 和 Rx 都在地面上方
    // Tx 在 (0,0,2), 部分射线打到地面反射后可以到达 Rx
    Scene scene = MakeGround();
    BruteForceAccelerator accel(scene);
    SbrEngine engine(std::make_unique<BruteForceAccelerator>(scene));

    SbrConfig config;
    config.trace_profile         = "DebugValidation";
    config.ray_count             = 50000;
    config.max_ray_depth         = 1;
    config.max_reflection_count  = 1;
    config.rx_sphere_radius_m    = 0.5;
    config.ray_power_threshold_dB = -120.0;

    NumericToleranceConfig tol;
    // Tx 和 Rx 均在地面上方, 直射和反射路径都可能存在
    Point3 txPoint = MakeVec3(0, 0, 2);
    std::vector<Point3> rxPoints = { MakeVec3(0, 2, 1) };
    MaterialDatabase matDb;

    GeometricPathSet result = engine.RunPointToPoint(
        scene, matDb, config, txPoint, rxPoints, tol);

    // 检查反射路径存在
    int reflCount = 0, losCount = 0;
    for (const auto& path : result.paths) {
        if (path.is_los) {
            losCount++;
            CHECK(path.nodes.size() == 2);
        } else if (path.nodes.size() >= 3 &&
                   path.nodes[1].interaction_type == InteractionType::Reflection) {
            reflCount++;
            CHECK(path.nodes.size() == 3);
            CHECK(path.nodes[0].interaction_type == InteractionType::Tx);
            CHECK(path.nodes[1].interaction_type == InteractionType::Reflection);
            CHECK(path.nodes[2].interaction_type == InteractionType::Rx);
            // 反射路径总长 > 直连距离
            double losDist = Length(Subtract(rxPoints[0], txPoint));
            CHECK(path.total_length > losDist);
        }
    }
    // 至少应有一条反射路径
    CHECK(reflCount > 0);
}

TEST_CASE("Smoke: 空场景 — 仅 LoS 无反射路径") {
    SceneBuilder sb;
    Scene scene = sb.Build();
    BruteForceAccelerator accel(scene);
    SbrEngine engine(std::make_unique<BruteForceAccelerator>(scene));

    SbrConfig config;
    config.trace_profile         = "DebugValidation";
    config.ray_count             = 5000;
    config.max_ray_depth         = 1;
    config.max_reflection_count  = 1;
    config.rx_sphere_radius_m    = 0.5;
    config.ray_power_threshold_dB = -120.0;

    NumericToleranceConfig tol;
    Point3 txPoint = MakeVec3(0, 0, 0);
    std::vector<Point3> rxPoints = { MakeVec3(0, 2, 0) };
    MaterialDatabase matDb;

    GeometricPathSet result = engine.RunPointToPoint(
        scene, matDb, config, txPoint, rxPoints, tol);

    for (const auto& path : result.paths) {
        CHECK(path.is_los);
        CHECK(path.nodes.size() == 2);
    }
}
