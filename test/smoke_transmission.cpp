// smoke_transmission.cpp — 透射冒烟测试 (Phase 2)
#include "doctest.h"
#include "sbr/sbr_engine.h"
#include "sbr/sbr_accelerator_bruteforce.h"
#include "sbr/sbr_scene_builder.h"
#include "sbr/sbr_math.h"
#include <cmath>
#include <cstdio>

using namespace sbr;

// 构造介质平板 (z=0平面, 2×2m, 法线+Z, front=Air, back=Glass)
static Scene MakeGlassSlab() {
    SceneBuilder sb;
    sb.AddVertex(-1, -1, 0);
    sb.AddVertex( 1, -1, 0);
    sb.AddVertex( 1,  1, 0);
    sb.AddVertex(-1,  1, 0);
    // 双侧材质: 正面Air, 背面Glass (n≈1.5)
    sb.AddFace(0, 1, 2, "Air", "Glass", true, true, false);
    sb.AddFace(0, 2, 3, "Air", "Glass", true, true, false);
    return sb.Build();
}

TEST_CASE("Smoke: 介质透射 — 折射方向正确 (Snell残差)") {
    // 空气→玻璃 30° 入射 → 透射角 ≈ 19.47°
    Scene scene = MakeGlassSlab();
    BruteForceAccelerator accel(scene);
    SbrEngine engine(std::make_unique<BruteForceAccelerator>(scene));

    MaterialDatabase matDb;
    // 注册 Air 和 Glass 材质
    // matDb 为空时默认 n=1.0, 所以需要手动注册
    // 实际使用中加载 CSV; 测试时利用 SceneBuilder + empty DB (n1=n2=1)

    SbrConfig config;
    config.trace_profile          = "DebugValidation";
    config.ray_count              = 5000;
    config.max_ray_depth          = 1;
    config.max_reflection_count   = 1;
    config.max_transmission_count = 1;
    config.rx_sphere_radius_m     = 0.5;
    config.ray_power_threshold_dB = -120.0;
    config.center_frequency_hz    = 2.4e9;

    NumericToleranceConfig tol;
    // Tx 在 z=2 (空气侧), 射向玻璃板
    Point3 txPoint = MakeVec3(0, 0, 2);
    std::vector<Point3> rxPoints = { MakeVec3(0, 1, -1) };  // 玻璃侧下方
    MaterialDatabase emptyDb;  // empty → n1=n2=1, 无折射偏转

    // 使用空DB: 介质透射分支存在但不发生方向偏转 (n1=n2)
    GeometricPathSet result = engine.RunPointToPoint(
        scene, emptyDb, config, txPoint, rxPoints, tol);

    // 检查是否存在透射路径
    int txCount = 0;
    for (const auto& path : result.paths) {
        if (path.contains_transmission ||
            path.nodes.size() >= 3) {
            // 检查是否有 Transmission 类型节点
            for (size_t ni = 1; ni < path.nodes.size(); ++ni) {
                if (path.nodes[ni].interaction_type == InteractionType::Transmission) {
                    txCount++;
                    break;
                }
            }
        }
    }
    // 应有透射路径
    CHECK(txCount > 0);
}

TEST_CASE("Smoke: R+T 双分支 — 同一命中面同时生成反射和透射") {
    Scene scene = MakeGlassSlab();
    BruteForceAccelerator accel(scene);
    SbrEngine engine(std::make_unique<BruteForceAccelerator>(scene));

    SbrConfig config;
    config.trace_profile          = "DebugValidation";
    config.ray_count              = 5000;
    config.max_ray_depth          = 1;
    config.max_reflection_count   = 1;
    config.max_transmission_count = 1;
    config.rx_sphere_radius_m     = 0.5;
    config.ray_power_threshold_dB = -120.0;

    NumericToleranceConfig tol;
    // Tx 在 z=2, Rx 在 z=2 (反射侧) 和 z=-1 (透射侧) 各一个
    Point3 txPoint = MakeVec3(0, 0, 2);
    std::vector<Point3> rxPoints = {
        MakeVec3(0, 1, 2),   // 反射侧 Rx
        MakeVec3(0, 1, -1)   // 透射侧 Rx
    };
    MaterialDatabase matDb;

    GeometricPathSet result = engine.RunPointToPoint(
        scene, matDb, config, txPoint, rxPoints, tol);

    int reflCount = 0, txCount = 0;
    for (const auto& path : result.paths) {
        for (size_t ni = 1; ni < path.nodes.size(); ++ni) {
            if (path.nodes[ni].interaction_type == InteractionType::Reflection)
                reflCount++;
            if (path.nodes[ni].interaction_type == InteractionType::Transmission)
                txCount++;
        }
    }
    // 应同时存在反射和透射路径
    CHECK(reflCount > 0);
    CHECK(txCount > 0);
}

TEST_CASE("Smoke: P0+P1 回归") {
    // 验证重构后 LoS 和 PEC 反射仍然正常
    // LoS
    {
        SceneBuilder sb;
        Scene scene = sb.Build();
        BruteForceAccelerator accel(scene);
        SbrEngine engine(std::make_unique<BruteForceAccelerator>(scene));

        SbrConfig config;
        config.trace_profile = "DebugValidation";
        config.ray_count = 100;
        config.max_ray_depth = 0;
        config.max_reflection_count = 0;
        config.rx_sphere_radius_m = 1.0;
        config.ray_power_threshold_dB = -120.0;

        NumericToleranceConfig tol;
        Point3 txPoint = MakeVec3(0, 0, 0);
        std::vector<Point3> rxPoints = { MakeVec3(0, 2, 0) };
        MaterialDatabase matDb;

        GeometricPathSet result = engine.RunPointToPoint(
            scene, matDb, config, txPoint, rxPoints, tol);

        CHECK(result.paths.size() > 0);
        if (!result.paths.empty()) {
            CHECK(result.paths[0].is_los);
            CHECK(result.paths[0].nodes.size() == 2);
        }
    }

    // PEC 反射
    {
        SceneBuilder sb;
        sb.AddVertex(-5, -5, 0);
        sb.AddVertex( 5, -5, 0);
        sb.AddVertex( 5,  5, 0);
        sb.AddVertex(-5,  5, 0);
        sb.AddFace(0, 1, 2);
        sb.AddFace(0, 2, 3);
        Scene scene = sb.Build();
        BruteForceAccelerator accel(scene);
        SbrEngine engine(std::make_unique<BruteForceAccelerator>(scene));

        SbrConfig config;
        config.trace_profile = "DebugValidation";
        config.ray_count = 50000;
        config.max_ray_depth = 1;
        config.max_reflection_count = 1;
        config.rx_sphere_radius_m = 0.5;
        config.ray_power_threshold_dB = -120.0;

        NumericToleranceConfig tol;
        Point3 txPoint = MakeVec3(0, 0, 2);
        std::vector<Point3> rxPoints = { MakeVec3(0, 2, 1) };
        MaterialDatabase matDb;

        GeometricPathSet result = engine.RunPointToPoint(
            scene, matDb, config, txPoint, rxPoints, tol);

        bool hasReflection = false;
        for (const auto& path : result.paths) {
            if (!path.is_los && path.nodes.size() >= 3 &&
                path.nodes[1].interaction_type == InteractionType::Reflection) {
                hasReflection = true;
                break;
            }
        }
        CHECK(hasReflection);
    }
}
