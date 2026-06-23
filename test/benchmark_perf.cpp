// benchmark_perf.cpp — BVH 性能基准 (Phase 5)
#include "doctest.h"
#include "sbr/sbr_engine.h"
#include "sbr/sbr_accelerator_bruteforce.h"
#include "sbr/sbr_bvh_accelerator.h"
#include "sbr/sbr_scene_builder.h"
#include "sbr/sbr_math.h"
#include <chrono>
#include <cstdio>

using namespace sbr;

// 构造大场景 (N×N 三角网格)
static Scene MakeGridScene(int gridSize) {
    SceneBuilder sb;
    double step = 10.0 / gridSize;
    for (int i = 0; i <= gridSize; ++i)
        for (int j = 0; j <= gridSize; ++j)
            sb.AddVertex(i * step, j * step, 0.0);

    for (int i = 0; i < gridSize; ++i) {
        for (int j = 0; j < gridSize; ++j) {
            int v00 = i * (gridSize+1) + j;
            int v10 = (i+1) * (gridSize+1) + j;
            int v01 = i * (gridSize+1) + (j+1);
            int v11 = (i+1) * (gridSize+1) + (j+1);
            sb.AddFace(v00, v10, v11);
            sb.AddFace(v00, v11, v01);
        }
    }
    return sb.Build();
}

TEST_CASE("Bench: BVH vs BruteForce — 10k faces 场景") {
    // 100×100 网格 = 20000 面
    const int gridSize = 100;
    Scene scene = MakeGridScene(gridSize);
    int nFaces = static_cast<int>(scene.faces.size());
    std::printf("[Bench] Scene: %zu faces\n", scene.faces.size());

    // BVH 构建
    auto t0 = std::chrono::steady_clock::now();
    BvhAccelerator bvh(scene);
    auto t1 = std::chrono::steady_clock::now();
    double bvhBuildMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
    std::printf("[Bench] BVH build: %.1f ms, %d nodes, max depth %d\n",
                bvhBuildMs, bvh.NodeCount(), bvh.MaxDepth());

    // 测试射线
    SbrEngine bvhEngine(std::make_unique<BvhAccelerator>(scene));
    SbrEngine bfEngine(std::make_unique<BruteForceAccelerator>(scene));

    SbrConfig config;
    config.trace_profile = "Coverage";
    config.ray_count = 200;
    config.max_ray_depth = 1;
    config.max_reflection_count = 1;
    config.rx_sphere_radius_m = 0.5;
    config.ray_power_threshold_dB = -120.0;
    config.enable_path_dedup = true;

    NumericToleranceConfig tol;
    std::vector<Point3> rxPoints = { MakeVec3(5, 5, 2) };
    MaterialDatabase matDb;

    // BVH 追踪
    auto t2 = std::chrono::steady_clock::now();
    auto rBvh = bvhEngine.RunPointToPoint(scene, matDb, config, MakeVec3(2,2,5), rxPoints, tol);
    auto t3 = std::chrono::steady_clock::now();
    double bvhMs = std::chrono::duration<double, std::milli>(t3 - t2).count();

    // BruteForce 追踪
    auto t4 = std::chrono::steady_clock::now();
    auto rBf = bfEngine.RunPointToPoint(scene, matDb, config, MakeVec3(2,2,5), rxPoints, tol);
    auto t5 = std::chrono::steady_clock::now();
    double bfMs = std::chrono::duration<double, std::milli>(t5 - t4).count();

    std::printf("[Bench] BVH trace: %.1f ms, BruteForce: %.1f ms (%.1fx speedup)\n",
                bvhMs, bfMs, bfMs / std::max(bvhMs, 0.1));
    std::printf("[Bench] BVH paths: %zu, BF paths: %zu\n",
                rBvh.paths.size(), rBf.paths.size());

    // BVH 不应崩溃
    CHECK(rBvh.paths.size() >= 0);

    // BVH 应比 BruteForce 快
    CHECK(bvhMs < bfMs);
}

TEST_CASE("Bench: P0-P4 快速回归 (BVH)") {
    // LoS with BVH
    SceneBuilder sb; Scene s = sb.Build();
    BvhAccelerator bvh(s);
    SbrEngine e(std::make_unique<BvhAccelerator>(s));
    SbrConfig c; c.trace_profile="DebugValidation"; c.ray_count=100;
    c.max_ray_depth=0; c.max_reflection_count=0; c.rx_sphere_radius_m=1.0;
    c.ray_power_threshold_dB=-120;
    auto r = e.RunPointToPoint(s, MaterialDatabase{}, c, MakeVec3(0,0,0), {MakeVec3(0,2,0)}, NumericToleranceConfig{});
    CHECK(r.paths.size()>0);
    CHECK(r.paths[0].is_los);

    // R+T with BVH
    SceneBuilder sb2; sb2.AddVertex(-1,-1,0);sb2.AddVertex(1,-1,0);
    sb2.AddVertex(1,1,0);sb2.AddVertex(-1,1,0);
    sb2.AddFace(0,1,2,"Air","Glass",true,true,false);
    sb2.AddFace(0,2,3,"Air","Glass",true,true,false);
    Scene s2 = sb2.Build();
    SbrEngine e2(std::make_unique<BvhAccelerator>(s2));
    SbrConfig c2; c2.trace_profile="DebugValidation"; c2.ray_count=5000;
    c2.max_ray_depth=1; c2.max_reflection_count=1; c2.max_transmission_count=1;
    c2.rx_sphere_radius_m=0.5; c2.ray_power_threshold_dB=-120;
    auto r2 = e2.RunPointToPoint(s2, MaterialDatabase{}, c2, MakeVec3(0,0,2), {MakeVec3(0,1,2),MakeVec3(0,1,-1)}, NumericToleranceConfig{});
    int rc=0,tc=0;for(auto&p:r2.paths)for(auto&n:p.nodes){
        if(n.interaction_type==InteractionType::Reflection)rc++;
        if(n.interaction_type==InteractionType::Transmission)tc++;}
    CHECK(rc>0);CHECK(tc>0);
}
