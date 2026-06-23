// smoke_diffraction.cpp — 绕射冒烟测试 (Phase 3)
#include "doctest.h"
#include "sbr/sbr_engine.h"
#include "sbr/sbr_accelerator_bruteforce.h"
#include "sbr/sbr_scene_builder.h"
#include "sbr/sbr_scene.h"
#include "sbr/sbr_math.h"
#include <cmath>

using namespace sbr;

// 构造楔边场景: 地面(z=0) + 墙面(y=0), 共享边沿X轴 形成90°楔边
// 手动设置 edge/wedge 拓扑
static Scene MakeWedgeSceneWithTopology() {
    SceneBuilder sb;
    // 顶点 (注意: 共享边的顶点需要是同一组索引)
    sb.AddVertex(0, 0, 0);  // v0 — 共享
    sb.AddVertex(2, 0, 0);  // v1 — 共享
    sb.AddVertex(2, 2, 0);  // v2 — face0 only
    sb.AddVertex(0, 2, 0);  // v3 — face0 only
    sb.AddVertex(2, 0, 2);  // v4 — face1 only
    sb.AddVertex(0, 0, 2);  // v5 — face1 only

    // face0: 地面 z=0, 逆时针 (从+Z看): v0→v2→v3, v0→v1→v2
    sb.AddFace(0, 2, 3);  // tri A: v0-v2-v3
    sb.AddFace(0, 1, 2);  // tri B: v0-v1-v2

    // face1: 墙面 y=0, 从+Y看逆时针: v0→v1→v4→v5
    // 修正法线方向: 墙面法线应指向 +Y
    sb.AddFace(0, 5, 4);  // tri C: v0-v5-v4
    sb.AddFace(0, 4, 1);  // tri D: v0-v4-v1

    Scene scene = sb.Build();

    // ── 手动设置边拓扑 ──
    // 共享边: v0(0,0,0) → v1(2,0,0) 沿X轴
    Edge sharedEdge;
    sharedEdge.edge_id       = 0;
    sharedEdge.vertex_index0 = 0;
    sharedEdge.vertex_index1 = 1;
    sharedEdge.start         = scene.vertices[0];
    sharedEdge.end           = scene.vertices[1];
    sharedEdge.direction     = Normalize(SubtractVec(scene.vertices[1], scene.vertices[0]));
    sharedEdge.midpoint      = Scale(AddVec(scene.vertices[0], scene.vertices[1]), 0.5);
    sharedEdge.length        = Length(SubtractVec(scene.vertices[1], scene.vertices[0]));
    sharedEdge.face_id0      = 0;  // face0
    sharedEdge.face_id1      = 2;  // face2 (第一个 face1 的三角)
    sharedEdge.supports_wedge = true;
    scene.edges.push_back(sharedEdge);

    // 将边关联到面
    for (auto& face : scene.faces) {
        face.adjacent_edge_id0 = 0;
    }

    // ── 手动设置楔边 ──
    Wedge wedge;
    wedge.wedge_id         = 0;
    wedge.source_edge_id   = 0;
    wedge.positive_face_id = 0;  // face0
    wedge.negative_face_id = 2;  // face2
    wedge.segment_start    = scene.vertices[0];
    wedge.segment_end      = scene.vertices[1];
    wedge.center_point     = sharedEdge.midpoint;
    wedge.direction        = sharedEdge.direction;
    wedge.length           = sharedEdge.length;
    wedge.wedge_angle_deg  = 90.0;
    wedge.diffractable     = true;
    wedge.valid_for_utd    = true;
    wedge.bounds.min = MakeVec3(0, 0, 0);
    wedge.bounds.max = MakeVec3(2, 0, 0);
    wedge.bounds.valid = true;
    scene.wedges.push_back(wedge);

    return scene;
}

TEST_CASE("Smoke: 单楔边绕射 — 绕射路径存在") {
    Scene scene = MakeWedgeSceneWithTopology();
    BruteForceAccelerator accel(scene);
    SbrEngine engine(std::make_unique<BruteForceAccelerator>(scene));

    SbrConfig config;
    config.trace_profile            = "DebugValidation";
    config.ray_count                = 30000;
    config.max_ray_depth            = 2;
    config.max_reflection_count     = 2;
    config.max_diffraction_count    = 1;
    config.diffraction_rays_per_event = 12;
    config.rx_sphere_radius_m       = 0.5;
    config.ray_power_threshold_dB   = -120.0;

    NumericToleranceConfig tol;
    // Tx 在楔边上方稍偏前 (y>0), Rx 在楔边"阴影"侧 (y<0)
    Point3 txPoint = MakeVec3(1.0, 0.8, 1.0);
    std::vector<Point3> rxPoints = { MakeVec3(1.0, -0.5, 0.5) };
    MaterialDatabase matDb;

    GeometricPathSet result = engine.RunPointToPoint(
        scene, matDb, config, txPoint, rxPoints, tol);

    // 检查是否存在绕射路径
    int diffCount = 0;
    for (const auto& path : result.paths) {
        for (size_t ni = 1; ni < path.nodes.size(); ++ni) {
            if (path.nodes[ni].interaction_type == InteractionType::Diffraction) {
                diffCount++;
                CHECK(path.nodes[ni].wedge_id >= 0);
                // Keller 残差应在合理范围
                double kr = path.nodes[ni].diffraction_diag.keller_residual;
                CHECK(kr < 1e-2);
                break;
            }
        }
    }
    CHECK(diffCount > 0);
}

TEST_CASE("Smoke: Keller锥残差 — 所有绕射节点满足锥条件") {
    Scene scene = MakeWedgeSceneWithTopology();
    BruteForceAccelerator accel(scene);
    SbrEngine engine(std::make_unique<BruteForceAccelerator>(scene));

    SbrConfig config;
    config.trace_profile            = "DebugValidation";
    config.ray_count                = 10000;
    config.max_ray_depth            = 2;
    config.max_reflection_count     = 1;
    config.max_diffraction_count    = 1;
    config.diffraction_rays_per_event = 8;
    config.rx_sphere_radius_m       = 0.5;
    config.ray_power_threshold_dB   = -120.0;

    NumericToleranceConfig tol;
    Point3 txPoint = MakeVec3(1.0, 0.5, 1.0);
    std::vector<Point3> rxPoints = { MakeVec3(1.0, -0.5, 0.5) };
    MaterialDatabase matDb;

    GeometricPathSet result = engine.RunPointToPoint(
        scene, matDb, config, txPoint, rxPoints, tol);

    for (const auto& path : result.paths) {
        for (size_t ni = 1; ni < path.nodes.size(); ++ni) {
            if (path.nodes[ni].interaction_type == InteractionType::Diffraction) {
                double kr = path.nodes[ni].diffraction_diag.keller_residual;
                CHECK(kr < 1e-2);
            }
        }
    }
}

TEST_CASE("Smoke: N_d增加 → 绕射路径数单调不降") {
    Scene scene = MakeWedgeSceneWithTopology();
    BruteForceAccelerator accel(scene);

    SbrConfig config;
    config.trace_profile          = "DebugValidation";
    config.ray_count              = 10000;
    config.max_ray_depth          = 2;
    config.max_reflection_count   = 1;
    config.max_diffraction_count  = 1;
    config.rx_sphere_radius_m     = 0.5;
    config.ray_power_threshold_dB = -120.0;

    NumericToleranceConfig tol;
    Point3 txPoint = MakeVec3(1.0, 0.5, 1.0);
    std::vector<Point3> rxPoints = { MakeVec3(1.0, -0.5, 0.5) };
    MaterialDatabase matDb;

    int prevDiff = -1;
    for (int nd : {4, 8, 16}) {
        config.diffraction_rays_per_event = nd;
        SbrEngine engine(std::make_unique<BruteForceAccelerator>(scene));
        GeometricPathSet result = engine.RunPointToPoint(
            scene, matDb, config, txPoint, rxPoints, tol);

        int diffCount = 0;
        for (const auto& path : result.paths) {
            for (size_t ni = 1; ni < path.nodes.size(); ++ni) {
                if (path.nodes[ni].interaction_type == InteractionType::Diffraction)
                    { diffCount++; break; }
            }
        }
        CHECK(diffCount >= prevDiff);
        prevDiff = diffCount;
    }
}

TEST_CASE("Smoke: 无楔边场景 — 无绕射路径") {
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
    config.trace_profile            = "DebugValidation";
    config.ray_count                = 5000;
    config.max_ray_depth            = 2;
    config.max_reflection_count     = 2;
    config.max_diffraction_count    = 1;
    config.diffraction_rays_per_event = 4;
    config.rx_sphere_radius_m       = 0.5;
    config.ray_power_threshold_dB   = -120.0;

    NumericToleranceConfig tol;
    Point3 txPoint = MakeVec3(0, 0, 2);
    std::vector<Point3> rxPoints = { MakeVec3(0, 2, 1) };
    MaterialDatabase matDb;

    GeometricPathSet result = engine.RunPointToPoint(
        scene, matDb, config, txPoint, rxPoints, tol);

    for (const auto& path : result.paths) {
        for (size_t ni = 1; ni < path.nodes.size(); ++ni) {
            CHECK(path.nodes[ni].interaction_type != InteractionType::Diffraction);
        }
    }
}

TEST_CASE("Smoke: P0-P2 回归") {
    // LoS
    {
        SceneBuilder sb; Scene scene = sb.Build();
        BruteForceAccelerator accel(scene);
        SbrEngine engine(std::make_unique<BruteForceAccelerator>(scene));
        SbrConfig cfg; cfg.trace_profile="DebugValidation"; cfg.ray_count=100;
        cfg.max_ray_depth=0; cfg.max_reflection_count=0; cfg.rx_sphere_radius_m=1.0;
        cfg.ray_power_threshold_dB=-120;
        NumericToleranceConfig tol;
        auto r = engine.RunPointToPoint(scene, MaterialDatabase{}, cfg,
            MakeVec3(0,0,0), {MakeVec3(0,2,0)}, tol);
        CHECK(r.paths.size()>0);
        CHECK(r.paths[0].is_los);
        CHECK(r.paths[0].nodes.size()==2);
    }
    // R+T 双分支
    {
        SceneBuilder sb;
        sb.AddVertex(-1,-1,0); sb.AddVertex(1,-1,0); sb.AddVertex(1,1,0); sb.AddVertex(-1,1,0);
        sb.AddFace(0,1,2,"Air","Glass",true,true,false);
        sb.AddFace(0,2,3,"Air","Glass",true,true,false);
        Scene scene = sb.Build();
        BruteForceAccelerator accel(scene);
        SbrEngine engine(std::make_unique<BruteForceAccelerator>(scene));
        SbrConfig cfg; cfg.trace_profile="DebugValidation"; cfg.ray_count=5000;
        cfg.max_ray_depth=1; cfg.max_reflection_count=1; cfg.max_transmission_count=1;
        cfg.rx_sphere_radius_m=0.5; cfg.ray_power_threshold_dB=-120;
        NumericToleranceConfig tol;
        auto r = engine.RunPointToPoint(scene, MaterialDatabase{}, cfg,
            MakeVec3(0,0,2), {MakeVec3(0,1,2),MakeVec3(0,1,-1)}, tol);
        int rc=0,tc=0;
        for(auto&p:r.paths){for(auto&n:p.nodes){if(n.interaction_type==InteractionType::Reflection)rc++;if(n.interaction_type==InteractionType::Transmission)tc++;}}
        CHECK(rc>0); CHECK(tc>0);
    }
}
