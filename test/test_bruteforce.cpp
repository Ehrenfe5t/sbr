// test_bruteforce.cpp — 暴力射线-三角求交单元测试
#include "doctest.h"
#include "sbr/sbr_accelerator_bruteforce.h"
#include "sbr/sbr_scene_builder.h"
#include "sbr/sbr_math.h"
#include <cmath>

using namespace sbr;

// 辅助: 构建一个单位正方形三角化场景 (2个三角, z=0 平面)
static Scene MakeUnitSquareScene() {
    SceneBuilder sb;
    // z=0 平面, 正方形 [0,1]×[0,1], 两个三角
    sb.AddVertex(0, 0, 0);  // v0
    sb.AddVertex(1, 0, 0);  // v1
    sb.AddVertex(0, 1, 0);  // v2
    sb.AddVertex(1, 1, 0);  // v3
    sb.AddFace(0, 1, 2);   // face0: 左下三角
    sb.AddFace(1, 3, 2);   // face1: 右上三角
    return sb.Build();
}

TEST_CASE("T3.1 单三角命中中心") {
    // 1个三角: (0,0,0)-(1,0,0)-(0,1,0), 射线从上方垂直射入
    SceneBuilder sb;
    sb.AddVertex(0, 0, 0);
    sb.AddVertex(1, 0, 0);
    sb.AddVertex(0, 1, 0);
    sb.AddFace(0, 1, 2);
    Scene scene = sb.Build();

    BruteForceAccelerator accel(scene);
    Ray ray;
    ray.origin    = MakeVec3(0.2, 0.2, 1.0);
    ray.direction = MakeVec3(0.0, 0.0, -1.0);

    FaceQueryContext ctx;
    FaceHit hit = accel.QueryClosestFaceHit(ray, ctx);

    CHECK(hit.hit);
    CHECK(hit.face_id == 0);
    CHECK(std::fabs(hit.distance - 1.0) < 1e-6);   // 从 z=1 到 z=0
    CHECK(std::fabs(hit.position.x - 0.2) < 1e-6);
    CHECK(std::fabs(hit.position.y - 0.2) < 1e-6);
    CHECK(std::fabs(hit.position.z - 0.0) < 1e-6);
}

TEST_CASE("T3.2 单三角未命中") {
    SceneBuilder sb;
    sb.AddVertex(0, 0, 0);
    sb.AddVertex(1, 0, 0);
    sb.AddVertex(0, 1, 0);
    sb.AddFace(0, 1, 2);
    Scene scene = sb.Build();

    BruteForceAccelerator accel(scene);
    Ray ray;
    ray.origin    = MakeVec3(2, 2, 1);
    ray.direction = MakeVec3(0, 0, -1);

    FaceQueryContext ctx;
    FaceHit hit = accel.QueryClosestFaceHit(ray, ctx);

    CHECK_FALSE(hit.hit);
}

TEST_CASE("T3.3 两三角-最近命中") {
    SceneBuilder sb;
    // z=0 平面三角
    sb.AddVertex(0, 0, 0);
    sb.AddVertex(1, 0, 0);
    sb.AddVertex(0, 1, 0);
    sb.AddFace(0, 1, 2);
    // z=0.5 平面三角 (更近)
    sb.AddVertex(0, 0, 0.5);
    sb.AddVertex(1, 0, 0.5);
    sb.AddVertex(0, 1, 0.5);
    sb.AddFace(3, 4, 5);
    Scene scene = sb.Build();

    BruteForceAccelerator accel(scene);
    Ray ray;
    ray.origin    = MakeVec3(0.2, 0.2, 1.0);
    ray.direction = MakeVec3(0.0, 0.0, -1.0);

    FaceQueryContext ctx;
    FaceHit hit = accel.QueryClosestFaceHit(ray, ctx);

    CHECK(hit.hit);
    CHECK(hit.face_id == 1);                    // 先命中 z=0.5 的面
    CHECK(std::fabs(hit.distance - 0.5) < 1e-6);
}

TEST_CASE("T3.4 自交忽略") {
    SceneBuilder sb;
    sb.AddVertex(0, 0, 0);
    sb.AddVertex(1, 0, 0);
    sb.AddVertex(0, 1, 0);
    sb.AddFace(0, 1, 2);   // face0
    sb.AddVertex(0, 0, 0.5);
    sb.AddVertex(1, 0, 0.5);
    sb.AddVertex(0, 1, 0.5);
    sb.AddFace(3, 4, 5);   // face1
    Scene scene = sb.Build();

    BruteForceAccelerator accel(scene);
    // 射线原点在 face0 上
    Ray ray;
    ray.origin    = MakeVec3(0.2, 0.2, 0.0);
    ray.direction = MakeVec3(0.0, 0.0, -1.0);  // 向下

    FaceQueryContext ctx;
    ctx.ignore_origin_self_hit = true;
    ctx.ignored_face_id = 0;   // 忽略 face0

    FaceHit hit = accel.QueryClosestFaceHit(ray, ctx);

    // 由于射线向下且忽略了原点面，且只有两个面都在z≥0
    // 射线向下命中不到任何面 (face1在z=0.5, 但射线沿-Z)
    CHECK_FALSE(hit.hit);
}

TEST_CASE("T3.5 边命中 (射线精确通过边)") {
    SceneBuilder sb;
    sb.AddVertex(0, 0, 0);
    sb.AddVertex(1, 0, 0);
    sb.AddVertex(0, 1, 0);
    sb.AddFace(0, 1, 2);
    Scene scene = sb.Build();

    BruteForceAccelerator accel(scene);
    // 射线从上方精确射向 v0→v1 边中点
    Ray ray;
    ray.origin    = MakeVec3(0.5, 0.0, 1.0);
    ray.direction = MakeVec3(0.0, 0.0, -1.0);

    FaceQueryContext ctx;
    FaceHit hit = accel.QueryClosestFaceHit(ray, ctx);

    // 共享边上命中 (属于 face0)
    CHECK(hit.hit);
    CHECK(hit.face_id == 0);
}

TEST_CASE("T3.6 IsOccluded — 有遮挡") {
    SceneBuilder sb;
    sb.AddVertex(-1, -1, 0.5);
    sb.AddVertex(1, -1, 0.5);
    sb.AddVertex(0,  1, 0.5);
    sb.AddFace(0, 1, 2);
    Scene scene = sb.Build();

    BruteForceAccelerator accel(scene);
    Point3 start = MakeVec3(0, 0, 0);
    Point3 end   = MakeVec3(0, 0, 1);

    VisibilityQueryContext ctx;
    bool occluded = accel.IsOccluded(start, end, ctx);

    CHECK(occluded);
}

TEST_CASE("T3.7 IsOccluded — 无遮挡") {
    SceneBuilder sb;
    sb.AddVertex(-1, -1, 0.5);
    sb.AddVertex(1, -1, 0.5);
    sb.AddVertex(0,  1, 0.5);
    sb.AddFace(0, 1, 2);
    Scene scene = sb.Build();

    BruteForceAccelerator accel(scene);
    // start 和 end 都在三角上方
    Point3 start = MakeVec3(0, 0, 2);
    Point3 end   = MakeVec3(0, 0, 3);

    VisibilityQueryContext ctx;
    bool occluded = accel.IsOccluded(start, end, ctx);

    CHECK_FALSE(occluded);
}
