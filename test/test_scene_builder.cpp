// test_scene_builder.cpp — 场景构造器单元测试
#include "doctest.h"
#include "sbr/sbr_scene_builder.h"
#include "sbr/sbr_math.h"
#include <cmath>

using namespace sbr;

TEST_CASE("T4.1 添加顶点") {
    SceneBuilder sb;
    sb.AddVertex(0, 0, 0);
    sb.AddVertex(1, 0, 0);
    sb.AddVertex(0, 1, 0);
    Scene scene = sb.Build();

    CHECK(scene.vertices.size() == 3);
    CHECK(std::fabs(scene.vertices[0].x - 0.0) < 1e-12);
    CHECK(std::fabs(scene.vertices[1].x - 1.0) < 1e-12);
    CHECK(std::fabs(scene.vertices[2].y - 1.0) < 1e-12);
}

TEST_CASE("T4.2 单面构造 — 面积") {
    SceneBuilder sb;
    sb.AddVertex(0, 0, 0);
    sb.AddVertex(1, 0, 0);
    sb.AddVertex(0, 1, 0);
    sb.AddFace(0, 1, 2);
    Scene scene = sb.Build();

    CHECK(scene.faces.size() == 1);
    CHECK(std::fabs(scene.faces[0].area - 0.5) < 1e-12);
}

TEST_CASE("T4.3 法线方向 — 右手定则") {
    SceneBuilder sb;
    sb.AddVertex(0, 0, 0);  // v0
    sb.AddVertex(1, 0, 0);  // v1
    sb.AddVertex(0, 1, 0);  // v2
    sb.AddFace(0, 1, 2);    // 逆时针 (从+Z看) → 法线应指向+Z
    Scene scene = sb.Build();

    CHECK(scene.faces.size() == 1);
    CHECK(std::fabs(scene.faces[0].normal.x - 0.0) < 1e-12);
    CHECK(std::fabs(scene.faces[0].normal.y - 0.0) < 1e-12);
    CHECK(std::fabs(scene.faces[0].normal.z - 1.0) < 1e-12);
}

TEST_CASE("T4.4 AABB") {
    SceneBuilder sb;
    sb.AddVertex(0, 0, 0);
    sb.AddVertex(1, 0, 0);
    sb.AddVertex(0, 1, 0);
    sb.AddFace(0, 1, 2);
    Scene scene = sb.Build();

    CHECK(scene.faces[0].bounds.valid);
    CHECK(std::fabs(scene.faces[0].bounds.min.x - 0.0) < 1e-12);
    CHECK(std::fabs(scene.faces[0].bounds.min.y - 0.0) < 1e-12);
    CHECK(std::fabs(scene.faces[0].bounds.min.z - 0.0) < 1e-12);
    CHECK(std::fabs(scene.faces[0].bounds.max.x - 1.0) < 1e-12);
    CHECK(std::fabs(scene.faces[0].bounds.max.y - 1.0) < 1e-12);
}

TEST_CASE("T4.5 重心") {
    SceneBuilder sb;
    sb.AddVertex(0, 0, 0);
    sb.AddVertex(1, 0, 0);
    sb.AddVertex(0, 1, 0);
    sb.AddFace(0, 1, 2);
    Scene scene = sb.Build();

    CHECK(std::fabs(scene.faces[0].centroid.x - 1.0/3.0) < 1e-12);
    CHECK(std::fabs(scene.faces[0].centroid.y - 1.0/3.0) < 1e-12);
    CHECK(std::fabs(scene.faces[0].centroid.z - 0.0)     < 1e-12);
}

TEST_CASE("T4.6 双侧材质") {
    SceneBuilder sb;
    sb.AddVertex(0, 0, 0);
    sb.AddVertex(1, 0, 0);
    sb.AddVertex(0, 1, 0);
    sb.AddFace(0, 1, 2, "Air", "Concrete", true, true, false);
    Scene scene = sb.Build();

    CHECK(scene.faces[0].front_material_name == "Air");
    CHECK(scene.faces[0].back_material_name  == "Concrete");
    CHECK(scene.faces[0].reflection_enabled);
    CHECK(scene.faces[0].transmission_enabled);
    CHECK_FALSE(scene.faces[0].diffraction_candidate_enabled);
}

TEST_CASE("T4.7 AddWedge — 基本楔边") {
    SceneBuilder sb;
    sb.AddVertex(0, 0, 0);
    sb.AddVertex(2, 0, 0);
    sb.AddVertex(0, 2, 0);
    sb.AddVertex(2, 2, 0);
    sb.AddFace(0, 1, 3);   // face0
    sb.AddFace(0, 3, 2);   // face1
    sb.AddWedge(MakeVec3(0, 0, 0), MakeVec3(2, 0, 0), 0, 1, 90.0);
    Scene scene = sb.Build();

    CHECK(scene.wedges.size() == 1);
    CHECK(scene.wedges[0].positive_face_id == 0);
    CHECK(scene.wedges[0].negative_face_id == 1);
    CHECK(scene.wedges[0].diffractable);
    CHECK(std::fabs(scene.wedges[0].wedge_angle_deg - 90.0) < 1e-12);
    CHECK(std::fabs(scene.wedges[0].length - 2.0) < 1e-12);
}
