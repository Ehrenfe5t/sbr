// test_math.cpp — 向量数学 + Snell 折射单元测试
#include "doctest.h"
#include "sbr/sbr_math.h"
#include <cmath>

using namespace sbr;

// ═══════════════════════════════════════════════════════
// 测试组 1: 向量基本运算
// ═══════════════════════════════════════════════════════

TEST_CASE("T1.1 Dot — 基本点积") {
    Vec3 a = MakeVec3(1, 2, 3);
    Vec3 b = MakeVec3(4, 5, 6);
    double result = Dot(a, b);
    CHECK(std::fabs(result - 32.0) < 1e-15);
}

TEST_CASE("T1.2 Cross — 右手系叉积") {
    Vec3 a = MakeVec3(1, 0, 0);
    Vec3 b = MakeVec3(0, 1, 0);
    Vec3 c = Cross(a, b);
    CHECK(std::fabs(c.x - 0.0) < 1e-15);
    CHECK(std::fabs(c.y - 0.0) < 1e-15);
    CHECK(std::fabs(c.z - 1.0) < 1e-15);
}

TEST_CASE("T1.3 Length — 3-4-5 三角形") {
    Vec3 v = MakeVec3(3, 4, 0);
    CHECK(std::fabs(Length(v) - 5.0) < 1e-15);
}

TEST_CASE("T1.4 Normalize — 基本归一化") {
    Vec3 v = MakeVec3(3, 0, 0);
    Vec3 n = Normalize(v);
    CHECK(std::fabs(n.x - 1.0) < 1e-15);
    CHECK(std::fabs(n.y - 0.0) < 1e-15);
    CHECK(std::fabs(n.z - 0.0) < 1e-15);
}

TEST_CASE("T1.5 SafeNormalize — 零向量 fallback") {
    Vec3 v = MakeVec3(0, 0, 0);
    Vec3 n = SafeNormalize(v, MakeVec3(1, 0, 0));
    CHECK(std::fabs(n.x - 1.0) < 1e-15);
    CHECK(std::fabs(n.y - 0.0) < 1e-15);
    CHECK(std::fabs(n.z - 0.0) < 1e-15);
}

TEST_CASE("T1.6 Reflect — 法向入射") {
    Vec3 inc = MakeVec3(0, 0, 1);
    Vec3 n   = MakeVec3(0, 0, -1);
    Vec3 ref = Reflect(inc, n);
    CHECK(std::fabs(ref.x - 0.0)  < 1e-15);
    CHECK(std::fabs(ref.y - 0.0)  < 1e-15);
    CHECK(std::fabs(ref.z + 1.0) < 1e-15);  // (0,0,-1)
}

TEST_CASE("T1.7 Reflect — 斜入射") {
    Vec3 inc = MakeVec3(0, -1, 0);
    Vec3 n   = MakeVec3(0, 1, 0);
    Vec3 ref = Reflect(inc, n);
    CHECK(std::fabs(ref.x - 0.0) < 1e-15);
    CHECK(std::fabs(ref.y - 1.0) < 1e-15);
    CHECK(std::fabs(ref.z - 0.0) < 1e-15);
}

TEST_CASE("T1.8 Reflect — 30°入射") {
    double sin30 = std::sin(kPi / 6.0);
    double cos30 = std::cos(kPi / 6.0);
    Vec3 inc = MakeVec3(sin30, 0, cos30);
    Vec3 n   = MakeVec3(0, 0, -1);
    Vec3 ref = Reflect(inc, n);
    CHECK(std::fabs(ref.x - sin30)  < 1e-12);
    CHECK(std::fabs(ref.y - 0.0)    < 1e-15);
    CHECK(std::fabs(ref.z + cos30)  < 1e-12);
}

// ═══════════════════════════════════════════════════════
// 测试组 2: Snell 折射 (SnellRefractV2)
// ═══════════════════════════════════════════════════════

TEST_CASE("T2.1 空气→玻璃 30°入射") {
    // 法线朝-Z, 入射方向从+Z侧30°斜入射
    Vec3 incident = MakeVec3(0.5, 0.0, 0.8660254037844386);  // 30° from +Z
    Vec3 normal   = MakeVec3(0.0, 0.0, -1.0);
    double n1 = 1.0, n2 = 1.5;

    SnellResult sr = SnellRefractV2(incident, normal, n1, n2);

    CHECK(sr.valid);
    CHECK_FALSE(sr.total_internal_reflection);
    CHECK(sr.residual < 1e-12);
    CHECK(std::fabs(sr.theta_i_rad - kPi / 6.0) < 1e-6);  // θ_i ≈ 30°

    double expectedThetaT = std::asin(1.0 / 1.5 * 0.5);      // θ_t ≈ 19.47°
    CHECK(std::fabs(sr.theta_t_rad - expectedThetaT) < 1e-6);
}

TEST_CASE("T2.2 玻璃→空气 45° (超过临界角 TIR)") {
    // 临界角 = asin(1.0/1.5) ≈ 41.81°, 45° > 临界角 → TIR
    // 法线朝-Z, 入射方向在XZ平面内, 与法线夹角45°
    double sin45 = std::sin(kPi / 4.0);
    double cos45 = std::cos(kPi / 4.0);
    Vec3 incident = MakeVec3(sin45, 0.0, cos45);  // 45° from +Z axis
    Vec3 normal   = MakeVec3(0.0, 0.0, -1.0);     // normal points -Z (towards glass)
    double n1 = 1.5, n2 = 1.0;                     // glass → air

    SnellResult sr = SnellRefractV2(incident, normal, n1, n2);

    CHECK(sr.valid);
    CHECK(sr.total_internal_reflection);
    // TIR 时 theta_t = π/2
    CHECK(std::fabs(sr.theta_t_rad - kHalfPi) < 1e-12);
}

TEST_CASE("T2.3 垂直入射") {
    // 法线朝-Z, 入射沿+Z (垂直入射)
    Vec3 incident = MakeVec3(0, 0, 1);
    Vec3 normal   = MakeVec3(0, 0, -1);
    double n1 = 1.0, n2 = 1.5;

    SnellResult sr = SnellRefractV2(incident, normal, n1, n2);

    CHECK(sr.valid);
    CHECK_FALSE(sr.total_internal_reflection);
    CHECK(std::fabs(sr.theta_i_rad - 0.0) < 1e-12);
    CHECK(std::fabs(sr.theta_t_rad - 0.0) < 1e-12);
    // 方向不变 (仅过介质)
    CHECK(std::fabs(sr.direction.x - 0.0) < 1e-12);
    CHECK(std::fabs(sr.direction.y - 0.0) < 1e-12);
    CHECK(std::fabs(sr.direction.z - 1.0) < 1e-12);
}

TEST_CASE("T2.4 玻璃→空气 临界角附近") {
    // 临界角 ≈ 41.81°, 入射 41.8° → 非 TIR, θ_t 接近 90°
    double theta_c = std::asin(1.0 / 1.5);  // ≈ 41.81°
    double theta_i = theta_c * 0.999;        // 略小于临界角
    double sint = std::sin(theta_i);
    double cost = std::cos(theta_i);
    Vec3 incident = MakeVec3(sint, 0, cost);
    Vec3 normal   = MakeVec3(0, 0, -1);
    double n1 = 1.5, n2 = 1.0;

    SnellResult sr = SnellRefractV2(incident, normal, n1, n2);

    CHECK(sr.valid);
    CHECK_FALSE(sr.total_internal_reflection);
    // θ_t 应接近 90°
    CHECK(sr.theta_t_rad > 1.4);  // > 80°
    CHECK(sr.theta_t_rad < kHalfPi);
}

TEST_CASE("T2.5 背面入射 (法线自动翻转)") {
    // 法线指向+Z, 入射也从+Z来 → 背面入射 → 法线应自动翻转
    Vec3 incident = MakeVec3(0.5, 0.0, 0.8660254037844386);
    Vec3 normal   = MakeVec3(0.0, 0.0, 1.0);   // +Z
    double n1 = 1.0, n2 = 1.5;

    SnellResult sr = SnellRefractV2(incident, normal, n1, n2);

    CHECK(sr.valid);
    CHECK_FALSE(sr.total_internal_reflection);
    CHECK(sr.residual < 1e-12);
    // 结果应与 T2.1 相同 (自动翻转了法线)
    double expectedThetaT = std::asin(1.0 / 1.5 * 0.5);
    CHECK(std::fabs(sr.theta_t_rad - expectedThetaT) < 1e-6);
}

TEST_CASE("T2.6 掠入射 (接近 90°)") {
    // 入射角 89.9°, 几乎平行于表面
    double theta_i = 89.9 * kPi / 180.0;
    double sint = std::sin(theta_i);
    double cost = std::cos(theta_i);
    Vec3 incident = MakeVec3(sint, 0, cost);
    Vec3 normal   = MakeVec3(0, 0, -1);
    double n1 = 1.0, n2 = 1.5;

    SnellResult sr = SnellRefractV2(incident, normal, n1, n2);

    CHECK(sr.valid);
    CHECK_FALSE(sr.total_internal_reflection);
    // direction 应为有限值
    CHECK(std::isfinite(sr.direction.x));
    CHECK(std::isfinite(sr.direction.y));
    CHECK(std::isfinite(sr.direction.z));
}

TEST_CASE("T2.7 cosI clamp (浮点误差保护)") {
    // cosI 可能因浮点误差 > 1.0 → 不应产生 NaN
    Vec3 incident = MakeVec3(0.0, 0.0, 1.000000000000001);  // 略超
    Vec3 normal   = MakeVec3(0.0, 0.0, -1.0);
    double n1 = 1.0, n2 = 2.0;

    SnellResult sr = SnellRefractV2(incident, normal, n1, n2);

    CHECK(sr.valid);
    CHECK(std::isfinite(sr.direction.x));
    CHECK(std::isfinite(sr.direction.y));
    CHECK(std::isfinite(sr.direction.z));
}
