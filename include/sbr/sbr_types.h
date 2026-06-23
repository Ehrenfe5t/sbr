// sbr_types.h — 基础几何类型定义
// 与 H2hRT rt::Vec3 / rt::Point3 / rt::Ray / rt::FaceHit 布局兼容
#pragma once
#include <cstdint>

namespace sbr {

// ── 数学常量 ──
constexpr double kPi      = 3.14159265358979323846;
constexpr double kHalfPi  = kPi * 0.5;
constexpr double kEpsilon0 = 8.8541878128e-12;  // 真空介电常数
constexpr double kC0       = 299792458.0;       // 真空中光速

// ── 三维向量 ──
struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};
using Point3 = Vec3;

// ── 轴对齐包围盒 ──
struct AABB {
    Vec3 min;
    Vec3 max;
    bool valid = false;
};

// ── 射线 ──
struct Ray {
    Point3 origin;
    Vec3 direction;  // 调用方应保证为单位向量 (或求交时归一化)
};

// ── 面元命中结果 ──
struct FaceHit {
    bool  hit       = false;
    int   face_id   = -1;
    int   object_id = -1;
    double distance = 0.0;   // 射线参数 t, 从 origin 沿 direction
    Point3 position;         // 命中点世界坐标
    Vec3   normal;           // 命中面法向 (归一化)
};

} // namespace sbr
