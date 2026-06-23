// sbr_math.h — 向量数学函数 (全部 inline, 与 H2hRT rt::Vec3 函数完全一致)
// 关键: SnellRefractV2 的自动法线翻转、cos clamp、TIR 处理均与 H2hRT 一致
#pragma once
#include "sbr_types.h"
#include <cmath>

namespace sbr {

// ═══════════════════════════════════════════════════════════
// 构造与基本运算
// ═══════════════════════════════════════════════════════════

inline Vec3 MakeVec3(double x, double y, double z) {
    Vec3 v; v.x = x; v.y = y; v.z = z; return v;
}

inline Vec3 SubtractVec(const Vec3& a, const Vec3& b) {
    return MakeVec3(a.x - b.x, a.y - b.y, a.z - b.z);
}

inline Vec3 Subtract(const Point3& a, const Point3& b) {
    return MakeVec3(a.x - b.x, a.y - b.y, a.z - b.z);
}

inline Vec3 AddVec(const Vec3& a, const Vec3& b) {
    return MakeVec3(a.x + b.x, a.y + b.y, a.z + b.z);
}

inline Point3 Add(const Point3& a, const Vec3& b) {
    Point3 r; r.x = a.x + b.x; r.y = a.y + b.y; r.z = a.z + b.z; return r;
}

inline Vec3 Scale(const Vec3& v, double s) {
    return MakeVec3(v.x * s, v.y * s, v.z * s);
}

// ═══════════════════════════════════════════════════════════
// 点积 / 叉积 / 长度 / 归一化
// ═══════════════════════════════════════════════════════════

inline double Dot(const Vec3& a, const Vec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline Vec3 Cross(const Vec3& a, const Vec3& b) {
    return MakeVec3(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x);
}

inline double LengthSq(const Vec3& v) {
    return Dot(v, v);
}

inline double Length(const Vec3& v) {
    return std::sqrt(LengthSq(v));
}

inline Vec3 Normalize(const Vec3& v) {
    double len = Length(v);
    if (len <= 0.0) { return Vec3{}; }
    return Scale(v, 1.0 / len);
}

// 安全归一化: 零向量时返回 fallback 而非零向量 (与 H2hRT 完全一致)
inline Vec3 SafeNormalize(const Vec3& v, const Vec3& fallback = MakeVec3(1, 0, 0)) {
    double len = Length(v);
    if (len <= 1e-12) { return fallback; }
    return Scale(v, 1.0 / len);
}

// ═══════════════════════════════════════════════════════════
// 钳制
// ═══════════════════════════════════════════════════════════

inline double Clamp(double value, double lo, double hi) {
    if (value < lo) return lo;
    if (value > hi) return hi;
    return value;
}

// ═══════════════════════════════════════════════════════════
// 反射方向
// d_r = d_i - 2(d_i · n) n
// ═══════════════════════════════════════════════════════════

inline Vec3 Reflect(const Vec3& dir, const Vec3& normal) {
    Vec3 n = Normalize(normal);
    double d = Dot(dir, n);
    return MakeVec3(
        dir.x - 2.0 * d * n.x,
        dir.y - 2.0 * d * n.y,
        dir.z - 2.0 * d * n.z);
}

// ═══════════════════════════════════════════════════════════
// Snell 折射 (旧版, 保持兼容)
// ═══════════════════════════════════════════════════════════

inline Vec3 SnellRefract(const Vec3& incident, const Vec3& normal, double n1, double n2) {
    double eta = n1 / n2;
    double cosI = -Dot(incident, normal);
    double sinT2 = eta * eta * (1.0 - cosI * cosI);
    if (sinT2 > 1.0) {
        return Vec3{};  // 全内反射
    }
    double cosT = std::sqrt(1.0 - sinT2);
    return MakeVec3(
        eta * incident.x + (eta * cosI - cosT) * normal.x,
        eta * incident.y + (eta * cosI - cosT) * normal.y,
        eta * incident.z + (eta * cosI - cosT) * normal.z);
}

// ═══════════════════════════════════════════════════════════
// SnellResult — 结构化折射结果 (v9, 与 H2hRT 完全一致)
// ═══════════════════════════════════════════════════════════

struct SnellResult {
    bool   valid                     = false;  // 计算是否有效
    bool   total_internal_reflection = false;  // 是否 TIR
    Vec3   direction;                          // 折射方向 (TIR 时为零向量)
    double cos_i                   = 0.0;      // |cos(入射角)|, clamped [0,1]
    double cos_t                   = 0.0;      // |cos(出射角)|, TIR 时为 0
    double theta_i_rad             = 0.0;      // 入射角 (弧度)
    double theta_t_rad             = 0.0;      // 出射角 (弧度), TIR 时为 π/2
    double residual                = 0.0;      // |n1·sin(θ_i) - n2·sin(θ_t)|
};

// ═══════════════════════════════════════════════════════════
// SnellRefractV2 — 增强版 Snell 折射 (与 H2hRT 完全一致)
//
// 相比旧版 SnellRefract 的改进:
//   - 自动翻转法线, 确保 cos_i >= 0 (背面入射时自动校正)
//   - cosI clamp 到 [0,1], 防止浮点误差导致 NaN
//   - 输出入射角 / 出射角 / TIR 标志 / Snell 残差等诊断信息
// ═══════════════════════════════════════════════════════════

inline SnellResult SnellRefractV2(const Vec3& incident, const Vec3& normal,
                                   double n1, double n2) {
    SnellResult result;

    // 自动翻转法线: 确保法线指向入射侧 (Dot(incident, effectiveNormal) <= 0)
    Vec3 effectiveNormal = normal;
    double dotIN = Dot(incident, normal);
    if (dotIN > 0.0) {
        // 入射方向与法线同侧 → 法线需要翻转
        effectiveNormal = Scale(normal, -1.0);
        dotIN = -dotIN;
    }

    // cos(入射角) = -Dot(incident, effectiveNormal), 应 >= 0
    double cosI = -dotIN;
    // Clamp 到 [0,1] 防止浮点误差导致 cosI>1 或 cosI<0
    if (cosI > 1.0) cosI = 1.0;
    if (cosI < 0.0) cosI = 0.0;

    result.cos_i = cosI;
    result.theta_i_rad = std::acos(cosI);

    double eta = n1 / n2;
    double sin2I = 1.0 - cosI * cosI;
    double sinT2 = eta * eta * sin2I;

    if (sinT2 >= 1.0) {
        // ── 全内反射 ──
        result.total_internal_reflection = true;
        result.valid = true;
        result.cos_t = 0.0;
        result.theta_t_rad = kHalfPi;
        // TIR 物理上仍在临界角满足 Snell: n1*sin(θ_c) = n2*sin(π/2)
        result.residual = 0.0;
        // direction 保持零向量
        return result;
    }

    double cosT = std::sqrt(1.0 - sinT2);
    result.cos_t = cosT;
    result.theta_t_rad = std::acos(cosT);

    // Snell 残差: |n1·sin(θ_i) - n2·sin(θ_t)|
    double sinI = std::sqrt(sin2I);
    double sinT = std::sqrt(1.0 - cosT * cosT);
    result.residual = std::fabs(n1 * sinI - n2 * sinT);

    // 计算折射方向: eta * incident + (eta*cos_i - cos_t) * effectiveNormal
    double coeff = eta * cosI - cosT;
    result.direction = MakeVec3(
        eta * incident.x + coeff * effectiveNormal.x,
        eta * incident.y + coeff * effectiveNormal.y,
        eta * incident.z + coeff * effectiveNormal.z);

    result.valid = true;
    return result;
}

} // namespace sbr
