// sbr_accelerator_bruteforce.cpp — 暴力 O(N) 射线-三角求交
// 使用 Möller-Trumbore 算法 (Journal of Graphics Tools, 1997)
#include "sbr/sbr_accelerator_bruteforce.h"
#include "sbr/sbr_math.h"
#include <algorithm>
#include <cmath>

namespace sbr {

BruteForceAccelerator::BruteForceAccelerator(const Scene& scene)
    : scene_(&scene) {}

void BruteForceAccelerator::BuildFromScene(const Scene& scene) {
    scene_ = &scene;
}

// ── Möller-Trumbore 射线-三角求交 ──
// 返回 true 表示命中, 输出参数 t, u, v (重心坐标)
static bool MollerTrumbore(const Ray& ray,
                            const Vec3& v0, const Vec3& v1, const Vec3& v2,
                            double& t, double& u, double& v) {
    const double eps = 1e-12;

    Vec3 e1 = SubtractVec(v1, v0);
    Vec3 e2 = SubtractVec(v2, v0);
    Vec3 pvec = Cross(ray.direction, e2);
    double det = Dot(e1, pvec);

    // 背面剔除 (可选): 若 det < eps 则跳过
    if (std::fabs(det) < eps) return false;

    double invDet = 1.0 / det;
    Vec3 tvec = SubtractVec(ray.origin, v0);
    u = Dot(tvec, pvec) * invDet;
    if (u < 0.0 || u > 1.0) return false;

    Vec3 qvec = Cross(tvec, e1);
    v = Dot(ray.direction, qvec) * invDet;
    if (v < 0.0 || (u + v) > 1.0) return false;

    t = Dot(e2, qvec) * invDet;
    return t > eps;  // 正向前方命中
}

FaceHit BruteForceAccelerator::QueryClosestFaceHit(
    const Ray& ray, const FaceQueryContext& ctx) const {
    FaceHit best;
    best.distance = 1e308;

    if (!scene_) return best;

    for (const auto& face : scene_->faces) {
        // 忽略过滤
        if (face.face_id == ctx.ignored_face_id ||
            face.face_id == ctx.ignored_face_id2) continue;
        if (face.object_id == ctx.ignored_object_id) continue;

        // 仅返回可传播面
        if (ctx.only_return_propagation_enabled_faces &&
            face.propagation_flags == FacePropagationNone) continue;

        const Vec3& v0 = scene_->vertices[face.vertex_index0];
        const Vec3& v1 = scene_->vertices[face.vertex_index1];
        const Vec3& v2 = scene_->vertices[face.vertex_index2];

        double t, u, v;
        if (MollerTrumbore(ray, v0, v1, v2, t, u, v)) {
            // 原点自交忽略
            if (ctx.ignore_origin_self_hit && t < ctx.origin_ignore_distance) continue;

            if (t < best.distance) {
                best.hit       = true;
                best.face_id   = face.face_id;
                best.object_id = face.object_id;
                best.distance  = t;
                best.position  = Add(ray.origin, Scale(ray.direction, t));
                best.normal    = face.normal;
            }
        }
    }

    return best;
}

bool BruteForceAccelerator::IsOccluded(
    const Point3& start, const Point3& end,
    const VisibilityQueryContext& ctx) const {
    if (!scene_) return false;

    // 偏移起点和终点 (避免自遮挡)
    Vec3 dir = Subtract(end, start);
    double fullLen = Length(dir);
    if (fullLen < 1e-12) return false;

    Vec3 uDir = Scale(dir, 1.0 / fullLen);
    Point3 s = Add(start, Scale(uDir, ctx.origin_offset_distance));
    double maxDist = fullLen - ctx.origin_offset_distance - ctx.target_shrink_distance;
    if (maxDist <= 0.0) return false;

    Ray ray;
    ray.origin    = s;
    ray.direction = uDir;

    for (const auto& face : scene_->faces) {
        if (face.face_id == ctx.ignored_face_id ||
            face.face_id == ctx.ignored_face_id2) continue;
        if (face.object_id == ctx.ignored_object_id) continue;

        const Vec3& v0 = scene_->vertices[face.vertex_index0];
        const Vec3& v1 = scene_->vertices[face.vertex_index1];
        const Vec3& v2 = scene_->vertices[face.vertex_index2];

        double t, u, v;
        if (MollerTrumbore(ray, v0, v1, v2, t, u, v)) {
            if (t > 1e-9 && t < maxDist) return true;  // 有遮挡
        }
    }

    return false;
}

} // namespace sbr
