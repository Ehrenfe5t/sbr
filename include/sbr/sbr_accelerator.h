// sbr_accelerator.h — 场景加速器抽象接口 (与 H2hRT rt::ISceneAccelerator 兼容)
#pragma once
#include "sbr_types.h"
#include "sbr_scene.h"
#include <vector>
#include <string>
#include <memory>

namespace sbr {

// ── 面元命中查询上下文 ──
struct FaceQueryContext {
    int  ignored_face_id   = -1;
    int  ignored_face_id2  = -1;   // 绕射可见性第二忽略面
    int  ignored_object_id = -1;
    bool ignore_origin_self_hit             = true;
    double origin_ignore_distance           = 1.0e-6;
    bool only_return_propagation_enabled_faces = false;
    bool require_dual_side_material_resolved   = false;
};

// ── 可见性查询上下文 ──
struct VisibilityQueryContext {
    int  ignored_face_id   = -1;
    int  ignored_face_id2  = -1;
    int  ignored_object_id = -1;
    bool ignore_origin_attached_face = true;
    bool ignore_target_attached_face = true;
    double origin_offset_distance    = 1.0e-6;
    double target_shrink_distance    = 1.0e-6;
};

// ── 场景加速器抽象接口 ──
class ISceneAccelerator {
public:
    virtual ~ISceneAccelerator() = default;

    // 单射线最近命中
    virtual FaceHit QueryClosestFaceHit(const Ray& ray, const FaceQueryContext& ctx) const = 0;

    // 两点可见性
    virtual bool IsOccluded(const Point3& start, const Point3& end,
                            const VisibilityQueryContext& ctx) const = 0;
    bool IsVisible(const Point3& start, const Point3& end,
                   const VisibilityQueryContext& ctx) const {
        return !IsOccluded(start, end, ctx);
    }

    // 后端名称
    virtual std::string BackendName() const = 0;

    // 场景生命周期
    virtual void BuildFromScene(const Scene& scene) = 0;

protected:
    ISceneAccelerator() = default;
    ISceneAccelerator(const ISceneAccelerator&) = delete;
    ISceneAccelerator& operator=(const ISceneAccelerator&) = delete;
};

} // namespace sbr
