// sbr_accelerator_bruteforce.h — 暴力 O(N) 加速器桩
// Phase 0-3 使用; Phase 5 替换为 BVH/Embree
#pragma once
#include "sbr_accelerator.h"
#include "sbr_scene.h"

namespace sbr {

class BruteForceAccelerator : public ISceneAccelerator {
public:
    explicit BruteForceAccelerator(const Scene& scene);

    FaceHit QueryClosestFaceHit(const Ray& ray, const FaceQueryContext& ctx) const override;
    bool IsOccluded(const Point3& start, const Point3& end,
                    const VisibilityQueryContext& ctx) const override;
    std::string BackendName() const override { return "BruteForce(CPU)"; }
    void BuildFromScene(const Scene& scene) override;

private:
    const Scene* scene_ = nullptr;
};

} // namespace sbr
