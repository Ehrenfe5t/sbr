// sbr_bvh_accelerator.h — SAH-BVH 加速器 (移植自 BVH_Project, 零外部依赖)
// 算法: 桶排序SAH分割 + 距离优先遍历 + 原子tMin早起终止
#pragma once
#include "sbr_accelerator.h"
#include "sbr_scene.h"
#include <memory>
#include <atomic>
#include <mutex>

namespace sbr {

// BVH 节点 (AABB 包围)
struct BvhNode {
    bool isLeaf = false;
    AABB bound;
    std::unique_ptr<BvhNode> left;
    std::unique_ptr<BvhNode> right;
    int depth = 0;
    std::vector<int> triIndices;  // leaf only

    BvhNode() = default;
    BvhNode(const BvhNode&) = delete;
    BvhNode& operator=(const BvhNode&) = delete;
    BvhNode(BvhNode&&) = default;
    BvhNode& operator=(BvhNode&&) = default;
};

class BvhAccelerator : public ISceneAccelerator {
public:
    BvhAccelerator();
    explicit BvhAccelerator(const Scene& scene);

    FaceHit QueryClosestFaceHit(const Ray& ray, const FaceQueryContext& ctx) const override;
    bool IsOccluded(const Point3& start, const Point3& end,
                    const VisibilityQueryContext& ctx) const override;
    std::string BackendName() const override { return "SAH-BVH(CPU)"; }
    void BuildFromScene(const Scene& scene) override;

    // 单面 AABB (公开, SAH 分割需要)
    static AABB ComputeFaceAABB(const Scene& scene, int faceIdx);

    // 诊断
    int NodeCount() const;
    int MaxDepth() const;

private:
    void BuildBVH();
    std::unique_ptr<BvhNode> BuildRecursive(std::vector<int> triIndices, int depth);

    const Scene* scene_ = nullptr;
    std::unique_ptr<BvhNode> root_;
    int nodeCount_ = 0;
    int maxDepth_  = 0;
};

} // namespace sbr
