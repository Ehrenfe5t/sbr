// sbr_bvh_accelerator.cpp — SAH-BVH 加速器实现
// 移植自 BVH_Project: 桶排序SAH + 距离优先遍历 + 原子tMin
#include "sbr/sbr_bvh_accelerator.h"
#include "sbr/sbr_math.h"
#include <algorithm>
#include <cmath>
#include <numeric>
#include <atomic>
#include <mutex>
#include <cstdio>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace sbr {

// ═══════════════════════════════════════════════════════════
// 常量
// ═══════════════════════════════════════════════════════════

static constexpr int    kMaxLeafSize       = 8;
static constexpr int    kMaxBVHDepth       = 64;
static constexpr int    kSAHBucketCount    = 16;
static constexpr int    kParallelThreshold = 200;
static constexpr double kAABBEpsilon       = 1e-6;
static constexpr double kRayMaxDist         = 1e6;
static constexpr int    kMaxThreadCount    = 8;

// ═══════════════════════════════════════════════════════════
// AABB 工具
// ═══════════════════════════════════════════════════════════

static AABB MergeAABB(const AABB& a, const AABB& b) {
    if (!a.valid) return b;
    if (!b.valid) return a;
    AABB r;
    r.min.x = std::min(a.min.x, b.min.x);
    r.min.y = std::min(a.min.y, b.min.y);
    r.min.z = std::min(a.min.z, b.min.z);
    r.max.x = std::max(a.max.x, b.max.x);
    r.max.y = std::max(a.max.y, b.max.y);
    r.max.z = std::max(a.max.z, b.max.z);
    r.valid = true;
    return r;
}

static double AABBSurfaceArea(const AABB& aabb) {
    double dx = aabb.max.x - aabb.min.x;
    double dy = aabb.max.y - aabb.min.y;
    double dz = aabb.max.z - aabb.min.z;
    return 2.0 * (dx*dy + dy*dz + dz*dx);
}

static Vec3 FaceCentroid(const Scene& scene, int fi) {
    const Face& f = scene.faces[fi];
    const Vec3& v0 = scene.vertices[f.vertex_index0];
    const Vec3& v1 = scene.vertices[f.vertex_index1];
    const Vec3& v2 = scene.vertices[f.vertex_index2];
    return Scale(AddVec(AddVec(v0, v1), v2), 1.0 / 3.0);
}

AABB BvhAccelerator::ComputeFaceAABB(const Scene& scene, int fi) {
    const Face& f = scene.faces[fi];
    const Vec3& v0 = scene.vertices[f.vertex_index0];
    const Vec3& v1 = scene.vertices[f.vertex_index1];
    const Vec3& v2 = scene.vertices[f.vertex_index2];
    AABB aabb;
    aabb.min = MakeVec3(std::min({v0.x, v1.x, v2.x}) - kAABBEpsilon,
                        std::min({v0.y, v1.y, v2.y}) - kAABBEpsilon,
                        std::min({v0.z, v1.z, v2.z}) - kAABBEpsilon);
    aabb.max = MakeVec3(std::max({v0.x, v1.x, v2.x}) + kAABBEpsilon,
                        std::max({v0.y, v1.y, v2.y}) + kAABBEpsilon,
                        std::max({v0.z, v1.z, v2.z}) + kAABBEpsilon);
    aabb.valid = true;
    return aabb;
}

static AABB BuildAABBForFaces(const Scene& scene, const std::vector<int>& indices) {
    if (indices.empty()) return AABB{};
    AABB aabb = BvhAccelerator::ComputeFaceAABB(scene, indices[0]);
    for (size_t i = 1; i < indices.size(); ++i)
        aabb = MergeAABB(aabb, BvhAccelerator::ComputeFaceAABB(scene, indices[i]));
    return aabb;
}

// ═══════════════════════════════════════════════════════════
// SAH 桶分割
// ═══════════════════════════════════════════════════════════

static double CentroidAxis(const Scene& scene, int fi, int axis) {
    Vec3 c = FaceCentroid(scene, fi);
    return (axis == 0) ? c.x : ((axis == 1) ? c.y : c.z);
}

struct Bucket { AABB bound; std::vector<int> tris; size_t cnt = 0; };

static double SAHCost(const AABB& parent, const AABB& left, size_t lc,
                       const AABB& right, size_t rc) {
    double pa = AABBSurfaceArea(parent);
    if (pa < 1e-12) return 1e308;
    return (AABBSurfaceArea(left)*lc + AABBSurfaceArea(right)*rc) / pa;
}

static bool SAHSplit(const Scene& scene, const std::vector<int>& indices,
                      const AABB& parentAABB, std::vector<int>& outLeft,
                      std::vector<int>& outRight) {
    outLeft.clear(); outRight.clear();
    if (indices.size() < 2) return false;

    double bestCost = 1e308; int bestAxis = -1;
    std::vector<int> bestL, bestR;

    for (int axis = 0; axis < 3; ++axis) {
        double cMin = 1e308, cMax = -1e308;
        for (int fi : indices) {
            double c = CentroidAxis(scene, fi, axis);
            if (c < cMin) cMin = c; if (c > cMax) cMax = c;
        }
        if (cMax - cMin < 1e-12) continue;
        double extent = cMax - cMin;

        Bucket buckets[kSAHBucketCount];
        for (int fi : indices) {
            double c = CentroidAxis(scene, fi, axis);
            int bi = static_cast<int>((c - cMin) / extent * kSAHBucketCount);
            if (bi < 0) bi = 0; if (bi >= kSAHBucketCount) bi = kSAHBucketCount - 1;
            buckets[bi].tris.push_back(fi); buckets[bi].cnt++;
            AABB fa = BvhAccelerator::ComputeFaceAABB(scene, fi);
            buckets[bi].bound = (buckets[bi].cnt == 1) ? fa
                                : MergeAABB(buckets[bi].bound, fa);
        }

        // 前缀和
        AABB la[kSAHBucketCount]; size_t lc[kSAHBucketCount];
        la[0] = buckets[0].bound; lc[0] = buckets[0].cnt;
        for (int i = 1; i < kSAHBucketCount; ++i) {
            la[i] = MergeAABB(la[i-1], buckets[i].bound);
            lc[i] = lc[i-1] + buckets[i].cnt;
        }
        // 后缀和
        AABB ra[kSAHBucketCount]; size_t rc[kSAHBucketCount];
        ra[kSAHBucketCount-1] = buckets[kSAHBucketCount-1].bound;
        rc[kSAHBucketCount-1] = buckets[kSAHBucketCount-1].cnt;
        for (int i = kSAHBucketCount-2; i >= 0; --i) {
            ra[i] = MergeAABB(ra[i+1], buckets[i].bound);
            rc[i] = rc[i+1] + buckets[i].cnt;
        }

        for (int s = 0; s < kSAHBucketCount - 1; ++s) {
            if (lc[s] == 0 || rc[s+1] == 0) continue;
            double cost = SAHCost(parentAABB, la[s], lc[s], ra[s+1], rc[s+1]);
            if (cost < bestCost) {
                bestCost = cost; bestAxis = axis;
                bestL.clear(); bestR.clear();
                for (int i = 0; i <= s; ++i)
                    bestL.insert(bestL.end(), buckets[i].tris.begin(), buckets[i].tris.end());
                for (int i = s+1; i < kSAHBucketCount; ++i)
                    bestR.insert(bestR.end(), buckets[i].tris.begin(), buckets[i].tris.end());
            }
        }
    }

    if (bestAxis < 0 || bestL.empty() || bestR.empty()) return false;
    outLeft = std::move(bestL); outRight = std::move(bestR);
    return true;
}

// ═══════════════════════════════════════════════════════════
// BVH 递归构建
// ═══════════════════════════════════════════════════════════

std::unique_ptr<BvhNode> BvhAccelerator::BuildRecursive(
    std::vector<int> triIndices, int depth) {
    auto node = std::make_unique<BvhNode>();
    node->bound = BuildAABBForFaces(*scene_, triIndices);
    node->depth = depth;
    nodeCount_++;
    if (depth > maxDepth_) maxDepth_ = depth;

    int dynLeaf = kMaxLeafSize + (depth / 8) * 2;
    if (static_cast<int>(triIndices.size()) <= dynLeaf || depth >= kMaxBVHDepth) {
        node->isLeaf = true;
        node->triIndices = std::move(triIndices);
        return node;
    }

    std::vector<int> leftIdx, rightIdx;
    if (!SAHSplit(*scene_, triIndices, node->bound, leftIdx, rightIdx)) {
        node->isLeaf = true;
        node->triIndices = std::move(triIndices);
        return node;
    }

    // 并行构建: 超过阈值时左右子树并行 (不使用 task, MSVC 兼容)
    if (triIndices.size() > kParallelThreshold) {
        std::unique_ptr<BvhNode> l, r;
#ifdef _OPENMP
        #pragma omp parallel sections
        {
            #pragma omp section
            l = BuildRecursive(std::move(leftIdx), depth + 1);
            #pragma omp section
            r = BuildRecursive(std::move(rightIdx), depth + 1);
        }
#else
        l = BuildRecursive(std::move(leftIdx), depth + 1);
        r = BuildRecursive(std::move(rightIdx), depth + 1);
#endif
        node->left = std::move(l);
        node->right = std::move(r);
        return node;
    }

    node->left  = BuildRecursive(std::move(leftIdx), depth + 1);
    node->right = BuildRecursive(std::move(rightIdx), depth + 1);
    return node;
}

// ═══════════════════════════════════════════════════════════
// 射线-AABB 碰撞 (slab test, 预计算逆方向)
// ═══════════════════════════════════════════════════════════

static bool RayAABBIntersect(const Vec3& origin, const Vec3& dirInv, const AABB& aabb) {
    double tMinX = (aabb.min.x - origin.x) * dirInv.x;
    double tMaxX = (aabb.max.x - origin.x) * dirInv.x;
    if (tMinX > tMaxX) std::swap(tMinX, tMaxX);

    double tMinY = (aabb.min.y - origin.y) * dirInv.y;
    double tMaxY = (aabb.max.y - origin.y) * dirInv.y;
    if (tMinY > tMaxY) std::swap(tMinY, tMaxY);

    if (tMinX > tMaxY || tMinY > tMaxX) return false;
    tMinX = std::max(tMinX, tMinY);
    tMaxX = std::min(tMaxX, tMaxY);

    double tMinZ = (aabb.min.z - origin.z) * dirInv.z;
    double tMaxZ = (aabb.max.z - origin.z) * dirInv.z;
    if (tMinZ > tMaxZ) std::swap(tMinZ, tMaxZ);

    if (tMinX > tMaxZ || tMinZ > tMaxX) return false;
    tMinX = std::max(tMinX, tMinZ);
    tMaxX = std::min(tMaxX, tMaxZ);

    return tMaxX >= 0.0 && tMinX < kRayMaxDist;
}

// ═══════════════════════════════════════════════════════════
// Möller-Trumbore 射线-三角求交
// ═══════════════════════════════════════════════════════════

static bool RayTriIntersect(const Scene& scene, int fi,
                             const Vec3& origin, const Vec3& dir,
                             double& tOut, double& uOut, double& vOut) {
    const Face& f = scene.faces[fi];
    const Vec3& p0 = scene.vertices[f.vertex_index0];
    const Vec3& p1 = scene.vertices[f.vertex_index1];
    const Vec3& p2 = scene.vertices[f.vertex_index2];

    Vec3 e1 = SubtractVec(p1, p0);
    Vec3 e2 = SubtractVec(p2, p0);
    Vec3 h = Cross(dir, e2);
    double a = Dot(e1, h);
    if (std::fabs(a) < 1e-12) return false;
    double invA = 1.0 / a;
    Vec3 s = SubtractVec(origin, p0);
    double u = invA * Dot(s, h);
    if (u < 0.0 || u > 1.0) return false;
    Vec3 q = Cross(s, e1);
    double v = invA * Dot(dir, q);
    if (v < 0.0 || u + v > 1.0) return false;
    double t = invA * Dot(e2, q);
    if (t > 1e-9 && t < kRayMaxDist) {
        tOut = t; uOut = u; vOut = v; return true;
    }
    return false;
}

// ═══════════════════════════════════════════════════════════
// 距离优先 BVH 遍历 (closest hit)
// ═══════════════════════════════════════════════════════════

// 射线到 AABB 的最短距离 (准确的 entry distance, 不是到中心)
static double NodeMinDist(const Vec3& origin, const Vec3& dir, const AABB& aabb) {
    // Slab test 计算 entry t
    double tMinX = (aabb.min.x - origin.x) * (std::fabs(dir.x) < 1e-12 ? 1e12 : 1.0/dir.x);
    double tMaxX = (aabb.max.x - origin.x) * (std::fabs(dir.x) < 1e-12 ? 1e12 : 1.0/dir.x);
    if (tMinX > tMaxX) std::swap(tMinX, tMaxX);

    double tMinY = (aabb.min.y - origin.y) * (std::fabs(dir.y) < 1e-12 ? 1e12 : 1.0/dir.y);
    double tMaxY = (aabb.max.y - origin.y) * (std::fabs(dir.y) < 1e-12 ? 1e12 : 1.0/dir.y);
    if (tMinY > tMaxY) std::swap(tMinY, tMaxY);

    if (tMinX > tMaxY || tMinY > tMaxX) return kRayMaxDist;  // no intersection

    double tMinZ = (aabb.min.z - origin.z) * (std::fabs(dir.z) < 1e-12 ? 1e12 : 1.0/dir.z);
    double tMaxZ = (aabb.max.z - origin.z) * (std::fabs(dir.z) < 1e-12 ? 1e12 : 1.0/dir.z);
    if (tMinZ > tMaxZ) std::swap(tMinZ, tMaxZ);

    if (tMinX > tMaxZ || tMinZ > tMaxX) return kRayMaxDist;

    double tEnter = std::max({tMinX, tMinY, tMinZ, 0.0});
    double tExit  = std::min({tMaxX, tMaxY, tMaxZ});
    if (tEnter > tExit) return kRayMaxDist;

    return tEnter;  // 真实 entry distance
}

static void TraverseClosest(const Scene& scene, const BvhNode* node,
                             const Vec3& origin, const Vec3& dir,
                             const Vec3& dirInv,
                             int ignoreFace, int ignoreFace2, int ignoreObj,
                             double& tMin, int& bestFace, int& bestObj,
                             Vec3& bestNormal, bool propOnly) {
    if (!node) return;

    // 距离优先早期终止
    if (NodeMinDist(origin, dir, node->bound) >= tMin) return;
    if (!RayAABBIntersect(origin, dirInv, node->bound)) return;

    if (node->isLeaf) {
        for (int fi : node->triIndices) {
            if (fi == ignoreFace || fi == ignoreFace2) continue;
            const Face& f = scene.faces[fi];
            if (f.object_id == ignoreObj) continue;
            if (propOnly && f.propagation_flags == 0) continue;
            double t, u, v;
            if (RayTriIntersect(scene, fi, origin, dir, t, u, v)) {
                if (t < tMin) {
                    tMin = t; bestFace = fi; bestObj = f.object_id;
                    bestNormal = f.normal;
                }
            }
        }
        return;
    }

    double dl = NodeMinDist(origin, dir, node->left->bound);
    double dr = NodeMinDist(origin, dir, node->right->bound);
    if (dl < dr) {
        TraverseClosest(scene, node->left.get(), origin, dir, dirInv,
                        ignoreFace, ignoreFace2, ignoreObj, tMin,
                        bestFace, bestObj, bestNormal, propOnly);
        TraverseClosest(scene, node->right.get(), origin, dir, dirInv,
                        ignoreFace, ignoreFace2, ignoreObj, tMin,
                        bestFace, bestObj, bestNormal, propOnly);
    } else {
        TraverseClosest(scene, node->right.get(), origin, dir, dirInv,
                        ignoreFace, ignoreFace2, ignoreObj, tMin,
                        bestFace, bestObj, bestNormal, propOnly);
        TraverseClosest(scene, node->left.get(), origin, dir, dirInv,
                        ignoreFace, ignoreFace2, ignoreObj, tMin,
                        bestFace, bestObj, bestNormal, propOnly);
    }
}

// ═══════════════════════════════════════════════════════════
// 遮挡查询 (any-hit, 提前终止)
// ═══════════════════════════════════════════════════════════

static bool TraverseOccluded(const Scene& scene, const BvhNode* node,
                              const Vec3& origin, const Vec3& dir,
                              const Vec3& dirInv,
                              int ignoreFace, int ignoreFace2, int ignoreObj,
                              double maxDist) {
    if (!node) return false;
    if (NodeMinDist(origin, dir, node->bound) >= maxDist) return false;
    if (!RayAABBIntersect(origin, dirInv, node->bound)) return false;

    if (node->isLeaf) {
        for (int fi : node->triIndices) {
            if (fi == ignoreFace || fi == ignoreFace2) continue;
            const Face& f = scene.faces[fi];
            if (f.object_id == ignoreObj) continue;
            double t, u, v;
            if (RayTriIntersect(scene, fi, origin, dir, t, u, v)) {
                if (t < maxDist) return true;
            }
        }
        return false;
    }

    // any-hit: 优先近子节点
    double dl = NodeMinDist(origin, dir, node->left->bound);
    double dr = NodeMinDist(origin, dir, node->right->bound);
    if (dl < dr) {
        if (TraverseOccluded(scene, node->left.get(), origin, dir, dirInv,
                             ignoreFace, ignoreFace2, ignoreObj, maxDist)) return true;
        return TraverseOccluded(scene, node->right.get(), origin, dir, dirInv,
                                ignoreFace, ignoreFace2, ignoreObj, maxDist);
    } else {
        if (TraverseOccluded(scene, node->right.get(), origin, dir, dirInv,
                             ignoreFace, ignoreFace2, ignoreObj, maxDist)) return true;
        return TraverseOccluded(scene, node->left.get(), origin, dir, dirInv,
                                ignoreFace, ignoreFace2, ignoreObj, maxDist);
    }
}

// ═══════════════════════════════════════════════════════════
// BvhAccelerator 公共接口
// ═══════════════════════════════════════════════════════════

BvhAccelerator::BvhAccelerator() = default;

BvhAccelerator::BvhAccelerator(const Scene& scene) : scene_(&scene) {
    BuildBVH();
}

void BvhAccelerator::BuildFromScene(const Scene& scene) {
    scene_ = &scene;
    BuildBVH();
}

void BvhAccelerator::BuildBVH() {
    if (!scene_ || scene_->faces.empty()) return;
    nodeCount_ = 0; maxDepth_ = 0;

    std::vector<int> indices(scene_->faces.size());
    std::iota(indices.begin(), indices.end(), 0);

    root_ = BuildRecursive(std::move(indices), 0);

    std::printf("[BVH] Built: %d nodes, max depth %d, %zu faces\n",
                nodeCount_, maxDepth_, scene_->faces.size());
}

FaceHit BvhAccelerator::QueryClosestFaceHit(const Ray& ray,
                                              const FaceQueryContext& ctx) const {
    FaceHit result;
    if (!root_ || !scene_) return result;

    Vec3 dirInv(MakeVec3(
        std::fabs(ray.direction.x) < 1e-12 ? 1e12 : 1.0/ray.direction.x,
        std::fabs(ray.direction.y) < 1e-12 ? 1e12 : 1.0/ray.direction.y,
        std::fabs(ray.direction.z) < 1e-12 ? 1e12 : 1.0/ray.direction.z));

    double tMin = kRayMaxDist;
    int bestFace = -1, bestObj = -1;
    Vec3 bestNormal;

    TraverseClosest(*scene_, root_.get(), ray.origin, ray.direction, dirInv,
                    ctx.ignored_face_id, ctx.ignored_face_id2,
                    ctx.ignored_object_id,
                    tMin, bestFace, bestObj, bestNormal,
                    ctx.only_return_propagation_enabled_faces);

    if (bestFace >= 0) {
        result.hit = true;
        result.face_id = bestFace;
        result.object_id = bestObj;
        result.distance = tMin;
        result.position = Add(ray.origin, Scale(ray.direction, tMin));
        result.normal = bestNormal;

        // 自交忽略
        if (ctx.ignore_origin_self_hit && tMin < ctx.origin_ignore_distance) {
            result = FaceHit{};
        }
    }
    return result;
}

bool BvhAccelerator::IsOccluded(const Point3& start, const Point3& end,
                                 const VisibilityQueryContext& ctx) const {
    if (!root_ || !scene_) return false;

    Vec3 dir = Subtract(end, start);
    double fullLen = Length(dir);
    if (fullLen < 1e-12) return false;

    Vec3 uDir = Scale(dir, 1.0 / fullLen);
    Point3 s = Add(start, Scale(uDir, ctx.origin_offset_distance));
    double maxDist = fullLen - ctx.origin_offset_distance - ctx.target_shrink_distance;
    if (maxDist <= 0.0) return false;

    Vec3 dirInv(MakeVec3(
        std::fabs(uDir.x) < 1e-12 ? 1e12 : 1.0/uDir.x,
        std::fabs(uDir.y) < 1e-12 ? 1e12 : 1.0/uDir.y,
        std::fabs(uDir.z) < 1e-12 ? 1e12 : 1.0/uDir.z));

    return TraverseOccluded(*scene_, root_.get(), s, uDir, dirInv,
                            ctx.ignored_face_id, ctx.ignored_face_id2,
                            ctx.ignored_object_id, maxDist);
}

int BvhAccelerator::NodeCount() const { return nodeCount_; }
int BvhAccelerator::MaxDepth() const { return maxDepth_; }

} // namespace sbr
