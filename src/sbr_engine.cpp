// sbr_engine.cpp — Phase 4 engine + occlusion fix + debug trace
// 关键修复: (1) dotDN < 0 → 命中正面; (2) Rx遮挡验证: Rx与来向异侧→跳过
// #define SBR_DEBUG_TRACE to enable per-hit logging
#include "sbr/sbr_engine.h"
#include "sbr/sbr_ray_emitter.h"
#include "sbr/sbr_math.h"
#include <cmath>
#include <cstdio>
#include <unordered_set>
#include <unordered_map>
#include <algorithm>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace sbr {

SbrEngine::SbrEngine(std::unique_ptr<ISceneAccelerator> accelerator)
    : accelerator_(std::move(accelerator)) {}

// ═══════════════════════════════════════════════════════════
// 辅助: 距离 / Fresnel
// ═══════════════════════════════════════════════════════════

static double PointToSegmentDistSq(const Point3& p, const Point3& a, const Point3& b) {
    double abx = b.x - a.x, aby = b.y - a.y, abz = b.z - a.z;
    double apx = p.x - a.x, apy = p.y - a.y, apz = p.z - a.z;
    double ab2 = abx * abx + aby * aby + abz * abz;
    if (ab2 <= 0.0) return apx * apx + apy * apy + apz * apz;
    double t = (apx * abx + apy * aby + apz * abz) / ab2;
    if (t <= 0.0) return apx * apx + apy * apy + apz * apz;
    if (t >= 1.0) {
        double dx = p.x - b.x, dy = p.y - b.y, dz = p.z - b.z;
        return dx * dx + dy * dy + dz * dz;
    }
    double cx = a.x + t * abx, cy = a.y + t * aby, cz = a.z + t * abz;
    double dx = p.x - cx, dy = p.y - cy, dz = p.z - cz;
    return dx * dx + dy * dy + dz * dz;
}

static double FresnelPowerNormal(double n1, double n2) {
    double r = (n1 - n2) / (n1 + n2);
    return r * r;
}

// ═══════════════════════════════════════════════════════════
// 绕射辅助
// ═══════════════════════════════════════════════════════════

static int DetectEdgeHit(const Point3& hitPt, const Face& face,
                          const Scene& scene, double eps) {
    int edgeIds[3] = {face.adjacent_edge_id0,
                      face.adjacent_edge_id1,
                      face.adjacent_edge_id2};
    for (int ei = 0; ei < 3; ++ei) {
        int eid = edgeIds[ei];
        if (eid < 0 || eid >= static_cast<int>(scene.edges.size())) continue;
        if (PointToSegmentDistSq(hitPt, scene.edges[eid].start, scene.edges[eid].end) < eps * eps)
            return eid;
    }
    return -1;
}

static int FindWedgeByEdgeId(const Scene& scene, int edgeId) {
    for (size_t wi = 0; wi < scene.wedges.size(); ++wi)
        if (scene.wedges[wi].source_edge_id == edgeId && scene.wedges[wi].diffractable)
            return static_cast<int>(wi);
    return -1;
}

static std::vector<Vec3> GenerateKellerConeDirections(
    const Vec3& incidentDir, const Vec3& wedgeDir, int numSamples) {
    std::vector<Vec3> dirs;
    if (numSamples <= 0) return dirs;
    double c = Dot(incidentDir, wedgeDir);
    if (c > 1.0) c = 1.0; if (c < -1.0) c = -1.0;
    double s = std::sqrt(std::max(0.0, 1.0 - c * c));
    Vec3 p = (std::fabs(wedgeDir.x) < 0.9) ? Cross(wedgeDir, MakeVec3(1, 0, 0))
                                             : Cross(wedgeDir, MakeVec3(0, 1, 0));
    double pLen = Length(p);
    if (pLen < 1e-12) return dirs;
    p = Scale(p, 1.0 / pLen);
    Vec3 q = Cross(wedgeDir, p);
    dirs.reserve(numSamples);
    for (int i = 0; i < numSamples; ++i) {
        double phi = 2.0 * kPi * static_cast<double>(i) / numSamples;
        dirs.push_back(Normalize(AddVec(Scale(wedgeDir, c),
            AddVec(Scale(p, s * std::cos(phi)), Scale(q, s * std::sin(phi))))));
    }
    return dirs;
}

// ═══════════════════════════════════════════════════════════
// 后处理
// ═══════════════════════════════════════════════════════════

static double PowerProxyScore(const GeometricPath& path) {
    double len = std::max(path.total_length, 1e-3);
    double score = -2.0 * std::log(len);
    for (const auto& node : path.nodes) {
        switch (node.interaction_type) {
        case InteractionType::Reflection:   score += std::log(0.45); break;
        case InteractionType::Transmission: score += std::log(0.35); break;
        case InteractionType::Diffraction:  score += std::log(0.08); break;
        default: break;
        }
    }
    return score;
}

static uint64_t BuildPathSignature(const GeometricPath& path) {
    uint64_t h = 1469598103934665603ull;
    auto mix = [&](uint64_t v) { h ^= v; h *= 1099511628211ull; };
    for (const auto& node : path.nodes) {
        mix(static_cast<uint64_t>(static_cast<int>(node.interaction_type) + 257));
        if (node.face_id >= 0)   mix(static_cast<uint64_t>(node.face_id + 4099));
        if (node.wedge_id >= 0)  mix(static_cast<uint64_t>(node.wedge_id + 8191));
        if (node.object_id >= 0) mix(static_cast<uint64_t>(node.object_id + 16381));
    }
    mix(static_cast<uint64_t>(static_cast<long long>(path.total_length * 100.0)));
    return h;
}

static uint64_t SimilarityKey(const GeometricPath& path, double lengthTolM) {
    double tol = std::max(lengthTolM, 1e-6);
    uint64_t h = 1469598103934665603ull;
    auto mix = [&](uint64_t v) { h ^= v; h *= 1099511628211ull; };
    mix(static_cast<uint64_t>(std::llround(path.total_length / tol)));
    for (const auto& node : path.nodes) {
        mix(static_cast<uint64_t>(static_cast<int>(node.interaction_type) + 257));
        if (node.face_id >= 0)  mix(static_cast<uint64_t>(node.face_id + 4099));
        if (node.wedge_id >= 0) mix(static_cast<uint64_t>(node.wedge_id + 8191));
    }
    return h;
}

static GeometricPathSet PostProcess(const GeometricPathSet& raw, const SbrConfig& config) {
    GeometricPathSet result;
    if (raw.paths.empty()) return result;
    if (config.enable_path_dedup) {
        std::unordered_set<uint64_t> seen;
        for (const auto& p : raw.paths)
            if (seen.insert(BuildPathSignature(p)).second) result.paths.push_back(p);
    } else {
        result.paths = raw.paths;
    }
    if (config.enable_path_similarity_pruning && result.paths.size() > 1) {
        std::unordered_map<uint64_t, size_t> bestIdx;
        for (size_t i = 0; i < result.paths.size(); ++i) {
            uint64_t key = SimilarityKey(result.paths[i], config.path_similarity_length_tol_m);
            auto it = bestIdx.find(key);
            if (it == bestIdx.end() || PowerProxyScore(result.paths[i]) >
                                         PowerProxyScore(result.paths[it->second]))
                bestIdx[key] = i;
        }
        GeometricPathSet pruned;
        for (auto& kv : bestIdx) pruned.paths.push_back(result.paths[kv.second]);
        result = std::move(pruned);
    }
    std::sort(result.paths.begin(), result.paths.end(),
        [](const GeometricPath& a, const GeometricPath& b) {
            return PowerProxyScore(a) > PowerProxyScore(b);
        });
    int topN = config.path_top_n_per_rx;
    if (topN > 0 && static_cast<int>(result.paths.size()) > topN)
        result.paths.resize(topN);
    return result;
}

// ═══════════════════════════════════════════════════════════
// 独立绕射寻径 — RT.XD 解析Fermat + UTD 两阶段验证
// ═══════════════════════════════════════════════════════════

// RT.XD BuildGeometryPathDByAnalyticalSolution_node_line 的移植
static bool AnalyticalFermatPoint(
    const Point3& tx, const Point3& rx,
    const Point3& edgeStart, const Point3& edgeEnd,
    Point3& diffPt, double& t) {
    Vec3 edgeDir = SubtractVec(edgeEnd, edgeStart);
    double edgeLen2 = LengthSq(edgeDir);
    if (edgeLen2 < 1e-12) return false;

    // 投影 Tx 到边直线 → 影子点 E
    Vec3 txVec = SubtractVec(tx, edgeStart);
    double tTx = Dot(txVec, edgeDir) / edgeLen2;
    Point3 E = Add(edgeStart, Scale(edgeDir, tTx));
    double h1 = Length(SubtractVec(tx, E));

    // 投影 Rx 到边直线 → 影子点 C
    Vec3 rxVec = SubtractVec(rx, edgeStart);
    double tRx = Dot(rxVec, edgeDir) / edgeLen2;
    Point3 C = Add(edgeStart, Scale(edgeDir, tRx));
    double h2 = Length(SubtractVec(rx, C));

    // Fermat加权插值: diffPoint = C + (h2/(h1+h2)) × (E - C)
    double hSum = h1 + h2;
    if (hSum < 1e-12) { diffPt = C; t = tRx; return true; }

    double k = h2 / hSum;
    Vec3 CE = SubtractVec(E, C);
    diffPt = Add(C, Scale(CE, k));

    // 验证点在边线段上 [0, 1]
    t = Dot(SubtractVec(diffPt, edgeStart), edgeDir) / edgeLen2;
    return (t >= 0.0 && t <= 1.0);
}

static GeometricPathSet RunDiffraction(
    const Scene& scene, const MaterialDatabase& matDb, const SbrConfig& config,
    const Point3& txPoint, const std::vector<Point3>& rxPoints,
    ISceneAccelerator* accel) {
    GeometricPathSet result;
    if (config.max_diffraction_count <= 0) return result;
    if (scene.wedges.empty()) return result;
    if (!accel) return result;

    int diagTotal = 0, diagFermat = 0, diagVisible = 0;

    for (const auto& wedge : scene.wedges) {
        if (!wedge.diffractable) continue;  // 阶段1: dihedral ∈ [3°,177°], 排除共面
        diagTotal++;

        for (const auto& rx : rxPoints) {
            // ★ 阶段2: 大侧(183°~357°) = 自由空间, 小侧(3°~177°) = 实体侧
            //   Tx和Rx必须都在大侧 → 即至少一面法线不指向该点(Dot<0)
            double extAngle = wedge.wedge_angle_deg;
            if (extAngle < 183.0 || extAngle > 357.0) continue;  // 非大侧→跳过
            if (wedge.positive_face_id >= 0 && wedge.negative_face_id >= 0 &&
                wedge.positive_face_id < (int)scene.faces.size() &&
                wedge.negative_face_id < (int)scene.faces.size()) {
                const Vec3& n0 = scene.faces[wedge.positive_face_id].normal;
                const Vec3& n1 = scene.faces[wedge.negative_face_id].normal;
                auto onLargeSide = [&](const Point3& p) {
                    double d0 = Dot(SubtractVec(p, wedge.center_point), n0);
                    double d1 = Dot(SubtractVec(p, wedge.center_point), n1);
                    return !(d0 > 0.0 && d1 > 0.0);  // 不同时>0 → 不在小侧 → 在大侧
                };
                if (!onLargeSide(txPoint) || !onLargeSide(rx)) continue;
            }

            // ── 解析Fermat: 闭式求解绕射点 ──
            Point3 diffPt; double t;
            if (!AnalyticalFermatPoint(txPoint, rx, wedge.segment_start, wedge.segment_end, diffPt, t))
                continue;
            diagFermat++;

            double s1 = Length(SubtractVec(diffPt, txPoint));
            double s2 = Length(SubtractVec(rx, diffPt));
            if (s1 < 0.01 || s2 < 0.01) continue;

            // ★ 唯一判据 (Kouyoumjian & Pathak 1974): 路径可见性
            //    忽略楔边两面后, Tx→绕射点 和 绕射点→Rx 均无遮挡 → 有效绕射径
            VisibilityQueryContext vqc;
            vqc.ignored_face_id  = wedge.positive_face_id;
            vqc.ignored_face_id2 = wedge.negative_face_id;
            vqc.origin_offset_distance = 1e-3;
            vqc.target_shrink_distance  = 1e-3;
            if (!accel->IsVisible(txPoint, diffPt, vqc)) continue;
            if (!accel->IsVisible(diffPt, rx, vqc)) continue;
            diagVisible++;

            // 记录路径
            GeometricPath gp; gp.is_los = false;
            PathNode txNode; txNode.interaction_type = InteractionType::Tx;
            txNode.point = txPoint; txNode.valid = true;
            gp.nodes.push_back(txNode);
            PathNode diffNode; diffNode.interaction_type = InteractionType::Diffraction;
            diffNode.point = diffPt; diffNode.wedge_id = wedge.wedge_id;
            diffNode.surface_normal = wedge.direction;
            diffNode.segment_length_from_previous = s1;
            diffNode.diffraction_diag.s1 = s1;
            diffNode.diffraction_diag.s2 = s2;
            diffNode.diffraction_diag.keller_residual =
                std::fabs(Dot(SubtractVec(diffPt, txPoint), wedge.direction) -
                          Dot(SubtractVec(rx, diffPt), wedge.direction)) / s1;
            diffNode.valid = true;
            gp.nodes.push_back(diffNode);
            PathNode rxNode; rxNode.interaction_type = InteractionType::Rx;
            rxNode.point = rx;
            rxNode.segment_length_from_previous = s2; rxNode.valid = true;
            gp.nodes.push_back(rxNode);
            gp.total_length = s1 + s2; gp.valid = true;
            result.paths.push_back(gp);
        }
    }

    std::printf("[Diff] candidates=%d  fermat=%d  visible=%d  paths=%zu\n",
                diagTotal, diagFermat, diagVisible, result.paths.size());
    return result;
}

// ═══════════════════════════════════════════════════════════
// RunPointToPoint (栈式 DFS, 确定性 R/T 分裂, 绕射已解耦)
// ═══════════════════════════════════════════════════════════

struct TraceState {
    Point3 curPt; Vec3 curDir; double curPwr;
    int cr, ct, depth, lastFaceId, currentMediumId;
    std::vector<PathNode> nodes;
    double cumulativeLength;
};

GeometricPathSet SbrEngine::RunPointToPoint(
    const Scene& scene, const MaterialDatabase& matDb, const SbrConfig& config,
    const Point3& txPoint, const std::vector<Point3>& rxPoints,
    const NumericToleranceConfig& tol) {

    if (!accelerator_) return GeometricPathSet{};
    auto directions = GenerateFibonacciRays(config.ray_count);
    int N = static_cast<int>(directions.size());
    if (N <= 0) return GeometricPathSet{};

    // ── RxHashGrid ──
    // ... using direct O(N) check for small Rx counts ...
    // For Phase 4 simplicity, direct per-Rx check
    const double baseRadius = config.rx_sphere_radius_m;
    const double baseR2 = baseRadius * baseRadius;

    int nTh = 1;
#ifdef _OPENMP
    nTh = omp_get_max_threads();
#endif
    std::vector<GeometricPathSet> threadResults(nTh);
    std::atomic<long long> diagOccluded{0}, diagAccepted{0};

    const double pwrTh = std::pow(10.0, config.ray_power_threshold_dB / 10.0);
    const double freqHz = config.center_frequency_hz;
    const int maxDepth = std::min(config.max_ray_depth,
        config.max_reflection_count + config.max_transmission_count + config.max_diffraction_count);
    const double originIgnoreDist = tol.self_hit_ignore_distance;
    const double FAR_DIST = 1e6;
    const bool hasMaterials = (!matDb.empty());

    // 动态接收球
    double tubeAngle = config.ray_tube_angle_rad;
    if (tubeAngle <= 0.0 && config.enable_dynamic_rx_radius)
        tubeAngle = std::sqrt(4.0 * kPi / config.ray_count);
    const double radiusScale = config.ray_tube_radius_scale;
    const double rMin = std::max(baseRadius, config.ray_tube_min_radius_m);
    const double rMax = (config.ray_tube_max_radius_m > 0.0) ? config.ray_tube_max_radius_m : 1e6;

#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic, 1)
#endif
    for (int ri = 0; ri < N; ++ri) {
        int tid = 0;
#ifdef _OPENMP
        tid = omp_get_thread_num();
#endif
        GeometricPathSet& localResult = threadResults[tid];
        const Vec3& dir = directions[ri];

        TraceState root;
        root.curPt = txPoint; root.curDir = dir; root.curPwr = 1.0;
        root.cr = config.max_reflection_count; root.ct = config.max_transmission_count;
        root.depth = 0; root.lastFaceId = -1; root.currentMediumId = 0;
        root.cumulativeLength = 0.0;
        PathNode txNode; txNode.interaction_type = InteractionType::Tx;
        txNode.point = txPoint; txNode.direction = dir; txNode.valid = true;
        root.nodes.push_back(txNode);

        std::vector<TraceState> stack;
        stack.push_back(std::move(root));

        while (!stack.empty()) {
            TraceState ts = std::move(stack.back());
            stack.pop_back();

            // ── 1. 求交 ──
            Ray ray; ray.origin = ts.curPt; ray.direction = ts.curDir;
            FaceQueryContext fqc;
            fqc.ignored_face_id = ts.lastFaceId;
            fqc.origin_ignore_distance = originIgnoreDist;
            FaceHit hit = accelerator_->QueryClosestFaceHit(ray, fqc);
            Point3 segEnd = hit.hit ? hit.position : Add(ts.curPt, Scale(ts.curDir, FAR_DIST));

            // ── 2. Rx 检测 (含遮挡验证) ──
            double effRadius = baseRadius;
            if (config.enable_dynamic_rx_radius && tubeAngle > 0.0) {
                double d = ts.cumulativeLength + hit.distance;
                effRadius = std::max(baseRadius, d * tubeAngle * radiusScale);
                effRadius = Clamp(effRadius, rMin, rMax);
            }
            double sphereR2 = effRadius * effRadius;

            for (size_t rxi = 0; rxi < rxPoints.size(); ++rxi) {
                double d2 = PointToSegmentDistSq(rxPoints[rxi], ts.curPt, segEnd);
                if (d2 < sphereR2) {
                    Vec3 segVec = SubtractVec(segEnd, ts.curPt);
                    double segLen2 = LengthSq(segVec);
                    double tClosest = 0.0;
                    if (segLen2 > 0.0) {
                        tClosest = Dot(SubtractVec(rxPoints[rxi], ts.curPt), segVec) / segLen2;
                        if (tClosest < 0.0) tClosest = 0.0;
                        if (tClosest > 1.0) tClosest = 1.0;
                    }
                    Point3 closestPt = Add(ts.curPt, Scale(segVec, tClosest));

                    // ★ 遮挡验证: 如果命中面元, 确保 Rx 不在面元背面
                    if (hit.hit) {
                        double rxSide = Dot(SubtractVec(rxPoints[rxi], hit.position), hit.normal);
                        double viewSide = Dot(SubtractVec(ts.curPt, hit.position), hit.normal);
                        // 如果 Rx 在面的另一侧 (与来向异侧), 则被面元遮挡 → 跳过
                        if (rxSide * viewSide < 0.0) { diagOccluded++; continue; }
                    }
                    diagAccepted++;

                    double lenToClosest = tClosest * std::sqrt(segLen2);
                    GeometricPath gp;
                    gp.is_los = (ts.depth == 0); gp.nodes = ts.nodes;
                    PathNode rxNode; rxNode.interaction_type = InteractionType::Rx;
                    rxNode.point = rxPoints[rxi]; rxNode.incident_direction = ts.curDir;
                    rxNode.segment_length_from_previous = lenToClosest; rxNode.valid = true;
                    gp.nodes.push_back(rxNode);
                    gp.total_length = ts.cumulativeLength + lenToClosest;
                    gp.valid = true;
                    localResult.paths.push_back(gp);
                }
            }

            // ── 3. 终止 ──
            if (!hit.hit) continue;
            if (ts.curPwr <= pwrTh) continue;
            if (ts.depth >= maxDepth) continue;

            const Face& face = scene.faces[hit.face_id];

            // ── 4a. 反射 ──
            if (face.reflection_enabled && ts.cr > 0) {
                TraceState rs;
                rs.curPt = hit.position; rs.curDir = Reflect(ts.curDir, hit.normal);
                rs.curPwr = ts.curPwr; rs.cr = ts.cr - 1; rs.ct = ts.ct;
                rs.depth = ts.depth + 1; rs.lastFaceId = hit.face_id;
                rs.currentMediumId = ts.currentMediumId;
                rs.cumulativeLength = ts.cumulativeLength + hit.distance;
                rs.nodes = ts.nodes;
                PathNode rn; rn.interaction_type = InteractionType::Reflection;
                rn.point = hit.position; rn.face_id = hit.face_id; rn.object_id = hit.object_id;
                rn.surface_normal = hit.normal; rn.incident_direction = ts.curDir;
                rn.direction = rs.curDir; rn.segment_length_from_previous = hit.distance;
                rn.valid = true;
                rs.nodes.push_back(rn);
                stack.push_back(std::move(rs));
            }

            // ── 4b. 透射 ──
            if (face.transmission_enabled && ts.ct > 0) {
                // ★ 修复: 正确判断入射侧
                double dotDN = Dot(ts.curDir, hit.normal);
                bool fromFront = (dotDN < 0.0);

                double n1 = 1.0, n2 = 1.0;
                int newMediumId = ts.currentMediumId;
                if (hasMaterials) {
                    auto p1 = matDb.QueryByName(
                        fromFront ? face.front_material_name : face.back_material_name, freqHz);
                    auto p2 = matDb.QueryByName(
                        fromFront ? face.back_material_name : face.front_material_name, freqHz);
                    n1 = std::sqrt(std::max(1.0, p1.epsilon_r));
                    n2 = std::sqrt(std::max(1.0, p2.epsilon_r));
                    newMediumId = fromFront ? face.back_medium_id : face.front_medium_id;
                }
                SnellResult sr = SnellRefractV2(ts.curDir, hit.normal, n1, n2);
                if (sr.valid && !sr.total_internal_reflection) {
                    double R = FresnelPowerNormal(n1, n2);
                    TraceState ts_tx;
                    ts_tx.curPt = hit.position; ts_tx.curDir = sr.direction;
                    ts_tx.curPwr = ts.curPwr * (1.0 - R);
                    ts_tx.cr = ts.cr; ts_tx.ct = ts.ct - 1;
                    ts_tx.depth = ts.depth + 1; ts_tx.lastFaceId = hit.face_id;
                    ts_tx.currentMediumId = newMediumId;
                    ts_tx.cumulativeLength = ts.cumulativeLength + hit.distance;
                    ts_tx.nodes = ts.nodes;
                    PathNode tn; tn.interaction_type = InteractionType::Transmission;
                    tn.point = hit.position; tn.face_id = hit.face_id;
                    tn.object_id = hit.object_id; tn.surface_normal = hit.normal;
                    tn.incident_direction = ts.curDir; tn.direction = sr.direction;
                    tn.segment_length_from_previous = hit.distance;
                    tn.medium_in_id = ts.currentMediumId;
                    tn.medium_out_id = newMediumId;
                    tn.front_medium_id = face.front_medium_id;
                    tn.back_medium_id = face.back_medium_id;
                    tn.entered_from_front_side = fromFront;
                    tn.snell_residual = sr.residual;
                    tn.snell_theta_i_rad = sr.theta_i_rad;
                    tn.snell_theta_t_rad = sr.theta_t_rad;
                    tn.transmission_semantic_complete = true; tn.valid = true;
                    ts_tx.nodes.push_back(tn);
                    stack.push_back(std::move(ts_tx));
                }
            }

            // ── 4c. 绕射: 已解耦, 由 RunDiffraction 独立处理 (不再混合 R/T) ──
        }
    }

    // ── 合并 ──
    GeometricPathSet merged;
    for (auto& tr : threadResults)
        merged.paths.insert(merged.paths.end(),
            std::make_move_iterator(tr.paths.begin()),
            std::make_move_iterator(tr.paths.end()));
    if (diagOccluded > 0 || diagAccepted > 0)
        std::printf("[Diag] Rx hits: accepted=%lld occluded=%lld (%.1f%% blocked)\n",
            (long long)diagAccepted, (long long)diagOccluded,
            100.0 * diagOccluded / std::max(1LL, diagAccepted + diagOccluded));
    return PostProcess(merged, config);
}

SbrCoverageResult SbrEngine::RunCoverage(
    const Scene& scene, const MaterialDatabase& matDb, const SbrConfig& config,
    const Point3& txPoint, const std::vector<Point3>& rxGrid,
    const NumericToleranceConfig& tol) {
    SbrCoverageResult covResult;
    covResult.trace_profile = config.trace_profile;
    covResult.total_rays = config.ray_count;

    // ── 独立绕射寻径 (每Tx-Rx对) ──
    GeometricPathSet diffPaths = RunDiffraction(scene, matDb, config, txPoint, rxGrid, accelerator_.get());
    if (config.max_diffraction_count > 0)
        std::printf("[Diff] Generated %zu diffraction paths\n", diffPaths.paths.size());

    for (size_t i = 0; i < rxGrid.size(); ++i) {
        auto paths = RunPointToPoint(scene, matDb, config, txPoint, {rxGrid[i]}, tol);

        // 合并绕射路径 (匹配当前Rx)
        for (auto& dp : diffPaths.paths) {
            if (dp.nodes.size() >= 3) {
                auto& rxNode = dp.nodes.back();
                double dx = rxNode.point.x - rxGrid[i].x;
                double dy = rxNode.point.y - rxGrid[i].y;
                double dz = rxNode.point.z - rxGrid[i].z;
                if (dx*dx + dy*dy + dz*dz < 1e-6) {
                    paths.paths.push_back(dp);
                }
            }
        }

        RxCoverageRecord rec; rec.rx_position = rxGrid[i]; rec.rx_index = (int)i;
        rec.ray_hit_count = (int)paths.paths.size();
        double tp = 0.0;
        for (auto& gp : paths.paths) {
            tp += std::exp(PowerProxyScore(gp));
            if (config.store_paths) rec.paths.push_back(gp);
        }
        rec.total_power_linear = tp;
        rec.total_power_dBm = (tp > 0.0) ? config.tx_power_dBm + 10.0 * std::log10(tp) : -999.0;
        covResult.rx_records.push_back(rec);
    }
    covResult.active_rx_count = (int)rxGrid.size();
    covResult.succeeded = true;
    return covResult;
}

} // namespace sbr
