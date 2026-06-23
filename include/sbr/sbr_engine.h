// sbr_engine.h — SBR 引擎主入口 (Phase 0 骨架, Phase 1-4 实现)
#pragma once
#include "sbr_config.h"
#include "sbr_path.h"
#include "sbr_scene.h"
#include "sbr_material.h"
#include "sbr_accelerator.h"
#include <vector>
#include <memory>
#include <string>

namespace sbr {

// ── Rx 覆盖记录 ──
struct RxCoverageRecord {
    Point3 rx_position;
    int    rx_index            = -1;
    double total_power_linear  = 0.0;
    double total_power_dBm     = 0.0;
    int    ray_hit_count       = 0;
    std::vector<GeometricPath> paths;
};

// ── SBR 覆盖结果 ──
struct SbrCoverageResult {
    bool   succeeded       = false;
    std::string trace_profile = "Coverage";
    int    total_rays      = 0;
    int    active_rx_count = 0;
    std::vector<RxCoverageRecord> rx_records;
    std::vector<std::string> trace_lines;

    // 诊断字段 (Phase 1-4 逐步填充)
    int total_bounces = 0, total_transmissions = 0, total_diffractions = 0;
    int rays_below_threshold = 0, rays_terminated_early = 0;
    long long generated_reflection_branches   = 0;
    long long generated_transmission_branches = 0;
    long long rejected_tir_transmissions      = 0;
    long long pruned_power_branches           = 0;
    long long rx_paths_recorded               = 0;
    long long rx_paths_skipped_by_cap         = 0;
    long long rx_paths_skipped_by_rx_cap      = 0;
    long long rx_paths_deduplicated            = 0;
    int peak_active_rays = 0;
    bool dynamic_rx_radius_enabled = false;
    double ray_tube_angle_rad = 0.0;
    double max_effective_rx_radius_m = 0.0;
    long long dynamic_rx_queries = 0, dynamic_rx_hits = 0;
    bool wedge_tube_coupling_enabled = false;
    long long wedge_tube_queries = 0, wedge_tube_candidates = 0, wedge_tube_rejected = 0;
    long long wedge_edge_fallback_hits = 0;
    int diffraction_rays_per_event = 4;
    long long diffraction_events = 0;
    long long generated_diffraction_branches = 0;
    long long rejected_keller_diffractions = 0;
    bool path_dedup_enabled = true, path_similarity_pruning_enabled = true;
    int path_top_n_per_rx = 0;
    long long paths_pruned_by_post_dedup = 0;
    long long paths_pruned_by_similarity = 0;
    long long paths_pruned_by_top_n = 0;
    long long paths_after_postprocess = 0;
    bool path_residual_filter_enabled = false;
    long long paths_evaluated_for_residual = 0;
    long long paths_pruned_by_residual = 0;
    double max_path_geometry_residual = 0.0;
    std::string convergence_notes;
};

// ── SBR 引擎 ──
class SbrEngine {
public:
    explicit SbrEngine(std::unique_ptr<ISceneAccelerator> accelerator);

    /// P2P 模式: Tx → 每个 Rx 独立寻径
    GeometricPathSet RunPointToPoint(
        const Scene& scene,
        const MaterialDatabase& matDb,
        const SbrConfig& config,
        const Point3& txPoint,
        const std::vector<Point3>& rxPoints,
        const NumericToleranceConfig& tol);

    /// Coverage 模式: 大规模虚拟 Rx 网格功率覆盖
    SbrCoverageResult RunCoverage(
        const Scene& scene,
        const MaterialDatabase& matDb,
        const SbrConfig& config,
        const Point3& txPoint,
        const std::vector<Point3>& rxGrid,
        const NumericToleranceConfig& tol);

private:
    std::unique_ptr<ISceneAccelerator> accelerator_;
};

} // namespace sbr
