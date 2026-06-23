// sbr_diagnostics.h — 诊断计数器收集 (Phase 1-4 逐步填充)
#pragma once
#include <cstdint>

namespace sbr {

struct DiagnosticsCollector {
    // 反射/透射/绕射统计
    long long total_bounces        = 0;
    long long total_transmissions  = 0;
    long long total_diffractions   = 0;

    // 分支统计
    long long generated_reflection_branches   = 0;
    long long generated_transmission_branches = 0;
    long long generated_diffraction_branches  = 0;

    // 剪枝统计
    long long rejected_tir_transmissions  = 0;
    long long pruned_power_branches       = 0;

    // Rx 命中统计
    long long rx_paths_recorded           = 0;
    long long rx_paths_skipped_by_cap     = 0;
    long long rx_paths_skipped_by_rx_cap  = 0;
    long long rx_paths_deduplicated       = 0;

    // 绕射统计
    long long diffraction_events          = 0;
    long long rejected_keller_diffractions = 0;
    long long wedge_tube_queries          = 0;
    long long wedge_tube_candidates       = 0;
    long long wedge_tube_rejected         = 0;

    void Reset() {
        total_bounces = total_transmissions = total_diffractions = 0;
        generated_reflection_branches = generated_transmission_branches = 0;
        generated_diffraction_branches = 0;
        rejected_tir_transmissions = pruned_power_branches = 0;
        rx_paths_recorded = rx_paths_skipped_by_cap = rx_paths_skipped_by_rx_cap = 0;
        rx_paths_deduplicated = 0;
        diffraction_events = rejected_keller_diffractions = 0;
        wedge_tube_queries = wedge_tube_candidates = wedge_tube_rejected = 0;
    }
};

} // namespace sbr
