// sbr_config.h — 配置类型 (与 H2hRT rt::SbrConfig 布局兼容)
#pragma once
#include <string>

namespace sbr {

// ── 数值容差配置 ──
struct NumericToleranceConfig {
    double eps_length              = 1.0e-6;
    double eps_angle               = 1.0e-6;
    double eps_intersection        = 1.0e-7;
    double eps_normal              = 1.0e-6;
    double eps_deduplicate         = 1.0e-5;
    double eps_power               = 1.0e-9;
    double self_hit_ignore_distance = 1.0e-5;
    double visibility_origin_offset = 1.0e-5;
    double visibility_target_shrink = 1.0e-5;
};

// ── SBR 寻径配置 ──
struct SbrConfig {
    // 基础参数
    bool   enabled              = false;
    std::string trace_profile   = "Coverage";   // "Coverage" | "FineChannel" | "DebugValidation"
    double center_frequency_hz  = 2.4e9;        // ★ Snell 折射率 n=√ε_r(f) 查询频率
    int    ray_count            = 10000;
    int    max_ray_depth        = 6;
    int    max_reflection_count = 6;
    int    max_transmission_count = 0;
    int    max_diffraction_count  = 0;
    double ray_power_threshold_dB = -60.0;

    // 接收参数
    double rx_sphere_radius_m   = 0.3;

    // Coverage 网格参数
    bool   auto_grid_bounds     = true;
    double rx_grid_min_x = -5.0, rx_grid_max_x = 5.0;
    double rx_grid_min_y = -5.0, rx_grid_max_y = 5.0;
    double rx_grid_min_z = 1.5,  rx_grid_max_z = 1.5;
    double rx_grid_step_x = 1.0, rx_grid_step_y = 1.0, rx_grid_step_z = 1.0;
    double tx_power_dBm = 0.0;
    bool   store_paths  = false;

    // 绕射参数
    double wedge_max_distance_m  = 5.0;
    int    wedge_max_candidates  = 8;

    // V10 增强参数
    bool   deterministic_interaction_split = false;
    bool   disable_no_new_hit_early_stop   = false;
    int    max_paths_per_ray  = 8;
    int    max_paths_per_rx   = 0;   // ≤0 = 不限制

    // 射线管参数
    bool   enable_dynamic_rx_radius = false;
    double ray_tube_angle_rad       = 0.0;   // ≤0 = 从 ray_count 自动估算
    double ray_tube_radius_scale    = 0.5;
    double ray_tube_min_radius_m    = 0.0;
    double ray_tube_max_radius_m    = 0.0;   // ≤0 = 无上限

    // 绕射增强参数
    bool   enable_wedge_tube_coupling  = false;
    double wedge_tube_radius_scale     = 1.0;
    int    diffraction_rays_per_event  = 4;

    // 后处理参数
    bool   enable_path_dedup              = true;
    bool   enable_path_similarity_pruning = true;
    double path_similarity_length_tol_m   = 0.05;
    int    path_top_n_per_rx              = 0;     // ≤0 = 禁用 top-N
    bool   enable_path_residual_filter    = false;
    double path_geometry_residual_tol     = 0.25;
    double reflection_residual_tol_m      = 0.25;
    double snell_residual_tol             = 1.0e-3;
    double keller_residual_tol            = 1.0e-3;
};

} // namespace sbr
