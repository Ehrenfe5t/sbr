// sbr_path.h — 几何路径类型 (与 H2hRT rt::GeometricPath / rt::PathNode 布局兼容)
#pragma once
#include "sbr_types.h"
#include <vector>
#include <string>
#include <cstdint>

namespace sbr {

// ── 交互类型枚举 ──
enum class InteractionType {
    None = 0,
    Tx,
    Rx,
    Los,
    Reflection,
    Transmission,
    Diffraction,
    Scattering
};

// ── 绕射诊断 (仅绕射节点有效) ──
struct DiffractionDiagnostics {
    double edge_parameter_t      = 0.0;  // Fermat 最优点在边上的参数 [0,1]
    double s1                    = 0.0;  // Tx → 绕射点距离
    double s2                    = 0.0;  // 绕射点 → Rx 距离
    double keller_residual       = 0.0;  // |cos(β_tx) - cos(β_rx)|
    bool   fermat_endpoint_warning = false;
    bool   visibility_from_source  = false;
    bool   visibility_to_rx        = false;
};

// ── 几何路径节点 ──
struct PathNode {
    InteractionType interaction_type = InteractionType::None;
    int object_id   = -1;
    int face_id     = -1;
    int wedge_id    = -1;

    // 透射语义 (仅透射节点)
    int medium_in_id   = -1;
    int medium_out_id  = -1;
    int front_medium_id  = -1;
    int back_medium_id   = -1;
    int front_material_id = -1;
    int back_material_id  = -1;
    bool entered_from_front_side       = true;
    bool transmission_semantic_complete = false;

    // 几何
    Point3 point;
    Vec3   direction;            // 出射方向 (从此节点指向下一节点)
    Vec3   incident_direction;   // 入射方向 (从上一节点指向此节点)
    Vec3   surface_normal;       // 交互面法向
    double segment_length_from_previous = 0.0;
    bool   valid = false;

    // 诊断
    double snell_residual      = 0.0;    // |n1·sinθ_i - n2·sinθ_t|
    double snell_theta_i_rad   = 0.0;
    double snell_theta_t_rad   = 0.0;
    bool   snell_tir           = false;  // 全内反射标志
    DiffractionDiagnostics diffraction_diag;
};

// ── 稳定几何路径 ──
struct GeometricPath {
    int    path_id              = -1;
    std::vector<PathNode> nodes;
    double total_length         = 0.0;
    bool   is_los               = false;
    bool   contains_transmission = false;
    std::uint64_t path_signature = 0;
    double geometry_residual     = 0.0;
    double reflection_residual_m = 0.0;
    double max_snell_residual    = 0.0;
    double max_keller_residual   = 0.0;
    std::string residual_reject_reason;
    bool   valid = false;
};

// ── 几何路径集合 ──
struct GeometricPathSet {
    std::vector<GeometricPath> paths;
};

} // namespace sbr
