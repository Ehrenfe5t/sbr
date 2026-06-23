// sbr_scene.h — 场景相关类型 (Face, Edge, Wedge, Scene)
// 与 H2hRT rt::Face / rt::Edge / rt::Wedge / rt::Scene 布局兼容
#pragma once
#include "sbr_types.h"
#include <string>
#include <vector>
#include <cstdint>

namespace sbr {

// ── 面传播标志位 ──
enum FacePropagationFlags : std::uint32_t {
    FacePropagationNone              = 0,
    FacePropagationReflect           = 1u << 0,
    FacePropagationTransmit          = 1u << 1,
    FacePropagationDiffractionBoundary = 1u << 2,
    FacePropagationScatter           = 1u << 3,
    FacePropagationDualSideResolved  = 1u << 4
};

// ── 局部坐标基 ──
struct LocalFrame {
    Vec3 tangent;
    Vec3 bitangent;
    Vec3 normal;
    bool valid = false;
};

// ── 场景三角面元 ──
struct Face {
    int face_id        = -1;
    int object_id      = -1;
    int vertex_index0  = -1;
    int vertex_index1  = -1;
    int vertex_index2  = -1;
    int normal_index   = -1;

    Vec3 normal;
    Vec3 centroid;
    AABB bounds;
    double area      = 0.0;
    LocalFrame local_frame;

    // ── 物体语义 ──
    std::string object_name;
    std::string object_type;
    std::string surface_material_name;
    int surface_material_id = -1;
    std::string front_material_name;
    int front_material_id = -1;
    int front_medium_id   = -1;
    std::string back_material_name;
    int back_material_id = -1;
    int back_medium_id   = -1;
    double surface_eps_r = 1.0;    // 预索引缓存 (真源: MaterialDatabase)
    double surface_sigma = 0.0;
    std::string normal_rule_tag;

    bool dual_side_material_resolved    = false;
    bool reflection_enabled             = true;
    bool transmission_enabled           = false;
    bool diffraction_candidate_enabled  = false;
    bool transmission_semantic_complete = false;
    bool degenerate                     = false;

    std::uint32_t propagation_flags = FacePropagationNone;

    int adjacent_edge_id0 = -1;
    int adjacent_edge_id1 = -1;
    int adjacent_edge_id2 = -1;
};

// ── 场景拓扑边 ──
struct Edge {
    int edge_id       = -1;
    int vertex_index0 = -1;
    int vertex_index1 = -1;
    int face_id0      = -1;
    int face_id1      = -1;

    Vec3 start;
    Vec3 end;
    Vec3 direction;
    Vec3 midpoint;

    double length            = 0.0;
    double dihedral_angle_deg = 0.0;

    bool is_boundary     = false;
    bool is_non_manifold = false;
    bool is_coplanar     = false;
    bool supports_wedge  = false;
};

// ── 楔边凸性分类 ──
enum class WedgeConvexity {
    Unknown  = 0,
    Convex   = 1,
    Concave  = 2,
    Boundary = 3
};

// ── 楔边传播标志位 ──
enum WedgeFlags : std::uint32_t {
    WedgeFlagNone              = 0,
    WedgeFlagDiffractable      = 1u << 0,
    WedgeFlagNonManifoldSource = 1u << 1,
    WedgeFlagCoplanarRejected  = 1u << 2,
    WedgeFlagAngleFiltered     = 1u << 3
};

// ── 场景绕射楔边 ──
struct Wedge {
    int wedge_id        = -1;
    int source_edge_id  = -1;

    int positive_face_id = -1;
    int negative_face_id = -1;
    int zero_face_id     = -1;    // UTD φ/φ' 参考面

    Point3 center_point;
    Point3 segment_start;
    Point3 segment_end;
    Vec3   direction;

    double length            = 0.0;
    double wedge_angle_deg   = 0.0;  // 外角
    double dihedral_angle_deg = 0.0;

    std::string positive_material_name;
    std::string negative_material_name;

    bool diffractable            = false;
    bool from_non_manifold_source = false;
    bool valid_for_utd           = false;
    WedgeConvexity convexity     = WedgeConvexity::Unknown;
    std::uint32_t wedge_flags    = WedgeFlagNone;
    AABB bounds;
};

// ── 场景对象记录 ──
struct SceneObjectRecord {
    int object_id = -1;
    std::string object_name;
    std::vector<int> face_ids;
};

// ── 统一场景 ──
struct Scene {
    std::vector<Vec3>               vertices;
    std::vector<Vec3>               normals;
    std::vector<SceneObjectRecord>  objects;
    std::vector<Face>               faces;
    std::vector<Edge>               edges;
    std::vector<Wedge>              wedges;
};

} // namespace sbr
