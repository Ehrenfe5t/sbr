// sbr_scene_builder.cpp — SceneBuilder 实现
#include "sbr/sbr_scene_builder.h"
#include "sbr/sbr_math.h"
#include <algorithm>
#include <cmath>

namespace sbr {

SceneBuilder& SceneBuilder::AddVertex(double x, double y, double z) {
    scene_.vertices.push_back(MakeVec3(x, y, z));
    return *this;
}

SceneBuilder& SceneBuilder::AddFace(int v0, int v1, int v2,
                                     const std::string& frontMat,
                                     const std::string& backMat,
                                     bool reflect, bool transmit,
                                     bool diffraction) {
    Face face;
    face.face_id       = next_face_id_++;
    face.object_id     = 0;
    face.vertex_index0 = v0;
    face.vertex_index1 = v1;
    face.vertex_index2 = v2;

    // 计算法向 (右手定则: (v1-v0) × (v2-v0))
    Vec3 e1 = Subtract(scene_.vertices[v1], scene_.vertices[v0]);
    Vec3 e2 = Subtract(scene_.vertices[v2], scene_.vertices[v0]);
    face.normal = Normalize(Cross(e1, e2));

    // 计算重心
    face.centroid = Scale(
        AddVec(AddVec(scene_.vertices[v0], scene_.vertices[v1]), scene_.vertices[v2]),
        1.0 / 3.0);

    // 计算面积
    face.area = 0.5 * Length(Cross(e1, e2));

    // 计算 AABB
    face.bounds.min = MakeVec3(
        std::min({scene_.vertices[v0].x, scene_.vertices[v1].x, scene_.vertices[v2].x}),
        std::min({scene_.vertices[v0].y, scene_.vertices[v1].y, scene_.vertices[v2].y}),
        std::min({scene_.vertices[v0].z, scene_.vertices[v1].z, scene_.vertices[v2].z}));
    face.bounds.max = MakeVec3(
        std::max({scene_.vertices[v0].x, scene_.vertices[v1].x, scene_.vertices[v2].x}),
        std::max({scene_.vertices[v0].y, scene_.vertices[v1].y, scene_.vertices[v2].y}),
        std::max({scene_.vertices[v0].z, scene_.vertices[v1].z, scene_.vertices[v2].z}));
    face.bounds.valid = true;

    // 材质
    face.front_material_name = frontMat;
    face.back_material_name  = backMat;
    face.surface_material_name = (frontMat == "Air") ? backMat : frontMat;

    // 传播能力
    face.reflection_enabled            = reflect;
    face.transmission_enabled          = transmit;
    face.diffraction_candidate_enabled = diffraction;

    // 设置传播标志位
    face.propagation_flags = FacePropagationNone;
    if (reflect)     face.propagation_flags |= FacePropagationReflect;
    if (transmit)    face.propagation_flags |= FacePropagationTransmit;
    if (diffraction) face.propagation_flags |= FacePropagationDiffractionBoundary;

    scene_.faces.push_back(face);
    return *this;
}

SceneBuilder& SceneBuilder::AddWedge(const Point3& start, const Point3& end,
                                      int facePos, int faceNeg, double angleDeg) {
    Wedge wedge;
    wedge.wedge_id         = next_wedge_id_++;
    wedge.positive_face_id = facePos;
    wedge.negative_face_id = faceNeg;
    wedge.segment_start    = start;
    wedge.segment_end      = end;
    wedge.direction        = Normalize(Subtract(end, start));
    wedge.length           = Length(Subtract(end, start));
    wedge.center_point     = Scale(AddVec(start, end), 0.5);
    wedge.wedge_angle_deg  = angleDeg;
    wedge.diffractable     = true;
    wedge.valid_for_utd    = (angleDeg > 1.0 && angleDeg < 359.0);

    // AABB
    wedge.bounds.min = MakeVec3(
        std::min(start.x, end.x), std::min(start.y, end.y), std::min(start.z, end.z));
    wedge.bounds.max = MakeVec3(
        std::max(start.x, end.x), std::max(start.y, end.y), std::max(start.z, end.z));
    wedge.bounds.valid = true;

    scene_.wedges.push_back(wedge);
    return *this;
}

Scene SceneBuilder::Build() {
    return std::move(scene_);
}

} // namespace sbr
