// sbr_scene_builder.h — 编程式场景构造器 (测试用)
#pragma once
#include "sbr_scene.h"
#include <string>

namespace sbr {

class SceneBuilder {
public:
    SceneBuilder& AddVertex(double x, double y, double z);
    SceneBuilder& AddFace(int v0, int v1, int v2,
                          const std::string& frontMat = "Air",
                          const std::string& backMat  = "Vacuum",
                          bool reflect = true, bool transmit = false,
                          bool diffraction = false);
    SceneBuilder& AddWedge(const Point3& start, const Point3& end,
                           int facePos, int faceNeg, double angleDeg);
    Scene Build();

private:
    Scene scene_;
    int next_face_id_  = 0;
    int next_edge_id_  = 0;
    int next_wedge_id_ = 0;
};

} // namespace sbr
