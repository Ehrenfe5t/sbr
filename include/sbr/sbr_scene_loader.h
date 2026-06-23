// sbr_scene_loader.h — 场景加载: OBJ + 材质JSON + 拓扑构建
#pragma once
#include "sbr_scene.h"
#include "sbr_material.h"
#include <string>

namespace sbr {

// 从 OBJ 文件加载顶点和面元
bool LoadOBJ(const std::string& path, Scene& scene);

// 从 material_map JSON 加载材质绑定, 应用到 Scene.faces
bool LoadMaterialMap(const std::string& path, Scene& scene,
                     MaterialDatabase& matDb, double freqHz);

// 构建边拓扑和楔边: 检测共享边 + 二面角 → 生成 Edge + Wedge
void BuildEdgeWedgeTopology(Scene& scene, double wedgeAngleMinDeg = 30.0);

} // namespace sbr
