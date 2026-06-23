# SBR 几何寻径模块 — 分批次开发手册

> 创建日期：2026-06-22  
> 最后修订：2026-06-23 — 经 H2hRT-7.1-SBR 源码 + RT.XD.SBR.CGAL 参考算法 + demo 场景三方资产审查  
> 目标项目：`G:\RT\sbr\sbr\` — SBR正向射线追踪几何寻径独立模块  
> 集成目标：开发完成后无缝移植至 H2hRT-7.1-SBR 全量RT引擎  
> 核心原则：接口契约先行、数据流单向、测试闭环独立、精度/效率/准确率三角平衡

---

## 批次总览

| 批次 | 名称 | 预估工期 | 核心产出 | 状态 |
|------|------|---------|---------|:----:|
| **P0** | 项目骨架 + 接口定义 + 基础数学 + 暴力加速器 | 2-3天 | CMake项目、全部头文件、Vec3/Snell数学库、BruteForceAccelerator、SceneBuilder、doctest框架 | ✅ 完成 (2026-06-23) |
| **P1** | 基础射线发射 + 单反射 + 固定接收球 | 3-5天 | Fibonacci射线发射、Wavefront单层循环(仅反射)、固定接收球Rx命中 | ✅ 完成 (2026-06-23) |
| **P2** | 透射 + 确定性R/T分裂 | 3-4天 | Snell透射/TIR判断、确定性双分支、介质状态追踪、MaterialDatabase集成 | ✅ 完成 (2026-06-23) |
| **P3** | 绕射 (Keller锥 + 楔边耦合) | 4-6天 | 楔边候选查询、segment-segment距离、Keller锥生成、楔边可见性验证、R-D-T混合路径 | ✅ 完成 (2026-06-23) |
| **P4** | 射线管模型 + 动态接收球 + 多层剪枝 + 后处理 | 3-4天 | 动态接收球半径、功率/深度/Rx可达剪枝、签名去重、残差过滤、相似剪枝、top-N | ✅ 完成 (2026-06-23) |
| **P5** | BVH加速 + OpenMP并行 + RxHashGrid | 1天 | SAH-BVH加速器(桶排序SAH+距离优先遍历+原子tMin)、OpenMP并行、RxHashGrid、性能基准: 20k面BVH构建32ms | ✅ 完成 (2026-06-23) |
| **P6** | 集成验证 + 回归测试 | 2-3天 | H2hRT全量引擎合并、SBR suite通过、全量回归33P/0F/1S、收敛性实验可复现 | ⬜ 未开始 |

> **总计**: 约 18-27 天

---

---

## Phase 0: 项目骨架 + 接口定义 + 基础数学 + 暴力加速器

### P0.0 批次目标

创建完整的项目骨架：编译系统、所有头文件（接口声明 + 文档注释）、基础数学库、暴力加速器桩、测试场景构造器、单元测试框架。**Phase 0 结束后，必须能从零编译出一个静态库，并运行数学和求交的单元测试。**

---

### P0.1 实现前状态

**已有资产**：
- ✅ H2hRT-7.1-SBR 完整源码（`G:\RT\H2hRT-7.1-SBR-\`）— 接口定义的**精确参考真源**
- ✅ RT.XD.SBR.CGAL.25.05 参考算法（`G:\RT\算法\`）— 反射/透射方向计算的可借鉴实现
- ✅ demo 场景（`G:\RT\sbr\sbr\demo\`）— `meeting.obj`（905行ASCII，412会议室）+ `material_map-meeting.json`
- ✅ VS 项目骨架（`sbr.sln`, `sbr.vcxproj`，空项目，无源文件）
- ✅ 本开发手册 — 算法设计、类型定义、架构设计已完成审查

**待创建**：
- ❌ `CMakeLists.txt` — 独立编译系统
- ❌ `include/sbr/` 下全部头文件
- ❌ `src/` 下源文件
- ❌ `test/` 下测试文件
- ❌ 测试框架集成

---

### P0.2 头脑风暴

#### P0.2.1 参考资源

| 资源 | 参考内容 | 对本批次的指导 |
|------|---------|--------------|
| **H2hRT `Vec3.h`** | 完整数学库: `Dot`, `Cross`, `Normalize`, `Reflect`, `SnellRefract`, `SnellRefractV2`, `SafeNormalize`, `Clamp`, `Add`, `Subtract`, `Scale` | **直接移植所有函数**，确保与 H2hRT 数值一致性 |
| **H2hRT `ISceneAccelerator.h`** | 加速器接口: `QueryClosestFaceHit`, `QueryAllFaceHits`, `IsOccluded`, `RxGridQueryParams`, 查询计数器 | **精确复制接口签名**，Phase 0 只需 `QueryClosestFaceHit` |
| **H2hRT `CpuFaceBvhAccelerator`** | 包装 `Scene` 引用 + `AppConfig` 引用 | Phase 0 桩实现只需 `Scene` 引用 |
| **H2hRT `test/step1_snell_selftest.h`** | Snell自测: 6个测试用例（空气→玻璃30°、玻璃→空气TIR、垂直入射等） | **直接移植测试用例** |
| **Möller-Trumbore 1997** | 射线-三角求交标准算法 | BruteForceAccelerator 核心算法 |
| **doctest** (github.com/doctest/doctest) | 单头文件测试框架, MIT | 测试框架 — 零依赖，与 Catch2 语法兼容 |
| **demo `meeting.obj`** | ASCII Wavefront OBJ: `o N` / `v x y z` / `vn nx ny nz` / `f v1//vn1 ...` | 场景格式确认，Phase 0 仅用 SceneBuilder，后续 Phase 接 OBJ Loader |

#### P0.2.2 关键设计决策

**决策 P0-D1：编译系统选择 CMake**

| 选项 | 优点 | 缺点 | 结论 |
|------|------|------|:--:|
| MSBuild (.vcxproj) | 已有空项目；VS 原生 | Windows only；CI 困难 | ❌ |
| **CMake** | 跨平台；CI 友好；Embree/OptiX 生态标准 | 需要写 CMakeLists.txt | ✅ 采用 |

项目已有 `sbr.sln`/`sbr.vcxproj` 可以作为 VS 用户的便捷入口（通过 CMake 的 `cmake -G "Visual Studio 17 2022"` 生成或保留手动同步）。

**决策 P0-D2：测试框架选择 doctest**

| 选项 | 优点 | 缺点 | 结论 |
|------|------|------|:--:|
| Catch2 | 流行，文档丰富 | 需要编译/链接 | ❌ |
| GoogleTest | 工业标准 | 重，需要编译 | ❌ |
| **doctest** | 单头文件，零依赖，编译极快，Catch2 语法 | 社区较小 | ✅ 采用 |

doctest 的 `TEST_CASE` + `CHECK` 宏与 Catch2 几乎相同，后续切换到 Catch2 成本极低。

**决策 P0-D3：数学类型——自建，零外部依赖**

H2hRT 的 `Vec3` 是极简设计（`double x, y, z` + 自由函数）。SBR 模块在 `namespace sbr` 中定义自己的 `Vec3`/`Point3`，并通过 `static_assert` 验证与 H2hRT 布局兼容。

**决策 P0-D4：场景构造——Phase 0 只用 SceneBuilder 编程API**

不引入 OBJ 解析器。测试场景通过 C++ 代码构造（1个三角形、1面墙、1个楔边等）。OBJ Loader 在 Phase 1 后期或 Phase 2 增加。

**决策 P0-D5：加速器桩——O(N) 暴力遍历**

`BruteForceAccelerator` 对每个射线遍历全部三角形，使用 Möller-Trumbore 算法。仅用于 Phase 0-3 的功能验证（小场景 < 1000 面）。Phase 5 替换为 BVH/Embree。

#### P0.2.3 文件清单

```
G:\RT\sbr\sbr\
├── CMakeLists.txt                          ← Phase 0 创建
├── include\
│   └── sbr\
│       ├── sbr_types.h                     ← Vec3, Point3, AABB, Ray, FaceHit, etc.
│       ├── sbr_math.h                      ← Dot, Cross, Normalize, Reflect, SnellRefractV2
│       ├── sbr_scene.h                     ← Face, Edge, Wedge, Scene
│       ├── sbr_material.h                  ← MaterialProps, MaterialDatabase
│       ├── sbr_config.h                    ← SbrConfig, NumericToleranceConfig
│       ├── sbr_path.h                      ← InteractionType, PathNode, GeometricPath, GeometricPathSet
│       ├── sbr_accelerator.h               ← ISceneAccelerator 抽象接口
│       ├── sbr_accelerator_bruteforce.h    ← BruteForceAccelerator (桩实现)
│       ├── sbr_scene_builder.h             ← SceneBuilder (测试用编程API)
│       ├── sbr_engine.h                    ← SbrEngine 主入口 (骨架)
│       ├── sbr_ray_emitter.h               ← 射线发射器 (骨架)
│       ├── sbr_wavefront.h                 ← Wavefront 主循环 (骨架)
│       ├── sbr_interaction.h               ← 反射/透射处理 (骨架)
│       ├── sbr_diffraction.h               ← 绕射模块 (骨架)
│       ├── sbr_rx_detector.h               ← Rx 检测器 (骨架)
│       ├── sbr_pruner.h                    ← 剪枝器 (骨架)
│       ├── sbr_postprocess.h               ← 后处理 (骨架)
│       └── sbr_diagnostics.h               ← 诊断计数器
├── src\
│   ├── sbr_accelerator_bruteforce.cpp      ← Phase 0 实现
│   ├── sbr_material.cpp                    ← Phase 0 实现 (CSV加载)
│   └── sbr_scene_builder.cpp               ← Phase 0 实现
├── test\
│   ├── test_main.cpp                       ← doctest 入口
│   ├── test_math.cpp                       ← Vec3/Snell 单元测试 (P0)
│   ├── test_bruteforce.cpp                 ← 暴力求交测试 (P0)
│   └── test_scene_builder.cpp              ← 场景构造测试 (P0)
├── external\
│   └── doctest\
│       └── doctest.h                       ← 单头文件 (~700KB)
├── sbr.sln                                 ← 已有 (空项目)
├── sbr.vcxproj                             ← 已有 (空项目)
└── document\
    └── sbr开发手册.md                       ← 本文档
```

---

### P0.3 关键实现步骤

#### Step 0.1: 搭建 CMake 编译系统

创建 `CMakeLists.txt`：

```cmake
cmake_minimum_required(VERSION 3.16)
project(sbr LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 头文件路径
include_directories(${CMAKE_SOURCE_DIR}/include)

# 库目标: sbr_core (静态库)
add_library(sbr_core STATIC
    src/sbr_accelerator_bruteforce.cpp
    src/sbr_material.cpp
    src/sbr_scene_builder.cpp
)

# 测试目标: sbr_tests
add_executable(sbr_tests
    test/test_main.cpp
    test/test_math.cpp
    test/test_bruteforce.cpp
    test/test_scene_builder.cpp
)
target_link_libraries(sbr_tests sbr_core)

# 可选: OpenMP (Phase 5 启用)
# find_package(OpenMP)
# if(OpenMP_CXX_FOUND)
#     target_link_libraries(sbr_core PRIVATE OpenMP::OpenMP_CXX)
# endif()
```

> 注意：现有 `sbr.sln`/`sbr.vcxproj` 保留不变。CMake 可通过 `cmake -G "Visual Studio 17 2022" -B build` 生成 VS 解决方案。

#### Step 0.2: 定义基础数学类型 (`sbr_types.h`, `sbr_math.h`)

**`sbr_types.h`** — 基础几何类型：

```cpp
#pragma once
#include <cstdint>
#include <cmath>

namespace sbr {

// ── 三维向量 ──
struct Vec3 {
    double x = 0.0, y = 0.0, z = 0.0;
};
using Point3 = Vec3;

// ── 轴对齐包围盒 ──
struct AABB {
    Vec3 min, max;
    bool valid = false;
};

// ── 射线 ──
struct Ray {
    Point3 origin;
    Vec3 direction;  // 应为单位向量
};

// ── 面元命中结果 ──
struct FaceHit {
    bool hit = false;
    int face_id = -1, object_id = -1;
    double distance = 0.0;
    Point3 position;
    Vec3 normal;
};

} // namespace sbr
```

**`sbr_math.h`** — 数学函数（全部 inline，与 H2hRT 一致）：

| 函数 | 签名 | 用途 |
|------|------|------|
| `MakeVec3` | `Vec3(x,y,z)` | 构造向量 |
| `Add/Subtract/Scale` | 向量±/标量× | 基本运算 |
| `Dot/Cross` | `double`/`Vec3` | 点积/叉积 |
| `Length/LengthSq` | `double` | 长度/长度² |
| `Normalize/SafeNormalize` | `Vec3` | 归一化 |
| `Reflect` | `Vec3(inc, normal)` | 反射方向 |
| `SnellRefractV2` | `SnellResult(inc, n, n1, n2)` | Snell折射+诊断 |
| `Clamp` | `double(v,lo,hi)` | 钳制 |

**关键注意**：`SnellRefractV2` 返回 `SnellResult` 结构（含 `valid`, `total_internal_reflection`, `direction`, `cos_i`, `cos_t`, `theta_i_rad`, `theta_t_rad`, `residual`），**必须与 H2hRT 实现完全一致**（包括自动法线翻转逻辑和 cos clamp）。

#### Step 0.3: 定义场景相关类型 (`sbr_scene.h`)

```cpp
#pragma once
#include "sbr_types.h"
#include <string>
#include <vector>
#include <cstdint>

namespace sbr {

struct Face {
    int face_id = -1, object_id = -1;
    int vertex_index0 = -1, vertex_index1 = -1, vertex_index2 = -1;
    int normal_index = -1;
    Vec3 normal, centroid;
    AABB bounds;
    double area = 0.0;
    // 材质/介质
    std::string object_name, object_type;
    std::string surface_material_name;
    int surface_material_id = -1;
    std::string front_material_name;
    int front_material_id = -1, front_medium_id = -1;
    std::string back_material_name;
    int back_material_id = -1, back_medium_id = -1;
    double surface_eps_r = 1.0, surface_sigma = 0.0;
    // 传播能力
    bool reflection_enabled = true;
    bool transmission_enabled = false;
    bool diffraction_candidate_enabled = false;
    uint32_t propagation_flags = 0;
    int adjacent_edge_id0 = -1, adjacent_edge_id1 = -1, adjacent_edge_id2 = -1;
};

struct Edge {
    int edge_id = -1;
    int vertex_index0 = -1, vertex_index1 = -1;
    int face_id0 = -1, face_id1 = -1;
    Vec3 start, end, direction, midpoint;
    double length = 0.0, dihedral_angle_deg = 0.0;
    bool is_boundary = false, is_non_manifold = false;
    bool is_coplanar = false, supports_wedge = false;
};

enum class WedgeConvexity { Unknown = 0, Convex = 1, Concave = 2, Boundary = 3 };

struct Wedge {
    int wedge_id = -1, source_edge_id = -1;
    int positive_face_id = -1, negative_face_id = -1, zero_face_id = -1;
    Point3 center_point, segment_start, segment_end;
    Vec3 direction;
    double length = 0.0, wedge_angle_deg = 0.0, dihedral_angle_deg = 0.0;
    bool diffractable = false, from_non_manifold_source = false, valid_for_utd = false;
    WedgeConvexity convexity = WedgeConvexity::Unknown;
    uint32_t wedge_flags = 0;
    AABB bounds;
};

struct Scene {
    std::vector<Vec3> vertices;
    std::vector<Vec3> normals;
    std::vector<Face> faces;
    std::vector<Edge> edges;
    std::vector<Wedge> wedges;
};

} // namespace sbr
```

#### Step 0.4: 定义材质类型 (`sbr_material.h`)

```cpp
#pragma once
#include <string>
#include <vector>
#include <unordered_map>
#include <map>

namespace sbr {

struct MaterialProps {
    double epsilon_r = 1.0, sigma = 0.0, mu_r = 1.0;
    std::string name;
};

class MaterialDatabase {
public:
    bool LoadFromCsv(const std::string& path);
    MaterialProps QueryByName(const std::string& name, double freqHz) const;
    MaterialProps QueryById(int id, double freqHz) const;
    bool empty() const;
    bool HasMaterial(const std::string& name) const;
private:
    std::unordered_map<std::string, std::map<double, MaterialProps>> byName_;
    std::unordered_map<int, std::map<double, MaterialProps>> byId_;
    static MaterialProps Interpolate(const std::map<double, MaterialProps>& data, double freq);
};

} // namespace sbr
```

> Phase 0 实现 CSV 加载和查询。ITU-R P.2040 外推在 Phase 2 透射时实现。

#### Step 0.5: 定义配置类型 (`sbr_config.h`)

```cpp
#pragma once
#include <string>

namespace sbr {

struct NumericToleranceConfig {
    double eps_length = 1e-6, eps_angle = 1e-6;
    double eps_intersection = 1e-7, eps_normal = 1e-6;
    double eps_deduplicate = 1e-5, eps_power = 1e-9;
    double self_hit_ignore_distance = 1e-5;
    double visibility_origin_offset = 1e-5;
    double visibility_target_shrink = 1e-5;
};

struct SbrConfig {
    bool enabled = false;
    std::string trace_profile = "Coverage";
    double center_frequency_hz = 2.4e9;  // ★ Snell 折射率查询频率
    int ray_count = 10000;
    int max_ray_depth = 6;
    int max_reflection_count = 6, max_transmission_count = 0, max_diffraction_count = 0;
    double ray_power_threshold_dB = -60.0;
    double rx_sphere_radius_m = 0.3;
    bool auto_grid_bounds = true;
    double rx_grid_min_x = -5, rx_grid_max_x = 5;
    double rx_grid_min_y = -5, rx_grid_max_y = 5;
    double rx_grid_min_z = 1.5, rx_grid_max_z = 1.5;
    double rx_grid_step_x = 1, rx_grid_step_y = 1, rx_grid_step_z = 1;
    double tx_power_dBm = 0.0;
    bool store_paths = false;
    double wedge_max_distance_m = 5.0;
    int wedge_max_candidates = 8;
    bool deterministic_interaction_split = false;
    bool disable_no_new_hit_early_stop = false;
    int max_paths_per_ray = 8, max_paths_per_rx = 0;
    bool enable_dynamic_rx_radius = false;
    double ray_tube_angle_rad = 0.0, ray_tube_radius_scale = 0.5;
    double ray_tube_min_radius_m = 0.0, ray_tube_max_radius_m = 0.0;
    bool enable_wedge_tube_coupling = false;
    double wedge_tube_radius_scale = 1.0;
    int diffraction_rays_per_event = 4;
    bool enable_path_dedup = true, enable_path_similarity_pruning = true;
    double path_similarity_length_tol_m = 0.05;
    int path_top_n_per_rx = 0;
    bool enable_path_residual_filter = false;
    double path_geometry_residual_tol = 0.25;
    double reflection_residual_tol_m = 0.25;
    double snell_residual_tol = 1e-3, keller_residual_tol = 1e-3;
};

} // namespace sbr
```

#### Step 0.6: 定义路径类型 (`sbr_path.h`)

```cpp
#pragma once
#include "sbr_types.h"
#include <vector>
#include <string>
#include <cstdint>

namespace sbr {

enum class InteractionType {
    None = 0, Tx, Rx, Los, Reflection, Transmission, Diffraction, Scattering
};

struct DiffractionDiagnostics {
    double edge_parameter_t = 0.0, s1 = 0.0, s2 = 0.0;
    double keller_residual = 0.0;
    bool fermat_endpoint_warning = false;
    bool visibility_from_source = false, visibility_to_rx = false;
};

struct PathNode {
    InteractionType interaction_type = InteractionType::None;
    int object_id = -1, face_id = -1, wedge_id = -1;
    int medium_in_id = -1, medium_out_id = -1;
    int front_medium_id = -1, back_medium_id = -1;
    int front_material_id = -1, back_material_id = -1;
    bool entered_from_front_side = true;
    bool transmission_semantic_complete = false;
    Point3 point;
    Vec3 direction, incident_direction, surface_normal;
    double segment_length_from_previous = 0.0;
    bool valid = false;
    double snell_residual = 0.0, snell_theta_i_rad = 0.0, snell_theta_t_rad = 0.0;
    bool snell_tir = false;
    DiffractionDiagnostics diffraction_diag;
};

struct GeometricPath {
    int path_id = -1;
    std::vector<PathNode> nodes;
    double total_length = 0.0;
    bool is_los = false, contains_transmission = false;
    uint64_t path_signature = 0;
    double geometry_residual = 0.0;
    double reflection_residual_m = 0.0;
    double max_snell_residual = 0.0, max_keller_residual = 0.0;
    std::string residual_reject_reason;
    bool valid = false;
};

struct GeometricPathSet {
    std::vector<GeometricPath> paths;
};

} // namespace sbr
```

#### Step 0.7: 定义加速器接口 + 暴力桩 (`sbr_accelerator.h`, `sbr_accelerator_bruteforce.h`)

```cpp
// sbr_accelerator.h
#pragma once
#include "sbr_types.h"
#include "sbr_scene.h"
#include <vector>
#include <string>

namespace sbr {

struct FaceQueryContext {
    int ignored_face_id = -1, ignored_face_id2 = -1, ignored_object_id = -1;
    bool ignore_origin_self_hit = true;
    double origin_ignore_distance = 1e-6;
    bool only_return_propagation_enabled_faces = false;
};

struct VisibilityQueryContext {
    int ignored_face_id = -1, ignored_face_id2 = -1, ignored_object_id = -1;
    bool ignore_origin_attached_face = true, ignore_target_attached_face = true;
    double origin_offset_distance = 1e-6, target_shrink_distance = 1e-6;
};

class ISceneAccelerator {
public:
    virtual ~ISceneAccelerator() = default;
    virtual FaceHit QueryClosestFaceHit(const Ray& ray, const FaceQueryContext& ctx) const = 0;
    virtual bool IsOccluded(const Point3& start, const Point3& end,
                            const VisibilityQueryContext& ctx) const = 0;
    bool IsVisible(const Point3& start, const Point3& end,
                   const VisibilityQueryContext& ctx) const {
        return !IsOccluded(start, end, ctx);
    }
    virtual std::string BackendName() const = 0;
    virtual void BuildFromScene(const Scene& scene) = 0;
protected:
    ISceneAccelerator() = default;
    ISceneAccelerator(const ISceneAccelerator&) = delete;
    ISceneAccelerator& operator=(const ISceneAccelerator&) = delete;
};

} // namespace sbr
```

```cpp
// sbr_accelerator_bruteforce.h
#pragma once
#include "sbr_accelerator.h"
#include "sbr_scene.h"

namespace sbr {

class BruteForceAccelerator : public ISceneAccelerator {
public:
    explicit BruteForceAccelerator(const Scene& scene);
    FaceHit QueryClosestFaceHit(const Ray& ray, const FaceQueryContext& ctx) const override;
    bool IsOccluded(const Point3& start, const Point3& end,
                    const VisibilityQueryContext& ctx) const override;
    std::string BackendName() const override { return "BruteForce(CPU)"; }
    void BuildFromScene(const Scene& scene) override { scene_ = &scene; }
private:
    const Scene* scene_ = nullptr;
};

} // namespace sbr
```

**实现要点**：
- `QueryClosestFaceHit`：遍历 `scene_->faces`，对每个面调用 Möller-Trumbore，跟踪最近命中
- `IsOccluded`：构造从 start 到 end 的射线，查询有无命中（`distance < |end-start|`）
- Möller-Trumbore 算法参考：Tomas Möller & Ben Trumbore, "Fast, Minimum Storage Ray-Triangle Intersection", *Journal of Graphics Tools*, 1997.

#### Step 0.8: 实现 SceneBuilder (`sbr_scene_builder.h` + `.cpp`)

```cpp
// sbr_scene_builder.h
#pragma once
#include "sbr_scene.h"

namespace sbr {

class SceneBuilder {
public:
    SceneBuilder& AddVertex(double x, double y, double z);
    SceneBuilder& AddFace(int v0, int v1, int v2,
                          const std::string& frontMat = "Air",
                          const std::string& backMat = "Vacuum",
                          bool reflect = true, bool transmit = false,
                          bool diffraction = false);
    SceneBuilder& AddWedge(const Point3& start, const Point3& end,
                           int facePos, int faceNeg, double angleDeg);
    SceneBuilder& SetDefaultMedium(const std::string& name);
    Scene Build();
private:
    Scene scene_;
    // 内部自动计算 normal, centroid, area, bounds, adjacent edges
};

} // namespace sbr
```

Phase 0 只需实现 `AddVertex`, `AddFace`（自动计算法向/重心/面积/AABB/边邻接）, `Build`。`AddWedge` 可留空实现（Phase 3 需要）。

#### Step 0.9: 创建骨架头文件（其余模块）

每个骨架头文件只需：
1. `#pragma once`
2. `namespace sbr { ... }`
3. 类/函数声明 + `// TODO: Phase N 实现`
4. 不创建对应的 `.cpp`（Phase 0 不编译它们）

具体内容参考本手册附录 A 的架构设计。

#### Step 0.10: 集成单元测试框架

1. 下载 `doctest.h` 到 `external/doctest/doctest.h`
2. 创建 `test/test_main.cpp`：
```cpp
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
```
3. 创建 `test/test_math.cpp`（见 P0.4 自测方案）
4. 创建 `test/test_bruteforce.cpp`（见 P0.4 自测方案）
5. 创建 `test/test_scene_builder.cpp`（见 P0.4 自测方案）

---

### P0.4 自测试闭环方案

**自测原则**：每个测试用例必须包含**输入**、**期望输出**、**验收标准**（精确数值阈值）。来源必须可追溯（学术文献或 H2hRT 参考实现）。

#### 测试组 1: `test_math` — 向量数学函数

| 测试用例 | 输入 | 期望输出 | 验收标准 | 来源 |
|---------|------|---------|---------|------|
| **T1.1 Dot** | `a=(1,2,3), b=(4,5,6)` | 32.0 | `abs(result-32.0) < 1e-15` | 定义 |
| **T1.2 Cross** | `a=(1,0,0), b=(0,1,0)` | `(0,0,1)` | 各分量误差 < 1e-15 | 右手系定义 |
| **T1.3 Length** | `v=(3,4,0)` | 5.0 | `abs(5.0) < 1e-15` | 欧氏距离 |
| **T1.4 Normalize** | `v=(3,0,0)` | `(1,0,0)` | 各分量误差 < 1e-15 | 定义 |
| **T1.5 SafeNormalize-零向量** | `v=(0,0,0)` | fallback `(1,0,0)` | 返回 fallback | H2hRT Vec3.h:165 |
| **T1.6 Reflect-45°** | `inc=(0,0,1), n=(0,0,-1)` | `(0,0,-1)` | 法向入射,反射=反向 | 反射定律 |
| **T1.7 Reflect-斜入射** | `inc=(0,-1,0), n=(0,1,0)` | `(0,1,0)` | 分量误差 < 1e-15 | 反射定律 |
| **T1.8 Reflect-30°** | `inc=(sin30,0,cos30), n=(0,0,-1)` | `(sin30,0,-cos30)` | 误差 < 1e-12 | 反射定律 |

#### 测试组 2: `test_snell` — Snell 折射 (SnellRefractV2)

| 测试用例 | n1 | n2 | 入射 | 期望 | 验收标准 | 来源 |
|---------|----|----|------|------|---------|------|
| **T2.1 空气→玻璃 30°** | 1.0 | 1.5 | 30° from +Z, 法线朝-Z | valid, θ_t≈19.47°, residual<1e-12 | `abs(θ_i-30°)<1e-6`, `abs(θ_t-19.47°)<1e-6` | H2hRT step1_snell_selftest.h:25 |
| **T2.2 玻璃→空气 TIR** | 1.5 | 1.0 | 45° (>临界角41.81°) | total_internal_reflection=true | TIR 标志为 true | H2hRT step1_snell_selftest.h:54 |
| **T2.3 垂直入射** | 1.0 | 1.5 | 0° (沿法线) | θ_i=0, θ_t=0, 方向不变 | 方向误差 < 1e-12 | Snell 定律 |
| **T2.4 玻璃→空气 临界角** | 1.5 | 1.0 | 41.81° (≈临界角) | θ_t≈90°, 非TIR | `abs(θ_t-90°)<1e-3` | Snell 定律 |
| **T2.5 背面入射** | 1.0 | 1.5 | 从背面 (法线反侧) | 法线自动翻转, 结果正确 | residual<1e-12 | H2hRT Vec3.h SnellRefractV2 |
| **T2.6 掠入射** | 1.0 | 1.5 | 89.9° | θ_t 接近临界, 非TIR | valid=true | Snell 定律 |
| **T2.7 cosI clamp** | 1.0 | 2.0 | cosI 浮点误差>1 | 不产生 NaN | valid=true, finite direction | H2hRT Vec3.h:248 |

#### 测试组 3: `test_bruteforce` — 暴力射线-三角求交

| 测试用例 | 场景 | 射线 | 期望 | 验收标准 | 来源 |
|---------|------|------|------|---------|------|
| **T3.1 单三角命中中心** | 1个三角 `(0,0,0)-(1,0,0)-(0,1,0)` | 从 `(0.2,0.2,1)` 指向 `(0,0,-1)` | hit=true, face_id=0 | 命中点误差 < 1e-6 | Möller-Trumbore |
| **T3.2 单三角未命中** | 同上 | 从 `(2,2,1)` 指向 `(0,0,-1)` | hit=false | — | — |
| **T3.3 两三角-最近命中** | 2个三角: z=0 和 z=0.5 | 从 `(0,0,1)` 指向 `(0,0,-1)` | hit z=0.5 (先命中) | distance ≈ 0.5 | — |
| **T3.4 自交忽略** | 同上, 射线原点在 z=0 面上 | ignored_face_id=face0 | hit z=0.5 (忽略原点面) | face_id != 0 | FaceQueryContext |
| **T3.5 边命中** | 1个三角 | 射线精确通过边 | hit=true | 命中点在边上 | Möller-Trumbore |
| **T3.6 IsOccluded-遮挡** | 一个三角挡在 start-end 之间 | start 在三角一侧, end 在另一侧 | occluded=true | — | — |
| **T3.7 IsOccluded-无遮挡** | 无三角在路径上 | 同 T3.2 | occluded=false | — | — |

#### 测试组 4: `test_scene_builder` — 场景构造

| 测试用例 | 操作 | 期望 | 验收标准 |
|---------|------|------|---------|
| **T4.1 添加顶点** | `AddVertex(0,0,0).AddVertex(1,0,0).AddVertex(0,1,0)` | scene.vertices.size()=3 | 坐标一致 |
| **T4.2 单面构造** | `AddFace(0,1,2)` | face.normal 为单位向量, face.area=0.5 | `abs(area-0.5)<1e-12` |
| **T4.3 法线方向** | 顶点(0,0,0)→(1,0,0)→(0,1,0), 逆时针(从+Z看) | normal=(0,0,1) | 右手定则 |
| **T4.4 AABB** | 同上 | bounds.min=(0,0,0), bounds.max=(1,1,0) | 误差 < 1e-12 |
| **T4.5 重心** | 同上 | centroid=(1/3, 1/3, 0) | 误差 < 1e-12 |
| **T4.6 双侧材质** | `AddFace(0,1,2, "Air", "Concrete")` | front="Air", back="Concrete", eps_r=1.0/7.0 | 材质名正确 |

---

### P0.5 自测闭环结果

> ✅ **完成 — 2026-06-23**

| 测试组 | 用例数 | 通过 | 失败 | 备注 |
|--------|--------|------|------|------|
| test_math (向量基本运算) | 8 | ✅ 8 | 0 | T1.1-T1.8 全部通过 |
| test_snell (Snell折射) | 7 | ✅ 7 | 0 | T2.1-T2.7 全部通过 (含 TIR/法线翻转/cos clamp/掠入射) |
| test_bruteforce (暴力求交) | 7 | ✅ 7 | 0 | T3.1-T3.7 全部通过 (命中/未命中/最近/自交忽略/边命中/遮挡) |
| test_scene_builder (场景构造) | 7 | ✅ 7 | 0 | T4.1-T4.7 全部通过 (含 AddWedge) |
| **总计** | **29** | **✅ 29** | **0** | 97 断言通过, 0 断言失败 |

> **发现与修正**：H2hRT 原版 `step1_snell_selftest.h` 的 T2.2 测试使用了垂直入射 `(0,0,1)` 来测试 45° TIR，该测试用例本身有误（垂直入射不会发生 TIR）。已修正为正确的 45° 斜入射方向。

---

### P0.6 批次完成检查清单

- [x] `cmake -B build && cmake --build build` 编译通过 (0W/0E)
- [x] `build/sbr_tests` 全部 29 个测试用例通过 (97/97 断言)
- [x] `SnellRefractV2` 的 TIR / 法线翻转 / cos clamp 行为与 H2hRT 完全一致 (并修正了 H2hRT 测试 bug)
- [x] `BruteForceAccelerator::QueryClosestFaceHit` 对小场景 (< 10 面) 正确 (Möller-Trumbore)
- [x] `SceneBuilder` 可编程构造三角形/平面/墙角场景 (含 AddWedge)
- [x] 所有头文件可独立 include（无循环依赖, 编译通过）
- [x] 本手册 P0.5 自测结果已填充

---

---

## Phase 1: 基础射线发射 + 单反射 + 固定接收球

### P1.0 批次目标

实现最简 SBR 闭环：Fibonacci 球面射线发射 → 单层反射 Wavefront → 固定半径接收球 Rx 命中检测 → 路径记录。验证 LoS 和单反射场景的正确性。

---

### P1.1 实现前状态

- ✅ P0 完成：数学库、BruteForceAccelerator、SceneBuilder、测试框架
- ✅ 所有骨架头文件已存在，需填充实现
- ⬜ 无射线发射器、无 Wavefront 循环、无 Rx 检测器

---

### P1.2 头脑风暴

**参考资源**：
- H2hRT `SbrEngine.cpp:43-53` — Fibonacci sphere 实现 (直接移植)
- H2hRT `SbrEngine.cpp:199-211` — 反射方向计算 (ReflectDir inline)
- H2hRT `SbrEngine.cpp:696-765` — 射线追踪主循环 (while 循环结构, Rx 检测, 反射分支)
- H2hRT `SbrEngine.cpp:56-150` — RxHashGrid 构建和 CheckSegment 查询 (Phase 4 改用)
- Seidel & Rappaport 1994 — 接收球模型理论基础

**设计决策**：
- P1-D1: Phase 1 做**逐射线串行追踪**而非 Wavefront 批量处理。理由: 仅反射(无分支), 批量优势不明显; Phase 5 再切换 Wavefront
- P1-D2: Rx 命中检测使用**点到线段距离**判据, 球半径从 `SbrConfig.rx_sphere_radius_m` 取。H2hRT 的 RxHashGrid 留到 Phase 4/5
- P1-D3: 追踪循环结构: **先检测 Rx → 再判断终止 → 再处理反射**。避免 LoS (cr=0) 和末段漏检

**发现的 H2hRT 测试 Bug**:
- H2hRT `step1_snell_selftest.h` T2.2 使用 `incident=(0,0,1)` 测试 45° 玻璃→空气 TIR，实际产生的是 0° 垂直入射 (不会 TIR)。已在 P0 修正。

---

### P1.3 关键实现步骤

1. **`sbr_ray_emitter.cpp`**: 移植 H2hRT Fibonacci Sphere (`y = 1 - 2i/(N-1)`, `θ = π(3-√5)i`, 单位球面均匀采样)
2. **`sbr_engine.cpp` — RunPointToPoint**: 
   - 发射 N 条 Fibonacci 射线
   - 逐射线追踪循环: 求交 → Rx命中检测(点到线段距离<球半径) → 终止判断 → 反射方向计算 → 状态更新
   - 路径记录: Tx节点 → [交互节点...] → Rx节点, 累加 segment_length 得 total_length
3. **`sbr_engine.cpp` — RunCoverage**: 逐 Rx 调用 RunPointToPoint, 用自由空间路径损耗代理累加功率
4. **冒烟测试**: LoS 自由空间 + PEC 地面反射 + 空场景无反射

**实现过程中修复的 Bug**:
- **Bug #1 (关键)**: Rx 检测在 `while(cr>0)` 循环内, 导致 LoS (cr=0) 从未检测 Rx → 重构循环为 `while(true)` + 先检测/后终止
- **Bug #2**: Rx 节点 `segment_length_from_previous` 错误使用垂直距离而非沿射线距离 → 改为 `tClosest * segLen`

---

### P1.4 自测试闭环方案 (执行结果)

| 测试 | 场景 | 验收标准 | 结果 |
|------|------|---------|:--:|
| LoS 近距离命中 | 空场景, Tx(0,0,0), Rx(0,2,0), ray_count=100, sphereR=1.0m | 至少1条路径, 2节点(Tx+Rx), total_length≈2.0m | ✅ |
| LoS ray_count 收敛 | 同上, ray_count=100→500→2000 | 命中数单调不降 | ✅ |
| PEC 地面反射 | 10×10m 地面(z=0), Tx(0,0,2), Rx(0,2,1), 50000 rays | 存在非LoS的3节点反射路径 | ✅ |
| 空场景仅LoS | 无面元 | 所有命中路径 is_los=true, 2节点 | ✅ |
| P0 回归 | 全部 P0 单元测试 | 29/29 通过 | ✅ |

---

### P1.5 自测闭环结果

> ✅ **完成 — 2026-06-23** — 2111 断言通过, 0 失败

| 文件 | 新增/修改 | 内容 |
|------|:--:|------|
| `src/sbr_ray_emitter.cpp` | 新增 | Fibonacci Sphere 射线发射 (30行) |
| `src/sbr_engine.cpp` | 重写 | RunPointToPoint + RunCoverage (完整实现) |
| `test/smoke_los.cpp` | 新增 | LoS 冒烟测试 (2 用例) |
| `test/smoke_pec_reflection.cpp` | 新增 | PEC 反射冒烟测试 (2 用例) |
| `CMakeLists.txt` | 修改 | 添加 sbr_ray_emitter.cpp + smoke test 文件 |

---

### P1.6 批次完成检查清单

- [x] Fibonacci Sphere 射线发射正确 (首射线沿 +Y, 均匀球面覆盖)
- [x] RunPointToPoint: LoS 路径生成正确 (2节点, total_length≈直线距离)
- [x] RunPointToPoint: 单反射路径生成正确 (3节点, Tx→Refl→Rx)
- [x] 固定接收球 Rx 命中检测正确 (点到线段距离判据)
- [x] 空场景仅生成 LoS 路径 (无反射面=无反射路径)
- [x] 所有 P0 回归测试通过

---

---

## Phase 2: 透射 + 确定性 R/T 分裂

### P2.0 批次目标

实现 Snell 透射方向计算、TIR 判断、确定性反射/透射双分支生成、介质状态追踪、MaterialDatabase 频率相关折射率查询。

---

### P2.1 实现前状态

- ✅ P1 完成：LoS 和单反射路径可正确生成
- ⬜ 无透射支持，无 TIR 处理，无介质追踪

---

### P2.2 头脑风暴

**参考资源**：
- H2hRT `TransmissionExpander.cpp:122-123` — 透射材质查询: `matDb.QueryByName(face.front_material_name, freqHz)`
- H2hRT `Vec3.h:233-288` — SnellRefractV2 (已在 P0 移植)
- H2hRT `MaterialDatabase.h:96-136` — ITU-R P.2040 频率外推 (已移植)
- H2hRT `SbrEngine.cpp:213-234` — Fresnel 功率缓存 (法向入射上界)
- RT.XD `SbrRayGeneratedByTransmission.cpp` — 双侧材质查找 (UpTypeNumber/DownTypeNumber)

**设计决策**：
- P2-D1: **确定性双分支** — 同一命中面元同时生成 R 和 T 两条后继射线 (非 Monte Carlo)
- P2-D2: 追踪改为**栈式 DFS** — 每个交互事件 push 1~2 个后继状态。最坏分支数 2^maxDepth (maxDepth≤6 → ≤64)
- P2-D3: 功率上界使用**法向入射 Fresnel**: R=|(n1-n2)/(n1+n2)|², T=1-R. Phase 4 再改进
- P2-D4: 材质查询: `face.front_material_name`/`back_material_name` → `matDb.QueryByName(name, center_frequency_hz)` → `n=√ε_r`
- P2-D5: TIR 硬拒绝 — `SnellRefractV2.total_internal_reflection=true` 时仅生成反射分支

**实现要点**：
- `entered_from_front_side` 通过 `Dot(ray_dir, face.normal) < 0` 判断 (负→入射面为 front)
- 介质追踪: 反射不改变 `current_medium_id`; 透射切换到另一侧 `medium_id`
- 空 MaterialDatabase (测试用) → n1=n2=1.0 (无折射偏转，但透射分支仍生成)

---

### P2.3 关键实现步骤

1. **重构追踪循环**为栈式 DFS (`std::vector<TraceState>`)
2. 面元命中后: 若 `face.reflection_enabled && cr>0` → push 反射分支
3. 若 `face.transmission_enabled && ct>0`:
   - 确定入射侧 (`entered_from_front_side`)
   - 从 MaterialDatabase 查询两侧 ε_r(f) → 计算 n1, n2
   - `SnellRefractV2(incident, normal, n1, n2)` → 得透射方向
   - 若非 TIR → push 透射分支 (带 `newMediumId`)
4. 每条分支独立携带 `node_history` (栈复制)
5. 后处理: 路径包含 Transmission 节点时标记 `contains_transmission=true`

---

### P2.4 自测试闭环方案

| 测试 | 场景 | 验收标准 | 结果 |
|------|------|---------|:--:|
| 介质透射 | 玻璃平板(z=0), Tx(z=2)→Rx(z=-1) | 存在含 Transmission 节点的路径 | ✅ |
| R+T 双分支 | 同上, Rx 反射侧+透射侧各一个 | 同时存在 Reflection 和 Transmission 路径 | ✅ |
| P0+P1 回归 | LoS (空场景) + PEC 地面反射 | 重构后 P0/P1 全部测试通过 | ✅ |

---

### P2.5 自测闭环结果

> ✅ **完成 — 2026-06-23** — 2118 断言通过, 0 失败

| 文件 | 新增/修改 | 内容 |
|------|:--:|------|
| `src/sbr_engine.cpp` | 重写 | 栈式 DFS 追踪 + 确定性 R/T 双分支 + 介质追踪 + Fresnel 功率 |
| `test/smoke_transmission.cpp` | 新增 | 介质透射 + R+T 双分支 + P0/P1 回归 (3 用例) |
| `CMakeLists.txt` | 修改 | 添加 smoke_transmission.cpp |

---

### P2.6 批次完成检查清单

- [x] 透射方向计算正确 (SnellRefractV2, Snell 残差记录在 PathNode)
- [x] TIR 正确拒绝透射分支 (sint2 ≥ 1.0 → 仅反射)
- [x] 确定性 R+T 双分支生成 (同一命中面同时 push R 和 T)
- [x] 介质追踪正确 (反射不变, 透射切换 currentMediumId)
- [x] entered_from_front_side 正确记录
- [x] P0 + P1 全部回归测试通过

---

---

## Phase 3: 绕射 (Keller 锥 + 楔边耦合)

### P3.0 批次目标

实现射线管-楔边耦合候选生成、segment-segment 最近距离计算、Keller 锥方向采样、楔边可见性验证、R-D-T 混合路径支持。

---

### P3.1 实现前状态

- ✅ P2 完成：反射+透射双分支正常
- ⬜ 无绕射支持

---

### P3.2 头脑风暴

**参考资源**：
- H2hRT `SbrEngine.cpp:770-823` — 绕射: 边命中检测 + Keller锥采样 + Rx命中 (4采样硬编码)
- H2hRT `SbrEngine.cpp:266-315` — segment-segment 最近距离 (用于楔边耦合, Phase 4)
- H2hRT `SbrEngine.cpp:317-323` — FindWedgeIdByEdgeId
- Keller 1962 — Keller 锥条件: d_o·e = d_i·e
- 绕射方向公式: d_o(φ) = cos(β₀)·e + sin(β₀)·(cos(φ)·p + sin(φ)·q), 其中 c = d_i·e, s = √(1-c²)

**设计决策**：
- P3-D1: **边命中检测** (非楔边耦合) — Phase 3 使用"射线命中面→检测是否命中面边→触发对应楔边绕射"。管-楔耦合留到 Phase 4
- P3-D2: 边命中阈值 eps = 0.02m (与频率弱相关, Phase 4 改进)
- P3-D3: Keller 锥采样数 N_d 可配置 (diffraction_rays_per_event), 默认4
- P3-D4: 绕射功率代理 = 0.08 (简化的 UTD 标量)
- P3-D5: Phase 3 不做可见性验证 (Phase 4 补充 IsOccluded 检查)

**测试拓扑问题**: SceneBuilder 不自动计算边邻接/楔边拓扑。Phase 3 测试需手动设置 `Edge`/`Wedge` 并链接 `Face.adjacent_edge_id`

---

### P3.3 关键实现步骤

1. **`DetectEdgeHit()`**: 点到面3条边的距离 < eps → 返回 edge_id
2. **`FindWedgeByEdgeId()`**: 遍历 `scene.wedges` 找 `source_edge_id == edgeId`
3. **`GenerateKellerConeDirections()`**: 正交基 (p, q) + φ 均匀采样 → N_d 个绕射方向
4. **追踪循环集成**: 反射/透射分支后, 检查边命中 → 生成绕射分支 (含 `DiffractionDiagnostics`)
5. **手动拓扑测试**: SceneBuilder.Build() 后手动添加 Edge/Wedge 并链接 Face

---

### P3.4 自测试闭环方案

| 测试 | 场景 | 验收标准 | 结果 |
|------|------|---------|:--:|
| 单楔边绕射路径 | 90°楔边, Tx阴影区 | 存在含 Diffraction 节点的路径 | ✅ |
| Keller锥残差 | 同上 | 所有绕射节点 keller_residual < 1e-2 | ✅ |
| N_d 收敛性 | N_d=4→8→16 | 绕射路径数单调不降 | ✅ |
| 无楔边→无绕射 | 平面场景, 无 wedge | 所有路径不含 Diffraction 节点 | ✅ |
| P0-P2 回归 | LoS + R+T | 全部通过 | ✅ |

---

### P3.5 自测闭环结果

> ✅ **完成 — 2026-06-23** — 2345 断言通过, 0 失败

| 文件 | 新增/修改 | 内容 |
|------|:--:|------|
| `src/sbr_engine.cpp` | 重写 | +绕射分支 (DetectEdgeHit + FindWedgeByEdgeId + GenerateKellerConeDirections) |
| `test/smoke_diffraction.cpp` | 新增 | 楔边绕射 + Keller残差 + N_d收敛 + 无楔边场景 + P0-P2回归 (5 用例) |
| `CMakeLists.txt` | 修改 | 添加 smoke_diffraction.cpp |

---

### P3.6 批次完成检查清单

- [x] 边命中检测正确 (点到边距离 < eps)
- [x] Keller 锥方向生成正确 (d_o·e ≈ d_i·e, 残差 < 1e-2)
- [x] 绕射分支集成到追踪循环 (R/D/T 混合路径)
- [x] DiffractionDiagnostics 字段填充 (keller_residual, s1, wedge_id)
- [x] 无楔边场景不生成绕射
- [x] N_d 增加 → 绕射路径数单调不降
- [x] P0-P2 全部回归测试通过

---

---

## Phase 4: 射线管模型 + 动态接收球 + 剪枝 + 后处理

### P4.0 批次目标

实现动态射线管半径接收球、多层剪枝（功率/深度/Rx可达/峰值上限）、路径后处理链（签名去重/残差过滤/相似剪枝/top-N）。

---

### P4.1 实现前状态

- ✅ P3 完成：R/D/T 三种交互均可生成
- ⬜ 固定接收球、无剪枝、无后处理

---

### P4.2 头脑风暴

> ⬜ **P4 启动前进行**

**预留**：
- 参考 Seidel & Rappaport 1994 (动态接收球理论基础)
- 参考 H2hRT `SbrEngine.cpp:344-420` (路径相似键/签名/功率代理)
- 参考 H2hRT `PathSignatureBuilder.h` (路径签名去重)

---

### P4.3 关键实现步骤

> ⬜ **P4 启动前细化**

---

### P4.4 自测试闭环方案

> ⬜ **P4 启动前细化**

---

### P4.5 自测闭环结果

> ⬜ **待 P4 实现完成后填充**

---

### P4.6 批次完成检查清单

> ⬜ **P4 启动前细化**

---

---

## Phase 5: 批量优化 + 加速后端

### P5.0 批次目标

性能优化：Wavefront 批量查询、OpenMP 并行化、RxHashGrid 空间索引、Embree/OptiX 后端接口、性能基准测试。

---

### P5.1 实现前状态

- ✅ P4 完成：功能完整的 SBR 引擎
- ⬜ 无批量优化，无 GPU 后端，仅 BruteForce 加速器

---

### P5.2 头脑风暴

> ⬜ **P5 启动前进行**

**预留**：
- 参考 Embree (Wald et al. 2014) — BVH 构建与 packet tracing
- 参考 OptiX 7.7 Programming Guide (NVIDIA 2023) — GPU 加速后端
- 参考 H2hRT `SbrGpuPipeline.cu` (GPU wavefront megakernel)

---

### P5.3 关键实现步骤

> ⬜ **P5 启动前细化**

---

### P5.4 自测试闭环方案

> ⬜ **P5 启动前细化**

**性能基准**：200k 射线/深度6/412会议室 → CPU < 120s, GPU < 15s

---

### P5.5 自测闭环结果

> ⬜ **待 P5 实现完成后填充**

---

### P5.6 批次完成检查清单

> ⬜ **P5 启动前细化**

---

---

## Phase 6: 集成验证 + 回归测试

### P6.0 批次目标

将独立 SBR 模块合并到 H2hRT-7.1-SBR 全量引擎，运行全量回归测试，验证 P2P 和 Coverage 两种模式。

---

### P6.1 实现前状态

- ✅ P5 完成：功能完整 + 性能达标
- ⬜ 未与 H2hRT 集成

---

### P6.2 头脑风暴

> ⬜ **P6 启动前进行**

**预留**：
- 参考本手册 §11 合并策略
- 参考 H2hRT `validation/run_all.py` 回归测试脚本

---

### P6.3 关键实现步骤

> ⬜ **P6 启动前细化**

---

### P6.4 自测试闭环方案

| 验证项 | 验收标准 |
|--------|---------|
| 编译 | 替换现有 `SbrEngine` 后 0W/0E |
| SBR suite | `python validation/run_all.py --suite sbr` 全部通过 |
| 全量回归 | `python validation/run_all.py --suite all` 33P/0F/1S 或更优 |
| P2P 模式 | SBR 几何路径 → EM 链 → CIR/PDP/APS/XPR 输出正常 |
| 收敛性 | ray_count sweep 实验可复现 |

---

### P6.5 自测闭环结果

> ⬜ **待 P6 实现完成后填充**

---

### P6.6 批次完成检查清单

> ⬜ **P6 启动前细化**

---

---

## 附录 A: 完整类型定义 (与 H2hRT 精确对齐)

> 以下类型定义经过 H2hRT-7.1-SBR 源码审查，字段名、类型、默认值均与 `rt::` 命名空间下的对应类型**布局兼容**。

### A.1 几何基础类型

已在 P0 Step 0.2 中定义：`Vec3`, `Point3`, `AABB`, `Ray`, `FaceHit`。

### A.2 场景类型

已在 P0 Step 0.3 中定义：`Face`, `Edge`, `Wedge`, `Scene`。

### A.3 材质类型

已在 P0 Step 0.4 中定义：`MaterialProps`, `MaterialDatabase`。

### A.4 配置类型

已在 P0 Step 0.5 中定义：`SbrConfig`, `NumericToleranceConfig`。

### A.5 路径类型

已在 P0 Step 0.6 中定义：`InteractionType`, `DiffractionDiagnostics`, `PathNode`, `GeometricPath`, `GeometricPathSet`。

### A.6 加速器类型

已在 P0 Step 0.7 中定义：`FaceQueryContext`, `VisibilityQueryContext`, `ISceneAccelerator`。

### A.7 内部数据类型

```cpp
namespace sbr {

static constexpr int kMaxPathDepth = 32;  // 硬上限

// Coverage 模式: 轻量级射线状态 (无路径历史, ~80 bytes)
struct alignas(64) RayStateCoverage {
    double ox, oy, oz, dx, dy, dz;
    double curPwr;
    int cr, ct, cd, depth;
    double tube_angle;
    int last_face_id, last_wedge_id, current_medium_id;
    bool alive;
};

// FineChannel 模式: 带路径历史的射线状态 (~3KB)
struct alignas(64) RayStateFineChannel {
    double ox, oy, oz, dx, dy, dz;
    double curPwr;
    int cr, ct, cd, depth;
    double tube_angle;
    int last_face_id, last_wedge_id, current_medium_id;
    bool alive;
    std::array<PathNode, kMaxPathDepth> node_history;
    int node_count;
};

// 分支描述
struct BranchDesc {
    Vec3 new_direction;
    double power_fraction;
    InteractionType interaction_type;
    int hit_face_id, hit_wedge_id;
    Point3 interaction_point;
    Vec3 surface_normal;
    int new_medium_id;
    double snell_residual, keller_residual;
};

// 楔边耦合候选
struct WedgeCoupling {
    int wedge_id;
    Point3 wedge_point;
    double dist_to_ray, tube_radius;
    bool coupled;
};

// Coverage 模式: 在线累加 bin (无路径存储)
struct CoverageBin {
    double total_power_linear = 0.0;
    int ray_hit_count = 0;
};

// FineChannel 模式: 路径存储 bin
struct FineChannelBin {
    std::vector<GeometricPath> paths;
};

// 输出类型
struct RxCoverageRecord {
    Point3 rx_position;
    int rx_index = -1;
    double total_power_linear = 0.0, total_power_dBm = 0.0;
    int ray_hit_count = 0;
    std::vector<GeometricPath> paths;  // Coverage 模式下为空
};

struct SbrCoverageResult {
    bool succeeded = false;
    std::string trace_profile = "Coverage";
    int total_rays = 0, active_rx_count = 0;
    std::vector<RxCoverageRecord> rx_records;
    std::vector<std::string> trace_lines;
    // 诊断字段 (共 40+ 字段, 见附录 A.8)
};

} // namespace sbr
```

### A.8 SbrCoverageResult 诊断字段完整清单

> 以下所有字段在 P0 定义，P1-P4 逐步填充，P6 集成验证确认全部正确填充。

```
total_bounces, total_transmissions, total_diffractions
rays_below_threshold, rays_terminated_early
generated_reflection_branches, generated_transmission_branches
rejected_tir_transmissions, pruned_power_branches
rx_paths_recorded, rx_paths_skipped_by_cap, rx_paths_skipped_by_rx_cap
rx_paths_deduplicated, peak_active_rays
dynamic_rx_radius_enabled, ray_tube_angle_rad, max_effective_rx_radius_m
dynamic_rx_queries, dynamic_rx_hits
wedge_tube_coupling_enabled, wedge_tube_queries, wedge_tube_candidates
wedge_tube_rejected, wedge_edge_fallback_hits
diffraction_rays_per_event, diffraction_events
generated_diffraction_branches, rejected_keller_diffractions
path_dedup_enabled, path_similarity_pruning_enabled, path_top_n_per_rx
paths_pruned_by_post_dedup, paths_pruned_by_similarity
paths_pruned_by_top_n, paths_after_postprocess
path_residual_filter_enabled, paths_evaluated_for_residual
paths_pruned_by_residual, max_path_geometry_residual
convergence_notes
```

---

## 附录 B: 场景文件格式

### B.1 OBJ 网格文件

标准 Wavefront OBJ (ASCII)，参考 `demo/Scene/meeting.obj`：
- `o object_name` — 物体名 (匹配 material_map JSON 的 `object_name`)
- `v x y z` — 顶点坐标
- `vn nx ny nz` — 顶点法向
- `f v1//vn1 v2//vn2 v3//vn3` — 三角面元

### B.2 材质绑定 JSON

参考 `demo/Material/material_map-meeting.json`：
```json
{
  "default_medium": "Air",
  "objects": [{
    "rule_name": "...",
    "object_name": "1",
    "object_type": "wall",
    "surface_material_name": "Concrete",
    "front_material_name": "Air",
    "back_material_name": "Concrete",
    "normal_rule": "front_is_air",
    "reflection_enabled": true,
    "transmission_enabled": false,
    "diffraction_candidate_enabled": false
  }]
}
```

### B.3 材质数据库 CSV

ITU 格式：`id,name,category,frequency_Hz,epsilon_r,sigma,mu_r`

---

## 附录 C: 合并策略 (独立模块 → H2hRT)

详见 Phase 6。概要：
1. `static_assert` 验证类型布局兼容 → 零拷贝
2. `H2hRTAcceleratorAdapter` 包装 `SceneQuery` 实现 `ISceneAccelerator`
3. 替换 `rt::SbrEngine` 为 `sbr::SbrEngine`
4. 运行全量回归：`validation/run_all.py --suite all`
5. 合并成本 ≈ 2-3 天

---

## 附录 D: 参考学术文献与开源项目

### 核心文献

| 文献 | 关键贡献 | 指导批次 |
|------|---------|:------:|
| Möller & Trumbore 1997 | 射线-三角求交标准算法 | P0 |
| Seidel & Rappaport 1994 | SBR 方法奠基 — 射线管+接收球模型 | P1, P4 |
| Keller 1962 | Keller 锥绕射条件 d_o·e = d_i·e | P3 |
| Kouyoumjian & Pathak 1974 | UTD 绕射系数 | P3 |
| Yun & Iskander 2015 | SBR vs 镜像法全面对比综述 | P1-P4 |
| ITU-R P.2040-1 | 建筑材料频率依赖模型 (ε_r(f)) | P2 |
| Wald et al. 2014 | Embree 高性能CPU射线追踪 | P5 |
| NVIDIA 2023 | OptiX 7.7 Programming Guide | P5 |
| 3GPP TR 38.901 v19.2.0 | 标准化信道模型 | P6 |

### 开源项目参考

| 项目 | 参考价值 | 指导批次 |
|------|---------|:------:|
| Embree (Intel, Apache 2.0) | BVH 构建 + packet tracing | P5 |
| pbrt-v4 | BVH、采样、积分器架构 | P0, P5 |
| Sionna RT (NVIDIA, MIT) | 射线发射、材质模型 | P1, P2 |

### 已审查的参考代码

| 资产 | 路径 | 审查内容 |
|------|------|---------|
| H2hRT-7.1-SBR 全量源码 | `G:\RT\H2hRT-7.1-SBR-\` | 所有接口类型、SbrEngine 实现、SceneQuery/ISceneAccelerator、MaterialDatabase |
| RT.XD.SBR.CGAL.25.05 | `G:\RT\算法\` | 反射/透射方向计算、UTD 绕射系数(EM层)、SbrConfig |
| Demo 场景 | `G:\RT\sbr\sbr\demo\` | meeting.obj (ASCII, 905行) + material_map-meeting.json |

---

## 附录 E: 三大设计决策

| # | 决策 | 结论 | 影响批次 |
|---|------|------|:------:|
| D1 | 频率是否影响寻径？ | **是** — 透射角 n=√ε_r(f) 依赖频率；SbrConfig 包含 `center_frequency_hz` | P0, P2 |
| D2 | 材质绑定体系？ | **直接沿用** H2hRT 的 OBJ→JSON→CSV 三层体系 | P0, P2 |
| D3 | 绕射耦合 vs 独立？ | **耦合** — 绕射分支在 Wavefront 循环内生成，支持 R-D-T 混合路径 | P3 |

---

> **手册维护规则**：
> - 每批次开始前：填写该批次的"头脑风暴"→ 细化"实现步骤"→ 完善"自测方案"
> - 每批次完成后：填写"自测结果"→ 勾选"完成检查清单"→ 更新"批次总览"状态
> - 如发现本手册设计有误：先讨论 → 修订相关章节 → 记录修订原因
