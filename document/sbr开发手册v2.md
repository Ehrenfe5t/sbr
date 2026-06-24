# SBR 几何寻径模块 — 开发手册 v2

> 2026-06-24 | 基于 v1 实施反馈 + 三方实现对比 + 文献调研

---

## 1. 多Rx寻径与终止条件 — 架构澄清

### 1.1 当前实现的多Rx支持

**是，完全支持多Rx。** 实现方式：

```
RunCoverage(scene, matDb, config, txPoint, rxGrid, tol)
  │
  └─ for each Rx in rxGrid:
       └─ RunPointToPoint(scene, matDb, config, txPoint, {singleRx}, tol)
            │
            └─ 发射 N 条 Fibonacci 射线
               └─ 每条射线: 栈DFS追踪
                  ├─ 每段检测: 当前Rx是否在接收球内 → 记录路径
                  ├─ 命中面元 → push R + T + D 分支
                  └─ 终止条件: 无命中 | 超深度 | 低于功率阈值
```

**关键特征**: 每条射线从Tx发出后，**不是**"碰到任意Rx就终止"。射线会持续追踪直到自然终止（无命中/深度上限/功率阈值），并在**每一段**检查是否命中Rx。同一条射线可以命中多个不同的Rx，也可以在不同深度命中同一个Rx。

### 1.2 终止条件（完整清单）

| 条件 | 说明 | 类型 |
|------|------|:--:|
| 无面元命中 | 射线飞向无穷远 (FAR_DIST=1e6m) | 自然终止 |
| 深度超限 | `depth >= max_ray_depth` | 配置限制 |
| 功率阈值 | `curPwr <= 10^(power_threshold_dB/10)` | 配置限制 |
| 无可交互面 | 命中面元 `reflection_enabled=false` 且 `transmission_enabled=false` | 场景限制 |
| 剩余次数耗尽 | `cr<=0 && ct<=0 && cd<=0` (但仍先检测Rx) | 配置限制 |

**明确指出**: 射线**不会**因为命中Rx而终止。这确保了同一条射线可以在传播路径的不同阶段命中多个Rx位置。

### 1.3 几何-EM 解耦设计

```
┌─────────────────────────────────────────────────────────┐
│  SBR 几何寻径层 (本模块)                                │
│  输入: 场景几何 + 材质εr(f) + 配置                       │
│  输出: GeometricPathSet (纯几何路径 + 诊断残差)          │
│                                                         │
│  使用材质信息的目的:                                     │
│  ✅ 折射率 n=√εr → Snell方向计算                        │
│  ✅ TIR判据 (n1/n2 比值)                                │
│  ✅ Fresnel功率上界 (用于几何层剪枝, 非精确EM)            │
│  ❌ 不做: 极化旋转、复振幅传播、天线响应                  │
├─────────────────────────────────────────────────────────┤
│  EM 求解层 (下游, H2hRT 或独立模块)                       │
│  输入: GeometricPath + 天线方向图 + 完整材料参数           │
│  输出: EMPathResult (CIR/PDP/APS/XPR/功率)               │
└─────────────────────────────────────────────────────────┘
```

**耦合点分析**: 当前唯一的几何-EM耦合是Fresnel功率上界（用于剪枝）。如果用 **FSPL (自由空间路径损耗)** 作为唯一剪枝阈值，可完全解耦：

```cpp
// 完全解耦方案: 仅用FSPL
double fsplThreshold = config.ray_power_threshold_dB;  // e.g. -180dB
double curFSPL_dB = 20.0 * log10(cumulativeLength) + 20.0 * log10(freqHz) - 147.55;
if (curFSPL_dB > fsplThreshold) break;  // 仅距离衰减, 无交互类型系数
```

**建议**: 保留当前Fresnel上界作为**可选优化**（减少明显不可能到达Rx的路径），但默认使用纯FSPL阈值实现几何-EM完全解耦。

---

## 2. 三种实现深度对比

### 2.1 RT.XD 递归 R→T vs 独立sbr 栈DFS R+T

```
RT.XD 递归 (R→T):                    独立sbr 栈DFS (R+T):
                                    
SbrFindPath(ray):                    while stack:
  求交                                  pop state
  Rx检测                                求交
  if hit:                               Rx检测
    reflect → SbrFindPath(reflected)     if hit:
    root.pop_back()                         push(反射state)
    transmit → SbrFindPath(transmitted)     push(透射state)
    root.pop_back()                          push(绕射state)
```

**递归 R→T 的特点**:
- 反射先于透射，反射路径完整追踪后再回溯追踪透射
- 新射线继承原功率（功率不在此层衰减，功率系数在上层EM计算）
- 递归深度 = 实际交互次数（受 `ejectionsMaxTotalNumber` 限制）
- 内存: O(depth) 调用栈, 无堆分配

**栈DFS R+T 的特点**:
- R和T平等push, 出栈顺序取决于push顺序 (R先push → R后出栈, T后push → T先出栈)
- 每个状态独立携带 node_history (完整路径节点)
- 栈深度 = 最大状态数 (可达 2^depth)
- 内存: O(2^depth × sizeof(TraceState))

**内存风险分析**:

```
TraceState 大小:
  Point3 curPt:        24B
  Vec3 curDir:         24B
  double curPwr:        8B
  5×int:               20B
  vector<PathNode>:    ~3KB (depth=8, 每个PathNode ~300B + vector overhead)
  double cumLen:        8B
  ─────────────────────
  合计:               ~3.1KB

最坏栈大小 (depth=8, R+T enabled):
  = 2^8 × 3.1KB = 256 × 3.1KB ≈ 800KB per ray
  × 2M rays + OpenMP threads ≈ 可能达到 GB 级别
```

**结论**: 栈DFS在 depth≤4 时安全, depth≥6 时有风险。可通过以下方式缓解:
1. 限制峰值栈大小 (per-ray hard cap)
2. 在 Coverage 模式使用 Monte Carlo (无分支)
3. 使用固定数组替代 `std::vector<PathNode>` (已在手册v1中设计)

### 2.2 加速结构差异

| | RT.XD CGAL | 独立sbr SAH-BVH | H2hRT FaceBVH |
|---|-----------|----------------|---------------|
| **构建** | CGAL AABB Tree (库函数) | 桶排序SAH (16桶, 3轴最优) | 逐点SAH |
| **遍历** | CGAL内置 | 距离优先 + 原子tMin | 标准递归 |
| **叶子大小** | CGAL默认 | 8 + depth/8×2 (动态) | 16 (固定) |
| **并行构建** | 无 | OpenMP sections | 无 |
| **依赖** | CGAL (GPL) | 零依赖 | 零依赖 |

**RT.XD也有BVH**: RT.XD使用CGAL的AABB Tree, 这是一个通用BVH库。与我们的自研SAH-BVH的主要区别:
- CGAL AABB Tree使用**中点分割** (非SAH), 构建O(N log N), 查询O(log N)
- 我们的SAH-BVH使用**桶排序SAH** (16个桶沿3轴评估分割成本), 构建质量通常更高
- 两者均为AABB层级结构, 功能等价

### 2.3 Rx检测: 圆柱法 vs 点线距离 vs RxHashGrid

**圆柱法 (RT.XD)**:
```
以射线为轴, 半径为Rx球半径的无限长圆柱
对每个Rx: 检查Rx到射线的垂直距离 < 球半径
O(N_rx) per segment
```

**点线距离法 (独立sbr)**:
```
对每个Rx: 计算Rx到射线段的最近距离
最近点参数 t = dot(rx-origin, dir) / |dir|², clamp to [0, segLen]
dist = |(origin + t*dir) - rx|
if dist < sphereR: 命中
O(N_rx) per segment
```

**两种方法在数学上等价**, 圆柱法等同于无限长射线的垂直距离, 点线法多了clamp到线段范围。当接收球半径远小于线段长度时, 二者结果一致。

**RxHashGrid (H2hRT)**:
```
3D均匀网格: cellSize = 2 × sphereRadius
每个Rx分配到对应网格单元 → O(N_rx) 构建
查询时: 对射线段采样点, 仅查询采样点所在的网格单元 + 邻域
使用 generation counter 掩码避免重复检测
复杂度: O(1) per segment (常数个网格单元)
```

**大规模Rx场景分析**:

| Rx数量 | 圆柱法/点线法 | RxHashGrid | 加速比 |
|:------:|:----------:|:---------:|:-----:|
| 5 | 5次距离计算 | 27格×查询 | ~1× |
| 100 | 100次距离计算 | 27格×查询 | ~4× |
| 10,000 | 10,000次距离计算 | 27格×查询 | ~370× |
| 1,000,000 | 不可行 (每段1M次计算) | 27格×查询 | ~37,000× |

**建议**: 恢复RxHashGrid但不影响寻径结果 — RxHashGrid仅改变**查询方式**, 命中判据(距离<球半径)完全相同, 结果不变。

### 2.4 并行策略: 手动线程池 vs OpenMP

**RT.XD 手动线程池**:
```cpp
// 主线程分配任务范围
for (int i = 0; i < starts.size(); ++i) {
    task.start = starts[i];
    task.end = starts[i] + indexGap;
    threadPool.submit(coreFunc, task);
}
threadPool.join();
```
- 静态任务分配 (固定范围)
- 需要自行管理线程生命周期
- 代码量大, 但可精细控制

**独立sbr OpenMP**:
```cpp
#pragma omp parallel for schedule(dynamic, 1)
for (int ri = 0; ri < N; ++ri) { ... }
```
- `schedule(dynamic, 1)`: 动态任务窃取, 自动负载均衡
- 编译器自动管理线程
- 代码简洁, 标准可移植

**优劣**: OpenMP `dynamic` 在射线间工作量差异大时(部分射线遇到复杂交互→更多分支)更优, 因为快线程可窃取慢线程的任务。RT.XD的静态分配可能导致负载不均。

---

## 3. 效率瓶颈与优化路线图

### 3.1 当前效率瓶颈分析

**Profile 估算** (2M rays, 422面 scene, R+T+D, depth 4):

| 操作 | 调用次数/ray | 耗时占比 |
|------|:----------:|:------:|
| BVH查询 (QueryClosestFaceHit) | ~8次 (depth+分支) | ~40% |
| Rx检测 (点到线段距离) | ~8次 × N_rx | ~25% |
| 节点拷贝 (TraceState复制) | ~8次 × 3KB | ~15% |
| 材质查询 (QueryByName) | ~4次 | ~10% |
| 后处理 (去重+排序) | 1次/total | ~10% |

**主要瓶颈**: BVH查询 + Rx检测 + 状态拷贝

### 3.2 接收球半径的动态调整

**Seidel & Rappaport 1994 理论基础**:
- 射线管截面积随距离线性增长: `A(d) ∝ d²`
- 接收球半径应与射线管截面匹配: `r_eff(d) = d × Δθ / 2`
- 其中 `Δθ = √(4π / N_rays)` 为射线平均角间距

**当前实现** (已在Phase 4实现):
```cpp
r_eff(d) = max(r_config, d × Δθ × k_scale)
clamped to [r_min, r_max]
```

**进一步优化方向**:
- `k_scale` 的自适应: 根据场景尺寸和Rx密度自动调整 (当前固定0.5)
- 距离Tx越远, 射线越稀疏, 需要更大的接收球 → **动态半径已经是正确的**
- 对于多跳路径, 累计距离d包含了所有反射段, 动态半径随总路径长度增长

**文献支持**:
- Durgin, Patwari, Rappaport (1997): 建立了 ray_tube_angle 与 Rx球半径的定量关系
- Yun & Iskander (2015): 综述指出动态半径是SBR在大场景中减少漏检的关键

### 3.3 多场景规模适应

**室内→室外扩展**:

| 场景 | 典型尺寸 | 面元数 | 推荐射线数 | 推荐深度 | 推荐Rx球半径 |
|------|---------|:------:|:---------:|:------:|:----------:|
| 会议室 | 20×20×3m | 400~2k | 50k~200k | 4~6 | 0.3~1.0m |
| 办公楼 | 50×50×10m | 2k~10k | 200k~2M | 6~8 | 0.5~1.5m |
| 宏小区 | 500×500×50m | 50k~500k | 5M~50M | 8~12 | 1.0~3.0m |
| 山地 | 2km×2km×500m | 500k~5M | 50M~500M | 10~15 | 2.0~5.0m |

**关键适配参数**:
1. **射线数**: 与场景表面积成正比 (确保每平方米有足够射线密度)
2. **深度**: 室内6~8, 室外10~15 (更长传播路径)
3. **接收球**: 随距离动态调整 (已实现)
4. **BVH叶节点大小**: 大场景可适当增大 (减少节点数)
5. **材质**: 室外场景植被 (ITU-R P.833), 大气吸收 (ITU-R P.676)

### 3.4 大规模Rx覆盖统计优化

**SBR在覆盖统计中的天然优势**:
- Yun & Iskander (2015): "SBR的前向追踪特性使其天然适合覆盖预测 — 不需要逐Rx反向寻径, 一次发射即可覆盖所有Rx"
- Fuschini et al. (2015): "对于大规模Rx网格, SBR的计算复杂度与Rx数量呈亚线性关系"

**优化策略** (按优先级):

1. **RxHashGrid (P0)**: 3D均匀网格将Rx查询从 O(N_rx) 降至 O(1)
2. **Coverage模式轻量化 (P1)**: 仅累加功率, 不存储完整路径
3. **多分辨率策略 (P2)**: 先用粗粒度网格+少量射线扫描 → 识别活跃Rx区域 → 仅对活跃区域加密射线和Rx
4. **自适应Rx网格 (P2)**: 根据场景几何复杂度自动调整Rx密度 (开阔区域稀疏, 复杂区域密集)
5. **GPU批量处理 (P3)**: 百万级Rx网格在GPU上并行处理

**H2hRT的CoarsePass就是多分辨率策略的实现**:
```
CoarsePass:
  - 粗扫射线 (50k vs 2M)
  - 放大接收球 (2.0m vs 0.3m)
  - 仅收集活跃面元/楔边集合
  - 用于后续FineChannel的搜索空间约束
```

### 3.5 BVH与并行效率优化

**BVH优化方向**:

| 方向 | 方法 | 预期提升 |
|------|------|:------:|
| **可见性预处理** | PVS (Potentially Visible Set): 预计算面元间可见性, 剔除不可能被命中的面元 → 减少BVH遍历范围 | 30~50% |
| **边邻接信息** | 预计算Edge Adjacency: 绕射查询时跳过不共享边的面元 | 绕射20~30% |
| **角网格** | Angular Grid: 按射线方向分组, 仅查询可能被命中的BVH子树 | 20~40% |
| **BVH缓存** | 场景不变时复用已构建的BVH (已通过Protobuf在BVH_Project中实现) | 构建时间→0 |
| **动态叶节点** | 根据场景密度动态调整叶子大小 (已部分实现: `kMaxLeafSize + depth/8*2`) | 10~15% |

**并行优化方向**:

| 方向 | 方法 |
|------|------|
| **CPU** | OpenMP `schedule(dynamic)` 已是最优; 可考虑 `guided` 调度 |
| **CPU SIMD** | 射线包追踪 (Packet Tracing): 4/8条射线同时遍历BVH (需重构) |
| **GPU** | OptiX Megakernel: 端到端GPU单次发射; 或 Wavefront: 按深度分批GPU处理 |
| **CPU-GPU混合** | GPU处理射线-场景求交 (批量), CPU处理分支逻辑和Rx检测 |

**GPU接口预留** (已在ISceneAccelerator中):
```cpp
virtual std::vector<FaceHit> QueryClosestFaceHitBatch(
    const std::vector<Ray>& rays, const FaceQueryContext& ctx) const;
```
当前CPU实现逐条查询; GPU实现可批量提交。

---

## 4. Fresnel系数详解

### 4.1 法向入射上界 (当前独立sbr)

```
R = |(n1 - n2) / (n1 + n2)|²
T = 1 - R
```

- 仅在入射角=0° (垂直入射) 时精确
- 对于TE极化: 随入射角增大, R单调递增 → 法向R是上界 ✅
- 对于TM极化: 在Brewster角附近R→0 → 法向R不是上界 ❌
- **结论**: 作为功率上界估计偏保守 (低估了TM透射), 但对于剪枝目的可接受

### 4.2 逐角度全Fresnel (RT.XD / H2hRT)

```
TE:  R_TE = |(cos(θi) - √(n²-sin²θi))/(cos(θi) + √(n²-sin²θi))|²
TM:  R_TM = |(n²·cos(θi) - √(n²-sin²θi))/(n²·cos(θi) + √(n²-sin²θi))|²
平均: R = (R_TE + R_TM) / 2
```

- 任意入射角精确
- 需要复数运算 (√(ε_r - sin²θi))
- H2hRT使用 thread_local 缓存 (16材质 × 20 cosI档 = 320条目)

### 4.3 对几何寻径的影响

| 使用场景 | 法向上界 | 全Fresnel |
|---------|:------:|:------:|
| Snell方向计算 | ✅ 无影响 (仅用n=√εr) | ✅ 同样 |
| TIR判断 | ✅ 无影响 (仅用n1/n2比) | ✅ 同样 |
| 功率剪枝 | ⚠️ 偏保守 (保留更多低功率路径) | ✅ 精确剪枝 |
| 路径数 | ⚠️ 可能略多 (未及时剪除低功率透射) | ✅ 更少无用路径 |
| 计算开销 | 极低 | 中等 (可缓存) |

---

## 5. 改进路线图 (优先级排序)

### P0 — 立即实施

| 改进 | 方法 | 影响 |
|------|------|------|
| RxHashGrid恢复 | 使用Phase 5的RxHashGrid实现 (已验证正确) | 大规模Rx效率 10~37000× |
| Monte Carlo R/T模式 | `deterministic_interaction_split=false` 时使用概率选择 | Coverage模式效率 2^depth× |
| 栈大小限制 | `max_paths_per_ray` 限制每射线最大活跃状态数 | 内存安全 |

### P1 — 短期

| 改进 | 方法 |
|:-----|:-----|
| 全Fresnel系数 | 移植H2hRT的FresnelPowerReflectionCached |
| 绕射可见性验证 | 绕射出射方向用IsOccluded检查遮挡 |
| FSPL纯几何剪枝 | 新增 `use_fspl_only` 配置选项 |

### P2 — 中期

| 改进 | 方法 |
|------|------|
| 多分辨率覆盖 | CoarsePass (粗扫) → FineChannel (精扫) |
| BVH可见性预处理 | 面元PVS + 边邻接 + 角网格 |
| 自适应Rx网格 | 根据场景几何复杂度自动调整Rx密度 |

### P3 — 长期

| 改进 | 方法 |
|------|------|
| GPU后端 | OptiX批量查询 + Wavefront |
| BVH缓存 | Protobuf序列化 (从BVH_Project移植) |
| SIMD射线包 | AVX2 4-wide packet tracing |
| 绕射UTD系数 | 移植RT.XD的Holm+公式 (EM层) |

---

## 6. 文献调研汇总

| 文献 | 关键贡献 | 对本项目的指导 |
|------|---------|-------------|
| Seidel & Rappaport 1994 | SBR奠基: 射线管+接收球 | 动态接收球理论基础 |
| Durgin et al. 1997 | 3D SBR射线发射方法 | 射线管角分辨率 vs Rx球半径 |
| Yun & Iskander 2015 | SBR vs 镜像法综述 | SBR覆盖统计优势论证 |
| Fuschini et al. 2015 | 室内RT综述, SBR重新重要 | 室内场景SBR适用性 |
| Keller 1962 | Keller锥绕射条件 | d_o·e = d_i·e |
| Kouyoumjian & Pathak 1974 | UTD绕射系数 | 绕射EM计算 |
| Holm 2000 | 启发式UTD (非理想导电楔) | RT.XD绕射系数来源 |
| ITU-R P.2040-1 | 建筑材料频率依赖 | ε_r(f) 模型 |
| NVIDIA OptiX 7.7 Guide | GPU射线追踪API | GPU后端设计 |
| Wald et al. 2014 | Embree CPU射线追踪 | BVH构建+packet tracing |
| 3GPP TR 38.901 v19.2 | 标准化信道模型 | RT输出→信道指标 |
