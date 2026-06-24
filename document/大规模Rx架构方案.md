# SBR 大规模Rx寻径 — 架构升级方案 v2

> 2026-06-24 | 基于当前"分离D&T/R"基线版本 | 经用户确认需求

---

## 0. 核心设计原则

1. **统一寻径机制**: P2P 和 Coverage 模式使用**完全相同的**确定性分裂DFS栈寻径。唯一的配置差异是 Rx 数量和存储策略。
2. **几何-EM 完全解耦**: 几何寻径只记录纯几何路径（`GeometricPath` + `PathNode`），不做任何功率/电场计算。Fresnel上界仅用于几何层剪枝。
3. **绕射与R/T分离**: 绕射独立寻径（`RunDiffraction`），不与R/T混合，结果合并到最终路径集。
4. **全量路径存储**: 两种模式均保存每个Rx的完整几何路径，为后续H2hRT合并、精细化Rx参数分析等提供完整数据。

---

## 1. 统一寻径架构

```
SbrEngine::Run(config, scene, matDb, tx, rxGrid)
  │
  ├─ [1] 一次性准备
  │     ├─ Fibonacci 射线发射 (N条)
  │     ├─ RxHashGrid 构建 (O(N_rx))
  │     └─ RunDiffraction 预计算 (可选)
  │
  ├─ [2] 单次射线追踪 (所有Rx共享)
  │     #pragma omp parallel for schedule(dynamic,1)
  │     for each ray:
  │       while alive:
  │         ├─ BVH 求交
  │         ├─ RxHashGrid.Query(curPt, segEnd, r) → [rx1, rx2, ...]
  │         ├─ for each rx in hits:
  │         │   ├─ 遮挡验证 (Rx在面背面→跳过)
  │         │   └─ 记录路径到 per-thread per-rx 收集器
  │         └─ R/T 确定性分裂 → push后继状态
  │
  ├─ [3] 合并 + 绕射合并
  │     ├─ 合并所有线程的 per-rx 收集器
  │     ├─ 合并 RunDiffraction 结果
  │     └─ per-Rx 后处理 (去重/相似剪枝/TopN)
  │
  └─ [4] 输出
        ├─ JSON (小规模) 或 二进制流 (大规模)
        └─ CSV 摘要
```

**与当前架构的关键差异**: 射线只追踪一次，所有Rx共享。当前架构对每个Rx单独调用 `RunPointToPoint`，导致 O(N_rx) 倍冗余追踪。

---

## 2. 大规模存储与内存管理

### 2.1 问题量化

| 场景 | Rx数 | 路径/Rx | PathNode/路径 | 总数据量 | 内存可行性 |
|------|:----:|:------:|:------------:|---------|:---------:|
| P2P | 5 | 1k | 3 | ~1.5 MB | ✅ |
| 会议室覆盖 | 100 | 1k | 3 | ~30 MB | ✅ |
| 楼层覆盖 | 10k | 1k | 3 | ~3 GB | ⚠️ 临界 |
| 宏站覆盖 | 100k | 10k | 5 | ~500 GB | ❌ 不可行 |
| 全城覆盖 | 1M | 100k | 5 | ~50 TB | ❌ 不可行 |

### 2.2 分级存储策略

```
┌─────────────────────────────────────────────────────────┐
│  Layer 1: 内存热存储 (N_rx ≤ 1000)                      │
│  ─────────────────────────────────────                   │
│  全量路径存储在内存 vector<GeometricPath> per-Rx        │
│  路径上限: max_paths_per_rx (默认 1000)                 │
│  后处理: 全局去重 → 相似剪枝 → TopN                     │
│  输出: JSON (可读) + CSV                                │
├─────────────────────────────────────────────────────────┤
│  Layer 2: 内存 + 磁盘混合 (1000 < N_rx ≤ 100k)          │
│  ─────────────────────────────────────                   │
│  在线维护 per-Rx Top-K (默认 100) 在内存                │
│  完整路径批量刷盘: 每 1000 Rx × 100 路径 → 临时文件     │
│  后处理: 分块去重 → 合并 → TopN                         │
│  输出: 二进制流 (.sbrbin) + CSV 摘要                    │
├─────────────────────────────────────────────────────────┤
│  Layer 3: 全磁盘流式 (N_rx > 100k)                      │
│  ─────────────────────────────────────                   │
│  每个线程写入独立临时文件                                │
│  射线追踪完成后: 多路归并 → per-Rx 去重 → 最终文件      │
│  后处理: 外部排序 + 合并去重                             │
│  输出: 二进制分块 (.sbrbin.part-XXXX) + 元数据JSON      │
└─────────────────────────────────────────────────────────┘
```

### 2.3 高效二进制存储格式 (.sbrbin)

```
文件头:
  magic:    4B  "SBR\0"
  version:  4B  uint32
  rx_count: 4B  uint32
  reserved: 4B

Rx索引区:
  rx_offset[0]:  8B  uint64 (指向第一个Rx数据)
  rx_offset[1]:  8B  uint64
  ...
  rx_offset[N]:  8B  uint64 (共 N+1 个, 最后一个指向文件尾)

每个Rx的数据块:
  rx_index:      4B  uint32
  rx_x, rx_y, rx_z: 3×8B double
  path_count:    4B  uint32
  paths[]:       变长
    每个 path:
      path_len:     4B  uint32 (字节数, 用于跳过)
      total_length: 8B  double
      node_count:   2B  uint16
      is_los:       1B  uint8
      nodes[]:      变长
        每个 node:
          interaction_type: 1B uint8
          face_id:          4B int32 (有符号, -1=无效)
          wedge_id:          4B int32
          point_x/y/z:      3×8B double
          direction_x/y/z:  3×8B double (出射方向)
          incident_x/y/z:   3×8B double (入射方向)
          surface_n_x/y/z:  3×8B double
          seg_length:       8B  double

估算: 每个 PathNode ≈ 100B, 每条路径(平均3节点) ≈ 320B
      10k Rx × 1k 路径 × 320B ≈ 3.2 GB (可接受)
```

**优势 vs JSON**:
- 体积: JSON的 ~1/3 (二进制无冗余)
- 读取: 可随机访问单个Rx (通过offset索引)
- 流式: 可顺序写入无需全部在内存

### 2.4 内存管理策略

```cpp
// Per-thread per-Rx 在线收集器
struct RxCollector {
    // 在线维护 Top-K (堆, 固定内存)
    std::priority_queue<ScoredPath> topK_heap;  // 始终 ≤ max_paths_per_rx
    std::unordered_set<uint64_t> signatures;    // per-Rx 去重
    
    // 触发刷盘阈值
    static constexpr size_t FLUSH_THRESHOLD = 10000;  // 路径数超过此值刷盘
    
    // 刷盘: 将 topK 序列化写入临时文件, 清空内存
    void FlushToTemp(const std::string& tempDir, int rxIdx);
};

// 每 1000 个 Rx 触发一次全局检查
if (totalPaths > MEMORY_BUDGET) {
    FlushColdRxCollectors();  // 最不活跃的 Rx 先刷盘
}
```

---

## 3. 全局去重策略

### 3.1 问题

当前 `PostProcess` 在每个 `RunPointToPoint` 调用中对**单个Rx**做去重。新架构中所有Rx共享射线追踪，可能出现**跨Rx的重复路径**（同一条射线路径命中多个Rx时，对不同Rx产生相同的路径签名）。

### 3.2 方案

```
Phase 1: 在线 per-Rx 去重 (追踪阶段)
  └─ 每个 RxCollector 维护自己的 signature set
  └─ 发现重复 → 保留功率最高者 (或首次命中者)

Phase 2: 跨Rx全局去重 (后处理阶段, 可选)
  └─ 仅在 FineChannel 模式启用
  └─ 对相邻Rx (空间距离 < rx_spacing/2) 做跨Rx去重
  └─ 避免相邻Rx因接收球重叠而产生完全相同的路径

Phase 3: 最终 per-Rx TopN (后处理)
  └─ 按功率代理分数排序
  └─ 截断到 path_top_n_per_rx
```

### 3.3 大规模去重优化

```
方案A (内存): 全局 unordered_set<uint64_t> (N_rx > 100k 时内存爆炸)
方案B (分块): Rx 按空间分块, 块内去重, 跨块不做 (绝大多数重复在块内)
方案C (bloom): 用 Bloom Filter 快速排除一定不重复的, 减少哈希查询
方案D (延迟): Coverage 模式不做去重, 仅在 FineChannel 模式做 per-Rx 去重

推荐: B + D 组合
  - Coverage 模式: per-Rx 在线去重
  - FineChannel 模式: 空间分块 per-Rx 去重
  - 均不跨Rx全局去重 (接收球半径 << Rx间距时重复概率极低)
```

---

## 4. 并行 I/O 策略

### 4.1 写阶段

```
┌────────────────────────────────────────────┐
│  Thread 0  │  Thread 1  │  ... │ Thread N  │
│  ──────────│──────────│─────│────────── │
│  独立临时文件 per-thread                   │
│  temp/t0_part0.sbrbin                     │
│  temp/t1_part0.sbrbin                     │
│  ...                                       │
│  (无锁, 无竞争, 顺序写入)                   │
└────────────────────────────────────────────┘
         │           │              │
         └───────────┴──────────────┘
                     │
              [主线程合并]
         多路归并 → 按 Rx 分组 → 最终文件
```

### 4.2 合并算法

```
Input: N 个线程临时文件, 每个文件按 rx_index 有序
Output: 单个 .sbrbin 文件

1. 打开 N 个文件, 各读取第一个 Rx 块
2. 建立 N-路最小堆 (按 rx_index)
3. 弹出最小 rx_index, 合并该 Rx 的所有路径
4. 对该 Rx 做后处理 (去重/TopN)
5. 写入最终文件
6. 从对应线程文件读取下一个 Rx 块
7. 重复直到所有文件读完
```

---

## 5. 配置设计

```json
{
  "scene": { "obj_file": "...", "material_map_file": "...", "material_csv_file": "..." },
  "sbr": {
    "center_frequency_hz": 2.4e9,
    "ray_count": 2000000,
    "max_ray_depth": 6,
    "max_reflection_count": 3,
    "max_transmission_count": 1,
    "max_diffraction_count": 1,
    "diffraction_rays_per_event": 8,
    "ray_power_threshold_dB": -180.0,
    "tx_power_dBm": 0.0
  },
  "receiver": {
    "rx_sphere_radius_m": 0.3,
    "enable_dynamic_rx_radius": true,
    "ray_tube_radius_scale": 0.5,
    "ray_tube_min_radius_m": 0.1,
    "ray_tube_max_radius_m": 2.0
  },
  "postprocess": {
    "enable_path_dedup": true,
    "enable_path_similarity_pruning": false,
    "path_similarity_length_tol_m": 0.05,
    "path_top_n_per_rx": 100,
    "store_paths": true
  },
  "storage": {
    "mode": "auto",
    "max_memory_mb": 4096,
    "binary_output": true,
    "temp_dir": "output/temp"
  },
  "tx": { "x": 10.0, "y": 2.0, "z": -2.0 },
  "rx_mode": "grid",
  "rx_grid": {
    "x_min": 3.0, "x_max": 17.0, "x_step": 1.0,
    "y_min": 0.5, "y_max": 2.5, "y_step": 2.0,
    "z_min": -18.0, "z_max": -2.0, "z_step": 1.0
  },
  "output": {
    "result_json": "output/sbr_result.json",
    "result_bin": "output/sbr_result.sbrbin"
  }
}
```

**关键差异 vs 当前配置**:
- `rx_mode: "grid"` 支持3D网格自动生成Rx
- `storage.mode: "auto"` 根据 Rx 数量自动选择存储层级
- `storage.binary_output: true` 大规模时使用二进制格式
- `path_top_n_per_rx: 100` Coverage 模式也保留路径

---

## 6. 实施计划

### Phase A: 架构重构 — Single-Pass 追踪 (2天)

**目标**: 射线只追踪一次，所有Rx共享

- [ ] RxHashGrid 移植（从 Phase5 版本）
- [ ] `RxCollector` 数据结构（per-thread per-rx）
- [ ] 重构 `RunCoverage` 为 single-pass
- [ ] 移除 per-Rx 外循环
- [ ] 保持 `RunPointToPoint` 不变（P2P模式仍可用）
- [ ] 自测: 100 Rx Coverage 结果与 per-Rx 版本一致

### Phase B: 二进制存储 (1天)

**目标**: 高效存储大规模结果

- [ ] `.sbrbin` 二进制格式读写
- [ ] 分级存储自动选择
- [ ] 临时文件合并逻辑
- [ ] 自测: 10k Rx 写入/读取正确性

### Phase C: 大规模优化 (1.5天)

**目标**: 100k+ Rx 可运行

- [ ] per-thread 临时文件并行写入
- [ ] 多路归并去重
- [ ] 内存预算控制 + 自动刷盘
- [ ] 性能基准: 100k Rx × 1M rays < 120s

### Phase D: 配置与集成 (0.5天)

**目标**: 用户友好

- [ ] JSON 配置扩展 (`storage` / `rx_grid`)
- [ ] Python 可视化适配大规模结果
- [ ] 文档更新

---

## 7. 风险评估

| 风险 | 缓解 |
|------|------|
| RxHashGrid 内存爆炸 (100k Rx时cell数过大) | cellSize自适应; 超50M格截断; 稀疏网格优化 |
| per-Rx path 内存爆炸 (每个Rx 10k+路径) | 在线Top-K + 自动刷盘; max_paths_per_rx硬限制 |
| 线程合并时磁盘I/O成为瓶颈 | per-thread独立文件无竞争; SSD顺序写入 >500MB/s |
| 二进制格式版本兼容 | magic + version字段; 向后兼容读取 |
| 绕射单独路径与single-pass不一致 | 绕射路径数少(O(wedges×Rx)), 单独合并不影响架构 |

---

## 8. 与H2hRT接口对齐

新架构的输出 `SbrCoverageResult` 与 H2hRT 完全兼容:

```cpp
struct SbrCoverageResult {
    bool succeeded;
    std::string trace_profile;
    int total_rays, active_rx_count;
    std::vector<RxCoverageRecord> rx_records;
    // ... 诊断字段 (40+ fields, v10)
};

struct RxCoverageRecord {
    Point3 rx_position;
    int rx_index;
    double total_power_linear, total_power_dBm;
    int ray_hit_count;
    std::vector<GeometricPath> paths;  // ★ 新架构也会填充
};
```

合并到 H2hRT 时: 直接替换 `rt::SbrEngine::Run()` 调用为 `sbr::SbrEngine::RunCoverage()`，输出结构布局兼容，零拷贝转换。
