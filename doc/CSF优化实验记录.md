# CSF 优化实验记录

本文档用于记录 CSF 项目每一次性能分析、工程改动、优化尝试、踩坑和结果。后续每做一次改动，都应追加一条实验记录，包含：改了什么、为什么改、使用了哪些参数、效果如何、是否影响兼容性、可以学习的系统/高性能知识点。

## 记录模板

```text
### 实验 N：标题

- 日期：
- 目标：
- 改动：
- 参数：
- 运行命令：
- 结果：
- 结论：
- 兼容性：
- 可学习技术点：
```

---

## 实验 0：建立 LAS 批处理与性能基线

- 日期：2026-04-29
- 目标：不再通过 txt 中转，直接读取 `data/*.las`，输出 ground / non-ground LAS，同时生成每个 LAS 和整轮的性能报告。
- 主要改动：
  - 新增 `tools/run_las_baseline.py`。
  - 使用 `laspy` 直接读取 LAS，构造 `numpy.float64` 的 `N x 3` 点云矩阵传给 CSF。
  - 输出目录结构：
    - `output/YYYYMMDD_HHMM/<las_name>/ground.las`
    - `output/YYYYMMDD_HHMM/<las_name>/non_ground.las`
    - `output/YYYYMMDD_HHMM/<las_name>/profile.json`
    - `output/YYYYMMDD_HHMM/<las_name>/profile.md`
    - `output/YYYYMMDD_HHMM/summary.csv`
    - `output/YYYYMMDD_HHMM/summary.json`
    - `output/YYYYMMDD_HHMM/summary.md`
  - 在 `setup.py` 中加入运行依赖 `numpy`、`laspy`。
  - 给 C++ 侧新增 `CSF::getLastProfileJson()`，让 Python 能拿到 C++ 模块耗时。
  - 同步更新 `python/CSF/CSF.py` 和 `python/CSF/CSF_wrap.cxx`，暴露新的 profiling API。
- 记录的 C++ 侧指标：
  - `bounding_box_ms`
  - `cloth_init_ms`
  - `rasterization_ms`
  - `simulation_ms`
  - `timestep_ms`
  - `collision_ms`
  - `postprocess_ms`
  - `classification_ms`
  - `total_filtering_ms`
- 记录的 Python / 端到端指标：
  - `read_las_ms`
  - `set_point_cloud_ms`
  - `write_las_ms`
  - `total_wall_ms`
- 兼容性：
  - 保持原有 Python `CSF.CSF()`、`setPointCloud()`、`do_filtering()` 调用方式不变。
  - 新增 API 是附加能力，不影响旧代码。
- 可学习技术点：
  - Python 调用 C++ 扩展模块。
  - SWIG 绑定和 `_CSF.so` 编译流程。
  - 端到端性能分析与核心 kernel 性能分析的区别。
  - LAS 点云读写和批处理工程化。

---

## 实验 1：修复 Python 构建和导入问题

- 日期：2026-04-29
- 目标：让 `pip install -e .` 和 `python tools/run_las_baseline.py` 能正常运行。
- 问题：
  - `setup.py`、`python/CSF/CSF.py`、`python/tests/csf_test.py` 顶部用了 C/C++ 风格 `//` 注释。
  - Python 解释器不支持 `//` 作为注释，导致 `SyntaxError`。
- 改动：
  - 将 Python 文件中的 `//` 注释改为 `#` 注释。
- 结论：
  - 这是工程兼容性问题，不是算法问题。
  - C++ 文件可以用 `//`，Python 文件必须用 `#`。
- 可学习技术点：
  - Python packaging。
  - `pip install -e .` 的 editable install 流程。
  - 源码文件类型和注释语法差异。

---

## 实验 2：默认参数下的初始 baseline

- 日期：2026-04-29
- 参数：当时未使用真实项目参数，基本沿用默认 / 非最终参数。
- 输入：
  - `1.las`：90,223,460 点
  - `2.las`：11,842,343 点
- 结果：

| File | Points | Particles | Iterations | Ground % | Total ms | Simulation ms | Raster ms | Classify ms |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| `1.las` | 90,223,460 | 649,992 | 500 | 11.31 | 37,708.847 | 11,649.205 | 1,210.690 | 1,045.812 |
| `2.las` | 11,842,343 | 13,455 | 500 | 7.45 | 3,574.916 | 838.917 | 57.735 | 81.891 |

- 分析：
  - 此时端到端耗时里有大量 LAS I/O、Python 数据构造、SWIG 传参、LAS 输出等表外开销。
  - 对 `1.las` 来说，C++ 核心三项 `simulation + raster + classification` 约 13.9s，占总耗时约 37%。
  - 由于参数不是最终真实参数，这轮主要用于验证 pipeline 和报告结构。
- 可学习技术点：
  - 端到端 profiling。
  - I/O bound 和 compute bound 的区分。
  - 点数、粒子数和耗时之间的关系。

---

## 实验 3：真实参数 `cloth_resolution=0.05` 导致内存爆炸

- 日期：2026-04-29
- 参数：

```yaml
csf:
  bSloopSmooth: false
  time_step: 0.5
  class_threshold: 0.05
  cloth_resolution: 0.05
  rigidness: 1
  interation: 500
```

- 运行现象：

```text
[0] Configuring cloth...
[0]  - width: 22203 height: 11616
terminate called after throwing an instance of 'std::bad_alloc'
  what():  std::bad_alloc
Aborted (core dumped)
```

- 分析：
  - 粒子数为 `22203 * 11616 = 257,989,248`，约 2.58 亿个粒子。
  - 当前 `Particle` 对象较重，包含多个 `Vec3`、`std::vector<Particle*> neighborsList`、`std::vector<int> correspondingLidarPointList` 等。
  - 该规模会需要几十 GB 甚至上百 GB 内存，当前数据结构无法承受。
- 结论：
  - `cloth_resolution` 是最敏感参数之一。
  - 网格是二维的，分辨率从 `0.1` 降到 `0.05`，理论粒子数约增至 4 倍；相对 `0.5` 则约增至 100 倍。
  - 完整大场景使用 `0.05` 时，后续必须考虑分块 / tile、边界 overlap、SoA 数据布局和内存压缩。
- 可学习技术点：
  - 算法复杂度与参数缩放。
  - 内存模型和 `std::bad_alloc`。
  - AoS 数据结构的内存成本。
  - 点云分块处理和边界一致性。

---

## 实验 4：真实参数 `cloth_resolution=0.1` 下的瓶颈拆解

- 日期：2026-04-29
- 参数：

```text
bSloopSmooth = false
time_step = 0.5
class_threshold = 0.05
cloth_resolution = 0.1
rigidness = 1
iterations = 500
```

- 输入规模：

| Metric | Value |
|---|---:|
| `point_count` | 11,842,343 |
| `cloth_width` | 1,122 |
| `cloth_height` | 1,138 |
| `particle_count` | 1,276,836 |
| `iterations_run` | 500 |
| `ground_count` | 2,324,272 |
| `non_ground_count` | 9,518,071 |

- 初始耗时：

| Stage | ms |
|---|---:|
| `read_las_ms` | 381.785 |
| `set_point_cloud_ms` | 94.420 |
| `bounding_box_ms` | 26.178 |
| `cloth_init_ms` | 519.116 |
| `rasterization_ms` | 828.755 |
| `simulation_ms` | 21,001.292 |
| `timestep_ms` | 19,760.878 |
| `collision_ms` | 1,239.647 |
| `classification_ms` | 289.785 |
| `write_las_ms` | 1,304.256 |
| `total_filtering_ms` | 22,674.188 |
| `total_wall_ms` | 25,687.918 |

- 分析：
  - 真实参数下，瓶颈变为 C++ 核心计算，而不是 Python I/O。
  - `simulation_ms` 占端到端约 81.8%。
  - `timestep_ms` 占端到端约 76.9%。
  - 每轮 `timestep` 约 `19,760.878 / 500 = 39.52 ms`。
- 结论：
  - 后续优化主线应集中在 `Cloth::timeStep()`。
  - 需要继续拆分 `timestep_ms`。
- 可学习技术点：
  - 热路径定位。
  - 粒子模拟性能分析。
  - 计算密集型 kernel 的 profiling 方法。

---

## 实验 5：继续细分 `timestep_ms`

- 日期：2026-04-29
- 目标：将 `timestep_ms` 拆成更细的三段：
  - `verlet_ms`
  - `constraint_ms`
  - `maxdiff_ms`
- 改动：
  - 在 `Cloth.h` 中新增 `ClothStepProfile`。
  - 将 `Cloth::timeStep()` 改为可选接收 `ClothStepProfile* profile`。
  - 在 `Cloth::timeStep()` 内分别统计：
    - Verlet 积分阶段
    - 约束满足阶段
    - 最大位移统计阶段
  - 将三个字段写入 `CSF::getLastProfileJson()`。
  - 在 `tools/run_las_baseline.py` 中将三个字段加入报告。
- 兼容性：
  - `timeStep()` 参数有默认值 `0`，旧调用仍然兼容。
  - 只新增 profiling，不改变算法逻辑。
- 细分后结果：

| Stage | ms |
|---|---:|
| `simulation_ms` | 21,319.677 |
| `timestep_ms` | 19,993.203 |
| `verlet_ms` | 2,981.228 |
| `constraint_ms` | 11,367.567 |
| `maxdiff_ms` | 5,643.932 |
| `collision_ms` | 1,325.851 |
| `total_wall_ms` | 26,074.101 |

- 分析：
  - `constraint_ms` 是第一瓶颈，占 `timestep_ms` 约 56.9%。
  - `maxdiff_ms` 是第二瓶颈，占 `timestep_ms` 约 28.2%。
  - `maxdiff` 每轮串行扫描 127 万粒子，500 轮约 6.38 亿次检查。
- 可学习技术点：
  - kernel 内部阶段拆分。
  - Reduction 统计模式。
  - OpenMP 并行化候选点识别。

---

## 实验 6：优化 `maxDiff`，使用 OpenMP reduction

- 日期：2026-04-29
- 目标：降低 `maxdiff_ms`，不改变算法结果。
- 改动位置：`src/Cloth.cpp`
- 改动内容：

```cpp
double maxDiff = 0;
#ifdef CSF_USE_OPENMP
#pragma omp parallel for reduction(max:maxDiff)
#endif
for (int i = 0; i < particleCount; i++) {
    if (particles[i].isMovable()) {
        double diff = fabs(particles[i].old_pos.f[1] - particles[i].pos.f[1]);
        if (diff > maxDiff)
            maxDiff = diff;
    }
}
```

- 兼容性：
  - 没有启用 `CSF_USE_OPENMP` 时仍然保持串行行为。
  - 只改变最大值统计方式，不改变粒子位置更新、碰撞和分类逻辑。
- 三次运行对比：

| 指标 | 原始版 | 优化后第 1 次 | 优化后第 2 次 |
|---|---:|---:|---:|
| `total_wall_ms` | 26,074.101 | 22,479.704 | 22,135.434 |
| `total_filtering_ms` | 23,024.542 | 19,624.854 | 19,224.457 |
| `simulation_ms` | 21,319.677 | 17,944.723 | 17,555.016 |
| `timestep_ms` | 19,993.203 | 16,422.794 | 16,177.728 |
| `verlet_ms` | 2,981.228 | 2,930.637 | 2,883.015 |
| `constraint_ms` | 11,367.567 | 11,417.275 | 11,278.099 |
| `maxdiff_ms` | 5,643.932 | 2,074.420 | 2,015.871 |
| `collision_ms` | 1,325.851 | 1,521.513 | 1,376.618 |
| `rasterization_ms` | 822.485 | 825.122 | 813.376 |
| `classification_ms` | 308.323 | 284.553 | 290.115 |
| `write_las_ms` | 1,360.162 | 1,292.385 | 1,337.592 |

- 效果：
  - `maxdiff_ms` 从 5,643.932 ms 降到约 2,015-2,074 ms。
  - `maxdiff` 阶段加速约 2.7-2.8x。
  - `total_wall_ms` 从 26.07s 降到约 22.14-22.48s。
  - 端到端加速约 1.16-1.18x。
- 是否偶然：
  - 不是偶然。
  - 优化后两次运行的 `maxdiff_ms` 分别为 2,074.420 ms 和 2,015.871 ms，波动约 2.8%，属于正常运行波动。
  - 分类结果完全一致：`ground_count = 2,324,272`，`non_ground_count = 9,518,071`。
- 当前新瓶颈：

| 模块 | 优化后第 2 次 ms | 占端到端 |
|---|---:|---:|
| `constraint_ms` | 11,278.099 | 50.9% |
| `verlet_ms` | 2,883.015 | 13.0% |
| `maxdiff_ms` | 2,015.871 | 9.1% |
| `collision_ms` | 1,376.618 | 6.2% |
| `write_las_ms` | 1,337.592 | 6.0% |
| `rasterization_ms` | 813.376 | 3.7% |

- 结论：
  - 第一刀优化成功。
  - 下一阶段应重点处理 `constraint_ms`。
- 可学习技术点：
  - OpenMP `reduction(max: variable)`。
  - 低风险并行化。
  - 实验重复运行和判断加速是否稳定。
  - 保持输出一致性作为正确性验证。

---

## 实验 7：新增可选 SoA 后端

- 日期：2026-04-29
- 目标：
  - 新增一个可选择的 SoA 后端，用数组替代 legacy 后端的 `std::vector<Particle>` 热路径。
  - 保留 legacy 默认路径，方便 A/B 对比和随时回退。
- 改动：
  - 新增 `src/ClothSoA.h/.cpp`。
  - 新增 `src/RasterizationSoA.h/.cpp`。
  - 新增 `src/c2cdistSoA.h/.cpp`。
  - 在 `Params` 中新增 `useSoA`，默认 `false`。
  - 在 `tools/run_las_baseline.py` 中新增 `--backend legacy|soa`，默认 `legacy`。
  - 在 profiling JSON/Markdown/CSV 中新增：
    - `backend`
    - `fallback_used`
    - `backend_fallback_reason`
  - 更新 `setup.py` 和 `src/CMakeLists.txt`，将 SoA 新文件纳入构建。
  - 更新 SWIG 生成文件和 Python wrapper，使 Python 可以设置 `csf.params.useSoA`。
- SoA 数据结构：
  - `pos_y[]`
  - `old_y[]`
  - `acceleration_y[]`
  - `movable[]`
  - `heightvals[]`
  - `nearest_height[]`
  - `tmp_dist[]`
  - `nearest_idx[]`
  - `neighbor_begin[] + neighbor_indices[]`
- 兼容策略：
  - 默认仍然使用 legacy 后端。
  - 显式传入 `--backend soa` 才启用 SoA。
  - 如果 `--backend soa` 且 `bSloopSmooth=true`，自动回退 legacy，原因记录为 `soa_postprocess_not_supported`。
  - 如果 `--backend soa` 且请求导出 cloth，自动回退 legacy，原因记录为 `soa_export_cloth_not_supported`。
- 目前状态：
  - 代码已实现。
  - 尚未在 WSL 上完成编译和性能验证。
- 需要执行的验证命令：

```bash
pip install -e . --no-build-isolation --force-reinstall

python -c "import CSF; csf=CSF.CSF(); print(csf.params.useSoA)"

python tools/run_las_baseline.py \
  --data-dir data \
  --output-dir output \
  --backend legacy \
  --no-slope-smooth \
  --time-step 0.5 \
  --class-threshold 0.05 \
  --cloth-resolution 0.1 \
  --rigidness 1 \
  --iterations 500

python tools/run_las_baseline.py \
  --data-dir data \
  --output-dir output \
  --backend soa \
  --no-slope-smooth \
  --time-step 0.5 \
  --class-threshold 0.05 \
  --cloth-resolution 0.1 \
  --rigidness 1 \
  --iterations 500
```

- 验证重点：
  - `backend` 字段是否分别为 `legacy` / `soa`。
  - `ground_count + non_ground_count == point_count`。
  - 与 legacy 相比，`constraint_ms`、`verlet_ms`、`maxdiff_ms`、`collision_ms`、`total_wall_ms` 的变化。
  - `OMP_NUM_THREADS=1` 时 legacy 与 SoA 的输出是否完全一致。
- 潜在风险：
  - SoA 第一版保留 legacy 的 Gauss-Seidel 风格并行约束，因此多线程下仍可能存在与 legacy 类似的数据竞争和非确定性。
  - SoA 光栅化不再写 `correspondingLidarPointList`，该字段在当前主流程未使用；如果某个外部逻辑依赖它，需要继续走 legacy。
  - `movableFilter()` 尚未移植到 SoA。
- 可学习技术点：
  - AoS vs SoA 数据布局。
  - 紧凑邻接表 `neighbor_begin + neighbor_indices`。
  - cache locality 和 pointer chasing。
  - 保留旧路径做 A/B test 和回退。
  - 后端开关和 profiling 元数据设计。

---

## 当前总结

截至目前，已经完成：

1. 建立直接 LAS 输入输出的 baseline pipeline。
2. 建立 C++ 分阶段 profiling。
3. 修复 Python packaging / import 的注释问题。
4. 发现 `cloth_resolution=0.05` 在整幅点云上会导致内存爆炸。
5. 用 `cloth_resolution=0.1` 建立真实参数 benchmark。
6. 将 `timeStep` 拆分为 `verlet`、`constraint`、`maxdiff`。
7. 用 OpenMP reduction 优化 `maxdiff`，获得稳定端到端约 1.16-1.18x 加速。
8. 新增可选 SoA 后端，等待 WSL 编译和 benchmark 验证。

当前稳定主瓶颈：

```text
constraint_ms
```

也就是 legacy 后端中的 `Particle::satisfyConstraintSelf()` 和 `neighborsList` 指针访问。SoA 后端的首要验证目标，就是确认紧凑邻接表能否降低这一部分开销。

## 后续建议

按风险从低到高继续推进：

1. 先验证 SoA 后端：
   - 对比 `--backend legacy` 和 `--backend soa`。
   - 记录输出一致性、`constraint_ms`、`total_wall_ms`。
2. 如果 SoA 正确且有收益，继续优化 SoA：
   - 去掉临时 `vector<vector<int>>` 构造邻接表的内存开销。
   - 进一步减少约束阶段的数据竞争。
3. 如果 SoA 收益不明显，回到低风险 legacy 内存优化：
   - 给每个粒子的 `neighborsList` 预留容量，例如 `reserve(12)`。
   - 删除或禁用 `Rasterization` 中未被后续使用的 `correspondingLidarPointList.push_back(i)`。
4. 结构性并行算法：
   - 将约束满足改为更确定、更适合并行的 stencil / red-black / Jacobi 形式。
5. GPU 阶段：
   - 先迁移 Verlet、collision、maxDiff。
   - 再迁移 constraint。
   - 最后考虑 rasterization 和分类。

每次后续实验都应追加到本文档，并记录：

- 对应代码开关或参数。
- 数据集和命令。
- 完整耗时表。
- 与上一版和 baseline 的对比。
- 是否改变输出结果。
- 学到的系统 / 高性能优化知识。
 
---

## 实验 8：三阶段优化开关系统（Memory → Deterministic SoA → CUDA）

- 日期：2026-05-16
- 目标：
  - 把后续优化做成显式 YAML 开关，方便逐阶段 A/B 对比。
  - 保留 legacy 默认路径，不传配置文件时旧命令继续可用。
  - 一旦通过 `--optimization-config` 进入阶段化优化路径，后置阶段必须要求前置阶段已经显式开启；不自动兜底。

- 新增配置文件：`configs/csf_optimization.yaml`

```yaml
optimization:
  memory_optimized: false
  deterministic_soa: false
  gpu_enabled: false

gpu:
  device_id: 0
  simulation: true
  classification: true
  rasterization: false

validation:
  strict_prerequisites: true
```

- 注意：
  - 当前 YAML 按计划保留了 `gpu.simulation=true` 和 `gpu.classification=true`。
  - 在 `strict_prerequisites=true` 下，如果 `gpu_enabled=false` 但任一 `gpu.*=true`，程序会直接报错。
  - 所以做 CPU-only 实验时，需要手动把 `gpu.simulation` 和 `gpu.classification` 改成 `false`；做 GPU 实验时，再同时开启 `memory_optimized=true`、`deterministic_soa=true`、`gpu_enabled=true`。

- C++ 参数新增：
  - `opt_memory_optimized`
  - `opt_deterministic_soa`
  - `opt_gpu_enabled`
  - `opt_gpu_simulation`
  - `opt_gpu_classification`
  - `opt_gpu_rasterization`
  - `gpu_device_id`

- Python 入口新增：

```bash
python tools/run_las_baseline.py \
  --data-dir data \
  --output-dir output \
  --optimization-config configs/csf_optimization.yaml
```

- profiling 新增字段：
  - `memory_optimized`
  - `deterministic_soa`
  - `gpu_enabled`
  - `gpu_simulation`
  - `gpu_classification`
  - `gpu_rasterization`
  - `gpu_device_id`
  - `backend_error`

### Stage 1：Memory Optimization

- legacy 路径新增 `memory_optimized` 控制：
  - `Rasterization.cpp` 中未被主流程使用的 `correspondingLidarPointList.push_back(i)` 可关闭。
  - `Cloth` 构造阶段给每个 `Particle::neighborsList` 预留容量 `reserve(12)`，减少邻接表构造时的小块内存分配。
- SoA 路径改动：
  - `ClothSoA::buildNeighbors()` 不再使用 `vector<vector<int>>` 临时邻接表。
  - 改成两遍 CSR 构造：第一遍统计邻居数，prefix sum 生成 `neighbor_begin`，第二遍写入 `neighbor_indices`。
- 预期效果：
  - 降低内存峰值。
  - 降低 `cloth_init_ms`，可能略微降低 `rasterization_ms`。
  - 不改变 legacy 算法语义。
- 当前状态：
  - 代码已实现。
  - Windows 主机缺少 Python/C++ 构建环境，尚未本地编译验证；需要在 WSL 上验证。

### Stage 2：Deterministic SoA Kernel

- 改动：
  - 当前 `ClothSoA::timeStep()` 从原先类似 Gauss-Seidel 的邻居写入方式，改为 Jacobi-style gather。
  - 每轮约束先复制 `pos_y_snapshot[]`。
  - 每个粒子只根据快照读取邻居位置，并只写自己的 `delta_y[i]`。
  - OpenMP 并行循环不再写邻居粒子，减少数据竞态，结果不依赖线程调度。
- 约束：
  - `deterministic_soa=true` 要求 `memory_optimized=true`。
  - `bSloopSmooth=true` 时直接报错：`deterministic_soa_postprocess_not_supported`。
  - 请求导出 cloth 时直接报错：`deterministic_soa_export_cloth_not_supported`。
- 影响：
  - 算法收敛路径会和 legacy/旧 SoA 不完全一样，因此分类索引可能与 legacy 不完全一致。
  - 但 deterministic SoA 自己在单线程、多线程重复运行时应保持稳定。
- 当前状态：
  - 代码已实现。
  - 需要在 WSL 上重点验证：重复运行一致性、`ground_count/non_ground_count`、与 legacy 的差异率、`constraint_ms` 变化。

### Stage 3：CUDA/GPU Backend

- 构建方式：

```bash
CSF_ENABLE_CUDA=1 pip install -e . --no-build-isolation --force-reinstall
```

- 新增文件：
  - `src/cuda/ClothCuda.cuh`
  - `src/cuda/ClothCuda.cu`
  - `src/cuda/CudaUtils.cuh`
  - `src/cuda/CudaReduction.cu`

- 第一版 GPU 支持：
  - `gpu.simulation=true`：
    - Verlet kernel
    - deterministic Jacobi constraint kernel
    - maxDiff reduction kernel
    - collision kernel
  - `gpu.classification=true`：
    - GPU 计算 ground flag。
    - flag 拷回 CPU 后再 compact 成 ground/non-ground 索引。
  - `gpu.rasterization=true`：
    - 第一版暂不支持，直接报错：`gpu_rasterization_not_supported_first_version`。

- 错误策略：
  - 未用 `CSF_ENABLE_CUDA=1` 构建却开启 GPU：直接报错。
  - CUDA 初始化、kernel、拷贝失败：直接报错，并把 `backend_error` 写入 profile。

- 当前状态：
  - CUDA 路径代码和 `setup.py` 可选 CUDA 构建逻辑已实现。
  - 尚未在 WSL/4080 SUPER 上编译和 benchmark。
  - 第一版 GPU 先使用 `double`，优先保证与 CPU deterministic SoA 可比。

### 建议验证顺序

真实参数仍使用：

```bash
--no-slope-smooth \
--time-step 0.5 \
--class-threshold 0.05 \
--cloth-resolution 0.1 \
--rigidness 1 \
--iterations 500
```

1. legacy baseline：不传 `--optimization-config`。
2. memory only：`memory_optimized=true`，其余阶段关闭，并把 `gpu.simulation=false`、`gpu.classification=false`。
3. deterministic SoA：`memory_optimized=true`、`deterministic_soa=true`，GPU 关闭。
4. GPU simulation：三阶段都开启，`gpu.simulation=true`，`gpu.classification=false`。
5. GPU classification：在上一步基础上打开 `gpu.classification=true`。

- 重点记录：
  - `cloth_init_ms`
  - `rasterization_ms`
  - `simulation_ms`
  - `constraint_ms`
  - `maxdiff_ms`
  - `collision_ms`
  - `classification_ms`
  - `total_wall_ms`
  - 输出差异率

- 可学习技术点：
  - 实验开关和 feature flag 设计。
  - 内存峰值优化与小对象分配优化。
  - CSR 邻接表。
  - Jacobi-style gather 与确定性并行。
  - OpenMP 数据竞态规避。
  - CUDA kernel 拆分、device memory 管理、host/device 数据传输。
  - GPU reduction。
  - CPU/GPU 混合 pipeline 的 profiling 和错误传播。
---

## 实验 9：补全后续优化所需的时间分析清单

- 日期：2026-05-16
- 目标：
  - 检查当前 profiling 是否足够支撑后续算法优化。
  - 确保以后每次改算法、改数据结构、改并行方式、改 GPU kernel 时，都能先看清当前瓶颈。

- 检查结果：
  - 单文件报告 `profile.json` 和 `profile.md` 已经记录完整核心耗时。
  - 整轮 `summary.csv/json` 已经记录完整字段。
  - 但整轮 `summary.md` 原来只展示 `total/simulation/raster/classify`，不够快速定位瓶颈。

- 本次补充：
  - `tools/run_las_baseline.py` 的单文件 `profile.md` 新增 `Wall-Time Share` 表。
  - `summary.md` 新增完整 `Timing Breakdown` 表。
  - `summary.md` 新增 `Wall-Time Share` 表，直接展示各阶段占端到端耗时的百分比。

- 后续每次优化前后都必须检查的时间字段：
  - `read_las_ms`：LAS 读取耗时，主要判断 IO 是否影响总时间。
  - `set_point_cloud_ms`：Python/numpy 数据进入 C++ 的耗时。
  - `bounding_box_ms`：点云包围盒计算耗时。
  - `cloth_init_ms`：布料粒子和邻接关系构造耗时。
  - `rasterization_ms`：点云到布料网格的投影和空洞填充耗时。
  - `simulation_ms`：整段布料模拟耗时。
  - `timestep_ms`：模拟中每轮 timeStep 累计耗时。
  - `verlet_ms`：Verlet 积分耗时。
  - `constraint_ms`：约束求解耗时；当前最重要的算法瓶颈。
  - `maxdiff_ms`：收敛检测耗时。
  - `collision_ms`：布料和地形碰撞处理耗时。
  - `postprocess_ms`：`bSloopSmooth=true` 时的后处理耗时。
  - `classification_ms`：点到布料面距离分类耗时。
  - `write_las_ms`：ground/non-ground LAS 写出耗时。
  - `total_filtering_ms`：C++ filtering 主流程耗时。
  - `total_wall_ms`：端到端总耗时。

- 后续每次实验还必须同时记录：
  - `backend`
  - `memory_optimized`
  - `deterministic_soa`
  - `gpu_enabled`
  - `gpu_simulation`
  - `gpu_classification`
  - `gpu_rasterization`
  - `backend_error`
  - `point_count`
  - `particle_count`
  - `iterations_configured`
  - `iterations_run`
  - `ground_count`
  - `non_ground_count`
  - `ground_ratio`

- 判断瓶颈的推荐顺序：
  1. 先看 `total_wall_ms` 是否真的下降。
  2. 再看 `Wall-Time Share` 中最大占比阶段。
  3. 如果最大阶段是 `simulation_ms`，继续看 `verlet_ms/constraint_ms/maxdiff_ms/collision_ms`。
  4. 如果最大阶段是 `rasterization_ms`，下一步应细分光栅化为点投影、最近高度更新、空洞填充。
  5. 如果最大阶段是 `classification_ms`，下一步应关注双线性插值、索引写入、CPU/GPU compact。
  6. 如果 GPU 开启后 `simulation_ms` 下降但 `total_wall_ms` 不下降，说明数据传输、compact 或 IO 可能吞掉收益，需要继续细分 GPU transfer 时间。

- 当前仍未细分但以后可能需要补的时间字段：
  - GPU host-to-device 拷贝耗时。
  - GPU device-to-host 拷贝耗时。
  - GPU kernel launch/sync 开销。
  - rasterization 的点投影、空洞填充、邻域搜索拆分。
  - classification 的插值计算、flag compact、索引写入拆分。

- 可学习技术点：
  - profiling 字段设计。
  - 端到端耗时和 kernel 耗时的区别。
  - percentage-based bottleneck analysis。
  - A/B 实验中“总时间下降”和“局部 kernel 下降”的区别。
  - GPU 优化中计算加速被数据传输抵消的常见问题。
---

## 实验 10：验证配置文件入口与本地 YAML 文件缺失问题

- 日期：2026-05-17
- 背景：
  - 用户在 WSL 中已经成功导入当前包，`import CSF` 正常。
  - 随后使用 `--optimization-config configs/verify_memory.local.yaml` 运行时出现 `FileNotFoundError`。

- 报错原因：
  - `configs/verify_memory.local.yaml` 是之前建议用于临时验证 memory-only 的本地配置文件。
  - 该文件不是仓库默认文件，也被 `.gitignore` 的 `configs/*.local.yaml` 规则忽略。
  - 因此如果没有手动创建，命令会在读取 YAML 阶段直接失败，程序还没有进入 CSF 算法流程。

- 用户采用的解决方式：
  - 改用仓库已有配置：

```bash
--optimization-config configs/csf_optimization.yaml
```

  - 用户反馈已经成功开跑。

- 需要注意：
  - `configs/csf_optimization.yaml` 的实际实验语义取决于文件里的开关。
  - 如果 `memory_optimized=false`、`deterministic_soa=false`、`gpu_enabled=false`，这是通过 YAML 入口跑 legacy baseline。
  - 如果 `memory_optimized=true` 且其他阶段关闭，这是 Stage 1 memory-only 验证。
  - `gpu_enabled=false` 时，`gpu.simulation`、`gpu.classification`、`gpu.rasterization` 必须保持 `false`，否则严格依赖检查会直接报错。

- 当前后续动作：
  - 等待用户贴出本轮 `summary.md` 或 `profile.md`。
  - 接下来应对比：
    - `backend`
    - `memory_optimized`
    - `ground_count/non_ground_count`
    - `cloth_init_ms`
    - `rasterization_ms`
    - `simulation_ms`
    - `constraint_ms`
    - `total_wall_ms`

- 可学习技术点：
  - 配置文件路径错误和算法运行错误要区分。
  - `.local.yaml` 适合本地实验，但不会随 GitHub 同步。
  - 可复现实验应该优先使用仓库内固定配置，临时实验再使用 local 配置。

---

## 实验 11：GPU 配置开启但扩展仍是 CPU-only 构建

- 日期：2026-05-17
- 背景：
  - 用户创建了 `configs/verify_gpu_sim.local.yaml`，其中：
    - `memory_optimized=true`
    - `deterministic_soa=true`
    - `gpu_enabled=true`
    - `gpu.simulation=true`
    - `gpu.classification=false`
    - `gpu.rasterization=false`
  - 运行参数使用 `cloth_resolution=0.3`、`rigidness=1`、`iterations=500`。

- 报错：

```text
RuntimeError: gpu_enabled requires building with CSF_ENABLE_CUDA=1
```

- 原因：
  - YAML 已经成功打开 GPU 路径。
  - 但是当前 WSL 环境里 `import CSF` 加载到的 `_CSF.so` 仍然是普通 CPU 构建。
  - `gpu_enabled=true` 不是运行时动态切换 CUDA；它要求安装阶段已经用 `CSF_ENABLE_CUDA=1` 编译，把 CUDA 源文件和 `CSF_ENABLE_CUDA` 宏编进扩展。

- 本次修复：
  - C++ 报错信息改为直接给出 CUDA 重新安装命令。
  - `tools/run_las_baseline.py` 增加 GPU 预检查：如果当前 `_CSF` 明显是 CPU-only 构建，会在读大 LAS 文件之前报错。
  - `tools/run_optimization_sweep.py` 增加同样的 GPU 预检查：overnight sweep 中 GPU 实验会快速记录错误并跳过，不再先读完整点云。
  - `setup.py` 在找不到 `nvcc` 时提示需要在 WSL 安装 CUDA Toolkit，避免把 `nvidia-smi` 可用误认为 CUDA 编译器可用。

- 正确修复命令：

```bash
cd ~/projects/csf-optimization
conda activate csf-baseline

rm -rf build *.egg-info python/CSF/*.egg-info

which nvcc
nvcc --version
nvidia-smi

CSF_CUDA_ARCH=sm_89 CSF_ENABLE_CUDA=1 \
  pip install -e . --no-build-isolation --force-reinstall -v
```

- 如果 `which nvcc` 没有输出：
  - 说明 WSL 里只有 NVIDIA driver/runtime，不一定有 CUDA Toolkit。
  - 需要先安装 CUDA Toolkit，或者设置 `CUDA_HOME` 指向含 `bin/nvcc` 的目录。

- 可学习技术点：
  - Python 包里的 C++/CUDA 扩展是安装时编译产物，不是每次运行自动编译。
  - `nvidia-smi` 只能说明驱动和 GPU 可见，不等价于 `nvcc` 可用。
  - 编译期宏 `CSF_ENABLE_CUDA` 决定 CUDA 代码是否被编译进 `_CSF.so`。
  - GPU 实验前要区分三层问题：YAML 配置、Python import 到的扩展版本、CUDA Toolkit/driver 环境。

---

## 实验 12：SoA 质量错误定位与 CPU SoA correctness-first 修复

- 日期：2026-05-17
- 背景：
  - 用户对比可视化结果后发现：memory-only 前后数据质量不变，但开启 SoA 后，大量植被/非地面点被错误分到 ground。
  - 这不是小幅数值误差，而是结果语义错误。
  - 因为 GPU simulation 复用了 SoA 的布料状态，所以 SoA 错误会直接传递到 GPU 结果。

- 错误现象：
  - legacy/memory-only 的 ground 点较稀疏，符合地面提取预期。
  - SoA 后 ground 点变成大片连续植被/冠层，明显误判。
  - 之前 sweep 里也能看到 ground 数量从 legacy 到 SoA 后显著变化，例如 `1.las` 从约 1469 万 ground 变成约 3477 万 ground。

- 根因：
  - 原 legacy 约束求解是原地 Gauss-Seidel 风格：

```text
for p1 in particles:
  for p2 in p1.neighbors:
    立即修改 p1
    同时反向修改 p2
```

  - 第一版 SoA 为了确定性并行，把它改成了 Jacobi/gather 风格：

```text
先复制 snapshot
每个 p1 独立累计 delta
最后一次性写回 p1
```

  - 这不是等价优化，而是换了物理求解器语义。
  - 由于布料最终形状变了，classification 自然会把大量非地面点误判成地面点。

- 本次修复：
  - 只修 CPU SoA，不修 GPU。
  - `ClothSoA::timeStep()` 的 constraint 阶段改回 legacy 等价的原地顺序更新。
  - 保持同样的粒子顺序、邻居顺序、`singleMove1/doubleMove1` 系数和 movable 处理。
  - 删除 CPU SoA 中不再使用的 `pos_y_snapshot` 和 `delta_y` 成员，避免误以为仍在使用 Jacobi 求解。
  - 这一步牺牲 constraint 阶段并行性，目标是先恢复正确性。
  - `tools/run_optimization_sweep.py` 默认关闭 GPU 两组实验，避免继续产出已知不可信的 GPU 质量结果。
  - `configs/csf_optimization.yaml` 注释已更新，说明当前 CPU SoA 是 correctness-first 路径，GPU 暂不建议用于结果产出。

- 当前 GPU 状态：
  - GPU simulation 仍然是旧的 Jacobi/gather 版本。
  - 在 GPU 版本单独修复前，不应该使用 `gpu_enabled=true` 产出质量结果。

- 当前建议配置：

```yaml
optimization:
  memory_optimized: true
  deterministic_soa: true
  gpu_enabled: false

gpu:
  simulation: false
  classification: false
  rasterization: false
```

- 验证重点：
  - 先用同一个 LAS、同一组参数分别跑：
    - memory-only legacy
    - fixed CPU SoA
  - 对比：
    - `ground_count/non_ground_count`
    - CloudCompare 可视化 ground.las
    - `constraint_ms`
    - `total_wall_ms`
  - 如果 CPU SoA 仍有明显质量差异，下一步继续查 SoA rasterization 的 hole filling 是否与 legacy 完全一致。

- 可学习技术点：
  - 数据结构 SoA 化不能顺手改变数值算法语义。
  - Gauss-Seidel 与 Jacobi 在约束求解中会产生完全不同的收敛轨迹。
  - 高性能优化必须先建立 correctness gate；否则速度提升没有意义。
  - GPU 迁移应建立在 CPU 参考实现正确之后。
