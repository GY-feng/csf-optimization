# CSF 优化项目新线程交接文档

更新时间：2026-04-30

本文档用于把当前长对话中的项目背景、已完成实现、性能实验、踩坑、当前代码状态和后续目标交接给新的线程或新的 agent。新线程接手时，建议先阅读本文档，再阅读 `doc/CSF优化实验记录.md`。

---

## 1. 项目背景

用户希望借 CSF（Cloth Simulation Filter）项目学习偏系统、AI infra、GPU 编程、高性能优化方向的知识，并将其包装成适合申请系统/HPC/AI infra 方向硕士的项目材料。

项目路径：

```text
E:\项目\codex\CSF-master-codex优化
```

用户真实执行环境不是当前 Windows 主机，而是 WSL：

```text
~/projects/CSF-master-codex优化
```

WSL 机器有 RTX 4080 SUPER，但当前 Codex 只能在 Windows 主机上修改代码，用户会把代码同步到 WSL 后自行编译和运行。

当前核心目标：

1. 建立可复现 baseline。
2. 直接读取 LAS 点云，不再经由 txt 中转。
3. 输出 ground / non-ground LAS。
4. 记录每个 LAS 和整轮任务的性能报告。
5. 逐步分析瓶颈并优化 C++ 核心路径。
6. 后续进一步推进 CPU 数据结构优化、SoA、并行算法、GPU 化。

---

## 2. 当前数据与真实参数

用户原始数据目录在 WSL 项目根目录下：

```text
data/
```

程序输出目录：

```text
output/YYYYMMDD_HHMM/
```

当前主要测试数据：

```text
2.las
```

约 400 MB，点数：

```text
11,842,343
```

用户一开始有一个约 2.8 GB 的大 LAS，但后来删除了。真实项目参数如下：

```yaml
csf:
  bSloopSmooth: false
  time_step: 0.5
  class_threshold: 0.05
  cloth_resolution: 0.05
  rigidness: 1
  interation: 500
```

注意：仓库代码里参数拼写是 `interations`，脚本命令行里使用 `--iterations`。

由于 `cloth_resolution=0.05` 会让粒子数暴涨，在完整大范围点云上出现过 `std::bad_alloc`。当前实际 benchmark 主要使用：

```text
cloth_resolution = 0.1
```

该参数下 `2.las` 的布料网格规模：

```text
cloth_width = 1122
cloth_height = 1138
particle_count = 1,276,836
```

---

## 3. 当前运行方式

推荐在 WSL 中使用单独 conda 环境：

```bash
conda create -n csf-baseline python=3.11 -y
conda activate csf-baseline

pip install -U pip setuptools wheel
pip install numpy laspy
pip install -e . --no-build-isolation --force-reinstall
```

如果要读取 `.laz`，额外安装：

```bash
pip install lazrs
```

每次修改 C++ 后都需要重新编译：

```bash
pip install -e . --no-build-isolation --force-reinstall
```

确认 Python 当前加载的模块路径：

```bash
python -c "import CSF; print(CSF.__file__)"
python -c "import _CSF; print(_CSF.__file__)"
```

legacy 后端运行命令：

```bash
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
```

SoA 后端运行命令：

```bash
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

---

## 4. 已实现的功能

### 4.1 LAS 批处理 baseline

新增文件：

```text
tools/run_las_baseline.py
```

功能：

- 默认读取 `data/*.las`。
- 用 `laspy` 直接读取 LAS。
- 构造 `numpy.float64` 的 `N x 3` 点云矩阵传给 CSF。
- 调用现有 Python/SWIG 绑定。
- 输出 ground / non-ground LAS。
- 生成单 LAS 和整轮性能报告。

输出结构：

```text
output/YYYYMMDD_HHMM/
  summary.csv
  summary.json
  summary.md
  <las_name>/
    ground.las
    non_ground.las
    profile.json
    profile.md
```

注意：

- 输出 LAS 保留原始 LAS header 和 point format。
- 用原始点索引切分 `las.points`。
- 目前只默认匹配 `.las`，不匹配 `.laz`。

### 4.2 C++ profiling API

在 C++ 中新增：

```cpp
std::string CSF::getLastProfileJson() const;
```

Python 中可调用：

```python
profile = json.loads(csf.getLastProfileJson())
```

profiling 字段包括：

```text
backend
fallback_used
backend_fallback_reason
point_count
cloth_width
cloth_height
particle_count
iterations_configured
iterations_run
ground_count
non_ground_count
ground_ratio
cloth_resolution
rigidness
time_step
class_threshold
bSloopSmooth
bounding_box_ms
cloth_init_ms
rasterization_ms
simulation_ms
timestep_ms
verlet_ms
constraint_ms
maxdiff_ms
collision_ms
postprocess_ms
classification_ms
total_filtering_ms
```

Python 脚本额外记录：

```text
read_las_ms
set_point_cloud_ms
write_las_ms
total_wall_ms
```

### 4.3 `timeStep` 内部细分

将原来的 `timestep_ms` 拆分为：

```text
verlet_ms
constraint_ms
maxdiff_ms
```

涉及文件：

```text
src/Cloth.h
src/Cloth.cpp
src/CSF.h
src/CSF.cpp
tools/run_las_baseline.py
```

新增结构：

```cpp
struct ClothStepProfile {
    double verlet_ms;
    double constraint_ms;
    double maxdiff_ms;
};
```

### 4.4 OpenMP reduction 优化 `maxDiff`

在 `src/Cloth.cpp` 中，将 `maxDiff` 串行统计改为 OpenMP reduction：

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

这是目前已经验证有效的优化。

### 4.5 可选 SoA 后端

新增文件：

```text
src/ClothSoA.h
src/ClothSoA.cpp
src/RasterizationSoA.h
src/RasterizationSoA.cpp
src/c2cdistSoA.h
src/c2cdistSoA.cpp
```

构建文件已更新：

```text
setup.py
src/CMakeLists.txt
```

Python/SWIG 已更新，使 Python 能设置：

```python
csf.params.useSoA = True
```

命令行开关：

```bash
--backend legacy
--backend soa
```

SoA 后端设计：

- `pos_y[]`
- `old_y[]`
- `acceleration_y[]`
- `movable[]`
- `visited[]`
- `heightvals[]`
- `nearest_height[]`
- `tmp_dist[]`
- `nearest_idx[]`
- `neighbor_begin[] + neighbor_indices[]`

SoA 第一版只支持：

```text
bSloopSmooth = false
exportCloth = false
```

如果用户请求 SoA，但 `bSloopSmooth=true`，自动回退 legacy：

```text
backend_fallback_reason = "soa_postprocess_not_supported"
```

如果用户请求 SoA 且导出 cloth，也自动回退 legacy：

```text
backend_fallback_reason = "soa_export_cloth_not_supported"
```

SoA 当前状态：

- 代码已实现。
- 尚未在 WSL 完成编译和性能验证。
- 新线程首要任务应是编译、跑小数据、修编译错误、验证正确性和性能。

---

## 5. 已遇到的问题和原因

### 5.1 `setup.py` / `CSF.py` 的 `//` 注释导致 Python 报错

曾出现：

```text
SyntaxError: invalid character '—' (U+2014)
```

根因不是 `—` 本身，而是 Python 文件使用了 C/C++ 风格：

```python
// comment
```

修复：

- `setup.py`
- `python/CSF/CSF.py`
- `python/tests/csf_test.py`

中的非法 `//` 注释已改为 `#`。

### 5.2 `cloth_resolution=0.05` 导致 `std::bad_alloc`

当使用完整大范围点云和 `cloth_resolution=0.05` 时，曾出现：

```text
[0] Configuring cloth...
[0]  - width: 22203 height: 11616
terminate called after throwing an instance of 'std::bad_alloc'
```

粒子数：

```text
22203 * 11616 = 257,989,248
```

当前 legacy `Particle` 对象很重，里面包含多个 `Vec3`、`std::vector<Particle*>`、`std::vector<int>` 等，2.58 亿粒子会消耗几十 GB 甚至上百 GB 内存。

结论：

- `cloth_resolution=0.05` 不能直接整幅大场景跑。
- 后续若必须使用 0.05，需要做 tile / block 分块处理，带 overlap。
- SoA 可以降低单粒子内存，但 2.58 亿粒子仍然非常大，分块仍然重要。

### 5.3 OpenMP 并不是所有地方都安全

安全的例子：

- `maxDiff` reduction，只读粒子状态，求最大值。

危险的例子：

- `Particle::satisfyConstraintSelf()` 会同时修改自己和邻居：

```cpp
p1->offsetPos(...)
p2->offsetPos(...)
```

并行时存在数据竞争。legacy 代码本身已经对 constraint 阶段用了 OpenMP，但严格来说结果可能依赖线程调度。SoA 第一版保留这种语义，后续若要严肃优化，需要切换到 Jacobi / red-black / stencil pass 等更确定的并行策略。

---

## 6. 已有性能结果

测试数据：

```text
2.las
point_count = 11,842,343
cloth_resolution = 0.1
rigidness = 1
time_step = 0.5
class_threshold = 0.05
bSloopSmooth = false
iterations = 500
particle_count = 1,276,836
```

### 6.1 细分 `timeStep` 后的原始结果

| Stage | ms |
|---|---:|
| `simulation_ms` | 21,319.677 |
| `timestep_ms` | 19,993.203 |
| `verlet_ms` | 2,981.228 |
| `constraint_ms` | 11,367.567 |
| `maxdiff_ms` | 5,643.932 |
| `collision_ms` | 1,325.851 |
| `classification_ms` | 308.323 |
| `total_wall_ms` | 26,074.101 |

结论：

- `constraint_ms` 是第一瓶颈。
- `maxdiff_ms` 是第二瓶颈。
- `maxDiff` 串行扫描 127 万粒子，500 轮，开销很大。

### 6.2 OpenMP reduction 优化后第 1 次

| Stage | ms |
|---|---:|
| `simulation_ms` | 17,944.723 |
| `timestep_ms` | 16,422.794 |
| `verlet_ms` | 2,930.637 |
| `constraint_ms` | 11,417.275 |
| `maxdiff_ms` | 2,074.420 |
| `collision_ms` | 1,521.513 |
| `classification_ms` | 284.553 |
| `total_wall_ms` | 22,479.704 |

### 6.3 OpenMP reduction 优化后第 2 次

| Stage | ms |
|---|---:|
| `simulation_ms` | 17,555.016 |
| `timestep_ms` | 16,177.728 |
| `verlet_ms` | 2,883.015 |
| `constraint_ms` | 11,278.099 |
| `maxdiff_ms` | 2,015.871 |
| `collision_ms` | 1,376.618 |
| `classification_ms` | 290.115 |
| `total_wall_ms` | 22,135.434 |

### 6.4 优化结论

`maxdiff_ms`：

```text
5643.932 ms -> 2015.871~2074.420 ms
```

`maxDiff` 阶段加速约：

```text
2.7x~2.8x
```

端到端加速约：

```text
1.16x~1.18x
```

分类结果保持一致：

```text
ground_count = 2,324,272
non_ground_count = 9,518,071
```

当前稳定主瓶颈：

```text
constraint_ms
```

---

## 7. 关键文件说明

### C++ 主流程

```text
src/CSF.h
src/CSF.cpp
```

职责：

- 参数管理。
- 点云输入坐标转换。
- legacy / SoA 后端分支。
- profiling 聚合。
- `getLastProfileJson()`。

### legacy 布料模拟

```text
src/Cloth.h
src/Cloth.cpp
src/Particle.h
src/Particle.cpp
```

职责：

- `std::vector<Particle>`。
- `Particle::neighborsList`。
- Verlet。
- constraint。
- maxDiff。
- collision。
- `movableFilter()` 后处理。

### legacy 光栅化和分类

```text
src/Rasterization.h
src/Rasterization.cpp
src/c2cdist.h
src/c2cdist.cpp
```

### SoA 后端

```text
src/ClothSoA.h
src/ClothSoA.cpp
src/RasterizationSoA.h
src/RasterizationSoA.cpp
src/c2cdistSoA.h
src/c2cdistSoA.cpp
```

状态：

- 已写入。
- 需要 WSL 编译验证。

### Python 批处理

```text
tools/run_las_baseline.py
```

职责：

- LAS 输入输出。
- 设置 CSF 参数。
- 选择 backend。
- 写报告。

### Python/SWIG

```text
python/CSF/CSF.py
python/CSF/CSF_wrap.cxx
python/CSF/CSF.i
```

注意：

- 当前直接修改了已生成的 `CSF_wrap.cxx` 和 `CSF.py`，因为没有要求重新跑 SWIG。
- 如果后续重新运行 SWIG，需要把 `useSoA` 和 `getLastProfileJson()` 确认同步进生成物。

### 文档

```text
doc/CSF优化实验记录.md
doc/CSF项目新线程交接文档.md
```

---

## 8. 新线程接手后的建议步骤

### 步骤 1：编译验证

在 WSL 中：

```bash
conda activate csf-baseline
pip install -e . --no-build-isolation --force-reinstall
```

如果编译报错，优先检查：

- 新增 SoA 文件是否被 `setup.py` 编译。
- `Params::useSoA` 是否在 SWIG wrapper 里正确访问。
- 是否有 C++11 不支持的写法。

### 步骤 2：确认 Python 参数

```bash
python -c "import CSF; csf=CSF.CSF(); print(csf.params.useSoA)"
```

预期：

```text
False
```

### 步骤 3：跑小数据或当前 `2.las`

先跑 legacy：

```bash
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
```

再跑 SoA：

```bash
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

### 步骤 4：对比结果

重点看：

```text
backend
fallback_used
backend_fallback_reason
ground_count
non_ground_count
ground_ratio
cloth_init_ms
rasterization_ms
simulation_ms
timestep_ms
verlet_ms
constraint_ms
maxdiff_ms
collision_ms
classification_ms
total_wall_ms
```

正确性优先：

- `ground_count + non_ground_count == point_count`
- legacy 和 SoA 的 ground/non-ground 数量是否一致。
- 如果设置 `OMP_NUM_THREADS=1`，尽量检查索引是否完全一致。

性能其次：

- 第一版 SoA 的目标是降低 `constraint_ms`。
- 如果 `constraint_ms` 未下降，需要分析是邻接表构造、数组访问、OpenMP 数据竞争，还是 cache 行为导致。

---

## 9. 后续优化路线

### 9.1 SoA 编译和正确性修复

这是最近优先级最高的工作。

如果 SoA 编译失败，先修编译。

如果 SoA 输出不一致：

- 先用 `OMP_NUM_THREADS=1` 比较。
- 检查邻接顺序是否和 legacy 的 `makeConstraint()` 顺序一致。
- 检查 `time_step2` 是否重复乘了平方。
- 检查 `addForceY(-gravity * time_step2)` 是否和 legacy `Vec3(0, -gravity, 0) * time_step2` 对齐。
- 检查 collision 是否只在 movable 时 offset，但始终标记 unmovable。当前 SoA 已按 legacy 语义处理。

### 9.2 SoA 性能优化

当前 SoA 构造邻接表时用了临时：

```cpp
std::vector<std::vector<int>>
```

这只是第一版，容易实现但内存不最优。后续可以直接两遍构造：

1. 第一遍统计每个粒子的 neighbor count。
2. prefix sum 得到 `neighbor_begin`。
3. 第二遍直接写 `neighbor_indices`。

这样可以减少大量小 vector 分配。

### 9.3 约束算法重构

legacy 和当前 SoA 都保留了 Gauss-Seidel 风格并行约束，存在数据竞争。

后续更严肃的优化路线：

- Jacobi：先计算 correction，再统一 apply。
- Red-black：棋盘格分组更新。
- Directional stencil pass：水平、垂直、对角、二阶约束分方向处理。

目标：

- 减少数据竞争。
- 提升可复现性。
- 更适合 GPU。

### 9.4 分块处理

如果用户必须用：

```text
cloth_resolution = 0.05
```

则需要做 tile 分块：

- 按 XY 平面切块。
- 每块带 overlap。
- 只取中心区域输出。
- 处理边界一致性。

这是支持大范围高分辨率点云的关键。

### 9.5 GPU 化

建议顺序：

1. Verlet。
2. collision。
3. maxDiff reduction。
4. c2cdist。
5. constraint。
6. rasterization。

constraint 是最难的，因为有数据依赖和写冲突。GPU 版本最好基于 Jacobi / red-black / stencil pass，而不是当前 Gauss-Seidel 写邻居的形式。

---

## 10. 重要注意事项

1. 用户希望后续新增/改变的东西都尽量兼容旧操作。
2. 默认路径应保持 legacy，除非显式打开新后端。
3. 每次优化都要记录到 `doc/CSF优化实验记录.md`。
4. 记录内容应包括：
   - 做了什么。
   - 为什么做。
   - 参数。
   - 运行命令。
   - 性能结果。
   - 是否正收益。
   - 是否踩坑。
   - 可学习的系统/HPC/AI infra 技术点。
5. 当前 Windows 主机无法编译验证，因为本地没有 `python/g++/cmake`。
6. 用户在 WSL 上执行后会贴错误或性能报告，新线程应根据实际输出继续修。

---

## 11. 给新线程的第一句话建议

如果新线程接手，建议先回应用户：

```text
我已经读取交接文档。当前代码已实现 legacy baseline、profiling、maxDiff OpenMP reduction 和可选 SoA 后端；SoA 尚未在 WSL 编译验证。下一步我会优先根据你的 WSL 编译输出修复 SoA 编译/运行问题，然后对比 legacy 与 SoA 的 correctness 和 constraint_ms。
```
---

## 12. 2026-05-17 最新验证状态

### 12.1 当前用户正在做什么

用户已经在 WSL 环境中重新安装并成功导入当前项目：

```bash
python -c "import CSF; csf=CSF.CSF(); print('CSF import ok')"
```

输出为：

```text
CSF import ok
```

这说明 Python 包和 C++ 扩展至少已经可以被加载。

### 12.2 最新一次命令问题

用户原本运行：

```bash
python tools/run_las_baseline.py \
  --data-dir data \
  --output-dir output \
  --optimization-config configs/verify_memory.local.yaml \
  --no-slope-smooth \
  --time-step 0.5 \
  --class-threshold 0.05 \
  --cloth-resolution 0.1 \
  --rigidness 1 \
  --iterations 500
```

报错：

```text
FileNotFoundError: No such file or directory:
/home/zjf/projects/csf-optimization/configs/verify_memory.local.yaml
```

这个不是 CSF 算法错误，也不是编译错误，只是命令里引用的 YAML 文件不存在。

### 12.3 用户已经采用的解决方式

用户采用了“解决方式 1”：直接使用仓库已有配置文件：

```bash
--optimization-config configs/csf_optimization.yaml
```

并且已经成功开跑。

下一线程不要再围绕 `verify_memory.local.yaml` 排查，除非用户明确说要创建 local 配置文件。

### 12.4 当前配置语义

需要查看用户 WSL 中的实际文件：

```bash
cat configs/csf_optimization.yaml
```

然后按文件内容判断当前实验阶段：

```yaml
optimization:
  memory_optimized: false
  deterministic_soa: false
  gpu_enabled: false

gpu:
  simulation: false
  classification: false
  rasterization: false
```

表示：通过 YAML 入口跑 legacy baseline，所有优化关闭。

```yaml
optimization:
  memory_optimized: true
  deterministic_soa: false
  gpu_enabled: false

gpu:
  simulation: false
  classification: false
  rasterization: false
```

表示：只验证 Stage 1 memory optimization，仍走 CPU legacy 主流程。

### 12.5 下一线程接手重点

用户下一步大概率会贴出最新输出目录中的：

```text
output/<timestamp>/summary.md
output/<timestamp>/<las_name>/profile.md
```

接手时优先检查：

```text
backend
memory_optimized
deterministic_soa
gpu_enabled
backend_error
point_count
ground_count
non_ground_count
cloth_init_ms
rasterization_ms
simulation_ms
constraint_ms
maxdiff_ms
collision_ms
classification_ms
write_las_ms
total_wall_ms
```

判断规则：

1. 如果当前是 all-false 配置，则它是 YAML 入口下的 legacy baseline，用来确认新配置系统不破坏旧流程。
2. 如果 `memory_optimized=true` 且其他阶段关闭，则重点比较 legacy baseline 和 memory-only 的 `cloth_init_ms`、`rasterization_ms`、`total_wall_ms`。
3. `ground_count + non_ground_count` 必须等于 `point_count`。
4. memory-only 理论上不应改变分类语义；如果数量变化，需要优先排查 memory optimization 是否误删了被使用的数据。
5. 如果用户贴的是 deterministic SoA 或 GPU 结果，再按实验记录中的 Stage 2/3 策略分析。

### 12.6 关于 local YAML 文件

`.gitignore` 已忽略：

```text
configs/*.local.yaml
configs/*.local.yml
```

所以 `verify_memory.local.yaml` 这类本地实验配置不会上传 GitHub。服务器或 WSL 上如果需要这些文件，要手动创建。

当前最稳妥的验证命令是：

```bash
python tools/run_las_baseline.py \
  --data-dir data \
  --output-dir output \
  --optimization-config configs/csf_optimization.yaml \
  --no-slope-smooth \
  --time-step 0.5 \
  --class-threshold 0.05 \
  --cloth-resolution 0.1 \
  --rigidness 1 \
  --iterations 500
```

## 13. 2026-05-17 GPU CPU-only 构建报错处理

用户开启 GPU 配置后遇到：

```text
RuntimeError: gpu_enabled requires building with CSF_ENABLE_CUDA=1
```

结论：这不是 YAML 错，也不是 CUDA kernel 已经运行失败；这是当前 `import CSF` 加载到的 `_CSF.so` 仍是 CPU-only 构建。GPU 开关要求安装阶段就把 CUDA 源码编进扩展。

已做的修复：

1. `src/CSF.cpp` 的报错信息改为给出可直接执行的 CUDA 重编译命令。
2. `tools/run_las_baseline.py` 增加 GPU 预检查，避免先读几百 MB/GB LAS 后才发现 `_CSF.so` 不是 CUDA 构建。
3. `tools/run_optimization_sweep.py` 增加同样的预检查，GPU 实验会快速失败并记录错误，CPU 实验不受影响。
4. `setup.py` 找不到 `nvcc` 时明确提示需要 WSL CUDA Toolkit。

用户在 WSL 上应执行：

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

如果 `which nvcc` 没输出，下一线程应先指导用户安装 CUDA Toolkit 或设置 `CUDA_HOME`。`nvidia-smi` 可用只代表驱动/GPU 可见，不代表能编译 CUDA。

## 14. 2026-05-17 SoA 质量错误与 CPU SoA 修复状态

用户可视化对比发现：memory-only 结果质量不变，但 SoA 后大量植被/非地面点被分成 ground。结论是之前 SoA/GPU 加速结果不能作为正确质量结果使用。

根因：

```text
legacy: 原地 Gauss-Seidel 约束求解，处理 p1 时会立即修改 p1 和 p2
旧 SoA: snapshot + delta 的 Jacobi/gather 求解，只写自己的 delta
```

这不是等价 SoA 化，而是改变了布料模拟求解语义，导致布料最终形状改变，classification 也随之错误。

已做修复：

1. 只修 CPU SoA，GPU 暂不修。
2. `src/ClothSoA.cpp` 中 `ClothSoA::timeStep()` 的 constraint 阶段改回 legacy 等价的原地顺序更新。
3. 保持粒子顺序、邻居顺序、`singleMove1/doubleMove1` 和 movable 分支一致。
4. 移除 CPU SoA 不再使用的 `pos_y_snapshot` / `delta_y` 成员。
5. `tools/run_optimization_sweep.py` 默认关闭 GPU 两组实验，避免继续产出已知不可信的 GPU 质量结果。
6. `configs/csf_optimization.yaml` 注释已更新，标明当前 CPU SoA 是 correctness-first 路径，GPU 暂不建议用于结果产出。

重要注意：

```text
gpu_enabled=true 仍然不能用于质量结果
```

因为 GPU simulation 代码还保留旧 Jacobi/gather 语义，后续需要单独修。

下一步验证命令建议：

```bash
cd ~/projects/csf-optimization
conda activate csf-baseline
pip install -e . --no-build-isolation --force-reinstall
```

然后先跑 memory-only legacy，再跑 fixed CPU SoA。SoA 验证配置应为：

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

验收重点：

1. `ground_count/non_ground_count` 是否接近 memory-only legacy。
2. CloudCompare 中 ground.las 是否不再包含大片植被。
3. `ground_count + non_ground_count == point_count`。
4. 如果仍有明显差异，下一步优先检查 `RasterizationSoA::findHeightValByNeighbor()` 的 hole filling 是否和 legacy 完全一致。
