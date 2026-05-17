# overnight sweep 脚本说明

新增脚本：

```text
tools/run_optimization_sweep.py
```

这个脚本用于无人值守地依次验证多个优化变体。它不读取 YAML，不依赖 `configs/*.yaml`。所有 CSF 参数和实验开关都直接写在脚本顶部，运行前改脚本即可。

## 运行方式

在 WSL 项目根目录执行：

```bash
python tools/run_optimization_sweep.py
```

## 需要在脚本里配置的位置

打开：

```text
tools/run_optimization_sweep.py
```

顶部有这些配置：

```python
DATA_DIR = Path("data")
OUTPUT_DIR = Path("output")
INPUT_PATTERNS = ("*.las",)

CSF_PARAMS = {
    "bSloopSmooth": False,
    "time_step": 0.5,
    "class_threshold": 0.05,
    "cloth_resolution": 0.1,
    "rigidness": 1,
    "iterations": 500,
}

WRITE_LAS_OUTPUTS = True
```

如果只是想比较性能，不想每个变体都写一份 `ground.las/non_ground.las`，可以改成：

```python
WRITE_LAS_OUTPUTS = False
```

这样能明显减少磁盘占用和 LAS 写出时间。

## 默认执行的实验

脚本默认依次执行 CPU 可信实验：

1. `00_legacy_baseline`
   所有优化关闭，作为基线。

2. `01_memory_only`
   只开启 `memory_optimized`，仍然走 legacy 后端。

3. `02_deterministic_soa`
   开启 memory + CPU SoA。当前 SoA 约束阶段已改为 correctness-first 的 legacy 等价顺序求解。

4. `03_gpu_simulation`
   GPU simulation，classification 仍在 CPU。当前默认 `enabled=False`，因为 GPU 约束求解仍待质量修复。

5. `04_gpu_simulation_classification`
   GPU simulation + GPU classification。当前默认 `enabled=False`，因为 GPU 约束求解仍待质量修复。

如果后续手动把 GPU 两组改回 `enabled=True`，但当前不是 CUDA 构建，GPU 两组会报错并被记录，然后脚本继续执行，不会影响前面的 CPU 实验。当前阶段不建议用 GPU 结果做质量判断。

## 输出目录

脚本会生成：

```text
output/sweep_YYYYMMDD_HHMM/
```

里面包括：

```text
sweep_manifest.json
sweep_summary.json
sweep_summary.csv
sweep_summary.md
error_log.json
error_log.md
00_legacy_baseline/
01_memory_only/
02_deterministic_soa/
```

如果手动打开 GPU 两组，才会额外生成：

```text
03_gpu_simulation/
04_gpu_simulation_classification/
```

每个实验目录下面，每个 LAS 会有自己的子目录：

```text
output/sweep_YYYYMMDD_HHMM/<experiment>/<las_name>/
  profile.json
  profile.md
  ground.las
  non_ground.las
  error.txt
```

其中 `ground.las/non_ground.las` 只有在 `WRITE_LAS_OUTPUTS=True` 时才会写出。`error.txt` 只有失败时才会出现。

## 报错处理策略

脚本会捕获单个实验/单个 LAS 的异常：

- 记录到该 LAS 的 `profile.json/profile.md`
- 记录到 `error_log.json`
- 继续执行下一个 LAS 或下一个实验

能捕获的典型错误：

- 配置依赖错误
- GPU 未编译但开启 GPU
- CUDA runtime 错误
- Python/laspy 读取或写出错误
- C++ 抛出的普通异常

不能保证捕获的情况：

- 进程被系统 kill
- native 层直接 abort
- 机器内存耗尽导致进程退出

所以 overnight 跑之前，建议继续用当前安全参数：

```python
"cloth_resolution": 0.1
```

不要直接改回 `0.05` 跑整幅大点云。

## 醒来后优先看什么

先打开：

```text
output/sweep_YYYYMMDD_HHMM/sweep_summary.md
```

重点看：

```text
status
experiment
backend
memory_optimized
deterministic_soa
gpu_enabled
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
error_message
```

如果有错误，先看：

```text
output/sweep_YYYYMMDD_HHMM/error_log.md
```

## 给后续线程的接手提示

如果用户贴出 sweep 结果，优先比较：

- `00_legacy_baseline` vs `01_memory_only`
  看 `cloth_init_ms`、`rasterization_ms`、`total_wall_ms`，并确认分类数量是否一致。

- `01_memory_only` vs `02_deterministic_soa`
  看 `constraint_ms` 是否下降，并记录分类数量差异。deterministic SoA 改变了约束求解语义，和 legacy 不一定完全一致。

- GPU 组如果报错
  先看是否没有用 `CSF_ENABLE_CUDA=1` 重新编译。

- GPU 组如果成功
  重点看 `simulation_ms`、`classification_ms` 和 `total_wall_ms`。如果局部 kernel 快了但总时间没快，下一步要细分 GPU 数据传输时间。
