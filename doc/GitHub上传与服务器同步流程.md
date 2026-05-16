# GitHub 上传与服务器同步流程

本文档用于把当前 CSF 优化项目上传到 GitHub，并在服务器或 WSL 机器上拉取、构建、运行。核心原则是：GitHub 只放代码、配置和 Markdown 文档，不放点云数据、输出结果、编译产物。

## 1. 上传前检查

确认以下内容可以提交：

- `src/`：C++/CUDA 源码。
- `python/CSF/`：SWIG Python 包装代码。
- `tools/`：批处理和 profiling 脚本。
- `configs/`：实验开关配置，例如 `configs/csf_optimization.yaml`。
- `doc/`：Markdown 文档和实验记录。
- `setup.py`、`pyproject.toml`、`CMakeLists.txt`、`README.md`。

确认以下内容不要提交：

- `data/`：真实 LAS/LAZ 点云数据。
- `output/`：每轮运行生成的 ground/non-ground LAS 和报告。
- `build/`、`dist/`、`*.egg-info/`：Python/C++ 构建产物。
- `*.las`、`*.laz`：大点云文件。
- `__pycache__/`、`.pytest_cache/`、`.venv/`、`.vscode/` 等本地缓存或环境目录。

这些规则已经写入根目录 `.gitignore`。

## 2. 第一次上传到 GitHub

在项目根目录执行：

```bash
git init
git status
git add .
git status
git commit -m "Initialize CSF optimization project"
```

在 GitHub 新建一个空仓库，例如：

```text
https://github.com/<your_name>/csf-optimization.git
```

然后绑定远程仓库并推送：

```bash
git branch -M main
git remote add origin https://github.com/<your_name>/csf-optimization.git
git push -u origin main
```

如果仓库已经存在远程地址，先检查：

```bash
git remote -v
```

如果需要替换远程地址：

```bash
git remote set-url origin https://github.com/<your_name>/csf-optimization.git
```

## 3. 后续本地更新代码

每次改完代码后，建议按这个顺序：

```bash
git status
git add src tools configs doc setup.py pyproject.toml CMakeLists.txt README.md .gitignore
git status
git commit -m "Describe the optimization change"
git push
```

不要直接 `git add .`，除非你先确认 `git status` 里没有误加数据文件或输出文件。

如果想查看哪些文件被忽略：

```bash
git status --ignored
```

如果某个文件已经被 Git 跟踪，后来才加入 `.gitignore`，忽略规则不会自动移除它。需要只从 Git 索引中移除，不删除本地文件：

```bash
git rm --cached path/to/file
git commit -m "Stop tracking generated file"
```

## 4. 服务器或 WSL 拉取代码

在服务器上选择一个目录：

```bash
mkdir -p ~/projects
cd ~/projects
git clone https://github.com/<your_name>/csf-optimization.git
cd csf-optimization
```

后续同步最新代码：

```bash
cd ~/projects/csf-optimization
git pull
```

如果服务器上也改了代码，先看状态：

```bash
git status
```

不要在服务器上直接覆盖未提交改动。需要保留的话先提交，或者另开分支。

## 5. 服务器安装与构建

建议使用单独 conda 环境：

```bash
conda create -n csf-baseline python=3.11 -y
conda activate csf-baseline
pip install -U pip setuptools wheel
pip install numpy laspy
```

CPU/OpenMP 版本：

```bash
pip install -e . --no-build-isolation --force-reinstall
```

CUDA 版本：

```bash
CSF_ENABLE_CUDA=1 pip install -e . --no-build-isolation --force-reinstall
```

RTX 4080 SUPER 对应 Ada 架构，必要时指定：

```bash
CSF_CUDA_ARCH=sm_89 CSF_ENABLE_CUDA=1 pip install -e . --no-build-isolation --force-reinstall
```

构建后快速检查：

```bash
python -c "import CSF; csf=CSF.CSF(); print(csf.getLastProfileJson())"
```

## 6. 数据文件处理

不要把真实点云数据上传到 GitHub。服务器上单独创建 `data/`：

```bash
mkdir -p data
```

把 LAS/LAZ 文件用 `scp`、`rsync`、移动硬盘或服务器已有数据目录放进去：

```bash
rsync -av /path/to/local/data/ user@server:/path/to/csf-optimization/data/
```

运行后结果会写入 `output/`，该目录也不进入 Git：

```bash
python tools/run_las_baseline.py \
  --data-dir data \
  --output-dir output \
  --no-slope-smooth \
  --time-step 0.5 \
  --class-threshold 0.05 \
  --cloth-resolution 0.1 \
  --rigidness 1 \
  --iterations 500
```

使用三阶段优化配置：

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

## 7. 实验结果如何保存

大文件不要进 Git：

- `output/**/ground.las`
- `output/**/non_ground.las`
- 原始 `data/*.las`

建议只把关键结果手动整理进 Markdown：

- `doc/CSF优化实验记录.md`
- 新增的实验分析文档

如果需要长期保存完整输出，把 `output/` 压缩后放到服务器数据盘、网盘、对象存储或实验归档目录，不要放 GitHub。

## 8. 推荐分支策略

稳定主线：

```bash
main
```

每个优化方向单独开分支：

```bash
git checkout -b opt-memory
git checkout -b opt-deterministic-soa
git checkout -b opt-cuda
```

完成并验证后再合并回 `main`：

```bash
git checkout main
git merge opt-memory
git push
```

## 9. 上传前最终检查清单

提交前必须看：

```bash
git status
```

确认不应该出现：

- `data/`
- `output/`
- `*.las`
- `*.laz`
- `build/`
- `.venv/`
- `__pycache__/`
- 大型二进制实验结果

确认应该出现：

- 源码改动。
- 配置改动。
- 文档记录。
- `.gitignore` 更新。

如果发现误加大文件，先取消暂存：

```bash
git restore --staged path/to/large-file.las
```

如果文件已经提交过，不要继续 push，先处理提交历史或重新提交干净版本。
