# 0-1 OpenMP：从 CSF 的 `maxDiff` 优化开始

这篇笔记不是为了继续优化代码，而是用 `doc/CSF优化实验记录.md` 里的真实实验，理解三个问题：

1. OpenMP 是什么。
2. 它为什么能加速。
3. 以后看到别的循环时，如何判断能不能举一反三迁移。

---

## 1. 先从你的真实实验开始

在实验记录里，最关键的一段路径是：

```text
实验 4：真实参数 cloth_resolution=0.1 下，瓶颈变成 C++ 核心计算
实验 5：继续拆分 timestep_ms，发现 maxdiff_ms 是第二瓶颈
实验 6：用 OpenMP reduction 优化 maxDiff
```

真实参数下，输入规模是：

| 指标 | 数值 |
|---|---:|
| `point_count` | 11,842,343 |
| `cloth_width` | 1,122 |
| `cloth_height` | 1,138 |
| `particle_count` | 1,276,836 |
| `iterations_run` | 500 |

也就是说，CSF 布料模拟里有约 127 万个粒子，并且主循环要跑 500 轮。

实验 5 里把 `Cloth::timeStep()` 拆成了三段：

| 阶段 | 耗时 ms |
|---|---:|
| `verlet_ms` | 2,981.228 |
| `constraint_ms` | 11,367.567 |
| `maxdiff_ms` | 5,643.932 |

其中 `maxdiff_ms` 做的事情很朴素：每一轮模拟结束后，扫一遍所有粒子，找出本轮 Y 方向移动量最大的粒子，用它判断布料是否已经稳定。

原始逻辑可以理解成：

```cpp
double maxDiff = 0;

for (int i = 0; i < particleCount; i++) {
    if (particles[i].isMovable()) {
        double diff = fabs(particles[i].old_pos.f[1] - particles[i].pos.f[1]);
        if (diff > maxDiff) {
            maxDiff = diff;
        }
    }
}
```

这个循环每轮扫描约 127 万个粒子。500 轮下来，约等于：

```text
1,276,836 * 500 = 638,418,000
```

也就是约 6.38 亿次检查。

实验 6 改成：

```cpp
double maxDiff = 0;

#ifdef CSF_USE_OPENMP
#pragma omp parallel for reduction(max : maxDiff)
#endif
for (int i = 0; i < particleCount; i++) {
    if (particles[i].isMovable()) {
        double diff = fabs(particles[i].old_pos.f[1] - particles[i].pos.f[1]);
        if (diff > maxDiff)
            maxDiff = diff;
    }
}
```

结果是：

| 指标 | 原始版 | OpenMP 后第 2 次 |
|---|---:|---:|
| `total_wall_ms` | 26,074.101 | 22,135.434 |
| `simulation_ms` | 21,319.677 | 17,555.016 |
| `timestep_ms` | 19,993.203 | 16,177.728 |
| `maxdiff_ms` | 5,643.932 | 2,015.871 |

换算一下：

```text
maxdiff 阶段加速 = 5643.932 / 2015.871 = 2.80x
端到端加速       = 26074.101 / 22135.434 = 1.18x
```

这个例子非常适合学习 OpenMP，因为它满足两个特点：

1. 循环次数很多，值得并行。
2. 每次循环之间几乎没有相互依赖，迁移风险低。

---

## 2. OpenMP 是什么

OpenMP 是一种面向多核 CPU 的共享内存并行编程接口。

它不是一个新算法，也不是 GPU 编程模型。它更像是给 C/C++/Fortran 编译器的一组提示：

```cpp
#pragma omp parallel for
for (int i = 0; i < n; i++) {
    work(i);
}
```

这句的意思是：

```text
这个 for 循环的很多次迭代可以同时做，请编译器和运行时把它分给多个 CPU 线程。
```

普通串行循环像这样：

```text
线程 0：i = 0
线程 0：i = 1
线程 0：i = 2
线程 0：i = 3
...
线程 0：i = n - 1
```

OpenMP 并行循环像这样：

```text
线程 0：i = 0      到 i = 319,999
线程 1：i = 320,000 到 i = 639,999
线程 2：i = 640,000 到 i = 959,999
线程 3：i = 960,000 到 i = 1,276,835
```

具体怎么切分不一定完全是上面这样，但核心思想就是：把大量相似、独立的工作分摊给多个 CPU 核。

在当前项目里，OpenMP 是通过两个地方接入的：

```cmake
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    add_compile_definitions("CSF_USE_OPENMP")
endif()
```

以及 Python 扩展构建里的编译参数：

```python
if platform.system() == "Windows":
    openmp_args = ["/openmp", "/std:c++11"]
    openmp_macro = [("CSF_USE_OPENMP", None)]
elif platform.system() == "Linux":
    openmp_args = ["-fopenmp", "-std=c++11"]
    openmp_linking_args = ["-fopenmp"]
    openmp_macro = [("CSF_USE_OPENMP", None)]
```

所以源码里常见这种写法：

```cpp
#ifdef CSF_USE_OPENMP
#pragma omp parallel for
#endif
for (...) {
    ...
}
```

这是一种兼容性写法：如果编译环境支持 OpenMP，就启用并行；如果不支持，代码仍然按普通串行循环编译运行。

---

## 3. OpenMP 为什么能加速

加速的本质不是 `#pragma` 本身神奇，而是它把一份大工作拆给多个 CPU 核同时做。

以 `maxDiff` 为例，串行版本是：

```text
一个线程扫描 1,276,836 个粒子
```

如果有 8 个线程，理想情况可以变成：

```text
线程 0 扫描一部分粒子，得到局部最大值 localMax0
线程 1 扫描一部分粒子，得到局部最大值 localMax1
线程 2 扫描一部分粒子，得到局部最大值 localMax2
...
线程 7 扫描一部分粒子，得到局部最大值 localMax7

最后再合并：
maxDiff = max(localMax0, localMax1, ..., localMax7)
```

这就是 `reduction(max : maxDiff)` 做的事情。

`reduction` 可以理解成：

```text
不要让多个线程同时抢着改同一个 maxDiff。
每个线程先用自己的私有 maxDiff。
线程结束后，OpenMP 按 max 规则把这些私有值合并成最终 maxDiff。
```

如果没有 `reduction`，直接这样并行是错的：

```cpp
#pragma omp parallel for
for (int i = 0; i < particleCount; i++) {
    double diff = ...;
    if (diff > maxDiff) {
        maxDiff = diff;
    }
}
```

问题在于多个线程会同时读写同一个 `maxDiff`。这叫数据竞争。

可能发生这种情况：

```text
线程 A 看到 maxDiff = 10，准备写 20
线程 B 看到 maxDiff = 10，准备写 15
线程 A 写入 20
线程 B 后写入 15

最终 maxDiff 变成 15，但正确答案应该是 20
```

`reduction(max : maxDiff)` 正是为这种“先各算各的，最后合并”的模式准备的。

---

## 4. 为什么 `maxDiff` 是低风险 OpenMP 候选点

判断一个循环能不能并行，核心不是看它是不是 `for`，而是看每一次迭代之间有没有互相影响。

`maxDiff` 循环里，每次迭代做的是：

```cpp
if (particles[i].isMovable()) {
    double diff = fabs(particles[i].old_pos.f[1] - particles[i].pos.f[1]);
    ...
}
```

它有几个好特征：

1. 第 `i` 次循环只读取 `particles[i]`。
2. 它不修改 `particles[i]`。
3. 它不修改 `particles[i + 1]` 或其他粒子。
4. 唯一需要合并的是 `maxDiff`，而最大值合并正好可以用 `reduction(max : maxDiff)`。

所以它很像一个数学上的归约问题：

```text
从一堆元素中计算一个汇总结果。
```

常见 reduction 类型包括：

| 目标 | OpenMP 写法 | 例子 |
|---|---|---|
| 求和 | `reduction(+ : sum)` | 总距离、总误差、总能量 |
| 求最大值 | `reduction(max : maxValue)` | 最大位移、最大高度 |
| 求最小值 | `reduction(min : minValue)` | 最近距离、最低高度 |
| 计数 | `reduction(+ : count)` | 满足条件的点数 |

你的实验验证了这个判断：

```text
ground_count     = 2,324,272
non_ground_count = 9,518,071
```

优化前后分类数量完全一致，说明这次并行没有改变结果。

---

## 5. 为什么 `maxdiff` 加速 2.8x，但总耗时只加速 1.18x

这是学习性能优化时非常重要的一点：局部加速不等于整体同比例加速。

实验 5 中：

```text
total_wall_ms = 26,074.101
maxdiff_ms    = 5,643.932
```

`maxdiff` 占端到端比例是：

```text
5643.932 / 26074.101 = 21.6%
```

这意味着即使你把 `maxdiff` 优化到 0 ms，理论上端到端最多也只能省掉 21.6% 的时间。

它的理论上限大约是：

```text
最大端到端加速 = 1 / (1 - 0.216) = 1.28x
```

实际实验中，`maxdiff` 没有变成 0，而是从 5,643.932 ms 变成 2,015.871 ms，所以端到端从 26.07s 变成约 22.14s，是合理的。

这个规律就是 Amdahl 定律的直观版本：

```text
整体加速受限于你优化部分在总耗时里的占比。
```

所以性能优化要先 profiling。否则你可能把一个只占 1% 的函数优化 10 倍，整体也几乎看不出来。

---

## 6. 不是所有循环都适合直接加 OpenMP

对比 `maxDiff`，当前最大瓶颈是：

```text
constraint_ms
```

它对应的是：

```cpp
particles[j].satisfyConstraintSelf(constraint_iterations);
```

继续看 `Particle::satisfyConstraintSelf()`，核心逻辑是：

```cpp
Particle *p1 = this;

for (std::size_t i = 0; i < neighborsList.size(); i++) {
    Particle *p2 = neighborsList[i];
    Vec3 correctionVector(0, p2->pos.f[1] - p1->pos.f[1], 0);

    if (p1->isMovable() && p2->isMovable()) {
        Vec3 correctionVectorHalf = ...;
        p1->offsetPos(correctionVectorHalf);
        p2->offsetPos(-correctionVectorHalf);
    }
    ...
}
```

这个循环和 `maxDiff` 的最大区别是：它不只是读，还会写粒子位置。

更麻烦的是，它可能写邻居：

```cpp
p2->offsetPos(...)
```

假设两个线程同时处理相邻粒子：

```text
线程 A 正在处理粒子 10，它要修改粒子 11
线程 B 正在处理粒子 11，它也要修改粒子 11
```

这就是数据竞争。

所以 `constraint` 阶段虽然耗时更高，但它不是一个干净的“加一行 `#pragma omp parallel for` 就完事”的学习样例。

这也是为什么你的实验记录里把后续建议分成了不同风险级别：

| 方向 | 风险 | 原因 |
|---|---|---|
| `maxDiff` reduction | 低 | 只读粒子，最后合并最大值 |
| `neighborsList reserve` | 低 | 改内存分配，不改算法结果 |
| 固定网格 offset 替代指针列表 | 中 | 改访问方式，需要确认边界 |
| AoS 改 SoA | 高 | 改数据布局，影响面广 |
| constraint 改 stencil / red-black / Jacobi | 高 | 改数值迭代方式，正确性要重新验证 |

这里要记住一个原则：

```text
越是写共享数据的循环，越不能机械迁移 OpenMP。
```

---

## 7. 举一反三：迁移 OpenMP 的判断流程

以后看到一个耗时循环，可以按这个流程判断。

### 第一步：先确认它真的是瓶颈

不要凭感觉优化。先问：

```text
这个循环占总耗时多少？
它在真实参数、真实数据下是否仍然耗时？
它是一次慢，还是会被重复调用很多次？
```

你的 `maxDiff` 之所以值得做，是因为：

```text
一次 timeStep 扫描 127 万粒子
总共 500 轮
maxdiff_ms 占 timestep_ms 约 28.2%
maxdiff_ms 占 total_wall_ms 约 21.6%
```

### 第二步：看每次迭代是否独立

安全的形态通常是：

```cpp
for (int i = 0; i < n; i++) {
    out[i] = f(in[i]);
}
```

每个线程写不同的 `out[i]`，通常比较安全。

低风险 reduction 形态是：

```cpp
for (int i = 0; i < n; i++) {
    sum += f(in[i]);
}
```

可以迁移成：

```cpp
#pragma omp parallel for reduction(+ : sum)
for (int i = 0; i < n; i++) {
    sum += f(in[i]);
}
```

高风险形态是：

```cpp
for (int i = 0; i < n; i++) {
    out[index[i]] += value;
}
```

因为多个 `i` 可能写到同一个 `out[index]`。

更高风险形态是：

```cpp
for (int i = 0; i < n; i++) {
    particles[i].update();
    particles[neighbor[i]].update();
}
```

这就是 `constraint` 类问题。

### 第三步：列出读写表

迁移前，强迫自己写一个表：

| 数据 | 当前循环读不读 | 当前循环写不写 | 是否多个线程会写同一个位置 |
|---|---|---|---|
| `particles[i].pos` | 是 | 否 | 否 |
| `particles[i].old_pos` | 是 | 否 | 否 |
| `maxDiff` | 是 | 是 | 是，但可用 reduction |

这是 `maxDiff` 的读写表，所以它安全。

如果是 `constraint`，表会变成：

| 数据 | 当前循环读不读 | 当前循环写不写 | 是否多个线程会写同一个位置 |
|---|---|---|---|
| `p1->pos` | 是 | 是 | 可能 |
| `p2->pos` | 是 | 是 | 可能 |
| `neighborsList` | 是 | 否 | 否 |

这个表一写出来，就能看出风险高很多。

### 第四步：选择 OpenMP 工具

常用工具可以先记这几个。

#### 1. 普通并行循环

适合每次迭代互不影响：

```cpp
#pragma omp parallel for
for (int i = 0; i < n; i++) {
    out[i] = f(in[i]);
}
```

#### 2. reduction

适合求和、最大值、最小值、计数：

```cpp
double maxValue = 0;

#pragma omp parallel for reduction(max : maxValue)
for (int i = 0; i < n; i++) {
    maxValue = std::max(maxValue, values[i]);
}
```

#### 3. atomic

适合非常简单的共享变量更新：

```cpp
#pragma omp parallel for
for (int i = 0; i < n; i++) {
    if (flag[i]) {
        #pragma omp atomic
        count++;
    }
}
```

但如果能用 `reduction(+ : count)`，通常优先用 reduction，因为它减少线程争抢。

#### 4. critical

适合保护一小段不能同时执行的代码：

```cpp
#pragma omp parallel for
for (int i = 0; i < n; i++) {
    Result r = compute(i);

    #pragma omp critical
    {
        results.push_back(r);
    }
}
```

`critical` 很容易让线程排队，可能并不快。它更像正确性兜底，不是首选性能方案。

### 第五步：验证正确性和性能

OpenMP 迁移后，至少做三类验证：

1. 输出是否一致。
2. 耗时是否稳定下降。
3. 多跑几次是否有明显波动。

你的实验里做得好的地方是：不仅看了耗时，还看了分类数量是否一致。

```text
ground_count     = 2,324,272
non_ground_count = 9,518,071
```

这比只看“快了”更可靠。

---

## 8. 一个可复用的 OpenMP 迁移模板

以后可以直接照这个模板写实验记录。

```text
1. 目标循环在哪里？
   文件：
   函数：
   循环：

2. 它占多少耗时？
   总耗时：
   当前阶段耗时：
   占比：

3. 循环每次迭代读什么？
   读：

4. 循环每次迭代写什么？
   写：

5. 是否存在多个线程写同一个变量或同一个数组元素？
   是 / 否：

6. 如果有共享写，能不能改成 reduction？
   能 / 不能：

7. 如果不能，是否需要 atomic、critical、分块、双缓冲、red-black 或算法重构？
   方案：

8. 改动是否有编译开关保护？
   例如 CSF_USE_OPENMP：

9. 正确性验证指标是什么？
   例如 ground_count、non_ground_count、输出文件 hash、误差范围：

10. 性能验证怎么跑？
    数据集：
    参数：
    重复次数：
    对比表：
```

对 `maxDiff` 来说，答案是：

```text
1. 目标循环：
   src/Cloth.cpp 的 Cloth::timeStep()，统计 maxDiff 的 for 循环。

2. 耗时：
   maxdiff_ms = 5,643.932 ms，占 total_wall_ms 约 21.6%。

3. 读：
   particles[i].movable
   particles[i].old_pos
   particles[i].pos

4. 写：
   maxDiff

5. 共享写：
   有，多个线程都会更新 maxDiff。

6. 能否 reduction：
   能，因为目标是求最大值。

7. 使用方案：
   #pragma omp parallel for reduction(max : maxDiff)

8. 编译开关：
   CSF_USE_OPENMP

9. 正确性验证：
   ground_count 和 non_ground_count 完全一致。

10. 性能结果：
   maxdiff_ms 从 5,643.932 ms 降到 2,015.871 ms。
```

---

## 9. 学习时要避开的几个误区

### 误区 1：只要是 for 循环就能并行

错。

真正要看的是循环之间有没有数据依赖。`maxDiff` 可以，`constraint` 就很危险。

### 误区 2：加了 OpenMP 一定更快

错。

OpenMP 也有开销：创建线程、分配任务、同步、合并结果。如果循环很小，开销可能比收益还大。

你的例子能快，是因为：

```text
粒子多，循环大，重复 500 轮，并且循环主体简单清晰。
```

### 误区 3：局部快 10 倍，整体也能快 10 倍

错。

整体加速受限于瓶颈占比。`maxdiff` 占总耗时约 21.6%，所以它再怎么优化，端到端也不可能靠这一项变成 10 倍。

### 误区 4：结果差不多就行

要看场景。

如果只是可视化预览，微小数值差异可能能接受。如果是分类结果、工程生产结果，就要保留验证指标。你的实验用 `ground_count` 和 `non_ground_count` 验证，是一个好的开始。

---

## 10. 回到 CSF：下一步应该怎么学

从学习角度，不建议一上来就硬啃 `constraint_ms` 的并行重构。可以按这个顺序学：

1. 先完全掌握 `maxDiff` 这种 reduction。
2. 再看 `Verlet` 这种每个粒子独立更新的并行循环。
3. 再看 `terrCollision` 这种按粒子独立碰撞的并行循环。
4. 最后再研究 `constraint` 这种带邻居写入的数据竞争问题。

对应难度大致是：

| 模块 | 并行难度 | 学习重点 |
|---|---|---|
| `maxDiff` | 低 | reduction |
| `Verlet` | 低 | 独立数组元素更新 |
| `terrCollision` | 中 | 条件分支、粒子状态更新 |
| `classification` | 中 | 输出容器、索引写入、计数 |
| `constraint` | 高 | 邻居依赖、数据竞争、算法重构 |

这条学习路线比直接追求最高加速更稳，因为它能逐步建立判断力。

---

## 11. 一句话总结

OpenMP 的核心不是“给循环加一句 pragma”，而是：

```text
找到真实瓶颈，证明循环迭代之间足够独立，把共享结果改成安全合并，再用实验验证结果一致且耗时稳定下降。
```

你的 `maxDiff` 实验正好是一个标准样例：

```text
profiling 找到热点
读写关系证明可并行
reduction 解决共享 maxDiff
CSF_USE_OPENMP 保持兼容
重复实验确认加速稳定
输出数量确认结果不变
```

以后迁移别的地方时，不要先问“能不能加 OpenMP”，而要先问：

```text
每个线程会不会同时写同一份数据？
如果会，我有没有一个数学上正确的合并方式？
如果没有，这可能就不是简单 OpenMP 的问题，而是数据结构或算法设计问题。
```
