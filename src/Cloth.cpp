// ======================================================================================
// Cloth.cpp — 布料模拟的核心实现
//
// 这个文件是 CSF 算法的"心脏"，包含：
//   1. Cloth 构造函数 — 创建粒子网格和约束关系
//   2. timeStep()     — 一次物理迭代（Verlet积分 + 约束满足）
//   3. addForce()     — 给所有粒子施力
//   4. terrCollision()— 碰撞检测（粒子不能穿过地面）
//   5. movableFilter()— 坡度平滑后处理（修复悬空粒子）
//
// 想象：一块布料从高处落下，被地形托住，最终贴合地形表面。
//       布料由很多"粒子"组成，相邻粒子之间有"弹簧"（约束）连接，
//       弹簧会拉住粒子不让布料变形太厉害。
// ======================================================================================

#include "Cloth.h"
#include <chrono>
#include <fstream>

namespace
{
    using Clock = std::chrono::steady_clock;

    double elapsedMs(const Clock::time_point &start, const Clock::time_point &end)
    {
        return std::chrono::duration<double, std::milli>(end - start).count();
    }
}

// ==========================================================================
// Cloth 构造函数 — 初始化粒子网格和约束关系
//
// 创建一块 width × height 的粒子网格，所有粒子初始在同一水平面上，
// 然后建立近邻和次近邻的约束关系。
//
// 参数说明：
//   _origin_pos          — 布料左下角的3D坐标
//   _num_particles_width — X方向的粒子数（列数）
//   _num_particles_height— Z方向的粒子数（行数）
//   _step_x              — X方向粒子间距（米）
//   _step_y              — Z方向粒子间距（米）
//   _smoothThreshold     — 坡度平滑时允许的高差阈值（米）
//   _heightThreshold     — 坡度平滑时允许的悬空高度阈值（米）
//   rigidness            — 刚性等级（1/2/3），等于约束迭代次数
//   time_step            — 物理模拟时间步长
//
// ------------------------------------------------------------------
// COPY From CSF.cpp
// 步骤3：创建布料对象
//
// 构造函数里会完成：
//   a) 初始化所有粒子（全部在同一水平面上，Y = origin_pos.y）
//   b) 建立近邻约束（上下左右+对角线，8个方向）
//   c) 建立次近邻约束（跳一格的邻居，4个方向）
//
// 参数说明：
//   origin_pos              — 布料左下角3D坐标
//   width_num               — X方向粒子数
//   height_num              — Z方向粒子数
//   params.cloth_resolution — X方向步长（米）
//   params.cloth_resolution — Z方向步长（米），通常与X相同
//   0.3                     — smoothThreshold: 坡度平滑时的高差阈值（米）
//                              相邻粒子高差<0.3才允许平滑
//   9999                    — heightThreshold: 粒子到地面高度差阈值（米）
//                              粒子悬空高度<9999才允许平滑（9999=不限制）
//   params.rigidness        — 刚性等级，决定约束校正系数
//   params.time_step        — 时间步长==========================================================================
Cloth::Cloth(const Vec3 &_origin_pos,
             int _num_particles_width,
             int _num_particles_height,
             double _step_x,
             double _step_y,
             double _smoothThreshold,
             double _heightThreshold,
             int rigidness,
             double time_step,
             bool memory_optimized)
    : constraint_iterations(rigidness), // 约束迭代次数 = rigidness
      time_step(time_step),
      smoothThreshold(_smoothThreshold),
      heightThreshold(_heightThreshold),
      origin_pos(_origin_pos),
      step_x(_step_x),
      step_y(_step_y),
      num_particles_width(_num_particles_width),
      num_particles_height(_num_particles_height)
{

    double time_step2 = time_step * time_step;

    // ==================================================================
    // 第一步：创建粒子网格
    // 所有粒子初始位置在同一水平面上：Y = origin_pos.f[1]（最高点上方）
    // X 和 Z 按网格间距排列
    // ==================================================================
    particles.resize(num_particles_width * num_particles_height);
    if (memory_optimized) {
        for (std::size_t i = 0; i < particles.size(); i++) {
            particles[i].neighborsList.reserve(12);
        }
    }

    for (int i = 0; i < num_particles_width; i++)
    {
        for (int j = 0; j < num_particles_height; j++)
        {
            // 计算粒子的3D位置
            Vec3 pos(origin_pos.f[0] + i * step_x,  // X = 起始X + 列号×步长
                     origin_pos.f[1],               // Y = 起始Y（所有粒子同一高度）
                     origin_pos.f[2] + j * step_y); // Z = 起始Z + 行号×步长

            // 把粒子放入一维数组（行优先存储）
            // 索引 = 行号 × 宽度 + 列号
            int idx = j * num_particles_width + i;
            particles[idx] = Particle(pos, time_step2);
            particles[idx].pos_x = i; // 记录在网格中的列号（后面后处理要用）
            particles[idx].pos_y = j; // 记录在网格中的行号
        }
    }

    // ==================================================================
    // 第二步：建立近邻约束（距离1和√2的邻居）
    //
    // 每个粒子与以下4类邻居建立约束：
    //   → 右邻 (i+1, j)     距离=1
    //   → 下邻 (i, j+1)     距离=1
    //   → 右下对角 (i+1, j+1) 距离=√2
    //   → 左下对角 (i+1, j) ↔ (i, j+1) 距离=√2
    //
    // 约束不是独立的对象，而是通过 neighborsList 指针互相引用
    // makeConstraint 会把对方加到自己的邻居列表里（双向）
    // ==================================================================
    for (int x = 0; x < num_particles_width; x++)
    {
        for (int y = 0; y < num_particles_height; y++)
        {
            if (x < num_particles_width - 1)
                makeConstraint(getParticle(x, y), getParticle(x + 1, y)); // 右邻

            if (y < num_particles_height - 1)
                makeConstraint(getParticle(x, y), getParticle(x, y + 1)); // 下邻

            if ((x < num_particles_width - 1) && (y < num_particles_height - 1))
                makeConstraint(getParticle(x, y), getParticle(x + 1, y + 1)); // 右下对角

            if ((x < num_particles_width - 1) && (y < num_particles_height - 1))
                makeConstraint(getParticle(x + 1, y), getParticle(x, y + 1)); // 左下对角
        }
    }

    // ==================================================================
    // 第三步：建立次近邻约束（距离2和√8的邻居）
    //
    // 与近邻类似，但跳一格：
    //   → 隔一个的右邻 (i+2, j)    距离=2
    //   → 隔一个的下邻 (i, j+2)    距离=2
    //   → 隔一个的右下对角 (i+2, j+2) 距离=√8
    //   → 隔一个的左下对角          距离=√8
    //
    // 次近邻约束让布料更"硬"，不容易产生褶皱
    // 每个粒子总共大约有 8~12 个邻居约束
    // ==================================================================
    for (int x = 0; x < num_particles_width; x++)
    {
        for (int y = 0; y < num_particles_height; y++)
        {
            if (x < num_particles_width - 2)
                makeConstraint(getParticle(x, y), getParticle(x + 2, y));

            if (y < num_particles_height - 2)
                makeConstraint(getParticle(x, y), getParticle(x, y + 2));

            if ((x < num_particles_width - 2) && (y < num_particles_height - 2))
                makeConstraint(getParticle(x, y), getParticle(x + 2, y + 2));

            if ((x < num_particles_width - 2) && (y < num_particles_height - 2))
                makeConstraint(getParticle(x + 2, y), getParticle(x, y + 2));
        }
    }
}

// ==========================================================================
// timeStep — 执行一次物理模拟迭代
//
// 这是被 CSF::do_cloth() 的 for 循环反复调用的核心方法。
// 每次迭代做三件事：
//   1. 对所有粒子做 Verlet 积分（根据速度和加速度更新位置）
//   2. 对所有粒子做约束满足（让相邻粒子不要离太远/太近）
//   3. 统计本轮的最大位移（用于早停判断）
//
// 返回值：所有可移动粒子的最大Y方向位移（用于早停判断）
//         如果 maxDiff < 0.005，说明布料基本稳定了
// ==========================================================================
double Cloth::timeStep(ClothStepProfile *profile)
{
    int particleCount = static_cast<int>(particles.size());

    // --- 第1步：Verlet 积分 ---
    // 对每个粒子，根据当前速度和加速度计算新位置
    // 详见 Particle::timeStep() 的注释
    Clock::time_point stageStart = Clock::now();
#ifdef CSF_USE_OPENMP
#pragma omp parallel for
#endif
    for (int i = 0; i < particleCount; i++)
    {
        particles[i].timeStep();
    }
    if (profile)
    {
        profile->verlet_ms += elapsedMs(stageStart, Clock::now());
    }

    // --- 第2步：约束满足 ---
    // 对每个粒子，检查它与所有邻居的Y方向距离，
    // 如果差距太大就把它拉回来
    // 详见 Particle::satisfyConstraintSelf() 的注释
    //
    // 注意：这里用了 OpenMP 并行，但实际上存在竞态条件——
    // 粒子A修改自己位置后，粒子B（A的邻居）可能同时读到A的旧/新位置
    // 这导致结果具有不确定性，但在实践中影响不大
    stageStart = Clock::now();
#ifdef CSF_USE_OPENMP
#pragma omp parallel for
#endif
    for (int j = 0; j < particleCount; j++)
    {
        particles[j].satisfyConstraintSelf(constraint_iterations);
    }
    if (profile)
    {
        profile->constraint_ms += elapsedMs(stageStart, Clock::now());
    }

    // --- 第3步：统计最大位移 ---
    // 遍历所有可移动粒子，找到Y方向位移最大的那个
    // 用于判断布料是否已经稳定（早停）
    stageStart = Clock::now();
    double maxDiff = 0;
#ifdef CSF_USE_OPENMP
#pragma omp parallel for reduction(max : maxDiff)
#endif
    for (int i = 0; i < particleCount; i++)
    {
        if (particles[i].isMovable())
        {
            double diff = fabs(particles[i].old_pos.f[1] - particles[i].pos.f[1]);
            if (diff > maxDiff)
                maxDiff = diff;
        }
    }
    if (profile)
    {
        profile->maxdiff_ms += elapsedMs(stageStart, Clock::now());
    }

    return maxDiff;
}

// ==========================================================================
// addForce — 给所有粒子施加一个力
// 在 CSF::do_cloth() 中被调用一次，施加重力
// 力向量会被加到每个粒子的加速度上（F = ma，m=1，所以 a = F）
// ==========================================================================
void Cloth::addForce(const Vec3 direction)
{
    for (std::size_t i = 0; i < particles.size(); i++)
    {
        particles[i].addForce(direction);
    }
}

// ==========================================================================
// terrCollision — 地形碰撞检测
//
// 检查每个粒子是否"穿过"了地面。
// 如果粒子的 Y 坐标低于它对应位置的地形高度（heightvals[i]），
// 就把它拉回到地形高度，并标记为"不可移动"。
//
// 为什么要标记不可移动？
//   粒子碰到地面后就应该停住不动了，后续迭代中不再移动
//   只有还没碰到地面的粒子才继续下落
//
// heightvals 数组由 Rasterization::RasterTerrian 填充，
// 记录了每个粒子对应位置的地形高度
// ==========================================================================
void Cloth::terrCollision()
{
    int particleCount = static_cast<int>(particles.size());

#ifdef CSF_USE_OPENMP
#pragma omp parallel for
#endif
    for (int i = 0; i < particleCount; i++)
    {
        Vec3 v = particles[i].getPos();

        // 如果粒子的Y值 < 地形高度，说明穿过了地面
        if (v.f[1] < heightvals[i])
        {
            // 把粒子拉回到地形表面（Y方向对齐）
            particles[i].offsetPos(Vec3(0, heightvals[i] - v.f[1], 0));
            // 标记粒子为不可移动（碰到了地面就不再动了）
            particles[i].makeUnmovable();
        }
    }
}

// ==========================================================================
// movableFilter — 坡度平滑后处理
//
// 模拟结束后，布料上可能还有些粒子仍然"可移动"（没碰到地面）。
// 这些粒子悬在空中，对应的区域可能是：
//   - 建筑物顶部（布料不应该碰到建筑顶）
//   - 陡坡边缘（布料本来应该碰到坡面，但因为约束太强没贴合）
//
// 后处理逻辑：
//   1. 用 BFS 找到所有悬空粒子的"连通区域"
//   2. 如果连通区域足够大（>50个粒子），进入步骤3
//   3. 找到连通区域边缘与"已落地"粒子相邻的点
//   4. 如果边缘悬空粒子与已落地邻居的高差不大，就把悬空粒子也拉到地面
//   5. 从这些边缘点开始，向连通区域内部扩散，把满足条件的粒子都拉到地面
//
// 效果：修复坡面处布料悬空的"洞"，让布料更好地贴合缓坡地形
// ==========================================================================
void Cloth::movableFilter()
{
    // 遍历布料网格中所有粒子
    for (int x = 0; x < num_particles_width; x++)
    {
        for (int y = 0; y < num_particles_height; y++)
        {
            Particle *ptc = getParticle(x, y);

            // 只处理"仍然可移动"且"未被访问过"的粒子
            // 可移动 = 没碰到地面，还悬在空中
            if (ptc->isMovable() && !ptc->isVisited)
            {
                // --- BFS 搜索连通区域 ---
                std::queue<int> que;
                std::vector<XY> connected;             // 存放连通区域中所有粒子的网格坐标
                std::vector<std::vector<int>> neibors; // 每个粒子的连通邻居索引
                int sum = 1;                           // 连通区域的粒子数
                int index = y * num_particles_width + x;

                // 初始化：把起始粒子加入连通区域
                connected.push_back(XY(x, y));
                particles[index].isVisited = true;
                que.push(index);

                // BFS 扩展：从起始粒子出发，找所有相邻的可移动粒子
                while (!que.empty())
                {
                    Particle *ptc_f = &particles[que.front()];
                    que.pop();
                    int cur_x = ptc_f->pos_x;
                    int cur_y = ptc_f->pos_y;
                    std::vector<int> neibor; // 当前粒子在连通区域内的邻居

                    // 检查左邻
                    if (cur_x > 0)
                    {
                        Particle *ptc_left = getParticle(cur_x - 1, cur_y);
                        if (ptc_left->isMovable())
                        {
                            if (!ptc_left->isVisited)
                            {
                                sum++;
                                ptc_left->isVisited = true;
                                connected.push_back(XY(cur_x - 1, cur_y));
                                que.push(num_particles_width * cur_y + cur_x - 1);
                                neibor.push_back(sum - 1);
                                ptc_left->c_pos = sum - 1; // 记录在connected中的索引
                            }
                            else
                            {
                                neibor.push_back(ptc_left->c_pos);
                            }
                        }
                    }

                    // 检查右邻
                    if (cur_x < num_particles_width - 1)
                    {
                        Particle *ptc_right = getParticle(cur_x + 1, cur_y);
                        if (ptc_right->isMovable())
                        {
                            if (!ptc_right->isVisited)
                            {
                                sum++;
                                ptc_right->isVisited = true;
                                connected.push_back(XY(cur_x + 1, cur_y));
                                que.push(num_particles_width * cur_y + cur_x + 1);
                                neibor.push_back(sum - 1);
                                ptc_right->c_pos = sum - 1;
                            }
                            else
                            {
                                neibor.push_back(ptc_right->c_pos);
                            }
                        }
                    }

                    // 检查上邻（y-1方向）
                    if (cur_y > 0)
                    {
                        Particle *ptc_bottom = getParticle(cur_x, cur_y - 1);
                        if (ptc_bottom->isMovable())
                        {
                            if (!ptc_bottom->isVisited)
                            {
                                sum++;
                                ptc_bottom->isVisited = true;
                                connected.push_back(XY(cur_x, cur_y - 1));
                                que.push(num_particles_width * (cur_y - 1) + cur_x);
                                neibor.push_back(sum - 1);
                                ptc_bottom->c_pos = sum - 1;
                            }
                            else
                            {
                                neibor.push_back(ptc_bottom->c_pos);
                            }
                        }
                    }

                    // 检查下邻（y+1方向）
                    if (cur_y < num_particles_height - 1)
                    {
                        Particle *ptc_top = getParticle(cur_x, cur_y + 1);
                        if (ptc_top->isMovable())
                        {
                            if (!ptc_top->isVisited)
                            {
                                sum++;
                                ptc_top->isVisited = true;
                                connected.push_back(XY(cur_x, cur_y + 1));
                                que.push(num_particles_width * (cur_y + 1) + cur_x);
                                neibor.push_back(sum - 1);
                                ptc_top->c_pos = sum - 1;
                            }
                            else
                            {
                                neibor.push_back(ptc_top->c_pos);
                            }
                        }
                    }
                    neibors.push_back(neibor);
                }

                // --- 连通区域后处理 ---
                // 只有连通区域足够大（>50个粒子）才做后处理
                // 小的悬空区域可能是建筑物顶部，不应该被拉到地面
                if (sum > MAX_PARTICLE_FOR_POSTPROCESSIN)
                {
                    // 找到连通区域边缘与"已落地"粒子相邻的悬空点
                    std::vector<int> edgePoints = findUnmovablePoint(connected);
                    // 从边缘点开始，向内部扩散，把满足条件的悬空粒子拉到地面
                    handle_slop_connected(edgePoints, connected, neibors);
                }
            }
        }
    }
}

// ==========================================================================
// findUnmovablePoint — 找到连通区域中与"已落地"粒子相邻的边缘点
//
// 对连通区域中的每个粒子，检查4邻域是否有"不可移动"（已落地）的粒子。
// 如果有，且满足以下两个条件，就把这个悬空粒子也拉到地面：
//   1. 两个位置的地形高差 < smoothThreshold（0.3米）
//   2. 悬空粒子的当前Y值与地形高度的差 < heightThreshold（9999米，基本不限制）
//
// 返回：所有被拉到地面的边缘点在 connected 数组中的索引
// ==========================================================================
std::vector<int> Cloth::findUnmovablePoint(std::vector<XY> connected)
{
    std::vector<int> edgePoints;

    for (std::size_t i = 0; i < connected.size(); i++)
    {
        int x = connected[i].x;
        int y = connected[i].y;
        int index = y * num_particles_width + x;
        Particle *ptc = getParticle(x, y);

        // 检查左邻是否已落地
        if (x > 0)
        {
            Particle *ptc_x = getParticle(x - 1, y);
            if (!ptc_x->isMovable())
            {
                int index_ref = y * num_particles_width + x - 1;
                // 条件1：两个位置的地形高差 < 0.3米（平滑过渡）
                // 条件2：悬空粒子离地面的高度 < 9999米（基本不限制）
                if ((fabs(heightvals[index] - heightvals[index_ref]) < smoothThreshold) &&
                    (ptc->getPos().f[1] - heightvals[index] < heightThreshold))
                {
                    // 把悬空粒子拉到地面
                    Vec3 offsetVec = Vec3(0, heightvals[index] - ptc->getPos().f[1], 0);
                    particles[index].offsetPos(offsetVec);
                    ptc->makeUnmovable();
                    edgePoints.push_back(i);
                    continue; // 这个粒子已经落地了，不用再检查其他方向
                }
            }
        }

        // 检查右邻是否已落地
        if (x < num_particles_width - 1)
        {
            Particle *ptc_x = getParticle(x + 1, y);
            if (!ptc_x->isMovable())
            {
                int index_ref = y * num_particles_width + x + 1;
                if ((fabs(heightvals[index] - heightvals[index_ref]) < smoothThreshold) &&
                    (ptc->getPos().f[1] - heightvals[index] < heightThreshold))
                {
                    Vec3 offsetVec = Vec3(0, heightvals[index] - ptc->getPos().f[1], 0);
                    particles[index].offsetPos(offsetVec);
                    ptc->makeUnmovable();
                    edgePoints.push_back(i);
                    continue;
                }
            }
        }

        // 检查上邻（y-1）是否已落地
        if (y > 0)
        {
            Particle *ptc_y = getParticle(x, y - 1);
            if (!ptc_y->isMovable())
            {
                int index_ref = (y - 1) * num_particles_width + x;
                if ((fabs(heightvals[index] - heightvals[index_ref]) < smoothThreshold) &&
                    (ptc->getPos().f[1] - heightvals[index] < heightThreshold))
                {
                    Vec3 offsetVec = Vec3(0, heightvals[index] - ptc->getPos().f[1], 0);
                    particles[index].offsetPos(offsetVec);
                    ptc->makeUnmovable();
                    edgePoints.push_back(i);
                    continue;
                }
            }
        }

        // 检查下邻（y+1）是否已落地
        if (y < num_particles_height - 1)
        {
            Particle *ptc_y = getParticle(x, y + 1);
            if (!ptc_y->isMovable())
            {
                int index_ref = (y + 1) * num_particles_width + x;
                if ((fabs(heightvals[index] - heightvals[index_ref]) < smoothThreshold) &&
                    (ptc->getPos().f[1] - heightvals[index] < heightThreshold))
                {
                    Vec3 offsetVec = Vec3(0, heightvals[index] - ptc->getPos().f[1], 0);
                    particles[index].offsetPos(offsetVec);
                    ptc->makeUnmovable();
                    edgePoints.push_back(i);
                    continue;
                }
            }
        }
    }

    return edgePoints;
}

// ==========================================================================
// handle_slop_connected — 从边缘点向连通区域内部扩散坡度平滑
//
// 原理：findUnmovablePoint 找到了与已落地粒子相邻的边缘悬空点，
//       并把它们拉到了地面。现在从这些新落地的点出发，
//       继续检查它们的悬空邻居，如果高差不大就也拉到地面。
//       用BFS扩散，直到没有更多满足条件的粒子为止。
//
// 这就像在水面上滴墨水，墨水会从接触点向周围扩散。
// 坡面上的悬空粒子会被"感染"成落地粒子，逐层向内扩散。
// ==========================================================================
void Cloth::handle_slop_connected(std::vector<int> edgePoints, std::vector<XY> connected, std::vector<std::vector<int>> neibors)
{
    std::vector<bool> visited;
    // 初始化：所有粒子都未访问
    for (std::size_t i = 0; i < connected.size(); i++)
        visited.push_back(false);

    std::queue<int> que;
    // 把所有边缘点加入队列，作为BFS的起点
    for (std::size_t i = 0; i < edgePoints.size(); i++)
    {
        que.push(edgePoints[i]);
        visited[edgePoints[i]] = true;
    }

    // BFS 扩散
    while (!que.empty())
    {
        int index = que.front();
        que.pop();

        // 当前粒子（已经落地）在布料网格中的索引
        int index_center = connected[index].y * num_particles_width + connected[index].x;

        // 遍历当前粒子在连通区域内的所有邻居
        for (std::size_t i = 0; i < neibors[index].size(); i++)
        {
            // 邻居在布料网格中的索引
            int index_neibor = connected[neibors[index][i]].y * num_particles_width + connected[neibors[index][i]].x;

            // 判断邻居是否也应该被拉到地面：
            // 条件1：两个位置的地形高差 < smoothThreshold
            // 条件2：邻居粒子当前离地面的高度 < heightThreshold
            if ((fabs(heightvals[index_center] - heightvals[index_neibor]) < smoothThreshold) &&
                (fabs(particles[index_neibor].getPos().f[1] - heightvals[index_neibor]) < heightThreshold))
            {
                // 把邻居也拉到地面
                Vec3 offsetVec = Vec3(0, heightvals[index_neibor] - particles[index_neibor].getPos().f[1], 0);
                particles[index_neibor].offsetPos(offsetVec);
                particles[index_neibor].makeUnmovable();

                // 如果这个邻居还没被访问过，加入队列继续扩散
                if (visited[neibors[index][i]] == false)
                {
                    que.push(neibors[index][i]);
                    visited[neibors[index][i]] = true;
                }
            }
        }
    }
}

// ==========================================================================
// toVector — 把布料所有粒子的坐标导出为一维数组
// 顺序：每3个double代表一个粒子的 (X, Z, -Y)
// 注意 -Y = 原始高程Z，这样导出的坐标是原始GIS坐标系
// ==========================================================================
std::vector<double> Cloth::toVector()
{
    std::vector<double> clothCoordinates;
    clothCoordinates.reserve(particles.size() * 3);
    for (auto &particle : particles)
    {
        clothCoordinates.push_back(particle.getPos().f[0]);  // X
        clothCoordinates.push_back(particle.getPos().f[2]);  // Z = 原始Y
        clothCoordinates.push_back(-particle.getPos().f[1]); // -Y = 原始Z（高程）
    }
    return clothCoordinates;
}

// ==========================================================================
// saveToFile — 把布料所有粒子的坐标保存到文本文件
// 格式：每行 X Z -Y（8位小数）
// ==========================================================================
void Cloth::saveToFile(std::string path)
{
    std::string filepath = "cloth_nodes.txt";

    if (path == "")
    {
        filepath = "cloth_nodes.txt";
    }
    else
    {
        filepath = path;
    }

    std::ofstream f1(filepath.c_str());

    if (!f1)
        return;

    for (std::size_t i = 0; i < particles.size(); i++)
    {
        f1 << std::fixed << std::setprecision(8) << particles[i].getPos().f[0] << "	" << particles[i].getPos().f[2] << "	" << -particles[i].getPos().f[1] << std::endl;
    }

    f1.close();
}

// ==========================================================================
// saveMovableToFile — 只保存仍然可移动（悬空）的粒子坐标
// 用于调试：看看哪些粒子模拟结束后还在空中
// ==========================================================================
void Cloth::saveMovableToFile(std::string path)
{
    std::string filepath = "cloth_movable.txt";

    if (path == "")
    {
        filepath = "cloth_movable.txt";
    }
    else
    {
        filepath = path;
    }

    std::ofstream f1(filepath.c_str());

    if (!f1)
        return;

    for (std::size_t i = 0; i < particles.size(); i++)
    {
        if (particles[i].isMovable())
        {
            f1 << std::fixed << std::setprecision(8) << particles[i].getPos().f[0] << "	"
               << particles[i].getPos().f[2] << "	" << -particles[i].getPos().f[1] << std::endl;
        }
    }

    f1.close();
}
