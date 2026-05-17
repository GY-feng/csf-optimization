// ======================================================================================
// CSF.cpp — CSF 主类的实现文件
//
// 这个文件是算法流程的"指挥官"，它按顺序调用各个模块完成滤波。
// 重点理解三个方法：
//   1. setPointCloud()  — 输入数据 + 坐标变换
//   2. do_cloth()       — 布料模拟的主流程（最核心！）
//   3. do_filtering()   — 在 do_cloth() 基础上加距离分类
// ======================================================================================

#define DLL_IMPLEMENT

#include "CSF.h"
#include "XYZReader.h"
#include "Vec3.h"
#include "Rasterization.h"
#include "RasterizationSoA.h"
#include "c2cdist.h"
#include "c2cdistSoA.h"
#include "ClothSoA.h"
#include <chrono>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <sstream>

#ifdef CSF_ENABLE_CUDA
#include "cuda/ClothCuda.cuh"
#endif

namespace {
using Clock = std::chrono::steady_clock;

double elapsedMs(const Clock::time_point& start, const Clock::time_point& end)
{
    return std::chrono::duration<double, std::milli>(end - start).count();
}

std::string jsonEscape(const std::string& value)
{
    std::ostringstream out;
    for (std::size_t i = 0; i < value.size(); i++) {
        char c = value[i];
        switch (c) {
        case '\\':
            out << "\\\\";
            break;
        case '"':
            out << "\\\"";
            break;
        case '\n':
            out << "\\n";
            break;
        case '\r':
            out << "\\r";
            break;
        case '\t':
            out << "\\t";
            break;
        default:
            out << c;
            break;
        }
    }
    return out.str();
}

#ifndef CSF_ENABLE_CUDA
const char* cudaBuildRequiredMessage()
{
    return "gpu_enabled requires a CUDA build. Reinstall in WSL with: "
           "CSF_CUDA_ARCH=sm_89 CSF_ENABLE_CUDA=1 pip install -e . --no-build-isolation --force-reinstall";
}
#endif
}

CSF::ProfileStats::ProfileStats()
    : bounding_box_ms(0),
      cloth_init_ms(0),
      rasterization_ms(0),
      simulation_ms(0),
      timestep_ms(0),
      verlet_ms(0),
      constraint_ms(0),
      maxdiff_ms(0),
      collision_ms(0),
      postprocess_ms(0),
      classification_ms(0),
      total_filtering_ms(0),
      point_count(0),
      cloth_width(0),
      cloth_height(0),
      particle_count(0),
      iterations_configured(0),
      iterations_run(0),
      ground_count(0),
      non_ground_count(0),
      ground_ratio(0),
      cloth_resolution(0),
      rigidness(0),
      time_step(0),
      class_threshold(0),
      bSloopSmooth(false),
      backend("legacy"),
      fallback_used(false),
      backend_fallback_reason(""),
      memory_optimized(false),
      deterministic_soa(false),
      gpu_enabled(false),
      gpu_simulation(false),
      gpu_classification(false),
      gpu_rasterization(false),
      gpu_device_id(0),
      backend_error("")
{
}

// ==========================================================================
// 构造函数：设置默认参数
// 这些默认值是论文作者的推荐值，一般不需要改
// ==========================================================================
CSF::CSF(int index)
{
    params.bSloopSmooth = true;   // 默认开启坡度平滑
    params.time_step = 0.65;      // 时间步长
    params.class_threshold = 0.5; // 分类距离阈值（米）
    params.cloth_resolution = 1;  // 布料网格分辨率（米）
    params.rigidness = 3;         // 刚性等级（1软/2中/3硬）
    params.interations = 500;     // 最大迭代次数
    params.useSoA = false;        // 默认保持旧后端
    params.opt_memory_optimized = false;
    params.opt_deterministic_soa = false;
    params.opt_gpu_enabled = false;
    params.opt_gpu_simulation = false;
    params.opt_gpu_classification = false;
    params.opt_gpu_rasterization = false;
    params.gpu_device_id = 0;

    this->index = index;
}

CSF::CSF()
{
    params.bSloopSmooth = true;
    params.time_step = 0.65;
    params.class_threshold = 0.5;
    params.cloth_resolution = 1;
    params.rigidness = 3;
    params.interations = 500;
    params.useSoA = false;
    params.opt_memory_optimized = false;
    params.opt_deterministic_soa = false;
    params.opt_gpu_enabled = false;
    params.opt_gpu_simulation = false;
    params.opt_gpu_classification = false;
    params.opt_gpu_rasterization = false;
    params.gpu_device_id = 0;
    this->index = 0;
}

CSF::~CSF()
{
}

void CSF::configureOptimization(bool memoryOptimized,
                                bool deterministicSoA,
                                bool gpuEnabled,
                                bool gpuSimulation,
                                bool gpuClassification,
                                bool gpuRasterization,
                                int gpuDeviceId)
{
    params.opt_memory_optimized = memoryOptimized;
    params.opt_deterministic_soa = deterministicSoA;
    params.opt_gpu_enabled = gpuEnabled;
    params.opt_gpu_simulation = gpuSimulation;
    params.opt_gpu_classification = gpuClassification;
    params.opt_gpu_rasterization = gpuRasterization;
    params.gpu_device_id = gpuDeviceId;
    params.useSoA = deterministicSoA;
}

void CSF::validateOptimizationConfig(bool exportCloth) const
{
    if (params.opt_deterministic_soa && !params.opt_memory_optimized) {
        throw std::runtime_error("optimization config error: deterministic_soa requires memory_optimized=true");
    }

    if (params.opt_gpu_enabled && !params.opt_deterministic_soa) {
        throw std::runtime_error("optimization config error: gpu_enabled requires deterministic_soa=true");
    }

    if ((params.opt_gpu_simulation || params.opt_gpu_classification || params.opt_gpu_rasterization) &&
        !params.opt_gpu_enabled) {
        throw std::runtime_error("optimization config error: gpu module flags require gpu_enabled=true");
    }

    if (params.opt_deterministic_soa && params.bSloopSmooth) {
        throw std::runtime_error("deterministic_soa_postprocess_not_supported");
    }

    if (params.opt_deterministic_soa && exportCloth) {
        throw std::runtime_error("deterministic_soa_export_cloth_not_supported");
    }

    if (params.opt_gpu_enabled) {
#ifndef CSF_ENABLE_CUDA
        throw std::runtime_error(cudaBuildRequiredMessage());
#endif
    }
}

void CSF::initializeProfile(const std::string& backend)
{
    last_profile = ProfileStats();
    last_profile.point_count = point_cloud.size();
    last_profile.iterations_configured = params.interations;
    last_profile.cloth_resolution = params.cloth_resolution;
    last_profile.rigidness = params.rigidness;
    last_profile.time_step = params.time_step;
    last_profile.class_threshold = params.class_threshold;
    last_profile.bSloopSmooth = params.bSloopSmooth;
    last_profile.backend = backend;
    last_profile.memory_optimized = params.opt_memory_optimized;
    last_profile.deterministic_soa = params.opt_deterministic_soa;
    last_profile.gpu_enabled = params.opt_gpu_enabled;
    last_profile.gpu_simulation = params.opt_gpu_simulation;
    last_profile.gpu_classification = params.opt_gpu_classification;
    last_profile.gpu_rasterization = params.opt_gpu_rasterization;
    last_profile.gpu_device_id = params.gpu_device_id;
}

// ==========================================================================
// setPointCloud — 输入点云（从 vector<csf::Point> 传入）
//
// 坐标变换：原始 (X, Y, Z) → 内部 (X, -Z, Y)
//   X 不变（东向）
//   Y = -Z（原始高程取负，这样重力方向 = Y减小的方向）
//   Z = Y（原始北向）
//
// 坐标系约定：
//   在内部坐标系中，Y 值越大 = 越高（天上），Y 值越小 = 越低（地下）
//   布料放在 Y 最大值附近（天上），重力让 Y 减小（往下掉）
//   当粒子 Y < heightvals（地形高度）时，说明穿过地面，碰撞检测把它拉回来
//
//   例如：原始高程 Z=0（谷底）→ 内部 Y=0（高处的Y值大？）
//   等等... 这里的坐标逻辑不太直觉，但算法确实跑通了，
//   关键只要记住：布料从 Y 大处开始，Y 减小 = 下落，Y < heightvals = 碰到地形
// ==========================================================================
void CSF::setPointCloud(std::vector<csf::Point> points)
{
    point_cloud.resize(points.size());

    int pointCount = static_cast<int>(points.size());
// 如果编译时开启了 OpenMP，这里会并行处理
#ifdef CSF_USE_OPENMP
#pragma omp parallel for
#endif
    for (int i = 0; i < pointCount; i++)
    {
        csf::Point las;
        las.x = points[i].x;  // X 不变（东向）
        las.y = -points[i].z; // Y = -Z（高程取负）
        las.z = points[i].y;  // Z = Y（北向）
        point_cloud[i] = las;
    }
}

// ==========================================================================
// setPointCloud — 从二维数组传入（行优先，N行×3列）
// Python 调用入口：numpy 的 ndarray 就是这种行优先布局
// 坐标变换逻辑同上
// ==========================================================================
void CSF::setPointCloud(double *points, int rows, int cols)
{
    point_cloud.resize(rows);
// A(i,j) 宏：按行优先访问 points[i][j]
#define A(i, j) points[i * cols + j]
    for (int i = 0; i < rows; i++)
    {
        point_cloud[i] = {A(i, 0), -A(i, 2), A(i, 1)}; // 同样的坐标变换
    }
}

// ==========================================================================
// setPointCloud — 从一维数组传入（列优先，3列×N行）
// MATLAB 调用入口：MATLAB 矩阵是列优先布局
// 坐标变换逻辑同上
// ==========================================================================
void CSF::setPointCloud(double *points, int rows)
{
// Mat(i,j) 宏：按列优先访问 points[j][i]，共3列（x,y,z），rows行
#define Mat(i, j) points[i + j * rows]
    point_cloud.resize(rows);
    for (int i = 0; i < rows; i++)
    {
        point_cloud[i] = {Mat(i, 0), -Mat(i, 2), Mat(i, 1)};
    }
}

// ==========================================================================
// setPointCloud — 从 csf::PointCloud 对象直接设置
// 坐标变换逻辑同上
// ==========================================================================
void CSF::setPointCloud(csf::PointCloud &pc)
{
    point_cloud.resize(pc.size());
    int pointCount = static_cast<int>(pc.size());
#ifdef CSF_USE_OPENMP
#pragma omp parallel for
#endif
    for (int i = 0; i < pointCount; i++)
    {
        csf::Point las;
        las.x = pc[i].x;
        las.y = -pc[i].z; // Y = -Z
        las.z = pc[i].y;  // Z = Y
        point_cloud[i] = las;
    }
}

// ==========================================================================
// readPointsFromFile — 从文本文件读取点云
// 文件格式：每行 X Y Z，空格或Tab分隔
// 内部调用 XYZReader 的 read_xyz 函数
// ==========================================================================
void CSF::readPointsFromFile(std::string filename)
{
    this->point_cloud.resize(0);
    read_xyz(filename, this->point_cloud);
}

// ==========================================================================
// do_cloth — 布料模拟的核心流程！！！
//
// 这是整个算法最核心的方法，理解了这个方法就理解了 CSF 的全部。
// 返回值：模拟完成后的 Cloth 对象（包含所有粒子的最终位置）
//
// 流程：
//   1. 计算点云包围盒 → 确定布料的大小和位置
//   2. 创建布料 → 初始化粒子网格和约束
//   3. 光栅化 → 把点云高度投影到布料网格
//   4. 物理模拟循环 → 反复"下落+碰撞"
//   5. 坡度平滑 → 修复坡面处悬空粒子
// ==========================================================================
Cloth CSF::do_cloth()
{
    initializeProfile("legacy");

    // ------------------------------------------------------------------
    // 步骤1：计算点云的包围盒
    // bbMin = 点云中 X/Y/Z 各维度的最小值
    // bbMax = 点云中 X/Y/Z 各维度的最大值
    // 注意：此时 Y = -Z（负高程），所以 bbMax.y 对应原始高程最小值
    // ------------------------------------------------------------------
    std::cout << "[" << this->index << "] Configuring terrain..." << std::endl;
    csf::Point bbMin, bbMax;
    Clock::time_point stageStart = Clock::now();
    point_cloud.computeBoundingBox(bbMin, bbMax);
    last_profile.bounding_box_ms = elapsedMs(stageStart, Clock::now());
    std::cout << "[" << this->index << "]  - bbMin: " << bbMin.x << " " << bbMin.y << " " << bbMin.z << std::endl;
    std::cout << "[" << this->index << "]  - bbMax: " << bbMax.x << " " << bbMax.y << " " << bbMax.z << std::endl;

    // ------------------------------------------------------------------
    // 步骤2：确定布料的初始位置和大小
    // ------------------------------------------------------------------

    // cloth_y_height: 布料初始位置比点云最高点再高出0.05米
    //   这样布料整体在点云上方，模拟时从上往下掉
    double cloth_y_height = 0.05;

    // clothbuffer_d: 布料四周的缓冲区宽度（以粒子数为单位）
    //   设为2，即布料比点云范围向四周多出2个粒子的边距
    //   防止边缘的点没有对应的布料粒子
    int clothbuffer_d = 2;

    // 布料左下角的起始3D坐标
    Vec3 origin_pos(
        bbMin.x - clothbuffer_d * params.cloth_resolution, // X：点云最左边再往左2格
        bbMax.y + cloth_y_height,                          // Y：点云最高点再往上0.05
        bbMin.z - clothbuffer_d * params.cloth_resolution  // Z：点云最前边再往前2格
    );


    //粒子其实就是网格的意思吧

    // 布料在 X 方向（宽度）的粒子数
    int width_num = static_cast<int>(
                        std::floor((bbMax.x - bbMin.x) / params.cloth_resolution)) +
                    2 * clothbuffer_d;

    // 布料在 Z 方向（深度/北向）的粒子数
    int height_num = static_cast<int>(
                         std::floor((bbMax.z - bbMin.z) / params.cloth_resolution)) +
                     2 * clothbuffer_d;

    std::cout << "[" << this->index << "] Configuring cloth..." << std::endl;
    std::cout << "[" << this->index << "]  - width: " << width_num << " "
              << "height: " << height_num << std::endl;
    last_profile.cloth_width = width_num;
    last_profile.cloth_height = height_num;
    last_profile.particle_count = static_cast<std::size_t>(width_num) * static_cast<std::size_t>(height_num);

    // ------------------------------------------------------------------
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
    //   params.time_step        — 时间步长
    // ------------------------------------------------------------------
    stageStart = Clock::now();
    Cloth cloth(
        origin_pos,
        width_num,
        height_num,
        params.cloth_resolution,
        params.cloth_resolution,
        0.3,
        9999,
        params.rigidness,
        params.time_step,
        params.opt_memory_optimized);
    last_profile.cloth_init_ms = elapsedMs(stageStart, Clock::now());

    // ------------------------------------------------------------------
    // 步骤4：光栅化 — 把点云的高度信息"印"到布料网格上
    //
    // 原理：
    //   对每个 LiDAR 点，计算它落在布料网格的哪个格子里
    //   把该点的高度记录到对应粒子的 nearestPointHeight 属性
    //   如果一个粒子范围内没有 LiDAR 点，就用扫描线/BFS找邻居的高度填充
    //
    // 输出：cloth.getHeightvals() 得到每个粒子的"地形高度"数组
    //       后续碰撞检测时，粒子不能低于这个高度
    // ------------------------------------------------------------------
    std::cout << "[" << this->index << "] Rasterizing..." << std::endl;
    stageStart = Clock::now();
    Rasterization::RasterTerrian(cloth, point_cloud, cloth.getHeightvals(), params.opt_memory_optimized);
    last_profile.rasterization_ms = elapsedMs(stageStart, Clock::now());

    // ------------------------------------------------------------------
    // 步骤5：物理模拟循环
    // ------------------------------------------------------------------

    double time_step2 = params.time_step * params.time_step;
    double gravity = 0.2;

    // 5a) 给所有粒子施加重力
    // 力的方向：(0, -0.2, 0) * time_step²
    // 这会让所有粒子获得一个 Y 轴负方向的加速度，使粒子"下落"
    std::cout << "[" << this->index << "] Simulating..." << std::endl;
    cloth.addForce(Vec3(0, -gravity, 0) * time_step2);
    /*

    Verlet 核心公式
    x_new = x + (x - x_old) + a * Δt²
    
    📌 各项解释：
    x         当前坐标
    x_old     上一帧坐标
    (x-x_old) ≈ 速度 * Δt
    aΔt²      加速度带来的位移
    */
    // 5b) 迭代循环（默认最多500次）
    // 每次迭代做两件事：
    //   a) timeStep():      Verlet积分更新位置 + 约束满足校正
    //                       返回本轮所有可移动粒子的最大位移
    //   b) terrCollision(): 碰撞检测——粒子不能低于地形
    //   c) 早停判断:        maxDiff < 0.005 说明布料基本不动了，提前退出
    Clock::time_point simulationStart = Clock::now();
    ClothStepProfile stepProfile;
    for (int i = 0; i < params.interations; i++)
    {
        Clock::time_point timestepStart = Clock::now();
        double maxDiff = cloth.timeStep(&stepProfile);
        last_profile.timestep_ms += elapsedMs(timestepStart, Clock::now());

        Clock::time_point collisionStart = Clock::now();
        cloth.terrCollision();
        last_profile.collision_ms += elapsedMs(collisionStart, Clock::now());
        last_profile.iterations_run = i + 1;
        if ((maxDiff != 0) && (maxDiff < 0.005))
        {
            // early stop — 布料已收敛，提前退出
            break;
        }
    }
    last_profile.simulation_ms = elapsedMs(simulationStart, Clock::now());
    last_profile.verlet_ms = stepProfile.verlet_ms;
    last_profile.constraint_ms = stepProfile.constraint_ms;
    last_profile.maxdiff_ms = stepProfile.maxdiff_ms;

    // ------------------------------------------------------------------
    // 步骤6：坡度平滑后处理（可选）
    //
    // 模拟结束后，布料上可能还有些粒子悬在空中（可移动状态）
    // 这些粒子通常出现在：建筑物顶边缘、陡坡边缘
    // 后处理逻辑：找到这些悬空粒子的连通区域，
    //   如果连通区域边缘有已落地粒子，且高差不大，就把悬空粒子也拉到地面
    //   这样可以修复坡面处布料悬空的"洞"
    // ------------------------------------------------------------------
    if (params.bSloopSmooth)
    {
        std::cout << "[" << this->index << "]  - post handle..." << std::endl;
        stageStart = Clock::now();
        cloth.movableFilter();
        last_profile.postprocess_ms = elapsedMs(stageStart, Clock::now());
    }

    return cloth;
}

// ==========================================================================
// do_cloth_export — 运行布料模拟并导出布料坐标
// 返回值：平铺的一维数组，每3个double代表一个粒子的 (X, Z, -Y)
//         注意这里把 Y 翻转回 -Y，恢复成原始的高程方向
// ==========================================================================
std::vector<double> CSF::do_cloth_export()
{
    auto cloth = do_cloth();
    return cloth.toVector();
}

void CSF::do_filtering_soa(std::vector<int> &groundIndexes,
                           std::vector<int> &offGroundIndexes,
                           bool exportCloth)
{
    (void)exportCloth;
    Clock::time_point filteringStart = Clock::now();

    initializeProfile(params.opt_deterministic_soa ? "deterministic_soa" : "soa");

    std::cout << "[" << this->index << "] Configuring terrain..." << std::endl;
    csf::Point bbMin, bbMax;
    Clock::time_point stageStart = Clock::now();
    point_cloud.computeBoundingBox(bbMin, bbMax);
    last_profile.bounding_box_ms = elapsedMs(stageStart, Clock::now());
    std::cout << "[" << this->index << "]  - bbMin: " << bbMin.x << " " << bbMin.y << " " << bbMin.z << std::endl;
    std::cout << "[" << this->index << "]  - bbMax: " << bbMax.x << " " << bbMax.y << " " << bbMax.z << std::endl;

    double cloth_y_height = 0.05;
    int clothbuffer_d = 2;
    Vec3 origin_pos(
        bbMin.x - clothbuffer_d * params.cloth_resolution,
        bbMax.y + cloth_y_height,
        bbMin.z - clothbuffer_d * params.cloth_resolution
    );

    int width_num = static_cast<int>(
                        std::floor((bbMax.x - bbMin.x) / params.cloth_resolution)) +
                    2 * clothbuffer_d;
    int height_num = static_cast<int>(
                         std::floor((bbMax.z - bbMin.z) / params.cloth_resolution)) +
                     2 * clothbuffer_d;

    std::cout << "[" << this->index << "] Configuring cloth (SoA)..." << std::endl;
    std::cout << "[" << this->index << "]  - width: " << width_num << " "
              << "height: " << height_num << std::endl;
    last_profile.cloth_width = width_num;
    last_profile.cloth_height = height_num;
    last_profile.particle_count = static_cast<std::size_t>(width_num) * static_cast<std::size_t>(height_num);

    stageStart = Clock::now();
    ClothSoA cloth(
        origin_pos,
        width_num,
        height_num,
        params.cloth_resolution,
        params.cloth_resolution,
        params.rigidness,
        params.time_step);
    last_profile.cloth_init_ms = elapsedMs(stageStart, Clock::now());

    std::cout << "[" << this->index << "] Rasterizing (SoA)..." << std::endl;
    stageStart = Clock::now();
    RasterizationSoA::RasterTerrain(cloth, point_cloud);
    last_profile.rasterization_ms = elapsedMs(stageStart, Clock::now());

    double time_step2 = params.time_step * params.time_step;
    double gravity = 0.2;

    std::cout << "[" << this->index << "] Simulating (SoA)..." << std::endl;
    cloth.addForceY(-gravity * time_step2);

    Clock::time_point simulationStart = Clock::now();
    ClothStepProfile stepProfile;
    for (int i = 0; i < params.interations; i++)
    {
        Clock::time_point timestepStart = Clock::now();
        double maxDiff = cloth.timeStep(&stepProfile);
        last_profile.timestep_ms += elapsedMs(timestepStart, Clock::now());

        Clock::time_point collisionStart = Clock::now();
        cloth.terrCollision();
        last_profile.collision_ms += elapsedMs(collisionStart, Clock::now());
        last_profile.iterations_run = i + 1;
        if ((maxDiff != 0) && (maxDiff < 0.005))
        {
            break;
        }
    }
    last_profile.simulation_ms = elapsedMs(simulationStart, Clock::now());
    last_profile.verlet_ms = stepProfile.verlet_ms;
    last_profile.constraint_ms = stepProfile.constraint_ms;
    last_profile.maxdiff_ms = stepProfile.maxdiff_ms;

    c2cdistSoA c2c(params.class_threshold);
    Clock::time_point classificationStart = Clock::now();
    c2c.calCloud2CloudDist(cloth, point_cloud, groundIndexes, offGroundIndexes);
    last_profile.classification_ms = elapsedMs(classificationStart, Clock::now());
    last_profile.total_filtering_ms = elapsedMs(filteringStart, Clock::now());
    last_profile.ground_count = groundIndexes.size();
    last_profile.non_ground_count = offGroundIndexes.size();
    if (last_profile.point_count > 0) {
        last_profile.ground_ratio = static_cast<double>(last_profile.ground_count) /
                                    static_cast<double>(last_profile.point_count);
    }
    updateLastProfileJson();
}

void CSF::do_filtering_gpu(std::vector<int> &groundIndexes,
                           std::vector<int> &offGroundIndexes,
                           bool exportCloth)
{
    (void)exportCloth;
    Clock::time_point filteringStart = Clock::now();
    initializeProfile("gpu");

#ifndef CSF_ENABLE_CUDA
    last_profile.backend_error = cudaBuildRequiredMessage();
    updateLastProfileJson();
    throw std::runtime_error(last_profile.backend_error);
#else
    if (params.opt_gpu_rasterization) {
        last_profile.backend_error = "gpu_rasterization_not_supported_first_version";
        updateLastProfileJson();
        throw std::runtime_error(last_profile.backend_error);
    }

    try {
        std::cout << "[" << this->index << "] Configuring terrain..." << std::endl;
        csf::Point bbMin, bbMax;
        Clock::time_point stageStart = Clock::now();
        point_cloud.computeBoundingBox(bbMin, bbMax);
        last_profile.bounding_box_ms = elapsedMs(stageStart, Clock::now());
        std::cout << "[" << this->index << "]  - bbMin: " << bbMin.x << " " << bbMin.y << " " << bbMin.z << std::endl;
        std::cout << "[" << this->index << "]  - bbMax: " << bbMax.x << " " << bbMax.y << " " << bbMax.z << std::endl;

        double cloth_y_height = 0.05;
        int clothbuffer_d = 2;
        Vec3 origin_pos(
            bbMin.x - clothbuffer_d * params.cloth_resolution,
            bbMax.y + cloth_y_height,
            bbMin.z - clothbuffer_d * params.cloth_resolution
        );

        int width_num = static_cast<int>(
                            std::floor((bbMax.x - bbMin.x) / params.cloth_resolution)) +
                        2 * clothbuffer_d;
        int height_num = static_cast<int>(
                             std::floor((bbMax.z - bbMin.z) / params.cloth_resolution)) +
                         2 * clothbuffer_d;

        std::cout << "[" << this->index << "] Configuring cloth (GPU SoA)..." << std::endl;
        std::cout << "[" << this->index << "]  - width: " << width_num << " "
                  << "height: " << height_num << std::endl;
        last_profile.cloth_width = width_num;
        last_profile.cloth_height = height_num;
        last_profile.particle_count = static_cast<std::size_t>(width_num) * static_cast<std::size_t>(height_num);

        stageStart = Clock::now();
        ClothSoA cloth(
            origin_pos,
            width_num,
            height_num,
            params.cloth_resolution,
            params.cloth_resolution,
            params.rigidness,
            params.time_step);
        last_profile.cloth_init_ms = elapsedMs(stageStart, Clock::now());

        std::cout << "[" << this->index << "] Rasterizing (CPU SoA for GPU path)..." << std::endl;
        stageStart = Clock::now();
        RasterizationSoA::RasterTerrain(cloth, point_cloud);
        last_profile.rasterization_ms = elapsedMs(stageStart, Clock::now());

        double time_step2 = params.time_step * params.time_step;
        double gravity = 0.2;
        cloth.addForceY(-gravity * time_step2);

        if (params.opt_gpu_simulation) {
            std::cout << "[" << this->index << "] Simulating (CUDA)..." << std::endl;
            ClothStepProfile stepProfile;
            csf_cuda::runSimulation(
                cloth,
                params.interations,
                params.rigidness,
                time_step2,
                params.gpu_device_id,
                stepProfile,
                last_profile.timestep_ms,
                last_profile.collision_ms,
                last_profile.simulation_ms,
                last_profile.iterations_run);
            last_profile.verlet_ms = stepProfile.verlet_ms;
            last_profile.constraint_ms = stepProfile.constraint_ms;
            last_profile.maxdiff_ms = stepProfile.maxdiff_ms;
        } else {
            std::cout << "[" << this->index << "] Simulating (CPU deterministic SoA)..." << std::endl;
            Clock::time_point simulationStart = Clock::now();
            ClothStepProfile stepProfile;
            for (int i = 0; i < params.interations; i++)
            {
                Clock::time_point timestepStart = Clock::now();
                double maxDiff = cloth.timeStep(&stepProfile);
                last_profile.timestep_ms += elapsedMs(timestepStart, Clock::now());

                Clock::time_point collisionStart = Clock::now();
                cloth.terrCollision();
                last_profile.collision_ms += elapsedMs(collisionStart, Clock::now());
                last_profile.iterations_run = i + 1;
                if ((maxDiff != 0) && (maxDiff < 0.005))
                {
                    break;
                }
            }
            last_profile.simulation_ms = elapsedMs(simulationStart, Clock::now());
            last_profile.verlet_ms = stepProfile.verlet_ms;
            last_profile.constraint_ms = stepProfile.constraint_ms;
            last_profile.maxdiff_ms = stepProfile.maxdiff_ms;
        }

        Clock::time_point classificationStart = Clock::now();
        if (params.opt_gpu_classification) {
            std::cout << "[" << this->index << "] Classifying (CUDA)..." << std::endl;
            csf_cuda::classify(
                cloth,
                point_cloud,
                params.class_threshold,
                params.gpu_device_id,
                groundIndexes,
                offGroundIndexes);
        } else {
            c2cdistSoA c2c(params.class_threshold);
            c2c.calCloud2CloudDist(cloth, point_cloud, groundIndexes, offGroundIndexes);
        }
        last_profile.classification_ms = elapsedMs(classificationStart, Clock::now());
        last_profile.total_filtering_ms = elapsedMs(filteringStart, Clock::now());
        last_profile.ground_count = groundIndexes.size();
        last_profile.non_ground_count = offGroundIndexes.size();
        if (last_profile.point_count > 0) {
            last_profile.ground_ratio = static_cast<double>(last_profile.ground_count) /
                                        static_cast<double>(last_profile.point_count);
        }
        updateLastProfileJson();
    } catch (const std::exception& e) {
        last_profile.backend_error = e.what();
        last_profile.total_filtering_ms = elapsedMs(filteringStart, Clock::now());
        updateLastProfileJson();
        throw;
    }
#endif
}

// ==========================================================================
// do_filtering — 完整的滤波流程（对外暴露的主接口）
//
// CSFDemo 和 Python 绑定都调用这个方法。
//
// 流程：
//   1. 调用 do_cloth() 运行布料模拟，得到 Cloth 对象
//   2. （可选）把布料保存到文件
//   3. 调用 c2cdist 计算每个点到布面的距离，按阈值分类
//
// 参数：
//   groundIndexes    — 输出：地面点在原始点云中的索引
//   offGroundIndexes — 输出：非地面点的索引
//   exportCloth      — 是否把布料坐标保存到文件（调试用）
// ==========================================================================

// ========================================================================
// COPY From CSFDemo.cpp
// 第四部分：执行滤波！这是核心调用
// do_filtering 内部会依次执行：
//   1. 计算点云包围盒
//   2. 创建布料（粒子网格 + 约束）
//   3. 光栅化：把点云投影到布料网格上
//   4. 物理模拟循环：重力→Verlet积分→约束满足→碰撞检测，重复N次
//   5. 坡度平滑后处理（如果开启）
//   6. 计算每个点到布面的距离，按阈值分类
//
// 输出：groundIndexes（地面点的索引）和 offGroundIndexes（非地面点的索引）
// ========================================================================
void CSF::do_filtering(std::vector<int> &groundIndexes,
                       std::vector<int> &offGroundIndexes,
                       bool exportCloth)
{
    bool hasOptimizationConfig = params.opt_memory_optimized ||
                                 params.opt_deterministic_soa ||
                                 params.opt_gpu_enabled ||
                                 params.opt_gpu_simulation ||
                                 params.opt_gpu_classification ||
                                 params.opt_gpu_rasterization ||
                                 (params.gpu_device_id != 0);

    if (hasOptimizationConfig) {
        try {
            validateOptimizationConfig(exportCloth);
        } catch (const std::exception& e) {
            initializeProfile("config_error");
            last_profile.backend_error = e.what();
            updateLastProfileJson();
            throw;
        }

        if (params.opt_gpu_enabled) {
            do_filtering_gpu(groundIndexes, offGroundIndexes, exportCloth);
            return;
        }

        if (params.opt_deterministic_soa) {
            do_filtering_soa(groundIndexes, offGroundIndexes, exportCloth);
            return;
        }
    } else {
        if (params.useSoA && !params.bSloopSmooth && !exportCloth) {
            do_filtering_soa(groundIndexes, offGroundIndexes, exportCloth);
            return;
        }
    }

    bool fallbackToLegacy = params.useSoA;
    std::string fallbackReason;
    if (fallbackToLegacy) {
        if (params.bSloopSmooth) {
            fallbackReason = "soa_postprocess_not_supported";
        } else if (exportCloth) {
            fallbackReason = "soa_export_cloth_not_supported";
        }
    }

    Clock::time_point filteringStart = Clock::now();

    // 步骤1~6：运行布料模拟
    auto cloth = do_cloth();
    if (fallbackToLegacy) {
        last_profile.fallback_used = true;
        last_profile.backend_fallback_reason = fallbackReason;
    }
    // （可选）导出布料到文件
    if (exportCloth)
        cloth.saveToFile();
    // 步骤7：距离分类
    // 对每个原始点，用双线性插值算出该点位置处布面的高度
    // 点高度与布面高度的差值 < class_threshold → 地面点，否则非地面点
    c2cdist c2c(params.class_threshold);
    Clock::time_point classificationStart = Clock::now();
    c2c.calCloud2CloudDist(cloth, point_cloud, groundIndexes, offGroundIndexes);
    last_profile.classification_ms = elapsedMs(classificationStart, Clock::now());
    last_profile.total_filtering_ms = elapsedMs(filteringStart, Clock::now());
    last_profile.ground_count = groundIndexes.size();
    last_profile.non_ground_count = offGroundIndexes.size();
    if (last_profile.point_count > 0) {
        last_profile.ground_ratio = static_cast<double>(last_profile.ground_count) /
                                    static_cast<double>(last_profile.point_count);
    }
    updateLastProfileJson();
}

std::string CSF::getLastProfileJson() const
{
    return last_profile_json;
}

void CSF::updateLastProfileJson()
{
    std::ostringstream out;
    out << std::fixed << std::setprecision(6);
    out << "{";
    out << "\"backend\":\"" << jsonEscape(last_profile.backend) << "\",";
    out << "\"fallback_used\":" << (last_profile.fallback_used ? "true" : "false") << ",";
    out << "\"backend_fallback_reason\":\"" << jsonEscape(last_profile.backend_fallback_reason) << "\",";
    out << "\"memory_optimized\":" << (last_profile.memory_optimized ? "true" : "false") << ",";
    out << "\"deterministic_soa\":" << (last_profile.deterministic_soa ? "true" : "false") << ",";
    out << "\"gpu_enabled\":" << (last_profile.gpu_enabled ? "true" : "false") << ",";
    out << "\"gpu_simulation\":" << (last_profile.gpu_simulation ? "true" : "false") << ",";
    out << "\"gpu_classification\":" << (last_profile.gpu_classification ? "true" : "false") << ",";
    out << "\"gpu_rasterization\":" << (last_profile.gpu_rasterization ? "true" : "false") << ",";
    out << "\"gpu_device_id\":" << last_profile.gpu_device_id << ",";
    out << "\"backend_error\":\"" << jsonEscape(last_profile.backend_error) << "\",";
    out << "\"point_count\":" << last_profile.point_count << ",";
    out << "\"cloth_width\":" << last_profile.cloth_width << ",";
    out << "\"cloth_height\":" << last_profile.cloth_height << ",";
    out << "\"particle_count\":" << last_profile.particle_count << ",";
    out << "\"iterations_configured\":" << last_profile.iterations_configured << ",";
    out << "\"iterations_run\":" << last_profile.iterations_run << ",";
    out << "\"ground_count\":" << last_profile.ground_count << ",";
    out << "\"non_ground_count\":" << last_profile.non_ground_count << ",";
    out << "\"ground_ratio\":" << last_profile.ground_ratio << ",";
    out << "\"cloth_resolution\":" << last_profile.cloth_resolution << ",";
    out << "\"rigidness\":" << last_profile.rigidness << ",";
    out << "\"time_step\":" << last_profile.time_step << ",";
    out << "\"class_threshold\":" << last_profile.class_threshold << ",";
    out << "\"bSloopSmooth\":" << (last_profile.bSloopSmooth ? "true" : "false") << ",";
    out << "\"bounding_box_ms\":" << last_profile.bounding_box_ms << ",";
    out << "\"cloth_init_ms\":" << last_profile.cloth_init_ms << ",";
    out << "\"rasterization_ms\":" << last_profile.rasterization_ms << ",";
    out << "\"simulation_ms\":" << last_profile.simulation_ms << ",";
    out << "\"timestep_ms\":" << last_profile.timestep_ms << ",";
    out << "\"verlet_ms\":" << last_profile.verlet_ms << ",";
    out << "\"constraint_ms\":" << last_profile.constraint_ms << ",";
    out << "\"maxdiff_ms\":" << last_profile.maxdiff_ms << ",";
    out << "\"collision_ms\":" << last_profile.collision_ms << ",";
    out << "\"postprocess_ms\":" << last_profile.postprocess_ms << ",";
    out << "\"classification_ms\":" << last_profile.classification_ms << ",";
    out << "\"total_filtering_ms\":" << last_profile.total_filtering_ms;
    out << "}";
    last_profile_json = out.str();
}

// ==========================================================================
// savePoints — 把指定索引组的点保存到文本文件
// 输出格式：每行 X Z -Y（注意这里把内部坐标翻转回原始方向）
// ==========================================================================
void CSF::savePoints(std::vector<int> grp, std::string path)
{
    if (path == "")
    {
        return;
    }

    std::ofstream f1(path.c_str(), std::ios::out);

    if (!f1)
        return;

    for (std::size_t i = 0; i < grp.size(); i++)
    {
        // 输出时把内部坐标转回原始坐标：X不变, Z→原始Y, -Y→原始Z
        f1 << std::fixed << std::setprecision(8)
           << point_cloud[grp[i]].x << "	"      // X（东向）
           << point_cloud[grp[i]].z << "	"      // Z = 原始Y（北向）
           << -point_cloud[grp[i]].y << std::endl; // -Y = 原始Z（高程）
    }

    f1.close();
}
