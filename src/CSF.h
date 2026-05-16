// ======================================================================================
// CSF.h — CSF（Cloth Simulation Filter）算法的主类头文件
//
// CSF 是整个算法的"门面类"（Facade），外部使用者只需要和这个类打交道。
// 它封装了从"输入点云"到"输出地面/非地面索引"的全部流程。
//
// 核心调用链：
//   CSF.setPointCloud()  →  输入点云
//   CSF.do_filtering()   →  执行滤波，输出地面/非地面索引
//       内部调用 do_cloth()，依次执行：
//         1. 计算点云包围盒
//         2. 创建布料 Cloth 对象
//         3. 光栅化 Rasterization
//         4. 物理模拟循环（重力+积分+约束+碰撞）
//         5. 坡度平滑后处理
//         6. c2cdist 距离分类
//
// 论文：Zhang W, Qi J, Wan P, et al. An Easy-to-Use Airborne LiDAR Data Filtering
//       Method Based on Cloth Simulation. Remote Sensing. 2016; 8(6):501.
// ======================================================================================

#ifndef _CSF_H_
#define _CSF_H_
#include <vector>
#include <string>
#include "point_cloud.h"
#include "Cloth.h"


// ==========================================================================
// Params 结构体 — 存放 CSF 算法的全部可调参数
// 这些参数的含义和调优建议参见 params.cfg 中的注释
// ==========================================================================
struct Params {
    // 是否开启坡度平滑后处理
    // 模拟结束后，布料上可能有些粒子还悬在空中（没碰到地面）
    // 开启后会把与已落地粒子高度差不大的悬空粒子也拉到地面
    bool bSloopSmooth;

    // 物理模拟的时间步长（默认0.65）
    // 影响每次迭代粒子移动的幅度，太大可能穿透地面，太小收敛慢
    double time_step;

    // 地面/非地面分类距离阈值，单位米（默认0.5）
    // 点到布面距离 < 此值 → 地面点
    double class_threshold;

    // 布料网格分辨率，单位米（默认1.0）
    // 布料上相邻粒子的水平间距，越小精度越高但越慢
    double cloth_resolution;

    // 布料刚性等级（默认3）
    // 1=软，2=中，3=硬
    // 决定了约束满足时每次校正的力度（查表 singleMove1 / doubleMove1）
    int rigidness;

    // 最大迭代次数（默认500）
    // 实际迭代可能更少（有早停：maxDiff < 0.005 时提前退出）
    int interations;  // 注意：原文拼写就是 interations（少了个 t），保持兼容不改

    // 是否启用 SoA 后端。默认 false，保持旧的 Particle/AoS 后端。
    bool useSoA;

    bool opt_memory_optimized;
    bool opt_deterministic_soa;
    bool opt_gpu_enabled;
    bool opt_gpu_simulation;
    bool opt_gpu_classification;
    bool opt_gpu_rasterization;
    int gpu_device_id;
};

// ==========================================================================
// DLL 导出宏（仅在定义了 _CSF_DLL_EXPORT_ 时生效）
// Windows 下用于从 DLL 中导出/导入类
// ==========================================================================
#ifdef _CSF_DLL_EXPORT_
# ifdef DLL_IMPLEMENT
#  define DLL_API    __declspec(dllexport)
# else // ifdef DLL_IMPLEMENT
#  define DLL_API    __declspec(dllimport)
# endif // ifdef DLL_IMPLEMENT
#endif // ifdef _CSF_DLL_EXPORT_

// ==========================================================================
// CSF 类 — 算法的主入口
// ==========================================================================
#ifdef _CSF_DLL_EXPORT_
class DLL_API CSF
#else // ifdef _CSF_DLL_EXPORT_
class CSF
#endif // ifdef _CSF_DLL_EXPORT_
{
public:

    CSF(int index);   // 带索引的构造函数（用于多实例并行时区分日志输出）
    CSF();            // 默认构造函数
    ~CSF();

    // ------------------------------------------------------------------
    // 点云输入方法（三种重载，适配不同的调用方式）
    // ------------------------------------------------------------------

    // 方式1：从 vector<csf::Point> 传入
    // 内部会做坐标变换：原始 (x,y,z) → 内部 (x, -z, y)
    // 即：Z轴取负变成Y轴（重力方向向下），Y轴变成Z轴（水平方向）
    void setPointCloud(std::vector<csf::Point> points);

    // 方式2：从 C 风格二维数组传入（行优先，N×3 矩阵）
    // Python numpy 数组通过 SWIG 绑定调用这个接口
    // points 指向一个 rows×cols 的 double 数组，cols 必须为 3
    void setPointCloud(double *points, int rows, int cols);

    // 方式3：从 C 风格一维数组传入（列优先，3×N 矩阵）
    // MATLAB 调用时使用这种列优先布局
    void setPointCloud(double *points, int rows);

    // 从文本文件读入点云（每行 X Y Z）
    void readPointsFromFile(std::string filename);

    // 获取内部点云的引用
    inline csf::PointCloud& getPointCloud() {
        return point_cloud;
    }

    inline const csf::PointCloud& getPointCloud() const {
        return point_cloud;
    }

    // 把指定索引组的点保存到文本文件
    void savePoints(std::vector<int> grp, std::string path);

    // 获取点云中点的数量
    std::size_t size() {
        return point_cloud.size();
    }

    // 从 csf::PointCloud 对象直接设置（避免拷贝）
    void setPointCloud(csf::PointCloud& pc);

    // ------------------------------------------------------------------
    // 核心方法：执行滤波
    // ------------------------------------------------------------------

    // do_filtering — 完整的滤波流程
    //   groundIndexes:    输出参数，存放地面点在原始点云中的索引
    //   offGroundIndexes: 输出参数，存放非地面点的索引
    //   exportCloth:      是否把布料形状保存到文件（调试用）
    void do_filtering(std::vector<int>& groundIndexes,
                      std::vector<int>& offGroundIndexes,
                      bool exportCloth=true);

    // do_cloth_export — 只运行布料模拟，返回布料所有粒子的坐标（平铺的一维数组）
    // 用于可视化布料形状
    std::vector<double> do_cloth_export();

    // 返回最近一次 do_filtering 的性能统计 JSON 字符串。
    std::string getLastProfileJson() const;

    void configureOptimization(bool memoryOptimized,
                               bool deterministicSoA,
                               bool gpuEnabled,
                               bool gpuSimulation,
                               bool gpuClassification,
                               bool gpuRasterization,
                               int gpuDeviceId);

private:

    struct ProfileStats {
        double bounding_box_ms;
        double cloth_init_ms;
        double rasterization_ms;
        double simulation_ms;
        double timestep_ms;
        double verlet_ms;
        double constraint_ms;
        double maxdiff_ms;
        double collision_ms;
        double postprocess_ms;
        double classification_ms;
        double total_filtering_ms;
        std::size_t point_count;
        int cloth_width;
        int cloth_height;
        std::size_t particle_count;
        int iterations_configured;
        int iterations_run;
        std::size_t ground_count;
        std::size_t non_ground_count;
        double ground_ratio;
        double cloth_resolution;
        int rigidness;
        double time_step;
        double class_threshold;
        bool bSloopSmooth;
        std::string backend;
        bool fallback_used;
        std::string backend_fallback_reason;
        bool memory_optimized;
        bool deterministic_soa;
        bool gpu_enabled;
        bool gpu_simulation;
        bool gpu_classification;
        bool gpu_rasterization;
        int gpu_device_id;
        std::string backend_error;

        ProfileStats();
    };

    // do_cloth — 执行布料模拟的核心方法（被 do_filtering 调用）
    // 流程：包围盒→布料创建→光栅化→物理模拟→后处理
    // 返回模拟完成后的 Cloth 对象
    Cloth do_cloth();

    void do_filtering_soa(std::vector<int>& groundIndexes,
                          std::vector<int>& offGroundIndexes,
                          bool exportCloth);

    void do_filtering_gpu(std::vector<int>& groundIndexes,
                          std::vector<int>& offGroundIndexes,
                          bool exportCloth);

    void validateOptimizationConfig(bool exportCloth) const;
    void initializeProfile(const std::string& backend);

    void updateLastProfileJson();
        
    // 内部存储的点云数据（经过坐标变换后的）
#ifdef _CSF_DLL_EXPORT_
    class __declspec (dllexport)csf::PointCloud point_cloud;
#else // ifdef _CSF_DLL_EXPORT_
    csf::PointCloud point_cloud;
#endif // ifdef _CSF_DLL_EXPORT_

public:

    Params params;  // 算法参数，外部可直接修改
    int index;      // 实例索引，用于多实例时的日志区分

private:
    ProfileStats last_profile;
    std::string last_profile_json;
};

#endif // ifndef _CSF_H_
