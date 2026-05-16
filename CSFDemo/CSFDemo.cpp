// ======================================================================================
// CSFDemo.cpp — CSF 算法的命令行演示入口
// 这是整个项目"从参数文件到结果输出"的完整调用链示例。
// 如果你第一次看这个项目，从这里开始！
//
// 调用流程：
//   1. 从 params.cfg 读取所有参数
//   2. 读入点云文件
//   3. 设置 CSF 参数
//   4. 调用 do_filtering 执行布料模拟滤波
//   5. 保存地面点 / 非地面点到文件
// ======================================================================================

#include <vector>
#include "Cfg.h"
#include "../src/CSF.h" 
#include <locale.h>
#include <time.h>
#include <cstdlib>
#include <cstring>

int main(int argc,char* argv[])
{
	// ========================================================================
	// 第一部分：从 params.cfg 配置文件中读取参数
	// Cfg 是一个极简的 "key=value" 配置文件解析器（见 Cfg.h）
	// ========================================================================

	Cfg cfg;

	// --- 读取 slop_smooth（是否开启坡度平滑后处理）---
	// 这个参数比较特殊，配置文件里可能是 "true"/"false"/"0"/"1"
	// 所以需要做多种格式的兼容解析
	std::string slop_smooth;
	cfg.readConfigFile("params.cfg", "slop_smooth", slop_smooth);
	bool ss = false;
	if (slop_smooth == "true" || slop_smooth == "True")
	{
		ss = true;
	}
	else if (slop_smooth == "false" || slop_smooth == "False")
	{
		ss = false;
	}
	else{
		// 如果不是 true/false 字符串，就当作数字来解析：0=false, 非0=true
		if (atoi(slop_smooth.c_str()) == 0){
			ss = false;
		}
		else
		{
			ss = true;
		}
	}

	// --- 读取其余数值型参数 ---
	// class_threshold: 地面/非地面分类的距离阈值（米），点云到布面的距离小于此值认为是地面点
	std::string class_threshold;
	cfg.readConfigFile("params.cfg", "class_threshold", class_threshold);
	// cloth_resolution: 布料网格的间距（米），越小精度越高但粒子越多、越慢
	std::string cloth_resolution;
	cfg.readConfigFile("params.cfg", "cloth_resolution", cloth_resolution);
	// iterations: 物理模拟的最大迭代次数，布料要反复"下落-碰撞"这么多次
	std::string iterations;
	cfg.readConfigFile("params.cfg", "iterations", iterations);
	// rigidness: 布料的刚性等级（1=软/2=中/3=硬），影响约束校正的强度
	//   刚性越强，布料越不容易穿过建筑物，但也越不容易贴合地形起伏
	std::string rigidness;
	cfg.readConfigFile("params.cfg", "rigidness", rigidness);
	// time_step: 物理模拟的时间步长，影响每次迭代中粒子移动的幅度
	std::string time_step;
	cfg.readConfigFile("params.cfg", "time_step", time_step);
	// terr_pointClouds_filepath: 输入点云文件路径（.txt 格式，每行 X Y Z）
	std::string terr_pointClouds_filepath;
	cfg.readConfigFile("params.cfg", "terr_pointClouds_filepath", terr_pointClouds_filepath);

	// ========================================================================
	// 第二部分：创建 CSF 对象，读入点云
	// ========================================================================

	CSF csf;

	// 步骤1：读入点云
	// 内部会调用 XYZReader 把文本文件解析成 PointCloud 对象
	// 同时会做坐标变换：原始的 Y 变成 Z（水平方向），原始的 Z 取负变成 Y（重力方向）
	// 这样布料就可以沿 Y 轴向下"掉落"
	csf.readPointsFromFile(terr_pointClouds_filepath);

	// 计时开始
	clock_t start, end;
	start = clock();

	// 如果你的程序已经有 PointCloud 对象，也可以用 setPointCloud() 直接传入
	// csf.setPointCloud(pc);  // pc 是 csf::PointCloud 类型

	// ========================================================================
	// 第三部分：设置 CSF 参数
	// 所有参数都存在 csf.params 这个 Params 结构体里
	// ========================================================================

	csf.params.bSloopSmooth = ss;                             // 是否做坡度平滑后处理
	csf.params.class_threshold = atof(class_threshold.c_str()); // 分类距离阈值（米）
	csf.params.cloth_resolution = atof(cloth_resolution.c_str()); // 布料网格分辨率（米）
	csf.params.interations = atoi(iterations.c_str());        // 最大迭代次数
	csf.params.rigidness = atoi(rigidness.c_str());           // 刚性等级 1/2/3
	csf.params.time_step = atof(time_step.c_str());           // 时间步长

	// ========================================================================
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

	std::vector<int> groundIndexes, offGroundIndexes;

	// 如果命令行参数带了 "-c"，就把布料的最终形状也导出到文件（用于可视化调试）
	if (argc == 2 && strcmp(argv[1], "-c")==0)
	{
		std::cout << "Export cloth enabled." << std::endl;
		csf.do_filtering(groundIndexes, offGroundIndexes, true);  // 第3个参数=true表示导出布料
	}
	else
	{
		csf.do_filtering(groundIndexes, offGroundIndexes, false); // 默认不导出布料
	}
		
	// 计时结束
	end = clock();
	double dur = (double)(end - start);
	printf("Use Time:%f\n", (dur / CLOCKS_PER_SEC));

	// ========================================================================
	// 第五部分：保存结果
	// 把地面点和非地面点分别保存到文本文件
	// 文件格式与输入相同：每行 X Y Z
	// ========================================================================

	csf.savePoints(groundIndexes,"ground.txt");
	csf.savePoints(offGroundIndexes, "non-ground.txt");

	return 0;
}
