// ======================================================================================
// XYZReader.cpp — 点云文件读取器
//
// 负责从文本文件读取点云数据，文件格式：
//   每行 X Y Z，用空格或Tab分隔
// 读入后会把坐标转换成内部坐标系：
//   Y = -Z（原始高程取负）
//   Z = Y（原始北向）
// 这样布料模拟时Y轴就是重力方向
// ===========================================================================

#include "XYZReader.h"
#include <sstream>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <cstdlib>


// ==========================================================================
// read_xyz — 从文件读取点云
//
// 参数：
//   fname: 文件名
//   pointcloud: 输出，读取的点云数据
//
// 文件格式：每行 X Y Z，空格或Tab分隔
// 坐标变换：Y = -Z, Z = Y
// ==========================================================================
void read_xyz(std::string fname, csf::PointCloud& pointcloud) {
    std::ifstream fin(fname.c_str(), std::ios::in);
    char     line[500];
    std::string   x, y, z;

    while (fin.getline(line, sizeof(line))) {
        std::stringstream words(line);

        words >> x;  // 读X
        words >> y;  // 读Y
        words >> z;  // 读Z

        csf::Point point;
        point.x = atof(x.c_str());   // X不变
        point.y = -atof(z.c_str()); // Y = -Z（高程取负）
        point.z = atof(y.c_str());   // Z = Y（北向）
        pointcloud.push_back(point);
    }
}