// ======================================================================================
// point_cloud.h — 点云数据结构定义
//
// 定义了 csf::Point 和 csf::PointCloud 两个核心类：
//   1. Point：单个点的3D坐标（X, Y, Z）
//   2. PointCloud：点的集合，继承自 std::vector<Point>
//   3. computeBoundingBox：计算点云的包围盒
// ===========================================================================

#ifndef _POINT_CLOUD_H_
#define _POINT_CLOUD_H_

#include <vector>

namespace csf {

// ==========================================================================
// Point 结构体 — 单个点的3D坐标
// 使用 union 让 x/y/z 和 u[3] 两种方式都能访问
// ==========================================================================
struct Point {
    union {
        struct {
            double x;
            double y;
            double z;
        };
        double u[3];
    };

    // 构造函数
    Point() : x(0), y(0), z(0) {}
    Point(double x, double y, double z) : x(x), y(y), z(z) {}
};

// ==========================================================================
// PointCloud 类 — 点云集合
// 继承自 std::vector<Point>，所以可以用 vector 的所有方法
// 增加了 computeBoundingBox 方法计算包围盒
// ==========================================================================
class PointCloud : public std::vector<Point>{
public:
    // 计算点云的最小包围盒
    // bbMin：各维度的最小值
    // bbMax：各维度的最大值
    void computeBoundingBox(Point& bbMin, Point& bbMax);
};

}

#endif // ifndef _POINT_CLOUD_H_