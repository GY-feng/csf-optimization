// ======================================================================================
// Vec3.h — 三维向量工具类
//
// 提供了基本的3D向量运算：
//   +, -, *, /, length(), normalized(), cross(), dot()
// 用于粒子位置、速度、加速度的计算
// ===========================================================================

#ifndef _VEC3_H_
#define _VEC3_H_


#include <cmath>
#include <string>
#include <iostream>
#include <iomanip>


// ==========================================================================
// Vec3 类 — 三维向量
// 提供了基本的向量运算，用于物理模拟中的位置、速度、加速度计算
// ==========================================================================
class Vec3 {
public:
    double f[3];  // 3个分量：f[0]=X, f[1]=Y, f[2]=Z

    // 构造函数
    Vec3(double x, double y, double z) {
        f[0] = x;
        f[1] = y;
        f[2] = z;
    }
    Vec3() {}

    // 向量长度（模）
    double length() {
        return sqrt(f[0] * f[0] + f[1] * f[1] + f[2] * f[2]);
    }

    // 单位向量（方向向量）
    Vec3 normalized() {
        double l = length();
        return Vec3(f[0] / l, f[1] / l, f[2] / l);
    }

    // 向量加法：this += v
    void operator+=(const Vec3& v) {
        f[0] += v.f[0];
        f[1] += v.f[1];
        f[2] += v.f[2];
    }

    // 向量除以标量：this / a
    Vec3 operator/(const double& a) {
        return Vec3(f[0] / a, f[1] / a, f[2] / a);
    }

    // 向量减法：this - v
    Vec3 operator-(const Vec3& v) {
        return Vec3(f[0] - v.f[0], f[1] - v.f[1], f[2] - v.f[2]);
    }

    // 向量加法：this + v
    Vec3 operator+(const Vec3& v) {
        return Vec3(f[0] + v.f[0], f[1] + v.f[1], f[2] + v.f[2]);
    }

    // 向量乘以标量：this * a
    Vec3 operator*(const double& a) {
        return Vec3(f[0] * a, f[1] * a, f[2] * a);
    }

    // 向量取负：-this
    Vec3 operator-() {
        return Vec3(-f[0], -f[1], -f[2]);
    }

    // 叉乘（叉积）
    Vec3 cross(const Vec3& v) {
        return Vec3(
            f[1] * v.f[2] - f[2] * v.f[1],
            f[2] * v.f[0] - f[0] * v.f[2],
            f[0] * v.f[1] - f[1] * v.f[0]
        );
    }

    // 点乘（点积）
    double dot(const Vec3& v) {
        return f[0] * v.f[0] + f[1] * v.f[1] + f[2] * v.f[2];
    }
};


#endif // ifndef _VEC3_H_