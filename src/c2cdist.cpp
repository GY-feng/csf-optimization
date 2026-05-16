// ======================================================================================
// c2cdist.cpp — 地面/非地面判别
//
// 这一步在布料模拟完成后执行，是滤波的最后一步。
// 对每个原始点云点，计算它到布面的距离，按阈值分类：
//   距离 < class_threshold → 地面点
//   距离 >= class_threshold → 非地面点（建筑物、树木等）
// ===========================================================================

#include "c2cdist.h"
#include <cmath>


// ==========================================================================
// calCloud2CloudDist — 计算点云到布面的距离并分类
//
// 参数：
//   cloth           — 模拟完成后的布料对象
//   pc              — 原始点云
//   groundIndexes   — 输出：地面点在原始点云中的索引
//   offGroundIndexes— 输出：非地面点的索引
//
// 方法：
//   1. 对每个点云点，找到布料网格中对应的4个粒子
//   2. 用双线性插值计算该点位置处布面的高度
//   3. 计算点高度与布面高度的差值
//   4. 按阈值分类
// ==========================================================================
void c2cdist::calCloud2CloudDist(Cloth           & cloth,
                                csf::PointCloud & pc,
                                std::vector<int>& groundIndexes,
                                std::vector<int>& offGroundIndexes) {
    groundIndexes.resize(0);
    offGroundIndexes.resize(0);

    // 遍历所有点云点
    for (std::size_t i = 0; i < pc.size(); i++) {
        double pc_x = pc[i].x;  // 点的X坐标（内部坐标系）
        double pc_z = pc[i].z;  // 点的Z坐标（内部坐标系）

        // 计算点落在布料网格的哪一列、哪一行
        double deltaX = pc_x - cloth.origin_pos.f[0];
        double deltaZ = pc_z - cloth.origin_pos.f[2];
        int col0 = int(deltaX / cloth.step_x);  // 四舍五入取整
        int row0 = int(deltaZ / cloth.step_y);
        int col1 = col0 + 1;  // 右列
        int row1 = row0;     // 同行
        int col2 = col0 + 1;  // 右列
        int row2 = row0 + 1; // 下一行
        int col3 = col0;     // 同列
        int row3 = row0 + 1; // 下一行

        // 计算点在网格单元内的相对位置（0~1）
        double subdeltaX = (deltaX - col0 * cloth.step_x) / cloth.step_x;
        double subdeltaZ = (deltaZ - row0 * cloth.step_y) / cloth.step_y;

        // 双线性插值计算布面高度
        // 公式：f(x,z) = f(0,0)*(1-x)*(1-z) + f(0,1)*(1-x)*z + f(1,1)*x*z + f(1,0)*x*(1-z)
        // 其中 (x,z) 是点的相对位置，f(i,j) 是4个角粒子的Y值（布面高度）
        double fxy = cloth.getParticle(col0, row0)->pos.f[1] * (1 - subdeltaX) * (1 - subdeltaZ) +
                    cloth.getParticle(col3, row3)->pos.f[1] * (1 - subdeltaX) * subdeltaZ +
                    cloth.getParticle(col2, row2)->pos.f[1] * subdeltaX * subdeltaZ +
                    cloth.getParticle(col1, row1)->pos.f[1] * subdeltaX * (1 - subdeltaZ);

        // 计算点高度与布面高度的差值
        double height_var = fxy - pc[i].y;

        // 按阈值分类
        if (std::fabs(height_var) < class_treshold) {
            groundIndexes.push_back(i);    // 地面点
        } else {
            offGroundIndexes.push_back(i); // 非地面点
        }
    }
}