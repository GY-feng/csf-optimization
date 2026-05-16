// ======================================================================================
// Rasterization.cpp — 点云到布料的"光栅化"
//
// 这一步的核心任务：给布料上的每个粒子赋予一个"地形高度值"
//   这样后续碰撞检测才知道粒子应该停在哪里。
//
// 方法分两步：
//   1. RasterTerrian: 遍历所有LiDAR点，找到每个点对应的最近布料粒子，
//      把点的高度记到粒子上。一个粒子可能对应多个点，取最近那个。
//   2. 对于没有对应LiDAR点的粒子（空洞），用扫描线/BFS从邻居那里"借"高度
//
// 想象：你把一块布盖在点云上方，然后把每个点云点"投影"到布上最近的格子里，
//       格子里就记录了地形的高度。有些格子没被投影到，就从旁边的格子抄一个。
// ======================================================================================

#include "Rasterization.h"
#include <queue>


// ==========================================================================
// findHeightValByScanline — 扫描线法查找最近的有效高度值
//
// 当某个布料粒子没有对应的LiDAR点时（nearestPointHeight == MIN_INF），
// 需要从它周围找到有高度值的粒子来填充。
//
// 方法：从当前粒子出发，沿4个方向做"扫描线"搜索
//   → 向右扫描一行
//   → 向左扫描一行
//   → 向下扫描一列
//   → 向上扫描一列
// 哪个方向先找到有效高度就返回哪个（贪心，不保证最优）
//
// 如果4个方向都扫不到（极端情况：整个边界都没有有效点），
// 就退回到 BFS 邻居搜索 findHeightValByNeighbor
//
// 参数：p     — 需要填充高度的粒子指针
//       cloth — 布料对象引用（用来访问其他粒子）
// 返回：找到的高度值，或 MIN_INF（没找到）
// ==========================================================================
double Rasterization::findHeightValByScanline(Particle *p, Cloth& cloth) {
    int xpos = p->pos_x;  // 粒子在布料网格中的列号
    int ypos = p->pos_y;  // 粒子在布料网格中的行号

    // 向右扫描：从当前列往右，逐列检查有没有有效高度
    for (int i = xpos + 1; i < cloth.num_particles_width; i++) {
        double crresHeight = cloth.getParticle(i, ypos)->nearestPointHeight;
        if (crresHeight > MIN_INF)  // 找到有效高度就立刻返回
            return crresHeight;
    }

    // 向左扫描：从当前列往左
    for (int i = xpos - 1; i >= 0; i--) {
        double crresHeight = cloth.getParticle(i, ypos)->nearestPointHeight;
        if (crresHeight > MIN_INF)
            return crresHeight;
    }

    // 向下扫描：从当前行往上（y减小方向，注意这里y是行号不是坐标）
    for (int j = ypos - 1; j >= 0; j--) {
        double crresHeight = cloth.getParticle(xpos, j)->nearestPointHeight;
        if (crresHeight > MIN_INF)
            return crresHeight;
    }

    // 向上扫描：从当前行往下（y增大方向）
    for (int j = ypos + 1; j < cloth.num_particles_height; j++) {
        double crresHeight = cloth.getParticle(xpos, j)->nearestPointHeight;
        if (crresHeight > MIN_INF)
            return crresHeight;
    }

    // 四个方向都没找到，使用BFS搜索
    return findHeightValByNeighbor(p);
}


// ==========================================================================
// findHeightValByNeighbor — BFS邻居搜索查找有效高度值
//
// 这是扫描线法的兜底方案。当某个粒子四条扫描线都找不到有效高度时，
// 就用广度优先搜索在粒子的"约束邻居"中扩散搜索。
//
// 原理：从当前粒子出发，把所有邻居加入队列，逐个检查有没有有效高度
//       如果邻居也没有，就把邻居的邻居也加入队列，继续扩散
//       直到找到为止（理论上整个布料网格一定有至少一个有效高度）
//
// 注意：这个函数使用了 isVisited 标志来防止重复访问，
//       搜索完成后会把 isVisited 清除（恢复现场）
//
// 参数：p — 需要填充高度的粒子指针
// 返回：找到的高度值，或 MIN_INF（没找到，理论上不会发生）
// ==========================================================================
double Rasterization::findHeightValByNeighbor(Particle *p) {
    std::queue<Particle *>  nqueue;     // BFS 队列
    std::vector<Particle *> pbacklist;  // 记录所有访问过的粒子，最后恢复 isVisited

    // 把当前粒子的所有邻居加入队列
    int neiborsize = p->neighborsList.size();
    for (int i = 0; i < neiborsize; i++) {
        p->isVisited = true;
        nqueue.push(p->neighborsList[i]);
    }

    // BFS 搜索
    while (!nqueue.empty()) {
        Particle *pneighbor = nqueue.front();
        nqueue.pop();
        pbacklist.push_back(pneighbor);  // 记录，最后要恢复 isVisited

        if (pneighbor->nearestPointHeight > MIN_INF) {
            // 找到了！先恢复所有访问过的粒子的 isVisited 标志
            for (std::size_t i = 0; i < pbacklist.size(); i++)
                pbacklist[i]->isVisited = false;

            // 队列里剩余的粒子也要恢复
            while (!nqueue.empty()) {
                Particle *pp = nqueue.front();
                pp->isVisited = false;
                nqueue.pop();
            }

            return pneighbor->nearestPointHeight;
        } else {
            // 这个邻居也没有有效高度，把它的邻居也加入队列
            int nsize = pneighbor->neighborsList.size();
            for (int i = 0; i < nsize; i++) {
                Particle *ptmp = pneighbor->neighborsList[i];
                if (!ptmp->isVisited) {
                    ptmp->isVisited = true;
                    nqueue.push(ptmp);
                }
            }
        }
    }

    // 整个布料网格都没有有效高度（理论上不应该发生）
    return MIN_INF;
}

// ==========================================================================
// RasterTerrian — 点云光栅化的主函数
//
// 把 LiDAR 点云"印"到布料网格上，给每个布料粒子赋予地形高度。
//
// 流程分两步：
//   第一步（遍历点云）：对每个 LiDAR 点，算出它落在哪个布料格子里，
//       把点的高度记到格子的 nearestPointHeight 上。
//       如果一个格子有多个点，取水平距离最近的那个。（水平距离：点在平面上离哪个格子中心最近）
//   第二步（空洞填充）：遍历所有布料粒子，如果没有被任何点投影到，
//       就用扫描线或BFS从邻居那里"借"一个高度。
//
// 参数：
//   cloth     — 布料对象（粒子网格）
//   pc        — 点云数据
//   heightVal — 输出：每个粒子对应的地形高度数组
// ==========================================================================
void Rasterization::RasterTerrian(Cloth          & cloth,
                                  csf::PointCloud& pc,
                                  std::vector<double> & heightVal,
                                  bool memory_optimized) {

    // ==================================================================
    // 第一步：遍历所有 LiDAR 点，投影到布料网格（这个地方可以GPU？）
    // ==================================================================
    for (std::size_t i = 0; i < pc.size(); i++) {
        double pc_x = pc[i].x;  // 点的X坐标（内部坐标系）
        double pc_z = pc[i].z;  // 点的Z坐标（内部坐标系，对应原始Y/北向）

        // 计算该点落在布料网格的哪一列、哪一行
        // deltaX = 点X - 布料左下角X，除以步长再四舍五入 = 列号
        double deltaX = pc_x - cloth.origin_pos.f[0];
        double deltaZ = pc_z - cloth.origin_pos.f[2];
        int    col    = int(deltaX / cloth.step_x + 0.5);  // +0.5 实现四舍五入
        int    row    = int(deltaZ / cloth.step_y + 0.5);

        // 检查行列号是否在布料范围内
        if ((col >= 0) && (row >= 0)) {
            Particle *pt = cloth.getParticle(col, row);
            // 把这个点的索引加到粒子的对应点列表里
            if (!memory_optimized) {
                pt->correspondingLidarPointList.push_back(i);
            }

            // 计算点到粒子的水平距离（平方，不开根号节省计算）
            double pc2particleDist = SQUARE_DIST(
                pc_x, pc_z,
                pt->getPos().f[0],
                pt->getPos().f[2]
            );

            // 如果这个点比之前记录的点更近，就更新粒子的最近点信息
            if (pc2particleDist < pt->tmpDist) {
                pt->tmpDist            = pc2particleDist;         // 更新最小距离
                pt->nearestPointHeight = pc[i].y;                 // 记录该点的Y值（内部坐标的高程）
                pt->nearestPointIndex  = i;                        // 记录点云中的索引
            }
        }
    }

    // ==================================================================
    // 第二步：空洞填充
    // ==================================================================
    heightVal.resize(cloth.getSize());

    // 注意：这里的 OpenMP 并行被注释掉了！
    // 原因是 findHeightValByScanline 和 findHeightValByNeighbor
    // 使用了 isVisited 标志，并行化会有竞态条件
    // #ifdef CSF_USE_OPENMP
    // #pragma omp parallel for
    // #endif
    for (int i = 0; i < cloth.getSize(); i++) {
        Particle *pcur          = cloth.getParticle1d(i);
        double    nearestHeight = pcur->nearestPointHeight;

        if (nearestHeight > MIN_INF) {
            // 这个粒子有对应的 LiDAR 点，直接使用
            heightVal[i] = nearestHeight;
        } else {
            // 这个粒子没有被任何点投影到，需要从邻居那里"借"高度
            // 先用扫描线法（快），不行再用BFS（慢）
            heightVal[i] = findHeightValByScanline(pcur, cloth);
        }
    }
}
