// ======================================================================================
// Particle.cpp — 布料粒子的物理更新细节
//
// 这个文件包含粒子运动的核心物理计算：
//   1. timeStep() - Verlet 积分更新位置
//   2. satisfyConstraintSelf() - 约束满足（让相邻粒子保持距离）
//
// 想象：每个粒子是一个小点，有位置、速度、加速度，
//       相邻粒子之间有"弹簧"（约束）连接，弹簧会拉住粒子不让它们离太远或太近。
// ======================================================================================

#include "Particle.h"


// ==========================================================================
// timeStep — Verlet 积分更新粒子位置
//
// 这是粒子运动的核心，被 Cloth::timeStep() 调用。
// Verlet 积分是一种数值积分方法，比简单的欧拉法更稳定。
//
// 公式：pos_new = pos + (pos - old_pos) * (1 - DAMPING) + acceleration * time_step2
//
// 参数解释：
//   pos:       当前位置
//   old_pos:   上一次的位置（用来计算速度）
//   acceleration: 当前加速度（由力计算得到，F = ma，m=1）
//   time_step2: time_step * time_step（时间步长的平方）
//   DAMPING:   阻尼系数（0.01），让运动逐渐衰减，避免振荡
//
// 为什么要用 Verlet？
//   比欧拉法更稳定，对时间步长不敏感，适合物理模拟
// ==========================================================================
void Particle::timeStep() {
    if (movable) {
        Vec3 temp = pos;  // 保存当前位置，用于计算速度
        // Verlet 积分公式：新位置 = 当前位置 + 速度项 + 加速度项
        pos = pos + (pos - old_pos) * (1.0 - DAMPING) + acceleration * time_step2;
        old_pos = temp;  // 更新 old_pos 为当前（计算前的）位置
    }
}


// ==========================================================================
// satisfyConstraintSelf — 约束满足（让相邻粒子保持距离）
//
// 被 Cloth::timeStep() 调用，在位置更新后执行。
// 作用：检查粒子与所有邻居的距离，如果差距太大就把它拉回来。
//
// 约束类型：
//   1. 双方可移动：两个粒子都可以移动，各移动一半距离
//   2. 单方可移动：只有一个粒子可移动，它移动全部距离
//   3. 双方都不可移动：不做任何事（已经都落地了）
//
// 参数：constraintTimes - 约束迭代次数（等于刚性等级）
//       决定了校正的力度：刚性越强，每次移动的距离越大
// ==========================================================================
void Particle::satisfyConstraintSelf(int constraintTimes) {
    Particle *p1 = this;  // 当前粒子

    // 遍历所有邻居（约束）
    for (std::size_t i = 0; i < neighborsList.size(); i++) {
        Particle *p2 = neighborsList[i];  // 邻居粒子
        // 计算Y方向校正向量（只校正Y方向，因为布料主要在Y方向运动）
        Vec3 correctionVector(0, p2->pos.f[1] - p1->pos.f[1], 0);

        // 情况1：双方都可移动
        if (p1->isMovable() && p2->isMovable()) {
            // 校正向量的一半，这样两个粒子各移动一半距离
            Vec3 correctionVectorHalf = correctionVector * (
                constraintTimes > 14 ? 0.5 : doubleMove1[constraintTimes]
            );
            p1->offsetPos(correctionVectorHalf);      // p1 向上移动
            p2->offsetPos(-correctionVectorHalf);     // p2 向下移动
        } 
        // 情况2：只有p1可移动
        else if (p1->isMovable() && !p2->isMovable()) {
            // p1 移动全部距离，p2 不动
            Vec3 correctionVectorHalf = correctionVector * (
                constraintTimes > 14 ? 1 : singleMove1[constraintTimes]
            );
            p1->offsetPos(correctionVectorHalf);
        } 
        // 情况3：只有p2可移动
        else if (!p1->isMovable() && p2->isMovable()) {
            // p2 移动全部距离，p1 不动
            Vec3 correctionVectorHalf = correctionVector * (
                constraintTimes > 14 ? 1 : singleMove1[constraintTimes]
            );
            p2->offsetPos(-correctionVectorHalf);
        }
        // 情况4：双方都不可移动（都已落地）- 不做任何事
    }
}