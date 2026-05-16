// ======================================================================================
// Constraint.cpp — 约束类（虽然没被使用，但代码还在）
//
// 这个文件定义了 Constraint 类，用于表示两个粒子之间的约束关系。
// 但在 Cloth.cpp 中，约束逻辑被内联到了 Particle::satisfyConstraintSelf 中，
// 所以这个类实际上没有被使用。
//
// 留在这里是为了理解原始设计意图：每个约束对象记录两个粒子指针，
// 以及它们在静止状态下的距离（rest_distance）。
// ===========================================================================

#include "Constraint.h"


// ==========================================================================
// satisfyConstraint — 约束满足（让两个粒子保持距离）
//
// 这个方法被设计用来让两个粒子保持它们在静止状态下的距离。
// 但在 Cloth.cpp 中，这个逻辑被直接写在了 Particle::satisfyConstraintSelf 里，
// 所以这个方法没有被调用。
//
// 参数：constraintTimes - 约束迭代次数，决定校正力度
// ==========================================================================
void Constraint::satisfyConstraint(int constraintTimes) {
    // 计算两个粒子Y方向的距离差
    Vec3 correctionVector(0, p2->pos.f[1] - p1->pos.f[1], 0);

    // 情况1：双方都可移动
    if (p1->isMovable() && p2->isMovable()) {
        // 各移动一半距离
        Vec3 correctionVectorHalf = correctionVector * (
            constraintTimes > 14 ? 0.5 : doubleMove[constraintTimes - 1]
        );
        p1->offsetPos(correctionVectorHalf);
        p2->offsetPos(-correctionVectorHalf);
    } 
    // 情况2：只有p1可移动
    else if (p1->isMovable() && !p2->isMovable()) {
        // p1 移动全部距离
        Vec3 correctionVectorHalf = correctionVector * (
            constraintTimes > 14 ? 1 : singleMove[constraintTimes - 1]
        );
        p1->offsetPos(correctionVectorHalf);
    } 
    // 情况3：只有p2可移动
    else if (!p1->isMovable() && p2->isMovable()) {
        // p2 移动全部距离
        Vec3 correctionVectorHalf = correctionVector * (
            constraintTimes > 14 ? 1 : singleMove[constraintTimes - 1]
        );
        p2->offsetPos(-correctionVectorHalf);
    }
    // 情况4：双方都不可移动 - 不做任何事
}