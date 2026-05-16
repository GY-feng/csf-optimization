# ======================================================================================
# csf_test.py - Python 测试用例
#
# 这是一个简单的测试，验证 CSF 算法的基本功能：
#   1. 创建一个平坦的地面点云（所有点高度相同）
#   2. 运行 CSF 滤波
#   3. 断言所有点都被分类为地面点
# ===========================================================================

# basic CSF python tests

import CSF
import numpy as np


def test_csf_from_numpy():
    """Test CSF+numpy with a flat ground plane."""
    # 创建一个平坦的地面：X和Y在-100到100之间随机，Z固定为-0.1
    x = np.random.uniform(-100, 100, size=10_000)
    y = np.random.uniform(-100, 100, size=10_000)
    z = np.random.uniform(-0.1, -0.1, size=10_000)

    # 创建 CSF 对象
    csf = CSF.CSF()
    # 设置点云（numpy 数组，格式：N×3）
    csf.setPointCloud(np.c_[x, y, z])

    # 准备输出容器
    ground, non_ground = CSF.VecInt(), CSF.VecInt()

    # 执行滤波
    csf.do_filtering(ground, non_ground)
    
    # 断言：所有点都应该是地面点
    assert len(ground) > 0      # 地面点列表不为空
    assert len(non_ground) == 0  # 非地面点列表为空（因为地面是平坦的）
