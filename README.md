![csf1](https://github.com/jianboqi/CSF/blob/master/CSFDemo/CSF1.png) ![csf2](https://github.com/jianboqi/CSF/blob/master/CSFDemo/CSF2.png)
# CSF
Airborne LiDAR filtering method based on Cloth Simulation.
This is the code for the article:

W. Zhang, J. Qi*, P. Wan, H. Wang, D. Xie, X. Wang, and G. Yan, “An Easy-to-Use Airborne LiDAR Data Filtering Method Based on Cloth Simulation,” Remote Sens., vol. 8, no. 6, p. 501, 2016.
(http://www.mdpi.com/2072-4292/8/6/501/htm)


**New feature has been implemented:**

Now, We has wrapped a Python interface for CSF with swig. It is simpler to use now. This new feature can make CSF easier to be embeded into a large project. For example, it can work with Laspy (https://github.com/laspy/laspy). What you do is just read a point cloud into a python 2D list, and pass it to CSF.
The following example shows how to use it with laspy.
```python
# coding: utf-8
import laspy
import CSF
import numpy as np

inFile = laspy.read(r"in.las") # read a las file
points = inFile.points
xyz = np.vstack((inFile.x, inFile.y, inFile.z)).transpose() # extract x, y, z and put into a list

csf = CSF.CSF()

# prameter settings
csf.params.bSloopSmooth = False
csf.params.cloth_resolution = 0.5
# more details about parameter: http://ramm.bnu.edu.cn/projects/CSF/download/

csf.setPointCloud(xyz)
ground = CSF.VecInt()  # a list to indicate the index of ground points after calculation
non_ground = CSF.VecInt() # a list to indicate the index of non-ground points after calculation
csf.do_filtering(ground, non_ground) # do actual filtering.

outFile = laspy.LasData(inFile.header)
outFile.points = points[np.array(ground)] # extract ground points, and save it to a las file.
out_file.write(r"out.las")
```

**Reading data from txt file:**

If the lidar data is stored in txt file (x y z for each line), it can also be imported directly.

```python
import CSF

csf = CSF.CSF()
csf.readPointsFromFile('samp52.txt')

csf.params.bSloopSmooth = False
csf.params.cloth_resolution = 0.5

ground = CSF.VecInt()  # a list to indicate the index of ground points after calculation
non_ground = CSF.VecInt() # a list to indicate the index of non-ground points after calculation
csf.do_filtering(ground, non_ground) # do actual filtering.
csf.savePoints(ground,"ground.txt")
```

### How to use CSF in Python
Thanks to [@rjanvier](https://github.com/rjanvier)'s contribution. Now we can install CSF from pip as:
```python
pip install cloth-simulation-filter
```

### How to use CSF in Matlab
see more details from file `demo_mex.m` under matlab folder.

### How to use CSF in R

Thanks to the nice work of @Jean-Romain, through the collaboration, the CSF has been made as a R package, the details can be found in the [RCSF repository](https://github.com/Jean-Romain/RCSF). This package can be used easily with the [lidR package](https://github.com/Jean-Romain/lidR):

```r
library(lidR)
las  <- readLAS("file.las")
las  <- lasground(las, csf())
```

### How to use CSF in C++
Now, CSF is built by CMake, it produces a static library, which can be used by other c++ programs.
#### linux
To build the library, run:
```bash
mkdir build #or other name
cd build
cmake ..
make
sudo make install
```
or if you want to build the library and the demo executable `csfdemo`

```bash
mkdir build #or other name
cd build
cmake -DBUILD_DEMO=ON ..
make
sudo make install

```

#### Windows
You can use CMake GUI to generate visual studio solution file.

### Binary Version
For binary release version, it can be downloaded at: http://ramm.bnu.edu.cn/projects/CSF/download/

Note: This code has been changed a lot since the publication of the corresponding paper. A lot of optimizations have been made. We are still working on it, and wish it could be better.

### Cloudcompare Pulgin
At last, if you are interested in Cloudcompare, there is a good news. our method has been implemented as a Cloudcompare plugin, you can refer to : https://github.com/cloudcompare/trunk

### Related project
A tool named `CSFTools` has been recently released, it is based on CSF, and provides dem/chm generation, normalization. Please refer to: https://github.com/jianboqi/CSFTools

### License
CSF is maintained and developed by Jianbo QI. It is now released under Apache 2.0.

---

# CSF 中文翻译

![csf1](https://github.com/jianboqi/CSF/blob/master/CSFDemo/CSF1.png) ![csf2](https://github.com/jianboqi/CSF/blob/master/CSFDemo/CSF2.png)
# CSF
基于布料模拟的机载LiDAR滤波方法。
本文代码对应的论文为：

W. Zhang, J. Qi*, P. Wan, H. Wang, D. Xie, X. Wang, and G. Yan, "An Easy-to-Use Airborne LiDAR Data Filtering Method Based on Cloth Simulation," Remote Sens., vol. 8, no. 6, p. 501, 2016.
(http://www.mdpi.com/2072-4292/8/6/501/htm)


**新功能已实现：**

现在，我们使用SWIG为CSF封装了Python接口，使用更加简单。这一新特性使CSF更容易嵌入到大型项目中。例如，它可以与Laspy（https://github.com/laspy/laspy）配合使用。您只需将点云读入Python的二维列表，然后传递给CSF即可。
以下示例展示了如何与laspy配合使用。
```python
# coding: utf-8
import laspy
import CSF
import numpy as np

inFile = laspy.read(r"in.las") # 读取las文件
points = inFile.points
xyz = np.vstack((inFile.x, inFile.y, inFile.z)).transpose() # 提取x、y、z并放入列表

csf = CSF.CSF()

# 参数设置
csf.params.bSloopSmooth = False
csf.params.cloth_resolution = 0.5
# 关于参数的更多细节：http://ramm.bnu.edu.cn/projects/CSF/download/

csf.setPointCloud(xyz)
ground = CSF.VecInt()  # 计算后用于指示地面点索引的列表
non_ground = CSF.VecInt() # 计算后用于指示非地面点索引的列表
csf.do_filtering(ground, non_ground) # 执行实际的滤波操作

outFile = laspy.LasData(inFile.header)
outFile.points = points[np.array(ground)] # 提取地面点，并保存为las文件
out_file.write(r"out.las")
```

**从txt文件读取数据：**

如果LiDAR数据存储在txt文件中（每行为x y z），也可以直接导入。

```python
import CSF

csf = CSF.CSF()
csf.readPointsFromFile('samp52.txt')

csf.params.bSloopSmooth = False
csf.params.cloth_resolution = 0.5

ground = CSF.VecInt()  # 计算后用于指示地面点索引的列表
non_ground = CSF.VecInt() # 计算后用于指示非地面点索引的列表
csf.do_filtering(ground, non_ground) # 执行实际的滤波操作
csf.savePoints(ground,"ground.txt")
```

### 如何在Python中使用CSF
感谢[@rjanvier](https://github.com/rjanvier)的贡献。现在可以通过pip安装CSF：
```python
pip install cloth-simulation-filter
```

### 如何在Matlab中使用CSF
详见matlab文件夹下的`demo_mex.m`文件。

### 如何在R中使用CSF

感谢@Jean-Romain的优秀工作，通过合作，CSF已被制作成R语言包，详情请见[RCSF仓库](https://github.com/Jean-Romain/RCSF)。该包可以与[lidR包](https://github.com/Jean-Romain/lidR)轻松配合使用：

```r
library(lidR)
las  <- readLAS("file.las")
las  <- lasground(las, csf())
```

### 如何在C++中使用CSF
现在，CSF使用CMake构建，它会生成一个静态库，可供其他C++程序使用。
#### Linux
要构建该库，请运行：
```bash
mkdir build #或其他名称
cd build
cmake ..
make
sudo make install
```
如果您想同时构建库和演示可执行文件`csfdemo`

```bash
mkdir build #或其他名称
cd build
cmake -DBUILD_DEMO=ON ..
make
sudo make install

```

#### Windows
您可以使用CMake GUI生成Visual Studio解决方案文件。

### 二进制版本
二进制发布版本可从以下地址下载：http://ramm.bnu.edu.cn/projects/CSF/download/

注意：自对应论文发表以来，本代码已经发生了很大变化，进行了大量优化。我们仍在持续改进，希望它变得更好。

### Cloudcompare插件
最后，如果您对Cloudcompare感兴趣，有一个好消息。我们的方法已作为Cloudcompare插件实现，请参阅：https://github.com/cloudcompare/trunk

### 相关项目
一个名为`CSFTools`的工具最近已发布，它基于CSF，提供了DEM/CHM生成、归一化等功能。请参阅：https://github.com/jianboqi/CSFTools

### 许可证
CSF由Jianbo QI维护和开发，目前在Apache 2.0许可下发布。

