# Pipeline Test

测试点云预处理与树干提取流水线（独立于 ROS/catkin）。

## 流水线步骤

1. **体素降采样** — `voxel_filter()` (leaf=0.1m, 保留 intensity)
2. **SOR 离群去除** — `sor_filter()` (K=50, stddev=1.0)
3. **PatchWork++ 地面去除** — `estimateGround()` (sensor_height=1.7m, max_range=80m)
4. **欧氏聚类** — `euclidean_clustering()` (tolerance=0.3m, min=30, max=50000)
5. **PCA 树干过滤** — 线性度>0.95, 垂直度>0.95, 高度>2.5m

## 构建

```bash
cd tests
mkdir build && cd build
cmake ..
make -j$(nproc)
```

## 运行

```bash
# 指定 PCD 文件路径
./pipeline_test /path/to/your/cloud.pcd

# 默认路径 test_data/sample.pcd
./pipeline_test
```

## 输出

所有结果写入 `output/` 目录：

| 文件 | 说明 |
|------|------|
| `01_voxel.pcd` | 体素降采样后的点云 (PointXYZI) |
| `02_sor.pcd` | SOR 过滤后的点云 (PointXYZI) |
| `03_ground.pcd` | PatchWork++ 地面点 (PointXYZ) |
| `03_nonground.pcd` | 非地面点，用于聚类 (PointXYZ) |
| `04_all_clusters.pcd` | 所有聚类点云 (PointXYZRGB, 每个聚类一个颜色) |
| `05_trunks.pcd` | PCA 判定为树干的聚类 (PointXYZRGB) |
| `05_discarded.pcd` | 被丢弃的聚类 (PointXYZRGB) |
| `06_trunk_centers.pcd` | 树干中心点 (PointXYZI, intensity=1) |
| `trunks_info.txt` | 每个树干的详细信息（坐标、PCA 特征值、线性度等） |

## CloudCompare 查看

```bash
# 一次性加载所有输出
cloudcompare output/*.pcd
```

建议：
- 用 **Colors → Scalar Field** 切换不同文件的颜色显示
- `04_all_clusters.pcd` 中每个聚类用不同颜色，便于区分独立物体
- `05_trunks.pcd` 中每个树干用不同颜色
- `06_trunk_centers.pcd` 可以用点大小放大以便观察

## RViz 查看

在 RViz 中添加多个 `PointCloud2` Display，分别加载不同 PCD（需要先通过 `pcl_ros` 发布为话题，或用 `pcl_viewer` 快速查看）。

快速查看：
```bash
pcl_viewer output/05_trunks.pcd output/06_trunk_centers.pcd
```

## 可调参数

在 `pipeline_test.cpp` 中直接修改：

```cpp
double voxel_size = 0.1;              // 体素大小
int sor_mean_k = 50;                  // SOR 邻域数
double sor_stddev = 1.0;              // SOR 标准差倍数
double cluster_tolerance = 0.3;       // 聚类容差
int cluster_min_size = 30;            // 最小聚类点数
double linearity_threshold = 0.95;    // PCA 线性度阈值
double verticality_threshold = 0.95;  // PCA 垂直度阈值
double min_height = 2.5;              // 最小树干高度
```
