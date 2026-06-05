# Forest-LIO-Loop

基于**三角形描述符 + 哈希表匹配**的 FAST-LIO2 森林场景回环检测与位姿图优化系统。

> FAST-LIO2 提供高精度 LiDAR-IMU 里程计，但缺少回环检测能力，长时间运行会产生累积漂移。本系统将 Forest_TLS_Reg 项目中的三角形描述符方法接入 FAST-LIO2，构建面向森林林冠场景的完整后端，消除累积漂移。

---

## 一、系统架构

```
┌─────────────────────────────────────────────────────────────────┐
│  Frontend: FAST-LIO2 (laserMapping 主线程)                       │
│  ┌─────────┐  ┌──────────┐  ┌───────────┐  ┌─────────────┐     │
│  │ LiDAR   │→│ IMU      │→│ EKF       │→│ ikd-Tree    │     │
│  │ + IMU   │  │ Deskew   │  │ 更新      │  │ 增量建图     │     │
│  └─────────┘  └──────────┘  └───────────┘  └─────────────┘     │
│         │                              │                        │
│         ↓                              ↓                        │
│   /cloud_registered                /Odometry                    │
│   (PointCloud2, world系)            (nav_msgs/Odometry)          │
└──────────────────────────────┬──────────────────────────────────┘
                               │ ROS Topics
                               ↓
┌─────────────────────────────────────────────────────────────────┐
│  Backend: forest_loop_detector (独立 ROS 节点)                   │
│                                                                 │
│  主线程 (ros::spin): 订阅 world 系点云 + 里程计                  │
│    → 关键帧提取 (距离/角度/时间阈值)                              │
│                                                                 │
│  loop_thread: 点云累积 → Voxel → SOR → PatchWork++ 地面滤除     │
│    → FEC 欧式聚类 + PCA → 树干中心 → build_stdesc → TriDesc     │
│    → 哈希表插入/搜索 → 投票候选 → GeoVerify → small_gicp 精配准  │
│                                                                 │
│  pgo_thread: GTSAM 位姿图优化 (batch Levenberg-Marquardt)        │
│    → 发布修正位姿                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### 核心思想

利用森林中树干空间排列的**几何不变性**（三角形边长不随坐标系旋转/平移变化），将树干聚类中心构成的三角形作为描述符，通过量化边长构建哈希键，实现高效的全局回环候选搜索，最终通过 small_gicp 精配准 + GTSAM 位姿图优化消除累积漂移。

### 设计原则

| 原则 | 说明 |
|------|------|
| **多线程分离** | FAST-LIO2 前端保持 50-100Hz 实时运行，回环检测和 PGO 在后端独立线程中运行 |
| **World 帧特征提取** | 订阅 FAST-LIO2 输出的 world 帧点云 (`/cloud_registered`)，所有描述符生成、聚类、法向量计算均在 world 坐标系下进行，几何不变性通过三角形边长保证 |
| **轻量化地图策略** | 回环优化后不重建 FAST-LIO2 内部的 ikd-Tree，仅在后端维护关键帧位姿和点云 |
| **纯 3D 管线** | 摒弃 TLS 的距离图像投影方案，采用纯 3D 处理管线，适配各种多线激光雷达 |

---

## 二、处理管线

```
累积 World 帧点云 (0.5~1.0s 窗口，世界系直接拼接)
    │
    ▼
┌─────────────────────────┐
│ 1. 体素滤波 (Voxel)      │  降采样 0.1m grid，去除冗余点，统一密度
└────────────┬────────────┘
             ▼
┌─────────────────────────┐
│ 2. SOR 离群点去除         │  MeanK=30, StddevMulThresh=1.0
│                          │  过滤混合像素噪声(mixed pixels)和灌木碎点
└────────────┬────────────┘
             ▼
┌─────────────────────────┐
│ 3. 地面滤除 (PatchWork++)│  同心带模型 (CZM) + RNR + R-VPF + R-GPF + TGR
│                          │  分离地面点和非地面点，自适应阈值更新
└────────────┬────────────┘
             ▼
┌─────────────────────────┐
│ 4. 快速欧式聚类 (FEC) +  │
│    + Union-Find         │  容忍距离 0.5m, 最小点数 30, 最大点数 50000
│    + PCA 几何过滤         │  PCA 筛选:
│                          │    linearity = (λ₁-λ₂)/λ₁ > 0.7
│                          │    verticality = |axis₁·Z| > 0.7 (倾斜<45°)
│                          │    height = max(z)-min(z) > 1.0m
│                          │  注: RNR 反射噪声去除已启用 (使用强度信息)
└────────────┬────────────┘
             ▼
┌─────────────────────────┐
│ 5. 提取树干中心          │  聚类质心 = 三角形顶点, PCA主轴 = 法向量
│    + 树干合并            │  树干 Z 调整至聚类 minZ
│                         │  合并空间邻近 + Z 范围重叠的过分割树干
└────────────┬────────────┘
             ▼
┌─────────────────────────┐
│ 6. build_stdesc          │  KNN(10) 找最近邻, 两两组合形成三角形
│                          │  过滤: 边长 1~20m, 排除近似等腰/等边
│                          │  帧内体素去重
└────────────┬────────────┘
             ▼
┌─────────────────────────┐
│ 7. 哈希表搜索            │  17×17×17 (±8 bins) 量化邻域搜索
│    + 帧级投票             │  边长距离过滤 → 投票选 top-K 候选
│                          │  时间窗口过滤 >= 10s
└────────────┬────────────┘
             ▼
┌─────────────────────────┐
│ 8. 候选验证              │  GeoVerify 评分预过滤 (阈值 0.15)
│                          │  顶点邻近性投票 → K=5 KNN 几何验证
└────────────┬────────────┘
             ▼
┌─────────────────────────┐
│ 9. 局部子图 ICP          │  拼接前后 N 帧 → small_gicp 精配准
│                          │  Fitness score >= 0.3 确认回环
└────────────┬────────────┘
             ▼
┌─────────────────────────┐
│ 10. GTSAM PGO           │  里程计边 (相邻关键帧相对位姿) + 回环边 (ICP fitness)
│                          │  Levenberg-Marquardt batch 优化 → 全局一致位姿
└─────────────────────────┘
```

---

## 三、环境依赖

| 依赖 | 版本 | 说明 |
|------|------|------|
| Ubuntu | 20.04 | 推荐 |
| ROS | Noetic | 必需 |
| PCL | 1.10+ | 点云处理 |
| Eigen3 | 3.3+ | 线性代数 |
| GTSAM | 4.2+ | 位姿图优化 |
| small_gicp | latest | ICP 精配准 |
| yaml-cpp | 0.6+ | YAML 配置解析 |
| OpenCV | 4.x | 已链接，当前未直接使用 |
| OpenMP | — | 多线程加速 |

### 依赖安装

```bash
# 基础依赖
sudo apt install -y \
    libpcl-dev \
    libeigen3-dev \
    libopencv-dev \
    libomp-dev \
    libyaml-cpp-dev

# GTSAM 4.2+
git clone https://github.com/borglab/gtsam.git
cd gtsam && mkdir build && cd build
cmake .. -DGTSAM_BUILD_EXAMPLES_ALWAYS_OFF=ON \
         -DGTSAM_BUILD_TESTS=OFF \
         -DGTSAM_WITH_TBB=OFF
make -j$(nproc)
sudo make install

# small_gicp
git clone https://github.com/koide3/small_gicp.git
cd small_gicp && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install
```

---

## 四、编译

```bash
# 进入工作空间
cd ~/$A_ROS_DIR$/src

# 克隆本仓库
git clone <your-repo-url> forest_loop_detector

# 返回工作空间根目录编译
cd ~/$A_ROS_DIR$
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```


---

## 五、启动

### 5.1 前置条件

1. **FAST-LIO2** 已启动并正常运行，发布以下 topic：
   - `/cloud_registered` — world 系点云 (`sensor_msgs/PointCloud2`)
   - `/Odometry` — 里程计位姿 (`nav_msgs/Odometry`)

2. LiDAR 传感器已连接并发送数据（支持 Ouster、Velodyne、Hesai 等机械旋转式多线激光雷达，代码中设为 `lidar_type: 2`）

### 5.2 启动回环检测节点

```bash
roslaunch forest_loop_detector loop_detector.launch
```

### 5.3 与 FAST-LIO2 联合启动

```bash
# Terminal 1: FAST-LIO2
roslaunch fast_lio mapping_xxx.launch

# Terminal 2: 回环检测
roslaunch forest_loop_detector loop_detector.launch

# Terminal 3: RViz 可视化 (由 launch 自动启动)
```


---

## 六、订阅/发布 Topic

### 订阅 (Subscribers)

| Topic | 类型 | 说明 |
|-------|------|------|
| `cloud_topic` (默认 `/cloud_registered`) | `sensor_msgs/PointCloud2` | FAST-LIO2 输出的 World 系点云，可通过 ROS param 覆盖 |
| `odom_topic` (默认 `/Odometry`) | `nav_msgs/Odometry` | FAST-LIO2 输出的里程计位姿 + 协方差，可通过 ROS param 覆盖 |

### 发布 (Publishers) — 主节点

| Topic | 类型 | 说明 |
|-------|------|------|
| `/loop_markers` | `visualization_msgs/MarkerArray` | 回环边可视化 (红色线段) |
| `/corrected_path` | `nav_msgs/Path` | PGO 优化后的修正路径 |
| `/global_map` | `sensor_msgs/PointCloud2` | PGO 优化后的全局点云地图 |

### 发布 (Publishers)

| Topic | 类型 | 说明 |
|-------|------|------|
| `ground` | `sensor_msgs/PointCloud2` | PatchWork++ 提取的地面点 |
| `nonground` | `sensor_msgs/PointCloud2` | 非地面点 |
| `trunks` | `sensor_msgs/PointCloud2` | PCA 筛选后的树干点云 (绿色) |
| `discarded` | `sensor_msgs/PointCloud2` | 被丢弃的非树干聚类 (灰色) |
| `trunk_centers` | `sensor_msgs/PointCloud2` | 树干中心点 |
| `tri_descriptors` | `visualization_msgs/MarkerArray` | 三角形描述符可视化 |

---

## 七、参数配置

入口文件 `config/loop_para.yaml`，通过 launch 文件的 `config_path` 参数指定。

### 关键帧提取

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `keyframe_min_distance` | 1.0 | 距上一关键帧最小平移距离 (m) |
| `keyframe_min_time` | 3.0 | 最小时间间隔 (s) |
<!-- | `keyframe_min_angle` | 10.0 | 距上一关键帧最小旋转角度 (deg) | -->

### 点云预处理

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `accumulation_window_sec` | 1.0 | 点云累积时间窗口 (s) |
| `voxel_size` | 0.1 | 体素降采样 grid 大小 (m) |

### 地面滤除 (PatchWork++)

PatchWork++ 为当前唯一地面滤除方法，无需配置 `ground_method`。
关键参数在代码中硬编码：传感器高度 1.7m、最小半径 0.5m、最大半径 80m，
RNR (反射噪声去除)、R-VPF (垂直平面拟合)、TGR (时序地面回退) 均已启用。

### 聚类 + PCA

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `cluster_tolerance` | 0.5 | 欧式聚类 (FEC) 容忍距离 (m) |
| `cluster_min_size` | 30 | 聚类最小点数 |
| `cluster_max_size` | 50000 | 聚类最大点数 |
| `pca_linearity_threshold` | 0.7 | PCA 线性度阈值 |
| `pca_verticality_threshold` | 0.7 | PCA 垂直度阈值 (cos 45°) |
| `pca_min_height` | 1.0 | 最小树干高度 (m) |
| `trunk_merge_dist` | 1.0 | 树干合并最大水平距离 (m) |
| `trunk_merge_z_overlap` | 0.1 | 树干合并最小高度重叠比例 |
| `trunk_merge_max_z_gap` | 3.0 | 树干合并最大 Z 间隙 (m) |

### 回环搜索

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `rough_dis_threshold` | 0.1 | 边长相对容差 |
| `candidate_num` | 10 | 候选帧最大数量 |
| `min_frame_votes` | 2 | 帧级投票最低票数 |

### 三角形描述符

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `descriptor_near_num` | 10 | KNN 邻居数 |
| `descriptor_min_len` | 1.0 | 最小边长 (m) |
| `descriptor_max_len` | 20.0 | 最大边长 (m) |
| `descriptor_len_diff` | 0.1 | 排除近似等腰/等边 (m) |
| `vertex_diff_threshold` | 0.5 | 顶点属性差异阈值 |
| `dist_candi_frames_verify` | 5 | 回环候选顶点最大距离 (m) |
| `dis_geo_verify` | 3.0 | GeoVerify 中心点最大距离 (m) |

### ICP 精配准

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `submap_window_size` | 5 | 局部子图前后帧数 |
| `submap_voxel_size` | 0.1 | 子图体素降采样 (m) |
| `icp_corr_distance` | 0.5 | ICP 对应点距离阈值 (m) |
| `icp_threshold` | 0.15 | GeoVerify 分数阈值，低于此值跳过 ICP |
| `fitness_threshold` | 0.3 | 回环确认 inlier ratio 阈值 |

### GTSAM PGO

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `pgo_optimize_interval` | 10.0 | 位姿图优化间隔 (s) |

> 注意：YAML 中的 `cluster_max_size` 映射到代码中的 `max_n`（ConfigSetting 默认 50000），
> `min_frame_votes` 映射到 `lGrp_Ele_Min`（ConfigSetting 默认 15，YAML 中覆盖为 2）。
> 修改 YAML 时需确保与 ConfigSetting 结构体一致。

---

## 八、测试

### 8.1 Bag 文件测试

```bash
# 1. 录制 bag (包含回环路径)
rosbag record /cloud_registered /Odometry -O forest_loop.bag

# 2. 启动 FAST-LIO2
roslaunch fast_lio mapping_xxx.launch

# 3. 启动回环检测
roslaunch forest_loop_detector loop_detector.launch

# 4. 播放 bag
rosbag play forest_loop.bag
```

### 8.2 树干提取测试

```bash
# 直接播放 bag 查看 trunk 提取效果
rosbag play <your_bag>.bag
```

该节点实时发布地面点、非地面点、树干点云和三角形描述符可视化，
适用于调参和验证 PCA 过滤效果。

### 8.3 预期输出

回环检测节点在终端输出彩色日志：

```
[Keyframe] Saved KF #5 (total: 5)
[ProcessKF] KF #5 (cloud size: 12345)
[ProcessKF] Accumulated cloud: 45678 points
[GenTriDescs] KF#5: input=45678 -> voxel=12345 -> sor=11234 -> clusters=23 -> trunks(before_merge)=23 -> trunks(after_merge)=8 -> tridesc=15 | Time: preprocess=..., PCA=..., merge=..., build_stdesc=...
[LoopCandidate] KF #5 <-> KF #0 (GeoVerify=0.620, dt=45.2s)
[LoopConfirmed] KF #5 <-> KF #0, fitness=0.730
[PGO] Optimization complete with 6 keyframes and 1 loop constraints
```

### 8.4 RViz 验证

launch 文件已配置自动启动 RViz，加载 `rviz_cfg/loop_fastlio.rviz`。建议显示：

1. **PointCloud2** — 订阅 `/cloud_registered`，查看原始点云
2. **MarkerArray** — 订阅 `/loop_markers`，查看红色回环边
3. **Path** — 订阅 `/corrected_path`，查看 PGO 优化后的修正路径
4. **Odometry** — 订阅 `/Odometry`，查看 FAST-LIO2 原始路径（对比漂移消除效果）

### 8.5 性能指标参考

| 指标 | 预期值 | 说明 |
|------|--------|------|
| 关键帧频率 | ~0.3 Hz | 每 3s 一个关键帧 |
| 描述符生成 | < 50ms | GenTriDescs 耗时 |
| 哈希搜索 | < 5ms | candidate_frames_selector 耗时 |
| GeoVerify | < 1ms | candidate_frames_verify 耗时 |
| ICP 精配准 | < 300ms | small_gicp + submap 耗时 |
| 回环检测总延迟 | < 1s | 从关键帧到回环确认 |

---

## 九、代码结构

```
forest_loop_detector/
├── CMakeLists.txt
├── package.xml
├── launch/
│   ├── loop_detector.launch          # 主节点启动文件 (含 RViz)
├── config/
│   └── loop_para.yaml                # 参数配置
├── rviz_cfg/
│   └── loop_fastlio.rviz             # RViz 可视化配置
├── include/
│   ├── dst/
│   │   └── DST.h                     # 核心数据结构
│   │                                 #   TriDesc, TriDesc_LOC, Hash, ConfigSetting...
│   ├── utils/
│   │   ├── FEC.h                     # Union-Find 欧式聚类
│   │   ├── HashRegObj.h              # HashRegDescManager 声明
│   │   ├── Hlp.h                     # 辅助函数声明
│   │   └── patchwork/
│   │       └── patchworkpp.h         # PatchWork++ 地面滤除 (官方实现)
│   ├── gtsam_opti/
│   │   └── gtsamOpti.h               # GTSAM 优化声明
│   └── KeyframeManager.h             # 关键帧管理
├── src/
│   ├── loop_detector_node.cpp        # ROS 节点入口 (main)
│   │                                 #   LoopDetectorNode 类: 双线程 (loop + pgo)
│   ├── KeyframeManager.cpp           # 关键帧提取实现
│   ├── utils/
│   │   ├── FEC.cpp                   # Union-Find 欧式聚类实现
│   │   ├── HashRegObj.cpp            # 描述符生成 + 哈希匹配 + 验证
│   │   │                             # 纯 3D 管线: Voxel→SOR→PatchWork++→FEC→PCA→merge→build_stdesc
│   │   ├── Hlp.cpp                   # YAML 读取, 矩阵操作, 计时器
│   │   └── patchworkpp.cpp           # PatchWork++ 地面滤除实现
│   └── gtsam_opti/
│       └── gtsamOpti.cpp             # GTSAM 位姿图优化 (batch Levenberg-Marquardt)
└── test/
    └── test_online.sh                 # 一键启动脚本 (4终端)
```

---

## 十、与 Forest_TLS_Reg 的关系

### 主要差异

| 方面 | Forest_TLS_Reg (TLS) | Forest-LIO-Loop |
|------|----------------------|---------------------------|
| 传感器 | 机械式 TLS 扫描仪 | 多线机械旋转式 LiDAR (Mid360/Ouster/Hesai) |
| 扫描模式 | 360° 重复 | 360° 机械旋转 |
| 特征提取 | 点→距离图像 → 图像分割 | 纯 3D 管线 (Voxel→SOR→PatchWork++→FEC→PCA) |
| 地面滤除 | CSF 布料模拟 | PatchWork++ (CZM 同心带模型) |
| 聚类 | FEC | FEC (基于 Union-Find 实现)+合并 |
| 回环候选 | 哈希匹配 + SVD 投票 | 哈希匹配 + 里程计位姿 + 顶点投票 + GeoVerify |
| ICP 后端 | — | small_gicp (4 线程) |
| PGO | — | GTSAM batch Levenberg-Marquardt |
| 坐标系 | TLS 固定站 | World 系 (camera_init) |
| 运行方式 | 离线批处理 | 在线实时 (ROS 节点) |

---

## 十一、已知限制

1. **稀疏树干场景**：在幼林或稀疏树干区域，PCA 过滤后可能无法生成足够的三角形描述符
2. **地形坡度**：PatchWork++ 对陡坡适应性有限，极端地形下建议考虑 RANSAC 方案 (当前未实现)
3. **内存增长**：哈希表随关键帧数量持续增长，长期运行需要实现 LRU 淘汰或滑动窗口策略
4. **ikd-Tree 不一致**：回环优化后不重建 FAST-LIO2 内部的 ikd-Tree，前端地图仍含漂移，仅后端地图全局一致
5. **哈希描述符**：当前仅使用边长 (x,y,z) 三属性进行哈希匹配，角度属性 (a,b,c) 已定义但未启用 (见 `DST.h`)
6. **全局地图未发布**：PGO 优化后 `/corrected_path` 已正常发布，但 `/global_map` 的拼接逻辑尚未实现
7. **Batch 优化**：使用 `LevenbergMarquardtOptimizer` batch 优化，未使用 ISAM2 增量优化，长时间运行需考虑改为增量模式
8. **未调用函数**：`triangle_solver()`、`transform_point_cloud()` 等函数已实现但不在主流程中调用

---

## 十二、引用

本系统基于以下项目开发：

- **Forest_TLS_Reg** — Hash Table based TLS Multi-Scan Registration in Forest Environments
- **FAST-LIO2** — Fast LiDAR-Inertial Odometry
- **GTSAM** — Georgia Tech Smoothing and Mapping Library
- **small_gicp** — Efficient and parallelized GICP algorithms
- **PatchWork++** — Concentric Zone Model based Ground Segmentation (https://github.com/url-kaist/patchwork-plusplus)

---
