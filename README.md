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
│   (PointCloud2, world系)            (nav_msgs::Odometry)          │
└──────────────────────────────┬──────────────────────────────────┘
                               │ ROS Topics
                               ↓
┌─────────────────────────────────────────────────────────────────┐
│  Backend: forest_loop_detector (独立 ROS 节点)                   │
│                                                                 │
│  主线程 (ros::spin): 订阅 world 系点云 + 里程计                  │
│    → 关键帧提取 (距离/角度/时间阈值)                              │
│                                                                 │
│  loop_thread: 时间窗口点云累积 → Voxel → SOR                    │
│    → PatchWork++ 地面滤除 → 欧式聚类 + PCA → 树干中心            │
│    → build_stdesc → TriDesc → 哈希表插入/搜索                    │
│    → 投票候选 → 几何验证 → PCL GICP 精配准                      │
│                                                                 │
│  pgo_thread: GTSAM 位姿图优化 → 发布修正位姿                     │
└─────────────────────────────────────────────────────────────────┘
```

### 核心思想

利用森林中树干空间排列的**几何不变性**（三角形边长不随坐标系旋转/平移变化），将树干聚类中心构成的三角形作为描述符，通过量化边长构建哈希键，实现高效的全局回环候选搜索，最终通过 SVD + ICP 精配准 + GTSAM 位姿图优化消除累积漂移。

### 设计原则

| 原则 | 说明 |
|------|------|
| **多线程分离** | FAST-LIO2 前端保持 50-100Hz 实时运行，回环检测在后端独立线程中运行 |
| **World 帧特征提取** | 订阅 FAST-LIO2 输出的 world 帧点云 (`/cloud_registered`)，所有描述符生成、聚类、法向量计算均在 world 坐标系下进行，几何不变性通过三角形边长保证 |
| **轻量化地图策略** | 回环优化后不重建 FAST-LIO2 内部的 ikd-Tree，仅在后端维护关键帧位姿和点云 |
| **纯 3D 管线** | 抛弃 TLS 的距离图像投影方案（不适用于 Livox Mid-360 非重复 Rosette 扫描），采用纯 3D 处理管线 |

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
│ 2. SOR 离群点去除         │  MeanK=50, StddevMulThresh=1.0
│                          │  过滤混合像素噪声(mixed pixels)和灌木碎点
└────────────┬────────────┘
             ▼
┌─────────────────────────┐
│ 3. 地面滤除 (PatchWork++)│  同心带模型 (CZM) + RNR + R-VPF + R-GPF + TGR
│                          │  分离地面点和非地面点，自适应阈值更新
└────────────┬────────────┘
             ▼
┌─────────────────────────┐
│ 4. 欧式聚类 (EC)          │  PCL EuclideanClusterExtraction
│    + PCA 几何过滤          │  容忍距离 0.3m, 最小点数 30
│                          │  PCA 筛选:
│                          │    linearity = (λ₁-λ₂)/λ₁ > 0.7
│                          │    verticality = |axis₁·Z| > 0.7 (倾斜<45°)
│                          │    height = max(z)-min(z) > 1.5m
│                          │  注: RNR 反射噪声去除已启用 (使用强度信息)
└────────────┬────────────┘
             ▼
┌─────────────────────────┐
│ 5. 提取树干中心          │  聚类质心 = 三角形顶点, PCA主轴 = 法向量
│                          │  树干 Z 调整至 PatchWork++ 估计的地面高度
└────────────┬────────────┘
             ▼
┌─────────────────────────┐
│ 6. build_stdesc          │  KNN(10) 找最近邻, 两两组合形成三角形
│                          │  过滤: 边长 2~50m, 排除近似等腰/等边
│                          │  帧内 voxel 去重
└────────────┬────────────┘
             ▼
┌─────────────────────────┐
│ 7. 哈希表搜索            │  17x17x17 (+/- 8 bins) 量化邻域搜索
│    + 帧级投票             │  边长距离过滤 → 投票选 top-K 候选
│                          │  (时间窗口过滤 >= 30s, 硬编码于节点代码)
└────────────┬────────────┘
             ▼
┌─────────────────────────┐
│ 8. 候选验证              │  里程计位姿计算初始变换
│                          │  顶点邻近性投票 → K=5 KNN 几何验证 (质心距离)
└────────────┬────────────┘
             ▼
┌─────────────────────────┐
│ 9. 局部子图 ICP          │  拼接前后 N 帧 → small_gicp 精配准
│                          │  Fitness score >= 0.3 确认回环
└────────────┬────────────┘
             ▼
┌─────────────────────────┐
│ 10. GTSAM PGO           │  里程计边 (EKF 协方差) + 回环边 (ICP fitness)
│                          │  Levenberg-Marquardt 优化 → 全局一致位姿
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
| OpenCV | 4.x | 已链接，当前未使用 |
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

编译成功后生成可执行文件：

```
devel/lib/forest_loop_detector/loop_detector_node
```

---

## 五、启动

### 5.1 前置条件

1. **FAST-LIO2** 已启动并正常运行，发布以下 topic：
   - `/cloud_registered` —  世界系点云 (`sensor_msgs/PointCloud2`)
   - `/Odometry` — 里程计位姿 (`nav_msgs/Odometry`)

2. **Livox Mid-360** 已连接并发送数据

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

# Terminal 3: RViz 可视化 (需自行配置)
rviz
```

---

## 六、订阅/发布 Topic

### 订阅 (Subscribers)

| Topic | 类型 | 说明 |
|-------|------|------|
| `/cloud_registered` | `sensor_msgs/PointCloud2` | FAST-LIO2 输出的 World 系点云 |
| `/Odometry` | `nav_msgs/Odometry` | FAST-LIO2 输出的里程计位姿 + 协方差 |

### 发布 (Publishers)

| Topic | 类型 | 说明 |
|-------|------|------|
| `/loop_markers` | `visualization_msgs/MarkerArray` | 回环边可视化 (红色线段) |
| `/corrected_path` | `nav_msgs/Path` | PGO 优化后的修正路径 |
| `/global_map` | `sensor_msgs/PointCloud2` | PGO 优化后的全局点云地图 |

---

## 七、参数配置

所有参数位于 `config/loop_para.yaml`，也可通过 ROS param server 覆盖。

### 关键帧提取

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `keyframe_min_distance` | 1.0 | 距上一关键帧最小平移距离 (m) |
| `keyframe_min_angle` | 10.0 | 距上一关键帧最小旋转角度 (deg) |
| `keyframe_min_time` | 5.0 | 最小时间间隔 (s) |

### 点云预处理 (Mid-360)

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `accumulation_window_sec` | 1.0 | 点云累积时间窗口 (s) |
| `voxel_size` | 0.1 | 体素降采样 grid 大小 (m) |
| `sor_mean_k` | 50 | SOR 邻居数 |
| `sor_stddev` | 1.0 | SOR 标准差倍数 |

### 地面滤除 (PatchWork++)

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `ground_method` | patchwork | 地面滤除方法 (patchwork 或 ransac) |
| `patchwork_sensitivity` | 0.5 | PatchWork++ 灵敏度 (0-1) |
| `sensor_height` | 1.7 | 传感器离地高度 (m) |
| `min_range` | 0.5 | 地面检测最小半径 (m) |
| `max_range` | 80.0 | 地面检测最大半径 (m) |

### 聚类 + PCA

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `cluster_tolerance` | 0.3 | 欧式聚类容忍距离 (m) |
| `cluster_min_size` | 30 | 聚类最小点数 |
| `cluster_max_size` | 50000 | 聚类最大点数 (代码硬编码默认 50) |
| `pca_linearity_threshold` | 0.7 | PCA 线性度阈值 |
| `pca_verticality_threshold` | 0.7 | PCA 垂直度阈值 (cos 45°) |
| `pca_min_height` | 1.5 | 最小树干高度 (m) |

### 回环搜索

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `rough_dis_threshold` | 0.1 | 边长相对容差 |
| `candidate_num` | 50 | 候选帧最大数量 |
| `min_frame_votes` | 2 | 帧级投票最低票数 (hainan 配置为 3) |

### 三角形描述符

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `descriptor_near_num` | 10 | KNN 邻居数 |
| `descriptor_min_len` | 2.0 | 最小边长 (m) |
| `descriptor_max_len` | 50.0 | 最大边长 (m) |
| `descriptor_len_diff` | 0.1 | 排除近似等腰/等边 (m) |

### ICP 精配准

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `submap_window_size` | 5 | 局部子图前后帧数 |
| `submap_voxel_size` | 0.1 | 子图体素降采样 (m) |
| `icp_corr_distance` | 0.5 | ICP 对应点距离阈值 (m) |
| `fitness_threshold` | 0.3 | 回环确认 fitness score |

### GTSAM PGO

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `pgo_optimize_interval` | 10.0 | 位姿图优化间隔 (s) |
| `pgo_loop_edge_weight` | 1.0 | 回环边权重系数 |

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

### 8.2 预期输出

回环检测节点在终端输出彩色日志：

```
[Keyframe] Saved KF #5 (total: 5)
[ProcessKF] KF #5 (cloud size: 12345)
[ProcessKF] Accumulated cloud: 45678 points
[GenTriDescs] KF#5: input=45678 → voxel=12345 → sor=11234 → clusters=23 → trunks=8 → tridesc=15 | Time: preprocess=..., PCA=..., build_stdesc=...
[LoopCandidate] KF #5 <-> KF #0 (score=0.620, dt=45.2s)
[LoopConfirmed] KF #5 <-> KF #0, fitness=0.73
[PGO] Optimization done with 6 keyframes and 1 loop constraints
```

### 8.3 RViz 验证

添加以下显示：

1. **PointCloud2** — 订阅 `/cloud_registered`，查看原始点云
2. **MarkerArray** — 订阅 `/loop_markers`，查看红色回环边
3. **Path** — 订阅 `/corrected_path`，查看 PGO 优化后的修正路径
4. **Odometry** — 订阅 `/Odometry`，查看 FAST-LIO2 原始路径（对比漂移消除效果）

### 8.4 性能指标参考

| 指标 | 预期值 | 说明 |
|------|--------|------|
| 关键帧频率 | ~0.2 Hz | 每 5-10s 一个关键帧 |
| 描述符生成 | < 200ms | GenTriDescs 耗时 |
| 哈希搜索 | < 50ms | candidate_frames_selector 耗时 |
| SVD 验证 | < 100ms | candidate_frames_verify 耗时 |
| ICP 精配准 | < 300ms | small_gicp + submap 耗时 |
| 回环检测总延迟 | < 1s | 从关键帧到回环确认 |

---

## 九、代码结构

```
forest_loop_detector/
├── CMakeLists.txt
├── package.xml
├── launch/
│   └── loop_detector.launch          # ROS 启动文件
├── config/
│   └── loop_para.yaml                # 参数配置
├── include/
│   ├── dst/
│   │   └── DST.h                     # 核心数据结构
│   │                                 #   TriDesc, TriDesc_LOC, Hash, ConfigSetting...
│   ├── utils/
│   │   ├── HashRegObj.h              # HashRegDescManager 声明
│   │   ├── Hlp.h                     # 辅助函数声明
│   │   └── patchwork/
│   │       └── patchworkpp.h         # PatchWork++ 地面滤除 (官方实现)
│   ├── gtsam_opti/
│   │   └── gtsamOpti.h               # GTSAM 优化声明
│   └── KeyframeManager.h             # 关键帧管理
└── src/
    ├── loop_detector_node.cpp        # ROS 节点入口 (main)
    ├── KeyframeManager.cpp            # 关键帧提取实现
    ├── utils/
    │   ├── HashRegObj.cpp            # 描述符生成 + 哈希匹配 + 验证
    │   │                             # 纯 3D 管线: Voxel→SOR→PatchWork++→EC→PCA→build_stdesc
    │   ├── Hlp.cpp                   # YAML 读取, 矩阵操作, 计时器
    │   └── patchworkpp.cpp           # PatchWork++ 地面滤除实现
    └── gtsam_opti/
        └── gtsamOpti.cpp             # GTSAM 位姿图优化
```

---

## 十、与 Forest_TLS_Reg 的关系

本系统复用并修改了 [Forest_TLS_Reg]() 项目的核心算法：

| 文件 | 用途 | 修改量 |
|------|------|--------|
| `dst/DST.h` | 数据结构定义 | 0 (原样拷贝，添加 include guard) |
| `utils/HashRegObj.cpp` | 描述符+匹配+验证 | **大改** — 重写 GenTriDescs 为纯 3D 管线, 移除 TLS 依赖 |
| `utils/Hlp.cpp` | 辅助函数 | 中 — 移除 TLS 特定依赖 (libLAS, CSF)，改用 yaml-cpp |
| `utils/FEC.cpp` | FEC 聚类 | 0 (原样拷贝，**已从项目中移除**，未使用) |
| `gtsam_opti/gtsamOpti.cpp` | GTSAM 优化 | 0 (原样拷贝) |
| `utils/patchwork/patchworkpp.h` | PatchWork++ 头文件 | **新增** — 来自 url-kaist/patchwork-plusplus |
| `utils/patchworkpp.cpp` | PatchWork++ 实现 | **新增** — 来自 url-kaist/patchwork-plusplus |
| ~~`utils/Point2img.cpp`~~ | ~~点云→距离图像~~ | **已移除** — Mid-360 不适合距离图像方案 |
| ~~`libSeg.so`~~ | ~~图像分割~~ | **已移除** — 依赖距离图像，随同移除 |

### 主要差异

| 方面 | Forest_TLS_Reg (TLS) | Forest-LIO-Loop (Mid-360) |
|------|----------------------|---------------------------|
| 传感器 | 机械式 TLS 扫描仪 | 固态 Livox Mid-360 |
| 扫描模式 | 360° 重复 | 非重复 Rosette |
| 特征提取 | 点→距离图像 → 图像分割 | 纯 3D 管线 (Voxel→SOR→PatchWork++→EC→PCA) |
| 地面滤除 | CSF 布料模拟 | PatchWork++ (CZM 同心带模型) |
| 回环候选 | 哈希匹配 + SVD 投票 | 哈希匹配 + 里程计位姿 + 顶点投票 |
| ICP 后端 | — | small_gicp (4 线程) |
| 坐标系 | TLS 固定站 | World 系 (camera_init) |
| 运行方式 | 离线批处理 | 在线实时 (ROS 节点) |

---



---

## 十一、已知限制

1. **稀疏树干场景**：在幼林或稀疏树干区域，PCA 过滤后可能无法生成足够的三角形描述符
2. **地形坡度**：PatchWork++ 对陡坡适应性有限，极端地形下建议使用 RANSAC 方案
3. **内存增长**：哈希表随关键帧数量持续增长，长期运行需要实现 LRU 淘汰或滑动窗口策略
4. **ikd-Tree 不一致**：回环优化后不重建 FAST-LIO2 内部的 ikd-Tree，前端地图仍含漂移，仅后端地图全局一致
5. **哈希描述符**：当前仅使用边长 (x,y,z) 三属性进行哈希匹配，角度属性 (a,b,c) 已定义但未启用 (见 `DST.h:129`)
6. **未使用代码**：`triangle_solver()`、`transform_point_cloud()`、`pcl_gicp_registration()` 等函数已实现但未在主流程中调用

---

## 十二、引用

本系统基于以下项目开发：

- **Forest_TLS_Reg** — Hash Table based TLS Multi-Scan Registration in Forest Environments
- **FAST-LIO2** — Fast LiDAR-Inertial Odometry
- **GTSAM** — Georgia Tech Smoothing and Mapping Library
- **small_gicp** — Efficient and parallelized GICP algorithms
- **PatchWork++** — Concentric Zone Model based Ground Segmentation (https://github.com/url-kaist/patchwork-plusplus)

---

## 十三、TODO

- [ ] 哈希表内存管理 (LRU 淘汰)
- [ ] Iterative Umeyama 精求解 + 合理性检查 (triangle_solver 已声明但未调用)
- [ ] RViz 可视化配置文件
- [ ] 离线 bag 回放性能评估脚本
- [ ] 不同森林密度场景的基准测试
- [ ] 清理未使用的 YAML 参数 (time_window_sec, icp_max_iter, SVD 系列参数等)
