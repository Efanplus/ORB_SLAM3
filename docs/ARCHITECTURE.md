# ORB-SLAM3 算法架构文档

> 版本：V1.0（2021-12-22）  
> 分析日期：2026-03-13

---

## 目录

1. [系统概览](#1-系统概览)
2. [线程架构与数据流](#2-线程架构与数据流)
3. [System — 入口与初始化](#3-system--入口与初始化)
4. [前端：Tracking（追踪线程）](#4-前端tracking追踪线程)
5. [中端：LocalMapping（局部建图线程）](#5-中端localmapping局部建图线程)
6. [后端：LoopClosing（回环检测与地图融合线程）](#6-后端loopclosing回环检测与地图融合线程)
7. [优化子系统：Optimizer](#7-优化子系统optimizer)
8. [特征子系统：ORB 提取与匹配](#8-特征子系统orb-提取与匹配)
9. [IMU 预积分子系统](#9-imu-预积分子系统)
10. [相机模型子系统](#10-相机模型子系统)
11. [核心数据结构](#11-核心数据结构)
12. [多地图系统（Atlas）](#12-多地图系统atlas)
13. [地图持久化（序列化）](#13-地图持久化序列化)
14. [传感器配置矩阵](#14-传感器配置矩阵)
15. [完整数据流图](#15-完整数据流图)

---

## 1. 系统概览

ORB-SLAM3 是第一个能在**视觉（Visual）、视觉-惯性（Visual-Inertial）和多地图（Multi-Map）**三种模式下统一运行的实时 SLAM 系统。支持六种传感器配置：

| 传感器类型 | 枚举值 |
|-----------|-------|
| 单目（Monocular）| 0 |
| 双目（Stereo）| 1 |
| RGB-D | 2 |
| IMU + 单目（IMU_MONOCULAR）| 3 |
| IMU + 双目（IMU_STEREO）| 4 |
| IMU + RGB-D（IMU_RGBD）| 5 |

**核心创新点：**

1. **多地图 SLAM**：通过 Atlas 管理多个子地图，在地图间丢失时创建新地图，后续通过回环检测或地图融合将子地图合并。
2. **完整视觉-惯性紧耦合**：IMU 预积分贯穿追踪、局部 BA、初始化的全过程。
3. **鱼眼相机支持**：KannalaBrandt8 模型，支持 > 180° 视场角。
4. **统一框架**：针孔与鱼眼、纯视觉与视觉惯性通过同一代码路径处理，仅通过相机模型多态分发。

---

## 2. 线程架构与数据流

```
┌─────────────────────────────────────────────────────────────────┐
│                        调用者线程                                │
│  System::TrackStereo / TrackRGBD / TrackMonocular               │
│                            │                                     │
│                     ┌──────▼──────┐                             │
│                     │   Tracking  │  ← GrabImage*()              │
│                     │  (前端)     │  ← GrabImuData()             │
│                     └──────┬──────┘                             │
│                            │ InsertKeyFrame(pKF)                 │
└────────────────────────────┼────────────────────────────────────┘
                             │
                             ▼
┌────────────────────────────────────────────────────────────────┐
│                    LocalMapping 线程（中端）                     │
│  ProcessNewKeyFrame → CreateNewMapPoints → LocalBA/LocalInertialBA │
│  → InitializeIMU → KeyFrameCulling                              │
│                            │ InsertKeyFrame(pKF)                 │
└────────────────────────────┼────────────────────────────────────┘
                             │
                             ▼
┌────────────────────────────────────────────────────────────────┐
│                   LoopClosing 线程（后端）                       │
│  NewDetectCommonRegions → CorrectLoop / MergeLocal             │
│  → RunGlobalBundleAdjustment                                    │
└────────────────────────────────────────────────────────────────┘

                             ▲ 反馈
                     UpdateFrameIMU()  ←── LocalMapping
                     InformNewBigChange() ← LoopClosing
```

**线程间同步机制：**

- `LocalMapping::InsertKeyFrame()` → 通过 `mlNewKeyFrames` 队列（`mMutexNewKFs` 保护）
- `LocalMapping::InterruptBA()` → 置 `mbAbortBA=true`，中断 g2o 优化
- `LoopClosing` 停止 LocalMapping：`RequestStop()` → `EmptyQueue()` → `isStopped()` 轮询 → 修正地图 → `Release()`
- Map 变更通知：`Atlas::InformNewBigChange()` → Tracking 通过 `GetMapChangeIndex()` 检测

---

## 3. System — 入口与初始化

**文件：** `src/System.cc`, `include/System.h`

### 3.1 初始化顺序

```
System::System()
  1. 解析 Settings YAML
  2. 加载 ORB 词袋（ORBVocabulary，DBoW2 格式）
  3. 创建 KeyFrameDatabase（词袋倒排索引）
  4. 创建 Atlas（或从 .osa 文件加载已有地图）
  5. 创建 FrameDrawer + MapDrawer
  6. 启动 Tracking（调用者线程）
  7. 启动 LocalMapping（std::thread）
  8. 启动 LoopClosing（std::thread）
  9. 交叉链接三个模块（SetLocalMapper / SetLoopClosing / SetTracker）
  10. 启动 Viewer（std::thread，可选）
```

### 3.2 公开 API

| 方法 | 描述 |
|------|------|
| `TrackMonocular(im, t, imu, fn)` | 单目追踪，返回 `Sophus::SE3f` |
| `TrackStereo(left, right, t, imu, fn)` | 双目追踪 |
| `TrackRGBD(im, depth, t, imu, fn)` | RGB-D 追踪 |
| `SaveTrajectoryTUM/EuRoC/KITTI(fn)` | 轨迹导出 |
| `SaveAtlas(type)` | 地图序列化保存 |
| `LoadAtlas(type)` | 地图反序列化加载 |
| `ActivateLocalizationMode()` | 切换为纯定位模式 |
| `Reset()` / `ResetActiveMap()` | 全局/局部复位 |

---

## 4. 前端：Tracking（追踪线程）

**文件：** `src/Tracking.cc`（4126 行），`include/Tracking.h`

### 4.1 状态机

```
SYSTEM_NOT_READY(-1)
NO_IMAGES_YET(0)   ──── 第一帧 ────► NOT_INITIALIZED(1)
                                          │
                               初始化成功  │
                                          ▼
                               ┌──────── OK(2) ◄────────────┐
                               │  追踪失败(>10 KF)           │
                               ▼              成功定位       │
                       RECENTLY_LOST(3) ──────────────────── ┘
                               │
                         超时/无法恢复
                               ▼
                            LOST(4)
                               │
              ┌────────────────┴─────────────────┐
              │ KF数量少                          │ KF数量多
              ▼                                   ▼
     ResetActiveMap()                    CreateMapInAtlas()
     → NO_IMAGES_YET                    → 新子地图
```

**关键时间阈值：**
- 视觉模式 RECENTLY_LOST 超时：3.0 秒
- IMU 模式 RECENTLY_LOST 超时：5.0 秒

### 4.2 每帧处理流水线

```
GrabImage*()
├── 灰度转换
├── 构造 Frame（ORB提取 + 双目匹配 + 特征网格分配）
└── Track()
    ├── IMU 预积分（PreintegrateIMU，仅惯性模式）
    ├── 检测 Map 变更索引 → 置 mbMapUpdated
    │
    ├── [NOT_INITIALIZED]
    │   ├── 双目/RGB-D：StereoInitialization()（单帧即可）
    │   └── 单目：MonocularInitialization()（需两帧 + 三角化）
    │
    ├── [OK]
    │   ├── CheckReplacedInLastFrame()
    │   ├── 主策略：TrackWithMotionModel()
    │   │   ├── 恒速运动模型预测（或 IMU 积分预测）
    │   │   └── PoseOptimization（纯视觉）
    │   └── 备选：TrackReferenceKeyFrame()
    │       ├── BoW 匹配
    │       └── PoseOptimization
    │
    ├── [RECENTLY_LOST]
    │   ├── IMU 模式：PredictStateIMU()
    │   └── 视觉模式：Relocalization()
    │       ├── BoW 候选帧查询
    │       ├── MLPnPsolver（RANSAC）
    │       └── 需 >= 50 内点
    │
    ├── TrackLocalMap()（全模式精化）
    │   ├── UpdateLocalKeyFrames()（共视投票 + 父子节点）
    │   ├── UpdateLocalPoints()
    │   ├── SearchLocalPoints()（局部地图点投影匹配）
    │   └── PoseOptimization 或 PoseInertialOptimization
    │
    ├── 更新运动模型 mVelocity = Tcur * Tlast⁻¹
    ├── NeedNewKeyFrame()？
    │   └── → CreateNewKeyFrame()
    │       ├── 创建 KF，链入 IMU 时序链
    │       └── InsertKeyFrame → LocalMapping 队列
    └── 返回 mCurrentFrame.GetPose()
```

### 4.3 关键帧判断条件（NeedNewKeyFrame）

| 条件 | 说明 |
|------|------|
| c1a | 距上次 KF 帧数 > maxFrames（= fps） |
| c1b | 帧数 > minFrames 且 LocalMapping 空闲 |
| c1c | 追踪弱（双目/RGB-D）：内点 < 25% 参考 KF 或需要近点 |
| c2 | 内点 < thRefRatio × 参考KF匹配数（单目0.9，双目0.75） |
| c3 | IMU：距上次 KF >= 0.5 秒 |
| c4 | IMU_MONOCULAR：内点 15–75 或 RECENTLY_LOST |

### 4.4 IMU 追踪关键方法

- **`PreintegrateIMU()`**：从队列提取 IMU 测量，维护两个预积分对象（从上一 KF 积分、从上一帧积分），使用梯形积分。
- **`PredictStateIMU()`**：用离散 IMU 运动方程预测当前帧位姿（R、t、V）。
- **`UpdateFrameIMU(s, b, pKF)`**：LocalMapping 完成 IMU 初始化后，更新 Tracking 中的尺度与偏置。

---

## 5. 中端：LocalMapping（局部建图线程）

**文件：** `src/LocalMapping.cc`（1522 行），`include/LocalMapping.h`

### 5.1 主循环（Run）

```
while(true):
  SetAcceptKeyFrames(false)
  if CheckNewKeyFrames() && !mbBadImu:
    ProcessNewKeyFrame()          // BoW计算、共视图更新、MP关联
    MapPointCulling()             // 剔除劣质新增地图点
    CreateNewMapPoints()          // 邻近KF对之间三角化
    if !CheckNewKeyFrames():
      SearchInNeighbors()         // 双向融合重复地图点
    if !CheckNewKeyFrames() && !stopRequested():
      LocalInertialBA() / LocalBundleAdjustment()  // 局部BA
      if 需要IMU初始化:
        InitializeIMU()
      KeyFrameCulling()           // 剔除冗余关键帧
      IMU精化（VIBA1 / VIBA2 / ScaleRefinement）
    LoopCloser.InsertKeyFrame(mpCurrentKeyFrame)
  SetAcceptKeyFrames(true)
  sleep(3ms)
```

### 5.2 新地图点创建（CreateNewMapPoints）

算法步骤：
1. 获取最佳共视邻居（双目取 10 个，单目取 30 个）
2. 检查基线合理性（避免退化）
3. 利用极线约束进行 ORB 匹配（`SearchForTriangulation`）
4. SVD 三角化得到 3D 候选点
5. 多重验证：正深度、重投影误差（χ²）、尺度一致性
6. 通过验证则创建 MapPoint，加入 `mlpRecentAddedMapPoints`

### 5.3 地图点剔除（MapPointCulling）

| 规则 | 条件 | 操作 |
|------|------|------|
| 已标坏 | `isBad()` | 移出列表 |
| 找回率低 | `FoundRatio < 0.25` | `SetBadFlag()` |
| 观测不足 | 创建 >= 2 KF 后，观测 <= 2（单目）或 3（双目） | `SetBadFlag()` |
| 通过试用期 | 创建 >= 3 KF | 移出列表（永久保留） |

### 5.4 关键帧剔除（KeyFrameCulling）

| 模式 | 冗余阈值 |
|------|---------|
| 视觉 | 90% 的地图点被 >= 3 个 KF 在同等或更细尺度观测 |
| IMU 双目/RGB-D | 50% |

IMU 模式附加约束：
- 不剔除 KF 数 <= 21 时的任何帧
- 不剔除最近 2 帧
- 时间间隔约束（初始化后 3 秒，初始化前 0.5 秒）
- 剔除时合并 IMU 预积分：`mNextKF->mpImuPreintegrated->MergePrevious(pKF->mpImuPreintegrated)`

### 5.5 IMU 初始化流水线

```
阶段 0（InitializeIMU，mTinit < 5s）：
  重力方向估计（从预积分ΔV累积）→ InertialOptimization（200次迭代）
  → 尺度检验（< 0.1 则中止）→ ApplyScaledRotation（全图）
  → FullInertialBA（100次迭代）→ SetImuInitialized()

阶段 1（VIBA1，mTinit > 5s）：
  InitializeIMU(priorG=1.0, priorA=1e5)  // 收紧偏置估计

阶段 2（VIBA2，mTinit > 15s）：
  InitializeIMU(priorG=0.0, priorA=0.0)  // 零先验，全数据驱动

阶段 3（ScaleRefinement，单目，mTinit ∈ [25,75]s 每10s）：
  InertialOptimization（仅精化尺度+重力）
```

**运动退化检测：** 若 VIBA2 之前累积运动 < 0.02m 且时间 < 10s，置 `mbBadImu=true`，触发地图重置。

---

## 6. 后端：LoopClosing（回环检测与地图融合线程）

**文件：** `src/LoopClosing.cc`（2539 行），`include/LoopClosing.h`

### 6.1 场所识别流水线（NewDetectCommonRegions）

```
NewDetectCommonRegions(KF):
├── 阶段 1：候选帧时序跟踪（mnLoopNumCoincidences / mnMergeNumCoincidences）
│   └── DetectAndReffineSim3FromLastKF()
│       ├── 基于上一次 Sim3 进行投影匹配
│       ├── 优化 Sim3（OptimizeSim3）
│       ├── 成功：coincidences++
│       └── 连续失败2次：重置候选
│
└── 阶段 2：新 BoW 候选查询（KFDatabase::DetectNBestCandidates）
    └── DetectCommonRegionsFromBoW()
        ├── ORBmatcher::SearchByBoW（需 >= 20 匹配）
        ├── Sim3Solver RANSAC（0.99置信度，15内点，300次迭代）
        ├── 粗投影验证（需 >= 50 匹配）
        ├── OptimizeSim3（10次迭代，需 >= 20 匹配）
        ├── 精投影验证（需 >= 80 匹配）
        └── 共视KF空间验证（需 >= 3 个邻居验证通过）
            → 达到 3 次一致 → 确认检测
```

**同地图 → 回环**；**不同地图 → 地图融合**。

### 6.2 回环修正（CorrectLoop）

```
1. 停止 LocalMapping，中止全局 BA
2. 用 mg2oLoopScw 传播 Sim3 修正到共视邻居 KF
3. 逆映射修正地图点位置
4. MapPoint 融合（SearchAndFuse）
5. 重建共视图，收集新增回环连接边
6. 本质图优化：
   - 视觉模式：OptimizeEssentialGraph（7DoF Sim3）
   - IMU 模式：OptimizeEssentialGraph4DoF（仅偏航+平移）
7. 添加双向回环边
8. 启动全局 BA 线程（非 IMU 或 KF < 200 时）
9. 释放 LocalMapping
```

**IMU 约束：** 旋转修正仅限偏航（滚转/俯仰可由重力观测，不可修正）。

### 6.3 地图融合

#### MergeLocal（视觉/视觉惯性融合）
将当前地图的局部窗口（~25 KF）迁移到融合目标地图：
1. 计算局部窗口内 KF 的修正位姿
2. 切换活跃地图（Atlas::ChangeMap），标记旧地图为坏
3. 重建生成树
4. 融合重复地图点（SearchAndFuse）
5. 焊接 BA（LocalBA 或 MergeInertialBA）
6. 迁移剩余 KF/MP
7. 本质图优化（非单目模式）

#### MergeLocal2（IMU 专用融合）
使用尺度对齐（`ApplyScaledRotation`），将融合地图的元素迁入当前地图（方向与 MergeLocal 相反），支持 IMU 重初始化。

### 6.4 全局 BA（RunGlobalBundleAdjustment）

独立线程，在回环修正后启动：
- 视觉：`GlobalBundleAdjustment`（10 次迭代）
- IMU：`FullInertialBA`（7 次迭代，优化位姿/速度/偏置）
- 通过生成树 BFS 传播修正到非直接优化的 KF
- 修正所有地图点位置

---

## 7. 优化子系统：Optimizer

**文件：** `src/Optimizer.cc`（5590 行），`include/Optimizer.h`

### 7.1 优化问题目录

| 函数 | 目的 | 求解器 | 顶点 | 迭代 |
|------|------|-------|------|-----|
| `PoseOptimization` | 单帧运动优化 | LM, 6×3 | VertexSE3Expmap | 4×10 |
| `LocalBundleAdjustment` | 局部视觉 BA | LM, 6×3 | SE3+Point | 10 |
| `LocalInertialBA` | 局部视觉惯性 BA | LM, Variable | Pose+V+Bias+Point | 10/4 |
| `FullInertialBA` | 全局视觉惯性 BA | LM, Variable | 全部 | 7/100 |
| `GlobalBundleAdjustment` | 全局视觉 BA | LM, 6×3 | SE3+Point | 10 |
| `PoseInertialOptimizationLastKeyFrame` | 单帧视觉惯性优化（锚KF） | GN, Variable | Pose+V+Bias | 4×10 |
| `PoseInertialOptimizationLastFrame` | 单帧视觉惯性优化（锚帧） | GN, Variable | Pose+V+Bias | 4×10 |
| `InertialOptimization` | IMU 初始化参数估计 | LM, Variable | GDir+Scale+Bias+V | 200 |
| `OptimizeEssentialGraph` | 回环后位姿图优化（7DoF） | LM, 7×3 | VertexSim3Expmap | — |
| `OptimizeEssentialGraph4DoF` | 回环后位姿图（4DoF，IMU） | LM, Variable | VertexPose4DoF | — |
| `OptimizeSim3` | 两 KF 间 Sim3 估计 | LM, Variable | Sim3+Points | 5+5 |
| `MergeInertialBA` | 地图融合焊接惯性 BA | LM, Variable | 双边窗口 | — |

### 7.2 自定义 g2o 顶点

| 顶点 | DoF | 说明 |
|------|-----|------|
| `VertexPose` | 6 | IMU 体坐标系位姿（Rwb, twb），派生相机位姿 |
| `VertexVelocity` | 3 | 世界系线速度 |
| `VertexGyroBias` | 3 | 陀螺仪零偏 |
| `VertexAccBias` | 3 | 加速度计零偏 |
| `VertexGDir` | 2 | 重力方向（2 DoF，约束 z 轴） |
| `VertexScale` | 1 | 地图尺度（对数参数化：`s *= exp(delta)`） |
| `VertexPose4DoF` | 4 | 偏航+平移（IMU 位姿图） |
| `VertexSim3Expmap` | 7 | 相似变换（可固定尺度） |

### 7.3 自定义 g2o 边

| 边 | 误差维度 | 顶点数 | 说明 |
|----|---------|-------|------|
| `EdgeInertial` | 9 | 6 | IMU 预积分约束（旋转3+速度3+位置3） |
| `EdgeInertialGS` | 9 | 8 | 含重力方向+尺度的初始化约束 |
| `EdgeGyroRW` | 3 | 2 | 陀螺仪零偏随机游走 |
| `EdgeAccRW` | 3 | 2 | 加速度计零偏随机游走 |
| `EdgePriorPoseImu` | 15 | 4 | 边际化先验（Schur 补） |
| `EdgeMono` / `EdgeMonoOnlyPose` | 2 | 2/1 | 单目重投影误差（IMU参数化） |
| `EdgeStereo` / `EdgeStereoOnlyPose` | 3 | 2/1 | 双目重投影误差 |
| `EdgeSim3ProjectXYZ` | 2 | 2 | Sim3 前向投影 |
| `EdgeInverseSim3ProjectXYZ` | 2 | 2 | Sim3 反向投影 |
| `Edge4DoF` | 6 | 2 | 4DoF 相对位姿约束 |

### 7.4 IMU 预积分残差

9D 残差向量（旋转3 + 速度3 + 位置3）：

```
er = Log( dR(b)ᵀ · R₁ᵀ · R₂ )
ev = R₁ᵀ · (v₂ - v₁ - g·Δt) - dV(b)
ep = R₁ᵀ · (p₂ - p₁ - v₁·Δt - ½g·Δt²) - dP(b)
```

信息矩阵 = 预积分协方差 C[0:9,0:9] 的逆。

### 7.5 鲁棒核

视觉边统一使用 **Huber 核**：
- 单目：`delta = sqrt(5.991)` ≈ 2.45（χ²，2 DoF，95%）
- 双目：`delta = sqrt(7.815)` ≈ 2.80（χ²，3 DoF，95%）

策略：前 2 轮优化使用鲁棒核，第 3 轮移除（切换到纯最小二乘精化内点）。

---

## 8. 特征子系统：ORB 提取与匹配

### 8.1 ORBextractor — 提取流水线

**文件：** `src/ORBextractor.cc`，`include/ORBextractor.h`

```
ComputePyramid()
  → 构建 nlevels=8 层图像金字塔（缩放因子 1.2）

ComputeKeyPointsOctTree()（每层）
  → 将图像划分为 35×35 像素格
  → 每格 FAST 角点检测（阈值 20 → 降级 7）

DistributeOctTree()
  → 四叉树均匀分布：叶节点仅保留响应值最高的角点
  → 确保空间均匀覆盖

IC_Angle()
  → 31×31 圆形窗口强度矩计算主方向

computeOrbDescriptor()
  → Gaussian 模糊（7×7，σ=2）
  → 512 点对预训练模式（256 位二进制描述子）
  → 按主方向旋转点对
```

**关键参数：** 默认 1000 特征/帧（单目初始化阶段 5000），8 层金字塔，尺度因子 1.2。

### 8.2 ORBmatcher — 匹配策略

**文件：** `src/ORBmatcher.cc`（2076 行），`include/ORBmatcher.h`

| 方法 | 使用场景 | 策略 |
|------|---------|------|
| `SearchByProjection` | 追踪、回环、重定位 | 投影地图点，半径内 Hamming 最近邻 |
| `SearchByBoW` | 重定位、回环检测 | 仅匹配共享词袋节点的特征，大幅剪枝 |
| `SearchForInitialization` | 单目初始化 | 窗口（10px）+ 交叉验证 |
| `SearchForTriangulation` | 新地图点创建 | BoW 约束 + 极线约束 |
| `Fuse` | 地图点融合 | 投影 + 替换/合并重复点 |
| `SearchBySim3` | 回环验证 | Sim3 变换后投影双向匹配 |

**Hamming 距离阈值：** TH_HIGH=100，TH_LOW=50  
**比率测试：** `nnratio = 0.6`（最近邻距离 < 0.6 × 次近邻）  
**方向一致性：** 30 bin 旋转直方图，仅保留最大 3 个 bin 的匹配  
**描述子距离：** 位并行 Hamming（256 bit，8 个 32 位字 popcount）

---

## 9. IMU 预积分子系统

**文件：** `src/ImuTypes.cc`，`include/ImuTypes.h`

### 9.1 核心数据类

| 类 | 作用 |
|----|------|
| `IMU::Point` | 单次 IMU 采样（a, w, t） |
| `IMU::Bias` | 6 维零偏（bax, bay, baz, bwx, bwy, bwz） |
| `IMU::Calib` | 外参 T_bc + 噪声参数（ng, na, ngw, naw） |
| `IMU::Preintegrated` | 预积分累积量 + 协方差 + Jacobians |

### 9.2 离散积分（IntegrateNewMeasurement）

```
acc_corrected = a - b_a
w_corrected   = w - b_g

dP += dV·dt + 0.5·dR·acc_corrected·dt²
dV += dR·acc_corrected·dt
dR  = Normalize(dR · Exp(w_corrected·dt))   // Rodrigues 公式
```

**协方差传播：** `C = A·C·Aᵀ + B·Nga·Bᵀ`（15×15 矩阵，误差状态线性化）

### 9.3 零偏修正 Jacobians（一阶近似）

```
dR(b) ≈ dR(b₀) · Exp(JRg · δbg)
dV(b) ≈ dV(b₀) + JVg·δbg + JVa·δba
dP(b) ≈ dP(b₀) + JPg·δbg + JPa·δba
```

允许偏置更新后无需重新积分。

### 9.4 流形运算

- **SO3 对数/指数映射：** `Exp(v) = I + [v]× sin(‖v‖)/‖v‖ + [v]×² (1-cos(‖v‖))/‖v‖²`
- **右 Jacobian：** `Jr(v) = I - [v]×(1-cos‖v‖)/‖v‖² + [v]×²(‖v‖-sin‖v‖)/‖v‖³`
- **旋转归一化：** SVD 强制 SO(3) 约束

---

## 10. 相机模型子系统

**文件：** `include/CameraModels/GeometricCamera.h`，`src/CameraModels/`

### 10.1 多态接口（GeometricCamera）

```cpp
virtual Eigen::Vector2d project(Eigen::Vector3d& v3D) = 0;
virtual Eigen::Vector3d unproject(cv::Point2f& p2D) = 0;
virtual Eigen::Matrix<double,2,3> projectJac(Eigen::Vector3d& v3D) = 0;
virtual float uncertainty2(Eigen::Matrix<double,2,1>& p2D) = 0;
virtual bool ReconstructWithTwoViews(...) = 0;
virtual bool epipolarConstrain(...) = 0;
```

类型常量：`CAM_PINHOLE = 0`，`CAM_FISHEYE = 1`

### 10.2 Pinhole 模型（4 参数：fx, fy, cx, cy）

**投影：**
```
u = fx·X/Z + cx
v = fy·Y/Z + cy
```

**反投影：** `ray = ((u-cx)/fx, (v-cy)/fy, 1)`

**Jacobian（2×3）：**
```
[fx/Z,   0,  -fx·X/Z²]
[  0,  fy/Z, -fy·Y/Z²]
```

### 10.3 KannalaBrandt8 鱼眼模型（8 参数：fx, fy, cx, cy, k1–k4）

**投影：**
```
θ = atan2(√(x²+y²), z)           // 入射角
ψ = atan2(y, x)                   // 方位角
r(θ) = θ + k₁θ³ + k₂θ⁵ + k₃θ⁷ + k₄θ⁹   // 多项式径向函数
u = fx·r·cos(ψ) + cx
v = fy·r·sin(ψ) + cy
```

**反投影：** Newton 法迭代（10次，精度 ≈ 1e-6）求解 r(θ) = θd 的逆

**Jacobian：** 通过链式法则解析推导（θ, ψ, f(θ), f'(θ)）

---

## 11. 核心数据结构

### 11.1 Frame（单帧表示）

**文件：** `src/Frame.cc`，`include/Frame.h`

| 字段 | 类型 | 说明 |
|------|------|------|
| `mvKeys` / `mvKeysUn` | `vector<cv::KeyPoint>` | 原始/去畸变特征点 |
| `mDescriptors` | `cv::Mat` (N×32) | ORB 描述子 |
| `mvpMapPoints` | `vector<MapPoint*>` | 关联地图点 |
| `mvbOutlier` | `vector<bool>` | 优化后外点标记 |
| `mvDepth` / `mvuRight` | `vector<float>` | 深度/右图x坐标（-1=未知） |
| `mTcw` | `Sophus::SE3<float>` | 相机位姿（camera←world） |
| `mVw` | `Eigen::Vector3f` | 速度（世界系） |
| `mpImuPreintegrated` | `IMU::Preintegrated*` | 从上一 KF 的预积分 |
| `mpImuPreintegratedFrame` | `IMU::Preintegrated*` | 从上一帧的预积分 |
| `mGrid` | `64×48` 格 | 特征点空间索引 |
| `mpReferenceKF` | `KeyFrame*` | 参考关键帧 |

**构造流水线（以双目为例）：**
```
1. 双线程并行 ORB 提取（左/右）
2. UndistortKeyPoints（仅单目/RGB-D）
3. ComputeStereoMatches / ComputeStereoFromRGBD
4. AssignFeaturesToGrid（64×48 格）
5. ComputeImageBounds（首帧初始化）
```

### 11.2 KeyFrame（关键帧图节点）

**文件：** `src/KeyFrame.cc`，`include/KeyFrame.h`

**图结构：**
```
共视图（Covisibility Graph）：
  mConnectedKeyFrameWeights: map<KF*, int>   // 共享地图点数
  mvpOrderedConnectedKeyFrames               // 按权重降序排列

生成树（Spanning Tree）：
  mpParent / mspChildrens                    // 树形连接

回环/融合边：
  mspLoopEdges / mspMergeEdges               // 强化本质图

IMU 时序链：
  mPrevKF <──> this <──> mNextKF
  mpImuPreintegrated                         // mPrevKF → this 的预积分
```

**关键操作：**
- `ComputeBoW()`：调用 DBoW2 在第 4 层词袋转换
- `UpdateConnections()`：遍历观测地图点统计共视权重，权重 >= 15 则建立边；首次连接时顶权 KF 成为生成树父节点
- `SetBadFlag()`：从所有图中解链，子节点重新父化

### 11.3 MapPoint（三维地标）

**文件：** `src/MapPoint.cc`，`include/MapPoint.h`

| 字段 | 说明 |
|------|------|
| `mWorldPos` | 世界系 3D 坐标（Eigen::Vector3f） |
| `mObservations` | `map<KF*, tuple<int,int>>`（左/右特征索引） |
| `mDescriptor` | 最具代表性的 ORB 描述子（最小中值 Hamming 距离） |
| `mNormalVector` | 平均观测方向 |
| `mfMinDistance/mfMaxDistance` | 尺度不变距离范围 |
| `mnVisible/mnFound` | 可见/被找到次数（用于剔除） |
| `mpReplaced` | 替换指针（融合时） |

**核心操作：**
- `ComputeDistinctiveDescriptors()`：选择与所有观测描述子中值距离最小者
- `UpdateNormalAndDepth()`：均值化观测方向，计算尺度不变深度范围
- `Replace(MP*)`：将所有观测迁移到替换点

### 11.4 KeyFrameDatabase（倒排索引）

**文件：** `src/KeyFrameDatabase.cc`，`include/KeyFrameDatabase.h`

```
mvInvertedFile[word_id] → list<KeyFrame*>
```

**候选帧检测流程：**
1. 倒排索引查找共享词汇的 KF（排除共视 KF）
2. 最大共享词数阈值过滤（保留 >= 80%）
3. DBoW2 相似度分组累积（top-10 共视邻居的最优）
4. 按地图分类（同地图→回环候选，异地图→融合候选）
5. 返回 top-N（默认 N=3）

---

## 12. 多地图系统（Atlas）

**文件：** `src/Atlas.cc`，`include/Atlas.h`

```
Atlas
├── mpCurrentMap ──► Map (active)
│                    ├── mspKeyFrames
│                    ├── mspMapPoints
│                    ├── mbIsInertial
│                    └── mbImuInitialized
├── mspMaps ──► {Map₁, Map₂, ...}  (all live maps)
├── mspBadMaps ──► {MapX, ...}     (absorbed maps)
└── mvpCameras ──► {Pinhole/KannalaBrandt8 instances}
```

**多地图生命周期：**
```
正常追踪 → 追踪丢失（KF充足） → CreateMapInAtlas() → 新子地图
         → 回环/融合检测     → MergeLocal() → 合并到目标地图
         → SetMapBad() → 原地图标为坏
         → RemoveBadMaps() → 清理
```

---

## 13. 地图持久化（序列化）

**技术：** Boost.Serialization（支持 `text_archive` 和 `binary_archive`）

**流程：**
```
SaveAtlas():
  Atlas::PreSave()
    ├── Map::PreSave()      → KF/MP 指针转 ID 备份向量
    ├── KF::PreSave()       → 图连接指针转 ID
    ├── MP::PreSave()       → Observations 指针转 ID
    └── KFDatabase::PreSave() → 倒排列表指针转 ID
  → Boost 序列化到 .osa 文件

LoadAtlas():
  → Boost 反序列化
  Atlas::PostLoad()
    ├── Map::PostLoad()     → ID 转指针恢复
    ├── KF::PostLoad()
    ├── MP::PostLoad()
    └── KFDatabase::PostLoad()
  → 重新链接词袋和词袋数据库
```

**关键设计：** `set` → `vector` 转换规避 Boost 1.58 的 `std::set` 序列化 bug；静态 ID 计数器一并序列化保证 ID 唯一性。

---

## 14. 传感器配置矩阵

| 能力 | MONO | STEREO | RGB-D | IMU_MONO | IMU_STEREO | IMU_RGBD |
|------|------|--------|-------|----------|------------|----------|
| 单帧初始化 | ✗ | ✓ | ✓ | ✗ | ✓ | ✓ |
| 绝对尺度 | ✗ | ✓ | ✓ | ✓(初始化后) | ✓ | ✓ |
| IMU 预积分追踪 | ✗ | ✗ | ✗ | ✓ | ✓ | ✓ |
| 丢失时 IMU 预测 | ✗ | ✗ | ✗ | ✓ | ✓ | ✓ |
| LocalInertialBA | ✗ | ✗ | ✗ | ✓ | ✓ | ✓ |
| 全局 BA（回环后）| ✓ | ✓ | ✓ | 条件✓ | 条件✓ | 条件✓ |
| 4DoF 本质图优化 | ✗ | ✗ | ✗ | ✓ | ✓ | ✓ |
| 回环尺度固定 | ✗ | ✓ | ✓ | ✓(BA2后) | ✓ | ✓ |

---

## 15. 完整数据流图

```
传感器输入
    │
    ├── 图像（左/右/RGB/深度）
    │        │
    │        ▼
    │   ORBextractor（提取ORB特征，8层金字塔）
    │        │ mvKeys, mDescriptors
    │        ▼
    │     Frame（特征点, 网格, 深度, BoW）
    │        │
    └── IMU 测量
             │ mlQueueImuData
             ▼
         Tracking::PreintegrateIMU()
             │ mpImuPreintegrated
             ▼
         Tracking::Track()
             │
    ┌────────┴──────────────────────────────────────────┐
    │                Tracking 前端                      │
    │  初始化 → 位姿预测 → ORBmatcher → PoseOptimization  │
    │  → TrackLocalMap → NeedNewKeyFrame?               │
    └────────┬──────────────────────────────────────────┘
             │ InsertKeyFrame(pKF)
             ▼
    ┌────────────────────────────────────────────────────┐
    │             LocalMapping 中端                      │
    │  ProcessNewKeyFrame → CreateNewMapPoints           │
    │  → MapPointCulling → SearchInNeighbors            │
    │  → LocalBA/LocalInertialBA → InitializeIMU        │
    │  → KeyFrameCulling                                 │
    └────────┬───────────────────────────────────────────┘
             │ InsertKeyFrame(pKF)    ↑ UpdateFrameIMU()
             ▼                       │
    ┌────────────────────────────────────────────────────┐
    │             LoopClosing 后端                       │
    │  DBoW2 查询 → Sim3验证（时序一致性×3）             │
    │  → CorrectLoop（本质图优化）                        │
    │  或 MergeLocal（地图融合 + 焊接BA）                 │
    │  → GlobalBA（独立线程）                             │
    └────────────────────────────────────────────────────┘
             │ InformNewBigChange()
             ▼
         Atlas（多子地图管理）
         ├── 活跃 Map（KF + MP）
         ├── 已存储 Map（待融合）
         └── ← Boost序列化保存/加载

输出：
  每帧 Sophus::SE3f 位姿
  完整轨迹（SaveTrajectory*）
  地图文件（.osa）
```

---

## 参考文献

- [ORB-SLAM3] Campos et al., IEEE T-RO 37(6), 2021. https://arxiv.org/abs/2007.11898
- [IMU-Init] Campos et al., ICRA 2020. https://arxiv.org/pdf/2003.05766.pdf
- [ORBSLAM-Atlas] Elvira et al., IROS 2019. https://arxiv.org/pdf/1908.11585.pdf
- [DBoW2] Gálvez-López & Tardós, IEEE T-RO 28(5), 2012.
