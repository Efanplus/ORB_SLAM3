# ORB-SLAM3 — Project Guide for Claude

## Project Overview

ORB-SLAM3 is a real-time Visual, Visual-Inertial, and Multi-Map SLAM library supporting monocular, stereo, and RGB-D cameras with pinhole and fisheye lens models.

**Authors:** Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M. M. Montiel, Juan D. Tardós  
**License:** GPLv3  
**Paper:** IEEE Transactions on Robotics 37(6):1874-1890, Dec. 2021

---

## Repository Structure

```
ORB_SLAM3/
├── include/            # C++ headers
│   ├── CameraModels/   # GeometricCamera, Pinhole, KannalaBrandt8
│   └── *.h             # Core module headers
├── src/                # C++ implementations
│   ├── CameraModels/   # Pinhole.cpp, KannalaBrandt8.cpp
│   └── *.cc / *.cpp    # Core module sources
├── Thirdparty/         # Bundled third-party libs (DBoW2, g2o, Sophus)
├── Vocabulary/         # Pre-trained ORB vocabulary (DBoW2)
├── Examples/           # RealSense demo programs + calibration YAMLs
├── Examples_old/       # Legacy example programs (EuRoC, TUM, KITTI)
├── evaluation/         # Python trajectory evaluation scripts
├── CMakeLists.txt      # Main CMake build file
├── build.sh            # Build script for Thirdparty + main lib
└── build_ros.sh        # ROS node build script
```

---

## Build System

- **Language:** C++11
- **Build:** CMake >= 2.8, optimized Release (`-O3 -march=native`)
- **Output:** `lib/libORB_SLAM3.so` (shared library)
- **Dependencies:**
  - OpenCV >= 4.4
  - Eigen3 >= 3.1.0
  - Pangolin (visualization)
  - DBoW2 (Thirdparty, modified)
  - g2o (Thirdparty, modified)
  - Sophus (Thirdparty, header-only)
  - Boost (for map serialization)

Build with:
```bash
chmod +x build.sh && ./build.sh
```

---

## Sensor Configuration Enum

```cpp
// include/System.h
enum eSensor {
    MONOCULAR     = 0,
    STEREO        = 1,
    RGBD          = 2,
    IMU_MONOCULAR = 3,
    IMU_STEREO    = 4,
    IMU_RGBD      = 5,
};
```

---

## Core Modules (see docs/ARCHITECTURE.md for full analysis)

| Module          | Files                   | Thread      | Role                                         |
|-----------------|-------------------------|-------------|----------------------------------------------|
| System          | System.cc/h             | Caller      | Entry point, thread orchestration            |
| Tracking        | Tracking.cc/h           | Caller      | Front-end: pose estimation per frame         |
| LocalMapping    | LocalMapping.cc/h       | Own thread  | Mid-end: map point creation, local BA        |
| LoopClosing     | LoopClosing.cc/h        | Own thread  | Back-end: loop detection, global BA, merge   |
| Atlas           | Atlas.cc/h              | Shared      | Multi-map container                          |
| Optimizer       | Optimizer.cc/h          | Shared      | g2o-based optimization problems              |
| ORBextractor    | ORBextractor.cc/h       | Per-frame   | Feature extraction                           |
| ORBmatcher      | ORBmatcher.cc/h         | Shared      | Feature matching strategies                  |
| Frame           | Frame.cc/h              | Per-frame   | Single image representation                  |
| KeyFrame        | KeyFrame.cc/h           | Shared      | Map keyframe with graph connectivity         |
| MapPoint        | MapPoint.cc/h           | Shared      | 3D landmark                                  |
| ImuTypes        | ImuTypes.cc/h           | Shared      | IMU preintegration, calibration              |
| CameraModels    | CameraModels/           | Shared      | Pinhole and KannalaBrandt8 fisheye           |
| KeyFrameDatabase| KeyFrameDatabase.cc/h   | Shared      | BoW inverted index for place recognition     |

---

## Key Conventions

- **Pose convention:** `mTcw` = camera-from-world (SE3), stored as `Sophus::SE3<float>`
- **IMU convention:** `T_wb` = world-from-body (IMU frame), `T_bc` = body-from-camera extrinsic
- **BoW:** DBoW2 at vocabulary level 4 (`mFeatVec`)
- **Feature count:** Default 1000 ORB features/frame (5000 during monocular init)
- **Scale pyramid:** 8 levels, scale factor 1.2
- **g2o reprojection thresholds:** chi2=5.991 (mono, 2 DOF), chi2=7.815 (stereo, 3 DOF)

---

## Documentation

- `docs/ARCHITECTURE.md` — Full algorithmic architecture and data flow documentation
- `README.md` — Setup, dependencies, running instructions
- `Changelog.md` — Version history
- `Dependencies.md` — Third-party library details
- `Calibration_Tutorial.pdf` — Camera+IMU calibration guide
