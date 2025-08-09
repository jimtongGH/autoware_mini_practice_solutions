# 🚗 Autoware Mini Practice Solutions

[English](#english-version) | [中文](#中文版本)

---

## English Version

### 📌 Project Overview
This repository contains practice solutions for the **Autoware Mini** autonomous driving project completed during the **University of Tartu Summer School**.  
It is based on the **Autoware Mini** architecture and **ROS 1** middleware, covering Python implementations and tests for key autonomous driving modules.

The project adopts a **modular design**, implementing major functions such as perception, localization, planning, and control, suitable for learning and validating core algorithms and system integration in autonomous driving.

---

### 🔑 Main Modules & Features

#### **Localization**
- GNSS data processing and UTM coordinate conversion.
- ROS TF broadcasting with fusion of vehicle and sensor data.  
  _(see `nodes/localization/localizer.py`)_

#### **Planning**
- **Global Planner**: Path computation based on **Lanelet2** HD maps.  
  _(see `nodes/planning/global/lanelet2_global_planner.py`)_
- **Local Planner**: Local trajectory extraction & speed planning, collision point detection.  
  _(see `nodes/planning/local/local_path_extractor.py`, `collision_points_manager.py`, `simple_speed_planner.py`)_

#### **Detection & Clustering**
- LiDAR point cloud clustering using **DBSCAN**.  
  _(see `nodes/detection/points_clusterer.py`, `cluster_detector.py`)_
- Traffic light recognition: Camera-based detection with ONNX model, MQTT interface supported.  
  _(see `nodes/detection/camera_traffic_light_detector.py`)_

#### **Control**
- Pure Pursuit algorithm for lateral control.  
  _(see `nodes/control/pure_pursuit_follower.py`)_
- Supports both lateral and longitudinal (braking) control.

---

### 🛠 Configuration & Visualization
- Configuration files: `config/`
- RViz visualization configs: `rviz/`
- Launch files: `launch/`

---

### 🧪 Testing & Simulation
- Includes simulation and testing records, e.g., `practice_8_simulation.txt`.

---

### 📦 Dependencies
- ROS 1 (**Melodic** / **Noetic** recommended)
- Python 3.x
- Autoware Mini
- CMake  
_See `package.xml` and source code comments for more details._

---

### 📂 Project Directory
```text
nodes/localization/       # Localization nodes — GNSS processing, UTM conversion, TF broadcasting
nodes/planning/global/    # Global path planning using Lanelet2 maps
nodes/planning/local/     # Local trajectory extraction, speed planning, collision point detection
nodes/detection/          # Perception modules — LiDAR clustering, traffic light detection
nodes/control/            # Control algorithms — Pure Pursuit, lateral & longitudinal control
config/                   # Configuration files for modules
launch/                   # ROS launch files
rviz/                     # RViz visualization configuration
```



---

## 中文版本

### 📌 项目概述
本仓库为 **塔尔图大学暑期学校** 期间完成的 **Autoware Mini** 自动驾驶项目实践解决方案。  
项目基于 **Autoware Mini** 架构与 **ROS 1** 中间件，实现并测试了自动驾驶系统关键模块的 Python 版本。

该项目采用 **模块化设计**，实现了感知、定位、规划与控制等主要功能，适用于学习和验证自动驾驶核心算法与系统集成。

---

### 🔑 主要模块与功能

#### **定位（Localization）**
- GNSS 数据处理与 UTM 坐标转换
- ROS 坐标广播，融合车型与传感器数据  
  _(见 `nodes/localization/localizer.py`)_

#### **规划（Planning）**
- **全局规划**：基于 **Lanelet2** 高精地图的路径计算  
  _(见 `nodes/planning/global/lanelet2_global_planner.py`)_
- **局部规划**：轨迹提取与速度规划，碰撞点检测  
  _(见 `nodes/planning/local/local_path_extractor.py`, `collision_points_manager.py`, `simple_speed_planner.py`)_

#### **感知与聚类（Detection & Clustering）**
- 基于 **DBSCAN** 的激光雷达点云聚类  
  _(见 `nodes/detection/points_clusterer.py`, `cluster_detector.py`)_
- 交通灯识别：基于相机和 ONNX 模型的检测，支持 MQTT 接口  
  _(见 `nodes/detection/camera_traffic_light_detector.py`)_

#### **控制模块（Control）**
- 纯跟踪算法（Pure Pursuit）实现横向控制  
  _(见 `nodes/control/pure_pursuit_follower.py`)_
- 支持横向与纵向（制动）控制

---

### 🛠 配置与可视化
- 配置文件：`config/`
- RViz 可视化配置：`rviz/`
- 启动文件：`launch/`

---

### 🧪 测试与仿真
- 提供 `practice_8_simulation.txt` 等仿真与测试记录

---

### 📦 依赖环境
- ROS 1（建议 Melodic / Noetic）
- Python 3.x
- Autoware Mini
- CMake  
_详见 `package.xml` 和各节点源码的依赖注释_

---

### 📂 目录结构
```text
nodes/localization/       # 定位相关节点 —— GNSS 数据处理、UTM 坐标转换、TF 广播
nodes/planning/global/    # 全局路径规划 —— 基于 Lanelet2 高精地图
nodes/planning/local/     # 局部轨迹与速度规划、碰撞点检测
nodes/detection/          # 感知模块 —— 激光雷达聚类、交通灯检测
nodes/control/            # 控制算法 —— 纯跟踪、横向与纵向控制
config/                   # 各模块的配置文件
launch/                   # ROS 启动文件
rviz/                     # RViz 可视化配置文件
```
