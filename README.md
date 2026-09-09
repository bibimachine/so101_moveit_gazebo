# so101_moveit_gazebo

[![ROS 2](https://img.shields.io/badge/ROS_2-Humble-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Classic_11-orange)](http://classic.gazebosim.org/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-E95420?logo=ubuntu&logoColor=white)](https://releases.ubuntu.com/22.04/)
[![GitHub](https://img.shields.io/badge/GitHub-bibimachine%2Fso101__moveit__gazebo-181717?logo=github&logoColor=white)](https://github.com/bibimachine/so101_moveit_gazebo)

SO101 机械臂 + ROS 2 + Gazebo 的**学习实践项目**。从 URDF 建模起步，逐步接入仿真、控制、视觉和数据采集，目标最终走到 MoveIt 规划与直接操控。

## 环境

| 项目 | 版本 |
|---|---|
| ROS 2 | Humble |
| Gazebo | Classic 11 |
| 系统 | Ubuntu 22.04(WSL2/WSLg) |
| Python | 3.10 |

## 目前做了什么

- [x] SO101 完整 URDF/xacro 描述(5 臂 + 单活动指夹爪,STL 网格)
- [x] ros2_control 双硬件后端:mock(快速显示)和 Gazebo(物理仿真)
- [x] Gazebo Classic 接入:world 固定、链式控制器加载、重力保持调参记录
- [x] effort 力控夹爪 + 一键抓取演示脚本 `grasp_cube.py`
- [x] 固定外置 RGB 相机(eye-to-hand):`/so101_camera/image_raw`
- [x] 数据采集:`ros2 bag record` 封装脚本(mcap,话题含控制指令,对齐 LeRobot 数据集要素)
- [x] RViz 显示与相机 FPS 查看工具

## TODO

- [ ] 接入 MoveIt 2(运动规划、碰撞检测)
- [ ] 直接操控(遥操作示教,采集 demonstration 数据)
- [ ] (可选)腕部第二相机、深度相机

## 快速开始

```bash
cd so101_moveit_gazebo
colcon build && source install/setup.bash

# 纯显示(mock 硬件,启动快)
ros2 launch so101_moveit_gazebo so101_display.launch.py

# Gazebo 仿真(重跑前:pkill -x gzserver; pkill -x gzclient)
ros2 launch so101_moveit_gazebo so101_gazebo.launch.py

# 夹取方块演示(需先启动 gazebo launch;回位→重置方块→闭合→抬升→验证,退出码 0=夹住)
ros2 run so101_moveit_gazebo grasp_cube.py --release

# 数据采集(需先启动 gazebo launch)
ros2 run so101_moveit_gazebo record_so101.sh -h
```

## 文档

- [src/so101_moveit_gazebo/doc/文件说明.md](src/so101_moveit_gazebo/doc/文件说明.md) — 各文件作用说明
- [src/so101_moveit_gazebo/doc/问题排查记录.md](src/so101_moveit_gazebo/doc/问题排查记录.md) — 全部踩坑记录(Gazebo 隐形、RViz mesh 加载、numpy 冲突等)
