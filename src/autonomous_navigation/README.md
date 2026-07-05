# TurtleBot3 Wheeled Robot for Autonomous Exploration

## 概述

本包实现了基于 TurtleBot3 Waffle Pi 轮式机器人的自主导航和探索系统，使用 Mid360 激光雷达进行环境感知。

## 系统架构

### 机器人配置
- **平台**: TurtleBot3 Waffle Pi (差速驱动)
- **传感器**: Livox Mid-360 激光雷达（与机器狗相同的安装位姿）
- **IMU**: 仿真 IMU，话题 `/trunk_imu`
- **控制接口**: `/cmd_vel` (geometry_msgs/Twist)

### 传感器对齐
| 传感器 | 坐标系 | 位置 (xyz) | 姿态 (rpy) | 话题 |
|--------|--------|-----------|-----------|------|
| Mid360 雷达 | `laser_livox` | `(0.2, 0.0, 0.08)` | `(0.0, 0.785, 0.0)` | `/scan` |
| IMU | `imu_link` | `(0, 0, 0)` | `(0, 0, 0)` | `/trunk_imu` |
| 里程计 | - | - | - | `/odom` |

## 文件结构

```
autonomous_navigation/
├── CMakeLists.txt              # Catkin 构建配置
├── package.xml                 # ROS 包依赖
├── launch/
│   ├── spawn_turtlebot3.launch    # 启动 Gazebo 和机器人
│   ├── slam.launch                # 启动 GMapping SLAM
│   ├── navigation.launch          # 启动 move_base 导航
│   └── exploration.launch         # 完整探索系统
├── config/
│   ├── robot_config.yaml       # 机器人参数配置
│   ├── slam_params.yaml        # GMapping 参数
│   └── nav_params.yaml         # 导航参数
├── urdf/
│   └── turtlebot3_waffle_pi.urdf.xacro  # 机器人 URDF
├── scripts/
│   ├── pointcloud_to_laserscan.py  # PointCloud2 转 LaserScan
│   └── exploration_manager.py      # 探索管理器
├── rviz/
│   └── exploration.rviz        # RViz 可视化配置
├── maps/                       # 存储生成的地图
└── worlds/
    └── turtlebot3_exploration.world  # Gazebo 世界文件
```

## 快速开始

### 1. 编译工作空间

```bash
cd /workspace
catkin_make
source devel/setup.bash
```

### 2. 启动完整探索系统

```bash
roslaunch autonomous_navigation exploration.launch
```

这将启动：
- Gazebo 仿真环境
- TurtleBot3 机器人
- GMapping SLAM
- move_base 导航栈
- explore_lite 前沿探索
- RViz 可视化

### 3. 单独启动组件

**仅启动机器人:**
```bash
roslaunch autonomous_navigation spawn_turtlebot3.launch
```

**仅启动 SLAM:**
```bash
roslaunch autonomous_navigation slam.launch
```

**仅启动导航:**
```bash
roslaunch autonomous_navigation navigation.launch
```

## 测试建图功能

1. 启动 SLAM:
```bash
roslaunch autonomous_navigation slam.launch
```

2. 在另一个终端手动控制机器人:
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

3. 保存地图:
```bash
rosservice call /dynamic_map
rosrun map_server map_saver -f ~/my_map
```

## 测试探索功能

```bash
roslaunch autonomous_navigation exploration.launch
```

系统会自动：
1. 检测未知区域边界（frontiers）
2. 选择最优探索目标点
3. 规划路径并移动
4. 重复直到完成探索

## 话题列表

| 话题 | 类型 | 说明 |
|------|------|------|
| `/cmd_vel` | geometry_msgs/Twist | 速度指令输入 |
| `/odom` | nav_msgs/Odometry | 里程计输出 |
| `/scan` | sensor_msgs/PointCloud2 | Mid360 原始点云 |
| `/scan_filtered` | sensor_msgs/LaserScan | 转换后的 2D 激光扫描 |
| `/trunk_imu` | sensor_msgs/Imu | IMU 数据 |
| `/map` | nav_msgs/OccupancyGrid | SLAM 生成的地图 |
| `/move_base/global_plan` | nav_msgs/Path | 全局路径 |
| `/move_base/local_plan` | nav_msgs/Path | 局部路径 |

## 服务

| 服务 | 类型 | 说明 |
|------|------|------|
| `/dynamic_map` | nav_msgs/GetMap | 获取当前地图 |
| `/move_base/clear_costmaps` | std_srvs/Empty | 清除代价地图 |

## 参数配置

### 机器人参数 (`config/robot_config.yaml`)
- 最大线速度：0.5 m/s
- 最大角速度：1.0 rad/s
- 轮距：0.287 m
- 轮子半径：0.033 m

### SLAM 参数 (`config/slam_params.yaml`)
- 地图分辨率：0.05 m/cell
- 粒子数：30
- 最大可用距离：10 m

### 导航参数 (`config/nav_params.yaml`)
- 全局规划器：NavfnROS
- 局部规划器：DWAPlannerROS
- 膨胀半径：0.55 m

## 故障排除

### 问题：机器人不移动
- 检查 `/cmd_vel` 话题是否有数据
- 确认 Gazebo 控制器插件加载正确
- 查看 `rostopic echo /odom` 是否有里程计数据

### 问题：SLAM 地图质量差
- 调整 `slam_params.yaml` 中的粒子数
- 降低机器人移动速度
- 确保激光雷达数据正常

### 问题：探索停滞不前
- 检查 `explore_lite` 节点状态
- 调整 `min_frontier_size` 参数
- 查看代价地图是否有障碍物阻塞

## 与机器狗的对比

| 特性 | 机器狗 (Unitree A1) | 轮式机器人 (TurtleBot3) |
|------|---------------------|------------------------|
| 自由度 | 12 (四足) | 2 (差速驱动) |
| CPU 占用 | 高 | 低 |
| 控制复杂度 | 高 (需要 RL) | 低 (直接速度控制) |
| 地形适应性 | 强 (可上下楼梯) | 弱 (仅平地) |
| 传感器配置 | Mid360 + IMU + 深度相机 | Mid360 + IMU |
| 控制接口 | `/cmd_vel` (RL 模式) | `/cmd_vel` (直接) |

## 下一步开发

1. [ ] 实现多层建筑探索
2. [ ] 添加回环检测优化
3. [ ] 集成危险区域检测
4. [ ] 优化探索策略
5. [ ] 添加多机器人协同探索

## 许可证

BSD License
