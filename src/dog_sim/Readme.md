# MyDog - 四足机器人仿真与控制系统

<p align="center">
  <strong>基于强化学习的四足机器人运动控制仿真平台</strong>
</p>

---

## 📋 项目简介

MyDog 是一个基于 ROS Noetic 的四足机器人仿真与控制系统，使用强化学习策略实现机器人运动控制。项目采用 MNN（Mobile Neural Network）作为神经网络推理引擎，在 Gazebo 仿真环境中进行验证。

## ✨ 主要功能

- 🤖 四足机器人 Gazebo 仿真环境
- 🧠 基于 MNN 的强化学习策略部署
- 🎮 支持手柄（Joystick）遥控
- 📊 RQT 可视化界面
- 🔧 状态估计与控制

## 📁 项目结构

```
mydog/
├── ask_3_description/      # ASK-3 机器人 URDF 描述文件
├── ask_4_description/      # ASK-4 机器人 URDF 描述文件
├── mydog_control_sim/      # 仿真控制节点（RL 策略推理）
├── mydog_gazebo/           # Gazebo 仿真环境配置
├── mydog_state_estimator/  # 状态估计模块
├── MNN/                    # MNN 深度学习推理框架
├── rqt_dog_gui/            # RQT 机器人 GUI 插件
├── rqt_robot_gui/          # RQT 通用机器人 GUI
├── rqt_rviz_wrapper/       # RQT RViz 封装
└── qt/                     # Qt 组件库
```

## 🛠️ 环境要求

- **操作系统**: Ubuntu 20.04
- **ROS 版本**: ROS Noetic
- **编译工具**: catkin tools

## 📦 依赖安装

### 1. ROS 控制器

```bash
sudo apt install ros-noetic-ros-control ros-noetic-ros-controllers
sudo apt install ros-noetic-joy
sudo apt-get install wmctrl
```

### 2. Catkin Tools

请参考 [官方安装文档](https://catkin-tools.readthedocs.io/en/latest/installing.html) 进行安装。

### 3. MNN 深度学习框架

MNN 是阿里巴巴开源的轻量级深度学习推理引擎，本项目用于部署强化学习策略。

```bash
cd mydog/MNN
mkdir build && cd build
cmake .. -DMNN_BUILD_CONVERTER=ON -DMNN_BUILD_TORCH=ON
make -j$(nproc)
```

## 🚀 编译与运行

### 编译项目

```bash
# 创建工作空间（如果尚未创建）
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# 克隆项目到 src 目录

# 返回工作空间根目录
cd ~/catkin_ws

# 初始化并配置 catkin
catkin init
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release

# 编译
catkin build

# 加载环境变量
source devel/setup.bash
```

### 启动仿真

```bash
roslaunch mydog_control_sim_ros sim.launch
```

## 🎮 使用说明

启动仿真后，会自动打开 RQT GUI 控制面板和 RViz 可视化界面。

### RQT GUI 控制面板

GUI 面板提供以下控制按钮：

| 按钮 | 功能 |
|------|------|
| **启动 (Start)** | 启用机器人控制，机器人开始响应运动指令 |
| **行走 (Walk)** | 切换到行走模式 |
| **站立 (Stand)** | 切换到站立模式 |
| **前进 (Forward)** | 控制机器人向前移动，可通过滑块调节速度 |
| **后退 (Back)** | 控制机器人向后移动，可通过滑块调节速度 |
| **左移 (Left)** | 控制机器人向左平移，可通过滑块调节速度 |
| **右移 (Right)** | 控制机器人向右平移，可通过滑块调节速度 |
| **左转 (Turn Left)** | 控制机器人原地左转，可通过滑块调节角速度 |
| **右转 (Turn Right)** | 控制机器人原地右转，可通过滑块调节角速度 |
| **显示状态 (Show State)** | 显示机器人关节状态信息 |
| **隐藏状态 (Hide State)** | 隐藏机器人关节状态信息 |

**操作步骤：**
1. 点击 **启动** 按钮激活机器人控制
2. 点击 **行走** 按钮进入行走模式
3. 使用方向按钮（前进/后退/左移/右移）控制机器人移动
4. 使用转弯按钮（左转/右转）控制机器人转向
5. 通过滑块调节各方向的移动速度和转弯角速度

### 手柄控制

本项目也支持手柄遥控，已在 **Logitech F710** 手柄上测试通过。如使用其他手柄，可能需要修改 `mydog_control.yaml` 中的 `linearX` 和 `linearY` 参数映射。

## 📧 联系方式

- **作者**: Guiyang Xin
- **邮箱**: gyxin@outlook.com

## 📄 许可证

本项目遵循 BSD 许可证。

---

<p align="center">
  <em>ASK-Robotics © 2023</em>
</p>

