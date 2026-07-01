# ASK-3 使用 dog_sim 原始运动控制的导航仿真说明

实验日期：2026-05-14

## 1. 调整目标

根据当前要求，路径规划部分只负责全局/局部路径决策和速度命令输出，机器人在 Gazebo 中的四足运动姿态不再由 `dog_hybrid_planner` 的外部动画节点生成，而是交给 `dog_sim` 中已有的低层运动控制代码。

`dog_sim` 中对应的运动控制入口为：

```text
dog_sim/mydog_control_sim/mydog_control_sim_ros/src/mydog_control_sim_ros/MydogControlSimRos.cpp
```

该节点订阅路径规划输出的三路命令：

```text
/ask/dog/forward_back
/ask/dog/left_right
/ask/dog/yaw
```

并通过 MNN/RL 推理发布 12 个关节位置控制命令，从而生成四足运动姿态。

## 2. 已完成的工程调整

1. 撤回了对 `dog_sim` 低层运动控制源码的改动，保持 `MydogControlSimRos.hpp/.cpp` 与仓库基线一致。
2. 删除外部步态动画脚本，不再使用 `dog_gait_animator.py`。
3. `dog_ask3_lab.launch` 默认改为：

```text
enable_low_level_gait = true
use_kinematic_driver = false
```

4. 路径规划节点仍发布 `/ask/dog/*` 速度命令，并在接收到目标时发布 `/ask/dog/start` 与 `/ask/dog/walk`，由 `dog_sim` 原始运动控制节点执行步态。

## 3. 保留的路径规划改动

路径规划层面仍保留适配 ASK-3 的修改：

- 真实 Gazebo lidar 发布 `/scan`；
- AMCL 使用 `/scan` 定位；
- 改进 A* 使用适合 ASK-3 尺寸的膨胀半径；
- OmniDWA 回退到更接近 8:45 版本的圆形局部安全半径与原始评分权重；
- 普通导航横向速度上限为 `0.16 m/s`，只通过轻量评分项降低非必要侧移；
- 全局 A* 仍使用较大的 ASK-3 膨胀半径，局部 DWA `hard_radius` 调整为 `0.13 m`，避免墙边/窄缝处把可通过候选全部过滤掉；
- DWA 制动约束只检查运动方向前方通道，减少平行侧墙导致的走走停停；
- 紧急停车检测从 `/scan` 的宽角度扇区改为头部正前方窄通道，避免贴墙直行时被侧墙误触发；
- `/scan` 障碍点加入 ASK-3 机身自体过滤，移除落在机身矩形内部的雷达点，避免机器人把自身识别为障碍；
- 导航器在 DWA 前新增路径对齐阶段：当机器人头部与规划路径夹角过大且转向空间充足时，先用后退、旋转和侧移组合完成头部对齐，再继续前进寻路；
- 若路径对齐所需转向空间不足，但机器人并未同时被前方和后/侧方围住，则不再强制进入脱困恢复，而是交回 DWA 继续沿窄通道通过；
- 局部目标由优先跟踪稀疏关键点改为优先沿稠密 `global_smooth_path` 前视，默认前视距离由 `0.50 m` 收短为 `0.35 m`，避免目标过早跳到墙端或拐角之后；
- 路径朝向对齐的前视距离恢复到 `0.60 m`，并将触发角改为 `22 deg`、完成角改为 `8 deg`，提高充足空间下头部对齐路线的灵敏度；
- 周期性全局重规划保持开启，但执行路径切换增加滞回：若新旧路径局部方向差超过 `70 deg`、局部目标跳变超过 `0.55 m`，或横向目标跳变超过 `0.30 m`，则继续跟踪当前路径，避免 DWA 在 A/B 两条差异很大的路线间反复切换；
- 当机器人偏离当前执行路径超过 `0.60 m` 时强制重规划，避免控制器长期抓住已经失效的旧路径，同时减少路口附近过于敏感的反复重规划；
- 路径对齐阶段最长持续时间调整为 `3.0 s`，超时后冷却增加到 `2.0 s`，让 DWA 获得足够连续控制时间；
- 稠密路径局部目标改为沿当前路径索引单调推进，避免目标与机器人隔墙时误选墙另一侧的路径段；
- 前方紧急检测不再直接全停并跳过 DWA，而是只限制继续正向 `vx`，允许机器人在路口/墙角附近继续执行侧移、后退或转向；
- `no_progress` 不再单独触发 recovery：只有机器人附近 `0.45 m` 内存在障碍、前方受限或侧向受限时，才允许由无进展状态启动 recovery；开阔空间中继续保留 DWA 控制权；
- recovery 结束后清空旧的进度监测样本，并给 DWA `2.5 s` 连续控制窗口，避免恢复前的无进展记录导致恢复刚结束又立刻再次进入 recovery；
- 障碍短边绕行时，若局部目标主要位于机器人侧向，DWA 会奖励同方向 `vy` 并惩罚无意义 `w` 旋转；
- 恢复策略调整为先后退、再小角度转向，最后侧步，减少墙角处旋转与侧移反复。

## 4. 窄通道卡顿原因评估

本次检查认为主要负面影响来自 DWA 局部跟踪/路径对齐层，而不是 A* 的贝塞尔平滑本身。依据如下：

- `ImprovedAStar.plan()` 会先生成贝塞尔平滑路径，再调用 `path_is_safe()` 在膨胀后的代价地图上检查每个路径点和路径段；
- 若平滑路径穿入膨胀障碍区域，代码会打印 `[A*] Bezier path intersects inflated obstacles; using raw safe path` 并回退到原始 A* 路径；
- 因此，危险的贝塞尔曲线通常不会直接交给局部控制执行；
- 更明显的问题是局部目标原先来自稀疏关键点，窄通道/墙端附近会让 DWA 追逐过远目标，从而反复在前进、旋转和侧移之间切换。

已采取的修复：

- `get_local_target()` 优先使用 `global_smooth_path`，并沿当前路径索引单调推进约 `0.35 m` 作为局部目标，避免隔墙路径段的欧氏距离误匹配；
- `_path_alignment_cmd()` 在转向空间不足时不再立即判定失败，只有当前方受阻且后方或侧方也受限，即接近“被墙角困住”时，才触发脱困恢复；
- `dog_nav.launch` 同步更新 `lookahead=0.35`、`align_path_lookahead=0.60`。

## 5. 路径更新抖动修复

问题现象：RViz 上全局路径会周期性更新，当新旧两条路径在机器人附近分布到不同侧时，机器人先原地旋转尝试对齐路径；旋转造成起点姿态和局部位置变化后，下一次重规划又可能把路径切到另一侧，于是形成“旋转找路径-路径换边-继续旋转”的死循环。

处理方式：

- `periodic_replan` 默认设回 `true`，RViz 中可以继续看到周期性全局规划结果；
- 新路径若相对当前执行路径出现明显局部大跳变，则拒绝这次周期路径切换：包括局部方向跳变超过 `70 deg`、局部目标跳变超过 `0.55 m`，或横向目标跳变超过 `0.30 m`。
- 如果机器人已经偏离当前执行路径超过 `0.60 m`，说明旧路径不再适合继续跟踪，此时跳过路径切换防抖并从当前位置强制重规划；重规划成功后同一控制周期继续进入局部目标和 DWA 计算，不再直接跳过速度输出。
- 路径对齐阶段最多连续执行 `3.0 s`；若仍未达到 `8 deg` 完成阈值，则交回 DWA 控制并冷却 `2.0 s`，避免刚交回 DWA 又立刻被原地对齐抢走控制权。
- 前方紧急检测距离调整为 `0.16 m`，并改为“只阻止继续正向顶撞”，不再阻断可行的侧向绕行或转向。
- recovery 的 `no_progress` 触发增加空间约束和 `3.0 s` 延迟：若 DWA 没有报告 stuck 且周围空间开阔，即使目标距离暂时没有下降，也不会进入固定后退/转向/侧步流程。
- recovery 完成后会重置 `ProgressMonitor`，并暂时抑制 `no_progress` 对 recovery 的再次触发，让 DWA 有 `2.5 s` 时间重新产生前进速度和实际位移。

## 6. 启动方式

推荐启动：

```bash
cd /home/cjx/catkin_ws
source devel/setup.bash
roslaunch dog_hybrid_planner dog_ask3_lab.launch paused:=false gazebo_gui:=true open_rviz:=true
```

此时：

- `dog_hybrid_navigator` 负责规划；
- `mydog_control_sim_ros` 负责四足低层运动；
- `dog_kinematic_driver` 默认不启动；
- 外部步态动画节点不启动。

## 7. 校验结果

已完成以下静态校验：

```bash
python3 -m py_compile dog_hybrid_planner/scripts/dog_navigation.py dog_hybrid_planner/scripts/omni_dwa.py dog_hybrid_planner/scripts/improved_astar.py
xmllint --noout dog_hybrid_planner/launch/dog_nav.launch dog_hybrid_planner/launch/dog_all.launch dog_hybrid_planner/launch/dog_ask3_lab.launch
```

同时进行了离线逻辑检查：当机器人处于窄通道且只是侧方靠近障碍时，不再触发强制恢复；当前方同时受阻、接近墙角困住时，仍会触发恢复。

另进行了路径切换与路径跟踪离线检查：局部目标不会因隔墙路径段的欧氏距离较近而跳到远处路径段；约 `25 deg` 头部偏差会触发对齐；大方向跳变、大横向跳变会被拒绝，同方向小幅调整会接受。

## 8. 注意事项

使用 `dog_sim` 原始低层运动控制后，机器人真实物理位移完全依赖原 MNN/RL 控制器与 Gazebo 接触动力学，不再由运动学节点直接写 Gazebo 位姿。因此，相比此前的规划验证模式，实验成功率和耗时会更受低层模型、地面摩擦、关节 PID、MNN 策略输出影响。

如果后续需要做纯路径规划算法对比，可以临时将：

```text
use_kinematic_driver = true
enable_low_level_gait = false
```

作为“规划算法验证模式”。但按照当前要求，默认实验模式已经切回 `dog_sim` 原始四足运动控制。
