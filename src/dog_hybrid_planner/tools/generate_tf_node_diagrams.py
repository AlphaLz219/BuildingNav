#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Generate TF and ROS node diagrams for the ASK-3 hybrid planner.

The diagrams are paper/report assets only. They reflect the launch files in
dog_hybrid_planner and the ASK-3 Gazebo integration in dog_sim.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager, rcParams
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch


def configure_fonts() -> None:
    """Use a Chinese-capable font when the host has one installed."""

    preferred = [
        "Noto Sans CJK SC",
        "Noto Sans CJK JP",
        "Source Han Sans SC",
        "WenQuanYi Zen Hei",
        "SimHei",
        "Microsoft YaHei",
        "Arial Unicode MS",
    ]
    available = {font.name for font in font_manager.fontManager.ttflist}
    for name in preferred:
        if name in available:
            rcParams["font.sans-serif"] = [name, "DejaVu Sans"]
            break
    else:
        rcParams["font.sans-serif"] = ["DejaVu Sans"]
    rcParams["axes.unicode_minus"] = False


def add_box(
    ax,
    x: float,
    y: float,
    w: float,
    h: float,
    text: str,
    *,
    fc: str,
    ec: str = "#2d3748",
    text_color: str = "#111827",
    lw: float = 1.3,
    fs: float = 10.5,
    weight: str = "normal",
    alpha: float = 1.0,
    dashed: bool = False,
) -> None:
    patch = FancyBboxPatch(
        (x, y),
        w,
        h,
        boxstyle="round,pad=0.035,rounding_size=0.08",
        linewidth=lw,
        edgecolor=ec,
        facecolor=fc,
        alpha=alpha,
        linestyle="--" if dashed else "-",
    )
    ax.add_patch(patch)
    ax.text(
        x + w / 2,
        y + h / 2,
        text,
        ha="center",
        va="center",
        fontsize=fs,
        color=text_color,
        weight=weight,
        linespacing=1.35,
    )


def add_frame_box(ax, x: float, y: float, text: str, fc: str = "#eff6ff") -> None:
    add_box(
        ax,
        x,
        y,
        1.55,
        0.72,
        text,
        fc=fc,
        ec="#1d4ed8",
        lw=1.6,
        fs=12,
        weight="bold",
    )


def add_arrow(
    ax,
    start: tuple[float, float],
    end: tuple[float, float],
    *,
    label: str | None = None,
    color: str = "#374151",
    lw: float = 1.6,
    dashed: bool = False,
    rad: float = 0.0,
    fs: float = 9.2,
    label_offset: tuple[float, float] = (0.0, 0.0),
    label_fc: str = "white",
    arrowstyle: str = "-|>",
) -> None:
    arrow = FancyArrowPatch(
        start,
        end,
        arrowstyle=arrowstyle,
        mutation_scale=14,
        linewidth=lw,
        color=color,
        linestyle="--" if dashed else "-",
        connectionstyle=f"arc3,rad={rad}",
        shrinkA=2,
        shrinkB=2,
    )
    ax.add_patch(arrow)
    if label:
        mx = (start[0] + end[0]) / 2 + label_offset[0]
        my = (start[1] + end[1]) / 2 + label_offset[1]
        ax.text(
            mx,
            my,
            label,
            ha="center",
            va="center",
            fontsize=fs,
            color=color,
            bbox=dict(boxstyle="round,pad=0.18", fc=label_fc, ec="none", alpha=0.90),
            linespacing=1.25,
        )


def add_title(ax, title: str, subtitle: str | None = None) -> None:
    ax.text(8.0, 9.72, title, ha="center", va="top", fontsize=18, weight="bold")
    if subtitle:
        ax.text(8.0, 9.27, subtitle, ha="center", va="top", fontsize=10.5, color="#475569")


def draw_tf_diagram(output_path: Path) -> None:
    fig, ax = plt.subplots(figsize=(16, 10), dpi=220)
    ax.set_xlim(0, 16)
    ax.set_ylim(0, 10)
    ax.axis("off")

    add_title(
        ax,
        "四足机器人路径规划 TF 框架图",
        "ASK-3 Gazebo + AMCL 定位 + 改进 A* / DWA 导航；主 TF 链为 map → odom → base → laser/腿部关节",
    )

    # Data sources.
    add_box(ax, 0.55, 7.35, 2.45, 0.88, "map_server\n/map 静态栅格地图", fc="#ecfdf5", ec="#047857")
    add_box(ax, 0.55, 6.05, 2.45, 0.88, "Gazebo LiDAR\n/scan, frame=laser", fc="#ecfdf5", ec="#047857")
    add_box(ax, 0.55, 4.75, 2.45, 0.88, "Gazebo 模型状态\n/gazebo/model_states", fc="#ecfdf5", ec="#047857")
    add_box(ax, 0.55, 3.45, 2.45, 0.88, "关节状态\n/ask_3/joint_states", fc="#ecfdf5", ec="#047857")

    # TF broadcasters.
    add_box(
        ax,
        4.05,
        7.02,
        3.25,
        1.25,
        "AMCL\n广播 map → odom\n输入 /map + /scan + /odom",
        fc="#eff6ff",
        ec="#2563eb",
        fs=10.2,
        weight="bold",
    )
    add_box(
        ax,
        4.05,
        4.52,
        3.25,
        1.18,
        "mydog_state_estimator_ros_node\n广播 odom → base\n发布 /odom",
        fc="#eff6ff",
        ec="#2563eb",
        fs=10.0,
        weight="bold",
    )
    add_box(
        ax,
        4.05,
        2.82,
        3.25,
        1.18,
        "robot_state_publisher\nURDF + joint_states\n发布 base → laser/腿部",
        fc="#eff6ff",
        ec="#2563eb",
        fs=10.0,
        weight="bold",
    )
    add_box(
        ax,
        4.25,
        6.05,
        2.85,
        0.68,
        "备用：gt_pose_bridge\nuse_amcl=false 时 map → odom",
        fc="#f8fafc",
        ec="#64748b",
        fs=8.6,
        dashed=True,
    )
    add_box(
        ax,
        4.25,
        1.98,
        2.85,
        0.62,
        "备用：fake_scan_from_map\n无雷达调试时生成 /scan",
        fc="#f8fafc",
        ec="#64748b",
        fs=8.4,
        dashed=True,
    )

    # Frame tree.
    add_frame_box(ax, 8.85, 7.55, "map", "#fef3c7")
    add_frame_box(ax, 10.95, 7.55, "odom", "#fef3c7")
    add_frame_box(ax, 13.05, 7.55, "base", "#fef3c7")
    add_frame_box(ax, 13.05, 6.05, "laser", "#fff7ed")
    add_box(
        ax,
        12.25,
        4.18,
        3.15,
        1.05,
        "四足腿部 TF 子树\nRF/LF/RH/LH hip-thigh-shank-foot",
        fc="#fff7ed",
        ec="#ea580c",
        fs=9.5,
        weight="bold",
    )

    add_arrow(ax, (10.40, 7.91), (10.95, 7.91), label="map → odom", color="#b45309", lw=2.0, fs=8.8, label_offset=(0, 0.32))
    add_arrow(ax, (12.50, 7.91), (13.05, 7.91), label="odom → base", color="#b45309", lw=2.0, fs=8.8, label_offset=(0, 0.32))
    add_arrow(ax, (13.83, 7.55), (13.83, 6.77), label="固定关节\nbase → laser", color="#ea580c", lw=1.8, fs=8.2, label_offset=(0.85, -0.05))
    add_arrow(ax, (13.83, 7.55), (13.83, 5.25), label="活动关节\nbase → legs", color="#ea580c", lw=1.8, fs=8.2, label_offset=(-0.85, -0.25), rad=0.0)

    # Source-to-broadcaster relations.
    add_arrow(ax, (3.0, 7.79), (4.05, 7.72), label="/map", color="#047857", fs=8.4)
    add_arrow(ax, (3.0, 6.49), (4.05, 7.28), label="/scan", color="#047857", fs=8.4, rad=0.18, label_offset=(-0.12, 0.13))
    add_arrow(ax, (3.0, 4.98), (4.05, 5.05), label="/gazebo/model_states", color="#047857", fs=8.0)
    add_arrow(ax, (3.0, 3.89), (4.05, 3.40), label="/joint_states", color="#047857", fs=8.4, rad=-0.15)

    add_arrow(ax, (7.30, 7.62), (9.05, 8.05), label="/tf", color="#2563eb", fs=8.4, rad=-0.10)
    add_arrow(ax, (7.30, 5.10), (12.95, 7.72), label="/tf + /odom", color="#2563eb", fs=8.2, rad=-0.13, label_offset=(0.20, 0.22))
    add_arrow(ax, (7.30, 3.42), (12.35, 4.80), label="/tf", color="#2563eb", fs=8.4, rad=0.08)
    add_arrow(ax, (7.10, 6.40), (9.02, 7.62), label="备用 /tf", color="#64748b", dashed=True, fs=7.8, rad=-0.15)

    # Consumers.
    add_box(
        ax,
        5.20,
        0.65,
        5.65,
        1.45,
        "dog_hybrid_navigator\nTF lookup：map → base、map → laser\n输入 /map /scan /odom /goal，输出 A*全局路径、DWA局部路径与三路速度",
        fc="#fff7ed",
        ec="#f97316",
        fs=9.4,
        weight="bold",
    )
    add_box(
        ax,
        12.20,
        0.78,
        3.10,
        1.18,
        "RViz\n显示 TF / Map / Scan\n/dog_global_path /dog_dwa_path",
        fc="#f8fafc",
        ec="#475569",
        fs=9.3,
        weight="bold",
    )

    add_arrow(ax, (10.60, 7.55), (8.00, 2.10), label="TF 缓冲查询", color="#7c2d12", dashed=True, fs=8.4, rad=0.18, label_offset=(-0.60, -0.10))
    add_arrow(ax, (13.85, 6.05), (9.35, 2.10), label="雷达点转入 map", color="#7c2d12", dashed=True, fs=8.2, rad=-0.22, label_offset=(0.50, -0.12))
    add_arrow(ax, (13.85, 7.55), (13.75, 1.96), label="TF 可视化", color="#475569", dashed=True, fs=8.4, rad=0.05, label_offset=(0.72, -0.25))
    add_arrow(ax, (10.85, 1.38), (12.20, 1.38), label="路径/Marker", color="#f97316", fs=8.4)

    # Legend.
    add_box(ax, 0.55, 0.75, 3.20, 1.15, "颜色说明\n绿色：传感/地图输入\n蓝色：TF广播节点\n橙色：规划与机器人坐标树", fc="#ffffff", ec="#cbd5e1", fs=8.6)

    fig.savefig(output_path, bbox_inches="tight", pad_inches=0.22)
    plt.close(fig)


def draw_node_diagram(output_path: Path) -> None:
    fig, ax = plt.subplots(figsize=(18, 11), dpi=220)
    ax.set_xlim(0, 18)
    ax.set_ylim(0, 11)
    ax.axis("off")

    ax.text(9.0, 10.65, "四足机器人路径规划 ROS 节点图", ha="center", va="top", fontsize=18, weight="bold")
    ax.text(
        9.0,
        10.18,
        "从 Gazebo 感知/状态反馈，到 AMCL 定位、改进 A* + DWA 规划，再到 ASK-3 四足步态控制与 RViz 可视化",
        ha="center",
        va="top",
        fontsize=10.6,
        color="#475569",
    )

    # Swimlane backgrounds.
    add_box(
        ax,
        0.35,
        0.95,
        4.45,
        8.85,
        "",
        fc="#f0fdf4",
        ec="#bbf7d0",
        text_color="#047857",
        lw=1.0,
        fs=10,
        weight="bold",
        alpha=0.45,
    )
    add_box(
        ax,
        4.95,
        0.95,
        5.55,
        8.85,
        "",
        fc="#eff6ff",
        ec="#bfdbfe",
        text_color="#2563eb",
        lw=1.0,
        fs=10,
        weight="bold",
        alpha=0.45,
    )
    add_box(
        ax,
        10.70,
        0.95,
        6.95,
        8.85,
        "",
        fc="#fff7ed",
        ec="#fed7aa",
        text_color="#f97316",
        lw=1.0,
        fs=10,
        weight="bold",
        alpha=0.45,
    )
    ax.text(0.60, 9.62, "仿真与传感层", ha="left", va="top", fontsize=10.5, weight="bold", color="#047857")
    ax.text(5.20, 9.62, "定位与 TF 层", ha="left", va="top", fontsize=10.5, weight="bold", color="#2563eb")
    ax.text(10.95, 9.62, "规划、执行与可视化层", ha="left", va="top", fontsize=10.5, weight="bold", color="#f97316")

    # Simulation and sensing nodes.
    add_box(ax, 0.85, 8.45, 3.25, 0.92, "Gazebo 世界 + ASK-3 模型", fc="#ecfdf5", ec="#047857", fs=10.2, weight="bold")
    add_box(ax, 0.85, 6.85, 3.25, 0.92, "gazebo_ros_laser\n发布 /scan, frame=laser", fc="#ecfdf5", ec="#047857", fs=9.7, weight="bold")
    add_box(ax, 0.85, 5.25, 3.25, 0.92, "joint_state_controller\n发布 /ask_3/joint_states", fc="#ecfdf5", ec="#047857", fs=9.7, weight="bold")
    add_box(ax, 0.85, 1.65, 3.25, 0.95, "mydog_control_sim_ros_node\n底层四足步态控制", fc="#f5f3ff", ec="#7c3aed", fs=9.7, weight="bold")

    # Localization and TF nodes.
    add_box(ax, 5.40, 8.45, 2.55, 0.92, "map_server\n/map", fc="#ecfdf5", ec="#047857", fs=10.0, weight="bold")
    add_box(ax, 5.35, 6.85, 2.85, 0.92, "AMCL\n/map + /scan + /odom\n广播 map→odom", fc="#eff6ff", ec="#2563eb", fs=9.2, weight="bold")
    add_box(
        ax,
        5.35,
        5.05,
        2.85,
        1.05,
        "mydog_state_estimator\n/gazebo/model_states\n发布 /odom 与 odom→base",
        fc="#eff6ff",
        ec="#2563eb",
        fs=8.9,
        weight="bold",
    )
    add_box(
        ax,
        5.35,
        3.35,
        2.85,
        1.05,
        "robot_state_publisher\nURDF + joint_states\n发布 base→laser/腿部",
        fc="#eff6ff",
        ec="#2563eb",
        fs=8.9,
        weight="bold",
    )
    add_box(
        ax,
        8.55,
        5.10,
        1.55,
        2.05,
        "TF 树\nmap→odom\nodom→base\nbase→laser\nbase→腿部",
        fc="#fef3c7",
        ec="#b45309",
        fs=9.2,
        weight="bold",
    )
    add_box(
        ax,
        8.58,
        2.85,
        1.48,
        0.92,
        "备用链路\nfake_scan\n/gt_pose",
        fc="#f8fafc",
        ec="#64748b",
        fs=8.3,
        dashed=True,
    )

    # Planning, visualization, and control nodes.
    add_box(
        ax,
        11.05,
        5.15,
        3.85,
        1.75,
        "dog_hybrid_navigator\n改进 A*：全局参考路径\n改进 DWA：实时局部执行轨迹\n订阅 /map /scan /odom /goal 与 TF",
        fc="#fff7ed",
        ec="#f97316",
        fs=9.1,
        weight="bold",
    )
    add_box(
        ax,
        14.95,
        8.35,
        2.28,
        1.00,
        "RViz\n目标点输入\nTF/Map/Scan/Path显示",
        fc="#f8fafc",
        ec="#475569",
        fs=9.0,
        weight="bold",
    )
    add_box(
        ax,
        11.05,
        1.55,
        2.95,
        1.22,
        "速度与步态命令话题\n/forward_back /left_right /yaw\n/start /walk /stand",
        fc="#f5f3ff",
        ec="#7c3aed",
        fs=8.6,
        weight="bold",
    )
    add_box(
        ax,
        14.55,
        1.55,
        2.70,
        1.22,
        "ASK-3 仿真执行\n四足规律步态\n状态反馈至 Gazebo",
        fc="#f5f3ff",
        ec="#7c3aed",
        fs=8.9,
        weight="bold",
    )

    # Simulation source edges.
    add_arrow(ax, (2.48, 8.45), (2.48, 7.77), label="雷达插件", color="#047857", fs=8.0, label_offset=(0.78, 0.0))
    add_arrow(ax, (4.10, 8.95), (5.35, 5.72), label="/gazebo/model_states", color="#047857", fs=7.8, rad=-0.16, label_offset=(0.35, 0.10))
    add_arrow(ax, (4.10, 7.31), (5.35, 7.31), label="/scan", color="#047857", fs=8.2)
    add_arrow(ax, (4.10, 5.71), (5.35, 3.88), label="/ask_3/joint_states", color="#047857", fs=7.8, rad=-0.12, label_offset=(0.10, 0.08))

    # Map, odom, and TF edges.
    add_arrow(ax, (6.68, 8.45), (6.72, 7.77), label="/map", color="#047857", fs=8.1)
    add_arrow(ax, (7.95, 8.88), (11.05, 6.58), label="/map", color="#047857", fs=8.0, rad=-0.10)
    add_arrow(ax, (7.95, 8.88), (14.95, 8.82), label="/map", color="#047857", fs=8.0, rad=0.05)
    add_arrow(ax, (8.20, 7.31), (8.55, 6.78), label="map→odom", color="#2563eb", fs=8.0)
    add_arrow(ax, (8.20, 5.58), (8.55, 6.10), label="odom→base", color="#2563eb", fs=8.0)
    add_arrow(ax, (8.20, 3.88), (8.55, 5.52), label="base→laser/legs", color="#2563eb", fs=7.8, rad=0.12)
    add_arrow(ax, (8.20, 5.38), (11.05, 5.58), label="/odom", color="#2563eb", fs=8.0)
    add_arrow(ax, (10.10, 6.12), (11.05, 6.08), label="TF lookup", color="#b45309", dashed=True, fs=8.0)
    add_arrow(ax, (10.10, 6.78), (14.95, 8.52), label="TF", color="#b45309", dashed=True, fs=8.0, rad=0.08)

    # Planning input/output edges.
    add_arrow(ax, (4.10, 7.02), (11.05, 6.28), label="/scan", color="#047857", fs=8.0, rad=-0.08)
    add_arrow(ax, (15.30, 8.35), (14.30, 6.90), label="/move_base_simple/goal", color="#475569", fs=7.7, rad=0.16)
    add_arrow(ax, (14.90, 6.24), (15.25, 8.35), label="/dog_global_path\n/dog_dwa_path\nMarker", color="#f97316", fs=7.8, rad=-0.18, label_offset=(0.50, 0.0))
    add_arrow(ax, (12.55, 5.15), (12.55, 2.77), label="三路机体速度\n非 /cmd_vel", color="#7c3aed", fs=8.0)
    add_arrow(ax, (14.00, 2.16), (14.55, 2.16), label="速度/步态", color="#7c3aed", fs=8.0)
    add_arrow(ax, (14.55, 1.92), (4.10, 2.12), label="驱动 Gazebo 中机器人运动", color="#7c3aed", fs=7.8, rad=0.10)
    add_arrow(ax, (16.05, 2.77), (2.25, 8.45), label="物理反馈：模型状态与关节状态", color="#64748b", fs=7.8, dashed=True, rad=0.27)

    # Optional chains.
    add_arrow(ax, (9.32, 3.77), (8.65, 6.88), label="use_amcl=false\nmap→odom", color="#64748b", fs=7.4, dashed=True, rad=0.10)
    add_arrow(ax, (9.32, 3.77), (11.05, 5.95), label="use_fake_scan=true\n备用 /scan", color="#64748b", fs=7.4, dashed=True, rad=-0.10)

    # Legend.
    add_box(
        ax,
        0.70,
        0.25,
        16.50,
        0.55,
        "图例：绿色为仿真/地图/传感输入，蓝色为定位与 TF，橙色为路径规划，紫色为四足运动控制，灰色虚线为可选备用链路或反馈链路。",
        fc="#ffffff",
        ec="#cbd5e1",
        fs=9.2,
    )

    fig.savefig(output_path, bbox_inches="tight", pad_inches=0.22)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output-dir",
        default="/tmp/dog_ros_diagrams",
        help="Directory where the generated PNG files are written.",
    )
    args = parser.parse_args()

    configure_fonts()
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    tf_path = output_dir / "四足机器人路径规划TF框架图.png"
    node_path = output_dir / "四足机器人路径规划ROS节点图.png"
    draw_tf_diagram(tf_path)
    draw_node_diagram(node_path)

    print(tf_path)
    print(node_path)


if __name__ == "__main__":
    main()
