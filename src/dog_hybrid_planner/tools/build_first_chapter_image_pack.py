#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Build an image pack for the thesis first chapter.

The pack contains verified open-license/public-domain images plus generated
schematics that fit the Chapter 1 narrative. It also writes a placement guide
with suggested insertion positions, captions, source URLs and license notes.
"""

from __future__ import annotations

import io
import json
import time
import textwrap
import urllib.request
import urllib.parse
import urllib.error
import zipfile
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager, rcParams
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch, Circle, Rectangle
from PIL import Image, ImageDraw


OUT_DIR = Path("/tmp/第一章配图素材包")
ZIP_PATH = Path("/tmp/第一章配图素材包.zip")


REMOTE_IMAGES = [
    {
        "file": "图1-1_ANYmal工业巡检四足机器人.jpg",
        "title": "File:ANYbotics robot dog ANYmal.jpg",
        "url": "https://commons.wikimedia.org/wiki/Special:Redirect/file/ANYbotics_robot_dog_ANYmal.jpg",
        "page": "https://commons.wikimedia.org/wiki/File:ANYbotics_robot_dog_ANYmal.jpg",
        "caption": "图1-1 四足机器人在工业巡检中的典型应用示意",
        "license": "CC BY-SA 4.0，作者/权利人：ANYbotics",
    },
    {
        "file": "图1-2_BigDog早期四足机器人平台.jpg",
        "title": "File:Big dog military robots.jpg",
        "url": "https://commons.wikimedia.org/wiki/Special:Redirect/file/Big_dog_military_robots.jpg",
        "page": "https://commons.wikimedia.org/wiki/File:Big_dog_military_robots.jpg",
        "caption": "图1-2 早期动态稳定四足机器人平台BigDog",
        "license": "Public domain，美国海军陆战队官方职务作品",
    },
    {
        "file": "图1-3_Spot四足机器人应用场景.jpg",
        "title": "File:Spot robot Royal Air Force.jpg",
        "url": "https://commons.wikimedia.org/wiki/Special:Redirect/file/Spot_robot_Royal_Air_Force.jpg",
        "page": "https://commons.wikimedia.org/wiki/File:Spot_robot_Royal_Air_Force.jpg",
        "caption": "图1-3 四足机器人在人机协同与场景验证中的应用",
        "license": "Public domain，美国空军官方职务作品",
    },
    {
        "file": "图1-4_Gazebo机器人仿真平台示意.png",
        "title": "File:Pioneer 3-AT in Gazebo.png",
        "url": "https://commons.wikimedia.org/wiki/Special:Redirect/file/Pioneer_3-AT_in_Gazebo.png",
        "page": "https://commons.wikimedia.org/wiki/File:Pioneer_3-AT_in_Gazebo.png",
        "caption": "图1-4 Gazebo机器人仿真平台示意",
        "license": "GPL自由软件截图，作者：Jiuguang Wang",
    },
]


def configure_fonts() -> None:
    preferred = [
        "Noto Sans CJK SC",
        "Noto Sans CJK JP",
        "Source Han Sans SC",
        "WenQuanYi Zen Hei",
        "SimHei",
        "Microsoft YaHei",
    ]
    available = {font.name for font in font_manager.fontManager.ttflist}
    for name in preferred:
        if name in available:
            rcParams["font.sans-serif"] = [name, "DejaVu Sans"]
            break
    else:
        rcParams["font.sans-serif"] = ["DejaVu Sans"]
    rcParams["axes.unicode_minus"] = False


def commons_file_url(title: str) -> str:
    params = urllib.parse.urlencode(
        {
            "action": "query",
            "format": "json",
            "titles": title,
            "prop": "imageinfo",
            "iiprop": "url|mime|size",
            "iiurlwidth": "1200",
        }
    )
    api_url = f"https://commons.wikimedia.org/w/api.php?{params}"
    req = urllib.request.Request(
        api_url,
        headers={"User-Agent": "Mozilla/5.0 first-chapter-thesis-image-pack"},
    )
    with urllib.request.urlopen(req, timeout=90) as resp:
        payload = json.loads(resp.read().decode("utf-8"))
    pages = payload["query"]["pages"]
    imageinfo = next(iter(pages.values()))["imageinfo"][0]
    return imageinfo.get("thumburl") or imageinfo["url"]


def download_and_resize(item: dict[str, str]) -> None:
    target = OUT_DIR / item["file"]
    if target.exists() and target.stat().st_size > 0:
        return
    url = commons_file_url(item["title"])
    time.sleep(0.8)
    req = urllib.request.Request(
        url,
        headers={
            "User-Agent": "first-chapter-thesis-image-pack/1.0 (local academic use)",
            "Referer": item["page"],
        },
    )
    for attempt in range(4):
        try:
            with urllib.request.urlopen(req, timeout=90) as resp:
                data = resp.read()
            break
        except urllib.error.HTTPError as exc:
            if exc.code != 429 or attempt == 3:
                raise
            time.sleep(4.0 * (attempt + 1))
    image = Image.open(io.BytesIO(data)).convert("RGB")
    max_width = 1800
    if image.width > max_width:
        new_height = int(image.height * max_width / image.width)
        image = image.resize((max_width, new_height), Image.LANCZOS)
    # Add a small white border so the image sits well in a thesis page.
    bordered = Image.new("RGB", (image.width + 28, image.height + 28), "white")
    bordered.paste(image, (14, 14))
    bordered.save(target, quality=92)


def box(ax, x, y, w, h, text, fc="#f8fafc", ec="#475569", fs=11, weight="normal"):
    p = FancyBboxPatch(
        (x, y),
        w,
        h,
        boxstyle="round,pad=0.03,rounding_size=0.06",
        linewidth=1.2,
        edgecolor=ec,
        facecolor=fc,
    )
    ax.add_patch(p)
    ax.text(x + w / 2, y + h / 2, text, ha="center", va="center", fontsize=fs, weight=weight, linespacing=1.25)


def arrow(ax, start, end, label=None, color="#334155", rad=0.0, dashed=False):
    patch = FancyArrowPatch(
        start,
        end,
        arrowstyle="-|>",
        mutation_scale=13,
        linewidth=1.5,
        color=color,
        linestyle="--" if dashed else "-",
        connectionstyle=f"arc3,rad={rad}",
    )
    ax.add_patch(patch)
    if label:
        ax.text((start[0] + end[0]) / 2, (start[1] + end[1]) / 2, label, fontsize=8.8, color=color,
                ha="center", va="center", bbox=dict(boxstyle="round,pad=0.15", fc="white", ec="none", alpha=0.88))


def generate_problem_schematic() -> None:
    fig, ax = plt.subplots(figsize=(10.8, 6.3), dpi=220)
    ax.set_xlim(0, 12)
    ax.set_ylim(0, 7)
    ax.axis("off")
    ax.text(6, 6.92, "四足机器人路径规划问题示意", ha="center", va="top", fontsize=17, weight="bold")
    ax.text(6, 6.40, "全局路径需要安全余量，局部规划需要兼顾头部朝前、必要侧移与墙角脱困", ha="center", fontsize=10.5, color="#475569")

    ax.add_patch(Rectangle((0.6, 0.6), 10.8, 5.15, fill=False, linewidth=2.0, edgecolor="#111827"))
    obstacles = [
        (2.0, 1.3, 0.45, 2.6),
        (3.9, 3.2, 2.2, 0.40),
        (7.0, 1.1, 0.55, 3.15),
        (8.6, 3.9, 1.85, 0.45),
        (9.1, 1.2, 0.50, 1.25),
    ]
    for x, y, w, h in obstacles:
        ax.add_patch(Rectangle((x, y), w, h, color="#111827"))
        ax.add_patch(Rectangle((x - 0.18, y - 0.18), w + 0.36, h + 0.36, fill=False, linestyle="--", linewidth=1.1, edgecolor="#f59e0b"))

    path_x = [1.1, 2.8, 3.6, 5.0, 6.4, 7.9, 9.6, 10.8]
    path_y = [1.0, 1.9, 2.6, 2.6, 4.35, 4.85, 5.05, 5.25]
    ax.plot(path_x, path_y, color="#ef4444", linewidth=3.0, label="A*全局参考路径")
    local_x = [5.0, 5.7, 6.35, 6.85, 7.25, 7.65, 8.1]
    local_y = [2.6, 3.05, 3.55, 3.88, 4.05, 4.18, 4.28]
    ax.plot(local_x, local_y, color="#2563eb", linewidth=3.0, linestyle="--", label="DWA局部实时轨迹")

    ax.add_patch(Circle((1.1, 1.0), 0.16, color="#16a34a"))
    ax.text(1.1, 0.68, "起点", ha="center", fontsize=9)
    ax.scatter([10.8], [5.25], marker="*", s=220, color="#dc2626")
    ax.text(10.8, 5.55, "目标点", ha="center", fontsize=9)

    body = FancyBboxPatch((4.55, 2.28), 0.78, 0.36, boxstyle="round,pad=0.04,rounding_size=0.06",
                          facecolor="#fde68a", edgecolor="#92400e", linewidth=1.4)
    ax.add_patch(body)
    ax.arrow(5.33, 2.46, 0.55, 0.0, width=0.015, head_width=0.13, head_length=0.15, color="#92400e")
    ax.text(5.18, 2.12, "头部朝前", fontsize=9, color="#92400e")

    ax.text(3.0, 4.1, "贴墙风险\n需安全膨胀", ha="center", fontsize=9.5, color="#b45309")
    ax.text(7.8, 3.15, "局部避障\n必要侧移", ha="center", fontsize=9.5, color="#1d4ed8")
    ax.text(8.8, 1.05, "墙角区域\n避免目标跳变", ha="center", fontsize=9.5, color="#7c2d12")
    ax.legend(loc="lower right", frameon=True, fontsize=9)
    fig.savefig(OUT_DIR / "图1-5_四足机器人路径规划问题示意.png", bbox_inches="tight", pad_inches=0.18)
    plt.close(fig)


def generate_technical_route() -> None:
    fig, ax = plt.subplots(figsize=(11.2, 6.2), dpi=220)
    ax.set_xlim(0, 12)
    ax.set_ylim(0, 7)
    ax.axis("off")
    ax.text(6, 6.92, "本文研究技术路线图", ha="center", va="top", fontsize=17, weight="bold")
    ax.text(6, 6.40, "从问题定位到算法改进，再到ROS/Gazebo仿真实验验证", ha="center", fontsize=10.5, color="#475569")

    box(ax, 0.7, 4.9, 2.05, 0.78, "研究背景\n四足机器人导航需求", fc="#ecfdf5", ec="#047857", weight="bold")
    box(ax, 3.25, 4.9, 2.05, 0.78, "问题分析\n贴墙/穿角/侧移滥用", fc="#fff7ed", ec="#f97316", weight="bold")
    box(ax, 5.85, 4.9, 2.05, 0.78, "A*全局规划改进\n安全余量+平滑", fc="#eff6ff", ec="#2563eb", weight="bold")
    box(ax, 8.5, 4.9, 2.05, 0.78, "DWA局部规划改进\n三维速度+头部对齐", fc="#eff6ff", ec="#2563eb", weight="bold")
    box(ax, 5.15, 3.15, 2.65, 0.82, "A*与DWA融合\n参考走廊+目标单调推进", fc="#fef3c7", ec="#b45309", weight="bold")
    box(ax, 8.55, 3.15, 2.30, 0.82, "执行稳定性\n速度保持+受限Recovery", fc="#f5f3ff", ec="#7c3aed", weight="bold")
    box(ax, 3.05, 1.42, 2.25, 0.82, "ROS/Gazebo平台\nASK-3模型+LiDAR", fc="#f8fafc", ec="#475569", weight="bold")
    box(ax, 6.20, 1.42, 2.10, 0.82, "消融实验\n对比各改进点", fc="#f8fafc", ec="#475569", weight="bold")
    box(ax, 9.05, 1.42, 2.10, 0.82, "结果分析\n效率/安全/姿态", fc="#f8fafc", ec="#475569", weight="bold")

    arrow(ax, (2.75, 5.29), (3.25, 5.29))
    arrow(ax, (5.30, 5.29), (5.85, 5.29))
    arrow(ax, (7.90, 5.29), (8.50, 5.29))
    arrow(ax, (6.90, 4.90), (6.60, 3.97), label="全局参考")
    arrow(ax, (9.50, 4.90), (9.65, 3.97), label="局部执行")
    arrow(ax, (7.80, 3.56), (8.55, 3.56))
    arrow(ax, (6.48, 3.15), (4.18, 2.24), rad=0.12)
    arrow(ax, (9.65, 3.15), (7.25, 2.24), rad=-0.12)
    arrow(ax, (5.30, 1.83), (6.20, 1.83))
    arrow(ax, (8.30, 1.83), (9.05, 1.83))

    ax.text(0.7, 0.55, "建议用于：1.3 论文主要内容及结构安排末尾，作为全文研究路线总览。", fontsize=9.5, color="#475569")
    fig.savefig(OUT_DIR / "图1-6_本文研究技术路线图.png", bbox_inches="tight", pad_inches=0.18)
    plt.close(fig)


def write_guides() -> None:
    placement = """# 第一章配图放置建议

| 建议图号 | 文件名 | 建议放置位置 | 建议图题 | 使用目的 |
|---|---|---|---|---|
| 图1-1 | 图1-1_ANYmal工业巡检四足机器人.jpg | 1.1 研究背景第1段之后，即“四足机器人凭借其优越的地形适应性...”段落后 | 四足机器人在工业巡检中的典型应用示意 | 用实物平台增强研究背景的直观性，说明四足机器人从实验室走向巡检、探测等应用场景。 |
| 图1-2 | 图1-2_BigDog早期四足机器人平台.jpg | 1.2 四足机器人国内外研究现状开头段之后 | 早期动态稳定四足机器人平台BigDog | 用于说明国外四足机器人平台的发展起点和动态稳定四足机器人的代表性平台。 |
| 图1-3 | 图1-3_Spot四足机器人应用场景.jpg | 1.2.5 文献综述与研究趋势中讨论产业应用、工程化落地的位置 | 四足机器人在人机协同与场景验证中的应用 | 用于说明四足机器人已从算法研究扩展到真实场景验证和人机协同应用。 |
| 图1-4 | 图1-4_Gazebo机器人仿真平台示意.png | 1.3 论文主要内容及结构安排第1段后，或介绍ROS/Gazebo仿真平台的位置 | Gazebo机器人仿真平台示意 | 引出本文基于ROS/Gazebo/RViz进行仿真实验的工程平台。 |
| 图1-5 | 图1-5_四足机器人路径规划问题示意.png | 1.2.6 现有研究的不足与本文研究定位之后 | 四足机器人路径规划问题示意 | 直接对应本文研究问题：贴墙风险、局部避障、头部朝前、必要侧移和墙角目标跳变。 |
| 图1-6 | 图1-6_本文研究技术路线图.png | 1.3 论文主要内容及结构安排末尾，进入第二章之前 | 本文研究技术路线图 | 总结第一章到后续算法章节之间的逻辑关系，使绪论更具总览性。 |

> 插入建议：第一章不宜一次性放入过多大图。若版面紧张，优先使用图1-1、图1-5、图1-6；若需要体现国内外研究现状和仿真平台，再加入图1-2、图1-3、图1-4。
"""
    sources = "# 图片来源与许可说明\n\n"
    for item in REMOTE_IMAGES:
        sources += (
            f"## {item['file']}\n"
            f"- 建议图题：{item['caption']}\n"
            f"- 来源页面：{item['page']}\n"
            f"- 下载地址：{item['url']}\n"
            f"- 许可说明：{item['license']}\n\n"
        )
    sources += (
        "## 图1-5_四足机器人路径规划问题示意.png\n"
        "- 来源：本次根据论文研究内容生成的原创示意图。\n"
        "- 可直接用于论文，图题建议：四足机器人路径规划问题示意。\n\n"
        "## 图1-6_本文研究技术路线图.png\n"
        "- 来源：本次根据论文结构和算法流程生成的原创示意图。\n"
        "- 可直接用于论文，图题建议：本文研究技术路线图。\n"
    )
    (OUT_DIR / "图片放置建议.md").write_text(placement, encoding="utf-8")
    (OUT_DIR / "图片来源与许可说明.md").write_text(sources, encoding="utf-8")


def make_zip() -> None:
    if ZIP_PATH.exists():
        ZIP_PATH.unlink()
    with zipfile.ZipFile(ZIP_PATH, "w", compression=zipfile.ZIP_DEFLATED) as zf:
        for path in sorted(OUT_DIR.rglob("*")):
            zf.write(path, arcname=path.relative_to(OUT_DIR.parent))


def main() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    configure_fonts()
    for item in REMOTE_IMAGES:
        download_and_resize(item)
    generate_problem_schematic()
    generate_technical_route()
    write_guides()
    make_zip()
    print(OUT_DIR)
    print(ZIP_PATH)


if __name__ == "__main__":
    main()
