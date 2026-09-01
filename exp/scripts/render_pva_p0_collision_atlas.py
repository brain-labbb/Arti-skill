#!/usr/bin/env python3
"""Replay and render the corrected-v2 PV-A P0 collision diagnostics.

The formal Table 4 state rows do not store link-pair identity.  This script
therefore regenerates each selected state from the frozen sampling plan,
checks its joint-vector hash, and replays that state with the v2 PyBullet pair
policy before invoking Blender.  Formal databases are opened read-only.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
import os
import sqlite3
import subprocess
import sys
import textwrap
import time
import xml.etree.ElementTree as ET
import zlib
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

from PIL import Image, ImageDraw, ImageFont, ImageStat


SCRIPT = Path(__file__).resolve()
REPO_ROOT = SCRIPT.parents[2]
ASSET_ROOT = Path("/mnt/zsn/data/particulate/datasets/PV-A/extracted")
ROSTER_DB = (
    REPO_ROOT
    / "exp/runtime/pva_table1234_full_release_20260826/evaluation/results.sqlite3"
)
PARENT_DB = (
    REPO_ROOT
    / "exp/runtime/pva_table4_mimic_aware_full_release_20260827/results.sqlite3"
)
OVERLAY_DB = (
    REPO_ROOT
    / "exp/runtime/pva_table4_v2_targeted_correction_20260828/overlay.sqlite3"
)
CORE_SOURCE = (
    REPO_ROOT
    / "exp/runtime/pva_table4_v2_targeted_correction_20260828/parent_source_snapshot"
    / "run_urdf_table4_partnet_mobility.py"
)
WORKER = REPO_ROOT / "exp/scripts/render_pva_p0_collision_asset_blender.py"
DEFAULT_BLENDER = Path(
    "/mnt/zsn/zsn_workspace/VoxHammer/third_party/blender-4.2.19-linux-x64/blender"
)
DEFAULT_OUTPUT = REPO_ROOT / "exp/runtime/pva_p0_blender_diagnostics_20260831"
FORMAL_PYTHON = Path("/tmp/arti_skill_table4_venv_20260827/bin/python")
DEFAULT_FONT_CANDIDATES = (
    Path("/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc"),
    Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"),
    Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"),
)
PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"


P0_CATEGORIES: tuple[tuple[str, str], ...] = (
    ("pictureX_0611_drawing_compass_with_adjustable_legs", "圆规"),
    ("rack_and_pinion_slider", "齿轮齿条滑台"),
    ("standing_desk_with_synchronous_telescoping_legs_and_articulated_controls", "同步升降桌"),
    ("watch", "手表"),
    ("Technology_Audio_Device", "音频设备"),
    ("clock_tower_with_rotating_hour_and_minute_hands", "钟楼"),
    ("pictureX_0611_garlic_press", "压蒜器"),
    ("Military_Rifle", "步枪"),
    ("pictureX_0611_bevel_gear_pair_with_perpendicular_shafts", "锥齿轮副"),
    ("sailboat_winch_with_pawl_and_handle", "帆船绞盘"),
    ("Bench_Wood_Swing", "木制秋千"),
    ("rolling_toolbox_with_telescoping_handle", "伸缩拉杆工具箱"),
    ("miter_saw_arm_assembly", "斜切锯臂"),
    ("stand_mixer", "厨师机"),
    ("Door_folding_door", "折叠门"),
    ("Urban_Environment_Tilt_Truck2", "倾斜式搬运车"),
    ("screwin_light_bulb_with_socket", "螺口灯泡与灯座"),
    ("Industrial_Mine_cart", "矿车"),
    ("pictureX_0611_ball_transfer_unit_with_spring_loaded_ball", "弹簧滚珠输送单元"),
    ("telescoping_boom", "伸缩臂"),
    ("Chair_Folding_chair", "折叠椅"),
    ("Astronomy_Space_shuttle", "航天飞机"),
    ("Industrial_Ore_crusher_jaw", "颚式破碎机"),
    ("pictureX_0611_drum_pedal_with_beater_and_spring_return", "鼓踏板"),
    ("Vehicle_Sports_car", "跑车"),
    ("pictureX_0611_gyroscope_with_spinning_wheel_and_gimbal_rings", "陀螺仪"),
    ("retractable_patio_awning", "伸缩遮阳篷"),
    ("globe", "地球仪"),
    ("metronome", "节拍器"),
    ("pictureX_0611_crimping_tool", "压线钳"),
    ("rivet_squeeze", "铆钉挤压钳"),
    ("wind_turbine", "风力机"),
    ("Container_Barrel", "带箍桶"),
    ("twojoint_prismatic_chain", "双滑动关节链"),
    ("louvered_shutter_assembly", "百叶窗组件"),
    ("turntable", "转台"),
)

CATEGORY_ZH = dict(P0_CATEGORIES)

LINK_ZH = {
    "leg_style__sheet_taper__needle_leg": "针尖腿",
    "leg_style__sheet_taper__lead_leg": "铅芯腿",
    "rack_carriage": "齿条滑架",
    "pinion": "小齿轮",
    "desktop": "桌面",
    "leg_0_outer": "0号外腿",
    "bezel": "表圈",
    "hour_hand": "时针",
    "minute_hand": "分针",
    "second_hand": "秒针",
    "bench": "座椅",
    "hanger_right": "右吊架",
    "front_leg_frame": "前腿框",
    "seat_pan": "座板",
    "frame": "机架/门框",
    "leaf_1": "第二扇门叶",
    "eccentric_shaft": "偏心轴",
    "swing_jaw": "动颚",
    "toggle_plate": "肘板",
    "cabinet": "柜体",
    "control_deck__rotary_knob_bank__knob_00": "0号旋钮",
    "hour_hand_0": "时针",
    "minute_hand_0": "分针",
    "lever_arm": "压柄",
    "pivot_pin": "枢轴销",
    "barrel": "枪管",
    "handguard": "护木",
    "horizontal_drive": "水平锥齿轮轴",
    "vertical_drive": "垂直锥齿轮轴",
    "crank": "曲柄",
    "drum": "绞盘鼓",
    "lid": "箱盖",
    "handle_inner": "内拉杆",
    "miter_table": "锯台",
    "blade": "锯片",
    "tool_attachment": "搅拌头",
    "mixing_bowl": "搅拌碗",
    "latch": "锁扣",
    "hopper": "料斗",
    "bulb_assembly": "灯泡组件",
    "thread_axis": "螺纹轴",
    "tub": "车斗",
    "latch_hook": "锁钩",
    "load_ball": "主滚珠",
    "outer_stage": "外套筒",
    "inner_stage": "内套筒",
    "left_rudder_panel": "左方向舵板",
    "right_rudder_panel": "右方向舵板",
    "frame_style__cast_post__frame": "踏板机架",
    "beater_style__fixed_head__angle_screw": "打击杆调节螺钉",
    "body": "车身",
    "wheel_front_left": "左前轮",
    "outer_gimbal_ring": "外万向环",
    "rotor": "转子",
    "head": "前横梁",
    "forearm_0": "0号前臂杆",
    "base": "底座",
    "globe": "球体",
    "housing": "外壳",
    "sliding_weight": "滑动配重",
    "moving_handle": "活动手柄",
    "drive_link": "驱动连杆",
    "head_module__stepped_cylinder_dies__upper_die": "上压模",
    "head_module__stepped_cylinder_dies__lower_die": "下压模",
    "lock_pin": "锁止销",
    "barrel_body": "桶体",
    "clasp_ring": "锁紧环",
    "end_effector": "末端滑块",
    "first_stage": "第一级滑块",
    "louver_slat_0_5": "第6片百叶片",
    "tilt_rod_0": "0号联动杆",
    "platter": "转盘",
    "tonearm": "唱臂",
}

SPECIAL_NOTES = {
    "pictureX_0611_drawing_compass_with_adjustable_legs": (
        "两腿真实间隙约 1 mm；v2 的两侧 Bullet margin 把间隙吃掉。"
        "同一 seed 在 numerical-zero-margin 的 v3 已通过，属于 v2 误伤。"
    ),
    "rack_and_pinion_slider": (
        "v2 未登记齿轮比和相位，齿条与小齿轮被当成两个自由度。"
        "v3 加入可信耦合并校准 margin 后，同一 seed 已通过。"
    ),
    "standing_desk_with_synchronous_telescoping_legs_and_articulated_controls": (
        "外腿伸入桌面安装腔 18 mm。它可能是装配所需接触，但当前 pair policy 只排除直接父子 link。"
    ),
    "watch": (
        "表圈与多层指针的轴向间隙小于默认 margin；v3 可消掉约 2 mm 数值膨胀，"
        "但仍有碰撞代理重叠，需要重新分层。"
    ),
    "Technology_Audio_Device": (
        "旋钮与机身是毫米级安装接触；同一 seed 在 v3 校准碰撞代理后已通过。"
    ),
    "clock_tower_with_rotating_hour_and_minute_hands": (
        "时针和分针共轴叠装，本应允许轴心局部重合；当前碰撞片在轴向做得过厚，"
        "把正常叠层判成 32 mm 穿透。"
    ),
    "pictureX_0611_garlic_press": (
        "压柄包住枢轴销是铰链装配关系；当前两者不是直接父子 pair，"
        "因此局部轴销接触被当作非法碰撞。"
    ),
    "Military_Rifle": (
        "护木包覆枪管属于嵌套装配；简化碰撞体没有挖出内腔，导致两者大面积重叠。"
    ),
    "pictureX_0611_bevel_gear_pair_with_perpendicular_shafts": (
        "锥齿轮齿面需要啮合接触；零接触 hard gate 和未登记的齿轮相位会共同惩罚该机构。"
    ),
    "sailboat_winch_with_pawl_and_handle": (
        "曲柄轴穿入绞盘鼓是传动装配；需要空心碰撞代理或局部轴孔接触规则。"
    ),
    "Bench_Wood_Swing": (
        "右吊架是闭环的第二侧连接点，在 URDF 树中不是直接父子；"
        "更像缺少局部 intended-contact 元数据，而不是错误运动。"
    ),
    "Door_folding_door": (
        "门框与第二扇门叶在静止位即大幅相交；v3 仍失败，应复核门叶安装原点和碰撞代理。"
    ),
    "miter_saw_arm_assembly": (
        "锯片穿过锯台开槽是正常工作结构；当前锯台碰撞体近似为实心，未表达刀槽。"
    ),
    "stand_mixer": (
        "搅拌头位于搅拌碗内部是正常嵌套，但当前碗的碰撞代理近似实心，"
        "因此产生大尺度体积重叠。"
    ),
    "screwin_light_bulb_with_socket": (
        "灯泡螺纹轴旋入灯座属于嵌套/啮合装配；实心代理无法表达螺纹内孔。"
    ),
    "pictureX_0611_ball_transfer_unit_with_spring_loaded_ball": (
        "主滚珠被壳体包持是设计要求；壳体需要带球窝的凹形代理，不能用实心凸体代替。"
    ),
    "Chair_Folding_chair": (
        "前腿框与座板在静止位相交；v3 仍失败，应区分铰接处 intended contact 与大面积穿透。"
    ),
    "Industrial_Ore_crusher_jaw": (
        "偏心轴与动颚出现大尺度重叠，另有机架与肘板接触；v3 仍失败，不能用 margin 解释。"
    ),
    "pictureX_0611_gyroscope_with_spinning_wheel_and_gimbal_rings": (
        "转子嵌在万向环内部；当前环体/转子的凸碰撞代理未保留内腔，造成大面积重叠。"
    ),
    "retractable_patio_awning": (
        "前臂杆插入前横梁的铰接安装区；需用局部轴孔代理或 intended-contact 元数据表达。"
    ),
    "globe": (
        "球体应嵌在底座支架的开口内；当前底座碰撞代理封住了内部空间，"
        "因此把正常包围关系报告成超大穿透。"
    ),
    "metronome": (
        "外壳与滑动配重在静止位已有明显重叠，运动后还会加深；需修碰撞代理或配重轨迹。"
    ),
    "Container_Barrel": (
        "桶体与锁紧环属于夹持装配区域；需要局部接触规则或更贴合的碰撞代理，不能放行整个 link。"
    ),
    "rivet_squeeze": (
        "上、下压模在闭合夹持姿态会接触；该 Sobol 状态应结合工具闭合语义和限位评估，"
        "不能只用全程零接触作为质量分数。"
    ),
    "wind_turbine": (
        "锁止销插入转子锁孔属于停机锁定结构；应表达孔洞或局部锁止接触。"
    ),
    "louvered_shutter_assembly": (
        "百叶片与联动杆的连接点属于多杆联动铰接；当前只排直接父子，"
        "跨树枝的连接接触会被判错。"
    ),
    "turntable": (
        "唱臂在多关节组合姿态碰到转盘；需复核唱臂联合限位和停靠区，而非仅看整机 hard pass。"
    ),
}

# README-facing explanations deliberately avoid evaluator terminology.  Each
# entry says what the two highlighted parts are doing and what should happen
# next; exact link names and replay measurements remain in the technical row.
PLAIN_ISSUES: dict[str, tuple[str, str]] = {
    "pictureX_0611_drawing_compass_with_adjustable_legs": (
        "针尖腿和铅芯腿实际上留有约 1 mm 缝隙，但评测把两条腿的简化外壳各向外加厚，结果把缝隙算成了 1 mm 重叠。",
        "这是评测误伤。同一物体在不额外加厚外壳的 v3 中已经通过，不需要修改圆规模型。",
    ),
    "rack_and_pinion_slider": (
        "图中齿条滑架和小齿轮的齿面已经重叠约 1.4 mm；运动时它们本应按固定比例一起走，旧评测却分别摆动，齿位还会错得更严重。",
        "应校准初始齿位，补上齿轮传动比例，并只采样一个主动关节；这样处理后同一物体已经通过。",
    ),
    "standing_desk_with_synchronous_telescoping_legs_and_articulated_controls": (
        "0 号外腿有 18 mm 伸进桌面的安装位置。这里看起来像桌腿插入桌面底部的固定槽，不是桌腿运动错了。",
        "应把桌面的安装槽做成空腔，或只允许这对部件在安装区域内接触，不能放行整条桌腿。",
    ),
    "watch": (
        "表圈压到了分针；其他指针之间也很近。评测外壳额外加厚解释了其中约 2 mm，但去掉加厚后仍有真实重叠。",
        "需要把表圈和三根指针按高度重新分层，并把每根指针的碰撞厚度做薄。",
    ),
    "Technology_Audio_Device": (
        "0 号旋钮与柜体只重叠约 0.5 mm，位置就在旋钮插入面板的安装处。",
        "这是毫米级评测误差；同一物体在 v3 已通过，不需要改可见模型。",
    ),
    "clock_tower_with_rotating_hour_and_minute_hands": (
        "时针和分针在中心轴位置叠在一起。两根指针本应一上一下安装，但简化外壳太厚，把中心叠装算成了 32 mm 穿透。",
        "应减薄两根指针的碰撞厚度并错开高度；中心轴附近也可单独定义为正常安装接触。",
    ),
    "pictureX_0611_garlic_press": (
        "压柄包住枢轴销约 14.5 mm，这正是压柄绕轴转动的铰链位置。",
        "这是正常的轴销连接。应只允许压柄与枢轴销在轴孔附近接触，而不是把整对部件全部放行。",
    ),
    "Military_Rifle": (
        "护木本来套在枪管外面，但护木的简化外壳没有中间的孔，所以把里面的枪管算成了 41.8 mm 重叠。",
        "应把护木改成带内孔的碰撞形状，保留枪管穿过的空间。",
    ),
    "pictureX_0611_bevel_gear_pair_with_perpendicular_shafts": (
        "水平和垂直锥齿轮的齿面正在啮合。齿轮工作时本来就要接触，齿位不同步时还会互相插得更深。",
        "应固定两只齿轮的传动比例和初始齿位，并允许齿面有很小的正常接触。",
    ),
    "sailboat_winch_with_pawl_and_handle": (
        "曲柄有 22.4 mm 插进绞盘鼓的中心，这里是曲柄轴安装到鼓上的传动位置。",
        "应在绞盘鼓中心保留轴孔，或只允许曲柄轴与轴孔局部接触。",
    ),
    "Bench_Wood_Swing": (
        "座椅与右吊架在第二个连接点重合约 50 mm。秋千需要左右两侧都吊住，但树形关节只把其中一侧记成直接连接。",
        "更像缺少第二侧闭环连接说明，不是座椅运动错误；应只登记右侧吊点这一小块正常连接区域。",
    ),
    "rolling_toolbox_with_telescoping_handle": (
        "箱盖与内拉杆重叠约 5.3 mm，位置在拉杆穿过箱盖或导向口的地方。",
        "应给箱盖留出拉杆通孔；若这里本来就是导向接触，只允许通孔边缘附近接触。",
    ),
    "miter_saw_arm_assembly": (
        "锯片穿过锯台约 37 mm。斜切锯本来就需要让锯片进入台面的刀槽，但简化锯台是实心的。",
        "应在锯台碰撞形状中挖出刀槽，不应把整块锯台与锯片的重叠都算成故障。",
    ),
    "stand_mixer": (
        "搅拌头伸进搅拌碗约 75.7 mm，这是厨师机的正常工作位置；问题是碗的简化外壳把内部也填实了。",
        "应把搅拌碗做成中空碰撞形状，并只检查搅拌头是否碰到碗壁和碗底。",
    ),
    "Door_folding_door": (
        "第二扇门叶在完全不动时就插进门框约 52 mm，不只是边缘轻微接触。",
        "这是需要修的装配问题：重点检查第二扇门叶的安装位置、转轴原点和门框碰撞形状。",
    ),
    "Urban_Environment_Tilt_Truck2": (
        "锁扣与料斗在扣合位置重叠约 0.53 mm，属于锁扣贴住料斗边缘的毫米级接触。",
        "先去掉评测外壳的额外加厚再判断；若仍接触，只允许锁扣扣合位置这一小块区域。",
    ),
    "screwin_light_bulb_with_socket": (
        "灯泡的螺纹轴有 9 mm 旋进灯座，这是正常安装；灯座的简化外壳没有螺纹孔，所以把插入部分算成重叠。",
        "应把灯座做成带内孔的形状，或专门允许螺纹啮合区域接触。",
    ),
    "Industrial_Mine_cart": (
        "锁钩伸进车斗边沿约 12 mm，位置像是锁钩扣住车斗的固定点。",
        "先确认锁钩是否设计为扣住车斗；若是，只允许钩口附近接触，否则应调整锁钩安装位置。",
    ),
    "pictureX_0611_ball_transfer_unit_with_spring_loaded_ball": (
        "主滚珠有 15.9 mm 嵌在外壳里，这是外壳托住滚珠、防止它掉出的正常结构。",
        "应把外壳做成带球窝的中空形状，只检查滚珠是否穿出球窝边界。",
    ),
    "telescoping_boom": (
        "内套筒在组合运动姿态下有 78.8 mm 位于外套筒内部。伸缩臂本来就要这样套叠，但外套筒被当成了实心。",
        "应把外套筒做成空心管，并检查内套筒是否越过管壁或超过伸缩限位。",
    ),
    "Chair_Folding_chair": (
        "前腿框在静止位插进座板约 32.6 mm，重叠范围明显大于一个小铰链接触点。",
        "需要检查座板和前腿框的安装位置；转轴附近可以局部接触，但大面积相交仍应修正。",
    ),
    "Astronomy_Space_shuttle": (
        "左右方向舵板在合拢的中缝处重叠约 2 mm，看起来是两块舵面贴得过紧。",
        "先去掉评测外壳的额外加厚；若仍重叠，应在中缝留出小间隙或减薄舵板碰撞形状。",
    ),
    "Industrial_Ore_crusher_jaw": (
        "偏心轴穿进动颚约 316 mm，另外机架和肘板也相交。这是大范围实体重叠，不是毫米级误差。",
        "需要重新检查偏心轴、动颚和肘板的安装位置及碰撞形状，不能用接触白名单掩盖。",
    ),
    "pictureX_0611_drum_pedal_with_beater_and_spring_return": (
        "打击杆调节螺钉有约 7 mm 穿进踏板机架，位置像螺钉穿过机架上的调节孔。",
        "若这是正常螺纹安装，应给机架留孔并只允许孔内接触；否则调整螺钉安装位置。",
    ),
    "Vehicle_Sports_car": (
        "左前轮被算进车身约 340 mm。可见模型有轮拱，但车身的简化外壳把轮拱封住了。",
        "应重做车身碰撞形状，明确挖出四个轮拱，不能把车身做成覆盖车轮的整块实体。",
    ),
    "pictureX_0611_gyroscope_with_spinning_wheel_and_gimbal_rings": (
        "转子位于外万向环内部，这是陀螺仪的正常结构；外环的简化外壳没有保留中间开口，因而重叠约 60.9 mm。",
        "应把万向环做成真正的环形碰撞形状，保留转子的旋转空间。",
    ),
    "retractable_patio_awning": (
        "0 号前臂杆插入前横梁的铰接安装座约 157.8 mm，位置是伸缩臂与横梁的连接处。",
        "应给前横梁保留安装槽和轴孔，并只允许铰接座内部的局部接触。",
    ),
    "globe": (
        "球体位于底座支架的开口内，但底座的简化外壳封住了开口，于是把半个球都算进底座，深度达到 514.9 mm。",
        "应把底座改成有开口的支架形状，保留球体旋转所需的内部空间。",
    ),
    "metronome": (
        "滑动配重在静止位已经伸进外壳约 15 mm，摆杆运动后还会更深。",
        "需要检查配重的滑动轨迹和外壳开口；配重只能沿摆杆移动，不能进入封闭外壳。",
    ),
    "pictureX_0611_crimping_tool": (
        "活动手柄与驱动连杆在转轴位置重叠约 3.1 mm，看起来是连杆装进手柄铰链的连接处。",
        "应把转轴孔做出来，并只允许铰链中心附近接触；其余位置仍需保持分离。",
    ),
    "rivet_squeeze": (
        "上压模与下压模在工具闭合姿态接触约 1 mm，这是夹紧铆钉时应有的闭合位置。",
        "应结合工具是否处于闭合终点来判断；闭合时允许压模端面接触，其他姿态仍不能互穿。",
    ),
    "wind_turbine": (
        "锁止销插进转子约 68.9 mm，这是风机停机时用销锁住转子的结构。",
        "应在转子上保留锁孔，并根据锁止销是否插入来判断；不能把锁定状态直接算成故障。",
    ),
    "Container_Barrel": (
        "锁紧环压住桶体约 4.1 mm，这是锁紧环夹住桶口或桶身的安装区域。",
        "应让锁紧环贴合桶体表面，并只允许夹持带这一圈发生小范围接触。",
    ),
    "twojoint_prismatic_chain": (
        "末端滑块在静止位顶进第一级滑块约 2.5 mm，位置在两级滑块的行程起点。",
        "需要检查零位偏移和机械止挡；若这是收到底的止挡接触，只允许端面接触，不能继续向里穿。",
    ),
    "louvered_shutter_assembly": (
        "第 6 片百叶片与 0 号联动杆在连接点重叠约 5 mm，这是联动杆带动叶片转动的铰接位置。",
        "应把这个跨支路连接登记为铰链，只允许销轴和孔附近接触。",
    ),
    "turntable": (
        "唱臂在多个关节一起运动的姿态下碰到转盘约 3.7 mm，说明唱臂高度、转动范围或停靠路线没有配合好。",
        "应联合限制唱臂的升降和旋转：落针时只让针尖接近唱片，移动和停靠时唱臂不能扫进转盘。",
    ),
}


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(8 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _canonical(value: Any) -> str:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True)


def _atomic_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary.write_text(
        json.dumps(value, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, path)


def _load_core() -> Any:
    path = CORE_SOURCE.resolve(strict=True)
    spec = importlib.util.spec_from_file_location("_pva_p0_frozen_v2_core", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import frozen core: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _selected_rows() -> list[dict[str, Any]]:
    parent_uri = f"file:{PARENT_DB.resolve(strict=True)}?mode=ro&immutable=1"
    connection = sqlite3.connect(parent_uri, uri=True)
    connection.execute("PRAGMA query_only=ON")
    connection.execute(
        "ATTACH DATABASE ? AS roster",
        (f"file:{ROSTER_DB.resolve(strict=True)}?mode=ro&immutable=1",),
    )
    connection.execute(
        "ATTACH DATABASE ? AS overlay",
        (f"file:{OVERLAY_DB.resolve(strict=True)}?mode=ro&immutable=1",),
    )
    placeholders = ",".join("?" for _ in P0_CATEGORIES)
    query = f"""
        WITH candidates AS (
          SELECT
            a.category,
            a.ordinal,
            a.asset_id,
            a.row_json,
            COALESCE(o.record_json, p.record_json) AS record_json,
            COALESCE(o.states_zlib, p.states_zlib) AS states_zlib,
            COALESCE(o.state_count, p.state_count) AS state_count,
            CASE WHEN o.parent_ordinal IS NULL THEN 'parent' ELSE 'overlay' END AS source,
            ROW_NUMBER() OVER (
              PARTITION BY a.category
              ORDER BY
                CASE WHEN a.asset_id LIKE '%/seed_0000' THEN 0 ELSE 1 END,
                a.ordinal
            ) AS category_rank
          FROM roster.assets AS a
          JOIN main.results AS p ON p.ordinal = a.ordinal
          LEFT JOIN overlay.results AS o ON o.parent_ordinal = a.ordinal
          WHERE a.category IN ({placeholders})
            AND COALESCE(
              json_extract(o.record_json, '$.measurement_complete'),
              json_extract(p.record_json, '$.measurement_complete'),
              0
            ) = 1
            AND COALESCE(
              json_extract(o.record_json, '$.strict_collision_pass'),
              json_extract(p.record_json, '$.strict_collision_pass'),
              0
            ) = 0
        )
        SELECT category, ordinal, asset_id, row_json, record_json, states_zlib,
               state_count, source
        FROM candidates
        WHERE category_rank = 1
        ORDER BY category
    """
    try:
        rows = connection.execute(
            query,
            tuple(category for category, _ in P0_CATEGORIES),
        ).fetchall()
    finally:
        connection.close()
    by_category = {str(row[0]): row for row in rows}
    missing = sorted(set(CATEGORY_ZH) - set(by_category))
    if missing:
        raise RuntimeError(f"no completed strict-fail representative for P0 categories: {missing}")
    selected: list[dict[str, Any]] = []
    for display_order, (category, label) in enumerate(P0_CATEGORIES, start=1):
        row = by_category[category]
        roster = json.loads(row[3])
        record = json.loads(row[4])
        states = [json.loads(line) for line in zlib.decompress(row[5]).splitlines()]
        if len(states) != int(row[6]):
            raise RuntimeError(f"state blob count mismatch for {row[2]}")
        seed_name = str(row[2]).rsplit("/", 1)[-1]
        asset_dir = (ASSET_ROOT / category / seed_name).resolve(strict=True)
        urdf = (asset_dir / "model.urdf").resolve(strict=True)
        expected_urdf_hash = str(roster["primary_urdf_sha256"])
        if _sha256(urdf) != expected_urdf_hash:
            raise RuntimeError(f"formal roster URDF hash mismatch: {urdf}")
        selected.append(
            {
                "display_order": display_order,
                "category": category,
                "category_zh": label,
                "ordinal": int(row[1]),
                "asset_id": str(row[2]),
                "seed": seed_name,
                "asset_dir": str(asset_dir),
                "urdf": str(urdf),
                "urdf_sha256": expected_urdf_hash,
                "effective_record_source": str(row[7]),
                "record": record,
                "states": states,
            }
        )
    return selected


def _schedule(core: Any, urdf: Path) -> tuple[list[dict[str, Any]], list[list[float]], list[dict[str, Any]]]:
    joints = core.parse_urdf_joints(urdf)
    plan = core.compile_joint_sampling_plan(joints)
    independent = list(plan["independent_joints"])
    descriptors: list[dict[str, Any]] = []
    values_by_state: list[list[float]] = []

    def add(
        phase: str,
        sample_index: int,
        joint_name: str | None,
        independent_values: Sequence[float],
    ) -> None:
        values = core.expand_joint_values(plan, [float(value) for value in independent_values])
        values = [float(value) for value in values]
        descriptors.append(
            {
                "phase": phase,
                "sample_index": sample_index,
                "joint_name": joint_name,
                "joint_values_sha256": core.canonical_sha256(values),
            }
        )
        values_by_state.append(values)

    add("rest", 0, None, [0.0] * len(independent))
    for position, joint in enumerate(independent):
        range_ok = bool(
            joint.get("sampling_range_evaluable", joint.get("range_evaluable"))
        )
        if not range_ok:
            continue
        for sample_index, value in enumerate(core.single_joint_values(joint)):
            vector = [0.0] * len(independent)
            vector[position] = float(value)
            add("single_joint_sweep", sample_index, str(joint["name"]), vector)
    if independent and all(
        bool(joint.get("sampling_range_evaluable", joint.get("range_evaluable")))
        for joint in independent
    ):
        for sample_index, vector in enumerate(
            core.sobol_joint_values(independent, seed=core.SOBOL_SEED)
        ):
            add("multi_joint_sobol", sample_index, None, vector)
    return descriptors, values_by_state, joints


def _visual_links(urdf: Path) -> set[str]:
    root = ET.parse(urdf).getroot()
    return {
        str(link.get("name"))
        for link in root.findall("link")
        if link.get("name") and link.find("visual") is not None
    }


def _state_candidates(states: Sequence[Mapping[str, Any]]) -> list[int]:
    failing = [
        index
        for index, state in enumerate(states)
        if int(state.get("non_adjacent_illegal_penetration_count", 0)) > 0
    ]
    rest = [index for index in failing if states[index].get("phase") == "rest"]
    motion = sorted(
        (index for index in failing if states[index].get("phase") != "rest"),
        key=lambda index: float(states[index].get("non_adjacent_max_penetration_m", 0.0)),
        reverse=True,
    )
    return [*rest, *motion]


def _replay_state(
    core: Any,
    urdf: Path,
    joints: Sequence[Mapping[str, Any]],
    values: Sequence[float],
    *,
    visual_links: set[str],
) -> dict[str, Any] | None:
    import pybullet as bullet

    client = bullet.connect(bullet.DIRECT)
    body: int | None = None
    try:
        flags = int(
            bullet.URDF_USE_INERTIA_FROM_FILE
            | bullet.URDF_USE_SELF_COLLISION
            | bullet.URDF_USE_SELF_COLLISION_INCLUDE_PARENT
            | bullet.URDF_IGNORE_VISUAL_SHAPES
        )
        body = bullet.loadURDF(
            str(urdf),
            useFixedBase=True,
            flags=flags,
            physicsClientId=client,
        )
        simulator_by_name: dict[str, int] = {}
        link_names = {
            -1: bullet.getBodyInfo(body, physicsClientId=client)[0].decode(
                "utf-8", "replace"
            )
        }
        for index in range(bullet.getNumJoints(body, physicsClientId=client)):
            info = bullet.getJointInfo(body, index, physicsClientId=client)
            joint_name = info[1].decode("utf-8", "replace")
            simulator_by_name[joint_name] = index
            link_names[index] = info[12].decode("utf-8", "replace")
        joint_indices = [simulator_by_name[str(joint["name"])] for joint in joints]
        for joint_index, value in zip(joint_indices, values, strict=True):
            bullet.resetJointState(
                body,
                joint_index,
                float(value),
                targetVelocity=0.0,
                physicsClientId=client,
            )
        bullet.performCollisionDetection(physicsClientId=client)
        direct = core._direct_parent_pairs(bullet, body, client)
        contacts: list[dict[str, Any]] = []
        for contact in bullet.getContactPoints(
            bodyA=body, bodyB=body, physicsClientId=client
        ):
            link_a_index = int(contact[3])
            link_b_index = int(contact[4])
            depth = max(0.0, -float(contact[8]))
            if (
                frozenset((link_a_index, link_b_index)) in direct
                or depth <= float(core.PENETRATION_THRESHOLD_M)
            ):
                continue
            link_a_name = str(link_names.get(link_a_index, link_a_index))
            link_b_name = str(link_names.get(link_b_index, link_b_index))
            if link_a_name not in visual_links or link_b_name not in visual_links:
                continue
            position_a = tuple(float(value) for value in contact[5])
            position_b = tuple(float(value) for value in contact[6])
            center = tuple((a + b) * 0.5 for a, b in zip(position_a, position_b))
            contacts.append(
                {
                    "link_a_index": link_a_index,
                    "link_b_index": link_b_index,
                    "link_a_name": link_a_name,
                    "link_b_name": link_b_name,
                    "penetration_depth_m": depth,
                    "position_on_a": position_a,
                    "position_on_b": position_b,
                    "contact_center": center,
                    "normal_on_b": tuple(float(value) for value in contact[7]),
                }
            )
        if not contacts:
            return None
        grouped: dict[frozenset[str], list[dict[str, Any]]] = {}
        for contact in contacts:
            key = frozenset((contact["link_a_name"], contact["link_b_name"]))
            grouped.setdefault(key, []).append(contact)
        pairs: list[dict[str, Any]] = []
        for rows in grouped.values():
            deepest = max(rows, key=lambda row: row["penetration_depth_m"])
            unique_points: list[tuple[float, float, float]] = []
            for row in sorted(rows, key=lambda item: item["penetration_depth_m"], reverse=True):
                point = tuple(row["contact_center"])
                if all(
                    math.dist(point, existing) > 1e-7
                    for existing in unique_points
                ):
                    unique_points.append(point)
            pairs.append(
                {
                    "link_a_name": deepest["link_a_name"],
                    "link_b_name": deepest["link_b_name"],
                    "penetration_depth_m": float(deepest["penetration_depth_m"]),
                    "contact_count": len(rows),
                    "contact_points": unique_points[:4],
                }
            )
        pairs.sort(key=lambda row: row["penetration_depth_m"], reverse=True)
        return {
            "collision_load_flags": flags,
            "illegal_contact_count": len(contacts),
            "primary_pair": pairs[0],
            "secondary_pairs": pairs[1:4],
            "replayed_max_penetration_m": max(
                contact["penetration_depth_m"] for contact in contacts
            ),
        }
    finally:
        if body is not None:
            bullet.removeBody(body, physicsClientId=client)
        bullet.disconnect(client)


def _phase_zh(phase: str) -> str:
    return {
        "rest": "静止位 q=0",
        "single_joint_sweep": "单关节扫描姿态",
        "multi_joint_sobol": "多关节 Sobol 姿态",
    }.get(phase, phase)


def _generic_note(category: str, phase: str, depth_m: float) -> str:
    if category in SPECIAL_NOTES:
        return SPECIAL_NOTES[category]
    if phase == "rest" and depth_m <= 0.0021:
        return (
            "静止位出现毫米级非父子接触；优先复核 Bullet margin、碰撞代理间隙，"
            "以及该接触是否为装配所需。"
        )
    if phase == "rest":
        return (
            "静止位即发生非父子穿透；需复核碰撞代理、装配原点，"
            "并判断是否属于局部功能接触。"
        )
    if phase == "single_joint_sweep":
        return "单关节扫动后出现碰撞；需复核关节限位、运动包络和驱动/从动关系。"
    return "多关节组合姿态出现碰撞；需复核联合限位、同步/闭环约束，不能默认各关节完全独立。"


def _diagnose_item(core: Any, item: dict[str, Any], pose_root: Path) -> dict[str, Any]:
    urdf = Path(item["urdf"])
    descriptors, values_by_state, joints = _schedule(core, urdf)
    source_states = item.pop("states")
    if len(descriptors) != len(source_states):
        raise RuntimeError(
            f"regenerated state count mismatch for {item['asset_id']}: "
            f"{len(descriptors)} != {len(source_states)}"
        )
    for index, (descriptor, state) in enumerate(zip(descriptors, source_states, strict=True)):
        for field in ("phase", "sample_index", "joint_name", "joint_values_sha256"):
            if descriptor[field] != state.get(field):
                raise RuntimeError(
                    f"formal state identity mismatch for {item['asset_id']} state {index}: {field}"
                )
    candidates = _state_candidates(source_states)
    if not candidates:
        raise RuntimeError(f"strict-fail representative has no illegal state: {item['asset_id']}")
    visual_links = _visual_links(urdf)
    replay = None
    selected_index = -1
    for candidate in candidates:
        replay = _replay_state(
            core,
            urdf,
            joints,
            values_by_state[candidate],
            visual_links=visual_links,
        )
        if replay is not None:
            selected_index = candidate
            break
    if replay is None:
        raise RuntimeError(f"no visual-link illegal pair reproduced for {item['asset_id']}")
    state = source_states[selected_index]
    joint_names = [str(joint["name"]) for joint in joints]
    joint_values = values_by_state[selected_index]
    if len(joint_names) != len(joint_values):
        raise RuntimeError("joint name/value vector mismatch")
    primary = replay["primary_pair"]
    depth_delta = abs(
        float(replay["replayed_max_penetration_m"])
        - float(state["non_adjacent_max_penetration_m"])
    )
    pose = {
        "schema_version": "pva_p0_blender_collision_pose_v1",
        "dataset_id": item["asset_id"],
        "category": item["category"],
        "ordinal": item["ordinal"],
        "urdf_sha256": item["urdf_sha256"],
        "protocol_id": item["record"]["protocol_id"],
        "effective_record_source": item["effective_record_source"],
        "phase": state["phase"],
        "sample_index": state["sample_index"],
        "joint_name": state.get("joint_name"),
        "joint_values_sha256": state["joint_values_sha256"],
        "joint_names": joint_names,
        "joint_values": joint_values,
        "source_non_adjacent_illegal_count": state["non_adjacent_illegal_penetration_count"],
        "source_non_adjacent_max_penetration_m": state["non_adjacent_max_penetration_m"],
        "replay_depth_abs_delta_m": depth_delta,
        **replay,
    }
    pose_path = pose_root / f"{item['display_order']:02d}_{item['category']}.json"
    _atomic_json(pose_path, pose)
    result = dict(item)
    result.pop("record", None)
    result.update(
        {
            "pose_json": str(pose_path),
            "phase": str(state["phase"]),
            "phase_zh": _phase_zh(str(state["phase"])),
            "sample_index": int(state["sample_index"]),
            "joint_name": state.get("joint_name"),
            "source_depth_m": float(state["non_adjacent_max_penetration_m"]),
            "replay_depth_m": float(primary["penetration_depth_m"]),
            "replay_depth_abs_delta_m": depth_delta,
            "link_a": str(primary["link_a_name"]),
            "link_b": str(primary["link_b_name"]),
            "link_a_zh": LINK_ZH.get(str(primary["link_a_name"]), "问题部件 A"),
            "link_b_zh": LINK_ZH.get(str(primary["link_b_name"]), "问题部件 B"),
            "note_zh": _generic_note(
                item["category"],
                str(state["phase"]),
                float(primary["penetration_depth_m"]),
            ),
            "state_identity_verified": True,
            "replay_contact_count": int(replay["illegal_contact_count"]),
            "secondary_pairs": replay["secondary_pairs"],
        }
    )
    return result


def _valid_png(path: Path, resolution: int | None = None) -> bool:
    try:
        if (
            not path.is_file()
            or path.stat().st_size <= len(PNG_SIGNATURE)
            or path.read_bytes()[: len(PNG_SIGNATURE)] != PNG_SIGNATURE
        ):
            return False
        with Image.open(path) as image:
            image.load()
            if resolution is not None and image.size != (resolution, resolution):
                return False
            extrema = image.convert("RGB").getextrema()
            return any(low != high for low, high in extrema)
    except (OSError, ValueError):
        return False


def _render_one(
    item: dict[str, Any],
    *,
    output: Path,
    blender: Path,
    resolution: int,
    samples: int,
    resume: bool,
) -> dict[str, Any]:
    stem = f"{item['display_order']:02d}_{item['category']}"
    paths = {
        "raw": output / "raw" / f"{stem}.png",
        "overview": output / "highlighted" / f"{stem}.png",
        "detail": output / "detail" / f"{stem}.png",
    }
    if resume and all(_valid_png(path, resolution) for path in paths.values()):
        return {**item, "render_status": "reused", "render_outputs": {key: str(value) for key, value in paths.items()}}
    for path in paths.values():
        path.parent.mkdir(parents=True, exist_ok=True)
    log_dir = output / "logs"
    log_dir.mkdir(parents=True, exist_ok=True)
    stdout_path = log_dir / f"{stem}.stdout.log"
    stderr_path = log_dir / f"{stem}.stderr.log"
    command = [
        str(blender),
        "-b",
        "--factory-startup",
        "-noaudio",
        "--python-exit-code",
        "1",
        "-P",
        str(WORKER),
        "--",
        "--asset-dir",
        item["asset_dir"],
        "--pose-json",
        item["pose_json"],
        "--raw-output",
        str(paths["raw"]),
        "--overview-output",
        str(paths["overview"]),
        "--detail-output",
        str(paths["detail"]),
        "--resolution",
        str(resolution),
        "--samples",
        str(samples),
    ]
    started = time.monotonic()
    env = dict(os.environ)
    env.update(
        {
            "BLENDER_USER_CONFIG": str(output / "blender_config" / stem),
            "CUDA_VISIBLE_DEVICES": str((int(item["display_order"]) - 1) % 8),
            "OMP_NUM_THREADS": "1",
            "OPENBLAS_NUM_THREADS": "1",
            "MKL_NUM_THREADS": "1",
        }
    )
    completed = subprocess.run(
        command,
        cwd=REPO_ROOT,
        env=env,
        capture_output=True,
        text=True,
        timeout=900,
        check=False,
    )
    stdout_path.write_text(completed.stdout, encoding="utf-8")
    stderr_path.write_text(completed.stderr, encoding="utf-8")
    status = "rendered"
    if completed.returncode != 0:
        status = "failed"
    elif not all(_valid_png(path, resolution) for path in paths.values()):
        status = "invalid_output"
    result = {
        **item,
        "render_status": status,
        "render_seconds": time.monotonic() - started,
        "returncode": completed.returncode,
        "command": command,
        "stdout_log": str(stdout_path),
        "stderr_log": str(stderr_path),
        "render_outputs": {key: str(value) for key, value in paths.items()},
    }
    if completed.stdout.strip():
        try:
            result["worker_result"] = json.loads(completed.stdout.strip().splitlines()[-1])
        except json.JSONDecodeError:
            result["worker_result_parse_error"] = True
    return result


def _find_font(explicit: Path | None) -> Path:
    candidates = (() if explicit is None else (explicit,)) + DEFAULT_FONT_CANDIDATES
    for candidate in candidates:
        if candidate.expanduser().is_file():
            return candidate.expanduser().resolve(strict=True)
    raise FileNotFoundError(
        "no Chinese font found; install/extract fonts-wqy-zenhei or pass --font"
    )


def _fit_lines(draw: ImageDraw.ImageDraw, text: str, font: Any, width: int) -> list[str]:
    lines: list[str] = []
    current = ""
    for character in text:
        candidate = current + character
        if current and draw.textlength(candidate, font=font) > width:
            lines.append(current.rstrip())
            current = character.lstrip()
        else:
            current = candidate
    if current:
        lines.append(current.rstrip())
    return lines


def _compose_card(item: dict[str, Any], output: Path, font_path: Path, resolution: int) -> Path:
    panel_paths = [
        Path(item["render_outputs"]["raw"]),
        Path(item["render_outputs"]["overview"]),
        Path(item["render_outputs"]["detail"]),
    ]
    images = [Image.open(path).convert("RGB") for path in panel_paths]
    panel = resolution
    top = 70
    label_height = 34
    footer = 270
    width = panel * 3
    height = top + panel + footer
    card = Image.new("RGB", (width, height), (244, 246, 248))
    draw = ImageDraw.Draw(card)
    title_font = ImageFont.truetype(str(font_path), 32)
    label_font = ImageFont.truetype(str(font_path), 22)
    body_font = ImageFont.truetype(str(font_path), 25)
    small_font = ImageFont.truetype(str(font_path), 20)

    title = f"P0-{item['display_order']:02d}  {item['category_zh']}  |  {item['seed']}  |  {item['phase_zh']}"
    draw.text((22, 16), title, font=title_font, fill=(20, 27, 37))
    labels = ("原始外观", "问题部件高亮", "局部放大（洋红点为接触位置）")
    for index, (image, label) in enumerate(zip(images, labels, strict=True)):
        image = image.resize((panel, panel), Image.Resampling.LANCZOS)
        x = index * panel
        card.paste(image, (x, top))
        draw.rectangle((x, top, x + panel, top + label_height), fill=(16, 22, 30))
        draw.text((x + 12, top + 4), label, font=label_font, fill=(250, 250, 250))
        if index:
            draw.line((x, top, x, top + panel), fill=(255, 255, 255), width=2)

    footer_y = top + panel
    draw.rectangle((0, footer_y, width, height), fill=(249, 250, 251))
    draw.rectangle((22, footer_y + 22, 42, footer_y + 42), fill=(209, 9, 21))
    draw.text(
        (52, footer_y + 17),
        f"红色：{item['link_a_zh']}",
        font=body_font,
        fill=(25, 29, 36),
    )
    second_x = width // 2
    draw.rectangle((second_x, footer_y + 22, second_x + 20, footer_y + 42), fill=(255, 79, 6))
    draw.text(
        (second_x + 30, footer_y + 17),
        f"橙色：{item['link_b_zh']}",
        font=body_font,
        fill=(25, 29, 36),
    )
    pair_text = f"link pair：{item['link_a']}  ↔  {item['link_b']}"
    pair_lines = _fit_lines(draw, pair_text, small_font, width - 44)
    for line_index, line in enumerate(pair_lines[:2]):
        draw.text(
            (22, footer_y + 56 + line_index * 25),
            line,
            font=small_font,
            fill=(76, 84, 95),
        )
    measurement = (
        f"本次独立重放深度：{item['replay_depth_m'] * 1000:.3f} mm；"
        f"formal v2 状态最大深度：{item['source_depth_m'] * 1000:.3f} mm。"
    )
    draw.text((22, footer_y + 108), measurement, font=body_font, fill=(25, 29, 36))
    note_lines = _fit_lines(draw, item["note_zh"], body_font, width - 44)
    for line_index, line in enumerate(note_lines[:3]):
        draw.text(
            (22, footer_y + 146 + line_index * 33),
            line,
            font=body_font,
            fill=(51, 60, 72),
        )
    draw.text(
        (22, height - 29),
        "判定口径：corrected v2，排除直接父子 link，非法穿透阈值 > 0.001 mm；图中深度采用本次可复现重放值。",
        font=small_font,
        fill=(95, 104, 116),
    )
    cards = output / "cards"
    cards.mkdir(parents=True, exist_ok=True)
    path = cards / f"{item['display_order']:02d}_{item['category']}.png"
    card.save(path, optimize=True)
    return path


def _write_sheets(items: Sequence[dict[str, Any]], output: Path) -> list[Path]:
    paths: list[Path] = []
    cards = [Image.open(item["card_path"]).convert("RGB") for item in items]
    chunk_size = 9
    thumbnail_width = 900
    margin = 18
    for chunk_index, start in enumerate(range(0, len(cards), chunk_size), start=1):
        chunk = cards[start : start + chunk_size]
        thumbnails = []
        for card in chunk:
            height = round(card.height * thumbnail_width / card.width)
            thumbnails.append(
                card.resize((thumbnail_width, height), Image.Resampling.LANCZOS)
            )
        rows = math.ceil(len(thumbnails) / 2)
        cell_height = max(image.height for image in thumbnails) + margin
        sheet = Image.new(
            "RGB",
            (thumbnail_width * 2 + margin * 3, rows * cell_height + margin),
            (225, 229, 234),
        )
        for index, image in enumerate(thumbnails):
            x = margin + (index % 2) * (thumbnail_width + margin)
            y = margin + (index // 2) * cell_height
            sheet.paste(image, (x, y))
        path = output / f"contact_sheet_{chunk_index:02d}.png"
        sheet.save(path, optimize=True)
        paths.append(path)
    return paths


def _write_index(items: Sequence[dict[str, Any]], sheets: Sequence[Path], output: Path) -> Path:
    lines = [
        "# PV-A P0 Blender 碰撞诊断图",
        "",
        "> 阅读顺序：先看“问题部件”和“发生了什么”，再看“怎么处理”。",
        "> 图中的红色、橙色是发生接触的两个部件，洋红点是评测找到的接触位置。",
        "> “简化外壳”是物理评测使用的不可见形状；它可能比画面里的真实零件更粗或缺少孔洞。",
        "> 每类选择 corrected v2 中最早完成但未通过的代表物体；精确技术信息保留在每项最后一行。",
        "",
        "## 总览",
        "",
    ]
    for sheet in sheets:
        lines.append(f"![P0 总览]({sheet.name})")
        lines.append("")
    lines.extend(["## 逐类", ""])
    for item in items:
        relative = Path(item["card_path"]).relative_to(output).as_posix()
        problem, action = PLAIN_ISSUES.get(
            item["category"],
            (
                f"{item['link_a_zh']}与{item['link_b_zh']}发生了评测不允许的重叠。",
                "需要检查两个部件的安装位置、活动范围和简化外壳。",
            ),
        )
        lines.extend(
            [
                f"### P0-{item['display_order']:02d} {item['category_zh']}",
                "",
                f"- **问题部件**：{item['link_a_zh']}（`{item['link_a']}`）和"
                f"{item['link_b_zh']}（`{item['link_b']}`）",
                f"- **发生了什么**：{problem}",
                f"- **怎么处理**：{action}",
                f"- **技术记录**：{item['phase_zh']}，重叠深度 "
                f"`{item['replay_depth_m'] * 1000:.3f} mm`；资产 "
                f"`{item['asset_id']}`，ordinal `{item['ordinal']}`，sample `{item['sample_index']}`",
                "",
                f"![{item['category_zh']}]({relative})",
                "",
            ]
        )
    path = output / "README.md"
    path.write_text("\n".join(lines), encoding="utf-8")
    return path


def _verify_images(items: Sequence[dict[str, Any]], sheets: Sequence[Path], resolution: int) -> dict[str, Any]:
    failures: list[str] = []
    for item in items:
        for key, value in item["render_outputs"].items():
            if not _valid_png(Path(value), resolution):
                failures.append(f"{item['category']}:{key}")
        card = Path(item["card_path"])
        if not _valid_png(card):
            failures.append(f"{item['category']}:card")
        else:
            with Image.open(card) as image:
                if sum(ImageStat.Stat(image.convert("RGB")).var) <= 1.0:
                    failures.append(f"{item['category']}:blank_card")
    for sheet in sheets:
        if not _valid_png(sheet):
            failures.append(f"sheet:{sheet.name}")
    return {
        "status": "PASS" if not failures else "FAIL",
        "expected_assets": len(P0_CATEGORIES),
        "rendered_assets": len(items),
        "sheet_count": len(sheets),
        "image_failures": failures,
        "all_state_identities_verified": all(
            item.get("state_identity_verified") is True for item in items
        ),
        "all_replays_have_contacts": all(
            int(item.get("replay_contact_count", 0)) > 0 for item in items
        ),
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--blender", type=Path, default=DEFAULT_BLENDER)
    parser.add_argument("--font", type=Path)
    parser.add_argument("--resolution", type=int, default=512)
    parser.add_argument("--samples", type=int, default=8)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--prepare-only", action="store_true")
    parser.add_argument("--limit", type=int)
    parser.add_argument("--resume", action="store_true")
    return parser


def _ensure_formal_python() -> None:
    """Re-exec under the frozen v2 environment when the shell Python differs."""

    if importlib.util.find_spec("pybullet") is not None:
        return
    if not FORMAL_PYTHON.is_file():
        raise RuntimeError(
            "formal v2 PyBullet environment is unavailable: "
            f"{FORMAL_PYTHON}"
        )
    os.execv(
        str(FORMAL_PYTHON),
        [str(FORMAL_PYTHON), str(SCRIPT), *sys.argv[1:]],
    )


def main(argv: Sequence[str] | None = None) -> int:
    _ensure_formal_python()
    args = build_parser().parse_args(argv)
    output = args.output.expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    if args.resolution < 256 or args.samples < 1 or args.workers < 1:
        raise ValueError("resolution, samples, and workers must be positive")
    core = _load_core()
    selected = _selected_rows()
    if args.limit is not None:
        if args.limit <= 0:
            raise ValueError("--limit must be positive")
        selected = selected[: args.limit]
    diagnostics = []
    for index, item in enumerate(selected, start=1):
        diagnostic = _diagnose_item(core, item, output / "poses")
        diagnostics.append(diagnostic)
        print(
            f"[replay {index}/{len(selected)}] {diagnostic['category']} "
            f"{diagnostic['phase']} {diagnostic['link_a']} <-> {diagnostic['link_b']} "
            f"{diagnostic['replay_depth_m'] * 1000:.3f} mm",
            flush=True,
        )
    preparation = {
        "schema_version": "pva_p0_blender_collision_atlas_preparation_v1",
        "created_at_utc": _utc_now(),
        "protocol_id": "urdf_sim_ready_table4_pva_full_release_v2_corrected_r1",
        "selection_policy": "P0 formal categories; completed strict-fail; prefer seed_0000; then lowest ordinal",
        "pair_policy": "exclude direct parent-child; penetration > 1e-6 m",
        "source_files": {
            "roster_database": str(ROSTER_DB),
            "parent_database": str(PARENT_DB),
            "overlay_database": str(OVERLAY_DB),
            "frozen_core": str(CORE_SOURCE),
            "frozen_core_sha256": _sha256(CORE_SOURCE),
        },
        "runtime": {
            "python_executable": str(Path(sys.executable).resolve()),
            "python_version": sys.version,
        },
        "items": diagnostics,
    }
    _atomic_json(output / "diagnostics.json", preparation)
    if args.prepare_only:
        print(json.dumps({"prepared": len(diagnostics), "output": str(output)}, ensure_ascii=False))
        return 0

    blender = args.blender.expanduser().resolve(strict=True)
    font = _find_font(args.font)
    render_results: list[dict[str, Any]] = []
    with ThreadPoolExecutor(max_workers=args.workers) as pool:
        futures = {
            pool.submit(
                _render_one,
                item,
                output=output,
                blender=blender,
                resolution=args.resolution,
                samples=args.samples,
                resume=args.resume,
            ): item
            for item in diagnostics
        }
        for index, future in enumerate(as_completed(futures), start=1):
            result = future.result()
            render_results.append(result)
            print(
                f"[render {index}/{len(diagnostics)}] {result['render_status']} "
                f"{result['category']}",
                flush=True,
            )
    render_results.sort(key=lambda item: int(item["display_order"]))
    failed = [item for item in render_results if item["render_status"] not in {"rendered", "reused"}]
    if failed:
        _atomic_json(output / "render_failures.json", failed)
        raise RuntimeError(
            f"{len(failed)} Blender renders failed; see {output / 'render_failures.json'}"
        )
    # A successful resume must not leave an earlier failed-attempt report behind.
    _atomic_json(output / "render_failures.json", [])
    for item in render_results:
        item["card_path"] = str(
            _compose_card(item, output, font, args.resolution)
        )
    sheets = _write_sheets(render_results, output)
    index_path = _write_index(render_results, sheets, output)
    verification = _verify_images(render_results, sheets, args.resolution)
    _atomic_json(output / "verification.json", verification)
    if verification["status"] != "PASS":
        raise RuntimeError(f"image verification failed: {verification['image_failures']}")
    manifest = {
        "schema_version": "pva_p0_blender_collision_atlas_v1",
        "created_at_utc": _utc_now(),
        "asset_count": len(render_results),
        "resolution": args.resolution,
        "samples": args.samples,
        "workers": args.workers,
        "blender": str(blender),
        "blender_version": subprocess.run(
            [str(blender), "--version"],
            check=True,
            capture_output=True,
            text=True,
        ).stdout.splitlines()[0],
        "font": str(font),
        "font_sha256": _sha256(font),
        "worker": str(WORKER),
        "worker_sha256": _sha256(WORKER),
        "driver": str(SCRIPT),
        "driver_sha256": _sha256(SCRIPT),
        "diagnostics": "diagnostics.json",
        "index": index_path.name,
        "contact_sheets": [path.name for path in sheets],
        "items": render_results,
        "verification": verification,
    }
    _atomic_json(output / "manifest.json", manifest)
    print(
        json.dumps(
            {
                "status": verification["status"],
                "assets": len(render_results),
                "sheets": len(sheets),
                "output": str(output),
            },
            ensure_ascii=False,
        ),
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
