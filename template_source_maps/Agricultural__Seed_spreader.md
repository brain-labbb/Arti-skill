# Source Map — Agricultural / Seed spreader

slug `spreader` · pattern **mixed**（parallel_children: hopper/frame + wheels + spinner + gate +
lever；短 lever→gate 计量链；vane/wheel loop multiplicity）。双 origin 占满 chassis 两端。

## Origins（全量对账，2/2 上格）
| id | pic | 建成形态 | 网格角色 |
|---|---|---|---|
| A `rec_use-the-attached-reference-image-as-the-primary-_20260625_155130_513620_b14fd48a` | 001 | 推行式:方锥 `hopper_shell` + T 把 + 2 轮 + 前脚轮 + 旋盘 `spinner`；wheels/spinner continuous, lever revolute, gate prismatic | hopper=square_taper / chassis=push_Thandle / mech=spinner |
| B `rec_use-the-attached-reference-image-as-the-primary-_20260626_034216_595689_91108848` | 002 | 牵引式:圆角方 `hopper` + 红钢架 + 牵引杆 `hitch` + 2 turf 轮 + 3 叶 `low_radial_spreader_vane_{i}` | hopper=rounded_square / chassis=tow_drawbar / mech=spinner |

## Slots
- **A hopper_form（③）**：square_taper(A) / rounded_square(B) / round_conical(fork@A) — 可外推 hex/octagon；drop-box 并入 C
- **B chassis/propulsion（① skeleton）**：push_Thandle(A) / tow_drawbar(B) — **双 origin 满，0 fork**
- **C mechanism（② joint）**：rotary_broadcast_spinner(A,B) / drop_bar(fork@A)
- **N**：spinner vanes ×N {3(B),4(A),6}；ground wheels ×N {2,3(caster)}；drop-hole ×N(在 dropbar)

## Slot 候选覆盖
### Slot A：hopper_form
| square_taper(origin) | A | converged |
| rounded_square(origin) | B | converged |
| round_conical | rec_spreader_var_hopper_conical | converged |
### Slot B：chassis/propulsion
| push_Thandle(origin) | A | converged |
| tow_drawbar(origin) | B | converged |
### Slot C：mechanism
| rotary_broadcast_spinner(origin) | A,B | converged |
| drop_bar(+`agitator` continuous, `drop_hole_{i}`) | rec_spreader_var_mech_dropbar | converged |
### 附加（② push handle 机构）
| folding push handle(revolute fold) | rec_spreader_var_handle_fold | converged |

## Multiplicity / Copy Logic
- vanes: count_param `n_vanes` — `low_radial_spreader_vane_{i}` 等角，FIXED 于 hub，共用 `hopper_to_spinner` continuous；N {3,4,6} → B / A / rec_spreader_var_spinner_vanes6
- wheels: count 2→3(加前 `caster_wheel` 独立 continuous) → rec_spreader_var_frame_caster；drive wheels 保持 `wheel_{i}` loop
- drop-hole `drop_hole_{i}` N∈{6,8,10}(dropbar 内)；模板 N_range vanes [3,10]、drop-hole 宽

## 视觉多样性 6 轴考察
| 轴 | 处理 | 取值 |
|---|---|---|
| ① 骨架图 | forked_anchor | hopper 3 + chassis 2 + mech 2；wheels N{2,3}、vanes N{3,4,6} |
| ② 关节类型 | forked_anchor | wheels/spinner/agitator/caster continuous、lever revolute、gate prismatic、folding handle revolute |
| ③ 主体形态家族 | forked_anchor + world_knowledge_extrapolation | hopper envelope；可外推 hex/octagon 面 |
| ④ 表面装饰 | record_only + world_knowledge_extrapolation | 品牌 decal、ribs/rims、rivets；可外推 fabric hopper cover、rate dial |
| ⑤ 尺寸/行程 | record_only | hopper W:H、tire Ø；gate prismatic 0→0.13、lever revolute |
| ⑥ 涂装 | record_only | black poly + red|green|black|yellow frame / galvanized 银轮 |

## 排除项
- handheld hand-crank shoulder-bag spreader — 无轮底盘、近类目边界，不可从任一 origin fork
- big-ag pendulum/oscillating-spout broadcaster — 与园艺 spreader 类目漂移
