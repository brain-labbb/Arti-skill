# Music / Headphone — template source map

pattern: mixed
parents: rec_over-ear-headphones-with-padded-leather-ear-cups_20260609_080248_682174_5669f0bd ← picture/Music/Headphone/001.png

> 父对象骨架（所有变体共享）：root `band` →（每侧并行对称）`*_slider` PRISMATIC →
> `*_yoke` FIXED → `*_cup` REVOLUTE（耳倾仰）。helper：`_band_arc_path` / `_band_shell_mesh` /
> `_crown_pad_mesh` / `_housing_mesh` / `_slider_mesh` / `_yoke_mesh` / `_cup_shell_mesh` /
> `_cup_inner_cap_mesh` / `_ear_pad_mesh`。每侧 slider 在 `band` 的 `housing_l/r` 套筒内伸缩。
> 父基线 = 圆鼓形 cup + 封闭实心球冠后盖 + 单条实心弧带 + 双叉 gimbal clevis yoke。

## Slot 候选覆盖

### Slot A:cup_shape（耳罩外形）
| 候选(future module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| round_drum (baseline) | rec_over-ear-headphones-with-padded-leather-ear-cups_20260609_080248_682174_5669f0bd | `_cup_shell_mesh`(CylinderGeometry drum + SphereGeometry dome); part `left_cup`/`right_cup`; joint `left_yoke_to_left_cup` REVOLUTE; visuals `{cp}_shell`/`{cp}_cap`/`{cp}_pad` | 圆鼓形：CUP_R 圆柱鼓 + 浅球冠 dome；圆环 TorusGeometry ear-pad | baseline |
| vertical_oval | rec_over_ear_headphones_var_cup_oval | `_cup_shell_mesh`(superellipse_profile + ExtrudeGeometry, CUP_RX<CUP_RZ); `_cup_inner_cap_mesh`/`_ear_pad_mesh` 椭圆化; visuals `{cp}_shell`/`{cp}_cap`/`{cp}_pad` | 竖椭圆鼓（高>宽，dz>dx·1.15）；dome 与 pad 椭圆缩放贴合 | converged(已同步) |
| rounded_rect | rec_over_ear_headphones_var_cup_rounded_rect | 新 helper `_rounded_rect_cup_body()`；`_cup_shell_mesh`/`_cup_inner_cap_mesh`/`_ear_pad_mesh`; visuals `{cp}_shell`/`{cp}_cap`/`{cp}_pad` | 圆角矩形耳罩本体（rounded_rect 轮廓拉伸） | converged(已同步) |

### Slot B:cup_back_style（耳罩后盖样式）
| 候选(future module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| closed_solid_dome (baseline) | rec_over-ear-headphones-with-padded-leather-ear-cups_20260609_080248_682174_5669f0bd | `_cup_shell_mesh` 内 SphereGeometry dome(scale y 0.28); visual `{cp}_shell` | 封闭实心球冠后盖（无开孔） | baseline |
| open_back_vented_grille | rec_over_ear_headphones_var_back_vented_grille | `_vented_back_plate_mesh`(ExtrudeWithHolesGeometry：中心孔 + VENT_COUNT 孔环) + `_vent_rim_mesh`; visuals `{cp}_grille`、`{cp}_vent_i`; drum-only `_cup_shell_mesh`(去 dome); model 名 `open_back_headphones` | 平板钻孔开放后盖 + 一圈通风孔 + 每孔 torus 孔缘 | converged(已同步) |
| exposed_driver_behind_ring | rec_over_ear_headphones_var_back_exposed_driver | helper `_back_ring_mesh`/`_grille_bars_mesh`/`_driver_cone_mesh`/`_dust_cap_mesh`; visuals `{cp}_ring`/`{cp}_bars`/`{cp}_driver`/`{cp}_dustcap`/`{cp}_innercap` | 后环 + 十字栅条 + 露出的驱动单元锥 + 中心防尘帽 | converged(已同步) |

### Slot C:headband_form（头梁形态）
| 候选(future module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| single_solid_arch (baseline) | rec_over-ear-headphones-with-padded-leather-ear-cups_20260609_080248_682174_5669f0bd | `_band_shell_mesh`(sweep_profile_along_spline + rounded_rect_profile); root part `band`; visuals `band_shell` + `band_pad` | 单条实心弧带 + 皮质 crown pad | baseline |
| twin_parallel_rods | rec_over_ear_headphones_var_band_twin_rods | helper `_rod_arc_path`/`_band_rod_mesh`/`_rod_clip_mesh`; visuals `band_rod_0`/`band_rod_1`(ROD_SPACING 分离) + `pad_clip_0`/`pad_clip_1`; polished_steel 材质; HOUSING_W 加宽 | 双平行钢丝弧杆（X 分离），夹片把 crown pad 连到两杆 | converged(已同步) |
| suspension_strap_under_arch | rec_over_ear_headphones_var_band_suspension_strap | root 改名 `arch`(`_arch_mesh`→`arch_frame`); 新增 part `suspension_strap`(`_strap_mesh`→`strap_body`); 新 joint `arch_to_strap` PRISMATIC(STRAP_DROOP 下垂); `strap_clip_0/1`; 每侧 joint 改 `arch_to_*_slider` | 薄金属外弧 + 下方悬吊皮带（可下垂 prismatic）；root 部件更名 | converged(已同步) |

### Slot D:cup_attachment（耳罩↔头梁连接 — 真正承载运动学的 slot ⚠）
| 候选(future module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| two_tine_gimbal_clevis (baseline) | rec_over-ear-headphones-with-padded-leather-ear-cups_20260609_080248_682174_5669f0bd | `_yoke_mesh`(ClevisBracketGeometry); part `left_yoke`; joints `left_slider_to_left_yoke` FIXED + `left_yoke_to_left_cup` REVOLUTE | 双叉 clevis yoke FIXED 于 slider；cup 在 yoke bore 上 REVOLUTE 仅做耳倾仰 | baseline |
| single_pivot_hanger | rec_over_ear_headphones_var_attach_single_pivot | **无 yoke part**；`_hanger_arm_mesh`(细吊臂+横 pivot post) inline 为 slider visual `{sl}_arm`; joint `slider_i_to_cup_i` REVOLUTE（slider→cup 直连）; 部件改名 `slider_i`/`cup_i` | 取消 clevis，单侧 pivot post 吊住 cup；cup REVOLUTE 直接挂在 slider 臂上 | converged(已同步) |
| collapsible_folding_hinge | rec_over_ear_headphones_var_attach_folding_hinge | 保留 yoke + `_hinge_barrel_mesh`→`{yk}_hinge`; **`left_slider_to_left_yoke` 由 FIXED 改 REVOLUTE 折叠**(FOLD_UPPER≈1.50rad/86°, fold_axis=∓side·X) + `left_yoke_to_left_cup` REVOLUTE 倾仰 | 折叠铰链：slider↔yoke 接口变可动、内折向头梁；每侧两个 revolute（折叠 + 倾仰） | converged(已同步) |

> ⚠ **运动学集中在本 slot**：基线里 slider↔yoke 是 FIXED，唯一活动是各值共享的 yoke↔cup 耳倾仰 REVOLUTE。
> 两个 attach 候选都在 cup↔band 连接处引入**非 fixed 关节**——`single_pivot` 用 `slider_i_to_cup_i` REVOLUTE
> 取代「FIXED+REVOLUTE」对（删 yoke）；`folding_hinge` 把 `slider→yoke` 升级为折叠 REVOLUTE。
> 故 cup_attachment 是承载真实关节差异的 slot，模板里该 slot 决定 articulation 拓扑，不能仅当作几何换皮。

## Multiplicity / Copy Logic
- count_param: vent_count（源码 `VENTS_PER_CUP`，每个 cup 后盖的均布通风条数）
- N 样本已覆盖: {8} → rec_over_ear_headphones_var_grille_vent_count（VENTS_PER_CUP=8，且测试显式断言 ≠12）。注：**父对象封闭实心 dome 无通风条（N=0）**；`back_vented_grille` 另用 VENT_COUNT=8 的钻孔环（不同实现，亦可作 N 来源）。
- 模板建议 N_range: [4, 16]（8 为已验证样本，避开测试排除的 12）
- copied object: 单根 `_vent_slot_mesh` 径向肋（BoxGeometry），外加固定的 `_grille_ring_mesh`(`{cp}_grille_ring`) + `_grille_hub_mesh`(`{cp}_grille_hub`) 框架
- naming: `for i in range(VENTS_PER_CUP)` → visual 名 `{cp}_vent_{i}`（每 cup 独立计数，left/right 各一份）
- placement: 绕 cup 轴 `rotate_y(2π·i/N)` 均布后 `translate(0, side·CUP_HD, −CUP_R)` 贴到后盖中心；left/right 两 cup 各复制一份
- joint policy: vent 肋是 cup part 上的**刚性 visual（无关节）**，纯外观复制，不新增 articulation；复制数变化只影响 cup 视觉，不动骨架

## 排除项(future compatibility matrix 素材)
- cup_back_style 三值互斥：`closed_solid_dome` / `open_back_vented_grille` / `exposed_driver_behind_ring` 同一 cup 只能取一种后盖。
- vent_count multiplicity 仅在开放后盖（vented_grille / grille 类）下有意义；基线 `closed_solid_dome` 对应 N=0 → **vent_count 与 closed_solid_dome 不兼容**（封闭盖无可复制的通风条）。
- cup_attachment 内部互斥：`single_pivot_hanger` 整体删除 yoke 部件，而 `collapsible_folding_hinge` 依赖 clevis yoke 作折叠铰链 → 二者只能取其一（同 slot 自然单选）；模板需把该 slot 当 articulation 拓扑开关而非纯几何。
- headband_form 内部互斥：`suspension_strap_under_arch` 把 root 部件 `band` 改名为 `arch` 并新增 part+PRISMATIC 关节，与 `single_solid_arch` / `twin_parallel_rods` 同 slot 互斥；模板需统一 root 命名以兼容三值。
- 各轴**值本身均已收敛**（无未通过 sweep 的轴值），上述仅为「同一 slot 单选」与「跨 slot 语义依赖」型不兼容，供 compatibility matrix 取材。
