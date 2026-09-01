# Parts / quick release clamp — template source map

pattern: parallel_children (3 structural slots: collar / actuation / nut; NO multiplicity axis)

slug: quick_release_clamp

parents:
- rec_model-a-bicycle-style-quick-release-seat-clamp-i_20260610_085231_449555_1b80e476 ← picture/Parts/quick release clamp/001.png(bicycle-style QR seat clamp;Omega open split-ring collar + cam-over-center side lever + knurled barrel adjuster nut;**fork 基线**)→ baseline 覆盖 Slot A=omega_split_ring, Slot B=cam_over_center_lever, Slot C=knurled_barrel_nut

三个 named slot 全部挂在同一刚性 `collar` 上;真正的运动学是 1 条 REVOLUTE(cam lever)+ 1 条 CONTINUOUS(adjuster nut)。6 个变体均 fork 自 1b80e476,每个只替换其中一个 slot。

## Slot 候选覆盖

### Slot A:collar(夹环主体——抱住 seatpost bore 的开口环)
| 候选(未来 module) | record_id | 关键 part/joint/helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| omega_split_ring(基线) | rec_...1b80e476 (parent) | `collar` part;`_collar_band_solid` / `_throat_notch_solid`;`lug_cap_side` / `lug_nut_side`(`_lug_solid`) | 单刚体 Omega 开口环,-X 楔形 throat 把 bore 豁开到两脚之间;无环上铰接 | converged(parent) |
| pinch_collar | rec_quick_release_clamp_var_pinch_collar | `collar` part;`_collar_band_solid` + `_pinch_slit_solid`(`PINCH_SLIT_HALF_W`);`lug_cap_side` / `lug_nut_side` | 整圆环,只在 -X 壁开一条细窄锯缝(saw slit),bore 保持连续圆通孔(非 Omega);仍是单刚体 | converged ✓ |
| hinged_collar | rec_quick_release_clamp_var_hinged_collar | 拆成 `cam_arc` / `nut_arc` 两 part;`_collar_arc_solid` / `_arc_profile_points` / `_arc_band_solid`;`cam_hinge_leaf`/`nut_hinge_leaf`(`_hinge_leaf_solid`);`cam_hinge_knuckle_{i}`/`nut_hinge_knuckle`(`_vertical_tube_solid`);`hinge_pin`(`_hinge_pin_solid`);**新增 joint `barrel_hinge`** REVOLUTE z(0→62°) | 两片半弧表带式合页,+X 可见 barrel hinge 把 nut 半弧铰到 cam 半弧;-X lugs 仍由 lever bolt 夹合 | converged ✓(注:此候选引入一条额外 REVOLUTE) |

### Slot B:actuation(+Y 端的快拆动作机构)
| 候选(未来 module) | record_id | 关键 part/joint/helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| cam_over_center_lever(基线) | rec_...1b80e476 (parent) | `cam_lever` part;`lever_handle`(`_lever_handle_solid`);`fixed_cam_barrel`(Cylinder);joint `lever_cam_pivot` REVOLUTE z(0→170°) | 实体侧扳手绕竖直 Z 轴 pivot 摆开,外置短 cam barrel 为固定 boss | converged(parent) |
| fold_flat_lever | rec_quick_release_clamp_var_fold_lever | `folding_lever` part;`flat_lever`(`_folding_lever_solid`,带 bored eye);固定件 `bolt_head` / `fork_cheek_0`/`fork_cheek_1`(`_fork_cheek_solid`)/ `hinge_pin` / `pin_head_{i}`;joint `lever_hinge` REVOLUTE **x**(0→95°) | 扁平折叠扳手绕 X 轴 hinge pin 向上翻起;闭合时沿 bolt 轴贴平,clevis 双颊夹销 | converged ✓ |
| recessed_hex_bolt | rec_quick_release_clamp_var_hex_bolt | `hex_key` part;固定件 `socket_head_bolt`(`_socket_head_solid`)/ `hex_socket_recess`(`_hex_socket_floor_solid`);动件 `hex_drive_bit`(`_hex_drive_bit_solid`)/ `hinge_knuckle`(`_hex_key_hinge_solid`)/ `folding_hex_arm`(`_folding_hex_key_arm_solid`);joint `hex_key_hinge` REVOLUTE **-x**(0→92°) | 沉孔 socket-head 螺栓 + 可收纳折叠 Allen-key:六角 bit 收进 socket,长六角臂绕 socket 口翻出 | converged ✓ |

### Slot C:nut(-Y 端的调节螺母)
| 候选(未来 module) | record_id | 关键 part/joint/helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| knurled_barrel_nut(基线) | rec_...1b80e476 (parent) | `adjuster_nut` part;`knurled_nut`(`KnobGeometry`,grip=knurled count=40);joint `adjuster_nut_spin` CONTINUOUS y | 滚花圆筒桶状调节螺母,绕 bolt 轴连续旋转 | converged(parent) |
| winged_thumb_nut | rec_quick_release_clamp_var_wing_nut | `adjuster_nut` part;`thumb_nut_hub`(`_thumb_nut_hub_solid`)+ `wing_0`/`wing_1`(`_thumb_wing_solid`);joint 沿用 `adjuster_nut_spin` CONTINUOUS y | 中心镗孔 hub + 两片对置扁平径向指翼(蝶形手拧螺母) | converged ✓ |
| domed_acorn_nut | rec_quick_release_clamp_var_dome_nut | `adjuster_nut` part;`acorn_cap_nut`(`_acorn_cap_nut_solid`:六角基 + 圆肩 + 旋转半球 dome + 盲 thread bore);joint 沿用 `adjuster_nut_spin` CONTINUOUS y | 封顶圆顶 acorn 盖形螺母,外端封闭半球、内端盲螺纹孔 | converged ✓ |

注:wing_nut / dome_nut 仅替换 `adjuster_nut` 的 visual + helper,沿用基线的 `adjuster_nut_spin` CONTINUOUS y joint(轴/origin 不变),是干净的 Slot C 几何替换。

## Multiplicity / Copy Logic
- **无,核心结构为固定 named slots(collar / actuation / nut),无 ×N 复制逻辑。**
- 没有 count_param、没有 `for i in range(N)` 链式发射、没有被复制的整块对象;三个 slot 各占一个固定命名位。
- 唯一一条真实非固定主 joint 是 cam-lever 那条 REVOLUTE(基线 `lever_cam_pivot` z-axis;fold 变体改为 `lever_hinge` x-axis;hex 变体改为 `hex_key_hinge` -x-axis),外加一条 `adjuster_nut_spin` CONTINUOUS y。两者都是单实例,不复制。
- (局部例外)`wing_nut` 的两片 `wing_{i}` 用 `for i in range(2)` 发射,但这是一个 slot 候选内部的固定双翼几何,不是模板级 multiplicity 轴,N 恒为 2。
- `hinged_collar` 候选额外引入一条 `barrel_hinge` REVOLUTE z —— 这是 Slot A 的结构性副作用(整圆环 → 双半弧合页),仍非复制逻辑。

## 跨层接口(未来 InterfaceSpec 预填)
- **collar ↔ actuation**:所有动作机构挂在 collar 的 +Y(cap-side)端,joint origin 位于 cap-side lug 外面(x=`PIVOT_X`,y≈`LUG_Y_OUT`+washer 外,z=`PIVOT_Z`=0.5·BAND_H);mating face = cap-side lug 外端面 + 固定承托件(cam barrel / bolt_head+fork_cheek clevis / socket_head_bolt mouth)。
- **collar ↔ nut**:`adjuster_nut` 挂在 collar 的 -Y(nut-side)端,joint origin x=`PIVOT_X`、y=`NUT_YC`(在 -Y lug 外、贴 `nut_side_thrust_washer`),CONTINUOUS 绕 +Y;与 `cross_bolt` 同轴(coaxial,thread-engagement proxy `allow_overlap`)。
- **共享脊梁**:`cross_bolt`(Cylinder,沿 Y,`BOLT_Y_MIN..BOLT_Y_MAX`)横穿两脚,是三个 slot 共同参照的固定轴线 —— actuation 在 +Y 端、nut 在 -Y 端、collar lugs 在中段。Slot B 变体会改 `BOLT_Y_MAX` 与 +Y 端头(cam barrel→bolt_head→socket head)。
- **hinged_collar 的 re-parent 例外**:Slot A=hinged_collar 时根 part 变为 `cam_arc`,`lever_cam_pivot` parent=`cam_arc`、`adjuster_nut_spin` parent=`nut_arc`,且 nut/lever 的 origin 减去 `HINGE_X` 偏置(child 半弧 local frame 在 +X hinge 轴)。模板做 A×B、A×C 跨格组合时需处理此 parent 重定向。

## 排除项(未来 compatibility matrix 素材)
- 暂无连续不收敛取值(6/6 候选均 converged ✓)。
- 结构耦合提示(交给模板 compatibility matrix,本批未跨格造):
  - Slot A=hinged_collar 把单刚体 collar 拆成 `cam_arc`/`nut_arc` 双 part 并新增 `barrel_hinge`,与 Slot B/Slot C 的 parent 绑定相关;A×B、A×C 组合需统一 re-parent 与 `-HINGE_X` 偏置。
  - Slot B 的 joint 轴随候选变化(基线 z / fold x / hex -x)、`BOLT_Y_MAX` 与 +Y 端头随之改;跨格时 actuation 端固定承托件须与 collar cap-side lug 几何对齐。
- 已主动排除(出类目风险,未列为候选):把 cam lever 整体换成无快拆的纯螺纹手拧(读作普通管箍/hose clamp,失去 quick-release 语义);把 collar 做成完全闭合无 throat/无 slit 的实心环(无法套上 seatpost,失去夹紧功能)。

---
## Post-fork verification (SEGMENT 1 complete)
All 6 planned variants forked via `articraft fork` from baseline 1b80e476, then verified on-disk: last compile = success, required joints present(每变体含 cam-lever 那条 REVOLUTE + `adjuster_nut_spin` CONTINUOUS;hinged_collar 另含 `barrel_hinge`),collections=['workbench'](workbench-only,未 promote),picture.json 绑入 `Parts__quick_release_clamp` subcat shard(reconcile rebuilt)。Slot A/B/C status cells 已 planned→converged ✓。无 multiplicity 轴,Copy Logic 段已记 "无,固定 named slots"。Ready for SEGMENT 2 (spec authoring).
