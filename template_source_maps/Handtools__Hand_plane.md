# Handtools / Hand plane — template source map

pattern: parallel_children
parents:
- rec_build-a-realistic-articulated-3d-model-of-a-hand_20260609_163932_985437_d722d7db ← picture/Handtools/Hand plane/001.png (black japanned No.4/No.5 bench smoothing plane: rosewood front knob + rear D-tote, 45° bedded iron, polished steel lever cap with flip-cam, brass depth wheel)

Bench/block hand plane. Core kinematics shared by all candidates: the cast-iron
`plane_body` (root) carries the integral `frog` bed (a 45° or 20° ramp visual on
the body) on which the `cutting_iron` is FIXED-bedded at the mouth. Three
non-fixed joints recur in every variant:

- PRIMARY = the blade-clamp actuator (joint type/name varies per Slot B candidate —
  see below);
- SECONDARY = `lateral_adjust` (`lateral_lever`, REVOLUTE about `BED_NORMAL`,
  lower/upper ±0.35) riding flat on the iron's exposed face;
- TERTIARY = `depth_adjust` (`depth_wheel`, CONTINUOUS about `BED_UP`) captured on
  the `depth_stud` rooted in the frog.

The sole/body silhouette, the blade-clamp mechanism, and the user grip are the
three independent structural slots below. Bed angle (45° vs 20°), body length and
width track Slot A and are continuous template parameters, not separate slots.

## Slot 候选覆盖

### Slot A:sole / body silhouette (`plane_body` casting + `frog`)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| solid_block_45 | rec_..._hand_..._d722d7db (parent) | `_bench_plane_body` (solid mid-block, slanted iron slot + spine relief), `_frog` 45° ramp, `_tilt_back`=45° | classic Bailey smoothing body: solid central casting, full-width closed throat, steep 45° frog | converged |
| low_block_trough | rec_hand_plane_var_blocksole | `_block_plane_body` (short sole + two low sidewalls = open trough, no tall casting), `BED_DEG`=20°, `WALL_HEIGHT` | stubby low-angle block plane: 155 mm sole, open trough between sidewalls, 20° integral bed | converged |
| narrow_open_cheek | rec_hand_plane_var_rabbet | `_rabbet_plane_body` (full-height -Y cheek + 3 mm `OPEN_CHEEK_HEIGHT` lip on +Y, full-width slot), full-width `_cutting_iron` (blade_w≈BODY_WIDTH) | narrow rabbet/shoulder body, one cheek open so iron runs to full width for flush cuts | converged |

### Slot B:blade-clamp mechanism (PRIMARY joint, over the iron)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| levercap_flipcam | rec_..._hand_..._d722d7db (parent) | `lever_cap` part (FIXED), `cam_lever` part + `lever_cap_cam` (REVOLUTE axis −Y, parent=cap, on `CAM_PIVOT`); `_cap_screw` inline body visual | polished steel lever cap clamps iron; spring flip-cam lever pivots up about a Y pin to release | converged |
| crossbar_thumbscrew | rec_hand_plane_var_clampbar | `clamp_bar` part (FIXED, pinned on two `clamp_boss_{i}` body visuals via `_thumbscrew_stud`), `thumbscrew` part + `thumbscrew` joint (REVOLUTE axis `BED_NORMAL`) | flat steel cross-bar held by two cast bosses; knurled brass thumbscrew turns on a stud to press the bar onto the iron | converged |
| capnut_post | rec_hand_plane_var_screwcap | `cap_post` inline body visual, `cap_nut` part (KnobGeometry, knurled brass) + `cap_nut_spin` (CONTINUOUS axis `BED_NORMAL`); iron has `post_hole` | threaded post rises from frog through the iron; knurled brass cap-nut spins down the post to clamp the stack | converged |

### Slot C:handle / grip (seated on the sole)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| knob_plus_Dtote | rec_..._hand_..._d722d7db (parent) | `front_knob` part (revolved mushroom) + `body_to_front_knob` (FIXED); `rear_tote` part (open-D extrude+ellipse cut) + `body_to_rear_tote` (FIXED) | traditional two-piece grip: turned front knob on toe boss + upright open-D rear tote on heel boss | converged |
| horn_plus_ringtote | rec_hand_plane_var_closedtote | `front_horn` part (`_front_horn_geom` section_loft curved horn) + `body_to_front_horn` (FIXED); `rear_tote` part (closed-loop ring with fully enclosed oval window) + `body_to_rear_tote` (FIXED) | forward-curving lofted front horn + closed-ring (fully enclosed grip window) rear tote | converged |
| single_palm_hump | rec_hand_plane_var_palmgrip | `palm_rest` part (revolved dome ∩ box footprint, cut finger groove) + `body_to_palm_rest` (FIXED); NO front knob (wider single heel boss) | one-hand low palm-hump rear rest with finger groove; front knob removed entirely | converged |

## Multiplicity / Copy Logic
- count_param: 无结构性复制轴,核心结构为固定 named slots(body / clamp / grip)。
- N 样本: 无 multiplicity 轴。(唯一的循环发射是 `depth_wheel` 的 knurl 凹槽 `for i in range(n)`,n≈20–24,以及 clampbar 的两端 `clamp_boss_{i}` / pin-hole `for i in range(2)`——都是装饰/对称硬件细节,不是可参数化的结构数量轴。)
- copied object / naming / placement / joint policy: depth-wheel 与 cap-nut knurl = 共享几何 helper 内 `for i in range(n)` 等角发射的内嵌 cut(非独立 part,合规)。clampbar 的 `clamp_boss_{i}`(i=0,1, y_sign=1−2i)与 bar pin-hole 用 `for i in range(2)` 对称发射,是正确的 copy-loop 写法。

## 组合数预审
Slot A(3) × Slot B(3) × Slot C(3) = 27 ≥ 10 ✓。每个 slot 恰好 3 个 converged 候选(parent 基线各计为其中一个)。pattern = parallel_children;无 multiplicity 轴,组合数完全由三槽候选乘积提供(已远超 MIN_DISTINCT=10 门槛)。无需补造 gap 变体。

## 排除项(未来 compatibility matrix 素材)
- 跨槽组合未抽检(由模板采样器生成):block-plane 低墙 trough body(Slot A `low_block_trough`)的开顶高度仅 WALL_HEIGHT≈20 mm,与高耸的 D-tote/ring-tote(Slot C 非 palm 候选)同框时 tote 会显著高出短机身——真实世界里 block plane 通常配 palm-hump,模板侧若组合 `low_block_trough × knob_plus_Dtote` 需复核比例(候选不冲突,但属"比例需 controlled-param 收口"项)。
- Slot B `capnut_post` 把 PRIMARY joint 从 REVOLUTE 改成 CONTINUOUS 且 child=body(post 是 body inline visual,nut 直接挂 body),而 `levercap_flipcam` 的 cam child=lever_cap(两级挂载)。模板侧抽 Slot B module 时注意 PRIMARY 关节的 parent 部件随候选而变(cap vs body)、joint type 随候选而变(REVOLUTE flip / REVOLUTE spin / CONTINUOUS spin)——这是 InterfaceSpec 的 consumer-joint 字段差异点,非排除项。
- Slot A `narrow_open_cheek` 的 iron 跑满全宽(blade_w≈BODY_WIDTH),与窄体 lever-cap 宽度耦合;若与 Slot B `crossbar_thumbscrew`(BAR_Y_HALF=0.022 跨距按 62 mm 体宽写死)组合,需把 bar 跨距改成随 body width 缩放——standard width-scale 收口,登记为未来 compatibility 复核项。
- 纯尺寸/比例(body 长 155/200/245 mm、宽 30/52/62 mm、bed 角 20°/45°)不作为候选——属模板连续参数(controlled local parameterization),随 Slot A 候选一起取值,不入独立 slot。
