# Container / Primer bottle — template source map

pattern: parallel_children

parents:
- rec_black-airless-cosmetic-primer-pump-bottle-with-a_20260606_074952_062921_ec0caf66 ← picture/Container/Primer bottle/001.png (black airless cosmetic primer pump bottle; cushion/squircle body + flat airless press-pump). Occupies cell (Slot A = cushion_squircle, Slot B = airless_press_pump).

Single-parent小类. Core kinematics: a `body` root (cushion shell + inline `gold_band` + `label_plate`
visuals, tapered shoulder + neck collar fused into one shell) carrying a top closure part that
articulates. The two independent structural slots are the body cross-section family (Slot A) and the
closure / dispenser mechanism (Slot B). The reference photo itself shows two body families (soft
cushion on the left, squared rectangular on the right), grounding Slot A; the closure family (Slot B)
is the dominant real-world structural vocabulary for cosmetic primer bottles (airless press-pump,
side-lever pump, screw twist cap, dropper). Color / material / pure scale are NOT axes.

## Slot 候选覆盖

### Slot A:body cross-section / footprint family (`body` root, `body_shell` visual, `_body_solid` helper)
| 候选(未来 module) | record_id | 关键 part·joint·helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| cushion_squircle | rec_black-airless-cosmetic-primer-pump-bottle-with-a_20260606_074952_062921_ec0caf66 (parent) | `body`/`body_shell`, `_rounded_prism(BODY_FILLET=0.008)`, `_body_solid` | slim heavily-filleted squircle cross-section (cushion); rounded vertical edges | converged(parent) |
| square_prism | rec_container_primer_bottle_var_square_body | `body`/`body_shell`, `_body_solid` with near-zero edge break | crisp angular rectangular prism, near-sharp vertical edges (right-hand bottle in photo) | converged |
| round_cylinder | rec_container_primer_bottle_var_round_body | `body`/`body_shell`, lathe/revolve circular-section `_body_solid` | true round cylindrical body revolved about +Z, tapering to shoulder/neck | converged |
| oval_section | rec_container_primer_bottle_var_oval_body | `body`/`body_shell`, lathe/loft elliptical-section `_body_solid` | flattened oval (elliptical) cross-section — wide across front face, shallow front-to-back; smooth elliptical lathe/loft (distinct from filleted-rect squircle and from true circle) | converged |

### Slot B:closure / dispenser mechanism (top closure part + its joint over the neck collar)
| 候选(未来 module) | record_id | 关键 part·joint·helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| airless_press_pump | rec_black-airless-cosmetic-primer-pump-bottle-with-a_20260606_074952_062921_ec0caf66 (parent) | `pump_top`/`pump_cap`, `pump_press` (PRISMATIC z, ±PUMP_TRAVEL), `_pump_solid` | flat airless actuator with hollow skirt over neck + dispense orifice; presses straight down then springs back | converged(parent) |
| side_lever_pump | rec_container_primer_bottle_var_lever_pump | `pump_lever` part, `lever_pivot` (REVOLUTE), pivot post visual on neck | curved lever arm hinging on a neck-top pivot post, swings down to dispense (REVOLUTE replaces prismatic) | converged |
| screw_twist_cap | rec_container_primer_bottle_var_twist_cap | `twist_cap` part, `cap_unscrew` (REVOLUTE z), threaded skirt over neck collar | threaded turning lid that unscrews about +Z to reveal the mouth (no press pump) | converged |
| dropper_cap | rec_container_primer_bottle_var_dropper_cap | `dropper` assembly (`squeeze_bulb` + `pipette`), `dropper_lift` (PRISMATIC z) | screw collar + rubber bulb + slim glass pipette lifting out of the neck along +Z | converged |
| spray_atomizer | rec_container_primer_bottle_var_spray_atomizer | `spray_head`/`atomizer_cap` part + side `nozzle_spout` visual, `spray_press` (PRISMATIC z) | fine-mist atomizer: finger-pad head over neck with a directional nozzle spout protruding SIDEWAYS out of the front, firing laterally; presses straight down (distinguishing structure = protruding side nozzle, not flat top orifice) | converged |
| flip_top_disc | rec_container_primer_bottle_var_flip_top | `flip_lid` part, `flip_hinge` (REVOLUTE, HORIZONTAL axis), low cap shell over neck | hinged snap flip-top disc cap; circular lid flips up about a horizontal back-edge living-hinge to reveal the orifice (horizontal-axis REVOLUTE — distinct from twist z-revolute and press z-prismatic) | converged |
| treatment_spout_pump | rec_container_primer_bottle_var_treatment_spout | `pump_head` part + curved `gooseneck_spout` visual, `pump_press` (PRISMATIC z) | tall treatment/lotion pump: cylindrical actuator collar over neck with a long curved gooseneck spout bending out/over to a downturned dispense tip; presses straight down (distinguishing structure = elongated curved spout, not flat capless puck) | converged |

## Multiplicity / Copy Logic
- count_param: 无,核心结构为固定 named slots(body / closure)。primer bottle 没有"同构子件 × N"的复制逻辑。
- N 样本计划: 无 multiplicity 轴。
- 模板建议 N_range: 无。
- copied object / naming / placement / joint policy: 无。(parent 无重复子件,无 for-i 循环需求;§4 可读性契约已满足——装饰 gold_band / label_plate 已内联为 parent visual,无 FIXED 装饰 part。)

## 组合数预审
组合数预审: Π(Slot A=4 × Slot B=7) × N(无) = 28 ≥ 10 ✓(deepened 2026-06-18:Slot A +oval_section,Slot B +spray_atomizer/+flip_top_disc/+treatment_spout_pump)。每个 slot ≥2 候选(A=4, B=7),pattern = parallel_children,无 multiplicity。Slot B 已达 §5 实务上限(7 个 ≈ 化妆品瓶真实闭合机构词汇表的边界:airless 顶压 / 侧杆泵 / 旋盖 / 滴管 / 雾化喷头 / 翻盖 / 长颈乳液泵)。
待填空格 cells = (Slot A 4−1=3) + (Slot B 7−1=6) = 9 个变体(parent 免费占 cushion×airless 格)。原 5 个 + 本轮新增 4 个 = 9。

## 排除项(未来 compatibility matrix 素材)
- 纯尺寸(更高/更矮/更宽/更扁的瓶身)与配色/哑光-亮光材质不作为候选——属模板连续参数(controlled local parameterization),不入 slot。
- tall slim cylinder(细高圆柱)未单独造样本:与 round_cylinder 同为圆形横截面,仅比例(更瘦更高)不同 = 连续参数,非结构 module,折入 round_cylinder。
- plain "tall actuator" foaming pump(单纯更高的顶压头、无独立喷口几何)未单独造样本:与 airless_press_pump 同为 PRISMATIC 顶压且仅 actuator 高度不同 = 连续尺寸参数。注意:treatment_spout_pump 已作为单独候选造样本——它的区分点是**真实独立的长弯颈喷口 part**(几何/part 拓扑差异),不是单纯加高。
- spray atomizer(雾化喷头)deepening 2026-06-18 已**提升为 Slot B 独立候选**(rec_container_primer_bottle_var_spray_atomizer):虽与 airless_press 同为 PRISMATIC 顶压,但其向前侧伸的 nozzle_spout 是真实独立 part(part 拓扑 + 侧向出口几何差异),按 §5 候选间需结构差异的标准成立,故不再折入 airless_press。
- 跨轴组合(如 round_cylinder × dropper_cap)未抽检——组合由模板采样器免费生成,样本池不负责枚举。
- 第 3 结构轴(neck/collar form、applicator type 等)经核验**不存在**:该瓶为简单短颈泵瓶,无 wand/brush applicator,颈口形态无真实结构变体——故仍维持 body × closure 两轴。
