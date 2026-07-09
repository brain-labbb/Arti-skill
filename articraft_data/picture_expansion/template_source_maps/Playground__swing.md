# Playground / swing — template source map

slug: `playground_swing_set`
pattern: frame+pendulum (fixed overhead frame + N loop-emitted swing seats, each hung on a top-rail REVOLUTE fore/aft pivot; suspension = chains/rods/ropes)

parents (2 — a playground swing set: a fixed frame carries a top rail; N seats hang and swing fore/aft on revolute pivots):
- rec_model-a-commercial-playground-swing-set-with-a-g_20260610_085408_807377_83c0a0b1 ← picture/Playground/swing — `commercial_playground_swing_set`; RED steel A-frame (top_rail Box 3.0m + 2 A-frame ends each 2 tilted box legs + base plates + braces + 4 hangers); 2 swings `swing_0`/`swing_1`, each 2 galvanized oval-link chains (34 links) + sagged rubber belt seat; 2 REVOLUTE `swing_{0,1}_pivot` axis (1,0,0) z=2.228. fills SlotA `red_steel_A_frame`, SlotB `oval_link_chains`, SlotC `rubber_belt_seat`. converged (parent)
- rec_model-a-rustic-playground-double-swing-built-fro_20260610_085359_406294_4094e053 ← picture/Playground/swing — `rustic_log_double_swing`; splayed log-post pairs rope-lashed (4 posts loop) + log crossbeam + 4 brass finials; 2 loop-emitted `swing_i` each 2 tapered rope cones + knots + seat_plank + back_rail + corner fittings; 2 REVOLUTE `beam_to_swing_i` axis (0,1,0) z=1.95. fills SlotA `rustic_log_frame`, SlotB `tapered_ropes`, SlotC `plank_seat`. converged (parent)

## Slot 候选覆盖

### Slot A:frame_type(固定支架 — 承 top rail + swing pivots)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| red_steel_A_frame | rec_model-a-commercial-playground-swing-set-with-a-g_20260610_085408_807377_83c0a0b1 | top_rail / a_frame_left / a_frame_right | 红钢 A 字端架 + 顶横梁 | converged (parent) |
| rustic_log_frame | rec_model-a-rustic-playground-double-swing-built-fro_20260610_085359_406294_4094e053 | log_post_i / crossbeam / finial | 外撇原木柱对(绳捆)+ 原木横梁 | converged (parent) |
| single_arch_frame | rec_pswg_var_arch | arch_beam(swept) / foot | 单高拱(倒 U 管)跨越摆区 | workbench (pending sync) — EMPTY cell |
| straight_H_frame | rec_pswg_var_hframe | post_left / post_right / top_rail | 直立 H 框立柱(无外撇) | workbench (pending sync) — EMPTY cell |

### Slot B:suspension(吊挂 — 核心 fore/aft REVOLUTE pendulum)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| oval_link_chains | rec_model-a-commercial-playground-swing-set-with-a-g_20260610_085408_807377_83c0a0b1 | chain_link_i(34) / swing_i_pivot(REVOLUTE) | 镀锌 oval 链节链(交替朝向) | converged (parent) |
| tapered_ropes | rec_model-a-rustic-playground-double-swing-built-fro_20260610_085359_406294_4094e053 | rope cone / knot / beam_to_swing_i(REVOLUTE) | 锥形麻绳吊 + 绳结 | converged (parent) |
| rigid_rods | rec_pswg_var_rod | rod_i / swing_i_pivot(REVOLUTE) | 刚性钢杆吊挂(仍顶 revolute) | workbench (pending sync) — EMPTY cell |

### Slot C:seat_type(座面 — fixed 到 driver 吊挂件,整座作 pendulum)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| rubber_belt_seat | rec_model-a-commercial-playground-swing-set-with-a-g_20260610_085408_807377_83c0a0b1 | belt_seat(sagged) | 下垂橡胶带座 | converged (parent) |
| plank_seat | rec_model-a-rustic-playground-double-swing-built-fro_20260610_085359_406294_4094e053 | seat_plank / back_rail | 木板座 + 靠条 | converged (parent) |
| tire_seat | rec_pswg_var_tire | tire_torus / hang_rope_i | 水平吊轮胎座(torus)+ 多绳汇聚 | workbench (pending sync) — EMPTY cell |

## Multiplicity / Copy Logic
- count_param: `swing_count`(座 N) — parents 覆盖 N=2(both);variant 扩 N ∈ {1, 3}
- N 样本已覆盖:{1(var_n1), 2(parents), 3(var_n3)};模板建议 N_range [1, 4]
- 次级 count:chain_link_count(链节,parent=34/链)— 采样器扫,真实几何每链 [8, 40]
- copied object / naming / placement / joint policy:
  - copied object:座 `swing_i` / 链节 `chain_link_i` / 绳段 / 吊杆 `rod_i`
  - naming:`for i in range(swing_count)` + `f"swing_{i}"`;沿 top_rail 等距 X
  - placement:座沿顶梁等距;每座两侧吊挂锚在顶梁
  - joint policy:每座 1 REVOLUTE fore/aft pivot(轴沿梁向 (1,0,0) 或 (0,1,0));座 fixed 到 driver 吊挂件;1 driver + 余 mimic(multiplier 1.0)

## 排除项
- swing_count N 不专门多 fork:parents N=2 + var_n1/var_n3 三档 → 采样器。
- chain_link 计数交采样器(parents loop-emit 34 链节)。
- 跨轴组合(arch × rods × tire × N3)交模板采样器。
- color / material / 比例不是结构轴。

---
6 个 variant 填格:
- var_arch → SlotA `single_arch_frame`(EMPTY)
- var_hframe → SlotA `straight_H_frame`(EMPTY)
- var_rod → SlotB `rigid_rods`(EMPTY)
- var_tire → SlotC `tire_seat`(EMPTY)
- var_n1 → swing_count N=1
- var_n3 → swing_count N=3
