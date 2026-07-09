# Bench / Wood Swing — template source map

pattern: mixed (parallel_children suspension arms/rods/chains/ropes + slat/chain multiplicity + one core fore/aft revolute pendulum)

parents (5 — an outdoor garden/porch swing: fixed frame + a seat hanging on a fore/aft revolute pendulum; all loop-clean):
- rec_a-frame-log-lawn-glider-swing-round-log-a-frame-_20260611_160845_772176_76ea42fb ← picture/Bench/Wood Swing — round-log A-frame glider; pitched gable roof; facing slatted seats. fills SlotA `log_A_frame`, SlotB `rigid_log_arms`, SlotC `facing_glider_double`, SlotD `pitched_gable`. converged (parent)
- rec_metal-a-frame-swing-bench-cad-style-tubular-stee_20260611_160923_294907_5faa70db ← picture/Bench/Wood Swing — tubular-steel A-frame; rigid link arms; slatted bench; no canopy. fills SlotA `tubular_A_frame`, SlotB `rigid_tubular_arms`, SlotC `slatted_bench`, SlotD `none`. converged (parent)
- rec_model-a-garden-swing-bench-sheltered-by-a-pitche_20260610_085418_407195_71fd0d8b ← picture/Bench/Wood Swing — slatted A-frame end-walls; pitched gable roof; rigid rods; slatted bench. fills SlotA `slatted_A_frame_walls`, SlotB `rigid_rods`, SlotC `slatted_bench`, SlotD `pitched_gable`. converged (parent)
- rec_outdoor-wooden-canopy-swing-chair-dark-stained-h_20260611_160902_717126_09703016 ← picture/Bench/Wood Swing — square-wood A-frame stand; fabric canopy; wooden arms; slatted bench. fills SlotA `square_wood_A_stand`, SlotB `rigid_wood_arms`, SlotC `slatted_bench`, SlotD `fabric_awning`. converged (parent)
- rec_wooden-pergola-garden-swing-daybed-a-heavy-timbe_20260606_115258_129620_2e535d45 ← picture/Bench/Wood Swing — four-post pergola; flat slatted roof; rigid steel rods; daybed platform. fills SlotA `four_post_pergola`, SlotB `rigid_rods`, SlotC `daybed_platform`, SlotD `flat_pergola`. converged (parent)

## Slot 候选覆盖

### Slot A:support_frame(固定支架)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| log_A_frame | rec_a-frame-log-lawn-glider-swing-round-log-a-frame-_20260611_160845_772176_76ea42fb | a_frame_left / a_frame_right / top_beam | 圆木 A 字两腿 + 顶横梁,自然雪松原木 | converged (parent) |
| tubular_A_frame | rec_metal-a-frame-swing-bench-cad-style-tubular-stee_20260611_160923_294907_5faa70db | a_frame_left / a_frame_right / crossbar | 钢管 A 字架 + 横管,CAD 风格灰 | converged (parent) |
| slatted_A_frame_walls | rec_model-a-garden-swing-bench-sheltered-by-a-pitche_20260610_085418_407195_71fd0d8b | shelter_frame | 板条填充的 A 字端墙(实墙感),整支架单 part | converged (parent) |
| square_wood_A_stand | rec_outdoor-wooden-canopy-swing-chair-dark-stained-h_20260611_160902_717126_09703016 | a_frame_stand | 方木深染 A 字立架 | converged (parent) |
| four_post_pergola | rec_wooden-pergola-garden-swing-daybed-a-heavy-timbe_20260606_115258_129620_2e535d45 | pergola_frame | 四柱重木凉棚,顶梁框 + side_beam_{i} | converged (parent) |
| arched_overhead_beam | rec_wood_swing_var_archframe | upright_left / upright_right / crossbar(顶弓梁/直梁) | 直立管腿(无外撇)+ 一根水平/弓形顶梁,梁端四 clevis 吊耳;archrope 用扫掠拱梁 mesh `_build_arch_beam` | converged (workbench, rating pending sync) — EMPTY cell filled by var_archframe & var_archrope |

### Slot B:suspension(吊挂机构 — 核心 fore/aft revolute pendulum)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| rigid_log_arms | rec_a-frame-log-lawn-glider-swing-round-log-a-frame-_20260611_160845_772176_76ea42fb | glider_swing(revolute driver) + swing_arm_*_pivot(revolute) | 4-bar glider 摇臂(driver + mimic),原木臂 | converged (parent) |
| rigid_tubular_arms | rec_metal-a-frame-swing-bench-cad-style-tubular-stee_20260611_160923_294907_5faa70db | swing_drive(revolute driver) + swing_follow_*(revolute) | 钢管刚性吊臂,1 driver + 3 mimic | converged (parent) |
| rigid_rods | rec_model-a-garden-swing-bench-sheltered-by-a-pitche_20260610_085418_407195_71fd0d8b / rec_wooden-pergola-garden-swing-daybed-a-heavy-timbe_20260606_115258_129620_2e535d45 | bench_pivot(revolute) / swing_daybed pivots | 刚性钢杆吊挂(单 pivot 或多 pivot) | converged (parent) |
| rigid_wood_arms | rec_outdoor-wooden-canopy-swing-chair-dark-stained-h_20260611_160902_717126_09703016 | swing_pivot_front_{0,1} / swing_pivot_rear_{0,1}(revolute) | 木质刚性吊臂,前后各两 pivot | converged (parent) |
| chains | rec_wood_swing_var_chainbench / rec_wood_swing_var_chaindaybed / rec_wood_swing_var_logchain | chain_link_{i}(part loop)；顶 link = REVOLUTE(swing_drive/driver + mimic) | 整链由交替 A/B 朝向的 oval 链节复制构成(`_chain_link_meshes` / `_make_chain_link_mesh`);链顶 revolute,座固定到 driver 链 | converged (workbench, rating pending sync) |
| ropes | rec_wood_swing_var_ropebench | rope_segment_{i}(part loop);顶端 REVOLUTE | 4 根麻绳吊段,各自 revolute pivot,`_add_rope_cylinder` 圆柱拼段 | converged (workbench, rating pending sync) |

### Slot C:seat(座面)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| facing_glider_double | rec_a-frame-log-lawn-glider-swing-round-log-a-frame-_20260611_160845_772176_76ea42fb | seat_front / seat_rear / center_table / glider_platform | 对坐双板条座 + 中置小桌,glider 平台 | converged (parent) |
| slatted_bench | rec_metal-a-frame-swing-bench-cad-style-tubular-stee_...  / rec_model-a-garden-swing-bench-...  / rec_outdoor-wooden-canopy-swing-chair-... | bench_seat / bench_back(metal) ; bench_swing(pitched) ; bench(canopy) | 板条座面 + 板条靠背,座板/背板循环复制 | converged (parent) |
| daybed_platform | rec_wooden-pergola-garden-swing-daybed-a-heavy-timbe_20260606_115258_129620_2e535d45 | swing_daybed | 宽平卧榻平台(座框 + 板条 + 坐垫) | converged (parent) |
| single_hanging_chair | rec_wood_swing_var_chairseat | swing_chair | 单人吊椅(座 + 靠背 + 扶手一体),挂于四 swing 臂 | converged (workbench, rating pending sync) |

### Slot D:canopy/roof(顶棚)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| none | rec_metal-a-frame-swing-bench-cad-style-tubular-stee_20260611_160923_294907_5faa70db | （无顶棚 part） | 裸支架,无遮盖 | converged (parent) |
| pitched_gable | rec_a-frame-log-lawn-glider-swing-round-log-a-frame-_... / rec_model-a-garden-swing-bench-... | roof_panel_front / roof_panel_rear(log) ; shelter_frame 顶(pitched) | 双坡人字屋顶,rafter/rib 循环复制 | converged (parent) |
| fabric_awning | rec_outdoor-wooden-canopy-swing-chair-dark-stained-h_20260611_160902_717126_09703016 | a_frame_stand 顶部织物罩 | 弧形织物遮阳篷 | converged (parent) |
| flat_pergola | rec_wooden-pergola-garden-swing-daybed-a-heavy-timbe_20260606_115258_129620_2e535d45 | pergola_frame 顶板条 | 平顶凉棚板条(saturated;无 variant) | converged (parent) |

## Multiplicity / Copy Logic
- count_param:
  - `seat_slat_count`(座板) — parents 覆盖 N ∈ {4, 6, 9}(canopy/metal/pitched 各档;ropebench n_slat=7)
  - `back_slat_count`(靠背板) — parents 覆盖 N ∈ {4, 5, 8, 9}
  - `roof_rib_count`(屋顶椽/肋) — N ∈ {13, 16, 24, 32}(chaindaybed n_raft=13;ropebench n_rib=24)
  - `chain_link_count`(链节,新 copy logic) — chainbench N_CHAIN_LINKS=19;logchain N_LINKS=13;chaindaybed 每链 oval 链节循环
  - `rope_segment_count` / 每绳分段 — ropebench 4 根绳(`for i in range(4)`),每绳由 `_add_rope_cylinder` 拼段
- N 样本已覆盖:
  - 座板 {4,6,9} → canopy / metal&ropebench / pitched&chaindaybed
  - 屋顶肋 {13,16,24,32} → 各 parent + chaindaybed(13)/ropebench(24)
  - 链节 {13,19} → logchain / chainbench(+chaindaybed 环形 oval 链)
- 模板建议 N_range:slats [3, 20];rafters [8, 32];chain links 按真实几何(每链约 [8, 30]);rope segments [1, 8]
- copied object / naming / placement / joint policy:
  - copied object:座/背板条、屋顶椽肋、链节(`chain_link_{i}` / `chain_link_a/b` 交替朝向)、绳段(`rope_segment_{i}`)
  - naming:`for i in range(n)` + `f"<name>_{i}"`(side_beam_{i} / rope_segment_{i} / chain_link_{i});slats/ribs 在 `for k/j in range(n)` 内等距
  - placement:slats/ribs 沿座宽/屋顶跨度等距;chains/ropes 锚定在四个 fore/aft × 左右角(CHAIN_Y、CHAIN_FRONT_X/REAR_X;rope ROD_Y);链节沿吊挂方向按 LINK_PITCH 等距堆叠
  - joint policy:整支架→各吊挂件一根 REVOLUTE(轴 Y,fore/aft 摆动);1 个 driver(swing_drive / glider_swing / DRIVE_JOINT="swing_pivot_0")+ 其余 mimic(multiplier 1.0);座面 fixed 到 driver 吊挂件,整座作单一 pendulum 摆动

## 排除项(未来 compatibility matrix 素材)
- Slot D(canopy/roof)已 saturated(none / pitched_gable / fabric_awning / flat_pergola 四候选均由 parent 覆盖)— 不专门做 variant。
- slat / rib 的数量 N 不 fork:parents 已 loop-emit 多档(座板 {4,6,9}、背板 {4,5,8,9}、屋顶肋 {13,16,24,32}),template seed sweep 覆盖计数轴 → 采样器的活,不消耗 variant 预算。
- 唯一新 fork 的 copy-logic 是 chain-link / rope-segment(没有 parent 含链节或绳段)— 由 var_chainbench / var_chaindaybed / var_logchain / var_ropebench 引入 `chain_link_{i}` / `rope_segment_{i}` 循环。
- 闭合 4-bar glider 连杆机构不深追:保留 driver + mimic idiom(glider_swing driver + 3 swing_arm_*_pivot mimic),不展开为真正闭环约束。
- 跨轴组合(如 tubular_A_frame × chains × daybed)交给模板采样器,不专造组合变体。
- color / scale / 纯比例不是结构轴(配色材质可自由叠加,不计变化)。

---
7 个 variant 填格情况:
- var_archframe → Slot A `arched_overhead_beam`(EMPTY cell)+ rigid_tubular_arms 基线
- var_archrope → Slot A `arched_overhead_beam`(扫掠拱梁 mesh)+ Slot B `ropes` 二填
- var_chainbench → Slot B `chains`(metal parent)
- var_chaindaybed → Slot B `chains`(pergola parent,环形 oval 链)
- var_logchain → Slot B `chains`(log parent)
- var_ropebench → Slot B `ropes`(pitched parent,`rope_segment_{i}`)
- var_chairseat → Slot C `single_hanging_chair`
