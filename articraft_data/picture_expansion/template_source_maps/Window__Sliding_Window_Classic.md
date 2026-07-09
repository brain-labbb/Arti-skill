# Window / Sliding Window Classic — template source map

pattern: mixed (parallel_children 固定 named slots: outer_frame(static root) + sill + sliding_sash 主机构(PRISMATIC) + fixed_glazing 模块; 叠加一条轻 multiplicity — meeting/muntin 分隔条 N，sash_count {1 sash-only / 2 sliding+fixed})

slug: sliding_window_classic   shard: Window__Sliding_Window_Classic   prefix: rec_sliding_window_classic_var_

> 注: 本 source pool 是 **54 个独立生成的 curated DATASET 记录**（category_slug=sliding_window，id `rec_sliding_window_0001..0005` + `rec_sliding_window_<hex>`），**不是单一母体的 fork**。因此下方「parents」改写为**每个结构 cluster 的代表性 archetype 记录**。已完整读取的代表样本 ~24 个，跨全部 54 个的结构变化已由分簇审计覆盖。本 slug 是 sibling slug `sliding_window`（建自另一 qwen fork pool，144 条 `rec_qwen37v_sliding_window_*`）的 **并行/替代模板** — 两者 identity 重叠（都是 PRISMATIC 横滑窗），可共存。

## archetype 代表记录（按 cluster，各覆盖若干槽位组合）

- **rec_sliding_window_0004**（2-part-fixed-lite + 共享 sash helper, separate FIXED part）← 1 sliding `sliding_sash`(PRISMATIC axis(1,0,0)) + 1 separate `fixed_lite`(FIXED)。helper `_build_sash(meeting_left, moving)` model.py:L90-263（双面复用建 fixed+sliding 两扇）、`_add_corner_plates` L71-88、`_add_fasteners` 嵌套 loop L48-69。joint `frame_to_fixed_lite` FIXED L449-455、`frame_to_sliding_sash` PRISMATIC axis(1,0,0) L456-469。rollers/guides: `bottom_glide_left/right`/`top_guide_left/right` L176-201。**= 最干净的 2-part 基线（separate-fixed-part + 共享 sash 工厂 + interlock fin）。**
- **rec_sliding_window_0005**（separate fixed_sash + moving_sash + 独立 articulated `latch` REVOLUTE）← `_add_glazed_sash(meeting_side)` helper L49-229、3 joints: `frame_to_fixed_sash` FIXED L419-425、`frame_to_moving_sash` PRISMATIC axis(1,0,0) L426-439、`moving_sash_to_latch` **REVOLUTE axis(0,1,0)** L440-453。runner/guide loop `for side_name,side_sign` L180-193。scale-assert run_tests L513-522。**= articulated 拇指扳手 latch 母体（Slot lock_style 的 thumbturn 候选）。**
- **rec_sliding_window_0001**（baked-fixed-lite 单 frame + 共享 `_add_glazed_panel`, rating 4）← fixed lite 烘焙进 `frame` part（`fixed_panel_*` visuals L235-251），1 sliding `sliding_sash`(PRISMATIC axis(1,0,0)) L301-314。helper `_add_glazed_panel(panel_name,...)` L51-147（gasket 四边 + glass rebate）。track: `fixed_track_base`/`sliding_track_base`/`track_separator`/`*_head_guide` L184-230。`pull_handle`/`latch_thumbturn`(visual) L276-285。`roller_cover_left/right` L279-299。**= baked-fixed 双轨道分离母体。**
- **rec_sliding_window_0002**（rugged industrial, 2-part baked-fixed, roller Cylinders + meeting post）← `_add_screw_head` Cylinder helper L48-62、fastener loop `for idx,xyz in enumerate(...)` L208-227。fixed `fixed_glass`+`meeting_post`+`mullion_rear_land` on frame L115-206。`left_roller`/`right_roller` Cylinder L319-330、`left/right_top_guide` L331-342。`pull_base`/`pull_grip` L344-355。joint `frame_to_sash` PRISMATIC axis(-1,0,0) L369-382。dark graphite (0.31,0.35,0.33)。
- **rec_sliding_window_0003**（premium, baked-fixed, rollers + interlock_fin + pull_plate/pull_grip）← `_add_box`/`_add_cylinder` L51-75、front/rear 双轨 `*_track_base`/`*_roller_rail`/`*_head_ceiling` L153-216、`roller_left/right` Cylinder L436-453、`interlock_fin` L468-474、`pull_plate`+`pull_grip` Cylinder L475-490。joint `sash_slide` PRISMATIC axis(1,0,0) L492-505。
- **rec_sliding_window_a9564b40**（field-service: REVOLUTE latch + REVOLUTE 维修盖 access cover, bronze）← `sash_to_latch` REVOLUTE axis(0,1,0) + `frame_to_access_cover` REVOLUTE axis(1,0,0)（continuous hinge pin）；bolt-grid loop L886-905、hinge-knuckle loop L909-921、glide-shoe loop L939-952、cover hinge-leaf loop L981-999；`fixed_mullion`/`fixed_glass` baked。dark bronze (0.09,0.075,0.055) + red handle + yellow PTFE shoes。**= 富 articulation archetype（latch + 维修盖两条 REVOLUTE）。**
- **rec_sliding_window_cbfa0ab2**（precision/calibration: CONTINUOUS fine knob + REVOLUTE clamp knob, sash-only, dark+brass）← `frame_to_sash` PRISMATIC axis(1,0,0) + `fine_knob_spin` **CONTINUOUS axis(0,-1,0)** + `clamp_knob_turn` **REVOLUTE axis(0,-1,0)**；index-tick loop `for i in range(...)` L334-342（11 刻度）；无 fixed lite（标定滑窗）。dark_anodized (0.08,0.09,0.10) + brass。
- **rec_sliding_window_b5bb4681**（industrial safety: PRISMATIC 锁定销 lockout pin + 防护栅 guard grille, sash-only, galvanized+yellow+red）← `frame_to_sash` PRISMATIC axis(1,0,0) + `frame_to_lock_pin` **PRISMATIC axis(0,0,1)**（lift-out T-handle）；guard-bar loop L1236-1237（7 条）、gusset/bolt loops L1249-1304；无 fixed lite；2.1m 大窗。dark_galvanized + safety_yellow + lockout_red + polycarbonate。
- **rec_sliding_window_dc83a7ca**（retrofit: 2 REVOLUTE 维修盖 + muntin 十字, aged cream wood-look）← `frame_to_sash` PRISMATIC axis(1,0,0) + `frame_to_lower_hatch` REVOLUTE axis(-1,0,0) + `frame_to_side_hatch` REVOLUTE axis(0,0,-1)；`middle_muntin`+`upright_muntin` 十字 on sash L1928-1929；adapter-bolt/corner-strap loops L1881-1912。aged_cream (0.78,0.73,0.61)。
- **rec_sliding_window_7404a1**（roller-truck rugged: nylon 轮 Cylinder on stainless track + MotionProperties damping, charcoal-black）← `sash_slide` PRISMATIC axis(1,0,0)；`roller_0/1` Cylinder + brackets L114-127、grip-rib loop `for idx` L93-94、corner-plate+screw loops L97-110、frame-screw loop L65-76。charcoal powder_coat (0.10,0.12,0.11) + smoked glass + nylon rollers。**= roller-truck track 母体。**
- **rec_sliding_window_645606**（bidirectional center-park, CadQuery boolean frame, frosted bathroom glass）← `frame_to_sash` PRISMATIC axis(1,0,0)，**lower=-travel/2, upper=+travel/2**（双向中停）L169-182；`_build_frame` CadQuery 切轨道 + drain-slot loop L56-64。vinyl_white + frosted (0.55 alpha)。
- **rec_sliding_window_dd9f8469**（wood cabin, ±Y slide, brass latch + 2 muntins, rating 2）← `frame_to_sash` PRISMATIC **axis(0,1,0)**（唯一非 X 轴）；`muntin_0`/`muntin_1` 竖条 L68-79；`brass_latch`(visual)。wood (0.55,0.27,0.07) + brass。**（仅作 muntin/colorway/±Y 边角证据，低 rating，不作主候选）**

## loop-emission 合约审计

- **PASS（已 loop 发射重复子件）**:
  - 0004 — fastener `_add_fasteners` 嵌套 `for x in xs / for z in zs` L48-69；corner-plate `for idx,(x,z)` L81-88；sash `_build_sash` 工厂复用建 fixed+sliding。
  - 0005 — runner/guide `for side_name,side_sign` L180-193；`_add_glazed_sash` 复用建 fixed+moving。
  - 0002 — screw `for idx,xyz in enumerate` L208-227；`_add_screw_head` helper。
  - a9564b40 — bolt-grid / hinge-knuckle / glide-shoe / cover-leaf 多条 for-loop L886-999。
  - cbfa0ab2 — index-tick loop L334-342。
  - b5bb4681 — guard-bar / gusset / bolt loops L1236-1304。
  - 7404a1 — grip-rib / corner-plate / screw / roller loops L65-127。
  - 645606 — drain-slot loop L56-64。
  - 66a60f / 720dbc / 948397 / cd856d31 / cf88dde9 / e7761bbe / f0621fda — rail/guide/track-stop 均 `for z,prefix` 或 `for col/row` loop 发射。
- **FAIL/N/A（手写或无重复）**:
  - 0001 / 0003 — rollers/guides 逐个手写（`roller_cover_left/right`、`roller_left/right`），无 range loop。0001 rating=4。**off 它们做 multiplicity 必须先改 `for i in range(n)`。**
  - dd9f8469 — `muntin_0/1` 手写两条（非 range），±Y 轴，rating=2。
  - **结论**: meeting/muntin-bar 与 sash_count 这条轻 multiplicity 轴的母体取 **0004 / 0005**（已 PASS loop-emission 的共享 sash 工厂）。任何 muntin 网格条 / 滑轨导轨 / 紧固件复制都从 PASS 母体（0004/0005/66a60f/948397 系）的 for-loop 发射，硬性满足 for-i-in-range 重写合约。

## 组合数预审（HARD GATE）

- Slot A fixed_glazing_topology = 3 候选（separate_fixed_part(FIXED) / baked_into_frame / sash_only_no_fixed）
- Slot B track_carriage_style = 4 候选（lipped_dual_channel / roller_truck_cylinders / triple_rib_guide / cadquery_hollow_channel）
- Slot C lock_articulation = 4 候选（none_passive_visual / thumbturn_revolute / crescent_cam_revolute / lockout_pin_prismatic）
- Slot D handle_style = 5 候选（pull_bar / flush_recess / finger_pull / molded_grip_ribs / d_pull_plate）
- 轻 multiplicity: sash_count distinct N {1, 2}；meeting/muntin bar distinct N {0, 1, 2}
- **组合数 = 3 × 4 × 4 × 5 = 240 ≥ 10 ✓**（即便仅关节承载的 A×C = 3×4 = 12 ≥ 10 已单独过闸）。

## Slot 候选覆盖

### Slot A:fixed_glazing_topology（固定玻璃光的承载拓扑 — 关节承载主槽）
| 候选（未来 module） | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| separate_fixed_part（基线） | rec_sliding_window_0004 (model.py:L422-455) / 0005 (L340-425) / 66a60f / 948397 / cd856d31 / cf88dde9 / e7761bbe / f0621fda / fe157a5d / 14f5b1aa / 06ab752a / 254147d6 / 327b4610 / 3811a338 / 44c133de / 491508bb / 80a9c9 | `fixed_lite`/`fixed_sash`/`fixed_segment`/`fixed_panel`(独立 part) + `frame_to_fixed_*` **FIXED** joint | 固定光是独立 part 经 FIXED 挂上 frame；2-part(或更多) tree | converged(archetype) |
| baked_into_frame | rec_sliding_window_0001 (L235-251) / 0002 (L177-206) / 0003 (L224-321) / 3ab1c950 / 43f22f89 / 470474d8 / 5424dd87 / a87e7bd / a9564b40 / b801112c / c51ac99b / cc24515a / 5e1f21 / 62da20 / 720dbc / 7404a1 / 7fca8a / dc83a7ca / f0907748 | `fixed_glass`/`fixed_meeting_stile`/`fixed_mullion` 作 `frame` part 的 visual | 固定光烘焙进 frame；仅 1 可动 sash part + frame（无独立 fixed part） | converged(archetype) |
| sash_only_no_fixed | rec_sliding_window_b5bb4681 / cbfa0ab2 / 168d233e / 56eeb897 / dd9f8469 | 无 `fixed_*`；frame 只是周边 ring，单 sliding sash 占满开口 | 无固定光（service hatch / calibration / safety / 单玻璃舱口） | converged(archetype) |

### Slot B:track_carriage_style（滑轨 / 承载方式）
| 候选 | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| lipped_dual_channel（基线） | rec_sliding_window_0004 (`*_runner`/`*_outer_lip`/`*_center_lip` L334-370) / 0001 (`*_track_base`/`*_head_guide` L184-230) / 多数样本 | frame 双(或三)槽 box 轨道 + 上下 lip 唇 + sash glide-shoe 滑块 | box 唇槽里滑动（无圆轮） | converged(archetype) |
| roller_truck_cylinders | rec_sliding_window_7404a1 (`roller_0/1` Cylinder + brackets L114-127) / 0002 (L319-330) / 0003 (L436-453) / 5424dd87 / f0907748 / 3ab1c950 | `roller_*` **Cylinder**(横轴) + `roller_housing`/`carriage_block` + 钢轨 | 圆柱尼龙轮在轨上滚（roller-truck） | converged(archetype) |
| triple_rib_guide | rec_sliding_window_f0621fda (back/separator/front 3 肋 loop L646-674) / 06ab752a (L178-212) / 66a60f (L158-180) / 491508bb (3-track L81-102) | 三道导轨肋（rear/center/front）分隔前后滑道 + 嵌套 loop 发射 | 三轨槽（前固定光 / 后滑扇分道）via for-loop | converged(archetype) |
| cadquery_hollow_channel | rec_sliding_window_645606 (`_build_frame` CQ cut + drain-slot loop L56-64) / bdc4eb7a / eb85aa07 | CadQuery boolean 切空 frame/sash shell，轨道槽是切出的 | 实心 slab 布尔切轨道与开口（非 box 拼） | converged(archetype) |

### Slot C:lock_articulation（锁五金 — 关节承载主槽）
| 候选 | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| none_passive_visual（基线） | rec_sliding_window_0001 (`latch_thumbturn` visual L283) / 0002 (`keeper_latch`) / 0003 / 多数样本 | 锁/keeper 仅 visual，无独立 joint；或纯 interlock fin | 无 articulated 锁（passive 装饰 / keeper） | converged(archetype) |
| thumbturn_revolute | rec_sliding_window_0005 (`moving_sash_to_latch` **REVOLUTE axis(0,1,0)** L440-453; latch part L388-417) / 02310a01 (`sash_to_latch` REVOLUTE axis(0,1,0)) / 7fca8a(`thumb_latch` 命名) | 独立 `latch` part + REVOLUTE 拇指扳手（escutcheon+pivot_boss+thumbturn） | 真 articulated 拇指锁绕 Y 轴转 | converged(archetype) |
| crescent_cam_revolute | rec_sliding_window_43f22f89 (`latch_pivot` REVOLUTE axis(0,1,0) L247-256; `pivot_hub`+`latch_bar`+jamb `keeper_plate`) | crescent / quarter-turn cam latch + keeper plate | 月牙凸轮锁绕 Y 转入 keeper | converged(archetype) |
| lockout_pin_prismatic | rec_sliding_window_b5bb4681 (`frame_to_lock_pin` **PRISMATIC axis(0,0,1)** lift-out red T-handle) | 独立 `lock_pin` part 竖向 PRISMATIC 升降锁定销 | 工业 lift-out 锁定销（竖滑入孔） | converged(archetype) |

### Slot D:handle_style（把手样式）
| 候选 | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| pull_bar（基线） | rec_sliding_window_0002 (`pull_base`/`pull_grip` L344-355) / 0003 (`pull_plate`+`pull_grip` L475-490) / 0004 (`pull_handle`/`handle_rib` L220-238) / 327b4610 / 5a5b05 / 948397 | 凸出拉杆 / D-bar（base + grip 两段） | 表面凸出抓杆 | converged(archetype) |
| flush_recess | rec_sliding_window_0001 (`flush_pull` recess) / 02310a01 (`flush_pull_recess`+`pull_lip` L234-245) / 0f5d4168 (`flush_pull`) / 3ab1c950 | 嵌入式凹槽拉手（recess + lip + shadow slot） | 平嵌指槽（不凸出） | converged(archetype) |
| finger_pull | rec_sliding_window_43f22f89 (`finger_pull` L203-208) / 5e1f21 (`pull_lip`) / 56eeb897 (Cylinder finger pull L119-127) / b801112c / c51ac99b / 9d395aef | 小指拉条 / 指洞 | 极简指拉（窄条或洞） | converged(archetype) |
| molded_grip_ribs | rec_sliding_window_7404a1 (`grip_rib_{idx}` loop L93-94) | 模塑防滑肋（for-loop 发射 3 条肋） | 多条横肋抓握面（loop-emitted） | converged(archetype) |
| d_pull_plate | rec_sliding_window_a9564b40 (`pull_handle` bar + 2 mounts L956-958) / cc24515a (spline-tube `pull_handle` L983-1003) / 80a9c9 / dc83a7ca | 工业 D 形把手装在外凸 mount/plate 上 | 重型 D 把手 + 安装座（field-service） | converged(archetype) |

## Multiplicity / Copy Logic

- **一条轻 multiplicity 轴（sash 单元/分隔条）+ 多条 for-loop 发射的同构 hardware 子件**:
  - count_param `sash_count` — distinct N {1 (sash_only), 2 (sliding + fixed)}；样本无 2 sliding / 3+ sash（pool 全是单 sliding sash）。copied object = glazed panel(`_build_sash`/`_add_glazed_panel`/`_add_glazed_sash` 工厂复用建 fixed 与 sliding 两扇)；naming `fixed_*` vs `sliding_*`/`moving_*`；placement = 沿框宽分两半，meeting stile/interlock fin 在中央；joint policy = sliding sash 一个 PRISMATIC，fixed lite 一个 FIXED（或烘焙进 frame 无 joint）。来源 0004 L90-263 / 0005 L49-229。
  - count_param `meeting_or_muntin_bar` — distinct N {0, 1 (single meeting/interlock divider), 2 (muntin cross/竖条)}；为弱轴（样本几乎都 0-1，仅 dc83a7ca/dd9f8469 有 muntin）。copied object = `muntin_{i}` 竖/横条 via `for i in range(n)`；naming `muntin_{i}` / `meeting_stile`；placement = sash 内框均分；joint policy = 随所属 sash 运动，无独立 joint。来源 dc83a7ca L1928-1929 / dd9f8469 L68-79（手写，模板须改 for-loop）。
  - **for-loop 发射的 hardware**（非独立 multiplicity 轴，但必须 loop 发射）：rollers/glides/top-guides（0004 L176-201、0005 L180-193、7404a1 L114-127）、fasteners/screws（0004 L48-69、0002 L208-227）、track ribs（f0621fda L646-674、948397 L72-91）、bolt grids（a9564b40/b5bb4681/cc24515a）。
- fixed_glazing(A)、track(B)、lock(C)、handle(D) 为固定 named slot，非复制轴。

## 格子覆盖（archetype 基线计入）

- Slot A:3 候选全部由 archetype 覆盖（separate-fixed 17 条 / baked 19 条 / sash-only 5 条）。
- Slot B:4 候选全部覆盖（lipped 多数 / roller-truck 6 条 / triple-rib 4 条 / cadquery 3 条）。
- Slot C:4 候选覆盖（none 多数 / thumbturn 0005+02310a01+7fca8a / crescent 43f22f89 / lockout-pin b5bb4681）。
- Slot D:5 候选覆盖（pull-bar / flush-recess / finger-pull / grip-ribs 7404a1 / d-pull-plate）。
- 每槽 ≥3 候选；关节承载槽 A(3) 与 C(4) 各 ≥3。无 padding，全部 archetype 实证。

## 排除项（未来 compatibility matrix 素材）

- **vertical double-hung 不在 pool**:54 条全是 PRISMATIC 横滑（53 条 ±X，1 条 ±Y），**无任何竖向双悬 ±Z 滑窗**。`window_orientation` 实证只支持 horizontal;vertical 作为「设计但无源」扩展须 reviewer 决定是否纳入（否则 Slot 退化为单值，见 spec §拓扑审计）。
- **无 2-sliding-sash / 3+ sash**:全部单 sliding sash 对 1 fixed lite（或无 fixed）。`sliding_sash_count` 实证恒 = 1;>1 无源。
- **casement / awning / fixed / sliding_door 不混入**:本 slug identity = 横滑 PRISMATIC sash;铰链外开（REVOLUTE 主开启）属 `window` 小类;落地人行滑门属 `sliding_door`。
- sibling slug `sliding_window`（qwen fork pool）是并行模板,可重叠;本 slug 用 curated DATASET pool。
- 颜色 / 材质 / 纯尺寸**不作结构轴**(归 palette_style + 连续 scale)。
- 低 rating 边角(dd9f8469 ±Y wood rating2)仅作 muntin/colorway 证据,不作主候选;0001/0003 手写 rollers 需改 for-loop。
- 潜在不收敛风险待 fork 后回填:roller-truck 圆轮在轨上「接触」断言脆弱(参 MEMORY knob-spin:轴对称件 AABB 难判 → 用 expect_contact/expect_gap 而非 AABB);lockout_pin 竖滑与 sash 横滑两条 PRISMATIC 共存须各自 motion_limits 不串扰;cadquery_hollow 布尔在 segments 高时退化(参 roller-skate 教训 ≤56)。
