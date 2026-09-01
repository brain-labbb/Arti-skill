# Source Map — Agricultural / Greenhouse vent roof

slug `ghvent` · pattern **mixed**（`roof_frame` 挂 multiplicity 玻璃格 `glass_{ci}_{ri}` +
parallel 开启机构；每 sash 短链 sash→stay_arm/latch）。origin 只有 1 个 sash——sash 计数
multiplicity 需 loop 重写(见下)。

## Origins（全量对账，1/1 上格）
| id | pic | 建成形态 | 网格角色 |
|---|---|---|---|
| A `rec_use-the-attached-reference-image-as-the-primary-_20260625_155418_892757_32a39adc` | 001 | 单坡铝框玻璃屋顶 + 1 顶铰 `vent_sash`(revolute `roof_to_vent_sash`) + `stay_arm` + `latch_handle`(共 3 非固定关节)；3×2 玻璃格 loop | roof=mono_pitch / vent=top_hinged / glazing=multi_pane / frame=aluminium |

## Slots
- **A roof_geometry（③ Volumetric Envelope）**：mono_pitch(A) / even_span(fork) / curved_eave(fork)
- **B vent_mechanism（①/② 主轴）**：top_hinged_prop(A) / louvre_bank(fork) / sliding_panel(fork,prismatic) / ridge_flap(fork)
- **C glazing（③ Planar/Macro）**：multi_pane_grid(A) / single_pane(fork) — 可外推 diamond-lap
- **D frame_member（③ Macro/⑥）**：aluminium_box(A) / timber_bar(fork) — 可外推 steel_tube
- **N**：vent sashes ×N {1(A),2,3}(loop 重写)；louvre blades ×N(6)；glass panes 格(loop，模板)

## Slot 候选覆盖
### Slot A：roof_geometry
| mono_pitch(origin) | A | converged |
| even_span(ridge+两坡) | rec_ghvent_var_roof_span | converged |
| curved_eave(loft/lathe 拱檐) | rec_ghvent_var_roof_curved_eave | converged |
### Slot B：vent_mechanism
| top_hinged_prop(origin, revolute+stay+latch) | A | converged |
| louvre_bank(`louvre_blade_{i}` ×6, 各 revolute) | rec_ghvent_var_vent_louvre | converged |
| sliding_panel(`roof_to_sliding_vent` prismatic) | rec_ghvent_var_vent_sliding | converged |
| ridge_flap(`roof_to_ridge_flap` revolute 沿脊) | rec_ghvent_var_vent_ridge_flap | converged |
### Slot C：glazing
| multi_pane_grid(origin) | A | converged |
| single_pane(大玻璃 rafter-to-rafter) | rec_ghvent_var_glazing_single_pane | converged |
### Slot D：frame_member
| aluminium_box(origin) | A | converged |
| timber_bar(+putty bead) | rec_ghvent_var_frame_timber | converged |

## Multiplicity / Copy Logic
- vent sashes: count_param `n_sashes` — `_build_vent_sash(i)` → `vent_sash_{i}` + 独立 `roof_to_vent_sash_{i}` revolute + `stay_arm_{i}` + `latch_{i}`，跨坡平铺；**origin 是单 sash 手写，需 loop 重写**；N {1,2,3} → A / rec_ghvent_var_sash_x2 / rec_ghvent_var_sash_x3；模板 N_range [1,6]
- louvre blades: `louvre_blade_{i}` N=6，各 revolute，模板 N_range [3,10]
- glass panes: `glass_lower_{ci}_{ri}` 已 loop-clean(共享 `_plane_box`, FIXED)；模板 rows [1,4]×cols [2,6]

## 视觉多样性 6 轴考察
| 轴 | 处理 | 取值 |
|---|---|---|
| ① 骨架图 | forked_anchor | roof 3 + vent 4 + glazing 2 + frame 2；sash N{1,2,3}、louvre N=6 |
| ② 关节类型 | forked_anchor | sash/louvre/ridge-flap revolute、sliding-panel prismatic |
| ③ 主体形态家族 | forked_anchor + world_knowledge_extrapolation | roof geom + glazing pattern；可外推 diamond-lap glass |
| ④ 表面装饰 | record_only + world_knowledge_extrapolation | W 玻璃夹、bar cap、hinge bracket、turnbuckle、screw;可外推 EPDM/putty bead |
| ⑤ 尺寸/行程 | record_only | pitch ~24°(0.35-0.52)；sash 0→1.05、louvre 0→1.4、slider 0→0.6m |
| ⑥ 涂装 | record_only | mill aluminium / white / green / timber / galvanized / anthracite / black + clear|tinted glass |

## 排除项
- diamond-lap horticultural glazing + steel-tube frame — 留模板外推，非失败
