# Sports / Karting — template source map

pattern: mixed (parallel_children for the 4 wheels + steering; multiplicity for the lateral cross-tubes)
parents: rec_racing-go-kart-with-a-tubular-steel-frame-a-sing_20260605_165903_685895_dbb0d663 ← picture/Sports/Karting/001.png (covers: Slot A=open_tubular, Slot B=molded_bucket, Slot C=round_3spoke, Slot D=bare_axle, cross-tubes baseline = 2 hand-written)

Core articulation (must be preserved by every variant, never an axis):
- 4 wheels CONTINUOUS roll about local X (front wheels are children of REVOLUTE steering knuckles; rear wheels children of chassis).
- 2 front steering knuckles REVOLUTE about vertical Z (the steering joint).
- steering wheel CONTINUOUS spin about its angled column axis.
Each variant changes exactly one structural slot below and keeps all of these joints.

## Slot 候选覆盖

### Slot A:frame_chassis_form (整体框架/车身形态)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| open_tubular (parent) | rec_racing-go-kart-with-a-tubular-steel-frame-a-sing_20260605_165903_685895_dbb0d663 | chassis: left_main_rail / right_main_rail / center_spine / front_cross_tube / rear_cross_tube (tube_from_spline_points) + left_side_pod/right_side_pod (superellipse_side_loft) | exposed welded tube rails + slim pink side pods, tubes visible, low ride height | converged (parent) |
| bodywork_shroud | rec_go_kart_var_bodywork | chassis: one continuous fairing skin replacing front_fairing+side_pods+rear_fairing (single superellipse_side_loft body), tube rails hidden | full CIK molded plastic body shell wrapping the lower chassis | built ✓ |
| flat_deck | rec_go_kart_var_flatdeck | chassis: floor_deck_pan (wide sheet Box/loft) + perimeter_bumper_rail loop (tube_from_spline_points all the way around) | flat sheet-metal rental-kart deck with wraparound bumper rail, no fore-aft tube rails+pods | built ✓ |
| offroad_buggy | rec_go_kart_var_buggy | chassis: roll_cage_hoop (arched spline tube over seat tying to side rails) + raised side rails | raised tubular roll-cage buggy, exposed welded tubes, higher ride height | built ✓ |

### Slot B:seat_form (座椅形态)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| molded_bucket (parent) | rec_racing-go-kart-with-a-tubular-steel-frame-a-sing_20260605_165903_685895_dbb0d663 | seat: seat_pan / seat_back / left_bolster / right_bolster (superellipse_side_loft); seat_mount FIXED | deep single molded bucket with side bolsters and mid backrest | converged (parent) |
| flat_sling | rec_go_kart_var_flatsling | seat: shallow seat_pan + low seat_back, no tall bolsters | thin low flat sling/pan seat close to the floor | built ✓ |
| high_back_shell | rec_go_kart_var_highback | seat: tall seat_back lofted to head height + forward-wrapping shoulder wings | full-wrap high-back racing shell seat | built ✓ |

### Slot C:steering_wheel_form (方向盘+柱形态)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| round_3spoke (parent) | rec_racing-go-kart-with-a-tubular-steel-frame-a-sing_20260605_165903_685895_dbb0d663 | steering_wheel: steering_rim (LatheGeometry torus) / steering_hub / steering_spoke_* ×3 / steering_marker; steering_spin CONTINUOUS | round rim, 3 radial spokes, on angled column | converged (parent) |
| butterfly_open | rec_go_kart_var_butterfly | steering_wheel: D-cut flat-top rim (upper arc removed), 2 side grips, 2 horizontal spokes, open center; steering_spin CONTINUOUS kept | open butterfly / cutaway wheel | built ✓ |
| quick_release_hub | rec_go_kart_var_qrhub | steering_wheel: qr_boss stacked cylinders between column top and rim, flat-bottom rim; steering_spin CONTINUOUS spins wheel+boss together | flat-bottom wheel proud on tall splined quick-release hub | built ✓ |

### Slot D:rear_drive_form (发动机/后驱动形态 — 母资产缺失,真实物体右后有引擎)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| bare_axle (parent) | rec_racing-go-kart-with-a-tubular-steel-frame-a-sing_20260605_165903_685895_dbb0d663 | chassis: rear_axle_bar (Cylinder, exposed live axle), no engine | exposed live rear axle, no engine block | converged (parent) |
| side_engine_pod | rec_go_kart_var_sideengine | chassis: engine_block + cylinder_fin_* (stacked lathed fin discs) + exhaust_header (curved tube) + fuel_tank, right-rear, parent visuals | single-cylinder finned engine pod with exhaust header | built ✓ |
| chain_sprocket_drive | rec_go_kart_var_chaindrive | chassis: rear_sprocket (disc keyed on axle) + engine_sprocket + clutch_drum + drive_chain (thin tube ring wrapping both), parent visuals | exposed chain-and-sprocket drivetrain on the live axle | built ✓ |

## Multiplicity / Copy Logic
- count_param: cross_tube_count(底盘横向桥接管数;parent 当前是 2 根手写 front_cross_tube/rear_cross_tube + 2 根 ad-hoc brace,未循环化)
- N 样本已覆盖: {2 (parent baseline, 手写未循环), 6} → parent / rec_go_kart_var_crosstubes6
  - 注:crosstubes6 变体的 prompt 显式要求把横管重写为 for-i-in-range 循环发射 cross_tube_0..5,补上 parent 的循环化缺口(parent 横管不是循环写的)。模板侧可用此样本读出 copy logic。
- 模板建议 N_range: [2, 10](横管 ladder 的合理范围;采样域远大于样本覆盖值属正常)
- copied object: 一根贯穿左右 side rail 的 spline 横管(shared tube_from_spline_points helper)
- naming: cross_tube_{i}
- placement: 前后轴之间等距 fore-aft Y 站位(regular spacing between FRONT_AXLE_Y and REAR_AXLE_Y)
- joint policy: 全部作为 chassis 的 visual,无独立 joint(随 chassis 动);属 parent-visual 复制,不是 jointed multiplicity

## 组合预审 / 规模
- Π(各槽候选数) × N 样本数 = A4 × B3 × C3 × D3 × (cross-tube N 2 样本) → 远大于 10;最小独立子集 A4 × C3 = 12 ≥ 10。
- 待填格子 cells = (A:4-1=3) + (B:3-1=2) + (C:3-1=2) + (D:3-1=2) + (cross-tube N:2-1=1) = 10。
- 计划变体数 = 10(单 parent;一格一个变体,无组合枚举,无重复格子)。

## 变体清单(10)
- rec_go_kart_var_bodywork — Slot A bodywork_shroud
- rec_go_kart_var_flatdeck — Slot A flat_deck
- rec_go_kart_var_buggy — Slot A offroad_buggy
- rec_go_kart_var_flatsling — Slot B flat_sling
- rec_go_kart_var_highback — Slot B high_back_shell
- rec_go_kart_var_butterfly — Slot C butterfly_open
- rec_go_kart_var_qrhub — Slot C quick_release_hub
- rec_go_kart_var_sideengine — Slot D side_engine_pod
- rec_go_kart_var_chaindrive — Slot D chain_sprocket_drive
- rec_go_kart_var_crosstubes6 — multiplicity cross_tube_count N=6 (loop-化)

## 跨层接口面(供 spec InterfaceSpec 预填)
- 4× wheel ↔ knuckle/chassis：轴线 mating(local X spin 轴),front wheel 经 knuckle(vertical Z steer)间接挂 chassis;allow_overlap 已声明(hub/axle 穿 frame tube)。
- steering_wheel ↔ chassis：column_lower_mount 套筒接 steering_column,沿倾斜柱轴 expect_overlap;steering_spin 轴 = 柱轴。
- seat ↔ chassis：seat_tray 上表面 FIXED mount,seat 局部 z=0 = 座底。
- Slot D 引擎/链轮 ↔ chassis：挂在 rear_cross_tube / rear_axle_bar 右侧真实面上(parent visuals,无 joint)。

## 排除项(未来 compatibility matrix 素材)
- 暂无(规划阶段)。已知潜在风险待 fork 后回填:
  - flat_deck × 现有 side_pod 装饰可能冲突(deck 取代了 pod;flat_deck 变体须移除 pod 装饰,否则穿插)。
  - offroad_buggy roll_cage_hoop 锚定:hoop 必须落在 side rail 真实面上,不可悬空(否则 disconnected island)。
  - quick_release_hub 高 boss 可能让 steering_marker 远离柱轴,steering-spin run_tests 的 moved 阈值须仍 > 0.05。

---
## Post-fork verification (SEGMENT 1 complete)
All planned variants forked via `articraft fork` (dashscope qwen3.7-max, thinking medium), then verified on-disk: last compile = success, ≥1 non-fixed joint present, collections=['workbench'] (workbench-only, not promoted), and picture.json bound into the correct `Sports__<小类>` subcat shard (reconcile rebuilt). Status cells above flipped planned→built ✓ accordingly. Ready for SEGMENT 2 (spec authoring).
