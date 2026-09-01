# Sports / Table football — template source map

pattern: multiplicity (rods × N are the dominant copy axis; figures-per-rod is a nested second multiplicity; cabinet/leg and grip are fixed named slots)
parents: rec_foosball-table-black-cabinet-with-rounded-corner_20260611_160945_143790_5586ac02 ← picture/Sports/Table football/001.png
  (single parent; fills one cell in every slot: 8 rods, box-cabinet-on-4-splayed-legs, plain cylinder grip, mixed 1/2/3/5 figures-per-rod)

## Slot 候选覆盖

### Slot A:player_rod_count (PRIMARY multiplicity — rods replicated by ROD_CONFIGS for-loop)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| rods_8 (parent) | rec_foosball-table-black-cabinet-with-rounded-corner_20260611_160945_143790_5586ac02 | rod_{idx} / rod_{idx}_carrier / rod_{idx}_slide(prismatic y) / rod_{idx}_spin(continuous y) | full eight-rod table, per-team 1-2-5-3 layout (GK/D/M/A) | converged (parent) |
| rods_6 | rec_foosball_table_var_rods6 | rod_{idx} / rod_{idx}_slide / rod_{idx}_spin (6-entry config table) | standard six-rod home table, per-team GK + 2-fig D + 3-fig M/A | built ✓ |
| rods_4 | rec_foosball_table_var_rods4 | rod_{idx} / rod_{idx}_slide / rod_{idx}_spin (4-entry config table) | compact four-rod mini table, per-team GK + combined outfield rod | built ✓ |

### Slot B:cabinet_leg_form (under-cabinet support structure)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| splayed_legs_4 (parent) | rec_foosball-table-black-cabinet-with-rounded-corner_20260611_160945_143790_5586ac02 | leg_{i} (4 flat boxes, LEG_TILT splay, inlined cab visuals) | four white flat rectangular legs splayed outward | converged (parent) |
| side_panels_full_height | rec_foosball_table_var_panellegs | ground_panel_{i} (2 full-height side panels to floor) | solid arcade-style side panels reach ground, no separate legs | built ✓ |
| cross_x_trestles | rec_foosball_table_var_crosslegs | x_trestle_{i} (2 crossed-bar X frames + brace, per end) | two crossed X-frame leg trestles under each cabinet end | built ✓ |
| tubular_legs_4 | rec_foosball_table_var_tubelegs | leg_tube_{i} (4 vertical cylinder tubes + leveling foot disc) | four plumb round tubular legs with round leveling feet | built ✓ |

### Slot C:rod_handle_grip_form (grip slipped over team-side rod end)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| plain_cylinder_grip (parent) | rec_foosball-table-black-cabinet-with-rounded-corner_20260611_160945_143790_5586ac02 | rod_{idx}.handle (Cylinder on team-side rod end) | plain straight cylindrical grip | converged (parent) |
| contoured_waisted_grip | rec_foosball_table_var_contourgrip | rod_{idx}.handle (lathe/revolved waisted barrel profile) | ergonomic pinched-center flared-end lathe grip | built ✓ |
| ball_knob_grip | rec_foosball_table_var_ballgrip | rod_{idx}.handle_stem + handle_ball (stem + sphere) | classic stem-and-ball knob grip | built ✓ |

### Slot D:figures_per_rod (nested SECONDARY multiplicity — per-rod inner figure loop)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| mixed_figures_1_2_3_5 (parent) | rec_foosball-table-black-cabinet-with-rounded-corner_20260611_160945_143790_5586ac02 | player_{j+1}_torso/_head/_legs/_foot (n_fig per ROD_CONFIGS) | realistic mixed GK1 / D2 / M5 / A3 figure counts | converged (parent) |
| uniform_figures_3 | rec_foosball_table_var_figures3 | player_{j+1}_* (n_fig = 3 on every rod) | uniform three figures on every rod | built ✓ |

## Multiplicity / Copy Logic
- count_param (primary): rod_count (ROD_CONFIGS length) ;同构子件 = one player rod assembly (carrier + rod + figures + slide + spin)
- count_param (secondary, nested): figures_per_rod (n_fig per rod) ;同构子件 = molded player figure (torso+head+legs+foot block)
- N 样本已覆盖 (primary rod_count): {4, 6, 8} → rec_foosball_table_var_rods4 / rec_foosball_table_var_rods6 / parent
- N 样本已覆盖 (secondary figures_per_rod): parent mixed {1,2,3,5} + uniform {3} → parent / rec_foosball_table_var_figures3
- 模板建议 N_range: rod_count ∈ [2, 8] even counts (real foosball is symmetric per team, almost always 4/6/8); figures_per_rod ∈ [1, 5] per rod
- copied object: one rod assembly = massless carrier link + steel rod + team-side handle grip + n_fig molded figures
- naming: rod_{idx} / rod_{idx}_carrier; inner figures player_{j+1}_torso/_head/_legs/_foot
- placement: rods evenly spaced along X across the table; figures evenly spaced along the rod axis (Y), centered, hanging along -Z at q=0
- joint policy: EVERY rod keeps TWO non-fixed joints — rod_{idx}_slide (PRISMATIC, axis Y, +/-travel) on the carrier + rod_{idx}_spin (CONTINUOUS, axis Y) on the rod; uniform across all N rods; this two-DOF spine is immutable and must survive every slot swap

## 排除项(未来 compatibility matrix 素材)
- (none — all 8 cells forked, built & workbench-bound)
- watch: cross_x_trestles (Slot B) must clear the goal slots / score posts at the cabinet ends; if the X brace fouls the goal mouth, retreat to a simpler single-crossbar trestle and note here.
- watch: at rods_4 the combined-outfield rod must keep figure swing radius < (ROD_Z - PITCH_TOP) and adjacent-rod clearance as in parent run_tests; wider rod spacing at N=4 makes this easy, but verify the figure spacing test still passes.

---
## Post-fork verification (SEGMENT 1 complete)
All planned variants forked via `articraft fork` (dashscope qwen3.7-max, thinking medium), then verified on-disk: last compile = success, ≥1 non-fixed joint present, collections=['workbench'] (workbench-only, not promoted), and picture.json bound into the correct `Sports__<小类>` subcat shard (reconcile rebuilt). Status cells above flipped planned→built ✓ accordingly. Ready for SEGMENT 2 (spec authoring).
