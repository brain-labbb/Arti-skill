# PV-A Table 4 corrected v2：531 类表现不佳清单

> 这份报告使用正式 Table 4 corrected v2 的 effective records。它不是 v3 companion，也没有把 v3 的 numerical-zero-margin 结果混入 v2。

## 正式 Table 4 口径

- Protocol：`urdf_sim_ready_table4_pva_full_release_v2_corrected_r1`。
- Cohort：Ours / PV-A full release，`N=302,440`，`J=1,453,516`，531 类。
- Effective records：v2 parent `pva_table4_mimic_aware_full_release_20260827` 加 847 条 corrected overlay 原子替换。
- 逐类表只使用 Table 4 的五个资产级 CF/pass 字段；`rest_pass_motion_fail` 等仅用于末尾的 supplementary 诊断，不冒充 Table 4 headline。
- 正式 receipt 没有封存 `category_macro`；下方逐类数据是绑定同一 effective records/roster 的只读重聚合。

## Table 4 overall headline

| Metric | Corrected v2 result |
|---|---:|
| Rest All-pair CF | 3,335 / 302,440 (1.103%) |
| Rest Non-adjacent CF | 217,613 / 302,440 (71.952%) |
| Single-joint Sweep CF | 199,157 / 302,440 (65.850%) |
| Multi-joint Sobol CF | 197,245 / 302,440 (65.218%) |
| Collision-state Rate | 14,543,876 / 48,090,121 (30.243%); executed 48,084,014; unexecuted 6,107 |
| AOR | N/E |
| Max Penetration | 0.845122 normalized (302,377 measured; PARTIAL) |
| Collision-free Range | 19,187,701 / 28,431,585 (67.487%) |
| Strict Collision Pass | 195,136 / 302,440 (64.521%) |

## 筛选阈值

- “明显差”：`Strict Collision Pass <25%`。
- `Strict Collision Pass =25%` 单列为边界。
- P0：`N≥500`；P1：`100≤N<500`；P2：`N<100`。

## 汇总

| 筛选 | 类别数 | 资产数 |
|---|---:|---:|
| strict = 0% | 110 | 52243 |
| strict <25% | 149 | 70671 |
| strict =25%（边界） | 1 | 52 |
| strict ≤25% | 150 | 70723 |
| strict <50% | 192 | 101739 |

## P0：大样本明显差（N ≥ 500，strict <25%）

| 类别 | N | Rest All-pair CF | Rest Non-adjacent CF | Single-joint Sweep CF | Multi-joint Sobol CF | Strict Collision Pass |
|---|---:|---:|---:|---:|---:|---:|
| `pictureX_0611_drawing_compass_with_adjustable_legs` | 3200 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `rack_and_pinion_slider` | 3200 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `standing_desk_with_synchronous_telescoping_legs_and_articulated_controls` | 3200 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `watch` | 3200 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `Technology_Audio_Device` | 2821 | 0 (0.00%) | 1883 (66.75%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `clock_tower_with_rotating_hour_and_minute_hands` | 2413 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `pictureX_0611_garlic_press` | 2286 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `Military_Rifle` | 1985 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `pictureX_0611_bevel_gear_pair_with_perpendicular_shafts` | 1881 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `sailboat_winch_with_pawl_and_handle` | 1881 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `Bench_Wood_Swing` | 1800 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `rolling_toolbox_with_telescoping_handle` | 1800 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `miter_saw_arm_assembly` | 1500 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `stand_mixer` | 1500 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `Door_folding_door` | 1056 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 11 (1.04%) | 0 (0.00%) |
| `Urban_Environment_Tilt_Truck2` | 1045 | 0 (0.00%) | 437 (41.82%) | 0 (0.00%) | 31 (2.97%) | 0 (0.00%) |
| `screwin_light_bulb_with_socket` | 958 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `Industrial_Mine_cart` | 935 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 66 (7.06%) | 0 (0.00%) |
| `pictureX_0611_ball_transfer_unit_with_spring_loaded_ball` | 914 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `telescoping_boom` | 712 | 0 (0.00%) | 23 (3.23%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `Chair_Folding_chair` | 653 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `Astronomy_Space_shuttle` | 627 | 0 (0.00%) | 104 (16.59%) | 50 (7.97%) | 0 (0.00%) | 0 (0.00%) |
| `Industrial_Ore_crusher_jaw` | 627 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `pictureX_0611_drum_pedal_with_beater_and_spring_return` | 627 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `Vehicle_Sports_car` | 588 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `pictureX_0611_gyroscope_with_spinning_wheel_and_gimbal_rings` | 557 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `retractable_patio_awning` | 557 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `globe` | 685 | 0 (0.00%) | 44 (6.42%) | 44 (6.42%) | 44 (6.42%) | 44 (6.42%) |
| `metronome` | 3200 | 0 (0.00%) | 1598 (49.94%) | 382 (11.94%) | 263 (8.22%) | 260 (8.12%) |
| `pictureX_0611_crimping_tool` | 940 | 0 (0.00%) | 77 (8.19%) | 77 (8.19%) | 77 (8.19%) | 77 (8.19%) |
| `rivet_squeeze` | 1045 | 0 (0.00%) | 824 (78.85%) | 690 (66.03%) | 97 (9.28%) | 97 (9.28%) |
| `wind_turbine` | 1200 | 0 (0.00%) | 175 (14.58%) | 143 (11.92%) | 134 (11.17%) | 129 (10.75%) |
| `Container_Barrel` | 2462 | 418 (16.98%) | 1634 (66.37%) | 413 (16.77%) | 414 (16.82%) | 413 (16.77%) |
| `twojoint_prismatic_chain` | 938 | 0 (0.00%) | 195 (20.79%) | 193 (20.58%) | 192 (20.47%) | 192 (20.47%) |
| `louvered_shutter_assembly` | 1498 | 0 (0.00%) | 399 (26.64%) | 341 (22.76%) | 340 (22.70%) | 339 (22.63%) |
| `turntable` | 1951 | 0 (0.00%) | 990 (50.74%) | 452 (23.17%) | 456 (23.37%) | 445 (22.81%) |

## P1：中等样本明显差（100 ≤ N < 500，strict <25%）

| 类别 | N | Rest All-pair CF | Rest Non-adjacent CF | Single-joint Sweep CF | Multi-joint Sobol CF | Strict Collision Pass |
|---|---:|---:|---:|---:|---:|---:|
| `Sports_Karting` | 490 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `Fountain_Drick_fountain` | 488 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `retractable_utility_knife` | 407 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `harmonic_drive` | 401 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `pictureX_0611_Butter_maker` | 372 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `Others_Safe` | 294 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `manual_pipe_bender` | 279 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `pictureX_0611_Hole_punch` | 279 | 0 (0.00%) | 115 (41.22%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `Urban_Environment_Manhole_cover` | 272 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `Others_Matchbox` | 242 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `pictureX_0611_Ice_crream_machine` | 235 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `pictureX_0611_folders` | 235 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `Astronomy_Pressurised_module_door` | 226 | 0 (0.00%) | 208 (92.04%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `pictureX_0611_juicer_press_with_handle` | 209 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `mechanical_timer_with_rotating_dial` | 199 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `threestage_telescoping_slide` | 186 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `manual_grain_mill2` | 174 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `pictureX_0611_Garden_pruner` | 174 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `Military_Turret` | 157 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `Technology_Remote_Control` | 157 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `Music_Violin_case` | 154 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 21 (13.64%) | 0 (0.00%) |
| `pictureX_0611_ergonomic_clamp_with_adjustable_components` | 139 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `dishwasher_with_dropdown_door_and_sliding_racks` | 131 | 0 (0.00%) | 66 (50.38%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `nutcracker` | 131 | 0 (0.00%) | 83 (63.36%) | 40 (30.53%) | 0 (0.00%) | 0 (0.00%) |
| `screwcap_bottle` | 131 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `pictureX_0611_car_scissor_jack_with_screw_mechanism` | 118 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `telescoping_fishing_rod` | 118 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `pictureX_0611_bi_fold_closet_door_system` | 109 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `pictureX_0611_compass` | 108 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `Music_keyboard` | 107 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `pilers_fencing_pliers` | 104 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `pilers_tongue_groove_pliers` | 104 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `pilers_wire_strippers` | 104 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `ironing_board` | 343 | 0 (0.00%) | 4 (1.17%) | 4 (1.17%) | 4 (1.17%) | 4 (1.17%) |
| `Container_Bottle` | 150 | 0 (0.00%) | 32 (21.33%) | 2 (1.33%) | 2 (1.33%) | 2 (1.33%) |
| `pictureX_0611_Air_blower` | 438 | 0 (0.00%) | 8 (1.83%) | 8 (1.83%) | 8 (1.83%) | 8 (1.83%) |
| `Art_Drawing_Models_Articulated_mannequin_Poseable_figure_mannequin` | 348 | 0 (0.00%) | 93 (26.72%) | 24 (6.90%) | 15 (4.31%) | 8 (2.30%) |
| `hand_operated_pasta_maker_with_rollers_and_crank` | 104 | 0 (0.00%) | 3 (2.88%) | 3 (2.88%) | 3 (2.88%) | 3 (2.88%) |
| `zippo_lighter` | 418 | 0 (0.00%) | 19 (4.55%) | 19 (4.55%) | 19 (4.55%) | 19 (4.55%) |
| `rotary_table_with_tilting_trunnion` | 418 | 0 (0.00%) | 289 (69.14%) | 44 (10.53%) | 46 (11.00%) | 44 (10.53%) |
| `makeup2` | 157 | 0 (0.00%) | 52 (33.12%) | 20 (12.74%) | 20 (12.74%) | 20 (12.74%) |
| `Stationary_Clipboard` | 104 | 0 (0.00%) | 14 (13.46%) | 14 (13.46%) | 14 (13.46%) | 14 (13.46%) |
| `Other_Built_in_oven` | 145 | 0 (0.00%) | 145 (100.00%) | 25 (17.24%) | 25 (17.24%) | 25 (17.24%) |
| `Door_Folding_gate` | 148 | 0 (0.00%) | 27 (18.24%) | 27 (18.24%) | 27 (18.24%) | 27 (18.24%) |
| `Sports_Bike` | 232 | 0 (0.00%) | 231 (99.57%) | 51 (21.98%) | 45 (19.40%) | 45 (19.40%) |
| `makeup3` | 209 | 0 (0.00%) | 97 (46.41%) | 46 (22.01%) | 134 (64.11%) | 46 (22.01%) |
| `Sports_game_console` | 218 | 0 (0.00%) | 63 (28.90%) | 55 (25.23%) | 53 (24.31%) | 53 (24.31%) |
| `tricycle` | 274 | 0 (0.00%) | 153 (55.84%) | 67 (24.45%) | 67 (24.45%) | 67 (24.45%) |

## P2：低样本观察（N < 100，strict <25%）

| 类别 | N | Rest All-pair CF | Rest Non-adjacent CF | Single-joint Sweep CF | Multi-joint Sobol CF | Strict Collision Pass |
|---|---:|---:|---:|---:|---:|---:|
| `simple_drying_rack` | 99 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `Military_knife` | 98 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `Technology_Keyboard` | 93 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `pictureX_0611_Hand_crank_clothes_wringer` | 93 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `telescoping_pointer` | 93 | 0 (0.00%) | 27 (29.03%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `Sports_Exercise_bike` | 87 | 0 (0.00%) | 63 (72.41%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `pictureX_0611_ergonomic_clamp_with_adjustable` | 87 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `ratchet_strap` | 87 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `pictureX_0611_hydraulic_jack` | 78 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `pictureX_0611_hydraulic_jack1` | 78 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `Electrical_Wiring_Wire_stripper` | 75 | 0 (0.00%) | 41 (54.67%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `ring_blinder` | 70 | 0 (0.00%) | 60 (85.71%) | 8 (11.43%) | 0 (0.00%) | 0 (0.00%) |
| `pilers_linesman_pliers` | 65 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `pictureX_0611_Kitchen_set` | 64 | 0 (0.00%) | 64 (100.00%) | 0 (0.00%) | 16 (25.00%) | 0 (0.00%) |
| `pictureX_0611_flaring_tool_with_cone_and_clamp` | 64 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `Curtain_blind` | 63 | 0 (0.00%) | 32 (50.79%) | 14 (22.22%) | 0 (0.00%) | 0 (0.00%) |
| `Bag_Suitcase_Treasure_chest` | 60 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 30 (50.00%) | 0 (0.00%) |
| `Handtools_Wrench` | 60 | 0 (0.00%) | 15 (25.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `Kitchen_Knife_set` | 59 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `Container_Dispenser` | 58 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `Headwear_Racing_helmet` | 58 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `Technology_Laptop` | 54 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `single_wheelbarrow` | 54 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `pictureX_0611_guitar_tuning_peg_mechanism` | 53 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `Powertools_drill` | 52 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `Urban_Environment_Fire_Hydrant` | 52 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `pilers_needle_nose_pliers` | 52 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `pilers_slip_joint_pliers` | 52 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `turnbuckle` | 52 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `Container_Paint_spray` | 48 | 0 (0.00%) | 9 (18.75%) | 2 (4.17%) | 0 (0.00%) | 0 (0.00%) |
| `Military_Aircraft` | 48 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `rotating_observatory_dome` | 48 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `tube_cutter` | 48 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `Military_Gun` | 47 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `Healthcare_First_aid_box` | 40 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `Military_Granade` | 40 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `Technology_Mobile_Phone` | 40 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `Accessories_glasses` | 36 | 0 (0.00%) | 36 (100.00%) | 14 (38.89%) | 0 (0.00%) | 0 (0.00%) |
| `Sports_Table_football` | 36 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `worm_gear_and_wheel_assembly` | 36 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `pilers_locking_pliers` | 32 | 0 (0.00%) | 6 (18.75%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `Kitchen_Corkscrew` | 30 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `Handtools_Hand_plane` | 27 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `Powertools_Lawn_mower` | 22 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `Handtools_clothes_peg` | 20 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `hole_punch` | 20 | 0 (0.00%) | 12 (60.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `sewing_machine` | 20 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `universal_joint` | 20 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `Bar_Piano` | 11 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `protractor_with_swing_arm` | 7 | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) | 0 (0.00%) |
| `Handtools_caulking_gun` | 48 | 0 (0.00%) | 1 (2.08%) | 1 (2.08%) | 5 (10.42%) | 1 (2.08%) |
| `Urban_Environment_Public_toilet1` | 20 | 0 (0.00%) | 1 (5.00%) | 1 (5.00%) | 1 (5.00%) | 1 (5.00%) |
| `manual_coffee_grinder` | 52 | 0 (0.00%) | 4 (7.69%) | 3 (5.77%) | 3 (5.77%) | 3 (5.77%) |
| `automotive_differential_differential_gear` | 70 | 0 (0.00%) | 5 (7.14%) | 5 (7.14%) | 5 (7.14%) | 5 (7.14%) |
| `Other_Lighter` | 36 | 0 (0.00%) | 3 (8.33%) | 3 (8.33%) | 3 (8.33%) | 3 (8.33%) |
| `Container_Shipping_container` | 91 | 0 (0.00%) | 26 (28.57%) | 26 (28.57%) | 10 (10.99%) | 10 (10.99%) |
| `Other_pliers` | 87 | 0 (0.00%) | 10 (11.49%) | 10 (11.49%) | 10 (11.49%) | 10 (11.49%) |
| `Other_Folding_screen` | 81 | 0 (0.00%) | 81 (100.00%) | 45 (55.56%) | 10 (12.35%) | 10 (12.35%) |
| `Container_Lipstick` | 28 | 0 (0.00%) | 16 (57.14%) | 4 (14.29%) | 4 (14.29%) | 4 (14.29%) |
| `Industrial_Electric_Saw` | 36 | 0 (0.00%) | 6 (16.67%) | 6 (16.67%) | 6 (16.67%) | 6 (16.67%) |
| `Science_Dental_setup` | 59 | 0 (0.00%) | 28 (47.46%) | 10 (16.95%) | 10 (16.95%) | 10 (16.95%) |
| `Kitchen_Dish_washer` | 20 | 0 (0.00%) | 20 (100.00%) | 4 (20.00%) | 4 (20.00%) | 4 (20.00%) |
| `Others_Binocular` | 20 | 0 (0.00%) | 4 (20.00%) | 4 (20.00%) | 4 (20.00%) | 4 (20.00%) |
| `trekking_pole_collapsible` | 59 | 0 (0.00%) | 13 (22.03%) | 14 (23.73%) | 17 (28.81%) | 13 (22.03%) |
| `Other_armchair` | 96 | 0 (0.00%) | 29 (30.21%) | 23 (23.96%) | 24 (25.00%) | 23 (23.96%) |

## 边界类（strict =25%）

| 类别 | N | Rest All-pair CF | Rest Non-adjacent CF | Single-joint Sweep CF | Multi-joint Sobol CF | Strict Collision Pass |
|---|---:|---:|---:|---:|---:|---:|
| `pictureX_0611_Chain_separator` | 52 | 0 (0.00%) | 35 (67.31%) | 13 (25.00%) | 22 (42.31%) | 13 (25.00%) |

## Warning：大样本但 25% ≤ strict <50%

| 类别 | N | Rest All-pair CF | Rest Non-adjacent CF | Single-joint Sweep CF | Multi-joint Sobol CF | Strict Collision Pass |
|---|---:|---:|---:|---:|---:|---:|
| `robotic_arms` | 3200 | 0 (0.00%) | 1932 (60.38%) | 807 (25.22%) | 807 (25.22%) | 807 (25.22%) |
| `wall_safe_with_hinged_door_and_dial` | 3200 | 0 (0.00%) | 1621 (50.66%) | 838 (26.19%) | 825 (25.78%) | 824 (25.75%) |
| `usb_drive_with_swivel_cover` | 3200 | 0 (0.00%) | 1341 (41.91%) | 948 (29.62%) | 890 (27.81%) | 841 (26.28%) |
| `wheelbarrow` | 697 | 0 (0.00%) | 189 (27.12%) | 189 (27.12%) | 189 (27.12%) | 189 (27.12%) |
| `Door_Garage_shutter` | 3019 | 1242 (41.14%) | 1400 (46.37%) | 1400 (46.37%) | 1524 (50.48%) | 870 (28.82%) |
| `wheelie_bin_with_hinged_lid` | 705 | 0 (0.00%) | 208 (29.50%) | 208 (29.50%) | 208 (29.50%) | 208 (29.50%) |
| `Door_Other` | 3200 | 0 (0.00%) | 1055 (32.97%) | 1055 (32.97%) | 1055 (32.97%) | 1055 (32.97%) |
| `defibrillator_case` | 792 | 0 (0.00%) | 535 (67.55%) | 273 (34.47%) | 273 (34.47%) | 273 (34.47%) |
| `Other_Tripod_Turnstile` | 3200 | 0 (0.00%) | 2401 (75.03%) | 1216 (38.00%) | 1216 (38.00%) | 1216 (38.00%) |
| `camcorder_with_flipout_screen` | 3200 | 0 (0.00%) | 3200 (100.00%) | 2241 (70.03%) | 1237 (38.66%) | 1237 (38.66%) |
| `Urban_Environment_utility_box` | 3200 | 0 (0.00%) | 3199 (99.97%) | 1307 (40.84%) | 1307 (40.84%) | 1307 (40.84%) |

## Supplementary：运动后失败诊断（不是 Table 4 headline）

| 类别 | N | Rest 通过后运动失败 | 比例 | Strict Collision Pass |
|---|---:|---:|---:|---:|
| `Astronomy_Pressurised_module_door` | 226 | 208 | 92.04% | 0 (0.00%) |
| `Other_Built_in_oven` | 145 | 120 | 82.76% | 25 (17.24%) |
| `Sports_Bike` | 232 | 186 | 80.17% | 45 (19.40%) |
| `rivet_squeeze` | 1045 | 727 | 69.57% | 97 (9.28%) |
| `simple_aframe_step_ladder` | 235 | 157 | 66.81% | 78 (33.19%) |
| `Technology_Audio_Device` | 2821 | 1883 | 66.75% | 0 (0.00%) |
| `nutcracker` | 131 | 83 | 63.36% | 0 (0.00%) |
| `camcorder_with_flipout_screen` | 3200 | 1963 | 61.34% | 1237 (38.66%) |
| `Urban_Environment_utility_box` | 3200 | 1892 | 59.12% | 1307 (40.84%) |
| `rotary_table_with_tilting_trunnion` | 418 | 245 | 58.61% | 44 (10.53%) |
| `dishwasher_with_dropdown_door_and_sliding_racks` | 131 | 66 | 50.38% | 0 (0.00%) |
| `Container_Barrel` | 2462 | 1221 | 49.59% | 413 (16.77%) |
| `tackle_box_with_simple_hinged_lid` | 1200 | 594 | 49.50% | 606 (50.50%) |
| `Urban_Environment_Tilt_Truck2` | 1045 | 437 | 41.82% | 0 (0.00%) |
| `metronome` | 3200 | 1338 | 41.81% | 260 (8.12%) |
| `Door_Sliding_Door` | 302 | 125 | 41.39% | 172 (56.95%) |
| `pictureX_0611_Hole_punch` | 279 | 115 | 41.22% | 0 (0.00%) |
| `Other_Tripod_Turnstile` | 3200 | 1185 | 37.03% | 1216 (38.00%) |
| `robotic_arms` | 3200 | 1125 | 35.16% | 807 (25.22%) |
| `Music_Headphone` | 274 | 93 | 33.94% | 181 (66.06%) |

## 覆盖率/状态例外

- `Door_Folding_gate`：148 assets；completed=85，error=63；native collision geometry=87，collision-measured=85。

`Door_Folding_gate` 的 63 个 error 是当前 PyBullet 高关节状态容量问题，release 分母中保留；`single_wheelbarrow` 在 corrected v2 中已完成测量，不再按 v3 的 package-binding 异常处理。

## 验证与来源

- Effective record aggregate 与 corrected v2 receipt 五个 Table 4 资产级 pass 计数逐项一致：{'rest_all_pair_cf': 3335, 'rest_non_adjacent_cf': 217613, 'single_joint_sweep_cf': 199157, 'multi_joint_sobol_cf': 197245, 'strict_collision_pass': 195136}。
- Effective records SHA256：`8f23852ba63758690d6b7a740a9ae3f6f1e31e9a85d4395fc5e719b69c4d4e4d`。
- Overlay SQLite SHA256：`7b42df08faf1e93de321ccff01a362c31b025680251d825f5ebb17a20720290a`。
- 原始 Table 4 corrected v2 receipt：[full_release_receipt.json](pva_table4_v2_targeted_correction_20260828/full_release_receipt.json)。
- 原始 Table 4 主文档：[URDF-Sim-Ready-Automatic-Evaluation.md](../URDF-Sim-Ready-Automatic-Evaluation.md)。
- v3 companion（仅作对照，不可与本表混用）：[pva_table4_poor_categories_v3.md](pva_table4_poor_categories_v3.md)。

