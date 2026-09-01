# PV-A v3 531 类表现不佳清单

> **版本警告：** 本文件是 `urdf_sim_ready_table4_pva_full_release_v3` 的 protocol-only companion，不是当前正式 Table 4 corrected v2 主表的逐类清单。正式 Table 4 corrected v2 请使用 [pva_table4_corrected_v2_poor_categories.md](pva_table4_corrected_v2_poor_categories.md)。

## 数据与口径

- 来源协议：`urdf_sim_ready_table4_pva_full_release_v3`。
- 来源文件：`pva_table4_status_aware_metrics_v2/category_metrics.json`。
- 类别数：531；release 资产数：302,440。
- 主指标：`strict_collision_pass / release assets`。strict 要求静止非相邻、单关节 sweep、联合 Sobol 三项均通过；未观测或保留错误按 release fail-closed 计。
- `rest_all_pair` 只作接触诊断，不用于筛选坏类，因为它包含 intended adjacent contact。
- 本文使用 `<25%` 作为明显差；恰好 `25%` 的类别单列为边界。

## 总览

| 筛选 | 类别数 | 资产数 |
|---|---:|---:|
| strict = 0% | 95 | 41307 |
| strict <25% | 131 | 61305 |
| strict =25%（边界） | 1 | 104 |
| strict <50% | 172 | 91870 |

## P0：大样本明显差（N ≥ 500，strict <25%）

| 类别 | 通过 / 资产 | 严格通过率 | rest 非相邻 | 单关节 | Sobol | rest 通过后运动失败 |
|---|---:|---:|---:|---:|---:|---:|
| `standing_desk_with_synchronous_telescoping_legs_and_articulated_controls` | 0 / 3200 | 0.00% | 0 | 0 | 0 | 0 |
| `watch` | 0 / 3200 | 0.00% | 0 | 0 | 0 | 0 |
| `clock_tower_with_rotating_hour_and_minute_hands` | 0 / 2413 | 0.00% | 0 | 0 | 0 | 0 |
| `pictureX_0611_garlic_press` | 0 / 2286 | 0.00% | 0 | 0 | 0 | 0 |
| `Military_Rifle` | 0 / 1985 | 0.00% | 0 | 0 | 0 | 0 |
| `pictureX_0611_bevel_gear_pair_with_perpendicular_shafts` | 0 / 1881 | 0.00% | 0 | 0 | 0 | 0 |
| `sailboat_winch_with_pawl_and_handle` | 0 / 1881 | 0.00% | 47 | 0 | 0 | 47 |
| `Bench_Wood_Swing` | 0 / 1800 | 0.00% | 0 | 0 | 0 | 0 |
| `rolling_toolbox_with_telescoping_handle` | 0 / 1800 | 0.00% | 0 | 0 | 0 | 0 |
| `miter_saw_arm_assembly` | 0 / 1500 | 0.00% | 0 | 0 | 0 | 0 |
| `stand_mixer` | 0 / 1500 | 0.00% | 0 | 0 | 0 | 0 |
| `Door_folding_door` | 0 / 1056 | 0.00% | 0 | 0 | 11 | 0 |
| `Urban_Environment_Tilt_Truck2` | 0 / 1045 | 0.00% | 677 | 0 | 107 | 677 |
| `screwin_light_bulb_with_socket` | 0 / 958 | 0.00% | 0 | 0 | 0 | 0 |
| `Industrial_Mine_cart` | 0 / 935 | 0.00% | 0 | 0 | 459 | 0 |
| `pictureX_0611_ball_transfer_unit_with_spring_loaded_ball` | 0 / 914 | 0.00% | 0 | 0 | 0 | 0 |
| `telescoping_boom` | 0 / 712 | 0.00% | 23 | 0 | 0 | 23 |
| `Chair_Folding_chair` | 0 / 653 | 0.00% | 0 | 0 | 0 | 0 |
| `Astronomy_Space_shuttle` | 0 / 627 | 0.00% | 513 | 245 | 0 | 513 |
| `Industrial_Ore_crusher_jaw` | 0 / 627 | 0.00% | 0 | 0 | 0 | 0 |
| `Vehicle_Sports_car` | 0 / 588 | 0.00% | 0 | 0 | 0 | 0 |
| `pictureX_0611_gyroscope_with_spinning_wheel_and_gimbal_rings` | 0 / 557 | 0.00% | 0 | 0 | 0 | 0 |
| `retractable_patio_awning` | 0 / 557 | 0.00% | 0 | 0 | 0 | 0 |
| `pictureX_0611_drawing_compass_with_adjustable_legs` | 191 / 3200 | 5.97% | 600 | 245 | 193 | 409 |
| `globe` | 44 / 685 | 6.42% | 44 | 44 | 44 | 0 |
| `metronome` | 275 / 3200 | 8.59% | 1630 | 401 | 278 | 1355 |
| `pictureX_0611_drum_pedal_with_beater_and_spring_return` | 74 / 627 | 11.80% | 74 | 74 | 74 | 0 |
| `wind_turbine` | 194 / 1200 | 16.17% | 199 | 194 | 197 | 5 |
| `Container_Barrel` | 416 / 2462 | 16.90% | 1639 | 416 | 478 | 1223 |
| `twojoint_prismatic_chain` | 195 / 938 | 20.79% | 198 | 196 | 195 | 3 |
| `louvered_shutter_assembly` | 349 / 1498 | 23.30% | 399 | 351 | 351 | 50 |
| `turntable` | 467 / 1951 | 23.94% | 1077 | 475 | 475 | 610 |

## P1：中等样本明显差（100 ≤ N < 500，strict <25%）

| 类别 | 通过 / 资产 | 严格通过率 | rest 非相邻 | 单关节 | Sobol | rest 通过后运动失败 |
|---|---:|---:|---:|---:|---:|---:|
| `Sports_Karting` | 0 / 490 | 0.00% | 0 | 0 | 0 | 0 |
| `Fountain_Drick_fountain` | 0 / 488 | 0.00% | 0 | 0 | 0 | 0 |
| `retractable_utility_knife` | 0 / 407 | 0.00% | 0 | 0 | 0 | 0 |
| `harmonic_drive` | 0 / 401 | 0.00% | 0 | 0 | 0 | 0 |
| `pictureX_0611_Butter_maker` | 0 / 372 | 0.00% | 0 | 0 | 0 | 0 |
| `Others_Safe` | 0 / 294 | 0.00% | 0 | 0 | 0 | 0 |
| `manual_pipe_bender` | 0 / 279 | 0.00% | 0 | 0 | 0 | 0 |
| `pictureX_0611_Hole_punch` | 0 / 279 | 0.00% | 115 | 0 | 0 | 115 |
| `Others_Matchbox` | 0 / 242 | 0.00% | 0 | 0 | 0 | 0 |
| `pictureX_0611_Ice_crream_machine` | 0 / 235 | 0.00% | 0 | 0 | 0 | 0 |
| `pictureX_0611_folders` | 0 / 235 | 0.00% | 0 | 0 | 0 | 0 |
| `Astronomy_Pressurised_module_door` | 0 / 226 | 0.00% | 208 | 0 | 0 | 208 |
| `pictureX_0611_juicer_press_with_handle` | 0 / 209 | 0.00% | 0 | 0 | 0 | 0 |
| `threestage_telescoping_slide` | 0 / 186 | 0.00% | 0 | 0 | 0 | 0 |
| `manual_grain_mill2` | 0 / 174 | 0.00% | 0 | 0 | 0 | 0 |
| `pictureX_0611_Garden_pruner` | 0 / 174 | 0.00% | 0 | 0 | 0 | 0 |
| `Military_Turret` | 0 / 157 | 0.00% | 0 | 0 | 0 | 0 |
| `Technology_Remote_Control` | 0 / 157 | 0.00% | 0 | 0 | 0 | 0 |
| `Music_Violin_case` | 0 / 154 | 0.00% | 0 | 0 | 27 | 0 |
| `pictureX_0611_ergonomic_clamp_with_adjustable_components` | 0 / 139 | 0.00% | 0 | 0 | 0 | 0 |
| `dishwasher_with_dropdown_door_and_sliding_racks` | 0 / 131 | 0.00% | 66 | 0 | 0 | 66 |
| `screwcap_bottle` | 0 / 131 | 0.00% | 0 | 0 | 0 | 0 |
| `pictureX_0611_car_scissor_jack_with_screw_mechanism` | 0 / 118 | 0.00% | 0 | 0 | 0 | 0 |
| `telescoping_fishing_rod` | 0 / 118 | 0.00% | 0 | 0 | 0 | 0 |
| `pictureX_0611_bi_fold_closet_door_system` | 0 / 109 | 0.00% | 0 | 0 | 0 | 0 |
| `Music_keyboard` | 0 / 107 | 0.00% | 0 | 0 | 0 | 0 |
| `pilers_fencing_pliers` | 0 / 104 | 0.00% | 0 | 0 | 0 | 0 |
| `pilers_tongue_groove_pliers` | 0 / 104 | 0.00% | 0 | 0 | 0 | 0 |
| `ironing_board` | 4 / 343 | 1.17% | 4 | 4 | 4 | 0 |
| `pictureX_0611_Air_blower` | 8 / 438 | 1.83% | 8 | 8 | 8 | 0 |
| `Art_Drawing_Models_Articulated_mannequin_Poseable_figure_mannequin` | 8 / 348 | 2.30% | 96 | 24 | 15 | 88 |
| `pictureX_0611_compass` | 11 / 108 | 10.19% | 11 | 11 | 11 | 0 |
| `rotary_table_with_tilting_trunnion` | 45 / 418 | 10.77% | 295 | 45 | 48 | 250 |
| `zippo_lighter` | 47 / 418 | 11.24% | 61 | 47 | 47 | 14 |
| `mechanical_timer_with_rotating_dial` | 30 / 199 | 15.08% | 30 | 30 | 30 | 0 |
| `Other_Built_in_oven` | 25 / 145 | 17.24% | 145 | 25 | 25 | 120 |
| `Door_Folding_gate` | 27 / 148 | 18.24% | 27 | 27 | 27 | 0 |
| `hand_operated_pasta_maker_with_rollers_and_crank` | 19 / 104 | 18.27% | 19 | 19 | 19 | 0 |
| `Container_Bottle` | 29 / 150 | 19.33% | 32 | 29 | 29 | 3 |
| `Sports_Bike` | 47 / 232 | 20.26% | 231 | 52 | 47 | 184 |
| `makeup2` | 34 / 157 | 21.66% | 103 | 34 | 34 | 69 |
| `Stationary_Clipboard` | 25 / 104 | 24.04% | 28 | 25 | 26 | 3 |
| `tricycle` | 67 / 274 | 24.45% | 154 | 67 | 67 | 87 |

## P2：低样本观察（N < 100，strict <25%）

| 类别 | 通过 / 资产 | 严格通过率 | rest 非相邻 | 单关节 | Sobol | rest 通过后运动失败 |
|---|---:|---:|---:|---:|---:|---:|
| `simple_drying_rack` | 0 / 99 | 0.00% | 0 | 0 | 0 | 0 |
| `Military_knife` | 0 / 98 | 0.00% | 0 | 0 | 0 | 0 |
| `Technology_Keyboard` | 0 / 93 | 0.00% | 0 | 0 | 0 | 0 |
| `pictureX_0611_Hand_crank_clothes_wringer` | 0 / 93 | 0.00% | 0 | 0 | 0 | 0 |
| `telescoping_pointer` | 0 / 93 | 0.00% | 44 | 0 | 0 | 44 |
| `Sports_Exercise_bike` | 0 / 87 | 0.00% | 65 | 0 | 0 | 65 |
| `pictureX_0611_ergonomic_clamp_with_adjustable` | 0 / 87 | 0.00% | 0 | 0 | 0 | 0 |
| `ratchet_strap` | 0 / 87 | 0.00% | 0 | 0 | 0 | 0 |
| `pictureX_0611_hydraulic_jack` | 0 / 78 | 0.00% | 0 | 0 | 0 | 0 |
| `pictureX_0611_hydraulic_jack1` | 0 / 78 | 0.00% | 0 | 0 | 0 | 0 |
| `Electrical_Wiring_Wire_stripper` | 0 / 75 | 0.00% | 41 | 13 | 0 | 41 |
| `pilers_linesman_pliers` | 0 / 65 | 0.00% | 0 | 0 | 0 | 0 |
| `pictureX_0611_Kitchen_set` | 0 / 64 | 0.00% | 64 | 0 | 16 | 64 |
| `pictureX_0611_flaring_tool_with_cone_and_clamp` | 0 / 64 | 0.00% | 0 | 0 | 0 | 0 |
| `Curtain_blind` | 0 / 63 | 0.00% | 32 | 14 | 0 | 32 |
| `Bag_Suitcase_Treasure_chest` | 0 / 60 | 0.00% | 0 | 0 | 30 | 0 |
| `Handtools_Wrench` | 0 / 60 | 0.00% | 15 | 0 | 0 | 15 |
| `Kitchen_Knife_set` | 0 / 59 | 0.00% | 0 | 0 | 0 | 0 |
| `Container_Dispenser` | 0 / 58 | 0.00% | 0 | 0 | 0 | 0 |
| `Headwear_Racing_helmet` | 0 / 58 | 0.00% | 0 | 0 | 0 | 0 |
| `Technology_Laptop` | 0 / 54 | 0.00% | 0 | 0 | 0 | 0 |
| `single_wheelbarrow` | 0 / 54 | 0.00% | 0 | 0 | 0 | 0 |
| `pictureX_0611_guitar_tuning_peg_mechanism` | 0 / 53 | 0.00% | 0 | 0 | 0 | 0 |
| `Powertools_drill` | 0 / 52 | 0.00% | 0 | 0 | 0 | 0 |
| `Urban_Environment_Fire_Hydrant` | 0 / 52 | 0.00% | 0 | 0 | 0 | 0 |
| `pilers_slip_joint_pliers` | 0 / 52 | 0.00% | 0 | 0 | 0 | 0 |
| `turnbuckle` | 0 / 52 | 0.00% | 0 | 0 | 0 | 0 |
| `Military_Aircraft` | 0 / 48 | 0.00% | 0 | 0 | 0 | 0 |
| `rotating_observatory_dome` | 0 / 48 | 0.00% | 0 | 0 | 0 | 0 |
| `tube_cutter` | 0 / 48 | 0.00% | 0 | 0 | 0 | 0 |
| `Military_Gun` | 0 / 47 | 0.00% | 0 | 0 | 0 | 0 |
| `Healthcare_First_aid_box` | 0 / 40 | 0.00% | 0 | 0 | 0 | 0 |
| `Military_Granade` | 0 / 40 | 0.00% | 7 | 0 | 0 | 7 |
| `Technology_Mobile_Phone` | 0 / 40 | 0.00% | 0 | 0 | 0 | 0 |
| `Sports_Table_football` | 0 / 36 | 0.00% | 0 | 0 | 0 | 0 |
| `Kitchen_Corkscrew` | 0 / 30 | 0.00% | 0 | 0 | 0 | 0 |
| `Handtools_Hand_plane` | 0 / 27 | 0.00% | 0 | 0 | 0 | 0 |
| `Powertools_Lawn_mower` | 0 / 22 | 0.00% | 0 | 0 | 0 | 0 |
| `Handtools_clothes_peg` | 0 / 20 | 0.00% | 0 | 0 | 0 | 0 |
| `hole_punch` | 0 / 20 | 0.00% | 12 | 0 | 0 | 12 |
| `sewing_machine` | 0 / 20 | 0.00% | 14 | 0 | 0 | 14 |
| `universal_joint` | 0 / 20 | 0.00% | 0 | 0 | 0 | 0 |
| `Bar_Piano` | 0 / 11 | 0.00% | 0 | 0 | 0 | 0 |
| `protractor_with_swing_arm` | 0 / 7 | 0.00% | 0 | 0 | 0 | 0 |
| `ring_blinder` | 1 / 70 | 1.43% | 64 | 11 | 1 | 63 |
| `Container_Paint_spray` | 2 / 48 | 4.17% | 9 | 2 | 2 | 7 |
| `Urban_Environment_Public_toilet1` | 1 / 20 | 5.00% | 1 | 1 | 1 | 0 |
| `manual_coffee_grinder` | 3 / 52 | 5.77% | 4 | 3 | 3 | 1 |
| `Other_Lighter` | 3 / 36 | 8.33% | 24 | 3 | 3 | 21 |
| `Container_Shipping_container` | 10 / 91 | 10.99% | 50 | 34 | 10 | 40 |
| `Other_pliers` | 10 / 87 | 11.49% | 10 | 10 | 10 | 0 |
| `Other_Folding_screen` | 10 / 81 | 12.35% | 81 | 45 | 10 | 71 |
| `Container_Lipstick` | 4 / 28 | 14.29% | 16 | 4 | 4 | 12 |
| `Handtools_caulking_gun` | 8 / 48 | 16.67% | 8 | 8 | 8 | 0 |
| `Kitchen_Dish_washer` | 4 / 20 | 20.00% | 20 | 4 | 4 | 16 |
| `automotive_differential_differential_gear` | 16 / 70 | 22.86% | 25 | 16 | 16 | 9 |

## 边界类（strict =25%）

| 类别 | 通过 / 资产 | 严格通过率 | rest 非相邻 | 单关节 | Sobol | rest 通过后运动失败 |
|---|---:|---:|---:|---:|---:|---:|
| `pilers_wire_strippers` | 26 / 104 | 25.00% | 26 | 26 | 26 | 0 |

## Warning：大样本但 25% ≤ strict <50%

| 类别 | 通过 / 资产 | 严格通过率 | rest 非相邻 | 单关节 | Sobol | rest 通过后运动失败 |
|---|---:|---:|---:|---:|---:|---:|
| `robotic_arms` | 807 / 3200 | 25.22% | 2007 | 807 | 807 | 1200 |
| `wall_safe_with_hinged_door_and_dial` | 840 / 3200 | 26.25% | 1844 | 858 | 841 | 1004 |
| `wheelbarrow` | 189 / 697 | 27.12% | 189 | 189 | 189 | 0 |
| `usb_drive_with_swivel_cover` | 873 / 3200 | 27.28% | 1458 | 1008 | 1043 | 585 |
| `wheelie_bin_with_hinged_lid` | 208 / 705 | 29.50% | 208 | 208 | 208 | 0 |
| `Door_Other` | 1055 / 3200 | 32.97% | 1055 | 1055 | 1055 | 0 |
| `defibrillator_case` | 273 / 792 | 34.47% | 535 | 273 | 273 | 262 |
| `Door_Garage_shutter` | 1063 / 3019 | 35.21% | 1710 | 1710 | 1648 | 647 |
| `Other_Tripod_Turnstile` | 1216 / 3200 | 38.00% | 2401 | 1216 | 1216 | 1185 |
| `camcorder_with_flipout_screen` | 1237 / 3200 | 38.66% | 3200 | 2299 | 1237 | 1963 |
| `Urban_Environment_utility_box` | 1307 / 3200 | 40.84% | 3199 | 1307 | 1308 | 1892 |

## 运动后恶化优先复核（按 rest 通过后运动失败比例排序，N ≥ 100，前 20）

| 类别 | 通过 / 资产 | 严格通过率 | rest 非相邻 | 单关节 | Sobol | rest 通过后运动失败 |
|---|---:|---:|---:|---:|---:|---:|
| `Astronomy_Pressurised_module_door` | 0 / 226 | 0.00% | 208 | 0 | 0 | 208 |
| `Other_Built_in_oven` | 25 / 145 | 17.24% | 145 | 25 | 25 | 120 |
| `Astronomy_Space_shuttle` | 0 / 627 | 0.00% | 513 | 245 | 0 | 513 |
| `Sports_Bike` | 47 / 232 | 20.26% | 231 | 52 | 47 | 184 |
| `simple_aframe_step_ladder` | 78 / 235 | 33.19% | 235 | 78 | 78 | 157 |
| `Urban_Environment_Tilt_Truck2` | 0 / 1045 | 0.00% | 677 | 0 | 107 | 677 |
| `camcorder_with_flipout_screen` | 1237 / 3200 | 38.66% | 3200 | 2299 | 1237 | 1963 |
| `rotary_table_with_tilting_trunnion` | 45 / 418 | 10.77% | 295 | 45 | 48 | 250 |
| `Urban_Environment_utility_box` | 1307 / 3200 | 40.84% | 3199 | 1307 | 1308 | 1892 |
| `dishwasher_with_dropdown_door_and_sliding_racks` | 0 / 131 | 0.00% | 66 | 0 | 0 | 66 |
| `Container_Barrel` | 416 / 2462 | 16.90% | 1639 | 416 | 478 | 1223 |
| `tackle_box_with_simple_hinged_lid` | 606 / 1200 | 50.50% | 1200 | 606 | 854 | 594 |
| `makeup2` | 34 / 157 | 21.66% | 103 | 34 | 34 | 69 |
| `metronome` | 275 / 3200 | 8.59% | 1630 | 401 | 278 | 1355 |
| `Door_Sliding_Door` | 172 / 302 | 56.95% | 297 | 172 | 172 | 125 |
| `pictureX_0611_Hole_punch` | 0 / 279 | 0.00% | 115 | 0 | 0 | 115 |
| `robotic_arms` | 807 / 3200 | 25.22% | 2007 | 807 | 807 | 1200 |
| `Other_Tripod_Turnstile` | 1216 / 3200 | 38.00% | 2401 | 1216 | 1216 | 1185 |
| `Music_Headphone` | 181 / 274 | 66.06% | 274 | 181 | 181 | 93 |
| `table_with_doors_and_drawers` | 109 / 206 | 52.91% | 178 | 109 | 124 | 69 |

## 覆盖率例外

- `Door_Folding_gate`：release 27/148 (18.24%)；observed 85; collision-measured 85。
- `single_wheelbarrow`：release 0/54 (0.00%)；observed 0; collision-measured 0。

其中 `single_wheelbarrow` 的 observed/measured 均为 0，属于包绑定漂移，不能据此判断碰撞几何；`Door_Folding_gate` 只有 85/148 被观测，release 率受未观测资产影响。

## 解释与使用

- P0 是首轮修复清单；P1 可作为第二批。
- P2 样本量很小，0/N 也可能有较宽的不确定性，建议先复核数据覆盖再修复资产。
- `watch`、drawing compass、standing desk、wood swing、folding door、folding chair、ore crusher 等已知接触/装配问题类别会出现在清单中；应结合 link-pair、深度和运动阶段定位，不应把整类直接 allowlist。
- 这是对封存 v3 结果的只读分类汇总，不改变封存结果。

源文件 SHA256：`d6cb106437a0cb9d83ff2ed5e053f01207c499145a29310e306f74413006d26f`。

[原始逐类指标](pva_table4_status_aware_metrics_v2/category_metrics.json) · [汇总说明](pva_table4_status_aware_metrics_v2/summary.md)
