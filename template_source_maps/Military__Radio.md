# Military / Radio — template source map

pattern: mixed
parents: rec_model-a-rugged-military-style-handheld-two-way-r_20260610_080423_521217_3ffa3864 ← picture/Military/Radio/001.png(single母资产,覆盖全部 Slot A/B/C 的母值格子:foldover_whip 天线 + N=1 顶旋钮 + keypad4x4+双PTT 控制)

母体核心固定结构(所有变体共享):root `body`(`lower_shell` desert_tan + `upper_shell` matte_black 双色 slab,`lcd_bezel`/`lcd_screen` 凹陷绿屏,`led_red`/`led_green`,`keypad_panel`,顶面 `knob_boss`/`antenna_boss`,背面 `clip_boss`);`belt_clip`(`clip_blade`,`clip_hinge` REVOLUTE 0–25°,X 轴);键盘双重循环 `for row,key_z … for col,key_x …` 生成 `key_{row}_{col}`(母体 4×4=16 键)。母体共 5 个非固定关节:`knob_spin`(CONTINUOUS,Z)+`antenna_fold`(REVOLUTE,−Y,0–90°)+`ptt_press_upper`/`ptt_press_lower`(PRISMATIC,X,0–3mm)+`clip_hinge`(REVOLUTE,X)。

## Slot 候选覆盖

### Slot A:antenna(顶面天线形态)
| 候选(module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| foldover_whip | rec_model-a-rugged-military-style-handheld-two-way-r_20260610_080423_521217_3ffa3864 | part `whip_antenna`(elem `antenna_body`)/ joint `antenna_fold`(REVOLUTE, axis −Y, 0–90°) | 单段锥形长鞭(ribbed collar + makeCone whip 0.350 + sphere tip),从 `antenna_boss` 折倒外伸 | converged (parent) |
| articulated_2seg | rec_field_radio_var_antfold2seg | parts `antenna_base`(`base_segment`)+`antenna_whip`(`whip_segment`)/ joints `antenna_fold`(REVOLUTE −Y)+`elbow_fold`(REVOLUTE −Y, parent=antenna_base, origin z=ANT_ELBOW_Z) | 两段折叠:刚性 base 短段带 elbow 枢轴 barrel,whip 段经第二肘关节链接(linear_chain 子结构),双 90° 折叠 | converged (workbench, rating pending sync) |
| stub | rec_field_radio_var_antstub | part `stub_antenna`(`antenna_stub`)/ joint `antenna_fold`(REVOLUTE −Y, 0–90°) | 短橡胶鸭 stub(粗锥 ANT_STUB_LEN 0.065 + helical groove rings + 大半球 tip),仍可折倒 | converged (workbench, rating pending sync) |

### Slot B:top-knob count(顶面旋钮多重度,见 Multiplicity)
| 候选(module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| single_knob N=1 | rec_model-a-rugged-military-style-handheld-two-way-r_20260610_080423_521217_3ffa3864 | part `volume_knob`(`knob_cap`/`knob_shaft`/`knob_pointer`)/ joint `knob_spin`(CONTINUOUS, Z) / boss `knob_boss` | 单手写旋钮:KnobGeometry 滚花 cap + 隐藏 shaft + off-axis `knob_pointer` 证明连续转 | converged (parent) |
| knob_count N=2 | rec_field_radio_var_knob2 | parts `top_knob_{i}` i∈0..1(`knob_cap`/`knob_shaft`/`knob_pointer`)/ joints `knob_spin_{i}` CONTINUOUS / bosses `knob_boss_{i}` | 单旋钮改写为 `for i,knob_x in enumerate(KNOB_POSITIONS)` 循环;每个独立 CONTINUOUS 关节 | converged (workbench, rating pending sync) |
| knob_count N=3 | rec_field_radio_var_knob3 | parts `top_knob_{i}` i∈0..2 / joints `knob_spin_{i}` CONTINUOUS / bosses `knob_boss_{i}` | `N_KNOBS=3` 驱动 `for i in range(N_KNOBS)`,KNOB_POSITIONS 三档均布;旋钮缩小(DIA 0.015)避让 | converged (workbench, rating pending sync) |

### Slot C:control(前面板交互形态)
| 候选(module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| keypad4x4+ptt | rec_model-a-rugged-military-style-handheld-two-way-r_20260610_080423_521217_3ffa3864 | `key_{row}_{col}`(4×4 循环)+ `ptt_button_upper`/`ptt_button_lower`(`ptt_cap`)/ `ptt_press_upper`/`ptt_press_lower`(PRISMATIC X) | 16 键凸出 panel + 左侧双 PTT 按压 3mm | converged (parent) |
| flip_cover | rec_field_radio_var_flipcover | part `front_cover`(`cover_panel`,elem mesh `front_cover`)+ body `cover_hinge_boss` / joint `cover_hinge`(REVOLUTE, 0–160°) | 键盘前盖翻盖:铰接面板罩住键盘,新增 1 个 REVOLUTE(总 6 非固定) | converged (workbench, rating pending sync) |
| rotary_selector | rec_field_radio_var_seldial | part `channel_dial`(`channel_dial_disc`)/ joint `dial_spin`(CONTINUOUS, axis +Y) + front `dial bezel/well` | 键盘换为前面板凹入旋转频道选择盘(detented disc,绕 Y 轴),无 keypad 循环(loops:0) | converged (workbench, rating pending sync) |
| keypad4x3 | rec_field_radio_var_keypad4x3 | `key_{row}_{col}`(4 rows × 3 cols 循环)/ joints 同母体 | 键盘多重度 4×4→4×3:KEY_COLS_X 三列,`key_{r}_3` 断言不存在(12 键) | converged (workbench, rating pending sync) |
| single_bar_ptt | rec_field_radio_var_dualptt | part `ptt_bar`(`ptt_bar`)/ joint `ptt_press`(PRISMATIC, X, 0–3mm) | 左侧双按钮合并为单条长 PTT bar(PRISMATIC 数 2→1,总 4 非固定) | converged (workbench, rating pending sync) |

## Multiplicity / Copy Logic
- count_param: top-knob 数(顶面 `top_knob_{i}` / `knob_spin_{i}` / `knob_boss_{i}` 三元组的份数);母体为单手写 `volume_knob`,在 knob-count 变体中改写为 `top_knob_i` 循环
- N 样本已覆盖: {1, 2, 3} → rec_model-a-rugged-military-style-handheld-two-way-r_…_3ffa3864(N=1,单写)/ rec_field_radio_var_knob2(N=2)/ rec_field_radio_var_knob3(N=3)
- 模板建议 N_range: [1, 4]
- copied object / naming / placement / joint policy: 复制对象 = 旋钮三件套(`knob_cap`+`knob_shaft`+`knob_pointer`)及其顶面 `knob_boss_{i}`;命名按 `top_knob_{i}` / `knob_spin_{i}` / `knob_boss_{i}` 索引;放置由 `KNOB_POSITIONS` 元组沿顶面 X 均布(N≥2 时缩小 KNOB_DIA 避让 + 把天线 ANT_X 推向远左);每个旋钮挂独立 CONTINUOUS 关节(axis Z),各带自己的 off-axis pointer sweep 检查;键盘 `key_{row}_{col}` 与左侧 PTT 也是循环复制但属 Slot C 多重度,不归此 count_param。

## 排除项(未来 compatibility matrix 素材)
- fixed long-whip(不可折叠定长鞭天线):已 drop,出本类目交互性(radio 天线必须 fold-over revolute 才计非固定结构,定长鞭无关节)
- battery pack / belt clip / speaker-mic:作为配件装饰,不入槽位轴;`belt_clip`+`clip_hinge` 虽是 REVOLUTE 但属共享固定结构,不作为差异候选
- size(机身尺寸 BODY_W/D/H):不作为多重度或槽位轴(纯缩放,不改拓扑)
