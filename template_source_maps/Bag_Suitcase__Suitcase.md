# Bag_Suitcase / Suitcase — template source map

pattern: mixed（固定 named slots: closure_mechanism + handle_system + reinforcement）
parents:
- rec_vintage-hard-sided-travel-suitcase-with-a-hinged_20260605_131200_241451_cb7b4d8c ← picture/Bag_Suitcase/Suitcase/001.png（复古皮硬箱;body + lid;2 REVOLUTE 翻盖;_add_box_shell/_add_corner_caps;**卡扣/固定提把/护角 基线**）

## 组合数预审
closure_mechanism 3 × handle_system 2 × reinforcement 2 × lid_profile 2 × internal_structure 2 = 48 ≥ 10 ✓

## Slot 候选覆盖
### Slot A: closure_mechanism（闭合机构 —— 主结构轴;翻盖 REVOLUTE 保持）
| 候选 | record_id | 关键 part/joint | 状态 |
|---|---|---|---|
| latches（基线） | P_suitcase | 翻盖 REVOLUTE + 金属搭扣 | parent |
| buckle_straps | rec_suitcase_var_buckle | 翻盖 + 两条皮带扣（扣舌 REVOLUTE） | converged |
| zipper_perimeter | rec_suitcase_var_zipper | 沿周拉链开合（拉头 PRISMATIC 滑） | converged |

### Slot B: handle_system（提手）
| 候选 | record_id | 关键 joint | 状态 |
|---|---|---|---|
| fixed_top_grab（基线） | P_suitcase | 固定顶提把 | parent |
| folding_handle | rec_suitcase_var_folding_handle | 折叠提把 REVOLUTE 收倒 | converged |

### Slot C: reinforcement（边角加固）
| 候选 | record_id | 关键 part | 状态 |
|---|---|---|---|
| corner_caps（基线） | P_suitcase | 8 金属护角 | parent |
| edge_banding | rec_suitcase_var_edgeband | 沿棱包边箍条（for-loop N 条） | converged |

### Slot D: lid_profile（盖型）
| 候选 | record_id | 关键 part/helper | 状态 |
|---|---|---|---|
| flat（基线） | P_suitcase | 平盖 | parent |
| dome_trunk | rec_suitcase_var_dome_trunk | 圆顶驼背 trunk 盖（lathe/曲面） | converged |

### Slot E: internal_structure（内部结构）
| 候选 | record_id | 关键 part/joint | 状态 |
|---|---|---|---|
| open（基线） | P_suitcase | 无内层,直接箱腔 | parent |
| lift_tray | rec_suitcase_var_lift_tray | 内部打包托盘,REVOLUTE 掀起露下层腔 | converged |

## Multiplicity / Copy Logic
- count_param: band_count / cap_count（护角与包边条 for-loop 发射）
- 模板建议 N_range: 护角固定 8;包边条 [2,6]
- copied object: 单护角 / 单包边条;naming cap_{i}/band_{i};placement 八角/沿棱;joint policy FIXED 随 body（Rule1 inline 或独立 part 视情况）

## 排除项
- 无,本批 4/4 变体全部 compile=success、≥1非fixed joint、workbench-only、单轴 diff、绑定门禁通过;无出类目、无排除项。
