# Bag_Suitcase / Luggage bag — template source map

pattern: mixed（固定 named slots: body_opening + wheel_system + handle_system）
parents:
- rec_hard-shell-rolling-luggage-bag-with-a-retractabl_20260605_132328_146368_110d261f ← picture/Bag_Suitcase/Luggage bag/001.png（绿色硬壳拉杆箱;body 单壳 + pull_handle;PRISMATIC 伸缩拉杆 + CONTINUOUS 直列滑轮;**整体壳/直列轮/伸缩把 基线**）

## 组合数预审
body_opening 3 × wheel_system 2 × handle_system 2 = 12 ≥ 10 ✓

## Slot 候选覆盖
### Slot A: body_opening（开合形式 —— 主结构轴）
| 候选 | record_id | 关键 part/joint | 状态 |
|---|---|---|---|
| integrated_shell（基线） | P_luggage | 单 body 壳,无独立盖 | parent |
| front_lid_flap | rec_luggage_var_frontlid | 正面翻盖 REVOLUTE;**空心薄壁舱** | converged |
| split_side | rec_luggage_var_split_side | 沿拉杆纵向面纵切两半,一半绕竖边 REVOLUTE 侧门甩开;**空心舱** | converged |

> 空心契约:会打开露内部的候选(front_lid/split_side)壳体用 .shell()/.cut() 做空心薄壁;闭合候选(integrated/wheel-only/handle-only)保持实心。

### Slot B: wheel_system（轮系）
| 候选 | record_id | 关键 joint | 状态 |
|---|---|---|---|
| inline_2（基线） | P_luggage | 2 直列轮 CONTINUOUS | parent |
| spinner_4 | rec_luggage_var_spinner4 | 4 万向轮 CONTINUOUS（for-loop） | converged |

### Slot C: handle_system（提拉机构）
| 候选 | record_id | 关键 joint | 状态 |
|---|---|---|---|
| telescoping_pull（基线） | P_luggage | 伸缩拉杆 PRISMATIC | parent |
| fixed_top_grab | rec_luggage_var_fixedgrab | 固定顶提把（活动关节由轮系提供） | converged |

## Multiplicity / Copy Logic
- count_param: wheel_count（spinner_4 用 for i in range(4) 发射万向轮）
- 模板建议 N_range: [2,4]
- copied object: 单只脚轮 caster；naming wheel_{i}；placement 四角/两侧；joint policy 每轮 CONTINUOUS

## 排除项
- clamshell_split (rec_luggage_var_clamshell)、split_flip (rec_luggage_var_split_flip) 经人工审核质量不佳,已删除（2026-06-18）。body_opening 槽保留 integrated/front_lid/split_side 3 候选,组合数 3×2×2=12 ≥10 仍达标。
- 留存 4/4 变体(frontlid/split_side/spinner4/fixedgrab) 全部 compile=success、≥1非fixed joint、workbench-only、单轴 diff、绑定门禁通过;split_side 另经几何修复(轮在箱外+框条对齐)。
