# Sign / sign — template source map

object: foldable A-frame two-panel plastic floor caution sign (sandwich-board style); top apex REVOLUTE hinge opens/folds the A-frame, integrated carry-handle cut-through near the apex, front leaf carries printed warning placard.

pattern: mixed（固定 named slots: panel_profile + base_support + handle_mechanism；外加 apex knuckle 的 multiplicity 复制轴）

parents:
- rec_build-a-realistic-articulated-3d-model-of-a-sign_20260609_215029_914922_9ae01b65 ← picture/Sign/sign/002.png（trapezoidal-tapered 直边梯形面板 + 模制角脚 base feet（left/right foot）+ 单 hinge barrel + 独立 hinge_handle 提手件；fold_back_panel REVOLUTE 沿 X；helper _build_panel_shell / _panel_profile / _build_hinge_handle / _build_graphic_plate；vent slots `for i in range(4)` 循环发射。**基线：tapered profile / corner feet / 单 knuckle / 固定提手孔**）
- rec_build-a-realistic-articulated-3d-model-of-a-sign_20260609_215026_933125_3171150b ← picture/Sign/sign/001.png（rounded-top 圆顶面板 + 平底 flush base + 三段 knuckle bands + 凸起警示三角/文字带；apex_hinge REVOLUTE 沿 Y；helper _panel_shell / _hinge_bands；ribs `for i in range(7)` + text bands `enumerate` + knuckle bands `for cy in centers` 均循环发射。**基线：rounded profile / flush base / 3 knuckle / 凸字凸三角**）

两个 parent 拓扑几乎相同（同一物体两张图），落在同一基线格；变体只沿单轴填空格。

## 组合数预审（硬门槛 ≥10）
panel_profile 4 × base_support 2 × multiplicity N{1,2,3,5} 4 个不同 N = 32 ≥ 10 ✓
（handle_mechanism slot 另算，作为机构轴；即便仅取 panel_profile 4 × N 4 = 16 也 ≥10。）

## Slot 候选覆盖

### Slot A: panel_profile（面板轮廓 —— 主结构轴；apex fold REVOLUTE 保持）
| 候选(未来 module) | record_id | 关键 part/helper | 结构特征 | 状态 |
|---|---|---|---|---|
| trapezoidal_tapered（基线） | P_9ae01b65 | _panel_profile / _build_panel_shell | 直边梯形,底宽顶窄 | converged(parent) |
| rounded_top（基线） | P_3171150b | _panel_shell（threePointArc 顶弧） | 圆顶弧形顶边 | converged(parent) |
| gabled_peaked | rec_sign_var_profile_gabled | _panel_shell（尖顶 gable profile） | 双坡尖顶面板 | converged (forked) |
| shield | rec_sign_var_profile_shield | _panel_shell（盾形曲边 profile） | 盾形圆角下尖 | converged (forked) |

### Slot B: base_support（底部稳定结构）
| 候选 | record_id | 关键 part/helper | 结构特征 | 状态 |
|---|---|---|---|---|
| flush_slab（基线） | P_3171150b | _panel_shell 平底边 | 面板底边直接落地 | converged(parent) |
| corner_feet | P_9ae01b65 / rec_sign_var_base_feet | foot helper（left/right foot；变体改为 loop 发射 foot_{i}） | 角脚外撇加宽脚印 | converged(parent) + converged (forked) |

### Slot C: handle_mechanism（提手机构）
| 候选 | record_id | 关键 part/joint | 结构特征 | 状态 |
|---|---|---|---|---|
| fixed_grab_hole（基线） | P_3171150b / P_9ae01b65 | handle_cut / _build_hinge_handle 固定提手孔 | apex 固定抓握孔 | converged(parent) |
| swing_up_bail | rec_sign_var_handle_swivel | handle_bar REVOLUTE 沿 Y（pivot 在 apex barrel 顶面） | 可上翻提梁(第二个非fixed joint) | converged (forked) |

## Multiplicity / Copy Logic
- count_param: knuckle_count（apex hinge knuckle barrels 沿 panel 宽度 for-loop 发射）
- N 样本已覆盖: {1, 3} parent → {2, 5} 变体 → rec_sign_var_knuckles2 / rec_sign_var_knuckles5（P_9ae01b65=1 宽 barrel, P_3171150b=3 bands）
- 模板建议 N_range: [1, 7]（奇偶皆可,对称分布于宽度中线）
- copied object: 单个 knuckle barrel；naming knuckle_{i}；placement 沿 apex 宽度等距/对称；joint policy 全部 knuckle 共享同一 apex REVOLUTE 销轴,不增加额外 joint（FIXED 视觉随 hinge）
- 第二复制轴(可选,clip_chain 变体): tether link_{i} 链式逐节铰接(linear_chain)作为 spreader 限位带；rec_sign_var_clip_chain；N_range 建议 [2,5]

## 排除项（未来 compatibility matrix 素材）
- 暂无；待 fork 后回填(任何连续不收敛的轴值组合记此,标原因：漂浮 / 穿插 / joint origin / 出类目)。
