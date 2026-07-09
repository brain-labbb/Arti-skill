# Container / Cosmetic — template source map

pattern: multiplicity (mixed: named structural slots A/B + one real multiplicity axis on the pan grid)

parents:
- rec_concealer-makeup-palette-compact-with-a-hinged-c_20260606_074704_321770_f124e974 ← picture/Container/Cosmetic/001.png
  — occupies cell: Slot A = rounded_rect_body, Slot B = rear_hinge_flip_lid, N = 8 pans (2x4). This single parent fills one cell across all three axes; every variant forks off it changing exactly one axis.

Notes on parent code form (§4 readability audit): PASS. Functional layers are cleanly split into `base` and `lid` parts with semantic visual names (`base_tray`, `base_frame`, `label_strip`, `lid_slab`, `lid_frame`, `mirror`, `hinge_knuckle_l/r`). The 8 pans ARE loop-emitted: `for (row, col, cx, cy) in _pan_centers(): base.visual(_pan_mesh(), ..., name=f"pan_{row}_{col}")` with a shared `_pan_mesh()` helper, regular grid placement, and a uniform "all pans are base visuals" policy — so the multiplicity axis reads mechanically. Non-moving decorations (frame, label, hinge knuckles) are inline `base.visual(...)`, not FIXED-joint parts. One real revolute joint `base_to_lid`. Geometry uses CadQuery (`_rounded_box`, boolean cuts for tray cavity / mirror well) and `CylinderGeometry` for hinge knuckles. No readability remediation needed; multiplicity variants can reuse the existing pan loop verbatim and only change the grid dimensions.

## Slot 候选覆盖

### Slot A:case_footprint(整体轮廓 / 形状家族)
| 候选(未来 module) | record_id | 关键 part·joint·helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| rounded_rect_body | rec_concealer-makeup-palette-compact-with-a-hinged-c_20260606_074704_321770_f124e974 | base / lid / base_to_lid(revolute -X) / `_rounded_box` / `_base_tray_mesh` | 长方形 (LEN_X≈0.13 × DEP_Y≈0.06) 圆角板坯托盘 + 同形盖 | converged(parent) |
| square_body | rec_container_cosmetic_var_square_body | base / lid / base_to_lid / `_rounded_box` | 近正方形紧凑盒,length≈depth,圆角方形轮廓,方形 pan 块 | converged |
| round_body | rec_container_cosmetic_var_round_body | base / lid / base_to_lid / lathe·CadQuery cylinder helper | 圆盘形机身 + 圆形铰接盖,扇形/圆形 pan 绕圆周排布 | converged |

### Slot B:closure_mechanism(开合机构 / joint 类型)
| 候选(未来 module) | record_id | 关键 part·joint·helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| rear_hinge_flip_lid | rec_concealer-makeup-palette-compact-with-a-hinged-c_20260606_074704_321770_f124e974 | lid / base_to_lid(REVOLUTE axis -X, 后缘) / hinge_knuckle_l·hinge_knuckle_r / mirror | 后缘铰链翻盖,镜子在盖内面,0→110° | converged(parent) |
| slide_drawer | rec_container_cosmetic_var_slide_drawer | outer_shell / drawer_tray / shell_to_drawer(PRISMATIC 长轴) / mirror on shell | 装 pan 的托盘作为抽屉从固定外壳线性滑出,frame/label 在外壳 | converged |
| clasp_latch_lid | rec_container_cosmetic_var_clasp_latch_lid | lid / base_to_lid(revolute 后缘) / latch_clasp / base_to_latch(REVOLUTE 前缘) | 保留后缘铰链翻盖,新增前缘可动卡扣件(自身 revolute)锁盖 | converged |

### Slot C(multiplicity 轴):pan_count(同构 concealer pan × N)
| 候选(未来 module) | record_id | 关键 part·joint·helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| n8_pans_2x4 | rec_concealer-makeup-palette-compact-with-a-hinged-c_20260606_074704_321770_f124e974 | `_pan_centers` / `_pan_mesh` / pan_{row}_{col} (8 个) | 2 行 × 4 列 = 8 pans,循环发射 | converged(parent) |
| n4_pans_2x2 | rec_container_cosmetic_var_n4_pans | `_pan_centers` / `_pan_mesh` / pan_{row}_{col} (4 个) | 2 行 × 2 列 = 4 pans,托盘与间距适配 | converged |
| n12_pans_3x4 | rec_container_cosmetic_var_n12_pans | `_pan_centers` / `_pan_mesh` / pan_{row}_{col} (12 个) | 3 行 × 4 列 = 12 pans,托盘加深、间距适配 | converged |

## Multiplicity / Copy Logic
- count_param: `pan_count`(由 PAN_ROWS × PAN_COLS 网格派生;parent 内为 PAN_COLS=4, PAN_ROWS=2)
- N 样本已覆盖: {4, 8, 12} → rec_container_cosmetic_var_n4_pans / rec_concealer-makeup-palette-compact-with-a-hinged-c_20260606_074704_321770_f124e974(parent) / rec_container_cosmetic_var_n12_pans
- 模板建议 N_range: [2, 24](真实彩妆/遮瑕盘单格到约 24 格;网格行列由 N 因式分解,模板采样域远大于样本是正常的)
- copied object / naming / placement / joint policy:
  - copied object: 单个 concealer pan(共享 `_pan_mesh()` rounded-box 凹槽膏体)
  - naming: `pan_{row}_{col}`(循环 `for (row,col,cx,cy) in _pan_centers()`)
  - placement: 规则网格,X/Y 等距,由 `_pan_centers()` 居中并向前缘偏移
  - joint policy: 统一为 base/tray 的 `base.visual(...)`(无独立关节,随托盘运动);非装饰但不可动的复制件,符合 §4 inline 规则

## 组合数预审
组合数预审: Π(Slot A=3 × Slot B=3) × N 样本=3 = 27 ≥ 10 ✓

## 排除项(未来 compatibility matrix 素材)
- magnetic_lift_off_lid(无铰链、纯磁吸分离上下两片):真实彩妆盒常见但会产生 0 个非 fixed joint(分离 = 脱离场景而非活动关节),违反 §3 第 2 条 ≥1 非 fixed joint,故不作为候选;clasp_latch_lid 已用一个带真实可动卡扣的翻盖覆盖"按扣开合"语义。
- 跨轴组合变体(如 round_body × slide_drawer):不造——组合是模板采样器免费产出,非样本池义务(§5)。圆形机身 × 抽屉的接口风险(圆筒套圆筒抽屉)若下游需要可后补 1 个组合抽检,目前未发现必须裁决的干涉,暂不加。
- footprint 第 4 候选(如 oval / 收腰):参考图仅 1 张(rounded-rect),square 与 round 已是该类真实词汇表内最稳的两个扩展;再加形态有出类目/凭空发明风险,故 Slot A 停在 3 候选(预审 27 已远超 10,无需补)。
