# Other / Coffin — template source map

pattern: mixed（固定 named slots:body_shape + lid_mechanism(主机构) + carry_handles;handles 内含 handle_count 多重性）

parents（1 个母资产,乡村六面 toe-pincher 木棺）:
- rec_model-a-rustic-toe-pincher-wooden-coffin-about-1_20260610_083844_610854_3d5f4f7f ← picture/Other/Coffin/001.png（基线 = body:toe_pincher_hex × lid:full_side_hinge × handles:none）

批次：other_coffin_qwen37max_20260617（dashscope qwen3.7-max / medium）。7 变体全部 compile=success、workbench-only、≥1 非 fixed joint。

## 组合数预审

body 3 × lid 4 × handles 3 = 36 ≫ 10。lid 槽含多种 REVOLUTE 拓扑(整盖/分两段/双叶/半身),handles 含 swing REVOLUTE 多重性。

## Slot 候选覆盖

### Slot A:body_shape（棺体足迹形状）
| 候选 | record_id | 结构特征 | 状态 |
|---|---|---|---|
| toe_pincher_hex（基线） | parent | 六面收肩 | parent |
| rectangular_casket | rec_variant-body-shape-rectangular-casket-replace-th_20260617_101630_082815_477ccc1d | 直角矩形 casket | converged(1) |
| tapered_trapezoid | rec_variant-body-shape-tapered-trapezoid-replace-the_20260617_101630_082682_26a4798a | 四面收锥梯形 | converged(1) |

### Slot B:lid_mechanism（**主机构槽**——盖板开合）
| 候选 | record_id | 关键 joint | 状态 |
|---|---|---|---|
| full_side_hinge（基线） | parent | `lid_hinge` REVOLUTE（长边）| parent |
| split_two_panel | rec_variant-lid-mechanism-split-two-panel-split-the-_20260617_101630_082204_1d8d2709 | 头/脚两段各 REVOLUTE | converged(2) |
| double_leaf_wings | rec_variant-lid-mechanism-double-leaf-wings-replace-_20260617_101630_085988_de26f9e8 | 双长边对开 REVOLUTE ×2 | converged(2) |
| half_couch_head_only | rec_variant-lid-mechanism-half-couch-head-only-make-_20260617_101630_087507_0d84ee10 | 仅头半身开,脚半固定 | converged(1) |

### Slot C:carry_handles（抬棺把手；含 handle_count 多重性）
| 候选 | record_id | 关键 joint | 状态 |
|---|---|---|---|
| none（基线） | parent | 无 | parent |
| fixed_side_rails | rec_variant-carry-handles-fixed-side-rails-add-long-_20260617_101915_308385_b642913b | 两长边固定导轨(inline visual,无关节) | converged(1, lid only) |
| swing_drop_bar_handles | rec_variant-carry-handles-swing-drop-bar-handles-add_20260617_102117_972906_de2a3aba | 摆动 drop-bar REVOLUTE ×N（循环 handle_{i}）| converged(7 = lid + 6 handles) |

## Multiplicity / Copy Logic
- count_param: **`handle_count`**（swing drop-bar 把手数,仅 swing 候选下生效）
  - N 样本: swing 变体内 6 个把手(每长边 3)。模板建议 N_range: [2, 8]（每侧 1–4）
  - copied object: bracket post + 摆杆 bar;naming `handle_{i}`/`for i in range(n)`;placement 沿长边等距对称;joint policy 每把手独立 REVOLUTE(上下摆)
  - 主结构 lid 仍为单一 named slot,无复制逻辑。

## 格子覆盖

| 槽 | 候选数(含基线) | 空格已填 |
|---|---|---|
| A body_shape | 3 | rectangular / tapered(2) |
| B lid(主机构) | 4 | split / double_leaf / half_couch(3) |
| C carry_handles | 3 | fixed_rails / swing_bars(2) |

每槽 ≥2(主机构 4) → 满足 §8。**Coffin 小类样本池就绪。**

## 排除项
- 无（7 变体全收敛）。
