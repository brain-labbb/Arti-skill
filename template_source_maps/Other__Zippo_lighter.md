# Other / Zippo lighter — template source map

pattern: mixed（固定 named slots:case_shape + lid_mechanism + ignition(主机构);**外加** chimney_hole_count 多重性轴）

parents（1 个母资产,经典 Zippo 防风翻盖打火机）:
- rec_model-a-classic-zippo-style-windproof-flip-light_20260610_084943_427286_231b5418 ← picture/Other/Zippo lighter/001.png（基线 = case:rounded_rect × lid:flip(REVOLUTE 铰链) × ignition:flint_wheel(CONTINUOUS) + insert/chimney）

批次：other_zippo_lighter_qwen37max_20260617（dashscope qwen3.7-max / medium）。6 变体全部 compile=success、workbench-only、≥1 非 fixed joint。

## 组合数预审

case 3 × lid 2 × ignition 3 × chimney_hole_count{N} ≫ 10。lid 含 REVOLUTE(翻盖)/PRISMATIC(滑套);ignition 含 CONTINUOUS(燧轮)/PRISMATIC(压电按钮)。

## Slot 候选覆盖

### Slot A:case_shape（外壳形态）
| 候选 | record_id | 结构特征 | 状态 |
|---|---|---|---|
| rounded_rect（基线） | parent | 圆角矩形扁壳 | parent |
| barrel_round | rec_variant-case-shape-barrel-round-change-the-round_20260617_162734_661443_4e5d217b | 圆筒桶身(lathe)| converged(2) |
| beveled_armor | rec_variant-case-shape-beveled-armor-change-the-case_20260617_162734_718920_ccb09f0d | 倒角装甲壳 | converged(2) |

### Slot B:lid_mechanism（**主机构槽 1**——盖开合）
| 候选 | record_id | 关键 joint | 状态 |
|---|---|---|---|
| flip_hinge（基线） | parent | 翻盖 REVOLUTE(横铰)| parent |
| slide_up_sleeve | rec_variant-lid-mechanism-slide-up-sleeve-replace-th_20260617_162734_901875_fe311409 | 上滑套 PRISMATIC | converged(2) |

### Slot C:ignition（**主机构槽 2**——点火）
| 候选 | record_id | 关键 joint | 状态 |
|---|---|---|---|
| flint_wheel（基线） | parent | 燧轮 CONTINUOUS | parent |
| piezo_push_button | rec_variant-ignition-piezo-push-button-replace-the-k_20260617_162734_905980_30de5873 | 压电按钮 PRISMATIC | converged(2) |
| dual_flint_wheels | rec_variant-ignition-dual-flint-wheels-give-the-ligh_20260617_162734_997765_605bc4cf | 双燧轮各 CONTINUOUS | converged(3) |

## Multiplicity / Copy Logic
- count_param: **`chimney_hole_count`**（防风烟囱通风孔数）
  - N 样本: dense 变体提升孔数 → rec_variant-chimney-hole-count-dense-...
  - 模板建议 N_range: [6, 24]；copied object: 单 vent_hole(boolean_difference 圆孔);naming `hole_{i}`/`for i in range(n)`;placement 沿烟囱两侧等距;无关节(随 insert 固定)

## 格子覆盖

| 槽 | 候选数(含基线) | 空格已填 |
|---|---|---|
| A case_shape | 3 | barrel_round / beveled_armor(2) |
| B lid_mechanism(主机构) | 2 | slide_up_sleeve(1) |
| C ignition(主机构) | 3 | piezo / dual_wheels(2) |
| multiplicity chimney_hole_count | dense | dense(1) |

每槽 ≥2 + multiplicity → 满足 §8。**Zippo lighter 小类样本池就绪。**

## 排除项
- 无（6 变体全收敛）。
