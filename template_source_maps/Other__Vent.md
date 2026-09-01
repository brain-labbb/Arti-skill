# Other / Vent — template source map

pattern: mixed（housing_form 由 4 母资产预填;固定 named slots:grille_style + backdraft_shutter(主机构);**外加** impeller_blade_count / shutter_flap_count / guard_ring_count 三根多重性轴）

parents（4 个母资产,均带 CONTINUOUS 风叶,4 种壳体形态已占满 housing_form 槽）:
- P1 round_through_wall rec_model-a-small-round-through-wall-exhaust-vent-ab_20260610_084653_136102_f4fe4250 ← .../Vent/001.png（duct_sleeve + bezel + mesh_grille + impeller CONTINUOUS）
- P2 square_wall rec_model-a-square-wall-mounted-exhaust-ventilation-_20260610_084713_390960_4c3504f2 ← .../002.png（housing + ring/spoke grille + impeller CONTINUOUS）
- P3 inline_duct rec_model-an-industrial-inline-duct-fan-a-hollow-ope_20260610_084733_114867_4010e666 ← .../003.png（duct_shell + 环形 front_guard + motor_mount + impeller）
- P4 round_flange rec_model-a-round-flange-mounted-wall-vent-fan-about_20260610_084754_275335_6e21aa96 ← .../004.png（housing + impeller + backdraft_flap REVOLUTE ×2 + bolt flange）

批次：other_vent_qwen37max_20260617（dashscope qwen3.7-max / medium）。8 变体全部 compile=success、workbench-only、≥1 非 fixed joint。

## 组合数预审

housing_form 4 × grille 3 × shutter 3 × blade_count{3,5,7} × flap_count{2,4} ≫ 10。含 CONTINUOUS(风叶) + REVOLUTE(百叶/防回流) 拓扑。

## Slot 候选覆盖

### Slot A:housing_form（壳体形态,4 母资产已占满）
| 候选 | record_id | 状态 |
|---|---|---|
| round_through_wall | P1 | parent |
| square_wall | P2 | parent |
| inline_duct_cylinder | P3 | parent |
| round_flange | P4 | parent |

### Slot B:grille_style（出风面格栅;fork 自 P1）
| 候选 | record_id | 结构特征 | 状态 |
|---|---|---|---|
| woven_mesh（基线 P1） | P1 | 编织网 | parent |
| louvered_front_grille | rec_variant-grille-style-louvered-front-grille-repla_20260617_135024_364964_2901e840 | 角度百叶片(循环) | converged(1) |
| perforated_plate | rec_variant-grille-style-perforated-plate-replace-th_20260617_135024_364904_67dfdf18 | 冲孔面板(循环孔) | converged(1) |

### Slot C:backdraft_shutter（**主机构槽**——防回流挡板）
| 候选 | record_id | 关键 joint | 状态 |
|---|---|---|---|
| none / spring_flap×2（基线;P4 有）| P1(无)/P4(×2) | REVOLUTE flap | parent |
| multi_blade_louver_shutter | rec_variant-backdraft-multi-blade-louver-shutter-add_20260617_135024_363235_d3db9306 | 多叶百叶 REVOLUTE ×N（循环）| converged(6) |
| single_gravity_flap | rec_variant-backdraft-single-gravity-flap-add-a-sing_20260617_135024_367499_bf2f8952 | 单重力翻板 REVOLUTE | converged(2) |

## Multiplicity / Copy Logic（三根轴）
- **`impeller_blade_count`**（风叶数;fork P2）: {3,5,7} → N=3 rec_variant-impeller-blade-count-3-... / N=7 rec_variant-impeller-blade-count-7-...；N_range [3,9]；copied object: FanRotorBlade,等角;随 impeller CONTINUOUS
- **`shutter_flap_count`**（P4 防回流翻板数）: {2,4} → N=4 rec_variant-shutter-flap-count-4-...；N_range [2,6]；naming `flap_{i}`/loop,等角,各独立 REVOLUTE
- **`guard_ring_count`**（P3 前护网同心环数）: N=4 rec_variant-guard-ring-count-4-...；N_range [2,6]；同心环循环,随壳固定(无关节)

## 格子覆盖

| 槽 | 候选数(含基线) | 空格已填 |
|---|---|---|
| A housing_form | 4(母资产) | — |
| B grille_style | 3 | louvered / perforated(2) |
| C backdraft_shutter(主机构) | 3 | louver_shutter / gravity_flap(2) |
| mult impeller_blade_count | {3,5,7} | N=3 / N=7(2) |
| mult shutter_flap_count | {2,4} | N=4(1) |
| mult guard_ring_count | {2,4} | N=4(1) |

每槽 ≥2 + 三 multiplicity → 满足 §8。**Vent 小类样本池就绪(壳体 4 型 + 三 multiplicity)。**

## 排除项
- 无（8 变体全收敛）。
