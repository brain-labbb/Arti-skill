# Other / Air conditioner — template source map

pattern: mixed（固定 named slots:body_form + airflow_mechanism(主机构) + service_panel;**外加** vane_count 多重性轴）

parents（1 个母资产,壁挂分体式空调室内机）:
- rec_model-a-wall-mounted-mini-split-air-conditioner-_20260610_081946_833476_e9cc92a3 ← picture/Other/Air conditioner/001.png（基线 = body:rounded_bottom_curve × airflow:3_independent_slim_vanes × panel:top_hinge_lift × vane_count:3）

批次：other_air_conditioner_qwen37max_20260617（dashscope qwen3.7-max / medium）。10 变体全部 compile=success、均 workbench-only、均 ≥1 非 fixed joint。

## 组合数预审


## Slot 候选覆盖

### Slot A:body_form（壳体侧剖面家族;连续尺寸由模板缩放,这里列结构不同的剖面)
| 候选 | record_id | 关键 part/visual | 结构特征 | 状态 |
|---|---|---|---|---|
| rounded_bottom_curve（基线） | parent | housing_shell（CadQuery 四分之一圆下沿 + 前倾面）| 仅下前沿圆角 | parent |
| boxy_rectangular | rec_variant-body-form-boxy-rectangular-replace-the-r_20260617_082555_062348_6caa9319 | housing_shell（平直前脸,方角）| 棱角箱体,平下前脸 | converged(4 joints) |
| raked_wedge | rec_variant-body-form-raked-wedge-reshape-the-housin_20260617_082555_060026_072b1710 | housing_shell（强前倾楔形剖面）| 楔形侧剖面 | converged(4) |
| full_bullnose_capsule | rec_variant-body-form-full-bullnose-capsule-reshape-_20260617_082555_065010_4b99b445 | housing_shell（lathe/CadQuery 上下双圆角胶囊）| 全圆角胶囊剖面 | converged(4) |

### Slot B:airflow_mechanism（**主机构槽**——出风导向动作）
| 候选 | record_id | 关键 joint | 结构特征 | 状态 |
|---|---|---|---|---|
| three_independent_slim_vanes（基线） | parent | `{label}_louver_pivot` REVOLUTE ×3（X 轴,独立）| 3 片独立横向导风叶 | parent |
| single_wide_deflector | rec_variant-airflow-mechanism-single-wide-deflector-_20260617_082555_066950_4f557258 | 单顶铰偏导板 REVOLUTE（X 轴）| 一片全宽导风板 | converged(2) |
| vertical_vane_bank | rec_variant-airflow-mechanism-vertical-vane-bank-kee_20260617_082555_066029_148eace8 | 垂直叶 REVOLUTE ×N（Z 轴,循环发射）| 左右摆动垂直叶组 | converged(13) |
| closing_outlet_door | rec_variant-airflow-mechanism-closing-outlet-door-re_20260617_083907_370519_97104f78 | 出风门 REVOLUTE（X 轴,下沿铰）| 关停时齐平闭合的整面出风门 | converged(2) |

### Slot C:service_panel（前面板/检修盖机构）
| 候选 | record_id | 关键 joint | 结构特征 | 状态 |
|---|---|---|---|---|
| top_hinge_lift（基线） | parent | `front_panel_hinge` REVOLUTE（顶沿,上掀）| 顶铰上掀检修盖 | parent |
| two_leaf_clamshell | rec_variant-service-panel-two-leaf-clamshell-split-t_20260617_083933_412704_3fe5a919 | 双叶顶铰 REVOLUTE ×2 | 蛤壳式双叶上掀 | converged(5) |
| bottom_hinge_drop_front | rec_variant-service-panel-bottom-hinge-drop-front-ch_20260617_084339_979241_fd012151 | 底沿铰 REVOLUTE（前下翻）| 底铰前翻露滤网 | converged(4) |

## Multiplicity / Copy Logic

- count_param: **`vane_count`**（横向导风叶数；基线 3）
- N 样本已覆盖: {2, 3, 5}
  - N=2 → rec_variant-vane-count-2-reduce-the-louver-outlet-to_20260617_084623_518824_62841830（panel + 2 vane = 3 nonfixed）
  - N=3 → parent（panel + 3 vane = 4 nonfixed）
  - N=5 → rec_variant-vane-count-5-increase-the-louver-outlet-_20260617_085011_793946_ce97feaa（panel + 5 vane = 6 nonfixed）
- 模板建议 N_range: **[2, 6]**（室内机出风口现实叶片数有限）
- copied object: 单片横向导风叶 + 两端 pivot pin,沿弧形下前脸 `_arc_point(theta)` 等角发射
- naming: `{label}_louver_vane` / `{label}_louver_pivot`,`for label, theta in VANE_SPECS`(基线已循环发射,可直接作 module 源码)
- placement: 沿下前弧 theta 等角分布
- joint policy: 每片**独立活动** REVOLUTE(X 轴,±45°)

## 格子覆盖

| 槽 | 候选数(含基线) | 空格已填 |
|---|---|---|
| A body_form | 4 | boxy / wedge / bullnose(3) |
| B airflow(主机构) | 4 | single_deflector / vertical_bank / outlet_door(3) |
| C service_panel | 3 | clamshell / drop_front(2) |
| multiplicity vane_count | N∈{2,3,5} | N=2 / N=5(2) |

每槽均 ≥2(目标 3–6)converged + multiplicity 3 个 N → 满足 §8 完成定义。**Air conditioner 小类样本池就绪。**

## 排除项
- 无（10 变体全收敛）。
