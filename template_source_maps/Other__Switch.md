# Other / Switch — template source map

pattern: mixed（固定 named slots:actuator_type(主机构/部件词汇表) + faceplate_shape;**外加** gang_count 多重性轴,核心）

parents（1 个母资产,欧式三联墙面开关插座板）:
- rec_model-a-european-three-gang-wall-switch-plate-mo_20260610_084601_505028_c46f379f ← picture/Other/Switch/001.png（基线 = actuator:rocker × gang_count:2 rockers + 1 Schuko socket × faceplate:rounded_rect_horizontal）

批次：other_switch_qwen37max_20260617（dashscope qwen3.7-max / medium）。7 变体全部 compile=success、workbench-only、≥1 非 fixed joint。

## 组合数预审

actuator 4 × gang_count{1,2,3,4} × faceplate 3 ≫ 10。actuator 含 REVOLUTE(rocker/toggle)/PRISMATIC(push)/CONTINUOUS(rotary) 全拓扑;gang_count 核心 multiplicity。

## Slot 候选覆盖

### Slot A:actuator_type（**主机构槽 / 部件词汇表**——开关执行件）
| 候选 | record_id | 关键 joint | 状态 |
|---|---|---|---|
| rocker（基线） | parent | `rocker_{i}_pivot` REVOLUTE 跷跷板（循环）| parent |
| toggle_lever | rec_variant-actuator-type-toggle-lever-replace-the-t_20260617_123635_014339_5e9273bf | 拨杆 REVOLUTE（循环 toggle_{i}）| converged(2) |
| push_button | rec_variant-actuator-type-push-button-replace-the-tw_20260617_123635_019678_531fc067 | 按钮 PRISMATIC(-Y,循环 button_{i})| converged(2) |
| rotary_dimmer | rec_variant-actuator-type-rotary-dimmer-replace-the-_20260617_123635_016750_f19a560b | 旋钮 CONTINUOUS(+Y,循环 dimmer_{i})| converged(2) |

### Slot B:faceplate_shape（面板足迹）
| 候选 | record_id | 结构特征 | 状态 |
|---|---|---|---|
| rounded_rect_horizontal（基线） | parent | 横向圆角矩形 | parent |
| square_single | rec_variant-faceplate-shape-square-single-change-the_20260617_123934_446656_812d68e4 | 方形单联 | converged(1) |
| round_plate | rec_variant-faceplate-shape-round-plate-change-the-h_20260617_124140_807529_491b75e5 | 圆形面板(lathe)| converged(2) |

## Multiplicity / Copy Logic（核心轴）
- count_param: **`gang_count`**（开关位数；基线 2 rockers）
  - N 样本: {1, 2, 4}
  - N=1 → rec_variant-gang-count-1-make-it-a-single-gang-switc_20260617_123635_022221_e21d2998（1 rocker）
  - N=4 → rec_variant-gang-count-4-make-it-a-four-gang-switch-_20260617_123635_019019_e3244d88（4 rocker）
  - 模板建议 N_range: **[1, 6]**；copied object: 单 module（actuator + 槽口）;naming `rocker_{i}`/`for i in range(n)`(母资产已循环);placement 沿 X 等距;joint policy 各 module 独立(REVOLUTE/PRISMATIC/CONTINUOUS 随 actuator 槽)
  - **跨槽组合提示**:gang_count × actuator_type 可自由组合(N 个同类型 actuator),是模板侧主要采样维。

## 格子覆盖

| 槽 | 候选数(含基线) | 空格已填 |
|---|---|---|
| A actuator_type(主机构) | 4 | toggle / push_button / rotary(3) |
| B faceplate_shape | 3 | square / round(2) |
| multiplicity gang_count | N∈{1,2,4} | N=1 / N=4(2) |

每槽 ≥2 + multiplicity 3 个 N → 满足 §8。**Switch 小类样本池就绪(actuator 词汇表 + gang multiplicity 双强)。**

## 排除项
- 无（7 变体全收敛）。
