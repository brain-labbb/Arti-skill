# Other / Tripod Turnstile — template source map

pattern: mixed（固定 named slots:pedestal_form + barrier + arm_mechanism;**外加** hub_count / arm_count 两根多重性轴,核心）

parents（1 个母资产,不锈钢腰高双道三辊闸；002 已建模,001 未映射）:
- rec_stainless-steel-waist-high-tripod-turnstile-acce_20260612_133446_998781_ea880fc3 ← picture/Other/Tripod Turnstile/002.png（基线 = 双道:中央 head 左右各一 tripod_hub(CONTINUOUS,3 arm 循环) + 两侧 railing；hub_count:2 / arm_count:3 / pedestal:rect_cabinet / barrier:tube_railings）

批次：other_tripod_turnstile_qwen37max_20260617（dashscope qwen3.7-max / medium）。8 变体全部 compile=success、workbench-only、≥1 非 fixed joint。

## 组合数预审

hub_count{1,2,3} × arm_count{3,4} × pedestal 3 × barrier 3 ≫ 10。核心 hub_count + arm_count 双 multiplicity;barrier/mechanism 含 REVOLUTE(玻璃门/防夹落臂)。

## Slot 候选覆盖

### Slot A:pedestal_form（基座/机头形态）
| 候选 | record_id | 结构特征 | 状态 |
|---|---|---|---|
| rect_cabinet_head（基线） | parent | 矩形机箱头 | parent |
| round_post | rec_variant-pedestal-form-round-post-replace-the-rec_20260617_130720_143696_fccb7b6c | 圆柱立柱(lathe)| converged(2) |
| slanted_optical_head | rec_variant-pedestal-form-slanted-optical-head-repla_20260617_130720_147451_e0cb8f21 | 斜面光学头 | converged(2) |

### Slot B:barrier（侧栏/拦挡）
| 候选 | record_id | 关键 joint | 状态 |
|---|---|---|---|
| tube_railings（基线） | parent | 管栏(固定)| parent |
| glass_swing_panel | rec_variant-barrier-glass-swing-panel-replace-the-tu_20260617_131222_754305_091848cd | 玻璃摆门 REVOLUTE(Z)| converged(3) |
| extended_guide_railings | rec_variant-barrier-extended-guide-railings-extend-t_20260617_131841_641231_dcc77210 | 多段延长导栏(循环)| converged(2) |

### Slot C:arm_mechanism（**主机构槽**——辊臂动作）
| 候选 | record_id | 关键 joint | 状态 |
|---|---|---|---|
| continuous_rotation（基线） | parent | hub CONTINUOUS(arm 随转)| parent |
| anti_panic_drop_arm | rec_variant-arm-mechanism-anti-panic-drop-arm-make-t_20260617_131919_219282_e7cf9f83 | 旋转 + 落臂 REVOLUTE(水平)| converged(4) |

## Multiplicity / Copy Logic（双核心轴）
- **count_param 1: `hub_count`**（通道/辊闸头数）
  - N 样本: {1, 2, 3}
  - N=1 → rec_variant-hub-count-1-make-it-a-single-lane-tripod_20260617_130720_139224_b6dc0f05
  - N=3 → rec_variant-hub-count-3-make-it-a-triple-lane-tripod_20260617_130720_141373_9ed09ba3
  - 模板建议 N_range: [1, 6]；copied object: 整 tripod hub + arms;naming `tripod_hub_{i}`/`for i in range(n)`;placement 沿 X 等距;joint policy 各 hub 独立 CONTINUOUS
- **count_param 2: `arm_count`**（每 hub 辊臂数;基线 3=三辊）
  - N 样本: {3, 4}
  - N=4 → rec_variant-arm-count-4-give-each-rotating-hub-four-_20260617_130720_136765_e977074b
  - 模板建议 N_range: [3, 4]（>4 出"三辊"类目,3 为典型）；copied object: 单 arm;naming `arm_{i}`(母资产已 `ARM_PHASES` 循环);placement 等角;joint policy 随 hub 旋转

## 格子覆盖

| 槽 | 候选数(含基线) | 空格已填 |
|---|---|---|
| A pedestal_form | 3 | round_post / slanted(2) |
| B barrier | 3 | glass_swing / extended(2) |
| C arm_mechanism(主机构) | 2 | drop_arm(1) |
| multiplicity hub_count | N∈{1,2,3} | N=1 / N=3(2) |
| multiplicity arm_count | N∈{3,4} | N=4(1) |

每槽 ≥2 + 双 multiplicity → 满足 §8。**Tripod Turnstile 小类样本池就绪。**

## 排除项
- arm_count > 4 出"三辊闸"典型类目,N_range 上限取 4。
