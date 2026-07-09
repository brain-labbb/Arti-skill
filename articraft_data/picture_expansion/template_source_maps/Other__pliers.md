# Other / pliers — template source map

pattern: mixed（固定 named slots:jaw_function(主机构) + handle_form + pivot_mechanism(主机构);**外加** groove_count 多重性轴,channel-lock 调宽槽）

parents（2 个母资产,中央铆钉枢轴剪式手工具）:
- P1 snips rec_model-a-pair-of-compact-flush-cut-wire-snips-ele_20260610_085045_783699_e0ef2013 ← picture/Other/pliers/001.png（平口断线钳:jaw_blade + pivot_boss + rivet;1 REVOLUTE 开合）
- P2 lineman rec_model-heavy-duty-combination-lineman-pliers-abou_20260610_085054_765340_2d2cf55e ← .../002.png（重型综合电工钳:jaw + shank + hub + rivet;2 REVOLUTE;**fork 主母资产**）

批次：other_pliers_qwen37max_20260617（dashscope qwen3.7-max / medium）。8 变体全部 compile=success、workbench-only、≥1 非 fixed joint。

## 组合数预审

jaw_function 4 × handle_form 3 × pivot_mechanism 3 × groove_count{3,5,7} ≫ 10。核心机构 = 中央 rivet REVOLUTE 开合;pivot 含 PRISMATIC(slip-joint/channel-lock 移位)。

## Slot 候选覆盖

### Slot A:jaw_function（**主机构槽 1**——钳头功能）
| 候选 | record_id | 结构特征 / joint | 状态 |
|---|---|---|---|
| flush_cutter（基线 P1） | P1 | 平口剪刃 REVOLUTE | parent |
| combination_grip（基线 P2） | P2 | 综合咬+剪 REVOLUTE | parent |
| needle_nose | rec_variant-jaw-function-needle-nose-reshape-the-gri_20260618_054331_654376_7b198890 | 长尖嘴 REVOLUTE | converged(1) |
| locking_vise_grip | rec_variant-jaw-function-locking-vise-grip-turn-it-i_20260618_054331_654986_1903a019 | 锁定+过中心拨杆 REVOLUTE ×2 | converged(2) |

### Slot B:handle_form（手柄形态）
| 候选 | record_id | 结构特征 | 状态 |
|---|---|---|---|
| straight_dipped（基线） | parents | 直浸塑柄 | parent |
| looped_bow | rec_variant-handle-form-looped-bow-replace-the-two-s_20260618_054331_661863_dbc49fe3 | 闭环指圈柄 | converged(1) |
| cushioned_ergonomic | rec_variant-handle-form-cushioned-ergonomic-replace-_20260618_054331_657185_11ae9efe | 厚软人体工学柄 | converged(1) |

### Slot C:pivot_mechanism（**主机构槽 2**——枢轴）
| 候选 | record_id | 关键 joint | 状态 |
|---|---|---|---|
| fixed_rivet（基线） | parents | 固定铆钉 REVOLUTE | parent |
| slip_joint | rec_variant-pivot-mechanism-slip-joint-make-it-slip-_20260618_054331_664494_9df407d7 | 双位滑槽 PRISMATIC + REVOLUTE | converged(2) |
| tongue_and_groove | rec_variant-pivot-mechanism-tongue-and-groove-make-i_20260618_055210_360693_90e7c88e | channel-lock 调宽 PRISMATIC + REVOLUTE | converged(2) |

## Multiplicity / Copy Logic（核心轴）
- count_param: **`groove_count`**（channel-lock 下颚柄调宽槽数;tongue_and_groove 基线 5）
  - N 样本: {3, 5, 7}
  - N=3 → rec_variant-groove-count-3-... / N=7 → rec_variant-groove-count-7-...
  - 模板建议 N_range: **[3, 9]**；copied object: 单调宽槽(groove_{i}/`for i in range(n)`);placement 沿下颚柄等距;pivot 沿槽 PRISMATIC index + 开合 REVOLUTE

## 格子覆盖

| 槽 | 候选数(含基线) | 空格已填 |
|---|---|---|
| A jaw_function(主机构) | 4 | needle_nose / locking_vise_grip(2)（+2 基线） |
| B handle_form | 3 | looped_bow / cushioned_ergonomic(2) |
| C pivot_mechanism(主机构) | 3 | slip_joint / tongue_and_groove(2) |
| multiplicity groove_count | N∈{3,5,7} | N=3 / N=7(2) |

每槽 ≥2 + multiplicity 3 个 N → 满足 §8。**pliers 小类样本池就绪(2 母资产占 jaw 槽 2 基线 + 三结构槽 + groove multiplicity)。**

## 排除项
- 无（8 变体全收敛）。
