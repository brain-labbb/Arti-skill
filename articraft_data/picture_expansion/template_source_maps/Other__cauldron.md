# Other / cauldron — template source map

pattern: mixed（固定 named slots:pot_form + lid_mechanism(主机构) + handle/suspension(主机构);**外加** leg_count 多重性轴）

parents（1 个母资产,传统铸铁吊环炊煮锅）:
- rec_model-a-traditional-cast-iron-cooking-cauldron-w_20260610_085035_017686_f944b57b ← picture/Other/cauldron/001.png（基线 = pot:rounded_belly × lid:lift_off_dome(PRISMATIC 提) × handle:swing_bail(REVOLUTE 摆动) × legs:none）

批次：other_cauldron_qwen37max_20260617（dashscope qwen3.7-max / medium）。8 变体全部 compile=success、workbench-only、≥1 非 fixed joint。

## 组合数预审

pot_form 3 × lid 3 × handle/suspension 3 × leg_count{0,3,4} ≫ 10。lid 含 PRISMATIC(提盖)/REVOLUTE(翻盖);handle 含 REVOLUTE(摆环/吊点)。

## Slot 候选覆盖

### Slot A:pot_form（锅身形态）
| 候选 | record_id | 结构特征 | 状态 |
|---|---|---|---|
| rounded_belly（基线） | parent | 鼓腹圆锅 | parent |
| straight_cylindrical | rec_variant-pot-form-straight-cylindrical-reshape-th_20260618_050548_068625_26cd72d3 | 直筒锅(lathe)| converged(2) |
| wide_shallow_bowl | rec_variant-pot-form-wide-shallow-bowl-reshape-the-d_20260618_050548_069801_cef6bfac | 宽浅碗形(lathe)| converged(2) |

### Slot B:lid_mechanism（**主机构槽 1**——锅盖）
| 候选 | record_id | 关键 joint | 状态 |
|---|---|---|---|
| lift_off_dome（基线） | parent | 提盖 PRISMATIC | parent |
| hinged_flip | rec_variant-lid-mechanism-hinged-flip-replace-the-li_20260618_050548_068309_8d43806f | 侧铰翻盖 REVOLUTE | converged(2) |
| clamp_locking | rec_variant-lid-mechanism-clamp-locking-replace-the-_20260618_050548_063736_607ffa31 | 提盖 PRISMATIC + 卡扣 REVOLUTE ×2 | converged(4) |

### Slot C:handle / suspension（**主机构槽 2**——提环/悬挂）
| 候选 | record_id | 关键 joint | 状态 |
|---|---|---|---|
| swing_bail（基线） | parent | 摆动吊环 REVOLUTE | parent |
| side_loop_ears | rec_variant-handle-side-loop-ears-remove-the-swing-b_20260618_050548_068968_09d73153 | 双固定耳柄(去吊环;保留提盖 PRISMATIC)| converged(1) |
| tripod_stand | rec_variant-suspension-tripod-stand-suspend-the-caul_20260618_051320_281205_2de73386 | 三脚架顶点吊挂 REVOLUTE | converged(3) |

## Multiplicity / Copy Logic（核心轴）
- count_param: **`leg_count`**（锅底铸铁腿数;基线 0 无腿）
  - N 样本: {0, 3, 4}
  - N=3 → rec_variant-leg-count-3-...（吉普赛三足锅）/ N=4 → rec_variant-leg-count-4-...
  - 模板建议 N_range: **[0, 4]**（0=吊挂锅,3=典型三足,4=立式）；copied object: 单腿(leg_{i}/`for i in range(n)`);placement 绕锅底等角;joint policy 全 fixed(随锅身)

## 格子覆盖

| 槽 | 候选数(含基线) | 空格已填 |
|---|---|---|
| A pot_form | 3 | straight_cylindrical / wide_shallow_bowl(2) |
| B lid_mechanism(主机构) | 3 | hinged_flip / clamp_locking(2) |
| C handle/suspension(主机构) | 3 | side_loop_ears / tripod_stand(2) |
| multiplicity leg_count | N∈{0,3,4} | N=3 / N=4(2) |

每槽 ≥2 + multiplicity 3 个 N → 满足 §8。**cauldron 小类样本池就绪。**

## 排除项
- 无（8 变体全收敛）。
