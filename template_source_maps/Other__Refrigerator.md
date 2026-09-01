# Other / Refrigerator — template source map

pattern: mixed（固定 named slots:door_config(主机构) + handle + base_support;**外加** crisper_drawer / shelf 多重性）

parents（1 个母资产,独立式上冷冻双门冰箱）:
- rec_model-a-free-standing-top-freezer-refrigerator-a_20260610_084517_847436_5e4e4aac ← picture/Other/Refrigerator/001.png（基线 = door:top_freezer_two_door（freezer + fridge 各 REVOLUTE）× handle:bar × base:plinth_kick × interior:fixed_shelves）

批次：other_refrigerator_qwen37max_20260617（dashscope qwen3.7-max / medium）。8 变体全部 compile=success、workbench-only、≥1 非 fixed joint。

## 组合数预审

door_config 4 × handle 3 × base 3 ≫ 10。door_config 含 REVOLUTE(单/双/侧开/法式)与 PRISMATIC(底部冷冻抽屉)拓扑;interior crisper 含 PRISMATIC 多重性。

## Slot 候选覆盖

### Slot A:door_config（**主机构槽**——门布局/数量）
| 候选 | record_id | 关键 joint | 状态 |
|---|---|---|---|
| top_freezer_two_door（基线） | parent | freezer_door + fridge_door REVOLUTE（同轴独立）| parent |
| single_door | rec_variant-door-config-single-door-make-it-a-single_20260617_115832_806919_b282bb89 | 单全高门 REVOLUTE | converged(1) |
| side_by_side | rec_variant-door-config-side-by-side-make-it-a-side-_20260617_115832_797670_15197e44 | 左右对开双全高门 REVOLUTE ×2 | converged(2) |
| french_door_bottom_freezer | rec_variant-door-config-french-door-bottom-freezer-m_20260617_115832_806231_923d13e6 | 上法式双门 REVOLUTE ×2 + 底冷冻抽屉 PRISMATIC | converged(3) |

### Slot B:handle
| 候选 | record_id | 结构特征 | 状态 |
|---|---|---|---|
| bar_handle（基线） | parent | 横向铝条把手 | parent |
| recessed_pocket | rec_variant-handle-recessed-pocket-replace-the-brush_20260617_115832_803461_ca93bb30 | 嵌入抠手 | converged(2) |
| tubular_tall | rec_variant-handle-tubular-tall-replace-the-short-ho_20260617_115832_804990_fa2af944 | 全高管状 pro 把手(cylinder)| converged(2) |

### Slot C:base_support
| 候选 | record_id | 结构特征 | 状态 |
|---|---|---|---|
| plinth_kick（基线） | parent | 凹踢脚板 | parent |
| short_legs | rec_variant-base-support-short-legs-replace-the-rece_20260617_120221_340146_c142b909 | 四短可调腿 | converged(2) |
| casters | rec_variant-base-support-casters-replace-the-recesse_20260617_120359_513314_deaad0d2 | 四脚轮(cylinder)| converged(2) |

## Multiplicity / Copy Logic
- count_param: **`crisper_drawer_count`**（下室抽拉保鲜抽屉数）
  - N 样本: {0(基线), 2}
  - N=2 → rec_variant-interior-crisper-drawers-add-two-pull-ou_20260617_120707_672391_760df2e7（2 PRISMATIC crisper + 2 门 = 4 nonfixed）
  - 模板建议 N_range: [0, 3]；copied object: crisper bin;naming `crisper_{i}`/`for i in range(n)`;placement 沿 +Z 等距;joint policy 各独立 PRISMATIC(+Y)
  - 另:interior fixed shelves 亦为循环固定件(shelf_count,连续多重性,无关节)。

## 格子覆盖

| 槽 | 候选数(含基线) | 空格已填 |
|---|---|---|
| A door_config(主机构) | 4 | single / side_by_side / french(3) |
| B handle | 3 | recessed / tubular(2) |
| C base_support | 3 | legs / casters(2) |
| multiplicity crisper_drawer | N∈{0,2} | N=2(1) |

每槽 ≥2(主机构 4)+ multiplicity → 满足 §8。**Refrigerator 小类样本池就绪。**

## 排除项
- 无（8 变体全收敛）。
