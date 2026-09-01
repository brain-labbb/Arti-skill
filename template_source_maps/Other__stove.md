# Other / stove — template source map

pattern: mixed（固定 named slots:door_mechanism(主机构) + cooktop_lid + base_storage;**外加** burner_count 多重性轴,核心）

parents（2 个母资产,独立式燃气灶台）:
- P1 compact rec_model-a-compact-freestanding-gas-range-cooker-ap_20260610_085103_861011_ee507c92 ← picture/Other/stove/001.png（紧凑燃气灶:烤箱门 REVOLUTE(下翻) + 旋钮 REVOLUTE;**fork 主母资产**）
- P2 stainless rec_model-a-freestanding-stainless-steel-gas-range-a_20260610_085113_420453_10691ac9 ← .../002.png（不锈钢燃气灶:门 + 旋钮 REVOLUTE ×3 + PRISMATIC + 玻璃掀盖 lid_glass）

批次：other_stove_qwen37max_20260617（dashscope qwen3.7-max / medium）。8 变体全部 compile=success、workbench-only、≥1 非 fixed joint。

## 组合数预审

door 3 × cooktop_lid 3 × base 3 × burner_count{2,4,6} ≫ 10。door 含 REVOLUTE(下翻/侧开/法式双门);cooktop_lid 含 REVOLUTE(玻璃掀盖);base 含 PRISMATIC(储物抽屉)。

## Slot 候选覆盖

### Slot A:door_mechanism（**主机构槽**——烤箱门)
| 候选 | record_id | 关键 joint | 状态 |
|---|---|---|---|
| drop_down_front（基线）| parents | 下翻门 REVOLUTE(底铰)| parent |
| side_hinged_swing | rec_variant-door-mechanism-side-hinged-swing-change-_20260618_063803_125635_754e1747 | 侧开门 REVOLUTE(竖铰)| converged(8) |
| french_double | rec_variant-door-mechanism-french-double-replace-the_20260618_063803_121384_3852f805 | 法式双门 REVOLUTE ×2(mirrored)| converged(9) |

### Slot B:cooktop_lid（灶面/掀盖）
| 候选 | record_id | 关键 joint | 状态 |
|---|---|---|---|
| open_burners（基线）| P1 | 明火灶头(无盖)| parent |
| glass_lift_lid | rec_variant-cooktop-lid-glass-lift-lid-add-a-hinged-_20260618_063803_108536_324b9170 | 玻璃掀盖 REVOLUTE(后铰)| converged(8) |
| griddle_flat_top | rec_variant-cooktop-lid-griddle-flat-top-replace-the_20260618_063803_110672_3991e157 | 平面铁板灶 | converged(7) |

### Slot C:base_storage（底座/储物）
| 候选 | record_id | 关键 joint | 状态 |
|---|---|---|---|
| plinth_kickbase（基线）| parents | 实心踢脚座 | parent |
| storage_drawer | rec_variant-base-storage-storage-drawer-replace-the-_20260618_063803_119100_ed3ce770 | 抽拉保温/储物抽屉 PRISMATIC | converged(8) |
| raised_legs | rec_variant-base-storage-raised-legs-lift-the-whole-_20260618_064727_806978_263d7de1 | 四撇复古灶腿(leg_{i})| converged(7) |

## Multiplicity / Copy Logic（核心轴）
- count_param: **`burner_count`**（灶面燃气灶头数;基线 4）
  - N 样本: {2, 4, 6}
  - N=2 → rec_variant-burner-count-2-... / N=6 → rec_variant-burner-count-6-...（专业灶双排）
  - 模板建议 N_range: **[1, 6]**；copied object: 单灶头(cap+ring+grate)+ 对应 knob(burner_{i}/knob_{i}/`for i in range(n)`,共享 helper);placement 灶面网格;joint policy 各 knob 独立 REVOLUTE

## 格子覆盖

| 槽 | 候选数(含基线) | 空格已填 |
|---|---|---|
| A door_mechanism(主机构) | 3 | side_hinged_swing / french_double(2) |
| B cooktop_lid | 3 | glass_lift_lid / griddle_flat_top(2) |
| C base_storage | 3 | storage_drawer / raised_legs(2) |
| multiplicity burner_count | N∈{2,4,6} | N=2 / N=6(2) |

每槽 ≥2 + multiplicity 3 个 N → 满足 §8。**stove 小类样本池就绪(2 母资产 + 三结构槽 + burner multiplicity)。**

## 排除项
- 无（8 变体全收敛）。
