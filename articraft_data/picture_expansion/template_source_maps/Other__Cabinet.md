# Other / Cabinet — template source map

pattern: mixed（固定 named slots:storage_mechanism(主机构) + base_support;**外加** door_count / drawer_count 两根多重性轴）

parents（2 个母资产,两种储物范式）:
- P1 rec_model-a-vintage-industrial-steel-locker-cabinet-_20260610_083716_659700_76107d7e ← picture/Other/Cabinet/001.png（四门钢皮储物柜:4 hinged door REVOLUTE + 4 latch_knob REVOLUTE,均循环 door_{i}；door_count=4 基线，splayed_legs）
- P2 rec_model-a-wide-black-wooden-dresser-double-dresser_20260610_083741_441368_58f0953c ← picture/Other/Cabinet/002.png（八抽屉宽斗柜:8 PRISMATIC drawer 循环；drawer_count=8 基线，square_legs）

批次：other_cabinet_qwen37max_20260617（dashscope qwen3.7-max / medium）。8 变体全部 compile=success、workbench-only、≥1 非 fixed joint。

## 组合数预审


## Slot 候选覆盖

### Slot A:storage_mechanism（**主机构槽**——储物开合范式）
| 候选 | record_id | 关键 joint | 状态 |
|---|---|---|---|
| hinged_doors（基线 P1） | P1 | `door_{i}_hinge` REVOLUTE ×4（Z）| parent |
| drawers（基线 P2） | P2 | drawer PRISMATIC ×8（+X）| parent |
| sliding_doors | rec_variant-storage-mechanism-sliding-doors-replace-_20260617_095211_694410_503dd3bb | 横移滑门 PRISMATIC ×2（Y）| converged(2) |
| door_over_drawers | rec_variant-storage-mechanism-door-over-drawers-conv_20260617_095211_689335_73ab06e1 | 上铰门 REVOLUTE ×2 + 下抽屉 PRISMATIC ×2 | converged(4) |

### Slot B:base_support（底座/支撑）
| 候选 | record_id | 结构特征 | 状态 |
|---|---|---|---|
| splayed/square_legs（基线） | P1 / P2 | 短腿 | parent |
| plinth_toe_kick | rec_variant-base-support-plinth-toe-kick-replace-the_20260617_095621_663735_0c11ba6d | 落地踢脚基座 | converged(8) |
| casters | rec_variant-base-support-casters-replace-the-four-sh_20260617_095850_910442_8bf476d7 | 四角脚轮(cylinder/mesh)| converged(12) |

## Multiplicity / Copy Logic

- **count_param 1: `door_count`**（钢柜门数,源自 P1）
  - N 样本: {2, 4(P1), 6}
  - N=2 → rec_variant-door-count-2-make-it-a-two-door-steel-lo_20260617_095211_693823_b094b80c
  - N=6 → rec_variant-door-count-6-make-it-a-six-door-steel-lo_20260617_095211_699202_1a090052
  - 模板建议 N_range: [2, 8]；copied object: 整门 + latch_knob;naming `door_{i}`/`latch_{i}`(母资产已 `for i in ...` 循环);placement 沿 Y 等距,左右铰对称;joint policy 每门独立 REVOLUTE + 各自 latch REVOLUTE
- **count_param 2: `drawer_count`**（斗柜抽屉数,源自 P2）
  - N 样本: {4, 6, 8(P2)}
  - N=4 → rec_variant-drawer-count-4-make-it-a-four-drawer-dre_20260617_095211_690724_8a365012
  - N=6 → rec_variant-drawer-count-6-make-it-a-six-drawer-dres_20260617_095557_705575_9830b46f
  - 模板建议 N_range: [2, 12]；copied object: hollow tray + front + knobs,由 `_build_drawer` helper;naming `*_drawer_{i}`;placement 行列网格;joint policy 每抽屉独立 PRISMATIC(+X)

## 格子覆盖

| 槽 | 候选数(含基线) | 空格已填 |
|---|---|---|
| A storage(主机构) | 4 | sliding_doors / door_over_drawers(2)（+2 基线范式) |
| B base_support | 3 | plinth / casters(2) |
| multiplicity door_count | N∈{2,4,6} | N=2 / N=6(2) |
| multiplicity drawer_count | N∈{4,6,8} | N=4 / N=6(2) |

每槽 ≥2 + 两根 multiplicity 各 3 个 N → 满足 §8。**Cabinet 小类样本池就绪。**

## 排除项
- 无（8 变体全收敛）。
