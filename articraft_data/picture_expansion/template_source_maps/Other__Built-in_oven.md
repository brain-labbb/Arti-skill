# Other / Built-in oven — template source map

pattern: mixed（固定 named slots:door_mechanism(主机构);**外加** door_count / rack_count / knob_count 三根多重性轴）

parents（2 个母资产,均嵌入式箱体电器,前开门）:
- P1 rec_model-a-built-in-single-electric-wall-oven-the-k_20260610_083623_507710_363e4e6b ← picture/Other/Built-in oven/001.png（单腔壁挂烤箱:door 底铰下翻 REVOLUTE + shelf_rack PRISMATIC;door_count=1 / rack_count=1 基线）
- P2 rec_model-a-built-in-compact-microwave-oven-brastemp_20260610_083650_171507_6c671696 ← picture/Other/Built-in oven/002.png（紧凑微波炉:door 底铰下翻 REVOLUTE + 4 knob CONTINUOUS 循环;knob_count=4 基线）

批次：other_builtin_oven_qwen37max_20260617（dashscope qwen3.7-max / medium）。7 变体全部 compile=success、workbench-only、≥1 非 fixed joint。

## 组合数预审


## Slot 候选覆盖

### Slot A:door_mechanism（**主机构槽**——炉门开合）
| 候选 | record_id | 关键 joint | 状态 |
|---|---|---|---|
| drop_down_bottom_hinge（基线） | P1 / P2 | `body_to_door`/`door_hinge` REVOLUTE（+X 底沿）| parent |
| side_hinge_single | rec_variant-door-mechanism-side-hinge-single-re-hing_20260617_093413_094318_e2a585a2 | 侧铰单门 REVOLUTE（Z）| converged(2) |
| french_double_door | rec_variant-door-mechanism-french-double-door-replac_20260617_093413_094321_4f000702 | 法式双开 REVOLUTE ×2（Z,镜像）| converged(3) |

## Multiplicity / Copy Logic

- **count_param 1: `door_count`**（炉腔/炉门数,叠层双烤箱）
  - N 样本: {1(P1), 2}
  - N=2 → rec_variant-door-count-2-make-it-a-double-wall-oven-_20260617_093413_100629_3d0a0f67（双腔叠层,各自下翻门 + rack）
  - 模板建议 N_range: [1, 3]；copied object: 整个炉腔 + 下翻门;naming `door_{i}` / `for i in range(n)`;placement 沿 +Z 叠层;joint policy 每门独立 REVOLUTE
- **count_param 2: `rack_count`**（炉腔内滑出烤架数）
  - N 样本: {1(P1), 2, 3}
  - N=2 → rec_variant-rack-count-2-fit-the-oven-cavity-with-tw_20260617_093413_093646_303083cb
  - N=3 → rec_variant-rack-count-3-fit-the-oven-cavity-with-th_20260617_093413_100448_541d21da
  - 模板建议 N_range: [1, 5]；copied object: 单层线烤架;naming `rack_{i}`;placement 沿 +Z 等距;joint policy 每架独立 PRISMATIC(向用户 -Y/+X 拉出)
- **count_param 3: `knob_count`**（前面板旋钮数,源自 P2 微波炉）
  - N 样本: {2, 4(P2), 6}
  - N=2 → rec_variant-knob-count-2-reduce-the-front-control-ro_20260617_093915_350227_47d387a5
  - N=6 → rec_variant-knob-count-6-increase-the-front-control-_20260617_093915_360954_16d89608
  - 模板建议 N_range: [2, 8]；copied object: 单旋钮(KnobGeometry);naming `knob_{i}`(母资产已 `for i in range(4)`);placement 沿 Y 等距;joint policy 每钮独立 CONTINUOUS(+X 前法向)

## 格子覆盖

| 槽 | 候选数(含基线) | 空格已填 |
|---|---|---|
| A door_mechanism(主机构) | 3 | side_hinge / french_double(2) |
| multiplicity door_count | N∈{1,2} | N=2(1) |
| multiplicity rack_count | N∈{1,2,3} | N=2 / N=3(2) |
| multiplicity knob_count | N∈{2,4,6} | N=2 / N=6(2) |

door_mechanism ≥2(目标侧可补 warming-drawer 第 4 候选)+ 三根 multiplicity 轴各 ≥2 个 N → 满足 §8。**Built-in oven 小类样本池就绪(多重性极强)。**

## 排除项
- 无（7 变体全收敛）。门机构第 4 候选(抽屉式 warming-drawer 烤箱)留待后续补格。
