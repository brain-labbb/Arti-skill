# Other / Folding screen — template source map

pattern: mixed（linear_chain 主体 + 固定 named slots:panel_infill + frame_detail;**外加** panel_count 多重性轴,核心）

parents（1 个母资产,三扇中式折叠屏风/隔断）:
- rec_model-a-three-panel-chinese-style-folding-screen_20260610_084402_869340_89fc1fef ← picture/Other/Folding screen/001.png（基线 = infill:fret_lattice × panel_count:3（中扇固定 + 两 wing REVOLUTE）× frame:flat_top）

批次：other_folding_screen_qwen37max_20260617（dashscope qwen3.7-max / medium）。8 变体全部 compile=success、workbench-only、≥1 非 fixed joint。

## 组合数预审


## Slot 候选覆盖

### Slot A:panel_infill（**部件词汇表槽**——扇面填充）
| 候选 | record_id | 结构特征 | 状态 |
|---|---|---|---|
| fret_lattice（基线） | parent | CadQuery 中式冰裂格 | parent |
| solid_painted_panel | rec_variant-panel-infill-solid-painted-panel-replace_20260617_104551_570471_4466e1a5 | 实心画板 | converged(2) |
| shoji_paper_grid | rec_variant-panel-infill-shoji-paper-grid-replace-th_20260617_104551_570180_c59458fc | 障子方格 + 纸面(循环 muntins) | converged(2) |
| louvered_slats | rec_variant-panel-infill-louvered-slats-replace-the-_20260617_104551_575330_8707f11c | 横向百叶片(循环) | converged(2) |

### Slot B:frame_detail（边框/顶冠/底脚)
| 候选 | record_id | 结构特征 | 状态 |
|---|---|---|---|
| flat_top（基线） | parent | 平顶横档 | parent |
| arched_crown_top | rec_variant-frame-detail-arched-crown-top-replace-th_20260617_105044_951969_073d817a | 弧形顶冠(lathe/CadQuery) | converged(2) |
| base_feet | rec_variant-frame-detail-base-feet-add-a-wider-foot-_20260617_105101_951980_63c52429 | 底脚加宽鞋(inline) | converged(2) |

## Multiplicity / Copy Logic（核心轴）

- count_param: **`panel_count`**（折扇数；基线 3）
- N 样本已覆盖: {2, 3, 4, 6}
  - N=2 → rec_variant-panel-count-2-make-it-a-two-panel-foldin_20260617_104551_571492_5e7c8b58（1 hinge）
  - N=3 → parent（中扇固定 + 2 wing hinge,parallel_children）
  - N=4 → rec_variant-panel-count-4-make-it-a-four-panel-foldi_20260617_104551_569191_40969f37（3 hinge,linear_chain）
  - N=6 → rec_variant-panel-count-6-make-it-a-six-panel-foldin_20260617_104925_490905_98759298（5 hinge,linear_chain）
- 模板建议 N_range: **[2, 12]**（屏风折扇数可较大,sweep 友好）
- copied object: 整扇(框 + infill),由共享 helper(`_add_panel` / `_fret_lattice`)发射
- naming: 变体已用 `panel_{i}`/`for i in range(n)`(母资产为 named center+wing,N≥4 变体已 linear_chain 循环化,可作 module 源码)
- placement: 沿 X 等距铰接成手风琴链
- joint policy: **linear_chain**——`panel_i` 铰接 `panel_{i-1}`,垂直 REVOLUTE,交替折向(N=3 母资产为 parallel_children,模板侧统一为 chain)

## 格子覆盖

| 槽 | 候选数(含基线) | 空格已填 |
|---|---|---|
| A panel_infill | 4 | solid / shoji / louvered(3) |
| B frame_detail | 3 | arched_crown / base_feet(2) |
| multiplicity panel_count | N∈{2,3,4,6} | N=2 / N=4 / N=6(3) |

每槽 ≥2 + multiplicity 4 个 N → 满足 §8。**Folding screen 小类样本池就绪(核心 multiplicity 极强)。**

## 排除项
- 无（8 变体全收敛）。
