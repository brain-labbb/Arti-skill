# URDF Sim-Ready Automatic Evaluation: Tables 1--5

这是基于主文档 Table 1--5 整理的可直接渲染版本，并按当前展示范围保留其余实验组。Articraft-10K 的 Table 1 full visual+collision 结构重测同步至 2026-08-31（near-duplicate diagnostic 保留 2026-08-28），Table 2 的四门 strict 与 Infinigen-Sim resource-preflight 修订同步至 2026-08-29，Table 3 的 Articraft full-roster receipt 同步至 2026-08-29，Table 4 的 Articraft full v2 重测同步至 2026-08-31；Table 5 仍为 2026-08-27/28 的 N=200 hybrid run。本文件保留五张主表，并对 Table 1--4 补充指标含义、计算方法、逐实验组表现和异常高低原因；Table 0、category-macro/diagnostic 子表、Table 2 supplementary、Paper Table 2、Table 4a/4b、Table S1 以及逐资产运行 receipt 仍见 [原始评测文档](URDF-Sim-Ready-Automatic-Evaluation.md)。

## Common conventions

- Table 1--5 的成功率单元格只显示百分比（例如 `1.103%`）；解析失败、资源失败、runtime error 和 timeout 仍按冻结协议保留在分母中，不补抽样。
- 百分比沿用各主表的显示精度（Table 4 和 Table 5 为三位小数，其余为两位）；资产、关节和状态分母的定义见各表说明及原始评测文档。
- `N` 表示资产级冻结评测 cohort；Table 3 的关节指标以 `J_eval` 为分母，资产级 strict 指标仍以 `N_eval` 为分母。
- `N/E` = not evaluable（没有可用的冻结测量或零分母）；`TBD` = 尚未完成冻结；`PARTIAL` = 仅部分意图样本完成测量，但未从分母删除其余样本；`COMPLETE` = 预期测量全部完成。
- `*` 表示 Articraft-10K 的 merged source cohort：9,996 个发布包加 791 个 GitHub-only source reconstruction（`N=10,787`），不是声称存在字节级官方 10,787-package release。
- `Ours / PV-A` 的主表行使用完整 frozen release（`N_eval=302,440`，`J_eval=1,453,516`，531 个 generator classes）。
- `↑` / `↓` 只表示在该指标定义内数值越高 / 越低越好，不表示跨数据集的因果归因。结构复杂度、类别组成、测量 coverage 和 fail-closed 失败都必须与数值一起解释。

## Table 1. Dataset Scale and Structural Diversity

| Dataset / Outputs | N | Links/Asset (mean / P90) | Movable Joints/Asset (mean / P90) | Var. Joints ↑ | Near-Duplicate Rate ↓ | Multi-joint Assets (%) ↑ |
|---|---:|---:|---:|---:|---:|---:|
| Ours / PV-A | 302,440 | 6.56 / 12 (n=302,435) | 4.81 / 10 (n=302,435) | 29.96 | 36.64% | 84.55% |
| Articraft-10K (full visual+collision export, rerun 2026-08-31)* | 10,787* | 5.11 / 8 (n=10,787) | 3.71 / 6 (n=10,787) | 7.63 | 2.10%† | 8,935 / 10,787 (82.83%) |
| LAM released outputs | 3,217 | 6.09 / 11 (n=3,005) | 3.00 / 5 (n=3,005) | 2.19 | 4.54% | 53.59% |
| Artiverse | 3,544 | 8.22 / 16 (n=3,526) | 4.60 / 7 (n=3,526) | 12.76 | 7.11% | 77.65% |
| PartNet-Mobility | 2,347 | 7.10 / 11 (n=2,347) | 5.10 / 9 (n=2,347) | 24.52 | 7.93% | 59.86% |
| PhysX-Mobility | 2,024 | 12.85 / 19 (n=2,024) | 4.89 / 7 (n=2,024) | 14.83 | 6.92% | 55.09% |
| SketchMobility | 4,956 | 3.38 / 5 (n=4,949) | 2.22 / 4 (n=4,949) | 1.05 | 14.49% | 48.31% |
| Infinite Mobility (supplementary generated cohort) | 720 | 15.04 / 41 (n=720) | 6.56 / 16 (n=720) | 36.37 | 9.58% | 76.39% |
| Infinigen-Sim | 8,226 | 5.89 / 11 (n=8,226) | 3.89 / 9 (n=8,226) | 2.15 | 15.71% | 73.86% |

### Table 1 指标含义与计算方法

| 指标 | 详细含义与计算方法 |
|---|---|
| `N` | 实际进入该行结构评测的冻结 cohort 资产数。失败、缺失和不可解析资产仍留在该资产分母中；它是本地冻结 release/cohort 边界，不是论文宣称的历史规模。 |
| `Links/Asset` | 对每个 parse-valid 资产计数 URDF `<link>`，在有效结构记录上报告算术平均数和 nearest-rank P90，即排序后取 `sorted[ceil(0.9*n)-1]`。单元格中的 `n` 是实际可计算的结构记录数；例如 PV-A 的 `n=302,435` 表示 5 个资产没有进入结构均值，而不是从 `N` 中删除。 |
| `Movable Joints/Asset` | 对每个 parse-valid 资产计数声明层全部非 `fixed` XML joints，再以同样方法报告 mean / nearest-rank P90。它包含 exporter extension joint types，只说明声明结构，不等同于 Table 3 已验证的可执行 DoF。 |
| `Var. Joints` | 先在每个 dataset-local 冻结类别内，对 parse-valid 资产的 movable-joint 数计算总体方差 `Var_c=(1/n_c) * sum_i (J_i-mean_c)^2`（`ddof=0`，singleton 类为 0），再对所有可评类别等权平均：`(1/C) * sum_c Var_c`。因此它衡量“同一类别内部的关节数变化”，不是全库 joint count 的普通方差；SketchMobility 的类别单元为 `{source}/{category}`。 |
| `Near-Duplicate Rate` | 只比较 frozen category label 相同且 fixed-contracted canonical kinematic graph 完全相同的候选。对 unit-normalized visual surface point clouds 计算 symmetric mean unsquared-L2 Chamfer，距离 `< tau` 时连边；当前 `tau=0.011901784067423636`。每个连通分量贡献 `size-1`，资产行报告 `sum(size-1)/requested N`。解析或几何失败仍在 `N` 中，candidate recall、跨库类别映射和独立人工真值尚未建立，所以这是可复现的诊断性下界，不是重复率真值。Articraft 的 `†` 表示 2026-08-31 结构重测沿用此前冻结的 visual-point-cloud diagnostic lower bound，并非本次重测重新估计的 prevalence。 |
| `Multi-joint Assets` | `#{i: J_i >= 2}/N`，即至少声明两个非 `fixed` joints 的资产比例。不可解析项不会被当作通过；该指标仍只衡量声明结构。 |

### Table 1 各实验组表现与高低原因

- **Ours / PV-A：** `N=302,440` 远大于其他行，84.55% 为多关节资产；`Var. Joints=29.96` 也很高，说明 531 个 generator classes 内部存在不同配置规模。near-duplicate 下界 36.64% 为表内最高，符合类别级 procedural template 加 seed/configuration 变体被“同类别 + 同拓扑 + Chamfer 阈值”规则捕获的现象；但由于该指标没有独立人工真值，只能解释为候选下界偏高，不能直接认定 36.64% 的资产语义重复。
- **Articraft-10K：** 2026-08-31 full visual+collision export rerun 得到 5.11 links、3.71 movable joints、`Var. Joints=7.63`，8,935 / 10,787（82.83%）为多关节资产；10,787 / 10,787 个资产进入结构统计。near-duplicate 的 2.10%† 仍是此前冻结的 visual-point-cloud 诊断性下界，只在当前候选和阈值下成立，不证明不存在跨类别或候选召回之外的重复。
- **LAM released outputs：** 只有 3,005 / 3,217 个资产进入结构 mean/P90，反映 212 个资产没有 parse-valid 结构记录。其 movable joints 均值 / P90 为 3.00 / 5，multi-joint 为 53.59%，`Var. Joints=2.19`；这表示 dataset-local 标签内的 joint count 较稳定且大量资产只有 0--1 个 movable joint，不表示运动学与碰撞已经有效。
- **Artiverse：** links 均值 / P90 为 8.22 / 16，在固定对照中结构较细；movable joints 为 4.60 / 7，multi-joint 为 77.65%。`Var. Joints=12.76` 和 near-duplicate 下界 7.11% 处于中间水平；其多来源、人工整理的 release 构成与较宽的 link-count 尾部一致，但这里的统计本身不能把差异唯一归因于来源。
- **PartNet-Mobility：** movable joints 均值 5.10 较高，`Var. Joints=24.52` 也很高，但 multi-joint 只有 59.86%。两者并不矛盾：较多 0--1 joint 资产与少量高 DoF 资产可以同时形成较低 multi-joint 比例和较高均值/方差，说明结构复杂度分布不均匀。
- **PhysX-Mobility：** links 均值 / P90 为 12.85 / 19，是固定 release 对照中最高；movable joints 仅为 4.89 / 7，说明额外 link subdivision 并未按同样比例转化为 movable DoF。`Var. Joints=14.83`、multi-joint 55.09%，体现类别和部件拆分的异质性；link 多本身不是 sim-ready 质量分数。
- **SketchMobility：** links 3.38 / 5、movable joints 2.22 / 4、`Var. Joints=1.05` 和 multi-joint 48.31% 都偏低，说明 `{source}/{category}` 单元内多为较简单且 joint count 稳定的结构。near-duplicate 下界 14.49% 偏高，可能与四个上游来源中的模板/派生变体有关；7 个非 parse-valid 资产不进入 mean/P90，但仍保留在 `N` 和保守分母中。
- **Infinite Mobility：** 20 factories × 36 seeds 的 supplementary operational cohort 具有全表最高的 links（15.04 / 41）、movable joints（6.56 / 16）和 `Var. Joints=36.37`，显示明显的复杂度重尾和 factory 内配置变化。它不是官方固定 release，不能把这些高值解释成对其他 full releases 的同口径优势。
- **Infinigen-Sim：** movable joints 为 3.89 / 9，73.86% 是多关节资产，但 `Var. Joints=2.15` 很低，表示 17 个 observed labels 内 joint count 较规则。near-duplicate 下界 15.71% 较高，与程序化类别中共享拓扑的 seed 变体被当前候选规则捕获一致；仍需遵守代理下界的 claim boundary。

## Table 2. URDF Validity and Structural Integrity

| Dataset / Outputs | Resource Resolution ↑ | Valid Tree ↑ | Valid Joint Spec. ↑ | Collision Coverage ↑ | Strict URDF Pass ↑ |
|---|---:|---:|---:|---:|---:|
| Ours / PV-A | 100.00% | 100.00% | 99.93% | 99.32% | 99.25% |
| Articraft-10K (full visual+collision export, rerun 2026-08-29)* | 99.99% | 100.00% | 100.00% | 99.88% | 99.87% |
| LAM released outputs | 97.14% | 92.26% | 95.68% | 45.97% | 41.09% |
| Artiverse | 99.97% | 99.49% | 99.89% | 97.57% | 97.15% |
| PartNet-Mobility | 98.59% | 100.00% | 99.79% | 0.00% | 0.00% |
| PhysX-Mobility | 100.00% | 100.00% | 100.00% | 0.00% | 0.00% |
| SketchMobility | 99.03% | 99.86% | 99.96% | 38.60% | 38.58% |
| Infinigen-Sim | 99.95% | 100.00% | 100.00% | 0.00% | 0.00% |

### Table 2 指标含义与计算方法

所有列都是资产级 gate，统一计算为 `通过该 gate 的资产数 / N_eval`。解析、资源、runner 和 timeout 失败均留在 `N_eval` 并 fail closed。

| 指标 | 详细含义与通过条件 |
|---|---|
| `Resource Resolution` | URDF 引用的全部 mesh、texture 和 material 均能在冻结 package containment 内定位，文件存在、可读、格式受支持，且几何不是 empty/non-finite。任一必需引用失败则整个资产失败。 |
| `Valid Tree` | link-joint graph 恰有一个 root、无环、无 orphan，且所有声明 links 都可由 root 到达。它检查拓扑，不检查运动语义是否符合真实物体。 |
| `Valid Joint Spec.` | 每个 joint 的 parent/child link 存在且不同，axis 非零；bounded joint 满足有限且 `lower < upper` 的范围，continuous joint 不错误声明有限区间。资产内任一 joint 不合格则该资产失败。 |
| `Collision Coverage` | 所有声明 links（包括 dummy/base/root/world link）都有可加载 collision geometry，是严格的“全 link 覆盖”资产 gate。`0%` 可能表示每个资产都至少缺一个 link，而不一定表示全库完全没有 collision elements。 |
| `Strict URDF Pass` | 对每个资产计算 `Resource Resolution AND Valid Tree AND Valid Joint Spec. AND Collision Coverage`，再以 `N_eval` 聚合。它不是各列百分比相乘，也不能由列级边际比例反推。 |

2026-08-29 的展示口径已把 parser compatibility 从 headline 表移除，只保留在逐资产 diagnostic provenance 中。`Finite Fields`、`Inertial Coverage` 和 `Inertia Validity` 也不进入当前四门 strict；因此这里的高分不等于质量、惯量或动力学参数已经完整。

### Table 2 各实验组表现与高低原因

- **Ours / PV-A：** Resource Resolution 和 Valid Tree 均为 302,435 / 302,440，Valid Joint Spec. 为 302,221 / 302,440，Collision Coverage 为 300,390 / 302,440，最终 strict 为 300,176 / 302,440。99.25% 的高 strict 来自接近完整的结构与资源闭包；剩余损失主要由 2,050 个 collision-coverage 失败贡献，其次是 joint spec，5 个资源/树失败也按完整分母保留。
- **Articraft-10K：** 2026-08-29 的 full visual+collision rerun 中，10,787 个资产全部通过 tree 和 joint spec；唯一 resource failure 是 unresolved material，13 个资产至少缺一个 collision link，strict 为 10,773 / 10,787（99.87%）。这是独立的 Table 2 export/rerun；Table 4 使用 2026-08-31 的 full-v2 collision rerun，两个表的 coverage 差异来自不同冻结运行，不是同一记录自相矛盾。
- **LAM released outputs：** Resource Resolution 97.14%，但 Valid Tree 92.26%、Valid Joint Spec. 95.68%，Collision Coverage 只有 45.97%，四门交集进一步降到 41.09%。低 strict 的首要瓶颈是全 link collision coverage，非法树和 joint references/specifications 又从已覆盖资产中继续扣分；这是多个 gate 的联合失败，不应只归因于 XML parse。
- **Artiverse：** 四个单项都在 97.57% 以上，strict 为 97.15%。其主要瓶颈仍是 Collision Coverage（3,458 / 3,544），其次是 18 个非 valid-tree 资产；因为失败集合部分重叠，strict 通过数为 3,443。
- **PartNet-Mobility：** resource、tree 和 joint spec 分别为 98.59%、100% 和 99.79%，但 2,314 个资源闭包完整资产都至少缺少空 `base` link 的 collision，其余 33 个还有资源失败；因此没有资产满足全声明-link覆盖，Collision Coverage 和 strict 均为 0%。这不表示每个资产都没有 collision geometry；Table 4 能在其他 collision-bearing links 上执行部分测量，两个表回答的是不同问题。
- **PhysX-Mobility：** resource、tree 和 joint spec 全部为 100%，但官方 URDF 的 collision element 总数为 0，因此 Collision Coverage 和 strict 都为 0%。这是“没有可审计 collision representation”的结构性失败，绝不能解释为没有检测到碰撞或机械上安全。
- **SketchMobility：** resource、tree 和 joint spec 都接近 100%，但仅 1,913 / 4,956 个资产满足全 link collision coverage；strict 为 1,912 / 4,956（38.58%），几乎完全由 coverage gate 控制。48 个 resource failures 都来自 `Infinigen/Pliers` OBJ/MTL 空格路径在该冻结 runner 中被拆成缺失引用；7 个 tree failures 都是 `root_count=2`，另有 2 个 invalid lower/upper joint specs。混合来源 release 中是否存在部分 collision elements 不改变全声明-link gate 的判定。
- **Infinigen-Sim：** corrected rerun 后 Resource Resolution 为 8,222 / 8,226（99.95%）；剩余 4 个 refrigerator 资产包含 empty/non-finite mesh geometry。tree 和 joint spec 全部通过，但每个资产的 dummy `world` link 都缺 collision，因而没有资产达到严格的全声明-link覆盖，strict 为 0%。其他 links 仍有大量可用于 Table 4 contact query 的 geometry。

## Table 3. Kinematic Executability

| Dataset / Outputs | Valid Range ↑ | Joint Sweep Success ↑ | Non-degenerate Motion ↑ | Subtree Consistency ↑ | FK Round-trip Error ↓ | Joint-level Pass ↑ | Strict Kinematic Pass ↑ |
|---|---:|---:|---:|---:|---:|---:|---:|
| Ours / PV-A | 99.92% | 99.92% | 99.92% | 98.36% | 0.000000 normalized translation / 9.424322e-08 rad rotation (1,452,330 / 1,453,516 measured; PARTIAL) | 98.16% | 99.17% |
| Articraft-10K (full visual+collision export, rerun 2026-08-29)* | 100.00% | 99.98% | 99.98% | 99.37% | 0.000000 normalized translation / 2.980232e-08 rad rotation (40,034 / 40,041 measured; PARTIAL) | 99.23% | 99.55% |
| LAM released outputs | 99.49% | 85.60% | 85.30% | 85.56% | 0.000000 normalized translation / 2.107342e-08 rad rotation (8,886 / 10,381 measured; PARTIAL) | 85.24% | 87.72% |
| Artiverse | 99.93% | 99.31% | 96.97% | 99.31% | 0.000000 normalized translation / 0.000000e+00 rad rotation (16,220 / 16,332 measured; PARTIAL) | 96.97% | 94.05% |
| PartNet-Mobility | 99.60% | 99.60% | 99.39% | 99.57% | 0.000000 normalized translation / 2.980232e-08 rad rotation (11,923 / 11,971 measured; PARTIAL) | 99.31% | 99.11% |
| PhysX-Mobility | 100.00% | 100.00% | 26.64% | 100.00% | 0.000000 normalized translation / 2.107342e-08 rad rotation (9,883 / 9,883 measured; COMPLETE) | 26.63% | 59.19% |
| SketchMobility | 99.98% | 99.89% | 82.29% | 99.85% | 0.000000 normalized translation / 2.107342e-08 rad rotation (10,997 / 11,009 measured; PARTIAL) | 82.18% | 71.95% |
| Infinigen-Sim | 100.00% | 100.00% | 80.53% | 100.00% | 0.000000 normalized translation / 0.000000e+00 rad rotation (31,975 / 31,975 measured; COMPLETE) | 80.53% | 69.44% |

**Articraft Table 3 current receipt.** 本行已同步原文证据段所绑定的 2026-08-29 full visual+collision receipt：`N=10,787`、`J=40,041`，10,786 个资产完成、1 个 unsupported `floating` joint 资产 retained error；Valid Range 为 40,040 / 40,041，Sweep 与 Non-degenerate 均为 40,034 / 40,041，Subtree 为 39,790 / 40,041，FK round-trip measured 为 40,034 / 40,041，Joint-level 为 39,734 / 40,041，Strict Kinematic 为 10,738 / 10,787。旧的 2026-08-27 `J=40,037` merged receipt 不再用于本行；证据见 `runtime/articraft_github_merged_10787_20260829/full/table3/summary.json`。

### Table 3 测试状态与计算方法

bounded joint 在含 lower/upper 的 `K=21` 个均匀状态上测试；continuous joint 使用冻结标准区间（例如 `[-pi, pi]`）。每次只改变一个 joint，其余 joints 固定在 `q0=0`（若 0 不在声明区间则 clip 到区间内）。除 `Strict Kinematic Pass` 外，各成功率均以全部声明非 `fixed` joints 的 `J_eval` 为分母；无法执行或未测量的 joints 不从分母删除。

| 指标 | 详细含义与计算方法 |
|---|---|
| `Valid Range` | joint 有非空、有限、可生成测试状态的区间。通过率为 `range-valid joints / J_eval`；zero-width、非有限、非法或 evaluator 不支持的 range 失败。 |
| `Joint Sweep Success` | 该 joint 的全部 `K` 个状态都能由冻结 FK engine 执行，且全部 link transforms 为有限值。任一状态失败即整 joint 失败。 |
| `Non-degenerate Motion` | bounded revolute/prismatic joint 比较 lower 与 upper；continuous joint 为避免 `-pi/+pi` 同姿态，比较 `q0` 与全部 `K=21` 个采样状态的最大 excursion。至少一个 descendant link 的 bbox-diagonal-normalized translation 严格大于 `1e-6`，或 rotation 严格大于 `1e-6 rad`，才通过。因此 full-turn **bounded revolute** 的两个端点仍可能同姿态并失败。 |
| `Subtree Consistency` | 驱动 joint 时，变化只应出现在其 descendant subtree；任一 non-descendant link 的 translation 或 rotation 超过 `1e-9` 即失败。bbox scale 可得时 translation 除以 diagonal；scale 不可得时冻结实现比较 raw translation。rotation 始终以 rad 比较。 |
| `FK Round-trip Error` | 对每个可测 joint 执行 `q0 -> q1 -> q0`，取所有 links 回到 `q0` 后的最大残差；bbox scale 可得时 translation 除以 diagonal，scale 不可得时冻结实现比较 raw translation，rotation 直接以 rad 报告，二者通过阈值均为 `1e-9`。表中数值是所有已测 joints 的最大值，括号给出 `measured / J_eval`；`PARTIAL` 下的 0 只约束已测子集。 |
| `Joint-level Pass` | 单个 joint 同时通过 Valid Range、Sweep、Non-degenerate、Subtree 和 Round-trip；通过率为 `passed joints / J_eval`。 |
| `Strict Kinematic Pass` | 单个资产的全部声明 non-fixed joints 都达到 Joint-level Pass，再以 `passed assets / N_eval` 聚合。任一 joint 或资产级执行失败都会使资产失败；冻结协议中的 zero-movable-joint 资产也 fail closed。 |

### Table 3 各实验组表现与高低原因

- **Ours / PV-A：** range、sweep 和 non-degenerate 都为 99.92%，Subtree Consistency 为 98.36%，Joint-level 为 98.16%。主要损失来自 25,518 个已测 round-trip residual 超过 `1e-9`，其中大部分也表现为 subtree residual；不能把 Joint-level 下降单独归因于 Subtree gate。仍有 99.17% 资产 strict 通过，说明失败 joints 集中在少数资产。另有 1,186 个 joints 未完成 round-trip，故即使已测 translation 最大为 0，也必须标记 `PARTIAL`；`9.424322e-08 rad` 是最坏残差而非平均误差。
- **Articraft-10K：** 最新 2026-08-29 full visual+collision rerun 中，Valid Range 为 40,040 / 40,041（100.00%），Sweep 与 Non-degenerate 均为 40,034 / 40,041（99.98%），Subtree 为 39,790 / 40,041（99.37%），Joint-level 为 39,734 / 40,041（99.23%），Strict 为 10,738 / 10,787（99.55%）。7 个 joints 未完成 FK round-trip，另有 1 个含 unsupported `floating` joint 的资产 retained error；这些项均按冻结规则保留在分母中。
- **LAM released outputs：** Valid Range 仍达 99.49%，但 Sweep 只有 85.60%，随后 Non-degenerate、Subtree 和 Joint-level 都约为 85%。冻结 full-release records 中 249 个 invalid-tree 资产以及 12 个 initial-FK errors（11 个 `floating`、1 个 `planar`）是从 range 到 sweep 大幅下降的主因。Strict asset pass 为 87.72%，高于 joint-level 比例是因为失败 joints 集中在部分坏资产；8,886 / 10,381 的 round-trip coverage 仍是 `PARTIAL`。
- **Artiverse：** range 99.93%、sweep/subtree 99.31%，但 Non-degenerate 和 Joint-level 都为 96.97%。除 112 个未成功 sweep 的 joints 外，383 个已执行的 non-degenerate failures 由 371 个 full-turn bounded revolute endpoint aliases 和 12 个位移小于 `1e-6` 的 prismatic joints 构成；主要问题因此不是 FK engine 执行。资产级 all-joints conjunction 将 96.97% 的 joint pass 放大为 94.05% strict；round-trip 的 0 只覆盖 16,220 / 16,332 个 joints。
- **PartNet-Mobility：** 各 joint gate 均在 99.31% 以上，Strict 为 99.11%，是 full-release 对照中很高的一组。48 个 joints 未进入有效 sweep，另有少量尺度不可得的 prismatic joints 和 `1e-9 rad` 级 residual/subtree 失败；这些小集合解释了从 99.60% sweep 到 99.31% Joint-level 的差异。
- **PhysX-Mobility：** Range、Sweep、Subtree 都是 100%，但 Non-degenerate 只有 26.64%，Joint-level 26.63%，Strict 59.19%，是最显著的异常。冻结 full-release records 中所有 2,024 个资产的 kinematic scale 都因 URDF 的 sibling `../partseg/...` layout 被当时的 containment rule 标为不可评测；7,250 / 7,250 个 prismatic joints 因无法归一化 translation 而 fail closed，2,633 个 revolute joints 则全部通过 non-degenerate，随后只有 1 个因 round-trip residual 未达到 Joint-level。因此低值主要反映该冻结 full-release runner 的 scale/resource-layout 覆盖，不应直接解释为 73% 的 joints 物理上不运动；后续 staged N=800 诊断属于另一 cohort，不能回填本行。
- **SketchMobility：** Range/Sweep/Subtree 接近 100%，但 Non-degenerate 为 82.29%，直接把 Joint-level 拉到 82.18%，再经资产级全 joint 交集降到 Strict 71.95%。记录显示 1,938 个已成功 sweep 的 non-degenerate failures 精确分为 1,574 个 prismatic 和 364 个 bounded revolute，continuous 为 0：前者所在资产缺少冻结 backend 可用的 kinematic scale（2,149 个 `.gltf` 与 21 个 `.glb` 资产不受支持），后者来自 192 个 `[0,2pi]` Skateboard joints，以及 122 个 Microwave 和 50 个 Dishwasher 的 `[-pi,pi]` joints，两个端点在 FK 中同姿态。因此 82.29% 主要反映尺度 coverage 与 bounded-endpoint 定义，不等同于 sweep 中没有运动；另有 12 个 joints 未完成 round-trip，故误差为 `PARTIAL`。
- **Infinigen-Sim：** Range、Sweep、Subtree 和 FK coverage 都是 100%，但 6,225 / 31,975 个 joints 未通过 Non-degenerate，Joint-level 为 80.53%、Strict 为 69.44%。失败精确分为 6,213 个 `[-pi,pi]` bounded revolute endpoint aliases，以及 12 个来自 4 个 refrigerator 资产、因 non-finite OBJ vertex 使尺度为 `N/E` 的 prismatic joints；资产 strict 的 2,514 个失败还包括 94 个 zero-movable `trash` 资产。因此低值不是 sweep、subtree 或 round-trip 失败造成的，也不能单独当作 joint semantic correctness。

Table 3 只证明冻结 URDF 描述在离散 FK 协议下的可执行性；它不验证 joint type、axis、origin、limit 与真实物体是否语义正确，也不验证动力学和碰撞。

## Table 4. Collision and Mechanical Clearance

| Dataset / Outputs | Rest All-pair CF ↑ | Rest Non-adjacent CF ↑ | Single-joint Sweep CF ↑ | Multi-joint Sobol CF ↑ | Collision-state Rate ↓ | AOR ↓ | Max Penetration ↓ | Collision-free Range ↑ | Strict Collision Pass ↑ |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Ours / PV-A (full v2; corrected final) | 1.103% | 71.952% | 65.850% | 65.218% | 30.243% | N/E | 0.845122 (302,377 / 302,440 measured; PARTIAL) | 67.487% | 64.521% |
| Articraft-10K (full visual+collision export; full v2 rerun 2026-08-31)* | 6.730% | 85.019% | 72.977% | 71.586% | 19.180% | N/E | 0.823099 (10,785 / 10,787 measured; PARTIAL) | 78.575% | 70.631% |
| LAM released outputs | 0.280% | 14.827% | 12.558% | 11.937% | 86.728% | N/E | 0.896712 (1,320 / 3,217 measured; PARTIAL) | 10.844% | 11.750% |
| Artiverse | 1.354% | 40.745% | 35.299% | 34.509% | 56.969% | N/E | 0.689776 (3,515 / 3,544 measured; PARTIAL) | 28.160% | 31.546% |
| PartNet-Mobility | 2.983% | 77.503% | 73.583% | 72.646% | 34.888% | N/E | 0.646960 (2,313 / 2,347 measured; PARTIAL) | 56.097% | 70.942% |
| PhysX-Mobility | N/E | N/E | N/E | N/E | N/E | N/E | N/E | N/E | N/E |
| SketchMobility (v2 sparse-equivalent) | 0.666% | 44.068% | 39.508% | 39.972% | 53.359% | N/E | 0.539062 (2,732 / 4,956 measured; PARTIAL) | 48.866% | 37.591% |
| Infinigen-Sim | 1.143% | 83.102% | 61.330% | 61.999% | 24.909% | N/E | 0.809588 (8,226 / 8,226 measured; COMPLETE) | 71.037% | 57.695% |

### Table 4 状态、碰撞规则与计算方法

rest state 在 v1 中为所有可评 joints 的 `q=0`；mimic-aware v2 对 independent roots 采样，并按 `q_follower=multiplier*q_target+offset` 展开 followers。令 `d_i` 为资产 `i` 在冻结计划中的独立采样轴数：v1 使用全部声明 non-fixed joints，非法或不可执行 range 仍计入 planned axes、相应 states 记为 unexecuted；v2 只使用 mimic 传播后 range-evaluable 且正宽的 independent roots，单点 fixed roots 和 followers 不进入 `d_i`。于是 `T_rest=N_eval`、`T_single=sum_i(21*d_i)`、`T_sobol=sum_i(64*1[d_i>0])`，总状态分母 `T=T_rest+T_single+T_sobol`。只使用 collision geometry、无 visual fallback；表面接触允许，仅把 penetration 严格大于 `1e-6 m` 判为非法。未执行状态和运行失败全部按 non-free 计入预期分母。

| 指标 | 详细含义与计算方法 |
|---|---|
| `CF` | Collision-free。资产级 CF 的一般形式是 `#{资产在该 phase 的全部预期状态均无非法穿透}/N_eval`；一个状态失败或缺失就使该资产在该 phase 失败。 |
| `Rest All-pair CF` | 在 rest state 检查所有不同 collision-bearing source-link pairs，包括 direct parent-child。它会把邻接几何的超阈值穿透/重叠计为失败；普通表面接触仍允许，因此该列主要是严格诊断项。 |
| `Rest Non-adjacent CF` | 在 rest state 排除 URDF graph 的 direct parent-child pairs 后检查其余 pairs。它是 headline pair policy 的 rest 视图；没有按方法或资产添加事后 allowlist。 |
| `Single-joint Sweep CF` | 对某资产的所有 single-joint `K=21` 状态取全称条件：全部状态都 free 才通过；再除以 `N_eval`。 |
| `Multi-joint Sobol CF` | 对某资产的全部 `R=64` Sobol states 取全称条件；zero-DoF、无有效 range 或状态不完整的资产按冻结规则 fail closed。 |
| `Collision-state Rate` | `(实际观测为 collision 的 states + 未执行 states) / 全部 intended rest+single+Sobol states`。它是状态加权 micro rate；高 DoF 资产贡献更多 single states，且它不是 `1-Strict Collision Pass`。 |
| `AOR` | Average Overlap Ratio，精确 collision-geometry overlap volume 相对部件体积的平均比例。当前主表没有冻结稳定的 exact-volume backend，全部为 `N/E`；不得用 AABB overlap、contact count 或 penetration depth 代填。 |
| `Max Penetration` | 每个资产在 rest 取 all-pair 最大深度，在 single/Sobol 取 non-adjacent 最大深度，三者的最大值除以该资产 rest collision-shape union AABB diagonal `D_i`；最后跨所有已有 finite observations 的资产/状态取全局最大。它是单个最坏值而非均值；括号中的 `measured` 是完成全部 intended states 的资产数，不是唯一进入最大值聚合的资产集合。 |
| `Collision-free Range` | `single-joint sweep 中 free states / 全部 intended single-joint states`，是 state/joint-range 加权比例。它和资产级 Sweep CF 的权重不同，数值可高也可低。 |
| `Strict Collision Pass` | 同一资产在预注册 non-adjacent pair policy 下同时通过 rest、全部 single-joint 和全部 Sobol tests；任何 coverage、range、runtime 或状态完整性失败均使该资产失败。 |

资产级 Rest/Single/Sobol/Strict 的分母都是 `N_eval`；Collision-state Rate 的分母是 `T`，Collision-free Range 的分母仅是 `T_single`。`N/E` 表示无合法测量或零分母，不是 0；`PARTIAL` 表示并非所有资产都完成全部 intended states，Max 仍可纳入 incomplete 资产中已经观测到的 finite states，但不对未执行 states 作深度断言。其他 pass/rate 始终使用完整 fail-closed 分母。

### Table 4 各实验组表现与高低原因

- **共同现象：** 所有具有 collision geometry 的行，其 Rest All-pair CF 都只有 0.280%--6.730%，而排除 direct parent-child 后显著升高到 14.827%--85.019%。差值对应只被 direct parent-child 超阈值穿透挡住的资产，例如 PV-A 有 214,278 个、Infinigen-Sim 有 6,742 个；普通表面接触本身允许。极低 all-pair 因而主要是邻接-link穿透诊断，不能单独用来宣称资产整体碰撞质量很差。
- **Ours / PV-A：** corrected full v2 在完整 `N=302,440` 上得到 Rest Non-adjacent 71.952%、Single 65.850%、Sobol 65.218%、Strict 64.521%，状态碰撞率 30.243%。从 rest 到 motion phase 的下降说明一部分资产静止时无非邻接穿透、运动后才碰撞；63 个 retained high-link backend errors 及其未执行状态也 fail closed，但只占很小部分。该行覆盖完整 531 类 release，数值应结合更宽的类别与难度分布解释。
- **Articraft-10K：** 2026-08-31 full v2 rerun 使用当前 10,787-asset visual+collision export，全部资产均有原生 collision geometry；Rest All-pair / Non-adjacent 为 6.730% / 85.019%，Single / Sobol 为 72.977% / 71.586%，Strict 为 70.631%，Collision-state Rate 为 19.180%。10,785 / 10,787 个资产完成全部 intended states，两个 retained error（food-processor 的 unsupported `floating` joint、parabolic-dish 的 q=0 不在声明范围）留下 170 / 1,530,841 个未执行状态并按 fail closed 计入；因此当前差异主要反映实际观测到的运动穿透与严格三阶段交集，而不是旧 9,996+791 source parent 的 coverage 缺口。Max Penetration 为 0.823099（10,785 / 10,787 measured; PARTIAL）。
- **LAM released outputs：** Rest Non-adjacent 14.827%、Strict 11.750% 和 Collision-free Range 10.844% 都是表内最低，Collision-state Rate 86.728% 最高；Max Penetration 0.896712 也最高。只有 1,320 / 3,217 个资产完成全部 intended states，1,330 个资产至少有 finite observation；238,821 / 421,026（56.724%）个 states 未执行并 fail closed，而已执行 states 中仍有 126,326 / 182,205（69.332%）实际碰撞。Table 2 的 collision coverage 45.97% 与 tree/spec failures 解释 coverage 部分，但低值不能只归因于 missing geometry。
- **Artiverse：** 3,515 / 3,544 个资产完成全部 intended states，3,520 个至少有 finite penetration observation，coverage 明显高于 LAM/Articraft；Rest Non-adjacent 40.745%，Single/Sobol 35.299% / 34.509%，Strict 31.546%，状态碰撞率 56.969%。未执行 states 只占 2.339%，所以该组中等偏低的 rate 主要反映实际观测到的非邻接与运动穿透；完整 coverage 不足仍使 Max Penetration 标为 `PARTIAL`。
- **PartNet-Mobility：** Rest Non-adjacent 77.503%、Single 73.583%、Sobol 72.646%、Strict 70.942%；在当前冻结表的完整 comparison-release 行中，该 strict 数值最高，但因协议版本混合，不构成统一排名。2,313 / 2,347 个资产完成全部 intended states，2,314 个至少有 finite observation。状态碰撞率为 34.888%，未执行 states 只占 1.062%，所以该 rate 主要来自实际检测穿透。Table 2 的 Collision Coverage 为 0% 是“每个声明 link 都有 collision”的资产级全覆盖 gate，而本表可在其他 collision-bearing links 上运行；这里的高分不能补写成 Table 2 strict 通过。
- **PhysX-Mobility：** 官方 URDF 没有任何 collision elements，eligible geometry 和有效 contact denominator 均不存在，因此整行必须为 `N/E`。把空 contact query 写成 100% 会产生 vacuous pass，本表明确禁止把它解释为 collision-free 或 mechanically safe。
- **SketchMobility：** v2 sparse-equivalent 的 Rest Non-adjacent 为 44.068%，Strict 为 37.591%，状态碰撞率 53.359%；2,732 / 4,956 个资产完成全部 intended states，2,779 个至少有 finite penetration observation。215,997 / 553,266（39.040%）个 states 未执行并 fail closed，已执行 states 中还有 79,218 / 337,269（23.488%）实际碰撞。Table 2 仅 38.60% 的资产达到全 link coverage；v2 又使 45 个 `q=0`/range 项变为 measurement-incomplete，所以 coverage 是主要下拉因素但不能解释全部差距。
- **Infinigen-Sim：** 8,226 / 8,226 个资产都有 penetration 测量且所有 states 均执行；Rest Non-adjacent 83.102% 为 full-release comparison 中最高，Collision-state Rate 24.909% 最低，但 Single/Sobol 仅 61.330% / 61.999%，Strict 为 57.695%。因此损失来自实际观测到的运动穿透和三 phase 交集，而非 coverage；Table 2 的 0% 仍是更严格的全声明-link coverage gate，不能用本行替代。

`Max Penetration` 跨行尤其不能脱离 coverage 比较：Articraft-10K 的 0.823099（10,785 / 10,787 measured; PARTIAL）、PV-A 的 0.845122 来自 `threestage_telescoping_slide/seed_0126`（0.50856 m / 0.60176 m），LAM 的 0.896712 来自 industrial belt conveyor 的 single-joint sample 10（10.002 m / 11.154 m），Infinigen-Sim 的 0.809588 来自 `refrigerator/1376` 的 drawer sweep sample 19（1.31967 m / 1.63005 m）。SketchMobility 较低的 0.539062 来自一个 completed Dishwasher Sobol outlier，但全库只有 2,732 / 4,956 个资产完成全部 states，不能据此声称其全库穿透更浅。当前 `AOR` 全部 `N/E`，本表没有体积重叠严重度的独立排序。

主表还混合了已明确标注的协议版本：PV-A 与 Articraft-10K 是 full v2，SketchMobility 是 v2 sparse-equivalent，其余完成行沿用 independent-sampling v1。在所有方法完成同一 mimic-aware 协议前，数值可用于审计现有冻结结果，但不应无说明地做严格方法排名。

---

## Table 5a. Per-Simulator Runtime Readiness

> **N=200 hybrid run (2026-08-27/28).** This update covers the six requested comparison sets, with 200 assets per dataset (`N_eval = 200`, 1,200 assets total). Articraft-10K is a GitHub-source-driven cohort: the source roster is [records_manifest.jsonl](Articraft-10K-github/records_manifest.jsonl) under `/mnt/zsn/lyb/arti-skill/exp/Articraft-10K-github`, while the runtime packages are the corresponding 9,996 locally materialized packages. Its parent roster was deterministically hash-ranked and the final manifest stores the first 200 rows; the other five cohorts use their exact stored first-200 rows. No replacement, resampling, or outcome filtering was used. The source manifest SHA256 is `296961b6926a56d61f1832d0289fd9f22ae0cb58f239d10f7373a2b7d2996b2b` and the source Git commit is `677ca9722427dce500873730255874c8c3f07eb2`.
>
> **Execution scope.** Genesis ran the full Table 5 protocol. PyBullet and MuJoCo ran strict `Load` only. All 3,600 planned simulator records (six datasets x three simulators x 200 assets) are terminal and schema-valid; `COMPLETE` means record completeness, not success. Metrics outside the execution plan are `N/E`, never zero.
>
> **Evidence.** [manifest.json](runtime/table5_n200_articraft_github_20260827/manifest.json), [aggregate summary](runtime/table5_n200_articraft_github_hybrid_20260827/aggregate/summary.json), [Table 5a CSV](runtime/table5_n200_articraft_github_hybrid_20260827/aggregate/table5a.csv), [Table 5b CSV](runtime/table5_n200_articraft_github_hybrid_20260827/aggregate/table5b.csv), and [run report](runtime/table5_n200_articraft_github_hybrid_20260827/aggregate/report.md). Manifest canonical SHA256 (receipt field): `fb08d2120c4eacbb50ac3d4d7a83c30bfeb7e1099e490a63d56150d12c6e7a5b`; manifest file SHA256: `aecf7473e5c72840adeb9f374469dfd0c0372fe6513b958fb5d6b9b960190691`; aggregate summary SHA256: `82df76371911a3100ce1377b62651c4037d48762ee2cfe0159172dab8f66674a`; protocol SHA256: `2694fdcfe1b5dde787e2cc03bae9c38292525147644a27ba959ef75f03d38322`.

| Dataset / Outputs | Simulator | Load Rate ↑ | Reset Success ↑ | Settling Stability ↑ | Actuation Success ↑ | Limit Enforcement ↑ | Constraint Drift ↓ | Simulator Pass ↑ |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| Articraft-10K | PyBullet | 100.000% | N/E | N/E | N/E | N/E | N/E | N/E |
| Articraft-10K | Genesis | 73.000% | 96.500% | 52.500% | 45.000% | 7.500% | 1.000% | 0.000% |
| Articraft-10K | MuJoCo | 0.000% | N/E | N/E | N/E | N/E | N/E | N/E |
| LAM released outputs | PyBullet | 85.500% | N/E | N/E | N/E | N/E | N/E | N/E |
| LAM released outputs | Genesis | 43.000% | 84.000% | 58.500% | 42.000% | 18.000% | 28.500% | 0.500% |
| LAM released outputs | MuJoCo | 0.000% | N/E | N/E | N/E | N/E | N/E | N/E |
| Artiverse | PyBullet | 99.500% | N/E | N/E | N/E | N/E | N/E | N/E |
| Artiverse | Genesis | 33.500% | 89.500% | 70.000% | 61.500% | 2.000% | 79.000% | 0.000% |
| Artiverse | MuJoCo | 0.000% | N/E | N/E | N/E | N/E | N/E | N/E |
| PartNet-Mobility | PyBullet | 98.000% | N/E | N/E | N/E | N/E | N/E | N/E |
| PartNet-Mobility | Genesis | 0.000% | 96.500% | 96.500% | 0.000% | 0.000% | 0.000% | 0.000% |
| PartNet-Mobility | MuJoCo | 0.000% | N/E | N/E | N/E | N/E | N/E | N/E |
| PhysX-Mobility | PyBullet | 100.000% | N/E | N/E | N/E | N/E | N/E | N/E |
| PhysX-Mobility | Genesis | 0.000% | 93.000% | 26.000% | 93.000% | 1.500% | 93.000% | 0.000% |
| PhysX-Mobility | MuJoCo | 0.000% | N/E | N/E | N/E | N/E | N/E | N/E |
| SketchMobility | PyBullet | 56.500% | N/E | N/E | N/E | N/E | N/E | N/E |
| SketchMobility | Genesis | 85.500% | 98.000% | 59.500% | 51.000% | 3.500% | 22.500% | 1.000% |
| SketchMobility | MuJoCo | 0.000% | N/E | N/E | N/E | N/E | N/E | N/E |

### Table 5a metric definitions

| Metric | Pass condition |
|---|---|
| `Load Rate` | 仿真器成功创建完整 multibody；link/joint 数量与解析后的 URDF 一致。任何自动丢弃的 link 或 joint 均判为失败。 |
| `Reset Success` | 连续执行固定次数的 load、reset 和 joint-state initialization，无崩溃、超时或非有限状态。 |
| `Settling Stability` | 在冻结的被动仿真时长内无 NaN、约束断裂、异常速度或超过阈值的非预期漂移。 |
| `Actuation Success` | 对每个关节施加统一的归一化目标后，实际运动达到至少 90% 的声明运动范围。 |
| `Limit Enforcement` | 施加越界目标时，实际 joint state 未超过声明 limit 与数值容差。 |
| `Constraint Drift` | 运动过程中关节锚点误差、轴向误差和禁止自由度位移的最大值。表中报告尺度归一化后的最大值。 |
| `Simulator Pass` | 同一资产在该仿真器中同时通过 Load、Reset、Settling、Actuation、Limit 和 Drift 检查。 |

每个仿真器分别使用其公开、冻结的稳定配置。三者的 timestep、gravity、base policy、初始状态、目标轨迹、运行时长和成功阈值保持一致；solver 实现差异不得通过修改资产来消除。

---

## Table 5b. Cross-Simulator Consistency and Overall Sim-Readiness

| Dataset / Outputs | PyBullet Pass ↑ | Genesis Pass ↑ | MuJoCo Pass ↑ | All-three Load ↑ | All-three Runtime Pass ↑ | Cross-sim Joint RMSE ↓ | Cross-sim Link-pose Error ↓ | Strict Sim-ready ↑ |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| Articraft-10K | N/E | 0.000% | N/E | 0.000% | N/E | N/E | N/E | N/E |
| LAM released outputs | N/E | 0.500% | N/E | 0.000% | N/E | N/E | N/E | N/E |
| Artiverse | N/E | 0.000% | N/E | 0.000% | N/E | N/E | N/E | N/E |
| PartNet-Mobility | N/E | 0.000% | N/E | 0.000% | N/E | N/E | N/E | N/E |
| PhysX-Mobility | N/E | 0.000% | N/E | 0.000% | N/E | N/E | N/E | N/E |
| SketchMobility | N/E | 1.000% | N/E | 0.000% | N/E | N/E | N/E | N/E |

### Table 5b metric definitions

| Metric | Definition |
|---|---|
| `PyBullet/Genesis/MuJoCo Pass` | 对应 Table 5a 的资产级 Simulator Pass。 |
| `All-three Load` | 同一资产可以在三个仿真器中全部成功加载。 |
| `All-three Runtime Pass` | 同一资产在三个仿真器中全部达到 Simulator Pass。 |
| `Cross-sim Joint RMSE` | 相同归一化目标轨迹下，三个仿真器 joint trajectory 的最大两两 RMSE；revolute 和 prismatic 分别报告。 |
| `Cross-sim Link-pose Error` | 对齐共同 root frame 后，三个仿真器最终 descendant-link pose 的最大两两误差；平移和旋转分别报告。 |
| `Strict Sim-ready` | 同一资产同时达到 Strict URDF Pass、Strict Kinematic Pass、Strict Collision Pass、All-three Runtime Pass，并满足冻结的 cross-simulator consistency 阈值。 |

Cross-simulator agreement 衡量实现一致性，不代表动力学参数与现实世界真值一致。

本次 `PARTIAL` 运行中，PyBullet/MuJoCo 的完整 runtime pass、all-three runtime、cross-simulator diagnostics、strict consistency 和 Strict Sim-ready 均不在执行计划内，因此统一记为 `N/E`；`All-three Load` 已按三者 Load 结果评测，表中的 `0.000%` 是实际失败而非未执行。下面保留的 N=800 段落仅用于历史审计，不能与本 N=200 面板合并或覆盖。

### Historical N=800 receipts (audit only)

下列 N=800 receipt 段落是在本次回填前写成的历史记录；其中旧的状态性措辞及其 `TBD` 描述只适用于旧 cohort，不代表上方 N=200 `PARTIAL` 面板的状态。

Artiverse formal N=800 result note: the cohort is the original ordered .assets[].manifest_root sequence from exp/runtime/table1_artiverse/manifest.json, with no resampling or replacement. Each simulator uses the full intent denominator 800; preflight_failure, diagnostic_failure, native_crash, timeout, and worker_error remain in the denominator. Terminal counts: PyBullet completed=784, preflight_failure=3, timeout=13; Genesis completed=754, diagnostic_failure=22, native_crash=5, preflight_failure=3, timeout=12, worker_error=4; MuJoCo completed=662, diagnostic_failure=134, preflight_failure=3, timeout=1. Upstream strict gates are Strict URDF 774 / 800 (96.750%), Strict Kinematic 762 / 800 (95.250%), and Strict Collision 254 / 800 (31.750%); Strict Sim-ready is their conjunction with all-three runtime and cross-simulator thresholds.

Cross-sim values are pairwise maxima over evaluable units: revolute joint n=914, prismatic joint n=1591, and link-pose n=4274; the table separates joint RMSE by joint type and link-pose translation / rotation. PyBullet 3.2.7 and MuJoCo 3.10.0 used CPU; Genesis 1.3.1 used the CUDA backend with one worker on GPU3 (UUID GPU-ebc0d328-a3fa-7e89-2733-cadb001661f7). The formal publication is at /root/.cache/torch/arti-skill/table5_artiverse_table1_n800_gpu_v4/aggregate/formal; the independent published-output validator passed. Cohort SHA256: 3e12e86fa61b9af14a411a2571c100e49f3ad49f6286394453366a64caeeb171. Protocol semantic SHA256: ebd1e6599f782511b0974208a0294cb2e42a7f1645614ac9a4e49df13c91e551.
PhysX-Mobility N=800 was drawn from the official Caoza/PhysX-Mobility release (revision d0768ee9e1415f6be8db78d6389ba018b85134c0, candidate closure 2024) by ascending (rank_sha256, integer dataset_id) order with rank salt arti-skill-table5-physx-mobility-n800-v1; no resampling or outcome filtering. The official URDFs contain zero collision elements, so Genesis 1.3.1 ran on the CPU backend with collision disabled, and Strict Collision Pass / Strict Sim-ready are N/E rather than 0. PyBullet 3.2.7 and MuJoCo 3.10.0 used CPU; Genesis load is reported under the exact-count load contract (any dropped/folded structure fails). The formal publication is at /mnt/zsn/lyb/arti-skill/exp/runtime/table5_physx_mobility_n800_v2/aggregate/formal; the publication self-check passed. Cohort SHA256: a9c9c710d9617dea366696603984e330780ce177fead2a34c60410588cc1273c. Protocol semantic SHA256: 4403a4190e2393c2812cf25193cbc6a08e75b350e65f47302db7f7c8a7321101.
Historical N=800 Articraft-10K Table 5 Genesis-only phase on the frozen Table 2 N=800 cohort (exact .records[].package stored order, seed 20260813; ordered manifest-root hash equals the Table 2 selected_asset_ids_sha256 79c44441600077513d3cde1cda8fef38324e1a0ee660730b860d5313f0ae9784). Genesis 1.3.1 used the CUDA backend with one worker on GPU3 (UUID GPU-ebc0d328-a3fa-7e89-2733-cadb001661f7) under the same frozen runtime configuration as Artiverse. Assets whose URDF lacks complete inertial data crash the Genesis native parser; those crashes are retained as terminal failures (514 malformed_response, 3 native_crash, 4 timeout, 5 worker_error, 1 diagnostic_failure, 273 completed) and fail closed in every metric with the full intent-to-evaluate denominator of 800. Constraint Drift additionally fails closed for the 577 assets without a Table 4 bounding-box normalizer. Strict Collision Pass / Strict Sim-ready are N/E in this phase; All-three and cross-simulator cells were TBD in that historical phase. The formal publication is at /root/.cache/torch/arti-skill/table5_articraft10k_table2_n800_gpu_v1/aggregate/formal; the publication self-check and an independent terminal-record recomputation both passed. Cohort SHA256: 5fb34ea89d43ace197a0a6431e031aaaafa66ebad89af8702dc26b19ef08b06a. Protocol semantic SHA256: fcb76c4e63dc56b5ebc1330bfd7c10ef85a958cd040fa4feb6c65122b91013bc.
Historical N=800 LAM released outputs Table 5 Genesis-only phase on the user-specified Table 3 formal N=800 cohort (the exact stored order from `exp/runtime/urdf_table3_lam_n800_seed20260813_20260814T022400Z_v3/asset_records.jsonl`, seed 20260813; no resampling, replacement, or outcome filtering). Genesis 1.3.1 used the CUDA backend with one worker on GPU3 (UUID GPU-ebc0d328-a3fa-7e89-2733-cadb001661f7); 707 records carry a matching CUDA device receipt (356 `completed`, 350 `malformed_response`, and 1 `diagnostic_failure`). The other 22 `diagnostic_failure` records failed during adapter initialization, two timed out, and two were retained as frozen GPU-gate `worker_error`; none fell back to CPU. Terminal counts are 356 `completed`, 23 `diagnostic_failure`, 350 `malformed_response`, 67 `preflight_failure`, 2 `timeout`, and 2 `worker_error`; all remain in the full 800 denominator. Strict Collision Pass / Strict Sim-ready are N/E in this Genesis-only phase; All-three and cross-simulator cells were TBD in that historical phase. The formal publication is at `/root/.cache/torch/arti-skill/table5_lam_table3_n800_gpu_v1/aggregate/formal`; the publication self-check and an independent terminal-record recomputation both passed. Cohort SHA256: 57bc9f95ccda4c8e1f0ba80a2048c8086ddcf47b8cd860a9913b5766af1bda6c. Protocol semantic SHA256: dc8aa5a84a43f1e0cec916185e7d49352059204363c405fd5a09a75ebc978595.

---
