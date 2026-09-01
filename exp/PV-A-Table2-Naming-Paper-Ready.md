# PV-A Table 2 Naming: Paper-Ready Text and Experiment Notes

本文档只对应 PV-A paper 的 **Table 2: Naming matched comparison**。它分为两部分：第一部分是可直接复制到英文论文中的内容；第二部分是实验设计、统计口径、结果和限制的中文详解。

## 【1】第一部分：可直接复制粘贴到 PV-A Paper 的内容

### 1.1 Evaluation protocol

#### Semantic Part Naming

We evaluate semantic part naming on a category-matched panel containing four methods, five appliance categories (microwave, dishwasher, oven, faucet, and refrigerator), and seven assets per category and method, for a total of 140 assets. PV-A and Infinite Mobility use frozen seeds 0--6. For LAM and Articraft, we select seven assets per category from their official releases using an outcome-independent SHA-256 ordering. This protocol controls category composition and sample count, but it is not a same-prompt or same-seed comparison for LAM and Articraft.

We define a part as a URDF link containing at least one valid renderable visual geometry. Multiple visuals attached to the same link are merged into one part. Invalid or unsupported geometries and empty hierarchy links are excluded. We report the mean number of parts per asset and lexical nameability, where a name is considered lexically present if it does not match the frozen placeholder-name rules. Lexical nameability alone does not establish semantic correctness; in particular, automatically generated indices such as `l_0` are treated as opaque names.

To evaluate semantic correctness, we construct an output-independent role gold before inspecting the evaluated URDFs or link names. The gold specifies minimal required roles, functional-core flags, conservative synonyms, optional roles, and repeated-instance rules for each category. Optional components are evaluated only when visibly represented and their absence is never penalized. We then create 1,107 method-blind link-level tasks from the 140 assets. Each task contains an anonymous asset identifier, the category, the link name, and deterministic zero-pose previews showing the target geometry in red and the remaining asset in gray, together with isolated target views.

Three isolated Codex LLM judge sessions independently annotate all tasks without access to method identity, source records, aggregate results, or other judges' decisions. A semantic verdict requires at least two identical non-uncertain votes. The judges separately annotate name correctness, the geometry role, repeated-instance identity conveyed by the name, and whether multiple links represent fragments of the same semantic part. All 1,107 name verdicts obtain consensus. For metric-specific fields without an initial majority, the same judges independently re-review 72 anonymous items while prior votes remain hidden. Fifteen geometry-role fields that remain split are resolved by one fresh blind tie-break adjudicator that has access to neither method identity nor prior votes. The original name verdicts and all fields that already reached consensus remain locked.

Semantic precision is the fraction of renderable links whose names truthfully describe a required or additional real part. Semantic recall is the asset-macro average of the fraction of minimal required roles recovered by consensus-correct names. Functional naming richness follows our pre-specified PV-A definition: the asset-macro fraction of required functional roles that are correctly named. Extra real parts are reported separately and do not increase functional richness. Instance discriminability is the micro fraction of nodes in repeated consensus geometry-role groups that receive distinct, non-ambiguous identities from their names. The over-segmentation rate is the number of consensus same-part excess fragments divided by the number of consensus geometry-real links. We report 95% confidence intervals from 10,000 category-cluster bootstrap resamples, retaining all assets from each sampled category.

### 1.2 Results paragraph

#### Naming Results

Table 2 reports the matched Naming evaluation. PV-A achieves perfect semantic precision (172/172 = 1.000), the highest numerical asset-macro semantic recall (0.886; 95% CI [0.686, 1.000]), and the highest numerical functional naming richness (0.886; 95% CI [0.686, 1.000]) among the four evaluated methods, while maintaining low over-segmentation (3/172 = 0.017). However, PV-A's instance discriminability is only 0.277 (26/94), showing that its names often fail to distinguish repeated instances even when the underlying parts are semantically named.

LAM obtains semantic precision of 0.993, recall of 0.871, functional naming richness of 0.871, and the highest instance discriminability (0.966) in this panel. Articraft reaches perfect semantic precision and zero detected over-segmentation, with recall and functional richness of 0.833 and instance discriminability of 0.504. Infinite Mobility produces a larger number of renderable links and reaches lexical nameability of 1.000 under the generic placeholder rule, but every selected link is named using an opaque `l_<index>` identifier. Consequently, its semantic precision, recall, functional richness, and instance discriminability are all zero. This result demonstrates that lexical nameability must not be interpreted as semantic naming quality.

The three initial judges show high agreement on name correctness (mean pairwise exact agreement 0.992; Fleiss' kappa = 0.987). We do not claim statistical superiority from small numerical differences because the category-cluster confidence intervals overlap. Cross-seed values for PV-A and Infinite Mobility are raw-name set/multiset consistency diagnostics rather than semantic-role consistency. LAM and Articraft do not receive cross-seed scores because their release records do not expose reusable generator seed identities; grouping unrelated release assets by category would not define a valid cross-seed experiment.

### 1.3 Copy-ready Markdown table

**Table 2. Category-matched semantic Naming evaluation.** We evaluate five categories with seven assets per category and method (35 assets per method). Parts are URDF links with valid renderable visual geometry. Prec. is semantic precision; Rec. is asset-macro semantic recall; Func. Rich. is asset-macro functional naming richness under the pre-specified PV-A definition; Inst. is micro instance discriminability; X-Seed is raw-name set/multiset Jaccard and is not semantic-role consistency; OverSeg. is the micro over-segmentation rate. Semantic values use three isolated method-blind LLM judges, a field-only blind re-review, and a blind tie-break for 15/1,107 unresolved geometry-role fields.

| Method | Parts | Nameability ↑ | Prec. ↑ | Rec. ↑ | Func. Rich. ↑ | Inst. ↑ | X-Seed ↑ | OverSeg. ↓ |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| **PV-A** | 4.914 | 1.000 | 1.000 | 0.886 | **0.886** | 0.277 | 0.390 | 0.017 |
| LAM | 8.257 | 1.000 | 0.993 | 0.871 | 0.871 | 0.966 | N/A | 0.0069 |
| Articraft | 6.143 | 1.000 | 1.000 | 0.833 | 0.833 | 0.504 | N/A | 0.000 |
| Infinite Mobility | 12.314 | 1.000† | 0.000 | 0.000 | 0.000 | 0.000 | 0.813 | 0.000 |

† All 431 Infinite Mobility links use opaque `l_<index>` names. Its lexical nameability of 1.000 therefore does not imply semantic correctness. LAM and Articraft use category-matched, equal-N official-release samples rather than same-prompt or same-seed generations. Parts are not monotonically better when larger. Zero OverSeg. means that judges did not identify duplicate fragments; it does not by itself imply optimal part granularity. In this cohort, all minimal required roles are marked functional; Func. Rich. therefore equals Rec. and is not an independent axis.

### 1.4 Copy-ready limitations paragraph

#### Limitations of the Naming Evaluation

Our matched semantic evaluation is restricted to five appliance categories and therefore does not establish naming performance over the full PV-A category space. The annotations are produced by isolated LLM judge sessions rather than human annotators; although name-verdict agreement is high, shared model biases may remain. The comparison is category-matched and equal in sample count, but LAM and Articraft are evaluated using deterministic official-release resampling rather than common-prompt regeneration. Moreover, our evaluator counts renderable URDF links, whereas some prior work reports GLB mesh nodes, so absolute part counts should not be compared across representations without a shared converter. Finally, the Infinite Mobility instance score is computed over 162 nodes in 49 repeated known-role groups, while only 270/431 of its opaque-name nodes receive a consensus known geometry role. We therefore report its geometry-role coverage together with instance discriminability and avoid extrapolating the matched-panel semantic results to the full release datasets.

## 【2】第二部分：实验相关信息详细说明

### 2.1 这个实验补了哪些 N/A

此前 matched 主表已经能直接计算 Parts、词法 Nameability，以及 PV-A/Infinite Mobility 的 raw-name Cross-Seed proxy，但以下五个 headline semantic metrics 因为缺少输出无关的 gold 和三份独立 verdict 而保持 N/A：

1. Semantic Precision；
2. Semantic Recall；
3. Functional Naming Richness；
4. Instance Discriminability；
5. Over-Segmentation Rate。

本轮在完全相同的 140-asset matched cohort 上补齐了这五项，并额外报告 supplementary `Validated Named-Link Density` 和 extra-real-part count。它没有重新选择资产，也没有根据结果剔除失败样本。

### 2.2 Matched cohort 是怎样构造的

| Method | 资产来源 | 选择方法 | 每类数量 | 总资产 | 是否 same-prompt / same-seed |
|---|---|---|---:|---:|---|
| PV-A | 冻结的五个 PV-A templates | seeds 0--6 | 7 | 35 | 对自身为固定 seed |
| LAM | official viable release | 类别池内按结果无关 SHA-256 排序取前 7 | 7 | 35 | 否 |
| Articraft | official retained release | 类别池内按结果无关 SHA-256 排序取前 7 | 7 | 35 | 否 |
| Infinite Mobility | 五个 official factories | seeds 0--6 | 7 | 35 | 对自身为固定 seed |

共同类别是 microwave、dishwasher、oven、faucet、refrigerator。每方法都是 5 类 x 7 个资产 = 35，总计 140 个资产。这个设计严格控制了类别混合和每方法样本数，但不能声称 LAM、Articraft 与 PV-A 使用了相同 prompt 或相同 seed 语义。

冻结 matched protocol：[`table2_naming_matched_protocol_v1.json`](reference/table2_naming_matched_protocol_v1.json)，SHA-256：

```text
f4c4d915ce2e1bed5c99efd9dcebb97526a7d2639ad5021e3b068c3cf6105113
```

### 2.3 Part 的统一定义

评测单位不是 XML 中任意一个 `<link>`，而是至少包含一个有效 renderable visual geometry 的 URDF link：

- 有效 mesh 必须存在、非空且能解析；
- box 的三个边长必须为有限正数；
- cylinder 的 radius 和 length 必须为有限正数；
- sphere 的 radius 必须为有限正数；
- 同一 link 的多个 visuals 合并为一个 part；
- 空 group、无 visual link、无效或不支持的 geometry 不进入分母。

四方法最终 part 数分别为：PV-A 172、LAM 289、Articraft 215、Infinite Mobility 431，共 1,107 个 link-level tasks。

### 2.4 为什么需要单独构建 semantic gold

不能用某个方法输出的 link names 反推“正确名称”，否则 gold 会偏向该方法。因此 semantic gold 由独立上下文构建，只允许读取冻结类别定义和公开厂商产品/支持文档，不允许读取：

- `exp/runtime`；
- 任何评测 URDF；
- records、manifest 或汇总结果；
- 任一方法的 link names。

每类冻结的 minimal required roles 如下：

| Category | Minimal required roles |
|---|---|
| microwave | `microwave_housing`, `microwave_door` |
| dishwasher | `dishwasher_housing`, `dishwasher_door`, `dish_rack` |
| oven | `oven_housing`, `oven_door`, `oven_rack` |
| faucet | `faucet_body`, `spout` |
| refrigerator | `refrigerator_cabinet`, `refrigerated_compartment_door` |

每方法共有 84 个 required-role instances，四方法合计 336。所有 minimal required roles 在本版 gold 中同时标为 functional core，所以本轮 Functional Naming Richness 与 Semantic Recall 数值相同。这不是实现错误，而是当前 gold 的定义结果；两列回答不同概念，但在这个 gold 上分子和分母恰好相同。

Gold 还包含 conservative synonyms、optional roles、替代关系和 repeated-instance 规则。Optional role 只有在资产中作为独立可渲染部件出现时才参与名称真实性判断；它不存在时不扣 Recall。例如 microwave 不一定有独立 handle，faucet 不一定有 manual handle，refrigerator 不一定有独立 freezer drawer。

冻结 semantic gold：[`table2_naming_semantic_gold_v1.json`](reference/table2_naming_semantic_gold_v1.json)，SHA-256：

```text
4b7a7e8060a17a636d92073b8f0e31675a1ee8b87c3e3bb2100b7e8fb1f1eff2
```

### 2.5 匿名评审包与视觉证据

每个 link 生成一条 blind task 和一张 672 x 298 PNG 预览。预览包括：

1. target link 为红色、其余 links 为灰色的 context view；
2. target 的 isolated oblique view；
3. target 的 isolated side view；
4. 匿名 asset ID、类别和原始 node name。

Blind task 中没有 method、source ID、source path、URDF path 或任何 aggregate result。方法到 source 的映射只存在于独立 audit 文件，judges 被明确禁止读取。

全量视觉 QA 结果：

- 1,107/1,107 previews 存在且 SHA-256 与 manifest 一致；
- target red pixels 最小值 460，中位数 12,018；
- context gray pixels 最小值 760，中位数 14,968；
- 空白图、只有标签无几何的图：0。

Blind queue SHA-256：

```text
b390bb0ff98e8ef8b852c6ca0088f464fab4d1b1ec34c4e6cb2e5dd7dc0e2dc2
```

Preview set digest：

```text
9ea572ae9a42392044a5ebc3e1d93da2fa64086706ebe5bc83ace921952ae3ef
```

### 2.6 三审、复审和盲仲裁流程

#### 第一轮：三位隔离 LLM judges

三位 judges 分别在无共享上下文的 Codex session 中完成全部 1,107 项。每位只能看到 gold、semantic protocol、blind tasks 和 previews，不能读取 method audit、source records、其他 judge 文件或主表结果。

| Judge | spec_match | extra_real_part | invalid / opaque | uncertain | SHA-256 |
|---|---:|---:|---:|---:|---|
| A | 274 | 402 | 431 | 0 | `607b1b76fd664a9fd4bd042d1f77e22ed052fd31d639d0c2bd8383bdd1e8b21c` |
| B | 276 | 398 | 433 | 0 | `2a0af61f9d262fb124fec7f20a62e7ab7d4efa41394d76d834311a6defcf9ca8` |
| C | 279 | 395 | 433 | 0 | `4cb3af57d9778bf29ef20cb7e04daf3c08bebd712b988104c88b24578f3a13a0` |

Name-verdict agreement：

- pairwise exact：0.9892 / 0.9883 / 0.9973；
- mean pairwise exact：0.9916；
- unanimous rate：0.9874；
- Fleiss' kappa：0.9871。

Geometry-role 和 instance-id 的一致率较低，mean pairwise exact 分别为 0.6128 和 0.6917，因此这些字段不能直接使用单个 judge 的结果。

#### 第二轮：72-item field-only blind re-review

首轮中 name verdict、matched required role、geometry-real flag 和 same-part 字段都完整取得多数。Metric-specific 字段还有 66 个 geometry-role 和 6 个 instance-id 没有两票一致。三位原 judges 在仍看不到 method 和其他票的条件下，只复审这些字段；首轮已共识的字段全部锁定，不允许修改。

复审后 6 个 instance-id 全部解决，geometry-role 仍有 15 项三票分裂。

复审协议：[`table2_naming_semantic_adjudication_v1.json`](reference/table2_naming_semantic_adjudication_v1.json)，SHA-256：

```text
46f84e8f250311e12902bc2245b5e427e279692e099852a03c795bcd4fb0a7f8
```

#### 第三轮：15-item fresh blind tie-break

一位没有参加前两轮的新 Codex judge 只看到 15 个匿名任务、gold 和 previews，看不到任何历史票、method 或 audit。它只裁决仍缺失的 geometry-role，不能修改首轮 verdict 或其他共识字段。

- adjudicated items：15/1,107 = 1.36%；
- unique assets：13；
- adjudicator SHA-256：`1a141cac3a94a491187ee06f29159729af17ba0288c1a33acfdeda5a63eef74f`。

Tie-break protocol：[`table2_naming_semantic_tiebreak_v1.json`](reference/table2_naming_semantic_tiebreak_v1.json)，SHA-256：

```text
b122b1e15fc9c7115239f2c3ac3aa2872db490391ca508dfb0a403d0b697ad97
```

这套流程应在论文中表述为 **three isolated LLM judges with blind field-only re-review and a fresh blind tie-break**，不能写成 human study，也不能写成四位 judges 对全部 1,107 项投票。

### 2.7 五个 headline 语义指标如何计算

#### Semantic Precision

```text
consensus truthfully named renderable links / all judged renderable links
```

`spec_match` 和 `extra_real_part` 都进入分子。`l_0` 即使对应真实几何，因为名称没有提供语义，也记为 invalid/opaque，不进入分子。

#### Semantic Recall

先对每个 asset 计算：

```text
covered minimal required roles / minimal required roles
```

再对 35 个 assets 取平均作为主报告的 asset-macro Recall。同时报告 pooled micro counts，例如 PV-A 为 76/84 = 0.905 micro，但其 asset-macro 为 0.886。

#### Functional Naming Richness

每个 asset 的定义为：

```text
covered required functional roles / required functional roles
```

再做 asset macro。这个公式来自实验设计文档中预先写明的 `named functional parts / spec functional parts`，不会因为一个方法输出更多装饰 links、重复 required-role links 或 extra-real parts 而增大。当前 gold 的所有 minimal required roles 都标为 functional，因此本轮 Functional Richness 与 Recall 数值相同。未来若 gold 区分 structural-required 与 functional-required，两列才会产生不同数值。

旧版聚合曾把 `consensus truthfully named links / minimal required roles` 误标为 Naming Richness。该公式会受 part granularity 和 extra links 影响，现已按 metric-correction v1 改名为 supplementary `Validated Named-Link Density`。Extra-real parts 也单独报告，不进入 Functional Richness 分子。

Metric correction：[`table2_naming_semantic_metric_correction_v1.json`](reference/table2_naming_semantic_metric_correction_v1.json)。它不修改 cohort、gold、预览或 judge verdict，只修正聚合字段名称与 headline 公式。

#### Instance Discriminability

先用 consensus geometry-role 找到同一 asset 内至少出现两次的 role group，再检查 node name 是否真正传达不同 identity，例如 `left/right`、`upper/lower` 或稳定 ordinal。定义为：

```text
nodes with distinct non-ambiguous name-conveyed identities
/
nodes in repeated known geometry-role groups
```

`l_0`、`l_1` 的数字虽不同，但属于无语义 generator index，不能当作可解释的 instance identity。

#### Over-Segmentation Rate

Judges 指出多个 links 是否其实是同一个 semantic part 的碎片。对同一 semantic part 的 connected component，除一个主 node 外的其余 nodes 都是 excess fragments：

```text
same-part excess fragments / geometry-real renderable links
```

0 只表示本轮 judges 没有识别出重复碎片，不表示 part 数量、拓扑或分割粒度必然最优。

### 2.8 最终结果与分母

| Method | Precision | Recall asset-macro | Recall micro | Functional Richness asset-macro | Functional Richness micro | Instance | Over-seg |
|---|---:|---:|---:|---:|---:|---:|---:|
| PV-A | 172/172 = 1.000 | 0.886 | 76/84 = 0.905 | **0.886** | 76/84 = 0.905 | 26/94 = 0.277 | 3/172 = 0.017 |
| LAM | 287/289 = 0.993 | 0.871 | 73/84 = 0.869 | 0.871 | 73/84 = 0.869 | 115/119 = 0.966 | 2/289 = 0.0069 |
| Articraft | 215/215 = 1.000 | 0.833 | 71/84 = 0.845 | 0.833 | 71/84 = 0.845 | 57/113 = 0.504 | 0/215 = 0.000 |
| Infinite Mobility | 0/431 = 0.000 | 0.000 | 0/84 = 0.000 | 0.000 | 0/84 = 0.000 | 0/162 = 0.000 | 0/431 = 0.000 |

Primary category-cluster bootstrap 95% CIs：

| Method | Precision | Recall | Functional Richness | Instance | Over-seg |
|---|---|---|---|---|---|
| PV-A | [1.000, 1.000] | [0.686, 1.000] | [0.686, 1.000] | [0.000, 0.640] | [0.000, 0.047] |
| LAM | [0.982, 1.000] | [0.771, 0.967] | [0.771, 0.967] | [0.944, 1.000] | [0.000, 0.018] |
| Articraft | [1.000, 1.000] | [0.576, 1.000] | [0.576, 1.000] | [0.198, 0.833] | [0.000, 0.000] |
| Infinite Mobility | [0.000, 0.000] | [0.000, 0.000] | [0.000, 0.000] | [0.000, 0.000] | [0.000, 0.000] |

Supplementary granularity-sensitive diagnostics：

| Method | Validated Named-Link Density | Extra real parts | Extra real parts / asset |
|---|---:|---:|---:|
| PV-A | 1.995 | 64 | 1.829 |
| LAM | 3.424 | 202 | 5.771 |
| Articraft | 2.490 | 131 | 3.743 |
| Infinite Mobility | 0.000 | 0 | 0.000 |

Bootstrap unit 是 canonical category cluster。每次有放回地抽取 5 个类别，并保留被抽中类别内所有方法和资产，重复 10,000 次，seed=`260811003`。因为只有五个类别，CI 较宽，尤其不能把 PV-A Recall 0.886 与 LAM 0.871 的小差距声称为统计显著优势。

### 2.9 如何理解各方法结果

#### PV-A

- 优点：172/172 名称全部通过语义正确性共识；Recall 和 Functional Richness 数值均为本 panel 最高；Over-seg 很低。
- 主要问题：Instance 只有 26/94 = 0.277。它说明 repeated-role nodes 中，很多名称没有传达稳定、可解释的 instance identity。
- 不能直接下结论的内容：当前实验没有完成专门的 error taxonomy，不能仅凭分数断言低 Instance 是模板规则、ordinal 命名还是 segmentation 的单一原因。

#### LAM

- 287/289 names 被判为语义正确；只有 2 个未通过。
- Functional Richness=0.871，数值略低于 PV-A 的 0.886；两者 CI 重叠。
- Instance=0.966 是该 panel 的最高数值。
- Supplementary Validated Named-Link Density=3.424、extra-real-parts/asset=5.771；这些值反映更多正确命名 links 和更细粒度，不是 headline Functional Richness。
- 其 release 资产没有 reusable seed identity，所以 Cross-Seed 不能填。

#### Articraft

- Precision=1.000，未检测到 same-part excess fragment。
- Recall=0.833，低于 PV-A 和 LAM；Instance=0.504，处于 PV-A 与 LAM 之间。
- 与 LAM 一样，它是 official-release category-matched resample，不是共同 prompt 重跑。

#### Infinite Mobility

- Generic placeholder regex 下 Nameability=1.000，但 431/431 都是 `l_<index>`，所以 Semantic Precision、Recall、Functional Richness 和 Instance 均为 0。
- 431 个 nodes 中只有 270 个获得 consensus known geometry-role；Instance 只在其中构成的 49 个 repeated groups、162 个 nodes 上计算，结果 0/162。
- Over-seg=0 表示 judges 未识别出 same-part fragments，不抵消其命名语义为 0 的问题。
- Raw Cross-Seed Jaccard=0.813 主要来自稳定复用 `l_0`, `l_1`, ... 这一命名模式，不代表 semantic-role consistency。

### 2.10 可以写进论文的结论

可以写：

- PV-A 在五类 matched panel 上达到 1.000 semantic precision、数值最高的 0.886 asset-macro recall / functional richness，以及 0.017 over-segmentation；
- PV-A 与 LAM/Articraft 都能产生高精度的语义名称；
- LAM 在本 panel 中具有最高的 instance discriminability；
- LAM 的 supplementary named-link density 和 extra-real-part count 更高，但这些不作为 Functional Richness 正收益；
- 纯词法 Nameability 会把 Infinite Mobility 的 opaque indices 误判成已命名，semantic judging 能揭示这一问题；
- 三位隔离 LLM judges 对 name verdict 的 agreement 很高。

不能写：

- “PV-A 显著优于 LAM”，因为 Recall CI 明显重叠；
- “这是 human evaluation”，因为 judges 是 Codex LLM sessions；
- “四方法使用相同 prompt/seed”，因为 LAM、Articraft 是 release resample；
- “LAM/Articraft Cross-Seed=0”或任意伪造数值；它们是 N/A；
- “Infinite Mobility 没有真实部件”，它有真实 geometry，失败的是名称语义；
- “Over-seg=0 表示最佳分割”；
- 把本 matched 35-asset/method 的语义结果外推到 LAM 2,533、Articraft 242 或 Infinite Mobility 720 的完整 supplementary cohort；
- 把 URDF renderable-link Parts 与其他论文的 GLB mesh-node Parts 当作完全相同的计数单位。

### 2.11 复现与审计产物

| Artifact | Path | SHA-256 / status |
|---|---|---|
| Semantic protocol | [`table2_naming_semantic_protocol_v1.json`](reference/table2_naming_semantic_protocol_v1.json) | `45f2a1aea0051579e84a76f7dff888ab8cba78838af3c28e96f3390451e8e733` |
| Metric correction | [`table2_naming_semantic_metric_correction_v1.json`](reference/table2_naming_semantic_metric_correction_v1.json) | `964f76d75e20b570ac62d37cdeec56ae48ff047fd3cadf94895493d5df6326ef` |
| Semantic gold | [`table2_naming_semantic_gold_v1.json`](reference/table2_naming_semantic_gold_v1.json) | `4b7a7e8060a17a636d92073b8f0e31675a1ee8b87c3e3bb2100b7e8fb1f1eff2` |
| Final summary | [`summary.json`](runtime/table2_naming_semantic_v1/summary.json) | `a9fabcc8ed6566039e8b6b714dec3bdd0a2f64ee871354da8b0d1b3704f2bb85` |
| Consensus records | [`consensus_records.jsonl`](runtime/table2_naming_semantic_v1/consensus_records.jsonl) | `9b8c8a8926af3ef6fbd99c212cde52064031a0139394beed60e1ba2afa7c042e` |
| Asset records | [`asset_semantic_records.jsonl`](runtime/table2_naming_semantic_v1/asset_semantic_records.jsonl) | `941b74114b6616ee12dcc351a8e104d609b5d94b7e233115bdf5acf08e809fdb` |
| Human-readable report | [`report.md`](runtime/table2_naming_semantic_v1/report.md) | `fdaf4843e1eb6ce151279f3edabad05c4389587aa0571988349284dd7a801301` |
| Self-check | [`self_check.json`](runtime/table2_naming_semantic_v1/self_check.json) | 22/22 PASS; SHA `646e5f939f8c9e093d4cbeb092dad60e98ccdfa9fd38e0e3a8182ee7ef85010b` |

正式聚合器：[`aggregate_table2_naming_semantic.py`](scripts/aggregate_table2_naming_semantic.py)。Packet builder、field-only re-review builder 和 tie-break builder 分别为：

- [`build_table2_naming_semantic_packet.py`](scripts/build_table2_naming_semantic_packet.py)；
- [`build_table2_naming_semantic_rereview.py`](scripts/build_table2_naming_semantic_rereview.py)；
- [`build_table2_naming_semantic_tiebreak.py`](scripts/build_table2_naming_semantic_tiebreak.py)。

验证结果：

- 140 assets、1,107 tasks、1,107 previews；
- 四方法均为 35 assets，每方法每类均为 7；
- 五个 headline semantic metrics 与 supplementary named-link density 对四方法全部 non-null；
- self-check 22/22 PASS；
- `summary.json`、`consensus_records.jsonl`、`asset_semantic_records.jsonl`、`report.md`、`self_check.json` 和 `manifest.json` 连续两次聚合 SHA-256 逐字节一致；
- 所有新增脚本通过 `python3 -m py_compile`；
- repository `git diff --check` 通过。

### 2.12 投稿前最值得继续做的实验

当前结果足以进入 paper draft 和主表，但若要加强最终投稿的 semantic-quality claim，优先级最高的是：

1. 对方法 x 类别分层抽样 250--350 个 tasks，收集 2--3 位 human annotators，报告 human-human 与 human-LLM agreement；
2. 冻结当前主表，不修改 gold 或 test verdict，对 PV-A 的 26/94 Instance 结果做 error taxonomy；
3. 在资源允许时扩展到 10--15 个类别，检验五个 appliance categories 之外的泛化；
4. 只有在获得官方 API/config 和共同 generation prompt 后，才运行 LAM/Articraft same-prompt 生成；不要用 release 内不同资产伪造 Cross-Seed。

不建议对 LAM 2,533、Articraft 242、Infinite Mobility 720 个 release assets 全量重复 3-judge semantic evaluation。它会显著增加标注成本，却不能解决当前最主要的 human-validation、类别覆盖和 same-prompt 公平性问题。
