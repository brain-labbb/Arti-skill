# Table 3: Expanded Hierarchy Evaluation (N=150)

## 【1】PV-A Paper Copy-Ready English Text, Caption, and Results Tables

### Hierarchy Evaluation

We evaluate hierarchy properties on a frozen expanded cohort of five common articulated-object categories: storage furniture/cabinet, table, refrigerator, dishwasher, and microwave. The balanced generated-method comparison contains 30 requested assets per category (`N=150`) for PV-A, Articraft, and Infinite Mobility. PV-A and Infinite Mobility use the frozen seeds 0--29; Articraft uses a frozen identity-only SHA-256 selection from rating-4/5 records in the official release and is freshly compiled once per selected record. Selection is completed before package availability, parsing, structure, or ontology-proxy scoring is inspected, and failures remain in the requested denominator without replacement. This is a final-package comparison across heterogeneous generation interfaces, not a common-prompt end-to-end reliability benchmark.

We first check whether each available final URDF is single-rooted, connected, and acyclic. Tree depth is the longest root-to-link path with the root counted as one, and movable joints include all non-fixed URDF joints. Node, depth, and joint statistics are equal-category macro means conditional on valid trees. Thus, larger values in these columns do not by themselves indicate better hierarchy quality.

#### Main Structural Results

| Method | Available / Requested | Valid Tree / Available | Valid Tree / Requested | Nodes (95% CI) | Kinematic Tree Depth (95% CI) | Movable Joints (95% CI) |
|---|---:|---:|---:|---:|---:|---:|
| **PV-A (ours)** | 125/150 | 125/125 (100.0%) | 125/150 (83.3%) | 4.950 [4.735, 5.165] | 2.200 [2.200, 2.200] | 3.859 [3.649, 4.073] |
| **Articraft** | 142/150 | 142/142 (100.0%) | 142/150 (94.7%) | 7.937 [7.449, 8.476] | 2.925 [2.837, 3.016] | 6.369 [5.933, 6.877] |
| **Infinite Mobility** | 146/150 | 146/146 (100.0%) | 146/150 (97.3%) | 10.365 [9.843, 10.904] | 3.069 [3.023, 3.113] | 4.911 [4.558, 5.269] |

All available packages in the balanced generated-method panel are valid trees. The requested-denominator rates retain 25 PV-A, eight Articraft, and four Infinite Mobility cases without final available packages; they must not be read as conditional structural quality. Infinite Mobility has a deeper representation, while Articraft has more movable joints among valid packages. These are representation diagnostics, not semantic-parent correctness measures.

### PartNet-Ontology-Referenced Semantic Hierarchy Alignment

We separately assess semantic recoverability against frozen PartNet category ontologies. PartNet supplies a category-level semantic part--whole ontology; it is not instance-matched geometry or independently annotated URDF parent--child ground truth. For each predicted package, final link labels are deterministically mapped to ontology roles, wrapper nodes without a mapped role are collapsed, and the reference parent for a mapped child is the nearest ontology ancestor role present in that predicted asset. Repeated instances remain separate link records.

Let `f_i` be the induced-edge F1 on the common mapped-role support of requested asset `i`, and let `c_i` be mapped predicted links divided by all final URDF links. We report Coverage-Weighted Induced Edge F1,

$$
S_m = \frac{1}{150}\sum_{i=1}^{150} c_i f_i,
$$

where unavailable and unscorable requested assets contribute zero. This ontology alignment proxy penalizes unrecoverable prediction-side semantics but cannot establish a unique physical decomposition or an instance-level parent--child annotation result. All reported confidence intervals use 10,000 category-stratified bootstrap replicates with equal category weight and fixed seed `20260813`. Methods are resampled independently within category, with no selection-rank pairing; pairwise intervals are unadjusted exploratory 95% percentile intervals.

| Method | Role Coverage / Requested | Scorable / Requested | Coverage-Weighted Induced Edge F1 (95% CI) | Induced Edge F1 / Requested (95% CI) | Induced Exact / Requested (95% CI) | Semantic-Parent Alignment / Requested (95% CI) |
|---|---:|---:|---:|---:|---:|---:|
| **PV-A (ours)** | 62.7% [58.2, 67.0] | 72.0% [66.0, 78.0] | 45.5% [41.0, 50.1] | 66.2% [61.3, 71.1] | 63.3% [58.7, 68.0] | 72.0% [66.0, 78.0] |
| **Articraft** | 50.7% [47.6, 53.9] | 83.3% [77.3, 88.7] | 44.8% [40.9, 48.6] | 80.0% [73.9, 85.6] | 75.3% [68.0, 82.0] | 80.2% [74.2, 85.9] |
| **Infinite Mobility** | 42.4% [40.2, 44.3] | 60.0% [60.0, 60.0] | 4.4% [4.1, 4.7] | 13.3% [13.3, 13.3] | 0.0% [0.0, 0.0] | 10.0% [10.0, 10.0] |

For the primary proxy, PV-A minus Articraft is +0.72 percentage points (95% CI [-5.09, +6.79]); this interval does not establish superiority or equivalence. PV-A minus Infinite Mobility is +41.10 percentage points (95% CI [+36.57, +45.63]) under this PartNet-ontology alignment proxy only. These difference intervals are computed directly from independent within-category resamples, not by subtracting marginal confidence-interval endpoints and not by rank pairing. Infinite Mobility uses package-local raw part-name metadata only to decode opaque final identifiers; this is prediction-side metadata, never reference hierarchy gold. Its unavailable four packages and 56 available-but-unscorable packages remain in the requested denominator.

### LAM Full Strict Release Audit (Supplementary Only)

LAM cannot enter the balanced `N=150` generated-method block. The intended balanced panel has 150 requested cases but only 93 observed official-release cases; the remaining 57 would require fresh generation, so the balanced result is `BLOCKED_INCOMPLETE_NOT_A_PAPER_RESULT`. We instead disclose a separate, severe class-imbalance release audit over all 300 strict official-release assets: storage furniture/cabinet 237, table 21, refrigerator 6, dishwasher 14, and microwave 22. It is supplementary only and is neither ranked with nor compared pairwise against the balanced methods.

| Supplementary release audit | Requested | Valid Tree, equal-category macro | Role Coverage, equal-category macro | Scorable, equal-category macro | Coverage-Weighted Induced Edge F1, equal-category macro (95% CI) |
|---|---:|---:|---:|---:|---:|
| LAM full strict, severely unbalanced | 300 | 93.5% | 63.6% | 82.4% | 47.1% [40.4, 54.2] |

### Artiverse Curated-Dataset Reference (Supplementary Only)

Artiverse is a curated pre-release dataset reference, not a generation method. It is excluded from generated-method rankings and pairwise method claims. Its primary reference panel has four categories--storage furniture, refrigerator, dishwasher, and microwave--with 30 requested assets each (`N=120`). The release has no exact `table` category. A separate five-category `N=150` sensitivity adds `coffee_table` as a non-exact table alias; it is never pooled with or substituted for the primary four-category result.

| Curated-dataset reference panel | Available / Requested | Valid Tree / Requested | Kinematic Tree Depth | Movable Joints | Role Coverage / Requested | Scorable / Requested | Coverage-Weighted Induced Edge F1 (95% CI) |
|---|---:|---:|---:|---:|---:|---:|---:|
| Artiverse primary four-category reference | 120/120 | 120/120 (100.0%) | 2.767 | 5.658 | 68.7% | 59/120 (49.2%) | 18.9% [15.8, 21.9] |
| Artiverse `coffee_table` alias sensitivity only | 150/150 | 150/150 (100.0%) | 2.727 | 4.993 | 75.0% | 89/150 (59.3%) | 34.7% [32.1, 37.3] |

The 15.8-point difference between the disclosed primary and alias-sensitivity point estimates is a category-schema sensitivity, not an improvement attributable to Artiverse. Artiverse articulation annotations are audited for availability but are not supplied to the PartNet scorer and are not treated as independent hierarchy gold.

### Suggested Caption

**Table 3: Structural validity and PartNet-ontology-referenced semantic hierarchy alignment on frozen expanded cohorts.** The balanced generated-method block contains PV-A, Articraft, and Infinite Mobility only, with five categories and 30 requested assets per category (`N=150` per method). Failures remain in requested denominators without replacement; node, depth, and movable-joint means are equal-category macro means conditional on valid final trees. Coverage-Weighted Induced Edge F1 multiplies each asset's induced Edge F1 by prediction-side mapped-link coverage and macro-averages over all requested assets, assigning unavailable and unscorable assets zero. Confidence intervals use 10,000 category-stratified bootstrap replicates with fixed seed `20260813`, independent within-category resampling across methods, and no rank pairing; pairwise intervals are unadjusted exploratory 95% percentile intervals. LAM full strict is a severely unbalanced supplementary release audit (`N=300`) and the intended balanced LAM `N=150` panel is blocked; neither is a main-method comparison. Artiverse is a supplementary curated pre-release dataset reference whose primary panel has four categories (`N=120`); the `coffee_table` `N=150` result is an explicitly non-exact sensitivity. PartNet-based measurements are ontology alignment proxies, not instance-level parent--child annotations.

## 【2】中文详细实验解释

### 1. 这张表回答什么，不能回答什么

本表分两层。结构层检查最终 URDF 是否形成单根、连通、无环树，并描述其深度、组织节点和可动关节数量。语义层使用冻结的 PartNet category ontologies，检查最终 package 中可恢复的预测侧部件标签能否诱导出与该类别 ontology 一致的 parent--child 边。第二层的正式名称必须统一为 **Coverage-Weighted Induced Edge F1 / ontology alignment proxy**，不得改称为真实运动学层级结果。

这不是 instance-matched PartNet mesh 对齐，也没有独立人工标注的运动学 parent--child gold。相同功能部件可以被合并到一个刚体、通过 fixed joint 表示，或使用不同但合法的 rigid decomposition。因此，结构树有效和 ontology 对齐都不能推出唯一正确的物理 URDF。

### 2. 主块的样本与分母

主块只包括 PV-A、Articraft、Infinite Mobility，均为五类各 30、总 `N=150` 的平衡 cohort。三种分母不可混用：

| 分母 / 状态 | 定义 | 本文如何使用 |
|---|---|---|
| Requested | 冻结后要求评估的资产数；失败不补抽 | 所有 requested-rate、Coverage-Weighted Induced Edge F1、scorable coverage |
| Available | 有最终可读取 package/URDF 的资产数 | `Valid Tree / Available` |
| Valid | available 中单根、连通、无环的最终树 | nodes、depth、groups、movable joints 等描述均值 |
| Scorable | 可从预测侧恢复足够 ontology roles，并形成至少一条可比较 induced edge 的资产 | 条件 Edge F1，以及 scorable / requested 的可审计分母 |

PV-A 是 `125/150` available 且 valid，故结构描述均值只对 125 个 valid assets 聚合；其余 25 个冻结请求仍以零贡献留在 requested-denominator 的 proxy 指标中。Articraft 为 `142/150`，Infinite Mobility 为 `146/150`，同理处理。主结构表的 nodes、depth、movable joints 是五类 equal-category valid-only macro，避免因不同失败率或类别可用数导致 pooled mean 过度加权。不能将 available 分母的 100% tree validity 与 requested 分母的 83.3%、94.7%、97.3% 混为同一种“成功率”。

Named Groups 未进入 copy-ready 主表，因为 aggregate 未为该量统一 equal-category bootstrap 口径。作为单独的 pooled representation diagnostic，Infinite Mobility 的 valid-package pooled mean 为 2.425 named groups per asset；它只描述该 package 的组织节点表达，既不是质量指标，也不与主结构 macro 作数值比较。

### 3. PV-A 结果如何解读

PV-A 最终 N=150 summary 为：125/150 valid；五类 equal-category valid-only macro 为 4.9498 nodes（95% CI [4.7347, 5.1655]）、深度 2.2000（[2.2000, 2.2000]）、3.8589 movable joints（[3.6486, 4.0733]）。ontology alignment proxy 的 role coverage 为 62.6571%（95% CI [58.2481%, 67.0037%]），scorable coverage 为 72%（[66%, 78%]），Coverage-Weighted Induced Edge F1 为 45.5460%（[41.0164%, 50.0705%]），requested Induced Edge F1 为 66.2222%（[61.3333%, 71.1111%]），induced exact 为 63.3333%（[58.6667%, 68%]），semantic-parent alignment 为 72%（[66%, 78%]）。

主 proxy 的差值由每个方法在类别内独立重采样后直接计算，不做 selection-rank pairing，也绝不是两个单方法 CI 端点相减；pairwise CI 是未多重校正的 exploratory 95% percentile interval。PV-A 相对 Articraft 为 +0.72 pp（95% CI [-5.09, +6.79]），该区间既不建立 superiority，也不建立 equivalence。PV-A 相对 Infinite Mobility 为 +41.10 pp（[+36.57, +45.63]），这一结论仅限于本 PartNet-ontology alignment proxy，不能推广为运动学层级正确性结论。

### 4. LAM 为什么不在主块

LAM 的 balanced N=150 意向 panel 已冻结，但官方 release 仅观察到 93 个可用 release cases，尚缺 57 个需要新生成。因此它的状态是 `BLOCKED_INCOMPLETE_NOT_A_PAPER_RESULT`，不能填入主表，也不能用局部 93 个的结果伪装成平衡 N=150。

已完成的是 full strict official-release audit，总数 `N=300`，类别分布为 237/21/6/14/22，严重不平衡。尽管其类别等权 macro 结果已验证，例如 CW Induced Edge F1 为 47.1% [40.4, 54.2]，它仍只能作为补充 release audit。它不与主块做排名、显著性、差值或“优于/等价”表述。

### 5. Artiverse 的 reference 边界

Artiverse 是 curated pre-release dataset reference，不是生成方法。primary 是四类 `N=120`：storage furniture（预先声明 crosswalk）、refrigerator、dishwasher、microwave；其余三类是 exact raw categories，且没有任何 exact table 结果。`coffee_table` 加入后形成的五类 `N=150` 只是一项 non-exact alias sensitivity，不能合并进 primary，也不能称作与主块完全匹配的 table 类比较。

Artiverse 的 primary CW proxy 为 18.9% [15.8, 21.9]；`coffee_table` sensitivity 为 34.7% [32.1, 37.3]。两者差异应解释为类别 schema 与 alias 选择敏感性，而不是性能提升。Artiverse 的 articulation JSON 仅做 availability audit，不输入 PartNet scorer，也绝不升格为独立 hierarchy gold。

### 6. 指标实现边界

对每个请求资产，`c_i` 是 `mapped predicted URDF links / all final URDF links`，`f_i` 是 common mapped-role support 上的 induced edge F1，主 proxy 是所有 150 个 requested 资产上 `c_i f_i` 的类别等权 macro。不可用或 unscorable asset 贡献零，所以该值同时反映 package 可得性、命名/语义可恢复性与可比较 parent--child edge 的一致性。

但 `c_i` 仍是预测相关的覆盖率：它会惩罚不透明 link names，却不能惩罚预测与未知实例 reference 都缺失的 optional role。重复实例不折叠，例如四条 table leg 仍对应四条 child-link records。以上限制要求论文中将该指标始终表述为 ontology alignment proxy。
