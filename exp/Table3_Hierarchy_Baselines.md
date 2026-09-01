# Table 3: Hierarchy Baseline Experiments

## 【1】第一部分：PV-A Paper 可直接复制粘贴的内容

### Hierarchy Evaluation

We compare PV-A with LAM, Articraft, and Infinite Mobility using a frozen, category-matched hierarchy evaluation. The structural panel contains five shared articulated-object categories (storage furniture/cabinet, table, refrigerator, dishwasher, and oven), with six assets per category and 30 requested assets per method. PV-A and Infinite Mobility use fixed seeds 0--5, whereas LAM and Articraft assets are selected from their official releases using deterministic identity-only SHA-256 ranking. All selections are frozen before compilation and hierarchy evaluation, and failed assets remain in the requested denominator without replacement. Because these systems expose different generation interfaces, this experiment is an official-release matched-category re-evaluation rather than a shared-prompt end-to-end generation benchmark. In particular, Articraft is sampled from its curated public release; the comparison therefore targets final-package quality and semantic recoverability, not raw generation reliability under a unified input protocol.

We evaluate whether each final URDF forms a valid single-rooted, connected, acyclic tree and whether it contains a non-flat hierarchy. Conditional on a valid final asset, we report kinematic tree depth (the longest root-to-link path, counting the root as depth one), named organizational groups, and movable joints (all non-fixed URDF joints, including revolute, continuous, and prismatic joints). We additionally report representation-level diagnostics, including hierarchy size, branching, geometry coverage, structural defects, and within-category canonical topology diversity. Tree depth, group count, movable-joint count, and topology diversity characterize the generated representation and are not metrics for which larger values are always better.

#### Main Structural Results

| Method | Available / Requested | Valid Tree / Available | Valid Tree / Requested | Has Hierarchy / Available |
|---|---:|---:|---:|---:|
| **PV-A (ours)** | 27/30 | 27/27 (100%) | 27/30 (90.0%) | 27/27 (100%) |
| **LAM** | 30/30 | 29/30 (96.7%) | 29/30 (96.7%) | 29/30 (96.7%) |
| **Articraft** | 30/30 | 30/30 (100%) | 30/30 (100%) | 30/30 (100%) |
| **Infinite Mobility** | 30/30 | 30/30 (100%) | 30/30 (100%) | 30/30 (100%) |

| Method | Kinematic Tree Depth | Named Groups | Movable Joints | Canonical Topology Mode / Pairwise Exact |
|---|---:|---:|---:|---:|
| **PV-A (ours)** | 2.222 [2, 3] | 0.000 | 4.407 | 0.433 / 0.200 |
| **LAM** | 2.552 [2, 5] | 0.000 | 3.621 | 0.307 / 0.080 |
| **Articraft** | 2.833 [2, 5] | 0.000 | 6.467 | 0.233 / 0.027 |
| **Infinite Mobility** | 3.500 [2, 5] | 3.167 | 8.467 | 0.433 / 0.227 |

All 27 available PV-A assets form valid hierarchies. Its 90.0% requested-denominator tree validity is caused by three refrigerator samples rejected by strict overlap or motion quality-control checks before a final URDF was produced. LAM contains one selected dishwasher asset with a multi-root structure, while all selected Articraft and Infinite Mobility assets are valid trees. Wilson intervals for requested-denominator validity overlap substantially (27/30: 74.4--96.5%; 29/30: 83.3--99.4%; 30/30: 88.6--100%), so we do not claim statistically significant validity differences. Infinite Mobility produces larger and deeper kinematic representations with more movable joints; its paper explicitly describes a URDF-like tree-growing articulation representation. These values indicate representation choice and complexity rather than semantic-parent correctness. Topology diversity is computed only over valid assets, with the valid counts reported above.

### PartNet-Ontology-Referenced Semantic Hierarchy Alignment

The structural metrics above establish that a hierarchy exists, but they do not determine whether exposed part semantics align with a semantic part--whole ontology. We therefore construct a separate alignment panel using the official PartNet after-merging category hierarchies~\cite{mo2019partnet}. PartNet was designed for fine-grained and hierarchical semantic part segmentation, not as URDF kinematic-tree ground truth. PartNet does not provide an oven hierarchy, so the alignment panel replaces oven with microwave before any output is scored; the other four category selections remain unchanged. The panel again contains five categories, six frozen assets per category, and 30 requested assets per method.

We deterministically map final-package part labels to PartNet roles and collapse unmapped wrapper nodes. For each mapped child instance, the induced reference parent is the nearest PartNet ancestor role present in the same predicted asset. Repeated instances, such as four table legs, remain separate child-link records and are not collapsed into a role-pair set. Let $f_i$ denote induced-edge F1 on the common mapped-role support of asset $i$, and let $c_i$ be mapped predicted URDF links divided by all final URDF links. Our primary metric is Coverage-Weighted Induced Edge F1,

$$
S_m = \frac{1}{30}\sum_{i=1}^{30} c_i f_i,
$$

where unavailable or unscorable assets contribute $c_i f_i=0$. This weighting reduces, but cannot eliminate, the prediction-dependent difficulty of the induced reference: it penalizes opaque or unrecoverable predicted semantics, but it cannot penalize an optional semantic role that is absent from both the prediction and the instance-specific unknown ground truth. We therefore report semantic-role coverage and scorable-asset coverage with every alignment score. Confidence intervals are obtained from 10,000 fixed-seed, category-stratified bootstrap replicates.

#### Ontology-Referenced Alignment Results

| Method | Available / Requested | Role Coverage | Scorable / Requested | Coverage-Weighted Induced Edge F1 (95% CI) | Semantic-Parent Alignment Requested |
|---|---:|---:|---:|---:|---:|
| **PV-A (ours)** | 26/30 | 65.7% | 24/30 (80.0%) | **50.1% [40.2, 59.7]** | 80.0% |
| **LAM** | 30/30 | 63.0% | 24/30 (80.0%) | **51.8% [43.5, 60.5]** | 69.8% |
| **Articraft** | 28/30 | 53.1% | 24/30 (80.0%) | **43.8% [35.6, 51.6]** | 74.4% |
| **Infinite Mobility** | 30/30 | 45.1% | 18/30 (60.0%) | **4.5% [4.1, 5.1]** | 10.0% |

| Method | Induced Edge F1 Requested | Induced Edge F1 Conditional | Induced Exact Requested |
|---|---:|---:|---:|
| **PV-A (ours)** | 71.1% | 88.9% | 66.7% |
| **LAM** | 68.9% | 86.2% | 56.7% |
| **Articraft** | 74.4% | 93.0% | 70.0% |
| **Infinite Mobility** | 13.3% | 22.2% | 0.0% |

PV-A obtains a Coverage-Weighted Induced Edge F1 of 50.1%, compared with 51.8% for LAM and 43.8% for Articraft. The PV-A--LAM difference is -1.7 percentage points (95% bootstrap CI: [-14.7, 11.2]), and the PV-A--Articraft difference is +6.3 points ([-6.2, 19.1]). These intervals are wide and include zero; the panel therefore establishes neither superiority nor equivalence among these three methods. Under the same final-package metadata and role-mapping protocol, PV-A scores 45.6 points above Infinite Mobility ([35.6, 55.2]) on this PartNet alignment proxy. PV-A also has the numerically highest requested-denominator semantic-parent alignment (80.0%), but we do not claim a statistically significant advantage for this metric. Pairwise bootstrap intervals are exploratory and are not adjusted for multiple comparisons.

Infinite Mobility exports opaque URDF link identifiers of the form `l_<index>`. For this method only, we decode these identifiers using raw part names shipped with the final generated package. This mapping is prediction-side metadata, is hash-pinned, and is never treated as reference hierarchy information. A URDF-name-only sensitivity analysis yields zero role coverage for Infinite Mobility and is therefore excluded from the primary table.

### PartNet-Mobility Curated-Dataset Reference Calibration

We additionally evaluate a frozen 30-asset subset of PartNet-Mobility~\cite{xiang2020sapien} as a curated released-dataset reference. It is reported separately because it is a dataset rather than a generation method and because its semantic annotations share provenance with the PartNet-derived ontology. Consequently, it is excluded from generated-method rankings and pairwise difference tests.

| Reference | Available / Requested | Valid Tree / Requested | Kinematic Tree Depth | Named Groups | Movable Joints |
|---|---:|---:|---:|---:|---:|
| PartNet-Mobility | 30/30 | 30/30 (100%) | 3.000 | 1.000 | 2.133 |

| Alignment View | Role Coverage | Scorable / Requested | Coverage-Weighted Induced Edge F1 (95% CI) | Semantic-Parent Alignment Requested |
|---|---:|---:|---:|---:|
| URDF link names only | 25.8% | 0/30 (0.0%) | 0.0% | 0.0% |
| Package `semantics.txt` assisted | 92.1% | 29/30 (96.7%) | 50.7% [45.6, 55.2] | 70.9% |
| Evaluator-imputed category root sensitivity | 92.1% | 29/30 (96.7%) | 68.0% | 96.7% |

The package-assisted row uses only released `semantics.txt` labels. The final sensitivity row additionally assigns the unlabeled URDF `base` link a category-conditioned ontology root; it is not package annotation and is not used as the reference headline. The 17.3-point change in Coverage-Weighted Induced Edge F1 demonstrates that the ontology proxy is sensitive to root-role conventions even on curated data. Re-evaluation from the extracted dataset root reproduces the same frozen IDs, source-file hashes, and metrics as the archived run. The evaluated local v0 archive is hash-pinned and all 30 selected IDs occur in the official gated repository, but the local bytes cannot currently be authenticated against the gated official revision. We therefore retain the status `PROVENANCE_LIMITED`; this calibration is suitable only with that explicit source-provenance disclosure.

### PartNet-Mobility to PhysX-Mobility Paired Representation Audit (Supplementary)

We further conduct a paired representation audit on PhysX-Mobility~\cite{cao2025physxanything}, which was constructed by collecting PartNet-Mobility assets and augmenting them with physical annotations. We freeze six common asset identities in each of the same five categories (30 matched pairs) before scoring and evaluate the original PartNet-Mobility package and its PhysX-Mobility derivative for the same identities. The selection uses exact PartNet-Mobility `meta.json.model_cat` labels and an identity-only SHA-256 rank over the common-ID pool; PhysX annotations and all result fields are excluded from selection, and failures are retained without replacement. This panel isolates changes introduced by released re-annotation and re-serialization. It neither increases the number of underlying shapes nor constitutes an independent dataset comparison and is excluded from generated-method rankings.

| Package Side | Available / Requested | Valid Tree / Requested | Raw Links | Kinematic Tree Depth | Fixed Joints | Movable Joints | Released Collision-Link Coverage (Link Micro) |
|---|---:|---:|---:|---:|---:|---:|---:|
| PartNet-Mobility source | 30/30 | 30/30 (100%) | 3.933 | 3.000 | 1.000 | 1.933 | 74.6% |
| PhysX-Mobility derivative | 30/30 | 30/30 (100%) | 9.133 | 6.667 | 6.400 | 1.733 | 0.0% |

Raw node count and depth are representation diagnostics because PhysX-Mobility introduces fixed wrapper chains. After contracting fixed joints, the PhysX-minus-PartNet mean component-count and depth differences are -0.200 (95% paired-bootstrap CI: [-0.433, -0.033]) and -0.067 ([-0.133, 0.000]), respectively. The raw-link difference is +5.200 ([3.500, 7.133]). These intervals use 10,000 synchronized, category-stratified paired-bootstrap replicates over the 30 matched identities.

| Same-ID Preservation Measure | Result |
|---|---:|
| Mesh-byte retention, mesh micro | 847/860 (98.49%) |
| Mesh-byte retention, equal-asset macro (95% CI) | 97.72% [94.83, 100.00] |
| Movable-joint count preserved | 26/30 (86.7%) |
| Exact joint-type multiset preserved | 24/30 (80.0%) |
| Rotational-class multiset preserved | 26/30 (86.7%) |
| Fixed-contracted graph preserved, exact type | 21/30 (70.0%) |
| Fixed-contracted graph preserved, rotational class | 22/30 (73.3%) |

Exact child-mesh-component matching recovers 50 paired movable joints, covering 86.2% of PartNet-Mobility and 96.2% of PhysX-Mobility movable joints. Among these matched joints, parent-component identity, exact-type preservation, rotational-class preservation, serialized local-axis direction equality, and limit preservation are 86.0%, 94.0%, 100.0%, 100.0%, and 88.0%, respectively. Axis location and Plucker-line preservation are not evaluated because the package-specific link frames have no proven common coordinate frame. The released PhysX URDFs do not reference collision geometry in this cohort; the reported 0.0% is a released-package linkage diagnostic and is not, by itself, evidence that the assets cannot be simulated.

Because both releases share PartNet lineage and our semantic reference is derived from PartNet, their ontology-alignment scores are interpreted only as in-domain package-annotation consistency and evaluator calibration, not as independent instance-level kinematic ground truth. Raw PhysX URDF names yield zero mapped-role coverage. A prediction-side `finaljson.parts[].name` sensitivity recovers 49.9% role coverage but only 1/30 scorable assets and 0.0% Coverage-Weighted Induced Edge F1; it never uses `finaljson` parent structure as reference gold. The paired preservation panel, rather than this name-alignment sensitivity, is the informative PhysX-Mobility result.

### Artiverse Pre-Release Curated-Dataset Reference (Supplementary)

We further evaluate Artiverse~\cite{iliash2026artiverse} as a separate curated released-dataset reference. The experiment uses the official Hugging Face pre-release pinned at revision `8c4b120418e7cbdf9ac4c9580c5dbfdbf128a248`; the evaluated release contains 3,544 model roots. Artiverse is a dataset rather than a generation method, so it is excluded from generated-method rankings, pairwise differences, and superiority claims. An exact five-category panel is infeasible because this release contains no exact `table` category. We therefore define a primary four-category matched-overlap panel with six assets each from refrigerator, dishwasher, microwave, and a fixed storage-furniture crosswalk, for 24 requested assets. Three categories are exact release labels; storage furniture uses the fixed subclasses `armoire`, `chest_of_drawers`, `display_cabinet`, `locker`, `sideboard`, `sink_cabinet`, and `wall_cabinet`. Identity-only SHA-256 selection is frozen before inspecting URDF availability or scores, and failures are retained without replacement.

| Reference Panel | Available / Requested | Parsed / Requested | Valid Tree / Requested | Nodes | Kinematic Tree Depth | Movable Joints | Canonical Mode / Pairwise Exact |
|---|---:|---:|---:|---:|---:|---:|---:|
| Artiverse four-category matched overlap | 24/24 | 24/24 | 24/24 (100%) | 11.083 | 2.750 | 5.042 | 0.333 / 0.083 |

| Alignment View | Role Coverage | Scorable / Requested | Coverage-Weighted Induced Edge F1 (95% CI) | Semantic-Parent Alignment Requested | Induced Exact Requested |
|---|---:|---:|---:|---:|---:|
| Raw URDF link names, four-category primary | 69.2% | 12/24 (50.0%) | 18.6% [12.2, 24.8] | 50.0% | 0.0% |
| Raw URDF link names, `coffee_table` $\rightarrow$ table sensitivity | 75.4% | 18/30 (60.0%) | 32.7% [26.0, 38.7] | 57.8% | 16.7% |

All 24 primary Artiverse assets are available, parseable, single-rooted, connected, and acyclic, with complete visual- and collision-link coverage. However, only 12 assets expose at least one comparable PartNet-induced edge through raw URDF link names. The separate five-category sensitivity adds six `coffee_table` assets as a non-exact table alias; its 14.1-point score increase demonstrates category-schema sensitivity and is not substituted for the primary result. Artiverse articulation JSON is audited for availability but is not supplied to the PartNet scorer and is not treated as independent hierarchy gold. Because the evaluated release is explicitly pre-release and lacks an exact table category, these results are suitable only as a disclosed supplementary curated-dataset reference, not as a fifth method in the main comparison.

### Limitations

The alignment panel measures consistency between prediction-side recoverable roles and a category-level PartNet semantic ontology. It is not an evaluation against instance-matched PartNet geometry, independently annotated kinematic hierarchy gold, or a unique physically correct URDF decomposition. A semantic child such as a handle may be merged into a door rigid body or represented through a fixed joint without changing the object's valid mobility. The PartNet-Mobility calibration is not independent gold because its annotations and the ontology share provenance. PhysX-Mobility is a PartNet-Mobility derivative and therefore supplies paired representation evidence rather than an additional independent shape cohort. Artiverse does not use the PartNet-Mobility annotation files in this evaluation, but its reported alignment uses raw package link names only; its articulation JSON is same-package metadata and is deliberately not treated as gold. Potential source or training-data overlap between Artiverse and the evaluated generators has not been audited. The evaluated Artiverse subset is also a gated pre-release and does not support an exact five-category match. Consequently, none of these measurements should be described as validated kinematic hierarchy correctness. A definitive instance-level evaluation would require method-blind parent--child and nesting annotations by multiple human annotators with independent adjudication.

### Suggested Table Caption

**Table 3: Structural validity and PartNet-ontology-referenced semantic hierarchy alignment on frozen category-matched cohorts.** The structural panel evaluates five categories with six requested assets per category and method. Selection is frozen before compilation, and failures are retained without replacement. Descriptive tree statistics are conditional on valid final assets. The separate alignment panel replaces oven with microwave because the official PartNet hierarchy does not define an oven category. Coverage-Weighted Induced Edge F1 multiplies per-asset induced Edge F1 by prediction-side mapped-link coverage and macro-averages over all 30 requested assets; unavailable or unscorable assets contribute zero. Confidence intervals use 10,000 independent-across-method, category-stratified bootstrap replicates. PartNet-Mobility is reported separately as an in-domain calibration, while the Artiverse pre-release is a supplementary four-category curated-dataset reference with a separately disclosed non-exact table-alias sensitivity. A same-ID PartNet-Mobility-to-PhysX-Mobility representation audit is reported separately in the supplement and uses paired, rather than independent, resampling. These datasets are excluded from generated-method rankings. PartNet-based measurements are semantic-ontology alignment proxies rather than instance-level kinematic hierarchy correctness.

## 【2】第二部分：实验信息与结果的详细说明

本文档单独汇总 PV-A、LAM、Articraft 和 Infinite Mobility 的 Hierarchy 对照实验。所有本地数字均来自冻结 cohort 的实际产物重评，不使用论文表格数字代替本地运行结果。

总体定位：这是小规模、冻结类别队列上的 package-level structural validity 与 PartNet ontology alignment proxy。各方法在不同列呈现 trade-off，不将局部最大值标记为 SOTA。Tree depth、groups、movable joints、fan-out 和 topology diversity 是描述量，没有统一的“越高越好”方向。

### 实验设计

#### Structure panel（Table 3A–3D）

- 类别：storage furniture/cabinet、table、refrigerator、dishwasher、oven。
- 每个方法每类 6 个资产，总计 `N=30/method`。
- PV-A 与 Infinite Mobility 固定 seeds 0–5。
- LAM 在显式类别 allowlist 内按 identity-only SHA-256 rank 取样。
- Articraft 先按官方 paper harness 的 rating 4–5 retained 定义筛选，再按 identity-only SHA-256 rank 取样。
- Selection 在 compile、tree 和 hierarchy 评分前冻结；失败保留，不补抽。
- Tree、kinematic depth、named groups 和 movable joints 使用统一 URDF-equivalent evaluator；扩展结构指标使用同一个共享 evaluator。

#### Semantic hierarchy alignment panel（Table 3E）

- 类别：storage furniture、table、refrigerator、dishwasher、microwave。
- 每个方法每类 6 个资产，总计 `N=30/method`。
- 前四类复用 structure panel 的冻结身份。
- PartNet 官方 hierarchy 没有 oven 类，因此在任何评分前将 oven 替换为 microwave。
- 外部参考为 [PartNet 官方 release meta-files](https://github.com/daerduoCarey/partnet_dataset) 的 after-merging category hierarchy，固定 commit `f321bc9d1533945ad4b22c5c1e7b27a7cccb4edb`。PartNet 的层级部件任务定义见[官方论文](https://cs.stanford.edu/~kaichun/partnet/partnet_main_paper_high_res.pdf)和[项目页](https://partnet.cs.stanford.edu/)。
- 该 panel 是 category-ontology-referenced semantic alignment proxy，不是 instance-matched PartNet geometry、人工 kinematic hierarchy gold 或“唯一正确 URDF tree”。

### Table 3A: Main hierarchy results

| Method | Available / Requested | Valid Tree / Available | Valid Tree / Requested | Has Hierarchy / Available |
|---|---:|---:|---:|---:|
| **PV-A** | 27/30 | 27/27 = 100% | 27/30 = 90.0% | 27/27 = 100% |
| **LAM** | 30/30 | 29/30 = 96.7% | 29/30 = 96.7% | 29/30 = 96.7% |
| **Articraft** | 30/30 | 30/30 = 100% | 30/30 = 100% | 30/30 = 100% |
| **Infinite Mobility** | 30/30 | 30/30 = 100% | 30/30 = 100% | 30/30 = 100% |

| Method | Kinematic Tree Depth | Named Groups | Movable Joints | Cross-Seed Consistency |
|---|---:|---:|---:|---:|
| **PV-A** | 2.222 [2, 3] | 0.000 | 4.407 | raw mode 0.367 / pairwise 0.147 |
| **LAM** | 2.552 [2, 5] | 0.000 | 3.621 | N/A (per-asset release) |
| **Articraft** | 2.833 [2, 5] | 0.000 | 6.467 | N/A (per-asset release) |
| **Infinite Mobility** | 3.500 [2, 5] | 3.167 | 8.467 | canonical mode 0.433 / pairwise 0.227 |

Kinematic Tree Depth、Groups 和 Movable Joints 均只在 valid assets 上聚合。Depth 定义为根 link 计 1 的最长 root-to-link 路径；Movable Joints 包含全部 non-fixed joints，包括 revolute、continuous 和 prismatic。Cross-Seed 只在 valid assets 上计算同类 topology stability，并报告各方法的 valid signature 数；missing 不作为一种独特拓扑。

### Table 3B: Size and branching diagnostics

| Method | Valid / Requested | Nodes / Asset | Edges / Asset | Leaves / Asset |
|---|---:|---:|---:|---:|
| **PV-A** | 27/30 | 5.407 | 4.407 | 4.185 |
| **LAM** | 29/30 | 6.379 | 5.379 | 4.517 |
| **Articraft** | 30/30 | 7.900 | 6.900 | 5.400 |
| **Infinite Mobility** | 30/30 | 15.500 | 14.500 | 11.800 |

| Method | Internal Nodes / Asset | Branch Nodes / Asset | Mean / Max Fan-out | Movable Depth |
|---|---:|---:|---:|---:|
| **PV-A** | 1.222 | 0.963 | 3.722 / 4.037 | 1.222 |
| **LAM** | 1.862 | 1.414 | 2.656 / 3.345 | 1.138 |
| **Articraft** | 2.500 | 1.733 | 3.021 / 4.100 | 1.700 |
| **Infinite Mobility** | 3.700 | 1.200 | 3.483 / 10.667 | 1.133 |

### Table 3C: Edge composition, coverage, and defects

| Method | Movable / Fixed Edges per Asset | Movable Ratio | Visual-Link Coverage | Collision-Link Coverage |
|---|---:|---:|---:|---:|
| **PV-A** | 4.407 / 0.000 | 1.000 | 1.000 | 1.000 |
| **LAM** | 3.621 / 1.759 | 0.821 | 1.000 | 0.828 |
| **Articraft** | 6.467 / 0.433 | 0.968 | 1.000 | 1.000 |
| **Infinite Mobility** | 8.467 / 6.033 | 0.480 | 0.768 | 0.000 |

| Method | Single Root / Connected | Root / Component Defects | Cycle | Malformed / Multi-Parent |
|---|---:|---:|---:|---:|
| **PV-A** | 27/27 / 27/27 | 0 / 0 | 0 | 0 / 0 |
| **LAM** | 29/30 / 29/30 | 1 / 1 | 0 | 0 / 0 |
| **Articraft** | 30/30 / 30/30 | 0 / 0 | 0 | 0 / 0 |
| **Infinite Mobility** | 30/30 / 30/30 | 0 / 0 | 0 | 0 / 0 |

Infinite Mobility 的 `Collision-Link Coverage=0` 表示这些最终 URDF 没有原生 collision geometry，不能据此推断碰撞质量为零或通过。

### Table 3D: Within-category canonical topology diversity

| Method | Valid Signatures | Unique Signature Rate | Mode Rate | Pairwise Exact | Normalized Entropy |
|---|---:|---:|---:|---:|---:|
| **PV-A** | 27/30 | 0.667 | 0.433 | 0.200 | 0.690 |
| **LAM** | 29/30 | 0.833 | 0.307 | 0.080 | 0.861 |
| **Articraft** | 30/30 | 0.933 | 0.233 | 0.027 | 0.948 |
| **Infinite Mobility** | 30/30 | 0.667 | 0.433 | 0.227 | 0.681 |

Canonical signature 去掉语义名称和 sibling order，但保留 rooted shape、joint type 与 visual/group role。Unique Rate/Entropy 描述多样性，Mode/Pairwise Exact 描述一致性；二者都不是单调质量指标。

### Table 3E: PartNet-ontology-referenced semantic hierarchy alignment

| Method | Available / Requested | Role Coverage | Scorable / Requested | Coverage-Weighted Induced Edge F1 (95% CI) | Semantic-Parent Alignment Requested |
|---|---:|---:|---:|---:|---:|
| **PV-A** | 26/30 | 65.7% | 24/30 = 80.0% | 50.1% [40.2, 59.7] | 80.0% |
| **LAM** | 30/30 | 63.0% | 24/30 = 80.0% | 51.8% [43.5, 60.5] | 69.8% |
| **Articraft** | 28/30 | 53.1% | 24/30 = 80.0% | 43.8% [35.6, 51.6] | 74.4% |
| **Infinite Mobility** | 30/30 | 45.1% | 18/30 = 60.0% | 4.5% [4.1, 5.1] | 10.0% |

| Method | Induced Edge F1 Requested | Induced Edge F1 Conditional | Induced Exact Requested |
|---|---:|---:|---:|
| **PV-A** | 71.1% | 88.9% | 66.7% |
| **LAM** | 68.9% | 86.2% | 56.7% |
| **Articraft** | 74.4% | 93.0% | 70.0% |
| **Infinite Mobility** | 13.3% | 22.2% | 0.0% |

正式定义为 $S_m=\frac{1}{30}\sum_i c_i f_i$：$f_i$ 是该资产 common mapped-role support 上的 Induced Edge F1；$c_i$ 是 `mapped predicted URDF links / all final URDF links`；不可用或不可评分资产令 $c_i f_i=0$。重复角色实例按 child-link 分开计数，例如四条 table legs 形成四条 instance records，而不是折叠成一个 `table→leg` role pair。该 coverage 会惩罚不透明命名和语义不可恢复，但无法惩罚“预测和未知实例 GT 中都未出现的 optional role”，所以仍不是完整的实例级语义或运动学正确性。

Infinite Mobility 的最终 URDF 使用 `l_<index>` 匿名 link ID。本实验只使用最终 package 随附的 raw part-name mapping 解码这些 ID；该映射属于 prediction-side metadata，状态固定为 `PREDICTION_ONLY_NOT_GOLD`，没有读取 factory source 或 PartNet label 作为预测输入。纯 URDF-name 敏感性分析得到 0 role coverage，因此不进入主表。

#### Scorable coverage failure breakdown

| Method | No Final URDF | Parse Failure | No Mapped Role | Mapped Roles but No Induced Edge | Scorable |
|---|---:|---:|---:|---:|---:|
| **PV-A** | 4 | 0 | 0 | 2 | 24/30 |
| **LAM** | 0 | 0 | 1 | 5 | 24/30 |
| **Articraft** | 2 | 0 | 0 | 4 | 24/30 |
| **Infinite Mobility** | 0 | 0 | 0 | 12 | 18/30 |

`No Mapped Role` 表示最终资产中没有任何 link 能通过冻结词典恢复到 PartNet role；`Mapped Roles but No Induced Edge` 表示至少存在 mapped role，但这些角色之间没有形成可比较的 PartNet ancestor edge。Ordered rule 在多条 regex 同时命中时选择第一条，同时保留 ambiguity audit：PV-A、LAM、Articraft、Infinite Mobility 分别有 6、12、2、18 个资产出现至少一次多规则命中，对应 6、21、9、46 个 link-level ambiguous matches。这些匹配没有被丢弃，但必须作为词法映射敏感性限制报告。

### Table 3F: PartNet-Mobility curated-dataset reference calibration

PartNet-Mobility 是 SAPIEN 论文发布的 articulated-object 数据集，不是生成方法。这里按同一五类、每类 6 个 dataset IDs 冻结为 30 项，只用于 released-package 结构参考和 evaluator calibration，不进入 PV-A 与生成方法的排名或差值检验。

| Reference | Available / Requested | Valid Tree / Requested | Kinematic Tree Depth | Named Groups | Movable Joints | Canonical Mode / Pairwise Exact |
|---|---:|---:|---:|---:|---:|---:|
| PartNet-Mobility | 30/30 | 30/30 = 100% | 3.000 | 1.000 | 2.133 | 0.533 / 0.293 |

| Alignment View | Role Coverage | Scorable / Requested | Coverage-Weighted Induced Edge F1 (95% CI) | Semantic-Parent Alignment Requested |
|---|---:|---:|---:|---:|
| URDF link names only | 25.8% | 0/30 = 0.0% | 0.0% | 0.0% |
| Package `semantics.txt` assisted | 92.1% | 29/30 = 96.7% | 50.7% [45.6, 55.2] | 70.9% |
| Evaluator-imputed category root sensitivity | 92.1% | 29/30 = 96.7% | 68.0% | 96.7% |

`semantics.txt`-assisted 行只使用发布包标签；root sensitivity 另外给未标注的 URDF `base` 注入类别本体根角色，因此不是 package annotation。两者的主指标相差 17.3 pp，直接说明本 proxy 对根角色约定敏感，不能解释成唯一的真实 hierarchy correctness。由于 PartNet-Mobility annotations 与 PartNet ontology 同源，这也是 in-domain calibration，不是独立 gold。

用户指定的 [HF 镜像](https://huggingface.co/datasets/yuchen0187/partnet-mobility) 已固定并完整下载：revision `bf39e304f19a6c131b5244f128b79ec35000bb02`、6 个 Parquet shards、1,745,807,861 bytes、2,290 rows，所有 shard SHA-256 与 HF LFS OID 一致。但其 schema 只有 `xyz/rgb/mask`，没有 category、object ID、URDF、hierarchy 或 mobility annotation，因此不能用于 Table 3 hierarchy 评分。实际 URDF calibration 使用用户提供的 `/mnt/zsn/lyb/PartNet_Mobility/data/dataset`。该目录的 frozen-30 `meta.json`、`mobility.urdf`、`semantics.txt` 共 90 个核心文件与 workspace-local `partnet-mobility-v0.zip` 逐字节一致；archive SHA-256 为 `b47247a44246111e8d09f2c0e64b4012ae35e0dcf4bb55f68a05b604455119ff`。直接目录复算与旧 Table 3F 的 ordered IDs、URDF hashes 和数值一致。30 个选择 ID 均出现在官方 gated HF repository revision `ee0aa3ef1df16181d76d83f7415aa8c94ed1da8f`，但当前账号无法下载官方 per-ID archives，因此本地 bytes 仍不能与该 revision 直接认证。状态保持 `PROVENANCE_LIMITED`；投稿时必须保留此披露及 PartNet-Mobility/ShapeNet 的上游非商业研究使用条款。

### Table 3G: PartNet-Mobility -> PhysX-Mobility paired representation audit

[PhysX-Mobility](https://huggingface.co/datasets/Caoza/PhysX-Mobility) 是 PhysX-Anything 论文发布的数据集。论文和 dataset card 均明确说明它从 PartNet-Mobility assets 构建并追加物理属性标注，因此它不是独立 shape cohort。这里把它作为同对象的 re-annotation / re-serialization 审计，不能把 PartNet-Mobility 和 PhysX-Mobility 计成两个独立对照组，也不能把 `30 pairs` 写成 `N=60`。

#### 共同身份冻结协议

- 类别来自 PartNet-Mobility `meta.json.model_cat` 的 exact match，不使用 PhysX `finaljson.category` 或 `object_name`。
- Eligible pool 是两侧共同 dataset IDs；五类候选数分别为 storage furniture 339、table 67、refrigerator 22、dishwasher 17、microwave 11。
- 每类在共同池内按下列 UTF-8 identity payload 的 SHA-256 升序取 6 项，总计 30 对；选择阶段不读取 URDF、part annotations 或任何评分结果。
- Selection SHA-256 为 `a0a8eaf00c2970598f3d6191001361dc1e1be1df43ba3e8c394cb6ef988d581b`，两侧 `paired_selection.json` 逐字节一致。失败保留且不补抽；本次两侧均 30/30 available、parsed、valid。

```text
nano3d-table3-physx-partnet-paired-v1
<category>
<dataset_id>
```

#### 原始 package 表示

| Package Side | Available / Requested | Parsed / Requested | Valid / Requested | Raw Links | Kinematic Depth | Fixed Joints | Movable Joints | Released Collision-Link Coverage (Link Micro) |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| PartNet-Mobility source | 30/30 | 30/30 | 30/30 = 100% | 3.933 | 3.000 | 1.000 | 1.933 | 74.6% |
| PhysX-Mobility derivative | 30/30 | 30/30 | 30/30 = 100% | 9.133 | 6.667 | 6.400 | 1.733 | 0.0% |

PhysX raw URDF 中存在 `l_world`、fixed wrapper chains 和 abstract links，所以 raw links、depth、fixed joints 的增加是序列化差异，不是质量提升。PhysX released URDF 在这 30 项中没有 collision tags；虽然 package 其他目录存在 convex meshes，也不能据此推断它们已被 URDF 引用。该 0% 只表示 released-URDF linkage，不等于“无法仿真”。

#### Fixed-joint contraction 后的 paired preservation

Fixed contraction 将所有由 fixed joints 连通的 links 合并，并保留全部 non-fixed edges。差值定义为 PhysX 减 PartNet；置信区间使用 `10,000` 次 synchronized category-stratified paired bootstrap，每个 replicate 在每类内同步重采样 6 个 same-ID pairs，统计单位始终是 30 对。

| Paired Quantity | Estimate | 95% Paired-Bootstrap CI |
|---|---:|---:|
| Raw-link count delta | +5.200 | [3.500, 7.133] |
| Contracted-component count delta | -0.200 | [-0.433, -0.033] |
| Contracted-depth delta | -0.067 | [-0.133, 0.000] |
| Movable-joint count delta | -0.200 | [-0.433, -0.033] |
| Mesh-byte retention, equal-asset macro | 97.72% | [94.83%, 100.00%] |

| Preservation Check | Result |
|---|---:|
| URDF-referenced unique visual meshes, byte-exact micro | 847/860 = 98.49% |
| All referenced meshes byte-exact within asset | 27/30 = 90.0% |
| Movable-joint count preserved | 26/30 = 86.7% |
| Exact joint-type multiset preserved | 24/30 = 80.0% |
| Rotational-class multiset preserved | 26/30 = 86.7% |
| Contracted full graph preserved, exact joint type | 21/30 = 70.0% |
| Contracted full graph preserved, revolute/continuous equivalence | 22/30 = 73.3% |

Mesh micro `847/860` 与 equal-asset macro `97.72%` 的分母不同，不能把 macro CI 直接附在 micro 点估计后。Joint matcher 只使用同一 asset 内唯一、精确的 child-component visual-mesh basename set；joint type、axis 和 limits 都不参与匹配，因此不会按待评指标挑配对。它找到 50 个 matched movable joints，占 PartNet 侧 `50/58 = 86.2%`、PhysX 侧 `50/52 = 96.2%`。Matched joints 中 parent-component identity、exact type、rotational class、serialized local-axis direction equality、limits 分别为 `43/50 = 86%`、`47/50 = 94%`、`50/50 = 100%`、`50/50 = 100%`、`44/50 = 88%`。这里的 axis equality 只比较各自 link frame 中序列化的方向；因为没有证明两套 package 的 frame 已配准，不评估 physical/world axis location 或 Plucker line。

#### 为什么 ontology alignment 不是 PhysX 的主结果

PhysX raw URDF 使用匿名 link names，当前冻结 scorer 得到 role coverage 0%、scorable 0/30。用 `finaljson.parts[].name` 通过 mesh basename 恢复 prediction-side labels 后，role coverage 为 49.9%，但仍只有 1/30 可评分，Coverage-Weighted Induced Edge F1 为 0%。这反映当前 PartNet role mapper 对 PhysX package 命名/层级序列化的不可恢复性，不能解释成 PhysX 的 kinematic hierarchy 错误。`finaljson` parent structure 从未作为 reference gold；本节有信息量的结果是同 ID mobility preservation，而不是 ontology score。

#### 数据身份、许可和论文位置

PhysX 官方 HF revision 为 `d0768ee9e1415f6be8db78d6389ba018b85134c0`，本地 archive SHA-256 `88308cc2a4cc6177c59e32c2de51e881e6b961737295e5082d7ed01cca221908` 与官方 LFS object 一致；dataset card 声明 `CC BY-NC 4.0`。但 PhysX 是 PartNet-Mobility derivative，不能将该许可理解为覆盖或取代 PartNet-Mobility/ShapeNet 上游条款。该 panel 已通过最终只读审查，状态为 `supplementary_reference_only`：主文最多一句引用，完整表格放 supplement，不加入 PV-A/LAM/Articraft/Infinite Mobility 排名或 superiority test。

### Table 3H: Artiverse pre-release curated-dataset reference

[Artiverse](https://3dlg-hcvc.github.io/artiverse/) 是一个带功能部件、运动关系和物理属性的 articulated-object 数据集，不是生成方法。官方[论文](https://arxiv.org/abs/2605.24403)描述的是约 5.4K objects / 88 categories 的完整集合；本实验实际使用的则是官方 [Hugging Face pre-release](https://huggingface.co/datasets/3dlg-hcvc/artiverse) 中固定 revision `8c4b120418e7cbdf9ac4c9580c5dbfdbf128a248` 的 3,544-model / 84-category subset，不能把完整论文规模写成本次实验规模。该 release 为 gated、`license=other`，资产许可服从各上游来源，且 dataset card 明确说明它仍在持续清理和验证。因此本节只能作为带披露的 supplementary curated-dataset reference，不能作为第五个生成方法参加排名。

#### 冻结类别与选择协议

严格五类 exact match 不可行，因为 pinned manifest 中 exact raw category `table=0`。主结果采用 `primary_4class_matched_overlap`，每类 6 个、总计 `N=24`：

| Target Category | Eligible Rule | Eligible Count | Frozen Count |
|---|---|---:|---:|
| storage furniture | `{armoire, chest_of_drawers, display_cabinet, locker, sideboard, sink_cabinet, wall_cabinet}` | 991 | 6 |
| refrigerator | exact `refrigerator` | 149 | 6 |
| dishwasher | exact `dishwasher` | 33 | 6 |
| microwave | exact `microwave` | 50 | 6 |

Storage furniture 是预先固定的窄类别 crosswalk，不是 exact label。主 cohort 实际抽到 `chest_of_drawers=2`、`sink_cabinet=2`、`wall_cabinet=2`。身份定义为 `target_category/raw_category/source/model_id`，在每个 target category 内按下列 UTF-8 payload 的 SHA-256 升序取前 6 项：

```text
nano3d-table3-artiverse-reference-v1
<target_category>
<raw_category>
<source>
<model_id>
```

Selection SHA-256 为 `03820ae54e58b0faf7c85c8a69342779fe11cd86e6abfdeae5f58dd3f7d3e944`。选择只依赖 manifest identity，不检查 URDF 是否存在、能否解析、层级是否合法或 role 是否可映射；所有失败保留且不补抽。3,544/3,544 个 manifest direct roots 均在冻结前完成路径存在性核验。

#### 主结果：四类 matched overlap

| Reference | Available / Requested | Parsed / Requested | Valid / Available | Valid / Requested | Nodes | Kinematic Tree Depth | Movable Joints |
|---|---:|---:|---:|---:|---:|---:|---:|
| Artiverse primary | 24/24 | 24/24 | 24/24 = 100% | 24/24 = 100% | 11.083 | 2.750 | 5.042 |

Nodes、Depth 和 Movable Joints 只在 valid assets 上聚合；这里 24 项全部 valid。Movable Joints 定义为全部 non-fixed joints。总计 `continuous=8`、`prismatic=68`、`revolute=45`、`fixed=121`，未支持或其他 joint type 为 0，因此共享 evaluator 的 movable 计数与 all-non-fixed 口径一致。Visual-link 和 collision-link coverage 均为 100%；root、component、cycle、malformed endpoint 和 multi-parent defects 均为 0。四类等权 canonical topology 统计为 `unique=0.833`、`mode=0.333`、`pairwise exact=0.083`、`entropy=0.859`。

| Alignment Input | Role Coverage | Scorable / Requested | Coverage-Weighted Induced Edge F1 (95% CI) | Semantic-Parent Alignment Requested | Induced Exact Requested |
|---|---:|---:|---:|---:|---:|
| Raw URDF link names only | 69.2% | 12/24 = 50.0% | 18.6% [12.2, 24.8] | 50.0% | 0/24 = 0.0% |

PartNet scorer 只读取 raw URDF link names。Artiverse 的 `articulations.json` 在 30 个 union assets 上都可解析，但它与 URDF 属于同一 release package，不是独立人工 gold，因此没有喂给 scorer。12 个 unscorable assets 均为“存在 mapped roles，但没有形成可比较的 induced edge”，不是缺文件或 parse failure。这个结果表示 package-name semantic recoverability 与 PartNet ontology alignment sensitivity，不表示 Artiverse 的 kinematic hierarchy correctness 只有 18.6%。

#### 非精确 table alias 敏感性

额外 sensitivity 将 `coffee_table -> table`，在 35 个候选中按同一 identity-only 规则冻结 6 项，使五类总计 `N=30`。没有使用 `desk`，也没有根据结果调 alias。

| Sensitivity Panel | Available / Requested | Valid / Requested | Role Coverage | Scorable / Requested | Coverage-Weighted Induced Edge F1 (95% CI) |
|---|---:|---:|---:|---:|---:|
| Add `coffee_table -> table` | 30/30 | 30/30 = 100% | 75.4% | 18/30 = 60.0% | 32.7% [26.0, 38.7] |

相对四类主结果，主指标增加 14.1 pp。这一变化来自 panel schema 和 category alias 的改变，所以该行只能作为 sensitivity，不能替换主结果，也不能称为 exact five-category calibration。

#### 同四类生成方法的补充上下文

为检查四类 schema 对原结论的影响，我们从四个方法既有的冻结 alignment cohorts 中只取相同四类，每法仍为 `6/category, N=24`，重新核对 URDF SHA-256 并用共享 structure evaluator 重算。该分析在看到 Artiverse schema 后定义，属于 post-hoc、schema-constrained supplementary sensitivity；四种方法各自在类别内独立抽样，不能按 index 配对。Artiverse 不加入此排名，也不计算它与生成方法的差值。

| Method | Available / Requested | Valid / Requested | Role Coverage Requested | Scorable / Requested | Coverage-Weighted Induced Edge F1 (95% CI) |
|---|---:|---:|---:|---:|---:|
| PV-A | 20/24 | 20/24 = 83.3% | 57.1% | 20/24 = 83.3% | 57.1% [45.1, 68.8] |
| LAM | 24/24 | 23/24 = 95.8% | 72.2% | 24/24 = 100% | 64.7% [54.4, 75.4] |
| Articraft | 22/24 | 22/24 = 91.7% | 51.7% | 18/24 = 75.0% | 44.8% [34.8, 54.3] |
| Infinite Mobility | 24/24 | 24/24 = 100% | 38.3% | 18/24 = 75.0% | 5.7% [5.1, 6.4] |

这张上下文表不能替换原五类 Table 3E：移除 table 后，PV-A 与 LAM 的数值关系发生变化，说明 headline 结论依赖冻结 category panel。这里没有计算 pairwise superiority，也不能据此宣称 PV-A SOTA。

### 统计结论

主比较采用 Coverage-Weighted Induced Edge F1：

| Comparison | Difference | 95% Bootstrap CI | Interpretation |
|---|---:|---:|---|
| PV-A minus LAM | -1.7 pp | [-14.7, 11.2] | 区间跨 0，不支持显著差异结论 |
| PV-A minus Articraft | +6.3 pp | [-6.2, 19.1] | 区间跨 0，不支持显著差异结论 |
| PV-A minus Infinite Mobility | +45.6 pp | [35.6, 55.2] | 本 cohort 与 evaluator 下差异明确 |

可用于论文的保守表述是：这组数据没有提供 PV-A 与 LAM 或 Articraft 存在差异的证据，但宽置信区间仍容纳有实际意义的正负差异，因此既不能证明 superiority，也不能证明 equivalence。PV-A 在相同 final-package metadata 与 role-mapping 协议下，比 Infinite Mobility 的 PartNet alignment proxy 数值高 45.6 pp；这不是“运动学层级更正确”的结论。

### 如何理解这些结果

#### 为什么 Table 3A 是 27/30，而 Table 3E 是 26/30

两张表不是完全相同的第五类。Table 3A–3D 使用 oven，PV-A 的三个 refrigerator seeds 因 strict overlap/motion QC 失败，因此最终为 27/30。Table 3E 为了匹配 PartNet 官方 ontology，将 oven 预先替换为 microwave；除了同样三个 refrigerator failures，microwave seed 0 又因 drawer 在运动姿态中与 grounded body 断开而失败，所以是 26/30。失败发生后没有换 seed，也没有只在成功资产上重新定义请求分母。

#### 为什么 PV-A 不是每一列最高

首先，并非每列都存在“越高越好”的 SOTA。Tree Depth、Groups、Movable Joints、Nodes、Fan-out、Unique Signature Rate 和 Entropy 都是结构描述量。例如 Infinite Mobility 的节点和 movable joints 更多，只能说明其 URDF-like tree-growing 表示更复杂，不能说明父子语义更正确；同样，更高或更低的 topology diversity 都可能是类别设计允许的结果。

其次，PV-A 的端到端结构成功率确实受到 strict QC failure 影响。Structure panel 的三次 refrigerator failure 和 alignment panel 新增的一次 microwave failure 均按 0 保留在 requested denominator。`27/27 final artifacts valid` 表明已生成的 PV-A 最终资产全部是合法树，但不能把这个 conditional 结果替换成端到端的 27/30。

最后，PV-A 仍有真实的语义 hierarchy 短板。Table 类别的 link names 基本可映射，但部分父子边与 PartNet induced hierarchy 不一致，使该类 conditional Edge F1 明显下降；dishwasher 的已映射边较准确，但 rack、wash-arm 等生成角色不完全落入 PartNet 的有限角色集合，因此 mapped-link coverage 偏低。Coverage-Weighted Induced Edge F1 同时惩罚这些问题，不能通过只保留少量易映射角色获得高分。

#### Requested、conditional 和 coverage-weighted induced 的区别

- `Requested`：以预先冻结的全部 30 个请求资产为分母。不可用或不可评分资产记 0，最接近端到端结果。
- `Conditional`：只在存在至少一条 ontology-induced reference edge 的资产上统计。它回答“能评分的子集有多准”，但可能因覆盖率低而偏乐观。
- `Semantic Role Coverage`：最终 URDF/交付包中有多少 links 能映射到 PartNet 角色。
- `Scorable Coverage`：30 个请求资产中，有多少至少形成一条可比较的 ontology-induced edge。
- `Coverage-Weighted Induced Edge F1`：逐资产 `Induced Edge F1 × Role Coverage` 后再对全部 30 个请求资产取平均，是本实验同时约束 induced alignment 和 prediction-side semantic recoverability 的主指标。

因此，Articraft 的 conditional Induced Edge F1 为 93.0%，但 role coverage 只有 53.1%，其 coverage-weighted 结果为 43.8%。PV-A 的 conditional Induced Edge F1 为 88.9%，role coverage 为 65.7%，coverage-weighted 后为 50.1%。LAM 的主指标为 51.8%，比 PV-A 高 1.7 pp，但差值置信区间较宽且跨 0。

#### 四种方法的输入与公平性边界

- PV-A：对冻结模板和 seeds 做 fresh strict compile，并执行当前完整 QC。
- LAM：从官方 release 的冻结类别候选中确定性取样，不重新请求模型生成。
- Articraft：从官方 curated Articraft-10K release 的 rating 4–5 retained records 中冻结取样，并使用 pinned paper harness 编译；它代表公开最终包质量，不代表未经筛选的 raw generation reliability。
- Infinite Mobility：复用与官方 pinned source tree 一致的真实 seed packages；alignment panel 没有重新启动 Blender。

因此本实验公平地统一了类别、样本数、最终 URDF evaluator、失败分母和 selection 时点，但没有统一成“同一条自然语言 prompt、同一生成预算”的端到端任务。比较目标是 officially released final packages 的结构质量和语义可恢复性，而非统一输入协议下的 raw generation reliability。论文中应称为 `official-release matched-category re-evaluation`，不能称为 shared-prompt generation ranking。

#### Bootstrap 实现核对

- 五个类别始终等权，每类在每个 replicate 内有放回抽取 6 个资产，因此每方法每次仍为 30 个 observations。
- 四个方法分别在各自类别内独立重采样；LAM、Articraft、PV-A 和 Infinite Mobility 的第 $k$ 个样本没有被当作同一实例，也没有做 paired bootstrap。
- 固定 bootstrap seed 为 `20260811`，replicates 为 `10,000`，区间为 percentile 95% CI。
- 三个 PV-A-minus-baseline 差值区间是 exploratory、未做多重比较校正的区间，不能作为 confirmatory family-wise significance test。
- 当前没有预注册 equivalence margin，也没有执行 TOST 等效性检验，因此区间跨 0 只能写成“未检测到差异”，不能写成“方法等效”或“perform comparably”。

#### 目前能够支持和不能支持的论文结论

可以支持：PV-A 生成成功后的 hierarchy 全部满足 single-root/connected/acyclic tree；当前 panel 没有检测到 PV-A 与 LAM/Articraft 主指标差异；PV-A 在本次 PartNet alignment proxy 上数值高于 Infinite Mobility；PV-A 的 final URDF 同时具有完整 visual/collision link representation。

不能支持：PV-A 在所有 hierarchy 指标上全局 SOTA；PV-A 与 LAM/Articraft 等效；PV-A 显著超过 LAM 或 Articraft；PartNet ontology proxy 等价于实例级 kinematic gold；更多 depth/groups/movable joints 自动代表更高质量。

### Claim Boundary

Table 3A–3D 是最终 URDF 的 package-level structural validity、表示和拓扑统计。Table 3E 检查最终交付包中可恢复的预测角色是否符合 PartNet 类别语义本体诱导的父子关系。Table 3F 是 PartNet-Mobility in-domain calibration；Table 3G 是同 ID 的 PartNet-Mobility -> PhysX-Mobility derivative representation audit；Table 3H 是 Artiverse pre-release curated-dataset reference。三者都不是新增生成方法。PartNet 是 semantic part--whole hierarchy，不是 URDF kinematic tree ground truth；这些实验都不等价于人工标注的实例级 kinematic hierarchy correctness。

PhysX Table 3G 能支持的结论是：相同 30 个底层资产从 PartNet-Mobility package 转成 PhysX-Mobility package 后，大部分 mesh bytes、movable-joint counts/types 和 fixed-contracted mobility graph 得到保留，但并非全部保留。它不能支持“新增独立数据集验证”、PhysX 相对生成方法的排名或 raw depth 更高即 hierarchy 更好。PartNet 本地源仍为 `PROVENANCE_LIMITED`；PhysX 的 CC BY-NC 4.0 也不取代 PartNet-Mobility/ShapeNet 上游条款。

Artiverse Table 3H 能支持的结论是：固定四类 cohort 的 24 个 release packages 全部可用且形成合法树；raw-name PartNet alignment 对类别 alias 很敏感。它不能支持 Artiverse 相对四个生成方法更好或更差，也不能证明其 package articulation annotations 是独立 ground truth。Artiverse 与各生成器训练数据或源资产是否重叠尚未审计。当前 Artiverse artifact 状态保持 `paper_ready=false`；原因是 release 明示为 pre-release 且 exact table 类缺失。带这些披露时可放 supplementary，不能放入主方法排名。

每个 mapped child 的参考 parent 定义为该资产已映射角色集合中的最近 PartNet ancestor。因此 Edge F1、Exact 和 Nesting 必须与 Semantic Role Coverage、Scorable Coverage 共同解释；低覆盖情况下的 conditional 高分不能单独作为 headline。

如果后续需要把层级正确性提升为实例级主结论，需要对冻结的 120 个资产进行方法匿名化，由两名标注者独立标注 parent-child/nesting，分歧交给第三名裁决。当前自动实验不能替代这一步。

### Reproducibility

主要协议和 scorer：

- [PartNet semantic-alignment protocol](/mnt/zsn/lyb/arti-skill/exp/reference/partnet_hierarchy_correctness_v1.json)，SHA-256 `a3d3e77df3d90bd3e6b26f89b13f0a2e63998941c7a67ea3d5cc61e0f893ca7c`。
- [Shared alignment scorer](/mnt/zsn/lyb/arti-skill/exp/scripts/partnet_hierarchy_correctness.py)。
- [Shared structure evaluator](/mnt/zsn/lyb/arti-skill/exp/scripts/hierarchy_extended_metrics.py)，SHA-256 `812c481a3d738b8d17893559716c275b6152f22942f37f06df2231d65f36fcd4`。
- [Combined alignment report](/mnt/zsn/lyb/arti-skill/exp/runtime/nano3d_hierarchy_correctness/combined/report.md)。
- [Combined alignment summary](/mnt/zsn/lyb/arti-skill/exp/runtime/nano3d_hierarchy_correctness/combined/summary.json)。
- [Alignment self-check](/mnt/zsn/lyb/arti-skill/exp/runtime/nano3d_hierarchy_correctness/combined/self_check.json)。
- [Structure-panel combined summary](/mnt/zsn/lyb/arti-skill/exp/runtime/nano3d_hierarchy_paper/summary.json)。
- [PartNet-Mobility direct-root reference summary](/mnt/zsn/lyb/arti-skill/exp/runtime/nano3d_hierarchy_correctness/partnet_mobility_official_reference/summary.json)，SHA-256 `aa8a17f518529af96d19049bb939a6e78e25b12784b930f6cd1bbe7d3dff0b4f`。
- [PartNet-Mobility direct-root frozen selection](/mnt/zsn/lyb/arti-skill/exp/runtime/nano3d_hierarchy_correctness/partnet_mobility_official_reference/frozen_selection.json)，SHA-256 `7c7a616f314d5ccc074a9bf412700dc475d255c7c0b27e52fbeee9f5c54c7dd6`。
- [PartNet-Mobility direct-root provenance](/mnt/zsn/lyb/arti-skill/exp/runtime/nano3d_hierarchy_correctness/partnet_mobility_official_reference/provenance.json)，状态 `PROVENANCE_LIMITED`。
- [PartNet-Mobility direct-root runner](/mnt/zsn/lyb/arti-skill/exp/scripts/run_partnet_mobility_official_reference.py) 与 [deterministic replay audit](/mnt/zsn/lyb/arti-skill/exp/runtime/nano3d_hierarchy_correctness/partnet_mobility_official_reference/determinism_verification.json)。
- [Downloaded HF mirror audit](/mnt/zsn/lyb/arti-skill/exp/baselines/partnet-mobility-hf/download_report.md)。
- [PhysX/PartNet paired protocol](/mnt/zsn/lyb/arti-skill/exp/reference/physx_partnet_paired_hierarchy_protocol_v1.json)，SHA-256 `2250b613242991afc1faf00d434f874ae1439560f494dff47dfeed8d9bcff48a`。
- [Paired PartNet-Mobility summary](/mnt/zsn/lyb/arti-skill/exp/runtime/nano3d_hierarchy_correctness/physx_partnet_paired_partnet_reference_v2/summary.json)，SHA-256 `e28081be69d5e50db4d53645873dee576690c7348a87e46fda470ccacf0d69a3`。
- [PhysX-Mobility paired summary](/mnt/zsn/lyb/arti-skill/exp/runtime/nano3d_hierarchy_correctness/physx_mobility_reference/summary.json)，SHA-256 `f80c8824095bd779426083f62e5b319798f8bf4c88f86c99ce254d28c1ec4849`。
- [PhysX-Mobility paired report](/mnt/zsn/lyb/arti-skill/exp/runtime/nano3d_hierarchy_correctness/physx_mobility_reference/report.md) 与 [30-pair preservation records](/mnt/zsn/lyb/arti-skill/exp/runtime/nano3d_hierarchy_correctness/physx_mobility_reference/paired_preservation_records.jsonl)。
- [PhysX-Mobility runner](/mnt/zsn/lyb/arti-skill/exp/scripts/run_physx_mobility_paired_reference.py)、[paired preservation evaluator](/mnt/zsn/lyb/arti-skill/exp/scripts/physx_partnet_paired_preservation.py) 与 [deterministic replay audit](/mnt/zsn/lyb/arti-skill/exp/runtime/nano3d_hierarchy_correctness/physx_mobility_reference/determinism_verification.json)。
- [Artiverse frozen protocol](/mnt/zsn/lyb/arti-skill/exp/reference/artiverse_hierarchy_reference_v1.json)。
- [Artiverse reference summary](/mnt/zsn/lyb/arti-skill/exp/runtime/nano3d_hierarchy_correctness/artiverse_reference/summary.json)，SHA-256 `c35962294a038719c209104a277deea5bce79cc3ae6170cc3cd18b20c04eb6d8`。
- [Artiverse frozen selection](/mnt/zsn/lyb/arti-skill/exp/runtime/nano3d_hierarchy_correctness/artiverse_reference/frozen_selection.json)，SHA-256 `03820ae54e58b0faf7c85c8a69342779fe11cd86e6abfdeae5f58dd3f7d3e944`。
- [Artiverse reference report](/mnt/zsn/lyb/arti-skill/exp/runtime/nano3d_hierarchy_correctness/artiverse_reference/report.md)。
- [Artiverse deterministic replay audit](/mnt/zsn/lyb/arti-skill/exp/runtime/nano3d_hierarchy_correctness/artiverse_reference/determinism_verification.json)。
- [Four-category generated-method context](/mnt/zsn/lyb/arti-skill/exp/runtime/nano3d_hierarchy_correctness/artiverse_reference/four_category_generated_method_context.json)。
- [Artiverse runner](/mnt/zsn/lyb/arti-skill/exp/scripts/run_artiverse_real_data_reference.py) 与 [replay verifier](/mnt/zsn/lyb/arti-skill/exp/scripts/verify_artiverse_real_data_reference.py)。

方法与数据 provenance：

| Source | Frozen revision / selection evidence |
|---|---|
| [PartNet meta-files](https://github.com/daerduoCarey/partnet_dataset) | commit `f321bc9d1533945ad4b22c5c1e7b27a7cccb4edb`; five source-file hashes listed below and embedded in the protocol |
| [LAM](https://openaccess.thecvf.com/content/CVPR2026/html/Gao_LAM_Language_Articulated_Object_Modelers_CVPR_2026_paper.html) | code commit `0b3a87beb8c35273a5acf8681221791aff746d8e`; explicit category allowlists; identity-only SHA rank |
| [Articraft](https://arxiv.org/html/2605.15187v1) | code `2179fe65271b3e9d771c92c41f7ff36c71ac4e9c`; Articraft-10K `677ca9722427dce500873730255874c8c3f07eb2`; paper harness `959f1455091fc8f86a489ee8cab38f686916099f`; rating 4–5 retained filter then identity-only SHA rank |
| [Infinite Mobility](https://arxiv.org/html/2503.13424v1) | code commit `5f5961736fcf5b7a6e6fd0a9b3b7af586f39e151`; fixed seeds 0–5; runtime Python source tree matched the official checkout |
| [PartNet-Mobility](https://huggingface.co/datasets/sapien-sim/PartNetMobility) | official gated revision `ee0aa3ef1df16181d76d83f7415aa8c94ed1da8f`; local archive SHA-256 `b47247a44246111e8d09f2c0e64b4012ae35e0dcf4bb55f68a05b604455119ff`; selected local bytes remain `PROVENANCE_LIMITED`; upstream non-commercial research/education and ShapeNet terms apply |
| [PhysX-Mobility](https://huggingface.co/datasets/Caoza/PhysX-Mobility) | official revision `d0768ee9e1415f6be8db78d6389ba018b85134c0`; archive SHA-256 `88308cc2a4cc6177c59e32c2de51e881e6b961737295e5082d7ed01cca221908`; CC BY-NC 4.0 plus upstream PartNet-Mobility/ShapeNet terms; paired selection SHA-256 `a0a8eaf00c2970598f3d6191001361dc1e1be1df43ba3e8c394cb6ef988d581b` |
| [Artiverse](https://huggingface.co/datasets/3dlg-hcvc/artiverse) | pre-release revision `8c4b120418e7cbdf9ac4c9580c5dbfdbf128a248`; 3,544 manifest roots verified; manifest SHA-256 `8fa6468254a1f74c58f0c25699598bf88f622fabdaf74f0cd9268ee5663c5586`; identity-only frozen selection |
| PV-A | fixed local templates and seeds 0–5; strict compile/QC; no failure replacement |

PartNet after-merging hierarchy source hashes：

| Category | SHA-256 |
|---|---|
| dishwasher | `5d915c7a8aa9e9897be1c66edc95b332688fda49fae93fe71215711d7f1d3176` |
| microwave | `fc1b3ce5688b7df25620295663a5f2ae9177759976a54834589ce31598afef4c` |
| refrigerator | `e2d2ac95fddaf107eeb388e1bc79b4d5dd8d6ec317c03d5308edb9a7f93ea648` |
| storage furniture | `0c4de933b297eb1ef92a4502a6059ced3742e2b4695a60f5ebd00033a26c6fac` |
| table | `643fa858c07f85b8bdc3b4e2c01a5e9312ba0cb356fbc31effa316c271fa67ab` |

完整 alignment manifest：

| Method | Manifest | SHA-256 |
|---|---|---|
| PV-A | [`evaluation_manifest.jsonl`](/mnt/zsn/lyb/arti-skill/exp/runtime/nano3d_hierarchy_correctness/pva/evaluation_manifest.jsonl) | `70df0cc4d50ebe353a819452e5a5b0620df329a6aac0381fcdb29920f99a06fb` |
| LAM | [`evaluation_manifest.jsonl`](/mnt/zsn/lyb/arti-skill/exp/runtime/nano3d_hierarchy_correctness/lam/evaluation_manifest.jsonl) | `6b1c2cf4fd7aff2fd6cbddb755935dd97c5764ad344a843cfe7f5568d60081e7` |
| Articraft | [`evaluation_manifest.jsonl`](/mnt/zsn/lyb/arti-skill/exp/runtime/nano3d_hierarchy_correctness/articraft/evaluation_manifest.jsonl) | `dcf8dbb9e190358e68cd498cb8cc93c6bb0fc903745eb51e830ee51e4cf65de0` |
| Infinite Mobility | [`evaluation_manifest.jsonl`](/mnt/zsn/lyb/arti-skill/exp/runtime/nano3d_hierarchy_paper/infinite_mobility/correctness_panel/evaluation_manifest.jsonl) | `61129788be792c77771e7f5c4b1a258a6290a3cf7596b6c6f950cbbcaccd5d5e` |

Infinite Mobility 的 [`raw-name decoding table`](/mnt/zsn/lyb/arti-skill/exp/runtime/nano3d_hierarchy_paper/infinite_mobility/correctness_panel/role_assignments.json) SHA-256 为 `a37626528e09e96e91ccb0b3a2127bb8a9bdea78d6ebb428ea9901d0c4f3c305`。`storage_furniture_cabinet → storage_furniture` category alias、完整 ordered role-mapping dictionary 和五个 PartNet source-file hashes 均固定在 protocol JSON 中。

验证命令：

```bash
cd /mnt/zsn/lyb/arti-skill
python3 exp/scripts/verify_nano3d_hierarchy_paper.py
python3 exp/scripts/verify_partnet_hierarchy_correctness.py
python3 exp/scripts/verify_partnet_mobility_official_reference.py
python3 exp/scripts/verify_partnet_mobility_paired_reference.py
python3 exp/scripts/verify_physx_mobility_paired_reference.py
python3 exp/scripts/verify_artiverse_real_data_reference.py
python3 exp/scripts/summarize_artiverse_four_category_context.py
```

所有正式 verifier 均已通过。PartNet direct-root replay 的 14 个主 artifacts 和 90 个 frozen source snapshots 逐字节一致；paired PartNet replay 的 10 个主 artifacts 一致；PhysX replay 的 103 个文件（含 90 个 selected snapshots）逐字节一致。Artiverse 隔离 replay 的所有可比 artifacts 与 60 个选中 metadata files 逐字节一致；既有 alignment scorer 与 bootstrap 汇总重复运行后，六个核心输出文件 SHA-256 逐字节一致。
