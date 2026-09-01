# PV-A Diversity 验证 V2

> 本文是 `div.md`（V1）的 V2。保留 V1 的四个核心问题与「2 表 + 1 图 + gallery」骨架，但按三条标准重新挑选指标：
> (a) 能回答四个核心问题之一；(b) 分母可以冻结、fail-closed；(c) 仓库已有可复用实现，或新增成本低。
> 所有「结论先于实验」、分母不闭合、或定义上单调的指标，要么移除，要么降级。
> V2 与 V1 的差异逐条见第 0 节；统计契约见第 1 节，先于任何指标生效。

# 0. 指标挑选总览（V1 → V2 处置）

| V1 指标 | V2 处置 | 理由 |
| --- | --- | --- |
| Categories/Generators、Assets（原生规模） | ✅ 保留 → Table 1A | 规模与 provenance，不做结构排名 |
| Avg. Joints | ✅ 保留为描述量（无箭头）→ 1B | 已有实现（Links / Movable Joints，mean/median/P90） |
| Var. Joints ↑ | ⬇️ 降为描述量、去箭头、移附录 | 高方差可能来自错误、dummy joints 或类别映射过粗，不是单调质量 |
| Avg. nTED ↑ | ⏸️ 推迟到附录（条件性） | 无实现；必须先冻结 edit cost、unordered sibling、语义替换代价、dummy 收缩；主文用已有 canonical topology 统计替代 |
| Effective Graph No. ↑ | ✏️ 改为 `N_eff / N_valid` + coverage + bootstrap CI → 1B | `exp(H)` 受 valid 样本数限制，失败数不同的方法不可直接比 |
| Near-Duplicate Rate ↓ | ✏️ 拆成 excess 三层定义 + 阈值校准 → 1B | 原定义漏掉 dummy 差异几何克隆、功能铰接差异、局部功能件差异；分母语义未定 |
| 「只评估 valid，补满每类 20 个」 | ❌ 改为 requested 分母 | 补样会隐藏生成失败；仓库契约是失败保留在分母 |
| FPC₂ ↑ | ✏️ 改名 Sampled-Support Pairwise Coverage（除非能精确枚举）→ Table 2 | 100K 采样 support 不闭合；大 support 变量对主导求和；conditional 变量未定义 |
| Parameter Entropy ↑（10-bin 均匀为理想） | ✏️ 改为 active-parameter 目标分布 coverage/divergence → Table 2 | 很多参数本应截断/对数/条件分布，均匀高熵不等于更真实 |
| Effective Config. No. ↑ | ✅ 保留（必须与 N_valid、coverage 同报）→ Table 2 + Scaling | 熵聚合稳健，但受样本数限制 |
| Dead Parameter Rate ↓ | ✏️ 改为干预/反事实检验 → Table 2 | 被动观察分不清「没采到」和「无效果」 |
| Mechanical Validity ↑ | ✅ 保留 → 1B / Table 2 联合报告 | 防止无效极端样本虚增 diversity |
| Scaling Panel A（pairwise coverage）/ Panel B（有近邻比例） | ❌ 换成四个可证伪 estimand（第 5 节） | 前者定义上单调不减、后者定义上随样本上升，无法证伪「重复堆量」 |
| 结论段断言口径 | ✏️ 条件化模板（第 9 节） | 所有结果列目前仍是 `—` |

# 1. 冻结统计契约（先于所有指标生效）

1. **Requested 分母**：先冻结 `category × method × requested asset IDs/seeds`；失败不替换；报告 `Valid / Requested`；diversity 指标是 valid subset 上的 conditional 统计，同时报告 requested-denominator coverage。与 `URDF-Sim-Ready-Automatic-Evaluation.md` Table 1 及既有 fail-closed 规则一致。
2. **Paper-reported 规模永不作分母**（`Paper-reported Assets` 仅作信息列）。
3. **分母必须报告相对 `N_eval` 的 coverage**；指纹/拓扑不完整的资产从其分母中排除、不计为 unique。
4. **先类别内计算，再 category macro-average**；category bootstrap 95% CI；重采样 5 次、冻结 seed。
5. **单调箭头只给可证伪且方向有意义的指标**；复杂度/集中度描述量不加箭头。
6. 所有 `N/E`、`not_evaluable` 显式呈现，不静默丢弃。

# 2. Table 1A — 原生规模与 provenance（无结构排名）

| Dataset | Paper-reported Assets（仅信息） | N_release | N_eval | Canonical categories | 版本 / license |
| --- | ---: | ---: | ---: | --- | --- |
| PartNet-Mobility | 2,346 | 2,347 | 800 | 46 | manifest 已冻结 |
| PhysX-Mobility | 2,024 | TBD | TBD | 47 | TBD |
| Infinite Mobility | on-demand | 冻结 commit 后定 | TBD | 论文 22 类（以实际 registry 为准） | commit TBD |
| Articraft-10K | 10,018 | 9,996 | 800 | 240（论文）/ 222（样本覆盖） | HF rev 3c79d5a0，cc-by-4.0 |
| Artiverse | 5,402 | 3,544（本地预发布） | 800 | 84 / 67 | 固定全局样本 seed 20260813 |
| LAM released outputs | N/R | 3,217（viable 2,533 / loads_only 299 / broken 385） | 800 | 787 prompt-categories / 305 eval 类 | HF rev 28cec4f5，mit |
| **PV-A-300K** | **300K / 500（论文声称）** | **❗manifest 未冻结** | ❗ | **❗canonical category roster 未冻结** | 阻塞项，见第 7 节 |

注：

- LAM 的 787/约 660 是 prompt-category 字段，不等于人工合并后的语义 taxonomy；必须合并同义词与措辞变体。
- 「重新运行 LAM 补齐」是新生成 cohort，必须单列版本、prompt、预算与失败策略，不能并入 release row。
- 前六列已冻结值来自 `dataset_inventory.json`、`DATASETS.md` 与已完成的正式 Table 1 run；TBD 项在冻结前不得填估计值。

# 3. Table 1B — 冻结共享类别 matched cohort（结构多样性 + validity）

Cohort 定义：`C_common` = 各核心数据源都能提供 ≥ n 个 requested 资产的类别；**名单只在下载全部 manifest 后确定**，不凭名称猜测；必须报告 `|C_common|`、每类 requested/valid、category mapping 表。预算：每类每方法 `n = 20`（requested），重采样 5 次。

| Dataset | Valid / Requested | Movable Joints mean/median/P90 | Multi-joint % | Unique Canonical Topology %（+分母 coverage） | Within-cat Signature：Unique Rate / Mode Rate / Pairwise Exact / Norm. Entropy | Normalized Effective Topology `N_eff/N_valid` [CI] | Exact Duplicate Excess % | Geometry Near-Duplicate Excess % |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| …（每个数据源一行，PV-A 同预算） | | | | | | | | |

列定义与来源：

- `Valid / Requested`：主列；失败不替换。
- `Movable Joints`、`Multi-joint %`：描述量，无箭头；复用现有 `run_table1_*` 实现（声明层统计，不等同 runtime 可执行 DoF，需在脚注重申）。
- `Unique Canonical Topology %`：规范化有根运动树 graph hash，分母 = 具有 valid rooted tree 的可评估资产，同时报 coverage；复用现有实现。
- Within-category signature 统计：沿用 Table 3D 定义（canonical signature 去语义名与 sibling order，保留 rooted shape、joint type、visual/group role）；**描述量，不做单调排名**。
- `Normalized Effective Topology`：`N_eff = exp(H)` 在 canonical signature 上计算，报告 `N_eff / N_valid` 与 bootstrap CI，避免失败数差异造成的假比较。
- `Exact Duplicate Excess`：canonical URDF + simulation-resource closure 指纹；重复簇 excess（簇大小 − 1）除以指纹完整的可评估资产；复用现有实现。
- `Geometry Near-Duplicate Excess`：新实现，按下述冻结定义。

**Geometry Near-Duplicate Excess 的冻结定义**

1. 表示：unit-normalized 表面点云，点数冻结。
2. 检索：embedding top-k 候选 → 候选上计算精确 Chamfer。
3. 判重：Chamfer < τ；τ 由约 1,000 对人工标注校准，标注集必须包含随机 negatives、困难 negatives 与独立 held-out，报告 precision/recall 与 CI，取高 precision 阈值。
4. Numerator 固定为 excess：near-neighbor 边构成连通簇，每簇计 `size − 1` 个 excess 资产，报告 `excess / evaluable` 与其分母 coverage。
5. 附录可选：joint-aware full-asset duplicate excess（拓扑 + 几何 + joint 状态联合）。

**Go/no-go**：若 `|C_common| < 10`，或任一方法在多数类别 valid < 10，则主表降级为「pairwise-overlap 面板 + available-only 附录」，并在正文显式说明交集退化。

# 4. Table 2 — PV-A 内部等预算 generator-space 覆盖

只评估 PV-A（静态数据集没有 generator blueprint / resolved parameters）。三行等 requested 预算、冻结 seed：Continuous-only / Structure-only / Full PV-A。与 `PV-A-Generator-Seed-必做消融实验.md` 的 E1–E4 边界对齐，不相互混用。

| Sampling Variant | Sampled-Support Pairwise Coverage（pair-balanced） | Active-Parameter Target-Distribution Coverage | Effective Config. No.（+N_valid, coverage） | Intervention-tested Dead Parameter Rate | Valid Novel Yield |
| --- | --- | --- | --- | --- | --- |
| Continuous-only | — | — | — | — | — |
| Structure-only | — | — | — | — | — |
| Full PV-A | — | — | — | — | — |

- `Sampled-Support Pairwise Coverage`：每个变量对先算 observed/feasible，再对 pair 做 macro-average、对 generator 做 macro-average。Feasible support 优先从 generator schema、dependency、compatibility rules **精确枚举**；不可行时使用 sampled support，但必须冻结独立采样 seed、样本量、收敛诊断、unseen-support sensitivity，并在指标名中显式标注 sampled-support。
- `Active-Parameter Target-Distribution Coverage`：只在参数 active 条件下评估；对照 generator 声明的目标分布；连续参数报 divergence/coverage；对数尺度、周期、bounded geometric 参数分别处理。
- `Dead Parameter Rate`：冻结其余参数，对目标参数做受控扰动/反事实，检查 resolved configuration / geometry / joint parameters 是否变化；「跨 seeds 未变化」只作辅助证据。
- `Valid Novel Yield`：`new valid nonduplicate assets / requested assets`，与 validity 联合报告。

# 5. Figure — Scaling（四个可证伪 estimand）

每个 generator 按 `n ∈ {10, 25, 50, 100, 250, 500, full}` 截断采样；5 个冻结随机 seed order；报告 median 曲线与 P10–P90 band，并同时给 Head/Mid/Tail 分层与「哪些 generator 达到 full budget」。

- Panel A：cumulative effective configurations（熵聚合）。
- Panel B：marginal novel configurations，每新增 100 个 requested 样本。
- Panel C：duplicate excess rate（exact + geometry calibrated 两条）。
- Panel D：valid novel yield = `new valid nonduplicate / requested`。

禁止把 pairwise coverage 曲线或「至少有一个近重复邻居」比例单独作为证据（定义上单调）。行文用四条曲线联合解释，不使用「证明」措辞。

# 6. Gallery

- 8–12 个类别，分层随机冻结选择（家电、家具、工具、机械结构、长尾）；选择规则与 seed 写入 receipt，禁止 cherry-pick。
- 每类 5–6 个同 generator 样本，并至少含 1 个 failure case。
- 每个样本给 rest pose 与一个 articulated state。
- 图下注明变化来源：module choice / part count / layout / continuous dimensions / joint configuration。
- PCA / t-SNE 仅作补充可视化，不替代定量指标。

# 7. 阻塞性前置条件（完成前不实现 evaluator、不跑正式数）

1. **PV-A dataset manifest 冻结**：资产清单、canonical category roster、每类数量、mechanical/URDF receipt 状态。在此之前，摘要中 “300K simulation-ready assets across 500 categories”（`exp.tex` abstract）属于超前表述，不得进入任何结果文本。
2. **Baseline 版本冻结**：Articraft / LAM 已在 `dataset_inventory.json`；Artiverse 固定样本 seed 20260813；Infinite Mobility commit + generator registry 待冻结；PartNet / PhysX manifest 复核。
3. **统一运动学表示**：dummy-link 收缩、fixed-joint 合并、joint-type 统一（fixed/revolute/continuous/prismatic/展开 multi-DoF）、part taxonomy；graph canonicalization、hashing、edit cost 对全部数据集一致。
4. **`C_common` roster + category mapping + 每类 requested/valid**。
5. **Near-duplicate 标注集**（含随机/困难 negatives 与 held-out）。

# 8. 执行顺序

1. 前置 1–4（阻塞）。
2. Table 1A：补齐 Infinite Mobility / PV-A 行。
3. Table 1B Tier-1 列：复用 `run_table1_*` 与 Table 3D signature 实现，扩展到 `C_common` cohort。
4. Near-duplicate 校准 → Table 1B geometry excess 列。
5. Table 2：优先 schema 精确枚举 feasible support，否则 sampled-support 口径。
6. Scaling figure（依赖 Table 2 signature 与 duplicate 判定）。
7. Gallery + 附录（raw TED/nTED robustness、joint-aware duplicate、per-category 分布、阈值校准细节、LAM available-only、pairwise-overlap 面板；仅在存在统一外部 reference 的重叠类别上可选报告 MMD/COV/1-NNA，不作为 500+ 类核心证据）。

# 9. 论文结论口径（条件化模板）

> If the preregistered matched-category evaluation confirms higher effective topology diversity and lower calibrated near-duplicate excess while preserving requested-denominator validity, these results would indicate that PV-A's scale is not explained solely by repeated sampling.

在 Table 1B / Figure 有数字之前，不写任何比较性断言。

# 10. 主文与附录分工（V2）

- 主文：Table 1A、Table 1B、Table 2、四 panel Scaling figure、Gallery。
- 附录：Var. Joints、nTED（若实现，含 raw TED 与 normalization robustness）、joint-aware duplicate、per-generator 分布与 failure cases、duplicate 阈值校准、LAM available-only 结果、各 baseline 更大的 pairwise-overlap 面板。

# 11. Sources 与仓库契约

- V1 所列外部来源（PV-A paper、Infinite Mobility、Infinigen-Articulated、Articraft、Artiverse、LAM release）不变；nTED 必须标注为 “our normalized variant”。
- 仓库契约：`URDF-Sim-Ready-Automatic-Evaluation.md` Table 1（requested 分母、coverage 同报、失败不替换）、`Table3_Hierarchy_Baselines.md` Table 3D（signature 描述量）、`dataset_inventory.json`、`DATASETS.md`、`PV-A-Generator-Seed-必做消融实验.md`（E1–E4 边界）。
