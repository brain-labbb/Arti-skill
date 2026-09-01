<aside>
✅

**最终方案**：主论文的 diversity 验证控制为 **2 张表 + 1 张 scaling figure + 1 组定性 gallery**。Table 1 做跨数据集的类别规模、资产规模与类内结构多样性；Table 2 只评估 PV-A 的 generator configuration space；Scaling figure 证明从少量 seed 扩展到 300K 时，多样性仍在增长而不是简单重复。

</aside>

# 1. 要回答的四个核心问题

1. **Category breadth**：500+ category-level generators 最终覆盖了多少规范化语义类别和多少资产？
2. **Within-category structural diversity**：同一类别内部是否具有不同的 part 数量、运动学拓扑和 joint 组合？
3. **Generator-space coverage**：generator 声明的离散模块组合与连续参数范围，最终数据是否真正覆盖？
4. **Redundancy and scaling**：300K 中是否存在大量近重复资产；每类样本量增加时，多样性是否继续提升？

# 2. Table 1：跨数据集规模与类内结构多样性

| Dataset / Generator | Categories / Generators | Assets | Avg. Joints | Var. Joints ↑ | Avg. nTED ↑ | Effective Graph No. ↑ | Near-Duplicate Rate ↓ |
| --- | --- | --- | --- | --- | --- | --- | --- |
| PartNet-Mobility | 46 categories | 2,346 | — | — | — | — | — |
| Infinite Mobility | 22 object generators in paper [Note 1] | On-demand; freeze evaluation set | — | — | — | — | — |
| Infinigen-Articulated | 18 categories | 20,000 released assets | — | — | — | — | — |
| Articraft-10K | 245 categories | 10,018 / 10K+ | — | — | — | — | — |
| Artiverse | 88 categories | 5,402 / 5.4K | — | — | — | — | — |
| LAM viable assets | ~660 prompt categories | 2,533 viable; 3,217 total | — | — | — | — | — |
| **PV-A-300K** | 500+ category-level generators [Note 3] | **300,000** | **—** | **—** | **—** | **—** | **—** |
|  |  |  |  |  |  |  |  |

<aside>
📌

**Assets 列已加入 Table 1。** 同时补全了此前遗漏的 **Artiverse** 和 **LAM viable assets**。Categories 与 Assets 使用各数据源的原生规模；后五个统计量不能直接用各自全量数据比较，必须使用统一类别和相同每类样本预算。

</aside>

- * Infinite Mobility 论文描述 22 种 common articulated objects；执行实验时还要锁定公开仓库 commit，并以实际 generator registry 为准。
- ** LAM 的约 660 个值是 prompt/category 字段，不等同于经过人工合并的语义 taxonomy；必须合并同义词、长描述和措辞变体。
- * PV-A 的 500+ 首先是 category-level generator 数。论文定稿前应另行冻结 canonical semantic category count，避免把多个同义 generator 重复算作不同类别。

## 2.1 Table 1 中每个指标测什么

### Avg. Joints

统计每个资产的独立可动关节数，并先在类别内求均值，再对类别做 macro-average。只统计 revolute、continuous、prismatic 等 movable joints；fixed joint 不计入。该指标反映**结构复杂度**，不是严格的 diversity 指标，因此不加单调方向箭头。

### Var. Joints ↑

先在每个类别内部计算关节数方差，再对类别做 macro-average。它来自 Infinite Mobility 的 diversity 评价思路，衡量同一类别中是否同时存在单门、多门、多抽屉、多关节等不同复杂度层级。

### Avg. nTED ↑

为每个资产构建 rooted kinematic tree：节点为 functional parts，边为 movable joints，节点标签为统一 part semantic，边标签为 joint type。采用归一化 Tree Edit Distance：

$$
d_{nTED}(T_i,T_j)=\frac{TED(T_i,T_j)}{|V_i|+|V_j|}.
$$

同类别资产两两计算后再做类别宏平均。它沿用 Infinite Mobility 的 TED 思路，但通过归一化降低大树天然获得更高距离的问题。主论文使用 nTED，附录可同时给 raw TED 以便和原论文口径对应。

### Effective Graph No. ↑

将 part connectivity、统一 semantic labels 和 joint types canonicalize 成 kinematic graph。若某类别内第 $g$ 种图的频率为 $p_g$：

$$
H_c=-\sum_g p_g\log p_g,\qquad N_{eff,c}=\exp(H_c).
$$

该指标来自 Artiverse 的 graph-entropy 思路。它表示有多少种被充分覆盖的有效结构模式，比 raw unique graph count 更稳健：只出现一次的稀有图不会与大量均衡出现的图获得相同权重。

### Near-Duplicate Rate ↓

只在同一 canonical category 内寻找最近邻。推荐将资产判为 near duplicate 的条件设为：

1. canonical kinematic graph 完全相同；
2. unit-normalized surface point cloud 的 Chamfer Distance 小于阈值 $\tau$。

阈值 $\tau$ 不要拍脑袋决定。先人工标注约 1,000 对同类资产是否属于近重复，再选择高 precision 的阈值。大规模计算时先用 point-cloud 或 multi-view embedding 检索 top-k 候选，再对候选计算精确 Chamfer Distance。

# 3. Table 1 的统一公平评测协议

## 3.1 先统一 taxonomy

- 合并同义词和近义类别，例如 `fridge` / `refrigerator`。
- 分开真正不同的功能类别，不因为外观接近而强行合并。
- 输出一份 `category_mapping.csv`，包含 original label、canonical label、dataset、mapping confidence 和人工复核状态。

## 3.2 只评估有效资产

所有 diversity 指标只在通过以下检查的资产上计算：

- mesh 与 link 引用完整；
- kinematic graph 连通且无环；
- URDF / MJCF 能加载；
- joint axis、limit 和 parent-child 合法；
- 多个 joint states 下无明显非预期穿插或失联。

必须同时报告 valid asset count，避免错误资产或异常结构虚假抬高 diversity。

## 3.3 固定共享类别和每类样本量

主表建议使用一个冻结的 `C_common`：优先选择各核心数据源都能提供足够样本的常见类别。下载 manifest 后再最终确定类别列表，不先凭名称猜测。

推荐设置：

- 每类 $n=20$ 个有效资产；
- 每个数据集使用相同类别与相同 $n$；
- 随机重采样 5 次；
- 先按类别计算，再做 macro-average；
- 用 category bootstrap 报告 95% confidence interval。

LAM release 的平均每个 prompt category 样本较少。若无法为 `C_common` 中每类提供 20 个 viable assets，应采用以下二选一并在论文中明确：

- 重新运行 LAM，为每个共享类别生成相同数量的资产；
- LAM 保留在规模统计行，但 diversity metric 标为 `N/A`，另在附录报告 available-only 结果，不能混入主表公平排名。

## 3.4 统一运动学表示

- 收缩由格式转换产生的 dummy links。
- 合并仅由 fixed joints 连接且无独立功能语义的 links。
- 将 joint type 统一到 fixed、revolute、continuous、prismatic 和展开后的 multi-DoF。
- 语义标签映射到同一 part taxonomy。
- 所有 graph canonicalization、hashing 和 TED edit cost 对全部数据集保持一致。

# 4. Table 2：PV-A Generator-Space Diversity

这张表只评估 PV-A，因为静态数据集没有 generator blueprint、可行配置空间和 resolved parameters，不能公平填写 Configuration Coverage。

| Sampling Variant | Pairwise Config. Coverage ↑ | Parameter Entropy ↑ | Effective Config. No. ↑ | Dead Parameter Rate ↓ | Mechanical Validity ↑ |
| --- | --- | --- | --- | --- | --- |
| Continuous-only | — | — | — | — | — |
| Module / structure-only | — | — | — | — | — |
| **Full PV-A** | **—** | **—** | **—** | **—** | **—** |

## 4.1 Pairwise Configuration Coverage ↑

离散变量包括 component candidate、optional part、repeated-part count、layout、attachment type 和可变 joint type。完整笛卡尔积在高维下会爆炸，因此主论文报告可行二元组合覆盖：

$$
FPC_2(c)=\frac{\sum_{j<k}|\Omega^{observed}_{jk}|}{\sum_{j<k}|\Omega^{feasible}_{jk}|}.
$$

- `observed`：最终 PV-A-300K 中实际出现的组合。
- `feasible`：满足 dependency、adaptation 和 compatibility rules 的可行组合。

可行空间的求法：小 generator 直接穷举；高维 generator 运行 100K 个 `Sample + Resolve` configuration-only seeds，不构建 mesh，用其稳定估计 feasible support。

## 4.2 Parameter Entropy ↑

将每个连续参数按照 generator 声明范围归一化到 $[0,1]$，划分 10 个 bins，计算归一化 entropy。只统计确实应该变化的 active parameters。该指标主要检查参数是否塌缩或失效，不应单独解释为“越均匀越真实”。

## 4.3 Effective Configuration Number ↑

将离散 module choices、optional/repeated counts、layout 和 joint-type choices 组成 configuration signature。若 signature 频率为 $p_q$：

$$
N_{eff-config}=\exp\left(-\sum_q p_q\log p_q\right).
$$

它比 unique configuration count 更可靠，因为只出现一次的极少数配置不会过度抬高结果。

## 4.4 Dead Parameter Rate ↓

一个声明为 free 的参数如果跨足够多 seeds 后仍不变化，或变化后不影响 resolved configuration / geometry / joint parameters，则记为 dead parameter。该指标直接检查“代码里看起来有随机参数，但实际上没有产生变化”的问题。

## 4.5 Mechanical Validity ↑

配置覆盖和参数 entropy 必须与 validity 同时报告。否则随机采样大量极端或错误资产也能获得高 diversity。Validity 使用论文现有的 geometry、connectivity、URDF loading、collision 和 multi-state joint motion tests。

# 5. Figure：Diversity Scaling Curve

对每个 generator 按以下 seed budget 截断采样：

$$
n\in\{10,25,50,100,250,500,\text{full}\}.
$$

建议做两个 panel：

- **Panel A**：Pairwise Configuration Coverage 随每类样本数的变化；
- **Panel B**：Near-Duplicate Rate 随每类样本数的变化。

统计方式：每个 budget 重复 5 个随机 seed order；报告 500+ generators 的 median curve，并画 10%–90% percentile band。

这张图直接回答：每类从几十个样本增加到约 600 个样本时，多样性是否仍增长，以及 300K 是否只是重复堆量。

# 6. Qualitative Diversity Gallery

主论文放一组紧凑 gallery：

- 选择 8–12 个代表类别，覆盖家电、家具、工具、机械结构和长尾类别；
- 每类展示 5–6 个同 generator 样本；
- 每个样本同时给 rest pose 和一个 articulated state；
- 在图下注明变化来自 module choice、part count、layout、continuous dimensions 还是 joint configuration。

PCA / t-SNE 可以作为补充可视化，但不能替代定量 diversity 指标。

# 7. 主论文与附录的最终分工

## 主论文

1. Table 1：Categories、Assets、Avg. Joints、Var. Joints、Avg. nTED、Effective Graph No.、Near-Duplicate Rate。
2. Table 2：Pairwise Config. Coverage、Parameter Entropy、Effective Config. No.、Dead Parameter Rate、Mechanical Validity。
3. Figure：Configuration Coverage 与 Duplicate Rate 的 scaling curves。
4. Qualitative gallery。

## 附录

- raw TED 与不同 normalization 的 robustness；
- topology-only graph 与 semantic-aware graph 两种定义；
- Head / Mid / Tail category 分组结果；
- duplicate threshold 的人工校准；
- 每个 generator 的 per-category metric 分布与 failure cases；
- 仅在存在统一外部 reference 的重叠类别上，可选报告 MMD / COV / 1-NNA。它们不作为完整 500+ 类数据集的核心 diversity 证据。

# 8. 执行顺序

- [ ]  冻结所有 baseline 的版本、commit、manifest 与 license。
- [ ]  建立 canonical category taxonomy 和 part taxonomy。
- [ ]  将全部资产转换为统一 articulated representation。
- [ ]  完成 validity filter，并记录每个数据源的有效资产数量。
- [ ]  冻结 `C_common` 和每类 $n=20$ 的公平评测集合。
- [ ]  实现 joint statistics、nTED、graph canonicalization、Effective Graph No. 和 Near-Duplicate Rate。
- [ ]  从 PV-A metadata 计算 configuration coverage、parameter entropy 和 effective configuration number。
- [ ]  运行 scaling experiment 与 5 次重采样。
- [ ]  输出主表、置信区间、scaling figure、gallery 和 appendix diagnostics。

# 9. 论文中可以使用的结论口径

> PV-A-300K 的 diversity 不仅体现在更广的 category 和 asset coverage，也体现在共享类别下更丰富且更均衡的 kinematic graph 分布、更高的类内结构距离和更低的近重复率。进一步地，由于每个资产保留了 resolved configuration 与 generator metadata，我们能够直接衡量 generator configuration space 的覆盖程度，这是静态 articulated datasets 无法提供的验证维度。
> 

# 10. Sources

- PV-A paper（当前对话上传的 PDF）：300K assets across 500 categories，并保留 resolved configurations、selected modules、part hierarchy 和 joint parameters。
- Infinite Mobility
- Infinigen-Articulated
- Articraft
- Artiverse
- LAM viable asset release