# PV-A Generator Seed 必做消融实验

> 状态：预注册执行稿，不包含新的实验结果。
>
> 目标：只保留能够直接支撑 PV-A 核心 claim 的实验，避免把主文预算消耗在模型、提示词或可视化类的弱消融上。

## 0. 结论

PV-A 的 generator 实验应收敛为 **3 组核心消融 + 1 个下游闭环**：

1. **Authoring Pipeline 消融**：Source Map 和 Template Design 是否提高未见类别 generator 的构建成功率。
2. **Generated Seed 空间消融**：PV-A 是否产生结构新组合，而不只是尺寸扰动或源码实例重放。
3. **Validation 与 Repair 消融**：分布式 gate 和结构化诊断是否减少 hidden failure，并提高同预算修复成功率。
4. **Equal-budget 下游实验**：PV-A seeds 的收益是否来自结构多样性，而不是单纯增加数据量。

前三组是 generator 方法论文不可缺少的内部消融。只要论文保留“300K 数据集改善下游任务”的 claim，第四组也必须完成。

此外，整篇论文需要一张统一协议的外部方法比较表，但它不是内部消融，不能替代上述实验。

## 1. Claim 与实验对应关系

| PV-A claim | 必须由什么实验支撑 | 不能作为替代证据 |
|---|---|---|
| Agent 能从源码构建可复用的类别级 generator | Authoring Pipeline 2x2 消融 | 单个成功案例、最终模板数量、同模型换 reasoning effort |
| Generated seeds 具有结构和组合多样性 | Size-only / Source-tuples-only / Full 对照 | 只报告参数范围、seed 数量、视觉拼图或 embedding 可视化 |
| Distribution-level validation 能发现系统性错误 | 累积 gate replay + 独立 hidden evaluator | authored tests、36/36 development seeds、默认姿态可加载 |
| Structured repair 能稳定修复失败 | 从相同 round-0 快照分叉的同预算对照 | 展示修复后的最终成功产物 |
| PV-A 数据对下游任务有用 | Equal-budget 数据源实验 | 数据规模、资产有效率、主观质量评分 |
| 资产 simulation-ready | 独立运动、碰撞和 native simulator 验证 | `strict_ready`、URDF load、rest-pose 无碰撞 |

## 2. P0：正式实验前必须冻结的内容

以下条件未完成时，只能运行 calibration 或 pilot，不能生成论文确认性结论。

### 2.1 Cohort 与泄漏控制

- 建立互斥的 development、fresh confirmatory 和 stress cohorts。
- 正式 Authoring 消融使用 **24 个 fresh 类别**；这些类别不得已有目标模板、Source Map、Template Design、authoring output 或人工反馈记录。
- 现有 seed `0-35`、authored corners、历史失败样例和调试过的类别均属于 development evidence，不能再次充当 hidden test。
- 类别复杂度在运行前冻结，建议包含 8 个单关节、8 个多组件或 multiplicity 类别、8 个多关节或高风险接触类别。

### 2.2 版本与预算

冻结并记录：

- raw source revision、文件清单和 SHA256；
- generator/template commit 与最终 hash；
- SDK、compiler、renderer 和 simulator 版本；
- provider、精确模型 ID、reasoning effort、temperature 和 API seed（若支持）；
- turn、token、wall-clock、tool-call 和费用上限；
- prompt、job 顺序、超时与 retry policy。

若模型 API 不支持 generation seed，不得把几何检查用的 `TestContext.seed` 写成 LLM seed。此时记录独立 run ID、请求时间、完整响应和随机化 job 顺序。

### 2.3 独立 hidden evaluator

- generator hash 封存后才生成或揭示 hidden cases。
- hidden config、covering array、数值边界、multiplicity 边界、FamilyGold 和 joint gold 由 evaluator 持有。
- hidden evaluator 不得调用被测模板自己的 `config_from_seed()`、`TEMPLATE_CORNERS` 或 authored tests 来定义 gold。
- hidden failure 不反馈给当前论文实验的 author/repair agent。
- 推荐用预先承诺的 secret salt 派生 hidden seed，例如 `SHA256(generator_hash || salt || index)`，实验结束后 commit-reveal。

### 2.4 Manifest 与分母

每个任务至少保存以下状态：

```text
requested -> attempted -> generated -> compiled -> exported
          -> automated-strict-valid -> semantic-reviewed -> accepted
```

Timeout、provider error、空输出和无法导出均保留在 intent-to-run 分母中。缺少碰撞体、joint gold 或 simulator 前置条件时，对应指标记为 `N/E` 或 `N/A`，不能从已成功子集重新定义分母。

## 3. E1：Authoring Pipeline 2x2 消融

### 3.1 研究问题

显式 source provenance/navigation 信息和显式 generator design 信息，是否分别或联合提高未见类别 generator 的构建成功率？

### 3.2 四个实验组

| Arm | `S_factor` | `D_factor` | Agent 可见信息 |
|---|---:|---:|---|
| A00 Raw-only | 否 | 否 | 相同 raw source pool、任务说明和 SDK |
| A10 Source-only | 是 | 否 | Raw pool + source inventory、provenance、span、candidate 和 owner/navigation |
| A01 Design-only | 否 | 是 | Raw pool + 去除 source path/span/evidence 的 generator blueprint |
| A11 Full PV-A | 是 | 是 | 完整 Source Map 与 Template Design 信息 |

`D_factor` 必须去除 source path、revision、span、evidence quote、source hash 和 implementation owner，防止 Source Map 信息从 Design 侧泄漏。四组的最终任务完全相同：生成同一接口的 **reusable generator**，不能让弱组只生成一个资产。

### 3.3 固定项与规模

- 24 个 fresh 类别；
- 4 个 arms；
- 每个 arm/category 独立运行 3 次；
- 总计 `24 x 4 x 3 = 288` 个 authoring runs；
- 相同模型、SDK、raw source pool、预算、development gate 和 repair 轮数；
- 每次使用新会话和隔离 workspace。

若预算只能支持 12 类，即 `12 x 4 x 3 = 144` runs，该轮必须标为 pilot；不能据此对 `S_factor x D_factor` interaction 下强结论。

### 3.4 指标

**共同主指标：**

1. Final Generator Success：固定预算结束后，通过预注册 hidden minimum suite 的 generator 比例；
2. Hidden Strict-valid Yield：全部 hidden attempts 中机械契约通过的比例，按 generator/category macro 汇总。

**次指标：**

- first-shot hidden yield；
- Category Precision 和 FamilyGold role recall；
- repair 次数、wall time、token 和费用；
- hidden pairwise/corner coverage；
- old-case retention 和新增 regression 数。

这组实验评估的是 authoring 时信息可用性的因果效应。它不能单独证明 Source Map 或 Template Design 的人工制作成本是否值得，后者需要另行记录 preparation cost。

## 4. E2：Generated Seed 空间消融

### 4.1 研究问题

PV-A 的有效多样性究竟来自连续尺寸扰动、源码已有实例重放，还是约束下的新结构组合？

### 4.2 三个实验组

| Arm | 连续参数变化 | 源码已有完整 tuple | 跨 source 新组合 |
|---|---:|---:|---:|
| G0 Size-only | 是 | 固定一个结构 | 否 |
| G1 Source-tuples-only | 是 | 是 | 否 |
| G2 Full combinatorial PV-A | 是 | 是 | 是，但必须满足 bindings、interfaces 和 adaptation 约束 |

三个 arms 从同一批冻结、Design-backed 的 Full generator 派生，只改变 domain policy，不重新执行 agent authoring。这样可以把结果归因于 seed 空间机制，而不是不同模板质量。

### 4.3 规模

- 24 个冻结 generator；
- 3 个 domain arms；
- 每个 generator/arm 至少 100 个 evaluator-owned hidden attempts，推荐 200 个；
- 最小规模 `24 x 3 x 100 = 7,200` attempts；
- 推荐规模 `24 x 3 x 200 = 14,400` attempts。

seed 是同一 generator 内的重复观测，不能把 14,400 个 attempts 当成 14,400 个独立样本。统计主单位仍然是 generator/category。

### 4.4 指标

**共同主指标：**

1. Hidden Strict-valid Yield：全部 requested attempts 中通过独立机械 gate 的比例；
2. Valid Novel Combination Rate：在盲审子集中，同时满足机械有效、类别/功能语义有效且不属于任何源码完整 tuple 的比例。

**次指标：**

- Category Precision；
- Structural Family Recall；
- joint motif、部件 multiplicity 和 topology coverage；
- graph near-duplicate rate；
- Invalid Combination Rate；
- worst-decile generator yield；
- throughput 和每个有效新组合的成本。

机械指标在全部 attempts 上计算。语义与新组合正确性必须在预注册盲审子集上估计；建议每个 generator/arm 随机抽 25 个 attempts，由两位评审者独立标注、冲突时第三人仲裁。VLM 可以做预筛或规模化 proxy，但不能代替主语义结果。

主文应同时展示 valid yield 和 novelty/coverage，不能构造一个加权“总质量分”。最适合的图是 quality-diversity Pareto 图。

## 5. E3：Validation 与 Repair 消融

### 5.1 Gate replay

对 E1 的同一批 immutable first-shot generators 离线执行累积 gate：

| Arm | 累积检查 |
|---|---|
| V0 | compile + default/authored checks |
| V1 | V0 + preflight |
| V2 | V1 + random-16 |
| V3 | V2 + random-36 |
| V4 | V3 + authored corner cases |

每一级 gate 的 accepted set 都必须再由相同的独立 hidden evaluator 检查。

**主指标：**

- Accepted-set False-accept Proportion：被该级 gate 接受、但在 hidden evaluation 中存在缺陷的 generator 比例；
- Incremental Defect Discovery：本级首次发现且前级未发现的缺陷数或 generator 数。

**次指标：** gate wall time、每个新增缺陷的检查成本、缺陷类型分布和 worst-case hidden yield。

该实验复用 E1 的产物，不需要新的 LLM authoring runs。`random-36` 和 corners 是 development gate，不因其通过就等同于 hidden validity。`strict_ready` 只表示预注册机械契约通过，不表示外观、类别语义、动力学或 simulator readiness。

### 5.2 Repair 分叉实验

只纳入 round-0 失败任务，并从完全相同的失败快照分叉：

| Arm | Repair 输入 |
|---|---|
| R0 No repair | 保留原始失败状态，作为共同起点 |
| R1 Raw feedback | 原始 compiler/test logs + 通用 retry 指令 |
| R2 Structured PV-A repair | 结构化 failure localization、owner/source routing、相关约束和 targeted repair 指令 |

R1 与 R2 使用相同模型、最多一轮 repair、相同 input/output token、tool、wall-clock 和费用预算。生产系统的三轮累计恢复曲线可以作为补充描述，但一轮分叉是更干净的因果比较。

**主指标：** round-0 failure-conditional hidden recovery rate。

**次指标：** development gate recovery、hidden regression count、无效编辑率、修改范围、时间与费用。所有 round-0 failures 采用 intent-to-treat 汇总；不能只统计成功启动 repair 的任务。

## 6. E4：Equal-budget 下游闭环

### 6.1 研究问题

PV-A 数据带来的收益是否来自结构/组合多样性，而不是数据数量或连续尺寸扰动？

### 6.2 Particulate 训练组

| ID | Training data | 作用 |
|---|---|---|
| D0 | Base-B | 原始训练基线 |
| D1 | Base-2B | 控制单纯增加同源数据量 |
| D2 | Base-B + PV-A Size-only-B | 控制连续尺寸增强 |
| D3 | Base-B + PV-A Full-B | 检验结构组合带来的额外收益 |

其中 `B` 按所有数据组在冻结类别配额下能够共同交付的最小 strict-valid 数量确定。D1、D2、D3 的总训练资产数、训练 steps、batch、backbone、optimizer 和数据预处理必须一致。

### 6.3 运行与测试划分

- 每组至少 3 个独立 train seeds，共 12 次正式训练；
- 主测试使用 generator-family/source-component-disjoint split；
- 同时报告 unseen combination 和 category-OOD；
- unseen seed 只代表同一 generator 内插，不作为主 OOD；
- 在运行前从 Particulate 协议中冻结一个主指标，推荐 OOD articulated gIoU；其余 rest/articulated gIoU、PC/dcDist、mIoU 和 OC 为次指标。

报告每个 train seed 的结果、均值与离散程度，并对 test asset/family 做成对 bootstrap。若 D3 只优于 D0、但不优于 D1 和 D2，不能把提升归因于结构组合多样性。

## 7. 外部 baseline：整篇论文必做，但不是消融

建议建立一个 common-protocol 表：

- **Infinite Mobility**：reusable procedural generator 的首要直接对照；
- **Infinigen-Sim / Infinigen Articulated**：在官方公开重叠类别上比较参数域、组合域、导出与下游；
- **Articraft**：per-asset agent 与 native package 的参照，不标成 reusable-generator baseline。

所有方法只在公开实现支持的重叠类别上比较，并固定 requested attempts、canonical render、hidden evaluator 和 intent-to-run 分母。Paper-reported 数字与本地统一协议结果必须分行。

这三项工作的实验设计可借鉴之处是统一类别/预算、人评、数据源对照和多训练 seed；它们并未提供足以替代 PV-A 内部组件消融的证据：

- [Infinite Mobility](https://arxiv.org/html/2503.13424) 主要是完整方法质量、结构统计、时间和下游数据实验；
- [Infinigen-Sim](https://arxiv.org/html/2505.10755) 主要是数据源、感知、RL generalization 与 sim-to-real；
- [Articraft](https://arxiv.org/html/2605.15187) 的正式 generator ablation 主要是单一 drone prompt 下的模型和 reasoning-effort 定性比较。

因此，外部 baseline 表不能回答 PV-A 的 Source Map、Design、组合域、gate 或 repair 分别贡献了什么。

## 8. 最小资源预算

| 实验 | 正式最小规模 | 说明 |
|---|---:|---|
| E1 Authoring | 288 authoring runs | 24 类 x 4 arms x 3 replicates |
| E2 Seed domain | 7,200 hidden attempts | 24 generators x 3 arms x 100；推荐 14,400 |
| E3 Gate replay | 复用 E1 first-shot 产物 | 无新增 authoring 调用 |
| E3 Repair | 最多 576 repair runs | 若 288 个 round-0 全失败，分别运行 R1/R2；实际按失败数决定 |
| E4 Downstream | 12 train runs | 4 个数据组 x 3 train seeds |
| E2 语义盲审 | 1,800 assets | 24 generators x 3 arms x 25；两人独立标注并仲裁 |

如果预算不足，优先级为：

1. 保住 E1 的 24 个独立类别，不要用增加 seed 数替代类别数；
2. E2 从每 arm/generator 200 attempts 降到 100；
3. E3 复用 E1 产物；
4. 缩减外部 baseline 的类别 panel；
5. 最后才考虑把 E4 降为预注册 pilot。

不能通过删除 intent-to-run failures、hidden evaluator 或语义盲审来节省预算。

## 9. 统计与报告规则

1. generator/category 是主要独立单位；seed 是 cluster 内重复观测。
2. 主结果使用 category/generator macro average，并按 category cluster bootstrap 给出 95% CI。
3. 同时报告 micro 结果，但不作为主结论。
4. 二元 seed 指标报告 Wilson 或 Clopper-Pearson CI；0 failure 仍报告上界。
5. 报告 median、IQR 和 worst decile，不能只报告均值。
6. 每个实验预注册 1-2 个主指标；同一指标 family 内做 Holm 或 BH 多重比较修正。
7. 人工评审隐藏方法名、seed 和左右顺序，报告一致性与仲裁比例。
8. 不把 compiled、exported、strict-valid、semantic-valid、physical-valid 和 simulator-ready 合并为一个 success 字段。
9. 不构造单一加权总分；fidelity、diversity、control、reliability、physics 和 utility 分开报告。
10. `N/A`、`N/E`、timeout、unknown 和被排除原因保留在结果表与 manifest 中。

## 10. 主文建议保留的四张结果表

| 表 | 内容 | 核心结论 |
|---|---|---|
| T-A | Authoring 2x2 消融 | Source Map、Design 及其联合是否提高 generator authoring 成功率 |
| T-B | Seed-domain 消融 | Full 是否在保持 valid yield 的同时产生更多有效新组合 |
| T-C | Gate + Repair 消融 | 分布式检查和结构化诊断是否减少 escaped defects 并提高修复率 |
| T-D | Equal-budget downstream | 结构组合数据是否优于同量数据和 size-only 增强 |

外部方法的 common-protocol comparison 另设一张 baseline 表，不与 T-A 至 T-C 混排。

## 11. 不属于主文必做的实验

以下内容可以放 supplementary，不能挤占上述实验预算：

- 不同 LLM、provider 或 reasoning effort；
- 每个 Template Design 字段逐项删除；
- prompt wording 和温度网格；
- PBR 材质、纹理或灯光消融；
- 大规模纯主观偏好排名；
- t-SNE/PCA 之类仅用于展示的 embedding 图；
- 完整复刻 CAGE training experiment；
- 多 simulator 扩展、复杂机器人交互或 VR demo。

其中 native simulator readiness 若继续作为论文主 claim，需要单独的正式验证表；它不是 generator seed 消融，也不能由本文件中的 `strict-valid` 结果推导。

## 12. Go / No-Go

满足以下条件后才能启动正式运行：

- [ ] 24 个 fresh 类别及排除清单冻结；
- [ ] 四个 authoring packets 通过信息等价性与 leakage 自检；
- [ ] generator/domain arms 的唯一变量经过机械 diff 审计；
- [ ] hidden evaluator 不依赖被测模板的 seed sampler、corners 或 authored tests；
- [ ] FamilyGold、joint gold、secret manifest 和 commit hash 冻结；
- [ ] 预算、超时、provider error 和 retry policy 预注册；
- [ ] intent-to-run manifest 与完整分母字段可用；
- [ ] 每个实验的主指标、统计单位和停止规则冻结；
- [ ] 300K/500-category 数据声明有独立 dataset manifest 支撑；
- [ ] `strict_ready`、semantic-valid、physical-valid 和 simulator-ready 的文案边界与运行时一致。

任一关键项未满足时，状态应记为 `BLOCKED` 或 `PILOT`，不能把准备好的 runner、部分输出或 development pass 写成正式实验完成。

## 13. 与现有材料的关系

- 完整实验池与 baseline 分工见 [PV-A-全新实验设计-Generator-Seed.md](./PV-A-全新实验设计-Generator-Seed.md)。
- Authoring、gate 与 repair 的详细预注册见 [Arti-Skill-Pipeline-Ablation-Design.md](./Arti-Skill-Pipeline-Ablation-Design.md)。
- 当前论文材料见 [_ICLR_2027__PV_A.pdf](./_ICLR_2027__PV_A.pdf) 与 [_ICLR_2027__PV_A.zip](./_ICLR_2027__PV_A.zip)。

本文是上述材料的“必做最小集”，不是新增实验结果，也不改变已有结果状态。
