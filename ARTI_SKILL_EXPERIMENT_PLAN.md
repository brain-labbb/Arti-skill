# arti-skill 项目实验方案

## 1. 实验定位

本方案评估的对象是 `arti-skill` 驱动的“参考资产/变体 → SourceMap → TemplateDesign → 自包含参数化模板 → seed 资产 → 机械与物理验证”完整管线，而不是单个模型或单个生成结果。

实验要证明的核心命题是：

> arti-skill 能把来源可追溯的 articulated 资产知识转化为可复用模板，并在不依赖运行时 LLM 的情况下，稳定生成结构多样、参数可控、运动合法且可进入仿真器的资产。

项目中的 `arti-template` 是模板作者与检查主仓；`articraft_data` 是上游资产/变体来源；`template_source_maps`、`template_maps` 是来源与类别组织信息；`seed_exports` 是生成数据；`data_check` 和各仿真器验证目录是独立质量证据。

## 2. 研究问题

| 编号 | 研究问题 | 需要回答的证据 |
|---|---|---|
| RQ1 | skill 协议是否提高模板构建可靠性？ | 首次成功率、最终成功率、修复次数、人工介入率、成本 |
| RQ2 | 模板是否能覆盖稳定且真实的生成域？ | 36-seed 通过率、corner 通过率、组合域覆盖、回归保持率 |
| RQ3 | 生成资产是否具备正确的结构与运动？ | link/joint、关节类型、轴、pivot、行程、静止/运动碰撞 |
| RQ4 | 生成域是否真正多样、可控、可复现？ | 拓扑/几何多样性、参数命中率、单调响应、重复编译一致性 |
| RQ5 | 生成资产是否具备仿真与下游使用价值？ | 多仿真器导入、重力稳定性、交互任务或下游分割性能 |

## 3. 评测对象与冻结协议

### 3.1 统计口径

正式实验开始前生成一份冻结的 `experiment_manifest.jsonl`，分别记录：

- `category_count`：语义类别数量；
- `template_count`：可执行模板数量；
- `template_slug` 与模板版本/hash；
- 是否存在 SourceMap、TemplateDesign、源资产池；
- 槽位数、每槽位候选数、连续参数维度；
- 声明的组合域、有效组合域和拓扑家族；
- 关节类型、link 数、可动关节数；
- 可运行的仿真器和验证版本。

类别、模板、seed 和导出资产必须分开计数，不能把“模板数”直接当成“类别数”。

### 3.2 Seed 套件

每个严格模板使用固定的 36 个标准 seed：

- 32 个覆盖 seed：离散候选采用 pairwise covering，连续参数采用固定的 LHS 或 Sobol 序列；
- 4 个边界 seed：连续参数接近最小值、最大值、最紧凑组合和最大展开组合；
- 另外保存由模板 Design 产生的 `corner` case，不将 corner 失败从随机 seed 中删除。

项目现有的 `preflight → random-16 → random-36 → corner` 是模板机械完成的权威阶段。实验报告既报告这些阶段的最终状态，也报告每个 seed 的原始失败、修复后状态和淘汰原因。

### 3.3 分层样本

轻量结构与运动检查覆盖全部冻结模板的 36-seed 套件。高成本的网格碰撞、凸分解、长时间仿真和人工/VLM 评测采用分层子集：

- 简单：单关节、少 link、无复杂耦合；
- 中等：多关节、多候选组件或多级装配；
- 复杂：mimic、闭链、多运动分支、空腔/薄壳或高非凸几何。

每个层级按类别、关节类型、拓扑复杂度和尺寸范围抽样，并固定抽样清单。高成本结果不得只在已通过筛选的发布资产上统计，必须同时报告筛选前失败率。

## 4. Baseline 与消融

### 4.1 Baseline

| 方法 | 说明 | 用途 |
|---|---|---|
| B0 Direct-Agent | 直接给同一模型参考信息，让其生成参数化模板；不提供 SourceMap、TemplateDesign 和模板 Harness | 验证协议价值 |
| B1 Source-backed | 使用来源资产池和 SourceMap，但不使用完整 TemplateDesign/组合域约束 | 分离来源知识的作用 |
| B2 Template-only | 使用 TemplateDesign 和模板 SDK，但去掉来源差异校验 | 分离设计表示的作用 |
| Ours | 完整 arti-skill：来源整理、Design、单文件模板、preflight、random、corner、回归与质量合同 | 主方法 |

所有方法固定模型版本、输入资产、提示词预算、修复预算、运行环境和评测 seed。失败结果保留，不用“换 seed”或人工删除候选来提高通过率。

### 4.2 关键消融

在按复杂度分层的模板子集上进行：

1. 去掉 SourceMap；
2. 去掉 TemplateDesign，仅保留自然语言规格；
3. 去掉候选组合/接口契约；
4. 去掉 preflight；
5. 去掉 random-16/random-36 多 seed 检查；
6. 去掉 corner 检查；
7. 去掉回归保持检查；
8. 去掉自动视觉语义门禁，但保留确定性机械门禁。

主结果不是单独比较“编译能否成功”，而是比较模板完成率、seed 有效率、机械质量、类别语义质量和修复后回归破坏率。

## 5. 实验一：模板构建可靠性

### 5.1 协议

每个方法对相同任务独立运行 3 次，记录：

- 模板是否生成；
- 是否自包含、可导入、可导出 URDF/mesh；
- 是否首次运行成功；
- 修复轮数、编译/检查次数、token、墙钟时间和成本；
- 是否需要人工修改；
- random-16、random-36、corner 是否通过；
- 修复新 seed 后旧 seed 是否回归失败。

### 5.2 指标

\[
R_{template}=\frac{N_{可运行模板}}{N_{输入任务}}
\]

\[
R_{seed}=\frac{N_{通过全部机械检查的 seed}}{N_{实际生成的 seed}}
\]

\[
R_{retention}=1-\frac{N_{修复后重新失败的旧 seed}}{N_{被重新检查的旧 seed}}
\]

同时报告 first-shot success、最终 success、人工介入率、平均修复次数、每模板成本和每通过 seed 成本。

## 6. 实验二：结构、几何与运动质量

### 6.1 硬闸门

对每个 seed 独立记录以下二值结果：

- 模板可执行、artifact 已保存；
- URDF XML、link/joint、parent-child 和 root 合法；
- visual/collision mesh 路径完整；
- 无孤立 part、静止态严重穿模或悬空 joint origin；
- 关节类型、axis、origin、limit 完整；
- 参数和候选组件真实响应；
- 运动采样不穿模；
- 导出包可复制到干净目录后重新加载。

硬闸门的通过率必须基于所有生成 seed 计算；发布集质量只作为第二行结果。

### 6.2 连续质量指标

| 维度 | 指标 | 定义 |
|---|---|---|
| 网格 | watertight/manifold/open edges | 按 link 和按资产分别统计 |
| 网格 | 面数、顶点数、文件大小 | 报告 p50/p90/max，监控复杂度膨胀 |
| 结构 | link/joint 完整率 | 引用存在、树可达、无环 |
| 语义 | named-part precision/recall | 仅在存在冻结语义 gold 时使用 |
| 层级 | assembly tree F1、深度、分支数 | 对 canonicalized link/joint tree 计算 |
| 关节 | joint type accuracy | 与 TemplateDesign/Gold specification 比较 |
| 关节 | axis angular error | \(\arccos(|a\cdot\hat a|)\) |
| 关节 | pivot-to-axis distance | 轴线到目标旋转/滑移部件基准的距离 |
| 关节 | range plausibility | 实际可达行程/声明行程，及限位合理性 |
| 运动 | full-range collision-free rate | 沿每个关节全行程连续细分检查 |
| 运动 | min-clearance curve | 记录行程上的最小间隙、均值和低于阈值比例 |
| 运动 | AOR | 互穿体积/相关部件体积，报告 mean、max 和零碰撞比例 |
| 约束 | constraint coverage/satisfaction | 组件数量、尺寸、接口、层级和运动约束分别统计 |

精确网格、凸包和凸分解必须分别评测，因为精确网格不碰撞不等价于仿真器中的凸近似不碰撞。

## 7. 实验三：碰撞表示与仿真就绪性

### 7.1 碰撞表示对照

对同一批资产比较：

1. exact visual mesh；
2. per-part convex hull；
3. convex decomposition 或简化 collision mesh。

记录几何保真差：碰撞体与视觉体积比、表面距离、碰撞几何面数、导入时间、静止/运动 AOR。

### 7.2 多仿真器协议

对分层子集在 MuJoCo、PyBullet、SAPIEN 和 Genesis 中执行三级检查：

1. URDF/MJCF/USD 解析与导入；
2. 创建场景并完成第一步仿真；
3. 运行固定时长的重力/驱动 rollout。

每个仿真器报告 load success、首步 success、NaN/爆炸率、最大穿透、关节限位违例、静止漂移和运行时间。不存在某仿真器时报告 `unavailable`，不能记为通过或失败。

### 7.3 物理属性

只有存在可靠质量/材料标注的资产才计算密度、质量、COM 和惯量误差。所有资产都可报告字段完备率：

- mass/inertia 完备率；
- inertia 正定率和三角不等式合法率；
- joint damping/friction 完备率；
- collision geometry 完备率。

重力测试在 rest pose、关节边界和最坏展开状态分别执行，报告姿态漂移、穿地深度、能量残差和是否倒伏。

## 8. 实验四：多样性、可控性与可复现性

### 8.1 模板独有的生成域指标

每个模板报告：

- raw 组合数与去除不可达/不合法组合后的 effective 组合数；
- 槽位数、候选数、连续参数维度；
- link/joint 数、运动链深度和关节类型分布；
- 拓扑家族数和 topology entropy；
- 只有尺寸变化、组件变化和拓扑变化的 seed 比例；
- 来源可追溯率和参数响应率。

### 8.2 跨 seed 多样性

对规范化运动学图计算图编辑距离、关节数/深度分布、结构签名熵和跨 seed 非重复率。对几何进行尺度归一化后计算点云 Chamfer 距离或体素 IoU，报告：

- exact duplicate rate；
- geometric near-duplicate rate；
- topology duplicate rate；
- 同模板内与跨模板的距离分布。

只有存在独立真实参考分布时，才使用 MMD、COV、1-NNA；没有 reference 时不把这些指标伪装成“真实度”。

### 8.3 条件可控性

预先指定目标属性，例如总高、底座宽度、可动件数量、旋转角或抽屉行程。生成后从 mesh/URDF/运动状态重新测量，不直接读取输入参数，计算：

- target hit rate；
- normalized MAE；
- monotonic response rate；
- parameter no-effect rate；
- 非目标属性保持率。

### 8.4 确定性

随机抽取 template-seed 对，重复编译至少 5 次，比较：

- URDF canonical hash；
- mesh hash；
- link/joint/参数摘要；
- 报告内容和失败状态。

报告 bit-identical rate、semantic-identical rate 和任何非确定性来源。

## 9. 实验五：视觉与语义质量

视觉指标只作为结构/机械指标的补充，不作为主结论。

固定相机、光照、背景、尺度、视角和运动帧，评估：

- 类别身份与关键部件存在性；
- 组件替换是否仍保持类别身份；
- 可动件与运动方向是否符合语义；
- 几何真实度、机构合理性和视觉接受度。

使用 CLIP/BLIP 或 VLM 时固定模型版本和 prompt，并报告与人工小样本的一致性。人工评测至少采用盲评、多 seed 配对和 inter-rater agreement。不能仅报告通过视觉门禁后的资产接受率。

## 10. 可选实验：下游数据价值

只有项目中存在可复现的下游训练/评测任务时开展，且测试集必须按模板或组件隔离，避免 seed 泄漏。

建议比较：

- 原始数据；
- 相同数量的单资产扩增；
- 相同数量的 arti-skill 模板数据；
- 相同数量但低多样性的模板数据；
- 完整模板数据。

在独立测试集上报告 rest-state 与 articulated-state 的 part/asset IoU、距离误差，以及未见模板、未见组件和未见组合的泛化性能。主实验不应只在自己生成的 seed 上测试。

## 11. 执行顺序

### 阶段 A：协议与基线冻结

1. 生成模板、类别、SourceMap、Design、seed 和仿真器 manifest；
2. 固定代码 commit、SDK、Python、仿真器和模型版本；
3. 确定 36-seed 与 corner 清单；
4. 固定 B0/B1/B2/Ours 的预算和输出格式。

### 阶段 B：全库轻量质量

1. 运行 `preflight`、`random-16`、`random-36`、`corner`；
2. 汇总执行、导出、结构、静止碰撞和运动采样；
3. 输出全库 seed-level manifest 和 failure taxonomy；
4. 不覆盖原始资产，所有结果写入独立 run 目录。

### 阶段 C：方法消融

对按复杂度分层的模板子集运行 B0/B1/B2/Ours，完成可靠性、成本和回归保持率比较。

### 阶段 D：高成本质量

在固定分层子集上完成 exact/convex collision、全行程 motion sweep、物理完备性和多仿真器检查。

### 阶段 E：多样性与控制

计算组合域、拓扑图、几何近重复、参数命中率和确定性指标，并单独报告模板级均值和库级分布。

### 阶段 F：语义/下游验证

完成视觉语义盲评；若下游任务可用，再进行模板/组件隔离的训练与测试。

## 12. 输出目录与报告

建议每次实验使用不可变 `run_id`：

```text
reports/arti_skill_eval/<run_id>/
├── experiment_manifest.jsonl
├── environment.json
├── method_manifest.json
├── seed_manifest.jsonl
├── template_results.jsonl
├── seed_results.jsonl
├── motion_results.jsonl
├── simulator_results.jsonl
├── diversity_results.jsonl
├── controllability_results.jsonl
├── failure_taxonomy.jsonl
├── summary.json
└── summary.md
```

每条 seed 记录至少包含：模板 hash、seed、配置摘要、输入/输出路径、阶段状态、失败原因、修复次数、URDF/mesh hash、link/joint 摘要、运动检查摘要和仿真器状态。

## 13. 论文主表

| 表格 | 内容 |
|---|---|
| Table 1 | 类别、模板、槽位、候选、参数维度、组合域与资产规模 |
| Table 2 | B0/B1/B2/Ours 的模板成功、seed 通过、修复、成本和回归保持 |
| Table 3 | 结构、网格、关节、静止/运动碰撞和 AOR |
| Table 4 | exact/convex collision 与多仿真器导入、首步和 rollout |
| Table 5 | 拓扑/几何多样性、可控性和确定性 |
| Table 6 | 视觉语义与可选下游结果 |

建议图：seed 通过率随验证阶段变化、QC 前后 AOR、碰撞表示的保真—速度权衡、模板组合域与拓扑多样性、目标属性误差分布。

## 14. 结果解释边界

本项目不应仅凭编译成功声称“资产真实”；不应仅凭已筛选发布集的 AOR=0 声称“生成没有碰撞”；不应在没有配对参考资产时使用 PSNR/Chamfer 作为重建精度；不应把 CLIP/BLIP 分数当成机械正确性；不应把 MMD/COV/1-NNA 用于没有参考分布的模板集合。

最小可投稿实验包是：

1. 完整模板管线消融；
2. 全库 36-seed + corner 机械验证；
3. 运动碰撞与碰撞表示对照；
4. 模板独有的组合、多样性、可控性和确定性；
5. 分层多仿真器验证；
6. 有独立测试协议时再加入下游学习实验。

