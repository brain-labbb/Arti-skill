# Arti-Skill 管线消融实验设计

> 状态：预注册草案，仅包含设计，不包含新的实验结果。**P1 正式运行当前为 `BLOCKED`，必须先完成第 4、12、14 节的 P0 go/no-go 条件。**
> 冻结日期：2026-08-12。
> 目标管线：`SourceMap -> TemplateDesign -> 单文件模板 -> preflight -> random-16 -> random-36 -> corner`。
> 本文中的“论文结果”与“建议实验”严格分开；建议实验在完成 manifest、代码和 evaluator 冻结前不能开始。

> **管线范围补正（2026-08-12）**：论文系统的完整范围从参考图像开始，还包括
> Articraft origin、六轴单变量变体、source-pool 验收、槽位重因子化和自动视觉发布门禁。
> 上面的目标管线是当前首先执行的 template-authoring core，不是整篇论文系统的全部阶段。
> 当前提供的 `_ICLR_2027__PV_A.pdf` 被有损 UTF-8 重编码，正文流不可恢复；在取得无损
> PDF 前，完整阶段名来自 Git 中的管线论文底稿，阶段行为则以当前 `.agents` 和代码为准。

## 0. 执行结论

Arti-Skill 的论文级消融不应只做一个 “Full vs no pipeline”。推荐拆成四个问题：

1. **作者阶段的信息可用性效应**：用规范化的 `S_factor x D_factor` 2x2 因子实验，回答显式 source provenance/navigation 信息、显式 design decision 信息及其交互分别带来什么。该实验不等价于 SourceMap/TemplateDesign 制作流程的总效应。
2. **验证阶段的检出贡献**：在同一批冻结产物上依次回放 preflight、random-16、random-36、corner，测增量缺陷检出率、成本和独立评测下的 false-accept rate。
3. **反馈与修复贡献**：从完全相同的 first-shot 快照分叉；用一轮冻结事件比较 raw logs 与同预算 structured diagnostic package，再将 source-review routing、quality contract 和三轮迭代视为不同 repair policies。
4. **字段簇信息与鲁棒性**：借鉴 CAGE 隔离单一变化的思想，屏蔽一个 TemplateDesign 信息字段簇；再像 UniPhysGen 一样加入旋转、part granularity、稀有组合和 source-pool 扰动压力测试。

核心确认性实验推荐 **24 个全新类别 x 4 个作者信息组 x 3 次独立重复 = 288 个 authoring runs**。类别而不是 seed 才是主要统计单元；3 次模型重复不能把有效类别 `N` 从 24 变成 72。若预算只能支持 12 类/144 runs，该轮必须标为 pilot，interaction 和复杂度切片只作 exploratory。

现有 `exp/runtime/t2_formal_v1` 已经包含 preparation、authoring records 和 summary。因此其中已经准备、生成或人工查看过的类别只能进入 development cohort，不能继续标记为本设计的 fresh held-out cohort。

### 0.1 完整论文管线与本轮开跑范围

完整系统按职责分为四段：

```text
参考图像 / JobSpec
  -> Articraft origin 生成与原生机械 QC
  -> 六轴规划、单变量 fork、parent-child diff、结构/视觉验收
  -> SourcePoolReport、槽位重因子化、严格 SourceMap、TemplateDesign
  -> 单文件模板、preflight、random-16、random-36、corner、repair
  -> coverage-driven 自动视觉发布门禁、seed export、evidence bundle
```

因此实验分成三层，不能用其中一层代替另外两层：

| 层 | 比较对象 | 回答的问题 | 当前状态 |
|---|---|---|---|
| W：whole-system | Direct same-model vs 完整端到端系统 | 整个 package 是否有效 | 需在所有阶段接通后运行；只能解释 bundle effect |
| U：upstream source vocabulary | origin、六轴规划、fork 和 source-pool gates | 来源词汇怎样影响真实结构候选、类别保持和下游可蒸馏性 | 需冻结图像、fork 数量、attempt budget 和独立 candidate gold |
| T：template-authoring core | SourceMap、TemplateDesign、模板、gate、repair | 作者信息、验证覆盖和修复机制的贡献 | 本轮 P0/P1 首先执行 |

本轮“开始测试”指先完成 T 层的零正式调用 P0。W/U 层尚未接通的 VLM evaluator、自动
orchestrator 或独立 gold 一律记 `N/R`/`BLOCKED`，不能用已有发布资产或论文数字填表。

### 0.2 完整管线的消融矩阵

完整论文实验除后文 T 层实验外，增加以下互不替代的实验：

| ID | 冻结输入与对照 | 主 endpoint | 解释边界 |
|---|---|---|---|
| W1 package effect | 同一参考图、同一底模/预算：Direct-to-template vs Full | intent-level 完成率、hidden all-pass、成本 | 类似 Articraft/Nova3D，只证明整体系统价值，不归因单组件 |
| U1 six-axis planning | 同一 origin、相同 fork intent/attempt 数：通用“增加变化” prompt vs 六轴 variant card | 独立判定的结构候选 yield、material/scale-only 率、类别保持 | 不以最终被 gate 接受的子集作分母，失败 fork 保留 |
| U2 source-pool gates | 同一批冻结 fork 逐级离线回放 mechanical、parent-diff/fingerprint、visual gate | first-rejected candidates、false-accept proportion、边际成本 | gate 只筛查同一批产物；不能把不同生成集的差异称 gate 因果效应 |
| U3 source vocabulary | origin-only pool vs 完整 accepted pool，之后使用同一蒸馏与模板预算 | source-backed candidate recall、core domain、hidden pair validity | 这是 source-pool package effect，不把收益分摊给某个 fork 或 gate |
| U4 slot refactorization | 同一上游 pool：未闭合的自然槽位 proposal vs dependency-closed refactorized design | frozen cross-product reachability、interface/host-adaptation violations | 两组仍须输出同一无 compatibility-gate 的正式域；不可通过删组合获胜 |
| V1 visual release gate | 同一批机械通过的模板/seed，离线比较无视觉 gate vs coverage-driven multi-view/motion gate | 对 blind human/VLM-independent gold 的 FAP、FRP、selective risk | 只评价筛查；若 gate 后再 repair，另立 repair 实验 |
| O1 orchestration | 注入中断、partial artifact、stale hash、provider retry、重复启动 | 正确 resume、幂等、重复调用、证据完整性 | 系统可靠性测试，不宣称几何/语义质量提升 |

U1、U2 和 V1 均固定 **intent** 分母。被拒绝、超时、生成失败和 evaluator 不可判定不得从
分母消失。视觉 gate 的阈值只用 development 集校准；正式测试集只运行一次并封存结果。

## 1. 本地管线及其可检验贡献

本设计以 [build-template skill](../.agents/skills/build-template/SKILL.md)、[现有 T2 设计](T2_redesign_v2.md)、[T2 pilot 协议](t2_authoring_pilot/README.md) 和当前脚本为实现依据。

| 阶段 | 正式职责 | 研究问题/estimand | 不能用什么替代 |
|---|---|---|---|
| SourceMap | 完整枚举 active source pool；记录 `use/reference/duplicate/reject`、revision、source span 和结构/运动证据 | 是否提高 source coverage、来源忠实度、候选发现率，并减少误用旧 revision | 文件名搜索命中率或模型自称“读过源码” |
| TemplateDesign | 定义 slots/candidates、独立参数、derived DAG、interfaces、multiplicity、bindings、category anchors、surface relations | 是否提高组合域、跨组件一致性、类别语义和运动接口正确性 | 最终代码行数或 named-link 数量 |
| 单文件模板 | 唯一 runtime truth；普通 function-style、可重建、无 source runtime 依赖 | 是否生成可复用 generator，而非单一漂亮实例 | 默认 Config 的一次成功 |
| preflight | 不建几何地检查 contract、schema、domain resolution 和 coverage | 是否早期发现 domain/声明问题，成本是否低于几何 build | compile success |
| random-16 | seeds 0-15；累计验证；至少 15/16，任何候选零通过均阻断 | 是否以较低成本捕获高频实现错误 | 只报总通过数、不报 candidate denominator |
| random-36 | seeds 0-35；至少 33/36；复用 0-15 cache | 新增检出率、escaped defects 和边际成本 | 把 0-15 再算一遍当新增证据 |
| corner | 自动补缺失 candidate、N min/max，并执行作者声明的风险姿态/数值边界；零容忍 | 是否捕获随机抽样难以命中的边界错误 | 将 authored numeric corners 误写成自动穷举全部数值边界 |
| repair loop | 聚合 seeds/entities/configs/nearest pass/owner lines；必要时路由回 source review；冻结 quality digest | structured feedback、source re-check、反 domain-shrink 约束各自是否有效 | 用不同 evaluator、不同 repair budget 的粗糙对照 |

生产 invariants 在所有实验组中保持：

- 所有组都必须输出同一个正式模板接口，包括 `TEMPLATE_DOMAIN`、`TEMPLATE_CORNERS`、`build_object_model()` 及正式 harness 需要的其他字段。
- 缺少 SourceMap 或 TemplateDesign 表示“不给该信息”，不表示允许回退到 legacy schema、单实例脚本或较弱 QC。
- 所有声明的核心 slot 组合都必须可构建；不得增加 compatibility gate 来隐藏非法组合。
- 对正式流程 hard boundary 的移除只发生在隔离实验副本或离线 gate replay 中，不修改生产 workflow。

## 2. 七篇论文实际怎样做消融

### 2.1 方法审计总表

| 论文 | 实际对照 | 主要数字 | 因果证据强度 | 对 Arti-Skill 的直接启发 |
|---|---|---|---|---|
| [LAM](https://openaccess.thecvf.com/content/CVPR2026/papers/Gao_LAM_Language_Articulated_Object_Modelers_CVPR_2026_paper.pdf) | Sec. 4.2, Tables 4-6：Geometry checker x articulation checker；1 vs 3 次迭代；checker 输入模态；角色模型 | 无 checker 50.6，geometry-only 61.4，articulation-only 56.6，两者 1 轮 66.3，两者 3 轮 75.9；2D single-view 60.2，2D multi-view 65.1，2D+motion 71.1，3D-only+multi-view+motion 62.7，2D+3D 75.9 | 1 轮表接近 2x2，但 3 轮只测双 checker，不能估计完整 checker x iteration 交互；每类仅 1 个样本、无 seed/CI；正文称 83 类，但 `46+27=73`，精确 manifest 未披露 | 将 semantic/geometry feedback、motion feedback 和迭代次数分开；补足重复、CI 和冻结 manifest |
| [CAGE](https://openaccess.thecvf.com/content/CVPR2024/papers/Liu_CAGE_Controllable_Articulation_GEneration_CVPR_2024_paper.pdf) | Fig. 9, Table 3：从所有 blocks 中一次只删除 LA、GA 或 GRA 并重训 | Full 的 MMD-AID/MMD-ID/AOR 为 .816/.049/.008；no-LA .840/.053/.016；no-GA .831/.052/.011；no-GRA .876/.157/.013 | 七篇中最干净的单网络模块消融之一；所有变体统一报告三项指标，并结合 Fig. 9 分析定性 failure mode；仍无多训练 seed/CI | 借鉴隔离单一变化的思想；Arti-Skill 的字段屏蔽只能解释显式信息 availability，不能等同于删除 runtime 模块 |
| [Artiverse](https://arxiv.org/html/2605.24403) | Sec. 4.3：半自动 proposal + 人工 correction 与全人工时间对照；Table 3 另做训练数据源替换 | 每类别抽 5 个代表物体；segmentation correction 1.5 min、节省 32.0%；motion correction 1.3 min、节省 33.5%；50.12% parts 无需调整；expert pass 另需 0.8 min | 没有逐 pipeline 模块质量消融；样本总量、配对和 CI 未披露；数据源实验还混杂规模 | 单独做 paired efficiency study，所有组使用相同 expert final gate；不能把省时当质量提升 |
| [Articraft](https://arxiv.org/html/2605.15187) | Fig. 6：同底座 GPT-5.5 的 generic Codex 与完整 SDK+harness bundle；Fig. 7 是单 prompt 模型/effort 展示 | 46 类 x 5 prompts；125 人、5000 comparisons；Full GPT-5.5 的 Top-1/2/3 为 42%/28%/14% | 是系统联合处理，不可归因到 SDK、tests、structured feedback 或 repair 中任一组件 | 先保留 same-model raw baseline 证明整体价值，再用真正 OAT 拆组件；独立评测不能只靠 authored tests |
| [UniPhysGen](https://arxiv.org/html/2607.13586) | Fig. 2, Tables 3-7：sampled assets 上 verification 前后人工评估（是否严格配对未披露）；rotation/axis representation；global PE；part granularity；image modality；model size | Separate PE -> unified global PE：joint acc 83.26 -> 89.96，pivot err .319 -> .099；challenging rotation 下 canonical joint-type acc 50.09，SO(3) Cartesian 80.53，但 axis error 同条件为 13.19 -> 30.61 degree；merged parts 指标基本持平 | PE 是较干净 OAT；rotation 和 granularity 是鲁棒性；verification 图缺 N/配对细节/CI；image/scaling 有控制和表述问题 | joint type/axis/pivot/limit 分开；加入 orientation/decomposition stress；同批产物做可核验的 before/after pairing |
| [PhysX-Omni](https://arxiv.org/html/2605.21572) | Sec. 4.7, Fig. 10, Tables 1-2：PhysX-Anything 整体系统 vs PhysX-Omni | 作者归因于 representation；我们的审计发现同时改变 voxel representation、post-hoc segmentation，且论文未报告 baseline 是否按 Omni 的 42K 数据和同预算重训 | 多因素混杂，不能称单组件消融；部分 kinematic 指标方向定义还不一致 | structured intermediate、assembly 和数据变化必须拆开，不能把 bundle 差异分摊给某个模块 |
| [Nova3D](https://arxiv.org/html/2607.22738) | Sec. 3.3, Table 4：同 base model/route，完整 system package vs naive；Table 18 是同对象 base-cage vs compiled-GLB UV 配对 | 54 个 intent：Full 54/54 executable，naive 31/54；Full first-shot 44/54，8 个修 1 次，2 个修 2 次；作者明确不把增益归因到单个 repair/checker | 系统级对照清楚，组件归因不成立；UV 表示对照才是严格配对消融 | 同时报 intent/generated/strict/evaluable 分母；first-shot 与 post-repair 分开；再做组件级分叉 |

### 2.2 从论文中保留、但需要改进的实验模式

1. **LAM 模式**：两个 checker 因子（Geometry on/off x Articulation on/off）加不完整的 iteration curve。保留“反馈通道”和“反馈轮数”分离的思想，但改为同一 first-shot 快照分叉，并至少 3 次模型重复。
2. **CAGE 模式**：一次只删除一个真实网络模块，统一报告三项指标并结合模块职责解释定性 failure mode。Arti-Skill 借鉴其隔离思想，并额外预注册字段簇敏感 endpoints，但不把字段屏蔽称为 runtime 模块消融。
3. **Artiverse 模式**：质量门槛一致时比较人工时间。需要补 task-level pairing、annotator effect、CI 和 expert rejection。
4. **Articraft/Nova3D 模式**：Full vs same-model naive 只能证明 package value，不能证明组件贡献。它适合 sanity check，不应成为唯一消融表。
5. **UniPhysGen 模式**：canonical 测试之外加入 orientation 和 decomposition stress；参数误差必须在明确的 GT movable denominator 上统计。
6. **PhysX-Omni 反例**：表示、分割、数据和预算同时变化时，任何单模块归因都无效。

## 3. 预注册问题和假设

| ID | 问题 | 预注册方向性假设 | 主 endpoint |
|---|---|---|---|
| H1 | 显式 `S_factor` 是否有 availability effect？ | 提高独立 gold 下的 source provenance/fidelity；复杂 source pool 上效应更大 | first-shot source fidelity；first-shot hidden config yield |
| H2 | 显式 `D_factor` 是否有 availability effect？ | 提高 valid domain coverage、pairwise validity、category-role recall 和 motion semantics | first-shot hidden pairwise valid yield；FamilyGold role recall |
| H3 | 两类信息是否有交互？ | source provenance/navigation 与 design decisions 联合时可能出现 interaction；方向作为待检验，不预先宣称成立 | `S_factor x D_factor` interaction 及 CI；12 类时 exploratory |
| H4 | 分阶段 gate 是否提供增量筛查覆盖？ | random-36 在 random-16 后仍有新增；corner 对 rare candidate/N boundary 有非零新增 | incremental defect discovery；accepted-set false-accept proportion |
| H5 | structured diagnostic package 是否优于 raw logs？ | 同一 round-0 事件、同一 input/output budget 和一轮 repair 下，聚合/关联/owner/source-review 等派生诊断提高 hidden quality，并减少无效编辑 | first-shot-failure conditional success；hidden config yield；regression count |
| H6 | source-review routing 是否必要？ | 对 axis/origin/topology/interface 错误尤其有效 | joint axis/pivot、interface violation、source fidelity |
| H7 | quality contract 是否防止“修复性缩域”？ | 去掉 digest enforcement 会提高表面 gate pass，但降低冻结目标域上的 coverage | declared vs frozen-domain gap；Gate FAP |
| H8 | canonical pass 能否迁移到 stress cases？ | Full 在旋转、alternate granularity 和稀有边界下退化较小，但不预设效应量 | stress delta 与 paired CI |

H3 是论文启发的研究假设，不是现有证据结论；若交互 CI 跨零，应如实报告。

## 4. 数据集、冻结和泄漏控制

### 4.1 三个互斥 cohort

| Cohort | 用途 | 最低规模 | 能否用于论文主结论 |
|---|---|---:|---|
| D：development reconstruction | 脚本调试、prompt 格式、evaluator 自检、功效模拟；可用现有 12 类 pilot 和 `t2_formal_v1` | 现有数据 | 否 |
| F：fresh confirmatory | E1 主 2x2 和主要结果 | 24 类，simple/medium/complex 各 8；12 类只能称 pilot | 是 |
| S：stress | 高风险 surface relation、mixed joints、multiplicity、缺失内腔/退化 source 等 | 至少 6 类 | 只用于预注册的补充鲁棒性结论 |

F 类别纳入条件：

- 冻结时不存在目标 `model.py`、SourceMap、TemplateDesign、authoring output 或人工评分。
- 类别及 prompt 未进入现有 pilot/formal/debug 日志；只要生成过、人工查看过或据此改过 prompt/evaluator，就降级到 D。
- active raw source pool 在冻结前可访问并 hash；目标模板不得存在于 authoring model 可读路径。
- 复杂度 strata 在 authoring 前由 source-pool facts 冻结，不按最后结果重新分层。
- 24 类 F 中至少覆盖：8 个 multi-candidate slots、8 个 multiplicity/repeated-sibling 类别、8 个 cross-component bindings、8 个 interface/surface-relation 高风险类别；这些条件可重叠。12 类 pilot 时上述最低数均为 4。

### 4.2 冻结顺序

1. 冻结 task intent、复杂度 strata、raw source record 列表、revision、文件 hash 和排除理由。
2. 由不参与最终 authoring 的 preparation 流程生成并复核 SourceMap 与 TemplateDesign。
3. 将 native artifacts 机械分解为下节定义的 `S_factor` 和 `D_factor`；运行信息等价性及 leakage self-check。candidate aliases/slots 仍可能承载 preparation 阶段的候选发现结果，所以 contrasts 只命名为 factor availability effects。
4. 冻结所有 arm prompts、模型精确 ID、reasoning effort、temperature/seed（若 provider 支持）、turn/token/wall/cost caps、SDK 版本和 commit。
5. 冻结 development gate 代码和 authoring 可见 cases。
6. 独立冻结 hidden evaluator、FamilyGold、joint gold、external canonical target schema/config adapter、secret salted case manifest、covering-array 规则及其 hash；这些内容不挂载到 authoring/repair 容器。实验后用 commit-reveal 公布 salt、manifest 和预先承诺的 hash。
7. 生成随机化 job order 后才开始运行。失败、超时和 provider error 均保留在 intent manifest 中。

### 4.3 两套 evaluator，防止 repair 污染测试集

| Evaluator | 可见性 | Cases | 用途 |
|---|---|---|---|
| Development gate | author/repair 可见反馈 | preflight；冻结的 public development configs；作者 corners | 生产式迭代和 repair |
| Hidden validation | 永不反馈 | secret salted external configs；独立 pairwise covering array；冻结 numeric/multiplicity boundaries；独立 motion/CCD；FamilyGold 与 joint gold | 最终比较和 escaped-defect 估计 |

现有 `evaluate_t2_generated_template.py` 调用被测模板自己的 `config_from_seed()`、`TEMPLATE_CORNERS` 和 authored tests，**不能**充当该 hidden evaluator。P0 必须先实现 evaluator-owned canonical cases/config adapter、blind gold mapping 和独立 collision prerequisites。hidden cases 只能在各 run 的 final hash 和所有 repair traces 封存后执行；不得因为 hidden failure 再修模型，也不得把 hidden failures 加回当前论文实验的 corners。

## 5. E1：S-factor x D-factor 作者信息实验

### 5.1 为什么需要 artifact normalization

native TemplateDesign 可能包含 source record id、revision、source span 或证据摘录。若把 native TemplateDesign 直接给 `without SourceMap` 组，SourceMap 信息会从 Design 泄漏，2x2 主效应无法解释。

因此在不丢失 Full 信息的前提下，将准备材料机械拆成两份**独立 authoring-packet schema**：

- `S_factor`：完整 source inventory、record/revision/span/evidence、`use/reference/duplicate/reject`、稳定 candidate alias 到 source record 的映射，以及 source-to-implementation owner/function mapping。
- `D_factor`：slots/candidate aliases、independent parameters、derived DAG、interfaces、multiplicity、bindings、category anchors、surface relations；删除路径、revision、line/span、evidence quote、source hash、`implementation_function` 和其他 owner/navigation 字段。
- `S_factor + D_factor` 的联合内容必须能通过 self-check 重建 native Full authoring packet 的全部信息；`D_factor` 单独不得暴露 source navigation。

`D_factor` 不是合法的 production `TemplateDesign`，不写入正式 `designs/`，也不交给 production loader；当前 schema 要求顶层 `source_map_path/source_map_sha256`，强行留空会 validation fail。实现必须支持 evaluator-side native Design 和 author-visible redacted packet 的双视图。

该实验估计的是**authoring 时显式信息 artifact 的可用性效应**，不是 SourceMap/TemplateDesign 制作过程本身的因果效应。由于 `D_factor` 的 slots/candidate aliases 仍包含 preparation 发现的候选，`S_factor` contrast 更准确地表示 provenance/navigation/evidence 的增量可用性，而不是 SourceMap 候选发现流程的总贡献。

### 5.2 四个组

| Arm | Raw source pool | `S_factor` | `D_factor` | 输出 contract | 解释 |
|---|---:|---:|---:|---|---|
| A00 Raw-only | 有 | 无 | 无 | 完整正式 contract | same-model naive/system baseline；允许模型自行分析，但不给预制 artifact |
| A10 Source-only | 有 | 有 | 无 | 完整正式 contract | S-factor availability |
| A01 Design-only | 有 | 无 | 有 | 完整正式 contract | D-factor availability；无 source provenance/navigation |
| A11 Full | 有 | 有 | 有 | 完整正式 contract | 正式信息联合组 |

所有组共同拥有相同 task prompt、raw sources、SDK/harness 文档、只读 examples、工具能力、authoring token/turn/wall cap 和封存后 evaluator。不能让 A00 输出较弱 schema。作者阶段只允许相同的 contract-only smoke checks；native Full Design/gold 不挂载，也不通过 findings 反向泄漏。

### 5.3 执行与快照

- 24 个 F 类别 x 4 arms x 3 固定重复，共 288 个 authoring runs；12 类/144 runs 只能作为 pilot。
- 每个 run 保存 `first_shot.py`、`first_shot.sha256` 和所有 tool/event logs；hash 封存后才运行 external strict/hidden evaluation。
- E1 的确认性 endpoint 固定为 **first-shot**。production repair 对 source/design prerequisites 的要求在四组间不兼容，因此 post-repair 不属于 E1 的公平对照。
- 一个 arm 一个 allowlist container/worker 和独立 output root；只挂载该 arm 的 inputs、SDK 文档和 writable run root。cache、临时目录和 provider transcript 不跨 arm 写共享状态。
- job order 用冻结 hash 随机化；同一 task/repeat 的四组构成 paired block。

### 5.4 主效应计算

对任一 task-macro endpoint `Y`，预注册以下 contrasts：

```text
S-factor availability = mean(A10, A11) - mean(A00, A01)
D-factor availability = mean(A01, A11) - mean(A00, A10)
Information interaction = (A11 - A10) - (A01 - A00)
Authoring packet bundle  = A11 - A00
```

二元结果使用 paired task-cluster bootstrap 的百分点差和 mixed-effects logistic sensitivity analysis；连续结果使用 paired task-level difference。不要把 `A11-A00` 再人为分摊给各阶段。

## 6. E2：TemplateDesign 信息字段簇消融

E2 复用 A11 的 Full first-shot 组，每次只屏蔽一个语义字段簇，其余输入、模型、预算和封存后 evaluator 完全相同。屏蔽发生在 authoring packet，不改变 evaluator-only native Design、冻结目标域和 hidden gold，也不将 full-gold findings 反馈给作者。estimand 是**显式字段簇信息的 availability effect**，不是 runtime 模块贡献。

| Variant | 唯一被屏蔽的模块 | Eligible task | 敏感 endpoint |
|---|---|---|---|
| D-Full | 无 | 全部 | 全部 |
| D-NoDependency | `bindings` 与 cross-slot derived edges | 至少一个 cross-slot dependency | hidden pairwise valid yield；invalid-combination rate；contact/overlap |
| D-NoInterface | `interfaces` 与直接配套的 `surface_relation` | 至少一个运动/装配 interface | joint type；axis angle；pivot distance；full-range collision；containment/protrusion |
| D-NoAnchor | `category_anchors` | 有独立 FamilyGold 的类别 | role recall；category precision；functional completeness |

这里把 bindings+cross-derived、interfaces+surface_relation 分成两个信息字段簇，是为减少同一事实从相邻字段直接泄漏。每个 redaction 必须是 dependency-closed、schema-valid 的**author packet**，而不是修改 production Design：例如删除 interface 时一并清理指向该 interface 的 author-packet endpoint/rephrasing，删除 anchor 时一并清理依赖 anchor role/check 的 surface-relation view。evaluator-only native Design 始终保留 full gold，但其 design-alignment findings 永不反馈给 E2 作者。normalizer 必须运行 schema、dependency closure 和字段重述 leakage scanner，并在 manifest 中列出所有被移除/级联脱敏路径。

以下 ablation 不做：

- 不删除 slot/candidate/multiplicity 后再按变体自己声明的更小域评分；这会改变目标任务。
- 不把 full combinatorial generator 换成单一默认实例；这不是信息消融，而是交付物降级。
- 不将两个不相关模块同时删除并声称其中一个负责变化。

E2 的最小高风险子集为 8 个 eligible fresh/stress tasks x 3 repeats。D-Full 可复用；所有比较使用 first-shot；每个 variant 只在其 eligible denominator 上与 D-Full 配对，不跨不同 task cohort 拼一个总分。

## 7. E3：验证 gate 与 repair loop

### 7.1 E3a：同产物的累计 gate replay

该实验不重新生成模型。对封存产物按正式规则离线计算五个累计 acceptance policies。它估计的是各 gate 的**筛查 operating characteristics 和边际成本**，不是 gate 反馈导致修复成功的因果效应；final 快照又经过 Full gate 选择，主分析以未选择的 A11 first-shot 为准，final 仅作描述性 sensitivity。

| Gate prefix | 包含内容 | 主要报告 |
|---|---|---|
| G0 | import/compile + default/authored tests | accepted N；Gate FAP |
| G1 | G0 + preflight Tier 0-A/0-B | incremental first rejection；wall time；held-out disagreement |
| G2 | G1 + random-16 正式阈值及 candidate-zero rule | 新增 seed/candidate defects；cache-adjusted cost |
| G3 | G2 + random-36 正式阈值 | 对 G2 的新增检出；边际 20 seeds 成本 |
| G4 | G3 + corner 零容忍 | rare candidate、N min/max、authored risk pose/numeric boundary 新增检出 |

每个 prefix 都与不参与 gate 的 hidden validation 比较：

```text
FAP_g = hidden quality fail 且被 gate g 接受 / 被 gate g 接受
Hidden-pass agreement_g = gate g 接受且 hidden quality pass / hidden quality pass
Hidden-fail rejection_g = gate g 拒绝且 hidden quality fail / hidden quality fail
Incremental first rejection_g = 首次在 gate g 被拒绝的 generators / 到达 gate g 的 intent generators
Mutation recall_g = 被 gate g 捕获的 unique mutation_ids / 注入的 mutation_ids
```

`FAP` 是 accepted-set false-accept proportion，不称作通常定义的 false-accept rate。`Hidden-pass agreement` 和 `Hidden-fail rejection` 只表示相对冻结 held-out reference 的一致性，不叫 false-reject rate、sensitivity 或 specificity，因为 development gate 可能发现 hidden finite cases 未覆盖的真实错误。每项同时给四格原始计数和 task-cluster CI。provider/timeout/缺 artifact 属于 intent/operational failure，不进入 hidden quality confusion matrix；缺独立 gold 的项记 `N/E`，也不被暗中当作 quality fail。另报 `overall intent satisfaction`，其中 operational failure 明确计 fail。

同时分别报告 `strict_ready`（36/36 + corner）和生产允许的 tolerance 状态（33-35/36 + corner）；不能将两者合并成一个 pass。

### 7.2 E3b：从同一 A11 first-shot 分叉的 repair 实验

对每个 A11 first-shot 建立只读基线快照并分叉：

| Repair arm | Round-0 development checks | 反馈 | 最大轮数 | 可作的归因 |
|---|---|---|---:|---|
| R0 No-repair | 相同 | 无 | 0 | repair bundle reference |
| R1-Structured | byte-identical frozen events | grouped findings，包括聚合、Config 关联、nearest pass、owner lines 和 routing policy | 1 | 与 R1-Raw 的 matched-budget diagnostic-package effect |
| R1-Raw | byte-identical frozen events | 按时间顺序 raw logs；等 input token cap并记录 truncation | 1 | reference；不假装与 structured 只有排版差异 |
| R-Full | 相同 deterministic checker/version/config schedule | structured findings + source-review routing + quality digest enforcement | 3 | 完整 repair policy bundle |
| R-NoSourceReview | 同上 | structured findings，但不强制 axis/origin/topology/interface 回查 source | 3 | policy ablation；后续轨迹可不同 |
| R-NoContract | 同上 | structured findings + source review，但不执行 frozen quality digest | 3 | policy ablation；后续轨迹可不同 |

R-Full 保存第 1 轮及 `up-to-3` 的 final 快照，因此 iteration curve 写作 `0 -> 1 -> up-to-3(final)`。一轮已通过时按 production policy 早停，并将第 1 轮结果 carry 到 final；不得为凑第 3 轮继续编辑。只有 R1-Structured vs R1-Raw 是同一冻结事件的一轮 matched-budget diagnostic-package 对照；多轮之后各产物和 checker events 已分叉，只解释为 repair-policy bundle differences。

公平性约束：

- 所有 repair arms 从 byte-identical A11 first-shot 开始，使用同一模型、checker/version/config schedule 和每轮 output token/wall cap；A11 拥有 production source/design prerequisites。
- 一轮 raw/structured 对照的 round-0 event hash 和 input/output budgets 必须相同；structured treatment 明确包含聚合和派生诊断，estimand 不是纯 presentation effect。三轮 policy arms 只保证 round-0 events 相同；后续重新检查各自最新产物，事件自然不同，不复用陈旧 findings。
- R-NoContract 仍按冻结的外部目标域评分。若它通过缩小 `TEMPLATE_DOMAIN` 获得内部通过，记为 hidden failure，而不是“成功修复”。
- 只对 A11 first-shot failure 调用 repair以节省费用。主 repair endpoint 是 first-shot failures 中的条件成功率；另报所有 A11 intent 的 carry-forward satisfaction，避免共同 first-shot pass 稀释 repair effect。

## 8. E4：压力测试

### 8.1 输出及 evaluator 鲁棒性

| Stress | 变换 | 不变量 | 目的 |
|---|---|---|---|
| S-Orient | 对完整生成物及 joint gold 应用冻结的全局 SO(3) 旋转 | joint type、axis 的相对方向、pivot 的归一化位置、limit、collision result | 检查 world-axis 假设；这主要是输出/evaluator 鲁棒性，不等同于作者能力 |
| S-Granularity | 合并固定辅助 visual components，或拆分不改变运动语义的 decoration | movable-link identity、joint graph、limits、functional roles | 检查 part granularity 对评分和导出的敏感性 |
| S-Rare | hidden covering array 强制 candidate pairs、N min/max、risk pose 和 numeric extrema | 冻结目标域 | 检查 random seeds 未覆盖的长尾配置 |

axis error 使用无符号轴线误差 `min(angle(a,b), angle(a,-b))`；pivot error 除以对象 bbox diagonal；只在 GT movable joint denominator 上报告 axis/pivot/limit，另外单独报告 movable detection。

### 8.2 Source-pool 呈现鲁棒性与 fail-fast audit

两类实验必须分开：

1. **呈现不变量**：canonical active manifest、record ids 和 hashes 不变，只随机打乱展示顺序，或在视图层重复呈现同一 record reference（不新增 canonical record）。对 6 个 stress tasks 重跑 A11，检验输出对顺序/重复展示的敏感性。
2. **完整性故障注入**：在冻结 manifest 后让一个 required record 缺失、hash mismatch 或 revision mismatch。正确结果是 preparation/authoring 前 fail-fast，并给出准确 provenance error；不继续生成，也不以“性能保持”评分。

删除或新增 canonical source records 会改变 active source pool，必须重做 SourceMap/Design，已不再是单一 presentation ablation。若研究这一变化，应作为新的 task revision 单独冻结，不能与原条件作因果配对。

## 9. E5：配对效率与人工评审

仿照 Artiverse 的省时实验，但固定最终质量门槛：

| Condition | 过程 |
|---|---|
| H-Manual | expert 从 raw source 直接完成 SourceMap、Design 和模板，经过同一 final gate |
| H-RawAssist | A00 产物 + expert correction + 同一 final gate |
| H-FullAssist | A11 产物 + expert correction + 同一 final gate |

建议 12 tasks x 3 conditions x 每格至少 2 位 experienced annotators，即至少 72 个 sessions。采用平衡不完全区组：每位 annotator 覆盖三种 conditions，但不重复看到同一 task；每个 task x condition 由至少两位不同 annotators 完成。这样 task 可配对，同时可在模型中放入 task 与 annotator 交叉随机效应。最终接受由不知道 condition 的独立 reviewer 判定。

每个 session 冻结最大人工时长；超时结果保留为 right-censored time 并在质量 endpoint 计 fail。必须报告：active human minutes、wall time、model turns、input/output tokens、API cost、build 次数、correction-free rate、expert rejection、最终 hidden all-pass。`proposal correction time`、`SourceMap/Design preparation+review time` 和 `expert final review time` 分开记录，但在 H-FullAssist 的总成本中全部计入。

generator 可复用成本按实际 valid assets 摊销：

```text
authoring_fixed = C_SourceMap + C_Design + C_author + C_repair + C_gate
cost_per_hidden_valid_asset(K) = (authoring_fixed + K * C_seed_export) / hidden_valid_assets(K)
```

hidden research evaluator 的成本另列，不伪装成生产运行成本。

## 10. 评价指标与分母

### 10.1 五个不可混用的状态/分母

| Denominator | 定义 | 失败处理 |
|---|---|---|
| intent-to-run | 冻结 manifest 中计划执行的 task x arm x repeat | provider error、timeout、缺 artifact 仍留在分母 |
| generated | 保存且 hash 可读的模板 | 只用于诊断生成后属性，不代替 intent success |
| operational-success | provider 正常返回、artifact 可读且 evaluator 能执行 | provider error、timeout、缺 artifact 计 operational fail |
| strict-success | 在 operational-success 中通过指定正式 gate 或 hidden quality evaluator | quality fail 与 operational fail 分列；overall intent satisfaction 中两者均 fail |
| independently evaluable | 存在该指标所需独立 gold/mesh/collision/语义证据 | 缺 gold 记 `N/E` 并给原因；不得用邻近论文值或 proxy 填充 |

每张表标题和单元格脚注写清 task、generator、seed/config、joint、constraint 中哪一个是统计单位。

### 10.2 主指标

| 指标 | 定义 | 单位/分母 |
|---|---|---|
| Hidden config valid yield | hidden strict pass configs / frozen hidden intent configs | 每 generator 先算比例，再做 task-macro；不与 role/joint 指标混成一个 case 总分 |
| Hidden all-pass generator rate | 所有 mandatory semantics、structure、motion 和 hidden configs 均通过的 generators / intent generators | task x repeat |
| First-shot artifact success | first-shot 可 import、可保存且满足正式输出 contract | intent runs |
| Gate FAP | internal gate 接受但 hidden quality fail / internal gate 接受 | hidden-evaluable、被内部接受的 generators；同时给四格计数/CI |
| Source provenance/fidelity | 独立 source gold 与输出 role/geometry/motion 的 blind semantic matching | source-gold items；没有自动 provenance 证据时 `N/E`，不从代码外观猜测 |
| Frozen-domain reachability | independent parameters/candidates/multiplicity 能成功解析且被 hidden covering set 实际到达的比例 | frozen domain elements，不按方法自报域 |
| Hidden pairwise valid yield | 严格通过的冻结 candidate-pair cases / 全部 intent pairs | pair cases；按 task macro |
| FamilyGold role recall | 独立 gold roles 中被正确实现的比例 | gold roles；不是 named-link density |
| Category precision | 实现的可见/功能 roles 中属于目标类别的比例 | realized roles |

### 10.3 Articulation 与几何诊断

| 指标 | 定义 |
|---|---|
| Movable detection | 是否存在应有 movable joint/link；独立于参数误差 |
| Joint type accuracy | 先按冻结的 role/topology/geometry matcher 匹配 predicted 与 GT joints；在 GT movable joints 上统计；缺 joint 计 movable miss且 type incorrect |
| Axis angular error | 无符号轴线角误差，degree |
| Pivot error | matched correct-type joint 的 point-to-GT-axis 或冻结定义距离，除以 object bbox diagonal；同时给 matched/type-correct denominator，缺 joint 不被填 proxy |
| Limit overlap | matched correct-type joint 的 range IoU；axis sign 翻转时先按冻结规则变换 range；另报 unsafe over-range/under-range。缺 joint 已在 movable/type 指标处罚，limit 项标 N/E 并保留级联分母 |
| Full-motion collision | 冻结 state sweep/CCD 上无非法 penetration，且应保持的 contact 不丢失 |
| Interface violations | containment、protrusion、gap、overlap、alignment 分项，不合成一个视觉分 |

### 10.4 不可作为主指标的 proxy

- URDF 能加载不代表 joint semantics 正确。
- authored test pass 不代表 independent functional correctness。
- named links、element 数、代码行数只可作为 complexity proxy。
- zero contact 在缺 collision mesh 时不是物理正确证据。
- 默认 Config 的漂亮 render 不代表组合域、corner 或 full motion 正确。
- 现有 internal seed/corner 全通过可以证明 project-native gate 通过，不能替代本设计的 hidden evaluator。

## 11. 统计分析

1. 以 task/category 为 cluster；seed/config 不是独立样本，不能把数千个 build 当作数千个 `N`。
2. E1 以 task-first hierarchical bootstrap 10,000 次给 absolute difference、95% CI；同一 task 内再抽模型 repeats，并报告 task-level raw points。二元 task summaries 另给 exact sign/permutation sensitivity。自然缺陷的“new defect”仅在 blind adjudication 按 `root_cause_id` 去重后作 exploratory；确认性检出使用 first-rejected generator N 和预注册 `mutation_id` recall。
3. 24 类确认性 2x2 sensitivity model 包含 S-factor、D-factor、interaction、complexity stratum；task 设随机截距。12 类 pilot 不以 interaction 或 mixed model 作确认性结论。
4. E2/E3 是同 first-shot 或同 task 的配对比较；二元结果用 paired bootstrap/McNemar sensitivity，连续结果用 paired bootstrap median/mean difference。
5. 对 axis/pivot/limit 先在 joint 内聚合，再到 asset、task，防止多关节物体支配总体。
6. 同一假设族内用 Holm correction；同时给未校正 effect size 和 CI，不只报 p-value。
7. 分 complexity、joint type、multiplicity、surface-relation 做预注册切片；样本不足的切片标 exploratory。
8. 不构造事后加权“总分”。成功率、semantic fidelity、motion、coverage、efficiency 分表报告。
9. 不做看到中间结果后的可选停止；provider-wide outage 可暂停，但冻结 manifest 不删失败记录。

正式开跑前必须用 development 数据做功效/precision simulation，并在不看 F outcome 的前提下冻结类别数。推荐 24 类；12 类只作 pilot，不能据 F cohort 的实际差异追加样本。

## 12. 运行结构、provenance 与自检

建议目录：

```text
exp/runtime/pipeline_ablation_v1/
  manifests/
  preparation/
    source_maps_native/
    designs_native/
    factors/
  authoring/
    A00_raw/
    A10_source/
    A01_design/
    A11_full/
  snapshots/
  gate_replay/
  repair_ablation/
  hidden_eval/
  human_efficiency/
  analysis/
```

每条 record 至少保存：

- `task_id/stratum/arm/repeat/intent_status`
- raw source manifest hash、S/D factor hash、prompt hash
- repo commit、submodule commit、SDK/harness/evaluator hash
- model exact ID、reasoning effort、provider seed/temperature（可用时）
- start/end UTC、wall time、turns、input/output/cached tokens、API cost
- first-shot/final hashes、每轮 findings/event hashes、repair parent hash
- internal gate 状态、hidden status、`N/E` 原因和 artifact paths

开跑前必须通过以下 self-check：

1. `F` 中任一 task 不得在现有 template/design/source-map/runtime/log 路径中命中。
2. 四个 arms 的 raw source manifest、SDK、model、budget 和 output contract hash 相同。
3. A01 的独立 author packet 不含 source path/revision/span/evidence/hash/implementation owner；A11 的 S+D union 与 native authoring packet 信息等价。redacted packets 通过 schema/dependency/leakage tests，但不写入 production `designs/`。
4. 现有 checker 的 `designs/<slug>.json` 存在性会切换 strict/legacy 路径。P0 必须实现 evaluator-only native Design/gold 与 author-visible packet 双视图，并对四组只返回相同 contract-only smoke feedback；否则不得开跑。
5. hidden evaluator 路径、gold、salt 和 cases 对 author/repair process 不可读；只在 first-shot/final hashes 封存后解锁，并在结束后 commit-reveal。
6. 现有 runner 使用 `--dangerously-bypass-approvals-and-sandbox` 且 cwd 可读全仓，只做 prompt/字符串事后审计，因此仅限 D。正式 F 必须使用 allowlist container/read ACL；hidden 路径完全不挂载，并以 canary 文件验证越权读取确实失败。
7. 每组独立 output root；同一 run 不复用别组模型文件、repair state 或 provider transcript。
8. manifest 中的 intent count 等于实际 job count；缺 record 使 run fail，而不是从表中消失。
9. evaluator 先用 seeded faults 做 mutation test：axis/plane、origin offset、limit、collision、domain shrink、missing role、source revision 等故障应被预期 stage/hidden check 捕获；预注册每类 mutation 的最低 recall。axis sign 只有在 joint/range 未作等价变换时才算故障。
10. 冻结跨方法 canonical schema/config adapters、source semantic matcher、多 joint matcher、missing-output penalty 和 collision prerequisites；A00 自创 schema 不能改变目标域或 denominator。
11. analysis 从 immutable records 重建所有表；手工编辑 summary 后 hash self-check 必须失败。

## 13. 建议论文表格

### Main Table A：Authoring information factorial

| Arm | Intent N | First-shot artifact % | Hidden config yield | Hidden all-pass % | Source fidelity | Pair valid % | Role recall | Input tokens | Wall time | Cost |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| A00 Raw-only |  |  |  |  |  |  |  |  |  |  |
| A10 Source-only |  |  |  |  |  |  |  |  |  |  |
| A01 Design-only |  |  |  |  |  |  |  |  |  |  |
| A11 Full |  |  |  |  |  |  |  |  |  |  |

表下另报 S-factor availability、D-factor availability、information interaction 和 A11-A00，均带 task-first hierarchical 95% CI；24 类为确认性，12 类时 interaction 只标 exploratory。

### Main Table B：Validation gate coverage

| Gate | Accepted / intent | First-rejected generators | Mutation recall | Builds | Wall time | Gate FAP | Hidden-pass agreement | Hidden-fail rejection |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| G0 compile/authored |  |  |  |  |  |  |  |  |
| G1 +preflight |  |  |  |  |  |  |  |  |
| G2 +random-16 |  |  |  |  |  |  |  |  |
| G3 +random-36 |  |  |  |  |  |  |  |  |
| G4 +corner |  |  |  |  |  |  |  |  |

### Main Table C：Repair components

| Repair arm | First-shot failures N | Conditional final strict % | Hidden config yield | Hidden all-pass % | Domain shrink | Regressions | Turns | Tokens | Cost |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| R0 No-repair |  |  |  |  |  |  | 0 | 0 | 0 |
| R1-Structured |  |  |  |  |  |  |  |  |  |
| R1-Raw |  |  |  |  |  |  |  |  |  |
| R-Full |  |  |  |  |  |  |  |  |  |
| R-NoSourceReview |  |  |  |  |  |  |  |  |  |
| R-NoContract |  |  |  |  |  |  |  |  |  |

Supplement 中放 Design OAT、stress、per-category raw results、human efficiency 和所有失败清单。任何空缺值使用 `N/A`、`N/E`、`N/R`、`TIMEOUT` 或 `BLOCKED` 及原因，不能拿论文值、邻近实验或 proxy 补齐。

## 14. 实施优先级

| Priority | 工作 | 新模型调用量 | 何时足够 |
|---|---|---:|---|
| P0 | 真正文件隔离、strict/gold 双视图、factor packet schema、hidden evaluator/adapters/mutation tests、fresh manifest 和功效模拟 | 0 formal authoring runs；preparation/gold/evaluator 成本单列 | 第 12 节 11 项 self-check 全通过；否则 P1 保持 `BLOCKED` |
| P1 | E1 24 x 4 x 3 | 288 first-shot authoring runs | 形成确认性作者信息 availability 表；不含 repair |
| P1-pilot | 预算受限时 E1 12 x 4 x 3 | 144 first-shot authoring runs | 仅 pilot；interaction/分层 exploratory |
| P2 | E3a gate replay | 0 新 authoring calls | 形成筛查 operating characteristics/成本/FAP 表，不声称 repair 因果效应 |
| P3 | E3b repair experiment | 最多 5 repair variants x 72 A11 bases；只修 first-shot failures | 一轮 raw/structured 因果对照 + 多轮 policy bundles |
| P4 | E2 field-cluster ablation | 8 high-risk tasks x 3 repeats，Full 可复用 | 给出字段簇信息敏感性解释 |
| P5 | E4/E5 stress 与效率 | 按预注册子集 | 补足鲁棒性和人工成本，不阻塞主表 |

若预算只能支持一个实验，优先完成 P0+P1；无法承担 24 类时明确降为 P1-pilot。只跑 `Full vs Raw` 只能得到 Articraft/Nova3D 式 package effect，无法回答各信息因素的 contribution。

## 15. 主要风险与解释边界

- **preparation leakage**：准备 Full artifacts 的人或 agent 不得产出 target code；SourceMap/Design 只描述证据和决策。
- **Design-only 污染**：必须使用独立 redacted `D_factor` packet；即使如此，S contrast 也只叫 provenance/navigation/evidence availability，不称 SourceMap 全流程主效应。
- **self-authored corner leakage**：正式 hidden boundary set 由 evaluator 根据冻结目标域生成，不采用作者自报 corners 作为唯一 gold。
- **repair overfit**：public development configs 和 authored corners 可反馈；secret salted configs、covering array 和 independent gold 永不反馈。
- **成功后分母偏差**：结构、命名、joint 表不能只在 generated assets 上比较而忽略缺失 artifact；必须同时给 intent satisfaction。
- **物理证据过度解释**：loadable URDF、metadata 存在、零 contacts 都不是物理/语义正确的充分条件。
- **freshness 误标**：现有 `t2_formal_v1` 六类及所有已准备类别属于 D；新 F manifest 必须重新选类并通过路径/log 扫描。
- **模型版本漂移**：同一论文表不得混用模型 alias 指向的不同 backend；记录精确 model snapshot 或 provider revision。
- **成本公平性**：Full 的额外输入 token 是方法成本的一部分，应记录而不是偷偷从总成本剔除；但四组 output/repair caps 必须一致。
- **结果外推**：24 个类别仍只支持所采 strata；不外推到全 500+ generator pool，除非另做预注册随机抽样。12 类只作 pilot。

这套设计把系统整体价值、作者信息、语义决策、验证覆盖、repair mechanism、鲁棒性和效率分开。每张表对应一个明确 estimand，因而不会重复 PhysX-Omni 的多因素混杂，也不会把 Articraft/Nova3D 的 bundle gain 误写成单组件贡献。
