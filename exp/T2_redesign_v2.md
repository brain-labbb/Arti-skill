# T2 v2：Source-grounded Template Authoring and Distribution Reliability

> 状态：新版协议草案，独立于原始 T2 方案；不覆盖旧实验结果。  
> 适用仓库：`/mnt/zsn/lyb/arti-skill`  
> 依据：`.agents/skills/build-template/SKILL.md`、`arti-template/articraft_template_authoring/README.md`、`AUTHORING.md`

## 1. 重新定义 T2 的问题

T2 不再把 SourceMap、TemplateDesign、Distribution Harness 和 Regression 混成一组
“消融方法”。新版 T2 只回答一个主问题：

> 在相同原始 source pool、相同模型和相同执行器下，SourceMap 与 TemplateDesign
> 这两个 authoring 产物分别是否提高了新模板的首轮可执行性、修复后成功率和跨 seed
> 分布可靠性？

项目的正式工作流是：

```text
SourceMap
→ TemplateDesign
→ 普通函数式单文件模板
→ preflight
→ random-16
→ random-36
→ corner
```

其中只有最终的 `arti-template/agent/templates/<slug>.py` 是运行真源。SourceMap 和
TemplateDesign 只在 authoring 阶段作为输入，生成后的模板运行时不得读取它们。

## 2. 两类 authoring 产物的严格定义

### 2.1 SourceMap：源资产事实层

SourceMap 必须覆盖指定 picture subcategory 的完整 active source pool，并记录：

- 每条 record/revision 的审阅状态：使用、参考、重复或拒绝；
- 采用候选的 `record/revision`、精确 `model.py` 行段和组件类型；
- 候选在结构或运动上为什么具有源依据；
- 被判为重复或参考的记录，以便完整性可审计。

SourceMap 不得包含：

- 参数范围或参数派生；
- interface、binding、multiplicity；
- 详细 category anchor；
- 组件源码、AST 闭包、snapshot 或运行时依赖。

### 2.2 TemplateDesign：语义设计决策层

TemplateDesign 从通过检查的 SourceMap 生成，并只记录无法由 SourceMap 机械推出的
设计决策：

- slot 的组织和每个 slot 的 candidate；
- 真正独立的参数及其语义单位；
- 局部派生参数及派生 DAG；
- plane/axis interface；
- `N`、spacing 和 host capacity；
- 组件间 binding 与跨组件派生；
- 候选轮廓、支承、开口和宿主过渡适配；
- 新建/重做模板所需的 category anchors、作者测试和 surface relations。

TemplateDesign 不是源码容器，不包含 Factory、snapshot、linker manifest 或生成的
实现代码。

## 3. 主实验：2×2 authoring 因子设计

所有 arm 使用完全相同的：

- 原始 source records；
- `arti-template/articraft_template_authoring/` 下的通用 authoring 文档；
- SDK、Python/依赖版本和执行器；
- authoring 模型、reasoning effort、采样设置、上下文预算；
- 自动修复预算和隐藏 evaluator。

只有两个输入因子变化：

| Arm | SourceMap | TemplateDesign | 正确解释 |
|---|---:|---:|---|
| **A00 Raw-only / Naive** | 0 | 0 | 模型从原始 source records 自己完成事实抽取和设计决策 |
| **A10 SourceMap-only** | 1 | 0 | 模型得到源事实层，仍需自己完成 slot、参数、interface 和 assembly 决策 |
| **A01 Design-only** | 0 | 1 | 模型得到冻结的设计决策；TemplateDesign 可能压缩了 SourceMap 信息，因此不是“没有任何源信息” |
| **A11 Full Ours** | 1 | 1 | 模型同时得到源事实层和语义设计决策 |

旧命名 `w/o SourceMap`、`w/o TemplateDesign` 保留为别名，但正式报告使用上表的
`Raw-only`、`SourceMap-only`、`Design-only`、`Full`，避免把“去掉一个文件”误写成
“完全没有该类知识”。

### 3.1 因果估计

以每个 `task × repeat` 为配对 block，估计：

```text
SourceMap 主效应       = mean(A10, A11) − mean(A00, A01)
TemplateDesign 主效应  = mean(A01, A11) − mean(A00, A10)
交互效应               = A11 − A10 − A01 + A00
```

交互效应是核心：如果 Full 只有在两者同时存在时才明显提升，说明 SourceMap 与
TemplateDesign 是互补的；如果只有 Design-only 提升，说明主要收益来自语义设计决策，
而不是源事实整理。

## 4. 任务与重复

### 4.1 Cohort

- 12 个真正 unseen category；simple / medium / complex 各 4 个；
- 采用现有 `formal_source_manifest.json` 的 12 类候选，但在正式执行前冻结 slug、
  source record hash、SourceMap hash 和 TemplateDesign hash；
- authoring model 禁止读取历史目标模板、其他 arm、其他 run 和 evaluator 内部状态；
- 已公开或已经用于调参的类别不得重新作为 paper test cohort。当前 6 类正式运行结果
  与新版协议不兼容，不与新版数字合并。

### 4.2 Repeats

- 每个 task × arm 做 3 个独立 authoring repeats；
- 总 authoring runs：`12 × 4 × 3 = 144`；
- repeat 使用冻结的独立 request seed；如果 provider 不支持可复现 seed，则记录完整
  request metadata，并将每次调用视为独立重复；
- run 顺序在 task、arm、repeat 内随机化，避免固定顺序偏差。

### 4.3 统一预算

- 首次 authoring 后最多 3 轮自动 repair；
- repair 只能看到归一化 evaluator feedback，不能看到隐藏 case 文件、目标模板、
  其他 arm 输出或完整运行时路径；
- 不允许人工编辑；任何人工编辑都使该 run 的正式结果失效，但仍保留为失败审计记录；
- authoring wall-time、input/output tokens、reasoning tokens 和 API cost 必须逐 run 记录。

## 5. 输入隔离与泄漏审计

每个 run 使用独立 worktree/sandbox，并由 executor 物化 allowlist：

### A00 Raw-only

- 共享 authoring contract、mechanical priors、visual diversity model、SDK；
- 该 task 的原始 source records；
- 不提供 SourceMap 和 TemplateDesign。

### A10 SourceMap-only

- A00 的全部输入；
- 该 task 的冻结 SourceMap；
- 不提供 TemplateDesign。

### A01 Design-only

- A00 的全部输入；
- 该 task 的冻结 TemplateDesign；
- 不提供 SourceMap。

### A11 Full

- A00 的全部输入；
- 该 task 的冻结 SourceMap 和 TemplateDesign。

所有 arm 都禁止：

- `arti-template/agent/templates/` 下的历史目标模板；
- 其他 task、其他 arm 和其他 run 的输入/输出；
- hidden evaluator case、target hash 和 repair 原始日志；
- web/network source；
- 在最终模板中 import SourceMap、TemplateDesign、source runtime、snapshot、其他组件
  Python 文件或其他模板。

executor 必须保存访问日志，并在 run manifest 中报告 allowlist、denylist、文件 hash、
违规路径和进程退出状态。违规 run 进入分母并标记 invalid，不可静默删除。

## 6. Authoring 阶段

authoring model 只写一个普通函数式、自包含的单文件模板，必须满足项目当前约定：

- 暴露一个公共 `build_*`、一个公共 `run_*_tests`、`config_from_seed`、新流程要求的
  `TEMPLATE_DOMAIN` 和必要的 `TEMPLATE_CORNERS`；
- 组件和装配逻辑在该文件内实现；
- 不使用 AST/source closure、component manifest、assembly manifest、linker manifest；
- `config_from_seed` 从声明域采样，不从预构建完整资产列表中选择；
- 不用删除测试、冻结运动、扩大 allowance 或删除失败组合来修复。

每个 run 的首轮输出先独立评估，再决定是否向 repair turn 提供归一化反馈。首轮结果
必须永久保留，即使 repair 后成功。

## 7. Panel A：Authoring Reliability

### 7.1 主指标

首要指标是 task-level paired binary outcome：

1. **First-shot executable + artifact saved**：无 repair 时模板可导入，并至少生成一个
   完整 URDF + mesh package；
2. **First-shot random-16 full-QC**：首轮在 seed 0–15 上达到项目 strict random-16
   门槛（至少 15/16，且不能有被抽到但零通过的 candidate）；
3. **Final random-16 full-QC**：最多 3 轮 repair 后达到相同门槛。

### 7.2 次指标

- first-shot contract/preflight pass；
- artifact saved rate；
- final executable rate；
- repair turns；
- repair success conditional on first-shot failure；
- human intervention rate；
- authoring wall time；
- input/output/reasoning tokens；
- API cost。

`Final Success` 不再单独作为唯一 headline，因为在 3 轮 repair 预算下很容易出现
所有 arm 饱和为 100%。必须同时报告 first-shot 和 final。

## 8. Panel B：Distribution Reliability

Panel B 不再把 “w/o Distribution Harness” 或 “w/o Regression” 当作与 authoring
输入同级的 method arm。它们是评估器能力/报告阶段，不能和 SourceMap 因子混在一起。

对于每个首轮模板和最终模板，分别执行：

1. **random-16**：累计 seed 0–15；
2. **random-36**：累计 seed 0–35，主报告同时给 pass rate 和严格 36/36 indicator；
3. **corner**：补齐随机 sweep 未覆盖的 candidate、`N` min/max 和声明的高风险组合；
4. **repair retention**：repair 前已经通过的 case，在 repair 后仍通过的比例；
5. **historical regression**：若该新 template 没有历史 seed，则记 `N/A`，不得把
   repair retention 改名为 historical regression。

### 8.1 分布指标

| 指标 | 定义 | 报告层级 |
|---|---|---|
| Seed Compile@36 | 完整 package 的 seed 数 / 36 | seed rate + task mean |
| Seed Full-QC@36 | 通过完整 CAD/URDF/topology/collision/motion QC 的 seed 数 / 36 | seed rate + task mean |
| Random-36 threshold | 至少 33/36 且每个被抽到 candidate 至少有一个 pass | task binary |
| Strict All-36 | 36/36 通过 | task binary |
| Corner pass | 通过 corner 数 / corner 总数 | case rate |
| Strict corner | 所有声明 corner 通过 | task binary |
| Repair retention | repair 后仍通过的原首轮 pass case / 原首轮 pass case | case rate |

Corner 分母由冻结的 `TEMPLATE_CORNERS` 和 protocol-generated missing candidates/
boundaries 决定，不能由模板作者临时增加或删除。

## 9. 不再使用的旧 T2 arms

以下旧 arms 从主 T2 删除，原因是它们不是与 SourceMap/TemplateDesign 同一层次的
因子：

- `w/o Distribution Harness`：是评估器/测试覆盖的移除，不是模板 authoring 输入消融；
- `w/o Regression`：是 repair 之后是否检查已有 pass case 的报告策略，不是生成方法；
- `w/o Cross-Component Constraints`：若移除的是 TemplateDesign 中的 binding/derived
  decisions，应改成预注册的 Design decision ablation；不能用模糊 prompt 删除；
- `w/o Slot Abstraction`：如果它实际等于不给 TemplateDesign，应并入 A01 Design-only；
  如果还改变了最终模板 API，则必须另做明确的 implementation factor；
- `Original Articraft`：不作为同一 authoring matrix 的直接 arm。若存在可运行的旧
  Articraft 模板，只作为 Panel B 的 legacy distribution reference，并标记 `reference`
  而非 authoring method。

## 10. Design decision ablation（可选扩展，不放主 T2）

如果论文必须说明 TemplateDesign 内部哪些决策有用，另开 T2b，不和 SourceMap 主效应
混合：

- Full Design；
- Full Design 去掉 cross-slot bindings；
- Full Design 去掉 category anchors；
- Full Design 去掉 surface relations（仅适用于有此类接口的 task）。

这些设计必须在生成前由脚本做 schema-validated、hash-stable 的机械变换；不能让作者
模型自行“忽略某些字段”，否则无法确认消融是否真的发生。

## 11. 统计分析与判定

### 11.1 主分析

以 `task × repeat` 为配对 block，使用 cluster bootstrap（cluster=task）报告 95% CI，
并给出每个 factor 的：

- absolute risk difference；
- paired odds ratio；
- interaction effect。

结果表同时列 task-level mean、case-level mean 和 95% CI，避免把 36 个 seed 当成
36 个独立 task。

### 11.2 结论门槛

只有满足以下条件，才允许写“Full 提升了 T2 authoring/distribution reliability”：

1. Full 在 preregistered primary metric（first-shot random-16 full-QC）上相对 Raw-only
   的 cluster-bootstrap CI 不跨 0；
2. 该提升在至少两个 complexity strata 方向一致；
3. 没有明显的 protocol violation 或 task-specific 单点驱动；
4. final metric 与 distribution metric 不出现完全相反且未解释的结果。

如果 Full 只在 repair 后的 random-36/corner 领先，应写成“Full 提升修复后的分布可靠性”，
不能写成“Full 提升首轮 authoring”。

## 12. 成本报告

SourceMap/TemplateDesign 的准备成本必须和 authoring cost 分开：

- `C_prepare_SM`：SourceMap 准备/审阅时间和 token；
- `C_prepare_TD`：TemplateDesign 准备/审阅时间和 token；
- `C_author`：每个 arm 的 authoring + repair 成本；
- `C_compile_QC`：seed/corner 编译与 QC 成本。

主表报告每个 run 的 authoring cost；补充表报告：

```text
C_amortized(K) = (C_prepare_SM + C_prepare_TD) / K
                 + C_author + C_compile_QC
```

其中 `K` 为同一 frozen SourceMap/TemplateDesign 被复用的模板实例数。不能把只给
Full 的 preparation cost 隐藏，也不能把 deterministic dataclass edit cost 当作 LLM
authoring cost。

## 13. 预注册结果表

### 主表：authoring factor effects

| Arm | #task | #repeat | First-shot executable | First-shot random-16 QC | Final random-16 QC | Repair turns | Authoring time |
|---|---:|---:|---:|---:|---:|---:|---:|
| A00 Raw-only | 12 | 3 |  |  |  |  |  |
| A10 SourceMap-only | 12 | 3 |  |  |  |  |  |
| A01 Design-only | 12 | 3 |  |  |  |  |  |
| A11 Full | 12 | 3 |  |  |  |  |  |

### 分布表：first-shot vs final

| Arm | Stage | Seed Compile@36 | Seed Full-QC@36 | Random-36 threshold | Strict All-36 | Corner pass | Repair retention |
|---|---|---:|---:|---:|---:|---:|---:|
| A00–A11 | first-shot |  |  |  |  |  | N/A |
| A00–A11 | final |  |  |  |  |  |  |

## 14. 执行前 gate

在任何 paid/full run 前，必须全部满足：

1. 12 个 task 的 source record manifest、SourceMap 和 TemplateDesign 已冻结并 hash；
2. SourceMap 通过 `source-map-check`；
3. TemplateDesign schema、字段边界和禁止内容通过静态检查；
4. 四个 arm 的 allowlist/denylist 在独立 sandbox 中做 1 task × 4 arm smoke；
5. smoke run 能证明最终模板不读取 SourceMap/TemplateDesign；
6. hidden evaluator、repair feedback normalization 和结果 schema 已冻结；
7. model identifier、reasoning effort、context budget、repair budget 和成本采集已冻结；
8. 旧的 24-run T2 结果被标记为 `legacy_partial_protocol`，不并入新版统计。

完成这些 gate 后，才运行 144 个正式 authoring packets。

