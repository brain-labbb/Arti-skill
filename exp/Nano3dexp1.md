# Nano3D 实验执行计划：33 个已导出资产的可行性与缺口

## 1. 实验定位

本计划依据 [`Nano3d.md`](/mnt/zsn/lyb/arti-skill/exp/Nano3d.md) 制定，使用已整理的 33 个资产清单 [`Nano3dasset.md`](/mnt/zsn/lyb/arti-skill/exp/Nano3dasset.md) 作为当前实验输入。

第一阶段回答：

> 当前已导出的资产能否被重新执行、保存为有效 artifact，并保留可解析的部件、层级和关节结构？

它不把已有导出结果误认为完整模板 benchmark，也不把编译成功误认为约束、语义或物理验证成功。

## 2. 结论摘要

### 2.1 当前可以做

基于现有工作区，可以直接开展 33 资产的：

- 静态 artifact/package 完整性审计；
- `model.py`、URDF 和 mesh 的重新执行 smoke test；
- URDF link/joint/root/depth/parent-child 基础结构统计；
- raw naming、可寻址性和跨 seed 词法一致性统计；
- URDF 表示层面的 joint type、axis、origin、limit 统计；
- source/URDF/assets 大小、路径、元数据字段存在性检查；
- 小规模渲染、结构 metrics 和 HTML 报告流程验证；
- 已有模板的 `random-16`、`random-36`、`corner` 管线检查；
- physics_10 clothes peg 的完整物理验证复核。

### 2.2 当前只能做降级版

以下轴可以做 pilot，但不能报告完整论文指标：

- Reliability：已有资产复现可靠性，而不是模板生成可靠性；
- Naming：raw name coverage，而不是 semantic precision/recall；
- Hierarchy：树合法性和结构统计，而不是 hierarchy exact match；
- Articulation：URDF 表示和少量运动 smoke test，而不是全行程物理有效性；
- Production Readiness：包和字段静态审计，而不是完整 mesh/仿真生产就绪度。

### 2.3 当前不能正式完成

目前不能声称完成 `Nano3d.md` 定义的完整实验，原因是：

- 当前是 33 个资产，不是 54 个冻结模板任务；
- 每个选定资产当前主要是一个 seed，不是每模板固定 36 个 seed；
- 没有覆盖所有任务的隐藏 `template_spec.json`；
- 没有完整四类 baseline 的公平运行记录；
- 没有 3 次独立模板生成和统一 repair budget；
- 没有固定的 LHS/Sobol、pairwise covering array、corner seed manifest；
- 没有完整 constraints gold、edit task、regression seed 和人工评测协议；
- physics_10 中只有 1/10 资产有 `validation_report.json` 且 `dataset_ready=true`。

当前最合理的结果名称是：

> **33 资产 existing-export pilot / static audit**，而不是 full Nano3D benchmark。

## 3. 当前工作区状态

| 项目 | 当前状态 | 影响 |
|---|---|---|
| 选定资产 | 33 个，L1/L2/L3 各 11 个 | 可作为 pilot cohort |
| 主导出输入 | 33 个资产均有 `model.py`、`model.urdf`、`assets/`、`compile_report.json` | 可做已有 artifact 检查 |
| 编译状态 | 33/33 的 `compile_report.json` 为 `status=success` | 只证明已有编译结果，不证明完整约束满足 |
| 模板 Python | 33 个选定名称均能找到对应模板 Python 文件 | 可做模板检查或重执行准备 |
| TemplateDesign | 22/33 有正式 `designs/<slug>.json` | 其余需补 Design 或走 legacy 检查 |
| SourceMap | 工作区有大量 SourceMap/picture source map | 可追溯来源，不等于评测 gold spec |
| `eval_pilot` | 默认 manifest 只有 3 个 Door_Double_Door seed | 可验证流程，不代表 33 资产或方法对比 |
| physics_10 | 10 个资产编译成功，只有 clothes peg 有验证报告 | 完整 physics-ready 样本为 1/10 |
| frozen seed manifest | 未发现本实验统一的 54×36 manifest | 正式分布级结果无法复现 |
| frozen template spec | 未发现覆盖全部选定任务的 gold spec | 严格 Naming/Hierarchy/Constraints 无法评分 |

### 3.1 当前缺少正式 TemplateDesign 的选定资产

以下资产需要补齐新管线 Design，或明确使用 legacy 兼容路径：

- `Astronomy_Antenna_dish`
- `Door_Double_Door`
- `Electrical_Wiring_Wire_stripper`
- `Industrial_Mine_cart`
- `Astronomy_Space_shuttle`
- `Vehicle_Sports_car`
- `Astronomy_Pressurised_module_door`
- `Door_Other`
- `Door_Trap_door`
- `Handtools_clothes_peg`
- `Stationary_Pencil_sharpener`

## 4. 七个评测轴的可行性矩阵

| Axis | 当前判断 | 当前能做什么 | 当前不能做什么 | 主要缺口 |
|---|---|---|---|---|
| Reliability | 可做降级版 | 33 个已有资产的执行、URDF/mesh/artifact 保存、错误和耗时审计 | 模板 first-shot、repair success、三次生成可靠性、36/36 seed 通过率 | baseline、run manifest、repair budget、telemetry、seed QC runner |
| Naming | 可做降级版 | link/joint 名称存在率、占位名称、重复实例可区分性、跨 seed 词法一致性 | semantic precision/recall、functional richness、人工 judge 结果 | gold taxonomy、同义词表、required/optional parts、judge 和人工复核 |
| Hierarchy | 可做降级版 | root、acyclic、reachable、深度、parent-child edge、pivot 基础统计 | edge F1、Hierarchy Exact Match、Semantic Nesting、可变拓扑一致性 | gold hierarchy、wrapper folding 规则、兼容拓扑规则 |
| Constraints | 已做 source-derived/operational v1；论文同口径仍不能正式做 | 33×36 seed 的 structured count/relations、URDF numeric/interface、motion-QC、valid-config compatibility、coverage/satisfaction/all-pass | hidden-spec 目标尺寸/接口 metrology、非法组合拒绝、论文 52-constraint 同集复现 | 独立 `template_spec.json`、几何测量 recipes、容差和合法/非法组合 manifest |
| Editability | 暂不能正式做 | 少量目标可寻址和重新导出 smoke test | 54 edits×16 seeds、target fulfillment、non-target preservation、locality、regression、盲评 | 18 模板、54 edit tasks、16 回归 seed、diff scorer、3 名评测员 |
| Articulation | 已完成 paper-aligned functional proxy；正式 semantic correctness 仍 unsupported | 33/33 native articulation、186 movable joints、URDF metadata、11-state single-joint sweep、24×64 Sobol multi-joint sweep、collision state logs、generic revolute-range audit | joint type accuracy、joint recall、parent-child accuracy、axis-on-moving-part、rest-pose-frozen、论文意义的 joint/asset geometric validity、CCD/clearance、独立 limit-reachability | frozen kinematic gold/spec、articulation 前后 artifact pair、axis semantic judge、CCD/clearance evaluator；当前使用 PyBullet 离散步进和 `URDF_USE_SELF_COLLISION_EXCLUDE_PARENT`，不能把 collision-only proxy 写成完整物理有效性 |
| Production Readiness | 可做降级版 | 包大小、路径、visual/collision 引用、语义/运动/物理字段存在性 | watertight、manifold、self-intersection、CoACD、clean rebuild、determinism、完整惯量质量 | mesh scorer、physics validator、collision variants、clean environment、hash manifest |

## 5. 第一阶段：33 资产 Existing-Export Pilot

这一阶段不生成新模板，只评估已有导出资产的当前状态。

### 5.1 输入冻结

建立只读 `asset_manifest.jsonl`，每行至少包含：

```yaml
asset_id:
asset_path:
source_dir: seed_exports | seed_exports_physics_10
slug:
seed:
difficulty: L1 | L2 | L3
model_py:
model_urdf:
assets_dir:
compile_report:
model_py_sha256:
model_urdf_sha256:
compile_report_sha256:
link_count:
joint_count:
asset_file_count:
physics_validation_status:
```

冻结后不替换失败资产；失败必须保留在报告中并分类。所有重执行输出写入独立 run 目录，不能覆盖原始资产。

### 5.2 静态 artifact 检查

逐个资产检查：

- `model.py` 是否存在且可读；
- `model.urdf` 是否存在且 XML 可解析；
- `assets/` 是否存在且非空；
- URDF 中的 mesh 相对路径是否可解析；
- visual/collision geometry 是否存在；
- link/joint 引用是否闭合；
- 是否有绝对路径或缺失依赖；
- `compile_report.json.status` 是否为 `success`；
- 包是否能被复制到独立目录后读取。

### 5.3 已有资产重执行

在固定环境中重新执行每个 `model.py`，不人工修改代码，记录：

- executable；
- URDF generated；
- mesh generated；
- artifact saved；
- first-shot success；
- repair required；
- repair count；
- wall time；
- compile/probe count；
- stdout/stderr 和错误类别；
- 输出文件大小。

这里的 first-shot 是**已有资产重执行 first-shot**，不能称为模板生成 first-shot。

### 5.4 URDF Hierarchy 基础检查

从最终 URDF 确定性计算：

- link 数量；
- joint 数量；
- root 数量；
- 是否有环；
- 所有 link 是否从 root 可达；
- 最大层级深度；
- parent/child 是否引用存在的 link；
- joint type、axis、origin、limit 是否可解析；
- 可识别的 subassembly 和 pivot 数量。

### 5.5 Raw Naming 基础检查

统计：

- link/joint/mesh 名称存在率；
- 占位名称比例，例如 `link_0`、`part_01`、`mesh_003`；
- 重复名称率；
- left/right、`_0`/`_1` 等实例区分率；
- 同一模板已有多个 seed 的名称集合稳定性；
- 名称与 SourceMap/Design 候选槽位的词法匹配率。

没有 gold taxonomy 时只报告 raw coverage 和 lexical consistency，不报告 semantic precision/recall。

### 5.6 Articulation 表示层基础检查

统计：

- native joint exposure；
- fixed/revolute/prismatic/continuous 类型分布；
- parent-child 完整率；
- axis/origin 完整率；
- lower/upper limit 完整率；
- 是否出现过宽的通用运动范围；
- 每资产 joint 数量和 movable joint 数量。

低复杂度资产可做少量 rest pose、边界 pose 和碰撞 smoke test；L3 资产先做解析和静态检查。

### 5.7 Production Readiness 静态检查

拆成三个维度：

#### Semantic completeness

- part names；
- hierarchy；
- metric scale；
- material label。

#### Kinematic completeness

- joint type；
- parent/child；
- axis/origin；
- limits；
- damping/friction。

#### Physical completeness

- visual mesh；
- collision mesh；
- mass；
- inertia；
- physics validation report。

当前只做字段和引用存在性检查，不把存在性当作物理质量证明。

### 5.8 Pilot 输出

建议生成：

```text
asset_manifest.jsonl
asset_static_checks.jsonl
asset_execution_results.jsonl
asset_hierarchy_results.jsonl
asset_naming_basic_results.jsonl
asset_articulation_basic_results.jsonl
asset_packaging_results.jsonl
experiment1_summary.json
experiment1_summary.md
```

主表：

| Group | N | Executable | Artifact Saved | Valid Tree | Raw Name Coverage | Joint Metadata | Portable Package |
|---|---:|---:|---:|---:|---:|---:|---:|
| All 33 assets | 33 | 待测 | 待测 | 待测 | 待测 | 待测 | 待测 |
| L1 | 11 | 待测 | 待测 | 待测 | 待测 | 待测 | 待测 |
| L2 | 11 | 待测 | 待测 | 待测 | 待测 | 待测 | 待测 |
| L3 | 11 | 待测 | 待测 | 待测 | 待测 | 待测 | 待测 |
| `seed_exports` | 23 | 待测 | 待测 | 待测 | 待测 | 待测 | 待测 |
| `seed_exports_physics_10` | 10 | 待测 | 待测 | 待测 | 待测 | 待测 | 待测 |

## 6. 当前可用的工作区工具

已有 `eval_pilot` 可做流程 smoke test：

```bash
cd /mnt/zsn/lyb/arti-skill/arti-template

uv run python ../eval_pilot/pilot.py metrics
uv run python ../eval_pilot/pilot.py render --renderer blender
uv run python ../eval_pilot/pilot.py pairs
uv run python ../eval_pilot/pilot.py report
```

当前默认 manifest 只有 3 个 `Door_Double_Door` seed，因此这些命令当前只能证明 pilot 流程可运行。若要用于 33 资产，必须先冻结新的 33 行 manifest，并通过 `EVAL_MANIFEST` 指向它。

VLM judge 需要另外冻结 endpoint、model、prompt、AB/BA 顺序、缓存和 API key。没有这些条件时，只运行本地 metrics、render 和 report，不运行语义 judge。

`arti-template` 的模板管线可按现有文档使用：

```bash
uv run articraft template check <slug> --stage random-16
uv run articraft template check <slug> --stage random-36
uv run articraft template check <slug> --stage corner
```

这些检查可验证模板管线的累计通过率和 corner 行为，但没有 frozen benchmark spec 时，不能自动产生 `Nano3d.md` 所要求的完整 Naming/Hierarchy/Constraints gold 评分。

## 7. 七个 Axis 的正式实验计划

### Axis 1：Reliability

#### 补齐后可做

对每个方法、每个任务独立运行 3 次，记录：

- Template Executable Rate；
- Artifact-Saved Rate；
- First-Shot Template Success；
- Final Template Success；
- wall time、Agent turns、tokens、API cost；
- source LOC、source bytes；
- Seed Compilation Rate；
- Full-QC Seed Pass Rate；
- 36/36 Pass；
- Corner Pass；
- Regression Retention。

#### 当前不能做

已有导出资产只能说明已存在的 artifact 曾经成功生成，不能反推模板 first-shot、repair success、三次生成可靠性或 36/36 通过率。

#### 缺什么

四类 baseline、统一 run manifest、三次调度器、repair budget、seed-level QC runner、token/cost telemetry 和失败 taxonomy。

### Axis 2：Naming

#### 当前可做

做 raw name coverage、占位名称、重复实例可区分性、跨 seed lexical consistency。

#### 当前不能做

没有 gold taxonomy 时，不能报告 semantic precision/recall、functional richness 或名称是否真实对应功能部件。

#### 缺什么

每模板 required/optional semantic parts、同义词表、extra-real-part 标注、三个固定 LLM judge、10% 人工复核和一致性统计。

### Axis 3：Hierarchy

#### 当前可做

做 root、acyclic、reachable、depth、parent-child edge、joint pivot 的确定性统计。

#### 当前不能做

没有 gold tree 和 exporter wrapper 折叠规则时，不能报告 edge F1、Hierarchy Exact Match、Semantic Nesting Accuracy 或跨 seed topology consistency。

#### 缺什么

每模板 gold hierarchy、语义 wrapper 规则、固定/可变拓扑标签、拓扑兼容规则。

### Axis 4：Constraints

#### 当前可做

做 link/joint count、部分 joint limit 字段、bounding box 和已有 compile/QC 信号检查。

#### 当前不能做

不能把 `compile_report.status=success` 当作 constraint satisfaction，也不能报告 coverage、satisfaction、all-pass asset、invalid combination rejection 或 36-seed constraint reliability。

#### 缺什么

每模板至少 8 条冻结约束，覆盖 count、numeric、relational、interface、kinematic、compatibility；还需要最终几何测量器、容差、合法/非法组合清单和 scorer。

### Axis 5：Editability

#### 当前可做

补充单个 edit task 后，可以做目标可寻址、source changed、output changed、URDF 可解析的 smoke test。

#### 当前不能做

不能报告 18 模板 × 3 类 edit、54 edits × 16 seeds、target fulfillment、anchor/scale、non-target preservation、geometry/structural locality、regression retention 或三人盲评。

#### 缺什么

18 个模板、54 个 edit task、每项 16 个回归 seed、修改前后 artifact 对齐、几何/结构 diff scorer、3 名 3D/机器人评测员和 Fleiss’ kappa 或 Krippendorff’s alpha 流程。

### Axis 6：Articulation

#### 当前可做

33 个资产的 URDF 可支持 native joint、type、parent-child、axis、origin、limit 的表示层统计；已完成 11-state single-joint sweep、24 个多关节资产各 64 点 Sobol sweep 和逐状态 collision log。clothes peg 的既有 physics validation 仍单独保留，不与本轮 sweep 混合。

#### 当前不能做

不能用 33 个 existing-export 资产替代论文 12-asset generated case study 或完整 1,944-seed benchmark；不能把 3,582/3,582 collision-only states 写成论文意义的 joint/asset geometric validity；没有 joint semantic gold 不能报告 type accuracy、joint recall、parent-child accuracy 或 axis-on-moving-part；没有 articulation 前后 artifact pair 不能报告 rest-pose frozen；没有 CCD/clearance evaluator 不能报告连续碰撞无关的 clearance/CCD 结论。

#### 缺什么

缺 frozen kinematic spec/gold、articulation 前后 artifact pair、axis-on-moving-part semantic annotation、CCD/clearance evaluator、跨 seed articulation manifest，以及论文原生 generated asset IDs 和 evaluator。低-clearance 加密采样、连续碰撞检测和独立 limit-reachability 仍未完成。

### Axis 7：Production Readiness

#### 当前可做

做 source/URDF/assets 大小、相对路径、visual/collision 引用、包自包含性、semantic/kinematic/physical 字段存在性检查。

#### 当前不能做

不能完整报告 watertight、manifold、open edge、degenerate face、self-intersection、exact-vs-convex-vs-CoACD、clean rebuild、determinism、完整惯量质量或 UV/material readiness。

#### 缺什么

mesh quality scorer、mass/inertia/dynamics validator、visual-collision deviation evaluator、single convex hull/CoACD 工具链、clean environment、重建 hash 和 versioned package manifest。

## 8. 正式 Nano3D benchmark 的补齐计划

### 8.1 冻结任务规模

正式目标为：

```text
6 domains × 3 difficulty levels × 3 templates = 54 templates
54 templates × 36 seeds = 1,944 assets
54 templates × 3 runs = 162 template-generation runs per method
```

33 个资产只能作为 pilot cohort，不能直接扩展成正式 54×36 结果。

### 8.2 冻结每模板 36 个 seed

每模板保存：

- 28 个常规覆盖 seed；
- 4 个离散组合边界 seed；
- 4 个连续参数 corner seed；
- 连续参数的 LHS 或 Sobol 采样；
- 离散组件的 pairwise covering array；
- 最小/最大尺寸、最小 gap、最大展开状态标签。

建议字段：

```yaml
template_id:
seed:
seed_class: regular | discrete_boundary | continuous_corner
continuous_parameters:
discrete_choices:
expected_components:
expected_joint_state:
is_corner:
```

### 8.3 建立隐藏 frozen spec

每个 benchmark item 至少要有：

```yaml
template_id:
category:
difficulty:
required_parts:
optional_parts:
semantic_names:
required_hierarchy:
joint_types:
joint_parent_child:
joint_axes:
joint_limits:
continuous_parameters:
discrete_slots:
candidate_components:
numeric_constraints:
count_constraints:
relational_constraints:
interface_constraints:
compatibility_rules:
edit_tasks:
```

该 spec 对 Agent 隐藏、对 evaluator 可见；所有指标从最终 URDF、mesh 和仿真状态重新测量，不读取源代码自报目标。

### 8.4 冻结四类 baseline

| 方法 | 要求 |
|---|---|
| Original Articraft | 原始单实例 `model.py`，不模板化 |
| Naive Same-LLM | 相同 LLM 和任务描述，直接要求参数化/增加变体 |
| Ours w/o Distribution Harness | 有 SourceMap/slot/constraint，但只验证少量默认 seed |
| Full Arti-Template | SourceMap、slot abstraction、constraints、36 seed、corner、regression |

必须统一原始输入、LLM、prompt、reasoning effort、token、工具、修复轮数、环境和随机重复协议。

## 9. 推荐执行顺序

### P0：冻结 33 资产 pilot

1. 从 `Nano3dasset.md` 生成 33 行 manifest；
2. 保存输入文件 hash；
3. 不覆盖原始资产；
4. 明确 physics validation 的 `validated`、`compile_only`、`missing_report` 状态；
5. 将输出命名为 `existing_asset_pilot`。

### P1：先做不依赖 gold 的静态结果

1. artifact/package audit；
2. URDF parse 和 hierarchy 基础指标；
3. raw naming 指标；
4. joint metadata 指标；
5. source/URDF/assets telemetry；
6. 本地 render 和 HTML report。

### P2：补齐 33 个资产的初版 gold

1. 补齐缺少 Design 的 11 个资产；
2. 将 TemplateDesign 与 frozen benchmark spec 分开；
3. 补齐 semantic parts、hierarchy、joints、constraints、compatibility、edit fields；
4. 人工审核 spec；
5. 为 scorer 编写正例和失败例。

### P3：33 资产 reduced benchmark

如果暂时不扩展到 54 模板，则明确声明 reduced benchmark，并固定额外的 16 或 36 seed；只报告实际覆盖的模板/seed，不与 Nano3D 54×36 数字直接比较。

### P4：正式 54-template benchmark

1. 冻结 54 个任务；
2. 冻结 36-seed manifest；
3. 运行四类 baseline；
4. 每方法每任务独立运行 3 次；
5. 分七个 axis 统计；
6. 生成 Table A/B/C 和 Evaluation Coverage Matrix。

## 10. 结果表

### 10.1 当前 Pilot 表

| Group | N | Executable | Artifact Saved | Valid Tree | Raw Name Coverage | Joint Metadata | Portable Package |
|---|---:|---:|---:|---:|---:|---:|---:|
| All 33 assets | 33 | 待测 | 待测 | 待测 | 待测 | 待测 | 待测 |
| L1 | 11 | 待测 | 待测 | 待测 | 待测 | 待测 | 待测 |
| L2 | 11 | 待测 | 待测 | 待测 | 待测 | 待测 | 待测 |
| L3 | 11 | 待测 | 待测 | 待测 | 待测 | 待测 | 待测 |
| `seed_exports` | 23 | 待测 | 待测 | 待测 | 待测 | 待测 | 待测 |
| `seed_exports_physics_10` | 10 | 待测 | 待测 | 待测 | 待测 | 待测 | 待测 |

### 10.2 正式 benchmark 主表

| Method | Template Exec. | Seed QC | Naming P/R | Tree Exact | Constraint Sat. | Edit Pass | Full-Range Valid | Portable | Semantic Meta. | Kinematic Meta. | Physical Meta. |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|

七个 axis 不平均为一个总分；至少分别报告 Reliability、Naming、Hierarchy、Constraints、Editability、Articulation 和 Production Readiness。

## 11. 当前结果的禁止性表述

在缺口补齐前，不能写：

- “33 个资产完成了 Nano3D 完整 benchmark”；
- “所有 33 个资产都 physics-ready”；
- “compile success 等于 constraint satisfaction”；
- “URDF 中有 joint 等于 articulation 全行程有效”；
- “有部件名称等于 semantic naming precision 高”；
- “有 parent-child edge 等于 hierarchy exact match”；
- “单次 edit 成功等于 16-seed distributional edit success”；
- “存在模板 Python 文件等于完成 Full Arti-Template baseline”；
- “已有 pilot judge 等于 Ours-vs-baseline 结果”；
- “33 个样本证明了 54-template 分布泛化”。

推荐使用：

> 在 33 个已导出资产上建立 existing-export 的执行、artifact、原生层级、基础命名、基础关节和包结构基线，并明确分布级模板实验的缺失条件。

## 12. 最终验收标准

### 12.1 实验一 Pilot 完成

满足以下条件即可称为实验一完成：

1. 33/33 资产完成静态检查；
2. 每个资产完成一次固定环境重执行；
3. 每个资产都有 executable、artifact-saved 和 error 状态；
4. 每个资产都有 hierarchy、raw naming、articulation-basic 和 packaging 结果；
5. 所有失败可归类为代码、依赖、路径、URDF、mesh、QC 或 physics validation 问题；
6. 报告明确区分已验证指标和因缺少 gold/spec 无法验证的指标。

### 12.2 完整 benchmark 完成

只有在以下内容全部存在后，才可以声称完成 `Nano3d.md` 的正式实验：

- 54-template frozen task manifest；
- 每模板 36-seed manifest；
- 每模板隐藏 `template_spec.json`；
- 四类 baseline 的公平运行记录；
- 162 个模板生成 run 的日志和 artifact index；
- Naming/Hierarchy/Constraints gold 和 scorer；
- 54 edit tasks × 16 regression seeds；
- articulation motion sweep 和 collision state logs；
- mesh/URDF/source production telemetry；
- 七轴 evaluator 和 failure taxonomy；
- Table A/B/C 与 reproducibility manifest。

## 13. 最终判断

当前应先完成 **33 资产 existing-export pilot**。它可以可靠回答：已有导出资产是否具备基本的可执行、可寻址、可解析和可运动结构。

它不能回答：模板经过三次生成是否仍可靠、36 个固定 seed 是否全部通过、局部 edit 是否传播到 16 个回归 seed、关节是否全行程无碰撞，以及不同 baseline 是否公平优于彼此。

在 frozen spec、seed manifest、baseline、edit protocol、physics validation 和统一 evaluator 补齐前，所有结果都必须标注为 pilot、static audit 或 reduced benchmark。

## 14. 本轮实际执行结果与支持缺口（2026-08-04）

本轮按“先测现有文件即可得到的指标”的要求，运行了 33 个已冻结资产的 existing-export pilot。固定输入为 [`Nano3dasset.md`](/mnt/zsn/lyb/arti-skill/exp/Nano3dasset.md) 中的 33 个链接，未修改任何 source asset；运行产物位于 `/mnt/zsn/lyb/arti-skill/exp/runtime/nano3d_asset_pilot/`，固定流程见 [`listharness.md`](/mnt/zsn/lyb/arti-skill/exp/test/listharness.md)。

### 14.1 已直接测得

| 项目 | 实测结果 | 结论边界 |
|---|---:|---|
| 资产冻结与来源 | 33/33，seed_exports 23 个、seed_exports_physics_10 10 个；L1/L2/L3 各 11 个 | 只代表当前选定资产集 |
| 既有 compile report | 33/33 status=success | 不等同于重新生成实验 |
| URDF XML 解析 | 33/33 valid | 只证明 XML/基本 link 结构可解析 |
| 层级树结构 | 33/33 valid tree；无环、单根、可达 | 不证明语义 hierarchy |
| 隔离重执行 | 33/33 success；33/33 保存 generated URDF；230.64 s | 只证明已有 model.py 在当前环境可重执行 |
| 原始名称覆盖 | 33/33 的 link/joint 名称通过非占位名检查；均值 100% | 不证明 semantic precision/recall |
| 关节元数据字段 | 均值 100%；平均 6.30 joints/asset、5.64 movable joints/asset | 只检查 parent/child/type，以及可动关节的 axis/limit 字段存在性 |
| 基本包完整性 | 33/33：model.py、model.urdf、compile_report、assets 存在，mesh 引用可解析且无绝对路径 | 不证明可移植构建、确定性或物理完整 |
| mesh 静态审计 | 387 个可读 mesh geometry；watertight 37.47%；winding-consistent 97.93%；open edges 290,335；degenerate faces 51 | 不是完整 manifold/self-intersection/碰撞评测 |
| physics_10 现状 | 10 个记录中仅 1 个 validation report 为 success+dataset_ready | 不能把其余 9 个当作 physics pass |

### 14.2 不能直接测、或需要支持后才能测

| Nano3D 轴/指标 | 本轮状态 | 缺少的支持或输入 |
|---|---|---|
| Reliability 的 54-template × 36-seed、first-shot/final、36/36、corner、regression、repair turns | **未完成；仅完成 33 个 existing-export 重执行** | 54-template frozen manifest、每模板 36 seed、统一生成入口、repair 日志、seed QC、hidden spec、baseline 运行记录 |
| Naming 的 semantic precision/recall、functional richness、instance discriminability、cross-seed、over-segmentation | **未完成；仅完成 raw name coverage** | 每模板/seed 的 semantic gold、part/instance/function 标注和 scorer |
| Hierarchy 的论文四项（has tree、depth、named groups、pivots） | **已完成 URDF-equivalent 复现**：1.000、3.121、0.061、5.636 | 论文原生 GLB evaluator/backend 未公开；当前映射与结果冻结在 `exp/runtime/nano3d_hierarchy/` |
| Hierarchy correctness 扩展（edge F1、exact match、semantic nesting、正式 cross-seed） | **仍 unsupported；论文自身未评分前三项，也未报告 cross-seed** | frozen hierarchy gold、wrapper/节点 canonicalization、固定/可变 topology 标签和 compatibility rules |
| Constraints | **Supplementary source-derived/operational v1 已完成**：33×36 seed、17,706/17,706 measurable、17,125 pass；但 paper-aligned preflight 为 prompt 0/18、spec 0/18，正式结果仍 N/A | 缺论文 constrained prompts、预冻结 `spec.yaml`、GLB semantic anchor + geometry measure recipes/comparator/tolerance 和公开 scorer；不得从输出反推目标 |
| Editability 全部指标 | **未运行** | edit task 集、target/anchor/scale 定义、非目标保持与 locality scorer、16-seed propagation、回归协议 |
| Articulation 的 joint type/recall/parent-child accuracy、origin、joint/asset geometry、full-range collision-free、generic range | **未完成；仅完成字段级 joint metadata** | joint semantic gold、运动范围协议、轨迹采样、碰撞检测、状态日志和几何有效性 evaluator |
| Production 的 manifold/self-intersection/deterministic build/semantic-kinematic-physical complete | **未完成；完成部分 mesh/package 静态审计** | manifold 与自交检测器、可复现构建 hash、semantic/kinematic completeness spec，以及除 1 个外其余 physics_10 的 validation report |
| Table 11 failure taxonomy 的完整覆盖 | **部分完成** | 本轮只覆盖 existing-export compile/probe；需把约束、编辑、运动、生产阶段接入统一失败 schema |
| Table 12 tokens/API cost/agent turns | **未完成** | API/agent telemetry、输入输出 token、每 run 成本采集器；本轮可记录 wall time、source LOC/size、probe count |

### 14.3 本轮允许写入 results 的口径

`Nano3dresults.md` 中直接写入的比例只包括实际存在的 artifact、URDF、结构树、字段存在性、mesh 静态审计、包完整性和隔离重执行；所有需要 gold、隐藏 spec、跨 seed、编辑任务或物理 sweep 的单元统一写 `N/A`/`未运行`。因此本轮结果是 **existing-export pilot / static audit / reduced benchmark**，不能外推为 Nano3D 设计中完整的 54-template 实验结论。

## 15. 低/中难度补测状态（2026-08-04）

在不修改 `/mnt/zsn/lyb/arti-skill/exp/` 目录之外代码的前提下，已完成低/中难度补测。新增 harness 为 [`run_nano3d_low_medium.py`](/mnt/zsn/lyb/arti-skill/exp/scripts/run_nano3d_low_medium.py)，运行产物位于 `/mnt/zsn/lyb/arti-skill/exp/runtime/nano3d_low_medium/`。

低难度复核结果：33/33 重执行成功、33/33 artifact 保存、33/33 valid tree、33/33 basic package complete、mesh parser errors 为 0。

中难度结果：选定 33 个 slug 中 23 个存在多 seed sibling，共 271 个可解析 seed；URDF valid rate 为 100%，多 seed name signature mode rate 均值 32.90%，pairwise name Jaccard 均值 55.52%，link-count mode rate 均值 54.49%。10 个 physics_10 cohort 为 single seed，不报告跨 seed consistency。

Articulation representation smoke 对 33/33 资产通过；隔离环境 `/mnt/zsn/lyb/arti-skill/exp/.venv_low_medium` 中 `urdfpy`/PyBullet 均可用，33/33 URDF 加载成功、33/33 rest step 成功、286/286 个单关节 boundary state step 成功，诊断 self-contact 为 0。该结果仍不是完整 physics sweep，不能报告 full-range collision-free、clearance 或 limit reachability。mesh 静态审计为 387 个可读 geometry、145 个 watertight、379 个 winding-consistent、290,335 open edges、51 degenerate faces。
