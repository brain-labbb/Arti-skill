# 面向 Arti-Template 的 Nova3D-Bench 适配实验设计

## 一、核心适配原则

Nova3D-Bench 评测的是：

> 一个生成出的 3D 资产，是否具有可执行代码、语义部件、层级、可测约束、局部编辑和关节等“可编程能力”。

它使用 54 个冻结样本，覆盖 6 个领域和 3 个难度等级，并针对不同评测轴选择真正适用的 baseline，而不是把所有方法强行放进同一张总榜。([arXiv][1])

Arti-Template 需要把问题扩展为：

> 一个从 Articraft 单实例程序提升得到的模板，能否可靠地生成一整个结构正确、约束满足、可编辑、可运动的资产分布？

因此，Nova3D 的每个轴都需要同时评价两个层次：

| 层次                       | 评测对象                          |
| ------------------------ | ----------------------------- |
| Template level           | 模板代码是否正确、结构是否完整、能否持续生成        |
| Asset-distribution level | 不同 seed、参数和组件组合下，输出资产是否持续满足要求 |

**不建议把七个轴平均成一个总分。** Reliability 高并不代表 Naming 好，Naming 好也不代表 Articulation 或 Production Readiness 好。应像 Nova3D 一样分别报告。

---

# 二、Benchmark 数据构造

## 2.1 54 个冻结模板任务

直接借鉴 Nova3D 的规模：

[
6\text{ domains}\times3\text{ difficulty levels}\times3\text{ templates}=54
]

建议从完整模板库中分层选择 54 个代表性模板。

### 六个领域

可以按你们真实类别体系调整，推荐：

1. Furniture & Storage：柜、抽屉、桌、架；
2. Architectural Fixtures：门、窗、百叶、闸门；
3. Household Appliances：冰箱、烤箱、洗衣机、小家电；
4. Tools & Mechanisms：夹具、剪刀、泵、机械支架；
5. Consumer & Office Devices：笔记本、显示器支架、打印设备；
6. Mobility & Industrial Assets：车辆部件、机械传动、工业设备。

### 三个难度等级

| 等级 | 建议判定标准                                     |
| -- | ------------------------------------------ |
| L1 | ≤3 个 link；1 个关节；≤2 个组件位；无跨组件耦合             |
| L2 | 4–8 个 link；2–3 个关节；3–5 个组件位；至少一个派生或接口约束    |
| L3 | >8 个 link，或 ≥4 个关节，或具有 mimic、闭链、强耦合、复杂兼容关系 |

每个领域每个难度选 3 个模板。

---

## 2.2 每个模板固定 36 个评测 Seed

每个模板生成：

* 28 个常规覆盖 seed；
* 4 个离散组合边界 seed；
* 4 个连续参数 corner seed。

因此：

[
54\times36=1,944
]

个评测资产。

36 个 seed 不能纯随机生成，应固定为：

* 连续参数：Latin Hypercube 或 Sobol sampling；
* 离散组件：pairwise covering array；
* 边界情况：最小尺寸、最大尺寸、最小间隙、最大展开状态。

全部方法使用同一套 seed manifest。

---

## 2.3 Frozen Template Spec

Nova3D 用隐藏的冻结 spec 记录要求，并从最终 GLB 中重新测量，而不是相信源代码自报。它的 constraint scorer 会从导出结果中定位语义节点，再测量数量和尺寸。

每个 Arti-Template benchmark item 应有一个人工审核的隐藏 `template_spec.json`：

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

评测原则：

> 所有指标尽量从最终导出的 URDF、mesh 和仿真状态中测量，而不是直接读取模板代码里的目标变量。

---

# 三、统一 Baseline 设置

建议设置四个主要系统：

| 方法                            | 描述                                             |
| ----------------------------- | ---------------------------------------------- |
| Original Articraft            | 原始单实例 `model.py`，不做模板化                         |
| Naive Same-LLM                | 使用相同 LLM，直接要求“把代码参数化并增加变体”                     |
| Ours w/o Distribution Harness | 有 Source Map、槽位和约束，但只验证少量默认 seed               |
| Full Arti-Template            | 完整 Source Map、槽位抽象、约束、多 seed、corner test 和回归验证 |

可额外加入：

* Ours w/o cross-component constraints；
* Ours w/o regression testing；
* 少量人工模板作为 oracle reference。

## 输入公平性

所有 Agent 方法必须使用：

* 相同原始 Articraft `model.py`；
* 相同模板化任务描述；
* 相同 LLM；
* 相同 reasoning effort；
* 相同 token、工具和修复轮数预算；
* 相同运行环境。

隐藏 benchmark spec 不提供给 Agent。

## 随机重复

模板生成阶段具有随机性，因此每个方法每个任务独立运行 3 次：

[
54\times3=162\text{ template-generation runs}
]

Reliability 使用全部三次运行统计。其他结构实验使用每个任务的**第一次有效输出**，不允许 best-of-three 选择。

---

# 四、Axis 1：Reliability

## 4.1 Nova3D 原指标

Nova3D 报告：

* Executable rate；
* Artifact-saved rate；
* First-shot rate；
* 修复后最终成功率；
* wall time；
* token；
* 代码行数和源文件大小。

Nova3D 的 54 个任务全部可执行并成功保存产物，first-shot 为 81.5%；同 LLM、无系统包的 naive ablation 只有 57.4% 可执行。该轴只证明运行可靠性，不证明视觉或结构质量。([arXiv][1])

## 4.2 Arti-Template 适配问题

> 系统能否把一个单实例程序可靠地转化为一个在整个 seed 分布上均可运行的生成器？

## 4.3 模板级指标

### Template Executable Rate

[
R_{\mathrm{template-exec}}
==========================

\frac{N_{\mathrm{可执行模板}}}
{N_{\mathrm{模板生成任务}}}
]

### Template Artifact-Saved Rate

能够成功输出：

* 模板源代码；
* seed manifest；
* URDF；
* mesh；
  -评测元数据。

### First-Shot Template Success

无需任何修复即可通过默认 seed 和基础验证的比例。

### Final Template Success

在不超过 3 次修复内最终成功的比例，以便与 Nova3D 的 repair budget 接近。

### 生成开销

* wall time；
* Agent turns；
* compile/probe 次数；
* prompt/output tokens；
* API cost；
* source LOC 和文件大小。

---

## 4.4 分布级可靠性

### Seed Compilation Rate

[
R_{\mathrm{seed-compile}}
=========================

\frac{N_{\mathrm{成功生成URDF和mesh的seed}}}
{N_{\mathrm{全部seed}}}
]

### Full-QC Seed Pass Rate

[
R_{\mathrm{seed-QC}}
====================

\frac{N_{\mathrm{通过所有QC的seed}}}
{N_{\mathrm{全部seed}}}
]

### All-Seeds-Pass Template Rate

[
R_{\mathrm{all-seed}}
=====================

\frac{N_{\mathrm{36个seed全部通过的模板}}}
{N_{\mathrm{模板}}}
]

该指标比平均 seed pass rate 更严格。

### Corner-Seed Pass Rate

单独评价极端参数和最易发生碰撞的 seed。

### Regression Retention

修复一个新 seed 后，历史已通过 seed 是否仍然通过：

[
R_{\mathrm{reg}}
================

1-
\frac{N_{\mathrm{旧seed回归失败}}}
{N_{\mathrm{历史通过seed}}}
]

## 4.5 建议结果表

| Method | Template Exec. | First-shot | Final Success | Seed Compile | Seed QC | 36/36 Pass | Corner Pass | Repair Turns |
| ------ | -------------: | ---------: | ------------: | -----------: | ------: | ---------: | ----------: | -----------: |

---

# 五、Axis 2：Naming

## 5.1 Nova3D 原指标

Nova3D 将 Naming 设计为能力阶梯：

[
\text{produces parts}
\rightarrow
\text{parts are named}
\rightarrow
\text{names are semantically rich}
]

它报告：

* parts 数；
* named rate；
* naming richness；
* repeated-instance discriminability；
* semantic recall；
* semantic precision。

Nova3D 还使用三个语义 judge 检查名称是否真的对应物体部件，以避免“看似合理但实际不存在”的名称幻觉。

## 5.2 Arti-Template 适配问题

> 模板生成的每个 seed 是否保留稳定、准确、可寻址的语义部件名称？

## 5.3 指标

### Name Coverage

[
\text{Name Coverage}
====================

\frac{N_{\mathrm{具有语义名称的part}}}
{N_{\mathrm{全部part}}}
]

以下名称不计为语义名称：

```text
link_0
part_01
mesh_003
geometry_0
object_new
```

### Semantic Precision

[
P_{\mathrm{name}}
=================

\frac{N_{\mathrm{名称正确对应真实部件}}}
{N_{\mathrm{全部命名部件}}}
]

### Semantic Recall

[
R_{\mathrm{name}}
=================

\frac{N_{\mathrm{spec部件被正确命名}}}
{N_{\mathrm{spec要求部件}}}
]

### Functional Naming Richness

不直接照搬 Nova3D 的“named parts/spec parts”，因为大量命名螺丝或装饰几何可能虚增分数。

建议使用：

[
\text{Functional Richness}
==========================

\frac{N_{\mathrm{命名功能部件}}}
{N_{\mathrm{spec功能部件}}}
]

同时报告 extra-real-part count，而不是直接把额外部件全部视为正面。

### Instance Discriminability

检查重复实例是否能被分别寻址：

```text
left_door / right_door
drawer_01 / drawer_02
hinge_01 / hinge_02
```

### Cross-Seed Naming Consistency

同一个组件位在不同 seed 中是否具有一致的语义角色：

[
C_{\mathrm{name}}
=================

\frac{N_{\mathrm{跨seed命名一致的槽位}}}
{N_{\mathrm{全部槽位}}}
]

### Over-Segmentation Rate

一个功能部件被无意义拆成多个 link 的比例。

## 5.4 标注协议

因为你们有 Source Map 和上游部件语义，可以优先采用：

1. 规范化字符串和同义词词典匹配；
2. 与隐藏 gold taxonomy 比较；
3. 仅对无法自动判断的名称使用三个 LLM judges；
4. 抽取 10% 样本由人工复核。

## 5.5 结果表

| Method | Part Exists | Name Coverage | Precision | Recall | Functional Richness | Instance Disc. | Cross-Seed Consistency |
| ------ | ----------: | ------------: | --------: | -----: | ------------------: | -------------: | ---------------------: |

---

# 六、Axis 3：Hierarchy

## 6.1 Nova3D 原指标

Nova3D 强调 Naming 不等于 Hierarchy。一个系统可能输出多个有名字的物体，但它们仍然是扁平列表。

Nova3D 从导出的 glTF scene graph 中确定性读取：

* has tree；
* semantic depth；
* named groups；
* pivots。

它明确承认该指标证明树存在，但不严格证明每个 nesting 决策都语义正确。

## 6.2 Arti-Template 适配优势

你们有冻结的 URDF/组件树真值，因此可以比 Nova3D 更严格，不只测“有没有树”，还可以测“树是否正确”。

## 6.3 指标

### Structural Validity

* single-root rate；
* acyclic rate；
* all-link-reachable rate；
* valid-tree rate。

### Has Hierarchy

不是所有 link 直接挂在世界根节点。

### Semantic Depth

先折叠无语义 exporter wrapper，再计算真实层级深度。

### Named Subassembly Count

例如：

```text
drawer_assembly
door_assembly
hinge_assembly
folding_arm_assembly
```

### Pivot Count

具有真实关节或编辑意义的 pivot 数量。

### Parent-Child Edge F1

[
F1_{\mathrm{edge}}
==================

F1(E_{\mathrm{output}},E_{\mathrm{spec}})
]

### Hierarchy Exact Match

输出树与冻结 spec 完全一致的资产比例。

### Semantic Nesting Accuracy

例如：

* handle 是否挂在 door 下；
* shelf 是否挂在 cabinet body 下；
* wheel 是否挂在 axle assembly 下。

### Cross-Seed Hierarchy Consistency

分两种情况：

* 拓扑固定模板：所有 seed 的 hierarchy 应完全一致；
* 拓扑可变模板：hierarchy 变化必须与组件选择和兼容规则一致。

## 6.4 结果表

| Method | Valid Tree | Depth | Named Groups | Pivots | Edge F1 | Exact Match | Semantic Nesting | Cross-Seed Consistency |
| ------ | ---------: | ----: | -----------: | -----: | ------: | ----------: | ---------------: | ---------------------: |

---

# 七、Axis 4：Constraints

## 7.1 Nova3D 原指标

Nova3D 将 frozen spec 中的约束从最终 GLB 中重新测量，报告：

[
\text{Coverage}
===============

\frac{N_{\mathrm{measurable}}}
{N_{\mathrm{all}}}
]

[
\text{Satisfaction}
===================

\frac{N_{\mathrm{passed}}}
{N_{\mathrm{all}}}
]

[
\text{Conditional Accuracy}
===========================

\frac{N_{\mathrm{passed}}}
{N_{\mathrm{measurable}}}
]

并单独报告 count-constraint pass rate。Nova3D 在 52 条约束中通过 51 条，其中数量约束为 32/32。([arXiv][1])

## 7.2 Arti-Template 约束类型

每个模板至少冻结 8 条约束，覆盖以下类别。

### A. Count Constraints

* 门数量；
* 抽屉数量；
* shelf 数量；
* hinge 数量；
* link 和 joint 数量。

要求 exact match。

### B. Numeric Constraints

* 总体尺寸；
* 部件厚度；
* 轴距；
* joint limit；
* 抽屉行程；
* 开门角度。

建议容差：

[
\tau_{\mathrm{dim}}
===================

\max(2%\text{ target},1\text{ mm})
]

### C. Relational Constraints

例如：

[
w_{\mathrm{panel}}
==================

w_{\mathrm{door}}-2w_{\mathrm{frame}}
]

或：

[
x_{\mathrm{hinge}}
==================

\frac{h_{\mathrm{door}}}{2}
]

从最终几何重新测量变量，并计算 normalized residual。

### D. Interface Constraints

* 安装面是否贴合；
* 轴线是否共线；
* 组件是否位于宿主合法区域；
* gap 是否位于允许范围；
* 组件尺寸是否适配宿主。

### E. Kinematic Constraints

* joint type；
* parent-child；
* axis direction；
* origin；
* lower/upper limits。

### F. Compatibility Constraints

需要同时测试：

* 合法组合是否生成成功；
* 非法组合是否被拒绝；
* 不兼容组合是否意外泄漏到发布资产。

## 7.3 新增模板分布指标

除了 Nova3D 的三个指标，还应增加：

### All-Constraints-Pass Asset Rate

[
R_{\mathrm{all-constraint}}
===========================

\frac{N_{\mathrm{所有约束均通过的资产}}}
{N_{\mathrm{全部资产}}}
]

这比逐条平均 satisfaction 更严格。

### 36-Seed Constraint Reliability

一个模板的 36 个 seed 是否全部满足约束。

### Invalid Combination Rejection Rate

[
R_{\mathrm{reject}}
===================

\frac{N_{\mathrm{正确拒绝的非法组合}}}
{N_{\mathrm{全部非法组合测试}}}
]

## 7.4 结果表

| Method | Coverage | Satisfaction | Conditional Acc. | Count Pass | Numeric Pass | Relational Pass | Interface Pass | Compatibility Pass | All-Pass Assets |
| ------ | -------: | -----------: | ---------------: | ---------: | -----------: | --------------: | -------------: | -----------------: | --------------: |

---

# 八、Axis 5：Editability

## 8.1 Nova3D 原实验

Nova3D 对 18 个资产进行局部编辑，并设置连续 gate：

* artifact 有效；
* target handle 存在；
* source 和 GLB 发生变化；
* hierarchy 保持；
* target fulfilled；
* anchor 正确；
* scale 正确；
* non-target preserved；
* locality preserved；
* final pass。

最终 14/18 编辑成功，而非目标保持和编辑局部性都是 18/18。([arXiv][1])

## 8.2 Arti-Template 的关键扩展

Nova3D 只评价**一个资产的局部编辑**。

你们必须评价两个层次：

1. 单实例编辑；
2. 模板编辑是否传播到整个 seed 分布。

## 8.3 编辑任务设置

选择 18 个模板：

[
6\text{ domains}\times3\text{ difficulty levels}=18
]

每个模板设计三类编辑，共 54 项。

### Edit A：参数编辑

例如：

* 将柜体高度增加 20%；
* 将开门角度从 (90^\circ) 改为 (120^\circ)；
* 将把手宽度增加 15%。

### Edit B：组件替换

例如：

* 将圆形把手替换为杆状把手；
* 将平板门替换为框架门；
* 将普通底座替换为带脚轮底座。

### Edit C：结构编辑

例如：

* 增加一个抽屉；
* 删除一层 shelf；
* 增加一个可折叠 arm；
* 将固定连接改成 revolute joint。

每次编辑后在 16 个固定回归 seed 上重新生成。

## 8.4 指标

### Representation Gates

* template executable；
* edited target addressable；
* source changed；
* output asset changed；
* hierarchy valid。

### Target Fulfillment

目标修改是否真正完成。

### Target Anchor and Scale

目标是否出现在正确位置、尺寸是否满足要求。

### Non-Target Preservation

非目标 part 的：

* 几何；
* hierarchy；
* material；
* joint；
* dimensions；

是否保持。

### Geometry Locality

[
L_{\mathrm{geo}}
================

1-
\frac{\Delta G_{\mathrm{non-target}}}
{\Delta G_{\mathrm{all}}+\epsilon}
]

### Structural Locality

非目标 link、joint 和 parent-child edge 被改变的比例。

### Post-Edit Constraint Pass

编辑后的资产是否仍满足没有被修改的约束。

### Distributional Edit Success

[
R_{\mathrm{distribution-edit}}
==============================

\frac{N_{\mathrm{16个seed均正确传播且通过QC的编辑}}}
{N_{\mathrm{全部编辑任务}}}
]

### Regression Preservation

编辑不应导致旧组件组合失效。

### Edit Cost

* 代码 diff 行数；
* Agent turns；
* wall time；
* token；
* API cost。

## 8.5 人工评测

确定性 gate 之后，使用 3 名具备 3D/机器人背景的盲评员评价：

* target 是否可识别；
* 编辑是否局部；
* 整体是否仍合理。

报告 Fleiss’ kappa 或 Krippendorff’s alpha。

## 8.6 结果表

| Method | Target Fulfilled | Anchor | Scale | Non-Target Preserved | Locality | Constraint Pass | 16-Seed Propagation | Final Pass | Edit Cost |
| ------ | ---------------: | -----: | ----: | -------------------: | -------: | --------------: | ------------------: | ---------: | --------: |

---

# 九、Axis 6：Articulation

## 9.1 Nova3D 原实验

Nova3D 将 Articulation 分为：

1. 是否原生暴露 joint；
2. 关节类型和召回是否正确；
3. rest pose 是否保持；
4. axis 是否位于运动部件；
5. 部件能否无碰撞运动。

它在 12 个资产的 59 个关节上得到 0.955 type accuracy、0.761 recall 和 58/59 几何有效关节，但 29/56 个旋转关节存在过于通用的运动范围。([arXiv][1])

## 9.2 Arti-Template 三层评测

### Tier 1：Native Articulation Capability

对 54 个 canonical seed 报告：

* articulable asset rate；
* joints per asset；
* native joint exposure rate；
* readiness pivots per asset。

### Tier 2：Spec Correctness

从冻结 spec 比较：

* joint type accuracy；
* joint recall；
* extra joint count；
* parent-child accuracy；
* axis orientation error；
* axis origin error；
* joint limit error；
* required/optional joint recall；
* generic-range rate。

### Tier 3：Full-Range Functional Validity

对全部 1,944 个 seed：

* 每个单关节均匀采样 11 个状态；
* 多关节资产额外采样 64 个 Sobol joint configurations；
* 对低 clearance 区间加密采样；
* 条件允许时使用 continuous collision detection。

## 9.3 指标

### Rest Pose Frozen

加入或修改 articulation 后，初始资产几何是否保持。

### Axis-on-Moving-Part Rate

轴是否位于正确的可动部件或安装接口。

### Joint-Level Geometric Validity

[
R_{\mathrm{joint-valid}}
========================

\frac{N_{\mathrm{轴正确且全行程无非法碰撞的joint}}}
{N_{\mathrm{全部joint}}}
]

### Asset-Level Geometric Validity

资产中所有 joint 都有效才算通过：

[
R_{\mathrm{asset-valid}}
========================

\frac{N_{\mathrm{全部joint均有效的资产}}}
{N_{\mathrm{全部资产}}}
]

### Full-Range Collision-Free Rate

### Invalid Joint-State Ratio

[
R_{\mathrm{invalid-state}}
==========================

\frac{N_{\mathrm{碰撞或非法状态}}}
{N_{\mathrm{全部测试状态}}}
]

### Max Penetration Depth

### Minimum Clearance

### Joint-Limit Reachability

joint 是否能到达上下限，而不被错误 collision geometry 阻挡。

### Generic-Range Rate

例如 revolute joint 统一使用 ([-180^\circ,180^\circ]) 或近似自由旋转，应作为问题单独统计。

### AOR

建议同时报告：

* sibling bbox AOR；
* sibling exact-mesh AOR；
* all-part exact-mesh AOR。

## 9.4 结果表

| Method | Articulable | Joints/Asset | Type Acc. | Joint Recall | Edge Acc. | Axis Valid | Limit Valid | Joint Geom. Valid | Asset Geom. Valid | Generic Range ↓ |
| ------ | ----------: | -----------: | --------: | -----------: | --------: | ---------: | ----------: | ----------------: | ----------------: | --------------: |

---

# 十、Axis 7：Production Readiness

## 10.1 Nova3D 原指标

Nova3D 的 Production Readiness 不是“最终艺术资产质量”，而是源代码表示保留的生产 affordance，包括：

* per-part primitive residual；
* UV readiness；
* material intent；
* compact source。

它报告 code-native part 的 primitive surface 更干净，且在同一资产的代码几何与 baked GLB 对照中，源几何产生的 UV islands 显著更少。

## 10.2 为什么不能完全照搬

你们的目标是机器人和仿真资产，不是主要面向贴图制作。因此应保留 Nova3D 的“源代码与几何生产性”，但把核心改成：

> Robotics Production Readiness。

当前项目已有的检测显示，visual/collision geometry 完全一致，但惯量完备率约 13.6%、joint dynamics 约 23.6%、水密率约 50%，且当前材质为纯色而非贴图资产。因此这些维度必须拆开报告，不能笼统宣称 production-ready。

## 10.3 Geometry Integrity

* watertight rate；
* manifold rate；
* open-edge count；
* degenerate-face rate；
* self-intersection rate；
* empty-mesh rate；
* triangle count；
* mesh size。

## 10.4 Source Compactness

* template source LOC；
* template source KB；
* URDF size；
* mesh package size；
* source size per functional part；
* source size per generated seed。

这里应同时报告模板摊销：

[
\text{Source bytes per asset}
=============================

\frac{\text{template source bytes}}
{N_{\text{可生成有效资产}}}
]

## 10.5 Packaging Readiness

* missing dependency rate；
* relative-path portability；
* self-contained package rate；
* deterministic rebuild rate；
* clean-environment rebuild success；
* versioned manifest completeness。

## 10.6 Semantic and Physical Metadata Completeness

| 元数据                    | 指标  |
| ---------------------- | --- |
| Part names             | 完备率 |
| Hierarchy              | 完备率 |
| Joint type/axis/limit  | 完备率 |
| Metric scale           | 完备率 |
| Visual mesh            | 完备率 |
| Collision mesh         | 完备率 |
| Material label         | 完备率 |
| Mass                   | 完备率 |
| Inertia                | 完备率 |
| Joint damping/friction | 完备率 |

不要把这些列平均成一个分数，至少分成：

* semantic completeness；
* kinematic completeness；
* physical completeness。

## 10.7 Collision Representation Readiness

### Visual–Collision Deviation

对 visual surface (P) 和 collision surface (Q)：

[
E_{Q\rightarrow P}
==================

\frac{1}{|Q|}
\sum_{q\in Q}
\min_{p\in P}|q-p|
]

[
E_{P\rightarrow Q}
==================

\frac{1}{|P|}
\sum_{p\in P}
\min_{q\in Q}|p-q|
]

目前 exact collision 与 visual mesh 一致时，两项应接近 0。

但还应分别测试：

* exact visual mesh；
* single convex hull；
* CoACD。

因为 exact mesh 的几何保真高，并不自动代表所有仿真器的加载效率和动力学稳定性最好。

## 10.8 UV 与材质指标的处理

可以作为辅助列报告：

* UV coverage；
* number of UV islands；
* overlap-free UV rate；
* material slot completeness；
* texture presence rate。

但不建议把它们作为主要贡献，因为当前项目不主打 PBR 和纹理真实度。

## 10.9 结果表

| Method | Watertight | Manifold | Open Edges ↓ | Self-Intersection ↓ | Source KB ↓ | Portable Package | Deterministic Build | Semantic Complete | Kinematic Complete | Physical Complete |
| ------ | ---------: | -------: | -----------: | ------------------: | ----------: | ---------------: | ------------------: | ----------------: | -----------------: | ----------------: |

---

# 十一、Nova3D 风格的 Evaluation Coverage Matrix

建议在论文中明确放一张类似 Nova3D Table 3 的表：

| Axis                 | 方法                                  | 数据范围                         | Metric Source                            | 该轴不证明什么          |
| -------------------- | ----------------------------------- | ---------------------------- | ---------------------------------------- | ---------------- |
| Reliability          | Full、naive、消融                       | 54 tasks × 3 runs，54×36 seed | 执行、导出、QC日志                               | 不证明语义和外观质量       |
| Naming               | Full、naive、Original Articraft       | 1,944 assets                 | URDF names、gold taxonomy、semantic judges | 不证明几何分割边界        |
| Hierarchy            | Full、naive、Original Articraft       | 1,944 assets                 | URDF tree 与冻结 spec                       | 不证明动力学稳定         |
| Constraints          | Full、naive、消融                       | 全部冻结约束                       | 最终 URDF/mesh 实测                          | 不证明视觉真实感         |
| Editability          | Full、naive；Original Articraft 仅实例编辑 | 54 edits × 16 seeds          | 确定性 gates + 盲评                           | 不证明通用 3D 编辑 SOTA |
| Articulation         | Full、naive、Original Articraft       | 54 canonical + 1,944 seeds   | URDF、CCD、运动扫掠                            | 不证明质量、摩擦真实       |
| Production Readiness | 结构化代码方法                             | 1,944 assets                 | mesh/URDF/source telemetry               | 不证明艺术家最终品质       |

---

# 十二、建议最终形成三张主表

## Table A：Reliability and Native Structure

| Method | Template Exec. | Seed QC | Naming P/R | Instance Disc. | Tree Exact | Edge F1 | Pivots |
| ------ | -------------: | ------: | ---------: | -------------: | ---------: | ------: | -----: |

## Table B：Constraints, Editability and Articulation

| Method | Constraint Sat. | All-Pass Asset | Edit Pass | 16-Seed Edit | Joint Type | Joint Recall | Full-Range Valid |
| ------ | --------------: | -------------: | --------: | -----------: | ---------: | -----------: | ---------------: |

## Table C：Production Readiness

| Method | Watertight | Manifold | Portable | Deterministic | Semantic Meta. | Kinematic Meta. | Physical Meta. | Source KB |
| ------ | ---------: | -------: | -------: | ------------: | -------------: | --------------: | -------------: | --------: |

---

# 十三、这组实验最终证明什么

这一部分实验应该形成一条和 Nova3D 相似、但更适合模板论文的因果链：

1. **Reliability**：系统不是只偶然生成一个可运行程序，而是稳定构建模板；
2. **Naming + Hierarchy**：模板保留了可寻址的语义部件和真实装配结构；
3. **Constraints**：参数、组件和接口要求能在最终资产中被测量并满足；
4. **Editability**：模板可以被局部修改，并将修改正确传播到整个资产分布；
5. **Articulation**：所有 seed 的关节结构和全行程运动持续有效；
6. **Production Readiness**：输出具有可部署的源代码、包结构、几何和仿真元数据。

最合适的论文表达是：

> **Nova3D-Bench verifies whether a generated asset is programmable. Our adaptation evaluates whether an asset-specific program can be lifted into a reusable procedural template while preserving programmability across an entire generated distribution.**

这七个轴是“模板可编程性实验”；类内多样性、coverage、长尾、吞吐量、重复率、多仿真器和物理稳定性应继续作为独立的“分布与 Sim-Ready 实验”，不要全部塞进 Production Readiness。

[1]: https://arxiv.org/pdf/2607.22738v1 "Nova3D: Code-Native Generation of Programmable 3D Assets"

---

# 十四、实验结果表（待填写）

以下表格只定义实验结果的字段，当前不填入任何结果。

## Table 1：Reliability

| Method | Template Exec. | Artifact Saved | First-shot | Final Success | Seed Compile | Seed QC | 36/36 Pass | Corner Pass | Regression Retention | Repair Turns | Wall Time | Tokens | API Cost |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|

## Table 2：Naming

| Method | Part Exists | Name Coverage | Semantic Precision | Semantic Recall | Functional Richness | Instance Discriminability | Cross-Seed Consistency | Over-Segmentation Rate |
|---|---:|---:|---:|---:|---:|---:|---:|---:|

## Table 3：Hierarchy

| Method | Valid Tree | Has Hierarchy | Semantic Depth | Named Groups | Pivots | Parent-Child Edge F1 | Hierarchy Exact Match | Semantic Nesting Accuracy | Cross-Seed Consistency |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|

## Table 4：Constraints

| Method | Coverage | Satisfaction | Conditional Accuracy | Count Pass | Numeric Pass | Relational Pass | Interface Pass | Kinematic Pass | Compatibility Pass | All-Pass Assets | Invalid Combination Rejection |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|

## Table 5：Editability

| Method | Target Fulfilled | Anchor | Scale | Non-Target Preserved | Geometry Locality | Structural Locality | Post-Edit Constraint Pass | 16-Seed Propagation | Regression Preservation | Final Pass | Edit Cost |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|

## Table 6：Articulation

| Method | Articulable | Joints/Asset | Native Joint Exposure | Joint Type Accuracy | Joint Recall | Parent-Child Accuracy | Axis Valid | Origin Valid | Limit Valid | Joint Geom. Valid | Asset Geom. Valid | Full-Range Collision-Free | Generic Range |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|

## Table 7：Production Readiness

| Method | Watertight | Manifold | Open Edges | Degenerate Faces | Self-Intersection | Source KB | URDF KB | Mesh KB | Portable Package | Deterministic Build | Semantic Complete | Kinematic Complete | Physical Complete |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|

## Table 8：Asset-Level Pilot Audit

| Asset ID | Source | Difficulty | Seed | Executable | Artifact Saved | Valid URDF | Valid Tree | Raw Name Coverage | Joint Metadata | Package Complete | Physics Validation |
|---|---|:---:|---:|---:|---:|---:|---:|---:|---:|---:|---|

## Table 9：Evaluation Coverage Matrix

| Axis | Method | Data Range | Metric Source | Required Gold/Spec | Current Availability | What the Axis Does Not Prove |
|---|---|---|---|---|---|---|

## Table 10：Method Coverage

| Method | Template Count | Generation Runs | Seed Count | Reliability | Naming | Hierarchy | Constraints | Editability | Articulation | Production Readiness |
|---|---:|---:|---:|---|---|---|---|---|---|---|

## Table 11：Failure Taxonomy

| Failure ID | Axis | Method | Template/Asset | Seed | Failure Stage | Failure Type | Error Summary | Repair Attempt | Final Status |
|---|---|---|---|---:|---|---|---|---:|---|

## Table 12：Resource and Cost

| Method | Template Runs | Agent Turns | Compile/Probe Count | Wall Time | Input Tokens | Output Tokens | Total Tokens | API Cost | Source LOC | Source Size |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
