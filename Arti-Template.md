# Arti-Template 实验设计方案

## 1. 实验总体目标

Articraft 的实验主要证明两件事：其 Agent 系统能生成质量较好的单个资产，以及 Articraft-10K 能提升 Particulate 的下游性能。你们的工作不能只复现这两类实验，因为你们的核心对象已经从“单个资产”变成了“可复用程序化模板”。

建议整篇论文围绕以下五个研究问题展开：

### RQ1：模板化管线是否有效？

验证 Source Map、组件位抽象、候选组件、派生约束、接口适配和回归测试是否真的提高模板生成成功率。

### RQ2：模板是否能稳定生成整个资产分布？

不能只展示几个成功案例，而要验证不同 seed、不同组件组合和边界参数下的有效率。

### RQ3：模板资产是否具备可靠的运动学和仿真质量？

重点检查全行程穿模、悬空部件、碰撞几何、物理属性和跨仿真器兼容性。

### RQ4：模板生成的数据是否真正多样、可控且可复现？

验证它不是对同一资产做尺寸缩放，而是产生真实的组件、拓扑和几何变化。

### RQ5：模板扩展数据是否能提升下游模型？

沿用 Articraft 的 Particulate 实验，但增加数据量控制和多样性控制，证明收益来自模板分布，而不只是增加样本数量。

---

# 2. 论文应建立的核心结论

建议将论文的核心结论明确写成：

> 相比逐资产生成，Arti-Template 将资产特定的 Articraft 程序自动提升为类别级程序化模板，通过组件抽象、参数化、跨组件约束和分布级回归测试，在近乎零边际生成成本下产生大规模、可控、运动学有效的可动资产分布。

需要用实验分别支撑四个关键词：

| 论文主张    | 对应实验               |
| ------- | ------------------ |
| 自动提升为模板 | 管线消融实验             |
| 大规模和多样  | 组合域、图多样性、非重复率      |
| 可控且机械有效 | 属性命中率、AOR、运动成功率    |
| 对学习有价值  | Particulate 数据增强实验 |

---

# 3. 统一评测数据协议

## 3.1 模板与类别统计必须分开

当前文档中存在“537 类”和“537 个模板”混用的情况。正式论文中必须分别报告：

* `# Categories`
* `# Templates`
* 每类模板数量
* 每模板组件位数量
* 每组件位候选组件数
* 每模板连续参数维度
* 每模板离散组合数

否则审稿人会质疑“537 个模板是否真的等于 537 个语义类别”。

---

## 3.2 标准化 Seed 套件

建议为每个模板固定一套 **36-seed 标准测试集**：

### 常规覆盖 Seed：32 个

* 离散候选组件采用 pairwise covering array；
* 连续参数采用 Latin Hypercube Sampling；
* 确保每个候选组件至少出现若干次；
* 确保主要跨组件组合被覆盖。

### 边界 Seed：4 个

至少包含：

1. 所有连续参数接近最小值；
2. 所有连续参数接近最大值；
3. 最紧凑、最容易发生碰撞的组合；
4. 最大展开、最容易失稳的组合。

如果所有 537 个模板都使用 36 个 seed，则轻量级评测总规模为：

[
537\times36=19,332
]

个资产。

这个规模适合运行：

* 编译与导出；
* 静止态碰撞；
* 运动学扫掠；
* 组合和多样性统计；
* 参数命中率；
* 确定性复现测试。

---

## 3.3 高成本仿真子集

跨仿真器和长时间物理仿真不必对全部资产运行。

建议分层抽取：

* 120 个模板；
* 每模板 10 个 seed；
* 共 1,200 个资产。

抽样时按以下属性分层：

* revolute、prismatic、continuous、mimic；
* 低部件数、中部件数、高部件数；
* 单关节和多关节；
* 空心与实心结构；
* 水密与非水密 mesh；
* 普通资产与闭链/复杂资产；
* 不同类别和尺寸范围。

---

## 3.4 防止筛选偏差

必须同时报告：

### 原始生成通过率

[
R_{\mathrm{raw}}
================

\frac{N_{\mathrm{通过全部闸门}}}
{N_{\mathrm{实际生成}}}
]

### 发布集质量

只在通过闸门的资产上计算最终 AOR、稳定率等。

不能只说：

> 发布资产 AOR 为 0，人工接受率为 100%。

因为这是经过筛选后的必然结果。应当同时报告：

* 闸门前错误率；
* 闸门淘汰率；
* 修复成功率；
* 闸门后的最终质量。

这样才能证明 Harness 本身有效，而不是简单删除所有失败资产。

---

# 4. 实验一：模板生成管线消融

## 4.1 实验目的

验证每个管线模块是否必要，回答：

> 为什么不能直接让 LLM 把 Articraft 的 `model.py` 改成带参数的模板？

## 4.2 实验数据

从全部模板中分层选择 120 个上游 Articraft 程序：

| 难度 | 定义                     | 数量 |
| -- | ---------------------- | -: |
| 简单 | ≤3 个 link，单关节          | 40 |
| 中等 | 4–7 个 link，多组件位        | 40 |
| 复杂 | ≥8 个 link、mimic、闭链或强耦合 | 40 |

固定：

* 同一个 LLM；
* 相同模型版本；
* 相同推理强度；
* 相同 token 和工具预算；
* 相同输入源文件；
* 相同 36-seed 测试集。

## 4.3 对照组

| 设置               | Source Map | 槽位抽象 | 派生/接口约束 | 多 Seed Harness | 回归测试 |
| ---------------- | ---------: | ---: | ------: | -------------: | ---: |
| A0：Vanilla Agent |          ✗ |    ✗ |       ✗ |              ✗ |    ✗ |
| A1               |          ✓ |    ✗ |       ✗ |              ✗ |    ✗ |
| A2               |          ✓ |    ✓ |       ✗ |              ✗ |    ✗ |
| A3               |          ✓ |    ✓ |       ✓ |              ✗ |    ✗ |
| A4               |          ✓ |    ✓ |       ✓ |              ✓ |    ✗ |
| A5：Full          |          ✓ |    ✓ |       ✓ |              ✓ |    ✓ |

还可以增加：

* Full without corner seeds；
* Full without cross-component constraints；
* Full without compatibility filtering。

## 4.4 指标

### 模板完成率

[
R_{\mathrm{template}}
=====================

\frac{N_{\mathrm{成功产生可运行模板}}}
{N_{\mathrm{输入源程序}}}
]

### Seed 有效率

[
R_{\mathrm{seed}}
=================

\frac{N_{\mathrm{通过全部机械检查的seed}}}
{N_{\mathrm{生成seed}}}
]

### 旧 Seed 回归保持率

每次修复新 seed 后，重新测试历史 seed：

[
R_{\mathrm{reg}}
================

1-
\frac{N_{\mathrm{旧seed重新失败}}}
{N_{\mathrm{旧seed总数}}}
]

### 人工介入率

[
R_{\mathrm{human}}
==================

\frac{N_{\mathrm{需要人工修改的模板}}}
{N_{\mathrm{模板总数}}}
]

### 其他指标

* 平均修复轮数；
* 平均 compile 次数；
* 平均 probe 次数；
* 每模板 LLM 成本；
* 总生成时间；
* 36-seed 全通过模板比例；
* corner-seed 通过率；
* 修复新 seed 后破坏旧 seed 的次数。

## 4.5 预期结论

这张表要证明：

1. Source Map 提高组件识别和代码定位能力；
2. 槽位抽象提高可参数化和可组合能力；
3. 跨组件派生约束减少几何断裂与错误装配；
4. 多 seed Harness 提高资产分布有效率；
5. 回归测试显著降低“修好新 seed、破坏旧 seed”的问题。

这是你们方法贡献最核心的一张消融表。

---

# 5. 实验二：模板级规模与生成经济性

## 5.1 数据规模统计

建议制作类似 Infinigen-Sim 和 Artiverse 的数据集属性表：

| 指标               | 统计方式                            |
| ---------------- | ------------------------------- |
| 类别数              | 独立语义类别                          |
| 模板数              | 可执行程序化生成器数量                     |
| 组件位数量            | 所有模板槽位总数与均值                     |
| 候选组件数            | 每槽位候选数量分布                       |
| 连续参数维度           | 每模板可采样实数参数数量                    |
| 离散组合数            | 各槽位候选数量乘积，考虑兼容约束                |
| 估计唯一资产数          | 离散组合数 × 有效连续参数分辨率               |
| link 数量          | 均值、中位数和区间                       |
| joint 数量         | revolute/prismatic/continuous 等 |
| 多 DoF 或 mimic 数量 | 单独统计                            |
| 语义和运动学标签         | 是否构造即真值                         |
| 每部件材料、质量、惯量      | 完备率                             |

当前管线已具备较强的类别和组合域优势，但物理属性仍是明显短板：文档记录的惯量完备率为 13.6%，关节 dynamics 完备率为 23.6%，水密率为 50%。因此在物理实验前，应先完成质量、惯量和关节 dynamics 自动补全。

---

## 5.2 成本指标

不能简单拿一次模板生成成本和 Articraft 的单资产成本比较，应使用摊销成本：

[
C_{\mathrm{asset}}
==================

\frac{
C_{\mathrm{模板生成}}
+
C_{\mathrm{模板修复}}
}{
N_{\mathrm{有效资产}}
}
+
C_{\mathrm{seed编译}}
]

分别报告：

* 模板初次生成成本；
* 模板修复成本；
* 单 seed CPU 编译时间；
* 单有效资产摊销成本；
* 生成 100、1K、10K、100K 个资产时的平均成本；
* 相比 Articraft 逐资产生成的成本下降比例。

Articraft 的成本指标约为每个生成结果 1.14 美元，并且每个资产都需要独立 Agent 运行；你们的核心优势应当是模板一次生成、后续 seed 不再调用 LLM。

---

# 6. 实验三：几何与运动学合法性

这是你们最应该主打的实验。

## 6.1 评测对象

对全部约 19,332 个标准 seed 资产运行。

## 6.2 静止状态指标

* URDF 解析成功率；
* mesh 文件存在率；
* 空 mesh 比例；
* 静止态部件穿模率；
* 悬空部件率；
* 非预期断连率；
* 非法 joint limit 比例；
* 零长度 joint axis 比例；
* parent-child 环路错误率。

## 6.3 全行程运动检查

对于单自由度关节：

* 在上下限之间均匀采样 11 个状态；
* 逐关节独立扫掠；
* 其他关节保持 rest pose。

对于多关节资产：

* 除逐关节扫掠外，再使用 64 个 Sobol joint configurations；
* 覆盖多个关节同时运动的情况。

报告：

### Full-range collision-free rate

[
R_{\mathrm{motion}}
===================

\frac{N_{\mathrm{全行程无碰撞资产}}}
{N_{\mathrm{测试资产}}}
]

### 最大穿透深度

[
D_{\max}
========

\max_{s,i,j}D_{\mathrm{penetration}}(P_i(s),P_j(s))
]

### 碰撞状态比例

[
R_{\mathrm{collision\ state}}
=============================

\frac{N_{\mathrm{发生非预期碰撞的关节状态}}}
{N_{\mathrm{全部测试关节状态}}}
]

### AOR

同时报告：

* CAGE 原始 bbox/sibling AOR；
* sibling 精确网格 AOR；
* all-part-pair 精确网格 AOR；
* mean AOR；
* max AOR；
* AOR 小于数值容差 (10^{-6}) 的资产比例。

你们目前的闸门比 CAGE 的 sibling/bbox 口径更严格，因此建议不要只写“AOR=0”，而是展示以下对比：

| 设置             | 静止穿模率 | Mean AOR | Max AOR | 全行程通过率 |
| -------------- | ----: | -------: | ------: | -----: |
| QC 前原始 seed    |       |          |         |        |
| 仅静止态 QC        |       |          |         |        |
| 仅离散关节状态 QC     |       |          |         |        |
| Full motion QC |       |          |         |        |

这能证明运动 QC 的贡献，而不是把零 AOR 描述成发布集筛选后的结果。

---

# 7. 实验四：碰撞表示与几何保真度

当前资产使用 visual mesh 作为 collision mesh，视觉—碰撞几何偏差理论上为零；但文档也指出，28% 的 mesh 存在凸包体积与实体体积比超过 2 的情况，因此单凸包会显著膨胀，精确网格的无碰撞结论不一定能迁移到使用凸近似的引擎。

建议将此风险独立做成一个实验。

## 7.1 三种碰撞表示

对 1,200 个仿真子集资产构建：

1. **Exact Mesh**：visual mesh 直接作为 collision；
2. **Single Convex Hull**：每个 part 一个凸包；
3. **CoACD**：每个 part 使用凸分解。

## 7.2 指标

### 碰撞—视觉几何偏差

参考 Real-IKEA：

[
E_{Q\rightarrow P}
==================

\frac{1}{|Q|}
\sum_{q\in Q}\min_{p\in P}|q-p|
]

[
E_{P\rightarrow Q}
==================

\frac{1}{|P|}
\sum_{p\in P}\min_{q\in Q}|p-q|
]

其中：

* (P) 为 visual surface；
* (Q) 为 collision surface；
* (E_{Q\rightarrow P}) 主要衡量碰撞体向外膨胀。

还应报告：

* AOR；
* max penetration；
* 碰撞组件数量；
* 加载时间；
* 仿真步耗时；
* 接触点数量；
* 内存消耗；
* 各仿真器加载成功率。

## 7.3 预期价值

该实验可以形成一条很强的结论：

> 精确碰撞网格提供最高几何保真和最低运动穿模；单凸包虽然计算便宜，但会破坏空心结构和关节间隙；CoACD 在效率和保真度之间提供折中。

即使 Exact Mesh 在部分引擎中效率较低，这也是可信的实验结果，不能回避。

---

# 8. 实验五：物理属性与静态稳定性

## 8.1 前置条件

在正式运行前，应把：

* per-part mass；
* inertia tensor；
* center of mass；
* joint damping；
* joint friction；

补全到接近 100%。

对于空心或非水密部件，不能直接使用凸包体积计算质量，否则会严重高估抽屉、柜体等结构的质量。应按照以下优先级计算体积：

1. 程序化基元使用解析体积；
2. 水密 mesh 使用封闭体积；
3. 已知空心参数的部件使用外体积减内腔体积；
4. 薄壳结构使用表面积乘厚度；
5. 无法可靠估计的 part 单独标记，而不是静默使用凸包体积。

## 8.2 物理完备性指标

* mass 完备率；
* inertia 完备率；
* material 完备率；
* joint dynamics 完备率；
* 非正质量比例；
* 非正定惯量比例；
* 惯量三角不等式违规率；
* 密度超出材料合理范围比例。

## 8.3 重力稳定性实验

在 MuJoCo、Genesis 等仿真器中：

1. 将资产放置在水平地面；
2. 根部使用 free joint；
3. 仿真 10 秒；
4. 不施加外力和控制。

稳定判据建议设为：

* 根节点位移小于包围盒对角线的 1%；
* roll/pitch 变化小于 (5^\circ)；
* 没有 NaN、爆炸或高速发散；
* 被动关节漂移小于其全行程的 2%。

报告：

* `%Stable`；
* 平衡后 rotation angle；
* 根部平移；
* 最大关节漂移；
* 仿真异常率。

## 8.4 最坏关节状态稳定性

这是你们可以新增、比已有论文更严格的指标。

对每个资产采样：

* rest pose；
* 每个关节 lower limit；
* 每个关节 upper limit；
* 64 个组合关节状态。

在每个状态下释放资产，取最差结果：

[
S_{\mathrm{worst}}
==================

\min_{q\in\mathcal Q}S(q)
]

报告：

* Rest-pose stable rate；
* Worst-articulated-state stable rate；
* 最大倾角；
* 最大质心偏移；
* 最容易失稳的关节状态。

这可以检查柜门完全打开、抽屉完全拉出后，物体是否倾覆。

## 8.5 几何稳定性闭式指标

同时计算：

* 质心投影是否落在支撑接触凸包内；
* 归一化支撑裕度；
* 理论倾覆角。

这一指标不依赖仿真器，可用于解释仿真稳定性的原因。

---

# 9. 实验六：跨仿真器 Sim-Readiness

建议使用四个引擎：

* MuJoCo；
* Genesis；
* PyBullet；
* Isaac Sim。

每个引擎分五级测试，不要只统计“文件能否打开”。

| Level                | 判定                       |
| -------------------- | ------------------------ |
| L1 Parse             | URDF 能被解析                |
| L2 Instantiate       | 模型能被创建，mesh/材质路径无误       |
| L3 First-step        | 执行一个 simulation step 无异常 |
| L4 Passive stability | 无控制仿真 10 秒稳定             |
| L5 Articulation      | 所有关节能在限位内完成运动            |

分别报告每一级成功率，以及：

* joint 数量是否保持一致；
* joint type 是否保持一致；
* joint limits 是否保持一致；
* root pose 是否一致；
* 质量和惯量是否正确读取；
* collision element 数量；
* 错误类型分布。

文档中只有 MuJoCo 的 40/40 解析结果，不能据此提前声称跨仿真器兼容。

---

# 10. 实验七：多样性与组合能力

## 10.1 组合域规模

每个模板报告：

* 连续参数维度 (d_c)；
* 离散槽位数量 (d_d)；
* 每槽候选数量 (n_i)；
* 兼容约束前组合数；
* 兼容约束后有效组合数。

[
N_{\mathrm{discrete}}
=====================

\left|\left{
(c_1,\ldots,c_k):
\text{compatibility}(c_1,\ldots,c_k)=1
\right}\right|
]

不要直接把所有候选数相乘，如果模板中存在兼容限制。

连续参数的“唯一数量”不能无限夸大，建议定义最小有效变化：

> 某个参数变化后，资产几何的归一化 Chamfer Distance 或尺寸变化超过预设阈值，才认为产生了一个可区分实例。

最终报告：

* 中位数组合数；
* 总组合域；
* (\log_{10}) 估计资产容量；
* 每模板有效参数维度；
* 只有尺寸变化的模板比例；
* 具有组件变化的模板比例；
* 具有拓扑变化的模板比例。

---

## 10.2 运动学图 Perplexity

将每个 seed 的运动学结构转化为规范图：

* 节点：link；
* 节点标签：语义角色或 part type；
* 边：parent-child；
* 边标签：joint type；
* 可选属性：axis 类别、mimic 关系。

使用图同构规范化或 Weisfeiler–Lehman hash 得到图类型 (g)。

[
H(G)
====

-\sum_g p(g)\log p(g)
]

[
\mathrm{Perplexity}(G)
======================

\exp(H(G))
]

分别报告：

* 全库图 perplexity；
* 类别内图 perplexity；
* 模板内图 perplexity；
* 独特运动学图数量；
* 拓扑变化模板比例。

必须同时报告“拓扑变化模板比例”，否则大量拓扑固定模板会让 perplexity 难以解释。

---

## 10.3 非重复率

建议使用两层重复检测：

### 精确重复

比较：

* URDF mechanical hash；
* mesh 文件 hash；
* 运动学图 hash；
* 参数配置 hash。

### 几何近重复

将资产表面归一化采样为点云，使用：

* 3D shape embedding；
* 或 Chamfer Distance；

判断不同 seed 是否只是几乎相同的形状。

报告：

* exact duplicate rate；
* geometric near-duplicate rate；
* structural unique rate；
* topology unique rate。

---

# 11. 实验八：条件可控性

这是模板方法比逐资产生成模型更有优势的部分。

## 11.1 目标属性

选择能够自动测量的属性：

* 整体宽度、高度、深度；
* 门数量；
* 抽屉数量；
* shelf 数量；
* link 数量；
* revolute/prismatic joint 数量；
* 开合角度；
* 抽屉最大行程；
* 某组件的候选类型。

## 11.2 实验协议

对于每个属性：

1. 在允许范围内随机采样目标值；
2. 由模板生成资产；
3. 从最终 URDF 和 mesh 重新测量实际值；
4. 不直接读取输入参数作为结果。

## 11.3 指标

### 命中率

[
R_{\mathrm{hit}}
================

\frac{
N(|y-\hat y|\leq \tau)
}{
N
}
]

### 归一化平均误差

[
\mathrm{NMAE}
=============

\frac{1}{N}
\sum_i
\frac{|y_i-\hat y_i|}
{y_{\max}-y_{\min}}
]

### 单调响应性

对连续参数报告 Spearman 相关系数：

[
\rho(y,\hat y)
]

### 参数失效率

参数变化但最终几何基本不变的比例。

最终表格建议按：

* 尺寸属性；
* 组件数量属性；
* 关节属性；
* 离散组件选择；

分别报告，而不是合成一个总分。

---

# 12. 实验九：确定性与可复现性

随机选取 1,000 个 template-seed 对，每个重复编译 5 次。

检查：

* URDF 文本 hash；
* mesh hash；
* mechanical graph hash；
* part pose；
* joint axis 和 limit；
* QC 输出。

定义：

[
R_{\mathrm{reproduce}}
======================

\frac{
N_{\mathrm{五次结果完全一致}}
}{
N_{\mathrm{测试template-seed}}
}
]

同时报告：

* bit-identical rate；
* mechanical-identical rate；
* floating-point tolerance 内的一致率。

如果 Blender 或第三方库导致文件顺序不稳定，可以将“机械结构一致”和“字节完全一致”分开报告。

---

# 13. 实验十：人工结构质量评测

不建议照搬 Articraft 的大众用户外观偏好实验，因为你们当前材质全部为纯色、没有贴图，mesh 面片中位数也较低，在视觉真实感方向明显不占优势。

如果需要人工实验，应只评价你们真正关心的内容。

## 13.1 参与者

* 3–5 名具备 3D、机器人或机械背景的评审；
* 每个样本至少 3 人评价；
* 盲测，不显示方法名称。

## 13.2 展示形式

每个资产展示：

* 静止状态多视图；
* 完整关节运动视频；
* 3–5 个不同 seed；
* 组件替换前后结果。

## 13.3 评分维度

每项 1–5 分：

* 类别和部件完整性；
* 组件安装合理性；
* 几何连续性；
* 关节位置和方向合理性；
* 全行程运动合理性；
* 组件组合后的整体协调性。

报告：

* 各维度平均分；
* 4 分及以上比例；
* Krippendorff’s alpha 或 Fleiss’ kappa；
* QC 前后评分变化；
* Full 与消融版本评分差异。

不要将“已通过人工关口的发布集”直接报告成 100% Human Acceptance。应同时给出原始签核通过率和淘汰率。

---

# 14. 实验十一：Particulate 下游数据价值

这是最重要的下游实验，也最贴合 Articraft 的现有证据链。Articraft 使用生成数据扩充 Particulate 训练集，并在独立的 Lightwheel benchmark 上评测 rest-pose 和 articulated geometry。

## 14.1 训练组

所有组使用：

* 相同 Particulate 模型；
* 相同训练轮数；
* 相同 batch size；
* 相同优化器；
* 相同数据增强；
* 3 个训练随机种子。

建议设置：

| 组别 | 训练数据                          | 目的                |
| -- | ----------------------------- | ----------------- |
| P0 | 原始 Particulate 数据             | 基线                |
| P1 | P0 + 10K Articraft-10K        | 复现 Articraft      |
| P2 | P0 + 10K Ours                 | 与 Articraft 等量比较  |
| P3 | P0 + 10K Ours-LowDiversity    | 控制数据量，只降低模板/组合多样性 |
| P4 | P0 + 30K Ours                 | 数据规模曲线            |
| P5 | P0 + 10K Articraft + 10K Ours | 检查互补性             |

### Ours-LowDiversity 的构造

样本数与 P2 完全相同，但只从少量模板或少量组件组合中重复采样。

这样：

[
P2-P3
]

反映的是**多样性贡献**，而不只是数据量贡献。

[
P4-P2
]

反映的是数据规模增长。

[
P2-P1
]

反映模板生成数据与 Articraft 单实例数据之间的差异。

## 14.2 外部评测

继续使用 Lightwheel，不在自己的模板数据上做主结果。

指标沿用 Articraft：

### Rest-Pose Segmentation

* gIoU ↑；
* PC ↓；
* mIoU ↑。

### Articulated Geometry

* articulated gIoU ↑；
* articulated PC ↓；
* whole-object Chamfer Distance，OC ↓。

同时拆分：

* 原训练集已见类别；
* 未见类别；
* 小型可动部件；
* 单关节资产；
* 多关节资产。

## 14.3 统计方法

* 每组训练 3 次；
* 报告 mean ± std；
* 对测试对象做 paired bootstrap 95% CI；
* 比较 P1 与 P2、P2 与 P3 时进行配对显著性检验；
* 同时报告逐类别变化，不能只报平均值。

## 14.4 期望支撑的结论

最理想的结果是：

1. P2 优于 P0：模板数据有用；
2. P2 优于 P3：收益来自结构和组合多样性；
3. P4 继续提升：数据规模具有扩展性；
4. P5 最优：你们的数据与 Articraft-10K 互补；
5. 未见类别和未见结构上的提升大于已见类别。

---

# 15. 实验十二：组合泛化专项实验

这是最能体现“slot 和 candidate component”价值的新实验。

## 15.1 数据选择

选择至少 50 个满足以下条件的模板：

* 至少 2 个可替换组件位；
* 每个组件位至少 3 个候选组件；
* 不同候选组合仍具有相同主要功能。

## 15.2 组合拆分

假设两个组件位分别有：

[
A={a_1,a_2,a_3},
\quad
B={b_1,b_2,b_3}
]

训练集中只使用部分组合，例如：

[
(a_1,b_1),(a_1,b_2),(a_2,b_1),(a_2,b_2)
]

测试集中使用：

[
(a_1,b_3),(a_3,b_1),(a_3,b_3)
]

要求每个单独组件都在训练中出现，但组件配对没有出现。

## 15.3 下游任务

可以训练：

* 部件分割模型；
* Particulate；
* 运动学图预测模型。

指标：

* part mIoU；
* joint type accuracy；
* axis angular error；
* axis position error；
* kinematic edge F1；
* articulated gIoU；
* unseen-combination performance drop。

该实验能够直接证明：

> 模板化不是简单增加相似样本，而是在学习模型中产生组合泛化收益。

---

# 16. 可选实验：机器人交互与策略训练

这一部分应放在物理属性完成后，不能现在直接作为主实验。

## 16.1 任务

选择三类标准任务：

* 打开 revolute 门或盖子；
* 拉开 prismatic 抽屉；
* 旋转 knob 或 handle。

## 16.2 数据组

* 只在 PartNet/Articraft 资产上训练；
* 只在 Ours 上训练；
* 混合训练；
* Ours-LowDiversity；
* Ours-FullDiversity。

## 16.3 测试集

* 未见 seed；
* 未见模板；
* 未见组件组合；
* 未见尺寸范围；
* 外部 PartNet-Mobility 或 Lightwheel 资产。

## 16.4 指标

* task success rate；
* normalized articulation progress；
* time to completion；
* unintended collision rate；
* peak contact force；
* policy variance over random seeds。

应优先采用状态输入或统一几何观测，避免纯色材质导致视觉域差干扰资产多样性结论。

---

# 17. 论文建议表格和图

## Table 1：数据集与模板属性对比

行：

* Ours；
* Articraft-10K；
* Infinigen-Sim；
* Artiverse；
* PartNet-Mobility；
* ArtVIP。

列：

* categories；
* templates；
* assets；
* discrete variants；
* continuous dimensions；
* part semantics；
* joint GT；
* metric scale；
* material；
* mass/inertia；
* marginal generation cost。

---

## Table 2：模板生成管线消融

列：

* template success；
* 36-seed pass rate；
* corner pass rate；
* regression retention；
* manual intervention；
* repair rounds；
* cost。

---

## Table 3：机械合法性和碰撞质量

列：

* parse rate；
* static overlap；
* isolated-part rate；
* exact-mesh mean/max AOR；
* convex AOR；
* full-range collision-free rate；
* (E_{Q\rightarrow P})；
* watertight rate。

---

## Table 4：跨仿真器与稳定性

列：

* parse；
* instantiate；
* first-step；
* passive stable；
* full articulation；
* rest stable；
* worst-state stable；
* rotation angle。

---

## Table 5：多样性和可控性

列：

* discrete combinations；
* estimated capacity；
* kinematic graph perplexity；
* structural unique rate；
* target hit rate；
* NMAE；
* reproducibility；
* amortized cost。

---

## Table 6：Particulate 下游结果

列：

* Rest gIoU；
* Rest PC；
* Rest mIoU；
* Articulated gIoU；
* Articulated PC；
* OC；
* OOD category average。

---

## Figure 1：方法消融曲线

显示从 Vanilla Agent 到 Full Pipeline：

* seed pass rate 上升；
* regression failure 下降；
* repair cost 变化。

## Figure 2：QC 前后运动穿模

展示：

* 原始 seed；
* 静止 QC 后；
* full-range QC 后。

## Figure 3：碰撞表示权衡

横轴为 collision complexity 或仿真耗时，纵轴为几何偏差与 AOR：

* Exact Mesh；
* Convex Hull；
* CoACD。

## Figure 4：模板组合多样性

同一模板固定主体，分别改变：

* 组件候选；
* 参数；
* 拓扑；
* 关节范围。

## Figure 5：数据规模曲线

横轴：

[
1K,\ 5K,\ 10K,\ 30K
]

纵轴：

* Particulate articulated gIoU；
* PC；
* OOD 类别性能。

---

# 18. 当前不建议做的指标

根据现有管线状态，以下方向不适合作为主实验：

## 视觉真实感指标

暂不主打：

* CLIP/t-SNE 与真实照片分布；
* PBR 材质质量；
* PSNR；
* 图像真实感用户研究。

原因是当前所有材质均为纯色、没有贴图，mesh 面片中位数也较低。这一方向容易掩盖你们的模板和机械结构优势。

## 重建指标

不应报告：

* 与真实对象配对的 CD/F-score；
* joint axis reconstruction error；
* joint origin reconstruction error；
* joint limit reconstruction error。

这些指标适用于单图或扫描重建任务，而你们当前是资产分布生成。只有在单独增加“参考图到模板参数拟合”任务时才适用。

## 真机物理保真

暂不建议将以下内容作为主实验：

* 光学动捕轨迹对照；
* 仿真—真实 Pearson correlation；
* 高精度 sim-to-real。

当前物理属性和外观还不足以正面对标 ArtVIP 的数字孪生路线。

---

# 19. 实验执行顺序

## 第一阶段：立即完成

1. 统计模板、类别、槽位、候选和参数规模；
2. 跑全库 36-seed 轻量 QC；
3. 统计 QC 前后 pass rate；
4. 计算 exact-mesh AOR；
5. 计算图 perplexity、非重复率和控制命中率；
6. 统计模板成本和单资产摊销成本；
7. 完成管线消融实验。

这一阶段已经足以形成论文的核心方法和数据集表格。

## 第二阶段：补齐物理前置条件

1. 补 per-part mass 和 inertia；
2. 补 joint damping/friction；
3. 为非水密、空心和薄壳物体建立体积估计规则；
4. 将物理属性完备率提升到接近 100%。

## 第三阶段：Sim-Ready 实验

1. Exact Mesh、Convex Hull、CoACD 对比；
2. MuJoCo/Genesis/PyBullet/Isaac Sim 导入矩阵；
3. rest-pose 和 worst-articulation stability；
4. 长时间仿真和异常统计。

## 第四阶段：下游实验

1. 复现 Particulate baseline；
2. 加入 Articraft-10K；
3. 加入等量 Ours；
4. 做 low-diversity 对照；
5. 做 1K–30K 数据规模曲线；
6. 做未见组件组合实验。

## 第五阶段：可选机器人实验

只有在物理参数和跨引擎测试完成后，再做 policy 或 sim-to-real。

---

# 20. 推荐的最小投稿实验包

如果时间有限，至少完成以下四组：

### 必做一：模板管线消融

证明 Source Map、槽位、约束、Harness 和回归测试有效。

### 必做二：全库分布级机械验证

报告约 19K 个资产的：

* seed pass rate；
* 静止穿模；
* full-range AOR；
* 悬空率；
* corner seed 通过率；
* QC 淘汰率。

### 必做三：多样性、可控性和成本

报告：

* 组合域；
* 图 perplexity；
* 非重复率；
* 属性命中率；
* 确定性；
* 摊销成本。

### 必做四：Particulate 下游实验

至少比较：

[
P0,\quad P0+\text{Articraft},\quad
P0+\text{Ours},\quad
P0+\text{Ours-LowDiversity}
]

这四组共同回答：

> Arti-Template 不只是生成更多资产，而是生成具有组合多样性、机械有效性和外部学习价值的资产分布。

---

# 21. 最终论文实验叙事

整篇实验部分建议按以下逻辑展开：

[
\text{模板管线为什么有效}
]

[
\Downarrow
]

[
\text{模板能否稳定覆盖整个组合域}
]

[
\Downarrow
]

[
\text{生成资产是否机械有效、可控且可复现}
]

[
\Downarrow
]

[
\text{是否具备仿真器部署能力}
]

[
\Downarrow
]

[
\text{是否能够提升外部下游模型}
]

最关键的不是追求所有现有论文指标，而是让每个实验都直接支撑你们与 Articraft 的本质区别：

> **Articraft 自动生成一个资产；Arti-Template 自动生成一个经过分布级验证、可以持续生产资产的生成器。**
