# 资产评估 + Particulate 评估：论文调研、指标抽取与落地映射

> 调研日期 2026-08-03。配套文件：[TEMPLATE_METRICS.md](TEMPLATE_METRICS.md)（本项目已有的指标盘点）。
> 本文回答三件事：**(1) 最新相关论文有哪些；(2) 它们实验部分到底报什么指标、怎么算；(3) 哪些能直接用在本项目、哪些要改造、哪些不要报。**

---

## 0. 先厘清评估对象：两条独立的实验线

调研中确认 **Particulate 是一个真实系统**，不是笔误：

- **Particulate: Feed-Forward 3D Object Articulation**（arXiv 2512.11798，CVPR 2026，Oxford VGG + Ruining Li 等）——输入一个 3D mesh，前馈预测**关节部件分割 + 运动学结构 + 关节参数**，秒级出一个可动资产。
- **Instruct-Particulate**（arXiv 2606.14699，2026-06）——加了运动学条件控制（部件描述/连接关系/关节类型/点提示）的放大版，训练集约 **150k** 个可动物体。
- 关键连接点：**Instruct-Particulate 的四个训练数据源之一就是 Articraft 生成的 10k 资产**（本项目的上游 SDK 就是 Articraft）。其 data-scaling ablation 明确写道：coding-agent 生成的数据（带完整关节参数监督）**主要贡献在 Joint Axes 指标**（AE 从 13.1° → 11.0°）。

因此本项目的评估天然是两条线：

| 线 | 评估对象 | 问题 | 主要参照论文 |
|---|---|---|---|
| **A 线：资产质量** | `seed_exports/` 里的 URDF/mesh 本身 | 我们的资产是否 sim-ready、几何/物理/运动是否合法、多样性与可控性如何 | Infinigen-Sim、Infinite Mobility、Artiverse、ArtVIP、EmbodiedGen V2、PhysX-Omni |
| **B 线：Particulate 下游收益** | 用我们的模板资产训练/微调 Particulate 或 Instruct-Particulate | 我们的数据能不能把关节化估计模型推得更好（尤其 Joint Axes / 结构复杂样本） | Particulate、Instruct-Particulate、Articraft |

B 线是最"可发表"的一条：**Instruct-Particulate 已经证明 Articraft 数据有用，而我们比 Articraft 多的是"参数化模板 → 无限 seed + 精确关节真值 + 可控分布"**，正好去打它 ablation 表里的下一行。

---

## 1. 论文清单（按簇，均为 2025-03 ~ 2026-07）

### 簇 1｜直接上下游（必读，B 线基础）
| 论文 | ID | 与本项目关系 |
|---|---|---|
| Particulate: Feed-Forward 3D Object Articulation | [2512.11798](https://arxiv.org/abs/2512.11798)（CVPR 2026） | 下游被评估模型；提供了**新的关节化评估协议**（带 unmatched 惩罚），可直接拿来当我们资产的"结构可感知度"探针 |
| Instruct-Particulate | [2606.14699](https://arxiv.org/abs/2606.14699) | 已用 Articraft-10K；其 data-scaling 表就是我们要加一行的表 |
| Articraft: An Agentic System for Scalable Articulated 3D Asset Generation | [2605.15187](https://arxiv.org/abs/2605.15187) | 本项目上游 SDK；报了 pass/retention、成本、人评 Top-1 |

### 簇 2｜程序化/模板同类（A 线最直接的对手与模板）
| 论文 | ID | 看点 |
|---|---|---|
| Infinigen-Sim: Procedural Generation of Articulated Simulation-Ready Assets | [2505.10755](https://arxiv.org/abs/2505.10755) | **和本项目方法论最像**（节点图/程序化模板 + 关节节点 + URDF/USD/MJCF 导出）。它的实验完全走**下游**：分割 mAP、RL 成功率、sim2real |
| Infinite Mobility | [2503.13424](https://arxiv.org/abs/2503.13424) | 程序化可动物体；提供了**多样性三件套**（关节数均值/方差、Tree-Edit-Distance）+ GPT-4V 判分 + 生成速度 |

### 簇 3｜数据集/基准（A 线的横向对比表来源）
| 论文 | ID | 看点 |
|---|---|---|
| Artiverse: A Diverse and Physically Grounded Dataset for Articulated Objects | [2605.24403](https://arxiv.org/abs/2605.24403)（CVPR 2026） | 5402 物体/88 类；**kinematic graph perplexity**（图困惑度）作多样性指标；两个下游任务（部件运动分析、条件生成）+ Genesis 落地 |
| ArtVIP | [2506.04941](https://arxiv.org/abs/2506.04941) | 992 资产；**光学动捕对齐真实关节响应**（Pearson 0.9886）；三角面数/渲染保真/IL+RL sim2real |
| PhysX / PhysXNet | [2507.12465](https://arxiv.org/abs/2507.12465) | 五类物理标注（绝对尺度、材料、affordance、运动学、功能描述）及其打分方式 |

### 簇 4｜sim-ready 生成（A 线的"就绪度"指标来源）
| 论文 | ID | 看点 |
|---|---|---|
| EmbodiedGen V2 | [2607.07459](https://arxiv.org/abs/2607.07459) | **最贴近工程验收**：human acceptance 96.5%、collision success 98.6%、每资产 2.6 min、"1.35 次尝试出一个合格资产"、跨 6 个仿真器可移植 |
| PhysX-Omni | [2605.21572](https://arxiv.org/abs/2605.21572) | 刚/柔/可动统一；kinematic score、绝对尺度 MSE、材料/affordance/描述评分 + 人类相关性 ρ |
| URDF-Anything+ | [2603.14010](https://arxiv.org/abs/2603.14010) | IoU/F-score/CD + AxisErr/PivotErr/TypeErr/LimitErr + 推理耗时 + 真机部署成功率 |
| PhysX-Anything / SIMART / SPARK / GAOT / ArtiLatent / LARM / KineDiff3D | 2511.13648 / [2603.23386](https://arxiv.org/abs/2603.23386) / 2512.01629 / 2512.03566 / 2510.21432 / 2511.11563 / 2510.17137 | 同一套指标族的更多样本点，用于确认"社区共识指标" |

### 簇 5｜物理保真与修复（A 线物理指标的**唯一严肃来源**）
| 论文 | ID | 看点 |
|---|---|---|
| Automatically Improving Simulation Physics for Articulated Objects | [2605.19136](https://arxiv.org/abs/2605.19136) | **给出了可直接抄的物理鲁棒三件套公式**：穿透深度 Φ、位姿漂移 D_pos/D_ori、关节振荡；以及 SRCC（sim-real 成功率相关系数）、VLM-as-judge 运动真实性 0–5 分 |
| Real-IKEA: Physical Fidelity is the Prerequisite for Robust Manipulation | [2606.08564](https://arxiv.org/abs/2606.08564) | 碰撞网格精度（对视觉网格稠密采样比对）、质量/惯量/摩擦/关节参数偏差 → 操作成功率 |
| JODA: Composable Joint Dynamics for Articulated Objects | [2605.09954](https://arxiv.org/abs/2605.09954) | 关节动力学（阻尼/摩擦/驱动）建模与评估 |
| URDF-X（Springer 2026） | — | 碰撞网格优化 + 关节参数纠正，正面打"碰撞几何=视觉几何"这个病 |

### 簇 6｜生成分布指标的原始出处
NAP（ID）、CAGE（AID、AOR）、SINGAPO（dgIoU/dcDist/dCD 体系）、ArtFormer（POR、MMD/COV/1-NNA）、MeshArt。

### 簇 7｜评估方法学（怎么让"人工签核"变成可报指标）
| 论文 | ID | 看点 |
|---|---|---|
| A Cross-Model VLM-Judge Protocol for Single-Image 3D Mesh Quality | [2606.18451](https://arxiv.org/abs/2606.18451) | **24 视图固定渲染台 + 双判官族 + 位置偏置纠正（只保留顺序一致的判决）**，Cohen κ=0.66；并实测 **render-space CLIP 相似度基本等于瞎猜**，几何统计量只是弱信号 → 别再用 CLIP score 当质量代理 |
| Judging to Improve: De-biased VLM-as-3D-Judge | [2606.20364](https://arxiv.org/abs/2606.20364) | 去偏 judge 协议 |
| 3D-DefectBench / DB-3DME | HF / CVPRW 2026 | 9 维二值缺陷向量（5 几何 + 4 纹理），可当我们视觉关口的 rubric |
| 综述：From Visual Synthesis to Interactive Worlds | [2604.23629](https://arxiv.org/abs/2604.23629) | 明确指出"**尚无公认的 production-readiness 评测套件**"——这正是我们可以占的位 |
| 综述：3D Generation for Embodied AI and Robotic Simulation | [2604.26509](https://arxiv.org/abs/2604.26509) | 给出 sim-ready 的四要素定义（几何有效性/物理参数化/运动学可执行性/仿真器兼容性），可直接当我们章节骨架 |

---

## 2. 指标清单：从各家实验部分抽出来的原始定义

### G1 关节化估计精度（Particulate 系）——**B 线主表**
| 指标 | 定义（论文原样） |
|---|---|
| **部件匹配** | Hungarian 按**部件质心距离**把预测部件匹配到 GT，得到单射 π |
| **gIoU↑** | 双向平均：预测侧 + GT 侧各自算匹配部件的 generalized IoU；**未匹配部件记 −1 惩罚**（这是 Particulate 相对旧协议的关键修正） |
| **mIoU↑** | 同结构，但未匹配惩罚 ε=0 |
| **PC↓** | 部件级双向 Chamfer；未匹配部件罚"输入 mesh 包围盒对角线的一半" |
| **OC↓** | 整体物体双向 Chamfer，**在"所有关节推到上限"的全展开姿态下**算，不需要部件匹配 |
| **AE↓ / LE↓** | 匹配关节的轴**角度误差**（度）/ 轴**位置误差** |
| **Part Match P/R** | Instruct-Particulate 追加：预测部件与 GT 的精确率/召回率 |
| 采样 | 评估用 10⁶ 点均匀采样；训练输入 N=2048（50% 均匀 + 50% 锐边，二面角 >30°） |
| 姿态 | rest state 一套 + fully articulated（每个关节推到最大行程）一套 |

> 重要旁证：Particulate 指出旧协议"只在匹配上的部件对上算分、忽略未匹配部件"，导致 **Naive Baseline（整个物体当一个部件）在所有指标上打赢所有方法**。所以我们若报关节化指标，**必须用带惩罚的版本**，否则会被审稿人一句话打死。

**基准与规模**：训练 = PartNet-Mobility + GRScenes 共 3800 物体/50 类，**排除 >16 个可动部件的物体（P_max=16）**；测试 = PartNet-Mobility test（77 类）+ 新建 **Lightwheel 基准（243 物体 / 14 类）**。

### G2 生成分布与多样性
| 指标 | 定义 | 出处 |
|---|---|---|
| ID（Instantiation Distance） | 跨关节状态取最小的成对 Chamfer-L1（每部件 2048 点） | NAP |
| AID | 用部件包围盒的体素 IoU 代替 CD，弱化细几何影响 | CAGE |
| **AOR / POR↓** | 图中**兄弟部件**在任意关节状态下的平均重叠体积比（OBB + vIoU）——**物理合理性代理** | CAGE / ArtFormer |
| MMD↓ / COV↑ / 1-NNA | 分布保真 / 多样性 / 可分性三件套 | 通用 |
| RS-/AS- × dgIoU/dcDist/dCD | 静止态/运动态 × 包围盒/质心/网格 三档粒度的统一距离族 | SINGAPO |
| Graph Acc | 拓扑图正确率 | SINGAPO |
| **关节数均值 / 方差** | Infinite Mobility：12.32 / 659.02，PartNet-Mobility：5.91 / 40.31 | Infinite Mobility |
| **Tree-Edit-Distance** | 运动学树两两编辑距离，衡量结构多样性：78.62 vs 3.88 | Infinite Mobility |
| **Kinematic graph perplexity** | 运动学图的"有效种类数"，Artiverse 报 1.5× PartNet-Mobility | Artiverse |

### G3 几何与外观
CD、F-score@τ（0.02 / 0.05 / 0.1）、体素 IoU、PSNR（30 视图）、CLIP score、3D 一致性、**topology validity / watertight**、UV/PBR 质量、三角面数分布、**部件间平均 IoU（越低越说明部件几何独立）**（PartCrafter：64³ 体素化）。

### G4 sim-ready 与物理鲁棒（本项目最大缺口，也是最容易做出增量的一块）
| 指标 | 定义 | 出处 |
|---|---|---|
| sim-ready 四要素 | 几何有效性 / 物理参数化（质量·惯量·密度·摩擦·恢复系数·质心） / 运动学可执行性（类型·轴·限位·约束） / 仿真器兼容（URDF·MJCF·USD） | 综述 2604.26509 |
| **穿透深度 Φ(q)** | `Φ(q)=Σ max(0, −s(p))`，接触点带符号间距求和 | 2605.19136 |
| **位姿漂移 D_pos / D_ori** | 静置 T_set 后相对参考位姿的最大位移/角偏；阈值 **≤1e-3 m / ≤1e-2 rad** | 2605.19136 |
| **关节振荡** | 关节速度符号反转 ≥3 次且幅值 A_j>ε 即判欠阻尼 | 2605.19136 |
| **Collision Success Rate** | SAPIEN 里脚本化 Franka 抓取-提起，4 个偏航角；提升高度超过与包围盒成比例的自适应阈值算成功（EmbodiedGen V2：98.6%） | EmbodiedGen V2 |
| **碰撞网格精度** | 在视觉网格与碰撞网格上稠密均匀采样比对偏差 | Real-IKEA |
| 凸分解质量 | CoACD 的 concavity（同时从表面与内部量到凸包的差） | CoACD |
| Stability rate / 静置稳定率 | 短时仿真后位移（PhyScene 式） | PhysX-Omni / 综述 |
| 尺度 / 材料 / 惯量误差 | 绝对尺度 MSE、材料 L2、密度与材料先验一致性 | PhysX / PhysX-Omni |
| **SRCC** | 同一批策略在仿真与真机上成功率的 Pearson 相关——衡量"这批资产做出来的仿真值不值得信" | 2605.19136 |
| 关节动力学保真 | 光学动捕（0.1 mm / 90 Hz）对比 1–2.5 N 拉抽屉的真实轨迹，Pearson 0.9886 | ArtVIP |

### G5 语义与人/VLM 验收
Human acceptance rate（EmbodiedGen V2：96.5%）、24 视图 VLM judge + 位置偏置纠正（κ=0.66）、GPT-4V 几何/纹理胜率（Infinite Mobility：几何 64.18% vs 35.45%，纹理 84.81% vs 14.44%）、affordance/grasp coverage rate、语义外观/网格几何/文对齐三关口通过率（76.2% / 75.9% / 91.0%）。

### G6 下游任务
可动部件分割 mAP（Infinigen-Sim：+I 后 48.23 → 50.13，小部件如把手/拨杆/开关涨最多）、RL success-once（Infinigen-Sim 平均 **2.86×** 于 PartNet-Mobility）、IL/ACT **零样本 sim2real**（开门 18/30 vs 3/30；推烤面包机 11/30 vs 0/30；拉冰箱 10/30 vs 0/30）、跨仿真器一致载入（Genesis / Isaac Gym / Isaac Sim / MuJoCo / PyBullet / SAPIEN）。

### G7 生产性与成本（"模板 vs 逐个建模"的经济性论据）
每资产 API 成本（Articraft：$1.14，区间 $0.60–3.14）、人工筛后留存率（Articraft：91%）、每合格资产平均尝试次数（EmbodiedGen V2：1.35）、生成耗时（Infinite Mobility 0.46 s/物体；URDF-Anything+ 34.70 s；EmbodiedGen V2 2.6±0.4 min）、端到端环境可直接用率（83.3%）。

---

## 3. 映射到 arti-skill：直接可用 / 需适配 / 不要报

### 档 A：直接可用（零或低改造，用现有导出物就能算）

| 指标 | 出处 | 本项目怎么算 | 现状 |
|---|---|---|---|
| 关节数均值/方差、link 数、关节类型混合 | Infinite Mobility | 直接扫 `seed_exports/**/model.urdf` | 数据已在盘上，没统计 |
| **Tree-Edit-Distance / 运动学图困惑度** | Infinite Mobility / Artiverse | 从 URDF 建运动学树，两两 TED + 图分布熵 | 无，一次性脚本可得 |
| **AOR / POR（rest pose）** | CAGE / ArtFormer | 我们已有精确网格重叠检测；额外用 OBB+vIoU 版本以便与他们同尺度对比 | 已有二元门，缺连续值 |
| 三角面数 / 文件体积 / watertight 率 | ArtVIP / 综述 | 已实测：面数中位 320、p90 3029；**流形率仅 50%** | 已测未指标化 |
| 多仿真器导入三级成功率 | 综述 / EmbodiedGen V2 | MuJoCo/Genesis/trimesh/coacd 本机已有，补 PyBullet + SAPIEN；分"解析成功 / 建模成功 / 首步不报错" | 只做过 MuJoCo 解析 40/40 |
| 生成成本 / 吞吐 / 留存率 / 尝试次数 | Articraft / EmbodiedGen V2 | `.articraft/template_sweep_state/*.json` 已有 pass_rate；补 agent token 成本与 seed/分钟 | 部分有 |
| 类别与部件语义标签完备率 | Artiverse / PhysX | 模板里 link/visual 名本来就是语义名，直接统计 | 事实具备，未报 |
| **穿透深度 Φ、D_pos/D_ori、关节振荡** | 2605.19136 | 载入 MuJoCo/SAPIEN，静置 T_set 后读接触与位姿；公式和阈值可直接照抄 | 完全没有 |
| Collision Success Rate | EmbodiedGen V2 | SAPIEN 脚本化抓取-提起，4 偏航角 | 没有 |
| 24 视图 VLM judge（几何/纹理/类别） | 2606.18451 / Infinite Mobility | 已有 `blender_gallery` / `template_renders` 渲染能力，套双判官 + 位置偏置纠正 | 现在靠人工关口③ |

### 档 B：需要适配（模板管线要重新定义分母/分子）

| 指标 | 原定义 | **本项目的适配版** | 为什么值得做 |
|---|---|---|---|
| gIoU/PC/OC/AE/LE | 预测 vs GT | **反过来用**：把 Particulate 当"结构可感知度探针"，对我们的资产跑推理，与我们**自带的真值关节**比 AE/LE/mIoU。分数低说明我们的几何没有把关节结构表达清楚（薄壳、部件贴死） | 唯一能自动量化"这资产的关节结构是否肉眼/模型可辨"的手段，正好补人工关口③ |
| AOR（静止态） | rest / 少量采样态 | **全行程 AOR**：沿关节 lower→upper 细分扫掠取最坏值；再报 min-clearance(q) 曲线 | 别人只在采样态报，我们能报连续行程——**A 线差异化主表** |
| MMD/COV/1-NNA | 生成模型 vs 数据集分布 | 分母换成"**同一模板跨 seed**"与"**模板库 vs PartNet-Mobility 类内分布**"，回答"我们的采样是否覆盖真实分布且不塌缩" | 模板法容易被质疑"只是换尺寸"，这是正面回应 |
| 可控性 | 文/图条件 → 生成一致性 | **参数可控性**：指定目标属性（高度/门数/开合角）→ 采样资产实测值与目标的误差；再加 KL(目标分布‖实得分布) | 程序化模板最有说服力的指标，扫描数据集**做不到** |
| 数据-规模 ablation | Instruct-Particulate 的 A/B/C/D 四档 | 加第 **E 档 = 我们的模板资产**，报 Part Match P/R、mIoU、gIoU、PC、OC、**AE/LE** | 它自己的结论是"coding-agent 数据主要涨 Joint Axes"，我们的关节参数是**程序化精确真值**，理应涨得更多 |
| 惯量/密度合理性 | 与 GT 标注比 | 我们没有 GT：改为**与材料先验区间比**（木 400–900、钢 ~7800 kg/m³）+ 惯量张量正定/三角不等式/与 AABB 量级相容 | 现状惯量完备率仅 13.6%，先补齐再报 |
| 碰撞几何质量 | 碰撞网格 vs 视觉网格偏差 | 我们碰撞=视觉（4318/4318），先跑 CoACD 生成凸分解，再报**凸包保真差**：精确网格无穿模但换凸包后出现互穿的比例（已实测 hull/vol 中位 1.15、28% >2） | TEMPLATE_METRICS.md 已把它列为第一优先级，与 URDF-X / Real-IKEA 的问题意识完全一致 |
| 下游 RL / sim2real | Infinigen-Sim 三任务 | 同款协议（ManiSkill3 PPO + ACT 零样本迁移），但**我们的卖点是类别广度而非单类深度**，建议选 3–5 个我们独有的机构类（夹钳、绞肉机、熨衣板）做"PartNet-Mobility 里根本没有"的论证 | 有真机才做；没真机就只做 RL 泛化 |

### 档 C：不适用 / 建议不要报

| 指标 | 为什么不报 |
|---|---|
| CD / F-score / IoU vs GT 网格 | 我们不是重建方法，**没有配对 GT**。硬凑一个"参考图对应的真实物体"只会自找麻烦 |
| PSNR / FID / **render-space CLIP score** | 2606.18451 实测 CLIP 相似度**基本等于随机**，且学习到的权重给它负系数。用它当质量代理会被直接质疑 |
| ID / AID | 定义上依赖"与 GT 数据集分布比"，我们做覆盖率论证时用 MMD/COV 就够，ID/AID 反而绑死在 PartNet-Mobility 的分布上 |
| 材料 Young's modulus / Poisson 比误差 | 我们不产出这些标注，除非决定接 PhysX 那套五元标注，否则别开这个坑 |
| 光学动捕对齐 | 需要真实同款物体 + 动捕棚（ArtVIP 的做法），成本远超收益 |

---

## 4. 建议的实验设计（论文表格草案）

**表 1｜资产库横向对比**（对标 Artiverse 的 Table 2 / ArtVIP 的对比表）
列：物体数、类别数、平均部件数、总关节数、平均关节数、**关节数方差**、多 DoF 关节数、**TED**、**图困惑度**、是否带惯量/阻尼、碰撞几何形式、许可。行：PartNet-Mobility / GAPartNet / AKB-48 / ArtVIP / Artiverse / Infinite Mobility / Articraft-10K / **Ours**。

**表 2｜sim-ready 三级导入 + 物理鲁棒**
列：解析成功率 / 建模成功率 / 首步仿真无异常率 × {MuJoCo, PyBullet, SAPIEN, Isaac, Genesis}；Φ、D_pos、D_ori、振荡率、Collision Success Rate。

**表 3｜运动完整性（本项目独家主表）**
全行程连续穿模率、全行程 AOR、min-clearance 曲线统计（min / 均值 / 低于阈值行程占比）、**travel_realized_ratio（实际无碰撞可达行程 ÷ 声明限位）**、多关节组合覆盖率、mimic 闭链残差。这一张表是"离散采样 vs 连续扫掠"的正面差异，别人报不出来。

**表 4｜多样性与可控性（模板独有）**
组合域规模（core / raw）、结构词汇量、跨 seed 结构非重复率、参数-几何响应率、**属性可控误差**、目标分布 KL、corner 覆盖率、确定性可复现率。

**表 5｜Particulate 数据增益（B 线核心，复刻 Instruct-Particulate 的 data-scaling 表）**
行：A=既有(PM+GRScenes) / B=+part-segmented / C=+VLM 伪标 / D=+Articraft-10K / **E=+Ours**。
列：Part Match P/R、rest mIoU/gIoU/PC、articulated gIoU/PC/OC、**AE/LE**。
测试集：Lightwheel(243/14 类) + PartNet-Mobility test。
预期落点：**AE/LE 与 articulated 指标**（我们的关节参数是精确程序化真值，而非伪标）。

**表 5b｜Particulate 当资产评估器（零训练成本，先做这个）**
对我们每个 seed 跑 Particulate 推理，与自带真值比 AE/LE/mIoU，报分布。低分样本 = 结构表达不清（薄壳、部件贴死、关节被几何淹没）→ 直接反哺模板修复队列。

**表 6｜成本与摊销**
每模板 agent 成本、每合格资产成本、seed/分钟、通过率、**一次投入摊销比 =（组合域 × 通过率）÷ 造模板成本**、与人工建模的小时/资产对比。

**表 7｜VLM/人工验收**
24 视图双判官 + 位置偏置纠正的类别可识别率、9 维缺陷向量分布、与人工签核的 Cohen κ（证明可以替掉人工关口③的一部分）。

---

## 5. 优先级（按 论文价值 ÷ 实现成本）

1. **表 5b（Particulate 当探针）** —— 开源实现已有（`RuiningLi/particulate`），`seed_exports/` 已是 URDF+obj，官方就带 `particulate.data.process_urdf`。零训练、当天可出数，且同时服务 A 线与 B 线。
2. **表 3（全行程运动完整性）** —— 现有 `generate_motion_pose_plan` / `geometry_qc` 改成细分扫掠即可，是最独特的一张表。
3. **表 2（sim-ready + 物理鲁棒）** —— 先补惯量与 `<dynamics>`（现在 13.6% / 23.6%），再跑 Φ/D_pos/D_ori/振荡；同时做 CoACD 凸分解并报**凸包保真差**。
4. **表 1 + 表 4（多样性/可控性汇总）** —— 几乎纯统计脚本，从已有 URDF + sweep_state 一次性出。
5. **表 6 + 表 7** —— 成本已有一半；VLM judge 用现成渲染管线。
6. **表 5（真训练 ablation）** —— 最贵（需 GPU + 复现 Instruct-Particulate 训练），放最后，等前面证明数据质量后再投。

---

## 6. 已知坑（做之前先知道）

- **mesh 流形率只有 50%**：任何体素化/vIoU/CoACD/惯量积分都会在非流形件上出错或退化。表 1 之前先跑一遍修复（EmbodiedGen V2 的 ablation 显示"不修 mesh"会让处理时间从 2.6 min 涨到 21.3 min，说明这步是刚需）。
- **碰撞几何 = 视觉几何（4318/4318）**：主流仿真器按凸包/凸分解算碰撞，我们的"无穿模"保证在真实仿真里可能不成立。这是缺陷，但把它做成"凸包保真差"诊断表反而是贡献。
- **Particulate 的 P_max=16**：我们不少模板部件数超 16，需要分层报告（≤16 / >16 两档），否则无法与其基准直接可比。同时它要求 **+Z up**，导出时注意坐标约定。
- **数据泄漏**：模板源自参考图片，若参考图来自 PartNet-Mobility 类别，B 线实验要显式声明与 test split 的隔离。
- **别用 CLIP score**，见 §3 档 C。
- **Naive Baseline 陷阱**：关节化指标必须用带 unmatched 惩罚的版本。

---

## 附：URL 索引

Particulate https://arxiv.org/abs/2512.11798 ｜ Instruct-Particulate https://arxiv.org/abs/2606.14699 ｜ Articraft https://arxiv.org/abs/2605.15187 ｜ Infinigen-Sim https://arxiv.org/abs/2505.10755 ｜ Infinite Mobility https://arxiv.org/abs/2503.13424 ｜ Artiverse https://arxiv.org/abs/2605.24403 ｜ ArtVIP https://arxiv.org/abs/2506.04941 ｜ PhysX https://arxiv.org/abs/2507.12465 ｜ PhysX-Omni https://arxiv.org/abs/2605.21572 ｜ URDF-Anything+ https://arxiv.org/abs/2603.14010 ｜ EmbodiedGen V2 https://arxiv.org/abs/2607.07459 ｜ 自动改进仿真物理 https://arxiv.org/abs/2605.19136 ｜ JODA https://arxiv.org/abs/2605.09954 ｜ Real-IKEA https://arxiv.org/abs/2606.08564 ｜ SIMART https://arxiv.org/abs/2603.23386 ｜ VLM-Judge 协议 https://arxiv.org/abs/2606.18451 ｜ 去偏 3D-Judge https://arxiv.org/abs/2606.20364 ｜ 综述(production-ready) https://arxiv.org/abs/2604.23629 ｜ 综述(embodied 3D gen) https://arxiv.org/abs/2604.26509 ｜ SINGAPO https://arxiv.org/abs/2410.16499 ｜ CAGE https://arxiv.org/abs/2312.09570 ｜ ArtFormer https://arxiv.org/abs/2412.07237 ｜ 代码 https://github.com/RuiningLi/particulate
