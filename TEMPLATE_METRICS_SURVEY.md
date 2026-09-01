# 相关论文实验指标调研 → 模板管线适配

调研日期：2026-08-03。目标：把最新（2024–2026）相关工作**实验部分实际用的指标**抽出来，逐条判断哪些能直接用在 `arti-template` 的模板/seed 资产上、哪些需要改造、哪些不适用。
配套文件：[TEMPLATE_METRICS.md](TEMPLATE_METRICS.md)（管线现状盘点 + 实测基线）。

> **优劣势标记（贯穿全文表格的「本项目」列）**
> - **★ 优势**：有直接证据（代码闸门保证或本次实测数据）支持我们会明显好于对标方法
> - **☆ 可能优势**：机制上应当占优，但尚无实测；需要先跑一遍才敢写进论文
> - **◐ 中性**：能报出来，但拉不开差距；或补齐前置条件（如惯量）后才是中性
> - **⚠ 劣势/风险**：当前状态明确弱于对标方法，或存在会被审稿人攻击的隐患
> - **— 不适用**：任务形态不匹配，不建议报
>
> 判断依据的实测事实（详见 [TEMPLATE_METRICS.md](TEMPLATE_METRICS.md) §C）：碰撞几何 = 视觉几何 100%；惯量完备率 13.6%；关节 dynamics 23.6%；mesh 水密率 50%；凸包/实体体积比 28% > 2；材质**全部为纯色、零贴图**（3965 个 material 全是 `<color>`，0 个 `<texture>`，0 个 .mtl/贴图文件）；mesh 面片中位数 320；每模型视觉元素约 60 个。

---

## 0. 优势地图（一页速览）

**★★ 该主打的（有硬证据，且对标方法结构性做不到）**
1. **AOR = 0（精确网格口径）** —— 运动穿模是硬闸门，比 CAGE 的 bbox+sibling 口径更严
2. **人工返修率 0% + 边际成本≈CPU 时间** —— 对标 Articraft $1.13/资产、ArtVIP 1.8–15 h/对象
3. **组合域规模 / 类别数** —— 537 类，对标 Infinigen-Sim（5 类）、Artiverse（88 子类）、PartNet-Mobility（46 类）
4. **静止穿模率 / 悬空件率 = 0** —— 闸门保证

**★ 稳赢但需要把口径写清楚**
接触几何保真 E_{Q→P}=0（碰撞=视觉）；自带精确真值（部件分割/关节轴/参数）；条件可控性命中率；确定性可复现率 100%；人工验收关口是前置条件而非抽检。

**☆ 机制上占优、但必须先实测才能写**
运动学图 perplexity（若模板拓扑单一会翻车）；跨仿真器导入矩阵（仅 MuJoCo 解析验过）；%Stable / 倾覆角（依赖补惯量）；COV vs PartNet-Mobility；下游分割 mAP 与关节估计增益。

**⚠ 当前明确劣势 / 会被审稿人打的点**
1. **per-part 质量与惯量只有 13.6%**，关节 dynamics 只有 23.6% —— 一切物理指标的前置短板（用模板已有的 material 名 + 密度表即可低成本补齐）
2. **AOR(凸包) 未知** —— 28% 的 mesh 凸包/实体体积比 >2，精确网格的"无穿模"未必迁移到按凸包算碰撞的引擎
3. **零贴图、纯色材质**（3965 个 material 全是 `<color>`），mesh 面片中位数 320 —— 视觉真实感与 CLIP 分布对照这条赛道打不过 ArtVIP/PhysX-Anything，建议**不要碰**这一类指标
4. **无凸分解代理**，抓取类 Collision Success Rate 会低于 EmbodiedGen（98.6%）
5. **水密率 50%**

**— 不建议报（任务形态不匹配）**
CD / F-score / PSNR、关节轴/原点/限位误差（重建类，需成对 GT）；光学动捕对照；场景级 Col-S/Inst-S。

---

## 1. 调研到的论文（按可比性分组）

### A. 程序化 / Agent 生成 articulated 资产（**最直接的对标对象**）
| 论文 | 出处 | 与本项目的关系 |
|---|---|---|
| **Infinigen-Sim: Procedural Generation of Articulated Simulation Assets** | arXiv 2505.10755 (CoRL'25) | 同为"程序化生成器 = 资产分布"，指标范式最可照搬 |
| **Articraft: An Agentic System for Scalable Articulated 3D Asset Generation** | arXiv 2605.15187 | 本项目的底层基础设施；Articraft-10K，245 类，$1.13/资产 |
| **Articulate-Anything** | arXiv 2410.13882 (ICLR'25) | VLM + mesh 检索 + actor-critic；PartNet-Mobility 成功率 8.7/11.6% → 75% |
| **URDF-Anything+** | arXiv 2603.14010 | 端到端生成"仿真就绪" URDF |
| **PhysX-Anything** | arXiv 2511.13648 (CVPR'26) | 单图 → sim-ready（URDF/MJCF/SDF/glTF），含物理参数 |
| **EmbodiedGen V2** | arXiv 2607.07459 | sim-ready 资产流水线（修网格 + 凸分解 + VLM 物性 + 跨仿真器导出），指标最工程化 |

### B. articulated 资产数据集（对标"数据集论文"的表格骨架）
| 论文 | 出处 | 关键点 |
|---|---|---|
| **Artiverse** | arXiv 2605.24403 | 5402 对象 / 24607 部件；**运动学图 perplexity** 作为多样性指标；per-part 材料/质量/尺度 |
| **ArtVIP** | arXiv 2506.04941 (ICLR'26) | 992 数字孪生；**光学动捕对照**验证物理保真；人工 1.8–15 h/对象 |
| **PhysX-3D / PhysXNet** | arXiv 2507.12465 (NeurIPS'25) | 五维物理标注：绝对尺度、材料、affordance、运动学、功能描述 |
| **Real-IKEA** | arXiv 2606.08564 | **碰撞网格保真度**双向曲面偏差；关节 damping/friction 三档标定 |
| PartNet-Mobility / GAPartNet | — | 传统基线（2346 对象 / 46 类；GAPartNet 1166） |

### C. articulated 生成模型（指标最标准化的一支）
NAP → **CAGE**（arXiv 2312.09570）→ **SINGAPO**（2410.16499）→ **ArtFormer**（2412.07237）→ **ArtiLatent**（2510.21432）/ **MeshArt** / **GAOT** / **PWM-ArtGen**（2607.02045）。
共同指标：Instantiation Distance (ID)、MMD / COV / 1-NNA、gIoU/mIoU、**AOR（Average Overlap Ratio）**。

### D. 物理合理性 / sim-readiness
**Atlas3D**（2405.18515, NeurIPS'24，重力自支撑）、**DSO**（ICCV'25，%Stable / Rotation angle）、**PhysComp**（重心投影落在接触点凸包内）、**PhysPart**（2408.13724, ICRA'25，稳定性/移动性损失 + motion success rate）、**PhyScene / PAT3D**（Col-O/Col-S/Inst-O/Inst-S）。

### E. 综述（指标分类法直接可引）
**3D Generation for Embodied AI and Robotic Simulation: A Survey**（arXiv 2604.26509）：把指标分为 几何外观 / 物理合理性与 sim-readiness / 关节运动学有效性 / 场景级 / 具身任务 / **格式合规与部署就绪** 六类，并明确指出"评测标准化"仍是开放问题。

---

## 2. 论文里实际用的指标全表（含精确定义）

### 2.1 几何 / 视觉
| 指标 | 定义 | 出处 | 本项目 |
|---|---|---|---|
| Chamfer Distance / F-score(τ=0.02) / IoU | 与 GT 点云/体素的几何误差 | URDF-Anything+、PhysX-Anything（CD 14.43，F 77.50） | **—** 无成对 GT（除非做"参考图→模板参数拟合"支线） |
| PSNR（渲染） | 生成 vs GT 渲染图 | PhysX-Anything（20.35） | **—** 同上 |
| 绝对尺度误差 | 预测实际尺寸与真值之差 | PhysX-Anything（43.44 → 0.30） | **☆** 模板里尺寸是显式带单位的参数、且有 clamp，误差应结构性接近 0；但没验过真实尺寸先验 |
| 三角面数对比 | 与 PartNet-Mobility / BEHAVIOR-1K 的网格密度对比 | ArtVIP | **⚠** mesh 面片中位数仅 320，几何细节远低于 ArtVIP；但每模型约 60 个视觉元素（Articraft 消融口径 39→78），换成"视觉元素数"这一列则不吃亏 |
| CLIP 特征 t-SNE + 真实设备采集对照 | 生成资产的渲染特征是否落在真实分布内 | ArtVIP | **⚠** 零贴图纯色材质，渲染分布与真实照片差距大，这项大概率输 |
| 特征分布熵 | 3D VAE(3DShape2VecSet) 特征的熵 = 几何多样性 | ASSIST-3D 等 | **☆** 跨 seed 结构真变，几何多样性熵应高；未实测 |

### 2.2 关节 / 运动学
| 指标 | 定义 | 出处 | 本项目 |
|---|---|---|---|
| **AOR（Average Overlap Ratio）** | **任意两个"兄弟部件"在所有关节状态下重叠体积比的平均**；假设 sibling 永不重叠，越低越好 | CAGE，被 SINGAPO/ArtFormer/Artiverse 沿用 | **★★ 最强优势**：运动 QC 是硬闸门，且用精确网格 + 全部件对（比 bbox + sibling 严格），结构上 AOR ≡ 0；生成模型普遍 >0 |
| **Instantiation Distance (ID)** | 逐关节状态、每部件 2048 点、最小配对 Chamfer-L1，同时刻画几何 + 运动 | NAP → CAGE 系列 | **◐** 只在"与参考集比分布"时有意义，不是我们的强弱项 |
| MMD / COV / 1-NNA | 生成集 vs GT 集的分布距离、覆盖率、1-NN 判别率 | NAP/CAGE 系列 | **☆ COV 可能优势**（可无限采样去覆盖参考集）；1-NNA 反而可能暴露分布差异，属双刃 |
| RS-gIoU / AS-gIoU、mIoU | 静止态 / 关节态下部件包围盒 IoU | Artiverse、Articraft（Particulate 评测） | **—** 需成对 GT；仅作为"用我们的数据训模型"的下游指标出现（见 2.5） |
| 关节轴误差 / 原点误差 / 限位误差 | 与 GT 的角度(rad)、距离(m)、限位差 | URDF-Anything+、CAGE、ArtiLatent | **—** 重建类指标，我们无 GT |
| 部件分割 F1 (+M / +MA / +MAO) | 分割 F1，逐级加上运动类型、轴、原点的正确性约束 | Artiverse | **★ 作为"自带真值"** 时优势：我们的部件分割/运动类型/轴/原点都是构造出来的精确真值，无需标注、无误差 |
| Overlap Consistency / PC | 关节态一致性、部件一致性 | Articraft（Lightwheel benchmark） | **☆** 同 AOR 逻辑，闸门应保证优于生成式方法 |
| **运动学图 perplexity** | 各运动学图出现概率分布的熵的指数（有效图数量）；Artiverse 是 PartNet-Mobility 的 **1.5×** | Artiverse | **☆ 可能优势**：槽位候选会真正改变 link/joint 图（非纯尺寸变化），有条件超过 1.5×；**必须实测**，若模板多为单一拓扑就变成劣势 |
| motion success rate | 部件沿指定轴运动时碰撞/接触损失是否越阈值 | PhysPart | **★** 全行程运动闸门已是它的严格版 |

### 2.3 物理 / sim-readiness
| 指标 | 定义 | 出处 | 本项目 |
|---|---|---|---|
| **%Stable / Rotation angle** | 重力仿真静置后仍站立的比例；平衡态平均倾角 | DSO、Atlas3D | **☆ 补惯量后可能优势**：几何是规则装配体、接地面明确；但**现在 86% 的 link 没惯量**，不补就测不了 |
| 重心-接触凸包判据 | CoM 投影是否落在接触点凸包内 | PhysComp、Atlas3D | **☆** 纯几何可算，同上依赖体积/密度 |
| Col-O / Col-S / Inst-O / Inst-S | 物体/场景碰撞率、物体/场景失稳率（仿真前后变换差） | PhyScene、PAT3D | **—** 场景级，我们是单资产管线 |
| **Collision Success Rate** | SAPIEN 中脚本化"抓起并抬升"，**每资产 4 个均匀 yaw 角**，抬升超过与包围盒高度成比例的阈值算成功 | EmbodiedGen V2（98.6%；去掉凸分解降到 96.5%） | **⚠ 当前劣势**：没有凸分解代理、缺质量与摩擦，直接跑大概率低于 EmbodiedGen；补齐后可转 ◐/☆ |
| **Human Acceptance Rate** | 标注员判定"可直接用于仿真"的比例（输入一致性、几何合理、不可见面连贯、整体可用性） | EmbodiedGen V2（96.5%）；Articraft 的 1–5 分人工评级 <4 剔除（保留率 91.8%） | **★ 优势**：人工视觉关口③是**发布前置条件**而非事后抽检，交付集接受率按定义 100%；应改报"关口通过率 + 淘汰率"以免被质疑口径 |
| **接触几何保真 E_{Q→P} / E_{P→Q}** | 视觉网格 P 与碰撞网格 Q 双向稠密采样最近邻偏差；E_{Q→P} 度量碰撞体"膨胀" | Real-IKEA | **★ 优势（当前口径）**：碰撞几何与视觉几何逐元素完全相同，偏差恒为 0，文献中靠凸分解的方法必然 >0 |
| 关节 damping / friction 档位 | 低/中/高（damping 2/5/10，friction 5/10/20）下的任务成功率 | Real-IKEA | **⚠ 劣势**：仅 23.6% 的关节有 `<dynamics>`，无法做阻力分档实验 |
| 光学动捕轨迹对照 | 0.1 mm / 90 Hz 追踪，1.1–2.5 N 定力拉抽屉，比对仿真与实测位移 | ArtVIP | **—** 需动捕设备与实物样本，不建议 |
| 物理参数真实性 | 质量/惯量/摩擦/恢复系数与真值的误差；材料密度采样 | PhysX-3D、Artiverse（质量 = 体积 × 材料密度） | **⚠→☆**：现在 13.6% 惯量完备率是明确劣势；但模板里**已有 per-part material 名**，接一张密度表即可自动补全，补完后反而比 VLM 猜物性更可靠（可能优势） |
| 跨仿真器可移植 / 导出合规 | URDF/MJCF/USD 能否在目标引擎解析加载 | 综述 §7；EmbodiedGen、PhysX-Anything | **☆ 可能优势**：只用标准基元 + OBJ、无奇异特性，MuJoCo 实测 40/40 解析通过；四引擎矩阵未验 |
| **人工返修率** | 部署前需要人工修复的资产比例 | 综述 §7（Seed3D）；EmbodiedGen "83.3% 无需人工修改" | **★★ 强优势**：闸门通过即可直接用，返修率 0%；且模板一次修复→整个组合域受益（返修成本按模板摊销，不按资产计） |
| 生成开销 | 每资产耗时 / API 成本 / 尝试次数 | EmbodiedGen（2.6±0.4 min，1.35 次/有效资产）、Articraft（$1.13/资产，16.8 轮） | **★★ 强优势**：造模板是一次性成本，之后每个 seed 只是确定性编译（无 LLM 调用），**边际成本≈CPU 时间**，摊销后应比 $1.13/资产低 1–3 个数量级 |

### 2.4 多样性 / 可控性（程序化方法的主战场）
| 指标 | 定义 | 出处 | 本项目 |
|---|---|---|---|
| **连续参数维度数 + 离散变体数 + 估计唯一资产数** | 如 Door 39 维、Fridge 32 维；每类估计 10⁶–10²⁰ 个唯一资产 | Infinigen-Sim Table 1 | **★★ 强优势**：537 个模板 × 每类 core 36–108 组合 + 连续参数；量级与 Infinigen-Sim 同阶而**覆盖类别数远多于其 5 类**。这是最该打的主表 |
| 类别/部件规模对比表 | 类别数、对象数、部件数、prismatic/revolute 关节数、多自由度关节数 | Artiverse、ArtVIP | **★ 优势（类别维度）**：537 类 vs Artiverse 88 子类 / ArtVIP 37 子类 / PartNet-Mobility 46 类；对象数按需生成 |
| 属性完备性打勾表 | 度量尺度 / per-part 材料 / per-part 质量 / 多 DoF 关节 有无 | Artiverse vs PartNet-Mobility vs ArtVIP | **◐ 混合**：语义标签 / 关节真值 / 参数真值 / 材料名 ✅；**per-part 质量 ✗（13.6%）**，补齐前这张表会输给 Artiverse |
| 条件一致性（可控性） | 给定结构图/部件规格/单图条件，生成结果与条件的吻合度 | CAGE、SINGAPO、ArtFormer | **★ 优势**：属性由参数直接构造而非采样逼近，目标命中率结构上接近 100%；生成模型只能"大致吻合" |
| VLM 打分 | VLM 对几何与运动学参数的合理性打分（0.94） | PhysX-Anything | **☆** 运动学合理性应占优；外观（纯色无贴图）可能拖低总分 |
| **确定性可复现率**（文献里少见，可自设） | 同 seed 重复编译是否产出 bit-identical 资产 | —（Infinigen 系隐含） | **★ 优势**：mechanical hash + 确定性编译，可报 100%；扫描类与生成式数据集都做不到 |

### 2.5 下游任务（最有说服力的一层）
| 指标 | 设置 | 出处 | 本项目 |
|---|---|---|---|
| 可动部件分割 **mAP**（mAP50/75/small/medium/large） | PartNet 15k vs 30k vs 15k+Infinigen 15k；48.23 → 50.13，小部件增益最大 | Infinigen-Sim | **☆ 可能优势**：类别覆盖比 Infinigen-Sim 广得多、且分割真值免费精确；风险是纯色渲染的域差可能吃掉增益 |
| RL 在未见 PartNet 实例上的成功率 | 加入程序化资产共训 | Infinigen-Sim | **☆** 机制同上；需要惯量/摩擦补齐才谈得上 |
| **Sim-to-real 成功率** | 只用程序化资产训 → 真机 7/10；只用 PartNet → 0/10 | Infinigen-Sim | **☆ 未知**：需要真机；若做，是最有分量的一张表 |
| IL（ACT/DP）真机成功率 | RO / SO / 真+仿混训，60 次 rollout | ArtVIP（如 PullDrawer 64%→81%） | **⚠** ArtVIP 走的是"高保真数字孪生"路线（贴图 + 动捕标定关节参数），我们在这条赛道上目前不占优 |
| 仿真-真实相关性 | Pearson 相关系数 0.9886 | ArtVIP | **⚠** 同上，依赖物理参数标定 |
| 训练 feed-forward 关节估计模型 | 用生成数据训 Particulate，测 Lightwheel：gIoU 0.332→0.394 | Articraft | **★ 可能优势且最容易接**：与底层系统同实验台，直接加一臂"+模板采样数据"看是否再涨；我们能提供的数据量不受标注成本限制 |
| 策略成功率提升 | 仿真 9.7%→79.8%，真机 21.7%→75.0% | EmbodiedGen V2 | **◐** 属于系统级论证，跟资产质量不是一一对应 |

---

## 3. 映射到本模板管线

记号：**✅ 已具备/可直接算**、**🔧 需适配**、**⛔ 不适用**。

### 3.1 直接可用（今天就能出数，且大概率是我们的强项）

| 优劣 | 论文指标 | 适配到模板管线 | 预期结论 |
|---|---|---|---|
| ★★ | **AOR**（CAGE） | 我们的运动 QC 已经在做**更严格**的版本：精确网格（FCL + manifold）而非包围盒，且覆盖所有部件对而非仅 sibling。可直接报告 **AOR = 0**，并额外报告"精确网格 AOR"与"bbox AOR"两栏 | 生成模型 AOR 通常 >0；我们结构上为 0，是硬优势 |
| ☆ | **%Stable / Rotation angle**（DSO/Atlas3D） | 导入 MuJoCo（本机已装 3.10）加 freejoint + 地面，静置 N 步，测根 link 倾角与位移。**再加一项文献没有的**：沿关节行程扫掠取最坏值（门全开时倒不倒） | 需先补惯量，否则测的是引擎默认密度 |
| ☆ | **重心-接触凸包判据**（PhysComp） | 纯几何：体积加权 CoM + 接地点凸包，输出归一化裕度与倾覆角。复用 `geometry_qc.compute_part_world_transforms` | 可当"静态稳定性"闭式指标，不依赖仿真器 |
| ☆ | **运动学图 perplexity**（Artiverse） | 把每个 seed 的 URDF 转成 (link 数, 关节类型序列, 父子边) 规范化图，统计分布熵的指数。可按模板内 / 库级两个粒度报 | 模板天生跨 seed 换拓扑，这项应显著高于 PartNet-Mobility |
| ★★ | **参数维度数 + 估计唯一资产数**（Infinigen-Sim Table 1） | 我们的 core/raw 组合数 + 连续参数维度，就是同一张表；已有 per-template 统计（core 36/48/72/96/108…），只差汇总 | 直接对标 Infinigen-Sim 的 10⁶–10²⁰ 口径 |
| ★★ | **人工返修率 / Human Acceptance**（EmbodiedGen、Articraft） | 我们的人工视觉关口③签核率 + 机械闸门通过率（`sweep_state.pass_rate`）已在盘上 | 可报"通过闸门后 0% 人工返修" |
| ★★ | **生成开销**（Articraft $1.13/资产） | 我们要报的是**摊销比**：造一个模板的 agent 成本 ÷ (组合域 × 通过率)。这是模板 vs 逐资产生成的核心经济性论据 | 量级上应低 1–3 个数量级 |
| ⚠ | **属性完备性打勾表**（Artiverse） | 直接做我们 vs PartNet-Mobility / ArtVIP / Artiverse / Articraft-10K 的对比表 | 但要先补 per-part 质量/材料（现 13.6%），否则这张表我们会输 |
| ☆ | **导出合规 / 跨仿真器**（综述 §7） | 分三级 × 四引擎（MuJoCo/PyBullet/Genesis/Isaac）；本机已有 MuJoCo+Genesis | 已验证 MuJoCo 解析 40/40 |

### 3.2 需要适配（改造后有效，成本中等）

| 优劣 | 论文指标 | 适配方案 | 注意 |
|---|---|---|---|
| ⚠ | **Collision Success Rate**（EmbodiedGen：SAPIEN 抓取抬升 ×4 yaw） | 本机无 SAPIEN，用 MuJoCo/Genesis + 平行夹爪脚本复现同一协议；对 articulated 资产还应加"抓把手并开合"变体 | 需要惯量与摩擦参数，否则测的是默认值 |
| ★ / ⚠ | **接触几何保真 E_{Q→P}/E_{P→Q}**（Real-IKEA） | 我们的 collision 与 visual **逐元素完全相同**（实测 4318/4318），所以原始偏差为 **0**——这是可以直接写进论文的强结论。真正要测的是：跑 CoACD 生成凸分解代理后，偏差随组件数的曲线，以及**凸包近似下 AOR 是否由 0 变正** | 这是本项目最大的隐藏风险点：QC 用精确网格证的"无穿模"在按凸包算碰撞的引擎里不一定成立（实测 28% 的 mesh 凸包/实体体积比 > 2） |
| ⚠→☆ | **物理参数真实性**（PhysX-3D、Artiverse） | Artiverse 的做法可直接抄：质量 = 近似体积 × 材料采样密度。我们的模板里**已经有 material 名**，补一张材料→密度表即可自动生成 per-part 质量与惯量，把 13.6% 提到 ~100% | 低成本高收益，且解锁上面一半的物理指标 |
| ☆ | **可动部件分割 mAP**（Infinigen-Sim） | 需要渲染管线（repo 已有 `blender_gallery`）+ 训 Mask R-CNN，按 P15k / P30k / P15k+Ours 三臂对比 | 工作量最大但说服力最强；是"程序化资产有用"的行业标准证明 |
| ★ | **训练关节估计模型**（Articraft → Particulate/Lightwheel） | 与 Articraft 论文同协议，加一臂"+模板采样数据"，看 gIoU/AS-gIoU 是否再涨 | 与底层系统同实验台，最容易接上 |
| ☆ | **ID / MMD / COV / 1-NNA**（CAGE 系列） | 我们不是"给条件生成"的模型，但可以反过来用：以 PartNet-Mobility 同类对象为参考集，报告 **COV（我们的采样覆盖了多少真实实例）** 和 1-NNA（分布是否可区分） | COV 高 + 1-NNA 接近 50% = "模板分布覆盖真实分布且难以区分"，正是程序化方法要的结论 |
| ★ | **条件一致性 / 可控性**（CAGE/SINGAPO） | 我们的版本：给定目标属性（高度、门数、开合角、抽屉数）→ 采样 → 实测值与目标的误差 / 命中率 | 这是模板独有且文献有对应协议的指标 |
| ☆ | **VLM 打分**（PhysX-Anything 0.94） | 用 VLM 判"这是不是 X 类 + 结构是否合理"，作为人工视觉关口③的自动代理 | 可把唯一不可机器化的关口部分自动化 |
| ⚠ | **CLIP/t-SNE 分布对照**（ArtVIP） | 渲染图 CLIP 特征 vs 真实照片，看覆盖范围 | 用于回应"程序化资产看起来假"的质疑 |

### 3.3 不适用 / 不建议

| 指标 | 原因 |
|---|---|
| CD / F-score / PSNR vs GT，关节轴/原点/限位误差 | 这些是**重建类**任务的指标，需要成对 GT。我们是分布生成，没有配对真值。**例外**：若做"参考图 → 模板参数拟合"这一支，则完全适用 |
| 光学动捕轨迹对照（ArtVIP） | 需要动捕设备与实物样本；除非某一类做数字孪生，否则性价比低 |
| Col-S / Inst-S（场景级） | 我们是单资产管线，没有场景合成 |
| 形变体 / 布料指标 | 不在范围内 |
| 标注效率指标（Artiverse 人工时长节省） | 我们没有人工标注环节；对应指标应换成"每有效资产的 agent 成本" |

---

## 4. 建议的评测套件（论文表格骨架）

每列后标注该列对我们是加分项还是风险项。

**Table 1 — 数据集属性对比**（对标 Artiverse/ArtVIP 表）
| 列 | 本项目 |
|---|---|
| 类别数 / 模板数 | ★★ 537 类，全表最高 |
| 可采样资产量级 / 连续参数维度 | ★★ 对标 Infinigen-Sim 10⁶–10²⁰ 口径 |
| revolute / prismatic 关节数 | ◐ 按需生成，数量不是问题 |
| 多 DoF 关节 | ☆ 有 mimic 闭链，需核对是否算多 DoF |
| 部件语义标签 / 关节真值 / 参数真值 | ★ 构造即真值，零标注 |
| per-part 材料 | ★ 模板里已有 material 名 |
| **per-part 质量 / 度量尺度** | **⚠ 当前缺（13.6%）——不补这一格就空着** |
| 人工工时每资产 | ★★ 对比 ArtVIP 的 1.8–15 h/对象 |

**Table 2 — 机械合法性与 sim-readiness**（**本项目主场**）
行：Ours / Articraft-10K / PartNet-Mobility / Articulate-Anything / PhysX-Anything / URDF-Anything+
| 列 | 本项目 |
|---|---|
| URDF 解析率 / 四引擎加载率 / 首步仿真无异常率 | ☆ MuJoCo 已验 40/40，其余待测 |
| 静止态穿模率 / 悬空件率 | ★★ 硬闸门，结构上为 0 |
| **AOR(精确网格)** | ★★ 结构上为 0，最强的一格 |
| **AOR(凸包)** | ⚠ 未知风险，必须自己先测 |
| 碰撞代理偏差 E_{Q→P} | ★ 碰撞=视觉，偏差恒 0 |
| 水密率 | ⚠ 实测 50%，是弱项 |
| 人工返修率 | ★★ 0%，且返修成本按模板摊销 |

**Table 3 — 物理稳定性**（文献有先例，我们加"关节态"这一列）
| 列 | 本项目 |
|---|---|
| %Stable(静止) / Rotation angle / CoM 支撑裕度 / 倾覆角 | ☆ 补惯量后可能优势 |
| **%Stable(最坏关节态)** | ★ 文献没有这一列，是我们能新增的指标 |
| Collision Success Rate(抓取抬升) | ⚠ 缺凸分解与摩擦，当前会输 |

**Table 4 — 多样性与可控性**（**模板独有，第二主场**）
| 列 | 本项目 |
|---|---|
| core 组合数 / 估计唯一资产数 | ★★ |
| 运动学图 perplexity | ☆ 应超 Artiverse 的 1.5×，**必须实测**（若模板拓扑单一则反成劣势） |
| 跨 seed 结构非重复率 | ☆ 需先自动化"薄壳"检测 |
| 参数-几何响应率 / 目标属性命中率 | ★ 可控性是构造出来的 |
| COV & 1-NNA vs PartNet-Mobility | ☆ COV 占优，1-NNA 双刃 |
| 确定性可复现率 | ★ 100% |
| 每有效资产成本 / 摊销比 | ★★ |

**Table 5 — 下游效用**（照 Infinigen-Sim / Articraft 协议）
| 列 | 本项目 |
|---|---|
| 关节估计 gIoU/AS-gIoU（±模板数据） | ★ 最易接、与 Articraft 同实验台 |
| 可动部件分割 mAP（P15k / P30k / P15k+Ours） | ☆ 类别广是优势，纯色域差是风险 |
| RL 未见实例成功率 | ☆ 依赖物理参数补齐 |
| 真机 sim2real | ⚠/☆ 未知，成本最高

---

## 5. 落地顺序

1. **补 per-part 质量/惯量**（用模板里已有的 material 名 + 密度表，Artiverse 口径）——它是 Table 1/2/3 一半指标的前置条件，成本最低。
2. **AOR(精确) vs AOR(凸包) 双栏 + 碰撞代理偏差**——把"我们的无穿模保证能不能过渡到真实仿真器"这个风险量化掉。
3. **稳定性三件套**（%Stable / Rotation angle / 最坏关节态）——纯几何版先出，MuJoCo 版随后。
4. **多样性四件套**（组合数、图 perplexity、非重复率、可控性命中率）——数据已在 `sweep_state` 与导出 URDF 里，基本只是统计脚本。
5. **跨仿真器三级导入矩阵**。
6. **下游 mAP / 关节估计**——最后做，最贵也最有分量。

---

## Sources

- [Infinigen-Sim: Procedural Generation of Articulated Simulation Assets (arXiv 2505.10755)](https://arxiv.org/abs/2505.10755)
- [Articraft: An Agentic System for Scalable Articulated 3D Asset Generation (arXiv 2605.15187)](https://arxiv.org/abs/2605.15187)
- [Articulate-Anything (arXiv 2410.13882, ICLR 2025)](https://arxiv.org/abs/2410.13882)
- [URDF-Anything+: End-to-End Generation for Simulation-Ready Articulated Assets (arXiv 2603.14010)](https://arxiv.org/html/2603.14010)
- [PhysX-Anything: Simulation-Ready Physical 3D Assets from Single Image (arXiv 2511.13648)](https://www.alphaxiv.org/overview/2511.13648)
- [PhysX-3D: Physical-Grounded 3D Asset Generation (arXiv 2507.12465)](https://arxiv.org/abs/2507.12465)
- [EmbodiedGen V2: An Agentic, Simulation-Ready 3D World Engine (arXiv 2607.07459)](https://arxiv.org/html/2607.07459v1)
- [ArtVIP: Articulated Digital Assets of Visual Realism, Modular Interaction, and Physical Fidelity (arXiv 2506.04941)](https://arxiv.org/html/2506.04941v3)
- [Artiverse: A Diverse and Physically Grounded Dataset for Articulated Objects (arXiv 2605.24403)](https://arxiv.org/html/2605.24403v1)
- [Real-IKEA: Physical Fidelity is the Prerequisite for Robust Manipulation (arXiv 2606.08564)](https://arxiv.org/pdf/2606.08564)
- [CAGE: Controllable Articulation GEneration (arXiv 2312.09570)](https://arxiv.org/html/2312.09570)
- [SINGAPO: Single Image Controlled Generation of Articulated Parts in Objects (arXiv 2410.16499)](https://arxiv.org/pdf/2410.16499)
- [ArtFormer: Controllable Generation of Diverse 3D Articulated Objects (arXiv 2412.07237)](https://arxiv.org/pdf/2412.07237)
- [ArtiLatent: Realistic Articulated 3D Object Generation via Structured Latents (arXiv 2510.21432)](https://arxiv.org/pdf/2510.21432)
- [PhysPart: Physically Plausible Part Completion for Interactable Objects (arXiv 2408.13724, ICRA 2025)](https://arxiv.org/html/2408.13724v1)
- [Atlas3D: Physically Constrained Self-Supporting Text-to-3D (arXiv 2405.18515, NeurIPS 2024)](https://arxiv.org/abs/2405.18515)
- [DSO: Aligning 3D Generators with Simulation Feedback for Physical Soundness (ICCV 2025)](https://openaccess.thecvf.com/content/ICCV2025/papers/Li_DSO_Aligning_3D_Generators_with_Simulation_Feedback_for_Physical_Soundness_ICCV_2025_paper.pdf)
- [3D Generation for Embodied AI and Robotic Simulation: A Survey (arXiv 2604.26509)](https://arxiv.org/pdf/2604.26509)
- [CoACD: Approximate Convex Decomposition with Collision-Aware Concavity (SIGGRAPH 2022)](https://arxiv.org/pdf/2205.02961)
