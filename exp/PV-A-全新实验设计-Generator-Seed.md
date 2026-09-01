# PV-A 全新实验设计：Generator（Template）与 Seed 分层评估

> 版本：v2，2026-08-09  
> 本文是全新方案，不继承旧方案中“template quality = Agent 写模板的成功率”的定义。  
> 依据：`Paper-metrics.docx`、`PV_A.pdf`、项目当前 `.agents` 工作流、`arti-template/agent/templates/`、`Nano3dresults.md`、现有 T2–T5 真实结果，以及各 baseline 原论文与公开实现。

## Revised paper-ready experimental tables

下面是应该放在论文实验部分的结果表，不是实验清单。`—` 为待实验回填，`N/A` 为任务或输出不支持，`L` 为统一协议本地复现，`R` 为原论文数字。`R` 与 `L` 不在同一列中排名。

**Baseline 准入原则。** 对照方法按评测对象进表：Infinite Mobility、Infinigen-Sim 和 Arti-PG 与 PV-A 比 reusable generator；NAP/CAGE/ArtFormer/LAM 与 PV-A 比生成资产分布；LAM/Articraft/Nova3D 与 PV-A 比 code-native seed；UniPhysGen/PhysX-Omni 只在共享输出协议的 physics panel 中出现。

### Table 1. Audit of the implemented PV-A generator fleet

| Fleet subset | Importable generators | Formal SourceMap | Formal Design | Domain basis | Core domain, median [Q1,Q3] | Raw domain, median [Q1,Q3] | Unsaturated probes |
|---|---:|---:|---:|---|---:|---:|---:|
| Design-backed | 106 | 106 | 106 | exact declared product; reachability unproven | 80 [48, 210] | 216 [128, 840] | N/A |
| Legacy | 425 | 0 | 0 | observed axes from ≤2,000 seeds | 64 [30, 216] | 240 [81, 1,050] | 192 / 425 |
| **All PV-A generators** | **531** | **106** | **106** | **mixed; not one exact space** | **—** | **—** | **192 / 531** |

**为什么这张表在最前面。** PV-A 的方法 claim 是 SourceMap → TemplateDesign → generator 的自动构建，但当前只有 106/531 个 generator 有这两个正式产物。所以全库规模能证明“有多少可执行 generator”，不能单独证明“完整 PV-A pipeline 构建了它们”。legacy domain 又来自 seed probe，其中 192 个未饱和；因此不能把 15.2M/86.1M 总和当成精确可达组合数写进 abstract。

### Table 2. Unseen-category construction of reusable generators

冻结 24 个未见类别，按结构复杂度分层；每个 arm 用同一 authoring model/budget 独立重复 3 次。测试 seed、cross-slot pairs 和连续极值在 generator hash 冻结后才公开。

| Method | First-shot gen. success ↑ | Final gen. success ↑ | Hidden valid yield ↑ | Pair coverage ↑ | Eval-corner all-pass ↑ | Family recall ↑ | Old-case retention ↑ | Authoring time (min) ↓ | LLM cost ($) ↓ |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Naive same-LLM reusable-program prompt | — | — | — | — | — | — | — | — | — |
| w/o SourceMap | — | — | — | — | — | — | — | — | — |
| w/o TemplateDesign | — | — | — | — | — | — | — | — | — |
| w/o cross-component adaptation | — | — | — | — | — | — | — | — | — |
| w/o distribution harness/regression | — | — | — | — | — | — | — | — | — |
| **PV-A full** | **—** | **—** | **—** | **—** | **—** | **—** | **—** | **—** | **—** |

**这些列为什么检验 PV-A。** `First-shot` 测 Agent 是否一次把类别编成 generator；`Hidden valid yield` 测它是否只会生成开发时见过的 seeds；`Pair coverage` 要求每两个 slot candidate 的组合至少被独立测到，正面检验 PV-A 声称的跨组件适配；`Old-case retention` 检查修理新组合时是否破坏旧组合。这张表不放 Infinite Mobility，因为它的 generator 是人工编写，不是同一个“自动 authoring”任务。

### Table 3(a). Quality of reusable procedural generators

`P5` 为 Infinigen-Sim 的 5 个共同类（door, toaster, refrigerator, dishwasher, lamp）；`P12` 为 PV-A 与 Infinite Mobility 的 12 个一对一 factory 共同类。Arti-PG 仅在其开源 26 类中与 PV-A 严格重叠的子集上报告。每类 200 次 attempts，不补采成固定成功数。

| Method | Set | Valid yield ↑ | Category precision ↑ | Family recall ↑ | Pair coverage ↑ | Valid novel ↑ | Graph PPL ↑ | NTED ↑ | Shape-Vendi ↑ | Near-dup. ↓ | Strict CF ↑ | Valid assets/h ↑ |
|---|:---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Infinigen-Sim | P5 | — | — | — | — | — | — | — | — | — | — | — |
| Arti-PG | shared | — | — | — | — | — | — | — | — | — | — | — |
| Infinite Mobility | P12 | — | — | — | — | — | — | — | — | — | — | — |
| PV-A size-only | matched | — | — | — | 0 | 0 | — | — | — | — | — | — |
| PV-A no-recombination | matched | — | — | — | — | 0 | — | — | — | — | — | — |
| **PV-A** | **matched** | **—** | **—** | **—** | **—** | **—** | **—** | **—** | **—** | **—** | **—** | **—** |

**为什么要这样比。** Infinigen-Sim 原论文会数离散组件笛卡尔积和连续自由度，Infinite Mobility 报告 joint 数、joint 数方差、raw Tree Edit Distance 和生成时间。PV-A 也声称“组件重组 + 可控参数 + 大规模廉价采样”，所以应对照这些轴。但笛卡尔积会把不可达或 resolver 折叠的组合也算进去，所以主表用实际 `Pair coverage` 和 `Valid novel`。raw TED 会奖励“拆成更多 joint”，所以用 `NTED` 和 `Graph PPL`。`Valid assets/h` 把失败资产留在分母。

### Table 3(b). Blinded perceptual comparison on compatible render modalities

每格为 `PV-A win / tie / baseline win (%)`。每个 baseline 使用自己与 PV-A 的严格重叠类别，不跨行比较绝对百分比。

| Baseline | Set | #Pairs | Articulation W/T/L ↑ | Geometry W/T/L ↑ | Category fidelity W/T/L ↑ | Appearance W/T/L ↑ | Per-sample plausibility (PV-A / Base) ↑ |
|---|:---:|---:|---:|---:|---:|---:|---:|
| CAGE | P4 | — | — | — | — | N/A | — |
| ArtFormer | P4 | — | — | — | — | N/A | — |
| LAM | P4 | — | — | — | — | — | — |
| Articraft | shared | — | — | — | — | — | — |
| Infinigen-Sim | P5 | 125 | — | — | — | — | — |
| Arti-PG | shared | — | — | — | — | as available | — |
| Infinite Mobility | P12 | 300 | — | — | — | — | — |

**为什么这些人评轴不能合成一个“好看”分数。** Infinite Mobility 原文分开 textureless motion video、normal map 和 RGB render，分别判断 articulation、geometry 和 texture；CAGE 使用 human plausibility，ArtFormer 使用 alignment/diversity 人评，Articraft 则让人选质量与 prompt alignment 最好的资产。对 PV-A 而言，运动差可能是轴/pivot/range 错，normal-map 差可能是组件适配或连接几何错，RGB 差才主要是材质和外观。分开后才能回到对应模块修复。`Per-sample plausibility` 是每个资产独立 1–5 分，用来防止 pairwise tie 掩盖“两个都差”。VLM 只作大规模 proxy，必须先在这些人评上校准。

### Table 4. Distribution fidelity on four shared PartNet-Mobility categories

主表只取 CAGE 和 ArtFormer 真正共同的 Storage Furniture, Safe, Oven, Washer 四类；PV-A source pool 与 official test assets 做 mesh/part 近重复去除。每种方法每类 200 次 attempts，集合指标在按 attempt 顺序得到的全部 `N_valid` 输出上计算，不挑样、不补采；`1-NNA` 每次从 real reference 抽等量 `N_valid`，重复 20 次以避免集合大小偏差。同时保留 yield。

| Method | Representation | Valid yield ↑ | MMD-ID ↓ | COV-ID ↑ | 1-NNA-ID →50 | MMD-AID ↓ | COV-AID ↑ | 1-NNA-AID →50 | CLIP-R ↑ | Near-dup. ↓ |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| NAP | direct part geometry | — | — | — | — | — | — | — | — | — |
| CAGE | graph + part retrieval | — | — | — | — | — | — | — | — | — |
| ArtFormer | direct SDF geometry | — | — | — | — | — | — | — | — | — |
| ArtFormer-PR | generated tree + part retrieval | — | — | — | — | — | — | — | — | — |
| LAM | text-to-code per asset | — | — | — | — | — | — | — | — | — |
| Source-pool resampling | retrieval lower control | — | — | — | — | — | — | — | — | — |
| **PV-A** | **source-grounded procedural** | **—** | **—** | **—** | **—** | **—** | **—** | **—** | **—** | **—** |

**ID/AID 到底比什么。** CAGE 将 `ID` 定义为 5 个同步关节状态下、每个部件点云 Chamfer-L1 的平均；它会同时惩罚“门板形状不像”和“门打开后位置不对”。`AID` 把部件表面换成包围盒 vIoU，主要看部件布局和运动。PV-A 如果 `AID` 好但 `ID` 差，说明 part layout 大体对，但来源组件保真或适配有问题；两者都差，则更像 blueprint/关节结构错。

`MMD` 问“每个真实样本能不能找到一个像它的生成样本”；`COV` 问“真实集合有多少不同区域被覆盖”；`1-NNA` 问 real/generated 是否容易被最近邻分开，50% 最好。它们检验 PV-A seed 是否只在少数 source 周围打转，但不能证明 collision-free、可控或 native simulation-ready。

### Table 5. Seed-level structure and kinematics

#### (a) Code-native structure on a common prompt/edit benchmark

| Method | Edit mode | Executable ↑ | Artifact saved ↑ | Semantic part F1 ↑ | Hierarchy-edge F1 ↑ | Numeric/count constraints ↑ | Edit success ↑ | Edit locality ↑ |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| Nova3D | local source edit | — | — | — | — | — | — | — |
| LAM | per-asset source/regeneration | — | — | — | — | — | — | — |
| Articraft | local asset-program edit | — | — | — | — | — | — | — |
| **PV-A config** | **predeclared in-domain control** | **—** | **—** | **—** | **—** | **—** | **—** | **—** |
| **PV-A generator edit** | **new cross-seed structural edit** | **—** | **—** | **—** | **—** | **—** | **—** | **—** |

#### (b) Functional kinematic audit on articulated outputs

| Method | Native URDF ↑ | Joint recall ↑ | Joint type acc. ↑ | Axis err. (°) ↓ | Pivot err./diag ↓ | Range mIoU ↑ | Kinematic all-correct ↑ |
|---|---:|---:|---:|---:|---:|---:|---:|
| LAM | — | — | — | — | — | — | — |
| Articraft | — | — | — | — | — | — | — |
| Infinigen-Sim | — | — | — | — | — | — | — |
| Infinite Mobility | — | — | — | — | — | — | — |
| **PV-A** | **—** | **—** | **—** | **—** | **—** | **—** | **—** |

**为什么分成两个 panel。** Nova3D 的主产物是 Blender source + GLB，原论文主要证明可执行性、named parts、assembly tree、prompt 约束和 local edits；把它强行填进 URDF 或原生物理列会把输出格式差异当成质量差异。Panel (a) 检验 PV-A 是否真的产生“可编辑结构化资产”。但 PV-A 的预声明 config 操作天然比“让 Agent 写一个新 edit”容易，因此必须分成 `PV-A config` 和 `PV-A generator edit`；只有后者与 Nova3D/Articraft 的新 local edit 比较，前者用于测 in-domain controllability。

LAM joint success 来自 masked-URDF reconstruction，UniPhysGen type/axis/pivot/range 也来自有 paired GT 的 grounding benchmark；PV-A 不是固定物体重建。Panel (b) 因而对每个最终输出独立标注“应该动的部件、功能轴、pivot 和 range”，再计算 UniPhysGen 式误差。`Kinematic all-correct` 要求所有 functional joints 都正确。

### Table 6. Full-range articulation and collision

#### (a) Four-category learned/code-generated comparison

| Method | Motion valid ↑ | CAGE AOR ↓ | ArtFormer POR ↓ | Discrete CF ↑ | Strict continuous CF ↑ | Max penetration/diag ↓ |
|---|---:|---:|---:|---:|---:|---:|
| NAP | — | — | — | — | — | — |
| CAGE | — | — | — | — | — | — |
| ArtFormer | — | — | — | — | — | — |
| LAM | — | — | — | — | — | — |
| **PV-A** | **—** | **—** | **—** | **—** | **—** | **—** |

#### (b) Procedural/per-asset generator comparison on shared categories

| Method | Motion valid ↑ | CAGE AOR ↓ | ArtFormer POR ↓ | Discrete CF ↑ | Strict continuous CF ↑ | Max penetration/diag ↓ |
|---|---:|---:|---:|---:|---:|---:|
| Articraft | — | — | — | — | — | — |
| Infinigen-Sim | — | — | — | — | — | — |
| Infinite Mobility | — | — | — | — | — | — |
| **PV-A** | **—** | **—** | **—** | **—** | **—** | **—** |

**AOR/POR 为什么有用、又为什么不够。** CAGE AOR 把各 articulation state 中 sibling parts 的 oriented-bbox 重叠当作物理合理性 proxy；它能廉价地找出“两个抽屉互相占空间”，但看不到 ancestor-child 穿透，也会把轴套/轴这种合法捕获接触当成错。ArtFormer POR 在 10 个均匀状态上平均所有部件对 vIoU；“平均很小”仍可能掩盖某个狭窄状态的严重碰撞。

因此 AOR/POR 用于与 CAGE/ArtFormer 对齐和诊断，`Strict continuous CF` 才检验 PV-A 的“全行程可仿真” claim。它使用 exact FCL + adaptive continuous certificate，超时或未认证都算失败。项目 pilot 的 33/33 离散通过但只有 12/33 strict 通过，已证明这两列不能合并。

### Table 7. Native physical completeness and simulator readiness

所有行都评估 method-native package，进表前不补 collision、mass 或 inertia。UniPhysGen 是 raw-asset grounding 方法，PhysX-Omni 是 image-conditioned physical generation；它们在本表只比“最终产物是否原生可部署”，不比 category-generation 质量。

| Method-native output | Task | Collision complete ↑ | Mass/inertia complete ↑ | Dynamics complete ↑ | Physically plausible values ↑ | Strict CF ↑ | PyBullet L5 ↑ | MuJoCo L5 ↑ | SAPIEN L5 ↑ | 3-sim all-pass ↑ | Worst-state stable ↑ |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Articraft | text-to-asset | — | — | — | — | — | — | — | — | — | — |
| Infinigen-Sim | reusable generator | — | — | — | — | — | — | — | — | — | — |
| Infinite Mobility | reusable generator | — | — | — | — | — | — | — | — | — | — |
| UniPhysGen | raw-asset grounding | — | — | — | — | — | — | — | — | — | — |
| PhysX-Omni | image-to-physical asset | — | — | — | — | — | — | — | — | — | — |
| **PV-A** | **reusable generator** | **—** | **—** | **—** | **—** | **—** | **—** | **—** | **—** | **—** | **—** |

**为什么不只报“能 load”。** UniPhysGen 把 simulation-ready 拆成 articulation semantics 与 material/density/friction/scale/mass；PhysX-Omni 也把 geometry、absolute scale、material、affordance 和 kinematics 分开。PV-A pilot 中 URDF compiler 能 189/189 复制 joint 字段，但这只证明导出器忠实复制 source intent，不证明 intent 正确；原生 collision+inertial 同时完整仅 53/244 links 和 9/33 assets。`Mass/inertia complete` 与 `Physically plausible values` 必须分开：填了一个正数不等于材料、尺度和惯量真实。normalized evaluation copy 只进 supplementary diagnosis。

### Table 8. Amortized production cost

所有方法用同一类别 prompt/spec、同一 strict-valid gate 和同一硬件。per-asset 方法每个新资产重新调 Agent；reusable 方法的 generator authoring 成本只计一次。

| Method | Production unit | Category setup/author (h) ↓ | LLM calls per additional asset ↓ | Marginal strict-valid latency (s) ↓ | Valid assets/h ↑ | Total cost @1 ↓ | @10 ↓ | @100 ↓ | @1K ↓ | Break-even N* ↓ |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| LAM | one asset program | — | >0 | — | — | — | — | — | — | N/A |
| Articraft | one asset program | — | >0 | — | — | — | — | — | — | N/A |
| Nova3D | one Blender program | — | >0 | — | — | — | — | — | — | N/A |
| Infinigen-Sim | manual reusable generator | N/R manual labor | 0 | — | — | — | — | — | — | — |
| Infinite Mobility | manual reusable factory | N/R manual labor | 0 | — | — | — | — | — | — | — |
| **PV-A** | **agent-built reusable generator** | **—** | **0** | **—** | **—** | **—** | **—** | **—** | **—** | **—** |

**这张表是 PV-A 中心 claim 的直接测量。** LAM、Articraft 和 Nova3D 都将模型推理放在每个资产上；PV-A 把这个成本移到每个 category generator 上。只报“seed 生成几秒”会隐藏 PV-A 的一次性 Agent 成本，只报“每资产 LLM 调用数”又会隐藏它在 N=1 时可能不划算。`Total cost @N = C_author + N·C_marginal/valid_yield`；`Break-even N*` 是 PV-A 首次低于同类 per-asset baseline 的 N。Infinite Mobility 论文的 0.46 s/object 只能作 context，因为没有计入人工编写 factory 的小时数。

### Table 9. Downstream utility under equal usable-data budgets

| Training data | Valid extra assets | Rest gIoU ↑ | Rest mIoU ↑ | Art. gIoU ↑ | Art. PC ↓ | OC ↓ | Family-disjoint Art. gIoU ↑ | Component-disjoint Art. gIoU ↑ | OOD Art. gIoU ↑ |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Base training set | 0 | — | — | — | — | — | — | — | — |
| Base + Artiverse | B_eq | — | — | — | — | — | — | — | — |
| Base + Articraft | B_eq | — | — | — | — | — | — | — | — |
| Base + Arti-PG | B_eq | — | — | — | — | — | — | — | — |
| Base + Infinigen-Sim | B_eq | — | — | — | — | — | — | — | — |
| Base + Infinite Mobility | B_eq | — | — | — | — | — | — | — | — |
| Base + PV-A size-only | B_eq | — | — | — | — | — | — | — | — |
| **Base + PV-A** | **B_eq** | **—** | **—** | **—** | **—** | **—** | **—** | **—** | **—** |
| Base + PV-A | 10K | — | — | — | — | — | — | — | — |
| Base + PV-A | 30K | — | — | — | — | — | — | — | — |

**为什么用这些对照和指标。** Infinigen-Sim 用程序化资产训练 movable-part segmentation 和操作策略；Infinite Mobility 用自身数据训练 CAGE；Articraft 用 Articraft-10K 增强 Particulate。因此这些数据源与 PV-A 的“可规模化训练数据” claim 真正可比。`B_eq = min(5,000, 所有入表数据源在共同类别配额下能交付的最小 strict-valid 数)`；这样不会因 Artiverse 全集只有约 5.4K、或某个共同类样本少而偷偷改变类别分布。PV-A 10K/30K 是另外的 scaling rows，不与 B_eq 行宣称 equal budget。`gIoU/mIoU` 检验部件分解；`Art. PC` 检验各部件运动到最大行程后的形状；`OC` 是最大行程姿态下的 whole-object Chamfer，不是 collision metric。`Family-disjoint` 屏蔽完整 generator/factory，`Component-disjoint` 屏蔽来源组件，用来区分“记住同一个模板”和“学会跨结构泛化”。CAGE/ArtFormer 只有在能交付同一 schema 的 B_eq strict-valid assets 时才增加行。

### Metrics deliberately excluded from the main tables

- Artiverse rest/articulated `dgIoU`/`dcDist` 和 PhysX-Omni paired `PSNR/CD/F-score` 需要一对一 GT；PV-A category seed 不应被假设成必须重建某个 GT。它们只放在 paired source-reference supplementary subset。
- UniPhysGen density/friction/scale/mass ALDE/MnRE 需要真实物理标注。PV-A 的实心铝块先验 normalized copy 不是 GT，不能伪装成 mass accuracy。
- CLIP/BLIP/VLM 只是类别和视觉对齐 proxy，不能替代 joint gold、strict collision 或 simulator L5。


## 0. 先把评测对象定义正确

PV-A 中的 template 不是一次“模板编写任务”，而是一个类别级资产生成器：

\[
G_c:(z,u)\rightarrow x
\]

- `c`：类别；
- `z`：随机 seed；
- `u`：组件选择、multiplicity、连续参数等可控配置；
- `x`：一个带 geometry、parts、joints、limits、collision 和 provenance 的 articulated asset。

因此有三个不同统计对象：

| 层级 | 评测对象 | 核心问题 |
|---|---|---|
| Generator / Template | 整个 `G_c` 及其输出分布 | 这个生成器覆盖了怎样的物体族，是否多样、可控、组合合理、稳定且高效？ |
| Seed / Asset | 单个 `x=G_c(z,u)` | 这个具体资产是否语义正确、几何合理、运动正确、可导入并可仿真？ |
| Dataset | 多 generator、多 seed 的联合数据 | 数据是否覆盖长尾，并能提升独立下游任务？ |

模板质量不能用若干 seed 的平均视觉分替代；反过来，单 seed 的 watertight、AOR 或 simulator pass 也不能说明生成器有覆盖度和可控性。

这里必须区分两件事：authoring first-shot、repair turns 和 Agent token 不是“已生成 template 的输出质量”，但它们是 PV-A “Agent 能自动构建 reusable generator”的核心方法证据。因此它们不进 Generator Quality 表，却必须单独进主文的 unseen-category authoring/ablation 表（Table 2），不能降成只有附录成本。

## 1. 当前证据告诉我们的真实问题

正式实验前必须冻结 manifest，不能直接沿用论文中的“500 类/300K”或历史文档中的模板数。当前磁盘快照可见：

- `arti-template/agent/templates/` 顶层有 534 个 `.py`，报告脚本确认 531 个可导入 generator；
- 只有 106 个正式 SourceMap + TemplateDesign + `TEMPLATE_DOMAIN`，另有 425 个 legacy generator；
- `metrics_test/` 有 500 个顶层 cohort，这不等同于 500 个语义不重复 category；
- `seed_exports/` 当前有 675 个 URDF；针对性搜索未找到本地完整 PV-A-300K materialization；
- 项目论文声称 500 categories、300K simulation-ready assets，这两个数都需要 release manifest 证明，不能用 generator/cohort 数代替。

深入核对当前实现后，另外有五个会直接改变实验结论的不一致：

1. `PV_A.pdf` 把 SourceMap 描述为保存构件代码，但当前 workflow 明确规定 SourceMap 只记录源池事实、role、provenance 和代码行证据；最终单文件 template 才是唯一 runtime truth。论文 Method 文本应先与实现对齐。
2. `TemplateDomain.audit()` 只把声明轴取笛卡尔积，不构建几何，也不证明 `resolve_config` 不会把不同输入折叠为同一 resolved tuple。所以 declared domain 不等于 effective/reachable domain。
3. 新 workflow 要求每个声明 slot 组合都能适配，不允许 compatibility gate；但若干 legacy spec 仍明确使用 gating、clamp 或 fallback。这些 legacy 不能与 Design-backed 的 exact Cartesian claim 混报。
4. random-36 只能覆盖被采到的 candidate；automatic corner 逐轴改一个 candidate/N 边界，不穷尽 cross-slot pairs，authored risk corners 又是作者自报。因此 `random-36 + corner` 是开发门禁，不是独立 distribution certificate。
5. strict 模式的 random-16/36 允许累计通过率低至 0.90，只是 candidate 不能全灭；因此“stage passed”不等于“每个 seed 都 valid”。主文必须报 attempt-based yield 和 worst-generator 风险。

这些计数和流程口径必须由冻结脚本重新生成，并作为 Table 1/release manifest；不能手工复制历史数字。

现有 pilot 也揭示了两个不能回避的断点：

1. 33 templates × 36 seeds 在项目自身 Full-QC 中是 1,188/1,188，通过 project-native corners 也是 231/231；但 source-derived constraints 的 all-pass asset 只有 960/1,188 = 80.81%。
2. 33 个代表资产的 PyBullet 离散状态是 3,615/3,615 collision-free；严格 exact-FCL + adaptive continuous certificate 只有 12/33 = 36.36% asset valid。原生物理字段完整的资产只有 9/33。

所以新实验不能继续把“内部 QC 通过”“离散 simulator 不报错”“补全惯量后的 evaluation copy 可运行”合并成 simulation-ready。后续主实验要直接测内部门禁对严格质量的 false-accept rate。

`Nano3dresults.md` 中以下 seed 轴保留：Naming、Hierarchy、Constraints、Articulation、Production Readiness；其 authoring reliability 表不再承担 template 质量结论。

## 2. 研究问题

| 编号 | 研究问题 | 主要证据 |
|---|---|---|
| M1 | SourceMap、TemplateDesign 和 distribution repair 是否真的帮 Agent 在未见类别构建 reusable generator？ | 多重复 authoring ablation、hidden yield、pair coverage、retention、成本 |
| G1 | 一个 PV-A generator 是否覆盖了真实、完整的类别结构词汇？ | family-role/candidate coverage、source fidelity、distribution precision/recall |
| G2 | generator 是否产生真正的组件、拓扑和几何多样性，而非尺寸/颜色扰动？ | graph entropy、Shape-Vendi、near-duplicate、novel recombination |
| G3 | generator 是否可控、可编辑，且控制不会污染非目标属性？ | target error、monotonicity、no-effect、control leakage、edit propagation |
| G4 | generator 在未参与开发的 seed 和组合上是否仍可靠？ | hidden-seed valid yield、independent corners、worst-decile/CVaR、QC false accept |
| G5 | 与 Infinite Mobility、Infinigen-Sim、Arti-PG 等人工程序化生成器相比，输出分布的质量—多样性—效率如何？ | 同类别同协议的感知、结构、运动、可靠性和吞吐比较 |
| S1 | 单 seed 是否是高质量 articulated asset？ | 独立语义 gold、关节 gold、mesh、constraints、full-range motion |
| S2 | 单 seed 是否能以原生形式进入 simulator 并完成运动任务？ | native L1–L5、稳定性、控制跟踪、碰撞表示敏感性 |
| D1 | 模板数据是否对独立模型有增益？ | template-disjoint / component-disjoint 下游结果 |

## 3. 冻结 cohort 与防泄漏协议

### 3.1 C0：全库 Generator Census

覆盖所有最终可执行 generator。每个 generator 记录：

- category、super-category、template hash；
- legacy / Design-backed；
- component slots、每 slot candidates、multiplicity；
- `core_domain`、`raw_domain`；
- 独立连续参数及单位；
- source assets、source components、provenance；
- link/joint type、运动链和复杂度；
- 原生 collision、mass、inertia、damping、friction 完备率。

`core_domain/raw_domain` 只统计结构 component 与 multiplicity；palette、连续尺寸和装饰不准用来放大结构组合数。但必须同时分开四个数：`declared_product`、`unique_resolved_tuple`、`strict-valid tuple` 和 `probe_saturated`。`TemplateDomain.audit()` 只提供第一个；legacy 甚至只能提供有限 seed 探测的轴和可达 tuple 下界。

### 3.2 C1：Reusable-generator 直接对照

#### C1a：PV-A–Infinite Mobility（12 类）

采用 12 个双方都能生成、且具有明确功能结构的语义子类：

1. office chair；
2. beverage refrigerator；
3. dishwasher；
4. microwave；
5. oven；
6. display TV；
7. faucet/tap；
8. toilet；
9. door；
10. lamp；
11. kitchen cabinet；
12. window。

先人工冻结一对一的 subtype 与 factory/template 映射，不将 OfficeChair 和 BarChair 合并后再挑更好结果。静态或语义不一致的 factory 不进入 articulation 指标。

每个方法每类生成 200 次 attempt：

- PV-A：200 个冻结 seed；
- Infinite Mobility：200 个冻结随机状态；
- Size-only ablation：固定 PV-A 所有结构 slot，只变化连续参数和 palette；
- No-recombination ablation：只采样 source pool 中已经完整出现过的 candidate tuple；
- Source-pool resampling：随机选择该类别已有 Articraft source asset，不进行新组合。

合计 12 × 5 × 200 = 12,000 attempts。每行同时报告 raw attempts、成功产物和最终 valid assets；比较时不以“补采直到 200 个成功”为分母。

Infinite Mobility 必须固定公开仓库 commit、part dataset v0.0.1 和其要求的 Infinigen commit；若无法在同机复现，只能作为 paper-reported context，不进入直接排名。论文报告的 0.46 s/object 不能与本地秒数混排。

#### C1b：PV-A–Infinigen-Sim（5 类）与 Arti-PG 共同类

- Infinigen-Sim 仅在 door、toaster、refrigerator、dishwasher、lamp 五个官方 generator 上进行同协议复现；不把 toolkit “可扩展”解释成已经支持其他类别。
- Arti-PG 只在官方 26 类中语义和功能都与 PV-A 一致的 factory 上进表；若官方代码/资产不能导出同一评测格式，只保留 scale/category/downstream context。
- 这两组都与 C1a 使用相同 attempts、FamilyGold、pair coverage、strict collision 和 valid-assets/hour，但不跨不同 category set 比较绝对数值。

### 3.3 C2：隐藏分布可靠性

现有 seeds 0–35 和 authored `TEMPLATE_CORNERS` 已参与模板开发，不能作为唯一 test set。

- 全部冻结 generator 都运行 64 个在模板冻结后才揭示的 hidden seeds；若最终 manifest 为 500 个 generator，则基础规模为 32,000 assets；
- 所有 Design-backed generator 另外生成 independent covering array：覆盖所有 candidates、所有 multiplicity 边界和所有 cross-slot pair；
- 连续参数由 evaluator 生成最小、最大、低间隙和最大展开组合，不使用模板作者自己声明的 corner 名单；
- legacy generator 运行相同 64 hidden seeds，并从 evaluator 扫描出的真实 Config 中选独立极值；缺少显式 domain 的能力记 `undeclared`，不能臆造 candidate coverage；
- 其中再分层抽 100 个 generator × 4 个 hidden seeds 运行高成本 strict CCD/独立语义约束，用于估计 internal-QC false accept。

隐藏 seed 由 `SHA256(template_hash || benchmark_salt || index)` 导出；benchmark salt 在模板 hash 冻结后公开。任何失败都进入分母，不能换 seed。

### 3.4 C3：Seed Gold Cohort

选 54 个 generator，按 6 个领域 × 3 个复杂度 × 3 个类别组织。每个 generator 取三个预注册 seed：

- typical：距参数中心最近且非重复；
- novel-combination：source pool 中未共同出现过的结构候选组合；
- adversarial：独立 evaluator 选出的最小间隙/最大展开/最大 multiplicity case。

共 162 个 PV-A assets。C1 的 12 个重叠类另对 PV-A 和 Infinite Mobility 各抽相同数量资产，进入同协议感知与结构对照。

所有语义、层级、关节和约束 gold 必须在评测输出解盲前冻结。不能从最终 URDF 反推“预期轴”“预期部件数”再给自己打分。

### 3.5 C4：Physics & Simulator Cohort

从 C3 分层选 48 个 generator × 4 seeds = 192 个原生资产，覆盖：

- revolute / prismatic / continuous / mimic；
- 单关节、多关节和多运动分支；
- 低、中、高非凸度；
- 水密与非水密 mesh；
- 低、中、高部件数；
- rest-stable 和最坏展开状态。

同一资产必须保留两份：

- `native`：生成器真实输出，用来评价 PV-A 是否 simulation-ready；
- `normalized-copy`：统一补质量、惯量或 collision proxy，只用于诊断“补全后是否可运行”。

两者不得合并成一个成功率。

### 3.6 C5：人工评测

每个对照只在它与 PV-A 的重叠类别上随机抽每类 25 对样本，每对由 3 位独立评审者盲评。Infinite Mobility 为 12×25=300 对，Infinigen-Sim 为 5×25=125 对；CAGE/ArtFormer/LAM 在四共同类上各为 100 对。评估模态为：

- textureless motion video：articulation fidelity；
- normal-map turntable：geometry quality；
- RGB turntable：appearance/material quality；
- multi-seed contact sheet：within-family diversity 与 category consistency。

Infinite Mobility panel 共 12 × 25 × 3 = 900 个 pair judgments/评价轴；其他 panel 按同一公式计数。左右顺序、seed 和方法名全部隐藏，允许 tie。无 texture/material 输出的方法在 appearance 上记 N/A，不渲染一个假材质后参赛。

## 4. Baseline 的按表分工

| 方法 / 数据源 | 进入的 paper table | 在这些表中的可比角色 | 明确不进入 |
|---|---|---|---|
| Infinite Mobility | T3、T5b、T6b、T7、T8、T9 | reusable procedural generator；PV-A 首要直接对照（论文称 22 类，实验只用公开代码已验证 factory） | Agent authoring ablation |
| Infinigen-Sim | T3、T5b、T6b、T7、T8、T9 | 5-category reusable generator、显式参数、sim export、downstream | 非官方类别 |
| Arti-PG | T3、T9 | 26-category procedural data generator 与丰富标注 | 未经官方支持的 native simulator 列 |
| Source-pool resampling | T4 | 不做新组合的 distribution lower control | reusable-generator 质量/效率 |
| NAP | T4、T6a | 四共同类的 articulated distribution 与 motion proxy | reusable generator、native simulator |
| CAGE | T4、T6a | retrieval-instantiated articulated distribution、ID/AID/AOR | PV-A domain、native physics、默认 downstream B_eq |
| ArtFormer / ArtFormer-PR | T4、T6a | direct/retrieval-matched distribution、ID/POR | reusable generator、native simulator、默认 downstream B_eq |
| LAM | T4、T5、T6a、T8 | per-asset text-to-code distribution、结构、运动和摊销对照 | reusable-generator domain |
| Articraft | T5、T6b、T7、T8、T9 | per-asset agent baseline、native package、data source | reusable generator 与显式 domain |
| Nova3D | T5a、T8 | executable Blender source、named hierarchy、constraints、local edits、per-asset cost | URDF/native-physics 主表 |
| UniPhysGen / PhysX-Omni | T7；paired supplementary | 最终 native physical package；有 GT 时的 property/kinematic 误差 | category-generator 质量、无配对分布 |
| PartNet-Mobility | T4 reference、T9 test | source-disjoint real reference/test，不是生成方法行 | generator/efficiency 排名 |
| Artiverse | T9 real-data control；paired supplement | human-verified real data 与物理标注 | generator/efficiency 排名 |
| PV-A size-only / no-recombination | T3、T9（size-only） | 结构组合机制消融 | 不相关 seed/simulator 表 |

这张分工表是“准入清单”而不是要求每行都必须出现：若公开实现无法本地复现、不能输出相应标签或不满足共同类别，预注册时将该方法从对应结果表删除，并在 supplementary 的 eligibility log 中说明原因。Paper-reported 数值只能作为带来源的 context 行，不与本地统一协议数值混排。

## 4.5 实验 M1：Unseen-category generator authoring

这是新方案必须恢复到主文的实验。它不把“Agent 写模板的成功率”冒充为 template 输出质量，而是单独检验 PV-A 的方法贡献。

- 冻结 24 个在现有 templates/designs/source_maps 中均未出现的 category，分为 8 个单关节、8 个多部件同类组合、8 个多关节/高风险接触类。
- 每个方法×类别独立运行 3 次 authoring replicate；每次使用新会话、相同 model/version、wall-clock/token/tool budget 和清空的 target workspace。
- arms 为 Naive same-LLM、w/o SourceMap、w/o TemplateDesign、w/o cross-component adaptation、w/o distribution harness/regression 和 Full PV-A。全部 arm 看到同一个冻结 raw source pool 和同一 SDK：Naive 直接从 raw pool 写 generator；w/o SourceMap 从 raw pool 直接做 Design+代码；w/o TemplateDesign 从正式 SourceMap 直接写代码。所有 arm 的最终任务都是产生同一接口的 reusable generator，不能让某个 arm 只生成一个资产。
- authoring 期可用的 tests 是开发数据；generator hash 冻结后，evaluator 才公开 hidden seeds、all-pairs covering array、multiplicity 边界、连续最小/最大/低间隙组合和独立 FamilyGold。
- 主指标是 final generator success 与 hidden strict-valid yield；first-shot、repair count、time/token/cost、pair coverage、eval-corner all-pass 和 old-case retention 是次指标。

当前 6 类×1 重复 pilot 不支持 Full 的 first-shot 优势：Naive 是 4/6，Full 只有 1/6；虽然 Full 最终达到 6/6 all-36 和 6/6 authored corners，但一次重复不能归因，authored corners 也不是独立 test。新实验不应隐藏这个 pilot，而应用多重复+冻结后 evaluator cases 验证 SourceMap/Design/harness 的因果作用。

## 5. 实验 G1：Generator Family Quality

### 5.1 FamilyGold：独立定义“这个类别应该覆盖什么”

每个 C1/C3 类别建立冻结 `family_gold.json`，信息来自 source pool、独立 reference assets 和类别资料，而不是从 PV-A 输出归纳：

- required/optional semantic roles；
- 合法结构 families；
- joint motifs 与合理运动方向；
- 关键候选组件形态；
- 部件 multiplicity 范围；
- 类别尺度先验；
- 明确非法或语义不合理的组合。

至少两位标注者独立标注，第三人仲裁。它既用于 PV-A，也用于 Infinite Mobility 和 source-resampling。

### 5.2 Family fidelity 与 coverage

主要指标：

| 指标 | 定义 |
|---|---|
| Category Precision | 生成资产中保持目标类别和关键功能的比例 |
| Family Role Recall | `FamilyGold` 中可由 generator 产生的角色 / 全部目标角色 |
| Structural Family Recall | 被 generator 覆盖的合法结构 family / gold family |
| Joint-Motif Recall | generator 能产生的合法 joint motif / gold motif |
| Invalid-Combination Rate | 违反 gold 结构或运动规则的生成配置比例 |
| Source Candidate Fidelity | 配对 source candidate 与 generator 实现组件对齐后的 geometry/normal/landmark 保真 |

Source Candidate Fidelity 只在存在真正配对 source component 时计算，并同时报告 Chamfer、F-score 和人工 source-recognizability；它不替代类别真实性。

### 5.3 生成域与结构多样性

每个 generator 报告：

- component slot 数、candidate 数、multiplicity；
- `core_domain` 和 `raw_domain`；
- declared candidate reachability；
- pairwise combination coverage；
- realized unique kinematic graphs；
- canonical graph entropy / perplexity；
- normalized Tree Edit Distance，而非只报会被 joint 数放大的 raw TED；
- joint 数均值、方差、类型分布和运动链深度；
- 只变化尺寸、变化组件、变化拓扑的 seed 比例。

定义规范化树距离：

\[
d_{NTED}(T_i,T_j)=\frac{TED(T_i,T_j)}{|T_i|+|T_j|}
\]

Infinite Mobility 的平均 joint 数、joint 数方差和 raw Tree Edit Distance 作为复现项保留，但主结论使用 joint-count-matched 的 NTED/graph entropy，避免“分更多 joint 就显得更多样”。

### 5.4 几何分布多样性与真实性

对每类 200 assets 做统一尺度和 canonical pose 后计算：

- Shape-Vendi@200；
- exact duplicate rate；
- geometric near-duplicate rate；
- intra-class point-cloud/voxel distance；
- reference-based MMD、COV、1-NNA；
- distribution Precision/Recall。

MMD/COV/1-NNA 只在有独立真实 reference distribution 的重叠类别上使用；reference 不能同时作为 PV-A source candidate，否则会产生数据泄漏。应另设：

- `Ref-clean`：与所有方法训练/源池去重的参考集；
- `Source-seen`：仅用于 source fidelity，不用于生成真实性排名。

### 5.5 Compositional novelty

PV-A 的关键价值不是复现 source pool，而是产生以前没有共同出现过的合法组合。

对每个 seed 计算其结构 candidate tuple 是否出现在任何原始 source asset 中：

\[
NovelComboRate=\frac{N_{valid\ output\ with\ unseen\ tuple}}{N_{all\ outputs}}
\]

但 novelty 只有和 plausibility 一起报告才有意义：

\[
ValidNovelRate=\frac{N_{unseen\ tuple\ and\ gold/human\ plausible}}{N_{all\ outputs}}
\]

主表同时放 NovelComboRate、ValidNovelRate 和 invalid novel combination rate，不能只报组合数。

### 5.6 Generator reliability

在 C2 hidden set 上报告：

- compile/package yield；
- Full-QC yield；
- hidden constraint all-pass；
- hidden full-range motion pass；
- all-hidden-seeds-pass generator rate；
- macro 平均、模板中位数、最差 10% 均值（CVaR-10）；
- failure-discovery curve：随已测 seed 数增长发现失败 generator 的比例；
- 95% binomial upper bound，而非“64/64 所以绝对无失败”。

最重要的新指标是 QC False Accept：

\[
FAR_{QC}=\frac{N_{internal\ QC\ pass\ but\ strict\ evaluator\ fail}}{N_{internal\ QC\ pass}}
\]

它直接解释现有离散 33/33 与严格 12/33 的落差。

## 6. 实验 G2：Controllability 与 Distributional Editability

### 6.1 单变量干预，不从 Config 自报结果

对每个 Design-backed generator 选取：

- 2 个连续几何参数；
- 1 个 component slot；
- 1 个 multiplicity；
- 1 个 motion-range 参数（若存在）。

固定其他随机因素，对目标变量执行成对或五点干预。所有输出属性从最终 mesh/URDF 重新测量。

| 指标 | 含义 |
|---|---|
| Target NMAE | 目标实测值与要求值的归一化误差 |
| Target Hit Rate | 在预注册 tolerance 内命中比例 |
| Monotonic Response | 输入增加时输出是否单调 |
| Parameter No-effect | 参数改变但资产无可测变化的比例 |
| Control Leakage | 非目标几何/拓扑/运动属性发生变化的比例 |
| Structural Edit Success | candidate/N 改动是否产生预期部件或拓扑 |
| 16-seed Propagation | 同一 edit 在 16 个未见 seed 上全部成立的任务比例 |
| Post-edit Valid Yield | edit 后通过独立完整质量门禁的比例 |

不要把 bitwise diff 当作 edit success；“代码/mesh 变了”不等于目标语义成立。

### 6.2 因果消融

Generator 质量消融只改变生成域能力，不改变 Agent 写作流程：

1. Full PV-A；
2. Size-only：固定全部结构 slot；
3. No-recombination：只输出 source 中原有的完整 candidate tuple；
4. Full-domain：允许跨 source 的新 tuple，并保留跨组件 adaptation/derivation。

该消融直接回答组件组合与参数化是否提供价值，比“有无 SourceMap 文件”更贴近 generator 贡献。

## 7. 实验 G3：与 procedural-generator baselines 的同协议比较

Infinite Mobility 原论文的可复现实验包括：

- textureless motion pairwise articulation fidelity；
- normal map 的 GPT-4V geometry comparison；
- RGB render 的 texture comparison；
- 平均 joint 数、joint 数方差和平均 pairwise Tree Edit Distance；
- generation time；
- CAGE training data experiment。

这些指标只在 PV-A–Infinite Mobility 的 12 类 panel 中按原论文复现。另设 PV-A–Infinigen-Sim 的 5 类 panel，因为 Infinigen-Sim 原文特别强调参数分布、组件笛卡尔组合、URDF/USD/MJCF 导出与下游分割/控制；Arti-PG 再设官方共同类 panel，主要比结构程序变形和标注下游价值。不在不同 category set 之间直接比绝对数。

本实验对这些原协议做四项加强：

1. 每个 panel 使用相同重叠类、相同 attempt 数和统一 canonical render；
2. 加入 valid yield，避免只评价成功导出的样本；
3. 加入 NTED、near-duplicate、FamilyGold coverage 和 control；
4. 加入严格 full-range collision 与 native simulator，而不是只看运动视频。

主表建议：

| Method | Valid Yield | Category Precision | Family Recall | Articulation Win/Tie/Loss | Geometry Win/Tie/Loss | Shape-Vendi | Near-Dup | Graph PPL | NTED | Valid Novel | Control Hit | Valid Assets/h |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|

人工 pairwise 使用含 tie 的 Davidson/Bradley–Terry 模型，并按 category cluster bootstrap 给 95% CI。若论文要声称“comparable”，应在运行前预注册非劣界，例如 win probability 相对 0.5 不低于 0.40；不能在看到结果后选 margin。

VLM 只能作为大规模 proxy：固定模型版本、prompt、图像顺序，并用人工 900 个 judgments 校准一致性。若一致性不足，主文使用人工结果。

## 8. 实验 S1：Seed Asset Quality

这部分沿用 `Nano3dresults.md` 中合理的评测骨架，但统一使用 C3 的独立 gold 和同一最终 artifact。

### 8.1 Semantic parts 与 hierarchy

| 维度 | 主指标 |
|---|---|
| Naming | semantic precision/recall/F1、functional core coverage、instance discriminability、over-segmentation |
| Hierarchy | valid tree、parent-child edge P/R/F1、hierarchy exact match、semantic depth |
| Constraints | coverage、satisfaction、all-constraints-pass asset rate、count/numeric/relational/interface 分项 |

空 hierarchy node 与 mesh-bearing semantic part 分开计数；不能用“都有名字”替代 semantic precision。

### 8.2 Kinematic correctness

使用冻结 joint gold 或 TemplateDesign 外部审核后的 spec：

- joint recall、type accuracy；
- parent-child accuracy；
- axis angular error：

\[
e_a=\arccos(|a\cdot \hat a|)
\]

- pivot-to-gold-axis distance / bbox diagonal；
- motion-range mIoU；
- articulation structure F1；
- axis-on-moving-part；
- endpoint reachability。

“axis 字段存在”“origin 是 finite”只保留为 metadata completeness，不进入 correctness。

### 8.3 Geometry 与 motion

- watertight、winding-consistent、manifold、open edges、degenerate/self-intersection；
- part disconnected/isolated；
- rest-state AOR；
- 11-state single-joint sweep；
- 64-state Sobol multi-joint proxy；
- adaptive exact FCL / CCD full-range certificate；
- maximum penetration、minimum clearance curve；
- ArtFormer POR；
- 有配对 reference 时的 rest/articulated dgIoU、dcDist 或 Chamfer/F-score。

离散 sweep 与严格 certificate 必须分两行。超时和未认证区间在 strict 指标中按失败处理，同时单列 `unknown/timeout` 便于诊断。

### 8.4 Perceptual seed quality

在固定相机、尺度、材质设置下评价：

- category identity；
- geometry realism；
- articulation plausibility；
- texture/material quality；
- source recognizability（仅配对 subset）。

CLIP/BLIP/VLM 是软分数；它们不能覆盖 joint error、collision 或 simulator validity。

### 8.5 Production completeness

原生输出报告：

- portable package；
- visual/collision geometry 完备率；
- mass/inertia 完备率；
- inertia 正定和三角不等式合法率；
- damping/friction/effort 完备率；
- deterministic rebuild；
- URDF/mesh/GLB 大小和构建时间。

主结果同时报告 raw attempts、internal-QC pass subset 和 release subset，防止质量筛选造成循环结论。

## 9. 实验 S2：Simulator 与物理可用性

### 9.1 Collision representation 三路对照

对同一资产比较：

1. exact visual mesh；
2. per-part convex hull；
3. convex decomposition / simplified collision mesh。

记录：

- collision/visual 体积比；
- surface deviation / Chamfer；
- collision triangle 数；
- rest/full-range AOR；
- strict valid yield；
- simulator step time 和 peak memory。

这一步直接验证项目当前“精确 mesh QC”能否迁移到真实 simulator collision backend。

### 9.2 Simulator L1–L5

核心环境：PyBullet、MuJoCo、SAPIEN；Genesis 在 collision kernel 可用后加入核心表，否则单列。Isaac Sim 只有在安装/EULA条件满足时运行，缺失记 `N/E`。

| Level | 成功定义 |
|---|---|
| L1 Parse | URDF/转换格式可解析 |
| L2 Instantiate | visual、collision、joint 和 inertial 成功建模 |
| L3 First Step | 固定 timestep 完成第一步，无异常/NaN |
| L4 Passive Stable | rest 与最坏展开状态 rollout 后无爆炸、穿地、非法漂移 |
| L5 Driven Articulation | 每个关节跟踪完整目标轨迹，达到端点且不碰撞、不越限 |

主指标：

- native per-simulator L5；
- native 3-simulator all-pass；
- normalized-copy L5（诊断项）；
- NaN/explosion、最大穿透、root drift、limit violation；
- trajectory tracking RMSE；
- effort saturation / drive feasibility；
- rest stable、worst-state stable。

Genesis collision disabled 的运行不能计入 L5 all-pass；用 AABB 铝块惯量补全后的副本也不能计入 native success。

### 9.3 Functional interaction subset

在门、抽屉、柜、洗碗机、微波炉、龙头等类别上增加低策略依赖的 scripted task：

- open/close 到目标角或位移；
- 保持 1 秒；
- 返回 rest；
- 检查目标误差、碰撞、失稳和能量异常。

该结果比“能 load”更接近 embodied use，但仍不把控制器失败简单归因于几何；需要区分 asset failure 与 controller failure。

## 10. 实验 D1：Dataset Utility

### 10.1 主下游：Particulate / articulation prediction

训练数据组保持数量相同：

| ID | Training data |
|---|---|
| D0 | 原始训练集 |
| D1 | D0 + Artiverse-B_eq（real-data control） |
| D2 | D0 + Articraft-B_eq |
| D3 | D0 + Arti-PG-B_eq（官方共同类与标签） |
| D4 | D0 + Infinigen-Sim-B_eq（五类 panel） |
| D5 | D0 + InfiniteMobility-B_eq（公开 factory 重叠范围） |
| D6 | D0 + PV-A-SizeOnly-B_eq |
| D7 | D0 + PV-A-B_eq |
| D8 | D0 + PV-A-10K（scaling） |
| D9 | D0 + PV-A-30K（scaling） |

固定 backbone、steps、batch、optimizer，并运行 3 个 train seeds。测试集按 template 和 source component 隔离：

- unseen seed：只测插值，不作为主 OOD；
- unseen combination；
- source-component-disjoint；
- generator/factory-family-disjoint；
- category OOD。

指标沿用现有 articulated reconstruction 协议：rest/articulated gIoU、dcDist/PC、mIoU、OC，以及 OOD articulated gIoU。

PV-A-B_eq 对 PV-A-SizeOnly-B_eq 是最关键的数据消融：它判断增益来自结构/组合多样性，还是仅来自更多尺寸扰动。默认对照选择了能直接交付部件、tree 和 joint labels 的真实/程序化数据源。主对照预算为 `B_eq = min(5,000, 共同类别配额下各数据源的最小 strict-valid 供给)`，PV-A-10K/30K 只用于学习曲线。CAGE/ArtFormer 只在能导出相同 label schema 且交付 B_eq strict-valid assets 时增加；否则按预注册 eligibility rule 删除，不以伪标签补齐。

### 10.2 可选下游：CAGE data-source test

为了直接衔接 Infinite Mobility 的实验，可在 6 个共同类别上分别使用 PartNet-Mobility、Infinite Mobility 和 PV-A 的等量数据训练同一 CAGE 配置，比较：

- generator sample valid yield；
- ID/AID；
- MMD/COV/1-NNA；
- AOR；
- 人工 plausibility。

该实验是可选项，不能替代 Particulate 的 template-disjoint 结果。

## 11. `Paper-metrics.docx` 到新方案的映射

| 论文来源 | 原论文指标 | 对 PV-A 具体检验什么 | 放置 | 限制 |
|---|---|---|---|---|
| LAM | masked-URDF success、strict asset acc.、CLIP/BLIP、MMD/COV/1-NNA | per-asset code 与 PV-A seed 的结构完整性、生成分布和摊销成本 | T4、T5、T8 | masked reconstruction 数字不直接当无配对 PV-A 分数 |
| Artiverse | rest/articulated dgIoU、dcDist、AOR、物理标注 | 为 paired geometry/physics 子集提供 human-verified reference | T9、supp. | dgIoU/dcDist 只在有一对一 GT 时 |
| Articraft | execution/URDF/overlap、人评几何/运动/物理、retention、downstream | per-asset agent 的 seed 质量、原生产物、成本和数据价值 | T5–T9 相容 panel | 不是 reusable-generator domain baseline |
| CAGE | ID/AID、MMD/COV/1-NNA、AOR、plausibility | 区分 PV-A 几何+运动分布与抽象 part-layout 分布，并廉价诊断重叠 | T4、T6a | AOR 不是 exact collision；四共同类才直比 |
| UniPhysGen | type/axis/pivot/range、structure F1、material/density/friction/scale/mass | 将 PV-A 的“字段存在”升级为“运动学/物理数值正确” | T7、paired supp. | 需要独立 joint/physics GT；它是 grounding 不是 category generator |
| PhysX-Omni | PSNR/CD/F-score、scale MSE、kinematic error、PhysX-Bench | 检验 PV-A 最终 native asset 的多属性可部署性 | T7、paired supp. | image-conditioned；paired 指标只在相应 subset |
| Nova3D | executable/artifact、named parts/tree、constraints、edits、joints | 检验 PV-A seed 是否真的结构化、可编辑程序资产 | T5a、T8 | 原生产物是 Blender+GLB，不进 URDF/physics 主表 |
| ArtFormer | POR、ID/MMD/COV/1-NNA、CLIP-R、人评 | 对齐 learned articulated generation 分布，POR 诊断多状态平均穿透 | T4、T6a | POR 会掩盖最坏状态，必须与 strict CCD 分开 |
| Infinite Mobility | motion/normal/RGB pairwise、joint #/variance/TED、time | 同类 reusable factory 的外观、运动、结构多样性和边际速度 | T3、T6–T9 | 补 valid yield、NTED、effective domain、strict physics；paper time 不含人工 authoring |
| Infinigen-Sim | Cartesian combinations、continuous DOF、export、segmentation/RL | 同类 generator 的可控空间、sim export 和下游效用 | T3、T7–T9 | 只有 5 个官方类；不外推 toolkit 能力 |
| Arti-PG | structure-program variation、rich annotations、downstream | 同类 procedural data 的结构覆盖和标注效用 | T3、T9 | 只在官方共同类/标签上直比 |

## 12. 统计与汇总规则

1. Generator 是主要独立单位。200 个 seed 不是 200 个独立类别；置信区间按 category/generator cluster bootstrap。
2. 同时报 macro（每 generator 等权）和 micro（每 seed 等权），主结论使用 macro。
3. 报告 median、IQR、worst decile/CVaR，不只报均值。
4. 人工 pairwise 使用含 tie 模型；报告评审者一致性和 category-level CI。
5. Binary seed metric 使用 Wilson/Clopper–Pearson CI；0 failure 仍报告上界。
6. 多指标检验在同一 family 内做 Holm 或 BH 修正；预注册 1–2 个 primary metric/实验。
7. 任何筛选、超时、N/E、unknown 都保留在 manifest；strict metric 中 timeout/uncertified 记失败。
8. Paper-reported 与 locally reproduced 分行，不能放在一个无来源标记的排行榜。
9. 不构造一个加权“总质量分”；Generator fidelity、diversity、control、reliability 和 seed physics 必须分别呈现。

## 13. 论文主表与图

完整 paper table pool 使用开头的 9 张结果表（T3 和 T5 有多个 panel）。若主文篇幅有限，保留 T1、T2、T3a、T4、T5b、T6、T7、T8、T9；T3b 只在主文保留 Infinite Mobility 与一个 learned baseline，完整人评和 T5a 编辑表移到 supplementary：

| 表 | 内容 |
|---|---|
| T1 | 实现过的 generator fleet 和 Design-backed/legacy 审计 |
| T2 | 未见类别的 Agent generator authoring 消融 |
| T3a | Procedural-generator 质量、多样性、可靠性与吞吐 |
| T3b | 兼容渲染模态下的多 baseline 盲评 |
| T4 | 四共同类的 ID/AID 分布质量 |
| T5 | Code-native seed 结构与 functional kinematics |
| T6 | Full-range motion、AOR/POR 与 strict continuous collision |
| T7 | Native physical completeness 与 multi-simulator L5 |
| T8 | Per-asset 与 reusable-generator 的摊销成本 |
| T9 | Equal-valid-budget downstream utility |

建议主图：

- quality–diversity Pareto：Category Precision/Valid Yield 对 Shape-Vendi/Graph PPL；
- generator failure-discovery curve 与 QC false-accept；
- target control curve 与 non-target leakage；
- discrete collision 通过率到 strict CCD、native simulator L5 的漏斗；
- amortized valid assets/hour 随生成规模 `N` 的曲线。

## 14. 项目落点

### 14.1 可直接复用

- `run_nano3d_seed_reliability.py`：seed compile/package/Full-QC；
- `run_nano3d_naming.py`、`run_nano3d_hierarchy.py`、`run_nano3d_constraints.py`：S1 骨架；
- `run_nano3d_articulation_paper.py`：离散 single/Sobol proxy；
- `run_t5_ccd_adaptive.py`：strict full-range；
- `run_t5_simulator_l5.py`、`run_t5_genesis_l5.py`：simulator 层级；
- TemplateDesign、`TEMPLATE_DOMAIN.audit()`、resolved config 和 provenance：G1/G2 的 domain 与控制输入；
- `preflight → random-16 → random-36 → corner`：开发门禁，作为被评估系统，不作为隐藏测试本身。

### 14.2 必须新增

建议新增独立只读 harness：

```text
exp/scripts/freeze_pva_generator_manifest.py
exp/scripts/audit_declared_resolved_valid_domain.py
exp/scripts/run_unseen_generator_authoring_ablation.py
exp/scripts/run_hidden_generator_distribution.py
exp/scripts/run_infinite_mobility_baseline.py
exp/scripts/run_infinigen_sim_baseline.py
exp/scripts/run_artipg_baseline.py
exp/scripts/score_family_gold.py
exp/scripts/score_generator_diversity.py
exp/scripts/score_generator_control.py
exp/scripts/score_qc_false_accept.py
exp/scripts/run_native_simulator_matrix.py
exp/scripts/aggregate_pva_generator_paper.py
```

这些 scorer 不能 import 模板作者测试来定义 gold；它们只能读取冻结 manifest、最终 artifact 和独立 annotation。

## 15. 推荐执行顺序

### P0：先解决论文最危险的 claim

1. 冻结全库 generator/category manifest；
2. 分开 106 个 Design-backed 与 425 个 legacy，重新审计 declared/resolved/strict-valid domain；
3. 建立 24 类×3 重复的 M1 authoring benchmark；
4. 建立 Infinite Mobility P12、Infinigen-Sim P5 和 Arti-PG shared 映射并跑 smoke；
5. 建立 100-generator hidden-seed + independent pair/corner benchmark；
6. 量化 internal QC → strict CCD 的 false-accept；
7. 分开 native 与 normalized-copy simulator 结果。

### P1：证明 generator 本身的价值

1. Full / Size-only / No-recombination / Source-resampling；
2. FamilyGold coverage；
3. graph/geometry diversity 与 ValidNovelRate；
4. controllability 与 leakage；
5. C1 人工 pairwise。

### P2：完成 seed 和数据价值闭环

1. C3 的 independent semantic/joint/constraint gold；
2. S1 全套 seed metrics；
3. C4 native multi-simulator；
4. Particulate 等量数据实验；
5. 可选 CAGE data-source 实验。

## 16. 结论书写边界

- 如果 hidden-seed 有效率高但 strict CCD 仍低，只能写“generator 在内部机械合同上可靠”，不能写“simulation-ready”。
- 如果 PV-A diversity 高但 Category Precision/ValidNovelRate 低，不能把 raw domain 或 TED 当成质量优势。
- 如果 Full 与 Size-only 的下游结果无显著差异，不能声称组件组合多样性带来学习收益。
- 如果 Infinite Mobility 只能使用论文报告值，应写 contextual comparison，不能写同协议超越。
- 如果 simulator 依赖补全副本，必须写“normalized evaluation copy”，不能称原生输出成功。
- 如果 Full 在人工 geometry 上不占优但在 coverage/control/scale 上占优，论文应据实定位为 scalable controllable generator，而不是全面视觉 SOTA。

## 参考

- [Infinite Mobility paper](https://arxiv.org/abs/2503.13424)
- [Infinite Mobility code](https://github.com/Intern-Nexus/Infinite-Mobility)
- [Infinigen-Sim](https://arxiv.org/abs/2505.10755)
- [Arti-PG](https://arxiv.org/abs/2412.14974)
- [CAGE](https://openaccess.thecvf.com/content/CVPR2024/html/Liu_CAGE_Controllable_Articulation_GEneration_CVPR_2024_paper.html)
- [ArtFormer](https://openaccess.thecvf.com/content/CVPR2025/html/Su_ArtFormer_Controllable_Generation_of_Diverse_3D_Articulated_Objects_CVPR_2025_paper.html)
- [LAM](https://openaccess.thecvf.com/content/CVPR2026/html/Gao_LAM_Language_Articulated_Object_Modelers_CVPR_2026_paper.html)
- [Articraft](https://arxiv.org/abs/2605.15187)
- [Nova3D](https://arxiv.org/abs/2607.22738)
- [UniPhysGen](https://arxiv.org/abs/2607.13586)
- [PhysX-Omni](https://arxiv.org/abs/2605.21572)
- [Artiverse](https://openaccess.thecvf.com/content/CVPR2026/html/Iliash_Artiverse_A_Diverse_and_Physically_Grounded_Dataset_for_Articulated_Objects_CVPR_2026_paper.html)
- [Particulate](https://openaccess.thecvf.com/content/CVPR2026/html/Li_Particulate_Feed-Forward_3D_Object_Articulation_CVPR_2026_paper.html)
- 项目内部指标来源：`exp/Paper-metrics.docx`
- 可保留的 seed 评测基础：`exp/Nano3dresults.md`
- 当前项目论文：`PV_A.pdf`
