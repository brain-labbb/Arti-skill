# Arti-Template：最终实验设计与 SOTA Baseline Pool

## 0. Baseline 使用规则

定义三个级别：

- **D — Direct**：任务和输出足够一致，可以进入主表直接比较。
- **P — Partial**：只能比较部分指标，其余填 `N/A`。
- **C — Context**：相关 SOTA，只作为 supplementary/context，不做数值排名。

重点：**每张表调研 20 篇，不等于每张主表硬塞 20 行。**

例如 Nova3D、P3D-Bench、ArtiCAD 都支持 code/assembly 层面的结构评价；UniPhysGen、PhysX-Omni、Real-IKEA 更适合 Physics；Particulate 则应该放 downstream，而不是 Template Reliability。最新工作已经明显从“mesh 看起来像不像”转向 executable structure、constraints、physical grounding 和 simulation readiness。

---

# Table 1 — Dataset & Generator Statistics

## 研究问题

> 我们的数据资源在类别规模、模板规模、结构标注和物理属性上处于什么位置？

## 最终锁定指标

| 指标 | 最终采用 | 说明 |
|---|---:|---|
| `#Categories` | ✓ | 独立语义类别 |
| `#Super-categories` | ✓ | 上层领域 |
| `#Templates` | ✓ | **重点**，区别于 assets |
| `#Assets` | ✓ | 实际发布资产规模 |
| `#Movable Parts / Asset` | ✓ | median + mean |
| `#Joints / Asset` | ✓ | median + mean |
| Joint Types | ✓ | revolute/prismatic/continuous/mimic/multi-DoF |
| Continuous Param Dims | ✓ | template-median |
| Discrete Slots | ✓ | template-median |
| Valid Configuration Space | ✓ | median `log10(#config)` |
| Part Semantics | ✓ | ✓/✗ |
| Kinematic GT | ✓ | ✓/✗ |
| Metric Scale | ✓ | ✓/✗ |
| Material | ✓ | ✓/✗ |
| Mass/Inertia | ✓ | ✓/✗ |

**不放：**视觉 CLIP、PSNR、CD；这张表只描述数据资源。

## 调研 20 篇候选

**D：**

1. Articraft / Articraft-10K  
2. Infinigen-Sim / Infinigen-Articulated  
3. Infinite Mobility  
4. Artiverse  
5. UniPhysGen / UniPhys-40K  
6. PhysX-Omni / PhysXVerse  
7. ArtVIP  
8. Real-IKEA  

**P：**

9. PhysX-Anything  
10. PhysX-3D  
11. PhysForge / PhysDB  
12. EmbodiedGen V2  
13. URDF-Anything+  
14. ArtLLM  
15. SIMART  
16. Articulate AnyMesh  
17. Articulate-Anything  
18. DreamArt  
19. PAct  
20. ArtiCAD  

这些工作覆盖程序化 articulated generation、人工/半自动 articulated datasets、simulation-ready physical assets 和 articulated CAD generation。

### 主文真正建议放

**Ours / Articraft / Infinigen-Sim / Infinite Mobility / Artiverse / UniPhys-40K / PhysXVerse / ArtVIP / Real-IKEA**

---

# Table 2 — Template Authoring & Distribution Reliability（v1，已被 v2 替代）

> T2 已按项目当前 `.agents/skills/build-template` 工作流重新设计。新版的正式协议、
> 2×2 SourceMap/TemplateDesign 因子、隔离规则、指标和统计判定见
> [T2_redesign_v2.md](</mnt/zsn/lyb/arti-skill/exp/T2_redesign_v2.md>)。
> 本节保留 v1 方案作为历史记录，不应再用于启动新的正式 T2 运行。

这是**方法贡献的第一主表**。

## Panel A：Template Authoring

### 最终指标

| 指标 | 定义 |
|---|---|
| Executable ↑ | 新生成 template 可以执行 |
| Artifact Saved ↑ | 能生成完整 URDF + mesh package |
| First-Shot ↑ | 无 repair 直接通过 |
| Final Success ↑ | 固定 repair budget 内通过 |
| Repair Turns ↓ | 平均自动修复次数 |
| Human Intervention ↓ | 需要人工修复比例 |
| Authoring Time ↓ | 单模板 wall time |
| Authoring Cost ↓ | API/token cost |

### 对照必须包含

1. Original Articraft  
2. Naive same-LLM  
3. w/o Source Map  
4. w/o Slot Abstraction  
5. w/o Cross-Component Constraints  
6. w/o Distribution Harness  
7. w/o Regression  
8. **Full Ours**

---

## Panel B：Distribution Reliability

### 最终指标

| 指标 | 定义 |
|---|---|
| Seed Compile ↑ | seed 成功产生完整 package |
| Seed Full-QC ↑ | 全 QC 通过 |
| All-36 Template ↑ | 一个模板 36/36 全通过 |
| Corner Pass ↑ | corner cases 通过 |
| Regression Retention ↑ | 修复后历史 seed 保持有效 |

### 这里当前已经有真实 pilot

\[
1188/1188
\]

Seed Compile 和 Full QC；

\[
33/33
\]

模板达到 36/36；

以及：

\[
231/231
\]

project-native corners 通过。

---

## 调研 20 篇候选

**D/P：**

1. Articraft  
2. Nova3D  
3. ArtiCAD  
4. P3D-Bench  
5. SceneCode  
6. MUSE / AuthorBench  
7. CadBench  
8. CAD-Coder (text → CadQuery)  
9. CAD-Coder (VLM → CadQuery)  
10. BlenderRAG  
11. Real2Code  
12. Articulate-Anything  
13. Articulate AnyMesh  
14. URDF-Anything+  
15. ArtLLM  
16. SIMART  
17. MotionAnyMesh  
18. REACT3D  
19. EmbodiedGen V2  
20. Infinigen-Sim  

Nova3D、P3D-Bench、ArtiCAD 和 MUSE 尤其适合这里，因为近期 code/CAD/scene authoring benchmark 已经开始系统报告 executable output、assembly consistency、requirement satisfaction 和 iterative repair，而不仅是最终 mesh fidelity。

### 主文 Baseline

**Naive same-LLM + 5 个方法消融 + Full Ours**

外部系统 Nova3D、Articraft、ArtiCAD 放 contextual row。

---

# Table 3 — Native Semantic Structure & Constraint Satisfaction

建议分两个 Panel。

---

## Panel A：Semantic Structure

### 最终指标锁定

| 指标 | 保留 |
|---|---:|
| Semantic Precision ↑ | ✓ |
| Semantic Recall ↑ | ✓ |
| Instance Discriminability ↑ | ✓ |
| Parent-Child Edge F1 ↑ | ✓ |
| Hierarchy Exact Match ↑ | ✓ |
| Semantic Nesting Accuracy ↑ | Supplementary |
| Cross-Seed Structural Consistency ↑ | Supplementary |

**删除主指标：**

- Raw Parts 数；
- Naming Richness。

它们过于依赖拆分粒度。

当前 GLB 直接得到 Parts/Nameability 是可靠结果，但正式 Semantic Precision 仍应完成独立 judge；现有 source-derived Recall/Richness 应留作 proxy/supplementary。

---

## Panel B：Constraints

### 全局指标

设：

- \(C\)：全部约束；
- \(M\)：可测；
- \(P\)：通过。

固定：

\[
Coverage=M/C
\]

\[
Conditional=P/M
\]

\[
Satisfaction=P/C
\]

### 约束类别指标

| 指标 | 最终保留 |
|---|---:|
| Count Pass ↑ | ✓ |
| Numeric Pass ↑ | ✓ |
| Relational Pass ↑ | ✓ |
| Interface Pass ↑ | ✓ |
| Kinematic Pass ↑ | ✓ |
| Compatibility Pass ↑ | ✓ |
| Invalid Combination Rejection ↑ | ✓ |

这些 Pass 均定义为：

> **该类别中 measurable constraints 的 conditional pass rate。**

避免不同模板约束数量不同导致统计混乱。

### 两个最重要的严格指标

#### All-Pass Assets

\[
\frac{
N(\text{所有适用约束全部通过})
}{
N(\text{assets})
}
\]

#### All-Seeds-Pass Templates

\[
\frac{
N(\text{该模板全部评测seed都是all-pass})
}{
N(\text{templates})
}
\]

这两个必须加。

---

## 调研 20 篇

1. Nova3D — D  
2. P3D-Bench — D  
3. ArtiCAD — D  
4. MUSE / AuthorBench — D/P  
5. MUSE-CAD benchmark — P  
6. CubePart — P  
7. PartCrafter — P  
8. Articraft — D/P  
9. Real2Code — P  
10. Articulate-Anything — P  
11. Articulate AnyMesh — P  
12. URDF-Anything+ — P  
13. SIMART — P  
14. ArtLLM — P  
15. PAct — P  
16. UniArt — P  
17. CAGE — P  
18. SINGAPO — P  
19. ArtFormer — P  
20. ArtiLatent — P  

Nova3D直接验证 named parts、hierarchy 和 measurable constraints；P3D-Bench 进一步评价 executable parametric programs、part structure 和 text-grounded constraints；ArtiCAD 则显式使用 Connector 规划 articulated assembly。

CAGE、SINGAPO、ArtFormer、ArtiLatent 等属于 structured/articulated generation reference，适合部分 kinematic/control 指标，但不能强行填写 code-native hierarchy 列。

### 主文建议

**Naive / Full / Nova3D / ArtiCAD / P3D-Bench / Articraft / CubePart / PartCrafter**

---

# Table 4 — Distributional Editability

这里指标可以正式冻结，不再改。

## 最终指标

| 指标 | 精确定义 |
|---|---|
| Target Fulfilled ↑ | 最终 artifact 中目标编辑实现 |
| Anchor ↑ | 位置、宿主、connector 正确 |
| Scale ↑ | 最终 mesh 的尺寸变化满足目标 |
| Non-Target Preserved ↑ | true non-target 未变化 |
| Geometry Locality ↑ | 几何变化集中于 target + allowed dependents |
| Structural Locality ↑ | 非目标 link/joint/tree 不变 |
| Post-Edit Constraint Pass ↑ | 编辑后 frozen constraints 仍通过 |
| 16-Seed Propagation ↑ | 每个 edit 16/16 seed 成功 |
| Regression Preservation ↑ | 历史 regression manifest 保持 |
| Final Pass ↑ | 上述 gate 全部通过 |
| Edit Cost ↓ | wall time + API cost |

### 特别规定

必须把：

- Target
- Allowed Dependents
- True Non-Targets

在 edit 前冻结。

否则“门变宽导致把手位置同步变化”会被错误算成污染。

---

## 20 篇相关 SOTA

1. Nova3D — D  
2. MUSE / AuthorBench — D  
3. SceneCode — D/P  
4. ArtiCAD — D/P  
5. P3D-Bench — P  
6. MUSE-CAD — P  
7. CubePart — P  
8. CAGE — P  
9. SINGAPO — P  
10. ArtFormer — P  
11. ArtiLatent — P  
12. PAct — P  
13. Sketch2Arti — P  
14. FreeArt3D — P  
15. ViPS — P  
16. Articraft — D/P  
17. Articulate-Anything — P  
18. Real2Code — P  
19. PartCrafter — P  
20. UniArt — P  

Nova3D 的 local edits、MUSE 的 preservation-aware editing、SceneCode 的 executable world programs 与 ArtiCAD 的 parametric assembly 是最值得主表比较的几组。MUSE 的 AuthorBench 已显式报告 All-Goal、preservation rate 和 unintended change rate，这和你们的 Non-Target Preservation / Final Pass 非常接近。

### 主文 Baseline

**Naive / w/o Regression / Full / Nova3D / MUSE / ArtiCAD / SceneCode**

其他 13 篇放 supplementary。

---

# Table 5 — Articulation, Collision & Simulation Readiness

这一张应该成为**铰链资产质量最重要的表**。

---

## Panel A：Kinematic Correctness

### 最终指标

| 指标 | 单位 |
|---|---|
| Joint Type Accuracy ↑ | % |
| Joint Recall ↑ | % |
| Parent-Child Accuracy ↑ | % |
| Axis Error ↓ | degree |
| Origin Error ↓ | bbox-diagonal normalized |
| Limit Error ↓ | range-normalized |

不要再使用：

> `Axis Valid = field exists`

这种 metadata existence proxy。

正式实验必须和 frozen spec 比。

---

## Panel B：Motion Validity

### 固定协议

每个 joint：

\[
11
\]

个均匀状态。

多关节资产：

\[
64
\]

个 Sobol configurations。

低 clearance 区域做 adaptive sampling / CCD。

### 最终指标

| 指标 | 保留 |
|---|---:|
| Joint-Level Geom. Valid ↑ | ✓ |
| Asset-Level Geom. Valid ↑ | ✓ |
| Full-Range Collision-Free ↑ | ✓ |
| AOR ↓ | ✓ |
| Max Penetration ↓ | ✓ |
| Minimum Clearance ↑ | Supplementary |
| Endpoint Reachability ↑ | Supplementary |

当前 286 个 boundary states 都成功只能继续叫 boundary smoke，不能升级成 Full-Range。

---

## Panel C：Simulation + Physics

测试：

- MuJoCo
- Genesis
- PyBullet
- Isaac Sim

每个 simulator 使用：

L1 Parse → L2 Instantiate → L3 First Step → L4 Passive Stable → **L5 Full Articulation**。

### 最终指标

| 指标 | 保留 |
|---|---:|
| MuJoCo L5 ↑ | ✓ |
| Genesis L5 ↑ | ✓ |
| PyBullet L5 ↑ | ✓ |
| Isaac L5 ↑ | ✓ |
| 4-Simulator All-Pass ↑ | ✓ |
| Rest Stable ↑ | ✓ |
| Worst-State Stable ↑ | ✓ |
| Physical Metadata Complete ↑ | ✓ |

Physics 实验前必须把 mass / COM / inertia / joint dynamics 补完整，否则只是测试 simulator defaults。

---

## 调研 20 篇

1. UniPhysGen  
2. PhysX-Omni  
3. PhysX-Anything  
4. PhysX-3D  
5. Real-IKEA  
6. ArtVIP  
7. EmbodiedGen V2  
8. Artiverse  
9. Infinigen-Sim  
10. Infinite Mobility  
11. URDF-Anything+  
12. SIMART  
13. ArtLLM  
14. MotionAnyMesh  
15. REACT3D  
16. Articulate-Anything  
17. Articulate AnyMesh  
18. PAct  
19. DreamArt  
20. ArtiLatent  

这组基本覆盖当前最新的 simulation-ready articulated/physical asset generation。UniPhysGen 联合评测 articulation 和 intrinsic physical properties；PhysX-Omni 将 geometry、scale、material、affordance、kinematics 等纳入统一 physical benchmark；Real-IKEA 专门提出 collision-surface deviation 和 resistance calibration；EmbodiedGen V2 则显式验证 collision success 和 cross-simulator/policy-ready usage。

### 主文建议 9 组

**Ours / Articraft / Infinigen-Sim / Infinite Mobility / Artiverse / UniPhysGen / PhysX-Omni / Real-IKEA / ArtVIP**

---

# Table 6 — Diversity & Coverage

这一表的指标也建议正式冻结。

## A. 类内几何多样性

只保留两个：

### Shape-Vendi@200 ↑

每类最多统一采：

\[
200
\]

assets。

### Near-Duplicate Rate ↓

使用冻结的 3D embedding / geometric distance threshold。

---

## B. 结构多样性

### Unique Kinematic Graph Ratio ↑

\[
\frac{\#unique\ canonical\ graphs}{N}
\]

### Kinematic Graph Perplexity ↑

\[
\exp
\left(
-\sum_g p(g)\log p(g)
\right)
\]

---

## C. 参数空间

### ParamCov ↑

10-bin marginal coverage。

### BoundaryCov ↑

上下边界是否真实覆盖。

---

## D. 组件组合

### SlotCov ↑

所有 candidate component 是否出现。

### Pairwise Combination Coverage ↑

不建议主文上 3-wise，放 supplementary。

---

## E. 跨类别 / 长尾

### Effective Category Number ↑

类别熵指数。

### TailCatCov@10 ↑

定义为：

> 每个 tail category 至少有 10 个有效、非重复资产的类别比例。

---

## F. 重复

### Near-Duplicate Rate ↓

不再额外加 4 种 duplicate 指标进主表。

Exact package duplicate 可以 supplementary 报。

---

## 最终 Table 6

| Dataset | #Cat. | Effective Cat. ↑ | TailCatCov@10 ↑ | Shape-Vendi ↑ | Near-Dup ↓ | Unique Graph ↑ | Graph PPL ↑ | ParamCov ↑ | BoundaryCov ↑ | SlotCov ↑ | Pairwise Comb. ↑ |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|

对非 procedural dataset：

`ParamCov / BoundaryCov / SlotCov / CombinationCov = N/A`

不是 0。

---

## 20 篇调研池

1. Infinigen-Sim  
2. Infinite Mobility  
3. Articraft  
4. Artiverse  
5. PhysX-Omni / PhysXVerse  
6. UniPhysGen / UniPhys-40K  
7. PhysForge / PhysDB  
8. Instruct-Particulate large corpus  
9. ArtLLM training corpus  
10. PartNet-Mobility  
11. AKB-48  
12. PAct  
13. CAGE  
14. SINGAPO  
15. ArtFormer  
16. ArtiLatent  
17. PWM-ArtGen  
18. UniArt  
19. DreamArt  
20. FreeArt3D  

程序化分布最重要的直接对照是 Infinigen-Sim 和 Infinite Mobility；Artiverse/Articraft 则提供人工或 Agent-generated articulated distributions。PWM-ArtGen、ArtiLatent、CAGE、SINGAPO 等提供 generative-distribution reference。

### 主文建议

**Ours / Infinigen-Sim / Infinite Mobility / Articraft / Artiverse / PartNet-Mobility / UniPhys-40K / PhysXVerse**

---

# Table 7 — Throughput & Cost

这个表必须**统一硬件重新跑**，不能直接拿不同论文自己的秒数做排行榜。

论文 published time 只能放一个 `Reported` supplementary table。

## 最终主表指标

| 指标 | 保留 |
|---|---:|
| Template Authoring Time ↓ | ✓ |
| Template Authoring Cost ↓ | ✓ |
| Valid Assets / Hour ↑ | ✓ |
| p50 Seed Latency ↓ | ✓ |
| p95 Seed Latency ↓ | ✓ |
| CPU-hours / 1K Valid ↓ | ✓ |
| Peak RAM ↓ | ✓ |
| Scaling Efficiency@32 ↑ | ✓ |
| Amortized Cost@100 ↓ | ✓ |
| Amortized Cost@1K ↓ | ✓ |
| Amortized Cost@10K ↓ | ✓ |

其中：

\[
C(N)=
\frac{C_{author}+C_{repair}}{N}
+
C_{compile+QC}
\]

是主成本定义。

当前已有模板 1,188 seed 在固定 8 workers 下 wall time 为 1,794.88 秒，可以作为 throughput pilot，但不是 template authoring speed。

---

## 20 篇相关效率参考

1. Articraft  
2. Nova3D  
3. ArtiCAD  
4. SceneCode  
5. MUSE / AuthorBench  
6. P3D-Bench  
7. CadBench  
8. BlenderRAG  
9. URDF-Anything+  
10. Particulate  
11. Instruct-Particulate  
12. ArtLLM  
13. SIMART  
14. PAct  
15. UniArt  
16. DreamArt  
17. FreeArt3D  
18. Articulate-Anything  
19. MotionAnyMesh  
20. EmbodiedGen V2  

Articraft 是最重要的成本 reference，因为它逐资产运行 Agent；Nova3D、ArtiCAD 等也是 executable program generation；URDF-Anything+ 和 Particulate 则代表 feed-forward articulated inference 的另一端。

### 主文真正跑

- Naive same-LLM
- Full Ours
- Articraft

如果 Nova3D/ArtiCAD 有可复现实例，也可加。

---

# Table 8 — Downstream Utility

这里不要放“20 个生成方法一起训练”。

**这张表的变量必须是训练数据。**

## 固定实验组

| ID | Training Data |
|---|---|
| P0 | Original Particulate |
| P1 | P0 + Articraft-10K |
| P2 | P0 + Ours-10K |
| P3 | P0 + Ours-LowDiv-10K |
| P4 | P0 + Ours-30K |
| P5 | P0 + Articraft-10K + Ours-10K |

所有组：

- 同 Particulate backbone；
- 同 optimizer；
- 同 steps；
- 同 batch size；
- 3 个 train seeds。

---

## 最终指标

严格沿用 Particulate / Articraft：

### Rest Pose

- gIoU ↑
- PC ↓
- mIoU ↑

### Articulated Geometry

- gIoU ↑
- PC ↓
- OC ↓

再增加一个：

### OOD Articulated gIoU ↑

仅在训练数据未出现的类别上平均。

所以最终 7 个指标：

| Training | Rest gIoU | Rest PC | mIoU | Art gIoU | Art PC | OC | OOD Art gIoU |
|---|---:|---:|---:|---:|---:|---:|---:|

Articraft 已证明使用 Articraft-10K 增强 Particulate 可以提升 Lightwheel 上的 rest-pose 和 articulated geometry 指标，因此这是一条直接可延续的数据效用协议。

---

## 下游相关 20 篇调研池

1. Articraft  
2. Particulate  
3. Instruct-Particulate  
4. Infinigen-Sim  
5. Artiverse  
6. Infinite Mobility  
7. EmbodiedGen V2  
8. Real-IKEA  
9. ArtVIP  
10. UniPhysGen  
11. PhysX-Omni  
12. PhysX-Anything  
13. URDF-Anything+  
14. ArtLLM  
15. SIMART  
16. PAct  
17. PWM-ArtGen  
18. ART  
19. REACT3D  
20. MotionAnyMesh  

这些最新工作覆盖 articulation estimation、generated-data augmentation、robot policy learning、physical interaction 和 sim-to-real，但它们不是都应该作为同一 Particulate 表中的 row。

---

# 最终冻结：8 张主表

| Table | 最终内容 | 主指标数量 | 推荐主表 baseline 数 |
|---|---|---:|---:|
| **T1** | Dataset / Generator Scale | 15 descriptive | 8–10 |
| **T2** | Template + Distribution Reliability | 12 | Full + 5 ablation + 2–3 refs |
| **T3** | Semantic Structure + Constraints | 5 + 12 | 6–8 |
| **T4** | Distributional Editability | 11 | 5–7 |
| **T5** | Articulation + Collision + Physics | 13–16，两 panel | 8–9 |
| **T6** | Diversity + Coverage | 11 | 7–8 |
| **T7** | Throughput + Cost | 11 | 3–5 local reruns |
| **T8** | Particulate Downstream | 7 | 6 training arms |

---

# 哪些指标现在正式删除

为了避免 paper 看起来像“指标大杂烩”，建议主文删掉这些：

| 删除/移附录 | 原因 |
|---|---|
| Raw Naming Richness | 可被过度分件刷高 |
| Raw #Parts comparison | 不同 representation 粒度不公平 |
| Raw tree depth ranking | 深不等于正确 |
| Named Groups 数量 | URDF 与 Blender group 表示不等价 |
| metadata-present Axis Valid | 不等于 axis 正确 |
| boundary smoke as motion validity | 不等于全行程 |
| 4 种 duplicate 指标同时主报 | 冗余 |
| 1-NNA / MMD 全库主指标 | 缺统一 reference 且任务不匹配 |
| CLIP/t-SNE | 当前主要论文贡献不是视觉 realism |
| PSNR/CD/F-score | 无成对 GT |
| Human Acceptance=100% | 经筛选发布集会造成循环定义 |

---

# 最终最重要的 15 个核心指标

如果 reviewer 只看一页，我希望看到的是：

### 方法
1. Template Final Success  
2. All-36 Pass Template  
3. Corner Pass  
4. Regression Retention  

### 结构与可控
5. Semantic P/R  
6. Hierarchy Exact Match  
7. Constraint Satisfaction  
8. All-Pass Assets  
9. 16-Seed Edit Propagation  

### 铰链与物理
10. Joint Accuracy  
11. Full-Range Collision-Free  
12. Worst-State Stable  
13. 4-Simulator L5 All-Pass  

### 分布
14. Shape-Vendi + Near-Duplicate  
15. Param/Combination Coverage  

最后由 **Particulate OOD improvement** 作为数据价值终点。

---

# 推荐实验优先级

| 优先级 | 必须完成 |
|---|---|
| **P0** | 54-task Template Authoring + naive/ablation |
| **P0** | Frozen Constraints |
| **P0** | Full-range motion sweep |
| **P0** | Diversity/coverage |
| **P0** | Particulate downstream |
| **P1** | Formal 18×16 editability |
| **P1** | Multi-simulator |
| **P1** | Regression manifest |
| **P2** | 深度 physics stability |
| **P2** | 辅助 VLM/user-study |

最终不要把 paper 写成“我们有 20 个 baseline × 8 张表”，而应该写成：

> **我们调研和覆盖了约 30–40 个最新相关方法，每个评测轴从其中选出与该轴任务兼容的 5–10 个直接 baseline，其余作为 supplementary reference。**

这既比只放 Articraft/Nova3D 两三个对照强很多，也比把任务不兼容的方法强行记为 0 更可信。
