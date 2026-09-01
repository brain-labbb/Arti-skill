# Nano3D 未直接可测指标的难度分析与补齐顺序

本文基于 [`Nano3dexp1.md`](/mnt/zsn/lyb/arti-skill/exp/Nano3dexp1.md) 整理，分析当前不能直接测出、或需要新增支持才能测出的指标。难度不是单纯的编码难度，而是综合考虑：数据/标注依赖、evaluator 复杂度、实验规模、跨 seed 依赖、物理验证和可复现要求。

## 1. 当前实验边界

当前实验是 **33 个已导出资产的 existing-export pilot / static audit**，不是完整的 Nano3D benchmark。

已具备的基础结果包括：

- 33/33 资产可以隔离重执行；
- 33/33 保存 generated URDF artifact；
- 33/33 URDF 有效；
- 33/33 层级树有效；
- raw name coverage 均值 100%；
- joint metadata 字段覆盖均值 100%；
- 33/33 基础资产包完整；
- 仅 1/10 个 physics_10 资产有有效 physics validation report。

这些结果主要证明“已有导出资产可执行、可解析、具备基础结构”，不能直接外推到语义正确性、约束满足、编辑传播、全行程物理有效性或 54-template benchmark 泛化。

## 2. 难度分级总览

| 难度 | 指标类别 | 当前可测范围 | 需要新增的核心支持 |
|---|---|---|---|
| 低 | Existing-export + seed-distribution Reliability | 已完成 33 asset 重执行；已完成 33 existing templates × 36 seeds 的 compile、Full QC、artifact、36/36、config/耗时/错误日志 | 若扩展到生成 benchmark，仍需 authoring/repair telemetry |
| 低 | Raw Naming、URDF 合法树、关节字段、基础 package | 名称存在率、占位名、root、depth、可达性、字段存在性 | parser 和静态检查器 |
| 中 | 跨 seed lexical consistency、基础 articulation smoke test、mesh 静态质量 | 多 seed 词法稳定性、少量 rest/boundary pose、watertight 等 | 多 seed manifest、渲染器、mesh/碰撞基础工具 |
| 中高 | Semantic Naming、Semantic Hierarchy | 需要对部件语义和装配语义进行判断 | semantic gold、hierarchy gold、canonicalization、judge/scorer |
| 高 | Constraints | 可检查部分 count、limit、bounding box | template spec、约束测量器、容差、合法/非法组合 |
| 很高 | 完整 Reliability benchmark | 54-template × 36-seed、first-shot、repair、36/36 | frozen benchmark、三次生成、repair budget、QC、telemetry |
| 很高 | Editability | 单个 edit smoke test | edit task、16-seed regression、diff scorer、盲评 |
| 很高 | Articulation full-range | joint 表示、少量状态测试 | kinematic gold、运动采样、碰撞检测、状态日志 |
| 很高 | Production Readiness | package、路径、部分 mesh 审计 | manifold/self-intersection、物理验证、determinism、clean rebuild |
| 最高 | 完整七轴 benchmark 与 baseline 对比 | 当前只能做 pilot 对照 | 全部 gold、baseline、evaluator、reproducibility manifest |

## 3. 低难度指标：不依赖语义 gold

### 3.1 Existing-export Reliability

目标分成两层：existing-export pilot 验证已有导出资产是否可以在固定环境中重复执行；新增 seed-distribution cohort 验证 33 个冻结既有模板在 seeds 0–35 上的生成质量。两层都不等于“从任务输入新生成模板”的 authoring success。

可以测量：

- `model.py` 是否可执行；
- URDF 是否重新生成；
- artifact 是否保存；
- wall time；
- stdout/stderr；
- compile/probe count；
- 错误阶段和错误类型。

已新增并完成：

- 33 个冻结既有模板 × seeds 0–35 = 1,188 seed；
- resolved config、template/source hash、编译结果、artifact 路径、Full-QC 结果、motion-QC report、失败原因和耗时；
- `Seed Compile = 1,188/1,188`、`Seed QC = 1,188/1,188`、`36/36 Pass = 33/33`；
- 正式运行固定 8 workers、数值线程=1、单 seed 180 s timeout，并将全部产物写在 `exp/runtime/nano3d_seed_reliability/`。
- 项目原生 Corner protocol 共运行 231 个 corner cases，231/231 通过，33/33 templates 达到 strict all-corner pass；产物写在 `exp/runtime/nano3d_corner/`。

仍不能测量：

- template first-shot success；
- repair success；
- 三次独立模板生成可靠性；
- Nano3d 草案所写的“每模板固定 4 个 continuous corners”精确协议，以及 regression retention。当前已完成的是项目原生 corner selector：22 个 domain 模板的 99 个显式 domain cases，加 11 个 legacy 模板的 132 个可达 corner seeds。

所需支持：冻结 asset/template manifest、隔离运行目录、统一 compiler、错误 taxonomy 和 telemetry。

### 3.2 Raw Naming 与结构合法性

可以直接从源文件和 URDF 解析：

- link/joint 名称存在率；
- 占位名称比例；
- 重复名称率；
- root 数量；
- 是否有环；
- link 可达性；
- 最大深度；
- parent-child 引用闭合；
- joint type、axis、origin、limit 字段是否存在；
- visual/collision mesh 路径是否可解析。

这些指标的限制是：

- 名称是可读字符串，不代表语义正确；
- parent-child 是合法图关系，不代表真实装配关系；
- 字段存在不代表数值正确；
- package 完整不代表物理可用。

## 4. 中等难度指标：需要多 seed 或基础工具

### 4.1 跨 seed lexical consistency

需要同一模板的多个 seed；本轮已在选定资产的原始 source root 中找到 23 个 multi-seed cohort、271 个可解析 sibling seed，但 10 个 physics_10 cohort 仍是 single seed，因此目前只有 reduced raw lexical/structural 对照，不能形成正式的全量跨 seed 结论。

可以分析：

- 名称集合的交并比；
- 左右、编号和重复实例命名是否稳定；
- joint 类型是否稳定；
- 层级深度和 link 数量是否稳定。

但这仍然只是词法/结构一致性，不是语义一致性。要进行跨 seed 正式分析，必须先冻结每模板 16 或 36 个 seed。

### 4.2 基础 articulation smoke test

低复杂度资产已完成第一版 smoke：

- rest pose 是否能加载；
- lower/upper boundary pose 是否能加载；
- joint 是否能驱动目标 link；
- 是否出现明显碰撞或断裂。

本轮实际完成 33/33 `urdfpy`/PyBullet load、33/33 rest step 和 286/286 单关节 boundary step；诊断 self-contact 为 0。

这不能替代：

- 全行程碰撞检测；
- 多关节组合状态；
- clearance；
- joint semantic correctness；
- formal kinematic gold 评分。

### 4.3 Mesh 静态质量

可以使用 mesh 工具统计：

- watertight；
- winding consistency；
- open edges；
- degenerate faces；
- 顶点/三角面数量；
- 文件大小。

不能仅凭这些结果判断：

- manifold 完整性；
- self-intersection；
- visual/collision 几何是否合理；
- 质量是否满足物理仿真。

## 5. 中高难度指标：需要语义 gold

### 5.1 Semantic Naming

目标指标包括：

- semantic precision；
- semantic recall；
- functional richness；
- instance discriminability；
- over-segmentation rate；
- cross-seed semantic consistency。

当前无法直接测量的原因是，名称存在并不等于名称对应真实部件。例如 `front_left_arm` 可能只是合理命名，也可能是错误命名。必须建立：

1. 每个模板的 required/optional semantic parts；
2. 同义词和别名规则；
3. 重复实例标识规范；
4. extra-real-part 标注；
5. 固定 judge prompt/model；
6. 人工复核样本和一致性指标。

没有这些定义时，只能报告 raw name coverage，不能报告 semantic precision/recall。

### 5.2 Semantic Hierarchy

目标指标包括：

- parent-child edge F1；
- hierarchy exact match；
- semantic nesting accuracy；
- named groups；
- pivots；
- cross-seed topology consistency。

当前可以确定树是否合法，但不能确定“这个父节点是否应该是这个部件的语义父节点”。还要解决：

- wrapper/group 节点是否折叠；
- visual/collision/helper 节点是否计入；
- pivot 节点与几何节点如何对齐；
- 固定拓扑和可变拓扑如何区分。

所需支持是每模板 frozen hierarchy gold、节点 canonicalization 和 topology compatibility rules。

## 6. 高难度指标：Constraints

Constraints 不能由 `compile_report.status=success` 推出。编译成功只说明程序或 artifact 基本可构建，不说明数量、尺寸、关系或接口约束满足。

正式约束评测至少需要：

- 每模板隐藏 `template_spec.json`；
- required/optional parts；
- count constraints；
- numeric constraints 与 tolerance；
- relational constraints；
- interface constraints；
- kinematic constraints；
- compatibility rules；
- valid/invalid combination manifest；
- 从最终 URDF、mesh、状态重新测量的 evaluator。

正式指标包括：

- coverage；
- satisfaction；
- conditional accuracy；
- count pass；
- numeric pass；
- relational/interface/kinematic/compatibility pass；
- all-pass assets；
- invalid combination rejection。

约束评测依赖 Naming 和 Hierarchy：如果 evaluator 无法找到语义部件或正确的装配节点，后续测量就不可靠。因此它不应在 semantic gold 之前作为最终结果运行。

## 7. 很高难度指标：完整 Reliability

完整 Reliability 仍需要从当前 33 个**既有模板**的 seed-distribution cohort 升级为任务到模板的生成实验：

```text
54 templates × 36 seeds = 1,944 assets
54 templates × 3 independent runs = 162 generation runs per method
```

需要记录：

- template executable rate；
- artifact-saved rate；
- first-shot success；
- final success；
- repair turns；
- seed compile/QC；
- 36/36 pass；
- corner pass（当前 33-template 项目原生协议已完成；完整 54-task/fixed-4 协议仍缺）；
- regression retention；
- wall time、agent turns、tokens、API cost。

关键缺口：

- 54-template frozen task manifest；
- 剩余 21 个 benchmark task/template 及其 36-seed 覆盖；当前 33-template seeds 0–35 manifest 已完成；
- 三次独立调度器；
- 统一 repair budget；
- 新生成模板的 seed-level QC 接入；当前 33 个既有模板的 runner/harness 已完成；
- baseline 运行记录；
- token/API telemetry。

这是实验基础设施问题，不是单独增加一个指标函数即可解决。

## 8. 很高难度指标：Editability

Editability 的正式验证需要证明“目标被改对，并且其他内容没有被破坏”。

需要覆盖：

- target fulfillment；
- anchor correctness；
- scale correctness；
- non-target preservation；
- geometry locality；
- structural locality；
- post-edit constraint pass；
- 16-seed propagation；
- regression preservation；
- final pass；
- edit cost。

当前最多只能做单个 edit smoke test：修改 source、重新生成、确认输出改变且 URDF 仍可解析。不能据此推出正式 edit pass。

正式执行需要：

1. 18 个模板；
2. 54 个 edit tasks；
3. 每个任务 16 个 regression seeds；
4. 编辑前后 artifact 对齐；
5. geometry/structure diff scorer；
6. 非目标保持判定；
7. 三名评测员的盲评和一致性统计。

Editability 还依赖稳定的 Naming、Hierarchy 和 Constraints，否则无法判断目标、锚点或非目标区域。

## 9. 很高难度指标：Articulation full-range

当前能测的是 URDF 表示层：

- joint type；
- parent-child；
- axis；
- origin；
- limits；
- movable joint 数量。

完整 articulation 需要进一步验证：

- joint type accuracy；
- joint recall；
- parent-child accuracy；
- axis 是否位于 moving part；
- origin 是否在真实 pivot；
- limit reachability；
- joint geometry validity；
- asset geometry validity；
- full-range collision-free；
- clearance；
- generic range rate。

所需支持：

- kinematic gold；
- 单关节 11-state 采样；
- 多关节 64-state Sobol 或等价采样；
- 低 clearance 区域加密采样；
- CCD 或等价碰撞检测；
- motion state logs；
- joint/asset failure taxonomy。

physics_10 中目前只有 1/10 资产有有效 validation report，因此其余 9 个不能被记为 physics pass。

## 10. 很高难度指标：Production Readiness

Production Readiness 至少包含以下层面：

### 10.1 Geometry

- watertight；
- manifold；
- open edges；
- degenerate faces；
- self-intersection；
- visual/collision deviation；
- exact、convex hull、CoACD collision variants。

### 10.2 Source and build

- clean rebuild；
- deterministic output；
- source/URDF/mesh hash；
- 可移植依赖；
- 版本和环境记录。

### 10.3 Physical and runtime

- mass；
- inertia；
- damping/friction；
- visual/collision geometry；
- physics validation；
- UV/material readiness。

当前的 package audit、路径检查和部分 mesh 静态检查只能构成 Production Readiness 的基础层，不能替代完整生产质量评估。

## 11. 最高难度：完整七轴 benchmark 与 baseline 对比

完整实验需要同时满足：

- 54-template frozen task manifest；
- 36-seed manifest；
- hidden template specs；
- 四类 baseline 公平运行；
- 162 个模板生成 run 的日志和 artifact index；
- Naming/Hierarchy/Constraints gold 与 scorer；
- 54 edit tasks × 16 regression seeds；
- articulation motion sweep 与 collision logs；
- mesh/URDF/source production telemetry；
- 七轴 evaluator；
- failure taxonomy；
- reproducibility manifest。

任何一个环节缺失，都只能报告 pilot、static audit 或 reduced benchmark，不能报告完整 Nano3D benchmark。

## 12. 指标依赖关系

```text
Frozen manifest / environment
          ↓
Static parser + package audit
          ↓
Raw Naming + structural Hierarchy + joint metadata
          ↓
Semantic Naming gold + semantic Hierarchy gold
          ↓
Constraint evaluator
          ↓
Edit task + diff scorer
          ↓
Articulation motion/collision evaluator
          ↓
Production readiness + cross-seed regression
          ↓
54-template × 36-seed × baseline comparison
```

其中：

- 33-asset manifest 与 33-template × seeds 0–35 manifest 已完成；尚无完整 54-task authoring manifest；
- 没有 semantic gold，不能报告语义 Naming/Hierarchy；
- 当时没有 constraint evaluator，不能报告 satisfaction；现已新增 source-derived/operational Constraints v1，但仍不是独立 hidden-spec 评测；
- 没有 edit task 和回归 seed，不能报告 Editability；
- 没有运动状态与碰撞日志，不能报告 full-range articulation；
- 没有 clean rebuild 和 hash，不能报告 Deterministic Build。

## 13. 推荐补齐顺序

### P0：冻结输入和环境

1. 已冻结 33 资产 manifest、路径和 SHA-256，并冻结 33 个既有模板的 source hash 与 seeds 0–35；
2. seed Reliability 已冻结 8 workers、数值线程=1、180 s timeout；compiler/Python/mesh/physics 的完整版本锁仍需补充；
3. 保留原始资产只读；
4. 明确 `validated`、`compile_only`、`missing_report` 三类 physics 状态。

### P1：完成基础静态层

1. artifact/package audit；
2. URDF parse 和结构树；
3. raw naming；
4. joint metadata；
5. mesh 静态审计；
6. source/URDF/assets telemetry。

### P2：补 semantic gold

1. 补齐缺少 TemplateDesign 的 11 个资产；
2. 建立 semantic parts、hierarchy、joints、constraints、compatibility、edit fields；
3. 人工审核 spec；
4. 编写 scorer 正例和失败例。

### P3：运行 reduced benchmark

已完成 33 个既有模板 × 36 seeds 的 reduced seed-distribution Reliability benchmark。结果只报告实际覆盖的 33 templates/1,188 seeds，不能与完整 54-task authoring benchmark 数字直接比较。

### P4：运行完整 benchmark

1. 冻结 54 个任务；
2. 冻结 36-seed manifest；
3. 运行四类 baseline；
4. 每方法每任务独立运行 3 次；
5. 分七个 axis 统计；
6. 生成结果表、coverage matrix、failure taxonomy 和 reproducibility manifest。

## 14. 当前禁止性结论

在上述支持补齐前，不能写：

- “33 个资产完成 Nano3D 完整 benchmark”；
- “所有 33 个资产都 physics-ready”；
- “compile success 等于 constraint satisfaction”；
- “URDF 有 joint 等于 articulation 全行程有效”；
- “有部件名称等于 semantic naming precision 高”；
- “有 parent-child edge 等于 hierarchy exact match”；
- “单次 edit 成功等于 16-seed distributional edit success”；
- “存在模板 Python 文件等于完成 Full Arti-Template baseline”；
- “33 个样本证明了 54-template 分布泛化”。

推荐表述为：

> 当前实验已建立 33 个已导出资产的执行、artifact、原生层级、基础命名、基础关节和包结构基线；语义、约束、编辑传播、全行程物理和完整 baseline 对比仍需补齐相应 gold、evaluator、seed manifest 和实验基础设施。

## 15. 低/中难度任务已执行状态（更新至 2026-08-05）

本轮严格只在 `/mnt/zsn/lyb/arti-skill/exp/` 内新增 harness、运行产物和文档更新，未修改 exp 目录之外的代码。

### 15.1 低难度：已完成

复用了既有 `run_nano3d_asset_pilot.py` 的结果并进行汇总核验：

- 33/33 资产隔离重执行成功；
- 33/33 保存 generated URDF artifact；
- 33/33 URDF 有效、33/33 valid tree；
- 33/33 basic package complete；
- raw name coverage 均值 100%；
- joint metadata rate 均值 100%；
- mesh errors 为 0；
- 387 个 mesh geometry 可读；
- watertight 率 32.49% per-asset mesh-fraction mean；37.47% geometry-level fraction；
- winding-consistent 率 97.93%；
- open edges 290,335；degenerate faces 51；
- 10 个 physics_10 资产中仅 1 个有有效 validation report。

新增 seed-distribution Reliability：

- 33 个冻结既有模板 × seeds 0–35，共 1,188 seed；
- 1,188/1,188 compile、1,188/1,188 Full QC、1,188/1,188 artifact saved；
- 33/33 templates 达到 36/36；
- 正式统一复跑为 8 workers、每数值库 1 thread、180 s/seed timeout；
- mean/median/p95 strict seed elapsed = 9.58/5.72/42.57 s，cohort wall = 1,794.88 s；
- 所有 resolved config、模板 hash、artifact lineage、motion-QC 和失败字段写入 `runtime/nano3d_seed_reliability/`；
- 项目原生 Corner 共 231 cases：22 个 domain 模板 99 个显式 domain cases，11 个 legacy 模板 132 个可达 corner seeds；231/231 compile、Full QC、artifact hash/non-rigid 校验通过，33/33 templates strict all-corner pass；
- Corner mean/median/p95 elapsed = 10.59/5.88/46.23 s，harness wall = 1,035.40 s；全部记录写入 `runtime/nano3d_corner/`；
- 未运行 template authoring、repair 或 regression；Nano3d 草案的固定 4 个 continuous corners/template 精确协议也未运行，因此不能把项目原生 Corner 数值当作该协议的结果。

### 15.2 中难度：跨 seed lexical/structural consistency

补测 harness 扫描了选定资产原始 source root 下的同 slug sibling seeds：

- 33 个选定 slug cohort；
- 23 个 multi-seed cohort；
- 10 个 single-seed cohort（均为 physics_10，未用于跨 seed 结论）；
- 共 271 个可解析 seed 的 URDF；
- seed-level URDF valid rate：100%；
- multi-seed cohort 的 name signature mode rate 均值：32.90%；
- multi-seed cohort 的 pairwise name Jaccard 均值：55.52%；
- multi-seed cohort 的 link-count mode rate 均值：54.49%；
- single-seed cohort 不报告跨 seed consistency。

这些结果是 lexical/structural stability，不是 semantic consistency。低 mode rate 说明不同 seed 的导出名称集合或结构计数变化较大，但不能直接判断哪一个 seed 是正确的，因为当前没有 semantic/hierarchy gold。

### 15.3 中难度：articulation representation smoke

对 33 个选定资产检查了：

- URDF 是否可解析；
- movable joint 是否具有非零有限 axis；
- joint origin 是否为有限值或合法默认值；
- revolute/prismatic limit 是否有限且 lower ≤ upper；
- rest-pose/boundary-pose 的表示层 readiness；
- visual/collision link 覆盖。

结果：

- 33/33 representation smoke pass；
- 33/33 `urdfpy` load success；
- 33/33 PyBullet load success；
- 33/33 rest-pose step success；
- 286/286 个单关节 boundary state step success；
- rest 和 boundary smoke 的 self-contact diagnostic 均为 0；
- 32/33 资产的所有 link 都有 visual/collision 配对；
- 1 个 Sports car 资产有 8/10 link 配对；
- 已建立隔离环境 `/mnt/zsn/lyb/arti-skill/exp/.venv_low_medium` 并安装 `pybullet`/`urdfpy`；本轮运行的是 rest + individual boundary smoke，不是完整 physics sweep。

因此本轮可以报告“URDF 可被 urdfpy/PyBullet 加载并完成 rest/单关节 boundary smoke”；仍不能报告 full-range collision-free、clearance、limit reachability 或 joint semantic correctness。

### 15.4 中难度：mesh 静态质量

本轮复用并重新汇总 mesh 静态审计：

- 387 个 geometry 可读；
- watertight 率 32.49% per-asset mesh-fraction mean；37.47% geometry-level fraction；
- 145 个 geometry watertight；
- 379 个 geometry winding-consistent；
- 290,335 个 open edges；
- 51 个 degenerate faces；
- 0 个 mesh parser errors。

这仍然不是完整 manifold、self-intersection、CoACD、视觉/碰撞偏差或物理稳定性验证。

### 15.5 新增输出

- [`run_nano3d_low_medium.py`](/mnt/zsn/lyb/arti-skill/exp/scripts/run_nano3d_low_medium.py)
- `/mnt/zsn/lyb/arti-skill/exp/runtime/nano3d_low_medium/cross_seed_records.json`
- `/mnt/zsn/lyb/arti-skill/exp/runtime/nano3d_low_medium/articulation_smoke_records.json`
- `/mnt/zsn/lyb/arti-skill/exp/runtime/nano3d_low_medium/low_medium_static_records.json`
- `/mnt/zsn/lyb/arti-skill/exp/runtime/nano3d_low_medium/summary.json`

## 16. 指标和对比组对齐后的当前状态

对照 `Nano3d.md` 正式结果表后，已修正以下口径问题：

1. `Existing-export pilot` 是已有资产重执行，不是 template generation；不再把 33/33 asset re-exec 填成 Template Exec.；
2. Table 2–5 恢复为 `Nano3d.md` 的正式表头，额外的 evaluation unit、constraint set、parts/asset 等说明移到注释；
3. 本地 `URDF link`、论文 `semantic parts` 不再当成同一 Part Exists 指标；
4. 本地 raw name Jaccard 不再填入正式 semantic Cross-Seed Consistency；
5. 本地 link-count mode rate 不再填入正式 hierarchy topology consistency；
6. 本地 mesh/open-edge/source/size 同时报告 per-asset mean 和 total；
7. 论文的 triangles 不再填入本地 `Degenerate Faces`；
8. 论文 GLB/STEP 与本地 URDF+raw mesh 明确标记为不同 artifact，不做文件大小排名；
9. PyBullet 的 rest/individual-boundary smoke 不再被写成 full-range collision-free 或 joint geometry validity。

## 17. 目前仍未完成的指标

### 17.1 Reliability

- 54-template frozen benchmark；
- 已完成本地 33 个既有模板的固定 seeds 0–35：1,188/1,188 compile、1,188/1,188 Full QC、33/33 templates 达到 36/36；尚缺完整 54-task cohort 的剩余 21 个任务；
- 每方法每模板 3 次独立生成；
- template first-shot 与 final generation success；
- 项目原生 corner seed pass 已完成：231/231 cases、33/33 strict all-corner templates；尚缺完整 54-task cohort 和 Nano3d 草案固定 4 个 continuous corners/template 的精确协议；
- regression retention；
- 统一 repair budget；seed-level QC、resolved-config 日志、artifact lineage 和 failure taxonomy harness 已完成，但尚未接入新模板 authoring/repair loop；
- agent turns、input/output tokens、API cost telemetry；
- 与 baseline 使用完全相同的输入、环境和运行协议。

当前有两组不同口径：33/33 existing-export asset re-execution，以及 33 个冻结既有模板 × 36 seeds 的分布测试。后者证明既有模板的 seed 质量，但两者都不能与论文“从 benchmark 输入生成模板”的 executable/first-shot/final success 直接横比。

### 17.2 Semantic Naming

- 已完成论文式能力阶梯拆分并完成 N=33 GLB 转换复核：239 个 mesh-bearing links 一一对应 239 个 GLB mesh nodes，Parts=7.242/asset，239/239 的 Nameability=1.000；这两项现为本地统一转换 GLB 的直接结果；
- 已完成 paper-aligned Richness candidate：排除 1 个 output-derived fallback 后，在 N=32 source-semantic 资产上计算 `named mesh links / required spec instances`，均值 1.482，asset bootstrap 95% CI [1.279, 1.709]；未经 judge 验证，不能当作论文同口径 richness；
- 已修复 role matcher：按实际命中的 pattern specificity 排序，并将 `min_count` 展开后做最大基数/最大 specificity 的一对一全局匹配，避免 `frame/leaf` 等泛词抢走 `leg_frame/drop_leaf/handle_leaf`；
- Naming v2.2 已将 role matching、judge queue、Instance 和 Cross-Seed raw names 统一限定为 mesh-bearing nodes；2 个无 mesh 的 Sports-car steering-knuckle groups 只参与 Hierarchy，不再计入 Naming；
- 修正后的 count-aware source proxy：source-role Recall macro=0.994、micro=147/149=0.987；Functional Core Coverage macro=0.993、micro=120/122=0.984；Instance=38/40=0.950（18 groups）。这些仍依赖复制后的 authoring/spec，不是 hidden gold；
- 匹配置信度审计：147 个 assignments 无 specificity exact tie，2 个 runner-up margin≤5；其中 120 个由 canonical role token 或至少两 token pattern 支持，27 个依赖冻结的 single-token alias。Strong-match sensitivity 为 macro=0.832、micro=120/149=0.805；它不是第二套 gold，只是规则敏感性下界；
- 已同时报告 Richness asset-macro=1.482 与 pooled micro=233/149=1.564，避免不同聚合方式被误认为同一结果；
- 已撤回旧 Semantic Precision=0.934：旧公式把未命中 core-role 的节点自动当 false positive，但其中包括 `guide_rail`、`gas_cylinder`、`release_lever` 等额外真实部件候选。当前 86 个额外候选已写入 `judge_queue.jsonl`，Semantic Precision 在三独立 judge 完成前保持 N/A；
- 已冻结 271 个 sibling-seed URDF 到 `exp/runtime/nano3d_naming/cross_seed_input_urdf/`；23 个 multi-seed cohort/261 seeds 的 mesh-only pair-micro raw name Jaccard=0.580、cohort-macro=0.542 [0.428, 0.655]；count-aware role multiset pair-micro=0.948、cohort-macro=0.952 [0.921, 0.979]，role-count signature mode rate=0.875；10 个 single-seed physics_10 cohort 排除；
- 已完成可复核输入 manifest、SHA-256、10,000 次 asset bootstrap 和三 judge 聚合脚本；
- 已使用 `reproduce_nano3d_naming.py` 独立写入 `runtime/nano3d_naming_repro_v22/` 并复跑；summary、asset records、cross-seed records、input manifest、judge queue 和 report 共 6 个正式产物 SHA-256 全部与 reference 一致，`all_files_match=true`；
- 已生成 `runtime/nano3d_naming_judge_packet_v1/`（packet protocol v1.1）：233 个 N=32 GLB mesh-node blind tasks、233 张 context-highlight + 三隔离视角预览、三份空白独立 judge 模板和完整 hash manifest；全量重建后 preview hash 集合完全一致；schema 明确区分未填写 `null`、无重复 `none` 与不适用 `not_applicable`；
- 独立 gold 标注模板覆盖 33/33 类别：全部有输出无关的类别文本，16 个 pictureX 类别另复制 46 张原始类别参考图，可开始 core-taxonomy 盲标；但现有 export 未保存精确 per-seed 生成请求，因此 optional/per-seed gold 仍为 0/33 ready；
- 三 judge 聚合器 v2.3 已支持 verdict/role/instance/same-part 分层 consensus，以及 Precision、Recall、Richness、Functional、Instance、Over-Segmentation 的独立 readiness gate；233 项 all-uncertain 时五类正式指标保持 N/A，完整 source-assignment synthetic 精确复现 Recall=147/149、Functional=120/122、Instance=38/40，一个人工 same-part pair 得到 excess=1、micro Over-Segmentation=1/233。以上 synthetic 只验证公式，不是语义实验数据；
- 仍未完成：三个独立 judge verdict 文件、independent hidden semantic gold、完整 required/optional per-seed part map、统一 functional ontology、重复实例人工 gold、over-segmentation 标注和 point/mesh-level semantic mask。

因此目前可以直接报告 Parts/Nameability；带 `*` 的 Recall、Richness、Functional Core Coverage、Instance 和 cross-seed 指标只能称为 source-derived/candidate proxy。Semantic Precision/三-judge Recall 必须保持 N/A，直到 [`aggregate_nano3d_naming_judges.py`](/mnt/zsn/lyb/arti-skill/exp/scripts/aggregate_nano3d_naming_judges.py) 成功聚合三个完整、独立的 judge 文件。

### 17.3 Semantic Hierarchy

- frozen hierarchy gold；
- wrapper/group/collision/helper 节点 canonicalization；
- parent-child edge F1；
- hierarchy exact match；
- semantic nesting accuracy；
- topology-fixed/topology-variable 规则；
- 正式的 cross-seed topology consistency。

当前的 33/33 valid tree 只证明单根、无环、可达和 parent-child 引用闭合。

### 17.4 Constraints

- 每模板隐藏 `template_spec.json`；
- count/numeric/relational/interface/kinematic/compatibility gold；
- 容差和测量定义；
- 从最终 URDF/mesh/state 重新测量的 evaluator；
- all-pass asset；
- invalid-combination rejection；
- 36-seed constraint reliability。

该阶段原本要求 Table 4 本地行保持 N/A，因为 compile success 和 PyBullet smoke 不能替代 constraint satisfaction。现已新增独立的 `run_nano3d_constraints.py`，按冻结 source-derived protocol 对 33×36 seed 的最终 URDF/design relations/motion-QC 评分并以 `*` 回填；论文同口径 hidden-spec metrology 与 invalid-combination rejection 仍保持 unsupported。

### 17.5 Editability

- 18 个模板和 54 个 edit tasks；
- target/anchor/scale 的正式定义；
- geometry/structure diff scorer；
- non-target preservation；
- post-edit constraint pass；
- 16-seed propagation；
- regression preservation；
- 三名盲评员及 Fleiss' kappa/Krippendorff's alpha。

当前没有正式 edit task，因此 Table 5 除论文报告行外，本地行全部 N/A。

### 17.6 Articulation

已完成：33/33 `urdfpy` load、33/33 PyBullet load、33/33 rest step、286/286 单关节 boundary step，rest/boundary smoke self-contact diagnostic 为 0。

仍未完成：

- joint semantic gold；
- type accuracy、joint recall、parent-child accuracy；
- axis/origin 误差；
- 多关节组合状态；
- 完整运动范围采样；
- clearance 和 continuous collision detection；
- limit reachability；
- full-range collision-free；
- 1,944 seed 级 articulation validation；
- physics_10 其余 9 个资产的 validation report。

### 17.7 Production Readiness

已完成基础静态层：package/path/ref audit、mesh 可读性、watertight proxy、winding consistency、open edges、degenerate faces、source/URDF/raw mesh per-asset size。

Production Readiness 仍未完成：

- manifold evaluator；
- self-intersection evaluator；
- exact/convex hull/CoACD collision variants；
- visual-collision deviation；
- mass/inertia/damping/friction validator；
- clean rebuild；
- deterministic output hash；
- semantic/kinematic/physical completeness gold；
- UV/material readiness；
- production-level portability across clean environments。

### 17.8 URDF → GLB 全量评测表示对齐（2026-08-05）

[`run_nano3d_urdf_glb_pilot.py`](/mnt/zsn/lyb/arti-skill/exp/scripts/run_nano3d_urdf_glb_pilot.py) 已参数化并扩展到全部 33 个完整 package。源 package 只读，复制输入、GLB 和记录全部位于 `exp/runtime/nano3d_glb_n33/`。

全量结果：

- 33/33 资产通过全部 conversion gates；
- 241/241 URDF links 保留为同名 GLB link nodes，另有每资产一个已知 `__urdf_world__` wrapper；
- 239/239 mesh-bearing links 保留为同名 GLB mesh nodes，239/239 通过 Nameability；
- Sports car 的 2 个空运动学 links 保留为空 GLB nodes；
- 1442/1442 source visuals 成功加载并合入对应 link mesh；
- 208/208 link-to-link hierarchy edges 保留；
- 33/33 world-space bounds 在 float 容差内保留；
- 33/33 资产两次导出的 GLB SHA-256 一致；
- GLB 总大小 22,133,936 bytes，均值 670,725 bytes/asset；仅作本地 artifact telemetry，不和论文不同 converter 的文件大小排名。

详细结果见 [`summary.json`](/mnt/zsn/lyb/arti-skill/exp/runtime/nano3d_glb_n33/output/summary.json)、[`records.json`](/mnt/zsn/lyb/arti-skill/exp/runtime/nano3d_glb_n33/output/records.json) 和 [`report.md`](/mnt/zsn/lyb/arti-skill/exp/runtime/nano3d_glb_n33/output/report.md)。Table 2 的 Parts/Named 现可直接写成 N=33 GLB mesh-node 统计，而不是 URDF proxy。

随后使用 [`verify_nano3d_naming_on_glb.py`](/mnt/zsn/lyb/arti-skill/exp/scripts/verify_nano3d_naming_on_glb.py) 直接从 33 个 GLB 读取 link/mesh node names，并用冻结 gold 与同一 role matcher 重算 direct/source-semantic Naming。最终 33/33 资产的 Parts、Named、required-role assignment、extra candidates、Recall、strong sensitivity、Richness、Functional 和 Instance 逐字段一致，全部汇总字段也一致。首次复算发现 4 个资产的同分 assignment 受 node 顺序影响；已在 matcher 和 protocol 中冻结名称字典序 tie-breaker，消除 URDF XML 与 glTF node order 差异。指标汇总没有变化，Naming 独立复现仍为 6/6 正式文件逐字节一致。验证记录见 [`naming_verification.json`](/mnt/zsn/lyb/arti-skill/exp/runtime/nano3d_glb_n33/naming_verification.json)。

仍未对齐的部分：converter 是本地 `trimesh` 实现，不是论文未公开实现；同一 link 内多个 visuals 的合并策略可能影响 Parts；shared headless scene 的尺度归一化、相机、灯光和渲染尚未实现；材质/纹理、articulation sidecar、semantic judge、point-mask IoU 和 render GT 仍未补齐。因此“GLB artifact 与计数单位一致”不等于全部论文 evaluator 已逐实现一致。

### 17.9 对比组和公平性

当前论文对比行是 source-grounded reported values，不是本地重跑。仍未完成：

- 在同一 33 或 54 item set 上重跑 paper baselines；
- 统一 text/image modality；
- 统一输出转换为同一 representation；
- 统一 success/artifact/mesh/physics evaluator；
- 统一 wall-time、token 和 cost 统计；
- 统一 per-asset/per-item aggregation；
- 明确哪些指标只对 code-native、CAD、part-aware 或 mesh-native 方法开放。

因此 `Nano3dresults.md` 现在可以做“口径透明的 paper-reported comparison”，但仍不能称为 local apples-to-apples benchmark。

## 18. 下一阶段优先级

1. 完成 semantic part gold 和 hierarchy correctness gold；论文 Hierarchy 四项已按 URDF-equivalent mapping 复现，剩余 N/A 是论文未评分的 Edge F1/Exact/Nesting 与正式 cross-seed；
2. 建立 constraints evaluator，先在 33 个 reduced cohort 上做小规模约束验证；
3. 扩展 articulation smoke 为多关节状态和 clearance/CCD 记录；
4. 补齐 physics_10 缺失 validation report；
5. 冻结 reduced benchmark 的 seed、task 和 spec；
6. 再运行 editability 和 36-seed reliability；
7. 最后才做统一 baseline 重跑和七轴最终比较。
