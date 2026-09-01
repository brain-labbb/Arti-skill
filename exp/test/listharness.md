# Nano3D Harness 清单

## 1. `run_nano3d_asset_pilot.py`

路径：[`/mnt/zsn/lyb/arti-skill/exp/scripts/run_nano3d_asset_pilot.py`](/mnt/zsn/lyb/arti-skill/exp/scripts/run_nano3d_asset_pilot.py)

用途：对 [`Nano3dasset.md`](/mnt/zsn/lyb/arti-skill/exp/Nano3dasset.md) 中冻结的 33 个资产执行可直接测量的 existing-export pilot。脚本只读 source asset，所有临时副本和输出写到 `exp/runtime/nano3d_asset_pilot/`。

### 1.1 运行命令

```bash
cd /mnt/zsn/lyb/arti-skill
/mnt/zsn/lyb/arti-skill/arti-template/.venv/bin/python \
  /mnt/zsn/lyb/arti-skill/exp/scripts/run_nano3d_asset_pilot.py \
  --timeout 180
```

只做静态审计、不重执行：

```bash
/mnt/zsn/lyb/arti-skill/arti-template/.venv/bin/python \
  /mnt/zsn/lyb/arti-skill/exp/scripts/run_nano3d_asset_pilot.py \
  --no-reexecute
```

### 1.2 流程

1. 从 `Nano3dasset.md` 解析 33 个绝对资产链接，并检查数量与去重结果。
2. 为每个资产生成 manifest，记录 source、difficulty、seed，以及 `model.py`、`model.urdf`、`assets`、`compile_report.json` 的 SHA-256。
3. 解析 `model.urdf`：检查 XML、link/joint 数量、单根、可达性、无环、parent-child edge，以及可动关节的 type/axis/limit 字段。
4. 检查 raw link/joint name 是否为通用占位名，得到 raw name coverage。
5. 解析 URDF 的 mesh 引用，检查缺失引用和绝对路径，并统计基本 package completeness。
6. 用 `trimesh` 读取 `assets` 下的 OBJ/STL/PLY/GLB/GLTF/OFF/DAE，记录可读数量、watertight、winding consistency、open edges 和 degenerate faces。
7. 若未指定 `--no-reexecute`，把每个资产复制到临时目录，在 `arti-template` 环境中调用 `agent.compiler.compile_urdf_report`，保存生成的 `model.urdf` 和 `reexecute_report.json`。
8. 写出逐资产记录、逐次重执行记录和汇总 JSON；Markdown 表由本 harness 结果人工核对后回填。

### 1.3 输出文件

| 文件 | 内容 |
|---|---|
| `asset_manifest.jsonl` | 冻结资产路径、来源、seed、difficulty、关键输入文件 hash |
| `static_records.json` | 33 条 URDF、命名、层级、关节字段、mesh 和 package 静态结果 |
| `reexecution_records.json` | 每个隔离重执行的 status、return code、告警、stderr/stdout tail、artifact path 和耗时 |
| `reexecuted/<asset_id>/model.urdf` | 重执行成功后保存的 generated URDF artifact |
| `reexecuted/<asset_id>/reexecute_report.json` | compiler 返回的 warnings、signals 和 generated artifact 信息 |
| `summary.json` | 33 资产的汇总指标和协议标识 |

### 1.4 已成功测评的指标映射

| Nano3D 表 | 本 harness 可提供的字段 |
|---|---|
| Reliability | existing export 的 re-execution、artifact saved、wall time；不提供 36/36、repair、tokens/API cost |
| Naming | raw name coverage；不提供 semantic precision/recall |
| Hierarchy | valid URDF tree、roots、acyclic、reachable、max depth、parent-child edges；不提供 semantic gold 指标 |
| Articulation | joint count、movable joint count、joint type distribution、metadata field rate；不提供 motion/collision correctness |
| Production Readiness | readable meshes、watertight、winding consistency、open edges、degenerate faces、mesh/source/URDF bytes、basic package completeness |
| Asset-Level Pilot Audit | 33 个资产的逐资产全部直接字段 |
| Resource & Cost | probe count、wall time、source LOC/size；不提供 agent/token/API telemetry |

### 1.5 本轮通过结果

- 33/33 existing exports 隔离重执行成功。
- 33/33 保存 generated URDF artifact。
- 33/33 URDF 有效、33/33 valid tree、33/33 basic package complete。
- raw name coverage 均值 100%；joint metadata rate 均值 100%。
- 387 个 mesh geometry 可读；watertight 率 37.47%；winding-consistent 率 97.93%；open edges 290,335；degenerate faces 51。
- 10 个 physics_10 资产中只有 1 个已有 `success=true` 且 `dataset_ready=true` 的 validation report。

### 1.6 不应由本 harness 推断的结论

本 harness 不含 hidden semantic gold、template constraint spec、edit task、16-seed regression、36-seed generation protocol、motion sweep、collision state log、self-intersection evaluator 或 API telemetry。因此它不能单独证明 Nano3D 的完整 Reliability、Naming、Hierarchy、Constraints、Editability、Articulation 和 Production Readiness 七轴结论；这些缺口和结果表口径已记录在 [`Nano3dexp1.md`](/mnt/zsn/lyb/arti-skill/exp/Nano3dexp1.md)。

## 2. 复现与审计注意事项

- 运行前应确认 `Nano3dasset.md` 仍包含 33 个唯一绝对链接；数量变化会让 harness 直接失败。
- `--timeout` 是单资产秒数，不是全局 timeout。
- 重执行使用 source asset 的临时副本，避免 compiler 对原始目录产生写入。
- `watertight` 和 `open_edges` 是 mesh 静态检查，不能替换 URDF 物理 validation。
- 修改 parser 或 compiler 版本后，应重新运行并保留新的 runtime 目录，避免把不同协议的 summary 混合。

## 3. `run_nano3d_low_medium.py`

路径：[`/mnt/zsn/lyb/arti-skill/exp/scripts/run_nano3d_low_medium.py`](/mnt/zsn/lyb/arti-skill/exp/scripts/run_nano3d_low_medium.py)

用途：执行 `Nano3dexp2.md` 中的低/中难度补测。脚本只在 `exp` 目录生成输出，不修改 source assets，也不修改 `arti-template` 或其他 exp 外代码。

运行：

```bash
cd /mnt/zsn/lyb/arti-skill
/mnt/zsn/lyb/arti-skill/arti-template/.venv/bin/python \
  /mnt/zsn/lyb/arti-skill/exp/scripts/run_nano3d_low_medium.py
```

检查内容：

- 读取既有 `nano3d_asset_pilot/summary.json`，核验低难度 33-asset 结果；
- 在每个选定 asset 的原始 source root 内扫描同 slug sibling seeds；
- 计算 name signature mode rate、pairwise name Jaccard、link/joint count mode rate；
- 对 33 个选定 URDF 做 axis、origin、joint limit 和 visual/collision 表示层 smoke；
- 汇总 mesh 可读性、watertight、winding consistency、open edges、degenerate faces。

本轮结果：23 个 multi-seed cohort、10 个 single-seed cohort、271 个可解析 seed；多 seed pairwise name Jaccard 均值 0.5552，link-count mode rate 均值 0.5449；33/33 `urdfpy` load、33/33 PyBullet load、33/33 rest step、286/286 boundary step 成功，诊断 self-contact 为 0。该流程是 rest + individual boundary smoke，不是完整 physics sweep，因此 full-range collision-free 结论仍 unsupported。

输出：

- `exp/runtime/nano3d_low_medium/cross_seed_records.json`
- `exp/runtime/nano3d_low_medium/articulation_smoke_records.json`
- `exp/runtime/nano3d_low_medium/low_medium_static_records.json`
- `exp/runtime/nano3d_low_medium/summary.json`

隔离环境复现：

```bash
uv venv --python /usr/bin/python3.12 /mnt/zsn/lyb/arti-skill/exp/.venv_low_medium
uv pip install --python /mnt/zsn/lyb/arti-skill/exp/.venv_low_medium/bin/python \
  pybullet==3.2.7 urdfpy==0.0.22 trimesh==5.0.0
uv pip install --python /mnt/zsn/lyb/arti-skill/exp/.venv_low_medium/bin/python \
  --no-deps 'networkx>=3.0'
```

第二条安装是因为 `urdfpy==0.0.22` 的旧元数据固定 `networkx==2.2`，而 Python 3.12 不兼容该版本；harness 对 NumPy 旧别名做了进程内兼容处理。主 `arti-template/.venv` 不需要降级 NetworkX。

## 4. `run_nano3d_hierarchy.py`

路径：[`/mnt/zsn/lyb/arti-skill/exp/scripts/run_nano3d_hierarchy.py`](/mnt/zsn/lyb/arti-skill/exp/scripts/run_nano3d_hierarchy.py)

用途：按 [Nova3D 论文 Section 7.2 / Table 10](https://arxiv.org/pdf/2607.22738v1) 复现确定性的 Hierarchy 轴，并为 URDF 冻结保守的 representation-equivalent mapping。论文只评分 tree existence、semantic depth、named groups、pivots，并明确不评分 nesting correctness；因此本 harness 不伪造 Edge F1、Exact Match 或 Semantic Nesting。

运行：

```bash
cd /mnt/zsn/lyb/arti-skill
python /mnt/zsn/lyb/arti-skill/exp/scripts/run_nano3d_hierarchy.py
```

固定映射：

| 论文 GLB scene-graph 概念 | URDF 等价映射 |
|---|---|
| scene node | 命名 URDF link |
| flat depth | 1 |
| semantic depth | 根 link 计 1 的最长命名 link root-to-leaf 路径；不额外插入 joint 层 |
| unnamed wrapper collapse | URDF link 强制命名，因此不适用 |
| named group | 有 child 且没有 visual geometry 的命名 link |
| pivot | 命名的 non-fixed URDF joint |
| has tree | valid tree 且 semantic depth > 1 |

输出：

- `exp/runtime/nano3d_hierarchy/selected_asset_records.json`：33 个选定资产的逐资产 tree/depth/group/pivot/edge 数据；
- `exp/runtime/nano3d_hierarchy/cross_seed_records.json`：33 cohort 的 sibling-seed raw edge signatures；
- `exp/runtime/nano3d_hierarchy/summary.json`：论文主指标、URDF 映射和 unsupported 指标声明。

本轮结果：33/33 valid tree、33/33 has tree；semantic depth 均值 3.121（范围 2–6）；named groups 均值 0.061（总计 2）；pivots 均值 5.636（总计 186）。23 个 multi-seed cohort/261 seeds 的 raw edge-signature mode rate 均值为 0.279，pairwise exact rate 均值为 0.144；这两个值没有区分合法的可变 topology，只能作为 supplementary strict-identity proxy，不能写成正式 Cross-Seed Consistency。

论文公开仓库当前只发布客户端和集成，generation backend 及论文原生 GLB evaluator 未公开。因此本 harness 复现论文公开的指标定义，但不声称逐行复用了论文 evaluator 实现。

## 5. `run_nano3d_naming.py`

路径：[`/mnt/zsn/lyb/arti-skill/exp/scripts/run_nano3d_naming.py`](/mnt/zsn/lyb/arti-skill/exp/scripts/run_nano3d_naming.py)

用途：补测 `Nano3dresults.md` 的 Table 2 Naming。该 harness 只读取 `exp` 内的 33 个 URDF 快照和复制后的 authoring/spec 文档，不写入 `seed_exports`、`seed_exports_physics_10`、`arti-template` 或其他 exp 外目录。

### 5.1 输入与语义依据

- URDF 输入快照：`exp/runtime/nano3d_naming/input_urdf/`，共 33 个。
- sibling-seed 冻结快照：`exp/runtime/nano3d_naming/cross_seed_input_urdf/`，共 271 个；正式跨 seed 统计使用其中 23 个 multi-seed cohort/261 seeds。
- 语义来源副本：`exp/reference/naming_sources/`，由 `specs_modular_v1/`、`designs/`、`source_maps/` 和 `picture_source_maps/` 复制得到。
- 冻结角色标注：[`naming_gold_v2.json`](/mnt/zsn/lyb/arti-skill/exp/reference/naming_gold_v2.json)。
- 公式、bootstrap 与 judge contract：[`naming_protocol_v2.json`](/mnt/zsn/lyb/arti-skill/exp/reference/naming_protocol_v2.json)。

该 gold 是 source-derived core-role gold，不是独立隐藏标注。`Stationary_Pencil_sharpener__seed_6` 只有 output-derived fallback，因此只参与直接 Parts/Named gate，不参与 N=32 的 source-semantic 聚合。Parts/Nameability 是直接结果；Recall、Richness candidate、Functional Core Coverage、Instance 和跨 seed role-count Jaccard 均标为 proxy。Semantic Precision 和 semantic-judge Recall 在三独立 judge 完成前保持 N/A。

### 5.2 运行命令

```bash
cd /mnt/zsn/lyb/arti-skill
python /mnt/zsn/lyb/arti-skill/exp/scripts/run_nano3d_naming.py
```

### 5.3 评测流程

1. 读取冻结的 33 个 URDF 快照；只有 `<visual><geometry>` 能解析出 primitive/mesh geometry 的 link 进入论文 Parts gate，避免空 visual tag 被误计为部件，得到 mesh-bearing URDF link/asset 的 GLB-node proxy。
2. 在 mesh-bearing links 上用占位名规则计算 Nameability；`link_0`、`part_01`、`mesh_003` 等不计为语义名称。
3. 将每个 role 的 `min_count` 展开为 required spec instances，在 named links 与 required instances 之间执行最大匹配数优先、实际命中 pattern specificity 次优先的一对一全局匹配；equal-score link 以名称字典序决胜，避免 URDF XML/glTF node 顺序影响 assignment。
4. 按 asset 计算 count-aware source-role Recall、Functional Core Coverage，以及论文方向的 `named mesh links / required spec instances` Richness candidate。
5. 对 149 个 assignment 审计 exact tie、runner-up specificity margin 和 evidence strength；另算 strong-match sensitivity，只接受 canonical role tokens 或至少两 token 的实际命中 pattern，用来量化 single-token aliases 对 permissive Recall 的影响。
6. 对要求多实例的 roles 提取数字、左右/前后、needle/lead 等实例 key，按 required instance 数加权计算 Instance Discriminability，不再对 18 个 groups 做简单布尔等权平均。
7. 所有没有被 required role assignment 使用的命名节点都进入 `judge_queue.jsonl`，作为 extra-real-part candidates；它们不会自动成为 semantic false positives。
8. 对冻结的 sibling-seed URDF 计算 raw name、role-set 和 count-capped role multiset Jaccard；同时报告 all-pairs micro 与 equal-cohort macro/median/bootstrap CI，避免 seed 较多的 cohort 隐性加权；10 个 single-seed physics_10 cohorts 排除。
9. 对 Parts、Richness candidate、source-role Recall 和 cohort-macro consistency 做 10,000 次 bootstrap；seed 和公式写入 protocol 文件。
10. source-proxy 主流程不计算 Over-Segmentation Rate，因为没有功能部件到 URDF link 的一对多分解 gold，而且该指标不是论文 Naming 表指标；v2.3 三-judge 聚合器可在人工填写 same-part 字段后计算本地扩展指标。

### 5.4 输出文件

| 文件 | 内容 |
|---|---|
| `runtime/nano3d_naming/asset_records.json` | 33 个资产的 link/visual、全局角色匹配、source recall、richness/functional/instance proxy |
| `runtime/nano3d_naming/cross_seed_records.json` | 23 个 multi-seed cohort 的 raw/role-set/role-count 一致性 |
| `runtime/nano3d_naming/summary.json` | Table 2 汇总指标、分母和限制 |
| `runtime/nano3d_naming/report.md` | 可人工核对的 Naming 报告 |
| `runtime/nano3d_naming/judge_queue.jsonl` | 233 个待独立 semantic judge 的 GLB mesh-node 等价节点；空 hierarchy nodes 已排除 |
| `runtime/nano3d_naming/input_manifest.json` | gold、protocol、harness、33+271 URDF 输入的 SHA-256 lineage |

### 5.5 本轮结果映射

- Parts：239 个 mesh-bearing links，7.242/asset，asset bootstrap 95% CI [5.667, 8.970]；是 GLB mesh-node proxy。
- Named / Nameability：239/239 = 1.000。
- Paper-aligned Richness candidate：asset-macro=1.482，95% CI [1.279, 1.709]；pooled micro=233/149=1.564；N=32 source-semantic assets。
- Semantic Precision：N/A；等待三独立 judge。旧值 0.934 已撤回，因为 86 个未占用 core-role 的名称是 extra-real-part candidates，而非自动 false positives。
- count-aware source-role Recall：permissive macro=0.994、micro=147/149=0.987；147 assignments 无 exact tie、2 个 margin≤5。
- Strong-match sensitivity：120/149=0.805 micro，0.832 asset-macro；27 个 required slots 依赖冻结的 single-token alias。该值是规则敏感性下界，不是另一套 semantic gold。
- Functional Core Coverage：macro=0.993、micro=120/122=0.984。
- Instance Discriminability：38/40=0.950，覆盖 18 个 repeated-role groups。
- mesh-only raw link-name Jaccard：pair-micro=0.580；equal-cohort macro=0.542，95% CI [0.428, 0.655]，cohort median=0.529。
- role-count weighted Jaccard proxy：pair-micro=0.948；equal-cohort macro=0.952，95% CI [0.921, 0.979]，cohort median=1.000；role-count signature mode rate=0.875。
- Over-Segmentation：N/A。

### 5.6 结论边界

本 harness 证明的是 mesh-bearing part handles、名称可解析性、source-derived required-role 覆盖和 sibling seed 的词法/角色计数稳定性；不证明名称对应的 mesh 区域、独立 hidden-gold semantic precision/recall 或 over-segmentation。

### 5.7 三独立 judge 聚合

固定聚合脚本：[`aggregate_nano3d_naming_judges.py`](/mnt/zsn/lyb/arti-skill/exp/scripts/aggregate_nano3d_naming_judges.py)。三个 judge 必须分别复制完整 `judge_queue.jsonl`，在独立上下文中填写：

- `judge_verdict`：`spec_match`、`extra_real_part`、`invalid_or_hallucinated` 或 `uncertain`；
- `judge_matched_role`：仅 `spec_match` 必填；
- `judge_instance_id`：匹配 role 的 `min_count>1` 时，填写从名称和预览可区分的实例身份；同一实例的碎片使用同一 ID，其他非 uncertain verdict 填 `not_applicable`；
- `judge_same_semantic_part_as`：真实节点必须显式填 `none`，或填同资产另一节点名表示属于同一语义部件；invalid 填 `not_applicable`，空值只表示未标注；
- `judge_reason`：必填短证据。

运行示例：

```bash
python /mnt/zsn/lyb/arti-skill/exp/scripts/aggregate_nano3d_naming_judges.py \
  --judge /mnt/zsn/lyb/arti-skill/exp/runtime/nano3d_naming/judges/judge_a.jsonl \
  --judge /mnt/zsn/lyb/arti-skill/exp/runtime/nano3d_naming/judges/judge_b.jsonl \
  --judge /mnt/zsn/lyb/arti-skill/exp/runtime/nano3d_naming/judges/judge_c.jsonl
```

至少两个相同的非 `uncertain` verdict 才形成 consensus；role、instance ID、same-part target 也分别要求至少两个相同值。聚合器 v2.3 报告 Semantic Precision、macro/micro Semantic Recall、judge-validated Richness、Functional Core Coverage、Instance Discriminability、Over-Segmentation 和各字段 coverage。每个指标有独立 readiness gate：verdict 完整不代表 role/instance/same-part 完整；缺少必要字段时正式值保持 `null`，不会把漏标当作错误或 `none`。

### 5.8 独立复现与逐字节核验

复现脚本：[`reproduce_nano3d_naming.py`](/mnt/zsn/lyb/arti-skill/exp/scripts/reproduce_nano3d_naming.py)。它调用同一冻结 harness，但写入独立目录，然后比较 6 个正式产物的 SHA-256：

```bash
cd /mnt/zsn/lyb/arti-skill
python /mnt/zsn/lyb/arti-skill/exp/scripts/reproduce_nano3d_naming.py
```

本轮独立输出为 `exp/runtime/nano3d_naming_repro_v22/`；`summary.json`、`asset_records.json`、`cross_seed_records.json`、`input_manifest.json`、`judge_queue.jsonl` 和 `report.md` 全部逐字节一致，结果记录在 [`reproduction_check.json`](/mnt/zsn/lyb/arti-skill/exp/runtime/nano3d_naming_repro_v22/reproduction_check.json)，`all_files_match=true`。

## 6. `run_nano3d_urdf_glb_pilot.py`

路径：[`/mnt/zsn/lyb/arti-skill/exp/scripts/run_nano3d_urdf_glb_pilot.py`](/mnt/zsn/lyb/arti-skill/exp/scripts/run_nano3d_urdf_glb_pilot.py)

用途：验证 URDF 资产转换到 GLB 评测表示时，link 名称、mesh-bearing link 集合、父子层级、空运动学 link 和 world-space bounds 是否保留。脚本默认运行 4-asset pilot，也可通过参数对复制到 `exp` 内的全部 33 个完整 package 运行；不修改 `exp` 外原文件。

### 6.1 代表资产

| 资产 | 选择目的 |
|---|---|
| `Astronomy_Antenna_dish__seed_343` | 多 OBJ 与 primitive、单链式 articulation |
| `Door_Double_Door__seed_222` | 简单三 link、双分支层级 |
| `Vehicle_Sports_car__seed_202` | 10 links，含 2 个无 visual 的运动学 link |
| `pictureX_0611_Industrial_rolling_work_table__seed_0` | 17 links、高部件数和多级 caster 子树 |

### 6.2 运行命令

```bash
cd /mnt/zsn/lyb
python /mnt/zsn/lyb/arti-skill/exp/scripts/run_nano3d_urdf_glb_pilot.py
```

完整 N=33 运行：

```bash
cd /mnt/zsn/lyb
python /mnt/zsn/lyb/arti-skill/exp/scripts/run_nano3d_urdf_glb_pilot.py \
  --input-root /mnt/zsn/lyb/arti-skill/exp/runtime/nano3d_glb_n33/input_packages \
  --output-root /mnt/zsn/lyb/arti-skill/exp/runtime/nano3d_glb_n33/output \
  --prepare-from-manifest /mnt/zsn/lyb/arti-skill/exp/runtime/nano3d_asset_pilot/asset_manifest.jsonl
```

`--prepare-from-manifest` 只读 manifest 指向的 source package；首次运行将完整 package 复制到指定的 exp-local input root，并验证 source/copied URDF SHA-256。若已存在的副本 hash 不一致，脚本直接失败，不覆盖副本。

### 6.3 固定转换规则与 gate

1. 解析 URDF 的 box、cylinder、sphere 和相对路径 mesh，并应用 mesh scale 与 visual origin。
2. 同一 link 的所有 visuals 合并为一个 mesh；GLB mesh node 使用对应 URDF link 的原名。
3. 无 visual 的 URDF link 导出为同名空 glTF node，避免运动学层级在 GLB 中消失。
4. 在关节零位应用 joint origin，保留所有 URDF parent-child link edge；只增加一个已知 wrapper `__urdf_world__`。
5. 用 `pygltflib` 重新读取 GLB，检查 link node 无缺失、无未知额外节点、mesh-node 集合精确相等、层级 edge 全保留。
6. 显式累计 URDF joint-origin world matrices，计算转换前 world-space bounds；用 `trimesh` 再次加载 GLB 并比较转换后 bounds（`rtol=1e-5, atol=1e-6`）。
7. 每个资产独立导出两次并比较 SHA-256，检查字节级确定性。

### 6.4 首轮结果

- 4/4 资产通过全部 gate。
- 33/33 URDF links 被保留为同名 GLB nodes；其中 31/31 mesh-bearing links 被保留为同名 mesh nodes。
- Sports car 的 2 个空运动学 links 仍以空 nodes 存在。
- 304/304 source visuals 成功加载并进入对应 link mesh。
- 全部 URDF link-to-link hierarchy edges 保留，无未知额外节点。
- 4/4 重复导出的 GLB SHA-256 完全一致。

输出：

- `exp/runtime/nano3d_glb_pilot/output/summary.json`
- `exp/runtime/nano3d_glb_pilot/output/records.json`
- `exp/runtime/nano3d_glb_pilot/output/report.md`
- `exp/runtime/nano3d_glb_pilot/output/<asset_id>/model.glb`

### 6.5 完整 N=33 结果

- 33/33 资产通过全部 gate。
- 241/241 URDF links 对应同名 GLB nodes；239/239 mesh-bearing links 对应同名 GLB mesh nodes。
- 239/239 GLB mesh nodes 通过与 Naming harness 相同的 placeholder-name gate，Nameability=1.000。
- 2 个空运动学 links 保留为空 nodes。
- 1442/1442 source visuals 成功加载。
- 208/208 link-to-link hierarchy edges 保留。
- 33/33 world-space bounds 保留。
- 33/33 重复导出的 GLB SHA-256 一致。

完整输出：

- `exp/runtime/nano3d_glb_n33/input_packages/input_manifest.json`
- `exp/runtime/nano3d_glb_n33/output/summary.json`
- `exp/runtime/nano3d_glb_n33/output/records.json`
- `exp/runtime/nano3d_glb_n33/output/report.md`
- `exp/runtime/nano3d_glb_n33/output/<asset_id>/model.glb`

### 6.6 结论边界

N=33 结果证明本地确定性 converter 能把完整评测集稳定落到 GLB scene graph，因此 Table 2 的 Parts/Named 可改为直接 GLB mesh-node 统计。它仍不能证明与论文未公开 converter 完全同实现：本 harness 把每个 link 内的多个 visuals 合并，因此 Parts 取决于 link 粒度；也没有执行论文 shared headless scene 的尺度归一化、相机、灯光、渲染、semantic judge、point-mask IoU、材质保真或 articulation sidecar 转换。

## 7. `verify_nano3d_naming_on_glb.py`

路径：[`/mnt/zsn/lyb/arti-skill/exp/scripts/verify_nano3d_naming_on_glb.py`](/mnt/zsn/lyb/arti-skill/exp/scripts/verify_nano3d_naming_on_glb.py)

用途：不再通过 URDF→GLB 名称保持关系间接推断 Table 2，而是直接打开 N=33 `model.glb`，读取 link/mesh node names，用冻结的 `naming_gold_v2.json`、`naming_protocol_v2.json` 和同一个 role matcher 重算 direct/source-semantic Naming，并逐资产、逐汇总字段对照正式 Naming 结果。

运行：

```bash
cd /mnt/zsn/lyb/arti-skill
python /mnt/zsn/lyb/arti-skill/exp/scripts/verify_nano3d_naming_on_glb.py
```

核验字段：Parts、Named/Nameability、required spec/matched counts、source-role Recall、strong-match sensitivity、Functional Core Coverage、Richness candidate、Instance Discriminability、required-role assignment 和 extra-real-part candidates。

结果：33/33 资产逐字段一致，全部汇总字段一致。首次运行时 4 个资产的总指标相同但 assignment 明细不同，原因是同分候选受 artifact node order 影响；`assign_required_roles` 现先按 link/node 名称字典序冻结输入顺序，protocol 同步记录 tie-breaker。修复后 URDF 与 GLB 逐资产 assignment 完全一致，正式汇总数值不变，Naming 独立复现仍为 6/6 文件逐字节一致。

输出：

- [`naming_verification.json`](/mnt/zsn/lyb/arti-skill/exp/runtime/nano3d_glb_n33/naming_verification.json)
- [`naming_verification.md`](/mnt/zsn/lyb/arti-skill/exp/runtime/nano3d_glb_n33/naming_verification.md)

范围：只重算选定 N=33 的 direct/source-semantic 指标。23 个 multi-seed cohort/261 seeds 的 Cross-Seed Consistency 仍来自冻结 URDF snapshots，未在本轮批量转换 sibling GLB；三 judge Semantic Precision/Recall 和 Over-Segmentation 仍未完成。

## 8. `build_nano3d_naming_judge_packet.py`

路径：[`/mnt/zsn/lyb/arti-skill/exp/scripts/build_nano3d_naming_judge_packet.py`](/mnt/zsn/lyb/arti-skill/exp/scripts/build_nano3d_naming_judge_packet.py)

用途：为 Naming v2.2 的三个独立 semantic judges 冻结 v1.1 GLB mesh-node 任务、盲评字段、几何预览、输入副本和 lineage hashes。自动 assignment/candidates 只写入 audit packet，不泄漏到 blind packet。

运行：

```bash
cd /mnt/zsn/lyb
python /mnt/zsn/lyb/arti-skill/exp/scripts/build_nano3d_naming_judge_packet.py
```

只更新任务 schema、复用并核验已有确定性预览时可加 `--skip-previews`。脚本会拒绝覆盖任何已经含 verdict 的 judge 文件。

流程：

1. 读取 mesh-only `runtime/nano3d_naming/judge_queue.jsonl`，强制 N=32、233 tasks、所有 item `has_visual=true`。
2. 直接打开 N=33 转换 GLB，逐资产核验 GLB mesh-node set 与 Naming mesh-bearing nodes 精确一致。
3. 对每个 mesh node 生成一张 1120×362 PNG：完整资产中目标红色高亮，以及目标节点的 iso/front/side 三个隔离视角。
4. 几何预览使用固定 seed=1729 的面积均匀表面采样和软件 point-splat rasterizer，不依赖 GPU/display；重复全量构建的 233 个 preview hashes 完全一致。
5. 写出 `blind_tasks.jsonl`，只包含输入、节点名称、参考角色和待填写字段；写出 `audit_tasks.jsonl` 保存自动 assignment、candidate 与 source evidence，明确禁止交给独立 judge。
6. 为 judge A/B/C 生成三份独立空白模板。schema v1.1 明确区分 `null`、`none` 与 `not_applicable`，并把 repeated-role instance ID 和 same-semantic-part 填写规则写入每个任务；聚合时严格校验。
7. 为 33 个类别生成输出盲 core-taxonomy 模板。33/33 有类别文本；16 个 pictureX 类别从只读 `articraft_data/picture/0611` 复制 46 张原始类别参考图到 packet 内。由于没有逐 seed 精确生成请求，optional/per-seed gold 仍为 0/33 ready。

输出：

- [`manifest.json`](/mnt/zsn/lyb/arti-skill/exp/runtime/nano3d_naming_judge_packet_v1/manifest.json)
- [`blind_tasks.jsonl`](/mnt/zsn/lyb/arti-skill/exp/runtime/nano3d_naming_judge_packet_v1/blind_tasks.jsonl)
- [`audit_tasks.jsonl`](/mnt/zsn/lyb/arti-skill/exp/runtime/nano3d_naming_judge_packet_v1/audit_tasks.jsonl)
- [`independent_gold_annotation_template.jsonl`](/mnt/zsn/lyb/arti-skill/exp/runtime/nano3d_naming_judge_packet_v1/independent_gold_annotation_template.jsonl)
- `runtime/nano3d_naming_judge_packet_v1/previews/`：233 张预览；
- `runtime/nano3d_naming_judge_packet_v1/judges/`：三份待填写 verdict 模板；
- `runtime/nano3d_naming_judge_packet_v1/benchmark_inputs/`：exp-local 原始类别参考图副本。

聚合器 [`aggregate_nano3d_naming_judges.py`](/mnt/zsn/lyb/arti-skill/exp/scripts/aggregate_nano3d_naming_judges.py) 已升级到 v2.3。它分别检查 verdict、role、instance 和 same-part 的 consensus coverage，并支持 Functional、Instance 和 Over-Segmentation。真实聚合命令：

```bash
python /mnt/zsn/lyb/arti-skill/exp/scripts/aggregate_nano3d_naming_judges.py \
  --queue /mnt/zsn/lyb/arti-skill/exp/runtime/nano3d_naming_judge_packet_v1/blind_tasks.jsonl \
  --judge /mnt/zsn/lyb/arti-skill/exp/runtime/nano3d_naming_judge_packet_v1/judges/judge_a.jsonl \
  --judge /mnt/zsn/lyb/arti-skill/exp/runtime/nano3d_naming_judge_packet_v1/judges/judge_b.jsonl \
  --judge /mnt/zsn/lyb/arti-skill/exp/runtime/nano3d_naming_judge_packet_v1/judges/judge_c.jsonl \
  --output /mnt/zsn/lyb/arti-skill/exp/runtime/nano3d_naming_judge_packet_v1/judge_consensus.json
```

当前 blocker：三份 verdict 尚未由独立上下文填写；core-taxonomy 模板也尚未由输出盲 annotators 完成。因此 Table 2 的 Semantic Precision、judge Recall 和 Over-Segmentation 仍保持 N/A。

公式 smoke 固定脚本：[`build_nano3d_naming_judge_smoke.py`](/mnt/zsn/lyb/arti-skill/exp/scripts/build_nano3d_naming_judge_smoke.py)。运行：

```bash
python /mnt/zsn/lyb/arti-skill/exp/scripts/build_nano3d_naming_judge_smoke.py
```

[`smoke_v23/manifest.json`](/mnt/zsn/lyb/arti-skill/exp/runtime/nano3d_naming_judge_packet_v1/smoke_v23/manifest.json) 记录了三条测试：

- 233 项 all-uncertain：coverage=0，Precision、Recall、Functional、Instance、Over-Segmentation 正式值全部为 `null`；
- 233 项 source-assignment synthetic：coverage=1，公式精确回到 Recall 147/149、Functional 120/122、Instance 38/40；synthetic Precision=1、Over-Segmentation=0 只用于公式测试；
- 人工指定一个 same-part fragment pair：excess fragments=1，micro Over-Segmentation=`1/233=0.0042918455`；另验证真实节点漏填 same-part 会被 schema 拒绝。

这三组都是机械构造的单元测试，不是三个独立语义 judge，任何 synthetic 输出都不得回填 Table 2。

## 7. `run_nano3d_seed_reliability.py`

路径：[`/mnt/zsn/lyb/arti-skill/exp/scripts/run_nano3d_seed_reliability.py`](/mnt/zsn/lyb/arti-skill/exp/scripts/run_nano3d_seed_reliability.py)

用途：对 `Nano3dasset.md` 对应的 33 个冻结既有模板运行固定 seeds 0–35。共有的 Executable、Artifact Saved、First-shot、Final Success、Repair 和耗时进入 Table 1A；Seed Compile、Seed Full QC、36/36 Pass 等 seed 专属指标进入 Table 1B。模板仓库只读；manifest、临时目录、日志和产物全部位于 `exp/runtime/nano3d_seed_reliability/`。

### 7.1 正式运行命令

```bash
cd /mnt/zsn/lyb
/mnt/zsn/lyb/arti-skill/arti-template/.venv/bin/python \
  /mnt/zsn/lyb/arti-skill/exp/scripts/run_nano3d_seed_reliability.py \
  --workers 8 \
  --timeout 180 \
  --force
```

不带 `--force` 时按 `<runtime>/templates/<slug>/outcomes.json` 断点续跑。可重复提供 `--slug <template_slug>` 只跑 cohort 内指定模板；`--out` 被强制限制在 `exp` 内。

### 7.2 固定协议

1. 从 `Nano3dasset.md` 的绝对资产链接提取 33 个唯一 slug，并在只读 `TEMPLATE_REGISTRY` 中解析 function stem。
2. manifest 记录资产清单 hash、模板文件 SHA-256、selected export lineage 和固定 seeds 0–35。
3. 每 seed 调用项目原生 `template_sweep` strict gate：`target=full`、`run_checks=True`、`motion_qc=True`。
4. Full QC 包括作者测试、compiler baseline、overlap policy、rigid-seed policy、disconnected-geometry policy 和实际 motion coverage。
5. strict pass 时保存 self-contained `model.py`、`model.urdf`、mesh assets、含 config/URDF hash 的 `artifact.json`，并校验 artifact hash。
6. strict fail 时额外运行 `run_checks=False`、`motion_qc=False` 的 compile-only probe，用于区分“不能生成 URDF/mesh”和“生成成功但 QC 失败”；正式全通过运行没有触发 fallback probe。
7. 每个严格结果记录 resolved config、全部 failure signals、allowances、motion-QC report、artifact path 和 elapsed time。
8. 正式复跑固定 8 个外层 workers，并将 OpenBLAS/OMP/MKL/NumExpr/VecLib 内层线程数设为 1，避免嵌套并发造成 `pthread_create` 假失败；每 seed 180 s hard timeout。

### 7.3 输出

| 文件/目录 | 内容 |
|---|---|
| `manifest.json` | 33-template/36-seed 协议、输入 hash、模板 hash、运行环境约束 |
| `templates/<slug>/outcomes.json` | 每模板 36 条 compile/QC/config/artifact 记录和运行配置 |
| `records.json` | 1,188 条扁平逐 seed 记录 |
| `summary.json` | Table 1A/1B 聚合、逐模板 36/36 和耗时分布 |
| `qc_artifacts/<slug>/seed_<n>/` | strict Full-QC 通过的 self-contained seed package |
| `compile_artifacts/` | 仅 strict fail 但 compile-only pass 时使用；正式接受运行未新增此类记录 |
| `diagnostics/exploratory_compile_artifacts/` | 过度并发探索运行留下的 56 个 compile-only 诊断包；不被最终 `records.json`/`summary.json` 引用 |

### 7.4 正式结果

- Template cohort：33 个冻结既有模板。
- Seeds：每模板 0–35，共 1,188。
- Seed Compile：1,188/1,188 = 100%。
- Seed Full QC：1,188/1,188 = 100%。
- Artifact Saved：1,188/1,188 = 100%。
- 36/36 Pass：33/33 templates = 100%。
- strict seed elapsed：mean 9.58 s、median 5.72 s、p95 42.57 s。
- 正式 cohort wall：1,794.88 s。

### 7.5 重要诊断与结论边界

探索运行曾使用 16 workers，触发 OpenBLAS/FCL 的线程资源耗尽，并形成 `pthread_create`、subprocess crash 和 timeout 假阴性。将数值线程固定为 1 后，先以低并发复测受影响模板，再以统一 8-worker 协议完整复跑 1,188 seeds，最终全部通过；探索错误不计入正式 failure taxonomy。

本 harness 证明的是 33 个**既有模板**在固定 seed 分布上的可编译性和 Full-QC 可靠性。Table 1A 因此可在 `existing template → generated seed asset` evaluation unit 下报告 First-shot=100%、Final Success=100%、repair=0；Table 1B 报告 Seed Compile=100%、Seed Full QC=100%、36/36=33/33。项目原生 corner 由下节的独立 harness 测量；本 harness 没有从 text/image task 新生成模板，也没有 post-repair regression、agent turns、token 或 API cost，且不可把 seed evaluation unit 解释为 task-to-template authoring 难度。

## 8. `run_nano3d_corner.py`

路径：[`/mnt/zsn/lyb/arti-skill/exp/scripts/run_nano3d_corner.py`](/mnt/zsn/lyb/arti-skill/exp/scripts/run_nano3d_corner.py)

用途：对 Table 1B 的 33 个冻结既有模板运行项目原生 Corner protocol，并以与 seed reliability 相同的 strict compile/Full-QC/artifact 校验口径统计 Corner Pass。模板仓库保持只读，缓存、临时目录、日志和产物全部位于 `exp/runtime/nano3d_corner/`。

### 8.1 正式运行命令

```bash
cd /mnt/zsn/lyb
/mnt/zsn/lyb/arti-skill/arti-template/.venv/bin/python \
  /mnt/zsn/lyb/arti-skill/exp/scripts/run_nano3d_corner.py \
  --workers 8 \
  --timeout 180 \
  --force
```

不带 `--force` 时复用逐模板结果断点续跑；可重复提供 `--slug <template_slug>` 限定 cohort。输出路径被限制在 `exp` 内，运行时 `TMPDIR` 也固定在 `exp/runtime/nano3d_corner/tmp/`。

### 8.2 Corner 选择与判定协议

1. 对 22 个声明 `TEMPLATE_DOMAIN` 的模板，运行项目中显式定义的 `TEMPLATE_CORNERS`，并检查 domain candidate/edge values 是否已被 seeds 0–35 覆盖；本 cohort 得到 99 个显式 domain cases，无需补充遗漏值。
2. 对 11 个 legacy 模板，调用项目原生 `select_corner_seeds`：扫描 seeds 0–511，根据数值极值以及未覆盖的 categorical/slot 组合，贪心选择每模板最多 12 个、且不与基础 seeds 0–35 重复的真实可达 seeds；本 cohort 得到 132 cases。
3. 每 case 运行作者测试、compiler checks、overlap/disconnected policy 与 motion QC；harness 进一步验证 artifact hash，并拒绝 rigid URDF。
4. case pass 要求 strict Full QC 和 artifact 校验均通过；template strict pass 要求该模板所有选中 corner cases 全部通过。
5. 为避免重复计算，harness 将已成功的 random-36 outcomes 复制到 exp-local 项目缓存，仅用于项目原生 corner selector 的基础覆盖判定；不会修改项目源文件或原 seed 结果。

### 8.3 输出

| 文件/目录 | 内容 |
|---|---|
| `manifest.json` | cohort、corner selector、模板/input hash 和运行参数 |
| `templates/<slug>/outcomes.json` | 每模板的 corner mode、选中 cases、strict/native 状态和逐 case 记录 |
| `records.json` | 231 条扁平逐 corner case 记录 |
| `summary.json` | Table 1B 聚合、逐模板结果和耗时分布 |
| `artifacts/<slug>/...` | 通过 strict QC 的 self-contained corner packages |
| `tmp/` | exp-local 临时目录和项目缓存 |

### 8.4 正式结果与边界

- Template cohort：33 个冻结既有模板。
- Domain templates：22 个，共 99 个显式 domain corner cases。
- Legacy templates：11 个，共 132 个可达 corner seeds（12/template）。
- Corner Pass：231/231 cases = 100%。
- Strict all-corner template pass：33/33 = 100%。
- 项目原生 corner gate：33/33 = 100%。
- Artifact hash/non-rigid 校验失败：0。
- case elapsed mean/median/p95：10.59/5.88/46.23 s；harness wall：1,035.40 s。

该结果是 `nano3d_project_native_corner_v1`，衡量当前项目自身定义或选择出的可达 corner cases。它不等同于 `Nano3d.md` 草案中的“每模板固定 4 个 continuous corners”协议：模板的 corner 数量并非统一为 4，legacy cases 也是由 seed 搜索得到。因此 Table 1B 必须以脚注保留这一口径差异，不能把 231/231 解释成尚未建立的 fixed-4 benchmark 结果。

## 9. `run_nano3d_constraints.py`

路径：[`/mnt/zsn/lyb/arti-skill/exp/scripts/run_nano3d_constraints.py`](/mnt/zsn/lyb/arti-skill/exp/scripts/run_nano3d_constraints.py)

用途：在不修改模板仓库的情况下，对 seed reliability 已冻结的 33×36 个最终 package 执行 source-derived/operational Constraints v1。协议见 [`constraints_protocol_v1.json`](/mnt/zsn/lyb/arti-skill/exp/reference/constraints_protocol_v1.json)。

### 9.1 运行命令

```bash
cd /mnt/zsn/lyb/arti-skill
python exp/scripts/run_nano3d_constraints.py
```

### 9.2 判定协议

1. Count：仅对带结构化 design JSON 的资产，根据 resolved config 激活适用 `category_anchors`，将 `exact_count`/`count_from` 转成 required-role lower-bound，并在最终 visual-bearing URDF links 上做一对一匹配。
2. Numeric：逐 movable joint 检查 finite/non-zero axis；revolute/prismatic 还要求 finite 且有序的 lower/upper limits。
3. Relational：检查适用 anchor 声明的 parent-role、child-role、joint type 和 axis 是否出现在最终 URDF。
4. Interface：逐 seed 要求 connected URDF tree、mesh references 可解析、strict compiler policy gate 通过；这是 operational interface-integrity proxy。
5. Kinematic：逐 seed 要求 motion-QC `collision_free=true`、无 missing edge/pose，且每个 reported joint 覆盖 required edges。
6. Compatibility：逐 seed 检查 resolved valid config 通过 strict Full-QC，只评 valid-combination acceptance。
7. v1 不生成非法配置；没有 frozen negative manifest，因此 Invalid Combination Rejection 保持 N/A。

### 9.3 输出与正式结果

| 文件 | 内容 |
|---|---|
| `runtime/nano3d_constraints_v1/records.json` | 1,188 seed assets 的逐 clause 期望、观测、证据与 pass/fail |
| `runtime/nano3d_constraints_v1/summary.json` | Table 4 汇总、分类分母、逐模板 36/36 |
| `runtime/nano3d_constraints_v1/report.md` | 可读结果摘要与口径限制 |

- Coverage：17,706/17,706 = 1.000。
- Satisfaction / Conditional Accuracy：17,125/17,706 = 0.967。
- Count：3,345/3,620 = 0.924。
- Numeric：6,402/6,402 = 1.000。
- Relational：3,814/4,120 = 0.926。
- Interface、Kinematic、valid-config Compatibility：均 1,188/1,188 = 1.000。
- All-Pass seed assets：960/1,188 = 0.808；36/36 all-pass templates：25/33。
- 连续两次运行的 `records.json`、`summary.json` 和 `report.md` SHA-256 完全一致。

这些数字必须带 `*`：source/design evidence 不是独立 hidden gold；Count 是 lower-bound；Numeric 不是目标几何尺寸；Interface/Compatibility 部分依赖项目 strict QC。不得将该行与论文 52 条 hidden-spec GLB constraints 当成同一 constraint set 排名。

## 10. `preflight_nano3d_paper_constraints.py`

路径：[`/mnt/zsn/lyb/arti-skill/exp/scripts/preflight_nano3d_paper_constraints.py`](/mnt/zsn/lyb/arti-skill/exp/scripts/preflight_nano3d_paper_constraints.py)

用途：按 Nova3D Section 4.1 / Section 8 / Table 11 对正式 Constraints 输入做 fail-closed 检查，防止从已有输出反推 target/tolerance 后伪称 paper-aligned。

```bash
cd /mnt/zsn/lyb/arti-skill
python exp/scripts/preflight_nano3d_paper_constraints.py
```

冻结协议：[`paper_constraints_protocol_v1.json`](/mnt/zsn/lyb/arti-skill/exp/reference/paper_constraints_protocol_v1.json)。正式运行必须具备：18 个 constrained items 的原始 prompts、18 个预先冻结的 `spec.yaml`、对应最终 GLB，以及每条 semantic anchor/count-or-dimension measure recipe、comparator 和 tolerance；总约束必须为 52，其中 exact count 32、numeric 20。

当前 preflight 结果为 `blocked_missing_paper_benchmark_inputs`：prompt 0/18、spec 0/18、matching paper item GLB 0/18，公开仓库也未提供论文 scorer。输出见 [`report.md`](/mnt/zsn/lyb/arti-skill/exp/runtime/nano3d_paper_constraints_preflight/report.md)。在这些输入补齐前，Table 4A 本地行必须保持 N/A；`run_nano3d_constraints.py` 的 17,706-clause 结果只能作为 supplementary operational audit。

## 10. `run_nano3d_editability.py`

路径：[`/mnt/zsn/lyb/arti-skill/exp/scripts/run_nano3d_editability.py`](/mnt/zsn/lyb/arti-skill/exp/scripts/run_nano3d_editability.py)

用途：在不修改 `arti-template` 和原始 `seed_exports` 的前提下，运行 reduced Editability benchmark。当前冻结 6 个已有程序化模板：Dressing table、bi-fold closet door、juicer press、garden pruner、ergonomic clamp、hand-crank clothes wringer。每个模板运行三类 edit：

- parameter：对一个数值配置字段做正向变化，超过模板合法上限时截到合法上限；
- component：将一个 `TEMPLATE_DOMAIN` component slot 替换为下一个合法值；
- structure：将一个 multiplicity slot 或结构 slot 替换为另一个合法值。

每个 edit 在 seeds `0–15` 上运行，因此共 `6 × 3 × 16 = 288` 个 edited cases，并为 96 个 template/seed 组合保留 baseline package。

### 10.1 正式运行命令

```bash
cd /mnt/zsn/lyb/arti-skill/arti-template
uv run python /mnt/zsn/lyb/arti-skill/exp/scripts/run_nano3d_editability.py \
  --out /mnt/zsn/lyb/arti-skill/exp/runtime/nano3d_editability \
  --workers 8 \
  --timeout 180 \
  --force
```

不带 `--force` 时复用已存在的逐 case `baseline_result.json` 和 `edited_result.json`，只重新聚合。输出目录必须位于 `exp/` 下；每个 compile case 在独立 wrapper 目录中运行，避免 SDK asset session 和 mesh 输出互相污染。数值库线程固定为 1，避免并发造成资源耗尽。

### 10.2 确定性 gate

单个 edited case 的 deterministic proxy final pass 要求：

1. edited wrapper 能执行并保存 `model.urdf` 与 `assets/` artifact；
2. edited URDF 通过单根、无环、可达、边数闭合的 valid-tree 检查；
3. edit field 达到预期的新值，且 package hash 相对 baseline 改变；
4. 模板自身 tests、compiler checks 和 motion-QC 通过。

`Target Fulfilled` 是 config-level contract；`Anchor` 不自动判为通过。`Post-Edit Constraint Pass` 是 template-QC operational proxy，不是独立 hidden `template_spec`。package diff 使用 `model.urdf + assets/**` 的 SHA-256，避免只比较 URDF 文本而漏掉 mesh 内容变化。

### 10.3 locality 与 preservation proxy

- `Geometry Locality`：按 baseline/edited URDF link XML signature 的变化计算 `1 - changed_non_target_links / changed_all_links`；target token 由 manifest 冻结。
- `Structural Locality`：按 parent-child edge 变化计算 non-target edge 未变化比例。
- `Non-Target Preserved`：检查未命中 target token 的共同 link，其 URDF link signature 是否保持一致。

这三项是可重复的 URDF/package proxy；没有语义 anchor gold、几何语义 diff 或人工盲评，不能称为视觉/语义 preservation 或 locality。

### 10.4 输出与本轮结果

| 文件/目录 | 内容 |
|---|---|
| `runtime/nano3d_editability/manifest.json` | 6 个模板、18 个 edit task、seed manifest、edit 字段和 target token |
| `runtime/nano3d_editability/records.json` | 288 个 edited cases 的 baseline/edited run、URDF parse、gate、diff 和错误证据 |
| `runtime/nano3d_editability/summary.json` | Table 5 聚合结果 |
| `runtime/nano3d_editability/report.md` | 结果摘要和 unsupported 边界 |
| `runtime/nano3d_editability/wrappers/` | exp-local baseline/edited wrappers 与 package artifact |

本轮结果：target fulfilled、artifact saved、output changed、valid tree、operational template QC、final deterministic proxy 均为 `288/288 = 100%`；16-seed task propagation 为 `18/18 = 100%`；parameter scale contract 为 `96/96 = 100%`；non-target structural preservation proxy 为 `168/288 = 58.3%`；geometry locality proxy 均值 `0.805`；structural locality proxy 均值 `1.000`；编辑 compile wall time 均值 `3.624 s/edit`。

仍 unsupported：semantic anchor、独立 historical regression preservation、三人 blind review/Fleiss kappa 或 Krippendorff alpha、tokens/API cost，以及完整 Nano3D 18-template/54-edit scope。Table 5 中本地行因此统一带 `*`，并标为 reduced/operational proxy。

## 11. `run_nano3d_paper_editability.py`

路径：[`/mnt/zsn/lyb/arti-skill/exp/scripts/run_nano3d_paper_editability.py`](/mnt/zsn/lyb/arti-skill/exp/scripts/run_nano3d_paper_editability.py)

用途：按 arXiv:2607.22738v1 Section 9 / Tables 13–14 的 Editability 流程，在 exp 目录内运行一个可复现的 18-item local slice。协议冻结为 13 个 additive edits + 5 个 modified-existing edits，每个资产一次 natural edit。资产来自本地可执行 procedural templates，不是论文私有 generated asset IDs。

### 11.1 正式运行命令

```bash
cd /mnt/zsn/lyb/arti-skill/arti-template
uv run python /mnt/zsn/lyb/arti-skill/exp/scripts/run_nano3d_paper_editability.py \
  --out /mnt/zsn/lyb/arti-skill/exp/runtime/nano3d_paper_editability \
  --timeout 180
```

脚本只在 `exp/` 下生成 wrapper、URDF/GLB、preview、manifest 和 report；不修改 `arti-template` 源码、`seed_exports` 或 `seed_exports_physics_10`。GLB 转换依赖本地已补齐的 `pygltflib` 环境包，但使用的是 exp-local URDF/trimesh converter。

### 11.2 与论文一致的确定性阶段

论文的确定性前置 gate 在本 harness 中具体化为：

1. `artifact_valid`：base/edit compile 成功，且两个 GLB artifact 存在；
2. `target_handle`：edited GLB 中存在 manifest 声明的目标 handle/link token；
3. `source_glb_changed`：edit source wrapper 与 base 不同，且 base/edit GLB hash 不同；
4. `hierarchy_preserved`：base/edit 均为 valid tree，且排除目标 token 后 parent-child edge set 不变。

本次真实运行结果：`artifact_valid=18/18`、`target_handle=14/18`、`source_glb_changed=18/18`、`hierarchy_preserved=18/18`、`all_gates=14/18`。4 个未过项是 E001 Bag、E002 bucket2、E003 garlic press 的目标 handle 未在导出 GLB 中显式暴露，以及 E017 clothes wringer 的替换 edit 未暴露 `feed/apron/shelf` 目标 handle；没有用后处理改写 GLB 来“补通过”。

### 11.3 盲审 packet 与未完成指标

脚本生成 18 个 base/edit preview pair 和盲审模板：

| 文件 | 内容 |
|---|---|
| `runtime/nano3d_paper_editability/manifest.json` | 18-item frozen task manifest、edit class、instruction、seed、target tokens |
| `runtime/nano3d_paper_editability/records.json` | 每项四个 deterministic gate、base/edit run、URDF/GLB hash、preview 路径 |
| `runtime/nano3d_paper_editability/summary.json` | deterministic gate 汇总与 human-review null 状态 |
| `runtime/nano3d_paper_editability/report.md` | 可读结果及限制 |
| `runtime/nano3d_paper_editability/blind_review_packet/public_packet.json` | 脱敏后的 reviewer packet |
| `runtime/nano3d_paper_editability/blind_review_packet/private_key.json` | task ID 与本地 case 的私钥映射，不交给 reviewer |
| `runtime/nano3d_paper_editability/blind_review_packet/reviewer_template.json` | 两名 reviewer 的标注字段模板 |

目标是否满足、anchor、scale、non-target preserved、visual/semantic locality、reviewer agreement、adjudicated final pass 等论文结果，必须读取两个独立 reviewer 的 blind labels 并完成 adjudication；当前这些字段保持 null/N/A。matplotlib preview 只是当前无 Blender 时的 deterministic preview，不等同于论文 canonical Blender render。

## 12. `run_nano3d_articulation_paper.py`

路径：[`/mnt/zsn/lyb/arti-skill/exp/scripts/run_nano3d_articulation_paper.py`](/mnt/zsn/lyb/arti-skill/exp/scripts/run_nano3d_articulation_paper.py)

用途：按 Nova3D arXiv:2607.22738v1 Section 10 / Tables 15–16 的指标分层，在冻结的 33 个 existing-export 资产上执行 Articulation paper-aligned functional proxy。脚本只读 `Nano3dasset.md` 链接指向的 source assets，输出全部位于 `exp/runtime/nano3d_articulation_paper/`。

### 12.1 运行命令

```bash
cd /mnt/zsn/lyb/arti-skill
/mnt/zsn/lyb/arti-skill/exp/.venv_low_medium/bin/python \
  /mnt/zsn/lyb/arti-skill/exp/scripts/run_nano3d_articulation_paper.py
```

### 12.2 固定协议

1. Tier 1：统计 articulable asset rate、movable joints/asset 和 native joint exposure。
2. Tier 2：逐 movable joint 检查 type、parent/child、axis、origin、有限上下限字段；按论文的 `revolute span >= 300°` 规则单列 generic range。没有 semantic kinematic gold 时，type accuracy、joint recall、parent-child accuracy 和 axis-on-moving-part 保持 N/A。
3. Tier 3：每个 movable joint 在其声明范围均匀取 11 个状态；continuous joint 用 `[-π, π]` 的 11 点 operational proxy；有多于一个 joint 的资产额外取 64 个 deterministic Sobol 配置。
4. PyBullet 使用 `URDF_USE_SELF_COLLISION_EXCLUDE_PARENT`，固定 base，离散 `stepSimulation()` 后读取 self-contact；这避免直接 parent-child joint interface 的预期接触，但不是 CCD，也不代表摩擦/动力学稳定性。
5. `Joint Geom. Valid` 和 `Asset Geom. Valid` 不把 collision-only pass 伪装成论文完整几何有效性，因为论文还要求 axis-on-moving-part；结果中同时保存 collision-only joint/asset proxy。

### 12.3 输出与结果

| 文件 | 内容 |
|---|---|
| `runtime/nano3d_articulation_paper/summary.json` | 协议、Tier 1/2/3 汇总、限制和分母 |
| `runtime/nano3d_articulation_paper/asset_records.json` | 33 个资产的 joint metadata、逐资产扫掠和资产级 pass |
| `runtime/nano3d_articulation_paper/state_records.jsonl` | rest、2,046 个单关节状态和 1,536 个多关节 Sobol 状态的逐状态记录 |
| `runtime/nano3d_articulation_paper/report.md` | 可读摘要 |

本轮结果：33/33 articulable；186 movable joints，5.636 joints/asset；axis metadata 186/186、origin metadata 186/186、bounded limits 143/143；generic revolute range 12/106，另有 43 continuous joints；2,046/2,046 single-joint states、1,536/1,536 multi-joint states collision-free；joint single-sweep 186/186、asset full-range collision-only proxy 33/33。Type/recall/parent-child/axis semantic、rest-pose frozen、CCD/clearance 和论文意义的 geometric validity 仍是 N/A。
