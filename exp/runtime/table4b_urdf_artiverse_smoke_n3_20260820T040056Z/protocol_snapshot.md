# 10K Sim-Ready URDF Dataset: Automatic Evaluation Protocol

本文档定义一套不依赖配对 GT 或人工评价的自动评测协议，用于比较以下七组铰接资产：

- Ours-500K
- Articraft-10K
- LAM released outputs
- Artiverse
- PartNet-Mobility
- PhysX-Mobility
- SketchMobility

评测覆盖五个方面：数据规模与结构多样性、URDF 合法性、运动学可执行性、碰撞与机械间隙、跨仿真器运行就绪度。目标仿真器为 PyBullet、Genesis 和 MuJoCo。

## 1. Evaluation Protocol

### 1.1 Evaluation units and denominators

- `N_release`：实际获取且纳入冻结 manifest 的发布资产数量。
- `N_eval`：冻结后进入统一评测的资产数量。
- `J_eval`：`N_eval` 中声明的全部非 fixed joints 数量。
- 资产级成功率以 `N_eval` 为分母；关节级成功率以 `J_eval` 为分母。
- 解析失败、资源缺失、转换失败、仿真器崩溃和超时均计为失败，不得从分母中删除。
- 若某方法没有发布可供评测的资产，或许可证/格式使评测无法执行，结果记为 `N/E`，不得用原论文结果代替本地统一评测。
- 比例统一写为 `passed / denominator (percentage)`，并同时报告 overall micro average 和 category-level macro average。

### 1.2 Evaluation cohorts

每张主表分别报告两个 panel：

- **Full Release Cohort**：各方法实际发布且进入冻结 manifest 的全部资产。
- **Shared-category Balanced Cohort**：七组数据共同覆盖的类别；每类使用相同资产数，并以固定随机种子抽样。

Full Release Cohort 衡量发布数据的实际可用性，Shared-category Balanced Cohort 用于减小类别组成和样本规模差异带来的影响。

### 1.3 Frozen configuration

正式运行前冻结并公开：

- 数据版本、下载地址、文件哈希、资产 ID 和排除原因；
- URDF 转换器版本和转换配置；
- joint-state 采样策略和随机种子；
- 合法接触规则、碰撞排除规则和穿透阈值；
- `visual-bearing link` 的资格定义、资产级与 link-micro 分母、解析/资源失败和零分母策略；
- 既有结果所用 rest state，以及新增 clearance 指标统一使用的尺度来源、signed-distance 约定和 measurement coverage 状态；
- 声明 DoF 的冻结计数规则及 `0`、`1`、`2--3`、`4--7`、`>=8` 分箱；无法解析 DoF 的资产进入单独的 `unknown/unparseable` 分箱，不得删除；
- visual/collision 双向表面采样的点数、面积加权策略、随机种子、坐标焊接容差、退化面策略和 mesh triangle 计数规则；
- headline 使用的统一 pair policy 不得包含 method-specific allowlist；任何方法自带 allowance 的注册时间、适用 pair 和敏感性分析只能作为 supplementary evidence；
- PyBullet、Genesis、MuJoCo 的版本、timestep、gravity、solver、控制器和运行时长；
- 每项测试的超时、失败条件和 `N/E` 条件。

### 1.4 Proposed metric amendment status

- 下文新增的 Table 2 supplementary、Table 4a、Table 4b 和 Supplementary Table S1 是协议修订提案。截至本文档本次修订，Table 2 supplementary 已由冻结 evaluator 完成 PartNet-Mobility、LAM released outputs、Artiverse 与 Articraft-10K 的正式运行；其中 PartNet-Mobility 运行与后续运行冻结的 `lam_supplementary_static` atom 模块版本不同（分别为 `1c2fdc2c3d9f8ebcb3ab6b0bf8144b307c86b4b44790cf3182c2395ab37267ff` 与 `04985b5adc97275f940c29bbb584e8f0b6d1dd62cd5ba543d1c71c4a64ae6cc5`，共享模块在两次运行之间被修改），最终跨方法比较前必须对全部方法统一重跑或做版本等价性确认；Table 4a 已由冻结 Genesis contact-penetration oracle 完成 LAM released outputs（含一次文档化恢复定稿）与 PartNet-Mobility 的正式运行，详见该表证据段。Table 4b 的 LAM released outputs 正式结果由同一冻结运行（与 Table 4a 同一 finalized run）的 Table 4b atoms 产出，详见该表证据段；其余方法与 S1 仍无与新增定义完全一致的冻结 evaluator 和正式结果，相应单元格记为 `TBD`。
- 新增指标不追溯改变既有 Table 2 的 `Strict URDF Pass`、Table 4 的 `Strict Collision Pass`、其冻结分母、历史 q=0 状态或已经写入的 N=800 数值。若未来将新增指标纳入 strict pass，必须发布新的协议版本并对全部方法在同一冻结 cohort 上重跑。
- 四个数据根上的每库 N=10 检查仅用于只读 metric reconnaissance。它是 valid-filtered 静态诊断，不是正式随机样本、成功率估计或数据集排名，也不得写回任何 proposed 正式结果单元格。
- 正式填充新增表前，必须冻结 cohort identity、`N_eval`/`J_eval`、pair policy、method-specific allowance 边界、阈值、尺度、surface sampler、默认质量注册表、计时环境、runner 和可重放 receipt。
- 对 LAM released outputs 的后续 Table 4a 专项运行，状态碰撞 oracle 固定为独立版本化的 `Genesis contact-penetration` backend，而非既有 PyBullet Table 4 runner：每个冻结 q-state 只做 direct kinematic detection（不 `scene.step`）；仅当 eligible source-URDF link pair 的 Genesis-reported penetration 严格大于 `1e-6 m` 时判为非法。必须开启 self / neutral / adjacent collision candidate generation，并按原始 URDF 的 direct parent-child graph 手工实施 headline exclusion；不得因 Genesis 默认过滤或 visual fallback 改变 pair policy。
- Genesis contact 不提供所有分离 pair 的完整 exact signed clearance。因此 LAM 的 `Normalized Clearance P5` 不得由 contact count、penetration、SDF 或 AABB 推导；除非另有独立 exact-distance backend 通过冻结资格验证，否则正式单元格记为 `N/E`。Table 4b 的双向 surface P95 继续使用独立的 exact surface backend，不以 Genesis SDF 代替。既有 PyBullet Table 4 数值保持历史证据，不能与这个新 protocol version 混写或覆盖。
- SketchMobility 于 2026-08-20 作为第七组对照资产加入本协议。其 Hugging Face 发布（`Arlo397/SketchMobility`，配套论文 Sketch2Arti，arXiv:2604.25781）已于当日完整下载至 `/mnt/zsn/lyb/arti-skill/exp/SketchMobility`：3 个 `tar.gz` chunk 加 `dataset_chunks/manifest.json`，manifest 声明 `objectCount = 4,956`、`fileCount = 193,872`、`uncompressedBytes = 14,979,180,697`，chunk 级 SHA-256 见该 manifest。截至本次修订，数据尚未解包，未建立冻结 per-asset manifest、cohort identity 或任何 evaluator 运行；因此 SketchMobility 的全部正式结果单元格记为 `TBD`，仅 Table 1 的 `Paper-reported Assets` 按发布 README 记为 4,956。后续对 SketchMobility 的任何正式填充必须满足与其余方法完全相同的冻结要求，且其混合许可（含 GPL-3.0 成分）须在冻结前完成合规确认。

---

## Table 1. Dataset Scale and Structural Diversity

| Dataset / Outputs | Paper-reported Assets | N_release | N_eval | #Categories | Links/Asset | Movable Joints/Asset | Multi-joint Assets (%) ↑ | Unique Topologies (%) ↑ | Exact Duplicate Rate (%) ↓ |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Ours-500K | 10K | 500 | 500 | 12 / 12 | 6.25 / 5 / 13 | 4.93 / 4 / 12 | 92.20 | 13.40 (n=500) | 0.00 (n=500) |
| Articraft-10K | 10,018 | 9,996 | 800 | 240 / 222 | 4.96 / 4 / 8 | 3.58 / 3 / 6 | 79.62 | 39.12 (n=800) | 0.00 (n=800) |
| LAM released outputs | N/R | 3,217 | 800 | 787 / 305 | 6.01 / 4 / 11 | 2.99 / 2 / 5 | 59.88 | 41.47 (n=733) | 1.25 (n=722) |
| Artiverse | 5,402 | 3,544 | 800 | 84 / 67 | 8.59 / 5 / 16 | 4.84 / 2 / 7 | 78.75 | 35.88 (n=797) | 0.00 (n=797) |
| PartNet-Mobility | 2,346 | 2,347 | 800 | 46 / 46 | 7.10 / 4 / 10 | 5.10 / 2 / 8 | 59.13 | 14.62 (n=800) | 0.00 (n=787) |
| PhysX-Mobility | 2,024 | 2,024 | 800 | 132 / 98 | 12.61 / 6 / 18 | 4.76 / 2 / 7 | 55.12 | 23.00 (n=800) | 0.00 (n=800) |
| SketchMobility | 4,956 | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD |

### Table 1 metric definitions

| Metric | Definition |
|---|---|
| `Links/Asset` | 每个资产的 link 数量，报告 mean / median / P90。 |
| `Movable Joints/Asset` | 每个资产中声明的非 `fixed` XML joint 数量，报告 mean / median / P90。该声明层统计包含 exporter extension joint types，不等同于通过严格 URDF/runtime 验证的可执行 DoF 数。 |
| `Multi-joint Assets` | 至少包含两个非 fixed joints 的资产比例。 |
| `Unique Topologies` | 忽略 link/joint 名称、网格路径和数值参数后，对规范化有根运动树计算 graph hash；唯一 hash 数除以具有 valid rooted tree 的可评估资产数，并同时报告该分母相对 `N_eval` 的 coverage。 |
| `Exact Duplicate Rate` | 对规范化 URDF 及其递归可解析的 simulation-resource closure 计算 fingerprint；重复超额资产数除以 fingerprint 完整的可评估资产数，并同时报告该分母相对 `N_eval` 的 coverage。 |

`Paper-reported Assets` 只用于说明论文声称的数据规模，不作为任何成功率的分母。正式结果必须使用本地冻结的 `N_release` 和 `N_eval`。

Ours-500K 使用 ModelScope 数据集 `Brain` 发布的官方归档 `arti_cabinet_drawer_geometry_500_20260813.zip` 的本地获取全量 roster（roster protocol `ours500k-table1-roster-v1`）。归档 SHA256 为 `ffedf5bd90ae5eb96a061d0e127b700915ed6c221eeb7c5afe282b7249bfbd66`，与发布 sidecar 一致；归档内 `manifest.json`（`exact-template-seed-cohort-from-geometry-batch/v1`）声明的 `source_zip_sha256` 为 `616ce6a2d74b3282ab61531756b28965b1a23600f0548e44ace9525f135af206`，对应再封装前的原始批次包，仅作 supplementary provenance。获取样本 roster 为 500 个资产，小于其余较大发布集所用的 N=800 抽样规模，因此全量 roster 进入评测，无抽样、无替换、无结果过滤，`N_release = N_eval = 500`；500 / 500 个资产完成 Table 1 结构评测，0 error、0 timeout，XML 解析与资源闭包 fingerprint 均完整。声明层结构统计使用每个资产的 `model.urdf`；movable joint 为声明 XML 计数（样本共 2,467 个：808 revolute、1,339 prismatic、320 continuous），包含全部非 `fixed` 声明 joint，不等同于 runtime 可执行 DoF。`#Categories` 按归档内模板声明目录名精确去重，为 12（release 与样本相同，未做语义合并）。Exact Duplicate Rate 的 fingerprint 遵循 `simulation-package-fingerprint-v2`，fingerprint containment root 为发布布局下的资产包目录本身（`model.urdf` 与 `assets/` 资源闭包原生完整，无 mesh/MTL 外部引用）。Topology 与 duplicate rate 的分母均为 500，coverage 均为 500 / 500。证据见 [summary.json](runtime/table1_ours_500k/summary.json)、[manifest.json](runtime/table1_ours_500k/manifest.json)、[asset_records.jsonl](runtime/table1_ours_500k/asset_records.jsonl) 和 [report.md](runtime/table1_ours_500k/report.md)；roster hash 为 `ed70ebeb97f9ad8a655288e2afce96b0c3a8e26f50653e50dbbdc00238cfea3b`，冻结 manifest 文件 SHA256 为 `bc3eb334b1fc1c57378e50e7c2fab5d765a7599db8b3e82bc1d91536570b7c06`。该运行冻结的是结果写回前的 protocol SHA256 `6468ff244f700e9f77d23fc9322180c0321c49f43e7f2af5379af18ecd395e38`；本段属于运行后的报告更新。

Artiverse 结果来自本地预发布版本的固定全局样本（seed 20260813）。Table 1 的轻量 XML 语法解析为 800 / 800；拓扑和重复率在 797 个可评估资产上计算。该 XML 语法检查不同于 Table 2 中包含资源加载的标准 `urdfpy` 完整解析，后者为 797 / 800。

Articraft-10K 使用 Table 2 的同一固定样本（seed 20260813），全部指标分母均为 800。论文报告 245 类；本地发布集按资产 ID 联结官方 `record.json` 后覆盖 240 类，样本覆盖 222 类。

LAM released outputs 使用 Table 3 的同一固定 N=800 样本（seed 20260813），800 / 800 XML 可解析；拓扑分母为 733，重复率分母为 722（22 个缺资源，56 个结构无法规范化），未替换失败样本。

PartNet-Mobility 使用 Table 4 的固定 N=800 样本；13 个缺资源资产仍计入结构指标，重复率按资源闭包完整的 787 个资产计算。论文报告 2,346 个对象，本地冻结发布 roster 为 2,347。

PhysX-Mobility 使用 Table 5 的同一冻结 N=800 样本（rank salt `arti-skill-table5-physx-mobility-n800-v1`，rank 规则 `ascending (rank_sha256, integer dataset_id), first N`，无替换），800 / 800 个资产完成 Table 1 结构评测，0 error、0 timeout。`N_release` 为官方 `Caoza/PhysX-Mobility` 归档（rev `d0768ee9e1415f6be8db78d6389ba018b85134c0`）中 urdf / finaljson / partseg 三模态闭包一致的 2,024 个资产，运行期对全部选中文件重新做了归档字节级绑定校验。声明层结构统计使用官方发布的 `urdf/<id>.urdf`；movable joint 为声明 XML 计数，包含 exporter extension joint types（样本中共 1 个 `floating` joint，资产 `11854`），不等同于 runtime 可执行 DoF；全部官方 URDF 均未声明 collision 元素。`#Categories` 按官方 finaljson `category` 声明字符串精确去重，为 132（release）/ 98（样本）；去除空白并大小写折叠后为 116 / 88（仅 supplementary 诊断），声明标签存在同一类别多种拼写变体（如 `Building Component` / `BuildingComponent`），正式单元格不做语义合并；早前草稿记录的 `47` 为未经本地核验的占位值，已由本次冻结计数替换。Exact Duplicate Rate 的 fingerprint 遵循 `simulation-package-fingerprint-v2`；由于 PhysX-Mobility 的资源位于扁平 `urdf/` 目录的 sibling 目录 `partseg/`，fingerprint containment root 取逐资产字节级复制（hash 校验）并复刻发布几何的 staging root。Topology 与 duplicate rate 的分母均为 800，coverage 均为 800 / 800。证据见 [summary.json](runtime/table1_physx_mobility/summary.json)、[manifest.json](runtime/table1_physx_mobility/manifest.json)、[asset_records.jsonl](runtime/table1_physx_mobility/asset_records.jsonl) 和 [report.md](runtime/table1_physx_mobility/report.md)；冻结 manifest 文件 SHA256 为 `ccb54f4b726fe717efd28a37948e6b92bac994a2c0ba8fb4ea9ac4548d3a9882`，cohort hash 为 `a9c9c710d9617dea366696603984e330780ce177fead2a34c60410588cc1273c`，协议 hash 为 `4403a4190e2393c2812cf25193cbc6a08e75b350e65f47302db7f7c8a7321101`。

SketchMobility 行仅登记发布规模，不是正式评测结果。本地于 2026-08-20 从 Hugging Face `Arlo397/SketchMobility` 完整下载发布包；`dataset_chunks/manifest.json` 声明 `objectCount = 4,956`、`fileCount = 193,872`、`uncompressedBytes = 14,979,180,697`，并逐 chunk 记录 SHA-256（`sketchmobility_data-00001-of-00003.tar.gz` `b3f41ae84a2c7200e456ede31b05b89e095361a6eaf42babc6ca78c7efaa656f`、`sketchmobility_data-00002-of-00003.tar.gz` `193798b632106c3c696402fabc98303571014b7edfe5749249d546495fae4cf1`、`sketchmobility_data-00003-of-00003.tar.gz` `c4e9c971be402d3b30d9351781b3e2044e138de00dea7fda9c79722144df1572`）。数据尚未解包，冻结 per-asset roster、`N_release`、`N_eval` 与全部 Table 1 结构指标（links/joints、multi-joint、topology、duplicate fingerprint）均须解包后按冻结协议另行产生，在此之前记为 `TBD`。发布 README 声明其来源组成为 Articraft (`Agentic`) 205、Infinigen-Articulated 726、PartNeXt 2,177、Shape2Motion 1,848，混合许可（CC BY 4.0 与 GPL-3.0 并存）。

---

## Table 2. URDF Validity and Structural Integrity

| Dataset / Outputs | Parse Rate ↑ | Resource Resolution ↑ | Finite Fields ↑ | Valid Tree ↑ | Valid Joint Spec. ↑ | Collision Coverage ↑ | Inertial Coverage ↑ | Inertia Validity ↑ | Strict URDF Pass ↑ |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Ours-500K | 500 / 500 (100.00%) | 500 / 500 (100.00%) | 500 / 500 (100.00%) | 500 / 500 (100.00%) | 500 / 500 (100.00%) | 500 / 500 (100.00%) | 4 / 500 (0.80%) | 4 / 500 (0.80%) | 4 / 500 (0.80%) |
| Articraft-10K | 800 / 800 (100.00%) | 800 / 800 (100.00%) | 800 / 800 (100.00%) | 800 / 800 (100.00%) | 800 / 800 (100.00%) | 223 / 800 (27.88%) | 317 / 800 (39.62%) | 317 / 800 (39.62%) | 10 / 800 (1.25%) |
| LAM released outputs | 719 / 800 (89.88%) | 777 / 800 (97.12%) | 800 / 800 (100.00%) | 733 / 800 (91.62%) | 759 / 800 (94.88%) | 372 / 800 (46.50%) | 25 / 800 (3.12%) | 25 / 800 (3.12%) | 24 / 800 (3.00%) |
| Artiverse | 797 / 800 (99.62%) | 800 / 800 (100.00%) | 800 / 800 (100.00%) | 797 / 800 (99.62%) | 800 / 800 (100.00%) | 777 / 800 (97.12%) | 800 / 800 (100.00%) | 800 / 800 (100.00%) | 774 / 800 (96.75%) |
| PartNet-Mobility | 95 / 800 (11.88%) | 787 / 800 (98.38%) | 800 / 800 (100.00%) | 800 / 800 (100.00%) | 800 / 800 (100.00%) | 0 / 800 (0.00%) | 0 / 800 (0.00%) | 0 / 800 (0.00%) | 0 / 800 (0.00%) |
| PhysX-Mobility | 800 / 800 (100.00%) | 800 / 800 (100.00%) | 800 / 800 (100.00%) | 800 / 800 (100.00%) | 800 / 800 (100.00%) | 0 / 800 (0.00%) | 800 / 800 (100.00%) | 800 / 800 (100.00%) | 0 / 800 (0.00%) |
| SketchMobility | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD |

### Table 2 metric definitions

| Metric | Pass condition |
|---|---|
| `Parse Rate` | 标准 URDF parser 能够完整解析文件。 |
| `Resource Resolution` | 所有 mesh、texture 和 material 引用均存在、可读取且格式受支持。 |
| `Finite Fields` | origin、axis、limit、mass、COM、inertia、damping 和 friction 不包含 NaN/Inf。 |
| `Valid Tree` | 恰有一个 root link；图无环；不存在 orphan link；所有 link 均可从 root 到达。 |
| `Valid Joint Spec.` | parent/child link 存在且不同；axis 非零；有界关节满足 `lower < upper`；continuous joint 不错误声明有限区间。 |
| `Collision Coverage` | 所有需要参与物理仿真的刚性 link 均包含可加载的 collision geometry。 |
| `Inertial Coverage` | 所有动态 link 均包含 mass、COM 和 inertia。 |
| `Inertia Validity` | mass 为正；inertia 对称正定，并满足主惯量三角不等式；COM 有限。 |
| `Strict URDF Pass` | 同一资产同时通过本表全部检查。 |

Ours-500K 行为 overall micro average。正式运行严格复用 Table 1 冻结的全量获取 roster（[manifest.json](runtime/table1_ours_500k/manifest.json) SHA256 `bc3eb334b1fc1c57378e50e7c2fab5d765a7599db8b3e82bc1d91536570b7c06`、[asset_records.jsonl](runtime/table1_ours_500k/asset_records.jsonl) SHA256 `930951dd083ad91865388213e04c03fa2acdee5ca086411380e87be5225a64fe`、roster hash `ed70ebeb97f9ad8a655288e2afce96b0c3a8e26f50653e50dbbdc00238cfea3b`），按 `selection_rank` 升序评测全部 500 个资产，没有重新抽样、替换或按结果筛选；cohort 即 Table 1 的 `FULL_ACQUIRED_RELEASE_SAMPLE_NO_SUBSAMPLING`。发布布局本身即为自包含资产包（`<category>/<seed_N>/model.urdf` 加 `assets/` 资源闭包），无需 staging；评测前后对每个包重新做内容绑定校验。

500 / 500 个资产在本轮独立完成评测，0 error、0 timeout，运行状态全部为 `completed`。标准 `urdfpy 0.0.22` 完整加载（含资源 eager load）通过 500 / 500。`Inertial Coverage` 与 `Inertia Validity` 仅 4 / 500：只有 `desk_with_drawer_card_catalog` 的 seed 0--3 在全部声明 link 上写了 URDF `<inertial>` 块；其余资产的物理参数以随包 `physics.json` 的材质/密度绑定（`density_kg_m3` 等）形式声明，而冻结 Table 2 定义只审计 URDF 声明字段，因此不计入 inertial 覆盖，该口径与 PartNet-Mobility 段的保守 operationalization 一致。`Collision Coverage` 以全部声明 link 为待检集合，500 / 500 个资产的所有声明 link 均具可加载 collision geometry。12 类等权 category-level macro average 为 Parse Rate 100.00%、Resource Resolution 100.00%、Finite Fields 100.00%、Valid Tree 100.00%、Valid Joint Spec. 100.00%、Collision Coverage 100.00%、Inertial Coverage 8.33%、Inertia Validity 8.33%、Strict URDF Pass 8.33%。证据见 [summary.json](runtime/table2_urdf_ours_500k_table1cohort_n500_20260819T094919Z/summary.json)、[summary.md](runtime/table2_urdf_ours_500k_table1cohort_n500_20260819T094919Z/summary.md)、[manifest.json](runtime/table2_urdf_ours_500k_table1cohort_n500_20260819T094919Z/manifest.json)、[asset_records.jsonl](runtime/table2_urdf_ours_500k_table1cohort_n500_20260819T094919Z/asset_records.jsonl) 和 [protocol_snapshot.md](runtime/table2_urdf_ours_500k_table1cohort_n500_20260819T094919Z/protocol_snapshot.md)；formal manifest self-hash 为 `8b1bdb53bef17ac104bb42daa331899dfa498020be131fe5727f16df0f0427fa`，evaluator SHA256 为 `296f2af6fa73721a586a3e4b60459533b4656ed4068d06935828aab61c074d75`，runner SHA256 为 `7b57060d8481d4fe75a0028913f6c5826333cece7994c9fdbc6e14c9ed963dbe`，protocol snapshot SHA256 为 `0fc201014c9dd063f100bd44a15e067ea7922d01978eb03331a791044b036f4b`，environment（Python 3.12.3、`arti-template/.venv`、urdfpy 0.0.22）SHA256 为 `6fb01b894c54c3abe13e2010bbae39e7167b7fe117f8f59a7ffda548336f86e1`。该运行冻结的是结果写回前的 protocol snapshot；本段属于运行后的报告更新。

Articraft-10K 行为 overall micro average。正式运行从本地冻结的 `camvsl/Articraft-10K@3c79d5a05bb7cb6bf7bfee5e090176636ee3ac65` 发布集（`N_release = 9,996`）中，以 seed `20260813` 在查看评测结果前确定性抽取 `N_eval = 800`；800 个资产均完成评测，无错误或缺失记录。该结果是 frozen random cohort，不是 Full Release Cohort；该轮未预注册类别联结，category-level macro average 因此记为 `N/E`（`not_evaluable`）。证据见 [summary.json](runtime/table2_urdf_articraft10k_n800_seed20260813_20260813T145915Z/summary.json)、[manifest.json](runtime/table2_urdf_articraft10k_n800_seed20260813_20260813T145915Z/manifest.json) 和 [asset_records.jsonl](runtime/table2_urdf_articraft10k_n800_seed20260813_20260813T145915Z/asset_records.jsonl)；冻结 cohort 的资产 ID 哈希为 `79c44441600077513d3cde1cda8fef38324e1a0ee660730b860d5313f0ae9784`。

LAM released outputs 行为 overall micro average。正式运行读取 [Table 3 asset records](runtime/urdf_table3_lam_n800_seed20260813_20260814T022400Z_v3/asset_records.jsonl) 中全部 800 个 `asset_key`，按同目录 Table 3 manifest 的 `selection_rank = 1..800` 冻结选样顺序重建并逐项校验；没有采用 JSONL 的 worker completion 行序，也没有重新抽样、替换失败项或按 Table 2 / Table 3 结果筛选。该 cohort 来自本地 `VERIFIED_RELEASE_COMPLETE` 的 `YipengGao/Articulated-Object-Code@28cec4f5be7e34fd4d586879ecfcb67f7c5e4cc0` released outputs（`N_release = 3,217`），seed 为 `20260813`，包含 621 个 `viable`、75 个 `loads_only` 和 104 个 `broken` 资产，覆盖 305 个 observed category。该结果是 frozen global random cohort，不是 Full Release Cohort 或 Shared-category Balanced Cohort。

800 / 800 个资产均在本轮独立完成评测，0 error、0 timeout。标准 `urdfpy 0.0.22` 完整加载通过 719 个资产：22 个 `broken` 资产因共 42 个缺失 mesh 引用，在标准 parser 前的 containment/resource preflight 中 fail closed；另有 59 个资源闭包通过的资产被标准 parser 拒绝。第 23 个 Resource Resolution 失败项为 `loads_only:objects/scissors/scissors_021`：两个 OBJ 文件存在，但顶点全部为 NaN；`urdfpy` 仍可加载，因此该资产 Parse Rate 通过、Resource Resolution 失败。`Finite Fields` 只检查 URDF 声明字段，不检查 mesh 顶点，所以其 800 / 800 与该 NaN OBJ 结果不矛盾。Table 1 / Table 3 的 800 / 800 是轻量 XML well-formed 解析，不等同于本表的标准 parser 完整加载；Parse Rate 与 Resource Resolution 也不是相互独立的指标。

Table 3 中两条 retained error 来自冻结运动学 runner 不支持 `floating` joint；两项均保留，并在本轮 Table 2 独立完成，且通过 parse、resource、finite-field、tree、joint-spec 和 collision 检查，只因缺少 complete inertial 而未通过 inertial、inertia 和 strict 指标。冻结 evaluator 保守地以全部声明 link 作为 collision/inertial 待检集合：372 个资产的全部声明 link 具有可加载 collision geometry，406 个资产完全没有 collision，另 22 个缺资源资产按失败保留；25 个资产的全部 98 个声明 link 具有 complete inertial，其余 775 个资产没有 complete inertial。上述 98 个 inertia tensor 的特征值均为 `[1, 1, 1]`；这只证明其代数有效，不证明质量或惯量经过几何或动力学校准。

| LAM category macro | Parse Rate ↑ | Resource Resolution ↑ | Finite Fields ↑ | Valid Tree ↑ | Valid Joint Spec. ↑ | Collision Coverage ↑ | Inertial Coverage ↑ | Inertia Validity ↑ | Strict URDF Pass ↑ |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 305 observed categories, unweighted mean | 80.17% | 92.76% | 100.00% | 86.40% | 88.99% | 74.04% | 0.83% | 0.83% | 0.79% |

该 category macro 仅作为补充结果：305 个 observed category 中有 203 个在样本里只有一个资产，且类别字符串未做统一语义归并，不能替代主表的 asset-level micro average。LAM Table 2 证据见 [summary.json](runtime/table2_urdf_lam_table3cohort_n800_seed20260813_20260814T081000Z/summary.json)、[summary.md](runtime/table2_urdf_lam_table3cohort_n800_seed20260813_20260814T081000Z/summary.md)、[manifest.json](runtime/table2_urdf_lam_table3cohort_n800_seed20260813_20260814T081000Z/manifest.json)、[asset_records.jsonl](runtime/table2_urdf_lam_table3cohort_n800_seed20260813_20260814T081000Z/asset_records.jsonl) 和 [protocol_snapshot.md](runtime/table2_urdf_lam_table3cohort_n800_seed20260813_20260814T081000Z/protocol_snapshot.md)。Table 3 cohort JSONL SHA256 为 `7ef1c38d61bc780e41f62c7dd359e66f0bfeabe655c7453c93e2ea9830122d94`，Table 3 manifest 文件 / self-hash 分别为 `7e16683bfe4e4f37d7972082d8512713c1d8d1ae4ce142b75bf7dfb0509b9951` / `f8f7fe4da5634d4f806e793c0da919689eab25be1ce0bbed7e2232f3453d15c2`，ordered selected asset-key SHA256 为 `643aa5b76ac61f57dd943bee26444a3525c01201a8dff3443763a7fd8d8267d3`。Table 2 formal manifest 文件 / self-hash 分别为 `9acbaeab8a46e7bf28cec019a2087285afb85cdfb53659e1336181828b58c6a6` / `11da8354602f42e2613d269a086d75710531be79d4cf7b881b30babb41ad22fa`，summary / asset records SHA256 分别为 `a96ea0960b8e3079e7b0298036fc3d673ed0706426724236afbe3025613e2e5c` / `5abe1d763c9d92aeb5fe2fefed83322bfd98b7b0fd3565f1e79203d50ff83b04`，evaluator SHA256 为 `296f2af6fa73721a586a3e4b60459533b4656ed4068d06935828aab61c074d75`，release manifest SHA256 为 `70216593ec02b71d596e456498ff9863ad0f8e519d5d27d2cf4f58792d412412`。该运行冻结的是结果写回前、输出目录内的 protocol snapshot SHA256 `3d1983d503b1ea0a848e0d81f07b88cc9047d5d2d0bbef9ca8132c0983299043`；本段属于运行后的报告更新。

Artiverse 行为 overall micro average。正式运行严格复用 `exp/runtime/table1_artiverse/manifest.json` 中 `.assets[].manifest_root` 的既有顺序，不重新抽样或按结果筛选：本地数据为 `PRE_RELEASE_SUBSET`（`N_release = 3,544`），固定全局样本为 `N_eval = 800`、seed `20260813`。800 个资产均完成评测，`error = 0`、`timeout = 0`；其中 774 个资产通过 Strict URDF，而不是 800 个资产全部通过。该 cohort 覆盖 67 个 observed `raw_category`，不是 Full Release Cohort，也不是 Shared-category Balanced Cohort。

| Artiverse category macro | Parse Rate ↑ | Resource Resolution ↑ | Finite Fields ↑ | Valid Tree ↑ | Valid Joint Spec. ↑ | Collision Coverage ↑ | Inertial Coverage ↑ | Inertia Validity ↑ | Strict URDF Pass ↑ |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 67 observed raw categories, unweighted mean | 99.79% | 100.00% | 100.00% | 99.79% | 100.00% | 94.30% | 100.00% | 100.00% | 94.09% |

Artiverse 证据见 [summary.json](runtime/table2_urdf_artiverse_table1cohort_n800_seed20260813_20260814T001002Z/summary.json)、[manifest.json](runtime/table2_urdf_artiverse_table1cohort_n800_seed20260813_20260814T001002Z/manifest.json) 和 [asset_records.jsonl](runtime/table2_urdf_artiverse_table1cohort_n800_seed20260813_20260814T001002Z/asset_records.jsonl)。冻结 cohort manifest SHA256 为 `f74575692b87605699c4f349186c4660d691c91bef39562bb976baf22ae72a8c`，选中资产 ID 哈希为 `118038a746cafb91251afde5eb4f1164915d141acb3b529ea721a9d376bde3fa`，formal manifest self-hash 为 `c4a65440a1b78e8195434a68368dd5a45e5e8310b86ae309e8065f6ab7b5c484`。该运行冻结的是结果写回前的 protocol SHA256 `8cb07983a85f49d485a35d7dc59ec08c2f02a38bb9b6c75e3aab7fb09be468fe`；本段属于运行后的报告更新。

PartNet-Mobility 行为 overall micro average。正式运行严格复用 [Table 4 frozen manifest](runtime/urdf_table4_partnet_mobility_n800_20260813/frozen_manifest.json) 中 `.items[].dataset_id` 的全部 800 项及既有顺序，不重新抽样或按结果筛选。本地冻结 release roster 为 `N_release = 2,347`，固定全局样本为 `N_eval = 800`，覆盖 46 个 observed category；原选择策略使用 salt `urdf-sim-ready-table4-partnet-mobility-n800-v1:20260813`。该 cohort 不是 Full Release Cohort 或 Shared-category Balanced Cohort。

800 / 800 个资产均完成评测，0 error、0 timeout。标准 `urdfpy 0.0.22` 完整解析通过 95 个资产；692 个资产因 `JointLimit` 缺少 parser 要求的 `effort` 属性而失败，另有 13 个资产因共 79 个缺失的 collision mesh 引用，在标准 parser 前的 containment/resource preflight 中 fail closed。`Valid Tree` 和 `Valid Joint Spec.` 是独立的 XML 结构检查，因此其 800 / 800 结果不等同于标准 parser 完整解析成功。该轮冻结 evaluator 以全部声明 link 作为 collision/inertial 待检集合：787 个资源闭包完整的资产均至少有一个声明 link 缺少可加载 collision geometry（通常为空 `base` link，其中 733 个资产仅因该 link 失败），其余 13 个资源缺失资产同样按失败保留，故 `Collision Coverage` 为 0 / 800。这是冻结实现对文字协议中“需要参与物理仿真的刚性 link”的保守 operationalization：空 dummy/root frame 也会计为 coverage 失败，因此 0 / 800 不应解读为所有承载实体几何的物理 link 都没有 collision geometry。全部 5,678 个待检 link 中没有 complete inertial，故 `Inertial Coverage`、`Inertia Validity` 和 `Strict URDF Pass` 均为 0 / 800。

| PartNet-Mobility category macro | Parse Rate ↑ | Resource Resolution ↑ | Finite Fields ↑ | Valid Tree ↑ | Valid Joint Spec. ↑ | Collision Coverage ↑ | Inertial Coverage ↑ | Inertia Validity ↑ | Strict URDF Pass ↑ |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 46 observed categories, unweighted mean | 12.00% | 98.15% | 100.00% | 100.00% | 100.00% | 0.00% | 0.00% | 0.00% | 0.00% |

PartNet-Mobility 证据见 [summary.json](runtime/table2_urdf_partnet_mobility_table4cohort_n800_salt20260813_20260814T033747Z/summary.json)、[summary.md](runtime/table2_urdf_partnet_mobility_table4cohort_n800_salt20260813_20260814T033747Z/summary.md)、[manifest.json](runtime/table2_urdf_partnet_mobility_table4cohort_n800_salt20260813_20260814T033747Z/manifest.json)、[asset_records.jsonl](runtime/table2_urdf_partnet_mobility_table4cohort_n800_salt20260813_20260814T033747Z/asset_records.jsonl) 和 [protocol_snapshot.md](runtime/table2_urdf_partnet_mobility_table4cohort_n800_salt20260813_20260814T033747Z/protocol_snapshot.md)。Table 4 cohort manifest SHA256 为 `2ff015ee6bb377ce693126b52dd632a7565a3eaa9f0007e26122a1bb4ab99900`，ordered asset IDs SHA256 为 `ef6cb964e50dc712280256c5b2f675cc2c957095c3553b21845d3562a5011883`，formal manifest self-hash 为 `f956813622e8fac7d8c00465ff827dc21c5edb43285ab846a1a10a364b66bf65`，evaluator SHA256 为 `be2211f6d7cda591125b7a6804ecce86d6b1fbc4c2a11639851a4844613c4d07`，archive SHA256 为 `b47247a44246111e8d09f2c0e64b4012ae35e0dcf4bb55f68a05b604455119ff`。本地数据状态为 `LOCAL_COMPLETE_PROVENANCE_LIMITED`：固定源为 `sapien-sim/PartNetMobility@ee0aa3ef1df16181d76d83f7415aa8c94ed1da8f`，但 gated revision 的对象 bytes 未与本地文件直接认证。该运行冻结的是结果写回前、输出目录内的 protocol snapshot SHA256 `be3813e1b40b4fb8e2ee5cf9bec89aa3b83d7dcca3050a0c6c3eeb3097c36ed1`；本段属于运行后的报告更新。

| PhysX-Mobility category macro | Parse Rate ↑ | Resource Resolution ↑ | Finite Fields ↑ | Valid Tree ↑ | Valid Joint Spec. ↑ | Collision Coverage ↑ | Inertial Coverage ↑ | Inertia Validity ↑ | Strict URDF Pass ↑ |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 98 observed categories, unweighted mean | 100.00% | 100.00% | 100.00% | 100.00% | 100.00% | 0.00% | 100.00% | 100.00% | 0.00% |

PhysX-Mobility 行为 overall micro average。正式运行严格复用 Table 5 冻结 receipt set（[manifest.json](runtime/table5_physx_mobility_n800_v2/manifest.json)，文件 SHA256 为 `ccb54f4b726fe717efd28a37948e6b92bac994a2c0ba8fb4ea9ac4548d3a9882`，cohort hash 为 `a9c9c710d9617dea366696603984e330780ce177fead2a34c60410588cc1273c`）中全部 800 行及既有 rank 顺序，没有重新抽样、替换失败项或按结果筛选。800 / 800 个资产均完成评测，0 error、0 timeout。官方 PhysX-Mobility 发布几何中资源位于扁平 `urdf/` 目录的 sibling 目录 `partseg/`，因此每个资产先按冻结 manifest 行哈希逐字节校验复制为复刻发布几何的自包含 staging package，再交给与 PartNet-Mobility 同一冻结审计核心（`run_table2_urdf_articraft.py` 的 `audit_asset_package`，urdfpy 0.0.22 标准 parser 与全部九项检查实现不变）评测；staging package 为临时产物，审计前后均重算 package binding，provenance 链为官方归档字节 → 冻结 manifest 行哈希 → staging 字节校验 → 本运行 manifest 冻结 package binding → 审计前后绑定校验，全部选中文件另做了官方归档字节级绑定校验（archive SHA256 `88308cc2a4cc6177c59e32c2de51e881e6b961737295e5082d7ed01cca221908`）。标准 `urdfpy 0.0.22` 完整解析通过 800 个资产；评测环境依赖与 PartNet-Mobility Table 2 冻结环境完全一致（Python 3.12.3、urdfpy 0.0.22、numpy 2.4.4、trimesh 4.12.2、Pillow 12.2.0、networkx 3.6.1、pycollada 0.6、pygltflib 1.16.5、lxml 6.1.1、six 1.17.0、scipy 1.17.1）。官方 URDF 未声明任何 collision 元素（冻结 manifest `xml_counts.collision_elements` 总和为 0），全部 10,086 个声明 link 均记为 `collision_missing`，故 `Collision Coverage` 为 0 / 800；该轮沿用 PartNet-Mobility 轮以全部声明 link 为待检集合的保守 operationalization，0 / 800 不应解读为承载实体几何的物理 link 均无 collision 之外的结构缺陷。全部声明 link 均具有 complete inertial，mass 为正、惯量张量对称正定且满足主惯量三角不等式，故 `Inertial Coverage` 与 `Inertia Validity` 均为 800 / 800；`Strict URDF Pass` 因 `Collision Coverage` 失败而为 0 / 800。98 个 observed category 等权 category-level macro average 为 Parse Rate 100.00%、Resource Resolution 100.00%、Finite Fields 100.00%、Valid Tree 100.00%、Valid Joint Spec. 100.00%、Collision Coverage 0.00%、Inertial Coverage 100.00%、Inertia Validity 100.00%、Strict URDF Pass 0.00%。证据见 [summary.json](runtime/table2_urdf_physx_mobility_table5cohort_n800_20260819T091324Z/summary.json)、[summary.md](runtime/table2_urdf_physx_mobility_table5cohort_n800_20260819T091324Z/summary.md)、[manifest.json](runtime/table2_urdf_physx_mobility_table5cohort_n800_20260819T091324Z/manifest.json)、[asset_records.jsonl](runtime/table2_urdf_physx_mobility_table5cohort_n800_20260819T091324Z/asset_records.jsonl) 和 [protocol_snapshot.md](runtime/table2_urdf_physx_mobility_table5cohort_n800_20260819T091324Z/protocol_snapshot.md)。formal manifest 文件 SHA256 为 `e1b1ce88f81a4adb5dad4b8658f257d8c50e9c28caed5c33f31831e2349f894d`，self-hash 为 `4218abd2bcb0d67acacb116f7ab03825b69b982afe4368a793c22567524f738c`，审计核心 evaluator SHA256 为 `296f2af6fa73721a586a3e4b60459533b4656ed4068d06935828aab61c074d75`，runner SHA256 为 `be04c50cae05f12d6c041e73384de0885b642600944c7fc873b280304e7e03bf`，summary / asset records SHA256 分别为 `c04730a170ae697a950054b228ab314b8f37ac4a460df5934c5d412add9012ad` / `0194a67146dafb9afd8f1c6ff2536bc3d2aa95e1edffd9afd1f9d62b56643f9b`。该运行冻结的是子进程启动前写入输出目录的 protocol snapshot SHA256 `6468ff244f700e9f77d23fc9322180c0321c49f43e7f2af5379af18ecd395e38`；本段属于运行后的报告更新。

### Table 2 supplementary. Collision, Joint, and Inertial Diagnostics (Proposed; the original six methods evaluated, SketchMobility pending)

| Dataset / Outputs | Visual-bearing Collision Coverage ↑ | Joint-limit Portability ↑ | Joint Dynamics Coverage ↑ | Placeholder-mass Incidence ↓ |
|---|---:|---:|---:|---:|
| Ours-500K | 500 / 500 (100.00%) | 2,467 / 2,467 (100.00%) | 266 / 2,467 (10.78%) | N/E |
| Articraft-10K | 224 / 800 (28.00%) | 2,865 / 2,865 (100.00%) | 79 / 2,865 (2.76%) | N/E |
| LAM released outputs | 372 / 800 (46.50%) | 1991 / 2395 (83.13%) | 0 / 2395 (0.00%) | N/E |
| Artiverse | 800 / 800 (100.00%) | 3,742 / 3,875 (96.57%) | 0 / 3,875 (0.00%) | N/E |
| PartNet-Mobility | 800 / 800 (100.00%) | 0 / 4078 (0.00%) | 0 / 4078 (0.00%) | N/E |
| PhysX-Mobility | 0 / 800 (0.00%) | 3,808 / 3,809 (99.97%) | 0 / 3,809 (0.00%) | N/E |
| SketchMobility | TBD | TBD | TBD | TBD |

#### Table 2 supplementary metric definitions

| Metric | Definition |
|---|---|
| `Visual-bearing Collision Coverage` | 主值为资产级 `passed / N_eval`：资产必须可解析、至少包含一个在 XML 中声明 `<visual>` geometry 的 visual-bearing link，且每个此类 link 都至少包含一个资源可解析、可加载的 collision geometry；解析失败、visual/collision 资源失败或零 visual-bearing link 均 fail closed。另补充报告 `covered visual-bearing links / L_visual_declared` 的 link-micro 值及 link extraction coverage。该指标补充而不替换既有按全部声明 link 计算的 `Collision Coverage`。 |
| `Joint-limit Portability` | 关节级 `passed / J_eval`。bounded revolute/prismatic joint 必须具有有限的 `lower < upper`、有限且非负的 `effort` 和有限且为正的 `velocity`；continuous joint 不要求有限 lower/upper，但仍须满足冻结 adapter 共同要求的 effort/velocity 字段。其他 joint type 按查看结果前冻结的 per-type mapping 处理；缺字段、unsupported mapping 和未执行项均保留为失败。 |
| `Joint Dynamics Coverage` | 同时声明有限、非负 `damping` 与 `friction` 的 movable joints 数除以 `J_eval`；缺失任一字段计为未覆盖。该项只衡量字段覆盖，不证明数值经过动力学校准，也不进入既有 `Strict URDF Pass`。 |
| `Placeholder-mass Incidence` | 在具有 complete inertial 的动态 link 中，mass 或完整 inertial tuple 命中预注册 exporter/simulator 默认模板的 link 数除以 complete-inertial link 数，并同时报告 `complete-inertial links / dynamic links` coverage。默认模板或 sentinel 只能来自冻结工具默认值或公开文档，禁止查看方法结果后添加；分母为零时，正式运行后记为 `N/E` 而不是 0。该项是诊断 flag，不证明被标记质量一定错误，也不进入既有 `Strict URDF Pass`。 |

本表全部单元格已完成正式评测；不得用 N=10 reconnaissance、旧 Table 2 asset records 或 XML 字段存在性直接填数。任何未来重跑必须使用同一版本 evaluator（当前冻结为 `lam-supplementary-static/v1.2`）对全部方法统一执行。

| LAM released outputs category macro | Visual-bearing Collision Coverage ↑ | Joint-limit Portability ↑ | Joint Dynamics Coverage ↑ | Placeholder-mass Incidence ↓ |
|---|---:|---:|---:|---:|
| 305 observed categories, unweighted mean (302 with declared movable joints) | 74.04% | 81.07% | 0.00% | N/E |

| PartNet-Mobility category macro | Visual-bearing Collision Coverage ↑ | Joint-limit Portability ↑ | Joint Dynamics Coverage ↑ | Placeholder-mass Incidence ↓ |
|---|---:|---:|---:|---:|
| 46 observed categories, unweighted mean | 100.00% | 0.00% | 0.00% | N/E |

LAM released outputs 行为 overall micro average。正式运行严格复用 [Table 3 manifest](runtime/urdf_table3_lam_n800_seed20260813_20260814T022400Z_v3/manifest.json) 中全部 800 条 `records[]` 及 `selection_rank = 1..800` 冻结选样顺序（与 [Table 3 asset records](runtime/urdf_table3_lam_n800_seed20260813_20260814T022400Z_v3/asset_records.jsonl) 的 `asset_key` 集合逐项校验），没有重新抽样、替换失败项或按 Table 2 / Table 3 结果筛选；`N_eval = 800`、`J_eval = 2,395`，覆盖 305 个 observed category（621 个 `viable`、75 个 `loads_only`、104 个 `broken`）。每资产关节分母取 Table 3 manifest 的 `declared_joint_count_hint`，并在运行时逐项对照 Table 3 `asset_records.declared_joint_count`，不一致即 fail closed；每资产 `generated.urdf` 的 SHA-256 必须等于 Table 3 manifest 的 `urdf_sha256`，且冻结绝对路径 `urdf_path` 必须等于 `<package>/generated.urdf`。四个 metric atom 经方法无关的 `table2_supplementary_static` wrapper 复用 `lam_supplementary_static` atom 模块；placeholder 模板注册表按冻结决定为空（与 PartNet-Mobility 及 LAM supplementary precedent 一致）；parse gate 为「XML well-formed 且根元素为 robot」的冻结 precedent，与 Table 2 `Parse Rate` 的 urdfpy 标准 parser 不同，两者不可直接比较。

800 / 800 个资产全部完成评测，0 error、0 timeout。`Visual-bearing Collision Coverage` 为 372 / 800：800 个资产全部至少含一个 visual-bearing link（link extraction complete 800 / 800，零 visual-bearing link 资产为 0），372 个资产的全部 visual-bearing link 均具可加载 collision geometry；link-micro 值为 2,713 / 4,812 (56.38%)。该资产级数值与既有 Table 2 按全部声明 link 计算的 LAM `Collision Coverage` 372 / 800 一致，因 LAM 的 collision 声明集中出现在 visual-bearing link 上。`Joint-limit Portability` 为 1,991 / 2,395 (83.13%)：全部 404 个失败关节中，391 个 `continuous` 关节因未声明 `<limit>` 元素（冻结基数规则要求该元素存在）失败，2 个 `floating` 关节按冻结 `unsupported_mapping` 记为失败，其余 11 个为完全缺少 `<limit>` 的 bounded joint（5 prismatic、6 revolute）；不存在仅因 effort/velocity 数值失败的关节。`Joint Dynamics Coverage` 为 0 / 2,395：LAM released outputs 不声明任何 `<dynamics>` damping/friction 字段。`Placeholder-mass Incidence` 为 `N/E`（registry 为空），complete-inertial coverage 为 98 / 4,812 (2.04%)。

Resource closure provenance：778 个资产 `COMPLETE`、22 个资产 `PARTIAL`；后者全部为 `broken:imperfect/*` tier 资产，共 42 个缺失 mesh 引用（与 Table 2 LAM 冻结计数一致），且均涉及至少一个 visual-bearing link 的 collision geometry，已按 fail closed 计入 `Visual-bearing Collision Coverage`。运行环境为 Python 3.12.3、numpy 2.5.1、trimesh 5.0.0；workers = 8，单资产超时 300 秒，总墙钟约 637 秒；最终验证 24 / 24 checks PASS。证据见 [summary.json](runtime/table2sup_urdf_lam_table3cohort_n800_seed20260813_20260819T075824Z/summary.json)、[summary.md](runtime/table2sup_urdf_lam_table3cohort_n800_seed20260813_20260819T075824Z/summary.md)、[frozen_manifest.json](runtime/table2sup_urdf_lam_table3cohort_n800_seed20260813_20260819T075824Z/frozen_manifest.json)、[asset_records.jsonl](runtime/table2sup_urdf_lam_table3cohort_n800_seed20260813_20260819T075824Z/asset_records.jsonl)、[verification.json](runtime/table2sup_urdf_lam_table3cohort_n800_seed20260813_20260819T075824Z/verification.json)、[environment.json](runtime/table2sup_urdf_lam_table3cohort_n800_seed20260813_20260819T075824Z/environment.json) 和 [protocol_snapshot.md](runtime/table2sup_urdf_lam_table3cohort_n800_seed20260813_20260819T075824Z/protocol_snapshot.md)。Table 3 cohort manifest 文件 SHA256 为 `7e16683bfe4e4f37d7972082d8512713c1d8d1ae4ce142b75bf7dfb0509b9951`，self-hash 为 `f8f7fe4da5634d4f806e793c0da919689eab25be1ce0bbed7e2232f3453d15c2`，Table 3 asset_records SHA256 为 `7ef1c38d61bc780e41f62c7dd359e66f0bfeabe655c7453c93e2ea9830122d94`，ordered asset keys SHA256 为 `643aa5b76ac61f57dd943bee26444a3525c01201a8dff3443763a7fd8d8267d3`，frozen_manifest 文件 / self-hash 分别为 `5d83e02a232ae541537eafa9d161d33badae870e8b9d6f77bcc92422bbc68b9a` / `9d01e792a9391a49237b06cece7a152adba950a6433b09ecf8d8641642f8050d`，summary / asset records SHA256 分别为 `1ed3442067c4ee44a4ab400df80fed7336413cc4e7c2964830ea9135b034e059` / `d38a2ca83ec51fabbaebf8767d88ef3fccc776d7a0b23c9c8b1398a79b65491f`，runner SHA256 为 `45bf541d9250cd8ce4e9c34fb9205d8a789872f286ebbf825c18e57fdc6c9541`，共享 atom 模块 `lam_supplementary_static` / `table2_supplementary_static` SHA256 分别为 `04985b5adc97275f940c29bbb584e8f0b6d1dd62cd5ba543d1c71c4a64ae6cc5` / `56e4d831f4a162ebea9dfa493401fe6ca2b5a1598b45933a364c9734de76fc72`，protocol snapshot SHA256 为 `d32ac4a511ec2cabf4e8f8959b5736d7e30e65fb441191f8bc245c8b37f4e422`。注意：本运行冻结的 `lam_supplementary_static` 版本与 PartNet-Mobility 正式运行冻结的 `1c2fdc2c3d9f8ebcb3ab6b0bf8144b307c86b4b44790cf3182c2395ab37267ff` 不同（共享 atom 模块在两次运行之间被修改），最终跨方法比较前必须对全部方法统一重跑或做版本等价性确认。该运行冻结的是子进程启动前写入输出目录的 frozen manifest 与 protocol snapshot；本段属于运行后的报告更新。

PartNet-Mobility 行为 overall micro average。正式运行严格复用 [Table 4 frozen manifest](runtime/urdf_table4_partnet_mobility_n800_20260813/frozen_manifest.json) 中 `.items[].dataset_id` 的全部 800 项及既有顺序，没有重新抽样、替换失败项或按结果筛选；`N_eval = 800`、`J_eval = 4,078`，覆盖 46 个 observed category。每资产关节分母取 Table 4 frozen manifest 的 `movable_dof_count`，并在运行时逐项对照 Table 3 `asset_records.declared_joint_count` 与 `len(joint_specs)`，三者不一致即 fail closed；每资产 `mobility.urdf` 的 SHA-256 必须等于 frozen manifest 的 `urdf_sha256`。这是 Table 2 supplementary 的首个正式结果：四个 metric atom 复用与 LAM supplementary 相同冻结版本的 `lam_supplementary_static` atom 模块，placeholder 模板注册表按冻结决定为空（与 LAM supplementary precedent 一致）；parse gate 为「XML well-formed 且根元素为 robot」的冻结 precedent，与 Table 2 `Parse Rate` 的 urdfpy 标准 parser 不同，两者不可直接比较。

800 / 800 个资产全部完成评测，0 error、0 timeout。`Visual-bearing Collision Coverage` 为 800 / 800：每个资产至少含一个 visual-bearing link，且全部 4,807 个 visual-bearing link 均至少含一个资源可解析、可加载的 collision geometry（link-micro 4,807 / 4,807）；link extraction 在 800 / 800 个资产上 complete，没有零 visual-bearing link 的 completed 资产。该结果与 Table 2 按全部声明 link 计算的 `Collision Coverage` 0 / 800 互补：后者因空 `base` dummy link 失败，本指标只要求 visual-bearing link 至少一个可加载 collision geometry。Table 2 中缺 79 个 collision mesh 引用的 13 个资产在本指标下仍全部通过，因为其每个 visual-bearing link 都保留了至少一个可加载 collision geometry；79 个缺失引用在 resource closure 中逐项留痕。`Joint-limit Portability` 为 0 / 4,078：全部 4,078 个 movable joint 均缺 `effort` 与 `velocity` 字段，其中 662 个 `continuous` joint 还缺少 `<limit>` 元素（limit cardinality 0）；670 个 `revolute` 与 2,746 个 `prismatic` joint 均具有有限 `lower < upper`（无 lower/upper 违规），但仍因缺 effort/velocity 失败。`Joint Dynamics Coverage` 为 0 / 4,078：没有任何关节声明 `<dynamics>` 元素。`Placeholder-mass Incidence` 为 `N/E`（reason `placeholder_registry_empty`）：冻结注册表为空，且 cohort 中 complete-inertial link 数为 0，coverage 按测得的 5,678 个 dynamic link 报告（与 Table 2 的 5,678 个待检 link 一致）。

Resource closure provenance：309 个资产 `COMPLETE`、491 个资产 `PARTIAL`，其中 484 个资产的 OBJ 嵌套 mtl 纹理引用（`../images/...`）按冻结 safe-path 规则判为非规范路径，仅记录为 provenance issue，不进入任何 Table 2 supplementary 指标的分子或分母。运行环境为 Python 3.12.3、numpy 2.5.1、trimesh 5.0.0；workers = 4，单资产超时 120 秒，总墙钟约 270 秒；最终验证 11 / 11 checks PASS。证据见 [summary.json](runtime/table2sup_urdf_partnet_mobility_table4cohort_n800_salt20260813_20260819T072557Z/summary.json)、[summary.md](runtime/table2sup_urdf_partnet_mobility_table4cohort_n800_salt20260813_20260819T072557Z/summary.md)、[manifest.json](runtime/table2sup_urdf_partnet_mobility_table4cohort_n800_salt20260813_20260819T072557Z/manifest.json)、[asset_records.jsonl](runtime/table2sup_urdf_partnet_mobility_table4cohort_n800_salt20260813_20260819T072557Z/asset_records.jsonl)、[frozen_config.json](runtime/table2sup_urdf_partnet_mobility_table4cohort_n800_salt20260813_20260819T072557Z/frozen_config.json) 和 [protocol_snapshot.md](runtime/table2sup_urdf_partnet_mobility_table4cohort_n800_salt20260813_20260819T072557Z/protocol_snapshot.md)。Table 4 cohort manifest SHA256 为 `2ff015ee6bb377ce693126b52dd632a7565a3eaa9f0007e26122a1bb4ab99900`，ordered asset IDs SHA256 为 `ef6cb964e50dc712280256c5b2f675cc2c957095c3553b21845d3562a5011883`，frozen config SHA256 为 `ba24d84c82c644dd1d041a3cc9a3b42f01b7a29ee92e12cb8cafcf349c02423c`，runner SHA256 为 `0c00a1c4d0be55016a4b56d5913621bc2ffae2aab0dd1c27db30e4e64f32c1bc`，共享 atom 模块 SHA256 为 `1c2fdc2c3d9f8ebcb3ab6b0bf8144b307c86b4b44790cf3182c2395ab37267ff`，summary / asset records SHA256 分别为 `c850217ddbc2076025979e6fd69bb422bd50d67d2b94d2678fb630c7c28f949b` / `45cbe2a2fbd4cc1b21aafbfddf23ee0c105ecf80cd5d9a7bfbd67eb421725b62`，manifest 文件 / self-hash 分别为 `b2355414e523e8f6e3378d677b679d0b8cac9e26e71d0fa1e8c895e0a804e222` / `e8767ccc8d095e19ac070cb3b0bce0b2f5b8fa879fa6304bac8b80a63234489e`。该运行冻结的是子进程启动前写入输出目录的 frozen config 与 protocol snapshot；本段属于运行后的报告更新。

| Artiverse category macro | Visual-bearing Collision Coverage ↑ | Joint-limit Portability ↑ | Joint Dynamics Coverage ↑ | Placeholder-mass Incidence ↓ |
|---|---:|---:|---:|---:|
| 67 observed categories, unweighted mean | 100.00% | 84.69% | 0.00% | N/E |

Artiverse 行为 overall micro average。正式运行的抽样样本是 `jq -r '.assets[].manifest_root' exp/runtime/table1_artiverse/manifest.json` 的全部 800 项（Table 1 manifest 文件 bytes SHA-256 `f74575692b87605699c4f349186c4660d691c91bef39562bb976baf22ae72a8c`），严格保持原顺序，没有重新抽样、替换失败项或按结果筛选；与既有 Table 2 / Table 3 / Table 4 Artiverse 正式运行是同一 cohort。`N_eval = 800`、`J_eval = 3,875`，覆盖 67 个 observed raw category。每资产关节分母取冻结 Table 3 asset records 的 `declared_joint_count`（总和 3,875，含 3 个 joint graph 带环资产的声明关节）。parse gate 为「XML well-formed 且根元素为 robot」的冻结 precedent（与 PartNet-Mobility / LAM supplementary 运行一致），800 / 800 通过；每个 package 绑定为 `<artiverse_root>/<manifest_root>/urdf_w_collider`、primary URDF `<model_id>.urdf`，评测前逐文件与冻结 Table 2 manifest 的 binding 做了顺序无关的 bytes 级校验（相同路径集合、相同 size 与 SHA-256），800 / 800 一致。初始按 path-sorted 顺序重算 content-manifest hash 时曾有 44 个资产不匹配，经核查是冻结 Table 2 hash 使用 walk order 而非 sort order 的规范化差异；改为顺序无关逐文件相等判定后全部通过，文件集合与 bytes 均无漂移。

共享 atom 模块在本轮冻结为 `lam-supplementary-static/v1.1`（SHA-256 `04985b5adc97275f940c29bbb584e8f0b6d1dd62cd5ba543d1c71c4a64ae6cc5`），方法无关 wrapper `table2_supplementary_static` SHA-256 为 `56e4d831f4a162ebea9dfa493401fe6ca2b5a1598b45933a364c9734de76fc72`：与 LAM released outputs 正式运行冻结的 evaluator 版本逐位相同。相对 PartNet-Mobility 运行所用的 v1（SHA-256 `1c2fdc2c3d9f8ebcb3ab6b0bf8144b307c86b4b44790cf3182c2395ab37267ff`），唯一修订是统一资源解析规则——折叠冗余 `.` 段（`./objs/part.obj` -> `objs/part.obj`），继续拒绝 `..`、绝对路径、反斜杠与空段。该修订在查看任何 Artiverse 指标结果前冻结：对全部 800 个 primary URDF 的只读兼容性扫描发现 20,045 个 `./` 前缀 mesh 引用与 45,882 个普通相对引用，标准 URDF 消费方（含 Table 2 既有运行的 resource resolution）均可解析 `./` 引用。v1.1 的接受集是 v1 的严格超集（只新增 `.` 折叠路径），因此 PartNet-Mobility 的全部数值在 v1.1 下逐位不变：其进入指标的资源引用均为普通相对路径，其被拒的 `../images/...` 嵌套 mtl 纹理引用在 v1.1 下仍被拒且本就只记为 provenance issue。

800 / 800 个资产全部完成评测，0 error、0 timeout、0 binding failure。`Visual-bearing Collision Coverage` 为 800 / 800：每个资产至少含一个 visual-bearing link，且全部 6,851 个 visual-bearing link 均至少含一个资源可解析、可加载的 collision geometry（link-micro 6,851 / 6,851）；link extraction 在 800 / 800 个资产上 complete。该结果与 Table 2 按全部声明 link 计算的 `Collision Coverage` 777 / 800 互补。`Joint-limit Portability` 为 3,742 / 3,875（96.57%）：947 个 `revolute` 与 2,795 个 `prismatic` joint 全部通过（有限 `lower < upper`、有限非负 `effort`、有限正 `velocity`）；全部 133 个 `continuous` joint 均缺少 `<limit>` 元素（limit cardinality 0）从而也缺 `effort` / `velocity`，按冻结定义计为失败，涉及 75 个资产。`Joint Dynamics Coverage` 为 0 / 3,875：没有任何关节声明 `<dynamics>` 元素（damping 与 friction 均缺失）。`Placeholder-mass Incidence` 为 `N/E`（reason `placeholder_registry_empty`）：冻结注册表为空（在查看结果前没有来自冻结工具默认值或公开文档、可验证的 Artiverse exporter 默认模板，与 LAM / PartNet-Mobility precedent 一致），complete-inertial coverage 按测得的 6,874 / 6,874 个 declared link 报告。

运行环境为 Python 3.12.3，workers = 16，单资产超时 900 秒，评测阶段墙钟约 382 秒；独立验证 22 / 22 checks PASS。证据见 [summary.json](runtime/table2_supplementary_artiverse_table1cohort_n800_seed20260813_20260819T083013Z/summary.json)、[summary.md](runtime/table2_supplementary_artiverse_table1cohort_n800_seed20260813_20260819T083013Z/summary.md)、[frozen_manifest.json](runtime/table2_supplementary_artiverse_table1cohort_n800_seed20260813_20260819T083013Z/frozen_manifest.json)、[asset_records.jsonl](runtime/table2_supplementary_artiverse_table1cohort_n800_seed20260813_20260819T083013Z/asset_records.jsonl)、[verification.json](runtime/table2_supplementary_artiverse_table1cohort_n800_seed20260813_20260819T083013Z/verification.json) 和 [protocol_snapshot.md](runtime/table2_supplementary_artiverse_table1cohort_n800_seed20260813_20260819T083013Z/protocol_snapshot.md)。frozen manifest self-hash 为 `ac39ca000781c658b6ba127531cfd6b61361a94717377a5d0586a1c6bc6c4176`，protocol snapshot SHA256 为 `6031a52c20662f6e06b6a298ea7e3fc6d409ccefd0d873601aca5c510c911e85`，runner SHA256 为 `3280001d6c9072f127720334ace95295d135b714924e6042e975fbdefce53af3`，verifier SHA256 为 `8ce64f4b4f3c428a2a5cfede5d21a24a4c07d220da141261f333dc06bea0a7b7`。该运行冻结的是评测开始前写入输出目录的 frozen manifest 与 protocol snapshot；本段属于运行后的报告更新。

Articraft-10K 行为 overall micro average。正式运行严格复用 Table 2 frozen cohort manifest 中 `.records[].package` 的全部 800 项及原始顺序（抽样样本即 `jq -r '.records[].package' runtime/table2_urdf_articraft10k_n800_seed20260813_20260813T145915Z/manifest.json`），没有重新抽样、替换失败项或按结果筛选；`N_eval = 800`、`J_eval = 2,865`（revolute 1,324 / prismatic 981 / continuous 560），覆盖 222 个 observed category。每资产关节分母取冻结 Table 3 asset records 的 `declared_joint_count`，并在加载时逐项对照冻结 cohort manifest 的 `model_urdf_sha256` 与逐文件 package binding，不一致即 fail closed；primary URDF 为 `model.urdf`。parse gate 为「XML well-formed 且根元素为 robot」的冻结 precedent（与 PartNet-Mobility / LAM / Artiverse 运行一致），800 / 800 通过，0 error、0 timeout；link extraction 与 joint extraction 均为 800 / 800 COMPLETE。`Visual-bearing Collision Coverage` 主值为资产级 224 / 800 (28.00%)，link-micro 为 1,222 / 3,966 (30.81%)，link extraction coverage 800 / 800；`Joint-limit Portability` 为 2,865 / 2,865 (100.00%)；`Joint Dynamics Coverage` 为 79 / 2,865 (2.76%)；placeholder 模板注册表按冻结决定为空（无文档化的 Articraft 工具默认 mass/inertia 模板，与 LAM supplementary precedent 一致），故 `Placeholder-mass Incidence` 记为 `N/E`，complete-inertial coverage 为 1,365 / 3,967 (34.41%)。Category macro（222 个 observed categories，unweighted）：`Visual-bearing Collision Coverage` 29.78%、`Joint-limit Portability` 100.00%、`Joint Dynamics Coverage` 2.75%。四个 metric atom 复用同一冻结版本的 `lam_supplementary_static` atom 模块（SHA256 `04985b5adc97275f940c29bbb584e8f0b6d1dd62cd5ba543d1c71c4a64ae6cc5`）；运行后 bytecode 审计确认全部 Table 2 supplementary atom 函数与 PartNet-Mobility 行所冻结版本（`1c2fdc2c3d9f8ebcb3ab6b0bf8144b307c86b4b44790cf3182c2395ab37267ff`）逐字节一致，两个版本之间唯一的行为差异（`safe_package_relative_path` 对 `.` 段折叠的接受）在本 cohort 的 766 个 collision mesh 引用中遇到 0 个非规范引用，不改变本行任何分子或分母。运行环境为 Python 3.12.3，workers = 4，单资产超时 120 秒；最终独立验证 29 / 29 checks PASS。证据见 [summary.json](runtime/urdf_table2sup_articraft10k_table2_n800_seed20260813_20260819T083015Z/summary.json)、[summary.md](runtime/urdf_table2sup_articraft10k_table2_n800_seed20260813_20260819T083015Z/summary.md)、[manifest.json](runtime/urdf_table2sup_articraft10k_table2_n800_seed20260813_20260819T083015Z/manifest.json)、[asset_records.jsonl](runtime/urdf_table2sup_articraft10k_table2_n800_seed20260813_20260819T083015Z/asset_records.jsonl)、[verification.json](runtime/urdf_table2sup_articraft10k_table2_n800_seed20260813_20260819T083015Z/verification.json) 和 [protocol_snapshot.md](runtime/urdf_table2sup_articraft10k_table2_n800_seed20260813_20260819T083015Z/protocol_snapshot.md)。Table 2 cohort manifest 文件 SHA256 为 `13c47e2b2affadb951a01cab826bae139852fca5769e99ec081cc916ffa6373d`，内容 SHA256 为 `576852cb6da00775e1c51360b82b4be40e0a614e4fb0cfb1bae066912eed56a3`，选中资产 ID SHA256 为 `79c44441600077513d3cde1cda8fef38324e1a0ee660730b860d5313f0ae9784`，类别联结 SHA256 为 `0305569f49d2aa1acb72fbb7bc8dcaf68ca3dd4a5bd7eba140b5bac4c8c0f449`；formal manifest 文件 / self-hash 分别为 `30188651f40818c987157b637d4e1ce6ee16ff8d7952cba00356bdfdd83a618b` / `ea93a41af76996d472f42a2a1f41e078a37df0b5c6032874a88fa14a6ab81d0f`，adapter SHA256 为 `8571d3de6095a6da118943c54a5a1fffcacd1fe4c22933f71c3fa1839a12703d`，共享 atom 模块 SHA256 为 `04985b5adc97275f940c29bbb584e8f0b6d1dd62cd5ba543d1c71c4a64ae6cc5`，summary / asset records / verification SHA256 分别为 `6126f078efdca229ca6a72183b825d51636c02db9f39478a9fd5b71d622f9cfd` / `654a073bc4244fa681d00463e78b668cb9ab88d6c8bf5a06f0bdb736d0ec3b5b` / `a06ebe7ec2fd3efc989f0e6dc604b7d704f3f05fe7212d835885989c9a66dde0`。该运行冻结的是评测开始前写入输出目录的 protocol snapshot，SHA256 `6031a52c20662f6e06b6a298ea7e3fc6d409ccefd0d873601aca5c510c911e85`；本段属于运行后的报告更新。

| PhysX-Mobility category macro | Visual-bearing Collision Coverage ↑ | Joint-limit Portability ↑ | Joint Dynamics Coverage ↑ | Placeholder-mass Incidence ↓ |
|---|---:|---:|---:|---:|
| 98 observed categories, unweighted mean | 0.00% | 99.99% | 0.00% | N/E |

PhysX-Mobility 行为 overall micro average。正式运行的抽样样本是 `jq -r '.assets[].asset_id' exp/runtime/table1_physx_mobility/manifest.json` 的全部 800 项（Table 1 manifest 文件 bytes SHA-256 `644c27fee308f211c76a6ff096216538e217d3397de5ee52c8f01aead866db6c`），严格保持原顺序，没有重新抽样、替换失败项或按结果筛选；与 Table 2 formal 运行（`table2_urdf_physx_mobility_table5cohort_n800_20260819T091324Z`，manifest self-hash `4218abd2bcb0d67acacb116f7ab03825b69b982afe4368a793c22567524f738c`）是同一 cohort（Table 5 cohort，salt `arti-skill-table5-physx-mobility-n800-v1`，从 `N_release = 2,024` 中按 rank_sha256 升序取前 800）。`N_eval = 800`；PhysX-Mobility 没有冻结的 Table 3 运行，关节分母在评测前由独立的 XML 扫描冻结（逐资产统计 staged primary URDF 中声明的非 fixed joints，解析失败即中止冻结），`J_eval = 3,809`，覆盖 98 个 observed raw category。parse gate 为「XML well-formed 且根元素为 robot」的冻结 precedent，800 / 800 通过。PhysX-Mobility 的磁盘布局是扁平的（`urdf/<id>.urdf` 与 `partseg/<id>/objs/...`），Table 2 formal 运行使用的临时 staging 目录已不存在；本运行按冻结 Table 2 `package_binding.files` 将每个资产重新 stage 为输出目录内的稀疏 package（硬链接，失败时复制），并在评测前做顺序无关的逐文件 bytes 级校验（相同路径集合、相同 size 与 SHA-256），800 / 800 一致，primary URDF SHA-256 亦 800 / 800 匹配。

共享 atom 模块在本轮冻结为 `lam-supplementary-static/v1.2`（SHA-256 `ac77a014a513cd7d0fa675e7aa46dcaf14433dbb7f01a47895c0010ea1bc3a73`），方法无关 wrapper `table2_supplementary_static` SHA-256 为 `56e4d831f4a162ebea9dfa493401fe6ca2b5a1598b45933a364c9734de76fc72`（与 Artiverse / LAM / Articraft-10K 正式运行相同）。相对 v1.1 的唯一修订是统一资源解析规则升级为 POSIX dot-segment 归一化：`.` 段丢弃、内部 `..` 段与前段抵消，带前导 `..` 的引用保留归一化形式、由解析时的 package containment 严格判定是否越界——PhysX-Mobility 的全部 33,434 个 mesh 引用均为 `./../partseg/<id>/objs/...` 形式（由 `urdf/` 子目录内的 URDF 声明，归一化后仍在 package 内），Table 2 formal 运行的 Resource Resolution 800 / 800 已确立其可解析 precedent。该修订在查看任何 PhysX-Mobility 指标结果前冻结。v1.2 在不含 `..` 的路径上与 v1.1 完全一致；Artiverse / LAM / Articraft-10K 的冻结记录中没有任何 `..` 路径 issue，PartNet-Mobility 唯一的 `..` 引用是逃逸 package 的嵌套 mtl 纹理引用（`../images/...`，v1.2 下仍被拒且本就只记为 provenance issue），因此四个已评测方法的全部数值在 v1.2 下逐位不变。

800 / 800 个资产全部完成评测，0 error、0 timeout、0 binding failure。`Visual-bearing Collision Coverage` 为 0 / 800：全部 800 个 URDF 均不声明任何 `<collision>` 元素（33,434 个 mesh 引用全部位于 `<visual>` geometry），5,477 个 visual-bearing link 无一具备可加载 collision geometry（link-micro 0 / 5,477），按冻结定义 fail closed；link extraction 在 800 / 800 个资产上 complete。该结果与 Table 2 按全部声明 link 计算的 `Collision Coverage` 0 / 800 一致。`Joint-limit Portability` 为 3,808 / 3,809（99.97%）：2,745 个 `prismatic` 与 1,063 个 `revolute` joint 全部通过；唯一失败为 1 个 `floating` joint，按冻结 per-type mapping 属 unsupported mapping，保留为失败。`Joint Dynamics Coverage` 为 0 / 3,809：没有任何关节声明 `<dynamics>` 元素。`Placeholder-mass Incidence` 为 `N/E`（reason `placeholder_registry_empty`）：冻结注册表为空（与 Artiverse / LAM / PartNet-Mobility precedent 一致），complete-inertial coverage 按测得的 10,086 / 10,086 个 declared link 报告。

运行环境为 Python 3.12.3，workers = 16，单资产超时 900 秒，评测阶段墙钟约 604 秒；独立验证 22 / 22 checks PASS。证据见 [summary.json](runtime/table2sup_urdf_physx_mobility_table5cohort_n800_20260819T100246Z/summary.json)、[summary.md](runtime/table2sup_urdf_physx_mobility_table5cohort_n800_20260819T100246Z/summary.md)、[frozen_manifest.json](runtime/table2sup_urdf_physx_mobility_table5cohort_n800_20260819T100246Z/frozen_manifest.json)、[asset_records.jsonl](runtime/table2sup_urdf_physx_mobility_table5cohort_n800_20260819T100246Z/asset_records.jsonl)、[verification.json](runtime/table2sup_urdf_physx_mobility_table5cohort_n800_20260819T100246Z/verification.json) 和 [protocol_snapshot.md](runtime/table2sup_urdf_physx_mobility_table5cohort_n800_20260819T100246Z/protocol_snapshot.md)。frozen manifest self-hash 为 `a6c933bd31798be2d675871cab3f080144ee7c7f6ed3f147ade419ce4d40dcc3`，protocol snapshot SHA256 为 `be72edb4919e421f670aac99b20db19648b21d16a968612bf4d4aff5189c2016`，runner SHA256 为 `505c8c0738652c6ea7e582a7104ac61104112c7615963cee5993edf5fc296817`，verifier SHA256 为 `8ce64f4b4f3c428a2a5cfede5d21a24a4c07d220da141261f333dc06bea0a7b7`。该运行冻结的是评测开始前写入输出目录的 frozen manifest 与 protocol snapshot；本段属于运行后的报告更新。

| Ours-500K category macro | Visual-bearing Collision Coverage ↑ | Joint-limit Portability ↑ | Joint Dynamics Coverage ↑ | Placeholder-mass Incidence ↓ |
|---|---:|---:|---:|---:|
| 12 observed categories, unweighted mean | 100.00% | 100.00% | 16.67% | N/E |

Ours-500K 行为 overall micro average。正式运行的抽样样本是 `jq -r '.assets[].asset_id' exp/runtime/table1_ours_500k/manifest.json` 的全部 500 项（Table 1 manifest 文件 bytes SHA-256 `bc3eb334b1fc1c57378e50e7c2fab5d765a7599db8b3e82bc1d91536570b7c06`，`FULL_ACQUIRED_RELEASE_SAMPLE_NO_SUBSAMPLING`），严格保持原顺序，没有重新抽样、替换失败项或按结果筛选；与 Table 2 / Table 3 Ours-500K 正式运行是同一 cohort。`N_eval = 500`、`J_eval = 2,467`（取冻结 Table 3 asset records 的 `declared_joint_count`，总和冻结校验一致），覆盖 12 个 observed raw category。parse gate 为「XML well-formed 且根元素为 robot」的冻结 precedent，500 / 500 通过。package 持久位于 `exp/Brain/extracted/arti_cabinet_drawer_geometry_500_20260813`，评测前逐资产按冻结 Table 2 records 的 walk-order content-manifest SHA-256 与 primary URDF SHA-256 做 bytes 级校验，500 / 500 一致。共享 atom 模块冻结为 `lam-supplementary-static/v1.2`（SHA-256 `ac77a014a513cd7d0fa675e7aa46dcaf14433dbb7f01a47895c0010ea1bc3a73`，与 PhysX-Mobility 运行逐位相同），方法无关 wrapper `table2_supplementary_static` SHA-256 为 `56e4d831f4a162ebea9dfa493401fe6ca2b5a1598b45933a364c9734de76fc72`。

500 / 500 个资产全部完成评测，0 error、0 timeout、0 binding failure。`Visual-bearing Collision Coverage` 为 500 / 500：每个资产至少含一个 visual-bearing link，且全部 3,123 个 visual-bearing link 均至少含一个资源可解析、可加载的 collision geometry（link-micro 3,123 / 3,123）；link extraction 在 500 / 500 个资产上 complete。`Joint-limit Portability` 为 2,467 / 2,467（100.00%）：320 个 `continuous`、1,339 个 `prismatic` 与 808 个 `revolute` joint 全部通过（bounded joint 具有有限 `lower < upper`，全部关节具有有限非负 `effort` 与有限正 `velocity`）。`Joint Dynamics Coverage` 为 266 / 2,467（10.78%）：217 个 `revolute` 与 49 个 `prismatic` joint 同时声明有限非负 `damping` 与 `friction`，320 个 `continuous` joint 均未声明 `<dynamics>`。`Placeholder-mass Incidence` 为 `N/E`（reason `placeholder_registry_empty`）：冻结注册表为空（与其他五个方法的 precedent 一致），complete-inertial coverage 按测得的 151 / 3,123 个 declared link（4.84%）报告。

运行环境为 Python 3.12.3，workers = 16，单资产超时 900 秒，评测阶段墙钟约 73 秒；独立验证 22 / 22 checks PASS。证据见 [summary.json](runtime/table2sup_urdf_ours_500k_table1cohort_n500_20260819T131421Z/summary.json)、[summary.md](runtime/table2sup_urdf_ours_500k_table1cohort_n500_20260819T131421Z/summary.md)、[frozen_manifest.json](runtime/table2sup_urdf_ours_500k_table1cohort_n500_20260819T131421Z/frozen_manifest.json)、[asset_records.jsonl](runtime/table2sup_urdf_ours_500k_table1cohort_n500_20260819T131421Z/asset_records.jsonl)、[verification.json](runtime/table2sup_urdf_ours_500k_table1cohort_n500_20260819T131421Z/verification.json) 和 [protocol_snapshot.md](runtime/table2sup_urdf_ours_500k_table1cohort_n500_20260819T131421Z/protocol_snapshot.md)。frozen manifest self-hash 为 `2e2a74e0ff8b57c9ca062fe111057b4be3ee4f07e879525b582929a5f2d74d14`，protocol snapshot SHA256 为 `6e7d20345a1303e018737bf9a6b6af34bcaa2c3d528508024971a94102ec8538`，runner SHA256 为 `42fb00dc5229eed6466cb3780f305b357c7761c2f78cc8a00d390842f41ef4a5`，verifier SHA256 为 `8ce64f4b4f3c428a2a5cfede5d21a24a4c07d220da141261f333dc06bea0a7b7`。该运行冻结的是评测开始前写入输出目录的 frozen manifest 与 protocol snapshot；本段属于运行后的报告更新。

---

## Table 3. Kinematic Executability

| Dataset / Outputs | Valid Range ↑ | Joint Sweep Success ↑ | Non-degenerate Motion ↑ | Subtree Consistency ↑ | FK Round-trip Error ↓ | Joint-level Pass ↑ | Strict Kinematic Pass ↑ |
|---|---:|---:|---:|---:|---:|---:|---:|
| Ours-500K | 2,467 / 2,467 (100.00%) | 2,467 / 2,467 (100.00%) | 2,467 / 2,467 (100.00%) | 2,467 / 2,467 (100.00%) | 0.000000 normalized translation / 0.000000 rad rotation (2,467 / 2,467 measured; COMPLETE) | 2,467 / 2,467 (100.00%) | 500 / 500 (100.00%) |
| Articraft-10K | 2,865 / 2,865 (100.00%) | 2,865 / 2,865 (100.00%) | 2,865 / 2,865 (100.00%) | 2,855 / 2,865 (99.65%) | 0.000000 normalized translation / 2.980232e-8 rad rotation (2,865 / 2,865 measured; COMPLETE) | 2,845 / 2,865 (99.30%) | 795 / 800 (99.38%) |
| LAM released outputs | 2,382 / 2,395 (99.46%) | 2,005 / 2,395 (83.72%) | 2,000 / 2,395 (83.51%) | 2,005 / 2,395 (83.72%) | 0.000000 normalized translation / 0.000000 rad rotation (2,005 / 2,395 measured; PARTIAL) | 2,000 / 2,395 (83.51%) | 692 / 800 (86.50%) |
| Artiverse | 3,875 / 3,875 (100.00%) | 3,854 / 3,875 (99.46%) | 3,782 / 3,875 (97.60%) | 3,854 / 3,875 (99.46%) | 0.000000 normalized translation / 0.000000 rad rotation (3,854 / 3,875 measured; PARTIAL) | 3,782 / 3,875 (97.60%) | 762 / 800 (95.25%) |
| PartNet-Mobility | 4,078 / 4,078 (100.00%) | 4,078 / 4,078 (100.00%) | 4,069 / 4,078 (99.78%) | 4,076 / 4,078 (99.95%) | 0.000000 normalized translation / 2.107342e-8 rad rotation (4,078 / 4,078 measured; COMPLETE) | 4,066 / 4,078 (99.71%) | 793 / 800 (99.12%) |
| PhysX-Mobility | 3,808 / 3,809 (99.97%) | 3,807 / 3,809 (99.95%) | 3,807 / 3,809 (99.95%) | 3,807 / 3,809 (99.95%) | 0.000000 normalized translation / 2.107342e-8 rad rotation (3,807 / 3,809 measured; PARTIAL) | 3,806 / 3,809 (99.92%) | 797 / 800 (99.62%) |
| SketchMobility | TBD | TBD | TBD | TBD | TBD | TBD | TBD |

### Table 3 evaluation states

- 每个有界关节在 lower、upper 以及二者之间 `K = 21` 个均匀状态上测试。
- continuous joint 在冻结的标准区间内测试，例如 `[-pi, pi]`。
- 每次只改变一个关节，其余关节固定在声明的初始状态。
- 所有位姿误差除以 object bounding-box diagonal，得到尺度归一化结果。

### Table 3 metric definitions

| Metric | Definition |
|---|---|
| `Valid Range` | 关节具有非空、有限且可采样的测试区间。 |
| `Joint Sweep Success` | 关节的全部测试状态均可由 FK 引擎执行，且所有 link transform 有限。 |
| `Non-degenerate Motion` | lower 与 upper 之间的 descendant link 位姿变化超过预注册的平移或旋转阈值。 |
| `Subtree Consistency` | 驱动一个关节时，只有该关节的 descendant subtree 发生位姿变化。 |
| `FK Round-trip Error` | 执行 `q0 -> q1 -> q0` 后，所有 link 相对初始位姿的最大归一化误差。 |
| `Joint-level Pass` | 同一关节通过 Valid Range、Sweep、Non-degenerate、Subtree 和 Round-trip 检查。 |
| `Strict Kinematic Pass` | 同一资产中的全部非 fixed joints 均达到 Joint-level Pass。 |

Ours-500K 行为 overall micro average。正式运行严格复用 Table 2 formal manifest（[manifest.json](runtime/table2_urdf_ours_500k_table1cohort_n500_20260819T094919Z/manifest.json) 文件 SHA256 `f6f2eb2e9a5a0b257d2843674e987946a9d014274348784018540772f2660b71`、self-hash `8b1bdb53bef17ac104bb42daa331899dfa498020be131fe5727f16df0f0427fa`）中 `.assets[]` 的全部 500 项及 `selection_index` 原序，没有重新抽样、替换或按结果筛选；cohort 即 Table 1 的 `FULL_ACQUIRED_RELEASE_SAMPLE_NO_SUBSAMPLING`，`N_eval = 500`、`J_eval = 2,467`（808 revolute、1,339 prismatic、320 continuous）。类别取归档内模板声明目录名（`raw_category`），覆盖 12 个 observed category，无外部联结。每关节按冻结 K=21 个状态测试，continuous joint 使用冻结区间 `[-pi, pi]`，q0 为 zero clipped to declared interval；位姿误差以 q0 visual/collision 几何联合 AABB diagonal 归一化。

500 / 500 个资产完成评测，最终 0 error、0 timeout。首轮有 8 个资产（`drawer_cabinet_with_sliding_drawers` seed 32--39）的子进程因宿主机瞬时资源耗尽（`BlockingIOError`，fork 压力）被中止，未产生任何评测结果；在冻结 manifest 字节级不变的前提下以 `--resume-frozen` 重跑该 8 项后全部完成，原始 env-error 记录备份为 [asset_records.env_error_pre_retry.jsonl](runtime/urdf_table3_ours_500k_table2_n500_20260819T101000Z/asset_records.env_error_pre_retry.jsonl)。FK round-trip 在 2,467 / 2,467 个关节上完成测量（coverage COMPLETE），最大归一化平移误差 0.000000、最大旋转误差 0.000000 rad。12 类等权 category-level macro average 为 Valid Range 100.00%、Joint Sweep Success 100.00%、Non-degenerate Motion 100.00%、Subtree Consistency 100.00%、Joint-level Pass 100.00%、Strict Kinematic Pass 100.00%。证据见 [summary.json](runtime/urdf_table3_ours_500k_table2_n500_20260819T101000Z/summary.json)、[summary.md](runtime/urdf_table3_ours_500k_table2_n500_20260819T101000Z/summary.md)、[manifest.json](runtime/urdf_table3_ours_500k_table2_n500_20260819T101000Z/manifest.json)、[asset_records.jsonl](runtime/urdf_table3_ours_500k_table2_n500_20260819T101000Z/asset_records.jsonl) 和 [checkpoint.json](runtime/urdf_table3_ours_500k_table2_n500_20260819T101000Z/checkpoint.json)；Table 3 manifest 文件 SHA256 为 `28f4106d8815a13719e303dfb06091f39b2427c8f4310d85a77746a6856b2997`、self-hash 为 `99226cba0df82016f752b1bebb3f3a354f3b6dfd9483ab9be2f6d78ee35d5409`，选中 asset IDs SHA256 为 `dcd19530ff3a3546fa149db58f331a042d5c3326f1b4fa1e5580914952e79289`，adapter SHA256 为 `4262d29a847a188c839cd43375f912f2c5feb7ac1f1b5af76a646a79c8291dc6`，FK core SHA256 为 `0da075f077ce13c78bb6b4ee66b0abe77668ccf7bb3c105660b321e667fc2acf`（与 Articraft-10K Table 3 所用 shared evaluator 相同），environment（Python 3.12.3、`.venv_low_medium`、numpy 2.5.1、单线程子进程）SHA256 为 `914c7aa17ed24651138bba071192053035ac689d549217e301f39e0a1d6f8a87`。该运行冻结的是结果写回前的 protocol SHA256 `81cfa85e9e2aaefa2720a62f3fc9d09c7454157671527c41409a1f29c141d7c6`；本段属于运行后的报告更新。该离散 FK 结果不证明 joint semantic correctness、连续配置空间、碰撞安全或动力学有效性。

Articraft-10K 行为 overall micro average。正式运行严格复用 `exp/runtime/table2_urdf_articraft10k_n800_seed20260813_20260813T145915Z/manifest.json` 中 `.records[].package` 的全部 800 项及原始顺序，没有重新抽样或按结果筛选。样本来自本地冻结的 `camvsl/Articraft-10K@3c79d5a05bb7cb6bf7bfee5e090176636ee3ac65` 发布集（`N_release = 9,996`），seed 为 `20260813`，共声明 `J_eval = 2,865` 个非 fixed joints。按 `asset_id` 联结官方 `record.json.category_slug` 后，cohort 覆盖 222 个 observed category。该结果是 frozen random cohort，不是 Full Release Cohort 或 Shared-category Balanced Cohort。

800 / 800 个 URDF 均完成 XML 解析，具有 valid rooted tree，q0 下 visual/collision geometry 联合 AABB 的尺度推导也全部完成。运行状态为 800 个 `completed`、0 error、0 timeout。冻结 manifest 中的 800 个 package binding、`model.urdf` 哈希和类别联结均在评测前校验，子进程在评测前后再次检查 package binding。

`FK Round-trip Error` 在 2,865 / 2,865 个关节上完成测量，最大归一化平移误差为 0.000000，最大旋转误差为 `2.980232e-8 rad`，因此 coverage 为 `COMPLETE`，而不是 `PARTIAL`。冻结 round-trip 阈值为 `1e-9 rad`：20 个关节未通过该阈值，其中 10 个同时未通过 Subtree Consistency，共影响 5 个资产的 Strict Kinematic Pass。这些关节和资产均保留在分母中。

| Articraft-10K category macro | Valid Range ↑ | Joint Sweep Success ↑ | Non-degenerate Motion ↑ | Subtree Consistency ↑ | FK Round-trip Error ↓ | Joint-level Pass ↑ | Strict Kinematic Pass ↑ |
|---|---:|---:|---:|---:|---:|---:|---:|
| 222 observed categories, unweighted mean | 100.00% | 100.00% | 100.00% | 99.68% | N/E | 99.19% | 99.27% |

Articraft-10K 证据见 [summary.json](runtime/urdf_table3_articraft10k_table2_n800_20260814T040300Z/summary.json)、[summary.md](runtime/urdf_table3_articraft10k_table2_n800_20260814T040300Z/summary.md)、[manifest.json](runtime/urdf_table3_articraft10k_table2_n800_20260814T040300Z/manifest.json) 和 [asset_records.jsonl](runtime/urdf_table3_articraft10k_table2_n800_20260814T040300Z/asset_records.jsonl)。Table 2 cohort manifest 文件 SHA256 为 `13c47e2b2affadb951a01cab826bae139852fca5769e99ec081cc916ffa6373d`，内容自哈希为 `576852cb6da00775e1c51360b82b4be40e0a614e4fb0cfb1bae066912eed56a3`，选中资产 ID 哈希为 `79c44441600077513d3cde1cda8fef38324e1a0ee660730b860d5313f0ae9784`，类别联结哈希为 `0305569f49d2aa1acb72fbb7bc8dcaf68ca3dd4a5bd7eba140b5bac4c8c0f449`。formal manifest self-hash 为 `9cba009db52b2fc40d8e31468fd6bad9b1a6551199f4ffaf4b218dc9280b8800`，adapter SHA256 为 `535c8a48a4fdf76f4f8760c9188892b2386c15856d242ddd1c99c653ad5fa560`，shared evaluator SHA256 为 `0da075f077ce13c78bb6b4ee66b0abe77668ccf7bb3c105660b321e667fc2acf`。该运行冻结的 protocol SHA256 为 `b6cee5d4c818d462cfea47c6413b84856e676e3d4b227628dd749b9c8c8ce78c`；运行后文档还有其他表格的并行写入，因此当前文件哈希与运行时哈希不同，本段属于运行后的报告更新。

LAM 行为 overall micro average。正式运行从本地冻结的 `YipengGao/Articulated-Object-Code@28cec4f5be7e34fd4d586879ecfcb67f7c5e4cc0` 发布 manifest 的全部 3,217 条记录中，以 seed `20260813` 在查看评测结果前确定性抽取 `N_eval = 800`；抽样单位为唯一的 `(tier, rel_path)`，不按 `viable`、`loads_only` 或 `broken` 标签预筛。冻结样本包含 621 个 `viable`、75 个 `loads_only` 和 104 个 `broken` 资产，覆盖 305 个 observed category，共声明 `J_eval = 2,395` 个非 fixed joints。该结果是 frozen global random cohort，不是 Full Release Cohort 或 Shared-category Balanced Cohort。

800 / 800 个 URDF 均完成 XML 解析，733 / 800 具有 valid rooted tree；运行状态为 798 个 `completed`、2 个 retained asset error、0 timeout。两条 error 均来自资产声明了当前冻结 FK 协议不支持的 `floating` joint，资产及其声明关节仍保留在分母中。q0 下 visual/collision geometry 联合 AABB 的尺度推导为 719 个 `COMPLETE`、12 个 `NOT_EVALUABLE`（mesh 缺失或顶点非有限）、2 个因 initial FK 失败而不可得、67 个因 invalid tree 而不可得。

`FK Round-trip Error` 的 0.000000 只是在成功完成 round-trip 的 2,005 / 2,395 个关节上观测到的最大归一化平移误差和最大旋转误差；其余 390 个关节未完成该测量，因此明确报告为 `PARTIAL`，并在 `Joint-level Pass` 中按失败保留。该离散 FK 结果不证明 joint semantic correctness、连续配置空间、碰撞安全或动力学有效性。

| LAM category macro | Valid Range ↑ | Joint Sweep Success ↑ | Non-degenerate Motion ↑ | Subtree Consistency ↑ | FK Round-trip Error ↓ | Joint-level Pass ↑ | Strict Kinematic Pass ↑ |
|---|---:|---:|---:|---:|---:|---:|---:|
| 305 observed categories, unweighted mean (302 with declared movable joints) | 96.19% | 81.43% | 80.11% | 81.43% | N/E | 80.11% | 79.32% |

LAM 证据见 [summary.json](runtime/urdf_table3_lam_n800_seed20260813_20260814T022400Z_v3/summary.json)、[summary.md](runtime/urdf_table3_lam_n800_seed20260813_20260814T022400Z_v3/summary.md)、[manifest.json](runtime/urdf_table3_lam_n800_seed20260813_20260814T022400Z_v3/manifest.json) 和 [asset_records.jsonl](runtime/urdf_table3_lam_n800_seed20260813_20260814T022400Z_v3/asset_records.jsonl)。冻结 selected-cohort SHA256 为 `643aa5b76ac61f57dd943bee26444a3525c01201a8dff3443763a7fd8d8267d3`，formal manifest self-hash 为 `f8f7fe4da5634d4f806e793c0da919689eab25be1ce0bbed7e2232f3453d15c2`，evaluator SHA256 为 `0da075f077ce13c78bb6b4ee66b0abe77668ccf7bb3c105660b321e667fc2acf`。该运行冻结的是结果写回前的 protocol SHA256 `8115a160ab229aa52f3c98498a652851bc27eabc96feb17e1463e61541f6cf22`；本段属于运行后的报告更新。

Artiverse 行为 overall micro average。正式运行严格复用 `exp/runtime/table1_artiverse/manifest.json` 中 `.assets[].manifest_root` 的全部 800 项及原始顺序，没有重新抽样或按结果筛选。本地数据为 `PRE_RELEASE_SUBSET`（`N_release = 3,544`），固定全局样本的 seed 为 `20260813`，覆盖 67 个 observed `raw_category`，共声明 `J_eval = 3,875` 个非 fixed joints。该 cohort 不是 Full Release Cohort 或 Shared-category Balanced Cohort。

800 / 800 个 URDF 均完成 XML 解析，运行状态全部为 `completed`，0 error、0 timeout。797 / 800 个资产具有 valid rooted tree；其余 3 个资产的 joint graph 不连通或含环，其 21 个声明关节仍保留在所有关节级分母中。q0 下 visual/collision geometry 联合 AABB 的尺度推导在 797 个 valid-tree 资产上完成，上述 3 个 invalid-tree 资产无法推导。

`FK Round-trip Error` 的 0.000000 只是在完成 round-trip 的 3,854 / 3,875 个关节上观测到的最大归一化平移误差和最大旋转误差；其余 21 个关节未完成该测量，因此报告为 `PARTIAL`。另有 72 个关节未达到冻结的 non-degenerate motion 阈值。两类失败均在 `Joint-level Pass` 和 `Strict Kinematic Pass` 中 fail closed。

| Artiverse category macro | Valid Range ↑ | Joint Sweep Success ↑ | Non-degenerate Motion ↑ | Subtree Consistency ↑ | FK Round-trip Error ↓ | Joint-level Pass ↑ | Strict Kinematic Pass ↑ |
|---|---:|---:|---:|---:|---:|---:|---:|
| 67 observed raw categories, unweighted mean | 100.00% | 99.85% | 95.66% | 99.85% | N/E | 95.66% | 91.09% |

Artiverse 证据见 [summary.json](runtime/urdf_table3_artiverse_table1_n800_20260814T031800Z/summary.json)、[summary.md](runtime/urdf_table3_artiverse_table1_n800_20260814T031800Z/summary.md)、[manifest.json](runtime/urdf_table3_artiverse_table1_n800_20260814T031800Z/manifest.json) 和 [asset_records.jsonl](runtime/urdf_table3_artiverse_table1_n800_20260814T031800Z/asset_records.jsonl)。Table 1 cohort manifest SHA256 为 `f74575692b87605699c4f349186c4660d691c91bef39562bb976baf22ae72a8c`，选中资产 ID 哈希为 `118038a746cafb91251afde5eb4f1164915d141acb3b529ea721a9d376bde3fa`，formal manifest self-hash 为 `6cf365a1b51d87c3ca3bb5709e65e262e31a74aca6763b73dcd96b129b8d0f99`，adapter SHA256 为 `28c1365099fff94d1cf6ce2f8607f522a83d76d3c030e5ec1048c4389e653c36`，shared evaluator SHA256 为 `0da075f077ce13c78bb6b4ee66b0abe77668ccf7bb3c105660b321e667fc2acf`。该运行冻结的是结果写回前的 protocol SHA256 `be3813e1b40b4fb8e2ee5cf9bec89aa3b83d7dcca3050a0c6c3eeb3097c36ed1`；本段属于运行后的报告更新。

PartNet-Mobility 行为 overall micro average。正式运行严格复用 [Table 4 frozen manifest](runtime/urdf_table4_partnet_mobility_n800_20260813/frozen_manifest.json) 中 `.items[].dataset_id` 的全部 800 项及原始顺序，没有重新抽样或按结果筛选。本地冻结 release roster 为 `N_release = 2,347`，固定样本为 `N_eval = 800`，共声明 `J_eval = 4,078` 个非 fixed joints，覆盖 46 个 observed raw category。选择策略为冻结的 hash-ranked sample，salt 为 `urdf-sim-ready-table4-partnet-mobility-n800-v1:20260813`；该 cohort 不是 Full Release Cohort 或 Shared-category Balanced Cohort。

800 / 800 个 URDF 均通过冻结运动学评测器的 XML 解析并形成 valid rooted tree；运行状态为 800 个 `completed`、0 error、0 timeout，4,078 个关节均完成全部 `K = 21` 个测试状态。q0 下 visual/collision geometry 联合 AABB 的尺度推导为 787 个 `COMPLETE`、13 个 `NOT_EVALUABLE`；后者包含共 79 个缺失的 collision mesh 引用，所有资产及关节均保留在冻结分母中。

`FK Round-trip Error` 在 4,078 / 4,078 个关节上完成测量，最大平移残差为 0.000000，最大旋转残差为 `2.1073424255447017e-8 rad`，coverage 为 `COMPLETE`。冻结 round-trip 旋转阈值为 `1e-9 rad`：资产 `103010` 的 3 个关节未通过，其中 2 个同时未通过 Subtree Consistency。另有 6 个缺资源资产中的 9 个 prismatic joints 因尺度不可得且没有旋转运动，按冻结的 Non-degenerate Motion 规则 fail closed；合计 12 个 joint-level failure、7 个 strict asset failure，均未替换。13 个尺度不可得资产的 round-trip 平移残差也均为 0；0 在任意有效尺度下归一化后仍为 0，但这不表示这些资产获得了可用的非零平移归一化尺度。

| PartNet-Mobility category macro | Valid Range ↑ | Joint Sweep Success ↑ | Non-degenerate Motion ↑ | Subtree Consistency ↑ | FK Round-trip Error ↓ | Joint-level Pass ↑ | Strict Kinematic Pass ↑ |
|---|---:|---:|---:|---:|---:|---:|---:|
| 46 observed raw categories, unweighted mean | 100.00% | 100.00% | 99.77% | 99.89% | N/E | 99.60% | 99.44% |

PartNet-Mobility 证据见 [summary.json](runtime/urdf_table3_partnet_mobility_table4_n800_20260814T070118Z/summary.json)、[summary.md](runtime/urdf_table3_partnet_mobility_table4_n800_20260814T070118Z/summary.md)、[manifest.json](runtime/urdf_table3_partnet_mobility_table4_n800_20260814T070118Z/manifest.json) 和 [asset_records.jsonl](runtime/urdf_table3_partnet_mobility_table4_n800_20260814T070118Z/asset_records.jsonl)。Table 4 cohort manifest 文件 SHA256 为 `2ff015ee6bb377ce693126b52dd632a7565a3eaa9f0007e26122a1bb4ab99900`，items SHA256 为 `5f4d0eaa7d50087edc3491a92868950a035dabcb2ad5f8c4d4970aa4c890e5e3`，ordered asset IDs SHA256 为 `ef6cb964e50dc712280256c5b2f675cc2c957095c3553b21845d3562a5011883`，candidate pool SHA256 为 `0203a510202510cea7e469048e84b133bd65ccbc6e1e3aa90c9bfeea7807959d`，inventory SHA256 为 `e281119f870bb6bae9599c3edc02de0a42a257e0d433335361d4a774592c1b5a`，archive SHA256 为 `b47247a44246111e8d09f2c0e64b4012ae35e0dcf4bb55f68a05b604455119ff`。formal manifest 文件 SHA256 为 `1ef65497378f384f181d6f1411cb0682b11c3584293ca3d7647714fb2c15f345`，self-hash 为 `96201e302c7f27d696473d83713342bc6ed4b056b44d5e0eb5376e1d83e11b26`，adapter SHA256 为 `dde9049fa786bce855def78e11a054c17252dc32b759ddb8a7b2af82acc2b4b2`，shared evaluator SHA256 为 `0da075f077ce13c78bb6b4ee66b0abe77668ccf7bb3c105660b321e667fc2acf`，frozen Table 4 contract SHA256 为 `e710d15cb79c50506487ff1335a88591bb58c11cf726c71198103c05f6d01ff0`。本地数据状态为 `LOCAL_COMPLETE_PROVENANCE_LIMITED`：固定源为 `sapien-sim/PartNetMobility@ee0aa3ef1df16181d76d83f7415aa8c94ed1da8f`，但 gated revision 的对象 bytes 未与本地文件直接认证。该运行冻结的是结果写回前的 protocol SHA256 `de6f83e93ddcc5f6414561aaa17e7dfd78b1f323b84ce1430cd4b3173707f813`；本段属于运行后的报告更新。

| PhysX-Mobility category macro | Valid Range ↑ | Joint Sweep Success ↑ | Non-degenerate Motion ↑ | Subtree Consistency ↑ | FK Round-trip Error ↓ | Joint-level Pass ↑ | Strict Kinematic Pass ↑ |
|---|---:|---:|---:|---:|---:|---:|---:|
| 98 observed categories, unweighted mean | 99.99% | 99.99% | 99.99% | 99.99% | N/E | 99.87% | 99.83% |

PhysX-Mobility 行为 overall micro average。正式运行严格复用 Table 5 冻结 receipt set（[manifest.json](runtime/table5_physx_mobility_n800_v2/manifest.json)，文件 SHA256 为 `ccb54f4b726fe717efd28a37948e6b92bac994a2c0ba8fb4ea9ac4548d3a9882`，cohort hash 为 `a9c9c710d9617dea366696603984e330780ce177fead2a34c60410588cc1273c`）中全部 800 行及既有 rank 顺序，没有重新抽样、替换失败项或按结果筛选。`N_eval = 800`，`J_eval = 3,809` 个声明非 fixed joints（1,063 revolute、2,745 prismatic、1 floating），覆盖 98 个 observed category。运行状态为 799 个 `completed`、1 个 retained asset error、0 timeout：资产 `11854` 声明了冻结 FK 协议不支持的 `floating` joint，initial FK fail closed，其 2 个声明 movable joints 仍保留在分母中（与 LAM 轮 floating 资产同一 fail-closed precedent）。

全部指标由同一冻结 FK 核心（`run_urdf_table3_lam.py`，shared evaluator SHA256 `0da075f077ce13c78bb6b4ee66b0abe77668ccf7bb3c105660b321e667fc2acf`，与 Articraft-10K / LAM / Artiverse / PartNet-Mobility 四轮完全一致）计算：K=21 均匀 sweep、每次只驱动一个关节、阈值 1e-6（non-degenerate motion）/ 1e-9（subtree unchanged 与 round-trip）。官方 PhysX-Mobility 发布几何中资源位于扁平 `urdf/` 目录的 sibling 目录 `partseg/`，而冻结核心的 kinematic scale 要求 mesh 引用在 URDF 目录内解析，因此每个资产被 staging 为自包含 evaluation package：资源按冻结 manifest 行哈希逐字节校验复制（与官方归档字节级绑定一致），URDF 仅将 mesh `filename` 属性相对化进 package；joint/link 声明在冻结前经结构化不变性校验，FK 输入（joint types、origins、axes、limits）与 mesh 顶点数据与发布资产完全一致。

Kinematic scale（q0 下 visual geometry 联合 AABB diagonal）在 799 / 800 个资产上 `COMPLETE`，1 个为 `UNAVAILABLE_INITIAL_FK`（上述 floating 资产）。`FK Round-trip Error` 在 3,807 / 3,809 个关节上完成测量，最大归一化平移误差为 0.000000，最大旋转误差为 `2.107342e-8 rad`，coverage 为 `PARTIAL`；冻结 round-trip 阈值为 `1e-9 rad`，资产 `100971` 的 `joint_revolute_l_1_abstract_1_0` 未通过该阈值并在 Joint-level Pass 中 fail closed。资产 `12562` 声明 0 个非 fixed joints（仅 fixed joints），按冻结语义其 `Strict Kinematic Pass` fail closed。共 3 个资产未通过 Strict Kinematic Pass，均保留在分母中。该离散 FK 结果不证明 joint semantic correctness、连续配置空间、碰撞安全或动力学有效性。证据见 [summary.json](runtime/urdf_table3_physx_mobility_table5cohort_n800_20260819T102939Z/summary.json)、[summary.md](runtime/urdf_table3_physx_mobility_table5cohort_n800_20260819T102939Z/summary.md)、[manifest.json](runtime/urdf_table3_physx_mobility_table5cohort_n800_20260819T102939Z/manifest.json) 和 [asset_records.jsonl](runtime/urdf_table3_physx_mobility_table5cohort_n800_20260819T102939Z/asset_records.jsonl)。formal manifest 文件 SHA256 为 `427d0418556e00e95e2c797d5523894c24f69d4553e2ab1c5af082aa57d8b322`，self-hash 为 `1dfc4c80707690bfcafe7e7b26d84f8d592ce059e49b80621b63192f61189da6`，runner SHA256 为 `8323711cfcfac7efa6f7786319dc4d12bb4cc08c39e5b5035fba850478df8bfd`，summary / asset records SHA256 分别为 `d0c93d3957aa17212fa2bf73a5e042f7d920aea99f0596f28df0727f842498c6` / `e97db210faf9a9e38a7083274a6553e1f10648b5df56b606e0ea068e079b614c`。该运行冻结的是子进程启动前写入输出目录的 protocol snapshot SHA256 `ec5ae6b97a625b9c1bb976f80c20d82503e0c9bd06109f69c33f9d3f46b71727`；本段属于运行后的报告更新。

该表只证明 URDF 描述的运动学可以执行，不证明 joint type、axis、origin 或 limit 与真实物体语义一致。

---

## Table 4. Collision and Mechanical Clearance

| Dataset / Outputs | Rest All-pair CF ↑ | Rest Non-adjacent CF ↑ | Single-joint Sweep CF ↑ | Multi-joint Sobol CF ↑ | Collision-state Rate ↓ | AOR ↓ | Max Penetration ↓ | Collision-free Range ↑ | Strict Collision Pass ↑ |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Ours-500K | 7 / 500 (1.400%) | 487 / 500 (97.400%) | 487 / 500 (97.400%) | 485 / 500 (97.000%) | 2,447 / 84,307 (2.902%) | N/E | 0.297148 (500 / 500 measured; COMPLETE) | 49,577 / 51,807 (95.696%) | 485 / 500 (97.000%) |
| Articraft-10K | 13 / 800 (1.625%) | 187 / 800 (23.375%) | 156 / 800 (19.500%) | 147 / 800 (18.375%) | 86,157 / 112,165 (76.813%) | N/E | 0.476553 (223 / 800 measured; PARTIAL) | 14,292 / 60,165 (23.755%) | 147 / 800 (18.375%) |
| LAM released outputs | 0 / 800 (0.000%) | 113 / 800 (14.125%) | 101 / 800 (12.625%) | 91 / 800 (11.375%) | 88,097 / 100,631 (87.545%) | N/E | 0.782640 (321 / 800 measured; PARTIAL) | 4,832 / 50,295 (9.607%) | 91 / 800 (11.375%) |
| Artiverse | 12 / 800 (1.500%) | 320 / 800 (40.000%) | 277 / 800 (34.625%) | 292 / 800 (36.500%) | 76,889 / 133,375 (57.649%) | N/E | 0.629995 (797 / 800 measured; PARTIAL) | 22,154 / 81,375 (27.225%) | 254 / 800 (31.750%) |
| PartNet-Mobility | 24 / 800 (3.000%) | 622 / 800 (77.750%) | 591 / 800 (73.875%) | 579 / 800 (72.375%) | 47,881 / 137,638 (34.788%) | N/E | 0.633017 (787 / 800 measured; PARTIAL) | 48,011 / 85,638 (56.063%) | 567 / 800 (70.875%) |
| PhysX-Mobility | 788 / 800 (98.500%) | 788 / 800 (98.500%) | 787 / 800 (98.375%) | 786 / 800 (98.250%) | 27,472 / 131,925 (20.824%) | N/E | 0.000000 (787 / 800 measured; PARTIAL) | 53,361 / 79,989 (66.710%) | 786 / 800 (98.250%) |
| SketchMobility | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD |

### Table 4 evaluation states

- Rest state（既有完成行）：冻结 runner 将全部可评测关节设为 `q = 0`。该历史状态只解释本表已经完成的行；若未来采用 clipped-zero 或其他 neutral configuration，必须作为新协议版本对全部方法重跑，不得与既有 q=0 结果混合汇总。
- Single-joint sweep：沿用 Table 3 的每关节 `K = 21` 个状态。
- Multi-joint states：每个至少含 1 个 non-fixed DoF 的资产使用固定种子的 `R = 64` 个 Sobol joint configurations；zero-DoF 资产不生成零维 Sobol 状态，但仍保留在资产级 Sobol / Strict 分母中并按 fail-closed 判定。
- 所有方法使用完全相同的接触距离、穿透阈值和拓扑排除规则。

### Table 4 metric definitions

| Metric | Definition |
|---|---|
| `CF` | Collision-free；允许表面接触，但任何超过冻结阈值的穿透均判为失败。 |
| `Rest All-pair CF` | rest state 中所有不同 link pair 均无超阈值穿透。 |
| `Rest Non-adjacent CF` | rest state 中排除直接 parent-child 后，其余 link pair 均无超阈值穿透。 |
| `Single-joint Sweep CF` | 全部单关节 sweep 状态均无非法穿透。 |
| `Multi-joint Sobol CF` | 全部 Sobol 多关节配置均无非法穿透。 |
| `Collision-state Rate` | 出现非法碰撞的测试配置数除以全部测试配置数。 |
| `AOR` | 发生重叠的 collision geometry 体积相对部件体积的平均比例；若无法稳定计算，报告 `N/E`，不得以包围盒重叠冒充精确体积。 |
| `Max Penetration` | 所有测试状态中的最大穿透深度，除以 object bounding-box diagonal。 |
| `Collision-free Range` | 单关节 sweep 中无非法碰撞的状态数除以全部 sweep 状态数。 |
| `Strict Collision Pass` | 同一资产通过预注册 pair policy 下的 rest、single-joint 和 multi-joint 全部测试。 |

必须同时公开 all-pair 和 non-adjacent 结果。允许接触和排除规则必须在查看方法结果前冻结，禁止为单个失败资产添加事后白名单。

Ours-500K 行为 overall micro average。正式运行严格复用 Table 2 formal manifest（文件 SHA256 `f6f2eb2e9a5a0b257d2843674e987946a9d014274348784018540772f2660b71`、self-hash `8b1bdb53bef17ac104bb42daa331899dfa498020be131fe5727f16df0f0427fa`）中 `.assets[]` 的全部 500 项及 `selection_index` 原序，没有重新抽样、替换或按结果筛选；cohort 即 Table 1 的 `FULL_ACQUIRED_RELEASE_SAMPLE_NO_SUBSAMPLING`。500 / 500 个资产均为 valid rooted tree 且 collision 资源闭包完整，共声明 2,467 个可评测非 fixed DoF（全部 range-evaluable）；PyBullet 加载成功 500 / 500，测量完成 500 / 500，0 timeout、0 child runtime failure。冻结状态分母为 84,307 = 500 个 rest + 51,807 个 single-joint（21 × 2,467）+ 32,000 个 Sobol（64 × 500 个含 DoF 资产）状态；实际执行 84,307 个（coverage 100%），无未执行状态，观测到 2,447 个碰撞状态，故 `Collision-state Rate` 分子为 2,447。`Max Penetration` 在全部 500 / 500 个资产上完成测量，最大归一化值为 `0.297148`，状态为 `COMPLETE`；归一化尺度为 PyBullet 中 q=0 时 collision shapes 的 union AABB diagonal（`pybullet_q0_collision_shape_union_aabb_v1`）。`AOR` 因未运行稳定的精确重叠体积计算而记为 `N/E`，未以包围盒重叠替代；评测只使用 collision geometry、没有 visual fallback，离散 sweep 不构成 CCD、关节语义正确性或物理动力学有效性结论。pair policy 为 all-pair 与 non-adjacent（排除直接 parent-child）双报，允许表面接触、穿透阈值 1e-6 m，无 method-specific allowlist；rest state 为全部可评测关节 q=0，single-joint 沿用 Table 3 的 K=21，multi-joint 为固定 seed 20260813 的 R=64 Sobol 配置。12 类等权 category-level macro average 为 Rest All-pair CF 0.941%、Rest Non-adjacent CF 97.879%、Single-joint Sweep CF 97.879%、Multi-joint Sobol CF 97.623%、Collision-free Range 97.927%、Strict Collision Pass 97.623%。证据见 [report.md](runtime/urdf_table4_ours_500k_table2_n500_20260819T104011Z/report.md)、[summary.json](runtime/urdf_table4_ours_500k_table2_n500_20260819T104011Z/summary.json)、[frozen_manifest.json](runtime/urdf_table4_ours_500k_table2_n500_20260819T104011Z/frozen_manifest.json)、[asset_records.jsonl](runtime/urdf_table4_ours_500k_table2_n500_20260819T104011Z/asset_records.jsonl)、[state_records.jsonl](runtime/urdf_table4_ours_500k_table2_n500_20260819T104011Z/state_records.jsonl)、[verification.json](runtime/urdf_table4_ours_500k_table2_n500_20260819T104011Z/verification.json) 和 [protocol_document_at_freeze.md](runtime/urdf_table4_ours_500k_table2_n500_20260819T104011Z/protocol_document_at_freeze.md)；冻结 Table 4 manifest 文件 SHA-256 为 `1b29d868112dcda326a08f8e3439d6b96c65833b99cc33af3bfcdb58fb4c2e24`、self-hash 为 `19f3c5e6063864f4506a9c1fd9817c183ffb59bb559c644ef570b7a647d3b735`，选中 ordered identities SHA-256 为 `2581b381b1eec2a36a7f02685b43dd3739f8f5406a2ec3e27af57803b62525f4`，adapter SHA-256 为 `e09bd4b7050bfb0e854f2462a27fdbc40ef3d59bb71040b7c73576a0fec3b84f`，collision core SHA-256 为 `e710d15cb79c50506487ff1335a88591bb58c11cf726c71198103c05f6d01ff0`（与既有 Table 4 运行相同），protocol snapshot SHA-256 为 `ec5ae6b97a625b9c1bb976f80c20d82503e0c9bd06109f69c33f9d3f46b71727`；独立验证 6 / 6 checks PASS，84,307 / 84,307 个冻结状态全部执行并闭合，本段属于运行后的报告更新。

Articraft-10K 行为 overall micro average。结果严格复用 Table 2 冻结 manifest 中全部 800 项及原始顺序，没有重新抽样或按结果筛选；来源为本地冻结的 `camvsl/Articraft-10K@3c79d5a05bb7cb6bf7bfee5e090176636ee3ac65` 发布集（`N_release = 9,996`）。该 cohort 不是 Full Release Cohort 或 Shared-category Balanced Cohort；本轮没有冻结权威类别标签，因此 category-level macro average 记为 `N/E`。800 / 800 个 URDF 均为 valid rooted tree，共声明 2,865 个可评测非 fixed DoF；223 / 800 个资产具有完整 collision coverage 并完成加载与测量，577 个 coverage 不完整资产保留且不补抽，其中 576 个为 0 / L link coverage，1 个为 2 / 3，0 timeout、0 child runtime failure。`Collision-state Rate` 采用 fail-closed 分母：共预期 112,165 个状态，实际执行 33,143 个，79,022 个未执行状态计为 non-free；实际观测到 7,135 个碰撞状态，故表中分子为 86,157。`Max Penetration` 仅在完成测量的 223 / 800 个资产上有观测值，因此报告为 `PARTIAL`；归一化尺度为 PyBullet 中 q=0 时 collision shapes 的 union AABB diagonal。`AOR` 因未运行稳定的精确重叠体积计算而记为 `N/E`，未使用包围盒重叠代替；离散 sweep 不构成 CCD、关节语义正确性或物理动力学有效性结论。证据见 [report.md](runtime/urdf_table4_articraft10k_n800_20260814/report.md)、[summary.json](runtime/urdf_table4_articraft10k_n800_20260814/summary.json)、[frozen_manifest.json](runtime/urdf_table4_articraft10k_n800_20260814/frozen_manifest.json)、[verification.json](runtime/urdf_table4_articraft10k_n800_20260814/verification.json)、[asset_records.json](runtime/urdf_table4_articraft10k_n800_20260814/asset_records.json)、[state_records.jsonl](runtime/urdf_table4_articraft10k_n800_20260814/state_records.jsonl) 和 [protocol_document_at_freeze.md](runtime/urdf_table4_articraft10k_n800_20260814/protocol_document_at_freeze.md)。Table 2 manifest SHA-256 为 `13c47e2b2affadb951a01cab826bae139852fca5769e99ec081cc916ffa6373d`，冻结 selected ordered ID SHA-256 为 `79c44441600077513d3cde1cda8fef38324e1a0ee660730b860d5313f0ae9784`，冻结 Table 4 manifest 文件 SHA-256 为 `6b4275cf3da29244af70c04acecd87094f0c158dee992db20b04e90c05292c20`、self-hash 为 `1c6ba7d9e19818580fe8573cf95bb1d065bf2235d0699070516888520f86d7b6`，runner SHA-256 为 `1e04f5a70a3b3b51f21ce9471c1ee52ae2fd09cc9c6bd1049b50e382a9cb0648`，collision core SHA-256 为 `e710d15cb79c50506487ff1335a88591bb58c11cf726c71198103c05f6d01ff0`。该运行冻结的是结果写回前的 protocol snapshot SHA-256 `be3813e1b40b4fb8e2ee5cf9bec89aa3b83d7dcca3050a0c6c3eeb3097c36ed1`；最终验证 31 / 31 checks PASS，800 / 800 个 authoritative child 和 800 / 800 个 frozen measurement replay 均闭合，本段属于运行后的报告更新。

LAM released outputs 行为 overall micro average。结果严格采用给定的 [Table 3 asset_records.jsonl](runtime/urdf_table3_lam_n800_seed20260813_20260814T022400Z_v3/asset_records.jsonl) 作为固定 membership authority，并与同目录的 [Table 3 manifest.json](runtime/urdf_table3_lam_n800_seed20260813_20260814T022400Z_v3/manifest.json) 按 `selection_rank = 1..800` 交叉重建顺序；没有采用 JSONL 的物理 worker 写入顺序、重新抽样或按结果筛选。来源为本地冻结的 `YipengGao/Articulated-Object-Code@28cec4f5be7e34fd4d586879ecfcb67f7c5e4cc0` 发布集（`N_release = 3,217`，`N_eval = 800`，seed `20260813`）；固定样本包含 621 个 `viable`、75 个 `loads_only`、104 个 `broken`，覆盖 305 个 observed raw category，不是 Full Release Cohort 或 Shared-category Balanced Cohort。323 / 800 个资产加载成功，其中 321 个完成全部测量；477 个预声明 package-audit failure 与 2 个加载成功但存在不可评测 joint range 的 partial 资产全部保留且不补抽，0 timeout。冻结状态分母为 100,631 = 800 个 rest + 50,295 个 single-joint + 49,536 个 Sobol 状态；实际执行 41,481 个，59,150 个未执行状态按 fail-closed 计为 non-free，实际观测到 28,947 个碰撞状态，因此 `Collision-state Rate` 分子为 88,097 = 28,947 + 59,150。`Max Penetration` 仅以 321 / 800 个 measurement-complete 资产计算，最大归一化值为 `0.782640066199297`，状态为 `PARTIAL`；归一化尺度为 PyBullet 中 q=0 时 collision shapes 的 union AABB diagonal。`AOR` 因未运行稳定的精确重叠体积计算而记为 `N/E`，未以包围盒重叠替代；评测只使用 collision geometry、没有 visual fallback，离散 sweep 不构成 CCD、关节语义正确性或物理动力学有效性结论。305 类等权 category-level macro average 为 Rest All-pair CF 0.000%、Rest Non-adjacent CF 24.986%、Single-joint Sweep CF 22.761%、Multi-joint Sobol CF 21.512%、Collision-state Rate 74.596%、Collision-free Range 25.076%、Strict Collision Pass 21.512%。正式证据见 [report.md](runtime/urdf_table4_lam_n800_20260814/report.md)、[summary.json](runtime/urdf_table4_lam_n800_20260814/summary.json)、[frozen_manifest.json](runtime/urdf_table4_lam_n800_20260814/frozen_manifest.json)、[verification.json](runtime/urdf_table4_lam_n800_20260814/verification.json)、[asset_records.json](runtime/urdf_table4_lam_n800_20260814/asset_records.json)、[state_records.jsonl](runtime/urdf_table4_lam_n800_20260814/state_records.jsonl) 和 [protocol_document_at_freeze.md](runtime/urdf_table4_lam_n800_20260814/protocol_document_at_freeze.md)。给定 Table 3 JSONL 文件 SHA-256 为 `7ef1c38d61bc780e41f62c7dd359e66f0bfeabe655c7453c93e2ea9830122d94`，Table 3 manifest 文件 SHA-256 为 `7e16683bfe4e4f37d7972082d8512713c1d8d1ae4ce142b75bf7dfb0509b9951`、self-hash 为 `f8f7fe4da5634d4f806e793c0da919689eab25be1ce0bbed7e2232f3453d15c2`，冻结 selected ordered asset-key SHA-256 为 `643aa5b76ac61f57dd943bee26444a3525c01201a8dff3443763a7fd8d8267d3`。正式 Table 4 manifest 文件 SHA-256 为 `8adc7d8698eaeab5ee5a62d881ed50d4e65c5dc80c9d1d8ae0f4a4a204474594`、self-hash 为 `9a46a1cb7668666cf3c485cc35086cdd79a113d23a8b00625ede012c8b039d2d`、items SHA-256 为 `ef29649907fe7c6ccd08bda75c9693b233b8601ec90b1b05a8d0c68b7bf5b5cc`；runner SHA-256 为 `cdba0dccbef991b2ed4a3f4e418f28725d4e233cad758b81431ce78cb1bbdd4a`，collision core SHA-256 为 `e710d15cb79c50506487ff1335a88591bb58c11cf726c71198103c05f6d01ff0`，结果写回前的 protocol snapshot SHA-256 为 `c59de12cd9b51fc8556291d4a590a36115060df259ee31f0ccf3e87fccb19d86`。最终 [verification.json](runtime/urdf_table4_lam_n800_20260814/verification.json) 文件 SHA-256 为 `e74ed91dca984af8aba900cf3915b490fb1298e5c2bc539af7ade43570edbc51`，31 / 31 checks PASS，800 / 800 个 authoritative child 和 800 / 800 个 frozen measurement replay 均闭合；本段属于运行后的报告更新。

Artiverse 结果严格复用 Table 1 manifest 中原顺序的固定 cohort（预发布 `N_release = 3,544`，`N_eval = 800`，覆盖 67 个观测 raw category），不是 Full Release Cohort 或 Shared-category Balanced Cohort。3 个抽中资产的 URDF joint graph 含环、不是合法 rooted tree，均保留且不补抽；797 / 800 个资产完成加载与测量，0 timeout。`Collision-state Rate` 采用 fail-closed 分母：共预期 133,375 个状态，实际执行 132,739 个状态，636 个未执行状态计为 non-free；实际观测到 76,253 个碰撞状态，故表中分子为 76,889。`Max Penetration` 仅在完成测量的 797 个资产上有观测值，因此报告为 `PARTIAL`；归一化尺度为 PyBullet 中 q=0 时 collision shapes 的 union AABB diagonal。`AOR` 因未运行稳定的精确重叠体积计算而记为 `N/E`，未使用包围盒重叠代替；离散 sweep 不构成 CCD、关节语义正确性或物理动力学有效性结论。67 类等权 category-level macro average 为 Rest All-pair CF 4.012%、Rest Non-adjacent CF 58.744%、Single-joint Sweep CF 47.682%、Multi-joint Sobol CF 47.763%、Collision-state Rate 39.261%、Collision-free Range 57.756%、Strict Collision Pass 45.037%。证据见 [report.md](runtime/urdf_table4_artiverse_table1_n800_20260814/report.md)、[summary.json](runtime/urdf_table4_artiverse_table1_n800_20260814/summary.json)、[frozen_manifest.json](runtime/urdf_table4_artiverse_table1_n800_20260814/frozen_manifest.json) 和 [verification.json](runtime/urdf_table4_artiverse_table1_n800_20260814/verification.json)；Table 1 manifest SHA-256 为 `f74575692b87605699c4f349186c4660d691c91bef39562bb976baf22ae72a8c`，冻结 Table 4 manifest SHA-256 为 `0e69335a3d1574a1e1510124ade6e743cfd66fe894c1da3816b072954c75aedb`，独立验证 24 / 24 checks PASS。

PartNet-Mobility 结果来自本地冻结 release roster 的 2,347 个资产中预先冻结的确定性抽样 cohort（`N_eval = 800`，覆盖 46 类），不是 Full Release Cohort 或 Shared-category Balanced Cohort。该本地数据状态为 `LOCAL_COMPLETE_PROVENANCE_LIMITED`：固定源 `sapien-sim/PartNetMobility@ee0aa3ef1df16181d76d83f7415aa8c94ed1da8f` 的 gated revision 对象 bytes 未与本地文件直接认证。13 个抽中资产包含缺失的 collision mesh 引用，均保留且不补抽；787 / 800 个资产完成加载与测量，0 timeout。`Collision-state Rate` 采用 fail-closed 分母：共预期 137,638 个状态，实际执行 136,100 个状态，1,538 个未执行状态计为 non-free；其中实际观测到 46,343 个碰撞状态。`Max Penetration` 仅在完成测量的 787 个资产上有观测值，因此报告为 `PARTIAL`，但资产级展示分母仍为 800。`AOR` 因未运行稳定的精确重叠体积计算而记为 `N/E`，未使用包围盒重叠代替。证据见 [report.md](runtime/urdf_table4_partnet_mobility_n800_20260813/report.md)、[summary.json](runtime/urdf_table4_partnet_mobility_n800_20260813/summary.json)、[frozen_manifest.json](runtime/urdf_table4_partnet_mobility_n800_20260813/frozen_manifest.json) 和 [verification.json](runtime/urdf_table4_partnet_mobility_n800_20260813/verification.json)；冻结 manifest SHA-256 为 `2ff015ee6bb377ce693126b52dd632a7565a3eaa9f0007e26122a1bb4ab99900`，独立验证 14 / 14 checks PASS。

PhysX-Mobility 行为 overall micro average。正式运行严格复用 Table 5 冻结 receipt set（[manifest.json](runtime/table5_physx_mobility_n800_v2/manifest.json)，文件 SHA256 为 `ccb54f4b726fe717efd28a37948e6b92bac994a2c0ba8fb4ea9ac4548d3a9882`，cohort hash 为 `a9c9c710d9617dea366696603984e330780ce177fead2a34c60410588cc1273c`）中全部 800 行及既有 rank 顺序，没有重新抽样、替换失败项或按结果筛选。评测使用与 PartNet-Mobility / LAM 正式运行相同的冻结 PyBullet collision core（`run_urdf_table4_partnet_mobility.py`，SHA256 `e710d15cb79c50506487ff1335a88591bb58c11cf726c71198103c05f6d01ff0`）：穿透阈值 1e-6 m、rest q=0、单关节 K=21、Sobol R=64（seed 20260813）、direct parent-child pair exclusion、`URDF_USE_SELF_COLLISION | URDF_USE_SELF_COLLISION_INCLUDE_PARENT | URDF_USE_INERTIA_FROM_FILE | URDF_IGNORE_VISUAL_SHAPES`，只使用 collision geometry、无 visual fallback；pair-policy smoke 为 PASS。官方发布几何中资源位于扁平 `urdf/` 目录的 sibling 目录 `partseg/`，因此每个资产先被 staging 为复刻发布几何的自包含 package（逐字节哈希校验，绑定冻结 manifest 行与官方归档，archive SHA256 `88308cc2a4cc6177c59e32c2de51e881e6b961737295e5082d7ed01cca221908`，归档字节级绑定校验通过）再交给子进程审计。归一化尺度冻结为 Table 5 manifest 的 release OBJ-vertex union bounding-box diagonal（`bounding_box_diagonal`，scale protocol `frozen_table5_release_obj_vertex_union_bbox_diagonal_v1`），即 PartNet-Mobility 轮 release bounding-box diagonal 的 PhysX 对应物。

**Claim boundary（必读）**：官方 PhysX-Mobility URDF 未声明任何 collision 元素（冻结 manifest `xml_counts.collision_elements` 总和为 0），全部 104,453 个实际执行状态的 PyBullet contact 计数均为 0；因此本行所有 collision-free 结果都是**空判定（vacuous）**——含义是"没有 collision geometry 可以发生碰撞"，不是"已验证的机械间隙"，不得解读为碰撞安全证据；Table 5 receipt set 已对该方法预注册 `strict_collision: N/E`（reason `official_urdf_zero_collision_elements`），本行数字与该 N/E 判定并存而非替代。

运行状态为 `COMPLETE_WITH_RETAINED_FAILURES`：800 / 800 资产 PyBullet 加载成功，0 timeout；12 个资产（全部为高自由度 `ComputerPeripheral` 键盘类资产：`7619, 12834, 12851, 12880, 12917, 12977, 12999, 13095, 13106, 13120, 13153, 13154`，77--115 个 prismatic 关节）在冻结 reset/readback 阶段因 PyBullet `getJointState` 失败而 fail closed，该 12 个资产与 Table 5 PyBullet runtime 的 `diagnostic_failure` 集合逐一对应；资产 `11854` 的 `floating` joint 不可评测（`joint_range_not_evaluable`），Sobol 未执行；资产 `12562` 声明 0 个非 fixed joints，按冻结 zero-DoF 规则 fail closed。冻结状态分母为 131,925 = 800 个 rest + 79,989 个 single-joint + 51,136 个 Sobol 状态；实际执行 104,453 个，27,472 个未执行状态按 fail-closed 计为 non-free（观测到的碰撞状态为 0），故 `Collision-state Rate` 分子为 27,472。`Max Penetration` 在 787 / 800 个 measurement-complete 资产上为 0.000000（空判定），状态 `PARTIAL`。98 个 observed category 等权 macro average 为 Rest All-pair CF 99.184%、Rest Non-adjacent CF 99.184%、Single-joint Sweep CF 99.165%、Multi-joint Sobol CF 99.136%、Collision-free Range 99.042%、Strict Collision Pass 99.136%。证据见 [summary.json](runtime/urdf_table4_physx_mobility_table5cohort_n800_20260819T143442Z/summary.json)、[report.md](runtime/urdf_table4_physx_mobility_table5cohort_n800_20260819T143442Z/report.md)、[manifest.json](runtime/urdf_table4_physx_mobility_table5cohort_n800_20260819T143442Z/manifest.json)、[asset_records.json](runtime/urdf_table4_physx_mobility_table5cohort_n800_20260819T143442Z/asset_records.json)、[state_records.jsonl](runtime/urdf_table4_physx_mobility_table5cohort_n800_20260819T143442Z/state_records.jsonl)、[pair_policy_smoke.json](runtime/urdf_table4_physx_mobility_table5cohort_n800_20260819T143442Z/pair_policy_smoke.json) 和 [protocol_snapshot.md](runtime/urdf_table4_physx_mobility_table5cohort_n800_20260819T143442Z/protocol_snapshot.md)。formal manifest 文件 SHA256 为 `671d107836fd1344fee34565aa9b16439b208598d179ee5bb59bbfb9bdbfef87`，self-hash 为 `904d7e6f0cfc8ee598e847819a564e9d1dcc731745d42667d83fb6ae078d0421`，runner SHA256 为 `826b91309414aa75b7a7016ad4cf85f2ec02687038d1109aecc328bb9e9d83b3`，summary / asset_records / state_records SHA256 分别为 `b27b71914fcae3a80c7245eedfca55797a5a001a0c5bde1fe6b8d961462b599c` / `1efcfea5920e8db21f7b8afe9de0713a6bb478577d8b4417a6feb2b207397a3d` / `eaa461e205e96b9bc2db4538de4295ba5efdd9bce0ac5d52ed0524648b6ec5fa`，protocol snapshot SHA256 为 `0643725385b73f2a92783ded85b18e44e149702ccbb4d3d60c96a4a201a007d4`；运行环境为 Python 3.12.3、pybullet 3.2.7、numpy 2.5.1、scipy 1.18.0（`.venv_low_medium`，与 Table 5 runtime 相同解释器谱系）。该运行冻结的是子进程启动前写入输出目录的 protocol snapshot；本段属于运行后的报告更新。

### Table 4a. DoF-aware Mechanical Safety (Proposed; LAM, PartNet-Mobility and Artiverse evaluated)

| Dataset / Outputs | Joint-level Full-range CF ↑ | Executable CF DoF/Asset ↑ | Collision-safe DoF Retention ↑ | Normalized Clearance P5 ↑ | Limit Reachability ↑ |
|---|---:|---:|---:|---:|---:|
| Ours-500K | TBD | TBD | TBD | TBD | TBD |
| Articraft-10K | TBD | TBD | TBD | TBD | TBD |
| LAM released outputs | 228 / 2395 (9.52%) | 0.285 / 0.0 / 1.0 (mean / median / P90) | 228 / 2395 (9.52%) | N/E | 222 / 2004 (11.08%) |
| Artiverse | 2,278 / 3,875 (58.79%) | 2.82 / 1.0 / 5.0 (mean / median / P90) | 2,256 / 3,875 (58.22%) | N/E | 2,217 / 3,742 (59.25%) |
| PartNet-Mobility | 2604 / 4078 (63.85%) | 3.255 / 1.0 / 4.0 (mean / median / P90) | 2604 / 4078 (63.85%) | N/E | 2321 / 3416 (67.94%) |
| PhysX-Mobility | TBD | TBD | TBD | TBD | TBD |
| SketchMobility | TBD | TBD | TBD | TBD | TBD |

#### Table 4a metric definitions

| Metric | Definition |
|---|---|
| `Joint-level Full-range CF` | 关节级 `passed / J_eval`。对每个 movable joint 使用既有 Table 4 的冻结测试区间和 `K = 21` 状态，其他关节保持历史 `q = 0`；全部 intended states 均须成功执行，并在统一、无 method-specific allowance 的 headline pair policy 与穿透阈值下无非法碰撞。任何未执行状态、无效区间、加载失败或资源失败均使该关节 fail closed。该离散指标不证明连续配置空间无碰撞。 |
| `Executable CF DoF/Asset` | 每个资产中同时通过 Table 3 `Joint-level Pass` 和本表 `Joint-level Full-range CF` 的关节数；跨 `N_eval` 报告 mean / median / P90。解析、加载或测量失败资产的安全可执行 DoF 计为 0，不得从资产分母删除。 |
| `Collision-safe DoF Retention` | 上述安全可执行关节总数除以冻结 `J_eval`，写为 `passed / J_eval (percentage)`。它是与 raw DoF 数配套的保留率，不得以只统计成功加载资产的条件分母替代。 |
| `Normalized Clearance P5` | 对每个实际测得的 intended state，取 headline pair policy 下所有 eligible non-adjacent collision-surface pair 的最小 signed clearance，再除以统一 `D_visual`；负值表示穿透，零表示接触。先对每个资产取 state-level minimum 的 P5，数据集单元格报告这些 asset-level P5 的中位数，并同时写出 `measured / intended` state 和 asset coverage，以及 `COMPLETE` 或 `PARTIAL`。未测状态不伪造距离，但对应 pass metrics 仍 fail closed。 |
| `Limit Reachability` | bounded joint 的 lower 和 upper endpoint 都可执行、transform 有限且在 headline pair policy 下无非法碰撞时通过；报告 `passed / J_bounded (percentage)`。continuous joint 不进入 `J_bounded`，其分母必须显式报告；endpoint 未执行或不可加载计为失败。 |

Table 4a 还必须按查看结果前冻结的 declared movable DoF 分箱报告 `N_eval`、`Joint-level Full-range CF`、`Collision-safe DoF Retention` 和既有 `Strict Collision Pass`：`0`、`1`、`2--3`、`4--7`、`>=8`。无法可靠提取声明 DoF 的资产进入 `unknown/unparseable` 分箱并保留在整体 intent-to-evaluate 分母中。若新增 runner 沿用历史 Table 4 状态，必须保持其他关节为 q=0；不得把 clipped-zero 或其他新 neutral rule 的结果拼入同一行。

| LAM released outputs declared movable DoF bin | N_eval | Joint-level Full-range CF ↑ | Collision-safe DoF Retention ↑ | Strict Collision Pass ↑ |
|---|---:|---:|---:|---:|
| `0` | 26 | 0 / 0 (N/E) | 0 / 0 (N/E) | 0 / 26 (0.00%) |
| `1` | 295 | 58 / 295 (19.66%) | 58 / 295 (19.66%) | 58 / 295 (19.66%) |
| `2--3` | 304 | 85 / 716 (11.87%) | 85 / 716 (11.87%) | 27 / 304 (8.88%) |
| `4--7` | 132 | 61 / 616 (9.90%) | 61 / 616 (9.90%) | 11 / 132 (8.33%) |
| `>=8` | 43 | 24 / 768 (3.12%) | 24 / 768 (3.12%) | 1 / 43 (2.33%) |

LAM released outputs 行为 overall micro average。正式运行复用 [Table 3 manifest](runtime/urdf_table3_lam_n800_seed20260813_20260814T022400Z_v3/manifest.json) 中全部 800 条 `records[]` 及 `selection_rank = 1..800` 冻结选样顺序（与 [Table 3 asset records](runtime/urdf_table3_lam_n800_seed20260813_20260814T022400Z_v3/asset_records.jsonl) 的 `asset_key` 与 `declared_joint_count` 逐项校验），没有重新抽样、替换失败项或按结果筛选；`N_eval = 800`、`J_eval = 2,395`，覆盖 305 个 observed category（621 个 `viable`、75 个 `loads_only`、104 个 `broken`）。状态碰撞 oracle 为独立版本化的 `Genesis contact-penetration` backend（`genesis_contact_penetration_v1`，Genesis 1.3.1 @ `b1ddc20e`，CPU backend）：每个冻结 q-state 只做 direct kinematic detection（不 `scene.step`）；仅当 eligible source-URDF link pair 的 Genesis-reported penetration 严格大于 `1e-6 m` 时判为非法；开启 self / neutral / adjacent collision candidate generation，并按原始 URDF 的 direct parent-child graph 手工实施 headline exclusion，无 method-specific allowance。`Joint-level Full-range CF` 使用既有 Table 4 的冻结测试区间与 `K = 21` 状态，其他关节保持历史 `q = 0`；strict 状态集为每资产 1 个 rest 状态加 64 个 multi-joint Sobol 状态（seed `20260813`、scramble），共 50,336 行。

运行状态为 `PARTIAL`（formal Genesis run，全部 800 个 rank 均被选中执行）：206 个资产完成 Genesis 评测，594 个资产 fail closed 并保留在全部分母中（398 个无可加载 collision geometry、104 个被冻结 pair-policy 过滤、67 个 Table-3 tree/parse 闭包失败、11 个缺 joint limit、7 个 child timeout、3 个启动失败、2 个几何提取不全、2 个其他）；full-range 状态执行 11,865 / 50,295，未执行状态全部按 non-collision-free 计。`Normalized Clearance P5` 记为 `N/E`（coverage 0 / 50,295 states、0 / 800 assets）：Genesis contact 不提供所有分离 pair 的完整 exact signed clearance，按冻结决定不得由 contact count、penetration、SDF 或 AABB 推导。`Limit Reachability` 分母为 2,004 个 bounded joints（391 个 continuous joints 显式排除）。上表为按 declared movable DoF 的冻结分箱结果；`unknown/unparseable` 分箱为空（全部 800 个资产的声明 DoF 均可可靠提取）。分箱内 `Strict Collision Pass` 为同一 Genesis oracle 下无 method-specific allowance 的 strict pass（overall 97 / 800 (12.12%)），不是历史 PyBullet Table 4 数值，两者不得混写。

证据见 [summary.json](runtime/urdf_lam_supplementary_n800_20260817_v2/summary.json)、[report.md](runtime/urdf_lam_supplementary_n800_20260817_v2/report.md)、[asset_records.jsonl](runtime/urdf_lam_supplementary_n800_20260817_v2/asset_records.jsonl)、[joint_records.jsonl](runtime/urdf_lam_supplementary_n800_20260817_v2/joint_records.jsonl)、[strict_state_records.jsonl](runtime/urdf_lam_supplementary_n800_20260817_v2/strict_state_records.jsonl)、[frozen_manifest.json](runtime/urdf_lam_supplementary_n800_20260817_v2/frozen_manifest.json)、[protocol_snapshot.md](runtime/urdf_lam_supplementary_n800_20260817_v2/protocol_snapshot.md)、[recovery_manifest.json](runtime/urdf_lam_supplementary_n800_20260817_v2/recovery_manifest.json)、[RECOVERY.md](runtime/urdf_lam_supplementary_n800_20260817_v2/RECOVERY.md) 和 [verification.json](runtime/urdf_lam_supplementary_n800_20260817_v2/verification.json)。Table 3 cohort manifest 文件 SHA256 为 `7e16683bfe4e4f37d7972082d8512713c1d8d1ae4ce142b75bf7dfb0509b9951`，self-hash 为 `f8f7fe4da5634d4f806e793c0da919689eab25be1ce0bbed7e2232f3453d15c2`，Table 3 asset_records SHA256 为 `7ef1c38d61bc780e41f62c7dd359e66f0bfeabe655c7453c93e2ea9830122d94`，ordered asset keys SHA256 为 `643aa5b76ac61f57dd943bee26444a3525c01201a8dff3443763a7fd8d8267d3`，verification aggregates SHA256 为 `86ef90bbf33d948809bbe2d444602e7a3e253100a6a4b274bcef88a9e43f09f4`。该运行于 2026-08-17/18 完成全部 800 个 child 评测；定稿聚合曾崩溃于 rank 46 的 receipt 序列化 bug（`intra_link_redundancy` 状态 `N/E` 但 `measured_link_count = 1`，冻结 verifier 要求 `N/E` 时 measured 为 0），该 receipt 已做单字段修复（`measured_link_count` `1 -> 0`，SHA256 `9ec40522c8158a81c2615819cb4c4fed76e45875bb5c7a6c1701cdbec290bbee` -> `797708b779057f194e6a1eb79811c81e888e983dc1094656408867eecf3ef760`，原始字节保留于 `child_attempts/rank_0046.json`，无任何指标值变化），并由恢复定稿器 `finalize_urdf_lam_supplementary_recovery.py` 逐字复现 `run_scope` 的聚合与发布完成定稿。恢复定稿时 runtime_binding 与冻结完全一致，runner / geometry / verifier 代码哈希与冻结一致；`lam_supplementary_static.py` 在全部 child receipts 完成后被无关工作修改（冻结 `1c2fdc2c3d9f8ebcb3ab6b0bf8144b307c86b4b44790cf3182c2395ab37267ff` -> `ac77a014a513cd7d0fa675e7aa46dcaf14433dbb7f01a47895c0010ea1bc3a73`），端到端 `verify_output` 因此仅在代码同一性检查上 FAIL（10 / 18 PASS，其余 7 个 FAIL 为其前置级联），指标聚合链（frozen verifier 的 strict 状态校验与 `aggregate_records`）全部通过。producer bug 已在 `lam_supplementary_geometry.py` 修复（6 处 `N/E` early-return 路径清零 measured 计数），后续方法的 Table 4a 运行不受该 bug 影响。本段属于运行后的报告更新。

| PartNet-Mobility declared movable DoF bin | N_eval | Joint-level Full-range CF ↑ | Collision-safe DoF Retention ↑ | Strict Collision Pass ↑（既有 Table 4 历史值） |
|---|---:|---:|---:|---:|
| `0` | 0 | 0 / 0 (N/E) | 0 / 0 (N/E) | 0 / 0 (N/E) |
| `1` | 327 | 271 / 327 (82.87%) | 271 / 327 (82.87%) | 318 / 327 (97.25%) |
| `2--3` | 288 | 429 / 654 (65.60%) | 429 / 654 (65.60%) | 152 / 288 (52.78%) |
| `4--7` | 99 | 276 / 449 (61.47%) | 276 / 449 (61.47%) | 53 / 99 (53.54%) |
| `>=8` | 86 | 1628 / 2648 (61.48%) | 1628 / 2648 (61.48%) | 44 / 86 (51.16%) |

PartNet-Mobility 行为 overall micro average。正式运行严格复用 [Table 4 frozen manifest](runtime/urdf_table4_partnet_mobility_n800_20260813/frozen_manifest.json) 中 `.items[].dataset_id` 的全部 800 项及既有顺序，没有重新抽样、替换失败项或按结果筛选；`N_eval = 800`、`J_eval = 4,078`，覆盖 46 个 observed category。状态碰撞 oracle 为独立版本化的 `Genesis contact-penetration` backend（`genesis_contact_penetration_v1`，Genesis 1.3.1 @ `b1ddc20e`，CPU backend、precision 64），复用与 LAM 运行相同的冻结 `GenesisTable4aAdapter`：每个冻结 q-state 只做 direct kinematic detection（不 `scene.step`）；仅当 eligible source-URDF link pair 的 Genesis-reported penetration 严格大于 `1e-6 m` 时判为非法；开启 self / neutral / adjacent collision candidate generation，并按原始 URDF 的 direct parent-child graph 手工实施 headline exclusion，无 method-specific allowance。`Joint-level Full-range CF` 严格沿用既有 Table 4 的冻结单关节 sweep：每关节 `K = 21` 个状态（含两端 endpoint，取值 `lower + i*(upper-lower)/20`；bounded joint 使用声明 lower/upper，continuous joint 使用冻结区间 [-π, π]），其他关节保持历史 `q = 0`；每个执行状态的全 DoF 向量按冻结 manifest 关节顺序重排后取 canonical SHA256，与既有 Table 4 `state_records.joint_values_sha256` 逐项核对，62,958 个执行状态全部 verified、0 mismatch（attempt 1 中 12 个 mismatch 均为引擎内部 DoF 排序与 manifest 排序的差异而非状态值差异，attempt 2 冻结改用 manifest 顺序核对）。

运行状态为 646 个 `completed`、154 个 fail closed 并保留在全部分母中：91 个 child process crash（SIGABRT，仿真器崩溃计为失败）、53 个加载失败（其中 13 个为冻结已知的缺 collision mesh 引用资产，38 个为 Genesis loader 内部确定性 `IndexError`，两次 attempt 间大部分可复现，另 2 个其他）、10 个 child timeout（3,600 s）。full-range 状态执行 62,958 / 85,638，未执行状态全部按 non-collision-free 计。`Normalized Clearance P5` 记为 `N/E`（coverage 62,958 / 85,638 states、646 / 800 assets，`PARTIAL`）：Genesis contact 不提供所有分离 pair 的完整 exact signed clearance，按冻结决定不得由 contact count、penetration、SDF 或 AABB 推导，且本轮未注册独立 exact-distance backend。`Limit Reachability` 分母为 3,416 个 bounded joints（662 个 continuous joints 显式排除），endpoint 状态复用同一 K=21 sweep 的 sample_index 0 / 20。上表分箱内 `Strict Collision Pass` 为既有 Table 4 的历史 PyBullet strict pass（overall 567 / 800），仅按分箱报告，不是 Genesis-oracle strict pass，与 LAM 行的 Genesis strict 口径不得混写；`unknown/unparseable` 分箱为空。本表共两次 attempt：attempt 1（`runtime/table4a_urdf_partnet_mobility_table4cohort_n800_salt20260813_20260819T093352Z`，单一共享 Genesis cache）因 cache 并发写导致 522 / 800 个资产 SIGABRT crash，仅作为失败尝试证据保留；attempt 2 改为每 child 独立 cache（rank 1 warmup cache 作为只读模板复制进后续各 rank）并冻结 manifest 顺序状态核对，workers=16、staggered launch、单 child 超时 3,600 s，总墙钟约 22,251 s。

证据见 [summary.json](runtime/table4a_urdf_partnet_mobility_table4cohort_n800_salt20260813_20260819T123437Z/summary.json)、[summary.md](runtime/table4a_urdf_partnet_mobility_table4cohort_n800_salt20260813_20260819T123437Z/summary.md)、[manifest.json](runtime/table4a_urdf_partnet_mobility_table4cohort_n800_salt20260813_20260819T123437Z/manifest.json)、[asset_records.jsonl](runtime/table4a_urdf_partnet_mobility_table4cohort_n800_salt20260813_20260819T123437Z/asset_records.jsonl)、[joint_records.jsonl](runtime/table4a_urdf_partnet_mobility_table4cohort_n800_salt20260813_20260819T123437Z/joint_records.jsonl)、[frozen_config.json](runtime/table4a_urdf_partnet_mobility_table4cohort_n800_salt20260813_20260819T123437Z/frozen_config.json) 和 [protocol_snapshot.md](runtime/table4a_urdf_partnet_mobility_table4cohort_n800_salt20260813_20260819T123437Z/protocol_snapshot.md)。Table 4 cohort manifest SHA256 为 `2ff015ee6bb377ce693126b52dd632a7565a3eaa9f0007e26122a1bb4ab99900`，ordered asset IDs SHA256 为 `ef6cb964e50dc712280256c5b2f675cc2c957095c3553b21845d3562a5011883`，frozen config SHA256 为 `c7815f555f89590eed5fce3c431083fcab2e3408d73163178b324c6e38e050ee`，runner SHA256 为 `2b4f4ee2d5caefa8d6cf96abcdc6ebdcb9beec734c5e7e5fc64ea3114d677474`，冻结 LAM adapter 模块 SHA256 为 `c43f3047553e4fbc9dfeefcbb7308bc42df8c6f0aab24f2a85f412c5efe12df5`，所用 `lam_supplementary_static` atom 模块 SHA256 为 `ac77a014a513cd7d0fa675e7aa46dcaf14433dbb7f01a47895c0010ea1bc3a73`，summary / asset records / joint records SHA256 分别为 `40aa0fb1581a2a74c0900e020085cd4545a7ac93fd5c704fc288fec7959e689c` / `da4b6c16dbdd550ff1a427d1c7fa727f98f1fe42086757b6e174733c8bab24d0` / `1f5f7cc39fb5e16fb5db3cf36f488b5c3dac97beb45a2f70fdbf59830c47e12e`，manifest 文件 / self-hash 分别为 `8c47553d4bac3ea90c3d3b57939a71eeb56deb06cb871363e247a96aba85341d` / `f1958ac31339e592cfa1db4401aaf4f11125a735abf0fd978dc252513c43c3bd`。最终验证 10 / 10 checks PASS。与其他方法运行之间共享模块版本可能不同，最终跨方法比较前须做版本等价性确认（同 Table 2 supplementary 注意事项）。本段属于运行后的报告更新。

| Artiverse declared movable DoF bin | N_eval | Joint-level Full-range CF ↑ | Collision-safe DoF Retention ↑ | Strict Collision Pass ↑（既有 Table 4 历史值） |
|---|---:|---:|---:|---:|
| `0` | 0 | 0 / 0 (N/E) | 0 / 0 (N/E) | 0 / 0 (N/E) |
| `1` | 170 | 109 / 170 (64.12%) | 103 / 170 (60.59%) | 108 / 170 (63.53%) |
| `2--3` | 363 | 500 / 858 (58.28%) | 492 / 858 (57.34%) | 102 / 363 (28.10%) |
| `4--7` | 196 | 522 / 979 (53.32%) | 514 / 979 (52.50%) | 37 / 196 (18.88%) |
| `>=8` | 71 | 1147 / 1868 (61.40%) | 1147 / 1868 (61.40%) | 7 / 71 (9.86%) |

Artiverse 行为 overall micro average。正式运行严格复用 [Table 4 frozen manifest](runtime/urdf_table4_artiverse_table1_n800_20260814/frozen_manifest.json) 中 `.items[]` 的全部 800 项及既有顺序（抽样样本即 `jq -r '.assets[].manifest_root' exp/runtime/table1_artiverse/manifest.json`，与 Table 2 / Table 3 / Table 4 Artiverse 正式运行同一 cohort），没有重新抽样、替换失败项或按结果筛选；`N_eval = 800`、`J_eval = 3,875`，覆盖 67 个 observed raw category。每资产关节分母取 Table 4 frozen manifest 的 `movable_dof_count`，与 Table 3 `asset_records.declared_joint_count` 总和一致；每资产 primary URDF SHA-256 与冻结 manifest `urdf_sha256` 匹配。状态碰撞 oracle 为冻结 Genesis contact-penetration backend（`genesis_contact_penetration_v1`，与 LAM / PartNet-Mobility 运行同一引擎协议；Genesis 1.3.1、trimesh 5.0.0、rtree 1.4.1，CPU backend、precision 64、每资产一个独立 Genesis 进程）：每个冻结 q-state 只做 direct kinematic detection，仅当 eligible source-URDF link pair 的穿透严格大于 1e-6 m 判为非法；headline pair policy 为 distinct source links 排除 URDF direct parent-child graph，开启 self / neutral / adjacent candidate generation。状态严格沿用 Table 4 冻结单关节 sweep（每关节 K=21，其余关节保持 q=0），每个状态的全 DoF 向量按 manifest 顺序重排后取 canonical SHA256，与 Table 4 `state_records.joint_values_sha256` 逐项核对，58,548 个已执行状态全部一致（0 mismatch）。

800 / 800 个 child 全部产出结果；运行状态为 595 个 `completed`、205 个 fail closed 并保留在全部分母中：96 个加载失败为 `Genesis filtered eligible non-adjacent collision geometry`——冻结 adapter 要求全部 eligible pair 存在于 Genesis valid-pair table，而 Genesis 碰撞候选生成对经 fixed joint 连接的 link 之间的 pair 系统性排除（实测验证：经 fixed joint 挂到同一父 link 的两个子 link，其相互 pair 不进入 `_valid_collision_pairs`），Artiverse URDF 广泛使用 fixed-joint 挂载子部件（如 shelf、handle），此类资产按冻结规则 fail closed；这是 oracle 覆盖限制而非资产缺陷。77 个为 child process crash（66 个 SIGABRT、11 个 SIGSEGV，集中在前约 100 个 rank；同一资产在隔离 smoke 中可正常完成，判定为运行初期引擎/并发不稳定）。18 个为单资产 3600 秒超时（含 7,140 collision elements / 91 DoF 等重尾资产）。其余 14 个为 mesh 加载或连通性失败（含 Table 4 已知的 joint graph 带环与 disconnected link 资产）。`Joint-level Full-range CF` 为 2,278 / 3,875（58.79%）；`Executable CF DoF/Asset` mean / median / P90 为 2.82 / 1.0 / 5.0（解析、加载或测量失败资产的安全可执行 DoF 计为 0，资产分母 800）；`Collision-safe DoF Retention` 为 2,256 / 3,875（58.22%）；`Limit Reachability` 为 2,217 / 3,742（59.25%），133 个 continuous joint 不进入 `J_bounded` 并已显式计数；`Normalized Clearance P5` 为 `N/E`（该 oracle 对分离 pair 无 signed clearance，未注册独立 exact-distance backend；state coverage 58,548 / 81,375、asset coverage 595 / 800，coverage 状态 PARTIAL）。DoF 分箱中 Strict Collision Pass 为既有 Table 4 历史值（asset-level micro 254 / 800）。

运行环境为 Python 3.12.3（genesis-main env），workers = 16，单资产超时 3600 秒，墙钟约 23,096 秒；最终验证 10 / 10 checks PASS。证据见 [summary.json](runtime/table4a_urdf_artiverse_table1cohort_n800_seed20260813_20260819T153459Z/summary.json)、[summary.md](runtime/table4a_urdf_artiverse_table1cohort_n800_seed20260813_20260819T153459Z/summary.md)、[manifest.json](runtime/table4a_urdf_artiverse_table1cohort_n800_seed20260813_20260819T153459Z/manifest.json)、[asset_records.jsonl](runtime/table4a_urdf_artiverse_table1cohort_n800_seed20260813_20260819T153459Z/asset_records.jsonl)、[joint_records.jsonl](runtime/table4a_urdf_artiverse_table1cohort_n800_seed20260813_20260819T153459Z/joint_records.jsonl)、[frozen_config.json](runtime/table4a_urdf_artiverse_table1cohort_n800_seed20260813_20260819T153459Z/frozen_config.json) 和 [protocol_snapshot.md](runtime/table4a_urdf_artiverse_table1cohort_n800_seed20260813_20260819T153459Z/protocol_snapshot.md)。Table 4 cohort manifest SHA256 为 `0e69335a3d1574a1e1510124ade6e743cfd66fe894c1da3816b072954c75aedb`，ordered asset IDs SHA256 为 `118038a746cafb91251afde5eb4f1164915d141acb3b529ea721a9d376bde3fa`，frozen config SHA256 为 `4aeda0c4db358be1c92d60f12f60baf4837d9996aa4c195a81f46df31e0bfd89`，runner SHA256 为 `0460239cf3dfb932108bf9e2e5621123549176714fa64ec0875562b8cf3c7064`，冻结 LAM adapter 模块 SHA256 为 `c43f3047553e4fbc9dfeefcbb7308bc42df8c6f0aab24f2a85f412c5efe12df5`，所用 `lam_supplementary_static` atom 模块 SHA256 为 `ac77a014a513cd7d0fa675e7aa46dcaf14433dbb7f01a47895c0010ea1bc3a73`（与 PartNet-Mobility attempt-2 运行逐位相同），summary / asset records / joint records SHA256 分别为 `6d274b6580e2fcf48910868e8eaf008a972276d66616007996b035b10915ee2c` / `9080c81c22e54a9b1ac6d1c38ebba689282e4b72f57b3acc0ce24b4858025561` / `afa25a4a7542444f17fce6750a757277874a13336a27aea8ad12a52a1597e2ba`，manifest 文件 / self-hash 分别为 `55dfc82db051509ec159b06704bfa351c2a0b9d96a9c7817a762323f37e1acc0` / `4ee2c7932bdad0c41024fbca7ef12a5c707430fca20df6054eac25fe6968a8ae`。本段属于运行后的报告更新。

### Table 4b. Collision Representation Quality and Cost (Proposed; LAM evaluated)

| Dataset / Outputs | Analytic Collision Share ↑* | Visual→Collision P95 ↓ | Collision→Visual P95 ↓ | Shapes/Visual-bearing Link ↓* | Collision Mesh Triangles/Asset ↓* | Intra-link Redundancy ↓ | Collision Load Time/Asset ↓ |
|---|---:|---:|---:|---:|---:|---:|---:|
| Ours-500K | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| Articraft-10K | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| LAM released outputs | 0 / 2084 (0.00%) | 8.28e-17 (326/800, PARTIAL) | 8.27e-17 (326/800, PARTIAL) | 1.0 / 1.0 / 1.0 (322/800, PARTIAL) | 3029.5 / 1676 / 6688 (322/800, PARTIAL) | 0.0 (links 1482/2084, assets 289/800, PARTIAL) | 0.0730 s / 0.2738 s (322/800, PARTIAL) |
| Artiverse | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| PartNet-Mobility | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| PhysX-Mobility | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| SketchMobility | TBD | TBD | TBD | TBD | TBD | TBD | TBD |

#### Table 4b metric definitions

| Metric | Definition |
|---|---|
| `Analytic Collision Share` | 所有资源可解析、可加载的 collision elements 中，由全部冻结 adapter 原生共同支持的非 mesh primitive 数量占比，报告 `analytic elements / loadable collision elements`。`↑*` 只表示在双向 surface error 不劣化时更利于表示与运行成本；不得单独作为质量排名。 |
| `Visual→Collision P95` | 在每个资产的 loadable visual surface 上按冻结面积权重和随机种子采样，计算每个样本到 collision-surface union 的最近距离，以统一 `D_visual` 归一化后取 asset-level P95；数据集报告 asset-level P95 的中位数和 measurement coverage。该方向主要揭示 visual surface 未被 collision 覆盖的区域。 |
| `Collision→Visual P95` | 对 collision surface 使用同一采样预算与归一化，计算到 visual-surface union 的最近距离并取 asset-level P95；数据集报告中位数和 measurement coverage。该方向主要揭示 collision 在 visual 外部的过度膨胀，不能以单向误差替代。 |
| `Shapes/Visual-bearing Link` | loadable collision shape 数除以 declared visual-bearing link 数，跨 `N_eval` 报告 mean / median / P90，并显式报告 visual-bearing-link extraction coverage。该成本只可在双向 surface error 与 coverage 相当时解释；缺 collision 得到的较小 shape 数不是优势。 |
| `Collision Mesh Triangles/Asset` | 每个资产所有 collision meshes 在冻结坐标焊接容差和退化面规则后保留的有效 triangle 数，报告 mean / median / P90 及 mesh-validation coverage；analytic primitives 不得人为三角化后加入该计数。`↓*` 只在表示误差相当时解释。 |
| `Intra-link Redundancy` | 对同一 link 的 collision shapes，以 `max(0, (sum_i V_i - V(union_i G_i)) / sum_i V_i)` 衡量内部重复体积，并按可测 collision volume 聚合。只有 analytic 或经冻结 watertight/finite-volume 检查的 geometry 才进入体积计算；不稳定或分母为零时报告 coverage、`PARTIAL` 或 `N/E`，不得用 AABB overlap 冒充 surface/volume 结果。 |
| `Collision Load Time/Asset` | 在冻结硬件、软件版本、线程数、进程隔离、cache mode 和重复次数下，仅加载并构建 collision representation 的 wall-clock 时间，报告 median / P90、失败数和 coverage；禁止失败时自动切换 visual fallback 或简化资产。 |

本表统一定义 `D_visual` 为历史 q=0 下全部 loadable visual geometry 经同一单位变换后的 union AABB diagonal。`D_visual` 缺失、非有限或非正时，对应数值误差记为不可测并保留 coverage；不得回退到 collision AABB、发布元数据尺度或各数据集自带的不同尺度。surface sampler 必须在查看结果前冻结样本数、面积权重、随机种子、单位、坐标焊接容差和退化面处理。AABB 只能用于这里明确规定的尺度归一化，不能代替双向 surface distance 或 union-volume redundancy。所有数值单元格均须带 `measured / intended` coverage 和 `COMPLETE`/`PARTIAL`；除 LAM released outputs 外的行仍为 `TBD`。

LAM released outputs 行为 overall micro average，来自与 Table 4a 相同的冻结运行 [urdf_lam_supplementary_n800_20260817_v2](runtime/urdf_lam_supplementary_n800_20260817_v2/summary.json)（`urdf_lam_supplementary_n800_genesis_v1`；cohort 冻结与恢复定稿披露见 Table 4a 证据段，verification aggregates SHA256 `86ef90bbf33d948809bbe2d444602e7a3e253100a6a4b274bcef88a9e43f09f4`）。Table 4b 冻结参数：exact surface backend 为独立的 `trimesh.proximity.ProximityQuery.on_surface + rtree`（不以 Genesis SDF 代替），每方向 32,768 个面积加权样本，坐标焊接相对容差 `1e-9`，`D_visual` = q0 loadable visual union AABB diagonal（`q0_loadable_visual_union_aabb_diagonal_v1`）。单元格内 `a / b / c` 依次为 mean / median / P90（`Collision Load Time` 为 median / P90，单位秒）。LAM 的 collision 表示是 visual mesh 的直接复制：双向表面误差在机器精度级别为零（asset-level P95 中位数约 `8.3e-17`），每个 visual-bearing link 恰好 1 个 loadable collision shape，link 内零冗余（冗余体积 0 / 形状体积 3,529,232.7 m³，volume-weighted `0.0`），analytic primitive 占比 0 / 2,084（LAM collision 全部为 mesh，无 analytic 三角化混入三角形计数）。`Collision Load Time` 冻结计时协议为 `trimesh.Trimesh(process=False).triangles_tree+rtree`、同资产一次未测 warmup 后 warm-cache 重复、每次重复重新构建几何、进程隔离、排除进程启动/导入/warmup/哈希/序列化 I/O、禁止 visual/simulator fallback。所有单元格均为 intent-to-evaluate：387 个资产不声明任何 collision geometry，其余不可测原因为 tree/q0 提取失败、child timeout 或 `D_visual` 不可得（76 个），均 fail closed 保留在 `N_eval = 800` 分母中，故全部指标记 `PARTIAL` 并带 measured / intended coverage。mean 值（Shapes/Visual-bearing Link 与 Triangles/Asset）由 finalized [asset_records.jsonl](runtime/urdf_lam_supplementary_n800_20260817_v2/asset_records.jsonl) 的逐资产记录直接投影；median / P90 / status / coverage 取自冻结 verifier 聚合（`summary.json` 的 `verification_aggregates.table4b`）。本段属于运行后的报告更新。

### Supplementary Table S1. Mechanical Evidence and Allowance Sensitivity (Proposed; not yet evaluated)

| Dataset / Outputs | Receipt-bound Assets ↑ | Receipt Replay Pass ↑ | Deterministic Rebuild Match ↑ | Allowance Density ↓ | Strict Pass (No Method-specific Allowance) ↑ | Registered-allowance Gain ↓ |
|---|---:|---:|---:|---:|---:|---:|
| Ours-500K | TBD | TBD | TBD | TBD | TBD | TBD |
| Articraft-10K | TBD | TBD | TBD | TBD | TBD | TBD |
| LAM released outputs | TBD | TBD | TBD | TBD | TBD | TBD |
| Artiverse | TBD | TBD | TBD | TBD | TBD | TBD |
| PartNet-Mobility | TBD | TBD | TBD | TBD | TBD | TBD |
| PhysX-Mobility | TBD | TBD | TBD | TBD | TBD | TBD |
| SketchMobility | TBD | TBD | TBD | TBD | TBD | TBD |

#### Supplementary Table S1 metric definitions

| Metric | Definition |
|---|---|
| `Receipt-bound Assets` | 具有 machine-readable mechanical receipt，且 receipt 同时绑定资产及递归 resource-closure hash、protocol/runner identity、pair policy、阈值和结论的资产数除以 `N_eval`。仅写有 `success`、未绑定机械协议的 compile report 不计为有效 receipt；缺失按未覆盖而非 `N/E`。 |
| `Receipt Replay Pass` | 独立 replay 后结果与 receipt 完全一致的资产数除以 `N_eval`；receipt 缺失、输入不闭合、无法 replay 或结论不一致均 fail closed。 |
| `Deterministic Rebuild Match` | 对公开了冻结 build recipe 和完整输入的资产，重建后的 canonical URDF/resource fingerprint 与发布物一致的数量除以 `N_rebuild_eligible`，并同时报告 `N_rebuild_eligible / N_eval` coverage。方法不存在可公开重建工作流时记为 `N/E`，不得写成 0 或通过。 |
| `Allowance Density` | 在查看结果前注册的 method-specific excluded non-adjacent pairs 总数除以同一资产集合的 eligible non-adjacent pairs 总数。未注册 method-specific allowance 时密度为 0；运行后追加的 allowance 禁止进入任何结果。 |
| `Strict Pass (No Method-specific Allowance)` | 仅应用全部方法共享的 topology exclusion、接触规则和阈值时得到的 `Strict Collision Pass`，报告 `passed / N_eval`。这是新增 headline/mandatory companion 所采用的 pair-policy 版本。 |
| `Registered-allowance Gain` | 预注册 method-specific allowance 版本的 strict pass rate 减去 `Strict Pass (No Method-specific Allowance)`，以 percentage points 报告。该差值只用于 supplementary sensitivity analysis，不能替代 headline；无注册 allowance 时正式结果应为 0 pp。 |

---

## Table 5a. Per-Simulator Runtime Readiness

| Dataset / Outputs | Simulator | Load Rate ↑ | Reset Success ↑ | Settling Stability ↑ | Actuation Success ↑ | Limit Enforcement ↑ | Constraint Drift ↓ | Simulator Pass ↑ |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| Ours-500K | PyBullet | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| Ours-500K | Genesis | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| Ours-500K | MuJoCo | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| Articraft-10K | PyBullet | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| Articraft-10K | Genesis | 215 / 800 (26.875%) | 273 / 800 (34.125%) | 128 / 800 (16.000%) | 62 / 800 (7.750%) | 17 / 800 (2.125%) | 123 / 800 (15.375%) | 4 / 800 (0.500%) |
| Articraft-10K | MuJoCo | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| LAM released outputs | PyBullet | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| LAM released outputs | Genesis | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| LAM released outputs | MuJoCo | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| Artiverse | PyBullet | 784 / 800 (98.000%) | 784 / 800 (98.000%) | 579 / 800 (72.375%) | 409 / 800 (51.125%) | 34 / 800 (4.250%) | 709 / 800 (88.625%) | 8 / 800 (1.000%) |
| Artiverse | Genesis | 278 / 800 (34.750%) | 754 / 800 (94.250%) | 587 / 800 (73.375%) | 532 / 800 (66.500%) | 19 / 800 (2.375%) | 687 / 800 (85.875%) | 0 / 800 (0.000%) |
| Artiverse | MuJoCo | 0 / 800 (0.000%) | 662 / 800 (82.750%) | 97 / 800 (12.125%) | 136 / 800 (17.000%) | 126 / 800 (15.750%) | 75 / 800 (9.375%) | 0 / 800 (0.000%) |
| PartNet-Mobility | PyBullet | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| PartNet-Mobility | Genesis | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| PartNet-Mobility | MuJoCo | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| PhysX-Mobility | PyBullet | 788 / 800 (98.500%) | 788 / 800 (98.500%) | 215 / 800 (26.875%) | 783 / 800 (97.875%) | 29 / 800 (3.625%) | 785 / 800 (98.125%) | 11 / 800 (1.375%) |
| PhysX-Mobility | Genesis | 0 / 800 (0.000%) | 765 / 800 (95.625%) | 207 / 800 (25.875%) | 762 / 800 (95.250%) | 9 / 800 (1.125%) | 764 / 800 (95.500%) | 0 / 800 (0.000%) |
| PhysX-Mobility | MuJoCo | 0 / 800 (0.000%) | 800 / 800 (100.000%) | 218 / 800 (27.250%) | 796 / 800 (99.500%) | 1 / 800 (0.125%) | 0 / 800 (0.000%) | 0 / 800 (0.000%) |
| SketchMobility | PyBullet | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| SketchMobility | Genesis | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| SketchMobility | MuJoCo | TBD | TBD | TBD | TBD | TBD | TBD | TBD |

### Table 5a metric definitions

| Metric | Pass condition |
|---|---|
| `Load Rate` | 仿真器成功创建完整 multibody；link/joint 数量与解析后的 URDF 一致。任何自动丢弃的 link 或 joint 均判为失败。 |
| `Reset Success` | 连续执行固定次数的 load、reset 和 joint-state initialization，无崩溃、超时或非有限状态。 |
| `Settling Stability` | 在冻结的被动仿真时长内无 NaN、约束断裂、异常速度或超过阈值的非预期漂移。 |
| `Actuation Success` | 对每个关节施加统一的归一化目标后，实际运动达到至少 90% 的声明运动范围。 |
| `Limit Enforcement` | 施加越界目标时，实际 joint state 未超过声明 limit 与数值容差。 |
| `Constraint Drift` | 运动过程中关节锚点误差、轴向误差和禁止自由度位移的最大值。表中报告尺度归一化后的最大值。 |
| `Simulator Pass` | 同一资产在该仿真器中同时通过 Load、Reset、Settling、Actuation、Limit 和 Drift 检查。 |

每个仿真器分别使用其公开、冻结的稳定配置。三者的 timestep、gravity、base policy、初始状态、目标轨迹、运行时长和成功阈值保持一致；solver 实现差异不得通过修改资产来消除。

---

## Table 5b. Cross-Simulator Consistency and Overall Sim-Readiness

| Dataset / Outputs | PyBullet Pass ↑ | Genesis Pass ↑ | MuJoCo Pass ↑ | All-three Load ↑ | All-three Runtime Pass ↑ | Cross-sim Joint RMSE ↓ | Cross-sim Link-pose Error ↓ | Strict Sim-ready ↑ |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| Ours-500K | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| Articraft-10K | TBD | 4 / 800 (0.500%) | TBD | TBD | TBD | TBD | TBD | TBD |
| LAM released outputs | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| Artiverse | 8 / 800 (1.000%) | 0 / 800 (0.000%) | 0 / 800 (0.000%) | 0 / 800 (0.000%) | 0 / 800 (0.000%) | rev max 3095.693403; prism max 321.180421 | trans max 84.273932; rot max 3.141592 | 0 / 800 (0.000%) |
| PartNet-Mobility | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| PhysX-Mobility | 11 / 800 (1.375%) | 0 / 800 (0.000%) | 0 / 800 (0.000%) | 0 / 800 (0.000%) | 0 / 800 (0.000%) | rev max 0.157887; prism max 172.097041 | trans max 0.188499; rot max 0.878166 | N/E |
| SketchMobility | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD |

### Table 5b metric definitions

| Metric | Definition |
|---|---|
| `PyBullet/Genesis/MuJoCo Pass` | 对应 Table 5a 的资产级 Simulator Pass。 |
| `All-three Load` | 同一资产可以在三个仿真器中全部成功加载。 |
| `All-three Runtime Pass` | 同一资产在三个仿真器中全部达到 Simulator Pass。 |
| `Cross-sim Joint RMSE` | 相同归一化目标轨迹下，三个仿真器 joint trajectory 的最大两两 RMSE；revolute 和 prismatic 分别报告。 |
| `Cross-sim Link-pose Error` | 对齐共同 root frame 后，三个仿真器最终 descendant-link pose 的最大两两误差；平移和旋转分别报告。 |
| `Strict Sim-ready` | 同一资产同时达到 Strict URDF Pass、Strict Kinematic Pass、Strict Collision Pass、All-three Runtime Pass，并满足冻结的 cross-simulator consistency 阈值。 |

Cross-simulator agreement 衡量实现一致性，不代表动力学参数与现实世界真值一致。

Artiverse formal N=800 result note: the cohort is the original ordered .assets[].manifest_root sequence from exp/runtime/table1_artiverse/manifest.json, with no resampling or replacement. Each simulator uses the full intent denominator 800; preflight_failure, diagnostic_failure, native_crash, timeout, and worker_error remain in the denominator. Terminal counts: PyBullet completed=784, preflight_failure=3, timeout=13; Genesis completed=754, diagnostic_failure=22, native_crash=5, preflight_failure=3, timeout=12, worker_error=4; MuJoCo completed=662, diagnostic_failure=134, preflight_failure=3, timeout=1. Upstream strict gates are Strict URDF 774 / 800 (96.750%), Strict Kinematic 762 / 800 (95.250%), and Strict Collision 254 / 800 (31.750%); Strict Sim-ready is their conjunction with all-three runtime and cross-simulator thresholds.

Cross-sim values are pairwise maxima over evaluable units: revolute joint n=914, prismatic joint n=1591, and link-pose n=4274; the table separates joint RMSE by joint type and link-pose translation / rotation. PyBullet 3.2.7 and MuJoCo 3.10.0 used CPU; Genesis 1.3.1 used the CUDA backend with one worker on GPU3 (UUID GPU-ebc0d328-a3fa-7e89-2733-cadb001661f7). The formal publication is at /root/.cache/torch/arti-skill/table5_artiverse_table1_n800_gpu_v4/aggregate/formal; the independent published-output validator passed. Cohort SHA256: 3e12e86fa61b9af14a411a2571c100e49f3ad49f6286394453366a64caeeb171. Protocol semantic SHA256: ebd1e6599f782511b0974208a0294cb2e42a7f1645614ac9a4e49df13c91e551.
PhysX-Mobility N=800 was drawn from the official Caoza/PhysX-Mobility release (revision d0768ee9e1415f6be8db78d6389ba018b85134c0, candidate closure 2024) by ascending (rank_sha256, integer dataset_id) order with rank salt arti-skill-table5-physx-mobility-n800-v1; no resampling or outcome filtering. The official URDFs contain zero collision elements, so Genesis 1.3.1 ran on the CPU backend with collision disabled, and Strict Collision Pass / Strict Sim-ready are N/E rather than 0. PyBullet 3.2.7 and MuJoCo 3.10.0 used CPU; Genesis load is reported under the exact-count load contract (any dropped/folded structure fails). The formal publication is at /mnt/zsn/lyb/arti-skill/exp/runtime/table5_physx_mobility_n800_v2/aggregate/formal; the publication self-check passed. Cohort SHA256: a9c9c710d9617dea366696603984e330780ce177fead2a34c60410588cc1273c. Protocol semantic SHA256: 4403a4190e2393c2812cf25193cbc6a08e75b350e65f47302db7f7c8a7321101.
Articraft-10K Table 5 currently covers the Genesis-only phase on the frozen Table 2 N=800 cohort (exact .records[].package stored order, seed 20260813; ordered manifest-root hash equals the Table 2 selected_asset_ids_sha256 79c44441600077513d3cde1cda8fef38324e1a0ee660730b860d5313f0ae9784). Genesis 1.3.1 used the CUDA backend with one worker on GPU3 (UUID GPU-ebc0d328-a3fa-7e89-2733-cadb001661f7) under the same frozen runtime configuration as Artiverse. Assets whose URDF lacks complete inertial data crash the Genesis native parser; those crashes are retained as terminal failures (514 malformed_response, 3 native_crash, 4 timeout, 5 worker_error, 1 diagnostic_failure, 273 completed) and fail closed in every metric with the full intent-to-evaluate denominator of 800. Constraint Drift additionally fails closed for the 577 assets without a Table 4 bounding-box normalizer. Strict Collision Pass / Strict Sim-ready are N/E in this phase; All-three and cross-simulator cells remain TBD until PyBullet and MuJoCo phases run. The formal publication is at /root/.cache/torch/arti-skill/table5_articraft10k_table2_n800_gpu_v1/aggregate/formal; the publication self-check and an independent terminal-record recomputation both passed. Cohort SHA256: 5fb34ea89d43ace197a0a6431e031aaaafa66ebad89af8702dc26b19ef08b06a. Protocol semantic SHA256: fcb76c4e63dc56b5ebc1330bfd7c10ef85a958cd040fa4feb6c65122b91013bc.

---

## 2. Headline Metrics and Claim Boundary

建议将以下四项作为主文 headline metrics：

1. `Strict URDF Pass`：静态结构和物理字段全部合法；
2. `Strict Kinematic Pass`：全部声明关节运动学可执行；
3. `Strict Collision Pass`：冻结采样状态和 pair policy 下无非法穿透；
4. `Strict Sim-ready`：前三项通过，并且在 PyBullet、Genesis、MuJoCo 中全部通过运行时测试和跨仿真器一致性阈值。

上述四项 strict metrics 保持不变。任何主文 headline comparison 还必须同时报告以下三项 mandatory companion metrics；它们用于解释 coverage、DoF 难度和 collision 表示成本，不替代四项 strict metrics，也不在未版本化重跑前进入 strict pass：

1. `Visual-bearing Collision Coverage`：报告资产级 `passed / N_eval`，并附 link-micro coverage；
2. `Collision-safe DoF Retention`：报告 `passed / J_eval`，并附 `0`、`1`、`2--3`、`4--7`、`>=8` DoF 分箱；
3. `Collision Representation Quality and Cost`：至少联合报告 Table 4b 的双向 surface P95、measurement coverage，以及一个 fidelity-matched representation/runtime cost；禁止只挑选 analytic share、triangle count 或 load time 单项排名。

在无 GT、无人工评价的协议下，可以支持以下结论：

> 资产具有合法的 URDF 结构、可执行的声明运动学、在冻结采样协议下满足机械间隙要求，并能在 PyBullet、Genesis 和 MuJoCo 中稳定运行。

该协议不能支持以下结论：

- 几何外观与真实物体一致；
- joint type、axis、origin 或 limit 在语义上符合真实物体；
- mass、inertia、friction 或 material 参数与现实真值一致；
- 离散采样之外的连续配置空间完全无碰撞；
- 跨仿真器一致等同于真实世界动力学准确。

## 3. Reporting Notes

- PhysX-Mobility 与 PartNet-Mobility 存在来源重叠，不能描述为两个独立几何来源的数据集。
- LAM 只评测实际取得并冻结的 released outputs，不能外推至所有 LAM 生成结果。
- SketchMobility 为二次策展合辑（Sketch2Arti 配套发布），与 Articraft-10K 存在部分来源重叠（205 个 Articraft `Agentic` 资产）；不得描述为与其全部上游来源相互独立的数据集，其结果不得与上游来源数据集的结果直接叠加比较。
- SketchMobility 为混合许可发布（CC BY 4.0 与 GPL-3.0 成分并存，见其 `LICENSE.md` / `LICENSE_MAP.json`）；冻结评测启动前必须确认许可条款不妨碍统一评测的执行、缓存与结果分发。
- `N/R` 表示原论文或发布页未明确报告。
- `TBD` 表示指标已经定义，但尚无与该定义一致、完成且冻结的正式结果；它既不是 0，也不是 `N/E`。
- `N/E` 表示在已经冻结并实际应用的协议下无法评测，或数学分母确为零；不得用 `N/E` 表示 evaluator 尚未实现、尚未运行或结果尚未审核，也不得把它替换为 0。
- `PARTIAL` 只用于无法安全 fail-closed 填补的数值诊断：单元格必须同时写出 `measured / intended` coverage；所有 pass-rate 指标仍保留完整 intent-to-evaluate 分母，不能因 `PARTIAL` 缩小分母。
- 本次新增 Table 2 supplementary、Table 4a、Table 4b 和 S1 不追溯改变既有 `Strict URDF Pass`、`Strict Collision Pass` 或 N=800 结果。未来纳入 strict 必须使用新协议版本并对全部比较方法统一重跑。
- 除完整结果外，应按 category、joint type、joint count bin 和 link count bin 报告 supplementary breakdown。
- 主文不得只展示成功加载的样本；所有统计都必须保留完整 intent-to-evaluate 分母。

## 4. Read-only metric reconnaissance provenance (not formal results)

| Dataset | Absolute data root | Current scale at inspection | Reconnaissance sample | Deterministic selection scope | Core static observations in N=10 |
|---|---|---:|---:|---|---|
| PV-A | `/mnt/zsn/lyb/arti-skill/seed_exports/` | 65 generator directories / 686 `seed_*` assets | 10 | generator-stratified deterministic hash order；每个 generator 取首个 static-valid seed | 73 L；63 J（62 movable）；503 V / 503 C；collision-bearing links 73 / 73；complete-inertial links 0 / 73；analytic collision elements 413 / 503 |
| Articraft-10K | `/mnt/zsn/lyb/arti-skill/exp/Articraft-10K` | `released_urdf` N_release = 9,996 | 10 | `released_urdf` basename 排序后，顺序取前 10 个 static-valid assets | 40 L；30 J；226 V / 0 C；collision-bearing links 0 / 40；complete-inertial links 21 / 40 |
| Artiverse | `/mnt/zsn/lyb/arti-skill/exp/artiverse` | frozen local roster N_release = 3,544 | 10 | raw category 排序后的前 10 类；每类取首个 static-valid asset | 39 L；29 J；68 V / 633 C；collision-bearing links 38 / 39；complete-inertial links 39 / 39 |
| PartNet-Mobility | `/mnt/zsn/lyb/arti-skill/exp/PartNet-Mobility` | frozen local roster N_release = 2,347 | 10 | `model_cat` 排序后的前 10 类；每类取最小 static-valid ID | 71 L；61 J；424 V / 424 C；collision-bearing links 58 / 71；complete-inertial links 0 / 71 |
| SketchMobility | `/mnt/zsn/lyb/arti-skill/exp/SketchMobility` | chunks manifest `objectCount` = 4,956（未解包） | 0 | 不适用：数据仍为 3 个 `tar.gz` chunk，未解包，无可抽样资产目录 | 未执行；解包并完成只读侦察前不得填充本行 |

这里的 L/J/V/C 分别表示静态读取到的 link、joint、visual element 和 collision element 数。`static-valid` 只表示本次只读侦察所用的 XML/tree/endpoint/resource 条件通过；候选失败时会顺延，因此这是 valid-filtered 条件性诊断，不是四库的随机或冻结 formal cohort。N=10 数字不得用于成功率、显著性、正式表格填充或数据集排名；本节也不声称运行了标准 parser、FK/collision sweep、PyBullet、Genesis、MuJoCo 或任何仿真评测。SketchMobility 行仅登记获取与解包状态：发布 chunk 已完整下载并由 `dataset_chunks/manifest.json` 记录 SHA-256，但尚未解包，未执行任何 N=10 只读侦察。
