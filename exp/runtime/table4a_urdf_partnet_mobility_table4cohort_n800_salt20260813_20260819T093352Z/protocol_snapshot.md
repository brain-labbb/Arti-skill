# 10K Sim-Ready URDF Dataset: Automatic Evaluation Protocol

本文档定义一套不依赖配对 GT 或人工评价的自动评测协议，用于比较以下六组铰接资产：

- Ours-500K
- Articraft-10K
- LAM released outputs
- Artiverse
- PartNet-Mobility
- PhysX-Mobility

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
- **Shared-category Balanced Cohort**：六组数据共同覆盖的类别；每类使用相同资产数，并以固定随机种子抽样。

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

- 下文新增的 Table 2 supplementary、Table 4a、Table 4b 和 Supplementary Table S1 是协议修订提案。截至本文档本次修订，Table 2 supplementary 已由冻结 evaluator 完成 PartNet-Mobility 与 LAM released outputs 的正式运行；两次运行冻结的 `lam_supplementary_static` atom 模块版本不同（分别为 `1c2fdc2c3d9f8ebcb3ab6b0bf8144b307c86b4b44790cf3182c2395ab37267ff` 与 `04985b5adc97275f940c29bbb584e8f0b6d1dd62cd5ba543d1c71c4a64ae6cc5`，共享模块在两次运行之间被修改），最终跨方法比较前必须对全部方法统一重跑或做版本等价性确认；其余方法与 Table 4a、Table 4b、S1 仍无与新增定义完全一致的冻结 evaluator 和正式结果，相应单元格记为 `TBD`。
- 新增指标不追溯改变既有 Table 2 的 `Strict URDF Pass`、Table 4 的 `Strict Collision Pass`、其冻结分母、历史 q=0 状态或已经写入的 N=800 数值。若未来将新增指标纳入 strict pass，必须发布新的协议版本并对全部方法在同一冻结 cohort 上重跑。
- 四个数据根上的每库 N=10 检查仅用于只读 metric reconnaissance。它是 valid-filtered 静态诊断，不是正式随机样本、成功率估计或数据集排名，也不得写回任何 proposed 正式结果单元格。
- 正式填充新增表前，必须冻结 cohort identity、`N_eval`/`J_eval`、pair policy、method-specific allowance 边界、阈值、尺度、surface sampler、默认质量注册表、计时环境、runner 和可重放 receipt。
- 对 LAM released outputs 的后续 Table 4a 专项运行，状态碰撞 oracle 固定为独立版本化的 `Genesis contact-penetration` backend，而非既有 PyBullet Table 4 runner：每个冻结 q-state 只做 direct kinematic detection（不 `scene.step`）；仅当 eligible source-URDF link pair 的 Genesis-reported penetration 严格大于 `1e-6 m` 时判为非法。必须开启 self / neutral / adjacent collision candidate generation，并按原始 URDF 的 direct parent-child graph 手工实施 headline exclusion；不得因 Genesis 默认过滤或 visual fallback 改变 pair policy。
- Genesis contact 不提供所有分离 pair 的完整 exact signed clearance。因此 LAM 的 `Normalized Clearance P5` 不得由 contact count、penetration、SDF 或 AABB 推导；除非另有独立 exact-distance backend 通过冻结资格验证，否则正式单元格记为 `N/E`。Table 4b 的双向 surface P95 继续使用独立的 exact surface backend，不以 Genesis SDF 代替。既有 PyBullet Table 4 数值保持历史证据，不能与这个新 protocol version 混写或覆盖。

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

---

## Table 2. URDF Validity and Structural Integrity

| Dataset / Outputs | Parse Rate ↑ | Resource Resolution ↑ | Finite Fields ↑ | Valid Tree ↑ | Valid Joint Spec. ↑ | Collision Coverage ↑ | Inertial Coverage ↑ | Inertia Validity ↑ | Strict URDF Pass ↑ |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Ours-500K | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| Articraft-10K | 800 / 800 (100.00%) | 800 / 800 (100.00%) | 800 / 800 (100.00%) | 800 / 800 (100.00%) | 800 / 800 (100.00%) | 223 / 800 (27.88%) | 317 / 800 (39.62%) | 317 / 800 (39.62%) | 10 / 800 (1.25%) |
| LAM released outputs | 719 / 800 (89.88%) | 777 / 800 (97.12%) | 800 / 800 (100.00%) | 733 / 800 (91.62%) | 759 / 800 (94.88%) | 372 / 800 (46.50%) | 25 / 800 (3.12%) | 25 / 800 (3.12%) | 24 / 800 (3.00%) |
| Artiverse | 797 / 800 (99.62%) | 800 / 800 (100.00%) | 800 / 800 (100.00%) | 797 / 800 (99.62%) | 800 / 800 (100.00%) | 777 / 800 (97.12%) | 800 / 800 (100.00%) | 800 / 800 (100.00%) | 774 / 800 (96.75%) |
| PartNet-Mobility | 95 / 800 (11.88%) | 787 / 800 (98.38%) | 800 / 800 (100.00%) | 800 / 800 (100.00%) | 800 / 800 (100.00%) | 0 / 800 (0.00%) | 0 / 800 (0.00%) | 0 / 800 (0.00%) | 0 / 800 (0.00%) |
| PhysX-Mobility | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD |

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

### Table 2 supplementary. Collision, Joint, and Inertial Diagnostics (Proposed; PartNet-Mobility, LAM and Artiverse evaluated)

| Dataset / Outputs | Visual-bearing Collision Coverage ↑ | Joint-limit Portability ↑ | Joint Dynamics Coverage ↑ | Placeholder-mass Incidence ↓ |
|---|---:|---:|---:|---:|
| Ours-500K | TBD | TBD | TBD | TBD |
| Articraft-10K | TBD | TBD | TBD | TBD |
| LAM released outputs | 372 / 800 (46.50%) | 1991 / 2395 (83.13%) | 0 / 2395 (0.00%) | N/E |
| Artiverse | 800 / 800 (100.00%) | 3,742 / 3,875 (96.57%) | 0 / 3,875 (0.00%) | N/E |
| PartNet-Mobility | 800 / 800 (100.00%) | 0 / 4078 (0.00%) | 0 / 4078 (0.00%) | N/E |
| PhysX-Mobility | TBD | TBD | TBD | TBD |

#### Table 2 supplementary metric definitions

| Metric | Definition |
|---|---|
| `Visual-bearing Collision Coverage` | 主值为资产级 `passed / N_eval`：资产必须可解析、至少包含一个在 XML 中声明 `<visual>` geometry 的 visual-bearing link，且每个此类 link 都至少包含一个资源可解析、可加载的 collision geometry；解析失败、visual/collision 资源失败或零 visual-bearing link 均 fail closed。另补充报告 `covered visual-bearing links / L_visual_declared` 的 link-micro 值及 link extraction coverage。该指标补充而不替换既有按全部声明 link 计算的 `Collision Coverage`。 |
| `Joint-limit Portability` | 关节级 `passed / J_eval`。bounded revolute/prismatic joint 必须具有有限的 `lower < upper`、有限且非负的 `effort` 和有限且为正的 `velocity`；continuous joint 不要求有限 lower/upper，但仍须满足冻结 adapter 共同要求的 effort/velocity 字段。其他 joint type 按查看结果前冻结的 per-type mapping 处理；缺字段、unsupported mapping 和未执行项均保留为失败。 |
| `Joint Dynamics Coverage` | 同时声明有限、非负 `damping` 与 `friction` 的 movable joints 数除以 `J_eval`；缺失任一字段计为未覆盖。该项只衡量字段覆盖，不证明数值经过动力学校准，也不进入既有 `Strict URDF Pass`。 |
| `Placeholder-mass Incidence` | 在具有 complete inertial 的动态 link 中，mass 或完整 inertial tuple 命中预注册 exporter/simulator 默认模板的 link 数除以 complete-inertial link 数，并同时报告 `complete-inertial links / dynamic links` coverage。默认模板或 sentinel 只能来自冻结工具默认值或公开文档，禁止查看方法结果后添加；分母为零时，正式运行后记为 `N/E` 而不是 0。该项是诊断 flag，不证明被标记质量一定错误，也不进入既有 `Strict URDF Pass`。 |

除 PartNet-Mobility、LAM released outputs 与 Artiverse 外的单元格仍为 `TBD`；不得用 N=10 reconnaissance、旧 Table 2 asset records 或 XML 字段存在性直接填数。其余方法的正式结果须由同一版本 evaluator 统一重跑。

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

---

## Table 3. Kinematic Executability

| Dataset / Outputs | Valid Range ↑ | Joint Sweep Success ↑ | Non-degenerate Motion ↑ | Subtree Consistency ↑ | FK Round-trip Error ↓ | Joint-level Pass ↑ | Strict Kinematic Pass ↑ |
|---|---:|---:|---:|---:|---:|---:|---:|
| Ours-500K | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| Articraft-10K | 2,865 / 2,865 (100.00%) | 2,865 / 2,865 (100.00%) | 2,865 / 2,865 (100.00%) | 2,855 / 2,865 (99.65%) | 0.000000 normalized translation / 2.980232e-8 rad rotation (2,865 / 2,865 measured; COMPLETE) | 2,845 / 2,865 (99.30%) | 795 / 800 (99.38%) |
| LAM released outputs | 2,382 / 2,395 (99.46%) | 2,005 / 2,395 (83.72%) | 2,000 / 2,395 (83.51%) | 2,005 / 2,395 (83.72%) | 0.000000 normalized translation / 0.000000 rad rotation (2,005 / 2,395 measured; PARTIAL) | 2,000 / 2,395 (83.51%) | 692 / 800 (86.50%) |
| Artiverse | 3,875 / 3,875 (100.00%) | 3,854 / 3,875 (99.46%) | 3,782 / 3,875 (97.60%) | 3,854 / 3,875 (99.46%) | 0.000000 normalized translation / 0.000000 rad rotation (3,854 / 3,875 measured; PARTIAL) | 3,782 / 3,875 (97.60%) | 762 / 800 (95.25%) |
| PartNet-Mobility | 4,078 / 4,078 (100.00%) | 4,078 / 4,078 (100.00%) | 4,069 / 4,078 (99.78%) | 4,076 / 4,078 (99.95%) | 0.000000 normalized translation / 2.107342e-8 rad rotation (4,078 / 4,078 measured; COMPLETE) | 4,066 / 4,078 (99.71%) | 793 / 800 (99.12%) |
| PhysX-Mobility | TBD | TBD | TBD | TBD | TBD | TBD | TBD |

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

该表只证明 URDF 描述的运动学可以执行，不证明 joint type、axis、origin 或 limit 与真实物体语义一致。

---

## Table 4. Collision and Mechanical Clearance

| Dataset / Outputs | Rest All-pair CF ↑ | Rest Non-adjacent CF ↑ | Single-joint Sweep CF ↑ | Multi-joint Sobol CF ↑ | Collision-state Rate ↓ | AOR ↓ | Max Penetration ↓ | Collision-free Range ↑ | Strict Collision Pass ↑ |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Ours-500K | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| Articraft-10K | 13 / 800 (1.625%) | 187 / 800 (23.375%) | 156 / 800 (19.500%) | 147 / 800 (18.375%) | 86,157 / 112,165 (76.813%) | N/E | 0.476553 (223 / 800 measured; PARTIAL) | 14,292 / 60,165 (23.755%) | 147 / 800 (18.375%) |
| LAM released outputs | 0 / 800 (0.000%) | 113 / 800 (14.125%) | 101 / 800 (12.625%) | 91 / 800 (11.375%) | 88,097 / 100,631 (87.545%) | N/E | 0.782640 (321 / 800 measured; PARTIAL) | 4,832 / 50,295 (9.607%) | 91 / 800 (11.375%) |
| Artiverse | 12 / 800 (1.500%) | 320 / 800 (40.000%) | 277 / 800 (34.625%) | 292 / 800 (36.500%) | 76,889 / 133,375 (57.649%) | N/E | 0.629995 (797 / 800 measured; PARTIAL) | 22,154 / 81,375 (27.225%) | 254 / 800 (31.750%) |
| PartNet-Mobility | 24 / 800 (3.000%) | 622 / 800 (77.750%) | 591 / 800 (73.875%) | 579 / 800 (72.375%) | 47,881 / 137,638 (34.788%) | N/E | 0.633017 (787 / 800 measured; PARTIAL) | 48,011 / 85,638 (56.063%) | 567 / 800 (70.875%) |
| PhysX-Mobility | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD |

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

Articraft-10K 行为 overall micro average。结果严格复用 Table 2 冻结 manifest 中全部 800 项及原始顺序，没有重新抽样或按结果筛选；来源为本地冻结的 `camvsl/Articraft-10K@3c79d5a05bb7cb6bf7bfee5e090176636ee3ac65` 发布集（`N_release = 9,996`）。该 cohort 不是 Full Release Cohort 或 Shared-category Balanced Cohort；本轮没有冻结权威类别标签，因此 category-level macro average 记为 `N/E`。800 / 800 个 URDF 均为 valid rooted tree，共声明 2,865 个可评测非 fixed DoF；223 / 800 个资产具有完整 collision coverage 并完成加载与测量，577 个 coverage 不完整资产保留且不补抽，其中 576 个为 0 / L link coverage，1 个为 2 / 3，0 timeout、0 child runtime failure。`Collision-state Rate` 采用 fail-closed 分母：共预期 112,165 个状态，实际执行 33,143 个，79,022 个未执行状态计为 non-free；实际观测到 7,135 个碰撞状态，故表中分子为 86,157。`Max Penetration` 仅在完成测量的 223 / 800 个资产上有观测值，因此报告为 `PARTIAL`；归一化尺度为 PyBullet 中 q=0 时 collision shapes 的 union AABB diagonal。`AOR` 因未运行稳定的精确重叠体积计算而记为 `N/E`，未使用包围盒重叠代替；离散 sweep 不构成 CCD、关节语义正确性或物理动力学有效性结论。证据见 [report.md](runtime/urdf_table4_articraft10k_n800_20260814/report.md)、[summary.json](runtime/urdf_table4_articraft10k_n800_20260814/summary.json)、[frozen_manifest.json](runtime/urdf_table4_articraft10k_n800_20260814/frozen_manifest.json)、[verification.json](runtime/urdf_table4_articraft10k_n800_20260814/verification.json)、[asset_records.json](runtime/urdf_table4_articraft10k_n800_20260814/asset_records.json)、[state_records.jsonl](runtime/urdf_table4_articraft10k_n800_20260814/state_records.jsonl) 和 [protocol_document_at_freeze.md](runtime/urdf_table4_articraft10k_n800_20260814/protocol_document_at_freeze.md)。Table 2 manifest SHA-256 为 `13c47e2b2affadb951a01cab826bae139852fca5769e99ec081cc916ffa6373d`，冻结 selected ordered ID SHA-256 为 `79c44441600077513d3cde1cda8fef38324e1a0ee660730b860d5313f0ae9784`，冻结 Table 4 manifest 文件 SHA-256 为 `6b4275cf3da29244af70c04acecd87094f0c158dee992db20b04e90c05292c20`、self-hash 为 `1c6ba7d9e19818580fe8573cf95bb1d065bf2235d0699070516888520f86d7b6`，runner SHA-256 为 `1e04f5a70a3b3b51f21ce9471c1ee52ae2fd09cc9c6bd1049b50e382a9cb0648`，collision core SHA-256 为 `e710d15cb79c50506487ff1335a88591bb58c11cf726c71198103c05f6d01ff0`。该运行冻结的是结果写回前的 protocol snapshot SHA-256 `be3813e1b40b4fb8e2ee5cf9bec89aa3b83d7dcca3050a0c6c3eeb3097c36ed1`；最终验证 31 / 31 checks PASS，800 / 800 个 authoritative child 和 800 / 800 个 frozen measurement replay 均闭合，本段属于运行后的报告更新。

LAM released outputs 行为 overall micro average。结果严格采用给定的 [Table 3 asset_records.jsonl](runtime/urdf_table3_lam_n800_seed20260813_20260814T022400Z_v3/asset_records.jsonl) 作为固定 membership authority，并与同目录的 [Table 3 manifest.json](runtime/urdf_table3_lam_n800_seed20260813_20260814T022400Z_v3/manifest.json) 按 `selection_rank = 1..800` 交叉重建顺序；没有采用 JSONL 的物理 worker 写入顺序、重新抽样或按结果筛选。来源为本地冻结的 `YipengGao/Articulated-Object-Code@28cec4f5be7e34fd4d586879ecfcb67f7c5e4cc0` 发布集（`N_release = 3,217`，`N_eval = 800`，seed `20260813`）；固定样本包含 621 个 `viable`、75 个 `loads_only`、104 个 `broken`，覆盖 305 个 observed raw category，不是 Full Release Cohort 或 Shared-category Balanced Cohort。323 / 800 个资产加载成功，其中 321 个完成全部测量；477 个预声明 package-audit failure 与 2 个加载成功但存在不可评测 joint range 的 partial 资产全部保留且不补抽，0 timeout。冻结状态分母为 100,631 = 800 个 rest + 50,295 个 single-joint + 49,536 个 Sobol 状态；实际执行 41,481 个，59,150 个未执行状态按 fail-closed 计为 non-free，实际观测到 28,947 个碰撞状态，因此 `Collision-state Rate` 分子为 88,097 = 28,947 + 59,150。`Max Penetration` 仅以 321 / 800 个 measurement-complete 资产计算，最大归一化值为 `0.782640066199297`，状态为 `PARTIAL`；归一化尺度为 PyBullet 中 q=0 时 collision shapes 的 union AABB diagonal。`AOR` 因未运行稳定的精确重叠体积计算而记为 `N/E`，未以包围盒重叠替代；评测只使用 collision geometry、没有 visual fallback，离散 sweep 不构成 CCD、关节语义正确性或物理动力学有效性结论。305 类等权 category-level macro average 为 Rest All-pair CF 0.000%、Rest Non-adjacent CF 24.986%、Single-joint Sweep CF 22.761%、Multi-joint Sobol CF 21.512%、Collision-state Rate 74.596%、Collision-free Range 25.076%、Strict Collision Pass 21.512%。正式证据见 [report.md](runtime/urdf_table4_lam_n800_20260814/report.md)、[summary.json](runtime/urdf_table4_lam_n800_20260814/summary.json)、[frozen_manifest.json](runtime/urdf_table4_lam_n800_20260814/frozen_manifest.json)、[verification.json](runtime/urdf_table4_lam_n800_20260814/verification.json)、[asset_records.json](runtime/urdf_table4_lam_n800_20260814/asset_records.json)、[state_records.jsonl](runtime/urdf_table4_lam_n800_20260814/state_records.jsonl) 和 [protocol_document_at_freeze.md](runtime/urdf_table4_lam_n800_20260814/protocol_document_at_freeze.md)。给定 Table 3 JSONL 文件 SHA-256 为 `7ef1c38d61bc780e41f62c7dd359e66f0bfeabe655c7453c93e2ea9830122d94`，Table 3 manifest 文件 SHA-256 为 `7e16683bfe4e4f37d7972082d8512713c1d8d1ae4ce142b75bf7dfb0509b9951`、self-hash 为 `f8f7fe4da5634d4f806e793c0da919689eab25be1ce0bbed7e2232f3453d15c2`，冻结 selected ordered asset-key SHA-256 为 `643aa5b76ac61f57dd943bee26444a3525c01201a8dff3443763a7fd8d8267d3`。正式 Table 4 manifest 文件 SHA-256 为 `8adc7d8698eaeab5ee5a62d881ed50d4e65c5dc80c9d1d8ae0f4a4a204474594`、self-hash 为 `9a46a1cb7668666cf3c485cc35086cdd79a113d23a8b00625ede012c8b039d2d`、items SHA-256 为 `ef29649907fe7c6ccd08bda75c9693b233b8601ec90b1b05a8d0c68b7bf5b5cc`；runner SHA-256 为 `cdba0dccbef991b2ed4a3f4e418f28725d4e233cad758b81431ce78cb1bbdd4a`，collision core SHA-256 为 `e710d15cb79c50506487ff1335a88591bb58c11cf726c71198103c05f6d01ff0`，结果写回前的 protocol snapshot SHA-256 为 `c59de12cd9b51fc8556291d4a590a36115060df259ee31f0ccf3e87fccb19d86`。最终 [verification.json](runtime/urdf_table4_lam_n800_20260814/verification.json) 文件 SHA-256 为 `e74ed91dca984af8aba900cf3915b490fb1298e5c2bc539af7ade43570edbc51`，31 / 31 checks PASS，800 / 800 个 authoritative child 和 800 / 800 个 frozen measurement replay 均闭合；本段属于运行后的报告更新。

Artiverse 结果严格复用 Table 1 manifest 中原顺序的固定 cohort（预发布 `N_release = 3,544`，`N_eval = 800`，覆盖 67 个观测 raw category），不是 Full Release Cohort 或 Shared-category Balanced Cohort。3 个抽中资产的 URDF joint graph 含环、不是合法 rooted tree，均保留且不补抽；797 / 800 个资产完成加载与测量，0 timeout。`Collision-state Rate` 采用 fail-closed 分母：共预期 133,375 个状态，实际执行 132,739 个状态，636 个未执行状态计为 non-free；实际观测到 76,253 个碰撞状态，故表中分子为 76,889。`Max Penetration` 仅在完成测量的 797 个资产上有观测值，因此报告为 `PARTIAL`；归一化尺度为 PyBullet 中 q=0 时 collision shapes 的 union AABB diagonal。`AOR` 因未运行稳定的精确重叠体积计算而记为 `N/E`，未使用包围盒重叠代替；离散 sweep 不构成 CCD、关节语义正确性或物理动力学有效性结论。67 类等权 category-level macro average 为 Rest All-pair CF 4.012%、Rest Non-adjacent CF 58.744%、Single-joint Sweep CF 47.682%、Multi-joint Sobol CF 47.763%、Collision-state Rate 39.261%、Collision-free Range 57.756%、Strict Collision Pass 45.037%。证据见 [report.md](runtime/urdf_table4_artiverse_table1_n800_20260814/report.md)、[summary.json](runtime/urdf_table4_artiverse_table1_n800_20260814/summary.json)、[frozen_manifest.json](runtime/urdf_table4_artiverse_table1_n800_20260814/frozen_manifest.json) 和 [verification.json](runtime/urdf_table4_artiverse_table1_n800_20260814/verification.json)；Table 1 manifest SHA-256 为 `f74575692b87605699c4f349186c4660d691c91bef39562bb976baf22ae72a8c`，冻结 Table 4 manifest SHA-256 为 `0e69335a3d1574a1e1510124ade6e743cfd66fe894c1da3816b072954c75aedb`，独立验证 24 / 24 checks PASS。

PartNet-Mobility 结果来自本地冻结 release roster 的 2,347 个资产中预先冻结的确定性抽样 cohort（`N_eval = 800`，覆盖 46 类），不是 Full Release Cohort 或 Shared-category Balanced Cohort。该本地数据状态为 `LOCAL_COMPLETE_PROVENANCE_LIMITED`：固定源 `sapien-sim/PartNetMobility@ee0aa3ef1df16181d76d83f7415aa8c94ed1da8f` 的 gated revision 对象 bytes 未与本地文件直接认证。13 个抽中资产包含缺失的 collision mesh 引用，均保留且不补抽；787 / 800 个资产完成加载与测量，0 timeout。`Collision-state Rate` 采用 fail-closed 分母：共预期 137,638 个状态，实际执行 136,100 个状态，1,538 个未执行状态计为 non-free；其中实际观测到 46,343 个碰撞状态。`Max Penetration` 仅在完成测量的 787 个资产上有观测值，因此报告为 `PARTIAL`，但资产级展示分母仍为 800。`AOR` 因未运行稳定的精确重叠体积计算而记为 `N/E`，未使用包围盒重叠代替。证据见 [report.md](runtime/urdf_table4_partnet_mobility_n800_20260813/report.md)、[summary.json](runtime/urdf_table4_partnet_mobility_n800_20260813/summary.json)、[frozen_manifest.json](runtime/urdf_table4_partnet_mobility_n800_20260813/frozen_manifest.json) 和 [verification.json](runtime/urdf_table4_partnet_mobility_n800_20260813/verification.json)；冻结 manifest SHA-256 为 `2ff015ee6bb377ce693126b52dd632a7565a3eaa9f0007e26122a1bb4ab99900`，独立验证 14 / 14 checks PASS。

### Table 4a. DoF-aware Mechanical Safety (Proposed; not yet evaluated)

| Dataset / Outputs | Joint-level Full-range CF ↑ | Executable CF DoF/Asset ↑ | Collision-safe DoF Retention ↑ | Normalized Clearance P5 ↑ | Limit Reachability ↑ |
|---|---:|---:|---:|---:|---:|
| Ours-500K | TBD | TBD | TBD | TBD | TBD |
| Articraft-10K | TBD | TBD | TBD | TBD | TBD |
| LAM released outputs | TBD | TBD | TBD | TBD | TBD |
| Artiverse | TBD | TBD | TBD | TBD | TBD |
| PartNet-Mobility | TBD | TBD | TBD | TBD | TBD |
| PhysX-Mobility | TBD | TBD | TBD | TBD | TBD |

#### Table 4a metric definitions

| Metric | Definition |
|---|---|
| `Joint-level Full-range CF` | 关节级 `passed / J_eval`。对每个 movable joint 使用既有 Table 4 的冻结测试区间和 `K = 21` 状态，其他关节保持历史 `q = 0`；全部 intended states 均须成功执行，并在统一、无 method-specific allowance 的 headline pair policy 与穿透阈值下无非法碰撞。任何未执行状态、无效区间、加载失败或资源失败均使该关节 fail closed。该离散指标不证明连续配置空间无碰撞。 |
| `Executable CF DoF/Asset` | 每个资产中同时通过 Table 3 `Joint-level Pass` 和本表 `Joint-level Full-range CF` 的关节数；跨 `N_eval` 报告 mean / median / P90。解析、加载或测量失败资产的安全可执行 DoF 计为 0，不得从资产分母删除。 |
| `Collision-safe DoF Retention` | 上述安全可执行关节总数除以冻结 `J_eval`，写为 `passed / J_eval (percentage)`。它是与 raw DoF 数配套的保留率，不得以只统计成功加载资产的条件分母替代。 |
| `Normalized Clearance P5` | 对每个实际测得的 intended state，取 headline pair policy 下所有 eligible non-adjacent collision-surface pair 的最小 signed clearance，再除以统一 `D_visual`；负值表示穿透，零表示接触。先对每个资产取 state-level minimum 的 P5，数据集单元格报告这些 asset-level P5 的中位数，并同时写出 `measured / intended` state 和 asset coverage，以及 `COMPLETE` 或 `PARTIAL`。未测状态不伪造距离，但对应 pass metrics 仍 fail closed。 |
| `Limit Reachability` | bounded joint 的 lower 和 upper endpoint 都可执行、transform 有限且在 headline pair policy 下无非法碰撞时通过；报告 `passed / J_bounded (percentage)`。continuous joint 不进入 `J_bounded`，其分母必须显式报告；endpoint 未执行或不可加载计为失败。 |

Table 4a 还必须按查看结果前冻结的 declared movable DoF 分箱报告 `N_eval`、`Joint-level Full-range CF`、`Collision-safe DoF Retention` 和既有 `Strict Collision Pass`：`0`、`1`、`2--3`、`4--7`、`>=8`。无法可靠提取声明 DoF 的资产进入 `unknown/unparseable` 分箱并保留在整体 intent-to-evaluate 分母中。若新增 runner 沿用历史 Table 4 状态，必须保持其他关节为 q=0；不得把 clipped-zero 或其他新 neutral rule 的结果拼入同一行。

### Table 4b. Collision Representation Quality and Cost (Proposed; not yet evaluated)

| Dataset / Outputs | Analytic Collision Share ↑* | Visual→Collision P95 ↓ | Collision→Visual P95 ↓ | Shapes/Visual-bearing Link ↓* | Collision Mesh Triangles/Asset ↓* | Intra-link Redundancy ↓ | Collision Load Time/Asset ↓ |
|---|---:|---:|---:|---:|---:|---:|---:|
| Ours-500K | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| Articraft-10K | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| LAM released outputs | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| Artiverse | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| PartNet-Mobility | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| PhysX-Mobility | TBD | TBD | TBD | TBD | TBD | TBD | TBD |

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

本表统一定义 `D_visual` 为历史 q=0 下全部 loadable visual geometry 经同一单位变换后的 union AABB diagonal。`D_visual` 缺失、非有限或非正时，对应数值误差记为不可测并保留 coverage；不得回退到 collision AABB、发布元数据尺度或各数据集自带的不同尺度。surface sampler 必须在查看结果前冻结样本数、面积权重、随机种子、单位、坐标焊接容差和退化面处理。AABB 只能用于这里明确规定的尺度归一化，不能代替双向 surface distance 或 union-volume redundancy。所有数值单元格均须带 `measured / intended` coverage 和 `COMPLETE`/`PARTIAL`；当前六行全部为 `TBD`。

### Supplementary Table S1. Mechanical Evidence and Allowance Sensitivity (Proposed; not yet evaluated)

| Dataset / Outputs | Receipt-bound Assets ↑ | Receipt Replay Pass ↑ | Deterministic Rebuild Match ↑ | Allowance Density ↓ | Strict Pass (No Method-specific Allowance) ↑ | Registered-allowance Gain ↓ |
|---|---:|---:|---:|---:|---:|---:|
| Ours-500K | TBD | TBD | TBD | TBD | TBD | TBD |
| Articraft-10K | TBD | TBD | TBD | TBD | TBD | TBD |
| LAM released outputs | TBD | TBD | TBD | TBD | TBD | TBD |
| Artiverse | TBD | TBD | TBD | TBD | TBD | TBD |
| PartNet-Mobility | TBD | TBD | TBD | TBD | TBD | TBD |
| PhysX-Mobility | TBD | TBD | TBD | TBD | TBD | TBD |

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
| Articraft-10K | Genesis | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
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
| PhysX-Mobility | PyBullet | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| PhysX-Mobility | Genesis | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| PhysX-Mobility | MuJoCo | TBD | TBD | TBD | TBD | TBD | TBD | TBD |

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
| Articraft-10K | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| LAM released outputs | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| Artiverse | 8 / 800 (1.000%) | 0 / 800 (0.000%) | 0 / 800 (0.000%) | 0 / 800 (0.000%) | 0 / 800 (0.000%) | rev max 3095.693403; prism max 321.180421 | trans max 84.273932; rot max 3.141592 | 0 / 800 (0.000%) |
| PartNet-Mobility | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| PhysX-Mobility | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD |

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

这里的 L/J/V/C 分别表示静态读取到的 link、joint、visual element 和 collision element 数。`static-valid` 只表示本次只读侦察所用的 XML/tree/endpoint/resource 条件通过；候选失败时会顺延，因此这是 valid-filtered 条件性诊断，不是四库的随机或冻结 formal cohort。N=10 数字不得用于成功率、显著性、正式表格填充或数据集排名；本节也不声称运行了标准 parser、FK/collision sweep、PyBullet、Genesis、MuJoCo 或任何仿真评测。
