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
- PyBullet、Genesis、MuJoCo 的版本、timestep、gravity、solver、控制器和运行时长；
- 每项测试的超时、失败条件和 `N/E` 条件。

---

## Table 1. Dataset Scale and Structural Diversity

| Dataset / Outputs | Paper-reported Assets | N_release | N_eval | #Categories | Links/Asset | Movable Joints/Asset | Multi-joint Assets (%) ↑ | Unique Topologies (%) ↑ | Exact Duplicate Rate (%) ↓ |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Ours-500K | 10K | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| Articraft-10K | 10,018 | 9,996 | 800 | 240 / 222 | 4.96 / 4 / 8 | 3.58 / 3 / 6 | 79.62 | 39.12 (n=800) | 0.00 (n=800) |
| LAM released outputs | N/R | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| Artiverse | 5,402 | 3,544 | 800 | 84 / 67 | 8.59 / 5 / 16 | 4.84 / 2 / 7 | 78.75 | 35.88 (n=797) | 0.00 (n=797) |
| PartNet-Mobility | 2,346 | TBD | TBD | 46 | TBD | TBD | TBD | TBD | TBD |
| PhysX-Mobility | 2,024 | TBD | TBD | 47 | TBD | TBD | TBD | TBD | TBD |

### Table 1 metric definitions

| Metric | Definition |
|---|---|
| `Links/Asset` | 每个资产的 link 数量，报告 mean / median / P90。 |
| `Movable Joints/Asset` | 每个资产中声明的非 `fixed` XML joint 数量，报告 mean / median / P90。该声明层统计包含 exporter extension joint types，不等同于通过严格 URDF/runtime 验证的可执行 DoF 数。 |
| `Multi-joint Assets` | 至少包含两个非 fixed joints 的资产比例。 |
| `Unique Topologies` | 忽略 link/joint 名称、网格路径和数值参数后，对规范化有根运动树计算 graph hash；唯一 hash 数除以具有 valid rooted tree 的可评估资产数，并同时报告该分母相对 `N_eval` 的 coverage。 |
| `Exact Duplicate Rate` | 对规范化 URDF 及其递归可解析的 simulation-resource closure 计算 fingerprint；重复超额资产数除以 fingerprint 完整的可评估资产数，并同时报告该分母相对 `N_eval` 的 coverage。 |

`Paper-reported Assets` 只用于说明论文声称的数据规模，不作为任何成功率的分母。正式结果必须使用本地冻结的 `N_release` 和 `N_eval`。

Artiverse 结果来自本地预发布版本的固定全局样本（seed 20260813）。Table 1 的轻量 XML 语法解析为 800 / 800；拓扑和重复率在 797 个可评估资产上计算。该 XML 语法检查不同于 Table 2 中包含资源加载的标准 `urdfpy` 完整解析，后者为 797 / 800。

Articraft-10K 使用 Table 2 的同一固定样本（seed 20260813），全部指标分母均为 800。论文报告 245 类；本地发布集按资产 ID 联结官方 `record.json` 后覆盖 240 类，样本覆盖 222 类。

---

## Table 2. URDF Validity and Structural Integrity

| Dataset / Outputs | Parse Rate ↑ | Resource Resolution ↑ | Finite Fields ↑ | Valid Tree ↑ | Valid Joint Spec. ↑ | Collision Coverage ↑ | Inertial Coverage ↑ | Inertia Validity ↑ | Strict URDF Pass ↑ |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Ours-500K | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| Articraft-10K | 800 / 800 (100.00%) | 800 / 800 (100.00%) | 800 / 800 (100.00%) | 800 / 800 (100.00%) | 800 / 800 (100.00%) | 223 / 800 (27.88%) | 317 / 800 (39.62%) | 317 / 800 (39.62%) | 10 / 800 (1.25%) |
| LAM released outputs | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| Artiverse | 797 / 800 (99.62%) | 800 / 800 (100.00%) | 800 / 800 (100.00%) | 797 / 800 (99.62%) | 800 / 800 (100.00%) | 777 / 800 (97.12%) | 800 / 800 (100.00%) | 800 / 800 (100.00%) | 774 / 800 (96.75%) |
| PartNet-Mobility | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
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

Artiverse 行为 overall micro average。正式运行严格复用 `exp/runtime/table1_artiverse/manifest.json` 中 `.assets[].manifest_root` 的既有顺序，不重新抽样或按结果筛选：本地数据为 `PRE_RELEASE_SUBSET`（`N_release = 3,544`），固定全局样本为 `N_eval = 800`、seed `20260813`。800 个资产均完成评测，`error = 0`、`timeout = 0`；其中 774 个资产通过 Strict URDF，而不是 800 个资产全部通过。该 cohort 覆盖 67 个 observed `raw_category`，不是 Full Release Cohort，也不是 Shared-category Balanced Cohort。

| Artiverse category macro | Parse Rate ↑ | Resource Resolution ↑ | Finite Fields ↑ | Valid Tree ↑ | Valid Joint Spec. ↑ | Collision Coverage ↑ | Inertial Coverage ↑ | Inertia Validity ↑ | Strict URDF Pass ↑ |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 67 observed raw categories, unweighted mean | 99.79% | 100.00% | 100.00% | 99.79% | 100.00% | 94.30% | 100.00% | 100.00% | 94.09% |

Artiverse 证据见 [summary.json](runtime/table2_urdf_artiverse_table1cohort_n800_seed20260813_20260814T001002Z/summary.json)、[manifest.json](runtime/table2_urdf_artiverse_table1cohort_n800_seed20260813_20260814T001002Z/manifest.json) 和 [asset_records.jsonl](runtime/table2_urdf_artiverse_table1cohort_n800_seed20260813_20260814T001002Z/asset_records.jsonl)。冻结 cohort manifest SHA256 为 `f74575692b87605699c4f349186c4660d691c91bef39562bb976baf22ae72a8c`，选中资产 ID 哈希为 `118038a746cafb91251afde5eb4f1164915d141acb3b529ea721a9d376bde3fa`，formal manifest self-hash 为 `c4a65440a1b78e8195434a68368dd5a45e5e8310b86ae309e8065f6ab7b5c484`。该运行冻结的是结果写回前的 protocol SHA256 `8cb07983a85f49d485a35d7dc59ec08c2f02a38bb9b6c75e3aab7fb09be468fe`；本段属于运行后的报告更新。

---

## Table 3. Kinematic Executability

| Dataset / Outputs | Valid Range ↑ | Joint Sweep Success ↑ | Non-degenerate Motion ↑ | Subtree Consistency ↑ | FK Round-trip Error ↓ | Joint-level Pass ↑ | Strict Kinematic Pass ↑ |
|---|---:|---:|---:|---:|---:|---:|---:|
| Ours-500K | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| Articraft-10K | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| LAM released outputs | 2,382 / 2,395 (99.46%) | 2,005 / 2,395 (83.72%) | 2,000 / 2,395 (83.51%) | 2,005 / 2,395 (83.72%) | 0.000000 normalized translation / 0.000000 rad rotation (2,005 / 2,395 measured; PARTIAL) | 2,000 / 2,395 (83.51%) | 692 / 800 (86.50%) |
| Artiverse | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| PartNet-Mobility | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
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

LAM 行为 overall micro average。正式运行从本地冻结的 `YipengGao/Articulated-Object-Code@28cec4f5be7e34fd4d586879ecfcb67f7c5e4cc0` 发布 manifest 的全部 3,217 条记录中，以 seed `20260813` 在查看评测结果前确定性抽取 `N_eval = 800`；抽样单位为唯一的 `(tier, rel_path)`，不按 `viable`、`loads_only` 或 `broken` 标签预筛。冻结样本包含 621 个 `viable`、75 个 `loads_only` 和 104 个 `broken` 资产，覆盖 305 个 observed category，共声明 `J_eval = 2,395` 个非 fixed joints。该结果是 frozen global random cohort，不是 Full Release Cohort 或 Shared-category Balanced Cohort。

800 / 800 个 URDF 均完成 XML 解析，733 / 800 具有 valid rooted tree；运行状态为 798 个 `completed`、2 个 retained asset error、0 timeout。两条 error 均来自资产声明了当前冻结 FK 协议不支持的 `floating` joint，资产及其声明关节仍保留在分母中。q0 下 visual/collision geometry 联合 AABB 的尺度推导为 719 个 `COMPLETE`、12 个 `NOT_EVALUABLE`（mesh 缺失或顶点非有限）、2 个因 initial FK 失败而不可得、67 个因 invalid tree 而不可得。

`FK Round-trip Error` 的 0.000000 只是在成功完成 round-trip 的 2,005 / 2,395 个关节上观测到的最大归一化平移误差和最大旋转误差；其余 390 个关节未完成该测量，因此明确报告为 `PARTIAL`，并在 `Joint-level Pass` 中按失败保留。该离散 FK 结果不证明 joint semantic correctness、连续配置空间、碰撞安全或动力学有效性。

| LAM category macro | Valid Range ↑ | Joint Sweep Success ↑ | Non-degenerate Motion ↑ | Subtree Consistency ↑ | FK Round-trip Error ↓ | Joint-level Pass ↑ | Strict Kinematic Pass ↑ |
|---|---:|---:|---:|---:|---:|---:|---:|
| 305 observed categories, unweighted mean (302 with declared movable joints) | 96.19% | 81.43% | 80.11% | 81.43% | N/E | 80.11% | 79.32% |

LAM 证据见 [summary.json](runtime/urdf_table3_lam_n800_seed20260813_20260814T022400Z_v3/summary.json)、[summary.md](runtime/urdf_table3_lam_n800_seed20260813_20260814T022400Z_v3/summary.md)、[manifest.json](runtime/urdf_table3_lam_n800_seed20260813_20260814T022400Z_v3/manifest.json) 和 [asset_records.jsonl](runtime/urdf_table3_lam_n800_seed20260813_20260814T022400Z_v3/asset_records.jsonl)。冻结 selected-cohort SHA256 为 `643aa5b76ac61f57dd943bee26444a3525c01201a8dff3443763a7fd8d8267d3`，formal manifest self-hash 为 `f8f7fe4da5634d4f806e793c0da919689eab25be1ce0bbed7e2232f3453d15c2`，evaluator SHA256 为 `0da075f077ce13c78bb6b4ee66b0abe77668ccf7bb3c105660b321e667fc2acf`。该运行冻结的是结果写回前的 protocol SHA256 `8115a160ab229aa52f3c98498a652851bc27eabc96feb17e1463e61541f6cf22`；本段属于运行后的报告更新。

该表只证明 URDF 描述的运动学可以执行，不证明 joint type、axis、origin 或 limit 与真实物体语义一致。

---

## Table 4. Collision and Mechanical Clearance

| Dataset / Outputs | Rest All-pair CF ↑ | Rest Non-adjacent CF ↑ | Single-joint Sweep CF ↑ | Multi-joint Sobol CF ↑ | Collision-state Rate ↓ | AOR ↓ | Max Penetration ↓ | Collision-free Range ↑ | Strict Collision Pass ↑ |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Ours-500K | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| Articraft-10K | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| LAM released outputs | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| Artiverse | 12 / 800 (1.500%) | 320 / 800 (40.000%) | 277 / 800 (34.625%) | 292 / 800 (36.500%) | 76,889 / 133,375 (57.649%) | N/E | 0.629995 (797 / 800 measured; PARTIAL) | 22,154 / 81,375 (27.225%) | 254 / 800 (31.750%) |
| PartNet-Mobility | 24 / 800 (3.000%) | 622 / 800 (77.750%) | 591 / 800 (73.875%) | 579 / 800 (72.375%) | 47,881 / 137,638 (34.788%) | N/E | 0.633017 (787 / 800 measured; PARTIAL) | 48,011 / 85,638 (56.063%) | 567 / 800 (70.875%) |
| PhysX-Mobility | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD |

### Table 4 evaluation states

- Rest state：URDF 声明的初始关节状态。
- Single-joint sweep：沿用 Table 3 的每关节 `K = 21` 个状态。
- Multi-joint states：每个资产使用固定种子的 `R = 64` 个 Sobol joint configurations。
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

Artiverse 结果严格复用 Table 1 manifest 中原顺序的固定 cohort（预发布 `N_release = 3,544`，`N_eval = 800`，覆盖 67 个观测 raw category），不是 Full Release Cohort 或 Shared-category Balanced Cohort。3 个抽中资产的 URDF joint graph 含环、不是合法 rooted tree，均保留且不补抽；797 / 800 个资产完成加载与测量，0 timeout。`Collision-state Rate` 采用 fail-closed 分母：共预期 133,375 个状态，实际执行 132,739 个状态，636 个未执行状态计为 non-free；实际观测到 76,253 个碰撞状态，故表中分子为 76,889。`Max Penetration` 仅在完成测量的 797 个资产上有观测值，因此报告为 `PARTIAL`；归一化尺度为 PyBullet 中 q=0 时 collision shapes 的 union AABB diagonal。`AOR` 因未运行稳定的精确重叠体积计算而记为 `N/E`，未使用包围盒重叠代替；离散 sweep 不构成 CCD、关节语义正确性或物理动力学有效性结论。67 类等权 category-level macro average 为 Rest All-pair CF 4.012%、Rest Non-adjacent CF 58.744%、Single-joint Sweep CF 47.682%、Multi-joint Sobol CF 47.763%、Collision-state Rate 39.261%、Collision-free Range 57.756%、Strict Collision Pass 45.037%。证据见 [report.md](runtime/urdf_table4_artiverse_table1_n800_20260814/report.md)、[summary.json](runtime/urdf_table4_artiverse_table1_n800_20260814/summary.json)、[frozen_manifest.json](runtime/urdf_table4_artiverse_table1_n800_20260814/frozen_manifest.json) 和 [verification.json](runtime/urdf_table4_artiverse_table1_n800_20260814/verification.json)；Table 1 manifest SHA-256 为 `f74575692b87605699c4f349186c4660d691c91bef39562bb976baf22ae72a8c`，冻结 Table 4 manifest SHA-256 为 `0e69335a3d1574a1e1510124ade6e743cfd66fe894c1da3816b072954c75aedb`，独立验证 24 / 24 checks PASS。

PartNet-Mobility 结果来自完整发布集 2,347 个资产中预先冻结的确定性抽样 cohort（`N_eval = 800`，覆盖 46 类），不是 Full Release Cohort 或 Shared-category Balanced Cohort。13 个抽中资产包含缺失的 collision mesh 引用，均保留且不补抽；787 / 800 个资产完成加载与测量，0 timeout。`Collision-state Rate` 采用 fail-closed 分母：共预期 137,638 个状态，实际执行 136,100 个状态，1,538 个未执行状态计为 non-free；其中实际观测到 46,343 个碰撞状态。`Max Penetration` 仅在完成测量的 787 个资产上有观测值，因此报告为 `PARTIAL`，但资产级展示分母仍为 800。`AOR` 因未运行稳定的精确重叠体积计算而记为 `N/E`，未使用包围盒重叠代替。证据见 [report.md](runtime/urdf_table4_partnet_mobility_n800_20260813/report.md)、[summary.json](runtime/urdf_table4_partnet_mobility_n800_20260813/summary.json)、[frozen_manifest.json](runtime/urdf_table4_partnet_mobility_n800_20260813/frozen_manifest.json) 和 [verification.json](runtime/urdf_table4_partnet_mobility_n800_20260813/verification.json)；冻结 manifest SHA-256 为 `2ff015ee6bb377ce693126b52dd632a7565a3eaa9f0007e26122a1bb4ab99900`，独立验证 14 / 14 checks PASS。

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
| Artiverse | PyBullet | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| Artiverse | Genesis | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
| Artiverse | MuJoCo | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
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
| Artiverse | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD |
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

---

## 2. Headline Metrics and Claim Boundary

建议将以下四项作为主文 headline metrics：

1. `Strict URDF Pass`：静态结构和物理字段全部合法；
2. `Strict Kinematic Pass`：全部声明关节运动学可执行；
3. `Strict Collision Pass`：冻结采样状态和 pair policy 下无非法穿透；
4. `Strict Sim-ready`：前三项通过，并且在 PyBullet、Genesis、MuJoCo 中全部通过运行时测试和跨仿真器一致性阈值。

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
- `N/R` 表示原论文或发布页未明确报告；`N/E` 表示依据冻结协议无法评测；二者均不得替换为 0。
- 除完整结果外，应按 category、joint type、joint count bin 和 link count bin 报告 supplementary breakdown。
- 主文不得只展示成功加载的样本；所有统计都必须保留完整 intent-to-evaluate 分母。
