# Table 5 仿真可执行性与跨仿真器一致性评测方案 v2

状态：**正式运行已完成（2026-08-29）；结果已回填 §5/§6；详见 `PV-A-Table5-Sim-Readiness-Paper-Ready.md`；run classification = INCOMPLETE（10 个 LAM 资产在 PyBullet 原生加载时段错误，已按固定分母 200 计负，2026-08-29 用户确认接受现状）**
更新日期：2026-08-29
主评测规模：每个数据集 200 个资产，共 8 个数据集、1,600 个资产

## 1. 最终设计

Table 5 拆成两个表：

- **Table 5a：Genesis simulation readiness**，回答资产能否在一个主仿真器中正确导入、保持数值稳定并完成关节轨迹。
- **Table 5b：Cross-simulator portability**，分别报告同一 canonical asset 在 Genesis、PyBullet、MuJoCo 中的导入、DoF 覆盖、Stable Rollout 和运动学误差。

主表只使用以下行业中普遍可理解的基础量：

- success rate；
- DoF coverage；
- RMSE / normalized RMSE；
- Euclidean position error；
- SO(3) geodesic rotation error；
- percentile，主表使用 P95。

不再把 `JPSR`、`PER`、`MR-CFR`、`JLCR`、`STB` 或 `XSim-KE` 写成仿佛已有统一定义的行业标准指标。这些名称没有跨论文、跨引擎统一的标准定义，容易引起审稿人对自定义指标和重复计数的质疑。

`Stable Rollout Rate` 仍然保留，因为 stability 是需要回答的问题；但必须明确它是本 benchmark 的**操作性通过率**，不是行业统一标准。它只表示仿真过程完成且状态保持有限，不表示物体自由放置时不会倾倒，也不表示真实世界动力学准确。

## 2. 冻结评测集

正式 cohort（五个原 baseline 已从各自完整原始 release roster 重新抽样；Articraft-10K 从 GitHub 合并的 10,787 源清单抽样；Infinigen-Sim 从官方 URDF/MJCF 同资产交集抽样；PV-A 从完整 release roster 抽样）：

```text
exp/runtime/table5_v2_core200_five_full_release_articraft10787_infinigen_paired_official/cohort_manifest.json
```

关键绑定：

```text
protocol_id     = table5-v2-core200-hash-sample-v1
selection_seed  = arti-skill-table5-v2-core200-20260828
cohort_sha256   = 660dbeeb01fcfa379c6ec33a572c78553bc06571d586407f92f52dc852781a60
manifest_sha256 = 3085bb408276f71e8c00173405f859c902cfb840645f12b6b5f9ebad83b69ebf
selection_protocol_sha256 = c6f6e4ff3d07a3d925e17685c9693b9c11849b0065057f53d66f6204100ef62b
```

r2 正式 prepared manifest 已生成并冻结（2026-08-29）：

```text
prepared manifest = exp/runtime/table5_v2_core200_prepared_five_full_release_articraft10787_infinigen_paired_official_metrics_r2/manifest.json
prepared manifest_sha256   = 5f481188a39bfa77bf7c140c5bce21eb3db6b8fcbddc2965ae75bebed83ce146
prepared protocol_sha256   = 5abe4976f3bd9554365699366bebbe2eba76d52c00da8dd6ca6c88e6f8041e84
prepared_cohort_sha256     = bdd3bb53d729e6f759145a8e9a7f8c293e91d9cedb23a4e8bf5f06b6f155352c
v2_runtime_script_sha256   = a353c62f176df9f8058440155a35c0455951a2939990cd035a7f20dbf70ba605
```

不得把旧 N=800 parent-roster cohort 的 prepared manifest 用于正式运行，也不得为生成新 prepared manifest 而重新抽样。

### 2.1 候选范围

八个数据集都从各自完整、冻结且与本次仿真结果无关的原始 release universe 抽样。Infinigen-Sim 的 universe 是官方 URDF 与官方 MJCF 的 8,225-asset identity intersection；其三个引擎始终使用同一组资产 ID。

| Dataset | Candidate | Eligible | Excluded | Selected |
|---|---:|---:|---:|---:|
| Articraft-10K | 10,787 | 10,690 | 97 | 200 |
| LAM released outputs | 3,217 | 3,074 | 143 | 200 |
| Artiverse | 3,544 | 3,472 | 72 | 200 |
| PartNet-Mobility | 2,347 | 2,266 | 81 | 200 |
| PhysX-Mobility | 2,024 | 1,941 | 83 | 200 |
| SketchMobility | 4,956 | 4,955 | 1 | 200 |
| Infinigen-Sim | 8,225 | 8,131 | 94 | 200 |
| Ours / PV-A | 302,440 | 293,385 | 9,055 | 200 |

论文中必须分别写清楚各 candidate universe 及其 hash；所有八个数据集均从上述完整冻结 universe 直接抽取 N=200，不再使用五个 baseline 的旧 N=800 parent roster。

### 2.2 统一 eligibility

对八个数据集应用完全相同的关节数规则：

```text
1 <= movable_joint_count <= 20
```

以下语义排除仅应用于 PV-A，不应用于其余七个数据集：

```text
fence / fences
sofa-bed / sofa-beds / folding-sofa
public-toilet / public-restroom / compound-restroom
```

`Bathroom_toilet` 这类单体马桶不属于 `public-toilet` 复合场景，不排除。

PV-A 匹配规则已经写入冻结 selection：将 `category + asset_id` 转为小写，将连续非字母数字字符归一为 `_`，然后匹配完整 normalized token sequence。其余七个数据集不执行这三个语义排除。

### 2.3 随机抽样

对每个合格候选计算：

```text
SHA256(protocol_id, seed, universe_sha256, dataset_slug, asset_id)
```

按 hash 升序取前 200 个，无放回。禁止：

- 根据旧 Table 5 成绩筛选；
- 根据 preflight、load 或 simulator 成功率筛选；
- 删除失败资产后补抽；
- 为 PV-A 使用不同关节数上限；
- 在查看结果后修改语义排除表。

选中清单和排除清单分别为：

```text
exp/runtime/table5_v2_core200_five_full_release_articraft10787_infinigen_paired_official/selected_assets.jsonl
exp/runtime/table5_v2_core200_five_full_release_articraft10787_infinigen_paired_official/excluded_assets.jsonl
exp/runtime/table5_v2_core200_five_full_release_articraft10787_infinigen_paired_official/cohort_manifest.json
```

Articraft 的导出严格遵循数据集仓库的官方 `full --validate` 默认行为：geometry QC 只写入 compile report warning，不作为本评测的筛选或阻断条件；不做几何修复、不删除重叠/断连部件。仅当冻结 source record 调用旧版 SDK warning helper 时，使用 [articraft_compile_compat.py](scripts/articraft_compile_compat.py) 做方法名/参数兼容，仍调用同一官方导出流程。

## 3. Canonical asset 与物理参数

### 3.1 评测输入必须先 canonicalize

每个资产冻结以下内容：

- source URDF 和 SHA256；
- link / joint 名称、类型、parent-child、axis、origin、limit；
- collision representation；
- q=0 object bounding box：有 collision 时使用全部 collision geometry；若 URDF 完全没有 collision，则使用全部 visual geometry；
- 每个 link 的 mass、COM、3×3 inertia；
- simulator adapter 的 link / DoF 映射；
- 所有派生文件及编译器源码 SHA256。

主实验采用 **released-first、target-native representation**：默认情况下，Genesis、PyBullet、MuJoCo 都加载 released 或 prepared URDF；唯一冻结例外是 Infinigen-Sim，其 Genesis/PyBullet 输入为官方 URDF，MuJoCo 输入为同一资产 ID 的官方 MJCF。该例外只使用发布方原生文件，不做格式转换或 URDF 修复。

三个仿真器仍共享同一份 canonical URDF joint/link schema、同一 200 个资产 ID、同一关节命令和成功判据。因输入格式并非逐字节相同，Table 5b 的 claim boundary 是 **officially released cross-simulator readiness**，不能写成 same-URDF importer portability。

同一个资产进入三个仿真器时，必须使用同一份 canonical joint schema 和 collision geometry；有效的 released inertial properties 必须保留。缺失或非法字段按当前仿真器的 native 机制补全，并在 physics receipt 中记录，不能覆盖有效的 released 属性。

### 3.2 PV-A 的 `physics.json` 如何使用

`physics.json` 中的 density 不是 mass。只有执行下列确定性编译后，它才真正参与刚体动力学：

\[
m_k = \rho_k V_k
\]

\[
\mathbf c = \frac{\sum_k m_k\mathbf c_k}{\sum_k m_k}
\]

\[
\mathbf I = \sum_k\left(\mathbf I_k + m_k\left(\|\mathbf d_k\|^2\mathbf 1-\mathbf d_k\mathbf d_k^\top\right)\right)
\]

其中 \(k\) 是一个 link 内的 collision solid。体积、局部 COM 和 unit-density inertia 来自 collision geometry，最后使用平行轴定理组合。

仓库已有确定性实现：

```text
exp/scripts/table5_pva_physics.py
```

它会：

- 校验 `physics.json` 与 source URDF 的 hash binding；
- 保留合法的 source `<inertial>`；
- 只为缺失或非法 inertial 的 link 从 collision volume 和 density 推导 mass、COM、inertia；
- 生成带 `<inertial>` 的 injected URDF 和逐 link `physics_plan.json`；
- 对非 watertight mesh 记录 convex-hull fallback，而不是静默处理；
- 将可在三引擎中一致表达的 dynamic friction 写入 adapter；
- 对无法在三引擎刚体 API 中等价表达的字段明确记为 unsupported。

因此，对旧结果应作如下判断：

> 如果旧 PV-A Table 5 run 直接加载 source URDF，而没有使用 injected URDF 和 adapter readback receipt，那么 `physics.json` 的 density 并没有自动变成 simulator 使用的 mass、COM、inertia。这样的 Genesis / MuJoCo 结果不能用于新的 Table 5 结论，必须重跑。

只有同时保存 injected URDF hash、physics plan hash，以及三引擎实际载入或 readback receipt，才能声称 `physics.json` 已被消费。

### 3.3 Baseline 的物理参数

主实验比较 **released-first/native-fallback simulation readiness**：

- baseline 使用其 release 中合法的 URDF inertial 或正式 physics sidecar；
- PV-A 使用 release 自带的 `physics.json`，通过上述冻结编译器转成 URDF inertial；
- 缺失或非法字段由当前 simulator 按 native 机制补齐；native fallback 只能补缺失字段，不能覆盖有效的 released 属性；
- 不能针对单个失败资产人工修改 mass、COM、inertia、friction 或 collision。

本轮 Table 5 不使用 `Common-Physics` 或 `Common-density` 作为主协议；如未来需要区分 physics metadata 与 geometry 质量，应另设独立 ablation，不能与本轮 native-track 结果混写。

## 4. 统一运行协议

### 4.1 基本设置

```text
base                  fixed
gravity               (0, 0, -9.81) m/s^2
contacts              enabled
timestep              1/240 s
control update         240 Hz
initial joint state    declared midpoint
random seed            frozen
retry                  none
timeout                terminal failure
```

solver iteration、integrator、contact parameters 无法跨引擎完全相同；必须逐引擎冻结并报告实际配置，不能声称三者 solver 相同。

### 4.2 Joint scope

DoF coverage 包含 canonical URDF 中的：

```text
revolute
continuous
prismatic
```

Tracking、limit 和 dynamic trajectory 只对满足下列条件的 bounded revolute / prismatic joint 计算：

- finite `lower < upper`；
- finite positive `effort`；
- finite positive `velocity`；
- 三引擎存在无歧义 canonical mapping。

continuous joint 计入 DoF coverage 和 stable rollout，但不进入基于有限 range 的 NRMSE。必须单独报告 eligible joint count，不能静默从分母中消失。

### 4.3 轨迹

每次只驱动一个 joint，其余 joint 固定在 midpoint。目标轨迹使用同一 normalized minimum-jerk profile：

```text
0% -> 100% range: 240 steps
hold:             120 steps
```

三个仿真器都使用同一直接 torque / force law，禁用各自默认 position motor。控制器参数、effort clipping、采样时刻必须由 protocol hash 绑定。

## 5. Table 5a：Genesis Simulation Readiness

推荐主表：

| Method | N | Import Success (%) ↑ | DoF Coverage (%) ↑ | Stable Rollout (%) ↑ | Trajectory Coverage (%) ↑ | Tracking NRMSE P95 (%) ↓ | Limit Violation P95 (%) ↓ |
|---|---:|---:|---:|---:|---:|---:|---:|
| Articraft-10K | 200 | 100.00 | 100.00 | 100.00 | 100.00 | 76.91 (463/463) | 39.26 (463/463) |
| LAM | 200 | 87.50 | 98.93 | 87.00 | 95.78 | 83.77 (363/379) | 17.09 (363/379) |
| Artiverse | 200 | 99.50 | 99.21 | 96.00 | 95.50 | 70.81 (573/600) | 28.73 (573/600) |
| PartNet-Mobility | 200 | 95.00 | 92.02 | 94.50 | 0.00 | N/E (0/396) | N/E (0/396) |
| PhysX-Mobility | 200 | 100.00 | 100.00 | 99.50 | 100.00 | 618.25 (479/479) | 779.31 (479/479) |
| SketchMobility | 200 | 99.50 | 99.02 | 99.50 | 80.14 | 1737.85 (234/292) | 336.74 (234/292) |
| Infinigen-Sim | 200 | 93.00 | 90.28 | 93.00 | 0.00 | N/E (0/813) | N/E (0/813) |
| **Ours / PV-A** | **200** | **100.00** | **100.00** | **100.00** | 99.36 | 76.94 (624/628) | 32.35 (624/628) |

主表标题使用完整、直白名称；不要在表中重新引入 `LSR/JPSR/PER/JLCR/STB`。

### 5.1 Import Success Rate

一个资产只要 frozen manifest 绑定的官方/发布源成功返回对应仿真器的原生资产加载调用，就通过该指标：

```text
Genesis:  Scene.add_entity + Scene.build
PyBullet: loadURDF
MuJoCo:   MjModel.from_xml_path
```

原生加载成功后立即写入 hash-bound import receipt。后续 canonical mapping、physics 应用/读取、首步、FK 或 rollout 失败不能反转 Import Success；这些失败只影响各自的独立指标或诊断。缺失的物理字段由当前 simulator 按 native fallback 补全。执行器不得为了导入成功而改名、补节点、删结构、替换格式或修改资产。

\[
\mathrm{Import\ Success\ Rate}
= \frac{N_{\mathrm{import\ pass}}}{200}
\]

固定 link fusion 的确定映射属于 DoF/FK 诊断，不属于 Import Success gate。Genesis 对单个、零位姿、fixed dummy root 的坐标映射兼容必须单独出具 receipt，并绑定实际执行代码 hash；它不改变原生加载结果，也不能伪造 observed link。

### 5.2 DoF Coverage

\[
\mathrm{DoF\ Coverage}
= \frac{\sum_a N^{(a)}_{\mathrm{mapped\ canonical\ DoF}}}
{\sum_a N^{(a)}_{\mathrm{declared\ canonical\ DoF}}}
\]

分母包括全部 200 个资产。Import failure 的 mapped DoF 为 0。一个 DoF 只有 canonical joint name 可无歧义解析到一个 simulator scalar coordinate 才算 covered。有限 `q/qdot`、axis、parent-child 与 link-frame correctness 分别由 Stable Rollout 和第 6.4/6.5 节的 FK error 验证，避免在 DoF Coverage 中重复计数。

### 5.3 Stable Rollout Rate

每个 Import Success 的资产都必须执行协议冻结步数的统一被动 rollout。初始位置对 finite bounded joint 取 limit midpoint，其余取 0；每一步对全部 mapped joint 施加 0 torque/force。该指标不要求存在 eligible 或 mapped DoF，但零 DoF 资产仍必须真实执行全部仿真步，不能因空 joint 列表而直接通过。

通过条件为：

- runner 无 crash、timeout 或 fatal solver error；
- reset 和全部固定步数真实完成；
- 每一步的 mapped `q`、`qdot` 和全部 observed link pose 有限；
- articulation mapping 在 rollout 前后保持一致；
- observed link 集合非空。

\[
\mathrm{Stable\ Rollout\ Rate}
= \frac{N_{\mathrm{complete,finite\ rollout}}}{200}
\]

该指标不检查 free-standing balance。固定 base 下的被动 joint drift 受 damping、friction、gravity 和初始姿态影响，不能命名为 `Static Stability Rate`。如需要自由落地稳定性，应作为按物体类别控制 placement 的独立 supplementary task。

### 5.4 Tracking NRMSE P95

对 eligible joint \(j\)：

\[
\mathrm{NRMSE}_j
= \frac{
\sqrt{\frac{1}{T}\sum_t(q_j(t)-q_j^*(t))^2}
}{q_{j,\max}-q_{j,\min}}
\]

主表在成功得到完整 tracking trace 的 joints 上计算 P95，并乘 100 表示 range percentage。同时单列 Trajectory Coverage：

\[
\mathrm{Trajectory\ Coverage}
=\frac{N_{\mathrm{declared\ joint\ with\ complete\ tracking\ trace}}}
{N_{\mathrm{declared\ revolute/prismatic\ joint}}}
\]

分母包含 200 个资产中全部 declared revolute/prismatic joints，包括 import、mapping、limit、effort 或 trace 不完整的 joint。连续值单元格固定显示 `P95 (evaluated/candidate)`；没有成功 trace 时显示 `N/E (0/candidate)`。低 coverage 不能通过只在成功 joint 上计算较小误差而获得优势，完整 success/failure breakdown 放 supplementary。

### 5.5 Limit Violation P95

对 bounded joint 的每个采样时刻：

\[
v_j(t)=
\frac{
\max(0,q_{j,\min}-q_j(t),q_j(t)-q_{j,\max})
}{q_{j,\max}-q_{j,\min}}
\]

先对每个成功得到完整 limit trace 的 bounded joint 取 \(\max_t v_j(t)\)，再在 joints 上报告 P95，乘 100 表示 range percentage。该单元格同样显示 `P95 (evaluated/candidate)`，candidate 使用全部 declared revolute/prismatic joints。该定义直接报告 violation magnitude，不再把它包装成自定义 `JLCR`。

## 6. Table 5b：Cross-Simulator Portability

推荐主表按仿真器分别列结果，不对三个仿真器做平均：

| Method | N | Genesis Import Success (%) ↑ | Genesis DoF (%) ↑ | Genesis Stable Rollout (%) ↑ | PyBullet Import Success (%) ↑ | PyBullet DoF (%) ↑ | PyBullet Stable Rollout (%) ↑ | MuJoCo Import Success (%) ↑ | MuJoCo DoF (%) ↑ | MuJoCo Stable Rollout (%) ↑ |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Articraft-10K | 200 | 100.00 | 100.00 | 100.00 | 100.00 | 100.00 | 100.00 | 100.00 | 100.00 | 100.00 |
| LAM | 200 | 87.50 | 98.93 | 87.00 | 88.00 | 99.36 | 88.00 | 44.00 | 54.60 | 42.00 |
| Artiverse | 200 | 99.50 | 99.21 | 96.00 | 100.00 | 100.00 | 100.00 | 85.00 | 85.51 | 84.50 |
| PartNet-Mobility | 200 | 95.00 | 92.02 | 94.50 | 98.50 | 99.37 | 98.50 | 42.00 | 25.82 | 42.00 |
| PhysX-Mobility | 200 | 100.00 | 100.00 | 99.50 | 100.00 | 100.00 | 99.50 | 100.00 | 100.00 | 99.50 |
| SketchMobility | 200 | 99.50 | 99.02 | 99.50 | 55.00 | 68.05 | 55.00 | 85.50 | 74.88 | 85.50 |
| Infinigen-Sim | 200 | 93.00 | 90.28 | 93.00 | 100.00 | 100.00 | 100.00 | 99.50 | 99.51 | 99.50 |
| **Ours / PV-A** | **200** | **100.00** | **100.00** | **100.00** | **100.00** | **100.00** | **100.00** | **100.00** | 99.76 | 99.50 |

运动学误差作为 Table 5b 的第二个 panel，按仿真器分别报告：

| Method | Genesis FK Position Error P95 (% diag.) ↓ | PyBullet FK Position Error P95 (% diag.) ↓ | MuJoCo FK Position Error P95 (% diag.) ↓ | Genesis FK Rotation Error P95 (deg) ↓ | PyBullet FK Rotation Error P95 (deg) ↓ | MuJoCo FK Rotation Error P95 (deg) ↓ |
|---|---:|---:|---:|---:|---:|---:|
| Articraft-10K | <1e-4 (3595/3595) | <1e-4 (3595/3595) | <1e-4 (3465/3595) | <1e-4 (3595/3595) | <1e-4 (3595/3595) | <1e-4 (3465/3595) |
| LAM | <1e-4 (3050/3065) | <1e-4 (3050/3065) | <1e-4 (1395/3065) | <1e-4 (3050/3065) | <1e-4 (3050/3065) | <1e-4 (1395/3065) |
| Artiverse | <1e-4 (5215/5300) | <1e-4 (5300/5300) | <1e-4 (2893/5300) | <1e-4 (5215/5300) | <1e-4 (5300/5300) | <1e-4 (2893/5300) |
| PartNet-Mobility | N/E (0/0) | N/E (0/0) | N/E (0/0) | N/E (0/0) | N/E (0/0) | N/E (0/0) |
| PhysX-Mobility | <1e-4 (5405/5405) | <1e-4 (5405/5405) | <1e-4 (2655/5405) | <1e-4 (5405/5405) | <1e-4 (5405/5405) | <1e-4 (2655/5405) |
| SketchMobility | <1e-4 (1415/1415) | <1e-4 (825/1415) | <1e-4 (1225/1415) | <1e-4 (1415/1415) | <1e-4 (825/1415) | <1e-4 (1225/1415) |
| Infinigen-Sim | N/E (0/0) | N/E (0/0) | N/E (0/0) | N/E (0/0) | N/E (0/0) | N/E (0/0) |
| **Ours / PV-A** | <1e-4 (5115/5115) | <1e-4 (5115/5115) | <1e-4 (4840/5115) | <1e-4 (5115/5115) | <1e-4 (5115/5115) | <1e-4 (4840/5115) |

### 6.1 Per-simulator Import Success

对每个 simulator (s\in\{G,P,M\})，使用固定分母 200：

\[
\mathrm{Import\ Success}_s
= \frac{|S_s|}{200}
\]

三个 simulator 的 import rate 分别报告，不做平均。

### 6.2 Per-simulator DoF Coverage

一个 canonical DoF 只有在当前 simulator 中存在正确映射才计入分子：

\[
\mathrm{DoF\ Coverage}_s
= \frac{\sum_a N^{(a)}_{\mathrm{mapped\ in\ s}}}
{\sum_a N^{(a)}_{\mathrm{declared\ canonical\ DoF}}}
\]

### 6.3 Per-simulator Stable Rollout

每个 simulator 独立使用第 5.3 节的 complete-and-finite rollout 判据：

\[
\mathrm{Stable\ Rollout}_s
= \frac{|R_s|}{200}
\]

三个 simulator 的共同通过交集可以作为 supplementary 统计，但不作为 Table 5b 的主列。

### 6.4 FK Position Error

运动学测试不 step dynamics：gravity off、contact response off，直接设置：

```text
0%, 25%, 50%, 75%, 100% joint range
```

每个 simulator 都与 canonical URDF forward kinematics 比较，而不是只做 simulator 两两比较。对 simulator \(s\)、link \(l\)、configuration \(k\)：

\[
e_p(s,l,k)
= \frac{\|\mathbf p_{s,l,k}-\mathbf p_{\mathrm{URDF},l,k}\|_2}
{D_{\mathrm{bbox}}}
\]

其中 (D_{\mathrm{bbox}}) 是 canonical URDF 在 q=0 下的 object AABB diagonal：优先由 collision geometry 计算；仅当 URDF 完全没有 collision 时，使用 visual geometry。该选择逐资产写入 receipt，不由仿真结果决定。主表报告全部有效 simulator-link-configuration samples 的 P95，乘 100 表示 object-diagonal percentage。Supplementary 同时给出 mm 和 coverage。

### 6.5 FK Rotation Error

\[
e_R(s,l,k)
= \cos^{-1}\left(
\operatorname{clip}\left(
\frac{\operatorname{tr}(R_{\mathrm{URDF}}^\top R_s)-1}{2},-1,1
\right)\right)
\]

主表使用 degree 并报告 P95。必须先对齐同名 canonical root frame。

## 7. 分母、聚合与统计

### 7.1 固定分母

- asset-level success rate：固定分母 200；
- DoF coverage：固定分母为 200 个资产声明的 canonical scalar DoFs；
- conditional continuous error：报告 P50、P95、有效 sample 数和 coverage；
- crash、timeout、preflight failure 和 unsupported 都保留为 terminal outcome，不补抽。

### 7.2 不要把多个 gate 重复当成多个独立优点

Import、DoF mapping、stable rollout 是递进 gate；误差指标只说明通过相应 gate 后的精度。论文不能把这些相关指标解释成互相独立的七个贡献。

### 7.3 置信区间

Asset-level rate 使用按 asset bootstrap 的 95% CI。DoF 和运动学误差指标也按 asset cluster bootstrap，不能把同一资产的数百个 joint-time samples 当成独立样本。

方法差异建议报告：

- rate difference 的 paired/unpaired bootstrap CI，取决于 cohort 是否配对；
- P95 difference 的 asset-cluster bootstrap CI；
- 不只报告粗体最优值。

当前八个数据集不是逐类别一一配对，因此不能使用 paired test 假装资产对应。

## 8. Failure taxonomy

所有失败只映射到一个 primary terminal reason，并可附 secondary diagnostics：

```text
SOURCE_BINDING_FAIL
URDF_PARSE_FAIL
RESOURCE_LOAD_FAIL
INERTIAL_COMPILE_FAIL
SIM_IMPORT_FAIL
CANONICAL_MAPPING_FAIL
FIRST_STEP_FAIL
ROLLOUT_NONFINITE
SOLVER_FATAL
TIMEOUT
FK_EVAL_FAIL
TRAJECTORY_EVAL_FAIL
```

禁止使用“physics explosion”“明显漂移”“永久 freeze”等没有数值或程序状态定义的人工标签作为主判定。

## 9. 执行顺序

```text
1. verify frozen Core-200 manifest and source hashes
2. compile canonical asset and adapter mappings
3. compile PV-A physics.json into injected URDF + physics plan
4. produce equivalent physics receipts for every dataset
5. run Genesis import and rollout for Table 5a
6. run PyBullet and MuJoCo on the same canonical artifacts
7. run non-dynamic FK comparison
8. run Genesis tracking/limit diagnostics for Table 5a; retain per-simulator rollout evidence without pairwise trajectory comparison
9. aggregate with fixed denominators and coverage
10. publish per-asset terminal records, failure breakdown and bootstrap CI
```

每个 runtime record 至少绑定：

```text
dataset_slug
asset_id
source_urdf_sha256
canonical_urdf_sha256
physics_plan_sha256
collision_manifest_sha256
cohort_manifest_sha256
protocol_sha256
adapter_source_sha256
simulator name/version/config
terminal status/reason
```

### 9.1 当前可直接执行的步骤

校验新 cohort、全部 manifest 自哈希和 source-universe hash：

```bash
python exp/scripts/table5_v2_sample_n200.py \
  --verify exp/runtime/table5_v2_core200_five_full_release_articraft10787_infinigen_paired_official/cohort_manifest.json
```

单独复核筛选与 hash-rank 逻辑：

```bash
python exp/scripts/test_table5_v2_sample_n200.py
python exp/scripts/test_table5_v2_pipeline.py
```

r2 正式 prepared manifest 已按上节绑定生成并通过 `--verify`。校验 prepared manifest：

```bash
python exp/scripts/table5_v2_prepare_r2.py \
  --verify exp/runtime/table5_v2_core200_prepared_five_full_release_articraft10787_infinigen_paired_official_metrics_r2/manifest.json
```

### 9.2 正式仿真需要的新 v2 执行器

以下旧入口不能直接用于新表：

```text
table5_n200_runtime.py
table5_n200_aggregate.py
table5_pva_physics_n200_manifest.py
table5_pva_physics_n200_runtime.py
```

原因不是它们完全不可复用，而是它们绑定旧 prefix cohort、旧 manifest schema 和旧的 load/reset/settling/actuation gate；直接运行会产生旧 Table 5 指标，不能填写本文件的 Table 5a/5b。

r2 执行器接口已经实现为：

```text
table5_v2_prepare_r2.py
  input:  frozen Core-200 manifest
  output: unchanged canonical asset preparation plus revision-2 protocol binding

table5_v2_runtime_r2.py run --simulator genesis
  output: native import receipt, DoF mapping, passive stable rollout,
          tracking trace, limit trace and per-simulator FK diagnostics

table5_v2_runtime_r2.py run --simulator pybullet
table5_v2_runtime_r2.py run --simulator mujoco
  output: the same revision-2 terminal schema

table5_v2_aggregate_r2.py
  output: Table 5a, Table 5b, coverage, failure taxonomy, bootstrap CI

run_table5_v2_native.py
  binds all three simulators to the same r2 entrypoint

run_table5_v2_formal_tmux.sh
  verifies the exact r2 prepared manifest, then runs all stages with 5 workers
```

实现复用已有 simulator adapters、`table5_pva_physics.py` 和 atomic per-asset records，但通过新文件冻结 r2 protocol，不修改旧 runtime、旧结果或资产。Genesis 的 coincident fixed-root 坐标映射兼容层已作为显式 implementation hash 绑定，不再通过兼容入口绕开原哈希校验。

对 canonical joint tree 无法编译的行（frozen preflight 已记录 `invalid_joint_tree`，全 cohort 22 行：21 LAM + 1 PhysX-Mobility），r2 worker 在原生加载成功后按 `CANONICAL_MAPPING_FAIL`（stage `canonical_mapping`）分类终止，并保留成功的 native import receipt（§5.1：映射失败不反转 Import Success）；原生加载本身失败则分类为 `simulator_asset_load_rejected`（stage `adapter_initialization`）。该分类不改变任何通过/失败判定，资产仍按固定分母判负，只把未分类崩溃改为分类学记录。

### 9.3 推荐运行顺序

```text
1. 保持现有 frozen Core-200 cohort 不变，不重新抽样。
2. 经单独批准后，为该 cohort 生成一次全量 r2 prepared manifest；当前不执行。
3. 只补 Infinigen-Sim 的 Genesis r2 smoke，完成后汇报并停止。
4. 再次获得正式批准后，在 tmux 中执行：

   bash exp/scripts/run_table5_v2_formal_tmux.sh

5. 脚本固定使用 5 workers，并依次运行 Genesis、PyBullet、MuJoCo 后聚合。
```

正式脚本当前绑定的目标是：

```text
prepared = exp/runtime/table5_v2_core200_prepared_five_full_release_articraft10787_infinigen_paired_official_metrics_r2/manifest.json
output   = exp/runtime/table5_v2_r2_formal_eight_datasets
workers  = 5
```

该 prepared 路径已存在并通过 `--verify`（绑定见 §2）。正式仿真启动前仍需检查 prepared manifest、physics receipts、协议 hash 和命令参数；不要用旧入口续跑 r2 结果。

### 9.4 正式运行前的 physics readiness 审计

旧 prepared manifest 的 `Eligible / Physics Ready / Object BBox` 统计属于旧 N=800 parent-roster cohort，不得复制到新表。新 full-release Core-200 的静态审计必须在 r2 prepared manifest 合法生成后重新汇总；当前不执行 prepare，因此暂不填写这些数字。

`Physics Ready` 只表示 release 静态物理字段完整性，不是 Table 5 的运行 gate。有效字段必须由 importer 消费；缺失字段由各 simulator 的 native fallback 处理，不能仅因 inertial 不完整就提前记 Import failure 或 Stable failure。正式结果应保留每个 simulator 的 fallback/unsupported 原始 provenance，但不将其作为 Table 5 主指标。

## 10. 主结果与 SOTA 叙事

这套设计可以体现 PV-A 的优势，但优势必须来自固定协议下的结果，而不是为 PV-A 改定义：

- `physics.json` 的完整 binding 可以提高 inertial compilation coverage；
- canonical URDF 与 sidecar hash binding 可以提高可追溯性；
- 同一 injected physics 进入三个 simulator，可以减少 importer default 导致的跨引擎差异；
- 大规模、多关节资产的优势通过统一 `<=20` scope 保留，同时去掉极端复合场景对 runtime budget 的支配。

允许的论文结论：

> PV-A achieves the best simulation readiness and cross-simulator portability under a frozen, outcome-independent Core-200 protocol.

前提是 Table 5a/5b 的正式结果和置信区间确实支持该结论。

不允许的做法：

- 在看到结果后移动 threshold；
- 把旧 PV-A prefix run 与新 baseline cohort 混用；
- 只为 PV-A 注入 physics、却让 baseline 使用未记录的 simulator defaults；
- 把 `physics.json` 存在本身当作 physics 已被 simulator 使用的证据；
- 将 `Stable Rollout Rate` 宣称为行业统一标准；
- 隐藏 continuous-error coverage 或删除失败资产。

## 11. 最终检查清单

- [x] 八个数据集各冻结 200 个资产；
- [x] 统一使用 `1 <= movable joints <= 20`；
- [x] 仅 PV-A 排除 fence、sofa-bed、public-toilet 语义族；其余七个数据集不做该语义排除；
- [x] 选择不依赖历史仿真结果；
- [x] manifest、selected ledger、excluded ledger 和 source hash 已生成；
- [x] 为新 PV-A 200 生成 physics-injected canonical URDF；
- [x] 为八个数据集冻结 canonical link/DoF schema；adapter mapping receipt 将由各 simulator runtime 生成；
- [x] r2 runtime、aggregate 与正式 tmux 入口已显式绑定；
- [x] Infinigen-Sim Genesis r2 smoke 已完成并汇报；
- [x] 三仿真器 × 8 数据集 r2 smoke（`table5_v2_r2_pre_formal_three_sim_smoke_v2`）已通过：0 worker_error，canonical-mapping/load 失败均按分类学记录，FK/tracking/limit 数值路径全部产出非平凡样本；
- [x] 生成并冻结 r2 prepared manifest/protocol hash；
- [x] 完成 Genesis / PyBullet / MuJoCo 正式重跑（2026-08-29，4,800 任务全部完成，`table5_v2_r2_formal_eight_datasets`）；
- [x] 生成 coverage、failure breakdown 和 asset-cluster bootstrap CI（`final/summary.json`）；
- [x] Table 5 中只填写新 cohort 的正式结果（本文件 §5/§6 已回填）。
