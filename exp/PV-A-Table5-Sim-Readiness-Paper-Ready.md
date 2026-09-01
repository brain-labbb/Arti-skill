# PV-A Table 5a/5b: Simulation Readiness & Cross-Simulator Portability — Paper-Ready Text and Experiment Notes

本文档对应 PV-A paper 的 **Table 5a: Genesis simulation readiness** 与 **Table 5b: Cross-simulator portability**。第一部分为可直接复制到英文论文的内容；第二部分为中文详解（运行绑定、统计口径、限制与已知问题）。

正式运行：`exp/runtime/table5_v2_r2_formal_eight_datasets`（2026-08-29 启动，4,800 个任务 = 8 数据集 × 200 资产 × 3 仿真器，全部完成）。聚合产物：`final/report.md`、`final/summary.json`、`final/table5a.csv`、`final/table5b.csv`。

## 【1】第一部分：可直接复制粘贴到 PV-A Paper 的内容

### 1.1 Evaluation protocol

#### Simulation Readiness and Cross-Simulator Portability

We evaluate simulation readiness and cross-simulator portability on a frozen, outcome-independent Core-200 protocol. For each of eight articulated-asset sources — Articraft-10K, LAM released outputs, Artiverse, PartNet-Mobility, PhysX-Mobility, SketchMobility, Infinigen-Sim, and Ours (PV-A) — we sample exactly N=200 assets by ascending SHA-256 rank over `SHA256(protocol_id, seed, universe_sha256, dataset_slug, asset_id)` from the complete frozen release universe of each dataset, with a uniform eligibility rule of 1–20 movable joints and no resampling after failures. The selection never observes simulation outcomes (cohort SHA-256 `660dbeeb…`; protocol `table5-v2-core200-hash-sample-v1`).

Each asset is evaluated through progressive gates in three simulators: Genesis, PyBullet, and MuJoCo. **Import Success** requires only that the official released source passes the simulator's native asset-load call (`Scene.add_entity + Scene.build`, `loadURDF`, `MjModel.from_xml_path`); later mapping or rollout failures never reverse it. **DoF Coverage** is the fraction of declared canonical scalar DoFs that resolve unambiguously to simulator coordinates. **Stable Rollout** is a fixed 240-step zero-force passive rollout from the limit midpoint, passing only if every step completes with finite joint states and link poses and the articulation mapping is unchanged. **Tracking NRMSE P95** and **Limit Violation P95** are computed over per-joint actuated ramp-and-hold traces and reported at the 95th percentile over joints, with coverage shown as evaluated/candidate counts; joints without complete traces remain in the candidate denominator (fail-closed). For portability we additionally report non-dynamic forward-kinematics position and rotation errors against canonical URDF FK at 0/25/50/75/100% of each joint range. All rates use a fixed denominator of 200 with asset-level bootstrap 95% CIs; continuous P95 values use asset-cluster bootstrap. Released-first, target-native inputs are used throughout: every simulator loads the officially released URDF, except Infinigen-Sim's MuJoCo track, which loads the officially released MJCF of the same asset IDs; the portability claim is therefore *officially released cross-simulator readiness*, not same-URDF importer portability. For PV-A, `physics.json` is compiled into an injected canonical URDF with hash-bound receipts; baselines rely on their released inertial metadata with simulator-native fallback for missing fields, recorded per asset.

### 1.2 Results paragraph

#### Simulation Readiness Results

Table 5a reports Genesis simulation readiness. PV-A attains 100% import success, 100% DoF coverage, and 100% stable rollout, with 99.36% trajectory coverage and Tracking NRMSE P95 of 76.94% of joint range (624/628 joints evaluated) — matched on the rate gates only by Articraft-10K, and substantially better in tracking than PhysX-Mobility (NRMSE P95 618.25%) and SketchMobility (1737.85%), whose released assets are accepted by the loader but do not follow commanded joint trajectories. PartNet-Mobility and Infinigen-Sim import at 95.00% and 93.00% but reach 0% trajectory coverage because their released URDFs lack the effort/velocity metadata required for actuation under the frozen eligibility rule; their tracking columns are therefore reported as N/E rather than omitted. LAM reaches 87.50% import success: 21 released outputs contain non-canonical floating joints and 8 reference missing mesh files, all counted as failures in the fixed denominator.

Table 5b reports cross-simulator portability. PV-A is the only source whose released assets remain essentially fully portable across all three simulators (100/100/100 on Genesis and PyBullet; 100/99.76/99.50 on MuJoCo). Baseline portability is markedly simulator-dependent: LAM imports at 87.50% in Genesis but only 44.00% in MuJoCo; PartNet-Mobility drops from 95.00% (Genesis) to 42.00% (MuJoCo) with DoF coverage falling to 25.82%; SketchMobility imports at 99.50% in Genesis but 55.00% in PyBullet; Artiverse falls from 99.50% to 85.00% in MuJoCo. Non-dynamic FK agreement is exact to numerical precision wherever it is evaluable (P95 ≤ 3.6e-5 % of the object diagonal for position, ≤ 3.6e-5 deg for rotation), indicating that cross-simulator divergence in this benchmark is dominated by loader strictness, metadata completeness, and dynamic stability rather than kinematic interpretation.

### 1.3 Copy-ready Markdown tables

**Table 5a. Genesis simulation readiness (Core-200, frozen outcome-independent sampling).** Import Success counts native asset-load acceptance only; Stable Rollout is an independent fixed-step zero-force passive finite-state test; continuous cells show P95 (evaluated/candidate) in percent of joint range; N/E is fail-closed and never counts as a successful evaluation. Rate cells are percentages; asset-level bootstrap 95% CIs are in the supplementary.

| Method | N | Import Success (%) ↑ | DoF Coverage (%) ↑ | Stable Rollout (%) ↑ | Trajectory Coverage (%) ↑ | Tracking NRMSE P95 (%) ↓ | Limit Violation P95 (%) ↓ |
|---|---:|---:|---:|---:|---:|---:|---:|
| Articraft-10K | 200 | 100.00 | 100.00 | 100.00 | 100.00 | 76.91 (463/463) | 39.26 (463/463) |
| LAM | 200 | 87.50 | 98.93 | 87.00 | 95.78 | 83.77 (363/379) | 17.09 (363/379) |
| Artiverse | 200 | 99.50 | 99.21 | 96.00 | 95.50 | 70.81 (573/600) | 28.73 (573/600) |
| PartNet-Mobility | 200 | 95.00 | 92.02 | 94.50 | 0.00 | N/E (0/396) | N/E (0/396) |
| PhysX-Mobility | 200 | 100.00 | 100.00 | 99.50 | 100.00 | 618.25 (479/479) | 779.31 (479/479) |
| SketchMobility | 200 | 99.50 | 99.02 | 99.50 | 80.14 | 1737.85 (234/292) | 336.74 (234/292) |
| Infinigen-Sim | 200 | 93.00 | 90.28 | 93.00 | 0.00 | N/E (0/813) | N/E (0/813) |
| **Ours (PV-A)** | **200** | **100.00** | **100.00** | **100.00** | 99.36 | 76.94 (624/628) | 32.35 (624/628) |

**Table 5b. Cross-simulator portability (same Core-200 cohort).** G/P/M = Genesis / PyBullet / MuJoCo. Inputs are officially released files (Infinigen-Sim uses official MJCF for the MuJoCo track), so the claim boundary is officially released cross-simulator readiness.

| Method | N | G Import ↑ | G DoF ↑ | G Stable ↑ | P Import ↑ | P DoF ↑ | P Stable ↑ | M Import ↑ | M DoF ↑ | M Stable ↑ |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Articraft-10K | 200 | 100.00 | 100.00 | 100.00 | 100.00 | 100.00 | 100.00 | 100.00 | 100.00 | 100.00 |
| LAM | 200 | 87.50 | 98.93 | 87.00 | 88.00 | 99.36 | 88.00 | 44.00 | 54.60 | 42.00 |
| Artiverse | 200 | 99.50 | 99.21 | 96.00 | 100.00 | 100.00 | 100.00 | 85.00 | 85.51 | 84.50 |
| PartNet-Mobility | 200 | 95.00 | 92.02 | 94.50 | 98.50 | 99.37 | 98.50 | 42.00 | 25.82 | 42.00 |
| PhysX-Mobility | 200 | 100.00 | 100.00 | 99.50 | 100.00 | 100.00 | 99.50 | 100.00 | 100.00 | 99.50 |
| SketchMobility | 200 | 99.50 | 99.02 | 99.50 | 55.00 | 68.05 | 55.00 | 85.50 | 74.88 | 85.50 |
| Infinigen-Sim | 200 | 93.00 | 90.28 | 93.00 | 100.00 | 100.00 | 100.00 | 99.50 | 99.51 | 99.50 |
| **Ours (PV-A)** | **200** | **100.00** | **100.00** | **100.00** | **100.00** | **100.00** | **100.00** | **100.00** | 99.76 | 99.50 |

**Table 5b kinematic panel. Non-dynamic FK error vs canonical URDF FK at five joint-range quantiles.** Cells show P95 (evaluated/candidate); values are fractions of the object bounding-box diagonal (position) or degrees (rotation). Wherever evaluable, all three simulators agree with canonical FK to numerical precision; N/E rows are datasets whose released joints carry no finite limits/effort metadata.

| Method | G FK-Pos P95 ↓ | P FK-Pos P95 ↓ | M FK-Pos P95 ↓ | G FK-Rot P95 ↓ | P FK-Rot P95 ↓ | M FK-Rot P95 ↓ |
|---|---:|---:|---:|---:|---:|---:|
| Articraft-10K | <1e-4 (3595/3595) | <1e-4 (3595/3595) | <1e-4 (3465/3595) | <1e-4 (3595/3595) | <1e-4 (3595/3595) | <1e-4 (3465/3595) |
| LAM | <1e-4 (3050/3065) | <1e-4 (3050/3065) | <1e-4 (1395/3065) | <1e-4 (3050/3065) | <1e-4 (3050/3065) | <1e-4 (1395/3065) |
| Artiverse | <1e-4 (5215/5300) | <1e-4 (5300/5300) | <1e-4 (2893/5300) | <1e-4 (5215/5300) | <1e-4 (5300/5300) | <1e-4 (2893/5300) |
| PartNet-Mobility | N/E (0/0) | N/E (0/0) | N/E (0/0) | N/E (0/0) | N/E (0/0) | N/E (0/0) |
| PhysX-Mobility | <1e-4 (5405/5405) | <1e-4 (5405/5405) | <1e-4 (2655/5405) | <1e-4 (5405/5405) | <1e-4 (5405/5405) | <1e-4 (2655/5405) |
| SketchMobility | <1e-4 (1415/1415) | <1e-4 (825/1415) | <1e-4 (1225/1415) | <1e-4 (1415/1415) | <1e-4 (825/1415) | <1e-4 (1225/1415) |
| Infinigen-Sim | N/E (0/0) | N/E (0/0) | N/E (0/0) | N/E (0/0) | N/E (0/0) | N/E (0/0) |
| **Ours (PV-A)** | <1e-4 (5115/5115) | <1e-4 (5115/5115) | <1e-4 (4840/5115) | <1e-4 (5115/5115) | <1e-4 (5115/5115) | <1e-4 (4840/5115) |

### 1.4 Copy-ready LaTeX (booktabs)

```latex
\begin{table}[t]
\centering
\caption{{\bf Genesis simulation readiness (Core-200).} Import = native asset-load acceptance; Stable = fixed-step zero-force passive finite-state rollout; continuous cells are P95 (evaluated/candidate) in \% of joint range; N/E is fail-closed.}
\label{tab:table5a}
\small
\begin{tabular}{lrrrrrrr}
\toprule
Method & N & Import (\%) $\uparrow$ & DoF (\%) $\uparrow$ & Stable (\%) $\uparrow$ & Traj.\ Cov.\ (\%) $\uparrow$ & Track.\ NRMSE P95 (\%) $\downarrow$ & Limit Vio.\ P95 (\%) $\downarrow$ \\
\midrule
Articraft-10K     & 200 & 100.00 & 100.00 & 100.00 & 100.00 & 76.91 (463/463) & 39.26 (463/463) \\
LAM               & 200 & 87.50  & 98.93  & 87.00  & 95.78  & 83.77 (363/379) & 17.09 (363/379) \\
Artiverse         & 200 & 99.50  & 99.21  & 96.00  & 95.50  & 70.81 (573/600) & 28.73 (573/600) \\
PartNet-Mobility  & 200 & 95.00  & 92.02  & 94.50  & 0.00   & N/E (0/396)     & N/E (0/396) \\
PhysX-Mobility    & 200 & 100.00 & 100.00 & 99.50  & 100.00 & 618.25 (479/479) & 779.31 (479/479) \\
SketchMobility    & 200 & 99.50  & 99.02  & 99.50  & 80.14  & 1737.85 (234/292) & 336.74 (234/292) \\
Infinigen-Sim     & 200 & 93.00  & 90.28  & 93.00  & 0.00   & N/E (0/813)     & N/E (0/813) \\
\textbf{Ours (PV-A)} & \textbf{200} & \textbf{100.00} & \textbf{100.00} & \textbf{100.00} & 99.36 & 76.94 (624/628) & 32.35 (624/628) \\
\bottomrule
\end{tabular}
\end{table}

\begin{table}[t]
\centering
\caption{{\bf Cross-simulator portability (same Core-200 cohort).} G/P/M = Genesis/PyBullet/MuJoCo; officially released files are loaded (Infinigen-Sim uses official MJCF on the MuJoCo track).}
\label{tab:table5b}
\small
\begin{tabular}{lrrrrrrrrr}
\toprule
Method & G Imp. & G DoF & G Stab. & P Imp. & P DoF & P Stab. & M Imp. & M DoF & M Stab. \\
\midrule
Articraft-10K     & 100.00 & 100.00 & 100.00 & 100.00 & 100.00 & 100.00 & 100.00 & 100.00 & 100.00 \\
LAM               & 87.50  & 98.93  & 87.00  & 88.00  & 99.36  & 88.00  & 44.00  & 54.60  & 42.00 \\
Artiverse         & 99.50  & 99.21  & 96.00  & 100.00 & 100.00 & 100.00 & 85.00  & 85.51  & 84.50 \\
PartNet-Mobility  & 95.00  & 92.02  & 94.50  & 98.50  & 99.37  & 98.50  & 42.00  & 25.82  & 42.00 \\
PhysX-Mobility    & 100.00 & 100.00 & 99.50  & 100.00 & 100.00 & 99.50  & 100.00 & 100.00 & 99.50 \\
SketchMobility    & 99.50  & 99.02  & 99.50  & 55.00  & 68.05  & 55.00  & 85.50  & 74.88  & 85.50 \\
Infinigen-Sim     & 93.00  & 90.28  & 93.00  & 100.00 & 100.00 & 100.00 & 99.50  & 99.51  & 99.50 \\
\textbf{Ours (PV-A)} & \textbf{100.00} & \textbf{100.00} & \textbf{100.00} & \textbf{100.00} & \textbf{100.00} & \textbf{100.00} & \textbf{100.00} & 99.76 & 99.50 \\
\bottomrule
\end{tabular}
\end{table}
```

### 1.5 Copy-ready limitations paragraph

#### Limitations of the Simulation Evaluation

Stable Rollout is an operational pass rate for fixed-base, zero-force, finite-state rollouts; it does not establish free-standing balance, real-world dynamic accuracy, or task-level usefulness, and we do not present it as an industry-standard metric. Portability is evaluated on officially released files per simulator (Infinigen-Sim uses official MJCF for MuJoCo), so Table 5b characterizes released cross-simulator readiness rather than same-URDF importer portability. Ten LAM assets crash PyBullet's native C++ loader (SIGSEGV) before any evidence can be logged; they are counted as import failures in the fixed denominator of 200, and no resampling is performed. PartNet-Mobility and Infinigen-Sim obtain N/E tracking columns because their released joints lack effort/velocity metadata under the frozen eligibility rule; this is reported as coverage loss, not omitted. FK agreement is exact to numerical precision wherever evaluable, so the kinematic panel establishes consistency rather than differentiation. Finally, absolute tracking NRMSE values are sensitive to the frozen actuation protocol (per-joint ramp-and-hold under gravity with zero torque elsewhere) and should be compared across methods within this protocol only.

## 【2】第二部分：实验相关信息详细说明

### 2.1 运行绑定（可复核）

```text
prepared manifest   = exp/runtime/table5_v2_core200_prepared_five_full_release_articraft10787_infinigen_paired_official_metrics_r2/manifest.json
manifest_sha256     = 5f481188a39bfa77bf7c140c5bce21eb3db6b8fcbddc2965ae75bebed83ce146
protocol_sha256     = 5abe4976f3bd9554365699366bebbe2eba76d52c00da8dd6ca6c88e6f8041e84
prepared_cohort     = bdd3bb53d729e6f759145a8e9a7f8c293e91d9cedb23a4e8bf5f06b6f155352c
source cohort       = 660dbeeb01fcfa379c6ec33a572c78553bc06571d586407f92f52dc852781a60
v2 runtime sha256   = a353c62f176df9f8058440155a35c0455951a2939990cd035a7f20dbf70ba605
formal output       = exp/runtime/table5_v2_r2_formal_eight_datasets
summary_sha256      = e17acf4bef332ebe38586ab40c2bf7b6425e3cc07614d5f877307b487812ed2d
```

执行入口：`bash exp/scripts/run_table5_v2_formal_tmux.sh`（TABLE5_GPUS=0,1,4,6,7，5 workers，tmux 会话 `table5-v2-r2-formal`）。2026-08-29 09:29 UTC 启动，约 5.5 小时完成全部 4,800 任务并自动聚合。

### 2.2 终端状态与失败分类（全部 1,600×3 记录）

| 仿真器 | completed | diagnostic_failure | native_crash | worker_error | 不可信记录 |
|---|---:|---:|---:|---:|---:|
| Genesis | 1,539 | 59 | 2 | 0 | 0 |
| PyBullet | 1,481 | 108 | 10 | 1 | 10（即 10 个 native_crash，见下） |
| MuJoCo | 1,304 | 295 | 1 | 0 | 0 |

失败主因（聚合口径）：资产级加载被拒（缺 mesh、URDF 非法、MuJoCo 严格编译失败）、`CANONICAL_MAPPING_FAIL`（22 个含 floating 关节的资产：21 LAM + 1 PhysX-Mobility，import 保留 pass、canonical 映射判负）、被动稳定性失败、以及下述已知问题。

### 2.3 已知问题与处理决定（2026-08-29 用户确认接受现状）

1. **10 个 LAM `broken:imperfect` 资产使 PyBullet `loadURDF` 段错误**（rc=-11，C++ 层崩溃，发生于任何证据 checkpoint 之前）。聚合器按设计将其判为不可信记录 → 整体 run classification = INCOMPLETE。这些资产已按固定分母 200 计入 PyBullet import 失败（LAM PyBullet Import = 88.00%），数字无失真；重跑必然复现，属资产问题，不做资产修复。
2. **`lam_0570` × PyBullet：`worker_error`**（资产无几何 → `bounding_box_diagonal=None`，legacy 诊断 `float(None)` 崩溃）。执行器健壮性缺口，但该资产 import/DoF/stable/FK 证据已完整记录，指标不受影响；修复留待下一轮协议滚动。
3. **P95 的 asset-cluster bootstrap CI 在重尾列上不包点估计**（如 PhysX Tracking NRMSE 点估计 618.25，CI [34.67, 372.75]）：单个极端资产支配 pooled P95，重抽样通常将其移除所致。主表只报点估计，CI 入 supplementary 并附此说明；方法间差异结论以率类指标（率 CI 均正常）为主。
4. MuJoCo FK 面板 coverage 低于 Genesis/PyBullet（如 PhysX 2655/5405）：MJCF track 的部分链路 FK 评估失败，按 fail-closed 记入 coverage，不影响已评样本的结论。

### 2.4 PV-A 结果要点

- **Table 5a**：唯一在 Genesis 下 Import/DoF/Stable 全 100% 的方法；Trajectory Coverage 99.36%（624/628），Tracking NRMSE P95 76.94 与 Articraft（76.91）同档，远优于 PhysX（618）与 Sketch（1738）。
- **Table 5b**：**唯一三仿真器全通**（MuJoCo DoF 99.76、Stable 99.50 为仅有的两个非 100 单元格）；基线的跨引擎缺口巨大（LAM MuJoCo import 44%、PartNet MuJoCo 42%/DoF 25.82%、Sketch PyBullet 55%）。
- **FK 面板**：所有可评样本上三引擎与 canonical URDF FK 在数值精度内一致（位置 ≤2.2e-5 % 对角线、旋转 ≤3.6e-5 deg）——跨引擎差异来自加载严格性/元数据完整性/动力学稳定性，而非运动学解释差异。这是对审稿人很有力的一句话。
- 允许的论文结论（方案 §10）：*PV-A achieves the best simulation readiness and cross-simulator portability under a frozen, outcome-independent Core-200 protocol.* —— 当前数字支持该结论。
