# Source Map — Technology / Conference_Phone

一 slug（`conference_phone`）：5 origin 共享同一骨架——静置桌面机身 + 喇叭格栅 + 静态 LCD +
loop 发射的 prismatic 按键（全部非固定关节均为按键）。运动 spine / root 框架 / slot graph 一致，无拆分。

## Origins（全量对账，5/5 上格）

| id | pic | 建成形态 | 网格角色 | 承载 fork |
|---|---|---|---|---|
| O1 `rec_gray-polycom-conference-room-speakerphone-a-tri-_20260605_174034_151108_9bc5796c` | 001 | tri_star，central 平格栅，flush deck | ③tri_star 锚 / B central / D flush | round_puck |
| O2 `rec_a-black-tri-star-conference-speakerphone-three-r_20260624_122658_139174_0672d2af` | 002 | tri_star，三织物翼，flush deck | B discrete_3 / C fabric 锚 | tilted_console_tri |
| O3 `rec_a-black-tri-star-conference-speakerphone-three-s_20260624_122658_137394_fe888f0f` | 003 | tri_star，三穿孔格栅，4x4 键盘 | B discrete_3 兄弟 / C perforated / 键盘 4x4 锚 | square_body, raised_console_tri |
| O4 `rec_a-black-conference-phone-with-a-winged-trapezoid_20260624_122658_133437_e395d9a0` | 004 | winged 梯形，双侧荚，raised 银台 | ③winged 锚 / B discrete_2 / D raised | quad_pods, flush_console_winged |
| O5 `rec_a-black-tri-lobed-conference-speakerphone-modele_20260704_082400_536320_ecc509b0` | 005 | **hex 六边**（⚠名实注记：id/prompt 写 "tri-lobed" 系滞后文本，以建成资产为准），穹顶 central，tilted 台 | ③hex 锚 / B central(domed) / D tilted | perimeter_ring, flush_console_hex |

## Slots

- **A ③ body_form（5 锚，行发现产物）**：tri_star(O1-3) / winged(O4) / hex(O5) / round_puck(fork) / square_rounded(fork)
- **B speaker_arrangement ≡ 喇叭 multiplicity N**：central N=1(O1,O5) / discrete_2(O4) / discrete_3(O2,O3) / discrete_4(fork quad_pods) / perimeter_ring(fork)
- **C grille_treatment（④ 表面，外推轴）**：perforated(O3) / fabric(O2) / domed(O5)——跨族移植走 world_knowledge 外推
- **D control_surface**：flush(O1-3 + fork@winged + fork@hex) / raised(O4 + fork@tri) / tilted(O5 + fork@tri)

## 交叉矩阵（四态）

| B×族 | tri_star | winged | hex | round | square |
|---|---|---|---|---|---|
| central | 源O1 | fork(center 已由 flush_console_winged 变体的 KEEP 保留双荚——注：central@winged 无锚，**外推**) | 源O5 | fork 携带(round_puck KEEP central) | 外推 |
| discrete_2 | gate(三臂放双荚不真实) | 源O4 | 外推 | 外推 | 外推 |
| discrete_3 | 源O2,O3 | gate(无臂) | 外推 | 外推 | fork 携带(square_body KEEP 3格栅) |
| discrete_4 | gate | fork quad_pods | 外推 | 外推 | 外推 |
| ring | gate(叶形轮廓断带) | gate(翼荚断带) | fork perimeter_ring | 外推 | 外推 |

| D×族 | tri_star | winged | hex | round | square |
|---|---|---|---|---|---|
| flush | 源 | fork | fork | fork 携带 | 外推 |
| raised | fork | 源 | 外推 | 外推 | 外推 |
| tilted | fork | 外推 | 源 | 外推 | 外推 |

（"外推"格 = 已验证接口上的组合〔平顶面贴装〕，显式留给模板 world_knowledge + sweep + 目检。）

## Multiplicity / Copy Logic

- 喇叭数 N ∈ {1,2,3,4}（≡ slot B，绑定不独立采样；ring 为连续带无 N）
- 键盘阵列 {3x4(O1,O2), 4x4(O3)}——按键 for-loop 发射，模板可参数化扩行
- 均为 loop 发射 ✓（§4 可读性契约全 origin 确认）

## Forks（8，全部 EXIT=0 + compile success + 轴断言在 run_tests）

round_puck@O1(⑥浅灰伴随) · square_body@O3(⑥白+银伴随) · quad_pods@O4 · perimeter_ring@O5(⑥双色调伴随) ·
flush_console_winged@O4(⑥全黑伴随) · flush_console_hex@O5 · raised_console_tri@O3 · tilted_console_tri@O2

## 排除项 / gates

- 无 origin 排除（5/5 上格）
- gate（抄 spec 兼容矩阵）：臂贴合格栅仅限 tri_star；discrete_2/4 与 ring 不上 tri_star；discrete_3/4 与 ring 不上 winged

> 2026-07-04 目检回炉：wall_arm_pan / tilted_console_tri / perimeter_ring / round_puck 四变体按图诊断重铸（落座/贴合/分区打靶约束），二轮均 compile success。
