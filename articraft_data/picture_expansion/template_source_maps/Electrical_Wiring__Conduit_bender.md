# Source Map — Electrical_Wiring / Conduit bender

一 slug（`conduit_bender`）：核心脊柱 = 弯管 shoe（曲面 annular sector 成型槽）+ 绕 shoe 中心的
revolute 弯管关节 + 杠杆。两 origin 属**不同 ③ 主体形态家族**（落地三脚架 vs 手持杠杆），是本类主轴。

## Origins（全量对账，2/2 上格）

| id | pic | 建成形态 | 网格角色 | 承载 fork |
|---|---|---|---|---|
| S1 `rec_use-the-attached-reference-image-as-the-primary-_20260625_164631_228873_1dc8f80a` | 001 | 黄管三脚架落地弯管机，焊接 yoke，黑铸弯头 + 曲面 shoe+channel+rims，单长杆，degree 刻度，脚踏 | ③floor_tripod / base=tripod / lever=single_long / shoe=large_grooved | hand_emt_bender · hydraulic_ram_bender |
| S2 `rec_use-the-attached-reference-image-as-the-primary-_20260625_164631_228545_39453ad4` | 002 | 蓝铸手持杠杆弯管机，铸入白 degree 刻度，脚踏+锯齿，单摆臂，压辊，弯铜管 | ③handheld_lever / base=foot_pedal / lever=single_swing / shoe=small | two_handle_scissor · bench_clamp_mount · ratchet_bender |

## Slots

- **A ③ form_family（主轴）**：floor_tripod(S1) / handheld_lever(S2) / hand_emt_bender(fork@S1，铸头+长手柄+脚蹬) /
  hydraulic_ram(fork@S1，液压油缸+段模)——真实四大形态
- **B base_mount**：tripod_stand(S1) / foot_pedal(S2) / bench_clamp(fork@S2) —— 模板可外推 freestanding
- **C ② lever_config**：single_long_lever(S1) / single_swing_arm(S2) / two_handle_scissor(fork@S2，双臂皆动) /
  ratchet_pump(fork@S2)
- **D shoe_radius**：large_rigid(S1) / small_emt(S2)——⑤ 尺寸，模板可外推 mid（不单 fork）

## Multiplicity / Copy Logic

- degree_tick loop（S1 ×15，S2 ×11，_add_radial_bar 对称规则）；cast_rib / tread_tooth / grip_rib 均 loop
- 三脚架腿/撑（S1 ×8 individual _add_tube）本为非对称 2 后 1 前，允许逐个但 fork 时可 list 化
- shoe/channel/hook/conduit = 曲面 Mesh annular sector + spline tube（不许降级 Box）
- 注：S1 的 rubber_foot_pad ×6 共用一名（须加后缀）；S2 无 meta（不影响 shard 绑定）

## 交叉矩阵（form × lever；接口=弯管 revolute，family 换 = part tree 换）

| form × lever | single_lever | two_handle | ratchet | ram |
|---|---|---|---|---|
| floor_tripod | 源S1 | 外推 | gate(三脚架少配 ratchet) | fork hydraulic_ram_bender@S1 |
| handheld_lever | 源S2 | fork two_handle_scissor@S2 | fork ratchet_bender@S2 | gate |
| hand_emt | fork hand_emt_bender@S1 | gate(手弯管单柄) | 外推 | gate |

## Forks（5，全部 EXIT=0 + compile success + ≥1 非fixed joint（scissor=2rev, hydraulic=prismatic+rev, ratchet=2rev）+ workbench-only + run_tests 断言主轴 + 绑定已核对）

hand_emt_bender@S1(③) · two_handle_scissor@S2(②双臂) · bench_clamp_mount@S2 ·
hydraulic_ram_bender@S1(③液压) · ratchet_bender@S2(②)

## 排除项

- 无 origin 排除（2/2 上格）
- degree 刻度样式、颜色 = ④/⑥，palette/装饰采样，不单 fork
