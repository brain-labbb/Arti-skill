# Source Map — Electrical_Wiring / Surge protector switch

一 slug（`surge_protector_switch`）：核心脊柱 = 一根挤出 bar 壳 + N 个 loop 排布的 outlet/bezel + N 个
独立 red-rocker revolute 子件 + cord/plug。outlet 数 N 与开关方案是主轴。

## Origins（全量对账，2/2 上格）

| id | pic | 建成形态 | 网格角色 | 承载 fork |
|---|---|---|---|---|
| S1 `rec_electrical_wiring_gpt55_surge_protector_switch_001_redo_recessed_sixrockers` | 001 | 黑金属条排插，N=5 outlet + 6 rocker（1 master 在线端 + 5 独立），编织线+三脚插头，角螺丝，无支架 | N≈6 / switch=master+individual / mount=flat / face=cord_plug | reset_breaker_button |
| S2 `rec_use-the-attached-reference-image-as-the-primary-_20260626_102016_767543_d5decd1b` | 002 | 黄金属条排插，N=8 outlet + 8 每孔 rocker，keyhole 壁挂支架+护杠+端盖，绿保护 LED，spline 弯线+圆插头 | N=8 / switch=per_outlet / mount=wall_bracket / face=cord+LED | four_outlet · twelve_outlet · single_master_switch · usb_ports |

## Slots

- **A outlet_count N（multiplicity，主轴）**：4(fork@S2) / 6(S1) / 8(S2) / 12(fork@S2)——模板可外推
- **B switch_scheme**：per_outlet(S2) / master+individual(S1) / single_master(fork@S2) —— 可外推 master_breaker+individual
- **C mount**：flat_feet(S1) / wall_keyhole+guard(S2)——双源，模板可外推 clamp/DIN
- **D face_module & power_entry**：cord+plug(S1) / cord+plug+LED(S2) / +reset_breaker_button(fork@S1) / +USB_ports(fork@S2)
  —— 可外推 IEC C14 inlet

## Multiplicity / Copy Logic

- outlet_xs list N（S1=5+master，S2=8），线性 pitch；outlet/bezel/rocker 三 loop 共用同一 x list
- **rocker pivot 轴两源不同**（S1=X，S2=Y）——每变体选一约定并使 pivot_pin rpy 一致
- master switch 存在时保持"N outlets vs N+1 switches"显式（S1 把 master x 前置到 list）
- S1 编织线 = 64 个小 box greeble（fork 应弃用，改 S2 的 spline tube 缆）
- 壳/outlet_plate/rocker/bezel/mount_tab 均 CadQuery（保真）

## Forks（5，全部 EXIT=0 + compile success + ≥1 非fixed joint（four=4rev, twelve=12rev, single_master=1rev）+ workbench-only + run_tests 断言主轴 + 绑定已核对）

four_outlet@S2(N=4) · twelve_outlet@S2(N=12) · single_master_switch@S2(方案) ·
reset_breaker_button@S1(面板) · usb_ports@S2(面板)

## 排除项

- 无 origin 排除（2/2 上格）
- 颜色（黑/黄）= ⑥ palette，不 fork
- 编织 cord 纹理 = ④ 装饰，不单 fork
