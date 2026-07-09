# Container / Locker — template source map

pattern: mixed（固定 named slots:closure_mechanism(主机构) + door_surface + latch_mechanism；**外加** bank_count 一根多重性轴——同一行内并排 locker bay × N）

parents（1 个母资产）:
- P1 rec_a-bank-of-two-metal-storage-lockers-with-hinged-_20260606_074856_914730_d0776185 ← picture/Container/Locker/001.png
  - 两并排金属储物柜:每 bay 一扇 REVOLUTE 侧铰门(`door_{idx}` / `hinge_{idx}`，绕左竖边 Z 轴外摆 0..100°)，门面带 VentGrilleGeometry 百叶 (`door_vent_{idx}`) + PerforatedPanel 号牌 (`door_plate_{idx}`)，门底锁盘上 10 颗 PRISMATIC 按键键盘 (`lockbtn_{idx}_{n}` / `btnjoint_{idx}_{n}`，循环发射)。共享 `carcass`(plinth/divider/per-bay back·side·top·bottom·shelf)。
  - parent 占格: closure=hinged_single_door, door_surface=louver_vents, latch=keypad_buttons, bank_count=2。
  - **§4 可读性问题**(见排除项/Multiplicity):bay 复制是 `for idx in (0, 1):` 手写元组，非 `range(n)`；键盘按键已是 `for r/c in range(...)` 合规循环。

## 组合数预审

组合数预审: Π(closure 4 × door_surface 4 × latch 3) × bank_count N(3) = 48 × 3 = 144 ≥ 10 ✓

## Slot 候选覆盖

### Slot A:closure_mechanism（**主机构槽**——锁门开合范式）
| 候选(未来 module) | record_id | 关键 part·joint·helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| hinged_single_door（基线 P1） | P1 | `door_{idx}` / `hinge_{idx}` REVOLUTE(左竖边 Z) | 单扇侧铰门外摆 | converged(parent) |
| double_leaf_doors | rec_container_locker_var_double_leaf_doors | `door_l_{i}`/`door_r_{i}` + `hinge_l_{i}`/`hinge_r_{i}` REVOLUTE | 一 bay 两窄扇，各铰外缘对开 | converged |
| sliding_door | rec_container_locker_var_sliding_door | `door_{i}` + `slide_{i}` PRISMATIC(沿 bank 宽 X) | 上下轨横移滑门 | converged |
| roll_down_shutter | rec_container_locker_var_roll_down_shutter | `shutter_slat_{i}`(循环) + `shutter_{i}` PRISMATIC(Z 升降) | 波纹卷帘逐片升入头箱 | converged |

### Slot B:door_surface（门面通风/表面结构）
| 候选 | record_id | 关键 part·joint·helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| louver_vents（基线 P1） | P1 | `door_vent_{idx}` (VentGrilleGeometry) | 顶部小百叶格栅 | converged(parent) |
| perforated_mesh_panel | rec_container_locker_var_perforated_mesh_panel | `door_mesh_{i}` (PerforatedPanelGeometry) | 整面满高冲孔板 | converged |
| horizontal_slot_vents | rec_container_locker_var_horizontal_slot_vents | `door_slot_{i}_{j}`(循环) | 多道细横缝等距列 | converged |
| solid_smooth_door | rec_container_locker_var_solid_smooth_door | `door_panel_{i}` + 内联加强肋边 visual | 无孔实心门(肋边为 parent visual) | converged |

### Slot C:latch_mechanism（锁/闩机构）
| 候选 | record_id | 关键 part·joint·helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| keypad_buttons（基线 P1） | P1 | `lockbtn_{idx}_{n}` / `btnjoint_{idx}_{n}` PRISMATIC(-Y 内压) | 10 颗循环按键键盘 | converged(parent) |
| padlock_hasp | rec_container_locker_var_padlock_hasp | `hasp_{i}` + `hasp_hinge_{i}` REVOLUTE + `padlock_{i}` | 铰式搭扣盖过门钉环 | converged |
| rotary_combo_dial | rec_container_locker_var_rotary_combo_dial | `dial_{i}` + `dial_spin_{i}` REVOLUTE(面法向) | 凹座内旋转密码盘 | converged |

## Multiplicity / Copy Logic
- count_param: `bank_count`（同一行并排 locker bay 数，源自 P1）
- N 样本已覆盖: {2(P1), 3, 4}
  - N=2 → P1（基线）
  - N=3 → rec_container_locker_var_bank_count_3
  - N=4 → rec_container_locker_var_bank_count_4（同时按 §4 把母资产 `for idx in (0, 1):` 改写为 `for i in range(n):` 循环链）
- 模板建议 N_range: [2, 8]（模板采样域，远大于样本覆盖值正常）
- copied object: 整个 bay = carcass 板件 + 一扇 REVOLUTE 门 + 门面通风 + 锁机构
- naming: `bay_{i}` / `door_{i}` / `hinge_{i}`（门底按键沿用 `lockbtn_{i}_{n}` 内层循环）
- placement: 沿 bank 宽度 X 等距排列，整 bank 居中 x=0
- joint policy: 每 bay 一个独立 REVOLUTE 铰门 + 各自锁机构 joint，统一策略

## 格子覆盖

| 槽 | 候选数(含基线) | 空格已填(变体) |
|---|---|---|
| A closure(主机构) | 4 | double_leaf / sliding / shutter |
| B door_surface | 4 | perforated_mesh / horizontal_slots / solid |
| C latch | 3 | padlock_hasp / rotary_dial |
| multiplicity bank_count | N∈{2,3,4} | N=3 / N=4 |

每槽 ≥2 候选 + 一根 multiplicity 3 个 N → 满足 §8 规划门槛。10 个变体填满全部空格。

## 排除项（未来 compatibility matrix 素材）
- **N=4 变体须先修复 parent 可读性契约**:母资产 bay 复制用 `for idx in (0, 1):` 手写元组，N=4 prompt 已显式要求重写为 `for i in range(n):` 循环链；其余 9 个变体不动 bay 复制层，照常 fork。
- 未列入轴的真实形态:open-front 无门储物格 / 整体多层堆叠(two-tier 上下两排) — 前者会丢失"锁柜"关节(无非 fixed joint，且接近出类目格物架)，后者本质是 bank_count 的二维扩展，留作未来模板 N 域/网格参数，不在本批铺设。
