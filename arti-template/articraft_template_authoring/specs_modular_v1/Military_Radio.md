# Modular Spec — Military / Radio

## 元信息
| 项 | 值 |
|---|---|
| slug | `field_radio` |
| template path | `agent/templates/Military_Radio.py` |
| stage | `SPEC_ONLY_DRAFT` |
| status | `pending` |
| __modular__ | `True` |
| pattern | `mixed`（2 个结构 slot：antenna × control-layout；外加 1 根 multiplicity 轴 `knob_count`，及次级 keypad-grid 多重性） |

## 5 星样本阅读摘要
| 项 | 值 |
|---|---|
| five_star_total | 9（1 parent + 8 variants） |
| read_count | 9 |

**共性骨架**：root `body`（`_rounded_slab`/`_lower_shell_shape`/`_upper_shell_shape`/`_lcd_bezel_shape`，keypad loop `key_{r}_{c}`），5 非 fixed 关节：`knob_spin`(CONTINUOUS) + `antenna_fold`(REVOLUTE) + 2×PTT(PRISMATIC) + `clip_hinge`(REVOLUTE)。共享 helper `_antenna_shape`(~L143)、`_ptt_button_shape`(~L107)、`_belt_clip_shape`(~L120)。

## 核心身份
军用手持双向对讲机（rugged handheld two-way radio）。识别 = **天线 pivot + 旋钮/PTT 机构**（≥1 非 fixed joint）。不该混入：手机（无天线 pivot/PTT）、遥控器、桌面基站。

## 槽位 + 候选模块表

### Slot A：antenna（天线机构）
| module_name | 5_star_source | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|
| foldover_whip | parent | `_antenna_shape` L143-L164 | eligible | 长 whip，单 `antenna_fold` REVOLUTE 0-90° |
| articulated_2seg | rec_field_radio_var_antfold2seg | `_antenna_base_shape` L145-L176 + `_antenna_whip_shape` L177-L197 | eligible | 两段：base + whip，elbow REVOLUTE（多 1 活动关节） |
| stub | rec_field_radio_var_antstub | `_antenna_shape` L143-L177 | eligible | 短橡胶 stub，仍 fold REVOLUTE |

### Slot B：control-layout（前面板控制）
| module_name | 5_star_source | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|
| keypad4x4_dualptt | parent | build L165-L351 | eligible | 4×4 keypad + 侧 PTT |
| flip_cover | rec_field_radio_var_flipcover | `_cover_panel_shape` L153-L194 + `_cover_hinge_boss_shape` L195-L218 | eligible | LCD/keypad 上翻盖（REVOLUTE 0-160°） |
| rotary_selector | rec_field_radio_var_seldial | `_dial_bezel_shape` L170-L182 | eligible | 大旋转频道选择盘（替下半 keypad，CONTINUOUS） |
| keypad4x3 | rec_field_radio_var_keypad4x3 | build L165-L351（key loop 4×3） | eligible | 次级 keypad-grid 多重性（4×4→4×3） |
| single_bar_ptt | rec_field_radio_var_dualptt | `_ptt_bar_shape` L107-L121 | eligible | 单长条 PTT（2 按钮→1 bar，PRISMATIC） |

> **single-candidate slot degrade**：无（A=3, B=5）。

## 槽位图（slot graph）
```
 body (root: shells + lcd + keypad loop)
   ├─ [Slot A antenna] top boss ─antenna_fold(REV)─► whip (单段/两段 elbow/stub)
   ├─ [multiplicity] top_knob_{i} (CONTINUOUS 各自) — 单 knob 改写为循环
   ├─ [Slot B control] keypad/flip-cover(REV)/rotary-dial(CONT)/单bar-PTT(PRISM)
   ├─ ptt (PRISMATIC) / belt_clip (REVOLUTE)
```

## 每槽位 Module Emits / Interfaces
- **Slot A**：emits 天线 visual + `antenna_fold` REVOLUTE 锚 top boss；2seg 加 elbow REVOLUTE。
- **Slot B**：keypad（key loop）；flip_cover emits 盖 + hinge REVOLUTE；rotary_selector emits 盘 + CONTINUOUS；single_bar_ptt 把 2 PTT 改 1 bar PRISMATIC。
- **Multiplicity**：`top_knob_{i}`（shared KnobGeometry helper + boss，各自 CONTINUOUS）—— **parent 单 knob 须 loop-rewrite** 成 `top_knob_{i}` 链，沿顶面等距。

## 参数范围汇总
| 参数 | 类型 | 取值 | 约束 | 来源 |
|---|---|---|---|---|
| antenna | enum | foldover_whip / articulated_2seg / stub | choice | Slot A |
| control_layout | enum | keypad4x4_dualptt / flip_cover / rotary_selector / keypad4x3 / single_bar_ptt | choice | Slot B |
| palette_style | enum | od_green / black / tan / coyote（≥3） | palette only | S 材质 |
| knob_count | int | {1,2,3}，N_range [1,4] | multiplicity（loop-rewrite 单 knob，各 CONTINUOUS） | knob2/knob3 |
| keypad_cols | int | {3,4} | 次级 multiplicity（key loop） | keypad4x3 |
| body_scale | float | [0.9,1.15] | independent clamp | parent |

## Multiplicity / Copy Logic
- **count_param**：`knob_count`（顶面旋钮数）。
- **N_range**：[1, 4]；采样 {1,2,3}（parent=1）。
- **copied object**：旋钮（KnobGeometry cap + shaft + boss）。
- **naming**：`top_knob_{i}`，沿顶面等距。
- **joint policy**：每旋钮独立 CONTINUOUS 绕 +Z。**parent 单 knob 是手写的**，N≥2 变体须改写成 `for i in range(n)` 循环（loop-rewrite 契约）。
- 次级轴 `keypad_cols`（4×4↔4×3）走已存在 key loop。

## 拓扑多样性审计
- A(3) × B(5) = **15** 纯 slot；× knob N{1,2,3} = **45+** distinct。
- procedural_first：采 A/B → 加权采 knob_count → palette → 连续 scale。
- 兼容矩阵：rotary_selector 替下半 keypad（与 keypad_cols 互斥）；其余两两兼容。
- Topology target ~84（含 keypad/PTT 次级）；远超 10。（统一口径：富类别建议 ≥300；低于 300 记录真实组合空间或兼容约束原因；report-only，不设门。）

## Validator
- `antenna_fold` REVOLUTE 恒在；2seg elbow 正确。
- knob_count loop-rewrite（非手写复制）；各 CONTINUOUS。
- flip_cover hinge REVOLUTE / rotary CONTINUOUS / single_bar PTT PRISMATIC 各自正确。
- element-scoped allow_overlap（antenna↔boss、knob↔body、cover↔body）。
- 连续 scale clamp。

## Reject cases
- antenna fold 降为 FIXED 且无其他活动 → 0 非 fixed，拒收（fixed long-whip 已 drop）。
- 单 knob 手写复制成多 knob（不循环）→ 违反 loop-rewrite 契约。
- 当成手机/无天线设备 → 出类目。
- palette/scale 当 candidate → 非结构差异。

## 与相邻类别的边界
- 手机/PDA：无可折天线 + PTT。
- TV 遥控器：无天线 pivot + 旋钮自转。
- 桌面/车载基站：非手持、有底座语义。

## 审核记录
| 项 | 结论 |
|---|---|
| reviewer status | pending |

## Module Source Index
| source_id | slot | module | record_id |
|---|---|---|---|
| S1 | A/B | foldover_whip / keypad4x4 (parent) | rec_model-a-rugged-...-r_..._3ffa3864 |
| S2 | A | articulated_2seg | rec_field_radio_var_antfold2seg |
| S3 | A | stub | rec_field_radio_var_antstub |
| S4 | B | flip_cover | rec_field_radio_var_flipcover |
| S5 | B | rotary_selector | rec_field_radio_var_seldial |
| S6 | B | keypad4x3 | rec_field_radio_var_keypad4x3 |
| S7 | B | single_bar_ptt | rec_field_radio_var_dualptt |
| S8/S9 | multiplicity | knob_count N=2/3 | rec_field_radio_var_knob{2,3} |
