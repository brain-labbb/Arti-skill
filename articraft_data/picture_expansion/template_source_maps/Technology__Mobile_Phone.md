# Technology / Mobile_Phone — template source map

pattern: mixed  (③ primary form family + multiplicity keypad-count + optional antenna subsystem)

parents:
- rec_nokia-3310-candybar-mobile-phone-dark-blue-with-_20260605_174005_164763_8c6bf79a <- picture/Technology/Mobile_Phone/001.png  (candybar monoblock, 15-key numeric+nav keypad, no hinge/antenna) — covers Slot A=candybar, Slot B=numeric_12+nav_cluster(N=15), Slot C=none
- rec_black-clamshell-flip-phone-for-seniors-with-a-hi_20260605_174014_810624_7ec9acc7 <- picture/Technology/Mobile_Phone/002.png  (senior clamshell flip, revolute flip_hinge, 21-key big-function keypad, dual screens) — covers Slot A=clamshell, Slot B=numeric_12+full_function(N=21), Slot C=none

Identity:
- a handheld personal mobile phone (candybar / flip / slider / swivel / touch-slab)
- has a front display and either a physical keypad OR retained physical buttons
- NOT a cordless landline handset, NOT a tablet, NOT a two-way radio/walkie-talkie, NOT a smart-watch

## Slot 候选覆盖

### Slot A: ③ primary form family (phone body form factor)  [dominant]
| 候选 (未来 module) | source_type | record_id / evidence | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|---|
| candybar_monoblock | forked_anchor (parent) | rec_nokia...8c6bf79a | body / body_shell; keys body_to_<name> (prismatic) | one fixed monoblock slab, keypad set into front face, no reveal joint | converged (origin) |
| clamshell_flip | forked_anchor (parent) | rec_black...7ec9acc7 | body + lid / flip_hinge (REVOLUTE +Y) | two hinged halves, screen on lid, keypad on base | converged (origin) |
| slider | forked_anchor | rec_mobile_phone_var_slider | body(lower) + screen slab / prismatic slide (+Y) | two stacked slabs, upper screen slab slides up to reveal keypad | converged |
| swivel_rotator | forked_anchor | rec_mobile_phone_var_swivel | body(lower) + screen slab / revolute pivot (+Z face-normal) | upper screen slab rotates ~180° on a corner pivot to reveal keypad | converged |
| touch_slab | forked_anchor | rec_mobile_phone_var_touch_slab | body / button_{i} (prismatic) | full edge-to-edge touchscreen, keypad removed, only home/power/volume buttons | converged |
| (bar_with_stub_antenna / rugged-brick / rotator-hinge variants) | world_knowledge_extrapolation (Volumetric Envelope / Macro Surface) | anchors: above + reviewer | same part tree/interface | template-side envelope/surface fan-out only | template-side |

### Slot B: keypad layout / count (= multiplicity axis)
| 候选 (未来 module) | source_type | record_id / evidence | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|---|
| numeric_12 + nav_cluster (N=15) | forked_anchor (parent) | rec_nokia...8c6bf79a | key_1..key_hash + key_nav/key_c/key_arrow; body_to_<name> | 4x3 numeric grid + 3-key function cluster | converged (origin) |
| numeric_12 + full_function (N=21) | forked_anchor (parent) | rec_black...7ec9acc7 | key_num_1..key_num_hash + key_soft_left..key_function_dark; press_<name> | 4x3 numeric grid + 9-key D-pad/soft/send/end cluster | converged (origin) |
| numeric_12 only (N=12) | forked_anchor | rec_mobile_phone_var_numeric_only | key_1..key_hash; body_to_<name> | bare 4x3 numeric grid, function cluster dropped (low-N copy logic) | converged |
| qwerty_matrix (N≈34-40) | forked_anchor | rec_mobile_phone_var_qwerty | key_r{r}_c{c}; body_to_<name> | ~4x10 rectangular QWERTY key matrix (high-N copy logic) | converged |

### Slot C: external antenna
| 候选 (未来 module) | source_type | record_id / evidence | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|---|
| none | forked_anchor (both parents) | rec_nokia...8c6bf79a, rec_black...7ec9acc7 | (no antenna part) | flush top, internal antenna | converged (origin) |
| telescoping_mast | forked_anchor | rec_mobile_phone_var_telescoping_antenna | antenna mast part / antenna prismatic (+Y) on body_shell boss | pull-up telescoping external stub, extends/retracts | converged |
| fixed_stub | world_knowledge_extrapolation (④, host-conformal) | anchors above + reviewer | body_shell.visual(...) | short fixed non-jointed nub (inline decoration) | template-side |

## Multiplicity / Copy Logic
- count_param: `key_count` (keypad keys emitted by a single for-loop with `key_<label>` / `key_r{r}_c{c}` naming + shared keycap helper + regular grid placement + uniform straight-down PRISMATIC press policy `body_to_<name>` / `press_<name>`)
- N 样本已覆盖: {12, 15, 21, ~34-40} → numeric_only(12) / nokia(15) / clamshell(21) / qwerty(~34-40)
- 模板建议 N_range: [10, 48] (numeric-basic ~12 .. senior-function ~21 .. full QWERTY ~40; sampler域远大于样本正常)
- copied object / naming / placement / joint policy: one keycap mesh helper; `key_*` names; row/column grid placement; every copy an independent straight-down PRISMATIC press (never chained, never fixed)

## 视觉多样性 6 轴考察 (对齐下游 SPEC §8.5)

| 轴 | 处理 | 本小类取值 / 范围 / 理由 |
|---|---|---|
| ① 骨架图 (+N) | forked_anchor → 见 Slot A / Multiplicity | 5 source-backed 骨架: candybar monoblock / clamshell 2-body-hinged / slider 2-body-prismatic / swivel 2-body-revolute / touch-slab monoblock. 无世界知识新增 candidate. |
| ② 关节类型 | forked_anchor (随 module) | REVOLUTE (flip_hinge +Y; swivel pivot +Z) / PRISMATIC (slider slide +Y; every keypad/button press -Z; telescoping antenna +Y). 覆盖 revolute + prismatic. |
| ③ 主体形态家族 / Primary Form Family | forked_anchor + world_knowledge_extrapolation | anchors: candybar, clamshell, slider, swivel, touch_slab. 可外推 Volumetric Envelope (rugged-brick 加厚 / slim slab) + Macro Surface (proportions) + 加装 fixed_stub 天线, 保持同 part tree/interface. |
| ④ 表面装饰 | record_only + world_knowledge_extrapolation | 真实样本: NOKIA 银牌 wordmark, 7-段数字+T9 字母键面, 屏内 LCD 像素/信号/电池图标, 银色高光条, hinge_end_cap 银点, side_rail 模压唇边. 可外推同类 host-conformal 印刷/铭牌/键面图例数量档. |
| ⑤ 尺寸/行程 | record_only | 机身高:宽 ≈ 2.0-2.4:1 (candybar 窄长) ; 键程 press ≈ 1.0-1.15 mm ; flip_hinge 0..~98-155° ; slider/antenna 行程 ≈ 机身长的 40-70%. |
| ⑥ 涂装 | record_only | 材质大类: glossy plastic (主) / metal accents (silver trim, gold contact) / glass screen. 配色 ≥6: dark-navy, gloss-black, graphite, silver, white(touch), two-tone. origin 均偏暗 → 变体挂 ⑥ colorway companions. |

## Compatibility Probes
| probe_id | source_type | record_id | 组合轴值 | 验证目标 | 结论 |
|---|---|---|---|---|---|
| (none) | — | — | — | — | 本批不设 probe; slot A/B/C 组合无高风险接口, 由模板采样器自由组合 |

## Origin 对账
- rec_nokia...8c6bf79a → 上格 anchor: Slot A=candybar, Slot B=N15, Slot C=none; hosts 5 forks (slider, touch_slab, swivel, numeric_only, qwerty).
- rec_black...7ec9acc7 → 上格 anchor: Slot A=clamshell, Slot B=N21, Slot C=none; hosts 1 fork (telescoping_antenna, StarTAC-style — keeps flip_hinge baseline, adds antenna prismatic).
- 两 origin 均已对账入上格并各有 ≥1 fork.

## 排除项 (未来 compatibility matrix 素材)
- swivel_rotator 为本批最高风险格 (screen 半体绕 +Z 角枢旋转的角枢+承托较难干净收敛); 若连续 2-3 次 compile/tests 不收敛, 退回更简单的 slider 形态或记 blocked, 并在此登记原因 (angle-pivot 承托 / 穿插 / 出类目).

## 备注
- 变体保持 workbench-only; 收敛即 rating=5 同步进 arti-template; 不 promote, 不传 category-slug.
- 每变体 ≥1 非 fixed joint 已保证 (slider prismatic / swivel revolute / touch_slab button prismatic / numeric_only+qwerty keypad prismatic / telescoping_antenna hinge revolute + antenna prismatic).
