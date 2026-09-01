# Technology / Graphics_Card — template source map

pattern: multiplicity  (dominant axis = number of cooling fans N; each fan is a homogeneous copy on its own CONTINUOUS spin joint; secondary named-slot structure for shroud / board / bracket / backplate / power)

parents (origins; forked ONLY from these):
- rec_an-msi-gaming-trio-style-triple-fan-gaming-graph_20260704_080843_205841_e43c8fd4 ← picture/Technology/Graphics_Card/004.png (MSI Gaming Trio, triple-fan) — covers fan N=3, open-axial gaming shroud, 8-pin power, no backplate
- rec_a-gigabyte-dual-fan-gaming-graphics-card-about-0_20260704_080416_406860_c9301248 ← picture/Technology/Graphics_Card/001.png (Gigabyte, dual-fan) — covers fan N=2 (side-by-side same face), open-axial gaming shroud, backplate PRESENT (with cutout)
- rec_an-nvidia-geforce-rtx-4090-founders-edition-grap_20260704_080420_464108_ccf04675 ← picture/Technology/Graphics_Card/002.png (RTX 4090 Founders Edition) — covers fan N=2 (flow-through dual-axial), flow-through founders shroud form, no backplate
- rec_a-zotac-gaming-compact-single-fan-graphics-card-_20260704_080424_599622_b4e75c5b ← picture/Technology/Graphics_Card/003.png (ZOTAC compact) — covers fan N=1, compact-ITX open-axial shroud, no backplate

All four origins accounted for as anchors (none excluded).

## Slot 候选覆盖

### Slot A: cooler_shroud_form  (③ Primary Form Family — Macro Surface Construction / Volumetric Envelope)
| 候选 (未来 module) | source_type | record_id / evidence | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|---|
| compact_itx_open_axial | forked_anchor | rec_...b4e75c5b (Zotac) | shroud.face_plate + fan_well_ring + slat_* diagonal vents; single fan_rotor | small dual-slot body, open circular window, diagonal slat flanks | converged (origin) |
| open_axial_gaming_dual | forked_anchor | rec_...c9301248 (Gigabyte) | shroud.faceplate (2 circle_holes) + accent ridges; fan_0/fan_1 | full-length card, 2 open windows same face, angular accent ridges | converged (origin) |
| open_axial_gaming_triple | forked_anchor | rec_...e43c8fd4 (MSI) | shroud.shroud_top_plate + accent_* wedges/straps + rgb_diffuser_strip; fan_0/1/2 | thick triple-slot, 3 open windows, gunmetal X accents, RGB strip | converged (origin) |
| flow_through_founders_dual | forked_anchor | rec_...ccf04675 (Founders) | shroud.top_panel + bottom_panel (stops mid-card) + fin_tail_* duct; tail_fan/bracket_fan | dual-axial, fans on OPPOSITE faces/ends, open finned tail duct, chevron X-trim | converged (origin) |
| blower_radial | forked_anchor (converged) | rec_graphics_card_var_blower ← parent Zotac | shroud enclosed face + top intake grille + bracket exhaust duct; fan_rotor cage wheel | fully enclosed shroud, single centrifugal cage fan, radial exhaust out bracket | converged |
| (further form variants) | world_knowledge_extrapolation (Macro Surface Construction / Volumetric Envelope) | anchors above + reviewer | same part tree (pcb/heatsink/shroud/bracket/fan) + same interface | e.g. vapor-chamber full-cover shroud, mesh-window shroud — only the boundary/envelope/surface form changes | template-side |

### Slot B: fan_count_multiplicity (N)  — see Multiplicity / Copy Logic (dominant axis)
| 候选 | source_type | record_id / evidence | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|---|
| N=1 | forked_anchor | rec_...b4e75c5b (Zotac) | fan_rotor / fan_spin (CONTINUOUS) | single central axial rotor | converged (origin) |
| N=2 | forked_anchor | rec_...c9301248 (Gigabyte) | fan_0/fan_1 / heatsink_to_fan_{i} (CONTINUOUS) | two side-by-side rotors, looped | converged (origin) |
| N=2 (flow-through) | forked_anchor | rec_...ccf04675 (Founders) | tail_fan/bracket_fan / *_fan_spin (CONTINUOUS) | two rotors opposite faces/ends (hand-written, asymmetric — see readability note) | converged (origin) |
| N=3 | forked_anchor | rec_...e43c8fd4 (MSI) | fan_0/1/2 / fan_{i}_spin (CONTINUOUS) | three rotors, looped `for i in range(3)` | converged (origin) |

### Slot C: backplate (rear-face optional structural part)
| 候选 | source_type | record_id / evidence | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|---|
| absent | forked_anchor | rec_...e43c8fd4 / ...b4e75c5b / ...ccf04675 (MSI, Zotac, Founders) | (no backplate part) | bare PCB rear | converged (origin) |
| present_with_cutout | forked_anchor | rec_...c9301248 (Gigabyte) | backplate.plate (rect_holes cutout) + brand_plate + screw_0..5 (looped), pcb_to_backplate FIXED | full metal backplate, vent cutout, branding, screws | converged (origin) |
| present_solid | world_knowledge_extrapolation (④ host-conformal / remove cutout) | anchors: Gigabyte + reviewer | same backplate part tree + FIXED interface | full-coverage plate without the vent cutout | template-side |

### Slot D: power_connector (board top-edge power input; multiplicity + form)
| 候选 | source_type | record_id / evidence | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|---|
| single_8pin | forked_anchor | rec_...e43c8fd4 (MSI) | board.power_connector_8pin (single hand-written block) | one PCIe 8-pin block on top edge | converged (origin) |
| triple_8pin (n_8pin=3) | forked_anchor (converged) | rec_graphics_card_var_power_triple_8pin ← parent MSI | board.power_connector_8pin_{i} looped (n_8pin=3) | row of three 8-pin connectors, looped copy chain | converged |
| 12vhpwr_16pin | forked_anchor (converged) | rec_graphics_card_var_power_12vhpwr ← parent MSI | board.power_connector_12vhpwr + sense-pin sub-band | single wide 16-pin 12V-2x6 connector | converged |
| (power absent) | record_only | Zotac/Gigabyte/Founders origins do not model a power block | — | low-power / slot-powered cards omit it | template-side (optional part = false) |

### Slot E: articulation_mechanism (② joint type — beyond the always-present spinning fans)
| 候选 | source_type | record_id / evidence | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|---|
| fans_only (CONTINUOUS spins) | forked_anchor | all 4 origins | fan_*_spin CONTINUOUS (axis = card thickness) | every card: N spinning rotors, no other moving part | converged (origin) |
| foldout_support_foot (REVOLUTE) | forked_anchor (converged) | rec_graphics_card_var_support_bracket ← parent MSI | board hinge-lug + support_foot part + support_foot_hinge REVOLUTE (axis X, ~0..90 deg) | anti-sag leg that swings down under the tail; keeps the 3 fan CONTINUOUS joints | converged |

## Multiplicity / Copy Logic
- Primary count_param: `fan_count` (N cooling fans). Each fan is a homogeneous copy: shared FanRotorGeometry/rotor mesh helper, regular placement along card length (FAN_CENTERS_X / FAN_X), one CONTINUOUS spin joint per fan about the card-thickness axis, off-axis hub badge to make spin observable.
- N 样本已覆盖: {1, 2, 3} → Zotac (1) / Gigabyte + Founders (2) / MSI (3). Founders' N=2 is a flow-through asymmetric pair (opposite faces/ends/axes).
- 模板建议 N_range: [1, 4] (real cards 1–3 common; 4-fan exists but rare — template extrapolates upward; do NOT fork N=4).
- copied object / naming / placement / joint policy: rotor part `fan_{i}` (or descriptive name for the flow-through pair) / equal-pitch along +X on the fan face / each on its own CONTINUOUS joint, axis (0,0,1)-thickness, independent spin.
- Secondary count_param: `n_8pin` (Slot D power-connector row). Samples {1 (MSI), 3 (var_power_triple_8pin)}; template N_range [1,3]; copied object `power_connector_8pin_{i}`, equal pitch along the board top edge, all FIXED to the board.

## 视觉多样性 6 轴考察 (对齐下游 SPEC §8.5)

| 轴 | 处理 | 本小类取值 / 范围 / 理由 |
|---|---|---|
| ① 骨架图 (+N) | forked_anchor → Slot A/B/C/D/E | Named-slot skeleton: board/pcb + heatsink(fins) + shroud + I/O bracket + N fans (+ optional backplate + optional power block + optional support foot). N fans is the multiplicity. No world-knowledge-only new skeleton candidates. |
| ② 关节类型 | forked_anchor (随 module) | CONTINUOUS fan spin (axis = card thickness) on every origin; + REVOLUTE fold-out support foot (var_support_bracket, axis = card length, limit ~0..90 deg). All FIXED for shroud/heatsink/bracket/backplate/PCB mounts. No world-knowledge new candidate. |
| ③ 主体形态家族 / Primary Form Family | forked_anchor + world_knowledge_extrapolation | Anchors: compact_itx_open_axial, open_axial_gaming (dual/triple), flow_through_founders_dual, blower_radial (fork). Extrapolatable (same part tree/primitive/interface): Volumetric Envelope (thickness 1/2/3-slot), Macro Surface Construction (full-cover vs open-fin vs mesh-window shroud), Planar Boundary (shroud outline). |
| ④ 表面装饰 | record_only + world_knowledge_extrapolation | Observed: gunmetal X/chevron accent panels & straps (MSI/Founders), angular accent ridges (Gigabyte), copper/amber strips + diagonal slat texture + ZOTAC/HDMI lettering (Zotac), RGB diffuser strip (MSI), brand lettering + screw dots (Gigabyte/backplate), silicon-die labels (RTX 4090 / GEFORCE RTX). Extrapolatable host-conformal: brand text, model badges, louver ribs, rivets/screws, RGB light bars. |
| ⑤ 尺寸/行程 | record_only | Card length ~0.17 m (ITX) → ~0.32 m (triple); thickness 2-slot (~0.035–0.047) → 3-slot (~0.058–0.070); support-foot swing 0..90 deg; fan velocity limits 30–180. |
| ⑥ 涂装 | record_only | Material大类: painted metal / plastic shroud (gunmetal, matte black), aluminum/nickel fins, gold PCIe fingers, silver steel bracket, translucent RGB diffuser, PCB black/green. Colorways ≥3–6: all-black, gunmetal-gray, black+red (MSI dragon), black+copper (Zotac), silver+black (Founders). |

## Compatibility Probes
| probe_id | source_type | record_id | 组合轴值 | 验证目标 | 结论 |
|---|---|---|---|---|---|
| (none) | — | — | — | — | — |

Note: blower_radial × backplate_present and flow_through × solid_backplate are real airflow conflicts (a solid backplate blocks flow-through / bottom intake). Flagged for the template compatibility matrix as gate-worthy, but not worth a fork probe now (single-object obviousness, low interface risk).

## 排除项 (未来 compatibility matrix 素材)
- fanless / passive (no fan) form: excluded — would leave 0 non-fixed joints (fails the ≥1-active-joint gate). Keep at least one spinning fan.
- AIO / waterblock (fans replaced by pump+tubes) form: excluded — high out-of-category risk and removes the fan mechanism; not anchored.
- backplate fork: NOT forked — Slot C already has both structural values source-backed (present=Gigabyte, absent=MSI/Zotac/Founders); a present_solid variant is ④ extrapolation, not a new cell.
- N=4 fans: NOT forked — multiplicity already covered by N∈{1,2,3}; template extrapolates via N_range.

## Readability audit (§4)
- MSI: fans looped `for i in range(3): _add_fan(...)` + `fan_{i}_spin`; fins/heatpipes/motor_pods/display_ports all looped. PASS.
- Gigabyte: fans looped `for i,fx in enumerate(FAN_X)` + `heatsink_to_fan_{i}`; fins/screws/ports looped; has clean backplate part. PASS.
- Zotac: single fan `fan_rotor` (N=1 — a single named part is acceptable for N=1); blades looped `blade_{i}`; slats/fins looped. PASS.
- Founders: fans HAND-WRITTEN as `tail_fan` + `bracket_fan` (NOT looped). FLAG — but justified: the flow-through pair is asymmetric (opposite faces/ends/spin-axes), not homogeneous copies; static bays use shared `_add_fan_bay` helper. NOT fork-blocking. Consequence: do NOT fork the fan-N multiplicity axis FROM Founders (N forks here use MSI/Gigabyte, which loop). No N fork is planned from Founders, so no action needed.
- MSI `power_connector_8pin` is a single hand-written block (fine at n=1); the triple-8-pin fork will rewrite it into a looped `power_connector_8pin_{i}` copy chain, improving readability.

## Sync note
Variants are workbench-only; on sync to arti-template, script writes rating=5. Do NOT promote / do NOT pass --category-slug.
