# Military / Aircraft — modular template spec

## 元信息
| 项 | 值 |
|---|---|
| slug | `military_aircraft` |
| template path | `agent/templates/Military_Aircraft.py` |
| test path (optional) | `tests/agent/test_military_aircraft_template.py` |
| stage | `SPEC_ONLY_DRAFT` |
| status | `pending` |
| __modular__ | `True` |
| pattern | `mixed` (parallel children parented to the fuselage spine + one multiplicity axis = engine/propeller count) |

## 5 星样本阅读摘要
| 项 | 值 |
|---|---|
| five_star_total | 10 |
| read_count | 10 |
| read_scope | all 5-star samples in this category (2 parents + 8 converged variants) |
| source_index_policy | only adopted module sources are indexed below |

**Reading notes (every source read in full):**

- **S1 `rec_model-a-wwii-era-single-engine-fighter...` (parent, fighter, 985 L)** — Bearcat/Mustang-style single-engine fighter. Root `fuselage` lathe hull (`HULL_PROFILE`, L58-71) at `CL_Z=1.08`; FIXED `wing`, FIXED `horizontal_stabilizer`, FIXED `tail_fin`; REVOLUTE `rudder` on a raked near-vertical hinge (`RUDDER_AXIS`, ±30°, L597-624); REVOLUTE one-piece `elevator` on lateral +Y (±25°, L641-663); single CONTINUOUS `propeller` (4-blade X-pattern) about +X at the nose centerline (L665-695). Gear-up (no gear). Glossy-blue palette + white "402" + yellow fin band/markings (7 materials L493-499).
- **S2 `rec_model-a-wwii-twin-engine-attack-bomber...` (parent, bomber, 1091 L)** — Douglas A-26-style twin-engine bomber. Root `fuselage` (longer `HULL_PROFILE` L62-76) at `CL_Z=1.28`; FIXED `wing`; **two HAND-WRITTEN engines** via `add_nacelle_and_prop(+1,"left")` / `(-1,"right")` (L715-760) — each emits a `{tag}_nacelle` (hull+cowl_ring+engine_face+antiglare_panel) FIXED to the wing + a 3-blade `{tag}_propeller` CONTINUOUS about +X. Tall **fixed** fin in three lofted bands (`fin_lower`/`fin_band`/`fin_upper`, yellow band, L777-787); FIXED one-piece stabilizer; extra CONTINUOUS dorsal `gun_turret` (L762-775). Gear-up. Bare-metal silver + olive-drab anti-glare + US star-and-bar insignia (10 materials L643-652).
- **S3 `rec_military_aircraft_var_stabilator` (959 L)** — fighter-derived; replaces the fixed stabilizer + hinged elevator with one all-moving `stabilator` (loft + `pivot_shaft`) on a REVOLUTE fuselage lateral +Y pivot (±20°, build L623-645); retains the hinged `rudder`. Single nose prop. Gloss-blue palette.
- **S4 `rec_military_aircraft_var_twintail` (1055 L)** — fighter-derived; replaces the single fin with **two endplate fins** at the stabilizer tips, each carrying its own hinged rudder, emitted in `for i, y_sign in enumerate((-1.0, 1.0))` (build L627-679); fins FIXED at `±TWIN_FIN_Y=1.75`, rudders REVOLUTE on a raked axis (±28°); retains one-piece elevator. Single nose prop. Gloss-blue palette.
- **S5 `rec_military_aircraft_var_rudderhinge` (1280 L)** — bomber-derived; splits the bomber's tall **fixed** fin into a fixed forward fin (`fin_fixed_lower`/`fin_fixed_band`/`fin_fixed_upper`) + a hinged `rudder` (`rudder_lower`/`rudder_band`/`rudder_upper`/`hinge_barrels`) REVOLUTE on `HINGE_AXIS` (±30°, build L844-903). Twin hand-written engines. Silver/insignia palette.
- **S6 `rec_military_aircraft_var_elevatorsplit` (1287 L)** — bomber-derived; splits the bomber's one-piece stabilizer into a fixed `horizontal_stabilizer` (`stabilizer_fixed_loft`) + a hinged `elevator` (`elevator_surface`/`torque_tube`) REVOLUTE on +Y (±`ELEVATOR_LIMIT_RAD`=22°, build L859-893). Twin hand-written engines. Silver/insignia palette.
- **S7 `rec_military_aircraft_var_taildragger` (1207 L)** — fighter-derived; adds a **taildragger** stance: two main `gear_strut_{i}`/`gear_wheel_{i}` FIXED visuals on the rigid wing via `for gear_i in range(2)` (`y_sign = 1 - 2*gear_i`, build L726-738) + a `tail_gear_strut`/`tail_gear_wheel` visual inline on the fuselage (L695-707). Single nose prop. Gloss-blue + `gear_metal` + `tire_rubber` palette (9 materials).
- **S8 `rec_military_aircraft_var_tricycle` (1309 L)** — bomber-derived; adds a **tricycle** stance: two main `gear_{i}` parts FIXED to their nacelles (`nacelle_to_gear_{i}`, build L837-859) + a `nose_gear` part FIXED to the fuselage (`fuselage_to_nose_gear`, L861-883). Twin hand-written engines. Silver/insignia + `gear_steel` + `tire_rubber` palette (12 materials).
- **S9 `rec_military_aircraft_var_singleengine` (1125 L)** — **the loop-rewrite, N=1.** Rewrites the bomber's two hand-written engines into `for i, (nac_y, nac_front_x, nac_dz) in enumerate(NACELLE_STATIONS)` (build L721-775) with `NACELLE_STATIONS=[(0.0, NAC_FRONT_X, 0.0)]` (L716-719); single centerline nose nacelle `engine_0` FIXED to the **fuselage** (`fuselage_to_engine_0`), `propeller_0` CONTINUOUS about +X. Fixed fin + stabilizer. Silver/insignia palette.
- **S10 `rec_military_aircraft_var_fourengine` (1134 L)** — **the loop-rewrite, N=4.** `for i in range(N_ENGINE)` (build L721-764) over `NAC_Y_POSITIONS=[3.5, 7.8, -3.5, -7.8]`, `N_ENGINE=len(...)` (L48-49); each `engine_{i}` FIXED to the **wing** (`wing_to_engine_{i}`), `propeller_{i}` CONTINUOUS about +X. Spanwise-mirrored. Fixed fin + stabilizer. Silver/insignia palette.

## 核心身份

A propeller-driven, fixed-wing **piston-era military aircraft** (WWII fighter ↔ multi-engine bomber family). The grounded root is a lofted **fuselage** at centerline height `CL_Z`, carrying a one-piece FIXED **wing**, a **tail empennage** (vertical fin + horizontal tail, with selectable movable control surfaces), and **N engine/propeller units** whose count is the multiplicity axis (1 nose-centerline prop for fighters, 2/4 wing-mounted props for bombers). Identity invariants:

- Always present: lofted fuselage hull, one-piece wing, vertical fin, horizontal tail, and ≥1 spinning propeller on a CONTINUOUS +X joint. Canopy is a fixed fuselage visual (no canopy articulation — see 排除项).
- The default mature domain is **piston/propeller** aircraft. National markings, gloss vs bare-metal finish, and overall size are cosmetic/parametric (handled by `palette_style` + scale), **not** module slots.
- Rest pose: lowest geometry (resting blade tips at rest pose) grazes ground plane `z≈0`; gear-down stance lowers the wheels to the ground instead.

Must NOT mix in: jet aircraft (no propeller / different thrust spine), helicopters (main rotor + tail rotor articulation, no fixed wing), or civil airliners (no military empennage/insignia identity, different proportions).

## 槽位 + 候选模块表

### Slot A：empennage / tail control-surface

| module_name | 5_star_source | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|
| hinged_rudder_plus_elevator | S1 `rec_model-a-wwii-era-single-engine-fighter...` | helpers L225-324; build L567-663 | eligible if compatible | Single fin + REVOLUTE `rudder` on raked near-vertical hinge (`RUDDER_AXIS`, ±30°) **and** one-piece REVOLUTE `elevator` on lateral +Y (±25°). Two movable links: `tail_fin`(FIXED)+`rudder`(REVOLUTE), `horizontal_stabilizer`(FIXED)+`elevator`(REVOLUTE). |
| fixed_tall_fin | S2 `rec_model-a-wwii-twin-engine-attack-bomber...` | helpers L248-319; build L777-794 | eligible if compatible | Tall **rigid** empennage: three-band lofted `tail_fin`(`fin_lower`/`fin_band`/`fin_upper`, yellow band) FIXED + one-piece `horizontal_stabilizer` FIXED. No movable rudder/elevator (surfaces baked into the loft). |
| all_moving_stabilator | S3 `rec_military_aircraft_var_stabilator` | helpers L287-320; build L623-645 (+ retains rudder block) | eligible if compatible | Replaces fixed stabilizer+elevator with one all-moving `stabilator` (`stabilator_loft`+`pivot_shaft`) REVOLUTE on fuselage lateral +Y (±20°); retains the single fin + hinged `rudder`. |
| twin_fin_rudders | S4 `rec_military_aircraft_var_twintail` | helpers L235-295; build L627-679 (+ stabilizer L587-600, elevator L602-624) | eligible if compatible | Two endplate `fin_{i}` FIXED at stabilizer tips (`±TWIN_FIN_Y`, emitted in `for i, y_sign in enumerate((-1,1))`), each carrying a REVOLUTE `rudder_{i}` (raked `TWIN_RUDDER_AXIS`, ±28°); retains one-piece elevator on the stabilizer. |
| split_rudder_off_bomber_fin | S5 `rec_military_aircraft_var_rudderhinge` | helpers L299-351 (+ HINGE_AXIS L279-282); build L844-903 | eligible if compatible | Bomber tall fin split into a fixed forward fin (`fin_fixed_lower`/`fin_fixed_band`/`fin_fixed_upper`) + hinged `rudder`(`rudder_lower`/`rudder_band`/`rudder_upper`/`hinge_barrels`) REVOLUTE on `HINGE_AXIS` (±30°); horizontal tail stays one-piece FIXED. |
| split_elevator_off_bomber | S6 `rec_military_aircraft_var_elevatorsplit` | helpers L330-389 (+ ELEVATOR_HINGE_X/LIMIT L326-327); build L859-893 | eligible if compatible | Bomber one-piece stabilizer split into fixed `horizontal_stabilizer`(`stabilizer_fixed_loft`) + hinged `elevator`(`elevator_surface`/`torque_tube`) REVOLUTE on +Y (±`ELEVATOR_LIMIT_RAD`=22°); vertical fin stays tall FIXED. |

### Slot B：landing gear / stance

| module_name | 5_star_source | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|
| gear_up | S1 `rec_model-a-wwii-era-single-engine-fighter...` / S2 (shared state) | (no gear parts/joints; grounding rule in tests, S1 L860-869 / S2 L941-954) | eligible if compatible | Clean gear-up airframe: no gear parts/joints. Lowest geometry is the resting propeller disc grazing `z≈0`. Both parents and all engine-multiplicity variants share this. |
| fixed_taildragger | S7 `rec_military_aircraft_var_taildragger` | helpers L508-631; build main loop L726-738 (`for gear_i in range(2)`, `y_sign=1-2*gear_i`), tail visual L695-707 | eligible if compatible | Two main `gear_strut_{i}`/`gear_wheel_{i}` FIXED visuals mirrored under the wing + an inline `tail_gear_strut`/`tail_gear_wheel` on the aft fuselage (taildragger stance); main legs are visuals on the rigid wing (no extra joints). |
| fixed_tricycle | S8 `rec_military_aircraft_var_tricycle` | helpers L460-496; build mains L837-859 (`nacelle_to_gear_{i}` FIXED, parent=nacelle), nose L861-883 (`fuselage_to_nose_gear` FIXED) | eligible if compatible | Two main `gear_{i}` parts FIXED to their engine nacelles + a `nose_gear` part FIXED to the forward fuselage (tricycle stance); each gear is its own FIXED-jointed part. Requires ≥1 wing-mounted nacelle (gated to N≥2). |

### Slot C：engine / propeller multiplicity (N axis)

| module_name | 5_star_source | model.py:Lx-Ly | sampling eligibility | 结构特征 |
|---|---|---|---|---|
| n1_centerline_engine | S9 `rec_military_aircraft_var_singleengine` | helpers L339-430; loop L721-775; stations L716-719 | eligible if compatible | `for i, (nac_y, ...) in enumerate(NACELLE_STATIONS)` with one centerline station `(0.0, NAC_FRONT_X, 0.0)`; `engine_0` FIXED to **fuselage** (`fuselage_to_engine_0`), `propeller_0` CONTINUOUS +X. Nose-prop fighter topology. |
| n2_wing_engines | S2 `rec_model-a-wwii-twin-engine-attack-bomber...` (hand-written) → loop primitive | hand-written add L715-760 (calls L759-760); copy primitive = S9/S10 loop body | eligible if compatible | Two wing-mounted nacelles at `±NAC_Y`; each `engine_{i}` FIXED to **wing** (`wing_to_engine_{i}`), `propeller_{i}` CONTINUOUS +X, spanwise-mirrored. NOTE: parents hand-write these two; the template rewrites into the `for i in range(N_ENGINE)` loop (S9/S10 primitive). |
| n4_wing_engines | S10 `rec_military_aircraft_var_fourengine` | helpers L343-434; loop L721-764; stations L48-49 (`NAC_Y_POSITIONS=[3.5,7.8,-3.5,-7.8]`) | eligible if compatible | `for i in range(N_ENGINE)` over 4 stations; each `engine_{i}` FIXED to **wing** (`wing_to_engine_{i}`), `propeller_{i}` CONTINUOUS +X. Inner+outer symmetric pairs. Bomber/heavy topology. |

(N=3 is a sampler-interpolated station list between N=2 and N=4 using the same loop body and a 3-station list `[0.0(nose-or-center), ±NAC_Y]`; see Multiplicity section. It reuses the n_wing_engines copy primitive — no new module source needed because the loop body is identical.)

**Degrade note:** No single-candidate slots. Slot A has 6 candidates, Slot B has 3, Slot C exposes 1/2/(3)/4 as distinct topology modules (≥3 distinct counts).

## 槽位图（slot graph）

pattern: mixed — parallel children parented to the fuselage spine, plus one multiplicity axis on the engine slot.

```
                          fuselage (root, grounded at CL_Z; lofted hull)
                          | FIXED               | FIXED            | parallel children
                          v                     v                  v
                        wing  ----------------- empennage(Slot A)   engine units ×N (Slot C)
                          | (carries engines    | fin/stab + movable    | each: nacelle FIXED to
                          |  for N≥2)           |  control surfaces      |  {wing if N≥2 | fuselage if N=1}
                          |                     |  (REVOLUTE hinges)     |  prop CONTINUOUS +X
                          v                                             v
              landing gear (Slot B)                              landing gear mains (tricycle)
              (taildragger: visuals on wing+fuselage;             attach to nacelles
               tricycle: parts FIXED to nacelles + fuselage)
```

Connections / interface points:

- **fuselage → wing**: FIXED at `cl = Origin(xyz=(0,0,CL_Z))`; mating = wing root carry-through buried in the belly (allow_overlap `wing_loft`↔`hull`). S1 L559-565 / S2 L712.
- **fuselage → empennage**: FIXED at `cl` for the vertical fin and (for fixed-stabilizer modules) the horizontal stabilizer; fin/stab roots seat into the tail cone + dorsal fairing (allow_overlaps). Movable surfaces hang off their fixed parent via REVOLUTE hinges with the axis/range from the chosen module.
- **empennage internal hinges**: rudder hinge axis is the module's `RUDDER_AXIS`/`HINGE_AXIS`/`TWIN_RUDDER_AXIS` (near-vertical, slight rake); elevator/stabilator axis is lateral +Y. Joint origin sits on real hinge hardware (hinge barrel / pivot shaft / torque tube).
- **{wing|fuselage} → engine_i**: FIXED nacelle mount. Parent is the **wing** when N≥2 (`wing_to_engine_{i}`, station `(NAC_FRONT_X, ±NAC_Y, NAC_DZ)`); parent is the **fuselage** when N=1 (`fuselage_to_engine_0`, centerline `(NAC_FRONT_X, 0, 0)`).
- **engine_i → propeller_i**: CONTINUOUS about +X at `(PROP_JOINT_DX,0,0)` in the nacelle frame; one independent spin joint per prop, no mimic.
- **gear ↔ host**: gear_up = none. taildragger mains/tail are FIXED visuals (wing + fuselage). tricycle mains are FIXED parts on the nacelles (requires N≥2 wing engines) + a FIXED nose_gear on the fuselage.

Mutual exclusion / derivation:

- Slot B `fixed_tricycle` is **gated to N≥2** (its mains parent to engine nacelles; with N=1 nose-centerline there are no wing nacelles). With N=1, gear ∈ {gear_up, fixed_taildragger}.
- Slot A movable-surface choices are independent of N and gear; all 6 are compatible with any engine count and any gear.

## 每槽位 Module Emits / Interfaces

### Slot A / module hinged_rudder_plus_elevator (S1)
| emits | 描述 | 来源 |
|---|---|---|
| parts | `tail_fin`(FIXED), `rudder`(movable), `horizontal_stabilizer`(FIXED), `elevator`(movable) | S1 / L567-663 |
| internal joints | `fuselage_to_tail_fin`(FIXED), `fin_to_rudder`(REVOLUTE `RUDDER_AXIS`, ±30°), `fuselage_to_stabilizer`(FIXED), `stabilizer_to_elevator`(REVOLUTE +Y, ±25°) | S1 / L589-663 |
| upstream interface | fin/stab roots FIXED to fuselage at `cl`; roots buried in tail cone (allow_overlap) | S1 / L589-595, L633-639 |
| downstream interface | rudder hinge barrel @ `RUDDER_HINGE_X0`; elevator torque tube @ `ELEV_HINGE_X` (captured-pin overlaps) | S1 / L609-624, L648-663 |

### Slot A / module fixed_tall_fin (S2)
| emits | 描述 | 来源 |
|---|---|---|
| parts | `tail_fin`(`fin_lower`/`fin_band`/`fin_upper`, FIXED), `horizontal_stabilizer`(FIXED) | S2 / L777-794 |
| internal joints | `fuselage_to_tail_fin`(FIXED), `fuselage_to_stabilizer`(FIXED) — no movable surfaces | S2 / L787, L792-794 |
| upstream interface | fin/stab roots FIXED to fuselage at `cl`; seat into dorsal fairing | S2 / L787, L834-861 |
| downstream interface | none (rigid empennage) | — |

### Slot A / module all_moving_stabilator (S3)
| emits | 描述 | 来源 |
|---|---|---|
| parts | `stabilator`(`stabilator_loft`+`pivot_shaft`, movable); retains `tail_fin`+`rudder` | S3 / L623-645 |
| internal joints | `fuselage_to_stabilator`(REVOLUTE +Y, ±20°); plus retained `fin_to_rudder`(REVOLUTE) | S3 / L633-645 |
| upstream interface | pivot shaft passes through fuselage at the stabilator station (captured-pin overlap) | S3 / L316-320, L633-645 |
| downstream interface | stabilator pitches about +Y; verified by AABB z-swing | S3 / test block |

### Slot A / module twin_fin_rudders (S4)
| emits | 描述 | 来源 |
|---|---|---|
| parts | `horizontal_stabilizer`(FIXED), `fin_{i}`(FIXED, i=0,1), `rudder_{i}`(REVOLUTE), `elevator`(movable) | S4 / L587-679 |
| internal joints | `stabilizer_to_fin_{i}`(FIXED, mirrored `±TWIN_FIN_Y`), `fin_{i}_to_rudder_{i}`(REVOLUTE `TWIN_RUDDER_AXIS`, ±28°), `stabilizer_to_elevator`(REVOLUTE +Y) | S4 / L627-679 |
| upstream interface | stabilizer FIXED to fuselage; fins FIXED at stabilizer tips (loop `for i, y_sign in enumerate((-1,1))`) | S4 / L587-600, L627-655 |
| downstream interface | each rudder hinge barrel on its fin TE (per-fin captured-pin overlap) | S4 / L289-295, L668-679 |

### Slot A / module split_rudder_off_bomber_fin (S5)
| emits | 描述 | 来源 |
|---|---|---|
| parts | fixed fin (`fin_fixed_lower`/`fin_fixed_band`/`fin_fixed_upper`), `rudder`(`rudder_lower`/`rudder_band`/`rudder_upper`/`hinge_barrels`), `horizontal_stabilizer`(FIXED) | S5 / L844-903 |
| internal joints | `fuselage_to_tail_fin`(FIXED), `fin_to_rudder`(REVOLUTE `HINGE_AXIS`, ±30°), `fuselage_to_stabilizer`(FIXED) | S5 / L862-903 |
| upstream interface | fixed fin FIXED to fuselage at `cl`; seats into dorsal fairing | S5 / L862-866 |
| downstream interface | hinge barrels along `HINGE_AXIS` captured against fixed-fin TE (allow_overlap) | S5 / L341-351, L885-903 |

### Slot A / module split_elevator_off_bomber (S6)
| emits | 描述 | 来源 |
|---|---|---|
| parts | `horizontal_stabilizer`(`stabilizer_fixed_loft`, FIXED), `elevator`(`elevator_surface`/`torque_tube`, movable), `tail_fin`(tall FIXED) | S6 / L859-893 |
| internal joints | `fuselage_to_stabilizer`(FIXED), `stabilizer_to_elevator`(REVOLUTE +Y, ±22°), `fuselage_to_tail_fin`(FIXED) | S6 / L864-893 |
| upstream interface | fixed stabilizer FIXED to fuselage; root through tail cone | S6 / L864-866 |
| downstream interface | torque tube captured in the stabilizer hinge line (allow_overlap) | S6 / L385-389, L880-893 |

### Slot B / module gear_up (S1/S2)
| emits | 描述 | 来源 |
|---|---|---|
| parts | none | S1/S2 |
| internal joints | none | — |
| upstream interface | n/a (grounding via resting blade tips at `z≈0`) | S1 / L860-869 |
| downstream interface | none | — |

### Slot B / module fixed_taildragger (S7)
| emits | 描述 | 来源 |
|---|---|---|
| parts | (visual-only) main `gear_strut_{i}`/`gear_wheel_{i}` on `wing`; `tail_gear_strut`/`tail_gear_wheel` on `fuselage` | S7 / L695-707, L726-738 |
| internal joints | none (FIXED visuals on rigid hosts) | S7 / L726-738 |
| upstream interface | main struts embed into wing underside; tail strut into aft fuselage (allow_overlap) | S7 / L726-738 |
| downstream interface | wheels rest on ground plane (stance lowers `zmin` to wheel contact) | S7 / test grounding |

### Slot B / module fixed_tricycle (S8)
| emits | 描述 | 来源 |
|---|---|---|
| parts | `gear_{i}`(i=0,1, FIXED to nacelles), `nose_gear`(FIXED to fuselage) | S8 / L837-883 |
| internal joints | `nacelle_to_gear_{i}`(FIXED, parent=nacelle), `fuselage_to_nose_gear`(FIXED) | S8 / L853-859, L877-883 |
| upstream interface | mains FIXED at the nacelle underside (gated N≥2); nose leg FIXED at forward fuselage | S8 / L853-883 |
| downstream interface | three wheels on ground plane | S8 / test grounding |

### Slot C / engine units (n1 / n2 / (n3) / n4) — shared copy primitive (S9/S10)
| emits | 描述 | 来源 |
|---|---|---|
| parts | per unit: `engine_{i}` nacelle (`nacelle_hull`/`cowl_ring`/`engine_face`/`antiglare_panel`) + `propeller_{i}` (`spinner`/`prop_shaft`/`blades`) | S9 L722-766 / S10 L722-755 |
| internal joints | `{wing|fuselage}_to_engine_{i}`(FIXED), `propeller_{i}_spin`(CONTINUOUS +X, independent, no mimic) | S9 L743-775 / S10 L738-764 |
| upstream interface | nacelle FIXED to wing at `(NAC_FRONT_X, ±NAC_Y, NAC_DZ)` (N≥2) or fuselage at centerline (N=1) | S9 L743-749 / S10 L738-744 |
| downstream interface | spinner ahead of cowl; prop disc clears fuselage flank (N≥2); resting blade tip grazes `z≈0` | S2 L956-974 / S10 tests |

## 参数范围汇总
| 参数 | 类型 | 取值范围 / 候选值 | 标称默认 | 约束类型 | 约束 / 函数 | 来源 |
|---|---|---|---|---|---|---|
| empennage_choice (Slot A) | enum | hinged_rudder_plus_elevator / fixed_tall_fin / all_moving_stabilator / twin_fin_rudders / split_rudder_off_bomber_fin / split_elevator_off_bomber | — | choice | deterministic procedural sampler | Slot A table |
| gear_choice (Slot B) | enum | gear_up / fixed_taildragger / fixed_tricycle | — | choice | sampler; `fixed_tricycle` gated by `conditional` below | Slot B table |
| engine_count (Slot C) | enum/int | n1 / n2 / n3 / n4 (N∈{1,2,3,4}) | — | choice | multiplicity sampler (weighted, see §8) | Slot C table |
| palette_style | enum | gloss_blue_fighter / bare_metal_bomber / olive_drab_warbird / raf_temperate_camo / navy_sea_blue (≥3; target 5 realistic colorways) | gloss_blue_fighter | choice | sampled per seed; maps to material set; see §palette | S1 L493-499, S2 L643-652, S7/S8 gear materials |
| (—) gear vs engine_count | constraint | fixed_tricycle ⇒ N≥2 | — | conditional | if `engine_count==n1`: drop `fixed_tricycle` from gear candidates (mains need wing nacelles); resolve before sampling gear | Slot B/C graph |
| fuselage_len_scale | float | [0.92, 1.10] | 1.0 | independent | uniform; scales `HULL_PROFILE` station x and `TAIL_X`/`NOSE_X` together (keeps fineness ratio) | S1 L58-71 / S2 L62-76 |
| wing_span_scale | float | [0.90, 1.12] | 1.0 | independent | uniform; scales `SPAN_HALF`/`HALF_SPAN` and station y in `_wing_mesh` | S1 L40,L204-216 / S2 L42,L231-245 |
| (—) span vs engine_count | constraint | span ≥ outermost nacelle station + tip margin | — | inequality | require `wing_span_scale·HALF_SPAN ≥ max(|NAC_Y_i|)·nac_y_scale + 0.6`; else re-clamp span up or nac stations in | engine placement |
| nac_y_scale | float | [0.92, 1.08] | 1.0 | conditional | only active when N≥2; scales `NAC_Y` / `NAC_Y_POSITIONS`; clamp so outer prop discs don't overlap & clear fuselage | S2 L46, S10 L48 |
| prop_radius_scale | float | [0.92, 1.08] | 1.0 | inequality | `PROP_TIP_R·scale ≤ 0.5·min adjacent NAC_Y spacing` (multi-engine) and `≤ CL_Z` (ground clearance at rest) | S1 L43 / S2 L51 |
| rudder_throw_deg | float | [22, 34] | 30 | independent | clamp to module's safe hinge range; default per module (S1/S5 30°, S4 28°) | S1 L621-623 |
| elevator_throw_deg | float | [18, 28] | 25 | independent | clamp per module (S1 25°, S6 22°, S3 stabilator 20°) | S1 L660-662 |
| cl_z (centerline height) | float | derived | 1.08–1.28 | equation | `= max(prop_radius_scale·PROP_TIP_R + blade_ground_clearance, gear_height if gear-down)`; ensures rest pose grazes ground | S1 L37 / S2 L39 |

**连续尺寸采样契约:** (1) sample `independent` masters (`fuselage_len_scale`, `wing_span_scale`, `rudder_throw_deg`, `elevator_throw_deg`); (2) derive `cl_z` (equation) from prop radius + clearance / gear height; (3) project `prop_radius_scale` and `wing_span_scale` against the inequality rows (prop-disc spacing, ground clearance, span-vs-nacelle), re-clamp or reject; (4) resolve `conditional` ranges — `nac_y_scale` only when N≥2, and `fixed_tricycle` legality from `engine_count`.

### palette_style colorways (sampled per seed; 5 realistic WWII finishes observed/grounded in the 5★ sources)

| palette_style | body | accent / control | markings | hardware | 来源 |
|---|---|---|---|---|---|
| gloss_blue_fighter | `gloss_blue`(0.16,0.34,0.62) | `marking_yellow`(0.95,0.78,0.15) tips/band | `marking_white`(0.93,0.93,0.93) "402" | `steel_gray`(0.38,0.39,0.42) hinges; `blade_black`; `engine_dark` | S1 L493-499 |
| bare_metal_bomber | `bare_metal_silver`(0.76,0.77,0.79) + `chrome_cowl`(0.86,0.88,0.91) | `olive_drab`(0.30,0.31,0.18) anti-glare; `tail_yellow`(0.94,0.78,0.15) band | `insignia_blue`(0.10,0.16,0.38)+`insignia_white`(0.94,0.94,0.94) star-and-bar; `nose_art_red`(0.75,0.10,0.10) | `dark_grey`(0.16,0.16,0.17); `gear_steel`(0.42,0.43,0.45); `tire_rubber`(0.07,0.07,0.08) | S2 L643-652, S8 gear |
| olive_drab_warbird | `olive_drab`(0.30,0.31,0.18) body | `neutral_gray` underside (0.55,0.56,0.55) | `insignia_white` star; `marking_yellow` codes | `gear_metal`(0.45,0.46,0.48); `tire_rubber`; `blade_black` | S2 olive + S7 gear (historically grounded recolor) |
| raf_temperate_camo | `raf_dark_green`(0.20,0.27,0.16) + `raf_ocean_grey`(0.40,0.43,0.46) | `sky_underside`(0.62,0.66,0.58) | `roundel_blue`(0.10,0.16,0.40)+`roundel_red`(0.66,0.12,0.16)+`roundel_white` | `steel_gray`; `tire_rubber` | grounded recolor of S1/S7 palette family |
| navy_sea_blue | `sea_blue`(0.12,0.18,0.30) gloss | `intermediate_blue`(0.30,0.40,0.52) panel | `insignia_white` star; `marking_white` codes | `engine_dark`; `gear_steel`; `tire_rubber` | grounded recolor of S1/S2 hardware sets |

(Implementation: `palette_style` selects a fixed material dict; the geometry never changes. Sampler default = `gloss_blue_fighter` for single-engine, biased toward `bare_metal_bomber`/`olive_drab_warbird` for multi-engine, but all are legal with all topologies.)

## Multiplicity / Copy Logic

**One multiplicity axis: engine/propeller count (Slot C).**

- `count_param`: `engine_count` → N ∈ {1, 2, 3, 4} (number of nacelle+propeller units).
- `N_range`: product domain `[1, 4]` (realistic piston-era multiplicity). Test偏小; sweep samples the full domain.
- sampling domain (weighted, small-N high frequency): `N=1 ≈ 45%`, `N=2 ≈ 35%`, `N=3 ≈ 8%`, `N=4 ≈ 12%` (single-engine fighters most common; twin bombers next; quad heavies rarer; N=3 rarest, interpolated). Tail (N=4) downweighted but safe by construction.
- copied object: one **engine unit** = nacelle `engine_{i}` (`nacelle_hull`+`cowl_ring`+`engine_face`+`antiglare_panel`) + **continuous** propeller `propeller_{i}` (`spinner`+`prop_shaft`+`blades`), built by one shared helper from S9/S10.
- naming: `engine_{i}` / `propeller_{i}` in the loop (the template canonicalizes the parents' bespoke `{tag}_nacelle`/`{tag}_propeller` names into the indexed scheme).
- placement: driven by a station list `NAC_STATIONS` resolved from N:
  - N=1 → `[(0.0, NAC_FRONT_X, 0.0)]` centerline nose, parent = **fuselage** (S9 L716-719).
  - N=2 → `[(+NAC_Y, NAC_FRONT_X, NAC_DZ), (-NAC_Y, ...)]`, parent = **wing** (S2 hand-written → loop).
  - N=3 → `[(0.0, nose_or_center), (+NAC_Y, wing), (-NAC_Y, wing)]` (sampler-interpolated, reuses loop body).
  - N=4 → `[3.5, 7.8, -3.5, -7.8]` inner+outer pairs, parent = **wing** (S10 L48).
- joint policy: per unit, `{wing|fuselage}_to_engine_{i}` FIXED nacelle mount + `propeller_{i}_spin` CONTINUOUS about nacelle +X; each prop an **independent** child link (no mimic) — verified by S2's "props independent" / "right prop unaffected" checks (S2 L916-920, L986-990).
- source/gating: the copy primitive is the S9/S10 `for i in range(N)` loop body. **The bomber HAND-WRITES its 2 engines** (S2 L715-760, two `add_nacelle_and_prop` calls); the template must adopt the loop-rewrite (S9/S10) as the copy primitive, NOT the parent's two literal calls. Gating: N=1 forbids `fixed_tricycle` gear (no wing nacelles for mains).

(Per-prop blade count — 4-blade fighter vs 3-blade bomber — is a per-unit detail of the copied propeller, NOT a separate axis; it can be tied to N or palette_style but is not module multiplicity.)

## 拓扑多样性审计

总组合数：Slot A (6) × Slot B (3) × Slot C (4 distinct N) = **72** raw combos.
Minus gating: N=1 forbids `fixed_tricycle` → remove (6 × 1 × 1) = 6 illegal combos ⇒ **66 legal topology combos** (engine-count is in `slot_choices`, so each N is a distinct tuple). `palette_style` (5) is cosmetic and does NOT change topology, so it is excluded from the count.

理由：66 legal slot-choice tuples ≫ 10; even a 20-seed sweep with weighted sampling comfortably hits ≥10 distinct (A and C alone give 6×4−gating = 22). N is encoded in `slot_choices_for_seed`, so each engine count is a distinct topology class.

seed_domain_policy：procedural_first

Procedural Sampling / Sweep Plan：`config_from_seed(seed)` seeds a `random.Random(seed)`, then (1) weighted-draws `engine_count` (N) from the §8 distribution; (2) resolves the legal gear candidate set from N (drop `fixed_tricycle` if N=1) and draws `gear_choice`; (3) draws `empennage_choice` uniformly over all 6 (independent of N/gear); (4) draws `palette_style` (biased by N but all legal); (5) samples continuous scales per the sampling契约 and clamps via the inequality/conditional rows in `resolve_config`. `slot_choices_for_seed` returns `[("empennage", A), ("gear", B), ("engine_count", f"n{N}")]`. seed=0 is not special. No curated/modulo table.

Topology target：1000-seed distinct expected ≈ 60–66 (full legal-combo coverage), well above the soft ≥300 only where the category supports it — here the legal ceiling is 66, which is the category's structural cap (documented; further diversity would require inventing unsourced structure). The ≥300 guideline is bounded by real sources; 66 is the honest maximum.（统一口径：富类别建议 ≥300；低于 300 记录真实组合空间或兼容约束原因；report-only，不设门。）

若使用 regression overrides：none planned for the initial template; add only if a specific seed reproduces a clamp/clearance regression (document seed + reason in template comments).

Controlled local parameterization：`fuselage_len_scale` [0.92,1.10], `wing_span_scale` [0.90,1.12], `nac_y_scale` [0.92,1.08] (N≥2 only), `prop_radius_scale` [0.92,1.08], `rudder_throw_deg`/`elevator_throw_deg` (clamped per module), derived `cl_z`. All clamped/derived in `resolve_config`; none change topology, InterfaceSpec mating faces, or N. Cross-part dependencies declared as `equation`/`inequality`/`conditional` (§7): prop radius ↔ nacelle spacing & ground clearance; span ↔ outer nacelle station; cl_z ↔ prop radius + gear.

| item | policy | validator / viewer focus |
|---|---|---|
| sampler | N weighted-draw → gear (gated) → empennage (uniform) → palette → scales | `slot_choices_for_seed` matches build choices; N tuple present |
| compatibility matrix | `fixed_tricycle`⇒N≥2 (mutually exclusive with n1); all empennage compatible with all N/gear; palette cosmetic | no floating gear, no prop-disc overlap, span clears outer nacelles, hinge axis/range valid |
| controlled local variation | scales clamped/derived; prop radius & span gated by inequalities | proportions vary without breaking mating, clearance, prop spacing, ground graze, or category identity |
| regression overrides | none / add only for a reproduced clamp regression | previously failed or reviewer-selected cases only |
| random sweep | seeds 0-49 initial pass; 0-999 maturity audit |; contract/clearance failures |

| slot | candidate_count | 是否 ≥2 | 是否 ≥3 | 备注 |
|---|---:|---|---|---|
| A (empennage) | 6 | yes | yes | 4 movable-surface families + 2 rigid/fixed |
| B (gear) | 3 | yes | yes | gear_up / taildragger / tricycle (tricycle gated N≥2) |
| C (engine_count) | 4 (N=1,2,3,4) | yes | yes | multiplicity axis; n3 interpolated from shared loop body |

## Validator

- `slot_choices_for_seed` returns implemented module names `[("empennage", ...), ("gear", ...), ("engine_count", "n{N}")]`.
- `config_from_seed` uses deterministic procedural sampling for all ordinary seeds (seed=0 not special).
- compatibility matrix prevents illegal combinations: `fixed_tricycle` only with N≥2; engine parent = wing for N≥2, fuselage for N=1.
- continuous scale params clamped/derived in `resolve_config` (prop radius ↔ spacing/ground; span ↔ outer nacelle; cl_z ↔ prop+gear); never break interfaces, clearance, joint origin, or category multiplicity.
- critical interfaces exist: fuselage→wing FIXED, fuselage→empennage FIXED, {wing|fuselage}→engine FIXED, engine→prop CONTINUOUS +X.
- key joints: prop spin CONTINUOUS about +X with no limits and no mimic (independent); rudder REVOLUTE near-vertical (axis z>0.95) within throw; elevator/stabilator REVOLUTE +Y within throw.
- copied engine units follow `engine_{i}`/`propeller_{i}` naming and station placement; props are independent links.
- captured-pin overlaps (hinge barrels, torque tube, pivot shaft, prop shaft in cowl bore, gear embeds) declared element-scoped, not broad.
- rest pose grazes ground (gear-up: blade tips; gear-down: wheels) — `zmin ∈ [-0.01, 0.12]`.

## Reject cases

- Sampling `fixed_tricycle` with N=1 (no wing nacelles for the mains) → floating/unsupported main gear.
- Hand-copying the bomber's two literal `add_nacelle_and_prop` calls instead of the `for i in range(N)` loop → can't reach N=1/3/4; multiplicity axis collapses.
- Propeller joint made REVOLUTE-with-limits or given a `mimic` instead of independent CONTINUOUS +X → fails spin/independence checks.
- Multi-engine with `prop_radius_scale` too large → adjacent prop discs overlap or props strike the ground at rest (inequality not enforced).
- Rudder hinge placed on +Y lateral axis (copying elevator) or elevator on the near-vertical axis → wrong control-surface kinematics.
- Movable surface emitted with no real hinge hardware at the joint origin (barrel/shaft/torque tube absent) → articulation origin >15 mm from geometry.
- Wing span not scaled with outer nacelle stations (N=4) → outer nacelle/prop hangs off the wingtip or floats.
- `palette_style` treated as a topology slot (counting it in diversity) → inflates diversity with cosmetic-only variation.

## 与相邻类别的边界

- 不该混入：**Jet aircraft** — no propeller, thrust is internal/turbine; the CONTINUOUS +X prop spin and nacelle+spinner identity are core here and absent on jets.
- 不该混入：**Helicopter / rotorcraft** — main-rotor + tail-rotor articulation on a mast, no fixed wing; this template's lift is a one-piece fixed wing with a fin/stab empennage.
- 不该混入：**Civil airliner / transport** — no military empennage/insignia identity, different fuselage fineness and engine layout; markings here (star-and-bar, RAF roundel, codes) and warbird proportions are identity, not decoration.
- 不该混入：**Drone / UAV** — no piloted canopy, different scale and multiplicity logic.

## 审核记录
| 项 | 结论 |
|---|---|
| reviewer status | pending |
| reviewer notes | — |

## 模板实现备注（可选）

- Shared helpers: one `_build_engine_unit(model, i, station, parent, palette)` consumed by the N loop (from S9/S10); one airfoil-loft helper family shared by wing/fin/stabilizer/elevator/rudder (parents use `_foil_pairs*`).
- The engine copy primitive is the **loop-rewrite (S9/S10)**, not the bomber's two literal calls — this is the module contract for the multiplicity axis.
- Element-scoped allow_overlap required for: wing↔hull carry-through, fin/stab roots↔tail cone & dorsal fairing, rudder hinge barrel↔fin TE, elevator torque tube↔stabilizer hinge line, stabilator pivot shaft↔fuselage, prop_shaft↔engine_face/cowl_ring/hull bore, nacelle_hull↔wing, antiglare_panel↔wing, gear struts↔wing/nacelle/fuselage embeds.
- `fin_to_rudder` / `stabilizer_to_elevator` mating barrels/tubes are captured-pin: grandfather (omit `mating=`) where two axis-aligned faces don't fit; keep allow_overlaps local.
- Combinations not yet in seed domain: none excluded beyond the N=1×tricycle gate; canopy articulation and underwing stores are out of scope (see source map 排除项 — no isolating source images).
- Module source index is the Slot tables above (slug-prefixed record ids); no separate index needed.
