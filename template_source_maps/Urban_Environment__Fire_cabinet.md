# Urban_Environment / Fire cabinet — template source map

pattern: parallel_children(carcass shell + closure 主机构 [multiplicity N] + base/support + front-face style)

## Chosen identity

PICDIR `picture/Urban Environment/Fire cabinet/001.png` shows a **tall dark-charcoal sheet-steel
four-drawer metal filing/utility cabinet** (upright box housing, four stacked sliding drawers, each
with a recessed pull + brass card-label holder, recessed base plinth, small red maker badge). Despite
the "fire cabinet" label it is NOT a glazed fire-hose box; it is a **STREET/UTILITY upright metal
cabinet with real openable closures**. Identity for the whole fork = upright metal cabinet whose
front opens via real non-fixed joints (drawers PRISMATIC, doors REVOLUTE, shutter PRISMATIC).

## Parent

- `rec_tall-four-drawer-metal-filing-cabinet-in-dark-ch_20260608_171125_602483_053b2c58`
  ← `picture/Urban Environment/Fire cabinet/001.png`
  (baseline = closure:**4 sliding drawers PRISMATIC** × base:**recessed plinth** × front:**solid steel face frame**;
  carcass = hollow shell: back wall + 2 side walls + top + bottom + front face-frame rails/stiles.)
  Single parent for the subcategory → variants carry all the structural candidates.

## Readability / loop emission (parent audit)

- Drawers: **loop-emitted** `for i in range(n_drawers)` with `name_i`-style parts (`drawer_{i}`), uniform PRISMATIC joint policy `cabinet_to_drawer_{i}`. ✓
- Front face rails: **loop-emitted** `for i in range(n_drawers + 1)` (`face_rail_{i}`). ✓
- Face stiles, carcass side walls, drawer side walls, slide runners: per-side `for s,tag in ((1,..),(-1,..))` loops. ✓
- Hand-written singletons (acceptable — fixed carcass, not a multiplicity): `base_plinth`, `bottom_panel`,
  `top_panel`, `back_wall`, `top_badge`. No hand-written drawer/shelf repeats to refactor.
- Variant guidance: keep drawer/shelf/door-panel/shutter-slat/louver-blade emission in a `for-i-in-range(n)`
  loop with a shared geometry helper; do not unroll.

## Combo pre-audit (HARD GATE)

closure-mechanism slot = **4** candidates (N-drawers PRISMATIC / single hinged door REVOLUTE /
double doors REVOLUTE / roller shutter PRISMATIC) × distinct multiplicity-N = **3**
({3,4,5} drawers; {1,2} doors; {2,3} shelves) → **4 × 3 = 12 ≥ 10 ✓ PASS**.
(Front-face style 3 × base 3 push the full product far above the floor; closure × N alone clears it.)

## Slot candidate coverage

### Slot A — closure_mechanism (主机构槽 — how the cabinet front opens; the real joint)
| candidate (future module) | variant | key joint / structure | status |
|---|---|---|---|
| n_sliding_drawers (baseline) | parent / drawers3 / drawers5 | N open-top trays pull +X on runners, **PRISMATIC** | parent + planned |
| single_hinged_door | hinged_door | full-front door, vertical-axis **REVOLUTE** hinge (left edge) | converged |
| double_doors | double_doors | center-seam pair, 2× vertical-axis **REVOLUTE** | converged |
| roller_shutter | roller_shutter | slat stack lifts vertically in side channels, **PRISMATIC** | converged |

### Slot B — closure_multiplicity_N (2–3 distinct N)
| N | variant | structure | status |
|---|---|---|---|
| 3 drawers | drawers3 | three equal stacked drawers | converged |
| 4 drawers (baseline) | parent / legs / casters | four equal stacked drawers | parent + planned |
| 5 drawers | drawers5 | five equal stacked drawers (denser) | converged |
| 1 door / 2–3 shelves | hinged_door / glazed_door / louvered_door | single door + 2 fixed shelves | converged |
| 2 doors / 3 shelves | double_doors | double doors + 3 fixed shelves | converged |

### Slot C — front_face_style (closure surface; glazing/venting)
| candidate | variant | structure | status |
|---|---|---|---|
| solid_steel (baseline) | parent / hinged_door / double_doors | opaque steel face / door panel | parent + planned |
| glazed_window | glazed_door | door central panel = transparent glazed pane in steel frame | converged |
| louvered | louvered_door | door face = regular stack of angled vent blades (loop) | converged |

### Slot D — base_support (how the carcass meets the floor)
| candidate | variant | structure | status |
|---|---|---|---|
| recessed_plinth (baseline) | parent (+ all door/drawer variants default) | recessed kick plinth | parent |
| steel_legs | legs | 4 straight corner legs lifting the body | converged |
| casters | casters | 4 swivel caster wheels under bottom corners | converged |

## Variants (9 new; cap ~8–10)

| record_id | label | axis | prompt file |
|---|---|---|---|
| rec_fire_cabinet_var_hinged_door | fire_cabinet-hinged_door | A: single hinged door REVOLUTE | /tmp/urb_fire_cabinet_var_hinged_door.txt |
| rec_fire_cabinet_var_double_doors | fire_cabinet-double_doors | A: double doors 2× REVOLUTE | /tmp/urb_fire_cabinet_var_double_doors.txt |
| rec_fire_cabinet_var_roller_shutter | fire_cabinet-roller_shutter | A: roller shutter PRISMATIC-vertical | /tmp/urb_fire_cabinet_var_roller_shutter.txt |
| rec_fire_cabinet_var_drawers3 | fire_cabinet-drawers3 | B: N=3 drawers | /tmp/urb_fire_cabinet_var_drawers3.txt |
| rec_fire_cabinet_var_drawers5 | fire_cabinet-drawers5 | B: N=5 drawers | /tmp/urb_fire_cabinet_var_drawers5.txt |
| rec_fire_cabinet_var_glazed_door | fire_cabinet-glazed_door | C: glazed window door | /tmp/urb_fire_cabinet_var_glazed_door.txt |
| rec_fire_cabinet_var_louvered_door | fire_cabinet-louvered_door | C: louvered door face | /tmp/urb_fire_cabinet_var_louvered_door.txt |
| rec_fire_cabinet_var_legs | fire_cabinet-legs | D: steel legs | /tmp/urb_fire_cabinet_var_legs.txt |
| rec_fire_cabinet_var_casters | fire_cabinet-casters | D: swivel casters | /tmp/urb_fire_cabinet_var_casters.txt |

manifest: `/tmp/manifest_urb_fire_cabinet.tsv` (TAB, 4 fields, no header, 9 rows)

## Dropped / out-of-scope axes

- **color / material / pure-scale** — forbidden as the change axis (allowed only as incidental dressing).
- **drawer-handle style, label-holder style, maker-badge** — cosmetic greebles, not structural; left as parent dressing.
- **internal organizers (dividers/file rails)** — non-articulated interior detail; not a distinct closure axis here.
- **wall-mount vs free-standing** — would break the upright free-standing footprint identity; dropped.
