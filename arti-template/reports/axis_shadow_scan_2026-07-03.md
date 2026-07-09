# Joint-origin gate SHADOW study: point distance vs axis-line distance (2026-07-03)

**Status: report-only. The live gate (`find_joint_origin_distance_findings`, flat 0.015 m + 0.05×bbox-diag relative tol) is untouched.** This study adds a shadow metric and measures, over real template builds, what would change if the gate measured distance from the joint **axis line segment** to parent/child geometry instead of from the joint **origin point**.

Raw data:

- `.articraft/axis_shadow_m2list.jsonl` — the 26-template M2 triage list (from the 2026-07-02 origin-recheck), seeds 0–5.
- `.articraft/axis_shadow_full.jsonl` — the full `TEMPLATE_REGISTRY` (289 slugs), seeds 0–5.
- Scanner: `scripts/origin_axis_shadow_scan.py` (build-only, resumable JSONL streaming; no sweep state written).
- Shadow metric: `measure_joint_axis_distances` / `find_joint_axis_distance_findings` in `sdk/_core/v0/geometry_qc.py` (additive; not wired into any gate).
- Unit tests: `tests/sdk/test_joint_axis_distance.py` (phantom mid-air pivot fails; axial-offset prismatic passes; axle-through-wheel passes; FIXED falls back to point distance).

## 1. Implementation notes

**Probe = thin FCL capsule, not point sampling.** `python-fcl` in this venv supports `fcl.Capsule` narrow-phase distance queries against every collision backend we use (Box/Cylinder/Sphere/BVH mesh — verified empirically before committing to it). The axis probe is a capsule of radius 1e-6 m, so the reported value is the segment→geometry distance minus 1e-6 (negligible), computed in ONE narrow-phase query per collision entry instead of N sample points. Entries are pre-culled with the existing `_aabb_min_distance` lower bound (visit entries in ascending AABB-gap order, stop when the bound exceeds the best exact distance so far).

**Segment design (bounded, deliberately).** An infinite axis line would spuriously graze distant geometry of the same part and make the metric vacuous on large assemblies. The probe segment is centered at the joint origin and spans `± half_length` where

```
half_length = max(1 mm, 0.5 × max(parent_bbox_extent_along_axis, child_bbox_extent_along_axis))
```

i.e. half the part-pair's larger AABB projection width onto the axis direction. In the CHILD frame the line passes through `(0,0,0)` along `joint.axis`; in the PARENT frame through `origin.xyz` along `R(origin.rpy)·axis` — the same physical line, expressed in the frame each part's collision entries already live in (`_compiled_part_collision_entries` with no part_tf). Consequences:

- an axial offset up to the pair's own size is legal (prismatic slide origins, axles through wheels) — the intended win;
- a mid-air phantom pivot still fails: its axis passes through neither part anywhere near the origin;
- an axial offset *larger than the pair's own size* still fails (nothing is silently legalized without bound);
- grazing risk is bounded: the check stays per parent/child part (never against unrelated parts), and the segment cannot extend beyond the pair's own scale.

**FIXED/FLOATING joints keep point semantics** (no meaningful axis): their "axis" distances duplicate the point distances, `axis_based=false`.

**Old verdict definition.** Recomputed with the canonical gate defaults (`tol=0.015`, `bbox_relative=0.05`). Templates that pass today's sweep with a *relaxed per-template tol* in `run_tests` therefore show up here as `old_fail` — intentional: the study compares metric semantics, not per-template waivers.

**Scan robustness note.** `ProcessPoolExecutor(max_tasks_per_child=…)` deadlocked once all workers retired simultaneously (stalled at exactly workers×tasks); the scanner now recreates a fresh spawn-context pool per batch instead.

## 2a. Scan summary — 26-template M2 list (seeds 0–5)

154/156 seeds ok (2 transient `cctv_mast` temp-mesh races). 701 joint-measurements: revolute 301, prismatic 187, continuous 124, fixed 89.

Disagreement by joint type at the 15 mm flat axis threshold (measurement-level):

| type | total | 误杀 (old pass → new fail) | 漏杀 (old fail → new pass) | both fail |
|---|---|---|---|---|
| revolute | 301 | **0** | **121** | 10 |
| continuous | 124 | **0** | 4 | 7 |
| prismatic | 187 | **103** | 1 | 38 |
| fixed | 89 | 4 | 0 | 29 |

## 2b. Scan summary — full registry (289 templates, seeds 0–5)

- 1598/1734 seeds ok; **all 136 failures are "Mesh file not found"** on mesh-asset templates built directly (the template's temp `AssetContext` meshes are gone by QC-compile time — a direct-build artifact, these templates compile fine through the normal `model.py` sweep path). 20 templates have no data (e.g. `Bathroom_Hair_dryer`, `Equipment_Lock`, `Others_Binocular`, `camera_lens`), 5 partial. Coverage: 269/289 templates measured.
- **7935 joint-measurements**: revolute 3651, prismatic 2095, continuous 1230, fixed 959.

Disagreement by joint type @15 mm flat (measurement-level):

| type | total | 误杀 (old pass → new fail) | 漏杀 (old fail → new pass) | both fail |
|---|---|---|---|---|
| revolute | 3651 | **3** (1 unique joint) | **143** | 10 |
| continuous | 1230 | **0** | 10 | 9 |
| prismatic | 2095 | **105** | 10 | 39 |
| fixed | 959 | 12 | 0 | 35 |

The full registry confirms the M2-list pattern at 5× the joint count: for **revolute/continuous the axis metric is essentially a strict improvement** (one unique new-failing joint library-wide, §4), while a **flat-threshold axis metric on prismatic re-flags exactly the hollow-cavity cases the relative tol existed for**, and fixed joints only ever lose the relative allowance.

## 3. Diff leaderboard (most old↔new disagreement @15 mm, unique joints × disagreeing seeds)

| template | disagreeing joint-seeds | direction | dominant joint type | what it is |
|---|---|---|---|---|
| louvered_shutter_assembly | 77 | 漏杀 ×26 joints | revolute | louver pivots: origin at slat end, axis runs the slat through both frame stiles (pt up to 0.45 m, ax = 0) |
| desk_with_drawer_card_catalog | 70 | 误杀 ×29 joints | prismatic | drawer slides: origin centered on the bay opening, axis through the hollow bay, walls 71 mm away |
| desktop_pc_tower | 22 | 误杀 ×6 | prismatic | drive-tray slides: axis through the empty tray bay (35 mm to chassis) |
| cantilever_articulated_arm | 16 | 漏杀 ×3 | revolute | elbow/shoulder/intermediate hinges: axis through the arm knuckles (pt 0.11–0.16, ax = 0) |
| Headwear_Racing_helmet | 11 | 漏杀 ×3 | revolute | visor/chin-bar pivots: axis through the shell bosses (pt 0.02–0.04, ax = 0) |
| Military_Aircraft | 11 | mixed | fixed 误杀 ×2, revolute 漏杀 ×1 | fixed empennage mounts lose the (0.42 m!) relative tol; elevator hinge axis passes through the stabilizer |
| serial_elbow_arm | 9 | 漏杀 ×2 | revolute | pitch hinges (pt 0.03–0.04, ax = 0) |
| cannon | 7 | mixed | revolute 漏杀 + prismatic 误杀 | trunnion pt 0.35 → ax 0; wedge_slide 20 mm |
| telescoping_boom | 7 | mixed | prismatic | hollow stages: some axes centered in the bore (误杀 at flat 15), one stage legalized (漏杀) |
| Powertools_drill | 6 | 漏杀 ×1 | continuous | chuck spin axis through the housing (pt 0.027, ax = 0) |
| simple_aframe_step_ladder | 6 | 漏杀 ×1 | revolute | frame fold hinge: child frame geometry 0.18 m below origin, axis through it |
| screwin_light_bulb_with_socket | 6 | 漏杀 ×2 | revolute/continuous | screw axes down the socket bore (ax 0.001–0.006) |
| barrier_gate_leaf_gate | 5 | 误杀 ×3 | prismatic | sliding panel / counterweight axes 23–65 mm from geometry (rel tol up to 0.31 m today) |
| refrigerator_with_hinged_doors | 5 | 漏杀 ×2 | prismatic | crisper-drawer slides: origin 0.23 m from fridge body along slide, axis passes through it |
| rolling_toolbox_with_telescoping_handle | 4 | 漏杀 ×1 | prismatic | telescoping-handle stage: pure axial offset (pt_child 0.083 → ax 0) |
| cctv_mast_with_pantilt_camera_head | 4 | 漏杀 ×1 | revolute | tilt axis through the pan head |
| Urban_Environment_Phone_box | 3 | **误杀 ×1 (the only revolute 误杀)** | revolute | bifold fold hinge floats 22.5 mm off the outer leaf — see §4 |
| Container_Locker | 2 | 误杀 ×2 | prismatic | locker slides, axis 40 mm from bay walls (rel tol 0.099 today) |
| screwcap_bottle / bicycle_crankset / camera_flash / platform_cart / wheelie_bin / lighthouse | ≤3 each | mostly 漏杀 | revolute/continuous/fixed | small hinges & screw axes; two fixed mounts lose relative tol |

## 4. 误杀 analysis (would-fail under flat-15 mm axis metric, passes today) — 49 unique joints, 10 templates

Eyeball classification from the template geometry:

**(a) Prismatic, axis through a hollow bay/bore — 44 joints, metric artifact, not defects.** `desk_with_drawer_card_catalog` (29 slides: the joint origin is *literally documented in the template as "centred on the front face of the opening"*, axis −Y through the empty drawer bay, nearest desk panel 71 mm), `desktop_pc_tower` (6 tray slides, 27–35 mm), `Container_Locker` (2, 40 mm), `barrier_gate_leaf_gate` (3, 23–65 mm), `telescoping_boom` (2 hollow stages, 53–69 mm), `cannon::wedge_slide` (20 mm; passes at flat 20). These are *correct* engineering — a slide axis centered in the cavity it slides through. Two aggravating factors: (1) for prismatic joints the axis is a free *direction*; the line's lateral position is physically meaningless, so penalizing lateral standoff adds no semantic value; (2) these are precisely the "origin at the hollow cross-section center" cases the relative tol was introduced for. **A flat threshold cannot serve prismatic.** With the relative allowance retained for prismatic (axis distance vs `max(0.015, 0.05×diag)`), every one of these 44 passes — see hybrid in §7.

**(b) FIXED joints losing the relative allowance — 4 joints, benign.** `wheelie_bin::body_to_axle` (26 mm: origin on the axle centerline below the body floor), `lighthouse::tower_to_central_shaft` (23 mm), `telescoping_boom::inner_stage_to_end_effector` (53 mm), `Military_Aircraft::fuselage_to_stabilizer`/`tail_fin` (25–52 mm — note today's effective tol there is 0.42 m, diag-driven). These are unchanged-semantics joints; they only fail because the candidate dropped the relative term. No reason to change FIXED handling at all.

**(c) Revolute — exactly 1 unique joint library-wide, and it looks like a genuine forgiven defect.** `Urban_Environment_Phone_box::outer_to_inner_leaf`: the bifold fold hinge origin is `(-fold_offset, -(leaf_w+seam_gap), 0)` in the outer-leaf frame; the vertical fold axis stands 22.5 mm clear of the outer leaf's geometry with no knuckle/hinge hardware bridging it (today forgiven by the 0.099 m relative tol of the big kiosk bbox). Physically the two leaves fold about a line in mid-air 22 mm off the parent leaf — exactly the class of defect the axis metric is designed to expose. Even if judged an intentional clearance trick, one joint across 269 measured templates is trivially fixable or waivable.

## 5. 漏杀 analysis (fails today's default gate, passes under axis metric) — 50 unique joints, 17 templates

All 50 have axis distance ≤ 15 mm on *both* sides, and 47/50 have axis distance ≤ 6 mm (mostly exactly 0.0): the axis line passes straight **through** the hinge hardware. Classification:

**Legit-axial (the intended win) — 50/50; no suspicious grazing found.**

- **Pivot-at-slat/arm-end revolutes (39):** `louvered_shutter_assembly` ×26 (origin at the slat end, pt up to 0.45 m — the axis runs the slat length and passes through both frame stiles, i.e. the actual hinge line), `cantilever_articulated_arm` ×3, `serial_elbow_arm` ×2, `Headwear_Racing_helmet` ×3 (visor/chin-bar pivots), `cannon::barrel_elevation` (trunnion, pt 0.35 m → ax 0), `cctv_mast::tilt_joint`, `camera_flash::neck_to_head_tilt`, `simple_aframe_step_ladder::frame_fold_joint`, `platform_cart::handle_joint`, `Military_Aircraft::stabilizer_to_elevator` (ax 12 mm — hinge slightly inboard of the stabilizer skin).
- **Screw/spin axes (6 continuous+revolute):** `screwin_light_bulb` ×2, `screwcap_bottle::cap_spin` (borderline, ax 15 mm — cap wall radius), `bicycle_crankset` pedal spins ×2 (the M2 axle-through-pedal case), `Powertools_drill::housing_to_chuck`.
- **Prismatic pure axial offsets (5):** `refrigerator_with_hinged_doors` crisper drawers ×2 (pt_parent 0.23 m along the slide → ax 0), `rolling_toolbox` handle stage, `telescoping_boom::middle_stage_2_to_inner_stage`, plus one `barrier_gate_leaf_gate` seed-variant.

Grazing audit: because the metric is evaluated per parent/child part only, the worst "accidental" contact found is a prismatic axis touching the *cabinet back wall* rather than slide rails (refrigerator drawers) — benign, since for prismatic the axis position is physically meaningless anyway, and the verdict (legal) matches the physical truth (a drawer that slides correctly). No case was found where the axis passes through unrelated geometry while genuinely missing the hinge hardware.

## 6. Threshold table (full registry; unique (slug,joint) failing on ≥1 seed / templates with ≥1 failing joint)

| gate | failing joints | failing templates | 误杀 joints (tpls) vs today | 漏杀 joints (tpls) vs today |
|---|---|---|---|---|
| today: point, 0.015 + 0.05×diag | 85 | 23 | — | — |
| axis flat **10 mm** | 146 | 49 | 119 (43) | 49 (16) |
| axis flat **15 mm** | 76 | 18 | 49 (10) | 50 (17) |
| axis flat **20 mm** | 72 | 18 | 48 (9) | 53 (17) |
| **hybrid (§7)** | **37** | **12** | **1 (1)** | **50 (17)** |

10 mm is clearly too tight (the 误杀 count triples, pulling in 33 more templates, mostly prismatic + fixed). 15→20 mm barely moves either count: the metric is not threshold-sensitive in this range — the residual 误杀 at flat 15/20 are structural (prismatic hollow bores + fixed joints losing the relative term), not near-threshold noise.

## 7. Recommendation: **hybrid — axis metric with flat 15 mm for revolute/continuous; axis metric with the relative allowance for prismatic; FIXED untouched**

Evaluated explicitly on the full-registry data:

- **revolute/continuous: axis segment, flat 15 mm, no relative term.** Evidence: 0 continuous and 1 unique revolute new-failure library-wide (and that one, `Phone_box::outer_to_inner_leaf`, looks like a real forgiven defect — a fold axis floating 22 mm off its leaf); 143+10 measurements of legitimate axles/hinges currently forgiven only by the relative fudge become legal on the merits (axis through the hardware), including every "saved by relative tol" case from the 2026-07-02 M2 cap scan. For these types the relative term can be **retired outright** — no more meter-scale effective tolerances on big structures (`crane_tower`'s current effective tol is 1.27 m = 0.05×25 m diag; the axis metric measures 0.0 for the same joints).
- **prismatic: axis segment, but keep `max(0.015, 0.05×diag)`.** A flat threshold is wrong for prismatic in principle (the axis is a free direction; its lateral position is meaningless) and in practice (44 hollow-bay 误杀). With the relative allowance applied to the *axis* distance instead of the point distance, prismatic gets the axial-offset wins (refrigerator drawers at 0.23 m, toolbox handle, telescoping stage) with **zero** new failures on the full registry.
- **fixed/floating: keep today's point + relative semantics unchanged.** No axis exists; all 12 fixed "误杀" at flat thresholds were artifacts of dropping the relative term.

Net effect of the hybrid vs today, full registry: failing joints 85 → 37, failing templates 23 → 12, **误杀 = 1 joint** (worth a one-off look regardless), **漏杀 = 50 joints across 17 templates, all classified legit-axial**. Threshold choice within {15, 20} mm is immaterial for the rev/continuous leg (identical 误杀 either way); 15 mm is the tighter defensible choice.

Do **not** switch to a pure flat-threshold axis gate for all types: the prismatic column of §2b is the counter-evidence (105 new-failing measurements, all correct hollow-bore slides).

Suggested path (not executed — out of scope for this shadow study): wire `find_joint_axis_distance_findings` behind a `warn_only` harness check first, burn it in on new-template sweeps, then swap the revolute/continuous leg of the live gate and shrink the prismatic relative term later if the axis data supports it.

## Caveats

- 20 mesh-asset templates (+5 partial) have no measurements (direct-build temp-asset race, §2b); their joints are unscanned. The pattern across 269 templates and 7935 measurements is uniform enough that this is unlikely to change the type-level conclusion; re-scan via the compile-report path if completeness is needed.
- "Old verdict" uses gate defaults; per-template relaxed tolerances in `run_tests` are deliberately ignored.
- Multi-component mesh parts are measured per component (same as the live gate's collision entries), so a mesh split into islands cannot hide an axis miss.
