# Urban_Environment / bucket2 — template source map

pattern: surface_of_revolution_vessel (staved wooden keg/barrel) + named-slot main mechanism (closure) + N-multiplicity hoop array + optional carry-handle joint

## Identity

Small wooden staved keg/barrel. Bulged stave body built as a surface of revolution about +Z with carved vertical stave seam grooves, a solid wooden floor, two dark metal hoop bands wrapping the body, a recessed inner seating ledge at the mouth, and a circular wooden lid with a blue arched grip handle. The REAL joint is the closure (lid slides off / hinges / bung) or a carry handle (swing bail / side rings). Variants stay wooden kegs/barrels/pails.

## Parent

- rec_small-wooden-keg-barrel-with-staved-bulged-body-_20260608_164508_499567_ad0a147f ← picture/Urban Environment/bucket2/001.png
  - baseline = body:bulged_barrel × hoop_N:2 (upper+lower) × closure:slide_off_lid(PRISMATIC +Z) × handle:lid_arched_grip(FIXED on lid)
  - model.py path: data/records/<parent>/revisions/rev_000001/model.py

## Loop / readability audit (drives the multiplicity-variant rewrite request)

- STAVES: ALREADY loop-emitted — `for i in range(N_STAVES)` (line ~103) carves the vertical seam grooves. BUT the staves are fused into one `barrel_body` shell mesh (grooves cut from a single revolved solid), not separate `stave_{i}` parts. Acceptable as-is; no separate stave-count part axis planned (staves are cosmetic grooves on one body, not jointed sub-parts).
- HOOPS: NOT loop-emitted — HAND-WRITTEN REPEATS. Two duplicate `_build_hoop_mesh(name, z_center)` calls produce `upper_hoop` / `lower_hoop` parts, each with its own `Inertial.from_geometry` block and its own duplicated FIXED joint (`body_to_upper_hoop`, `body_to_lower_hoop`). The hoop-count multiplicity variant (`hoop_count`) MUST request the loop rewrite to `hoop_0..hoop_{N-1}` over a shared helper with a uniform FIXED joint policy and regular z-pitch.
- HANDLE: single hand-written arched grip (legs + crossbar union), FIXED to lid. Fine as-is.

## 组合数预审 (HARD GATE)

Slots × candidates: body(3) × hoop_N(3 distinct N) × closure(3) × handle(3) = **81 ≥ 10 ✓**.
Even the two structural-topology slots alone clear it: closure(3) × hoop_N(3) = 9, plus body(3) → 27 ≥ 10. Distinct-N = 3 (N ∈ {2,3,4}) on the hoop array.

## Slot 候选覆盖 (each slot ≥2 candidates)

### Slot A: body_profile (vessel form; structural shape, not pure scale)
| 候选 (future module) | variant | structure | status |
|---|---|---|---|
| bulged_barrel (baseline) | parent | symmetric parabolic mid-bulge stave body | parent (existing) |
| straight_keg | rec_bucket2_var_straight_keg | constant-radius cylinder, vertical staves, no belly | converged |
| tapered_pail | rec_bucket2_var_tapered_pail | conical taper, wide mouth → narrow base | converged |

### Slot B: hoop_count_N (multiplicity — REQUIRES loop rewrite to hoop_{i})
| 候选 | variant | structure | status |
|---|---|---|---|
| N=2 (baseline) | parent | hand-written upper_hoop + lower_hoop (2 FIXED joints) | parent (existing) |
| N parametric {2,3,4} | rec_bucket2_var_hoop_count | `for i in range(N)` → hoop_0..hoop_{N-1}, shared helper, regular z-pitch, uniform FIXED policy | converged |

distinct-N = 3 (2 / 3 / 4).

### Slot C: closure (main mechanism — the keg's open/close action; keep ≥1 non-fixed)
| 候选 | variant | key joint / structure | status |
|---|---|---|---|
| slide_off_lid (baseline) | parent | wooden lid slides straight up off mouth — PRISMATIC +Z | parent (existing) |
| hinged_lid | rec_bucket2_var_hinged_lid | lid flips up on rear-rim edge — REVOLUTE (horizontal Y axis) | converged |
| bunghole_plug | rec_bucket2_var_bunghole_plug | fixed top head + side bunghole; tapered bung pulls out radially — PRISMATIC (radial) | converged |

### Slot D: handle / carry (keep ≥1 non-fixed where used)
| 候选 | variant | key joint / structure | status |
|---|---|---|---|
| lid_arched_grip (baseline) | parent | arched grip FIXED on lid | parent (existing) |
| swing_bail | rec_bucket2_var_swing_bail | two FIXED pivot ears (for-i loop ×2) + arched bail — REVOLUTE (horizontal axis) | converged |
| side_ear_rings | rec_bucket2_var_side_ear_rings | two FIXED ears (for-i loop ×2) + two fold-out rings, each on its own REVOLUTE | converged |

## Multiplicity / Copy Logic

- count_param: **hoop_N** on Slot B; N ∈ {2,3,4} (distinct-N = 3). Loop `for i in range(N)` over a shared hoop-band helper, regular z-pitch, uniform FIXED joint policy → hoop_0..hoop_{N-1}.
- Secondary fixed loops (not multiplicity axes): swing_bail and side_ear_rings each use `for i in range(2)` over a shared ear helper for the two pivot ears (uniform per-side policy).
- Staves: N_STAVES groove loop exists on the body but stays a cosmetic single-body feature (not a jointed part axis).

## Variant count

7 NEW variants planned (cap ~8–10): straight_keg, tapered_pail, hoop_count, hinged_lid, bunghole_plug, swing_bail, side_ear_rings. All status = planned (PHASE 0 — not yet forked/compiled).

## 排除项 / dropped axes

- pure_scale (taller/shorter/wider keg with no topology change) — DROPPED (forbidden: pure-scale).
- color/material/wood-species/hoop-finish reskins — DROPPED (forbidden: color/material only).
- stave_count as a jointed part axis — DROPPED: staves are cosmetic grooves on one fused body mesh, not separate jointed parts; rewriting them to N parts would invent geometry the parent doesn't have. The hoop array carries the N-multiplicity axis instead.
- spigot/tap as a separate axis — folded conceptually into bunghole_plug closure (kept to one closure candidate to avoid axis bloat).
