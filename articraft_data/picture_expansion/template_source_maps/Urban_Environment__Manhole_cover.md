# Urban Environment / Manhole cover — variant-fork source map

- **Slug:** `manhole_cover`
- **Shard:** `Manhole_cover`
- **Subcat index:** `data/index/subcat/Urban_Environment__Manhole_cover.jsonl`
- **Picture dir:** `picture/Urban Environment/Manhole cover/` (001–004.png)

## Identity (must hold for every variant)

A cast-iron or concrete **cover/grate seated in a recessed frame** that sits flush
in the ground, with a hollow shaft/throat void beneath. The cover is **openable**:
the cover is the articulated child and the open mechanism is a **real non-fixed
joint** — every parent uses a +Z PRISMATIC lift-out; one new variant introduces a
REVOLUTE edge hinge. The frame and the hollow shaft below it stay FIXED.

## Parents (4)

| # | pic | record_id | model `name=` | outline | surface | mechanism | loop emission |
|---|-----|-----------|---------------|---------|---------|-----------|---------------|
| 1 | 004 | `rec_small-dark-cast-iron-square-floor-drain-grate-se_20260608_165941_241927_27f846c6` | `cast_iron_fan_slot_floor_drain` | square | curved teardrop fan slots | PRISMATIC +Z lift | `for s in range(N_SLOTS)` (slots) + `for i in range(steps)` (arc) |
| 2 | 003 | `rec_square-rusty-cast-iron-drainage-grate-with-a-bas_20260608_165623_537538_01cd7eed` | `cast_iron_basket_weave_grate` | square | basket-weave staggered slot lattice | PRISMATIC +Z lift | nested `for r in range(n_rows)` / `for c in range(n_cols)` (N derived from field) |
| 3 | 002 | `rec_square-cast-iron-inspection-chamber-cover-with-a_20260608_165116_917790_e40ac516` | `cast_iron_inspection_cover` | square | solid lid + anti-slip diamond stud grid + center panel + 2 key recesses | PRISMATIC +Z lift | nested `for ix in range(n)` / `for iy in range(n)` (studs) + `for sx in (-1,1)` (key recesses) |
| 4 | 001 | `rec_square-weathered-concrete-utility-access-cover-s_20260608_164425_869735_edad8300` | `concrete_utility_access_cover` | square | solid concrete slab + center pry slot | PRISMATIC +Z lift | **no loops** (single slot, single slab) |

### Loop-emission notes
- Parents 1 & 2 already loop-emit the grate slots correctly. Parent 3 loop-emits
  the stud grid + key recesses. Parent 4 has no repeats (single pry slot).
- **Multiplicity variants (`slot_count_dense`, `slot_count_coarse`) inherit parent 2's
  existing nested slot loop** — they only change N; no loop rewrite required.
- The **new pattern variants must be loop-emitted as `grate_{i}`**: `parallel_slots`
  (single loop over straight slots), `cross_grid` (nested loop over square holes),
  `radial_pattern` (single loop of spoke slots around the circle). These are fresh
  loops, authored from scratch per the prompt; a hand-written slot list must NOT be used.

## Axis / slot plan

| slot (axis group) | candidates | count |
|---|---|---|
| **A. Cover outline** | square (parents) · round (`round_outline`, `radial_pattern`) · rectangular/oblong (`rectangular_outline`) | 3 |
| **B. Surface pattern** | solid lid (parents 3/4) · parallel straight slots (`parallel_slots`) · cross/waffle grid (`cross_grid`) · radial spoke slots (`radial_pattern`) · basket-weave (parents) | 4 (incl. parent baseline) |
| **C. Open mechanism** | lift-out PRISMATIC (parents) · hinged swing-up flap REVOLUTE (`hinged_flap`) | 2 |
| **D. Slot-count multiplicity N** | basket-weave default N · dense fine-pitch N (`slot_count_dense`) · coarse sparse N (`slot_count_coarse`) | 3 distinct N |

At least one real non-fixed joint always present (PRISMATIC lift on most;
REVOLUTE hinge on `hinged_flap`).

## COMBO PRE-AUDIT (HARD GATE)

product(outline=3 × pattern=4 × mechanism=2) = **24** structural combinations,
**× 3 distinct-N (multiplicity)** = **72 >= 10**. PASS.

## Variants (8 — within the ~8–10 cap)

| record_id | label | axis | parent | prompt |
|---|---|---|---|---|
| `rec_manhole_cover_var_hinged_flap` | `manhole_cover-hinged_flap` | mechanism → REVOLUTE edge hinge | parent 3 | `/tmp/urb_manhole_cover_var_hinged_flap.txt` |
| `rec_manhole_cover_var_round_outline` | `manhole_cover-round_outline` | outline → round | parent 3 | `/tmp/urb_manhole_cover_var_round_outline.txt` |
| `rec_manhole_cover_var_radial_pattern` | `manhole_cover-radial_pattern` | pattern → radial spoke slots (round) | parent 1 | `/tmp/urb_manhole_cover_var_radial_pattern.txt` |
| `rec_manhole_cover_var_cross_grid` | `manhole_cover-cross_grid` | pattern → cross/waffle grid | parent 2 | `/tmp/urb_manhole_cover_var_cross_grid.txt` |
| `rec_manhole_cover_var_parallel_slots` | `manhole_cover-parallel_slots` | pattern → straight parallel slots | parent 2 | `/tmp/urb_manhole_cover_var_parallel_slots.txt` |
| `rec_manhole_cover_var_slot_count_dense` | `manhole_cover-slot_count_dense` | multiplicity N → dense | parent 2 | `/tmp/urb_manhole_cover_var_slot_count_dense.txt` |
| `rec_manhole_cover_var_slot_count_coarse` | `manhole_cover-slot_count_coarse` | multiplicity N → coarse | parent 2 | `/tmp/urb_manhole_cover_var_slot_count_coarse.txt` |
| `rec_manhole_cover_var_rectangular_outline` | `manhole_cover-rectangular_outline` | outline → rectangular/oblong | parent 4 | `/tmp/urb_manhole_cover_var_rectangular_outline.txt` |

Manifest: `/tmp/manifest_urb_manhole_cover.tsv` (TAB, 4 fields, no header).

## Dropped / deferred axes

- **Color / material / pure-scale** — disallowed (never a structural change).
- **Key-slot / pick-bar lift mechanism** — deferred; the REVOLUTE `hinged_flap`
  already covers the non-PRISMATIC mechanism axis, and pick-bar would be a cosmetic
  greeble on the existing lift rather than a distinct joint. Re-add later if more
  mechanism diversity is wanted.
- **Stud / lettering panel detail variants** — cosmetic top-surface decoration on
  the solid-lid parent; dropped as non-structural.

## Status

PHASE 0 PLAN COMPLETE — planning files written; no forks run, no records mutated.
Next phase: fork each row in the manifest from its listed parent.
