# Urban Environment / Well lid — variant-fork source map

- **Slug:** `well_lid`
- **Shard:** `Well_lid`
- **PICDIR:** `picture/Urban Environment/Well lid` (001.png)
- **Index:** `data/index/subcat/Urban_Environment__Well_lid.jsonl` (1 parent)

## Identity (must hold across all variants)

Round cast-iron well / manhole cover seated in a **round** cast-iron frame ring
over a shaft void. Decorative cast top (waffle grid in the parent), a pick-hole
for lifting, and a **real OPEN mechanism** as the non-fixed joint (parent: rear
edge-hinge REVOLUTE). Variants stay **ROUND well covers in a round frame** —
they are NOT the square drain-grate covers (that is the separate Manhole-cover
小类). Never color/material/pure-scale as the change.

## Parent

| record_id | role | notes |
|---|---|---|
| `rec_round-cast-iron-manhole-well-cover-with-a-dense-_20260608_172202_787515_e3176cb1` | PARENT (only) | Round cast-iron cover, square WAFFLE-GRID top, pick-hole slot, REVOLUTE rear edge-hinge swing-up over a round frame seat + shaft void. |

### Parent loop / readability audit

- Parent `model.py` (`revisions/rev_000001/model.py`) has exactly **one** loop
  (`for i in range(...)` at the groove-cutter step) that cuts a cross-hatch of
  grooves into a single fused CadQuery slab. The resulting waffle **cells are
  NOT emitted as named `cell_{i}` parts/visuals** — they live only as fused
  geometry inside one mesh (`manhole_cover`).
- Therefore the **cell-multiplicity (N) axis variants MUST request the loop
  rewrite**: extract the raised pads into a `for i in range(n)` loop emitting
  `cell_{i}` named visuals from a shared square-pad geometry helper on a regular
  grid. The pattern-style variants (rings / spokes / medallion-stud-ring)
  likewise must loop-emit their repeats (`ring_{i}`, `spoke_{i}`, `stud_{i}`).
- Pick-hole, frame seat, collar, shaft void are single inlined/parent visuals —
  fine as parent visuals (no FIXED-joint decoration parts needed).

## Axis / slot plan (3 structural axes + multiplicity)

| Axis (slot) | Candidates (>=2) | Notes |
|---|---|---|
| **Cast top pattern** | waffle-grid (parent) · concentric rings · radial spokes · lettered center medallion + stud ring | each non-grid pattern loop-emits its repeats |
| **Open mechanism** (>=1 non-fixed joint) | rear edge-hinge REVOLUTE (parent) · vertical lift-out PRISMATIC (+Z) · swing-up center pick-bar REVOLUTE | always a real non-fixed joint |
| **Frame style** | flush ground ring (parent) · raised collar curb | round seat preserved |
| **Pattern-cell multiplicity N** (`cell_{i}` loop) | denser/finer · coarser/larger — 2 distinct N + parent ⇒ 3 N values | requires loop rewrite |

### COMBO PRE-AUDIT (HARD GATE)

`product(candidates) x distinct-N`
= pattern(4) x open-mechanism(3) x frame(2) x distinct-N(3)
= **4 x 3 x 2 x 3 = 72 >= 10**  ✅ PASS

(Even the conservative count — 4 patterns x 3 mechanisms x 2 frames = 24, or
just patterns x N = 4 x 3 = 12.)

## Variants (8 NEW, cap 8-10) — single-axis each

| record_id | label | axis touched | status |
|---|---|---|---|
| `rec_well_lid_var_pattern_rings` | `well_lid-pattern_rings` | pattern → concentric rings (`ring_i` loop) | converged |
| `rec_well_lid_var_pattern_spokes` | `well_lid-pattern_spokes` | pattern → radial spokes (`spoke_i` loop) | converged |
| `rec_well_lid_var_pattern_medallion` | `well_lid-pattern_medallion` | pattern → center medallion + stud ring (`stud_i` loop) | converged |
| `rec_well_lid_var_open_liftout` | `well_lid-open_liftout` | open mechanism → vertical PRISMATIC lift-out (+Z) | converged |
| `rec_well_lid_var_open_pickbar` | `well_lid-open_pickbar` | open mechanism → swing-up center pick-bar REVOLUTE | converged |
| `rec_well_lid_var_cells_n` | `well_lid-cells_n` | cell multiplicity → denser grid + `cell_i` loop rewrite | converged |
| `rec_well_lid_var_cells_coarse` | `well_lid-cells_coarse` | cell multiplicity → coarser grid + `cell_i` loop rewrite | converged |
| `rec_well_lid_var_frame_collar` | `well_lid-frame_collar` | frame style → raised collar curb | converged |

Prompts: `/tmp/urb_well_lid_var_<axis>.txt` (one-paragraph single-axis prose +
blank line + verbatim `/tmp/urb_suffix_well_lid.txt`).
Manifest: `/tmp/manifest_urb_well_lid.tsv` (4 TAB fields, no header, 8 rows).

## Dropped / not-spawned axes

- **Square waffle-grid restyle as a new variant** — dropped; it is the parent's
  own pattern (would be a no-op identity change). Reused only as the loop-rewrite
  base for the two `cells_*` multiplicity variants.
- **Square drain-grate / slotted-bar cover** — dropped; that is the neighboring
  **Manhole cover** 小类, not the round well_lid identity.
- **Color / material / pure-scale** — never an allowed change axis.
