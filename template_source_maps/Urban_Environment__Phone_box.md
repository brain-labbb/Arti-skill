# Urban Environment / Phone box — variant-fork source map

- **Slug:** `phone_box`
- **Shard:** `Phone_box`
- **Picdir:** `picture/Urban Environment/Phone box` (001.png)
- **Identity:** Telephone booth / kiosk — tall boxy upright enclosure on a low base plinth, four corner pilasters, glazed window walls of a regular glazing-bar grid of rectangular panes, a "TELEPHONE" frieze/sign band wrapping the top, a crown roof. One face is the entrance; the single moving part is a door (or fold leaf) on a VERTICAL hinge.

## Parent (1)

| record_id | model.py | notes |
|---|---|---|
| `rec_red-british-k6-telephone-booth-phone-box-a-cast-_20260612_113148_944128_c00c1818` | `data/records/<id>/revisions/rev_000001/model.py` | Red British K6: domed crown roof, single front (+X) REVOLUTE door on vertical (+Z) hinge at front corner; +Y/-Y/-X are fixed glazed walls. Square ~0.92 m plan, ~2.36 m tall. |

### Readability / loop-emission audit (parent)

- **Glazing grid IS loop-emitted** — helper `_glazed_grid(...)` emits `{prefix}_glass`, `{prefix}_vbar_{c}` (cols+1), `{prefix}_hbar_{r}` (rows+1). Driven by `COLS=3`, `ROWS=6`. Door glazing also loop-emitted: `door_vbar_{c}`, `door_hbar_{r}` (d_cols=COLS, d_rows=ROWS-1). No hand-written repeated panes. GOOD — N is varyable by editing COLS/ROWS.
- Corner pilasters loop-emitted: `pilaster_{i}` (4). Dome loop-emitted: `roof_step_{s}` (dome_steps=11).
- **Hand-written (cosmetic) repeats:** 4 frieze signs (`sign_{fc}`/`signtext_{fc}`) and 4 crown emblems (`crown_emblem_{fc}`) use explicit per-face spec lists; kick panels per-face. These are cosmetic greebles, NOT the glazing grid — acceptable; do not over-loop them.
- **Real joint:** one REVOLUTE `body_to_door`, axis `(0,0,1)`, vertical hinge at front corner, lower=0 / upper≈95°. This is the joint variants must preserve.

## Axis plan (PHASE 0)

| Slot | Candidates | N |
|---|---|---|
| A. Roof / crown form | domed-K6 (baseline) · flat modern slab · triangular pediment | 3 |
| B. Door mechanism | single hinged REVOLUTE (baseline) · bi-fold (vertical-hinge outer leaf) · open-side no front door | 3 |
| C. Glazing grid multiplicity N | medium 3×6 (baseline) · sparse 2×4 · dense 4×8 | 3 distinct N |
| D. Footprint | square one-person (baseline) · wider two-person (~1.4×) | 2 |

**COMBO PRE-AUDIT (HARD GATE):** product(candidates) × distinct-N = 3 × 3 × 3 × 2 = **54 ≥ 10 → PASS.** Distinct-N alone = 3 (2×4, 3×6, 4×8).

Dropped axes (forbidden / non-structural): kiosk color & materials (red/glass/gold) — cosmetic; pure uniform scale — non-structural; "TELEPHONE" text content — cosmetic. The door hinge stays the real articulation in every variant; never remove the single vertical-hinge moving part.

## New variants (9)

| record_id | label | prompt | axis | status |
|---|---|---|---|---|
| `rec_phone_box_var_roof_flat` | phone_box-roof_flat | `/tmp/urb_phone_box_var_roof_flat.txt` | A: flat modern slab roof | converged |
| `rec_phone_box_var_roof_pediment` | phone_box-roof_pediment | `/tmp/urb_phone_box_var_roof_pediment.txt` | A: triangular pediment roof | converged |
| `rec_phone_box_var_door_bifold` | phone_box-door_bifold | `/tmp/urb_phone_box_var_door_bifold.txt` | B: bi-fold door (vertical hinge) | converged |
| `rec_phone_box_var_door_openside` | phone_box-door_openside | `/tmp/urb_phone_box_var_door_openside.txt` | B: open-side, no front door (one side leaf hinge) | converged |
| `rec_phone_box_var_glazing_sparse` | phone_box-glazing_sparse | `/tmp/urb_phone_box_var_glazing_sparse.txt` | C: sparse 2×4 glazing | converged |
| `rec_phone_box_var_glazing_dense` | phone_box-glazing_dense | `/tmp/urb_phone_box_var_glazing_dense.txt` | C: dense 4×8 glazing | converged |
| `rec_phone_box_var_footprint_wide` | phone_box-footprint_wide | `/tmp/urb_phone_box_var_footprint_wide.txt` | D: wider two-person footprint | converged |
| `rec_phone_box_var_flat_dense_wide` | phone_box-flat_dense_wide | `/tmp/urb_phone_box_var_flat_dense_wide.txt` | A+C+D combo carrier | converged |
| `rec_phone_box_var_pediment_sparse_bifold` | phone_box-pediment_sparse_bifold | `/tmp/urb_phone_box_var_pediment_sparse_bifold.txt` | A+C+B combo carrier | converged |

- Manifest: `/tmp/manifest_urb_phone_box.tsv` (TAB, 4 fields, no header).
- Suffix (verbatim appended to each prompt): `/tmp/urb_suffix_phone_box.txt`.
- Cap: 9 new variants (within ~8–10).
