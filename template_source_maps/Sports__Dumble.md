# Sports / Dumble — template source map

pattern: multiplicity (plate stack x N per side) over fixed named slots (handle / collar)

parents:
- rec_adjustable-black-iron-dumbbell-with-stacked-rubb_20260605_165834_309488_e9921a05 (iron) <- picture/Sports/Dumble/002.png
  covers: Slot A=round plate, Slot B=star-lock finger collar, Slot C=knurled straight grip; asymmetric stack (3 left / 5 right). Plate helper = add_stack(sign,count,side) loop emitting left_plate_j / right_plate_j.
- rec_adjustable-chrome-dumbbell-with-stacked-round-st_20260605_165825_655766_e0c0e76a (chrome) <- picture/Sports/Dumble/001.png
  covers: Slot A=round plate, Slot B=plain knurled spin-lock nut, Slot C=knurled straight grip; symmetric stack (5 / 5). Plate helper = _plate_stack_geom(sign) loop; spin-lock collars via per-sign loop. ALL fork variants below fork from this symmetric parent (cleaner single-axis diff).

## Slot 候选覆盖

### Slot A: plate shape (footprint family)
| 候选 (future module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| round | (both parents) | body / plates_pos · plates_neg (chrome), left_plate_j · right_plate_j (iron) | circular disc plate via CylinderGeometry, raised hub + emboss ring | converged (parent) |
| hex | rec_dumbbell_var_hexplate | body / plate_i mesh from CadQuery 6-gon prism | regular hexagon prism plate (flat-sits, no roll), same bore/hub/stacking | built ✓ |
| dodecagonal | rec_dumbbell_var_dodecaplate | body / plate_i mesh from CadQuery 12-gon prism | 12-sided faceted plate (near-round, faceted edges) | built ✓ |

### Slot B: plate-locking collar mechanism
| 候选 (future module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| star-lock finger collar | rec_...e9921a05 (iron) | left_collar / right_collar · body_to_left_collar · body_to_right_collar (CONTINUOUS x) | knurled nut + radiating star/lever finger lobes (_collar_solid) | converged (parent) |
| plain knurled spin-lock nut | rec_...e0c0e76a (chrome) | spinlock_collar_pos · spinlock_collar_neg · collar_spin_pos/neg (CONTINUOUS x) | single knurled KnobGeometry nut seated on thread | converged (parent) |
| hex jam-nut pair | rec_dumbbell_var_hexnut | inner_hex_nut_{side} (parent visual, fixed) + outer_hex_nut_{side} (jointed) · jamnut_spin_{side} (CONTINUOUS/REVOLUTE x) | two stacked 6-sided hex nuts per side, outer one spins to lock against inner | built ✓ |

### Slot C: handle grip form
| 候选 (future module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| knurled straight bar | (both parents) | body / grip_core · grip_knurl (chrome), handle (iron) | constant-radius knurled cylindrical grip | converged (parent) |
| contoured ergonomic | rec_dumbbell_var_contourgrip | body / grip_contour (lathe-revolved varying-radius profile) | barrel/hourglass bulged mid grip, knurled surface, swept radius | built ✓ |

## Multiplicity / Copy Logic
- count_param: plates_per_side (N) — the per-side plate stack copied object.
- copied object: one weight plate (shared plate geometry helper), emitted via for i in range(N): name plate_i (or plates_pos/_neg merged stack as in chrome parent).
- naming: plate_{i} / per-side {side}_plate_{i}.
- placement: linear along the bar axis (+X / -X), equal PLATE_THICK spacing, butted against the grip sleeve, stacking outward; collar re-seats just outboard of the resulting stack end on the exposed thread.
- joint policy: plates are FIXED to the body (static load); the only non-fixed joints are the two spin-lock collars (CONTINUOUS about +X). Multiplicity does NOT add joints — it lengthens the static stack and shifts the collar seat.
- N 样本已覆盖: 5 (parents) -> chrome symmetric / iron 3+5 ; 2 -> rec_dumbbell_var_plates2 ; 4 -> rec_dumbbell_var_plates4 ; 6 -> rec_dumbbell_var_plates6.
- 模板建议 N_range: [1, 8] per side (real adjustable-dumbbell loadout; collar must always leave exposed thread to seat on — cap where sleeve/thread length runs out).

## 组合数预审
Pi(slots) = A(3) x B(3) x C(2) = 18 (already >= 10 before multiplicity); with N samples (4 distinct N: 2/4/5/6) the topology diversity is far above the gate.
variant cells filled this batch = A(3-1=2: hex,dodeca) + B(3-2=1: hex jam-nut; two parents fill star-lock + plain-nut) + C(2-1=1: contour) + N(3 new: 2,4,6) = 7.

## 排除项 (future compatibility matrix material)
- none yet (P0 planning only — no forks run). Watch in fork/build:
  - hex jam-nut (Slot B) x non-round plates (Slot A): two faceted-prism layers stacking — verify the outer jam nut still leaves a real visible contact face with the inner nut for the joint origin (no phantom anchor pad).
  - N=6 heavy stack: bar SLEEVE_LEN / THREAD_LEN must lengthen so the collar still seats on exposed thread outboard of the longer stack (do not bury the thread under plates).

---
## Post-fork verification (SEGMENT 1 complete)
All planned variants forked via `articraft fork` (dashscope qwen3.7-max, thinking medium), then verified on-disk: last compile = success, ≥1 non-fixed joint present, collections=['workbench'] (workbench-only, not promoted), and picture.json bound into the correct `Sports__<小类>` subcat shard (reconcile rebuilt). Status cells above flipped planned→built ✓ accordingly. Ready for SEGMENT 2 (spec authoring).
