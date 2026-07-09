# Urban Environment / Tipping Barrow — variant source map

slug: `tipping_barrow`  shard: `Tipping_Barrow`  picdir: `picture/Urban Environment/Tipping Barrow/`

identity: a tub / tray / hopper carried on a wheeled steel frame that TIPS FORWARD to dump
its load. The real joints are the forward tip pivot (REVOLUTE about the lateral axle line)
and the wheel roll (CONTINUOUS about the lateral axis); supports are handles + legs / axle
frame / casters. Every variant stays a tipping barrow or dump cart.

pattern: single-axis fork variants off two parents (single-wheel garden barrow + heavy plastic
tilt-truck). Each variant changes exactly one structural axis: wheel count, tub shape, tip
mechanism, or ground support. No color/material/pure-scale changes.

## Parents

- `rec_single-wheel-garden-wheelbarrow-a-gray-sheet-met_20260608_164456_401819_ffd09fd0`
  ← `picture/Urban Environment/Tipping Barrow/002.png`
  Single-wheel garden wheelbarrow: lofted sheet-metal bowl tray (FIXED on the frame) on a
  tubular steel frame; two rear stand legs + foot rail; one front pneumatic wheel
  (`frame_to_wheel`, CONTINUOUS about Y); two yellow grips. Tipped by lifting the barrow
  (tray itself has no tip joint). Loop-emitted: handle tubes, legs, tray-mount crosses, fork
  stubs, grips (all `for side in (1,-1)`); rrect corners via `for i in range(n_corner+1)`.

- `rec_heavy-duty-plastic-tilt-truck-dump-cart-a-large-_20260608_164439_693743_747a4fc9`
  ← `picture/Urban Environment/Tipping Barrow/001.png`
  Heavy plastic tilt-truck / dump cart: tapered ribbed PE hopper that TIPS FORWARD
  (`frame_to_hopper`, REVOLUTE about Y at the rear axle line) on a low steel base frame; two
  big rear wheels (`frame_to_wheel_l/r`, CONTINUOUS); two front swivel casters (yoke REVOLUTE
  yaw about Z + wheel CONTINUOUS roll). Loop-emitted: hopper sections/ribs, saddle plates,
  rails, axle blocks, caster plates (`for sy in (1,-1)`); wheels via
  `for sy,wheel_name,joint_name in (...)`; casters via `for i,sy in ((0,1),(1,-1))`.
  This parent is the baseline tip-mechanism reference (it already has the real dump REVOLUTE).

## Combo pre-audit (HARD GATE: product(candidates) × distinct-N ≥ 10)

Four structural axes, each ≥2 candidates; distinct wheel-count N = {1, 2, 4} → 3 N values.

- wheel_count slot N candidates: {1, 2, 4} → 3 distinct N
- tub_shape candidates: {shallow_tray, deep_dump_tub, rounded_pan} → 3
- tip_mechanism candidates: {front_pivot_tip (REVOLUTE), lift_and_tip (linkage REVOLUTE×2)} → 2
- support candidates: {two_rear_legs+wheel, axle_frame, front_swivel_casters} → 3

product(tub_shape 3 × tip 2 × support 3) = 18 ≥ 10 ✓, and multiplied by distinct-N (3) far
exceeds 10. Even the minimal cross (wheel_count distinct-N 3 × tub_shape 3) = 9 → with tip 2
→ 18 ≥ 10. **GATE PASSED.**

## Slot plan (each slot ≥2 candidates; multiplicity 2–3 N)

### Slot A: wheel_count N — the ground-wheel multiplicity {1 / 2 / 4}
| candidate | variant | key joint / structure | status |
|---|---|---|---|
| one_central_wheel (baseline) | parent P1 | single front wheel CONTINUOUS roll | parent (existing) |
| two_axle_wheels | wheel2 | mirrored pair on one rear axle, CONTINUOUS×2, loop-emitted | converged |
| four_wheels (2 front + 2 rear) | wheel4 | front pair + rear pair, CONTINUOUS (+optional swivel), single loop | converged |
| (two_big_wheels baseline) | parent P2 | two big rear wheels + 2 casters | parent (existing) |

### Slot B: tub_shape — load-body cross-section
| candidate | variant | structure | status |
|---|---|---|---|
| shallow_tray (baseline) | parent P1 | lofted shallow sheet-metal bowl | parent |
| tapered_hopper (baseline) | parent P2 | tapered ribbed angular hopper | parent |
| deep_dump_tub | tubdeep | tall steep-walled lofted bin, larger volume | converged |
| rounded_pan | tubround | curved near-circular lathe/loft pan + rolled rim | converged |

### Slot C: tip_mechanism — how it dumps (≥1 real non-fixed joint always)
| candidate | variant | key joint | status |
|---|---|---|---|
| front_pivot_tip (baseline) | parent P2 | tub REVOLUTE about rear axle line | parent |
| lift_off_then_lift (baseline) | parent P1 | tipped by lifting whole barrow (wheel roll real) | parent |
| lift_and_tip linkage | lifttip | mirrored lift arm REVOLUTE + tub tip REVOLUTE (raise-then-tip), loop arms | converged |

### Slot D: support — ground stance at rest
| candidate | variant | structure | status |
|---|---|---|---|
| two_rear_legs + wheel (baseline) | parent P1 | mirrored rear stand legs + front wheel | parent |
| axle_frame (baseline) | parent P2 | low steel cradle on axle | parent |
| two_rear_stand_legs | legs | mirrored rear legs + cross-rail, parked on legs+wheel, loop-emitted | converged |
| front_swivel_casters | caster | mirrored front casters: REVOLUTE yaw + CONTINUOUS roll, loop-emitted | converged |

## Variants (8 cap; 7 planned — see /tmp/manifest_urb_tipping_barrow.tsv)

| record_id | label | axis changed | parent | prompt |
|---|---|---|---|---|
| rec_tipping_barrow_var_wheel4 | tipping_barrow-wheel4 | wheel_count → 4 | P2 | /tmp/urb_tipping_barrow_var_wheel4.txt |
| rec_tipping_barrow_var_wheel2 | tipping_barrow-wheel2 | wheel_count → 2 | P1 | /tmp/urb_tipping_barrow_var_wheel2.txt |
| rec_tipping_barrow_var_tubdeep | tipping_barrow-tubdeep | tub_shape → deep tub | P2 | /tmp/urb_tipping_barrow_var_tubdeep.txt |
| rec_tipping_barrow_var_tubround | tipping_barrow-tubround | tub_shape → rounded pan | P1 | /tmp/urb_tipping_barrow_var_tubround.txt |
| rec_tipping_barrow_var_lifttip | tipping_barrow-lifttip | tip_mechanism → lift-and-tip | P2 | /tmp/urb_tipping_barrow_var_lifttip.txt |
| rec_tipping_barrow_var_legs | tipping_barrow-legs | support → two rear legs | P1 | /tmp/urb_tipping_barrow_var_legs.txt |
| rec_tipping_barrow_var_caster | tipping_barrow-caster | support → front swivel casters | P2 | /tmp/urb_tipping_barrow_var_caster.txt |

statuses: all `planned` (Phase 0 — no forks run yet). Suffix verbatim from
`/tmp/urb_suffix_tipping_barrow.txt` appended to every prompt after a blank line.

## Loop notes

Both parents already loop-emit all repeated sub-parts (wheels, legs, ribs, casters, saddle
plates, frame rails) via `for side/sy in (1,-1)` and tuple-driven loops — no hand-written
repeats to fix. Variant prompts explicitly require each new repeated set (4 wheels, 2 casters,
2 lift arms, 2 legs) to be emitted from a single for-loop over a shared geometry helper with a
uniform joint policy.

## Dropped axes

- color / material / pure-scale — forbidden by the variant rule (never the targeted change).
- "tray with no tip joint at all" as a new variant — rejected: would drop the tipping
  identity; the no-tip-joint stance is already covered by parent P1 (lift-to-tip) which keeps
  the wheel as the real non-fixed joint.
- handle style (straight vs loop vs T-bar) — cosmetic/greeble, not a structural mechanism axis;
  dropped to keep ≤8 high-value variants.
- one-wheel barrow is kept as the N=1 baseline (parent) rather than a new variant (would be a
  near-duplicate of P1).
