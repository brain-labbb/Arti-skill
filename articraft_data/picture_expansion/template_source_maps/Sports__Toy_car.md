# Sports / Toy car — template source map

pattern: multiplicity (4 wheels copied via for-loop) over fixed named slots (body + deck-top mechanism); a chunky wooden push toy car with a smiling bug-face character and a bead-maze wire arch.

parents:
- rec_wooden-push-toy-car-with-a-smiling-bug-face-a-be_20260605_165950_419759_2a7a9bf4 ← picture/Sports/Toy car/001.png — covers: Slot A=bug_face, Slot B=bead_maze_arch, Slot C=round_disc, N(wheels)=4

Single parent fills one cell of every slot. Every variant forks from this single parent (smallest diff = one structural layer).

## Slot 候选覆盖

### Slot A:body_character_form (overall body silhouette + driver/character figure)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| bug_face_beetle | rec_wooden-push-toy-car-...2a7a9bf4 | body.body_block (_body_solid loft) / bug_head, eye_i, smile, nose, antenna_i (parent visuals) | rounded beetle deck + sphere bug head with face + antennae, no driver | converged (parent baseline) |
| classic_car_driver | rec_toy_car_var_classic | body.body_block (reshaped two-box loft) / driver head visual in open cockpit, dot eyes + smile | stubby classic two-box car, sloped hood + raised cabin, round driver head pokes up | built ✓ |
| pickup_truck_bed | rec_toy_car_var_pickup | body.body_block (cab + open cargo bed loft) / driver head visual, bed side walls + tailgate visuals | front cab block + lower open rear cargo bed with raised side walls | built ✓ |
| open_racer_cockpit | rec_toy_car_var_racer | body.body_block (tapered hull loft, pointed nose) / driver head visual in round cockpit pocket | low tapered racer hull, pointed nose, round cockpit, rounded tail | built ✓ |

### Slot B:deck_top_play_mechanism (the secondary articulated toy feature on the deck)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| bead_maze_arch | rec_wooden-push-toy-car-...2a7a9bf4 | wire_arch part (tube_from_spline_points) + bead_i parts / arch_to_bead_i (CONTINUOUS, axis=local wire tangent) | bent metal wire arch fixed to deck, 3 colored beads each spin about their wire segment | converged (parent baseline) |
| rooftop_spinner | rec_toy_car_var_spinner | spinner part (round pinwheel disc on post) / body_to_spinner (CONTINUOUS revolute, axis +Z, origin at post-top collar) | vertical post + colorful propeller/pinwheel disc spinning about vertical axis | built ✓ |
| steering_column | rec_toy_car_var_steering | steering_wheel part (spoked ring on tilted post) / body_to_steering (CONTINUOUS revolute about tilted column axis, origin at post-top boss) | tilted column + round 3/4-spoke steering wheel that turns about the column axis | built ✓ |

### Slot C:wheel_form (the round rolling wheel module — geometry of one wheel)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| round_disc | rec_wooden-push-toy-car-...2a7a9bf4 | wheel_* part (WheelGeometry) + hub_cap + spin_marker / axle_wheel_* (CONTINUOUS, axis +Y) | smooth round red disc wheel, light hub cap, recessed face | converged (parent baseline) |
| knobby_offroad | rec_toy_car_var_knobby | wheel_* part (WheelGeometry + lug ring) / lug_i visuals via for-loop, axle joint unchanged | fat round tyre with a regular ring of raised rounded lug knobs around the tread | built ✓ |
| spoked_wheel | rec_toy_car_var_spoked | wheel_* part (open rim + hub + spoke_i) / spoke_i via for-loop, axle joint unchanged | open round rim + central hub joined by a ring of chunky radial spokes (cartwheel) | built ✓ |

## Multiplicity / Copy Logic
- count_param: wheel_count (drives the number of wheel parts + axle peg visuals)
- N 样本已覆盖: {4, 3, 6} → rec_wooden-push-toy-car-...2a7a9bf4 (N=4, parent) / rec_toy_car_var_wheels3 (N=3, 1 front centerline + 2 rear) / rec_toy_car_var_wheels6 (N=6, 3 axles x L/R)
- 模板建议 N_range: [3, 8] (3-wheel trike, 4-wheel standard, 6/8-wheel hauler; odd N uses a single centerline front/rear wheel, even N uses paired L/R per axle)
- copied object: one wheel part (WheelGeometry) + its hub_cap + spin_marker visuals; one CONTINUOUS roll joint each.
- naming: wheel_{i} (or keep semantic wheel_front_left etc. when N=4); axle pegs as axle_{j} parent visuals.
- placement: paired left/right per axle at +/- AXLE_Y; axles spaced regularly along X between AXLE_RX..AXLE_FX; odd counts add one centerline wheel at y=0.
- joint policy: every wheel is an independent CONTINUOUS joint about the wheel-local Y axle (parent=body), uniform effort/velocity limits; no chaining, each wheel rolls independently.

## Slot interface notes (future InterfaceSpec hints)
- Slot A <-> Slot B: deck-top mechanism mounts on the upper deck face (body_block top, near z=BODY_TOP_Z). Single mating face + one anchor (post collar / arch feet) -> consumer joint = the mechanism's own continuous joint. Bead-maze arch is FIXED to body then beads are CONTINUOUS on the arch; spinner/steering are a single CONTINUOUS revolute directly on body.
- Slot A <-> Slot C: wheels mount on the body axle pegs at z=AXLE_Z (=WHEEL_R) along +/- AXLE_Y. Mating face = axle peg cylinder side; anchor = axle origin; consumer joint = CONTINUOUS about Y. Wheel-count multiplicity drives how many axle pegs/wheel pairs are emitted.
- Body must keep wheels reaching ground (wheel min_z <= ~0.004) for any N; body length stretches with axle count (see wheels6 variant).

## 排除项(未来 compatibility matrix 素材)
- none yet (P0 planning only; no fork runs executed). Watch for: open_racer_cockpit hull may be too low to seat the bead_maze_arch feet (Slot A x Slot B interface) — flag if arch feet float or pierce the cockpit; pickup_truck_bed cargo bed walls may clash with a rooftop_spinner post if both are ever combined (combination is sampler-only, not a fork obligation).

---
## Post-fork verification (SEGMENT 1 complete)
All planned variants forked via `articraft fork` (dashscope qwen3.7-max, thinking medium), then verified on-disk: last compile = success, ≥1 non-fixed joint present, collections=['workbench'] (workbench-only, not promoted), and picture.json bound into the correct `Sports__<小类>` subcat shard (reconcile rebuilt). Status cells above flipped planned→built ✓ accordingly. Ready for SEGMENT 2 (spec authoring).
