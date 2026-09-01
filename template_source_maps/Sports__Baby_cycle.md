# Sports / Baby cycle — template source map

pattern: mixed (parallel_children steering+wheel mechanisms + a multiplicity wheel-loop axis)
parents: rec_toddler-balance-trike-baby-cycle-white-frame-wit_20260605_165756_379018_4aab549e ← picture/Sports/Baby cycle/001.png
  (covers: Wheel-count N=3 [1 front + 2 rear] · Frame=step_through_loop · Foot=none_balance · Bar=swept_riser)

The parent is a toddler balance trike: white tubular step-through frame (root), blue saddle
FIXED to the deck, blue swept handlebar + fork that steer together (REVOLUTE about the raked
head tube), one front wheel rolling as a child of the fork (CONTINUOUS), and two splayed rear
wheels rolling on the rear axle (CONTINUOUS). Wheels share a `_wheel_part` geometry helper but
the parent calls it three times by hand (front_wheel / rear_left_wheel / rear_right_wheel) —
it is NOT a `for i in range(n)` loop yet, so the multiplicity variants below must rewrite the
wheels as a `wheel_{i}` loop with one shared helper and a uniform CONTINUOUS roll joint policy.

## Slot 候选覆盖

### Slot A:wheel_arrangement (multiplicity — wheel count N + front/rear split)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| trike_1f2r (N=3) | rec_toddler-balance-trike-...4aab549e (parent) | front_wheel + rear_left_wheel/rear_right_wheel · front_wheel_roll/rear_left_roll/rear_right_roll (CONTINUOUS) | 1 front on fork + 2 splayed rear on rear axle; baseline | converged (parent) |
| quad_2f2r (N=4) | rec_baby_cycle_var_quad4 | wheel_{i} loop · wheel_{i}_roll (CONTINUOUS); 2 front children of steer, 2 rear children of frame | widened transverse front axle carries 2 front wheels + 2 rear = 4 total | built ✓ |
| inline_1f1r (N=2) | rec_baby_cycle_var_inline2 | wheel_{i} loop · wheel_{i}_roll (CONTINUOUS); centerline rear stub | 1 front on fork + 1 centered rear inline balance bike | built ✓ |

### Slot B:frame_form
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| step_through_loop | parent | frame.frame_tube (_swept_tube backbone + head + rear stays) | low single curved U backbone, no top tube | converged (parent) |
| crossbar_diamond | rec_baby_cycle_var_crossbar | frame top_tube + down_tube + rear_stay + seat_post; saddle on seat post | closed diamond profile w/ horizontal cross-bar above wheels | built ✓ |
| twin_beam_deck | rec_baby_cycle_var_twinbeam | frame rail_{i} loop (for i in range(2)) + saddle deck plate | two mirrored parallel side rails + flat deck plate | built ✓ |

### Slot C:foot_interface
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| none_balance | parent | (no foot parts) | pure balance trike, feet on ground | converged (parent) |
| front_pedals | rec_baby_cycle_var_pedals | crank_{i} loop + pedal on front-wheel axle; rides front_wheel_roll (CONTINUOUS) | 2 cranks 180deg apart + pedals turn with front wheel = pedal trike | built ✓ |
| footrest_pegs | rec_baby_cycle_var_footrest | footrest_{i} loop on frame boss (mirrored, non-moving frame visuals) | 2 fixed side pegs to rest feet while coasting | built ✓ |

### Slot D:handlebar_form
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| swept_riser | parent | steering.handlebar_bar (_handlebar_mesh swept double-rise) + grip_left/grip_right | low swept bar with two rises, straight grips | converged (parent) |
| t_bar_straight | rec_baby_cycle_var_tbar | steering.handlebar_bar (straight crossbar cyl) + grip_left/grip_right angled back | flat horizontal T crossbar | built ✓ |
| ape_hanger_loop | rec_baby_cycle_var_apehanger | steering.handlebar_bar (_swept_tube tall U) + grips high/close | tall high-rise rounded-U cruiser loop | built ✓ |

All Slot B/C/D variants keep the parent's steering REVOLUTE + 3 wheel-roll CONTINUOUS joints
live; the steering part owns Slot D so the bar always turns with the fork.

## Multiplicity / Copy Logic
- count_param: wheel_count (front_count + rear_count); each wheel emitted in a `for i in range(n)`
  loop via the shared `_wheel_part` / wheel geometry helper as `wheel_{i}`.
- N 样本已覆盖: {3, 4, 2} → parent (1f+2r) / rec_baby_cycle_var_quad4 (2f+2r) / rec_baby_cycle_var_inline2 (1f+1r)
- 模板建议 N_range: total wheels [2, 4] (front ∈ {1,2}, rear ∈ {1,2}); a wide-track 3-rear scooter
  base could push rear to 3 but is not sampled here.
- copied object: one wheel = black tire ring + blue side discs + central hub barrel + off-axis
  valve-stem marker (for AABB spin detection).
- naming: wheel_{i} (+ wheel_{i}_tire / wheel_{i}_disc / wheel_{i}_marker visuals).
- placement: front wheels children of the steering part at the front axle (symmetric about x=0 when
  2 front); rear wheels children of the frame at the rear axle (symmetric about x=0 when 2 rear);
  inline cases sit on the centerline.
- joint policy: every wheel gets its own CONTINUOUS roll joint about the local x axis (axle),
  uniform MotionLimits(effort=2, velocity=20); front wheels inherit steering via parent=steering.
- secondary copy loops: Slot B twin_beam frame rails = `rail_{i}` for i in range(2) (mirrored,
  part of frame, no joints); Slot C pedals = `crank_{i}` for i in range(2) (180deg apart, ride the
  front-wheel roll joint, no separate joint); Slot C footrests = `footrest_{i}` for i in range(2)
  (mirrored, non-moving frame visuals).

## 组合数预审
Π(Slot B 3 × Slot C 3 × Slot D 3) × N-samples 3 = 27 × 3 = 81 ≥ 10. PASS.
(Slot A IS the N-sample axis, so it is the ×3 factor, not an extra Π term.)

## 排除项(未来 compatibility matrix 素材)
- inline_1f1r (N=2) × front_pedals: a single-rear inline balance bike with front cranks is a real
  but tippy combo for a toddler; sampled only as separate single-axis cells, not as a combo here.
- quad_2f2r (N=4) × ape_hanger_loop: no real-world conflict, just not sampled (combination is the
  template sampler's job, not the fork batch's).
- No values were dropped to a single candidate; every slot has ≥3 structurally-distinct candidates.

---
## Post-fork verification (SEGMENT 1 complete)
All planned variants forked via `articraft fork` (dashscope qwen3.7-max, thinking medium), then verified on-disk: last compile = success, ≥1 non-fixed joint present, collections=['workbench'] (workbench-only, not promoted), and picture.json bound into the correct `Sports__<小类>` subcat shard (reconcile rebuilt). Status cells above flipped planned→built ✓ accordingly. Ready for SEGMENT 2 (spec authoring).
