# Urban Environment / Draft Wagon — template source map

slug: draft_wagon
identity: wooden wheeled draft wagon / cart — a plank cargo body riding on large spoked wooden wheels with a fixed/steerable axle chassis and forward draw poles (or pull shafts). Spans the 2-wheel hand cart / dray, the 4-wheel open farm wagon, and the 4-wheel covered caravan. PRIMARY articulation is always the wheels rolling (CONTINUOUS spin about world Y); 4-wheel members add a steerable front bolster (REVOLUTE yaw about world Z). All variants stay wagons.

pattern: shared spoked-wheel helper (`_wheel_visuals`) + count-driven plank/board/spoke loops + per-wheel CONTINUOUS roll joints on axle stubs, with a 4-wheel REVOLUTE steering bolster carrying the front pair and draw poles.

## Parents (originals on the grid)

- rec_two-wheeled-wooden-hand-cart-tip-cart-dray-with-_20260608_164414_778806_b1d205ac ← picture/Urban Environment/Draft Wagon/001.png
  - 2-wheel hand cart (tip cart / dray): single rear axle, one wheel pair, low plank side rails + tall plank back wall, two long forward pull shafts, angled front prop legs. SPOKE_COUNT=12. Two CONTINUOUS wheel-spin joints (world Y), no steering.
  - grid cell = wheel_config:single_axle_two_wheel × superstructure:open_bed_back_wall_shafts
- rec_four-wheeled-wooden-farm-wagon-cart-with-two-pai_20260608_164439_774589_56699604 ← picture/Urban Environment/Draft Wagon/002.png
  - 4-wheel open farm wagon: smaller steerable front pair on a REVOLUTE kingpin bolster + larger fixed rear pair, open plank box (3 side planks/side, end walls, corner posts), two forward draw poles + rope tie on the bolster. spoke_count=12. Four CONTINUOUS roll joints + one REVOLUTE front_steer (world Z).
  - grid cell = wheel_config:four_wheel_steered × superstructure:open_plank_box
- rec_four-wheeled-covered-wooden-wagon-caravan-with-a_20260608_164450_131123_085f4a98 ← picture/Urban Environment/Draft Wagon/003.png
  - 4-wheel covered caravan / vardo: same steered 4-wheel chassis + tall plank cabin walls (5 stacked boards/side) + peaked GABLED plank roof (ridge beam, two sloped panels, triangular gable end fills), single central draw pole + swingletree on the bolster. spoke_count=10. Four CONTINUOUS roll joints + one REVOLUTE front_steer.
  - grid cell = wheel_config:four_wheel_steered × superstructure:covered_gabled_cabin

## Readability / loop emission (grep notes)
- Spokes loop-emitted from a count var: `for s in range(spoke_count)` / `range(SPOKE_COUNT)` with `name=f"spoke_{s}"`, all wheels share one `_wheel_visuals` helper. GOOD.
- Floor planks loop-emitted: `n_floor` var + `for i in range(n_floor)`, `floor_plank_{i}`. GOOD.
- Side/end wall boards loop-emitted: hand-cart `for k in range(3)`/`range(4)`; caravan `n_wall=5` + `for k in range(n_wall)` `wall_board_{k}`. GOOD.
- Caravan gable boards + roof panels loop-emitted over (front/rear)×(left/right). GOOD.
- Left/right symmetry via `for side_name, ysign in (("left",1.0),("right",-1.0))`; sills, legs, shafts, chassis beams all loop-emitted. GOOD.
- Hand-written repeats (acceptable / conventional): individual wheel PARTS are instantiated by name; farm wagon + caravan use a `make_wheel(name, radius)` helper, hand cart writes left/right wheel blocks explicitly. Per-wheel joints are written out one-by-one (4 calls). These are part/joint instances, not greeble repeats — fine, but six-wheel/single-axle variants should fold the axle stations into a loop where natural.
- No FIXED-joint decorations; rope tie / swingletree / hub bands are inline parent visuals. GOOD.

## Axis / slot plan (2–4 structural axes, never color/material/pure-scale)

### Slot A — wheel_config (PRIMARY rolling topology; biggest joint-count difference)
| candidate | record_id | key part/joint/helper | structure | status |
|---|---|---|---|---|
| single_axle_two_wheel (baseline-hand) | rec_two-wheeled-wooden-hand-cart-tip-cart-dray-with-_20260608_164414_778806_b1d205ac | left_wheel/right_wheel · left_wheel_spin/right_wheel_spin (CONTINUOUS Y) · _wheel_visuals | one rear axle, 2 wheels, shafts+prop legs | parent |
| four_wheel_steered (baseline-farm) | rec_four-wheeled-wooden-farm-wagon-cart-with-two-pai_20260608_164439_774589_56699604 | front_bolster · front_steer (REVOLUTE Z) · 4× *_spin (CONTINUOUS Y) · make_wheel | steered front bolster + fixed rear axle, 4 wheels | parent |
| six_wheel_triple_axle | rec_draft_wagon_var_six_wheel_triple_axle | 6× *_spin (CONTINUOUS Y) over 3 axle stations + front_steer | three axles, 6 wheels, front pair on bolster | converged |
| single_axle_two_wheel_dray (4-wheel→2-wheel reduction onto farm lineage) | rec_draft_wagon_var_single_axle_two_wheel_dray | 2× *_spin (CONTINUOUS Y) + prop legs, no bolster | reduce to one axle / 2 wheels + pull shafts | converged |

### Slot B — spoke_count (wheel-internal multiplicity, N axis)
| candidate | record_id | key | structure | status |
|---|---|---|---|---|
| spokes_10 (baseline-caravan) | rec_four-wheeled-covered-wooden-wagon-caravan-with-a_20260608_164450_131123_085f4a98 | _wheel_visuals spoke_count=10 | 10-spoke wheels | parent |
| spokes_12 (baseline-hand/farm) | (both 12-spoke parents) | SPOKE_COUNT=12 / spoke_count=12 | 12-spoke wheels | parent |
| heavy_eight_spoke (8) | rec_draft_wagon_var_heavy_eight_spoke_wheels | spoke_count=8 in helper loop | thick 8-spoke cartwheels | converged |
| fine_sixteen_spoke (16) | rec_draft_wagon_var_fine_sixteen_spoke_wheels | spoke_count=16 in helper loop | slender 16-spoke wheels | converged |

### Slot C — side_plank / sideboard form (bed-wall multiplicity + footprint)
| candidate | record_id | key | structure | status |
|---|---|---|---|---|
| low_three_plank_rails (baseline-farm) | rec_four-wheeled-wooden-farm-wagon-cart-with-two-pai_20260608_164439_774589_56699604 | for k in range(3) side_plank_k | low 3-plank open box | parent |
| tall_back_wall (baseline-hand) | rec_two-wheeled-wooden-hand-cart-tip-cart-dray-with-_20260608_164414_778806_b1d205ac | range(3) side + range(4) back_plank | low sides + tall back wall | parent |
| high_sided_grain_box (6 boards/side) | rec_draft_wagon_var_high_sided_grain_box | range(6) side+end board loop | deep-walled grain hauler | converged |
| flat_rack_stake_bed | rec_draft_wagon_var_flat_rack_stake_bed | range(n) stake_post_i around perimeter | open deck + vertical stake posts | converged |

### Slot D — superstructure / cover (top form; one member adds a real joint)
| candidate | record_id | key | structure | status |
|---|---|---|---|---|
| open (no cover) (baseline-farm) | rec_four-wheeled-wooden-farm-wagon-cart-with-two-pai_20260608_164439_774589_56699604 | (no roof) | open box | parent |
| gabled_plank_roof (baseline-caravan) | rec_four-wheeled-covered-wooden-wagon-caravan-with-a_20260608_164450_131123_085f4a98 | ridge_beam + roof_panel + gable fills | peaked plank cabin roof | parent |
| canvas_bow_tilt_cover | rec_draft_wagon_var_canvas_bow_tilt_cover | bow_hoop_i loop + draped tilt skin | arched covered-wagon canvas top | converged |
| drop_tailgate_open_box | rec_draft_wagon_var_drop_tailgate_open_box | tailgate part · tailgate_hinge (REVOLUTE Y @ bottom rear edge) | open box + hinged drop tailgate ramp | converged |

## Combo pre-audit (HARD GATE)
candidates per slot: A=4, B=4, C=4, D=4.
distinct-N (multiplicity values realized across variants): spoke_count {8,10,12,16}, axle/wheel stations {1,2,3}, side-board count {3,4,6}, stake count + bow-hoop count = several N. distinct-N ≥ 3.
product(candidates) × distinct-N = (4 × 4 × 4 × 4) ÷ (treat as min one productive axis) — minimum honest read: take the single largest slot (4) × distinct-N (≥3) = 12 ≥ 10 ✓.
Full product Π(A·B·C·D) = 256, × distinct-N ≥ 3 = 768 ≥ 10 ✓ (HARD GATE PASS).

## Multiplicity / copy logic
- count params: spoke_count (wheel helper), axle_station_count / wheel pairs (Slot A), side_board_count + end_board_count (Slot C), stake_post_count (flat rack), bow_hoop_count (Slot D canvas), tailgate plank count.
- N samples realized: spoke_count {8,10,12,16}; axle stations {1 (dray), 2 (parents), 3 (six-wheel)}; side boards {3,4,6}; stake/hoop counts ≥2.
- suggested template N_range: spoke_count [6,16]; side_board_count [2,7]; stake_post_count [6,16]; bow_hoop_count [3,7]; axle_station_count [1,3].
- copy objects / naming / placement / joint policy: spokes, planks, boards, stakes, hoops all emitted via count-driven for-loops with name_i naming + shared geometry; wheel visuals via shared `_wheel_visuals`; per-wheel joints uniform CONTINUOUS world-Y; six-wheel/single-axle fold axle stations into a loop; steering stays single REVOLUTE world-Z on the front bolster; tailgate adds one REVOLUTE world-Y hinge at the real bottom rear edge.

## Grid coverage (3 parents occupy 3 cells; 8 variants fill empties from closest parent)
- Slot A: six_wheel_triple_axle ← farm parent; single_axle_two_wheel_dray ← hand-cart parent
- Slot B: heavy_eight_spoke_wheels ← farm parent; fine_sixteen_spoke_wheels ← caravan parent
- Slot C: high_sided_grain_box ← farm parent; flat_rack_stake_bed ← farm parent
- Slot D: canvas_bow_tilt_cover ← caravan parent; drop_tailgate_open_box ← farm parent
- 8 NEW variants (cap ~8–10). batch size = empty-cell count.

## Dropped axes (compatibility-matrix material, not structural axes here)
- color / wood-species / weathering / paint: non-structural per policy; template-side material variety only.
- pure scale (long/short bed, tall/short wheels, wide/narrow track): not a structural axis; template-side dimension params.
- decorative iron strapping / lanterns / brake levers as FIXED greebles: inline as parent visuals, never new FIXED-joint decoration parts; not opened as an axis this batch.
- seat box / driver bench: real but un-celled this batch; future Slot E (forward furniture) if revisited.
