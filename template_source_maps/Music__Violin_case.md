# Music / Violin_case — template source map

pattern: mixed (linear_chain shell↔lid hinge core + parallel_children carry/interior/closure additive parts + multiplicity on latch_count)
parents: rec_hardshell-violin-case-with-a-hinged-clamshell-li_20260605_161837_673361_7727811a ← picture/Music/Violin case/001.png

## Slot 候选覆盖

### Slot A:shell_footprint
Defines the bottom_shell outer/recess outline + the matching lid outline. The violin-vs-rectangular profile swaps the `_violin_outline_wire`/`_violin_half_points` helpers for `_rounded_rect_solid`; everything downstream (recess inset, red liner scaling, lid shell, hinge knuckle seating) keys off the same outline helper.

| 候选(future module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| violin_contour (baseline) | rec_hardshell-violin-case-with-a-hinged-clamshell-li_20260605_161837_673361_7727811a | bottom_shell / bottom_exterior / red_interior; helpers `_violin_outline_wire`,`_violin_half_points`,`_half_width_at_x`,`_bottom_shell_solid` | spline violin silhouette (scroll→upper bout→C-bout waist→lower bout→tail), recess hollowed by inset spline, hinge knuckles seated at `_half_width_at_x` rim | baseline |
| rounded_dart_taper | rec_violin_case_var_outline_dart | bottom_shell / bottom_exterior / red_interior (name="violin_case_dart") | rounded dart taper footprint variant of the violin spline | converged(已同步) |
| rectangular_oblong | rec_violin_case_var_outline_rectangular | bottom_shell / bottom_exterior; helper `_rounded_rect_solid`, `CORNER_R` fillet | rounded-rectangle tub + rect recess; hinge knuckles seated at fixed `HALF_W` (no width interp); test asserts oblong ratio | converged(已同步) |

### Slot B:closure_mechanism (carries the lid↔body kinematics)
This slot owns the real opening kinematics. Baseline = clamshell hinge (`bottom_to_lid` REVOLUTE 0..180°) + 2 metal flip latches (the latch parts are also the Multiplicity below). Zipper replaces the rigid latches with a PRISMATIC slider on a fabric track; buckle straps add a 2-link strap→buckle chain off the lid.

| 候选(future module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| hinge_plus_flip_latches (baseline) | rec_hardshell-violin-case-with-a-hinged-clamshell-li_20260605_161837_673361_7727811a | lid / `bottom_to_lid` REVOLUTE; latch_i / latch_body_i / `bottom_to_latch_i` REVOLUTE 0..80° | clamshell lid folds flat to +Y at 180°; 2 metal hook-lip levers flip outward on front (-Y) rim | baseline |
| zipper_perimeter | rec_violin_case_var_closure_zipper | lid + zipper_track / zipper_tooth visuals; zipper_pull / pull_body / `bottom_to_zipper_pull` PRISMATIC; lid_padding (name="violin_case_zipper") | soft-style fabric zipper cord around 3 rim sides, slider translates along front seam; lid still hinges | converged(已同步) |
| buckle_straps | rec_violin_case_var_closure_buckle_straps | strap_i / strap_band_i / `lid_to_strap_i` REVOLUTE 0..60°; buckle_i / buckle_frame_i / `strap_to_buckle_i` REVOLUTE 0..40°; bottom catch_plate_i; leather material | 2-link leather strap→metal buckle chain hinged off lid front edge, wraps over front rim onto catch plates | converged(已同步) |

### Slot C:carry_hardware (additive moving/optional grab parts)
Optional grab hardware bolted to bottom_shell (or lid). Parent has NO carry handle. Each candidate adds parts + REVOLUTE joints off bottom_shell; static mount feet/plates are inline non-moving visuals on bottom_shell.

| 候选(future module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| none (baseline) | rec_hardshell-violin-case-with-a-hinged-clamshell-li_20260605_161837_673361_7727811a | (no carry part) | parent default has no carry handle/strap hardware | baseline |
| single_top_handle | rec_violin_case_var_carry_top_handle | carry_handle / handle_bar / `bottom_to_handle` REVOLUTE | one curved top grab bar on a swinging revolute pivot | converged(已同步) |
| dual_side_handles | rec_violin_case_var_carry_dual_side_handles | handle_i / handle_bar_i / `bottom_to_handle_i` REVOLUTE 0..90°; handle_mount_i_j feet; helper `_handle_assembly`,`tube_from_spline_points` | 2 wall-contoured grab bars (one per long side), pivot stubs in mount-foot bosses, swing outward | converged(已同步) |
| d_ring_strap_loops | rec_violin_case_var_carry_shoulder_strap_loops | d_ring_i / dring_ring_i / `bottom_to_d_ring_i` REVOLUTE; dring_mount_plate_i, dring_pivot_bar_i; helper `_d_ring_mesh` | 2 D-ring shoulder-strap loops on pivot bars/mount plates, swing about X | converged(已同步) |

### Slot D:interior_fitting
What lives inside the cavity. Baseline = plain molded red plush recess only (`_red_interior_solid`: floor pad + raised wall lip). Candidates add a static cradle, jointed spinner bow clips, or a lidded pocket.

| 候选(future module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| plain_plush_recess (baseline) | rec_hardshell-violin-case-with-a-hinged-clamshell-li_20260605_161837_673361_7727811a | bottom_shell.red_interior; helper `_red_interior_solid` | violin-shaped plush floor pad + raised cavity-wall lip, no fittings | baseline |
| neck_cradle | rec_violin_case_var_interior_neck_cradle | bottom_shell.neck_cradle visual; helper `_neck_cradle_solid` | static molded suspension neck cradle block inside the bottom cavity (no joint) | converged(已同步) |
| bow_spinner_clips | rec_violin_case_var_interior_bow_clips | bow_clip_i / clip_arm_i / `lid_to_bow_clip_i` REVOLUTE 0..90°; lid clip_mount_i, bow_stick, bow_frog, bow_cradle_pad_i; helper `_bow_clip_arm_solid` | 2 spinner retainer clips mounted in lid cavity capturing a stored bow stick; clips swivel about Z | converged(已同步) |
| accessory_pocket | rec_violin_case_var_interior_accessory_pocket | pocket_lid / pocket_lid_panel / `bottom_to_pocket_lid` REVOLUTE; bottom pocket_box, pocket_pad visuals | lidded accessory compartment box in cavity with a hinged flip pocket lid | converged(已同步) |

## Multiplicity / Copy Logic
- count_param: latch_count ; applies only when Slot B = hinge_plus_flip_latches (the latch family); zipper/buckle closures supply their own fixed closure count.
- N 样本已覆盖: {2, 3, 4} → parent(=2) / rec_violin_case_var_latch_count_3 / rec_violin_case_var_latch_count_4
- 模板建议 N_range: [2, 4]
- copied object: the `latch_i` part (lever plate + hook lip + pivot pin, built by the per-i loop) plus its `bottom_to_latch_i` REVOLUTE joint.
- naming: `latch_{i}` / visual `latch_body_{i}` / joint `bottom_to_latch_{i}`, i = 0..N-1.
- placement: evenly spaced `latch_positions` X-list along the front (-Y) rim; pivot Y = `-_half_width_at_x(lx)+0.002` so each seats on the contoured front wall at that x; spread widens from lower-bout pair (N=2) to upper→lower-bout span (N=4).
- joint policy: each copy is an independent REVOLUTE on bottom_shell, axis=(1,0,0), 0..80°, flips outward (-Y); allow_overlap latch↔bottom (pin seat) and latch↔lid (hook clamp) replicated per copy.

## 排除项
- latch_count multiplicity × non-latch closures (zipper / buckle_straps): excluded — those closures replace the flip latches entirely, so latch_count is N/A there (set N to the closure's own native count).
- bow_spinner_clips interior is lid-mounted (`lid_to_bow_clip_i`); it rides on the lid and assumes a violin-contour lid cavity — pairing with a heavily rectangular footprint or zipper-soft lid is structurally weaker (lid liner geometry differs), so prefer it with violin_contour + hinge closure.
- Otherwise no combos identified that would fail to converge: shell_footprint, carry_hardware, and interior_fitting are largely orthogonal and additive.
