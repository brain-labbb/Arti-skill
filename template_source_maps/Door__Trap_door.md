# Door / Trap door — template source map

pattern: single-parent fork (one converged parent + 10 converged single-axis workbench fork variants on disk)

slug: trap_door
shard: Trap_door
picdir: picture/Door/Trap door (note: space in dir name)

parents:
- rec_door_trapdoor <- picture/Door/Trap door/001.png (round cast-iron rear-hinged floor-access
  hatch: recessed cross-wheel relief + 12 rim bolts, square diamond-mesh collar over a round
  concrete well shaft). Occupies Slot A=solid_cast_slab, Slot B=single_revolute_flap,
  Slot C=round_hatch, Slot D=cross_wheel_relief (no separate pull).

Trap door family with a separable leaf surface/fill, hinge mechanism, leaf footprint shape, and
top grip/pull, all riding on a fixed square diamond-mesh collar + hollow round concrete well-shaft
support. Variants isolate the leaf fill, the opening mechanism, the leaf plan, and the pull while
retaining at least one real non-fixed joint (the hatch always opens). The parent already loop-emits
its rim bolts (`for i in range(N_BOLTS)`) and cross-wheel spokes (`for i in range(N_SPOKES)`); the
multiplicity + bi-fold candidates extend that loop pattern with `plank_{i}` / `slat_{i}` /
`leaf_{i}` (half-leaves) over shared geometry helpers.

NOTE: The converged on-disk set is exactly the 10 ids in /tmp/ids_Trap_door.txt plus the parent.
This file is the authoritative refresh: it lists ONLY ids that actually exist on disk. (The previous
version of this file described several "planned" forks — glazed / liftout / bifoldplank / rect /
grateround / plankround / grate6 — that were never created; the real converged set instead includes
biparting, foldhandle, and raisedcurb, classified below.) No reference uses line numbers; all
references are by part / joint / helper name.

## 组合数预审 (GATE P1)

Every slot has >= 2 candidates; multiplicity has >= 3 distinct N:
- Slot A (leaf surface/fill): {solid_cast_slab, checker_plate_steel, planked_deck, barred_grate} = 4 candidates (>=2 OK)
- Slot B (hinge mechanism): {single_revolute_flap, double_bifold} = 2 candidates (>=2 OK)
- Slot C (leaf footprint): {round, square, rectangular} = 3 candidates (>=2 OK)
- Slot D (grip / pull): {cross_wheel_relief / none, ring_pull, rope_loop, fold_handle} = 4 candidates (>=2 OK)
- Multiplicity N (plank / grate-bar count): distinct N = {4, 6, 9 planks; 12 grate slats} = 4 distinct values (>=3 distinct OK; also N_LEAVES=2 bifold, N_CURB_WALLS=4 curb)

Combo number = product(primary independent slots A x B x C) x distinct-N
             = 4 x 2 x 3 x (>=3) = 72 >= 10  -> GATE P1 MET.
(Even the minimal product A x B = 8 times distinct-N 3 = 24 >= 10.) No slot is single-candidate.

## Slot 候选覆盖

### Slot A:leaf surface / fill
| 候选(未来 module) | record_id | 关键 part·joint·helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| solid_cast_slab (基线) | rec_door_trapdoor (parent) | lid_disc / lid_relief / lid_knuckle / collar_to_lid(REVOLUTE) / _build_lid_body_mesh + _build_lid_relief_mesh + rim-bolt loop (N_BOLTS=12) | solid cast-iron disc slab with recessed cross-wheel relief (N_SPOKES=4) + 12 rim bolts | converged (parent) |
| solid_cast_slab (ring-pull skin) | rec_trapdoor_var_ringpull | lid_disc / ring_pull (torus) / lid_knuckle / collar_to_lid(REVOLUTE) / _build_ring_pull_mesh | same solid cast disc; cross-wheel replaced by a flush recessed ring-pull torus | converged (variant) |
| solid_cast_slab (rope-loop skin) | rec_trapdoor_var_ropeloop | lid_disc / rope_loop / eyelet_{i} loop (N_EYELETS=2) / _build_rope_loop_mesh + _build_eyelet_mesh | same solid cast disc; hemp rope loop arched through 2 eyelets | converged (variant) |
| solid_cast_slab (fold-handle skin) | rec_trapdoor_var_foldhandle | lid_disc / lid_pocket / handle_lug_{i} loop (N_HANDLE_LUGS=2) / handle_bar / lid_to_handle(REVOLUTE) | same solid cast disc; recessed pocket + folding bar handle on its own revolute | converged (variant) |
| checker_plate_steel | rec_trapdoor_var_checkerplate | hatch_plate / hatch_knuckle / collar_to_hatch_leaf(REVOLUTE) / _build_hatch_plate_mesh (diamond-tread grid nx*ny + 3 folded edge lips) | solid square steel checker-plate leaf with raised diamond tread + folded lips | converged (variant) |
| planked_deck | rec_trapdoor_var_plank4, rec_trapdoor_var_plank6, rec_trapdoor_var_plank9 | lid / plank_{i} loop (N_PLANKS) + batten_{j} loop (N_BATTENS=2) / lid_knuckle / collar_to_lid(REVOLUTE) / _board(_geometry) | square timber leaf filled by N edge-to-edge planks banded by 2 cross battens | converged (variant x3) |
| barred_grate | rec_trapdoor_var_grate | grate_frame / slat_{i} loop (N_SLATS=12) + grate_knuckle / collar_to_grate(REVOLUTE) / _slat_bar_geometry + _build_grate_frame_mesh | open rectangular grate of N see-through parallel slat bars in a 4-bar border frame | converged (variant) |

### Slot B:hinge mechanism
| 候选(未来 module) | record_id | 关键 part·joint·helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| single_revolute_flap (基线) | rec_door_trapdoor (parent) | collar_to_lid(REVOLUTE, hinge along rear rim) + hinge_mount lugs + lid_knuckle | one rear-hinged flap swings up off the throat | converged (parent) |
| double_bifold (cast round) | rec_trapdoor_var_biparting | leaf_{i} loop (N_LEAVES=2; per-leaf body/bolts/seat/relief/knuckle) / collar_to_leaf_0(REVOLUTE axis=(-1,0,0)) + collar_to_leaf_1(REVOLUTE axis=(+1,0,0)) / _build_leaf_body_cq + _semicircle_profile + _half_cutter | two semicircular half-leaves meet at center seam, swing apart on opposite-sign axes | converged (variant) |

(Also note: rec_trapdoor_var_foldhandle adds a SECOND revolute — lid_to_handle — nested on the single
revolute flap; that is a Slot D grip joint, not a distinct main hinge mechanism.)

### Slot C:leaf footprint shape
| 候选(未来 module) | record_id | 关键 part·joint·helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| round_hatch (基线) | rec_door_trapdoor (parent) | lid_disc (LID_R) over circular COLLAR_THROAT_R | round disc leaf over circular throat | converged (parent) |
| round_hatch (other skins) | rec_trapdoor_var_ringpull, _ropeloop, _foldhandle, _biparting, _raisedcurb | lid_disc / leaf_{i} (LID_R) | round footprint reused across ring/rope/fold/bifold/curb variants | converged (variant) |
| square_hatch | rec_trapdoor_var_checkerplate, rec_trapdoor_var_plank4/6/9 | hatch_plate / lid (LEAF_SIZE / LEAF_SIDE square) over square throat | square steel/timber leaf over a square throat opening | converged (variant) |
| rectangular_hatch | rec_trapdoor_var_grate | grate_frame (GRATE_W=0.66 x GRATE_D=0.70 oblong) | rectangular/oblong grate leaf over a rectangular clear span | converged (variant) |

### Slot D:grip / pull
| 候选(未来 module) | record_id | 关键 part·joint·helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| cross_wheel_relief / none (基线) | rec_door_trapdoor (parent) | lid_relief (inlined visual, no joint) | recessed cross-wheel relief, no separate movable pull | converged (parent) |
| recessed_ring_pull | rec_trapdoor_var_ringpull | ring_pull (torus inlined in shallow pocket) | flush recessed ring-pull torus | converged (variant) |
| rope_loop_pull | rec_trapdoor_var_ropeloop | rope_loop + eyelet_{i} (N_EYELETS=2) | hemp rope loop anchored through 2 eyelet grommets | converged (variant) |
| folding_bar_handle | rec_trapdoor_var_foldhandle | handle_bar on lid_to_handle(REVOLUTE) + handle_lug_{i} (N_HANDLE_LUGS=2) + handle_hinge_pin | folding metal bar handle that lays into a pocket; its own real revolute joint | converged (variant) |

### Support / curb sub-axis (collar coaming)
| 候选(未来 module) | record_id | 关键 part·joint·helper 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| flush_mesh_collar (基线) | rec_door_trapdoor (parent) + most variants | collar_frame (square diamond-mesh) FIXED to shaft top / _build_collar_mesh | flush square diamond-mesh collar plate over round throat | converged (parent) |
| raised_rect_kerb_curb | rec_trapdoor_var_raisedcurb | curb_body (4 wall sections, N_CURB_WALLS=4) FIXED / curb_to_lid(REVOLUTE) / _curb_wall_section + _build_curb_mesh | raised rectangular kerb coaming around the throat; lid hinges off the kerb rear wall | converged (variant) |

## Multiplicity / Copy Logic
- count_param: plank_count (N_PLANKS), grate_slat_count (N_SLATS), bifold_leaf_count (N_LEAVES),
  curb_wall_count (N_CURB_WALLS). All copied sub-parts use a for-i-in-range(n) loop with name_{i}
  naming over a shared geometry helper, regular pitch, and a uniform joint policy.
- N samples already on disk: N_PLANKS {4, 6, 9} (plank4/plank6/plank9), N_SLATS {12} (grate),
  N_LEAVES {2} (biparting), N_CURB_WALLS {4} (raisedcurb), N_EYELETS {2} (ropeloop),
  N_HANDLE_LUGS {2} (foldhandle), N_BOLTS {12} (parent rim-bolt loop). Distinct leaf-fill N = {4,6,9,12}.
- copied object / naming / placement / joint policy:
  - planked_deck: copy = plank_{i} (edge-to-edge boards) + batten_{j}; planks inlined as lid
    visuals (no per-plank joint), single collar_to_lid REVOLUTE.
  - barred_grate: copy = slat_{i} (parallel see-through bars at SLAT_PITCH); inlined as grate
    visuals, single collar_to_grate REVOLUTE.
  - double_bifold: copy = leaf_{i} (half-discs); EACH leaf gets its own collar_to_leaf_{i} REVOLUTE
    with opposite-sign axis (the only candidate where the loop emits a real joint per copy).
  - raised_rect_kerb_curb: copy = 4 curb wall sections inlined into one curb_body visual (FIXED).
- 模板建议 N_range: plank_count [4, 12]; grate_slat_count [6, 20]. These N only change leaf-fill
  geometry/density and are emitted as leaf visuals via the shared helper, never new movable joints
  (except bifold N_LEAVES which is structurally fixed at 2).

## 格子覆盖 (1 parent + 10 variants)
- Slot A leaf fill: 4 candidates covered (solid_cast_slab parent + 3 skins; checker_plate;
  planked_deck x3 N; barred_grate).
- Slot B hinge mechanism: 2 candidates covered (single_revolute_flap parent; double_bifold biparting).
- Slot C footprint: 3 candidates covered (round parent + skins; square checker/plank; rectangular grate).
- Slot D grip/pull: 4 candidates covered (cross-wheel parent; ring_pull; rope_loop; fold_handle).
- Support sub-axis: 2 candidates (flush mesh collar; raised rect kerb curb).
All planned slots have >=2 candidates and the combo number is 72. Trap door
小类 sample pool is ready; NO gap forks required.

## 排除项 (未来 compatibility matrix 素材)
- color / material (cast-iron / steel / timber / hemp / concrete tints): non-structural axis,
  handled template-side as params/materials, not a slot.
- pure dimensional scaling (leaf radius/side, shaft depth): non-structural, template scale param.
- liftout-prismatic lid, glazed skylight pane, square diamond-mesh leaf-fill: real-world plausible
  but NOT on disk; each would be a future single-axis fork if a wider pool is wanted. The current
  pool already clears GATE P1 without them, so they are deferred, not gap forks.
- raised_rect_kerb_curb is a support/coaming sub-axis change, not a leaf change; in principle it can
  combine with any Slot A/B/C leaf but is sampled here only with the round solid revolute lid.

## Broken variants
None. All 10 converged variant model.py files plus the parent read cleanly with the expected
slot-bearing parts, loops, and at least one real non-FIXED joint each (parent + 8 single-revolute,
biparting = 2 revolute leaves, foldhandle = lid revolute + handle revolute).

## Gap forks
NONE. No slot is single-candidate (min = Slot B at 2), distinct-N >= 3, combo = 72 >= 10.
/tmp/manifest_final_trap_door.tsv is written EMPTY.
