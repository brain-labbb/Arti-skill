# Door / wooden plank door with a ring pull - template source map

pattern: vertical plank field (multiplicity) + plank seam profile + face bracing + top edge profile + ring-pull hardware + clavos/stud field
parents:
- rec_door_plank_ringpull_closed (picture/Door/wooden plank door with a ring pull/001.png) - leaf shown CLOSED in a stone doorway above a step. 5 vertical planks emitted by a clean `for i in range(PLANK_COUNT)` one-box-per-plank loop (`plank_{i}`), backing board, two plain horizontal iron ledge braces (`ledge_top`/`ledge_bottom`) with hinge barrels wrapping jamb pintles, FLAT top edge, ring on a SQUARE backplate + boss (ring is its own revolute part `ring_pull`), and an iron lock plate. No clavos/stud field.
- rec_door_plank_ringpull_open (picture/Door/wooden plank door with a ring pull/002.png) - heavy ledged-and-braced leaf shown OPEN in a rough rounded-arch stone surround. 6 vertical planks via a GROOVE-CUT loop (`for i in range(1, PLANK_COUNT)` cutting V-grooves into one leaf box, not box-per-plank), two long tapered WROUGHT-IRON STRAP HINGES across the face (loop over `STRAP_LOCAL_ZS`, `strap_hinge_{idx}`) with diamond finials and three HAND-LISTED studs each (`stud_xs`, not a loop), FLAT top, ring on a 4-POINTED STAR plate + boss (own revolute part `ring_pull`), plus a sliding door bolt (`door_bolt`, prismatic) through two keepers.

Plank/ring door family: a vertical-plank leaf on a vertical (Z) hinge in a stone doorway, with a hung pull ring as a second real joint. Variants isolate plank multiplicity N, the plank seam profile, the front bracing pattern, the leaf top edge profile, the ring-pull hardware style, and the clavos/stud nail pattern. Color/material/scale never count as the change.

## Readability / loop-emission status
- Planks: LOOP-EMITTED. Closed parent = clean `plank_{i}` box-per-plank (ideal). Open parent = groove-cut loop (`range(1, PLANK_COUNT)`). Multiplicity-N variants (planks3, planks9), the v-groove profile, and the arched-top variant all keep the clean `plank_{i}` box-per-plank loop with a shared plank helper and regular placement.
- Strap studs / clavos: NOT loop-emitted in the open parent (3 hand-listed `stud_xs` per strap). The clavos/stud variants request a single `stud_{i}` (or nested `stud_{row}_{col}`) loop with a shared domed-head helper and uniform spacing. The converged `studgrid` variant does this on the leaf face (`_make_clavo_stud` helper + nested `stud_{row}_{col}` loop). The converged `strapstud` variant does it as a single-axis along-strap row: `STUDS_PER_STRAP` bolt-head studs loop-emitted as `stud_{tag}_{i}` (tag in bottom/top) at uniform spacing down each spade-tipped strap hinge.
- Bracing: ledges/straps emitted via per-element loops; the converged `zbrace` variant rewrites the ledgers as a `ledge_i` loop plus one diagonal brace board; the converged `framedbrace` adds vertical iron stiles with the ledgers as a `ledge_i` loop spanning between them.
- Joints: both parents keep a real revolute leaf hinge (jamb pintle captured) + a real revolute hung ring; open adds a prismatic slide bolt. Every variant keeps >= 1 real non-fixed joint (the leaf hinge plus the ring / thumb-lever joint).

## Slot 候选覆盖

### Slot A:plank multiplicity N (vertical plank count)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| parent_5_planks | rec_door_plank_ringpull_closed | `plank_{i}` x5 | flat leaf, 5 vertical planks (closed parent baseline) | converged |
| parent_6_planks | rec_door_plank_ringpull_open | groove-cut x6 | 6-plank leaf (open parent, groove-cut) | converged |
| three_wide_planks | rec_plank_ringpull_door_var_planks3 | `plank_{i}` x3 helper | 3 wide boards, clean box-per-plank loop | converged |
| nine_narrow_planks | rec_plank_ringpull_door_var_planks9 | `plank_{i}` x9 helper | 9 narrow boards, clean box-per-plank loop | converged |

### Slot B:front bracing pattern
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| horizontal_ledges | rec_door_plank_ringpull_closed | `ledge_top`/`ledge_bottom` | two plain horizontal ledge boards (closed baseline) | converged |
| strap_hinge_face | rec_door_plank_ringpull_open | `strap_hinge_{idx}` loop + knuckles | long tapered iron strap hinges across face (open parent) | converged |
| z_brace_ledged | rec_plank_ringpull_door_var_zbrace | `ledge_i` loop + diagonal brace | ledged-and-braced Z/N diagonal | converged |
| framed_and_braced | rec_plank_ringpull_door_var_framedbrace | vertical stiles + `ledge_i` loop | framed door: vertical iron stiles + ledgers | converged |

### Slot C:leaf top edge profile
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| flat_top | rec_door_plank_ringpull_closed | square leaf top | flat (square) top edge (both parents) | converged |
| arched_top | rec_plank_ringpull_door_var_archtop | graduated `plank_i` heights | rounded (semicircular) round-headed top | converged |
| gothic_pointed_top | rec_plank_ringpull_door_var_gothic_top | two-arc pointed trim | pointed lancet (gothic) top | converged |
| shouldered_camber_top | rec_plank_ringpull_door_var_shoulder_top | shoulder-cut camber trim | chamfered-shoulder cambered head | converged |

### Slot D:ring-pull / handle hardware style
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| ring_on_square_plate | rec_door_plank_ringpull_closed | `ring_pull` + square boss/backplate | plain hung pull ring on square plate (closed baseline) | converged |
| ring_on_star_plate | rec_plank_ringpull_door_var_starplate | `ring_pull` + star escutcheon | hung ring on forged four-pointed star quatrefoil plate | converged |
| knocker_ring | rec_plank_ringpull_door_var_knocker_ring | `ring_pull` + `strike_boss` | knocker ring that swings down against a strike boss | converged |
| thumb_latch | rec_plank_ringpull_door_var_thumb_latch | `thumb_lever` revolute part | Suffolk thumb-latch grip + pivoting lever | converged |

### Slot E:clavos / stud nail pattern
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| no_clavos | rec_door_plank_ringpull_closed | (none on face) | bare plank face, no nail field (closed baseline) | converged |
| clavos_grid | rec_plank_ringpull_door_var_studgrid | `stud_{row}_{col}` grid loop + `_make_clavo_stud` | regular row/col domed clavos grid across leaf face | converged |
| strap_studs_row | rec_plank_ringpull_door_var_strapstud | `stud_{tag}_{i}` along-strap loop (`STUDS_PER_STRAP`=6) | even bolt-head stud row riveting each spade-tipped strap hinge | converged |

### Slot F:plank seam profile (secondary appearance axis)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| flat_butted | rec_door_plank_ringpull_closed | `plank_{i}` flat box | flat butted boards (closed baseline) | converged |
| v_groove | rec_plank_ringpull_door_var_vgroove | `plank_{i}` + `_v_groove_plank_mesh` | chamfered V-jointed tongue-and-groove seam edges | converged |

## Multiplicity / Copy Logic
- count_param: `plank_count` (vertical plank field). Secondary local counts: `stud_count` (clavos grid / strap stud row), `ledge_count` / stile count (bracing), `strap_count` (strap-hinge face).
- N 样本已覆盖: plank_count = {3, 5(parent), 6(parent), 9} -> 4 distinct N values (covers small wide-board through dense narrow-board).
- 模板建议 N_range: plank_count [3, 9] with even regular placement across a fixed leaf width; stud_count derived as a row/col grid or along-ledger row over the plank field.
- copied object / naming / placement / joint policy: planks emitted as `plank_{i}` with a shared box-per-plank helper at regular even spacing; clavos as `stud_{i}` / `stud_{row}_{col}` with a shared domed-head helper at uniform grid/row spacing; ledgers/straps as `ledge_i`/`strap_{i}` with a shared helper at regular heights, each terminating in the same hinge knuckle. Uniform joint policy: the leaf hinge stays one revolute, the ring/thumb-lever stays one revolute; decorations stay inlined visuals (no FIXED-joint parts).

## 组合数预审 (HARD GATE)
Per-slot CONVERGED-on-disk candidate counts: Slot A=4, Slot B=4, Slot C=4, Slot D=4, Slot E=3, Slot F=2.
Every slot has >= 2 candidates. ✓ (no single-candidate slot.)
Distinct multiplicity N (plank_count) = {3, 5, 6, 9} = 4 (2-3+ distinct N requirement satisfied). ✓
Core combo (A x B x C x D x E x F, converged) = 4 x 4 x 4 x 4 x 3 x 2 = 1536.
1536 x distinct-N(4) = 6144 >= 10. ✓ GATE P1 met.
(The four former gap-fill candidates — gothic_top, shoulder_top, knocker_ring, thumb_latch — are now forked and converged on disk.)

## 排除项(未来 compatibility matrix 素材)
- arched_top / gothic_top / shouldered_camber_top assume a flat-square stone head clearance; the rounded-arch surround already accommodates a curved leaf top, so these tops pair best with the arched surround. Keep the leaf top below the stone arch soffit.
- strap_hinge_face bracing (Slot B) overlaps in iron real-estate with strap_studs_row (Slot E): combine only by applying the stud row to the new straps, not stacking a separate ledge.
- thumb_latch (Slot D) removes the hung ring; pairing it with knocker_ring or the star/square ring plates is mutually exclusive (one hardware style at a time).
- clavos_grid / strap_studs_row must route around the ledge boards and the ring/lock plates; keep studs clear of hardware footprints.
- N=9 narrow planks leave little width per board; keep clavos grid columns aligned to plank centers, not seams, at high N.
- v_groove (Slot F) is a plank-seam appearance axis only; it composes freely with any N and any bracing/top/hardware as long as the `plank_{i}` box-per-plank loop is preserved.
