# Door / Gate - template source map

pattern: single-parent fan-out
slug: gate
shard: Gate
parents:
- rec_door_gate <- picture/Door/Gate/001.png (ornate wrought-iron double-leaf entrance gate in an arched stone surround with a fixed decorative arched fanlight grille; two leaves swing outward toward -Y on vertical REVOLUTE hinges, door_1 mimic-coupled to door_0). Covers Slot A=ornamental_scroll_infill, Slot N=picket_count(7), Slot B=straight_top, Slot C=plain_rails_frame, Slot D=double_leaf.

Door/Gate family: a double-leaf hinged iron gate in a fixed masonry surround. Variants isolate the
leaf infill pattern, the vertical-element multiplicity, the picket-top / top-rail profile, the
framing/brace topology, and the leaf count, while retaining the parent's outward-swinging
mirror-coupled REVOLUTE leaves and fixed fanlight. Pickets are already loop-emitted in the parent
(`for i in range(N_BARS)` inside `_leaf_iron`).

## Slot 候选覆盖

### Slot A:leaf infill pattern
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| ornamental_scroll_infill | rec_door_gate (parent) | door_0/door_1 iron leaves, `_leaf_scroll_iron`, `_scroll_panel_iron`, `_volute`, door_*_scrolls | dense C-scroll + volute + oval-boss panels overlaid on a light picket field | parent (converged) |
| vertical_picket_infill | rec_gate_var_plainbars | `_leaf_iron`, `for i in range(N_BARS)` picket loop (scrollwork helpers removed) | plain edge-to-edge field of straight vertical pickets, no scrollwork | converged |
| panel_and_bar_infill | rec_gate_var_panelinfill | `_leaf_iron` solid kick panel (bottom third) + `for i in range(N_BARS)` upper bar field | solid flat iron kick panel across the bottom third with vertical bars in the upper field | converged |

### Slot N:vertical-element multiplicity (picket count)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| N=7 (parent baseline) | rec_door_gate (parent) | `N_BARS = 7`, `for i in range(N_BARS)` in `_leaf_iron` | seven loop-emitted pickets per leaf | parent (converged) |
| N=11 dense | rec_gate_var_n11 | `N_BARS = 11`, same picket loop / helper | eleven evenly spaced pickets per leaf (dense field) | converged |
| N=5 sparse | rec_gate_var_n5 | `N_BARS = 5`, same picket loop / helper | five evenly spaced pickets per leaf (sparse field) | converged |

### Slot B:picket-top / top-rail profile
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| straight_top | rec_door_gate (parent) | flat head/mid rails in `_leaf_iron`; arched stone fanlight head on surround | straight flat top rail, square picket heads (arched fanlight is surround, not leaf head) | parent (converged) |
| straight_flat_rail_head | rec_gate_var_flatrail | `_top_rail`, flat lintel surround (no arch / no fanlight); `TOP_RAIL_H` / `TOP_RAIL_D` | plain rectangular surround with a flat horizontal iron top rail across the opening head | converged |
| spear_pointed_tops | rec_gate_var_speartop | `_spear_finial` helper, `for i in range(N_BARS)` finial loop (`finial`), open spear-topped head (no fanlight) | each picket capped by a tapered cast spear-tip finial above the rail; row of spear heads | converged |
| arched_cambered_top | — | (curved head-rail via lathe/CadQuery; curve-following picket heights) | convex cambered top rail; closed pair forms a continuous arch | not present (out of scope; Slot B already >=2) |

### Slot C:framing / brace topology
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| plain_rails_frame | rec_door_gate (parent) | `_leaf_iron` perimeter stile-and-rail frame + horizontal rails | rectangular perimeter frame with horizontal rails only | parent (converged) |
| z_brace_diagonal | rec_gate_var_zbrace | `_leaf_iron` top + bottom ledger rails + continuous corner-to-corner diagonal brace bar (shared brace helper) | classic Z-brace ledged-and-braced leaf (top rail, bottom rail, corner-to-corner diagonal) | converged (gap-fill; closes Slot C; compile success, 2 revolute leaf joints) |
| ring_and_arch_brace | — | (circular ring annulus via lathe/CadQuery fused to frame) | large structural decorative ring set into the leaf | not present (not needed for Slot C >=2) |

### Slot D:leaf count
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| double_leaf | rec_door_gate (parent) | door_0 + door_1, surround_to_door_0 / surround_to_door_1 REVOLUTE, door_1 mimic-coupled | two mirror-coupled leaves swinging outward | parent (converged) |
| single_leaf | rec_gate_var_single | one `gate_leaf`, single `surround_to_leaf` REVOLUTE Z hinge, latch stile at the opposite jamb | one wide leaf hinged at one jamb, latch stile meeting the opposite jamb (mimic removed) | converged |

## Multiplicity / Copy Logic
- count_param: `N_BARS` (parent picket count); reused verbatim by every infill / top-profile / frame variant.
- N 样本已覆盖: `N_BARS` {5 (rec_gate_var_n5), 7 (parent), 11 (rec_gate_var_n11)} = 3 distinct values.
- 模板建议 N_range: [5, 11] for the picket / bar field; the panel-and-bar variant keeps the bar loop above the solid kick panel; the closed boarded / lattice infill branch is out of the current converged set.
- copied object / naming / placement / joint policy: repeated vertical elements are loop-emitted as picket/bar via `for i in range(N_BARS)` in `_leaf_iron` with a shared box helper and uniform even spacing recomputed from the count; spear finials reuse the same index loop (`_spear_finial` per picket). All copies are inlined leaf visuals (no per-picket joints). The only non-fixed joints are the two `surround_to_door_*` REVOLUTE hinges (door_1 mimic-coupled to door_0), collapsed to a single `surround_to_leaf` REVOLUTE in the single-leaf variant.

## 组合数预审
Per-slot candidate counts (converged + the one NEW fork):
Slot A(3) x Slot B(2) x Slot C(2, with rec_gate_var_zbrace) x Slot D(2) x Slot N(3 distinct) = 72 >> 10 ✓.
Even the minimal Slot A(3) x Slot D(2) x Slot N(3) = 18 >= 10 ✓.
GATE P1: every slot >=2 candidates — A=3, B=2, C=2 (after zbrace), D=2; multiplicity = 3 distinct N (5/7/11). Met.
One parent fills cell {ornamental_scroll_infill, N=7, straight_top, plain_rails_frame, double_leaf}; the seven existing
converged variants fill the other axes; ONE NEW variant (rec_gate_var_zbrace) closes the only deficient slot (Slot C
had a single candidate before the fork).

## 排除项(未来 compatibility matrix 素材)
- panel_and_bar_infill keeps a closed solid kick panel across the bottom third, so its bottom band is incompatible with a full-height picket-count reading or with see-through ring/lattice brace motifs in that band; treat the kick panel as its own lower branch.
- arched_cambered_top (not present) would change the head-rail curve; combine with picket / plain-bar infill but not directly with the closed kick panel without re-profiling the panel top.
- ring_and_arch_brace (not present) is a heavier structural ring; not needed for Slot C >=2 and would compete with the open scrollwork field.
- All double-leaf variants must retain the two outward-swinging REVOLUTE leaves with the door_1->door_0 mimic coupling; the single-leaf variant retains exactly one real REVOLUTE Z hinge. Do not convert the hinge to FIXED.
- The fixed fanlight grille + gold transom stays a parent visual on the surround in every variant that keeps the arch (parent, n5, n11, plainbars, panelinfill, single); flatrail and speartop intentionally drop the fanlight as part of their targeted head-profile change.
