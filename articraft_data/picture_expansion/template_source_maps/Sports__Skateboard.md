# Sports / Skateboard — template source map

pattern: parallel_children (two trucks under one deck; each truck = baseplate + revolute hanger + 2 continuous wheels) with a multiplicity sub-loop (mounting bolts per truck)

parents:
- rec_wooden-skateboard-with-a-maple-deck-two-metal-tr_20260605_165931_193582_b6b31add ← picture/Sports/Skateboard/001.png — covers: Slot A=popsicle, Slot B=cast_kingpin, Slot C=street_hard, bolts N=4 (parent fills one cell on every axis)

Single parent. Every variant forks from this parent; each changes exactly one axis off the parent baseline.

## Slot 候选覆盖

### Slot A:deck outline (`_deck_mesh` lofted board)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| popsicle (parent) | rec_wooden-skateboard-...b6b31add | deck part · `_deck_mesh` LoftGeometry stations (symmetric raw[]) · deck_board visual | symmetric double-kick popsicle, rounded nose+tail both kicked up | converged (parent baseline) |
| oldschool | rec_skateboard_var_oldschool | deck · `_deck_mesh` re-stationed · deck_board | wide flat board, squared pointed nose, single broad rear kicktail, mellow concave | built ✓ |
| longboard | rec_skateboard_var_longboard | deck · `_deck_mesh` stretched + pintail taper · deck_board; FRONT_X/REAR_X moved outboard | long drop pintail, no kicks, trucks bolt near long ends | built ✓ |
| penny | rec_skateboard_var_penny | deck · `_deck_mesh` short rounded-rect · deck_board; FRONT_X/REAR_X pulled inboard | short compact molded mini board, rounded rectangle, short tail kick only | built ✓ |

### Slot B:truck / hanger form (`_hanger_mesh` + `_baseplate_mesh`, kingpin REVOLUTE)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| cast_kingpin (parent) | rec_wooden-skateboard-...b6b31add | `{label}_baseplate` / `{label}_hanger` parts · joint `{label}_baseplate_to_hanger` (REVOLUTE kp_axis tilt inward) · `_hanger_mesh` body+neck+axle+arms | standard cast street truck, kingpin leans inward, narrow hanger | converged (parent baseline) |
| inverted_truck | rec_skateboard_var_inverted_truck | same parts/joint names · `_hanger_mesh`+`_baseplate_mesh` rebuilt, kp_axis tilt reversed (outboard) · taller hanger | reverse-kingpin longboard truck, bushing/neck on outboard side, taller hanger | built ✓ |
| wide_truck | rec_skateboard_var_wide_truck | same parts/joint names · `_hanger_mesh` widened, longer cross-axle, exposed bushing crown · WHEEL_Y track widened | gullwing/cruiser wide-spread truck, exposed bushing seat, wider wheel track | built ✓ |

### Slot C:wheel form (`_wheel_mesh` WheelGeometry + `_tire_mesh` TireGeometry, CONTINUOUS roll)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| street_hard (parent) | rec_wooden-skateboard-...b6b31add | `{label}_wheel_{side}` parts · joint `{label}_hanger_to_wheel_{side}` (CONTINUOUS axis Y) · wheel_rim/wheel_tire/roll_marker visuals · WHEEL_RADIUS/WHEEL_WIDTH | small hard narrow street wheel, conical contact, circumferential tread | converged (parent baseline) |
| cruiser_wheels | rec_skateboard_var_cruiser_wheels | same parts/joint names · `_wheel_mesh`/`_tire_mesh` larger radius+width, rounded bulge sidewall · AXLE_Z dropped to keep coplanar | big soft fat cruiser wheel, rounded bulging sidewall, larger diameter | built ✓ |
| cored_wheels | rec_skateboard_var_cored_wheels | same parts/joint names · inner hub core w/ for-i spoke windows (shared spoke helper, equal angles) + outer urethane ring · wheel_rim/wheel_tire | cored longboard wheel: hard spoked inner core + softer outer contact ring | built ✓ |

## Multiplicity / Copy Logic
- count_param: `bolt_count` (mounting bolts per truck). Parent currently emits bolts via a hardcoded nested 2x2 loop (`for sx in (-1,1): for sy in (-1,1)`), NOT a clean `for i in range(n)`. Multiplicity variants must rewrite this as a single `for i in range(bolt_count)` loop with shared `_bolt_mesh` helper, regular symmetric placement, `bolt_i` naming, bolts inlined as deck visuals (no FIXED joints).
- N 样本已覆盖: {2, 4, 6} → rec_skateboard_var_bolts2 / parent (N=4) / rec_skateboard_var_bolts6
- 模板建议 N_range: [2, 8] (real trucks are 4-bolt or 6-bolt; 2 is old-school minimal, allow up to 8 for dense/longboard hardware. N is bolts-per-truck and applies symmetrically to both trucks.)
- copied object: bolt head (`_bolt_mesh` cylinder, dark) · naming: `deck_bolt_i` / `bolt_i` · placement: regular symmetric grid over each truck baseplate footprint, proud on deck top, replicated at both FRONT_X and REAR_X · joint policy: none — bolts are inlined parent (deck) visuals, no joint per Rule 3.
- Secondary multiplicity (NOT a fork axis, but a real copy loop the template must keep): wheels ×4 (2 per truck × 2 trucks) emitted via per-truck `for s,side in ((-1,"0"),(1,"1"))`, each a CONTINUOUS roll joint. Standard skateboard N_wheels is fixed at 4; left out of the fork grid because changing it would break category readability. Template may expose wheels-per-truck but samples keep 2.

## 排除项(未来 compatibility matrix 素材)
- penny deck × wide_truck (gullwing): cruiser-wide track on a tiny penny deck likely splays wheels past the deck rails / loses category read — flag as a compatibility risk for the future matrix, not forked here.
- longboard deck × bolts6: a denser 6-bolt grid on a stretched longboard mount is geometrically fine but is a cross-axis combo, intentionally not forked (combos are the template sampler's job, not the sample pool).
- wheels-per-truck multiplicity (e.g. 1 or 3 wheels/truck): excluded — a skateboard with non-4 wheels stops reading as a skateboard (out-of-category). Kept fixed at 4.

## Notes
- All variants keep the parent's non-fixed joints: 2 REVOLUTE kingpin lean joints + 4 CONTINUOUS wheel roll joints. Articulation floor (>=1 non-fixed joint) is satisfied by construction on every cell.
- Deck-outline variants (Slot A) must keep wheel bottoms coplanar with the ground (parent run_tests asserts this) — moving FRONT_X/REAR_X or AXLE_Z requires re-checking the coplanar-wheels + wheels-lowest-part tests.
- Slot B/C variants must keep the same `{label}_baseplate_to_hanger` and `{label}_hanger_to_wheel_{side}` joint names + axes so the template can read joint semantics unchanged.

---
## Post-fork verification (SEGMENT 1 complete)
All planned variants forked via `articraft fork` (dashscope qwen3.7-max, thinking medium), then verified on-disk: last compile = success, ≥1 non-fixed joint present, collections=['workbench'] (workbench-only, not promoted), and picture.json bound into the correct `Sports__<小类>` subcat shard (reconcile rebuilt). Status cells above flipped planned→built ✓ accordingly. Ready for SEGMENT 2 (spec authoring).
