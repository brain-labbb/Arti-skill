# Door / Double Door — template source map

pattern: mixed

parents:
- rec_classic-double-entry-door-in-dark-walnut-wood-tw_20260606_115214_146776_3b44fd42 <- picture/Door/Double Door/001.png (classic dark-walnut two-leaf raised-panel entry door). Covers Slot A=raised_panel_leaf, Slot B=flat_head, Slot C=both_revolute_opposite, Multiplicity N=3 raised panels.
- rec_commercial-hospital-double-door-in-off-white-ste_20260606_120526_255019_1fe7f29a <- picture/Door/Double Door/002.png (off-white steel hospital door, vision window + push bar + bumper stripes). Covers Slot A=vision_window_pushbar_leaf, Slot B=flat_head, Slot C=both_revolute_opposite.
- rec_ornate-double-door-in-honey-brown-wood-with-a-da_20260606_115210_265714_26551efa <- picture/Door/Double Door/003.png (honey-brown wood, raised-molding central circle + dark inset, sidelights + transom). Covers Slot A=carved_circle_motif_leaf, Slot B=transom_over_flat (with sidelights), Slot C=both_revolute_opposite.
- rec_arched-carriage-style-double-door-in-warm-wood-t_20260606_115230_910775_7b8384ab <- picture/Door/Double Door/004.png (warm-wood carriage door, upper arched divided glass + lower X-brace boards, stone arch surround). Covers Slot A=upper_glass_muntin_lower_xbrace_leaf, Slot B=arched_head, Slot C=both_revolute_opposite, Multiplicity = muntin/divided-lite count.
- rec_saloon-style-swinging-cafe-double-doors-batwing-_20260606_115237_933064_1a02f0ce <- picture/Door/Double Door/005.png (saloon batwing louvered cafe doors, scalloped crown, double-acting spring). Covers Slot A=louvered_slat_leaf, Slot B=scalloped_crown_head, Slot C=double_acting_spring.
- rec_aluminum-storefront-double-door-two-narrow-stile_20260606_115902_647305_38236092 <- picture/Door/Double Door/006.png (anodized-aluminum narrow-stile storefront, full single glass pane, diagonal push bars). Covers Slot A=full_glass_single_pane_leaf, Slot B=flat_head, Slot C=both_revolute_opposite.

Double-door family with a fixed frame/surround root carrying two mirror-symmetric operable leaves. The
separable structural layers are the leaf infill style, the head/top profile of the frame, the swing
mechanism of the two leaves, and the per-leaf multiplicity of stacked raised panels or divided glass
lites. Every leaf is authored once in a leaf-local frame and the second leaf is the X-mirror (sign or
mirror helper); both leaves keep at least one real revolute (or double-acting spring-pivot) joint. All
six parents and all eight workbench variants below are converged; the slot grid is fully covered and no
gap-fill forks were required.

## Slot 候选覆盖

### Slot A:leaf infill style
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| raised_panel_leaf | rec_classic-double-entry-door-in-dark-walnut-wood-tw_20260606_115214_146776_3b44fd42 (parent) | _build_leaf / door_{idx}_leaf, frame_to_door_{idx} (revolute z) | stacked fielded raised panels with bolection molding cut+union into leaf | converged (parent) |
| vision_window_pushbar_leaf | rec_commercial-hospital-double-door-in-off-white-ste_20260606_120526_255019_1fe7f29a (parent) | _leaf_body / _glass_pane / _push_bar, door_leaf_{n}_* | cut vision window + glass + stainless push bar + bumper stripes | converged (parent) |
| carved_circle_motif_leaf | rec_ornate-double-door-in-honey-brown-wood-with-a-da_20260606_115210_265714_26551efa (parent) | _leaf_body / _leaf_inset / _handle, door_body_{idx} | raised molding half-ring forming a central circle + dark recessed inset disc | converged (parent) |
| upper_glass_muntin_lower_xbrace_leaf | rec_arched-carriage-style-double-door-in-warm-wood-t_20260606_115230_910775_7b8384ab (parent) | _leaf_frame_and_panels, door_frame_{idx}/door_glass_{idx}/door_xbrace_{idx} | upper divided glass over lower X-braced ledged boards | converged (parent) |
| louvered_slat_spring_leaf | rec_saloon-style-swinging-cafe-double-doors-batwing-_20260606_115237_933064_1a02f0ce (parent) | _louver_grille_mesh / door_{n}_louver | framed horizontal louver-slat field (VentGrilleGeometry) on a batwing spring leaf | converged (parent) |
| full_glass_single_pane_leaf | rec_aluminum-storefront-double-door-two-narrow-stile_20260606_115902_647305_38236092 (parent) | _leaf_frame_body / _glass_pane, door_leaf_{n}_glass | narrow-stile frame around one large glass pane | converged (parent) |
| louvered_slat_revolute_leaf | rec_double_door_var_louvered_infill | _build_louver_slat helper, door_{idx}_slat_{i} loop (slat_count=18), frame_to_door_{n} revolute | full-height horizontal angled louver-shutter infill on a standard outward-revolute walnut leaf (decoupled from the saloon spring) | converged (variant) |
| cross_buck_board_leaf | rec_double_door_var_x_brace_solid | _make_tg_board / _build_ledger / _build_xbrace, door_{idx}_board_{i} loop (BOARD_COUNT), revolute leaves | full-height ledged-and-X-braced tongue-and-groove board infill, two diagonal braces + top/bottom ledgers, no glass | converged (variant) |

### Slot B:head / top profile
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| flat_head | rec_classic-double-entry-door-in-dark-walnut-wood-tw_20260606_115214_146776_3b44fd42 (parent) | frame head_jamb | flat rectangular head jamb across the top | converged (parent) |
| arched_stone_head | rec_arched-carriage-style-double-door-in-warm-wood-t_20260606_115230_910775_7b8384ab (parent) | _stone_surround arch_ring / keystone | semicircular arched stone head with keystone (frame surround) | converged (parent) |
| scalloped_crown_head | rec_saloon-style-swinging-cafe-double-doors-batwing-_20260606_115237_933064_1a02f0ce (parent) | _scalloped_leaf_profile crown | cyma/ogee scalloped crown peaking at center | converged (parent) |
| transom_over_flat_head | rec_ornate-double-door-in-honey-brown-wood-with-a-da_20260606_115210_265714_26551efa (parent) | _surround_members head_rail, _fixed_panels transom_panel | rectangular fixed transom band over flat-topped leaves | converged (parent) |
| arched_leaf_top (spring) | rec_double_door_var_arched_louvered | _arched_leaf_profile (CadQuery polyline/lathe arch), door_{n}, spring-pivot frame_to_door_{n} | full-height semicircular arched leaf top so the closed pair completes one round arch (arch lives in the leaf profile, not the frame surround), on the louvered spring leaf | converged (variant) |
| arched_leaf_top (revolute + arched ring header) | rec_double_door_var_arched_glazed | _arched_leaf_frame(sign) / _arched_glass_pane(sign) / _arched_frame_header, door_leaf_{n}, frame_to_door_leaf_{n} revolute | each leaf has a quarter-circle arched top (rect + threePointArc profile) so the closed pair completes one round arch; the FIXED frame also carries a semicircular ring _arched_frame_header above the spring line, and the glass pane follows the inner arch — proves arched leaf top + full single glass pane + outward revolute swing coexist | converged (variant) |

### Slot C:swing mechanism
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| both_revolute_opposite | rec_classic-double-entry-door-in-dark-walnut-wood-tw_20260606_115214_146776_3b44fd42 (parent) | frame_to_door_0 (+z) / frame_to_door_1 (-z) | both leaves revolute, opposite-axis, swing outward symmetric | converged (parent) |
| double_acting_spring | rec_saloon-style-swinging-cafe-double-doors-batwing-_20260606_115237_933064_1a02f0ce (parent) | frame_to_door_{n} symmetric +/- limits | spring pivots swinging both directions, rest at 0 | converged (parent) |
| active_inactive_astragal | rec_double_door_var_one_active_astragal | frame_to_door_0 REVOLUTE (active) + inactive leaf as inline frame visual + frame_astragal meeting bead | one active leaf hinges; the other is permanently fixed to the frame with a vertical overlapping astragal meeting strip at the meeting edge | converged (variant) |

## Multiplicity / Copy Logic
- count_param: `panel_count` (stacked raised panels per leaf) primary; `lite` row/col grid count secondary for glazed leaves; `slat_count`/`board_count` for the louvered/cross-buck infills.
- N 样本已覆盖: panel_count {2, 3, 6} -> rec_double_door_var_panels_two (N=2) / parent classic (N=3) / rec_double_door_var_panels_six (N=6); lite grid {1, 6} -> parent storefront (single pane) / rec_double_door_var_six_light_glazed (2x3 = 6 lites via nested loops). 3 distinct N on the primary panel axis.
- 模板建议 N_range: panel_count [1, 6] (real raised-panel doors run 1–6 stacked fields); lite grid [1, 12] (single pane up to multi-lite French grid). Sample coverage stays small; sweep fills the rest.
- copied object / naming / placement / joint policy: each stacked raised panel is one fielded-panel module emitted via `for i in range(panel_count)` as `panel_{i}` from a shared raised-panel helper (cut recessed field + bolection molding + proud fielded pad), placed at regular vertical pitch computed from panel_count, all inlined into the leaf solid as non-jointed visuals (no per-panel joint). Divided lites/muntins emitted via nested `for row/col` `lite_{row}_{col}` + `muntins` from a shared lite-and-muntin helper, regular grid spacing, uniform fixed policy seated into the cut window opening. Per-leaf copy logic is identical on both mirror leaves; the two leaves themselves are the multiplicity-2 of the leaf module via the sign/mirror helper.

## 组合数预审
Slot A(8 candidates: 6 parent + 2 variant) x Slot B(6: 4 parent + 2 variant) x Slot C(3: 2 parent + 1 variant) x distinct-N(panel_count 3). Spec gate Π(slot candidates) x distinct N:
- Every slot >= 2 candidates: A=8 ✓, B=6 ✓, C=3 ✓; distinct-N = 3 (2,3,6) ✓.
- Smallest two-slot product: Slot C(3) x Slot B(6) = 18 >= 10 ✓ (the smallest single slot, C=3, x distinct-N 3 = 9; combo gate below uses the product of all slots x distinct-N).
- Slot A(8) x distinct-N(3) = 24 >= 10 ✓.
- Combo = Π(slot candidates) x distinct-N = 8 x 6 x 3 x 3 = 432 >= 10 ✓.
GATE P1 met entirely by the converged parents + existing variants; zero gap-fill forks emitted.

## Readability notes for downstream spec authors
- Parent 001 (classic) builds its raised panels with a `for _ in range(n_panels)` loop that does NOT name the panels (`n_panels=3` hardcoded; panels are union/cut into the leaf solid, no `panel_{i}` visual names). Use rec_double_door_var_panels_six as the panel-multiplicity copy-logic source: it has the clean `for i in range(panel_count)` loop with `panel_{i}` naming + a shared `_add_raised_panel` helper. (rec_double_door_var_panels_two still uses `for _ in range(panel_count)` without `panel_{i}` names, so prefer panels_six for the loop pattern.)
- Parent 004 (carriage) builds muntins as a hardcoded 2-bar tuple unioned into the frame; rec_double_door_var_six_light_glazed is the canonical divided-lite copy-logic source — nested `for row in range(N_ROWS): for col in range(N_COLS)` emitting `lite_{row}_{col}` from a shared `_glass_lite` helper plus a `_muntin_grid` helper, seated into the cut vision-window opening. Use it, not the carriage parent, for the lite/muntin grid loop.
- rec_double_door_var_louvered_infill is the louver-slat copy-logic source: `for i in range(slat_count)` (slat_count=18) emitting `door_{idx}_slat_{i}` from a shared `_build_louver_slat` helper at equal vertical spacing, on standard outward-revolute leaves (cleaner to reuse than the saloon spring leaf).
- rec_double_door_var_x_brace_solid is the boarded copy-logic source: `for i in range(BOARD_COUNT)` emitting `door_{idx}_board_{i}` from a shared `_make_tg_board` helper, plus `_build_ledger` and two `_build_xbrace` diagonals.
- Parents 002 (hospital) and 006 (storefront) call `_add_leaf_visuals(..., sign=+1/-1)` twice rather than looping the two leaves, but each uses a single clean sign-parametric helper, so the leaf module is still cleanly extractable. No repeated sub-units to loop in those leaves.
- rec_double_door_var_arched_glazed is the canonical source for the arched-leaf-top-on-revolute-leaf + matching ring frame header: `_arched_leaf_frame(sign)` draws the leaf outer profile (rect + threePointArc quarter-circle) and cuts the glass opening following the inner arch; `_arched_glass_pane(sign)` follows the same inner arch; `_arched_frame_header()` builds the FIXED semicircular ring band (outer half-cylinder minus inner) seated at the spring line. The two leaf arches share one circle centered at the opening midpoint so the closed pair completes the full arch, and the frame ring is the SAME circle. It is a sign-parametric (not looped) leaf like the storefront parent it forks. Use it (not arched_louvered) when the arch-top must combine with a glass infill and outward revolute swing, and when the FIXED frame surround should also be arched.
- All parents and variants author one leaf in a leaf-local frame and produce the second leaf via a `sign` flip or a `mirror=True` `_mirror_x` helper; both front faces stay +Y. This mirror policy is the canonical leaf copy logic for the template. (rec_double_door_var_one_active_astragal is the exception by design: only door_0 is a real revolute part; the inactive leaf + its hardware + the astragal are inline frame visuals — use it only as the active/inactive swing source, not as a mirror-leaf source.)

## 排除项(未来 compatibility matrix 素材)
- scalloped_crown_head and arched_leaf_top both live in the LEAF top profile (not the frame surround), so they cannot coexist with each other on the same leaf, and pairing either with a full-height square-topped leaf infill requires the leaf body to be reprofiled (the saloon batwing relies on short leaves; arched_louvered proves a full-height arched leaf works with the spring frame, and arched_glazed proves the same arched leaf top works with a full single glass pane on outward revolute leaves — so arched_leaf_top is swing-mode agnostic and infill-agnostic, but the leaf body MUST be reprofiled to the arch).
- arched_leaf_top optionally pairs with a matching semicircular ring frame header above the spring line (arched_glazed's _arched_frame_header) so the FIXED surround follows the arch; this is the canonical pairing for a square-jamb frame whose head is the arch. arched_stone_head and transom_over_flat_head live in the FRAME surround and assume flat/square leaf tops; pairing EITHER of those frame-surround heads with arched_leaf_top would double the arch — exclude those two combinations (the ring header in arched_glazed is the SAME arch as the leaf top, by design, not a doubling).
- active_inactive_astragal (Slot C) makes one leaf a fixed inline visual, so it is incompatible with double_acting_spring (Slot C is mutually exclusive by definition) and changes the mirror-leaf assumption: a template must drop the second revolute and add the astragal strip + (optionally) a slide bolt when this swing mode is selected.
- louvered_slat_revolute_leaf vs louvered_slat_spring_leaf are the same infill on two different Slot C swing modes; keep them as one infill module parametrized by the swing slot, not two infill candidates, when building the compatibility matrix.
