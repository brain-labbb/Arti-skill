# Music / CD_case — template source map

pattern: mixed (parallel_children over body_type / closure_hinge / inner_tray slots + multiplicity over disc_count, with two distinct copy-logics)
parents: rec_clear-plastic-cd-jewel-case-with-a-hinged-transp_20260605_161820_827338_61eddc85 ← picture/Music/CD case/001.png

Parent baseline: standard rigid clear jewel case. Parts `base` (clear `base_frame` shell + dark `inner_tray` with central `hub` boss, 8-tooth `rosette`, two front finger notches), `lid` (clear `lid_shell`), `disc` (silver `disc_body` + off-center `disc_marker`). Joints `base_to_lid` (REVOLUTE, axis +X, rear +Y hinge, 0→70°) and `hub_to_disc` (CONTINUOUS, axis +Z, on the center hub). 1 disc.

## Slot 候选覆盖

### Slot A:body_type
| 候选(future module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| standard_rigid (baseline) | rec_clear-plastic-cd-jewel-case-with-a-hinged-transp_20260605_161820_827338_61eddc85 | base(base_frame+inner_tray:hub+rosette), lid(lid_shell), disc; base_to_lid REVOLUTE, hub_to_disc CONTINUOUS | clear rectangular shell + dark center-hub tray, swing-up lid; full-height jewel case | converged(已同步) |
| slimline | rec_cd_jewel_case_var_body_slimline | base(base_plate), lid(lid_cover+lid_tray), disc; base_to_lid REVOLUTE axis −X, hub_to_disc CONTINUOUS | thin case: tray migrates INTO the lid (lid_tray rides under lid_cover), flat base_plate; disc seats within the lid cover footprint | converged(已同步) |
| doublewide | rec_cd_jewel_case_var_body_doublewide | base(base_frame+tray_floor+hub_rosette_i loop), lid(lid_shell), disc; base_to_lid REVOLUTE axis +X | width-widened multi-slot shell: single shared tray_floor with N hub_rosette bosses placed along X (HUB_X[]); lid spans ≥80% of widened width | converged(已同步) |
| digipak | rec_cd_jewel_case_var_body_digipak | tray_panel(tray_board+spine+disc_tray) root, cover_panel(cover_board), disc; spine_fold REVOLUTE axis +Y 0→170°, hub_to_disc CONTINUOUS | book-fold card case: opaque cardboard panels hinged at a spine strip (NOT a clamshell shell), molded disc_tray glued onto +X board | converged(已同步) |

### Slot B:closure_hinge
| 候选(future module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| clamshell_swing (baseline) | rec_clear-plastic-cd-jewel-case-with-a-hinged-transp_20260605_161820_827338_61eddc85 | lid(lid_shell), base_to_lid REVOLUTE origin rear +Y edge, axis +X, 0→70° | rear-edge swing-up transparent lid; hinge line runs left-right (X) | converged(已同步) |
| topflip | rec_cd_jewel_case_var_hinge_topflip | lid(lid_shell), base_to_lid REVOLUTE origin right +X short-side edge, axis +Y, 0→70° | hinge rotated to the short side: lid flips sideways about the +X edge, free −X edge lifts up | converged(已同步) |
| slidingsleeve | rec_cd_jewel_case_var_hinge_slidingsleeve | base(base_frame+inner_tray), sleeve(sleeve_shell), base_to_sleeve PRISMATIC axis +X 0→0.17 m | no hinge: clear slipcase sleeve (open −X end) slides off along the long axis; allow_isolated_part + closed-overlap proof instead of contact | converged(已同步) |

### Slot C:inner_tray
| 候选(future module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| center_hub (baseline) | rec_clear-plastic-cd-jewel-case-with-a-hinged-transp_20260605_161820_827338_61eddc85 | inner_tray(hub+8-tooth rosette+2 front notches), hub_to_disc CONTINUOUS on base | fixed dark tray, single center-hub rosette captures one disc; disc spins on hub | converged(已同步) |
| trayless | rec_cd_jewel_case_var_tray_trayless | base(base_frame+sleeve_pocket), disc_body as a FIXED base visual (no hub, no spin joint); only base_to_lid REVOLUTE | sleeve/pocket interior: disc lies flat in a recessed pocket, no rosette hub and no disc articulation (disc is rigid to base) | converged(已同步) |
| dualsided_flip | rec_cd_jewel_case_var_tray_dualsided | base(base_frame+pivot_post_0/1), flip_tray(tray_panel+hub_face_0/hub_face_1+integral pivot pins), lid, disc; base_to_flip_tray REVOLUTE axis +X 0→π, base_to_lid REVOLUTE, tray_to_disc CONTINUOUS on flip_tray | tray flips about a CENTER X pivot on bearing posts, hub+rosette on BOTH faces; disc parented to the flip_tray, not the base | converged(已同步) |
| bookletclip | rec_cd_jewel_case_var_tray_bookletclip | base(base_frame+inner_tray center-hub), lid(lid_shell+booklet_card+clip), disc; base_to_lid REVOLUTE, hub_to_disc CONTINUOUS | baseline center-hub tray PLUS a booklet_card retained by a clip feature inside the lid shell (additive lid-side feature, tray unchanged) | converged(已同步) |

## Multiplicity / Copy Logic
- count_param: disc_count ; (parent default = 1)
- N 样本已覆盖: {1, 2, 4} → parent(=1) / rec_cd_jewel_case_var_discs_n2 / rec_cd_jewel_case_var_discs_n4
- 模板建议 N_range: [1, 6]  (1=baseline single hub; 2=side-by-side; 3–6=stacked book; >6 turns the case impractically fat)
- TWO distinct copy-logics observed — template must pick one per body_type:
  - copied object (coplanar / N=2): `_tray_section(cx)` floor+hub+rosette+notches per slot, named `tray_{i}` on the shared base; one `disc_{i}` per section via loop, each `disc_{i}_body`; one CONTINUOUS `hub_to_disc_{i}` per disc. placement: even X spacing across case width (TRAY_SECTION_W = (CASE_W−2·WALL)/N, cx = `_section_cx(i)`); shared single `base`/`lid`. joint policy: N parallel CONTINUOUS spin joints, all axis +Z, all parented to base.
  - copied object (stacked book / N=4): shared `leaf_mesh`/`disc_mesh`, parts `leaf_{i}` (page) + `disc_{i}` stacked vertically; per-leaf hinge `base_to_leaf_{i}` REVOLUTE axis −X (0→120°) at `_leaf_hinge_z(i)` = LEAF_FIRST_Z + i·LEAF_SPACING; per-disc `leaf_{i}_to_disc_{i}` CONTINUOUS axis +Z parented to that leaf. naming: leaf_{i}/disc_{i}/disc_body_{i}/disc_marker_{i}, per-leaf material `leaf_dark_{i}`. placement: stacked +Z, taller `base_frame` (WALL_H) capped by one `lid`. joint policy: each disc spins relative to its OWN hinged leaf (parent=leaf), not the base.

## 排除项
- digipak (body_type) × slidingsleeve (closure_hinge): digipak is a self-folding card book with no separate rigid shell to slip a sleeve over — the spine fold IS the closure; the two closure mechanisms are mutually exclusive. 排除.
- slidingsleeve (closure_hinge) × dualsided_flip / bookletclip (inner_tray): the dual-sided flip tray needs a hinged top opening to flip, and the booklet clip lives inside a swing/flip lid; a sliding sleeve has no openable lid to host either → 排除 (sleeve only co-converges with center_hub or trayless).
- trayless (inner_tray) × stacked-book disc multiplicity (N≥3): trayless has no hub and no per-disc spin/leaf — it holds a single flat disc in a pocket; multi-disc stacking presupposes hubs/leaves. trayless is effectively N=1 only. 排除.
- topflip / slidingsleeve combined with the coplanar N=2 widening: the side-flip and slide axes both run along X, which collides with widening the case along X for side-by-side discs; coplanar multiplicity pairs cleanly only with the rear-swing clamshell. 排除 (side-by-side N uses clamshell_swing).
