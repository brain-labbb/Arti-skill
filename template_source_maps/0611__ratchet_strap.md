# 0611 / ratchet_strap — template source map

pattern: mixed
parents: `rec_use-the-attached-image-as-the-primary-and-author_20260711_160740_098181_6a70af39` (`pictureY/0611/ratchet_strap/001.png`), `rec_picturex_0611__ratchet_strap__002__png_8a06b96fc10447be86c17bcba7c52ec4` (`pictureY/0611/ratchet_strap/002.png`)
canonical_baselines: none; both current origins satisfy the §11 readability contract.
underfilled_reason: none; 2 origins + 10 planned forks provide 12 honest candidate anchors across frame form, end fitting, release, and webbing topology.

## Subcategory Contract

```yaml
subcategory_contract:
  category: 0611
  subcategory: ratchet_strap
  core_identity: cargo tie-down webbing tensioned by a handled ratchet, toothed/slotted winding spool, and pawl/release mechanism
  must_keep: [ratchet tensioning function, captured winding spool, handled actuation, webbing load path, at least one real non-fixed joint]
  must_not_become: [cam buckle strap, seat belt, tow strap, hand winch]
  image_evidence: [orange two-piece hooked cargo strap with webbing roll, blue feed-through threading diagram]
  parent_evidence: [stamped U-frame, slotted/toothed spindle, articulated handle, articulated release, loop-emitted woven detail]
```

## Slot Candidates

| slot | candidate | diversity_axis | source_type | record/evidence | key parts/joints/helpers | status |
|---|---|---|---|---|---|---|
| origin_design | orange hooked two-piece tie-down with separate roll | ①/②/③ observed | origin_anchor | `rec_use-the-attached-image-as-the-primary-and-author_20260711_160740_098181_6a70af39` / `001.png` | ratchet_frame, winding_spindle, ratchet_handle, drive_pawl, release_lever, free_strap, webbing_roll, spindle_rotation, handle_pivot, pawl_pivot, release_pivot, _frame_shape, _hook_shape, _routed_strap_shape | built ✓ |
| origin_design | blue feed-through raised-handle ratchet | ①/②/③ observed | origin_anchor | `rec_picturex_0611__ratchet_strap__002__png_8a06b96fc10447be86c17bcba7c52ec4` / `002.png` | frame, spool, handle, release_lever, webbing, frame_to_spool, frame_to_handle, handle_to_release, _frame_shape, _spool_drum_shape | built ✓ |
| frame_form | compact mini ratchet | ③ | forked_anchor | `rec_0611_ratchet_strap_var_frame_form_compact_mini_ratchet` from origin `001` | ratchet_frame, _side_plate, _frame_shape, handle_pivot, spindle_rotation | built ✓ (`gpt-5.6-sol`, high) |
| frame_form | long-handle heavy-duty ratchet | ③ | forked_anchor | `rec_0611_ratchet_strap_var_frame_form_long_handle_heavy_duty_ratc` from origin `001` | ratchet_frame, ratchet_handle, _frame_shape, _heavy_handle_arm, _heavy_rubber_grip, handle_pivot | built ✓ (`gpt-5.6-sol`, high) |
| frame_form | wide-body cargo ratchet | ③ | forked_anchor | `rec_0611_ratchet_strap_var_frame_form_wide_body_cargo_ratchet` from origin `002` | frame, frame_shell, _frame_shape, frame_to_spool, frame_to_handle | built ✓ (`gpt-5.6-sol`, high) |
| end_fitting | wire J hook | ① | origin_anchor | origin `001` | fixed_end_hook, free_end_hook, _hook_shape | built ✓ |
| end_fitting | plain threaded tail / no hook | ① | origin_anchor | origin `002` | webbing, feed_webbing, tail_webbing | built ✓ |
| end_fitting | flat stamped J hook | ① | forked_anchor | `rec_0611_ratchet_strap_var_end_fitting_flat_j_hook` from origin `001` | fixed_end_hook, free_end_hook, _hook_shape | built ✓ (`gpt-5.6-sol`, high) |
| end_fitting | S hook | ① | forked_anchor | `rec_0611_ratchet_strap_var_end_fitting_s_hook` from origin `001` | fixed_end_hook, free_end_hook, _hook_shape | built ✓ (`gpt-5.6-sol`, high) |
| end_fitting | gated snap hook | ① | forked_anchor | `rec_0611_ratchet_strap_var_end_fitting_snap_hook` from origin `001` | _snap_hook_body, _snap_hook_gate, snap_gate_0..1, snap_gate_pivot_0..1 | built ✓ (`gpt-5.6-sol`, high) |
| end_fitting | E-track fitting | ① | forked_anchor | `rec_0611_ratchet_strap_var_end_fitting_e_track_fitting` from origin `001` | _etrack_end_fitting, fixed_end_hook, free_end_hook | built ✓ (`gpt-5.6-sol`, high) |
| end_fitting | reinforced sewn soft loop | ① | forked_anchor | `rec_0611_ratchet_strap_var_end_fitting_soft_loop` from origin `001` | _soft_loop_shape, _add_soft_loop, reinforcement stitches | built ✓ (`gpt-5.6-sol`, high) |
| release | frame-mounted thumb release | ② | origin_anchor | origin `001` | release_lever, release_pivot | built ✓ |
| release | handle-mounted finger paddle | ② | origin_anchor | origin `002` | release_lever, handle_to_release, _release_shape | built ✓ |
| release | central pull tab | ② | forked_anchor | `rec_0611_ratchet_strap_var_release_pull_tab_release` from origin `002` | release_lever, _release_shape, release_mechanism, handle_to_release | built ✓ (`gpt-5.6-sol`, high) |
| webbing_topology | two-piece hooked strap with separate roll | ① | origin_anchor | origin `001` | free_strap, webbing_roll, strap_mount, webbing_roll_rotation | built ✓ |
| webbing_topology | loose feed-through tail | ① | origin_anchor | origin `002` | webbing, feed_webbing, tail_webbing, frame_to_webbing | built ✓ |
| webbing_topology | endless closed loop | ① | forked_anchor | `rec_0611_ratchet_strap_var_webbing_topology_endless_loop` from origin `002` | webbing, _endless_webbing_shape, frame_to_webbing | built ✓ (`gpt-5.6-sol`, high) |

## Variant Cards

| variant_id | parent | product archetype / why same subcategory | primary axis | structural delta | acceptance focus |
|---|---|---|---|---|---|
| `rec_0611_ratchet_strap_var_frame_form_compact_mini_ratchet` | origin `001` | compact light-duty ratchet; same tensioning mechanism | frame_form / ③ / compact mini | shorten only stamped frame family and preserve captured pivots | compact cheek geometry plus axis-specific test |
| `rec_0611_ratchet_strap_var_frame_form_long_handle_heavy_duty_ratc` | origin `001` | high-leverage cargo ratchet; same tie-down function | frame_form / ③ / long-handle heavy-duty | reinforce/lengthen frame and handle only | long handle and supported pivots asserted |
| `rec_0611_ratchet_strap_var_frame_form_wide_body_cargo_ratchet` | origin `002` | wide-webbing cargo ratchet | frame_form / ③ / wide body | widen U-frame bearing span only | wide frame and spool capture asserted |
| `rec_0611_ratchet_strap_var_end_fitting_flat_j_hook` | origin `001` | flat-hook cargo tie-down | end_fitting / ① / flat J hook | replace only paired hook geometry | both hooks attached; flat slot geometry asserted |
| `rec_0611_ratchet_strap_var_end_fitting_s_hook` | origin `001` | S-hook cargo tie-down | end_fitting / ① / S hook | replace only paired hook geometry | S profile and webbing eyes asserted |
| `rec_0611_ratchet_strap_var_end_fitting_snap_hook` | origin `001` | gated-hook cargo tie-down | end_fitting / ① / snap hook | replace only fittings; fitting-local gate joints allowed | supported gates and gate motion asserted |
| `rec_0611_ratchet_strap_var_end_fitting_e_track_fitting` | origin `001` | trailer E-track tie-down | end_fitting / ① / E-track fitting | replace only fittings; local releases allowed | tongue/slot/release geometry asserted |
| `rec_0611_ratchet_strap_var_end_fitting_soft_loop` | origin `001` | vehicle soft-loop tie-down | end_fitting / ① / soft loop | replace metal hooks with sewn loop eyes only | loop continuity and indexed stitches asserted |
| `rec_0611_ratchet_strap_var_release_pull_tab_release` | origin `002` | pull-tab ratchet strap | release / ② / pull tab | replace only finger paddle release interface | bounded real release joint and pull tab asserted |
| `rec_0611_ratchet_strap_var_webbing_topology_endless_loop` | origin `002` | hookless bundling ratchet | webbing_topology / ① / endless loop | close only the webbing route into one continuous loop | continuous loop and unchanged ratchet joints asserted |

All ordinary cards preserve parent counts unless the named fitting-local helper requires homologous pair emission. Companion variations are limited to realistic ④/⑤/⑥ finish, restrained proportion, or stitch/color treatment and may not alter the part tree, main joint graph, interfaces, or primitive family.

## Multiplicity / Copy Logic

- count_param: no product-level multiplicity axis; repeated ratchet teeth, grip ribs, woven picks, stitches, and paired fitting details remain implementation copy logic rather than template candidates.
- N samples: origin loop counts plus the accepted variants' loop-emitted details.
- suggested N_range: preserve source-derived bounded detail counts; do not expose these cosmetic counts as a structural slot.
- copied object / naming / placement / joint policy: shared helpers and `name_{i}` naming; regular angular/linear placement; fitting gates, if present, use uniform fitting-local revolute policy.

## Six-Axis Diversity Record

| axis | treatment | values / range / reason |
|---|---|---|
| ① skeleton / structural topology | source-backed | paired hooked, plain tail, flat J, S hook, snap hook, E-track, soft loop, endless loop |
| ② joint / mechanism type | source-backed | frame-mounted thumb release, handle-mounted finger paddle, central pull tab; all retain spool/handle articulation |
| ③ primary form family | source-backed | standard photographed forms, compact mini, long-handle heavy-duty, wide-body cargo |
| ④ surface decoration | record_only / world_knowledge_extrapolation | stamped cheek windows, grip ribs, weave picks, reinforcement stitches, small labels; host-conformal only |
| ⑤ proportion / size / travel | record_only | compact/light-duty through long-handle/wide-body heavy-duty; bounded handle and release travel from sources |
| ⑥ material / palette / finish | record_only | galvanized/zinc steel, black grip, orange/blue/black polyester webbing, zinc/black/vinyl-coated fittings |

## Compatibility Probes

| probe | record_id | combined axes | risk tested | result |
|---|---|---|---|---|
| none planned | — | — | ordinary variants remain single-axis and each fitting is anchored on the standard photographed frame | — |

## Blocked / Excluded

- cam-buckle and over-center buckle mechanisms: neighbor products, not ratchet straps.
- hand-winch crank and geared winch frames: neighbor category and bundled mechanism change.
- ④/⑤/⑥-only forks: excluded; recorded for downstream sampling rather than candidate-anchor padding.
- old variants referencing deleted origin `rec_picturex_0611__ratchet_strap__001__png_6f674dbcd3e94a3c82489564889ae1f8`: deleted and excluded from all new source accounting.
