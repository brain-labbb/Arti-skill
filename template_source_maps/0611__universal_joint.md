# 0611 / universal_joint — template source map

pattern: mixed
parents: `rec_build-a-reference-accurate-compact-steel-double-_20260713_110333_920261_f7e21841` (`pictureY/0611/universal_joint/001.png`), `rec_picturex_0611__universal_joint__002__png_884db28383a4428c922743204f4a575c` (`pictureY/0611/universal_joint/002.png`)
canonical_baselines: none planned; add only if a selected origin fails the §11 readability contract during fork review.
underfilled_reason: none at P0; adaptive coverage follows the approved per-category slot vocabulary.

## Subcategory Contract

- core_identity: shaft-coupling universal joint transmitting rotation across an angled axis via a cross/spider (or centered double-cardan) with real revolute journal joints between the spider and the end yokes
- must_keep: defining use, picture identity, visible shaft-connection interfaces, and the real non-fixed revolute joints at the spider journals.
- must_not_become: rigid flange coupling (no articulation), constant-velocity Rzeppa ball joint housing, plain clevis/pin linkage
- image_evidence: pictureY/0611/universal_joint/001.png, pictureY/0611/universal_joint/002.png
- parent_evidence: rec_build-a-reference-accurate-compact-steel-double-_20260713_110333_920261_f7e21841, rec_picturex_0611__universal_joint__002__png_884db28383a4428c922743204f4a575c

## Slot Candidates

| slot | candidate | diversity_axis | source_type | record/evidence | key parts/joints/helpers | status |
|---|---|---|---|---|---|---|
| origin_design | compact_double_cardan | ①/②/③ observed | origin_anchor | `rec_build-a-reference-accurate-compact-steel-double-_20260713_110333_920261_f7e21841` / `pictureY/0611/universal_joint/001.png` | compact_double_cardan, middle_section, double_yoke_forging, left_spider, right_spider, left_section, right_section, sleeve_yoke_body, spider_core, journal_pos_z, journal_neg_z, journal_pos_y, journal_neg_y, four REVOLUTE journal joints, _spider_core_shape, _journal_shape, _end_section_shape, _middle_section_shape, _bearing_cup_shape, _add_spider_visuals | built ✓ |
| origin_design | universal_joint_002 | ①/②/③ observed | origin_anchor | `rec_picturex_0611__universal_joint__002__png_884db28383a4428c922743204f4a575c` / `pictureY/0611/universal_joint/002.png` | universal_joint_002, spider, forged_hub, journal_x, journal_y, yoke_a, yoke_b, shaft_yoke, cap_pos, cap_neg, retaining_ring_pos, retaining_ring_neg, bore_liner, mouth_edge, spider_to_yoke_a, spider_to_yoke_b (both REVOLUTE), _yoke_a_shape, _yoke_b_shape, _bearing_cap, _cylinder, _annulus, _fuse | built ✓ |
| joint_topology | single cross/cardan (Hooke) | ①/③ | origin_anchor | `rec_picturex_0611__universal_joint__002__png_884db28383a4428c922743204f4a575c` | spider, forged_hub, journal_x, journal_y, spider_to_yoke_a, spider_to_yoke_b | built ✓ |
| joint_topology | double-cardan | ①/③ | origin_anchor | `rec_build-a-reference-accurate-compact-steel-double-_20260713_110333_920261_f7e21841` | compact_double_cardan, middle_section, left_spider, right_spider, four REVOLUTE journal joints | built ✓ |
| joint_topology | pin-and-block joint | ① | forked_anchor | `rec_0611_universal_joint_var_joint_topology_pin_and_block_joint` from `rec_picturex_0611__universal_joint__002__png_884db28383a4428c922743204f4a575c` | pivot_block, spider, journal_x, journal_y, shaft_yoke, spider_to_yoke_a, spider_to_yoke_b, _yoke_a_shape, _yoke_b_shape | built ✓ |
| yoke_form | forged open yoke | ② | forked_anchor | `rec_0611_universal_joint_var_yoke_form_forged_open_yoke` from `rec_picturex_0611__universal_joint__002__png_884db28383a4428c922743204f4a575c` | shaft_yoke, forged_hub, cap_pos, cap_neg, retaining_ring_pos, retaining_ring_neg, mouth_edge, _yoke_a_shape, _yoke_b_shape | built ✓ |
| yoke_form | enclosed block yoke | ② | forked_anchor | `rec_0611_universal_joint_var_yoke_form_enclosed_block_yoke` from `rec_picturex_0611__universal_joint__002__png_884db28383a4428c922743204f4a575c` | shaft_yoke, bore_liner, cap_pos, cap_neg, journal_x, journal_y, _yoke_a_shape, _yoke_b_shape | built ✓ |
| yoke_form | enclosed block yoke (double-cardan end) | ② | forked_anchor | `rec_0611_universal_joint_var_end_enclosed_block_yoke` from `rec_build-a-reference-accurate-compact-steel-double-_20260713_110333_920261_f7e21841` | block_yoke_body, left_cap_pos_z, left_cap_neg_z, right_cap_pos_z, right_cap_neg_z, spider_core, _end_section_shape | built ✓ |
| yoke_form | flange yoke (double-cardan end) | ② | forked_anchor | `rec_0611_universal_joint_var_end_flange_yoke` from `rec_build-a-reference-accurate-compact-steel-double-_20260713_110333_920261_f7e21841` | sleeve_yoke_body, left_cap_pos_z, left_cap_neg_z, right_cap_pos_z, right_cap_neg_z, spider_core, _end_section_shape | built ✓ |
| middle_member | centering ball (centered double-cardan) | ① | forked_anchor | `rec_0611_universal_joint_var_middle_centering_ball` from `rec_build-a-reference-accurate-compact-steel-double-_20260713_110333_920261_f7e21841` | centering_ball, centering_ball_stub, centering_socket, middle_section, double_yoke_forging, _centering_ball_shape, _centering_ball_stub_shape, _centering_socket_shape | built ✓ |
| middle_member | intermediate shaft (uncentered) | ① | forked_anchor | `rec_0611_universal_joint_var_middle_intermediate_shaft` from `rec_build-a-reference-accurate-compact-steel-double-_20260713_110333_920261_f7e21841` | middle_section, double_yoke_forging, sleeve_yoke_body, _middle_section_shape | built ✓ |
| shaft_connection | plain bore / keyed (default) | ② | origin_anchor | `rec_picturex_0611__universal_joint__002__png_884db28383a4428c922743204f4a575c` | shaft_yoke, bore_liner, mouth_edge | built ✓ |
| shaft_connection | splined shaft | ② | forked_anchor | `rec_0611_universal_joint_var_connection_splined_shaft` from `rec_picturex_0611__universal_joint__002__png_884db28383a4428c922743204f4a575c` | shaft_yoke, forged_hub, _splined_shaft_x, _splined_shaft_z, journal_x, journal_y | built ✓ |

## Multiplicity / Copy Logic

- count_param: no strong repeated-part axis planned — the spider journal count is fixed at 4 by the cross geometry (two orthogonal journal pairs) and is not a diversity axis.
- N samples: origins only
- suggested N_range: bounded by accepted source samples and downstream compile budget.
- copied object / naming / placement / joint policy: journal pair emitted per spider (`journal_pos_z`/`journal_neg_z`/`journal_pos_y`/`journal_neg_y` on double-cardan; `journal_x`/`journal_y` on single cross), regular orthogonal placement, uniform REVOLUTE joint policy; exact names resolve from accepted variants.

## Six-Axis Diversity Record

| axis | treatment | values / range / reason |
|---|---|---|
| ① skeleton / structural topology | source-backed | origin rows plus joint_topology (single cross / double-cardan / pin-and-block) and middle_member (centering ball / intermediate shaft) candidates |
| ② joint / mechanism type | source-backed | origin rows plus yoke_form and shaft_connection candidates |
| ③ primary form family | source-backed | joint_topology is the ③ primary form family (single cross/cardan vs double-cardan); form differences flow from the declared anchors |
| ④ surface decoration | record_only / world_knowledge_extrapolation | host-conformal grease fittings, retaining-ring grooves, cap dimples, forging seams only |
| ⑤ proportion / size / travel | record_only | origin ranges plus modest safe companion tuning of yoke length, journal diameter, articulation angle |
| ⑥ material / palette / finish | record_only | origin steel/forged/plated materials plus realistic companion colorways |

## Compatibility Probes

| probe | record_id | combined axes | risk tested | result |
|---|---|---|---|---|
| none at P0 | — | — | add only if cross-family interface review finds a real risk | — |

## Blocked / Excluded

- ④/⑤/⑥-only forks: excluded; these do not count as candidate anchors.
- neighbor categories (rigid flange coupling, constant-velocity Rzeppa ball joint, plain clevis/pin linkage): excluded.
- COMPATIBILITY GATE: `middle_member` (centering ball / intermediate shaft) applies ONLY to the `joint_topology=double-cardan` family — a single cross/cardan has no middle member; the template must gate this slot on topology and must not sample a middle_member on a single-cross seed.
- COMPATIBILITY GATE: the double-cardan end yoke forms (enclosed block yoke / flange yoke, from origin 001) and the single-cross yoke forms (forged open yoke / enclosed block yoke, from origin 002) are sourced per topology family; keep yoke_form candidates matched to their topology origin rather than cross-composing across families.
- failed or unfit candidates will be appended with one-line reasons after 2–3 attempts.

## Origin Accounting

- `rec_build-a-reference-accurate-compact-steel-double-_20260713_110333_920261_f7e21841` (001.png, double-cardan) — placed as origin_anchor; hosts 4 forks (end_enclosed_block_yoke, end_flange_yoke, middle_centering_ball, middle_intermediate_shaft).
- `rec_picturex_0611__universal_joint__002__png_884db28383a4428c922743204f4a575c` (002.png, single cross/cardan) — placed as origin_anchor; hosts 4 forks (connection_splined_shaft, yoke_form_enclosed_block_yoke, yoke_form_forged_open_yoke, joint_topology_pin_and_block_joint).
- No unaccounted origins in `data/index/subcat/0611__universal_joint.jsonl`.
