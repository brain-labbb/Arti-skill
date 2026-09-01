# 0611 / trumpet — template source map

pattern: mixed
parents: `rec_picturex_0611__trumpet__001__png_228e1d1cb922462593313c01c202acc9` (`pictureY/0611/trumpet/001.png`), `rec_picturex_0611__trumpet__002__png_f6e4ecae886141d49062a10d06c055a2` (`pictureY/0611/trumpet/002.png`)
canonical_baselines: none planned; add only if a selected origin fails the §11 readability contract during fork review.
underfilled_reason: none at P0; adaptive coverage follows the approved per-category slot vocabulary.

## Subcategory Contract

- core_identity: brass trumpet retaining bell, mouthpiece air path, and compact valve-controlled tubing
- must_keep: defining use, picture identity, visible support interfaces, and at least one real non-fixed joint.
- must_not_become: trombone, bugle without valve mechanism
- image_evidence: pictureY/0611/trumpet/001.png, pictureY/0611/trumpet/002.png
- parent_evidence: rec_picturex_0611__trumpet__001__png_228e1d1cb922462593313c01c202acc9, rec_picturex_0611__trumpet__002__png_f6e4ecae886141d49062a10d06c055a2

## Slot Candidates

| slot | candidate | diversity_axis | source_type | record/evidence | key parts/joints/helpers | status |
|---|---|---|---|---|---|---|
| origin_design | pictureX_0611_trumpet_001 | ①/②/③ observed | origin_anchor | `rec_picturex_0611__trumpet__001__png_228e1d1cb922462593313c01c202acc9` / `pictureY/0611/trumpet/001.png` | trumpet_body, main_slide, third_slide, water_key, f'valve_{index}', main_slide_pull, third_slide_pull, water_key_lift, f'valve_{index}_press', _shell_mesh, _tube_mesh | built ✓ |
| origin_design | pictureX_0611_trumpet_002 | ①/②/③ observed | origin_anchor | `rec_picturex_0611__trumpet__002__png_f6e4ecae886141d49062a10d06c055a2` / `pictureY/0611/trumpet/002.png` | trumpet_body, main_tuning_slide, third_valve_slide, f'valve_{index}', adjust_main_slide, adjust_third_slide, f'press_valve_{index}', f'operate_{name}', _mesh, _tube, _ring_band, _hollow_tube, _bell_geometry, _mouthpiece_geometry | built ✓ |
| body_family | pocket trumpet | ③ | forked_anchor | `rec_0611_trumpet_var_body_family_pocket_trumpet` from `rec_picturex_0611__trumpet__001__png_228e1d1cb922462593313c01c202acc9` | trumpet_body, _shell_mesh, pictureX_0611_trumpet_001, button_base, bell_shell | built ✓ |
| body_family | piccolo trumpet | ③ | forked_anchor | `rec_0611_trumpet_var_body_family_piccolo_trumpet` from `rec_picturex_0611__trumpet__002__png_f6e4ecae886141d49062a10d06c055a2` | trumpet_body, pictureX_0611_trumpet_002, bell_shell | built ✓ |
| body_family | herald trumpet | ③ | forked_anchor | `rec_0611_trumpet_var_body_family_herald_trumpet` from `rec_picturex_0611__trumpet__001__png_228e1d1cb922462593313c01c202acc9` | trumpet_body, _shell_mesh, pictureX_0611_trumpet_001, button_base, bell_shell | built ✓ |
| body_family | bass trumpet | ③ | forked_anchor | `rec_0611_trumpet_var_body_family_bass_trumpet` from `rec_picturex_0611__trumpet__001__png_228e1d1cb922462593313c01c202acc9` | trumpet_body, _shell_mesh, pictureX_0611_trumpet_001, button_base, bell_shell | built ✓ |
| valve_mechanism | rotary valves | ② | forked_anchor | `rec_0611_trumpet_var_valve_mechanism_rotary_valves` from `rec_picturex_0611__trumpet__001__png_228e1d1cb922462593313c01c202acc9` | third_slide_pull, main_slide_pull, f'valve_{index}_press', third_slide, main_slide, f'valve_{index}', third_slide_sleeve_1, third_slide_sleeve_0 | built ✓ |
| valve_mechanism | top-sprung piston valves | ② | forked_anchor | `rec_0611_trumpet_var_valve_mechanism_top_sprung_piston_valv` from `rec_picturex_0611__trumpet__001__png_228e1d1cb922462593313c01c202acc9` | third_slide_pull, main_slide_pull, f'valve_{index}_press', third_slide, main_slide, f'valve_{index}', third_slide_sleeve_1, third_slide_sleeve_0 | built ✓ |
| valve_count | 3 valves | N | forked_anchor | `rec_0611_trumpet_var_valve_count_3_valves` from `rec_picturex_0611__trumpet__002__png_f6e4ecae886141d49062a10d06c055a2` | f'press_valve_{index}', f'valve_{index}', third_valve_slide, valve_stem, valve_button, valve_block | built ✓ |
| valve_count | 4 valves | N | forked_anchor | `rec_0611_trumpet_var_valve_count_4_valves` from `rec_picturex_0611__trumpet__002__png_f6e4ecae886141d49062a10d06c055a2` | f'press_valve_{index}', f'valve_{index}', third_valve_slide, valve_stem, valve_button, valve_block | built ✓ |
| slide_control | first-valve trigger | ② | forked_anchor | `rec_0611_trumpet_var_slide_control_first_valve_trigger` from `rec_picturex_0611__trumpet__002__png_f6e4ecae886141d49062a10d06c055a2` | third_valve_slide, f'press_valve_{index}', adjust_third_slide, adjust_main_slide, main_tuning_slide, f'valve_{index}', valve_stem, valve_button | built ✓ |
| slide_control | main-slide trigger | ② | forked_anchor | `rec_0611_trumpet_var_slide_control_main_slide_trigger` from `rec_picturex_0611__trumpet__002__png_f6e4ecae886141d49062a10d06c055a2` | adjust_main_slide, main_tuning_slide, adjust_third_slide, third_valve_slide, main_slide_tube, main_slide_sleeves, third_slide_tube, third_slide_sleeves | built ✓ |

## Multiplicity / Copy Logic

- count_param: valve_count_count
- N samples: 3 valves, 4 valves
- suggested N_range: bounded by accepted source samples and downstream compile budget.
- copied object / naming / placement / joint policy: shared helper, `name_{i}`, regular placement, uniform joint policy; exact names resolve from accepted variants.

## Six-Axis Diversity Record

| axis | treatment | values / range / reason |
|---|---|---|
| ① skeleton / structural topology | source-backed | origin rows plus planned ① candidates |
| ② joint / mechanism type | source-backed | origin rows plus planned ② candidates |
| ③ primary form family | source-backed | origin rows plus planned ③ candidates |
| ④ surface decoration | record_only / world_knowledge_extrapolation | host-conformal seams, ribs, labels, bezels only |
| ⑤ proportion / size / travel | record_only | origin ranges plus modest safe companion tuning |
| ⑥ material / palette / finish | record_only | origin materials plus realistic companion colorways |

## Compatibility Probes

| probe | record_id | combined axes | risk tested | result |
|---|---|---|---|---|
| none at P0 | — | — | add only if cross-family interface review finds a real risk | — |

## Blocked / Excluded

- ④/⑤/⑥-only forks: excluded; these do not count as candidate anchors.
- neighbor categories (trombone, bugle without valve mechanism): excluded.
- failed or unfit candidates will be appended with one-line reasons after 2–3 attempts.
