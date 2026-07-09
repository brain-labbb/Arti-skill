# Accessories / glasses - template source map

pattern: mixed
parents:
- rec_create-a-highly-detailed-articulated-3d-model-of_20260620_143104_211959_619782b5 <- picture/Accessories/glasses/001.png (sunglasses / sport glasses parent with tinted lens/frame evidence). Covers Slot A=sport_front_frame, Slot B=standard_bridge, Slot C=sport_temples.
- rec_create-a-highly-detailed-articulated-3d-model-of_20260620_143104_209562_7e979b64 <- picture/Accessories/glasses/002.png (clear eyeglasses with separate left/right rims, bridge, nose pads, folding temple arms). Covers Slot A=rectangular_clear_rims, Slot B=standard_bridge, Slot C=folding_temples.

Eyewear family with front frame/lenses, bridge/nose support, and temple hinge/arm modules. Core
articulation is the left/right temple fold; variants isolate front lens/rim shape, bridge/nose
structure, and temple/hinge construction.

## Slot 候选覆盖

### Slot A:front lens / rim shape
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| rectangular_clear_rims | rec_create-a-highly-detailed-articulated-3d-model-of_20260620_143104_209562_7e979b64 (parent) | left/right lens rims, temple hinge joints | clear separated eyeglass lenses in a conventional front frame | converged |
| sport_front_frame | rec_create-a-highly-detailed-articulated-3d-model-of_20260620_143104_211959_619782b5 (parent) | front frame, tinted lenses, temple hinges | parent sport/tinted glasses front | converged |
| cat_eye_frame | rec_glasses_var_cat_eye_frame | cat-eye rim/lens visuals, original hinge joints | upswept cat-eye lens/rim silhouette | converged |
| round_wire_rims | rec_glasses_var_round_wire_rims | round wire rim visuals, nose bridge | thin round wire rims | converged |
| wraparound_shield | rec_glasses_var_wraparound_shield | shield lens/front frame, temple hinges | one-piece wraparound sport shield lens | converged |
| hexagonal_lenses | rec_glasses_var_qwen_hexagonal_lenses | lens_right/lens_left, lens_rim_right/lens_rim_left, left_hinge/right_hinge | crisp geometric hexagonal lenses with angled upper/lower corners | converged |
| oval_panto_lenses | rec_glasses_var_qwen_oval_panto_lenses | lens_right/lens_left, lens_rim_right/lens_rim_left, left_hinge/right_hinge | soft oval panto lenses, rounded top and fuller lower curve | converged |

### Slot B:bridge / nose structure
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| standard_bridge | parents | bridge and nose-pad visuals | simple bridge between two lenses with retained nose support | converged |
| keyhole_bridge | rec_glasses_var_keyhole_bridge | keyhole bridge/nose support visuals | keyhole-shaped bridge and nose cutout | converged |

### Slot C:temple arm / hinge mechanism
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| folding_temples | rec_create-a-highly-detailed-articulated-3d-model-of_20260620_143104_209562_7e979b64 (parent) | left/right temple revolute joints | slim folding temple arms | converged |
| spring_hinges | rec_glasses_var_spring_hinges | spring hinge blocks, secondary flex links | enlarged spring hinge barrels at both temple roots | converged |
| thick_sport_temples | rec_glasses_var_thick_sport_temples | temple arm parts, hinge joints | thick curved sport temples with broad ear tips | converged |

## Multiplicity / Copy Logic
- count_param: side_count is fixed at 2 for symmetric left/right temples, hinges, lenses, and nose pads.
- N 样本已覆盖: left/right mirrored copies across all parents/variants.
- 模板建议 N_range: fixed 2 for eyewear sides; local rim screws or hinge barrels can be slot-local loop details.
- copied object / naming / placement / joint policy: mirrored left/right temple assemblies should use semantic side names or loop-emitted `temple_{side}` / `hinge_{side}` with identical revolute fold policy and mirrored origins.

## 组合数预审
Slot A(7) x Slot B(2) x Slot C(3) = 42 >= 10 ✓.

## 排除项(未来 compatibility matrix 素材)
- No blocked cells in this batch; all planned glasses variants converged.
- wraparound_shield may constrain bridge candidates because the front lens is one continuous shield; treat bridge geometry as internal/nose-support detail for that candidate.
- Pure tint/material swaps are not slot candidates unless they come with structural frame/lens topology changes.
