# Sports / Fidget Spinner — template source map

pattern: multiplicity
parents: rec_red-three-arm-fidget-spinner-with-a-central-bear_20260605_165852_289472_5329f102 ← picture/Sports/Fidget Spinner/001.png (covers Slot A=round_pods, Slot B=open_bearing, Slot C=flat_button, N=3)

Whole-object structure: a central pinch cap (`center_cap`, the held ROOT) carries the silver hub barrel and one red button per face; the `spinner_body` is the spinning tri-lobe plate (CONTINUOUS revolute `cap_to_body` about +Z — the primary non-fixed joint, axisymmetric); each lobe holds a `bearing_i` that itself spins CONTINUOUSLY about its own lobe axis (`body_to_bearing_i`). Bearings + lobes are the multiplicity unit. Per memory note: axisymmetric spinners can fail AABB spin checks, so the body keeps off-axis lobes and each bearing race carries an off-axis marker tab to make spin detectable.

## Slot 候选覆盖

### Slot A:body_outline (spinner_body plate form)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| round_pods | rec_red-three-arm-fidget-spinner-with-a-central-bear_20260605_165852_289472_5329f102 | spinner_body / tri_lobe_body visual; helper _tri_lobe_body; cap_to_body (continuous +Z) | separate round weight pods on tapered arm webs, open gaps between arms | converged (parent) |
| solid_disc | rec_fidget_spinner_var_solid_disc | spinner_body / solid disc visual; cap_to_body | one continuous rounded-triangle (Reuleaux) filled plate, no open gaps, bearings bored straight through | built ✓ |
| gear_edge | rec_fidget_spinner_var_gear_edge | spinner_body / lobe + tooth_i inline visuals; tooth helper; cap_to_body | gear-style toothed rim around each lobe (sawtooth perimeter, inline parent visuals) | built ✓ |

### Slot B:lobe_weight (what fills/covers each lobe pocket)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| open_bearing | rec_red-three-arm-fidget-spinner-with-a-central-bear_20260605_165852_289472_5329f102 | bearing_i (ring + race + marker); helpers _bearing_ring_mesh/_bearing_race_mesh; body_to_bearing_i (continuous) | exposed skateboard bearing: black rubber outer ring + silver race + open center hole | converged (parent) |
| domed_weight | rec_fidget_spinner_var_domed_weight | weight_i (domed cap visual); shared lathe/CadQuery dome helper; body_to_weight_i (continuous) | closed polished convex metal dome capping each lobe seat, off-axis facet for spin detect | built ✓ |
| hex_weight | rec_fidget_spinner_var_hex_weight | weight_i (hex prism visual); shared hexagon helper; body_to_weight_i (continuous) | six-sided brass hex-nut weight with small center hole; six flats make spin AABB-detectable | built ✓ |

### Slot C:center_cap (pinch hub form)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| flat_button | rec_red-three-arm-fidget-spinner-with-a-central-bear_20260605_165852_289472_5329f102 | center_cap / cap_button_top + cap_button_bottom (flat cylinder discs); cap_hub barrel | flat red button disc proud of each face | converged (parent) |
| domed_cap | rec_fidget_spinner_var_domed_cap | center_cap / domed pinch visuals (revolved); cap_hub | convex domed/mushroom pinch dome on each face, true lathe surface | built ✓ |
| knurled_cap | rec_fidget_spinner_var_knurled_cap | center_cap / cap body + ridge_i inline visuals; ridge helper; cap_hub | tall knurled metal spin-cap on top face (+ low knurled bottom), fine vertical ridges as inline visuals | built ✓ |

> center_cap is the held ROOT in every Slot C candidate; the body-to-cap central spin joint (cap_to_body, continuous +Z) stays the active non-fixed axisymmetric joint and the cap stays still while the body spins.

## Multiplicity / Copy Logic
- count_param: arm_count (number of lobes/arms; each lobe carries one bearing/weight). Parent hand-builds around 3 (LOBE_ANGLES tuple + per-lobe loops); the multiplicity variants must promote this to a single arm_count param driving 360/arm_count angular spacing.
- N 样本已覆盖: {2, 3, 4, 5} → rec_fidget_spinner_var_arms2 / parent (N=3) / rec_fidget_spinner_var_arms4 / rec_fidget_spinner_var_arms5
- 模板建议 N_range: [2, 8] (2-bar through multi-arm star; real fidget spinners run 2..~7 arms; >8 stops reading as a spinner)
- copied object: one lobe unit = {lobe disc + connecting web on body (parent visual)} + {one bearing/weight part with its own continuous spin joint}.
- naming: lobe_i / web_i inline on body; bearing_i (or weight_i) child parts; joints body_to_bearing_i.
- placement: equal angular spacing ang_i = base_angle + i * 2*pi / arm_count at fixed lobe radius LOBE_DIST; lobe centers (LOBE_DIST*cos, LOBE_DIST*sin).
- joint policy: each lobe part is its own CONTINUOUS revolute about its lobe +Z axis relative to the body; uniform across all i. The central cap_to_body CONTINUOUS +Z joint is independent and unchanged by N.

## 排除项(未来 compatibility matrix 素材)
- N=1 不收敛/出类目:单臂只有中心轴一个旋转面,读不出 fidget spinner,排除(N_range 下界取 2)。
- 待跑后回填:若 solid_disc × gear_edge 在真实物体上无法共存(实心盘把齿吃掉)或 knurled_cap 在 N=2 dumbbell 上 hub 长度与 bar 厚度干涉,记为组合排除项。
- gear_edge 若 CadQuery 布尔齿在 segments 偏高时出现 "Profile area must be non-zero" 退化(见 roller-skate memory),退回较低分段或较粗齿距。

---
## Post-fork verification (SEGMENT 1 complete)
All planned variants forked via `articraft fork` (dashscope qwen3.7-max, thinking medium), then verified on-disk: last compile = success, ≥1 non-fixed joint present, collections=['workbench'] (workbench-only, not promoted), and picture.json bound into the correct `Sports__<小类>` subcat shard (reconcile rebuilt). Status cells above flipped planned→built ✓ accordingly. Ready for SEGMENT 2 (spec authoring).
