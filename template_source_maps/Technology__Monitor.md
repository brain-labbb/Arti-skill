# Technology / Monitor — template source map

pattern: parallel_children (panel + stand support tree; no strong N-copy multiplicity)

parents:
- rec_a-standard-16-10-widescreen-desktop-lcd-monitor-_20260624_123957_925849_015db054 <- picture/Technology/Monitor/002.png
  (covers: ③ flat_16_10 panel × Stand flat_plate_pillar_base × Mechanism tilt_only)
- rec_ultrawide-curved-computer-monitor-on-a-central-s_20260605_173926_571270_0ea51d17 <- picture/Technology/Monitor/001.png
  (covers: ③ superwide_curved_32_9 panel × Stand V_foot_central_neck × Mechanism tilt+swivel+height)

Identity:
- a desktop computer monitor: one display panel carried by a support (desk stand / arm / wall bracket)
- MUST keep at least the panel tilt joint; a fixed no-DOF monitor is rejected
- NOT a TV cabinet, NOT a laptop, NOT an all-in-one PC, NOT a tablet

Origin A part vocabulary (widescreen_lcd_monitor): part `stand` {hex_base, slotted_pillar, base_collar, top_crosshead, yoke_arm_{i}, hinge_cheek_{i}} + part `screen` {rear_shell, bezel(BezelGeometry), display_panel, rear_mount, hinge_barrel}; joint stand_to_screen (REVOLUTE X = tilt).
Origin B part vocabulary (curved_monitor): parts `base_foot`{base_foot_shell}, `neck_riser`{neck_riser_shell, swivel_hub}, `neck_carriage`{carriage_plate, tilt_barrel}, `screen_panel`{panel_housing(loft), screen_glass(loft), vesa_mount}; joints base_to_neck_swivel (REVOLUTE Z), neck_to_carriage_height (PRISMATIC Z), carriage_to_screen_tilt (REVOLUTE X).

## Slot 候选覆盖

### Slot A: Stand support family (base topology) — strongest structural axis
| 候选(未来 module) | source_type | record_id / evidence | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|---|
| flat_plate_pillar_base | forked_anchor | parent A | stand: hex_base + slotted_pillar + top_crosshead/yoke head; stand_to_screen tilt | single flat plate foot + central slotted column | converged(parent) |
| V_foot_central_neck | forked_anchor | parent B | base_foot_shell (V/T foot) + neck_riser + neck_carriage | wide splayed V-foot lying flat + central neck | converged(parent) |
| twin_leg_A_base | forked_anchor | rec_monitor_var_twin_leg_base | leg_{i} for-loop + central column, keeps top_crosshead/yoke + stand_to_screen tilt | two discrete splayed legs meeting at a column | converged |
| ergonomic_arm | forked_anchor | rec_monitor_var_ergonomic_arm | desk_clamp + arm_lower/elbow/arm_upper + vesa_head; multi-revolute arm | dual-hinge desk-clamp articulating arm (no desk plate) | converged |
| vesa_wall_mount | forked_anchor | rec_monitor_var_wall_mount | wall_plate + mount_bracket + bracket_to_screen tilt | wall plate + short bracket, NO ground base | converged |

### Slot B: ③ Primary Form Family (panel form)
| 候选(未来 module) | source_type | record_id / evidence | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|---|
| flat_16_10 | forked_anchor | parent A | screen: bezel(BezelGeometry) + display_panel (flat Box) | flat rectangular panel, ~16:10 | converged(parent) |
| superwide_curved_32_9 | forked_anchor | parent B | screen_panel: panel_housing + screen_glass (YZ-section loft along X) | extreme concave super-ultrawide slab (~3.4:1, deep bow) | converged(parent) |
| curved_21_9 | forked_anchor | rec_monitor_var_curved_21_9 | same panel_housing/screen_glass loft, retuned PANEL_W/PANEL_H/BOW | moderate curved ultrawide (~2.3:1, gentle bow) | converged |
| flat_16_9 / flat_ultrawide_flat / aspect ratios | world_knowledge_extrapolation (Planar Boundary + Volumetric Envelope) | anchors: parents A/B + curved_21_9 + reviewer | same screen part tree + bezel/loft primitive | aspect ratio + curvature depth are continuous form params template extrapolates | template-side |

### Slot C: ② stand MECHANISM (joint set on panel↔support path)
| 候选(未来 module) | source_type | record_id / evidence | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|---|
| tilt_only | forked_anchor | parent A | stand_to_screen (REVOLUTE X) | 1 DOF: panel tilt | converged(parent) |
| tilt+swivel | forked_anchor | rec_monitor_var_tilt_swivel | new base_to_column swivel (REVOLUTE Z) + stand_to_screen tilt | 2 DOF: swivel base + tilt | converged |
| tilt+pivot (portrait) | forked_anchor | rec_monitor_var_portrait_pivot | new pivot (REVOLUTE Y, panel-normal) + stand_to_screen tilt | 2 DOF: landscape↔portrait pivot + tilt | converged |
| tilt+swivel+height | forked_anchor | parent B | base_to_neck_swivel (REV Z) + neck_to_carriage_height (PRIS Z) + carriage_to_screen_tilt (REV X) | 3 DOF full ergonomic | converged(parent) |

Note: Slot A and Slot C are correlated in the real world (arm/wall-mount candidates carry their own joint sets), but each fork changes exactly ONE slot vs its nearest origin so diffs stay single-axis. The arm's extra revolutes are intrinsic to the ergonomic_arm base candidate, not a separate mechanism experiment.

## Multiplicity / Copy Logic
- count_param: none dominant — a monitor has no strong "same subpart × N" structural axis. Weak candidates: OSD/bezel buttons (a short button row) and VESA bolt holes (×4) — both cosmetic ④, not a structural slot. twin_leg_A_base uses a fixed 2-leg for-loop (leg_{i}), and Origin A already loops yoke_arm_{i}/hinge_cheek_{i} (2 copies) — these are structural-pair loops, not a tunable N.
- N 样本已覆盖: n/a (no dedicated N variant; N would only be button count if buttons are ever added)
- 模板建议 N_range: OSD button row [0, 6] if buttons authored; else none
- copied object / naming / placement / joint policy: if OSD buttons authored -> `button_{i}` for-loop, equal spacing along the bezel bottom edge, all FIXED visuals on the bezel (no joint). Stand feet/legs: `leg_{i}` mirrored across x, FIXED to the base/column.

## 视觉多样性 6 轴考察

| 轴 | 处理 | 本小类取值 / 范围 / 理由 |
|---|---|---|
| ① 骨架图(+N) | forked_anchor → 见 Slot A/B/C | flat-panel-on-desk-stand (A), curved-panel-on-neck-carriage (B), twin-leg, arm-tree, wall-bracket; no world-knowledge new skeleton beyond forked anchors |
| ② 关节类型 | forked_anchor(随 module) | REVOLUTE X (tilt, always kept), REVOLUTE Z (swivel), PRISMATIC Z (height), REVOLUTE Y (portrait pivot); arm adds shoulder/elbow revolutes |
| ③ 主体形态家族 / Primary Form Family | forked_anchor + world_knowledge_extrapolation | anchors: flat_16_10 (A), superwide_curved_32_9 (B), curved_21_9 (fork). Extrapolate: Planar Boundary = aspect ratio (16:9 / 16:10 / 21:9 / 32:9); Volumetric Envelope = flat↔curved bow depth. Same screen part tree + bezel/loft primitive. |
| ④ 表面装饰 | record_only + world_knowledge_extrapolation | observed: near-frameless vs thick bezel (A moderate), rear_shell/rear_mount VESA boss, cable slot in slotted_pillar, brand label. Extrapolate host-conformal: bezel-bottom logo, OSD button row, power LED, rear ventilation ribs — all as visuals, non-structural. |
| ⑤ 尺寸/行程 | record_only | panel aspect ~1.6 (A) to ~3.4 (B); tilt ~[-13°,+15°], swivel ~±30°, height ~±40 mm, pivot 0..90°; bezel width, stand height are continuous params |
| ⑥ 涂装 | record_only | material: plastic housing + glass screen + metal/plastic base. Palette ≥3-6: matte black, graphite/dark-silver (A), silver/white bezel, brushed-aluminium foot, white consumer, gunmetal pro |

## Compatibility Probes
| probe_id | source_type | record_id | 组合轴值 | 验证目标 | 结论 |
|---|---|---|---|---|---|
| (none planned) | — | — | — | — | — |

Note: superwide_curved_32_9 × {wall_mount, ergonomic_arm, portrait_pivot} are physically unrealistic (a 49" superwide is not desk-arm/portrait mounted) — treated as gated/excluded combos in the future compatibility matrix, NOT forked. That is why all stand/mechanism variants fork from the flat-panel origin A and only the moderate-curve form (curved_21_9) forks from B.

## 排除项(未来 compatibility matrix 素材)
- superwide_curved_32_9 × portrait_pivot: excluded (superwides do not pivot to portrait) — realism gate, not a compile failure.
- superwide_curved_32_9 × ergonomic_arm / wall_mount: gated (mass/width beyond typical arm/bracket) — record as compatibility-matrix gate.

## Readability review (§4) of the two origins
- No hand-written N-repeat violations: Origin A's yoke_arm_{i}/hinge_cheek_{i} are already for-loop emitted (2 copies via enumerate); Origin B foot/neck are single union solids. No OSD-button arrays exist in either model, so no button multiplicity to fix.
- Real articulations confirmed: tilt REVOLUTE X present in both; B adds swivel REVOLUTE Z + height PRISMATIC Z. These are the genuine monitor DOFs (tilt kept on every variant).
- Guidance for forks: any repeated arm hardware (ergonomic_arm) or legs (twin_leg_base) or added OSD buttons MUST be for-loop `_{i}` chains; decorative labels/LEDs as parent.visual, not FIXED parts.

status: converged (planning only — no `articraft fork` run)
