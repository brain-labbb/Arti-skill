# Technology / Flashlight — template source map

pattern: parallel_children (body root; head + switch are jointed children / fixed head-optics on body; repeated crenellation ribs are multiplicity)

parents:
- rec_a-yellow-plastic-handheld-flashlight-torch-a-cyl_20260624_122307_472672_5b5f681c ← picture/Technology/Flashlight/002.png — yellow plastic handheld torch: smooth reflector cone head, longitudinal rib bezel, straight barrel, side push button (prismatic), lanyard strap loop. Covers Slot A=smooth-cone, Slot B=straight-tube, Slot C=side-push, Slot D=lanyard-strap, Mult=16 head ribs.
- rec_black-tactical-flashlight-torch-with-a-knurled-g_20260605_173847_924564_baf8fda5 ← picture/Technology/Flashlight/001.png — black tactical torch: crenellated strike bezel, deep reflector cone + twist focus head (continuous), diamond knurled grip, stepped/tapered tube, side push button (prismatic). Covers Slot A=crenellated-strike, Slot B=stepped-tactical, Slot C=side-push + twist-focus, Slot D=none, Mult=8 bezel teeth.

## Slot 候选覆盖

### Slot A: head_form (③ Primary Form Family — head/bezel envelope + optics)
| 候选(未来 module) | source_type | record_id / evidence | 关键 part/visual 名 | 结构特征 | 状态 |
|---|---|---|---|---|---|
| smooth_reflector_cone | forked_anchor | rec_a-yellow...5b5f681c | head_shell, parabolic_reflector, lens_disc, front_bezel_ring, head_rib_{i}, led_bulb | rounded plastic bezel, smooth reflector, big clear lens, longitudinal ribs | converged (origin) |
| crenellated_strike_bezel | forked_anchor | rec_black...baf8fda5 | head_shell, bezel_ring (teeth), reflector, led_emitter, bezel_marker | scalloped attack bezel, deep funnel reflector, twist head | converged (origin) |
| wide_floodlight_head | forked_anchor | rec_flashlight_var_floodhead (← origin A) | head_shell/parabolic_reflector/lens_disc widened | large-diameter shallow reflector dish + big flat lens (flood beam) | converged |
| (template extrapolation) penlight_micro_head / lantern_diffuser_head | world_knowledge_extrapolation (③ Volumetric Envelope) | anchors above + reviewer | 同 head part tree/lathe primitive | only head envelope discretely varies | template-side |

### Slot B: body_form (③ Primary Form Family — body/grip envelope)
| 候选 | source_type | record_id / evidence | 关键 part/visual 名 | 结构特征 | 状态 |
|---|---|---|---|---|---|
| straight_cyl_barrel | forked_anchor | rec_a-yellow...5b5f681c | barrel_shell (Cylinder), shoulder_shell | constant-radius plastic tube, tapered shoulder to head | converged (origin) |
| stepped_tactical_tube | forked_anchor | rec_black...baf8fda5 | body_shell (loft): tail cap + grip swell + front lip | stepped aluminum tube, mid-body swell, seating lip | converged (origin) |
| right_angle_head_body | forked_anchor | rec_flashlight_var_anglehead (← origin A) | barrel_shell + perpendicular head-neck | L-shaped angle-head; beam fires 90° to grip | converged (boundary case — bend baked into body geometry, part tree/joints unchanged; if repeatedly non-convergent, record as blocked) |

### Slot C: switch_mechanism (② joint type / placement)
| 候选 | source_type | record_id / evidence | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|---|
| side_push_button | forked_anchor | rec_a-yellow...5b5f681c & rec_black...baf8fda5 | button (button_base/button_cap) + body_to_button; push_button (button_cap) + button_press | radial inward PRISMATIC press on body surface | converged (both origins) |
| twist_focus_head | forked_anchor | rec_black...baf8fda5 | focus_head + head_focus_twist (CONTINUOUS, axis +X) | head twists about body axis (zoom/focus) | converged (origin B) |
| tailcap_click_switch | forked_anchor | rec_flashlight_var_tailswitch (← origin A) | tail_cap → jointed tail button + tail_press (PRISMATIC +X) | rear click switch presses forward | converged |
| longitudinal_slide_switch | forked_anchor | rec_flashlight_var_slideswitch (← origin A) | slider + body_to_slide (PRISMATIC +X) in recessed track | thumb slider along body axis, off↔on | converged |

### Slot D: carry_feature (提手/握持机构 — minor slot; "none" = origin B is a non-module value)
| 候选 | source_type | record_id / evidence | 关键 part/visual 名 | 结构特征 | 状态 |
|---|---|---|---|---|---|
| lanyard_strap_loop | forked_anchor | rec_a-yellow...5b5f681c | strap (strap_loop tube) + body_to_strap (FIXED), tail_eyelet | flexible wrist strap through tail eyelet | converged (origin A) |
| spring_pocket_clip | forked_anchor | rec_flashlight_var_pocketclip (← origin B) | pocket_clip (inline body visual) | bent metal spring clip on body tube | converged |

## Multiplicity / Copy Logic
- count_param: head_crenellation_count (bezel/strike-rim ridges or teeth around the head circumference) — real `for i in range(n)` loop in BOTH origins.
- copied object: one crenellation element (origin A `head_rib_{i}` Cylinder ridge; origin B `bezel_ring` tooth via `for i in range(teeth)`) — identical geometry helper per copy.
- naming: `head_rib_{i}` (A) / per-tooth loop index (B); placement: equal-angle around the head axis; joint policy: all copies are FIXED inline visuals on the head part (no per-copy joint).
- N 样本已覆盖: {8 → origin B bezel_ring teeth, 12 → rec_flashlight_var_bezelN, 16 → origin A head_rib_{i}}
- 模板建议 N_range: [6, 24] (采样域远大于样本;5–6 点粗齿 strike bezel 到细密防滑肋)
- 次要复制逻辑(记录,不额外 fork): grip_knurl 菱形滚花 `for i in range(28)` × 2 sign lattices (origin B);barrel_grip_{i} × 4 molded rails (origin A);高功率灯头散热 cooling fins 是世界知识里另一条真实 N-copy(两 origin 均无,模板侧可外推)。

## 视觉多样性 6 轴考察(对齐下游 SPEC §8.5)

| 轴 | 处理 | 本小类取值 / 范围 / 理由 |
|---|---|---|
| ① 骨架图(+N) | forked_anchor → 见 Slot A/B/C | body(root) + head(fixed optics 或 twist child) + switch(child);body-form 家族 = 直筒 / 阶梯筒 / 直角头(fork anchor);无世界知识新增 candidate |
| ② 关节类型 | forked_anchor(随 module) | 侧按钮 PRISMATIC(径向内压,A+B)、旋焦头 CONTINUOUS(轴 +X,B)、尾帽 click PRISMATIC(+X,fork)、侧滑 PRISMATIC(+X,fork);无世界知识新增 candidate |
| ③ 主体形态家族 | forked_anchor + world_knowledge_extrapolation | anchors: head_form={smooth_cone, crenellated_strike, floodlight};body_form={straight_tube, stepped_tactical, angle_head}。可外推(Volumetric Envelope):penlight 微头 / lantern 漫射头 / 更长值勤筒身,保持同 part tree/lathe primitive/interface |
| ④ 表面装饰 | record_only + world_knowledge_extrapolation | 真实样本:diamond knurl(B grip_knurl)、molded grip rails(A barrel_grip_{i})、longitudinal head ribs(A head_rib_{i})、bezel teeth(B)、黑色 bezel 环带(A front_bezel_ring/rear_bezel_band)。可外推 host-conformal:rubber grip bands、warning labels/印字、fluting 竖槽(数量档 low/med/high) |
| ⑤ 尺寸/行程 | record_only | 长径比 ~5:1(handheld)→ 更长值勤筒;头径:筒径 ≈1.5–2.2;按钮行程 1.5–4mm、尾 click 2–3mm、侧滑 ~8mm、旋焦 continuous |
| ⑥ 涂装 | record_only | 材质大类:glossy plastic(yellow 002)、matte/anodized aluminum(black 001)、rubber accents;配色 ≥6:safety-yellow、hi-vis orange、matte black、gunmetal、olive-drab、FDE/tan、safety-green、red |

## Compatibility Probes
| probe_id | source_type | record_id | 组合轴值 | 验证目标 | 结论 |
|---|---|---|---|---|---|
| (none planned) | — | — | — | — | 单物体简单,普通候选无高风险跨轴接口;若 angle_head × twist_focus 组合将来上模板需 probe perpendicular-neck 旋焦轴 clearance | n/a |

## 排除项(未来 compatibility matrix 素材)
- 规划阶段无确认排除项。观察风险:right_angle_head_body 是唯一 boundary case(体量包络大改),若连续 2–3 次 compile/tests 不收敛(漂浮头 / neck 穿插 / 出类目),退化为更温和的斜头(canted head)或记为 blocked 并在此登记。
