# Technology / Telescope — template source map (DRAFT, converged)

slug: telescope
pattern: mixed (alt-az/EQ mount hub + parallel optical-tube + multiplicity on tripod legs and draw-tube segments)

parents (2 origins, both source-backed alt-az refractors-on-tripods; both fully accounted):
- A `rec_brass-and-leather-refractor-spyglass-telescope-o_20260605_173839_625665_87949333` ← picture/Technology/Telescope/002.png — brass & leather refractor SPYGLASS on a tall wooden tripod. `tripod` root (leg_hub, pedestal_collar, looped `leg_{index}`/`foot_{index}`/`spreader_{index}`), `azimuth_head` = brass yoke + iron TRUNNION FORK (azimuth_ring, yoke_block, looped `trunnion_plate_{side}`, pivot_axle), `tube` = leather CadQuery loft (leather_body, objective_ring, objective_lens, rear_collar, looped `saddle_lug_{side}`, focus_knob), `draw_tube` (draw_tube, eyecup). Joints: azimuth_rotation (continuous Z), altitude_tilt (revolute -Y), drawtube_extend (prismatic -X). Covers Slot A=leather_tapered_spyglass, Slot B=alt_az_trunnion_tripod(wooden), draw-tube N=1.
- B `rec_small-refractor-telescope-on-an-adjustable-tripo_20260605_173830_455032_6d2f7e30` ← picture/Technology/Telescope/001.png — small blue/white banded refractor on a metal photo tripod. `tripod` root (hub, hub_collar, looped `leg_{i}`/`foot_collar_{i}`/`foot_tip_{i}`/`spreader_{i}`), `azimuth_head` = alt-az U-YOKE (az_turntable, az_post, yoke_base, looped `yoke_cheek_{side}`/`tilt_boss_{side}`, azimuth_marker), `optical_tube` (tube_shell CadQuery, blue_band, dew_shield, objective_ring, cradle_ring, focuser_housing, focus_knob), `focuser_drawtube` (drawtube_barrel, eyepiece_body, eyepiece_cup). Joints: azimuth_rotation (continuous +Z), tube_altitude (revolute -Y), focuser_slide (prismatic -X). Covers Slot A=banded_straight_refractor, Slot B=alt_az_uyoke_tripod(metal), fixed legs, draw-tube N=1.

Readability (§4): BOTH origins pass — tripod legs/feet/spreaders and yoke cheeks/trunnion plates are all emitted by `for i in enumerate(...)` loops with `name_{i}` naming + shared `tube_from_spline_points` helper + equiangular placement. No hand-written leg_1/leg_2/leg_3 repeats. The single draw-tube is one prismatic segment (legitimate N=1); the multi-segment variant must loop `draw_segment_{i}`.

## Slots (2 structural slots + 2 multiplicity/mechanism axes)

- **Slot A — optical_tube family (③ Volumetric Envelope + focuser placement)**: leather_tapered_spyglass(A) / banded_straight_refractor(B) / reflector_newtonian(fork) / maksutov_catadioptric(fork)
- **Slot B — mount family (strongest structural axis; ①/② joint topology)**: alt_az_trunnion_tripod(A) / alt_az_uyoke_tripod(B) / equatorial_eq_counterweight(fork) / dobsonian_rocker_box(fork) / tabletop_pillar_stand(fork)
- **Multiplicity M1 — tripod legs** N=3 (copy logic only, fixed N) + leg MECHANISM (fixed vs telescoping prismatic)
- **Multiplicity M2 — draw-tube telescoping segments** N (1 origin → 3 nested chained-prismatic segments)

## Slot 候选覆盖

### Slot A: optical_tube family
| 候选(未来 module) | source_type | record_id / evidence | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|---|
| leather_tapered_spyglass | forked_anchor | A (parent) | leather_body/objective_ring/rear_collar + draw_tube, altitude_tilt(revolute) | tapered leather CadQuery loft, front brass bezel, rear telescoping draw-tube eyepiece | converged(parent) |
| banded_straight_refractor | forked_anchor | B (parent) | tube_shell/blue_band/dew_shield/focuser_housing + focuser_drawtube | straight cylindrical shell, black dew shield front, rear brass focuser + drawtube | converged(parent) |
| reflector_newtonian | forked_anchor | rec_telescope_var_tube_reflector | tube_shell(fat/open) + secondary spider + focuser_housing/focuser_drawtube RELOCATED to side, focuser_slide(prismatic radial) | fat short open-front OTA, side eyepiece near front top | converged (parent B) |
| maksutov_catadioptric | forked_anchor | rec_telescope_var_tube_maksutov | tube_shell(short/stubby) + front corrector meniscus disc + rear visual back + focuser_drawtube, focuser_slide(prismatic -X) | squat wide closed catadioptric, front corrector lens, rear axial focuser | converged (parent B) |
| (更多 refractor/SCT 主体形态) | world_knowledge_extrapolation (Volumetric Envelope) | anchors: A,B + reflector/maksutov forks + reviewer | 同 part tree/interface | only Volumetric Envelope of the OTA changes (long-thin ↔ short-fat); keep tube+focuser+tilt topology | template-side |

### Slot B: mount family
| 候选(未来 module) | source_type | record_id / evidence | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|---|
| alt_az_trunnion_tripod | forked_anchor | A (parent) | azimuth_ring/yoke_block/trunnion_plate_{side}/pivot_axle; azimuth_rotation(continuous Z)+altitude_tilt(revolute -Y) | tall wooden tripod + brass yoke + iron trunnion fork | converged(parent) |
| alt_az_uyoke_tripod | forked_anchor | B (parent) | yoke_base/yoke_cheek_{side}/tilt_boss_{side}; azimuth_rotation(continuous +Z)+tube_altitude(revolute -Y) | metal photo tripod + U-yoke cheeks + tilt bosses | converged(parent) |
| equatorial_eq_counterweight | forked_anchor | rec_telescope_var_mount_equatorial | polar/wedge block + counterweight_shaft/counterweight_ball; RA(continuous, tilted polar axis)+DEC(revolute) | German EQ head, tilted polar axis + declination + counterweight arm | converged (parent B) |
| dobsonian_rocker_box | forked_anchor | rec_telescope_var_mount_dobsonian | ground_board + side boards + altitude_bearing_{side}; azimuth_rotation(continuous +Z on ground board)+tube_altitude(revolute -Y) | NO tripod: square rocker box on ground, azimuth rocker + side altitude bearings | converged (parent B) |
| tabletop_pillar_stand | forked_anchor | rec_telescope_var_mount_pillar | center post + base_disc + azimuth_ring/yoke_block/trunnion_plate_{side}; azimuth_rotation(continuous Z)+altitude_tilt(revolute -Y) | single turned column on a weighted base disc (desk stand), no legs | converged (parent A) |

## Multiplicity / Copy Logic
- **M1 count_param: `leg_count`** — copied object = splayed tripod leg; naming `leg_{i}` (B) / `leg_{index}` (A) + `foot*_{i}` + `spreader_{i}`; placement = 3 legs equiangular (120°, base angle π/2); joint policy = legs FIXED to the hub (the mount's real joints are azimuth + tilt).
  - N 样本: {3} only — leg count is fixed at 3 for tripods (N is NOT an axis here; the copy logic is the deliverable). 模板建议 N_range: [3,4] (rare 4-leg pier legs); do not sweep N for legs.
  - **Leg MECHANISM sub-axis**: fixed one-piece legs (A,B) vs telescoping/adjustable legs → `rec_telescope_var_legs_telescoping` adds a per-leg PRISMATIC `leg_extend_{i}` (looped two-stage `leg_upper_{i}`/`leg_lower_{i}`, feet on lower stage). This is a mechanism candidate, not an N sweep.
- **M2 count_param: `drawtube_segment_count`** — copied object = concentric brass draw-tube segment; naming `draw_segment_{i}`; placement = nested decreasing-radius along -X; joint policy = linear_chain, each segment its own PRISMATIC (draw_segment_0 out of rear_collar, draw_segment_1 out of draw_segment_0, ...), eyecup on innermost.
  - N 样本已覆盖: {1 (A,B origins), 3 → rec_telescope_var_drawtube_segments}. 模板建议 N_range: [1,4] (single focuser draw-tube ↔ 4-draw pirate spyglass).

## 视觉多样性 6 轴考察(对齐下游 SPEC §8.5)
| 轴 | 处理 | 本小类取值 / 范围 / 理由 |
|---|---|---|
| ① 骨架图(+N) | forked_anchor → 见 Slot 覆盖 / Multiplicity | tube+mount+draw-tube tree; mount family swaps root (tripod ↔ EQ ↔ rocker-box ↔ pillar); legs looped N=3, draw segments looped N∈[1,4]. No world-knowledge new skeleton candidates. |
| ② 关节类型 | forked_anchor (随 module) | azimuth CONTINUOUS(Z / tilted polar RA); altitude/dec REVOLUTE(-Y); focuser/draw-tube PRISMATIC(-X or radial); telescoping-leg PRISMATIC. Every mount keeps ≥2 non-fixed joints; no world-knowledge new joint candidates. |
| ③ 主体形态家族 / Primary Form Family | forked_anchor + world_knowledge_extrapolation | source anchors: leather_spyglass, banded_refractor, reflector_newtonian, maksutov. Extrapolatable = **Volumetric Envelope** (long-thin refractor ↔ short-fat catadioptric/SCT) + **Macro Surface** (open-truss vs solid OTA) — keep same tube+focuser+tilt tree. |
| ④ 表面装饰 | record_only + world_knowledge_extrapolation | real: brass bezel/collar rings, leather wrap seam, blue accent band, focus_knob, azimuth_marker, finder-scope, slow-motion knobs, accessory rail. Host-conformal extrapolation only (rings/labels/knurling/finder), non-structural. |
| ⑤ 尺寸/行程 | record_only | tube L:D refractor [4,10] vs catadioptric [1,2.5]; tripod height [0.3,1.4]; altitude tilt [-30°,+60°]; azimuth continuous; focuser travel [0,0.03], spyglass draw [0,0.05]. |
| ⑥ 涂装 | record_only | metal(brass/chrome/anodized), painted(white/blue/orange/black), leather+wood(spyglass), plastic(consumer). ≥6 liveries: brass-and-leather / blue-white / matte-white reflector / pearl-white catadioptric / gloss-black / green-enamel. |

## Compatibility Probes
| probe_id | source_type | record_id | 组合轴值 | 验证目标 | 结论 |
|---|---|---|---|---|---|
| (none planned) | — | — | reflector_newtonian × dobsonian_rocker is the canonical real pairing but each is proven independently; no interface risk needing a probe | — | — |

## 排除项(未来 compatibility matrix 素材)
- handheld / no-mount spyglass — real but drops azimuth+tilt (leaves only draw-tube prismatic); represented instead by tabletop_pillar_stand which keeps the mount joints. Recorded as ⑤ "collapse mount to zero" extrapolation, not a fork.
- Fork mount (dual-arm SCT/Dobsonian truss fork) folds into dobsonian_rocker_box (rocker) + alt_az_uyoke (fork cheeks) candidates; not separately forked.
- Leg N-sweep (2/4/5 legs) — out of vocabulary; tripods are N=3. Not forked.

## Notes
- Downstream `astronomical_telescope_on_tripod` template exists but is NOT linked to these seeds; this map is planned fresh from origins A/B.
- Sync: workbench-only both sides; never promote / never pass --category-slug. Batch-write rating=5 on sync.
