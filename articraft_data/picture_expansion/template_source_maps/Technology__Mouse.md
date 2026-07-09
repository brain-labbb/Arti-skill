# Technology / Mouse — template source map

pattern: mixed (fixed named controls: left/right click + wheel + dpi; multiplicity side-button cluster)

parents (5 origins, all accounted; each occupies its own ③ form cell for free):
- rec_a-white-wireless-gaming-mouse-a-symmetrical-ergo_20260624_124409_453756_e0fd3af7 ← picture/Technology/Mouse/005.png — covers ③=symmetrical-ambidextrous-low-gaming; ② click=REVOLUTE(-X), wheel=CONTINUOUS(X), dpi=PRISMATIC, side_button×2 (loop-emitted); MULTIPLICITY N=2 side buttons.
- rec_a-black-wireless-gaming-mouse-an-ergonomic-right_20260624_124020_837973_714f0615 ← picture/Technology/Mouse/003.png — covers ③=ergonomic-right-hand-hump; ② click=PRISMATIC(-Z), wheel=CONTINUOUS(Y), dpi=PRISMATIC, thumb_button×2 (loop-emitted); MULTIPLICITY N=2 thumb buttons.
- rec_an-apple-magic-mouse-a-smooth-seamless-one-piece_20260624_124116_679456_0a1a21d6 ← picture/Technology/Mouse/004.png — covers ③=seamless-touch-slab; ② = NO discrete buttons/wheel (all joints FIXED, 0 active). MULTIPLICITY N=0.
- rec_dark-gray-ergonomic-wireless-mouse-with-left-and_20260605_173935_903765_680c4ad1 ← picture/Technology/Mouse/002.png — covers ③=ergonomic-dome-standard; ② click=PRISMATIC(-Z), wheel=CONTINUOUS(Y); MULTIPLICITY N=0 side buttons.
- rec_small-black-wireless-computer-mouse-with-left-an_20260605_173916_347046_2f6f2c2b ← picture/Technology/Mouse/001.png — covers ③=compact-rounded-standard; ② click=REVOLUTE(+Y hinge), wheel=CONTINUOUS(Y); MULTIPLICITY N=0.

## Slot 候选覆盖

### Slot A：③ Primary Form Family (body shell)
| 候选 (未来 module) | source_type | record_id / evidence | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|---|
| symmetrical_ambidextrous_low | forked_anchor | rec_a-white-...e0fd3af7 | shell / main_shell (lofted footprint + palm_hump) | low symmetric gaming egg, both-side buttons | converged (origin) |
| ergonomic_right_hump | forked_anchor | rec_a-black-...714f0615 | body / ergonomic_shell | right-handed sculpted hump + thumb rest + grips | converged (origin) |
| seamless_touch_slab | forked_anchor | rec_an-apple-...0a1a21d6 | mouse_body / smooth_top_shell | one-piece low dome, no discrete buttons | converged (origin) |
| ergonomic_dome_standard | forked_anchor | rec_dark-gray-...680c4ad1 | body / body_shell + lower_band | medium ergonomic dome, 3-button | converged (origin) |
| compact_rounded_standard | forked_anchor | rec_small-black-...2f6f2c2b | body / body_shell (arched ridge) | small rounded travel mouse | converged (origin) |
| vertical_handshake | forked_anchor (FORK) | rec_mouse_var_vertical ← parent 714f0615 | body / ergonomic_shell (retilted) | tall ~57° near-vertical click face + thumb rest | converged |
| trackball_housing | forked_anchor (FORK) | rec_mouse_var_trackball ← parent 680c4ad1 | body / body_shell + new `trackball` | stationary housing + large rolling ball | converged (borderline category) |
| primary_form_extra (e.g. low-profile-flat, tall-arch-vertical mid variants) | world_knowledge_extrapolation (Volumetric Envelope / Macro Surface) | anchors: the 7 above + reviewer | same part tree/interface | only Volumetric Envelope / Macro Surface form varies | template-side |

### Slot B：② Scroll / pointer mechanism (joint type)
| 候选 (未来 module) | source_type | record_id / evidence | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|---|
| notched_wheel_continuous | forked_anchor | rec_dark-gray-...680c4ad1 (+ 001/003/005) | scroll_wheel / body_to_scroll_wheel (CONTINUOUS Y) | ribbed rim + off-axis marker | converged (origin) |
| touch_top_no_wheel | forked_anchor | rec_an-apple-...0a1a21d6 | mouse_body (all FIXED) | seamless capacitive top, no wheel part | converged (origin) |
| tilt_scroll_wheel | forked_anchor (FORK) | rec_mouse_var_tiltwheel ← parent 680c4ad1 | new `wheel_yoke` (REVOLUTE X tilt) + scroll_wheel (CONTINUOUS Y) | 2-DOF wheel: spin + L/R rocker | converged |
| trackball_ball_spin | forked_anchor (comes with trackball form) | rec_mouse_var_trackball | trackball (CONTINUOUS) | ball rotation as pointing | converged |

Note: primary-CLICK joint type is also ② and is already source-backed with ≥2 candidates from origins: REVOLUTE click (005, 001) vs PRISMATIC click (003, 002). No fork needed for click-type.

## Multiplicity / Copy Logic
- count_param: side_button_count (thumb/side button cluster on the grip face)
- N 样本已覆盖: {0, 2, 6} → 0: 002/001/004(apple) · 2: 005 (side_button_0/1, loop) & 003 (thumb_button_0/1, loop) · 6: rec_mouse_var_sidebtncluster ← parent 714f0615
- 模板建议 N_range: [0, 12] (MMO clusters realistically reach ~12; sampler域 >> 样本)
- copied object / naming / placement / joint policy: one `_rounded_button` mesh cap; named `thumb_button_{i}` / `side_button_{i}`; placed on a regular equal-spacing 1-D row or 2-D grid on the +Y (or side) grip face; each an independent PRISMATIC inward press (parallel_children, all children of body). Already loop-emitted in both gaming origins — clean copy logic; the N=6 fork only widens the loop range + grows `thumb_recess` support face.

## 视觉多样性 6 轴考察

| 轴 | 处理 | 本小类取值 / 范围 / 理由 |
|---|---|---|
| ① 骨架图 (+N) | forked_anchor → Slot A/B + Multiplicity | body(root) + {2 primary clicks} + {wheel OR touch-top OR trackball} + {dpi} + {side/thumb ×N}. No world-knowledge-invented skeletons. |
| ② 关节类型 | forked_anchor (随 module) | click: REVOLUTE(-X / +Y hinge) or PRISMATIC(-Z); wheel: CONTINUOUS(spin axis Y or X); tilt add: REVOLUTE(X); dpi/side: PRISMATIC; trackball ball: CONTINUOUS; touch-top: all FIXED. |
| ③ 主体形态家族 | forked_anchor + world_knowledge_extrapolation | anchors: symmetrical-ambidextrous, ergonomic-right-hump, seamless-touch-slab, ergonomic-dome, compact-rounded, vertical-handshake(FORK), trackball-housing(FORK). Extrapolate Volumetric Envelope (hump height/tilt) + Macro Surface (touch vs seamed) between anchors. |
| ④ 表面装饰 | record_only + world_knowledge_extrapolation | observed: textured rubber side grips + ribs (003), center seam / button-rear seam (005), RGB accent strips + dpi glow (005), glossy lower band (001/002), brand logo disc/decal (003/004), wheel tread ribs (all). Extrapolate host-conformal ribs/seams/labels/knurling counts. |
| ⑤ 尺寸/行程 | record_only | length ~0.10–0.114 m, width ~0.057–0.066 m, height ~0.022–0.046 m; click travel: revolute ~7° / prismatic ~1.5–2 mm; wheel continuous; side/dpi press ~1.5–2.5 mm. |
| ⑥ 涂装 | record_only | material大类: matte/satin plastic, glossy plastic, rubberized grip, thin glide/PTFE, metal axle. 配色: matte-white, matte-black, dark-gray, glossy-white, two-tone, RGB accents (≥6). Sampler/palette free. |

## Compatibility Probes
| probe_id | source_type | record_id | 组合轴值 | 验证目标 | 结论 |
|---|---|---|---|---|---|
| (none planned) | — | — | — | — | — |

## 排除项 (未来 compatibility matrix 素材)
- trackball_housing (rec_mouse_var_trackball): borderline ③ anchor — if普通人 reads it as a "trackball" not a "mouse", or the ball socket won't converge (float/interpenetration), record here as excluded and fold Slot A back to the 6 remaining form anchors. Not yet excluded — attempt fork first.
- touch_top_no_wheel origin (apple, 004) has 0 non-fixed joints by nature; NOT a valid fork parent for any variant that must add motion — do NOT fork from it (any child must add a non-fixed joint; not needed since other origins cover the form cells).

## Notes
- Sync: workbench-only, script copy; never promote; batch rating=5 on sync.
- 4 forks planned (vertical, trackball, tiltwheel, sidebtncluster) — 5 origins pre-fill most cells; cells count is small by design (§2). Each fork keeps ≥1 non-fixed joint (clicks + wheel/ball; multiplicity fork keeps full baseline joint set).
- Readability: NO hand-written-repeat violations — side/thumb buttons, grip ribs, RGB strips, wheel treads are all already `for`-loop emitted in the origins.
