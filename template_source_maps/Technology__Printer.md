# Technology / Printer — template source map (DRAFT, converged)

pattern: mixed (named functional slots + a small multiplicity axis on control-panel buttons)

parents (origins; each = one reference image, no parent_record_id):
- rec_a-white-hp-deskjet-compact-all-in-one-inkjet-pri_20260624_124934_795287_c7dd9a61 ← picture/Technology/Printer/003.png — HP DeskJet compact flatbed AIO. Covers: A1 flatbed_scanner_top, B1 rear_vertical_feed+output_extension, C1 flat_button_strip, N=3 buttons.
- rec_a-white-and-gray-hp-officejet-pro-all-in-one-ink_20260624_124702_530779_bbac26a7 ← picture/Technology/Printer/002.png — HP OfficeJet Pro ADF office AIO. Covers: A2 adf_document_feeder_top, B2 front_cassette_drawer, C2 fixed_flush_touchscreen.
- rec_white-brother-all-in-one-inkjet-printer-with-a-f_20260605_173945_310830_9cae5bca ← picture/Technology/Printer/001.png — white Brother flatbed AIO. Covers: A1 flatbed_scanner_top, B3 front_output_tray+stopper, C1 flat_button_strip, N=4(+1 slider) buttons.

## Slot 候选覆盖

### Slot A: primary_form_family (③ body + scanner/top structure)
| 候选 (future module) | source_type | record_id / evidence | key part/joint names | 结构特征 | 状态 |
|---|---|---|---|---|---|
| flatbed_scanner_top | forked_anchor | DeskJet + Brother origins | scanner_lid (lid_panel/lid_frame, cover_top) / body_to_scanner_lid REVOLUTE (axis -X, rear hinge) | flatbed glass + hinged document-cover lid on top | converged (origin) |
| adf_document_feeder_top | forked_anchor | OfficeJet origin | scanner_lid (adf_upper_cover, adf_white_feed_face, adf_top_tray, adf_rear_hinge_strip) / chassis_to_scanner_lid REVOLUTE | automatic-document-feeder hump built onto the lid | converged (origin) |
| flat_top_no_scanner (single-function) | forked_anchor | rec_printer_var_singlefunction (← DeskJet) | rounded_body closed flat top; scanner_lid + body_to_scanner_lid REMOVED | print-only inkjet, no scan glass/lid; articulation via output extension | converged |
| tall_workgroup_body | forked_anchor | rec_printer_var_workgroup (← OfficeJet) | chassis (lower_side_0/1 grown, taller Z/D) | tall floor/desk workgroup envelope, same ADF top + cassette | converged |
| ink_tank_supertank | forked_anchor | rec_printer_var_inktank (← Brother) | body_shell + side ink-tank reservoir visuals (+X bulge) | supertank/EcoTank side reservoir enlarges body silhouette | converged |
| primary_form_extra (extrapolation) | world_knowledge_extrapolation (Volumetric Envelope / Macro Surface) | anchors: above 5 + reviewer | same part tree / same primitives / same interfaces | e.g. wide-format, portable slim, dome-top — template-side only | template-side |

### Slot B: paper_handling (input + output tray mechanism; ② joint types)
| 候选 (future module) | source_type | record_id / evidence | key part/joint names | 结构特征 | 状态 |
|---|---|---|---|---|---|
| rear_vertical_feed + output_extension | forked_anchor | DeskJet origin | rear_tray (upright_support) FIXED + paper_stack; output_tray + extension_arm / output_tray_to_extension_arm PRISMATIC | upright rear feed stack + front sliding output extension | converged (origin) |
| front_cassette_drawer | forked_anchor | OfficeJet origin | input_tray (tray_floor, tray_side_0/1) / chassis_to_input_tray PRISMATIC (front pull-out) | bottom pull-out paper cassette | converged (origin) |
| front_output_tray + stopper | forked_anchor | Brother origin | output_tray (tray_plate, tray_rail_l/r) / body_to_output_tray PRISMATIC; paper_stopper (stopper_flap) / tray_to_paper_stopper REVOLUTE | front pull-out output tray + flip-up paper stopper (nested joint) | converged (origin) |
| foldout_rear_input (revolute) | forked_anchor | rec_printer_var_foldinput (← DeskJet) | body_to_rear_tray re-typed FIXED→REVOLUTE (axis -X); paper_stack rides along | rear paper support folds flat/up instead of rigid standing | converged |

### Slot C: control_panel (② joint type on the operator panel)
| 候选 (future module) | source_type | record_id / evidence | key part/joint names | 结构特征 | 状态 |
|---|---|---|---|---|---|
| flat_button_strip | forked_anchor | DeskJet (control_deck + 3 buttons) + Brother (control_panel + button_0..4) | body_to_{power,cancel,wireless}_button / body_to_button_i PRISMATIC press caps | flat panel band with pressable button row + printed display | converged (origin) |
| fixed_flush_touchscreen | forked_anchor | OfficeJet origin | control_panel_bezel + touchscreen_glass + screen_icon_0/1/2 (chassis visuals, FIXED) | large flush color touchscreen, no moving panel | converged (origin) |
| tilting_touchscreen_panel (revolute) | forked_anchor | rec_printer_var_tiltpanel (← OfficeJet) | new control_panel part / chassis_to_control_panel REVOLUTE (axis -X, bottom-edge hinge) | touchscreen tilts up/out from flush | converged |

## Multiplicity / Copy Logic
- count_param: panel_button_count (control-panel round push-buttons)
- N 样本已覆盖: {3 → DeskJet (power/cancel/wireless), 4 → Brother button_0..3 + button_4 slider, 6 → rec_printer_var_buttoncount}
- 模板建议 N_range: [2, 8] (typical inkjet operator panels)
- copied object / naming / placement / joint policy: round Cylinder cap; `button_{i}`; evenly spaced along X on the panel band at constant Y/Z; each cap an independent `body_to_button_{i}` PRISMATIC press-in joint (parallel children of body).
- 备注: 次要复制单元 feed_roller_{0,1,2} (DeskJet, N=3) 和 sliding paper-width guides (未在任何 origin 建模, prismatic ×2 mirror) 记录但不单独 fork; 若模板侧启用须写成循环 (见下方可读性 flag)。

## 视觉多样性 6 轴考察 (对齐 SPEC §8.5)
| 轴 | 处理 | 本小类取值 / 范围 / 理由 |
|---|---|---|
| ① 骨架图 (+N) | forked_anchor → Slot A/B/C | body(root) + scanner_lid + input/output tray(s) + control panel; N-copy only on panel buttons (Multiplicity). No world-knowledge new candidate. |
| ② 关节类型 | forked_anchor (随 module) | scanner lid REVOLUTE (axis -X); output extension / cassette / output tray PRISMATIC; paper stopper REVOLUTE (nested on tray); buttons PRISMATIC; NEW rear-input REVOLUTE (foldinput) + control-panel REVOLUTE (tiltpanel). |
| ③ 主体形态家族 | forked_anchor + world_knowledge_extrapolation | anchors: flatbed_scanner_top, adf_feeder_top, flat_top_no_scanner, tall_workgroup_body, ink_tank_supertank. Extrapolate (Volumetric Envelope / Macro Surface): wide-format, slim portable, dome-top — same part tree/primitives/interface. |
| ④ 表面装饰 | record_only + world_knowledge_extrapolation | observed: HP roundel + strokes, brand plates (deskjet_label / brand_plate / brother), screen app-icons, dark output-slot shadow, feed rollers, cartridge-door seams/labels. Extrapolate host-conformal labels/seams/vents. |
| ⑤ 尺寸/行程 | record_only | body W 0.40–0.50, D 0.31–0.42, H 0.13–0.17 (compact) → workgroup taller; lid open 0–70°; tray pull-out 0.07–0.16 m; button press ~1.5 mm; stopper 0–80°. |
| ⑥ 涂装 | record_only | plastic (matte white, off-white, white+gray two-tone, charcoal); accents: yellow-green extension, cyan/green/magenta screen icons, translucent ink-tank window. ≥4 colorways. |

## Compatibility Probes
| probe_id | source_type | record_id | 组合轴值 | 验证目标 | 结论 |
|---|---|---|---|---|---|
| (none planned) | — | — | — | — | — |

Note: no compatibility_probe planned. Highest cross-slot risk = ADF top (A2) × front output tray+stopper (B3) clearance under the ADF hump, and tilting panel (C3) × front cassette drawer (B2) front-face contention — flagged for template-side compatibility matrix, not fork-probed here.

## 排除项
- (none yet — no unconverged axis values at planning time.)

## 可读性 flags (§4 code-shape, for downstream template extraction)
- DeskJet origin: `feed_roller_0/1/2` (3 near-identical visuals) and `hinge_saddle_0/1` are hand-unrolled, NOT a `for` loop → violates code-shape rule 2. Its 3 buttons use a named-tuple loop (power/cancel/wireless) — acceptable but not a `range(n)` multiplicity loop. Forks from DeskJet (singlefunction, foldinput) KEEP these as-is (they do not touch that layer); a future feed-roller multiplicity module must rewrite them as a loop.
- OfficeJet origin (213 lines): NOT oversimplified for rejection — it has 2 real non-fixed joints (chassis_to_scanner_lid REVOLUTE + chassis_to_input_tray PRISMATIC) and a faithful ADF hump / touchscreen / cassette. Consolidation into a single `chassis` part with inlined visuals is code-shape-COMPLIANT (rule 3: non-articulated decoration = visuals, not FIXED-joint parts). Weakness: output tray is a fixed shelf (no fold-out) and buttons are a fixed touchscreen visual (no separate parts). Minor hand-unrolled: `screen_icon_0/1/2`, `lower_side_0/1`, `hp_stroke_0/1`. Usable as the sole A2/B2/C2 anchor.
- Brother origin: cleanest — buttons via `for i, bx in enumerate(btn_xs)` → `button_{i}`, `panel_icon_0/1` loop, `tray_rail_{l,r}` loop. Chosen parent for the button-count multiplicity fork.

## 同步备注
- 变体 workbench-only, 不 promote; 收敛后脚本批量 rating=5 同步进 arti-template。
- generated_assets.jsonl 待 fork 后登记, 与本 source map 通过 record_id 关联。
