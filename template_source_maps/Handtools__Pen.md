# Handtools / Pen — template source map

pattern: parallel_children
parents:
- rec_build-a-realistic-articulated-3d-model-of-a-pen-_20260609_163942_733176_1c47d9da ← picture/Handtools/Pen/001.png (silver metal click-action retractable ballpoint: round lathed barrel, windowed pocket clip, knurled push-button plunger). **Baseline for all 7 variants** (every `rec_ht_pen_var_*` forks from this parent — confirmed via each variant's `record.json` `parent_record_id`). Fills: actuation=click, profile=round, clip=windowed_clip, grip=plain.
- rec_build-a-realistic-articulated-3d-model-of-a-pen-_20260609_200037_041760_bd7a9b72 ← picture/Stationary/Pen/001.png (STABILO BOSS highlighter: chunky rounded-RECTANGULAR barrel, black chisel nib, removable pull-off cap). Standalone parent (no variants descend from it). Fills the unique cells: actuation=removable_pull_cap AND profile=rounded_rect.

Writing instrument. Core kinematics shared by all candidates: a `barrel` (root) carrying the nib/tip, and exactly ONE actuated user mechanism (the pen's "how it opens / advances the point"). Every source has exactly one non-fixed joint; the pocket clip, when present, is a FIXED structural trim part (or an inline parent visual). The actuation mechanism, the barrel cross-section, the clip, and the grip section are the four independent structural slots below.

## Slot 候选覆盖

### Slot A:actuation mechanism (the single non-fixed joint — pen "point control / opening")
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| click_plunger | rec_..._pen_..._1c47d9da (parent A) | `plunger` part, `barrel_to_plunger` (PRISMATIC axis -Z) | top push-button knob + shaft slides axially into the barrel bore; ~7 mm stroke; button protrudes above collar at rest | converged |
| twist_sleeve | rec_ht_pen_var_twist | `sleeve` part, `barrel_to_sleeve` (REVOLUTE axis +Z) | upper barrel section ROTATES about the long axis at a seam band; quarter-turn (0→π/2); clip rides on the sleeve and rotates with it; dome top (no button) | converged |
| side_slider | rec_ht_pen_var_slider | `slider` part, `barrel_to_slider` (PRISMATIC axis -Z) | thumb slider pad+tab rides in a longitudinal slot cut into the barrel +Y face; slides DOWN to advance nib; ~20 mm; dome top | converged |
| hinged_cap | rec_ht_pen_var_capped | `cap` part, `barrel_to_cap` (REVOLUTE axis -Y) | friction-fit cap on a snap hinge at the nose-cone top swings up/clear of the writing tip (0→~2.8 rad); barrel has closed crown + hinge lug | converged |
| removable_pull_cap | rec_..._pen_..._bd7a9b72 (parent B) | `cap` part, `barrel_to_cap` (PRISMATIC axis +X) | hollow cap pulls straight off the front over the nib (push/pull friction fit), travels forward to clear the tip; rectangular highlighter | converged |

### Slot B:barrel cross-section / profile (`barrel_body`)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| round_lathe | rec_..._pen_..._1c47d9da (parent A) | `barrel_body` via `LatheGeometry` (tip→collar profile, hollow bore) | classic round revolved barrel + conical nose + ball tip | converged |
| hex_prism | rec_ht_pen_var_hex | `barrel_body` via `_hex_prism` CadQuery (6 flat faces, across-flats), round conical nose unioned in | hexagonal barrel cross-section; XY non-circular (dy>dx); 6 face grip strips | converged |
| rounded_rect | rec_..._pen_..._bd7a9b72 (parent B) | `barrel_body` = `_rounded_rect_prism` (filleted rectangular extrude) + stepped collar | chunky rectangular highlighter barrel, rounded corners | converged |

### Slot C:pocket clip (`clip` part / `barrel_to_clip` FIXED, or absent)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| windowed_clip | rec_..._pen_..._1c47d9da (parent A) | `clip` part `clip_blade`, `barrel_to_clip` FIXED; blade+bridge+foot, oval `window` cut | spring-steel blade with oval cut-out window, ball foot, bridge anchor on +X collar | converged |
| solid_clip | rec_ht_pen_var_solidclip | `clip` part `clip_blade` (no window cut); tapered blade + swept `curl` + spherical `ball` foot | solid (no window) tapered clip with pronounced curled-over ball-foot; uses CadQuery `sweep`+`sphere` | converged |
| no_clip | rec_ht_pen_var_noclip | NO `clip` part; NO `barrel_to_clip` joint (exactly 2 parts, 1 joint) | clean clipless collar; decorative grip rings inlined as barrel visuals | converged |

(Note: parent B's highlighter clip is an INLINE cap visual `clip`+`boss`, not a separate part — it folds into the `windowed_clip`/solid family conceptually but is authored as decoration; the three rows above are the clean separable candidates.)

### Slot D:grip section / surface detail (lower-barrel hand zone)
| 候选(未来 module) | record_id | 关键 part/joint 名 | 结构特征 | 状态 |
|---|---|---|---|---|
| plain | rec_..._pen_..._1c47d9da (parent A) | (none — smooth `barrel_body` mid-section) | smooth metal barrel, no grip detail | converged |
| contoured_rubber_grip | rec_ht_pen_var_grip | `barrel_body` profile waist/bulge/neck/flare + `grip_ring_{i}` torus loop (`TorusGeometry`) | dedicated waisted/bulged rubber grip zone with 6 raised texture rings (for-loop emitted) | converged |
| ring_bands | rec_ht_pen_var_noclip / rec_ht_pen_var_capped | `grip_ring_{i}` revolved/lathed bands via for-loop (`for i in range(N)`, `grip_ring_{i}` naming) | a set of N raised annular grip bands on the lower barrel (no profile change) | converged |
| face_strips | rec_ht_pen_var_hex | `face_strip_{i}` (6 strips, one per hex face, for-loop + shared mesh + rpy rotation) | longitudinal rubber grip strips, one per flat face (hex-coupled) | converged |

## Multiplicity / Copy Logic
- count_param: **no hard multiplicity axis for the pen's core structure** — the pen is固定 named slots (one barrel, one actuator, ≤1 clip). The only repeated sub-parts are decorative grip details (rings / face strips), which ARE for-loop emitted and could become a templated `grip_count`.
- copied object / naming / placement / joint policy (grip detail, secondary axis):
  - **copied object**: a single grip ring band / face strip (shared geometry helper).
  - **naming**: `grip_ring_{i}` (twist N=3, capped N=5, grip N=6, noclip N=4) / `face_strip_{i}` (hex N=6, coupled to face count).
  - **placement**: equal Z spacing along the barrel (rings) or equal angular spacing around the section (hex face strips, `rpy=(0,0,angle)`).
  - **joint policy**: all FIXED to / inlined as `barrel.visual(...)` parent visuals — NO independent joints (pure decoration, per §4 Rule 3).
- N 样本 (grip detail): rings {3, 4, 5, 6} → twist / noclip / capped / grip ; face strips {6} → hex.
- 模板建议 N_range: grip_ring_count ∈ [0, ~10] (0 = plain candidate); hex face_strips coupled 1:1 to barrel facet count.

## 组合数预审
Slot A (5: click_plunger / twist_sleeve / side_slider / hinged_cap / removable_pull_cap)
× Slot C (3: windowed_clip / solid_clip / no_clip) = **15 ≥ 10 ✓**.
Including Slot B (×3: round / hex / rounded_rect) and Slot D (×4: plain / contoured_rubber / ring_bands / face_strips) the topology space is far larger. Every slot ≥2 candidates. pattern = parallel_children (single actuated child + structural trim children on a common barrel root); no hard multiplicity axis (grip detail is the soft secondary copy-logic axis).

Both parents are placed: Parent A = {click_plunger, round_lathe, windowed_clip, plain}; Parent B = {removable_pull_cap, rounded_rect, (inline clip), plain}.

## 排除项(未来 compatibility matrix 素材)
- **Cross-slot interface coupling (template compatibility-matrix material):**
  - `side_slider` REQUIRES a barrel-side longitudinal slot (`SLOT_*` cut on +Y face) — the slot is part of the barrel geometry, so side_slider × {hex_prism, rounded_rect} needs the slot re-cut into that profile (slot↔profile mating face dependency).
  - `twist_sleeve` REQUIRES the barrel to be cut at a mid-axis SEAM (`SEAM_Z`) so the rotating sleeve is a distinct upper section; incompatible with `removable_pull_cap` profile authoring without a seam.
  - `hinged_cap` / `removable_pull_cap` REQUIRE a closed (capped, non-open-bore) barrel top + nib exposed; `click_plunger` / `side_slider` / `twist_sleeve` use an open bore or domed top. Top-closure is coupled to Slot A.
  - `face_strips` (Slot D) is coupled to `hex_prism` (Slot B) — strip count = facet count; not meaningful on a round/rect barrel.
- **Not slots (template continuous params / controlled local parameterization, excluded):** barrel length / diameter / taper, stroke/travel length, hinge sweep angle, color/material (silver vs navy vs lime) — all continuous, NOT candidates.
- No convergence failures: all 7 variants + both parents compile-pass with ≥1 non-fixed joint and stay in category ("pen"). 0 blocked cells.

## 备注
- All variants are workbench-only (parent A is workbench; forks inherit). Not promoted; `category_slug` not set.
- `picture_expansion/generated_assets.jsonl` currently has no `ht_pen` rows — register this batch's 7 variant records there (keyed by `record_id`) to keep the small-类 view consistent with this source map.
- A passing modular `pen.py` template already exists downstream (arti-template); this map documents the variant pool for the record and to cross-check the template's slots (Slot A actuation ×5 / Slot B profile ×3 / Slot C clip ×3 / Slot D grip ×4).
