# FORK_VARIANTS.md — Variant Planning Rules

Purpose: tell the agent **what to fork** for one `picture/<Category>/<Subcategory>` pool. Commands live in `VARIANT_PIPELINE.md`. The canonical fixed suffix already lives at `picture_expansion/FORK_SUFFIX.txt`; do not rewrite or duplicate it.

Use internal API fork only:
```bash
uv run articraft fork --provider dashscope --model qwen3.7-max --thinking-level high data/records/<parent_id> "<variant prompt>"
```
Do not use `articraft external init/fork/check/finalize`.

---

## 1. Goal

Build a source-backed sample pool for future modular templates. The pool should expose functional slots, candidate modules, multiplicity/copy logic, part names, joint names, primitive families, interfaces, and safe visual ranges for decoration/proportion/material/palette.

Do not enumerate cross-slot combinations. One clean source anchor per useful structural candidate is enough. Use `compatibility_probe` only for risky interface combinations.

---

## 2. Hard Rules

1. Fork from origin parent or `canonical_baseline`, never from a normal semantic variant.
2. One ordinary variant changes exactly one primary structural axis.
3. The result must remain the same `Category/Subcategory`.
4. Variants must compile and pass `run_tests()`.
5. Variants need at least one real non-fixed joint unless explicitly marked `static_only`.
6. Candidate-anchor axes ①/②/③ and multiplicity/N must be source-backed by `origin_anchor` or converged `forked_anchor`.
7. Axes ④/⑤/⑥ are mandatory audit axes, but not standalone candidate-anchor axes.
8. Do not pad. If structural vocabulary runs out, stop and record why.
9. Keep variants workbench-only; never promote into the curated dataset.
10. `model.py` must be readable enough for module extraction.

Quality/naming references: `agent/prompts/sections/designer_common.md`, `agent/prompts/sections/link_naming.md`.

---

## 3. Source Types

| label | meaning |
|---|---|
| `origin_anchor` | original parent directly shows the candidate |
| `forked_anchor` | converged variant forked from origin/canonical baseline |
| `canonical_baseline` | geometry-preserving cleanup; not a candidate |
| `compatibility_probe` | risky combination test; not a normal candidate |
| `record_only` | observed visual/proportion/material/palette range |
| `world_knowledge_extrapolation` | only safe host-conformal surface decoration |
| `blocked` | failed, out-of-category, unsupported, or insufficiently sourced |

---

## 4. Subcategory Contract

Before planning variants, write:
```yaml
subcategory_contract:
  category: <Category>
  subcategory: <Subcategory>
  core_identity: <what it must remain>
  must_keep: [<essential function>, <essential structure>, <essential articulation/static condition>]
  must_not_become: [<neighbor category>, <neighbor category>]
  image_evidence: [<features from reference pictures>]
  parent_evidence: [<features from parent model.py>]
```

---

## 5. Slots and Candidates

Pick real functional layers, not arbitrary looks. Typical pools have 2–4 slots; complex classes may have 5+.

Common slots: `body_form`, `surface_construction`, `opening_or_motion`, `handle_or_grip`, `support_or_base`, `internal_structure`, `multiplicity`.

Search candidates from origin assets, reference pictures, usage, structure, mechanism, material/manufacturing, and market form. Keep real product/use-case candidates. Prune padding, neighbor categories, unsupported values, and repeated failures.

---

## 6. Six-Axis Diversity Audit

Before choosing final variants, audit the subcategory along all 6 axes. This is mandatory planning input.

| axis | candidate-anchor status | allowed source / use |
|---|---|---|
| ① skeleton / structural topology | candidate-anchor axis | origin asset or world-knowledge-discovered value forked into `forked_anchor` |
| ② joint / mechanism type | candidate-anchor axis | origin asset or world-knowledge-discovered value forked into `forked_anchor` |
| ③ primary form family | candidate-anchor axis | origin asset or world-knowledge-discovered value forked into `forked_anchor` |
| ④ surface decoration | not standalone candidate anchor | `record_only`, optional companion variation, or controlled `world_knowledge_extrapolation` |
| ⑤ proportion / size / travel | not candidate-anchor axis | mandatory `record_only`; may ride along as low-risk companion variation |
| ⑥ material / palette / finish | not candidate-anchor axis | mandatory `record_only`; may ride along as low-risk companion variation |

Axes ①/②/③ may be discovered from both origin assets and world knowledge, but they become valid template candidates only after they are source-backed by an `origin_anchor` or a converged `forked_anchor`. World knowledge may decide what to fork; it may not directly create unsourced structural candidates.

Axis ③ includes real discrete body-form families: planar boundary, volumetric envelope, and macro surface construction. If a ③ value comes from world knowledge, fork it into a source-backed anchor before it enters the candidate table.

Axis ④ may use controlled `world_knowledge_extrapolation` for host-conformal, non-structural surface decoration. It does not need to appear as a dedicated variant.

Axes ⑤ and ⑥ are mandatory audit axes but not candidate-anchor axes. They are recorded for template sampling and may ride along as low-risk companion variations on a structural fork, but they must never be used as standalone variants or to satisfy the 8–30 candidate-anchor target.

---

## 7. Multiplicity / Copy Logic

Multiplicity/N is a structural candidate-anchor axis, not a cosmetic diversity axis.

Use this axis when the subcategory contains repeated homogeneous subparts, such as drawers, louvers, ribs, spokes, keys, chain links, blades, shelves, panels, holes, clips, buttons, or modular cells.

A multiplicity candidate is valid only when source-backed by `origin_anchor` or converged `forked_anchor`. World knowledge may suggest useful N values or copy patterns, but the template candidate must be supported by actual source code.

For a complete pool, cover 2–3 representative N samples when multiplicity exists. The goal is to expose copy logic, not enumerate the full count domain. The source map must record `count_param`, source-backed `N samples`, wider `N_range`, `copied_object`, indexed `naming`, `placement_rule`, and `joint_policy`.

A multiplicity fork changes only repeated-part count or copy layout. It must not also change main body family, joint type, category identity, or unrelated slots unless explicitly marked `compatibility_probe`.

Repeated homogeneous parts must be emitted with loop-based code, shared helpers, regular placement, and stable indexed names. Hand-written repeated blocks are not acceptable template sources.

---

## 8. Variant Budget

Use coverage first, density second, hard cap last.

| richness | normal candidate anchors |
|---|---:|
| simple | 8–12 |
| normal | 12–18 |
| rich | 18–24 |
| very rich | up to 30 |

Count `origin_anchor` and converged `forked_anchor`. Do not count `canonical_baseline`, `compatibility_probe`, or any ④/⑤/⑥-only record.

Allocation order: each real slot gets at least 2 source-backed candidates when possible; multiplicity gets 2–3 N samples; add high-value ①/②/③ structural candidates; record ④/⑤/⑥ ranges for template sampling; add a few probes only for risky combinations. If fewer than 8 honest candidate anchors exist, write `underfilled_reason` instead of padding.

---

## 9. Variant Card

Write one card before each fork:
```yaml
variant_card:
  variant_id: rec_<subcat_slug>_var_<axis>
  source_type: origin_anchor | forked_anchor | compatibility_probe
  parent_record_id: rec_<origin_or_canonical>
  positioning: {product_archetype: <real use-case>, why_same_subcategory: <reason>}
  primary_axis: {slot: <slot>, diversity_axis: <①|②|③|N|probe>, target_candidate: <value>}
  structural_delta:
    change: [<specific geometry/joint/interface change>]
    keep_parts: [<parent part/joint/helper names>]
    joint_policy: <preserve/add/replace exactly one primary-axis mechanism>
    interface_policy: <mating face/pivot/rail/socket/support rule>
  multiplicity: {applies: true|false, target_n: <N|null>, copied_object: <part|null>, placement_rule: <spacing/radial/chain/grid/none>}
  companion_variations: {allowed_④⑤⑥: [<surface/proportion/material only>], forbidden: [<category drift or bundled axes>]}
  acceptance_focus: [<specific test/render check>]
```
If the card is vague, do not fork.

---

## 10. Axis Prompt Format

Each axis file contains only object-specific fields. The pipeline appends the existing suffix and replaces `<Category>` / `<Subcategory>` placeholders.
```text
CATEGORY: <Category>
SUBCATEGORY: <Subcategory>
TARGET: change <slot/part/helper name from parent model.py> to <target candidate value>.
DIVERSITY_AXIS: <① skeleton/topology | ② joint/mechanism | ③ primary form family | N multiplicity | compatibility_probe>.
POSITIONING: <real product archetype and why it remains the same subcategory>.
KEEP: preserve these parent functional layers by name: <part_a>, <joint_b>, <helper_c>, ...
STRUCTURAL_DELTA: <specific geometry, joint, interface, and copied-object change>.
MULTIPLICITY: <only if this is the N axis; target N and placement/joint policy>.
COMPANION_VARIATIONS: <optional ④/⑤/⑥ only; no part tree / joint graph / interface / primitive-family change>.
FORBIDDEN: <neighbor categories and bundled-axis changes to avoid>.
COMPATIBILITY_PROBE: <only for probes; combined axes and clearance/interface risk>.
```
Do not paste `FORK_SUFFIX.txt` into the axis file.

---

## 11. Model.py Readability

Accepted variants must support module extraction: functional names map to slots; repeated subparts use loops/shared helpers/regular placement/stable indexed names; static decoration is host visuals; active children have visible supports; primitive choice is faithful; adjacent slots expose clear faces or anchors for future interface extraction.

---

## 12. Acceptance and Blocking

Reject or re-fork if compile/tests fail; no real non-fixed joint unless `static_only`; more than one primary axis changed; object drifts category; main change is ④/⑤/⑥-only; repeated objects are hand-written; active parts float/collide/use invisible anchors; source map cannot identify slot/candidate/part/joint/source type.

After 2–3 failed attempts for the same candidate, mark it `blocked` with the reason.

---

## 13. Source Map Output

Do not assume all slot candidates freely combine with all ③ primary form families. If a candidate depends on a specific body family, record the constraint as `compatibility_probe`, `gated`, or `blocked` in the source map.

For a complete pool, create `picture_expansion/template_source_maps/<Category>__<Subcategory>.md` with these sections:

```markdown
# <Category> / <Subcategory> — template source map
pattern: <linear_chain / parallel_children / multiplicity / mixed>
parents: <rec_id list and picture paths>
canonical_baselines: <optional>
underfilled_reason: <optional>

## Slot Candidates
| slot | candidate | diversity_axis | source_type | record/evidence | key parts/joints | status |
|---|---|---|---|---|---|---|

## Multiplicity / Copy Logic
- count_param:
- N samples:
- suggested N_range:
- copied object / naming / placement / joint policy:

## Six-Axis Diversity Record
| axis | treatment | values / range / reason |
|---|---|---|
| ① skeleton / structural topology | source-backed | |
| ② joint / mechanism type | source-backed | |
| ③ primary form family | source-backed | |
| ④ surface decoration | record_only / world_knowledge_extrapolation | |
| ⑤ proportion / size / travel | record_only | |
| ⑥ material / palette / finish | record_only | |

## Compatibility Probes
| probe | record_id | combined axes | risk tested | result |
|---|---|---|---|---|

## Blocked / Excluded
- <candidate or combination>: <reason>
```
Also update `picture_expansion/generated_assets.jsonl` when a persistent ledger is required.

---

## 14. Completion Definition

Complete when planned source-backed slots have at least 2 structurally distinct candidates when supported; the 6-axis audit is filled; ⑤/⑥ are recorded even if not forked; normal candidate anchors follow the 8–30 budget or explain underfill; multiplicity has 2–3 N samples when present; every accepted variant passes compile/tests and suffix constraints; every parent is accounted for; source map and generated-assets ledger are consistent.
