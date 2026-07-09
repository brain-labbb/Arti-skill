---
name: build-template
description: >-
  Phase-scoped batch orchestrator for the image→seed→variant→spec→template pipeline
  across the two Articraft repos. Invoke when the user wants to drive one PHASE
  across many picture subcategories: ingest reference images + author original seed
  assets for new 小类 in articraft_data, batch-fork variants from those seeds, then
  author a modular spec and build the sweep-passing template together in arti-template.
  Use whenever the user says "加新小类/类别", "入库图片", "造原始资产/seed", "批量造变体",
  "扩变体", "走变体→模板流程", "造模版", "ingest pictures", "add subcategories",
  "build/fork variants for these 小类", "write the specs", "author the templates", or
  names subcategories + this pipeline. Runs in three segments (INGEST+SEED / UPSTREAM /
  SPEC+TEMPLATE) with TWO combined human gates: inspect seeds after ingest,
  inspect variants after upstream (spec authoring flows straight into the template in
  one subagent). Batches across subcategories; the seed and spec+template segments fan
  out one subagent per subcategory. Resumable: re-derives each item's phase from
  on-disk artifacts every invocation.
---

# build-template — phase-scoped, multi-subcategory pipeline orchestrator

## How the user actually works (the shape this skill must fit)

The user sweeps **one phase across many 小类**, not one 小类 end-to-end:
first stand up new 小类 (ingest images + author the original seed asset), then
batch-fork variants for a whole set of subcategories (upstream), and only later go
downstream to author the spec + template. So this skill runs in three **segments**,
each bounded by a human gate, and each able to cover many subcategories at once:

```
SEGMENT 0: INGEST+SEED       SEGMENT 1: UPSTREAM         SEGMENT 2: SPEC+TEMPLATE
(articraft_data)             (articraft_data)            (→ arti-template)
INGEST imgs + SEED asset ═▶  P0 proto-spec + P1 fork ═▶  P2 sync 5★ (MAIN) → P3+P4 subagent
  new 小类: images→folders,     forks from the seeds        1 subagent/小类: spec → template
  author original, doctor       MAIN loop                   → iterate to verdict=pass (capped)
                       ▲STOP                       ▲STOP
                inspect seeds               inspect variants
```

Two hard gates, never crossed unprompted: **after INGEST+SEED** you inspect the
original assets; **after UPSTREAM** you inspect the variants (seed identity and
variant quality decide everything downstream). After the variant gate, SPEC and
TEMPLATE run **in one subagent per 小类** — no spec-approval stop; the subagent
reads the 5★ sources once, writes the spec, then implements the template and
iterates to `verdict=pass`. `SPEC_REVIEW_TEMPLATE.md` remains an optional QA tool.
If implementation reveals a declared candidate is unfit (broken geometry, can't join the
slot graph), the subagent revises its OWN spec — candidate moves to the exclusion list with a
one-line reason; a slot dropping below 2 candidates records a degrade reason or escalates.
Never silently drop a declared candidate; never force-adapt a broken one.

SEGMENT 0 only applies to **new** 小类 (no original asset yet). A 小类 that already
has a picture-bound parent starts at SEGMENT 1 — the phase detector (Step 0) routes
each item, so a mixed batch is fine.

## Repos & paths (fixed)

- **Upstream** = `/mnt/zsn/lyb/arti-skill/articraft_data` · **Downstream** = `/mnt/zsn/lyb/arti-skill/arti-template`
- Source map: `articraft_data/picture_expansion/template_source_maps/<大类>__<小类>.md`
- Spec: `arti-template/articraft_template_authoring/specs_modular_v1/<slug>.md`
- Template: `arti-template/agent/templates/<slug>.py` · Sweep state: `arti-template/.articraft/template_sweep_state/<slug>.json`

Run repo commands as `cd <repo> && uv run articraft ...` (each repo has its own venv).

## Why this skill exists

The rules already live in the repo docs. The failure mode is **skipping phase
boundaries**: forking before a proto-spec, not delivering the source map, writing a
template before its spec exists, declaring a template done on pytest instead of
`sweep-pipeline verdict=pass`. This skill does NOT restate the rules — it enforces
the **segment order, the per-item gates, the two combined stops, and the
batch/subagent dispatch**. Read the referenced doc at each phase; follow it exactly.

---

## Step 0 — resolve scope, build the work list, detect each item's phase (always)

1. **Scope.** Determine which segment the user wants:
   - `ingest` (images→folders + author original seeds) · `upstream` (fork variants) ·
     `spec+template` (sync 5★ → author spec + template together).
   - If the user named a segment / verb ("加新小类/入库图片/造原始资产"→ingest,
     "批量造变体"→upstream, "写spec/造模板/走spec→模板"→spec+template), use it. If ambiguous,
     ask which segment.
2. **Work list.** Collect the target subcategories (大类/小类) or slugs from the user.
   "all pending" = every item whose artifacts show it is ready for this segment but not
   done (use the detection table). Confirm the resolved list before doing work.
3. **Slugs are not mechanical** (`Bag_Suitcase/Shopping bucket` → `shopping_bucket`,
   `Chair/Folding chair` → `folding_chair`). Propose + confirm any slug not given.
4. **Detect each item's phase from disk** (resumable, un-skippable). Per item, first
   incomplete row = where it stands:

   | Check (in order) | State |
   |---|---|
   | `picture/<大类>/<小类>/` folder absent or holds 0 images | needs **INGEST** |
   | images present but 小类 is seedless (in `picture_doctor` `seedless` list — no bound original asset) | needs **SEED** |
   | seed authored + compiles, user hasn't confirmed seed quality | at **seed-review gate** |
   | source map complete (all slots converged + 完成定义 §14) | needs **UPSTREAM** |
   | source map done, user hasn't confirmed variant quality | at **variant-review gate** |
   | variants confirmed, not yet synced into `arti-template/data/records/` with `rating=5` | needs **SPEC+TEMPLATE** (P2 sync first) |
   | synced 5★, but spec + template not both done to `verdict=pass` | needs **SPEC+TEMPLATE** (P3+P4 subagent) |
   | template `.py` exists AND latest sweep `verdict=pass` (read `sweep_history[-1].verdict` from `arti-template/.articraft/template_sweep_state/<slug>.json`) | item complete |

5. Seed a `TodoWrite` list: one entry per (item × phase) in the active segment, so batch
   progress is visible. Run only the requested segment; never roll into the next segment
   past a gate.

---

## SEGMENT 0 — INGEST + SEED: stand up new 小类 (repo: articraft_data)

Only for **new** 小类 with no original asset yet. Produces the picture-bound original
that SEGMENT 1 forks from. **Read first:** `articraft_data/EXTERNAL_AGENT_DATA.md`
(authoring contract — `external` workflow, never hand-create records).

**Binding model (the integrity that keeps the workbench grid correct):** each 小类's
归类 lives in a per-record sidecar `data/records/<id>/picture.json` — the **single source of
truth** (the legacy fuzzy `external_assets_map.json` is retired). The viewer's 小类 browser is
a live walk of `picture/` (no index build — a new folder shows on viewer restart). `external
seed` writes the sidecar as `source=explicit` and **refuses to create an imageless seed**, so
归类 is correct by construction; forks later inherit it. `picture_doctor.py` is the gate that
proves all of this held.

### INGEST — images → picture folders (MAIN loop, mechanical)
- Source is a CodeArt-style markdown (`## 大类` → `### 小类` → `![Image](url)`). Run from
  the MAIN loop (it downloads; do not fan out to subagents for I/O):
  ```
  cd /mnt/zsn/lyb/arti-skill/articraft_data
  uv run python scripts/ingest_pictures.py --md CodeArt-end.md --categories "<大类>,..." --dry-run  # preview plan
  uv run python scripts/ingest_pictures.py --md CodeArt-end.md --categories "<大类>,..."            # download
  ```
  It unescapes markdown names, sanitizes the path-illegal `/`→`_` (e.g. `Pill bottle/box`
  →`Pill bottle_box`, `Electrical / Wiring`→`Electrical_Wiring`), writes `001.png…` in
  source order, validates each as a real PNG, and skips files that already exist (re-runnable).
- If one picture shows several distinct designs (e.g. 4 crutches in a row), split it into
  one-object-per-file before seeding (each becomes a clean reference; extra designs are fork
  axes downstream, not the seed).
- Then `uv run python scripts/picture_doctor.py` — the new 小类 must appear under their 大类
  and show as `seedless` (images, 0 assets). That seedless state is expected here; SEED clears it.
- Restart the `:8765` viewer (it caches the folder walk at startup; needs user OK).
- **GATE INGEST:** every requested 小类 has a `picture/<大类>/<小类>/` folder with ≥1 valid PNG;
  `picture_doctor` reports 0 `bad_*` and lists each new 小类 as `seedless`.

### SEED — author the original asset (one subagent per 小类, parallel)
Seeding CAN batch — dispatch one **subagent per 小类 in a single message**. Each subagent:
- Create the record with the binding forced (pick the canonical reference image):
  ```
  uv run articraft external seed "<大类>/<小类>" "<authoring prompt>" --agent claude-code --image 001.png
  ```
- Author `data/records/<id>/revisions/rev_000001/model.py` from the reference image(s) per
  `EXTERNAL_AGENT_DATA.md` + the SDK docs (`sdk/_docs/`). Build the object the picture shows;
  keep parts readable (repeated sub-parts via `for i in range(n)`, the §11 contract SEGMENT 1
  later relies on). Do NOT author a modular template here — this is a single concrete asset.
- Compile + validate: `uv run articraft external check <record>`.
- **Stay workbench-only** — never promote to the curated dataset (`collections` == `['workbench']`).
- Return: 小类, record id, the bound `picture_category/subcategory`, compile verdict.
- **GATE SEED (per 小类):** `external check` compile success; `picture.json` sidecar present with
  `source=explicit` and the correct 大类/小类; the asset reads as the object in the picture.

## ═══ HARD STOP: seed inspection (combined, once for the whole batch) ═══

When all seed subagents return and pass GATE SEED, run `picture_doctor.py` once more (assert
**0 hard problems** and that none of the batch's 小类 remain in `seedless` — every new 小类 now
has a bound original). Then **make the seeds visible in the workbench — DO NOT SKIP**: `external
seed`/`generate` write the record + `picture.json` sidecar but do **NOT** update the two indexes
the workbench grid reads, so the new assets stay invisible *even after a viewer restart* until you
rebuild them. Run BOTH, then restart the viewer (it caches at startup):
```
cd /mnt/zsn/lyb/arti-skill/articraft_data
uv run articraft data reconcile            # rebuilds per-小类 subcat shards + search index
uv run articraft data build-record-index   # rebuilds records_index.jsonl (the viewer browse list)
# then restart the :8765 viewer (needs user OK) so it reloads the rebuilt indexes
```
Verify before declaring done: the new record ids appear in `records_index.jsonl` and in
`data/index/subcat/<大类>__<小类>.jsonl`. (This is the most common SEGMENT 0 failure: "I seeded
but the workbench is empty" = indexes not rebuilt and/or viewer not restarted.) Then **STOP**.
Machine gates prove compile + binding, not identity — whether the seed actually reads as the
object is what every downstream fork inherits. Post one combined summary: per 小类, record id + the bound 小类 + a one-line identity
note. Then tell the user:

> Original seeds for [<小类 list>] are authored, bound, and compiling. Inspect them in the
> viewer (workbench → each 小类). I will NOT fork variants until you confirm which seeds are good.

Proceed only when the user confirms. Rejected seeds loop back into SEED, then stop here again.
Confirmed seeds become SEGMENT 1's parents — bound by their `picture.json` sidecar and listed in
the subcat shard (rebuilt by the reconcile above) as originals (`parent_record_id` empty). Pass
each seed's record id (from SEED's output) directly to `fork`, or look it up in the shard.

---

## SEGMENT 1 — UPSTREAM: batch-fork variants (repo: articraft_data)

Per subcategory, do P0 then P1. **Read first:** `articraft_data/FORK_VARIANTS.md`
(rules) + `articraft_data/VARIANT_PIPELINE.md` (operational checklist).

### P0 — proto-spec + axis plan (per 小类)
- Find parent(s) via the per-小类 shard `data/index/subcat/<大类>__<小类>.jsonl` (rows with no
  `parent_record_id` = originals; VARIANT_PIPELINE Step 1) — or use a seed's record id straight
  from SEGMENT 0. No parent → skip this 小类 and report it (this skill only forks from an existing
  parent). (The legacy `external_assets_map.json` is retired.)
- Read each parent's `revisions/rev_000001/model.py`; confirm the §11 readability contract
  (repeated sub-parts emitted by `for i in range(n)`, not hand-written).
- Plan 2–4 structural axes (= future slots) + a multiplicity axis if there are N identical
  sub-parts. Place parents on the grid first (free cells); variants fill empty cells.
  Multi-origin 小类: allocate forks by each origin's real merit (sturdy/typical origins host
  more, ≥1 fork per on-grid origin, nearest-anchor parent per cell, sibling origins may fill
  cells directly; total count is an outcome, not a target — see FORK_VARIANTS §8). For EACH
  planned fork, actively evaluate attaching ④/⑤/⑥ COMPANION_VARIATIONS (declared in the
  field, esp. ⑥ colorways when origins are color-monotone — free palette_style vocabulary);
  skip where unnatural.
  ROW DISCOVERY first: enumerate each structural slot's values from BOTH the origins AND
  world knowledge keyed by the 类别/小类 name (what other mainstream forms does this object
  have in reality?); mint the missing real forms as fork anchors.
  ANTI-PATTERN: candidates read from different origins are NOT freely composable across
  families; unsourced cross-family cells the template will sample must be forked into a real
  anchor, or recorded as `compatibility_probe`/`gated`/`blocked` in the source map — never
  silently extrapolated (FORK_VARIANTS §13).
- **GATE P0:** source map draft records every planned slot with ≥2 target candidates
  (目标 3–6), multiplicity axes with representative N samples if present, any planned
  ③ primary-form anchors (Planar Boundary / Volumetric Envelope / Macro Surface Construction), ④/⑤/⑥ record/extrapolation ranges, and optional `compatibility_probe`
  rows for high-risk combinations. Do not fork to satisfy an old product-of-combinations
  target; fork only missing source-backed anchors and explicit probes.

### P1 — fork the primary-axis variants (per 小类)
- Batch size = primary candidate cells (`全部目标格子 − parent 已占格子`) + optional
  `compatibility_probe` rows, NOT a fixed 20.
- One prompt per variant = axis description (TARGET/KEEP plus optional COMPANION_VARIATIONS
  or COMPATIBILITY_PROBE fields) + the fixed suffix.
  Author ONLY the axis description; the suffix is the single file `picture_expansion/FORK_SUFFIX.txt`,
  appended by the run script (`cat axis_file FORK_SUFFIX.txt`) — never transcribe it (`FORK_VARIANTS.md` §10).
  Ordinary variants have exactly one primary structural axis; ④/⑤/⑥ companion variations
  are allowed only if they do not alter part tree / joint graph / interface / primitive.
  ①/②/main ③ primary form family/multiplicity N combinations are forbidden unless this row is explicitly a
  `compatibility_probe`.
- **Run forks as ONE background Bash job from the MAIN loop** (VARIANT_PIPELINE Step 4).
  When batching several 小类, put every variant's `run_one` line into one script and launch
  it once with `run_in_background: true`. ⛔ Do NOT spawn a sub-agent per fork — a subagent
  that backgrounds a fork ends its turn and the orphaned process is killed, producing no
  record. (This is the one place subagents are forbidden; downstream segments DO use them.)
  ```
  uv run articraft fork --provider dashscope --model qwen3.7-max --thinking-level medium \
    --skip-search-index --record-id rec_<slug>_var_<axis> --label <...> <PARENT> "<prompt>"
  ```
- After the batch: `cd articraft_data && uv run articraft data reconcile` (the proven Step-5
  command — rebuilds the per-小类 shards the workbench grid needs + the search index). Note
  the three index axes so you don't mis-optimize: (1) the per-小类 **subcat shards** are
  rebuilt on EVERY reconcile and inherently iterate all records to bucket them — you cannot
  "skip the scan" here; (2) `--skip-search-index` skips ONLY the SQLite global-search index
  (used by search / `external examples`-style discovery), it does NOT avoid the record scan,
  and leaving it stale can hide variants from search — so don't add it by default; (3) the
  viewer's browse listing reads `records_index.jsonl`, which a fork does NOT update — if
  forked variants don't appear, add `--with-records-index`. The only genuinely scan-free way
  to inspect ONE variant is `uv run articraft view data/records/<id>` (per-record, no index).
  Then restart the `:8765` viewer (scans index once at startup) so the workbench grid shows them.
- Verify EVERY variant individually (VARIANT_PIPELINE Step 6) — don't trust "all passed"
  self-reports; compile-sweep gives false 60s-timeout failures under parallel load, so
  re-verify with clean foreground checks.
- Write the source map per `FORK_VARIANTS.md` §13 (slot tables + Multiplicity/Copy Logic +
  排除项). Use part/joint/helper names, NOT line numbers (resolved downstream via AST).
- **GATE P1 (per variant):** `compile: success` · ≥1 non-fixed joint (0 = re-fork) ·
  **`'dataset' not in collections`** (i.e. collections stays `['workbench']` — this, NOT
  `category_slug`, is the real promotion gate; a parent built with a `category_slug` label
  while staying workbench-only passes the slug through to its forks, which is fine — only a
  `dataset` collection membership = promotion to avoid) · diff scoped to the one axis (a
  footprint/primary-form or multiplicity axis legitimately rewrites geometry helpers, so a large
  diff is OK as long as part tree / joint topology changes are scoped to the primary axis) · **`run_tests`
  asserts the changed axis** — at least one assertion references the changed part/joint name
  (grep the variant's `model.py`); a variant whose only checks are generic/compile-level did
  NOT verify its own axis → re-fork (closes the loop on `FORK_VARIANTS.md` §12's test rule).
  **GATE P1 (per 小类):** every slot ≥2 converged candidates; multiplicity axis covers ≥2–3 N; 完成定义 §14 met.
  **Origin 全量对账:** EVERY origin in the shard (rows with no `parent_record_id`) must appear
  in the source map — either placed on the grid (anchor/candidate) or listed in 排除项 with a
  one-line reason. An unaccounted sibling origin never gets synced, so the spec never sees it
  and that picture's design vocabulary silently drops out of the template.
  **③ 主体形态家族 / Primary Form Family slot 的 fork = source-backed anchors**,数量按类别复杂度决定(常见 2–5,复杂类更多),不必 fork 每个主体形态候选——模板侧可 `world_knowledge_extrapolation` 扇出其余 candidate(同 part tree / primitive / interface,只改变 Planar Boundary / Volumetric Envelope / Macro Surface Construction 中的离散形态参数,Rule 3 内,由 sweep 必过 + reviewer 兜)。
  **④ 表面装饰**记录真实例子,模板侧可 `world_knowledge_extrapolation` 扩展 host-conformal、非结构、非关节、非新功能模块的 ribs / panel seams / rivets / labels / bands。
  **①/② 默认 source-backed**:世界知识可辅助命名和归纳,不得直接新增未被原始资产或 fork anchor 支撑的 skeleton/joint candidate。
- **GATE P1 (category↔picture binding — the integrity that keeps downstream from scrambling):**
  after reconcile, assert EVERY variant appears in `data/index/subcat/<大类>__<小类>.jsonl`
  with `picture_category` == 大类, `picture_subcategory` == 小类, `picture_path` under
  `picture/<大类>/<小类>/`, AND a `parent_record_id`/`origin_record_id` set. This binding is
  NOT in `record.json` (those `picture_*` fields read None even on parents) — it is re-derived
  from parent lineage into the shard at each reconcile. Two prerequisites make it hold: (a)
  each variant carries `parent_record_id` (the fork sets it), and (b) the parent itself has a
  `data/records/<parent>/picture.json` sidecar (the single source of truth — SEGMENT 0 writes it;
  `picture_doctor` proves it).
  If a variant is missing from the shard or its picture_category/subcategory is wrong, the
  parent's sidecar or the fork lineage broke — fix it HERE, never let a mis-bound variant
  flow downstream. Check:
  `python3 -c "import json;[print(r['record_id'],r.get('picture_category'),r.get('picture_subcategory'),bool(r.get('parent_record_id'))) for r in (json.loads(l) for l in open('data/index/subcat/<大类>__<小类>.jsonl'))]"`
  This shard IS the per-小类 export manifest: it groups picture(s) + original assets +
  variants under one 小类, and `parent_record_id`==self ⇒ original asset, else ⇒ variant. A
  clean per-小类 export = read the shard → collect the record ids + the `picture_path`(s) →
  bundle each record's `data/records/<id>/` + `data/cache/record_materialization/<id>/` + the
  picture file(s). Before declaring the 小类 done, assert bundle completeness (every record
  has both its record dir AND materialization cache, and every referenced picture file
  exists) so a later export never finds a dangling member.

## ═══ HARD STOP: variant inspection (combined, once for the whole batch) ═══

When all batched 小类 reach GATE P1 and their source maps are written, restart the viewer
and **STOP**. Machine gates only prove compile/articulation/in-category mechanically — they
cannot judge "does this still read as the same object" or "is the slot vocabulary right,"
and that judgment gates everything downstream. Post one combined summary: per 小类, the
converged candidates per slot (record id + one-line feature), N covered, 排除项, and the
viewer filter. Then tell the user:

> Variants for [<小类 list>] are done; source maps written. Inspect them in the viewer
> (workbench → each 小类). I will NOT sync or write specs until you confirm which 小类 are good.

Proceed only when the user confirms. For any 小类 they reject / cells to refill, loop those
back into P1, then stop here again.

---

## SEGMENT 2 — SPEC+TEMPLATE: sync, then spec→template in one subagent (repo: arti-template)

Run only for 小类 the user confirmed. **P2 sync first (main loop)**, then **P3 (spec) and
P4 (template) run together in ONE subagent per 小类** — the subagent reads the 5★ sources
once, writes the spec, then immediately implements the template and iterates to
`verdict=pass`. No spec-approval stop between them.

### P2 — sync confirmed variants in as 5★ sources (main loop)
Sync the parents + every converged variant. Either derive the ids from the source map
(`--source-map`) or pass them explicitly (`--records rec_id1,...`) — both work:
```
cd /mnt/zsn/lyb/arti-skill/arti-template && uv run python scripts/sync_from_source.py \
  --source-repo /mnt/zsn/lyb/arti-skill/articraft_data \
  --source-map /mnt/zsn/lyb/arti-skill/articraft_data/picture_expansion/template_source_maps/<大类>__<小类>.md \
  --rating 5 --execute
```
Run it without `--execute` first (dry run) to confirm the resolved id set looks right.
`--execute` copies records + materialization, stamps `rating=5`, and rebuilds the search
index. Records stay **workbench-only** on both sides — never the curated dataset.
**GATE P2:** scope the query by category — a bare `external examples --rating-min 5` only
prints the first `--limit` (8) of thousands of 5★ records and looks empty for your 小类:
```
uv run articraft external examples --rating-min 5 --category-slug <slug-or-inherited-category> --limit 50
```
must surface all synced sources (`total_matches` == your synced count). Also cross-check the
per-小类 shard: every origin (no `parent_record_id`) is either in the synced set or listed in
the source map's 排除项 — a missing origin means the map predates the origin-accounting rule;
account for it (sync or exclude+reason) before spec work.

### P3+P4 — spec→template in one subagent per 小类 (parallel, capped)
Dispatch one **subagent per 小类 in a single message** (concurrent, capped — see the
concurrency model below). Each subagent authors the spec, then **without stopping**
implements the template and iterates it to `verdict=pass`. Its brief (spec phase first):
- Pre-fill (optional): `scaffold_spec.py --source-map <map> --records-root data/records
  --slug <slug> --out ...specs_modular_v1/<slug>.md` lays down the slot/candidate skeleton and
  resolves `model.py:Lx-Ly` spans from helper names where it can; unresolved cells come back as
  explicit `TODO:` markers. Treat it as a skeleton — you still read the 5★ sources yourself and
  resolve every `TODO:` (real `Lx-Ly` line ranges) by hand.
- Then **read ALL 5★ samples** for the category (if <5 exist, stop and report — do not
  guess) and fully author the spec per `README.md` spec 阶段 + `SPEC_TEMPLATE.md`: 5★
  reading summary, identity/neighbor boundary, slot+candidate tables, slot graph, per-module
  emits, parameter ranges (INCLUDE a `palette_style` colorway parameter — list ≥3, target 4–6,
  realistic material/color sets observed across the 5★ sources, to be sampled per seed so the
  template's output is color-diverse), Multiplicity/Copy Logic, **§8.5 视觉多样性 6 轴考察**
  (declare each of the 6 axes present/absent+reason; form-dominated 小类 MUST register a ③
  Primary Form Family slot — see `SPEC_TEMPLATE.md §8.5` + `VISUAL_DIVERSITY_MODEL.md`),
  **topology-diversity audit** (total combos, sampling/sweep plan, compatibility
  matrix), validator, reject cases. Resolve every `TODO:` the scaffold left.
- **GATE P3 (before writing template code):** spec complete per `SPEC_TEMPLATE.md`; every
  candidate has a real `model.py:Lx-Ly`; no single-candidate slot without a documented degrade
  reason; topology audit present; §7.5 编译预算 filled. Only then move to the template phase —
  do NOT stop for approval.
- Then, **in the same subagent**, implement the template (P4 brief below) and iterate to
  `verdict=pass`.

**Concurrency model — two multiplying levels (don't conflate them):**
- `--max-workers N` = parallel seed compiles *inside one sweep* (a process pool, one slug).
  NOT agents.
- subagent count = parallel *template authors*, one per slug; each runs its own sweeps.
- There is a THIRD multiplying level that's easy to miss: each worker process's math libs
  (OpenBLAS/OMP/MKL) spawn **~one thread per core (~64 here)** for intra-compile linear algebra.
  Real thread demand ≈ **(#subagents) × (max-workers) × (~cores)** — counting PROCESSES
  (`#subagents × max-workers ≤ nproc`) is necessary but **NOT sufficient**. Geometry compile
  (CadQuery/OCC booleans, mesh, FCL) is single-threaded geometry, NOT BLAS math, so those
  threads buy ~nothing and only blow past the thread limit → `pthread_create` EAGAIN → false
  `subprocess_crash`/`compile_timeout` clusters. (This bit the 20-container batch at 6 subagents
  × 12 workers = 72 ≤ 192 processes, yet thread demand was 72×~64 ≈ 4600.)

**The real fix — ALWAYS thread-cap the sweep; this, NOT a low worker count, is the lever.**
Pin each worker to 1 BLAS thread (then 1 worker = 1 core and the process-count rule becomes
accurate). Canonical command — use it verbatim:

```
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 NUMEXPR_NUM_THREADS=1 \
  uv run articraft template sweep-pipeline <slug> --max-workers 16 --compile-timeout 120
```

**Fixed defaults: threads = 1, `--max-workers 16`.** 16 = ⌊nproc/12⌋ on this 192-core box — sized to
a 12-concurrent-sweep ceiling, so it's safe at ANY realistic concurrency (`#concurrent × 16 ≤ 192`
holds for ≤12 sweeps at once) with no assumption about how many agents run. Threads stay 1 because
geometry compile (CadQuery/OCC/FCL) doesn't use BLAS math — capping costs ~nothing (verify: per-seed
compile time doesn't grow) and spare cores go to MORE WORKERS, never BLAS threads. Two exceptions, not
the everyday rule: a SOLO sweep may raise to 32–50 (compiles all final-stage seeds at once); a DIFFERENT
machine or >12 concurrent sweeps → recompute the constant once as `min(≈50, ⌊nproc / #concurrent⌋)`. If
you see a `subprocess_crash`/`compile_timeout` cluster (vs a geometry fail with a concrete dist value),
re-run the SAME thread-capped command before treating it as real. Subagents write disjoint files
(`agent/templates/<slug>.py`), so no write conflicts.

P4 — template phase of the same subagent (模板阶段, right after its own spec):
- **Read first:** `AUTHORING.md` (§A design rules + §B modular contracts + §C iteration loop) —
  the single mandatory-read. Consult `VISUAL_DIVERSITY_MODEL.md` when designing diversity axes and
  `MATURE_TEMPLATE_METHOD.md` when adapting 5★ sources; deep-read 1–3 close reference templates from
  its map chosen by slot graph / motion topology, not category name.
- **Smoke-first (AUTHORING §C):** after every edit, run a 1–5 seed probe before a full
  pipeline; honor the spec's §7.5 编译预算 (≤20s/seed) from the first version.
- Write `agent/templates/<slug>.py` exporting `config_from_seed`, `resolve_config`,
  `build_<slug>`, `build_seeded_<slug>`, `slot_choices_for_seed`, `run_<slug>_tests`,
  `__modular__ = True`, adapting the declared 5★ sources.
- Obey the 4 hard rules: ①decorations as `parent.visual`, not FIXED joints; ②every non-FIXED
  joint declares a `MatingContract` to real visuals; ③derive structure from declared 5★
  sources — never downgrade Lathe/mesh to Box/Cylinder; ④geometric quantities traceable
  (AUTHORING Contract 3e): relations derive/solve, attributes self-parameterize, no
  sweep-tuned unsourced constants.
- **REQUIRED: per-seed palette diversity** (the swept output, not just the variant pool, must
  be colorful). Add a `palette_style` config field with ≥3 (target 4–6) realistic colorways
  drawn from the 5★ sources, have `config_from_seed` sample it per seed (`rng.choice(PALETTE_STYLES)`),
  resolve it to a `mats[...]` dict, and drive EVERY `.visual(..., material=mats[...])` off it.
  Axis realization and geometry reports do NOT judge color/⑥, so a monochrome pool can pass
  machine checks; this per-seed palette requirement is what prevents one. (See `cushion.py`:
  `PALETTE_STYLES` + `palette_style=rng.choice(...)`.)
- Iterate EVERY edit with the thread-capped canonical command: `cd /mnt/zsn/lyb/arti-skill/arti-template &&
  OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 NUMEXPR_NUM_THREADS=1 uv run articraft template
  sweep-pipeline <slug> --max-workers 16 --compile-timeout 120`. Fix the largest failure cluster first
  (not per-seed patches). If a cluster's
  `streak_count` hits 3 or `pass_rate` stalls 3 sweeps → escalate: stop patching, write a
  handoff note recommending `config_from_seed` narrowing or a slug split. Never lower
  `--pass-threshold`, widen seeds to dilute, or edit baseline tols.
- **GATE P4 (= done):** `verdict == "pass"` AND `pass_rate >= 0.90` on the final (0-35) stage
  with the corner stage clean. READ `report.axis_realization` (per-slot `slot_value_counts` — every
  declared slot/module actually appears) and `report.failed_corner_seeds` (edge-case defects) for
  the diversity + edge visibility. Then a human-judgment check the gates can't do —
  `uv run articraft template batch <slug> --seeds <probe-selected ~10> --agent claude-code` +
  `just viewer`, walk all seeds. Pick the seeds via `slot_choices_for_seed` (pure function,
  free) so every ③ family AND every world-extrapolated candidate appears at least once —
  extrapolated values are exactly what the eyeball check exists for, and a blind 0-9 can
  miss them. Report which candidate each seed exhibits. `template batch` at small N **is**
  the quality inspection — run it, don't gate it behind a separate approval. Not done if any
  seed looks broken (identity/proportion/closed-pose/mechanism) OR if the seeds are visibly
  monochrome (palette_style not actually varying — confirm distinct colorways appear).
- Return: slug, spec path, final `verdict`/`pass_rate`, axis_realization/corner notes, and any
  escalation handoff note.

After subagents return, report the per-slug spec paths + verdicts. Templates that escalated need
the user's decision (narrow vs split) — surface those, don't silently retry.

---

## Invariants (anti-drift — never violate, regardless of haste)

1. **Detect each item's phase from disk every invocation**; run only the requested segment.
2. **Two hard stops, never crossed unprompted:** inspect seeds after INGEST+SEED; inspect
   variants after UPSTREAM. All combined (one stop per batch, not per item). Within
   SPEC+TEMPLATE, spec → template runs straight through in one subagent (no spec-approval stop).
3. **The `picture.json` sidecar is the single source of truth for 归类** (the legacy fuzzy
   `external_assets_map.json` is retired). New 小类 are bound by an explicit sidecar written by
   `external seed` (which refuses an imageless seed); `picture_doctor` 0-hard-problems is the
   proof. INGEST/SEED runs only for 小类 with no original yet; a 小类 with a bound parent starts
   at SEGMENT 1.
4. **Forks run from the MAIN loop**, never from subagents (orphaned-process kill). **Seeds run
   in parallel subagents (one per 小类); spec+template run in ONE combined parallel subagent per
   小类** (P2 sync stays in the main loop).
5. **Always fork the original parent**, never a variant. One structural axis per variant. ≥1
   non-fixed joint. Color/material/scale are never the axis.
6. **Seeds and variants are workbench-only, never promoted**, on both repos.
7. **Source map is a required UPSTREAM deliverable** — no item advances to SPEC+TEMPLATE without it.
8. **A template is done only on `sweep-pipeline verdict=pass`** (pass_rate ≥0.90 on the final
   0-35 stage + corner stage clean; read `axis_realization`/`failed_corner_seeds` for slot + edge
   visibility) + viewer check — never
   on pytest, QC scripts, or eyeballing.
9. **ALWAYS thread-cap the sweep** (`OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
   NUMEXPR_NUM_THREADS=1`) + **`--max-workers 16`** (= ⌊nproc/12⌋ here; safe for ≤12 concurrent sweeps —
   recompute as `min(≈50, ⌊nproc/#concurrent⌋)` only on a different machine or >12 concurrent) +
   `--compile-timeout` ≥ 120s. The thread cap — NOT a low worker count — is what avoids the false
   `subprocess_crash`/`compile_timeout` clusters: counting processes (`#concurrent-sweeps × max-workers ≤
   nproc`) is necessary but not sufficient because each worker's BLAS spawns ~1 thread/core.
