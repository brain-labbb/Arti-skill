# Pipeline Optimization TODO

Consolidated after the **fence_cascade** run (the first end-to-end
"articraft_data 小类资产扩变体 → arti-template 小类参数化模板" pass). fence
validated the **single-multiplicity-axis** case (one count param: `panel_count`).

> **ARCHIVED 2026-07-04** (reconciled): items ①②⑦⑧ shipped (⑦'s flat-tol fix was
> later superseded twice — relative tol restored 07-02, then replaced by the
> per-joint-type clean semantics 07-03/04, see find_joint_origin_distance_findings).
> Items ③④⑤⑥⑨⑩ remain VALID deferred backlog — ③/④ explicitly wait for the
> second real multi-axis template ("rule of three"); ⑥'s doc target is now
> AUTHORING.md (post-merge). This file moved from repo root to reports/ because
> root-level TODO files rot; the living backlog pointer is this archive.


Ranked by leverage (how many subcategories each helps).

---

## ✅ Shipped (fence pass)

- **① 上下游同步工具** — `scripts/sync_from_source.py`: copies a 小类's 5★ records +
  materialization cache from articraft_data into this repo, stamps `rating=5`, rebuilds the
  search index (records via `--records` or `--source-map`; dry-run default, `--execute` applies).
- **② source-map → spec 预填** — `scripts/scaffold_spec.py`: pre-fills the spec's §4
  slot/candidate tables + §8 multiplicity from the source map and **auto-resolves
  `model.py:Lx-Ly` from named helpers via AST** (unresolvable → `TODO:` markers); human only
  reviews, never transcribes line numbers.
- **⑦ 关节原点 tol 文档/实现对齐** — fixed in `MODULAR_TEMPLATE_AUTHORING.md`: doc claimed a
  relative tol (`0.05*bbox diagonal`) but the baseline is a flat absolute **0.015 m**
  (`_BASELINE_ARTICULATION_ORIGIN_TOL`); doc now states the flat value + the fix pattern
  (anchor the joint origin on real hardware). Check behavior unchanged (blast radius 100+ templates).
- **⑧ viewer 预检太脆** — fixed in `cli/main.py`: the `articraft viewer` `dataset validate-format`
  precondition is now **warn-and-continue** (+ a `--skip-validate` flag), so pre-existing
  malformed records no longer abort the viewer before serving.

---

## ⏳ Pending

### 框架层 — multi-axis multiplicity (defer until a real multi-axis template is authored)

#### ③ 多 multiplicity 轴一等公民  ← **defer to the 2nd (multi-axis) template**
multiplicity can have **0 / 1 / K** axes (helicopter = main-rotor blades N +
tail-rotor blades M; could be more). Target design: declare axes as **spec data**
(per axis: `count_param`, `range`, `weight` profile), **range set per-subcategory
per-axis**, **human-reviewed spec is authoritative**; a shared helper consumes the
declared axes (sampling + slot_choice encoding + clamp + per-axis sweep cap).
**Do NOT build the shared helper now** — only fence exists (single axis); extract
from real code when a multi-axis template (e.g. helicopter) is actually authored
(rule of three). Open questions only a real example answers: are axes independent
or coupled? does a count depend on an upstream slot choice? per-axis weight shape?
Lightweight guidance has been added to SPEC_TEMPLATE §8 / this doc; the abstraction
is deferred.

#### ④ 加权小-N 采样器 → 可复用 helper (per 轴)
fence's per-N weighted draw over `[2, N_MAX]` (small-biased, e.g. weight
`20 if n<=8 else 1 if n<=50 else 0.2`, ≈74% N≤8 / ~5% N>50) should be extracted
into a shared helper applied **per axis** (each axis its own range + weight). Part
of ③ — extract alongside it.

#### ⑤ sweep 的 multiplicity 护栏 (per-axis)
With multiple count axes, compile time scales with the **product** of counts
(N=80 × M=80). The sweep must cap each axis in test mode (or weight its tail very
low) so a full sweep can't choke. Framework-level, not per-template.

#### ⑥ mesh 复用模式文档化
"N identical sub-parts → tessellate the geometry once and reuse the Mesh across
all instances" makes large counts nearly free (fence: 2 geometries — root +
linked — reused across N panels). Document in MODULAR_TEMPLATE_AUTHORING's
multiplicity pattern; per repeated-part-type for multi-axis (main blade once,
tail blade once).

### 便利层 — 偏发需求(降级)

#### ⑨ `template batch --config-override <field>=<value>`
Routine generation is random (that's the point); overriding to a specific value
(`panel_count=52`, `main_blades=5,tail_blades=3`) is occasional (targeted sample /
debug / regression). Generic field override, NOT a per-param flag. Lower priority
— it caused friction this session only because we were producing a one-off demo
sample, not in normal flow. (We worked around it by searching for a seed whose
`config_from_seed` happened to yield the target N — see seed 1223 → N=52.)

### 单模板层 — 回上游补(低优先)

#### ⑩ fence `feet_style` 只有 2 候选
At the 2-candidate floor. **Under the per-key coverage gate this already PASSES** —
`feet_style` realizes 2 distinct values (≥2), and N/multiplicity is no longer counted
toward diversity, so a 3rd foot family is **not needed to pass the gate**. It is still
worth adding for **diversity quality** (richer real variety): if a 3rd real foot family
exists (casters / ground-stakes), add one upstream variant in articraft_data and re-sync.
Generally: source-map planning should flag slots stuck at the 2-candidate floor for a
targeted extra fork — for quality, not for gate-passing.
