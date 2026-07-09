# Template Authoring (AUTHORING.md)

The single mandatory-read for implementing a procedural template under
`agent/templates/<slug>.py` after its modular spec is approved. It has three
layers:

- **§A — Design judgment & hard rules.** The 5 rules every template must obey.
- **§B — Modular system: slots, contracts, interfaces, patterns, pitfalls.**
  How to decompose a category and wire modules that assemble and stay supported.
- **§C — Iteration loop, gates & verdicts.** How you run the sweep, read its
  signals, and decide when done.

New category templates are **modular by default**: they set
`__modular__ = True` and derive structure from per-module 5-star sources declared
in the spec (per-slot realization is read from the sweep's `axis_realization`
report). Read this before writing the
first line of a module factory. For the spec→template workflow (continuous) and the
doc map, see `README.md`. The authoritative diversity definition is
`VISUAL_DIVERSITY_MODEL.md`; source-adaptation depth notes are in
`MATURE_TEMPLATE_METHOD.md` — consult both on demand, not cover-to-cover.

---

# §A — Design judgment & hard rules

Five rules every template must follow. Violating any one blocks
`verdict=pass`: some are gate-enforced (a short "why" note here), the rest are
judgment checked at spec review and `template batch` visual inspection.

## Rule 1: 不动就不是 part ("if it doesn't move, it isn't a part")

**Decorative sub-elements that don't articulate (no revolute / prismatic /
continuous joint) MUST be attached as `parent.visual(...)`, NOT as a separate
part joined by a FIXED articulation.**

The LLM 5-star records fuse decorative geometry into the same `Part`; the
low-star records (and many old templates) instead spawn a separate part joined
by FIXED to a tiny "interface" disk. That disk passes `fail_if_isolated_parts`
/ `fail_if_parts_overlap` at 1µm tol but is visually invisible — the
"floating decoration" failure mode (dj_knob / gpu_bracket / knife_slider).

```python
# ⛔ Bad — purely decorative pad as a FIXED-joint part.
pad = model.part(f"pad_{i}")
model.articulation(f"pad_fixed_{i}", FIXED, parent=body, child=pad, ...)

# ✓ Good — fold the pad into the body as a named visual.
body.visual(Box((0.030, 0.030, 0.003)),
            origin=Origin(xyz=(x, y, panel_z + 0.0015)), name=f"pad_{i}")
```

If the element genuinely articulates (REVOLUTE/PRISMATIC/CONTINUOUS) it is a
real part — but the parent must still have a real anchoring visual (Rule 2),
not a 3mm disk. The only legitimate FIXED articulation is two parts that need
independent reference frames (composed kinematic sub-assemblies); document why
visual fusing won't work and verify the parent's anchoring visual.

## Rule 2: parent must really anchor the child ("no phantom anchors")

**Every articulation creating a separate child part MUST declare a
`MatingContract` pinning both mating faces to real, visually justifying visuals
(not a sub-mm "interface" disk).** A joint origin can be within 15mm of both
parts (`fail_if_articulation_origin_far_from_geometry` passes) while the mating
*surfaces* are still millimetres of air apart.

Enforced: `fail_if_joint_mating_has_gap` (compiler baseline) fails when named
parent/child faces are farther than `contact_tol` (default 1mm). Joints whose
geometry can't be two axis-aligned faces in contact (pin-through-sleeve,
captured trunnion, ball-in-socket) **omit** `mating=...` and are grandfathered
(the gap check is skipped) — use sparingly, prefer a real contract where the
geometry fits.

## Rule 3: derive structure from declared 5-star sources

**Each module factory's part tree, joint topology, per-part visual count, and
primitive types must be derivable from the module's declared 5-star source
records.** You may freely parameterize literal dimensions, enum branches, and
Multiplicity ranges; you may NOT invent structural elements the source module
family lacks, and you may NOT downgrade sophisticated primitives (LatheGeometry
/ mesh_from_geometry / cadquery output → `Mesh`) to crude Box/Cylinder
placeholders. Downgrading primitives is the thing this rule exists to prevent.

- ✓ `Cylinder(radius=config.hub_radius, length=config.hub_height)` in place of
  literal dims; `for i in range(config.blade_count)` in place of `(0,1,2)`.
- ✓ An `if r.blade_shape == "swept":` branch calling a *different*
  `mesh_from_geometry(...)` profile — still a mesh, not a `Box`.
- ⛔ Replacing `LatheGeometry([...])` with `Cylinder(...)`.

Without declared module sources, agents drift to "imagine what a category looks
like" — crude boxes where the sample has sculpted lathe profiles, or invented
extra parts/joint chains. Authoring workflow: read the spec's per-slot 5-star
sources; identify each helper, `part.visual(...)`, `model.articulation(...)`,
and intended `InterfaceSpec`/`MatingContract`; sketch the slot graph and
topology variants; adapt geometry keeping primitive types; add a `MatingContract`
to every child-creating joint; expose `slot_choices_for_seed(seed)`; sweep.

## Rule 4: 装饰共形嵌入 (decoration conforms to the host surface)

**Applied surface decoration (④ 表面装饰: labels, printed bands, stripes, ribs,
knurling, plaques) MUST be derived from the host part's actual surface — its
per-z radius profile or the specific face it sits on — so it hugs the body
across ③ Primary Form Family and ⑤ dimension changes.** A decoration built at a
CONSTANT radius/size laid over a tapered/curved/scaled face reads as detached.

Decoration is the LAST geometry generated — derive order is **③ primary form →
⑤ dims → ④ decoration** — because it must read the *final* surface. The
`Container_Tube` `label_band` is the canonical failure: a constant belly radius
sits proud of a waisted-conical body. Sample the host across the decoration's
z-span (a band wraps `radius(z)`; a plaque follows its slanted/curved face) and
build to match. Enforced by spec `§8.5` ④ row + visual check at `template batch`.

## Rule 5: 活动关节全程不穿模 + 动态语义可测

**A template with any non-FIXED joint MUST call
`fail_if_parts_overlap_in_sampled_poses(...)` in its `run_tests()` (unless it
declares a `sampled-pose exemption`), plus at least one targeted `ctx.pose(...)`
check per key mechanism proving intended motion.** The compiler baseline only
checks the CLOSED pose, so a part that clears when closed but collides mid-travel
slips through unless the template opts into dynamic tests.

```python
def run_tests(model, config):
    ctx = TestContext(model)
    # ... category asserts ...
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=96,   # drop to 32 for 6+ independent joints (60s/seed budget)
        ignore_fixed=True,
    )
    return ctx.report()
```

- Sampling is **discrete per joint** — `{0, lower, upper, mid}` for
  revolute/prismatic, `{0, ±90°, 180°}` for continuous — bounded by
  `max_pose_samples` (the multi-joint Cartesian-product cap).
- If defaults don't express the mechanism, add joint `meta["qc_samples"]` or
  `meta["qc_sample_values"]` for meaningful open/extended/folded/latched/service
  states.
- Eliminate every collision, or declare a real mid-travel overlap with
  `ctx.allow_overlap(a, b, reason=...)` — only for real hinge barrels, rails,
  sleeves, captured pins; never to mask an over-wide joint range.
- Each key mechanism gets one targeted `with ctx.pose({joint: value}): ...`
  asserting visible displacement / open direction / reachable endpoint / folded
  clearance / source-record semantics. Collision passing alone is not enough.
- Lower/upper collision failures usually mean shrink or conditionally clamp the
  sampled range; mid failures mean the origin/axis/envelope/clearance is wrong.
  A failing targeted pose while collision passes means the motion semantics are
  wrong — do not hide it by shrinking the range.
- Spec `§8.5` ⑤ row writes each non-continuous joint's motion envelope (axis /
  opening direction / `[closed, feasible-upper]`) and a `motion_test_plan`.

`sweep-pipeline` warns (`motion_test_audit`) when a template defines non-FIXED
joints but has no sampled check, no targeted `ctx.pose(...)`, and no exemption —
a must-fix for new authoring.

## New-template checklist (§A)

1. [ ] Spec declares `__modular__ = True` and per-module 5-star sources.
2. [ ] Every module factory was authored after reading its source; literals
   parameterized, primitive types + joint semantics preserved (Rule 3).
3. [ ] No FIXED articulation unless a docstring justifies why visual fusing
   won't work (Rule 1).
4. [ ] Every non-FIXED articulation declares a `MatingContract` to real visuals
   on both sides (Rule 2).
5. [ ] Applied surface decoration (④) is host-derived, hugs the body across ③/⑤ (Rule 4).
6. [ ] Every non-FIXED-joint template has Rule 5 coverage
   (`fail_if_parts_overlap_in_sampled_poses(...)` + `ctx.pose(...)`, or exemption).
7. [ ] `SPEC_TEMPLATE.md §8.5` 视觉多样性 6 轴考察 filled (each axis
   present/absent + reason; form-dominated category registers a ③ slot).
8. [ ] `sweep-pipeline` `verdict=pass` on `0-35` (see §C for the full verdict).
9. [ ] Previews for seeds 0, 1, 2 look like the category with no closed-pose gaps.

---

# §B — Modular system: slots, contracts, interfaces, patterns

A modular template defines **slots**, each populated per seed by one of several
**module** factories, producing topology-level diversity (different part trees,
joint counts, chain depths) rather than only parameter variation. This is the
only authoring route; do not use a single flat parts list or `primary_anchor`.

## Slot design — sizing slots and candidates

Deciding **how many slots** and **how many candidates per slot** is the single
most impactful design call.

**Slot count.** List the independent *structural* variation axes in the 5-star
samples (part trees / joint counts / types — not dims or color). An axis is a
slot only if it has **≥2 candidate modules from genuinely different samples**,
and adjacent slots must be **chainable** — share a mating face (serial chain
like arm links) or a common parent (parallel children like a deck + controls).
If two "axes" can't share a mating surface, they're alternative modules of one
slot, not separate slots.

| Slot count | Trade-off |
|---|---|
| 1 | No topology variation — collapses to a fixed structure (re-decompose) |
| 2 | Both slots vary (≥2 candidates); thin — prefer ≥3 candidates each |
| **3** | Sweet spot — three independent axes, each module tractable |
| 4 | Narrower modules, more mating seams, allow_overlap declarations multiply |
| ≥5 | Code explodes; a "slot" usually folds into **slot-level multiplicity** |

Most templates land at 3. Complex categories that look like 5+ slots usually
have axes that belong in slot-level multiplicity (office_chair armrests are a
multiplicity feature of the seat slot, not a slot).

**Candidate count: 3-6 per slot; degrade to 2 only if the 5-star pool can't
yield more structurally distinct sources.** A single reachable candidate is
exempt-but-flagged (a smell); don't lean on it. Each candidate must be
structurally distinct, **not a re-skin**:

- ✅ Two housings from different records with different part/joint counts.
- ✅ Same part tree but a different **recognizable ③ Primary Form Family**
  prototype (Planar Boundary Form / Volumetric Envelope Form / Macro Surface
  Construction). Swapping the ③ prototype is a structural distinction **on its
  own** — no part-count change required. Form-dominated categories live here.
- ❌ Same part tree, different palette colors → a palette parameter.
- ❌ Differing only in linear dimensions → a parameter sample.

**More candidates vs multiplicity vs extra slot.** When a new structural
variation appears: (1) different way to do the same job at the same attachment
point → **new candidate**; (2) same part repeated N times with regular spacing
→ **slot-level multiplicity** (one module per N, named `tri_blade`,
`quad_blade`, …; N becomes the topology variation); (3) a physically separate
functional layer absent from the slot graph → **new slot**, only if ≥2
candidates exist. Rule of thumb: **prefer more candidates over more slots** —
each slot adds inter-module mating geometry; each candidate is self-contained.

## Reference implementations

Read the slug closest to your target structure before writing.

| Template | Slot graph | Highlights |
|---|---|---|
| `agent/templates/retractable_utility_knife.py` | housing → mechanism → blade | Linear chain, 2 candidates/slot, 8 combos |
| `agent/templates/monitor_mount.py` | base → arm → head | Variable arm chain, 32 combos |
| `agent/templates/dj_equipment.py` | chassis → deck_layout → controls | Parallel children (deck/controls parent to housing) |

## Architecture

The abstraction lives in `agent/templates/_modular.py`. Core dataclasses:

- **`InterfaceSpec`** — a face a module exposes to mate with another module's
  opposite face: `part_name` / `visual_name` / `face_side` (which visual + axis
  face the MatingContract references), `anchor_local` (face center in local
  frame), `face_extents_uv` (in-plane size children derive footprint from via
  `fit_to_upstream`), `iface_key` (optional identity; both sides must match), and
  `consumer_joint_type/axis/motion_limits` (on `upstream` interfaces — what joint
  the assembler emits when chaining to this module).
- **`Module` / `ModuleBuild`** — a factory emits parts + internal articulations
  and returns a `ModuleBuild` listing what it emitted and its interfaces
  (typically `"upstream"` and `"downstream"`).
- **`SlotSpec`** — one slot; owns `candidates: dict[name, factory]`.
- **`assemble(...)`** — the driver: per slot in order, runs the resolved
  factory and emits a chain joint between the previous module's downstream
  interface and this module's upstream interface.

## Design contracts

**Contract 1 — Module chain wiring.** Each factory
`def _build_<name>(ctx: ModuleBuildContext) -> ModuleBuild` pulls resolved
config from `ctx.config`, palette from `ctx.palette`, per-module randomness
from `ctx.rng` (never a fresh `random.Random`). Inspect `ctx.prior_choices`
when geometry depends on an upstream choice; read `ctx.upstream_interface.part_name`
to parent joints directly to an upstream part (parallel pattern).

**Contract 2 — Upstream interface anchor.** A child `upstream` interface's
`anchor_local` must have a **0 normal-axis component** (tangential x/y free) so
its mating face coincides with the parent's. Enforced: `_emit_chain_joint`
raises.

**Contract 2b — Combination legality is declared (`iface_key`).** Whether two
modules may mate (a 28mm neck takes only 28mm closures) is category semantics no
geometry gate sees — an illegal pair assembles and passes every compile check.
Give both sides the same `iface_key`, keyed across a whole slot's candidates
(one-sided keys check nothing); `_validate_pair` raises on mismatch.

**Contract 2c — Child footprint derives from the upstream face
(`fit_to_upstream`).** A child sized to the face it mounts on should derive its
footprint from `ctx.upstream_interface.face_extents_uv` via `fit_to_upstream(ctx, ...)`
rather than re-state it from global config — otherwise it gaps/overhangs when
the parent's realized size changes. Fixed sizes stay legitimate for
standardized hardware (comment why).

**Contract 3 — Visible support within and across parts.** Every visible element
should have a support path to the grounded body (contact, slight embed, a
supported neighbor, or a passing `MatingContract`). The baseline enforces this
at **part** granularity: `fail_if_isolated_parts` (FAIL — every part touches the
connected tree within `contact_tol`) and
`warn_if_part_contains_disconnected_geometry_islands` (WARN, promoted to a hard
seed FAIL in `compile-sweep`). There is **no islands escape hatch**
(`allow_disconnected_islands` was removed): seat separated rigid pieces (comb,
grille, fin stack) on a shared carrier or split them into FIXED parts with real
contact.

**Contract 3b — FIXED placement is single-sourced (`mount_fixed`).** For
stacked/nested children, stating placement twice — child mesh centered but the
FIXED joint origin hand-written at a corner post — silently cantilevers the
child off its parent while every mechanical gate passes (connected, no overlap,
origin proximity ≈ 0). `mount_fixed` derives both the joint origin (on parent
geometry) and the child rebase from one mount point. Set
`tangential_containment=True` on the `MatingContract` (or `InterfaceSpec` for
chained slots) so the baseline also verifies the child face stays within the
parent face's footprint — the exact check the cantilever bug slips past.

**Contract 3c — Shared geometric quantities are single-sourced.** Any quantity
more than one element depends on (a face height something sits on, a radius
something reaches, a motion envelope something clears) must live in exactly one
place: a named module-level helper (`_tower_window_face_radius(r)`) or a
`Resolved<Slug>Config` field. Two elements independently hand-writing "the same"
quantity is the top drift-bug source (a roof at `0.48*cap_h` while the ridge
starts at `0.50*cap_h` → permanent gap). When repairing, factor the scattered
copies into one helper *first*, then fix the value.

**Contract 3d — Mechanisms come from the idiom library first.** Before writing
a raw `model.articulation(...)` for a moving mechanism, check
`agent/templates/_mechanisms.py` and the SDK clearance solver:

- `hinged_panel(...)` — every lid/door/hatch/flap/leaf hinge; declares the world
  direction the panel opens toward (a wrong axis sign or pivot edge RAISES — the
  reversed-swing 穿模 family), opening range clamped by the clearance solver.
- `sliding_member(...)` — drawers/trays/telescoping; realized travel solved so
  the slider stays clear.
- `coupled_chain(...)` — concertina/folding chains: followers mimic one driver
  (independent-joint chains ALWAYS self-intersect), optional fold budget,
  solver-clamped driver limits.
- `from sdk import clamp_joint_limits, max_joint_value` — any other
  clearance-dependent joint: solve it, don't hand-derive per-template trig.

Idioms create NO geometry — appearance stays yours. Raw joints are fine where no
idiom fits; leave a comment on which you considered and why.

**Contract 3e — Geometric quantities are traceable.** Relations between parts
(seating height, reach, clearance, mating offset — anything that must co-vary
when another part changes) are DERIVED or solved, never frozen constants.
Attributes of one part (its own size/proportion/detail) self-parameterize
freely; the bound where an attribute meets another part is a relation. Real
absolute dimensions are constants with a one-line basis. Forbidden: constants
tuned until the sweep passes with no stateable source.

**Contract 4 — Procedural sampling is the seed domain.** `config_from_seed(seed)`
must use deterministic procedural sampling for every ordinary seed **including
seed 0** (no small curated/modulo table as the main domain). Enforced:
`config_from_seed(0)` must succeed (`test_template_registry_contract.py`). Sparse
regression overrides are allowed only for known regressions or reviewer-selected
cases, documented in the spec/comments. `slot_choices_for_seed(seed) ->
list[tuple[str, str]]` exports per-seed module picks, including multiplicity
axes (narrow ranges: raw N; wide ranges: the spec §8 band, e.g. ≤8/9-50/>50);
consumed by the report-only `axis_realization` visibility and failure attribution.

**Contract 5 — `__modular__ = True`.** Set at module scope so the sweep treats
the template as modular and emits slot/module realization data for diagnostics.
This metadata is report-only and does not gate the verdict.

## Required deliverables

For a new modular template `<slug>`:

1. **`agent/templates/<slug>.py`** exporting: `<Slug>Config` (frozen public),
   `Resolved<Slug>Config` (clamped internal), `config_from_seed(seed)`,
   `resolve_config(config)`, `build_<slug>(config, *, assets=None)`,
   `build_seeded_<slug>(seed)`, `slot_choices_for_seed(seed)`,
   `run_<slug>_tests(model, config)`, and `__modular__ = True`.
2. **`tests/agent/test_<slug>_template.py`** — OPTIONAL. The authoritative
   acceptance signal is `compile-sweep`, not pytest; per-template tests are
   auto-tagged `template_asset` and excluded from the default run. Skip while
   batch-authoring; write one only to lock a finished template against
   regressions (all-combinations build loop, or specific regression seeds).
3. **`specs_modular_v1/<slug>.md`** — the spec (per-module source tables,
   compatibility rules, procedural sampling / sweep plan; no `primary_anchor`).

## Common pitfalls (learned the hard way)

**Disconnected geometry islands** (most common). Final quality is the visible
support graph: every visible element, even inside one part, needs a
contact/embed/mating path to the grounded body. Symptoms: a lug floats near a
pivot; a bridge mesh sits above a carrier with a gap; a bracket sits above a deck
without overlap. Fix: rest it on a supported surface, give it a real
stem/riser/bracket/collar, or split it into a separate part that physically
mates — no fake hidden bridges, no escape hatch (`allow_disconnected_islands`
removed).

**Joint anchor honesty.** `fail_if_articulation_origin_far_from_geometry(tol=0.015)`
verifies every joint frame sits on real hardware (flat 15mm, per type,
`bbox_relative` accepted-and-ignored): rotational passes if the axis touches
that side's hardware OR is its symmetry centerline (ring bearings, domes, hubs);
prismatic/floating are exempt (origin is gauge freedom); FIXED needs the origin
at the welded interface on both sides (prefer `mount_fixed`). If a hinge origin
lands >15mm from the nearest visual, move it onto the contact geometry and shift
the child's authoring to compensate (fence_cascade: coupler hinge origin on the
bottom eye, linked panel z-shifted by `-coupler_bot_z`). For assembler chain
joints the **child module's visuals must contain (0,0,0) in part frame** — emit
the upstream mating face so its AABB includes the origin.

**Mating gap along normal axis.** `fail_if_joint_mating_has_gap` measures
distance **only along the parent face's outward normal**, not 3D Euclidean — so
sliding rails (long parent face, short child face) and asymmetric mounts work if
the normal-axis position matches (child upstream anchor free in tangential x/y).

**Captured-pin / high-risk allowances.** Mechanical pivots (pin-through-sleeve,
hub straddling clevis lugs, spindle-in-cup) have intentional overlap. Keep
allowances **local and element-scoped** — avoid broad part-level
`allow_overlap(part_a, part_b)` / `allow_isolated_part(...)`:

```python
ctx.allow_overlap(parent, child, elem_a="bearing_socket", elem_b="shaft",
                  reason="<short rationale>")
```

See `_allow_internal_pivot_overlaps` in monitor_mount. When a connector
neck/riser you added to fix islands now collides with an adjacent part, declare
element-scoped `allow_overlap` between the new element and the neighbor. A joint
without real contact or a passing `MatingContract` is not a support path. There
is no `--quality-profile` switch: acceptance is one fixed level (always-on
compiler baseline, `pass_rate >= 0.90`); broad/floating allowances are an
authoring smell, not a toggle.

## Pattern: parallel-children slot

Multiple slots' parts all parent to a single chassis (not a serial chain). Read
the upstream housing name, parent joints to it, and **do NOT define an
"upstream" interface** (that suppresses the assembler's automatic chain joint);
re-export the housing's downstream face so later slots also parent to it:

```python
def _build_deck_layout(ctx: ModuleBuildContext) -> ModuleBuild:
    housing = ctx.model.get_part(ctx.upstream_interface.part_name)
    left_platter = ctx.model.part("left_platter")
    # ... build platter visuals ...
    ctx.model.articulation("housing_to_left_platter", ArticulationType.REVOLUTE,
                           parent=housing, child=left_platter, origin=..., axis=(0,0,1), mating=...)
    return ModuleBuild(module_name="dual_jog_decks",
                       parts_emitted=["left_platter", "right_platter"],
                       internal_articulations=["housing_to_left_platter", "housing_to_right_platter"],
                       interfaces={"downstream": ctx.upstream_interface})
```

## Pattern: variable-multiplicity radial (fan blades, propellers, gear teeth)

A slot module emits **N identical sub-parts radially attached to a common hub**.
Declare **one module per N** and emit N FIXED children of the hub at even
angular spacing (`angle = i * 2*pi / n`), reading the hub name from
`ctx.upstream_interface.part_name`. This is the parallel-children pattern +
multiplicity in the module name (`f"{n}_blade_set"`), declaring `downstream`
only. Practical notes:

- All N blades share **identical geometry** (one shared helper) so visual count
  and bbox stay proportional across N variants.
- Declare `ctx.allow_overlap(hub, blade_i, ...)` per blade root ↔ hub cylinder
  (radial contact isn't modeled by MatingContract); grandfather the FIXED joint.
- Per-blade islands pass trivially, but **connect each blade's own visuals** (a
  tip cap not touching the root is a per-part island).
- Blade extends along +x (radial outward); the hub joint rpy rotates it around z.

## Pattern: variable-multiplicity chain (N-link arm)

A slot needs a parameterized count of repeated sub-parts (1-8 arm links).
Declare **one module per N** (`f"{prefix}_link_arm"`, e.g. `quint_link_arm`) so
the `axis_realization` report shows ≥2 chain lengths; N never counts toward
distinctness (VISUAL_DIVERSITY_MODEL.md §2) — encoding buys coverage, not
inflation. The `_build_n_link_arm(ctx, *, n)` helper emits primary_arm +
shoulder_hub + elbow_clevis, N-2 mid links via a shared `_emit_mid_arm_link`, and
secondary_arm, chained by `elbow_fold_i` REVOLUTE joints. Aspirational — no
template implements it yet (monitor_mount is a fixed two-arm chain).

## Where new strict gates come from

The compiler baseline (`agent/compiler.py` `_run_compiler_owned_baseline_tests`)
runs on every compile. Current policy: raw part-internal islands are warn-level
diagnostics, but unsupported visible islands, broad/floating allowances, and
sampled articulation overlaps are acceptance failures. New strict checks follow
the same pattern: audit existing templates, fix real regressions, then turn on. A
new SDK/assembler capability lands only with ≥1 production template adopting it in
the same change; zero-adopter capabilities get removed.

---

# §C — Iteration loop, gates & verdicts

The single source of truth for whether a template is done:

```bash
uv run articraft template sweep-pipeline <slug>
```

You may **not** declare a template done on the strength of `pytest`,
`scripts/check_template_qc.py`, or eyeballing previews alone. Plain
`compile-sweep` omits the corner/extreme-seed stage the pipeline treats as mandatory.

## The loop

1. Edit `agent/templates/<slug>.py`.
2. **Smoke-first:** after every edit, run a 1–5 seed probe (a 1-seed
   `compile-sweep <slug> --seeds 0`, or `run_seed_outcomes` on `0-4`) BEFORE a
   full pipeline. Never debug via full pipelines — a broad sweep confirms a fix,
   it doesn't find one; it wastes minutes per iteration.
3. Once the probe is clean, run `sweep-pipeline <slug>`.
4. Parse the JSON and act on the structured signals (below).
5. Repeat until `verdict=pass`.

Honor the spec's self-declared compile budget (SPEC_TEMPLATE `编译预算`; set the sweep's `--compile-timeout` hang-guard to ~3x the declared value for heavy categories — it is a watchdog, never a quality bar) from
the first version — cheap tessellation is a design choice, not a late optimization.

The pipeline runs two incremental full sweeps — **fast** (`0-15`) then **final**
(`16-35`, 36 cumulative) — stops at the first failing stage with a repair
summary, then appends a **corner** stage. `fast` runs fully parallel (wall-clock
≈ one seed's compile); the smoke probe above replaces the old seed-0 compute
guard. State persists under
`<repo_root>/.articraft/template_sweep_state/<slug>.json`; each run increments
`streak_count` for surviving clusters and updates `pass_rate_trend` — this is what
detects "I am bouncing on the same root cause." Manual fallback (CLI unavailable;
the pipeline compiles only newly added seeds per run):

```bash
uv run articraft template compile-sweep <slug> --seeds 0-15
uv run articraft template compile-sweep <slug> --seeds 0-35
```

## What each seed runs (compiler baseline)

Each seed goes through the full compile pipeline records use:

- `check_model_valid` / `check_single_root_part` / `check_mesh_assets_ready`.
- `fail_if_isolated_parts` (geometric connectivity).
- `fail_if_parts_overlap_in_current_pose` (closed-pose overlap).
- `fail_if_articulation_origin_far_from_geometry(tol=0.015)` (flat 15mm, per
  type — see §B joint-anchor honesty).
- `fail_if_joint_mating_has_gap` (every joint with a declared `MatingContract`;
  `tangential_containment=True` additionally checks footprint containment — both
  assume axis-aligned parent visuals, rpy-rotated faces not covered).
- `harness_motion_qc` (**motion gate, on by default**) — for any template with
  movable (non-mimic revolute/prismatic/continuous) joints, parts are checked
  for overlap at **sampled joint poses**, not just closed (still static geometry,
  evaluated at each joint's `{0, lower, upper, mid}` plus small combos).
  Element-scoped `ctx.allow_overlap(...)` in `run_tests` is honored at every
  pose; disable only for debugging via `--no-motion-qc`. **Sequenced mechanisms**
  (drawer opens only once the door is open): declare a scoped `allow_overlap` on
  the sequenced pair (reason states the sequence) and keep the FULL designed
  travel. Never strangle travel to pass an unphysical combo.

Every failing seed reports its FULL defect list (primary failure + an
`ADDITIONAL FAILURES` section — nothing masked). The report carries
`failure_triage` (failures grouped by check + subject across seeds, repair
priority, worst depth / triggering pose / `built_by`) and `allowance_audit`
(every declared allowance with seed reach; weak-reason and `new_since_last_pass`
`allow_overlap`s flagged). Do NOT silence a genuine 穿模 with `allow_overlap` —
the audit diffs it against the last-good baseline.

**Corner stage.** After `0-35` passes, the pipeline appends deterministically
*selected* real seeds hitting per-numeric-field extremes (each config field at
its reachable min/max) and slot combos `0-35` never realized — so a corner
failure is a defect a batch user can hit. More than `DEFAULT_MAX_CORNER_FAILURES`
(currently 1) corner failures fails the run even if cumulative pass-rate stays
above threshold; read `failed_corner_seeds`. Plan in `report.corner_seed_plan`.

**Axis realization (report-only) — read this for per-slot type visibility.**
`report.axis_realization` shows what the swept seeds actually realized per axis
(numeric: min/max/mean + histogram; slot keys: `slot_value_counts` per value).
It is where you confirm each declared slot/module actually appears.

## Verdict

`verdict=pass` iff (computed in `agent/template_sweep.py`):

- `pass_rate >= pass_threshold` (default 0.90 — at 36 seeds ≤3 failing seeds);
  plus the corner tolerance (≤`DEFAULT_MAX_CORNER_FAILURES` corner failures).
  Every passing seed clears the compiler baseline.

## Decision rules

**Read `failure_clusters` before individual `failed_seeds`.** Clusters are
pre-aggregated by normalized failure_type + shared config axes; the biggest
cluster is the likeliest root cause. Fix order: (1) largest cluster first;
(2) if same failure_type with different `shared_config_axes`, treat the more
distinctive axis pattern as the structural bug; (3) use `diagnosis_hint` as the
first hypothesis, grounded by `example_failure_details`. **Do not** make
per-seed surgical edits when a cluster spans more than one seed.

**Escalation.** When `escalation.required = true`, **stop patching code**:

| Reason | Required next action |
|---|---|
| cluster signature unchanged for ≥3 sweeps | The same structural failure survived 3 attempts. Either **narrow `config_from_seed`** to exclude the failing axis, OR **split the slug** into templates with disjoint motion spines. Record which in handoff notes. |
| `pass_rate` not improved over the last 3 sweeps | Local edits aren't converging. Re-read the spec; it may need tightening or splitting. Do NOT keep trying random tweaks. |

After escalating, write a short handoff note (what was tried, recommended
narrow/split) and stop iterating the same slug.

## Stop conditions (done iff all hold on the latest sweep)

- `verdict == "pass"`; `pass_rate >= 0.90` on `0-35` with the corner stage clean
  (check `axis_realization`/`failed_corner_seeds` for slot + edge visibility).
- You visually inspected previews for seeds `0, 1, 2` (via
  `uv run articraft template batch <slug> --seeds 0-9 --agent claude-code` +
  `just viewer`, or `scripts/render_template_previews.py --slugs <slug>
  --seeds 0-2`) and confirmed they look like the category with no obvious
  identity/proportion/closed-pose problems. `template batch` at small N **is**
  the quality inspection — run it, don't gate it behind a separate approval.

The first is mechanical and in the JSON; the preview check needs your judgment
because no gate answers "does this look like a <category>."

## What you may NOT do

- Declare done because `pytest tests/agent/test_<slug>_template.py` passes (it
  covers a smaller baseline than the sweep).
- Lower `--pass-threshold` to force the verdict (policy, not a tuning knob — if
  0.90 looks unachievable, escalate).
- Disable streak tracking via `--state-dir ""` to escape escalation.
- Widen the seed range (`--seeds 0-99`) to dilute a real cluster (the signature
  is independent of seed count).
- Edit `_BASELINE_ARTICULATION_ORIGIN_TOL` in `agent/compiler.py` (a
  cross-template floor).

## Timing & concurrency

Target <2 min per sweep. **Always thread-cap** so 1 worker = 1 core — this, NOT
a low worker count, prevents false `subprocess_crash`/`compile_timeout` clusters
under load (geometry compile is single-threaded OCC/FCL, not BLAS math, so extra
threads buy nothing and blow past the thread limit):

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 NUMEXPR_NUM_THREADS=1 \
uv run articraft template sweep-pipeline <slug> --max-workers 16 --compile-timeout 120
```

`--max-workers 16` = ⌊nproc/12⌋ on a 192-core box; a solo sweep can go 32-50;
recompute as `min(≈50, ⌊nproc/#concurrent⌋)` only on a different machine or >12
concurrent sweeps. If a single seed compile takes >20s the template's `_build_*`
has accidental O(n²) or over-fine tessellation — fix it before iterating (see the
spec compile budget). A `subprocess_crash`/`compile_timeout` cluster (vs a
geometry fail with a concrete dist value): re-run the SAME command before trusting.

## Related tools

| Tool | Use |
|---|---|
| `articraft template sweep-pipeline <slug>` | Per-iteration ground-truth (incl. `motion_test_audit`). Read every iteration. |
| `articraft template compile-sweep <slug> --seeds X` | Manual fallback / targeted diagnosis. |
| `articraft template batch <slug> --seeds X` | Promote into real records; run only after stop conditions met. |
| `articraft external check <record>` | Record-level baseline + author tests (same baseline as sweep); use after batch to validate a specific record. |
