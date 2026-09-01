---
name: build-template
description: >-
  Build, migrate, repair, and mechanically certify Articraft templates through the lightweight
  Agent workflow: SourceMap, compact TemplateDesign, ordinary function-style single-file
  implementation, cumulative mechanical checks through strict corner completion, and optional
  post-mechanical export of portable PBR appearance and material-physics sidecars.
---

# Build-template workflow

Use this skill when creating, migrating, repairing, or checking an Articraft template.
The current workflow is Agent-driven and API-ready, but does not implement API scheduling.

```text
SourceMap
→ TemplateDesign
→ material-free ordinary function-style single-file template
→ preflight
→ random-16
→ random-36
→ corner
→ strict_ready / strict_ready_with_tolerance

[only when portable asset export is requested]
→ export-assets
→ model.urdf + OBJ + PNG + appearance.json + physics.json
```

## Required reading

Before template work, read these current documents completely:

- `arti-template/articraft_template_authoring/README.md`
- `arti-template/articraft_template_authoring/AUTHORING.md`
- `arti-template/articraft_template_authoring/MECHANICAL_PRIORS.md`
- `arti-template/articraft_template_authoring/VISUAL_DIVERSITY_MODEL.md` when designing slots,
  N, cross-slot adaptation, or reporting core/raw diversity.

For upstream asset generation only, read the relevant `articraft_data` instructions. Rendering
and visual certification are outside this workflow unless the user explicitly requests them.

For post-mechanical portable asset export, also read
`arti-template/pbr_material_library/README.md` completely and inspect the live
`arti-template/pbr_material_library/unified/latest.json` pointer. The pointer and its immutable
snapshot, not resumable `.build` output, are the runtime material-library truth.

## Fixed locations

- Workspace: `/mnt/zsn/lyb/arti-skill`
- Source repo: `articraft_data`
- Template repo: `arti-template`
- Final template: `arti-template/agent/templates/<slug>.py`
- TemplateDesign: `arti-template/articraft_template_authoring/designs/<slug>.json`
- Check cache: `arti-template/.cache/template_check/<slug>/<mechanical-hash>/`
- Unified PBR pointer: `arti-template/pbr_material_library/unified/latest.json`
- Portable seed output: `<out>/<slug>/seed_<n>/` at the user-selected output root

Run commands in the owning repository with its `uv run` environment.

## Phase detector

Inspect disk state rather than relying on chat memory.

| Evidence | Next action |
|---|---|
| No valid source pool | finish upstream asset work |
| SourceMap missing or stale | write or repair SourceMap |
| TemplateDesign missing or incomplete | scaffold or complete TemplateDesign |
| Template missing or inconsistent with design | implement the single file |
| preflight/random-16 failing | repair the concrete finding |
| random-16 passing, random-36 incomplete | run random-36 |
| random-36 passing, corner incomplete | run corner |
| all three stages passing, no portable export requested | mechanical workflow complete; retain the corner report |
| all three stages passing, portable export requested | run the independent `export-assets` stage for the requested seeds |

## Runtime mode detector

- No TemplateDesign: keep the template on the legacy-compatible path; it does not need
  `TEMPLATE_DOMAIN` or a fleet-wide migration.
- TemplateDesign present with `TEMPLATE_DOMAIN`: use the strict Design-backed path.
- TemplateDesign present without a valid `TEMPLATE_DOMAIN`: treat the template as an incomplete
  migration and rebuild it through the new single-file path. The formal `designs/` directory is
  the opt-in migration marker; do not place speculative drafts there.

## Hard boundaries

1. The final template is the only runtime truth. SourceMap and TemplateDesign are authoring
   inputs and never participate in seed construction.
2. Do not create AST/source closure, SourceAssetGraph, snapshots, source runtime adapters,
   Producer tasks, component manifests, assembly manifests, or linker manifests.
3. Do not import `source_modules`, `source_module_runtime`, `source_bundle_runtime`, snapshots,
   another component Python file, or another template from a new/rebuilt final template.
4. The final file contains the category-specific component geometry and assembly logic as
   ordinary private functions. SDK imports are allowed for stable generic capabilities.
5. Preserve source-asset geometric quality. Do not replace characteristic profiles, meshes,
   joints, or mechanisms with simplistic placeholders merely to shorten the template.
6. `config_from_seed` samples combinations; it does not select from a
   list of already-built complete assets.
7. `TemplateDomain` has no compatibility gates. Every declared independent slot combination must
   build through parameter derivation, interface/host adaptation, or local transition geometry.
   Candidates that require a whole-host topology change belong in one structural-family slot.
8. N is multiplicity. It contributes to raw domain but not core domain. Palette, continuous
   parameters, decoration, and decorative counts contribute to neither.
9. Do not repair failures by deleting tests, special-casing failing seeds, freezing motion,
   broadening contact allowances, or removing failed combinations from the sampling domain.
10. Follow the authoritative allowance rule in
    `arti-template/agent/prompts/sections/sdk_base.md` for new and rebuilt Design-backed
    templates. `ctx.allow_isolated_part(...)` remains unchanged for now; legacy templates
    remain compatible.
11. Do not migrate the legacy fleet unless the user places specific templates in scope.
12. Preflight, random-16, random-36, corner, and the existing `target="full"` compiler path are
    purely mechanical. They must not open the PBR library or physics sidecars, sample a
    `material_seed`, or resolve a concrete material.
13. Existing `material_class_id` and `appearance_only` values remain PBR material-pool indexing
    metadata. Mechanical checks ignore them and do not resolve a concrete material ID.
14. `export-assets` is a post-mechanical packaging command, not another mechanical gate. It does
    not read a mechanical receipt, rerun authored tests or Full, or change the meaning of the
    retained corner report. The caller owns the ordering requirement that corner completed first.
15. `appearance.json` is the authoritative PBR rendering description. Do not write the sampled
    PBR material back into URDF visual/material elements.
16. Every exported visual must declare exactly one branch: a scalar `material_class_id` for a
    physical surface, or a typed `AppearanceOnlySpec` for rendering-only overlay, emission, text,
    decal, or screen content. Appearance-only visuals never enter a PBR pool and have null material
    identity and six null values in `physics.json`.
17. A sidecar package does not by itself prove simulator readiness. Target-specific adapters may
    consume the six values later, but they are outside this template-authoring workflow.

## A. SourceMap

Discover the complete active workbench pool for the exact picture subcategory before reviewing:

```bash
cd /mnt/zsn/lyb/arti-skill/arti-template
uv run python -m cli.main template --repo-root . source-map-init <slug> \
  --records-root <records-root> \
  --picture-category <picture-category> \
  --picture-subcategory <picture-subcategory>
```

Review every scaffold row. SourceMap is a small factual map from the complete source pool to
source-backed slots and candidates. For each accepted candidate, record the source
record/revision, exact `model.py` spans, component type, structural or motion distinction, and
evidence. Keep duplicate/reference/rejected sources in the review table so full coverage remains
auditable, but do not create duplicate candidates for them.

SourceMap must not contain copied code or dependency closure. Variant diff may inform the map,
but is not a separately maintained runtime truth. It must not contain parameter domains,
derivations, interfaces, bindings, multiplicity, or detailed category anchors.

Before `design-init`, require:

```bash
uv run python -m cli.main template --repo-root . source-map-check \
  --source-map <source-map.md> \
  --records-root <records-root>
```

## B. TemplateDesign

Scaffold the JSON from an accepted SourceMap:

```bash
cd /mnt/zsn/lyb/arti-skill/arti-template
uv run python -m cli.main template --repo-root . design-init <slug> \
  --source-map <source-map.md> \
  --records-root <records-root>
```

Then complete only decisions that require semantic judgment:

- component slots and structurally distinct candidates;
- honest independent parameters and local derived-parameter DAGs;
- semantic units for new independent parameters (`m`, `rad`, `ratio`, or `count`);
- plane/axis interfaces and their owning parts;
- multiplicity values, spacing, and host capacity;
- assembly bindings and cross-component derivations;
- host adaptation for candidate footprints, edge profiles, support spans, and required openings.
- for a new or rebuilt Design-backed template, lightweight `category_anchors` covering
  source-recognizable part roles, role/joint relations, and named author checks.
- for controls or inserts mounted through a local face, a plane-interface `surface_relation`
  recording the subject role, opening/footprint rule, signed normal exposure band, optional
  full-travel rule, minimum protrusion at maximum travel, and the category author check that
  enforces it.

TemplateDesign is a compact machine-readable design specification, not executable code and not
a verbose prose spec. Keep source code, caches, snapshots, and generated implementation blocks out.
Prefer fully using reviewed source components over inventing an additional candidate-extension
layer. A new structural candidate should first be made inspectable as source evidence.

## C. Implement one ordinary single-file template

Read the SourceMap evidence and TemplateDesign, then write
`agent/templates/<slug>.py` directly. Use private functions for component candidates and assembly.
The shape may resemble the established ordinary function-style templates in the fleet; a Factory
class hierarchy is not required.

New structured templates should expose:

```text
TEMPLATE_DOMAIN
Config / ResolvedConfig
config_from_seed / resolve_config
private component helpers and build functions
assembly derivations
build_<stem>
run_<stem>_tests
TEMPLATE_CORNERS (when extra continuous/high-risk cases are needed)
```

Use the SDK's lightweight `PlaneInterface` / `AxisInterface`, `mate_planes` / `mate_axes`, pose
sampling, and `TemplateDomain` where they reduce duplicated generic code. Do not introduce
adapter objects, component/binding manifests, or whole-component grafting. Keep category-specific
geometry and parameterization in the template. Existing templates are not forced to migrate merely
because the lightweight interface helpers are available.

Before mechanical certification, keep visual names unique within each part. Existing
`material_class_id` and `appearance_only` PBR labels may remain in the template, but the mechanical
workflow does not read a material pool or interpret those labels.

`TEMPLATE_DOMAIN` marks the new runtime contract. Every revolute/continuous joint in such a
template must be built from `mate_axes` and registered with `register_interface_mate`; the compiler
fails missing, non-axis, drifted, or wrong-axis registrations. For shelves, trays, panels, and other
planar mounts, declare the real supporting footprint with `PlaneInterface.extent` and let
`mate_planes` reject out-of-bounds placement. The legacy `MatingContract.tangential_containment`
field remains an opt-in axis-aligned mount-footprint check, not a rotation-axis check.

## D. Cumulative mechanical checks

Use one mechanical tolerance contract throughout authoring and repair:

- `expect_contact(...)`: 0.5 mm by default. Use a tighter explicit `contact_tol` only when the
  mechanism and collision geometry genuinely support that precision; preflight warns on literal
  values below 0.5 mm.
- part-to-part support: contact or a true gap up to and including 1 mm counts as supported.
- part-to-part overlap: true penetration up to and including 5 mm is tolerated; a failure also
  requires intersection volume above the SDK threshold. World AABB is broad phase only.
- same-part disconnected-island topology remains a separate 1e-6 m check. Do not use that numeric
  epsilon as an assembly-contact requirement.

```bash
uv run python -m cli.main template --repo-root . check <slug> --stage random-16
uv run python -m cli.main template --repo-root . check <slug> --stage random-36
uv run python -m cli.main template --repo-root . check <slug> --stage corner
```

**Preflight builds nothing.** It is a declaration gate before every requested stage, and its
report separates a static `contract` phase (Tier 0-A, AST + Design only) from a
`domain_resolution` phase (Tier 0-B, `config_from_seed`/`resolve_config` only). Both are
exhaustive; neither calls a build function. Across all 105 Design-backed templates it averages
0.15 s.

The division of labour: whether a candidate *builds* is proved by the sweep, which compiles it
regardless — random-16/36 sampling plus corner's supplement of candidates and N min/max absent
from seeds 0–35 guarantees the coverage. Building it in preflight first was duplicated work 89%
of the time (1324 of 1492 cached fleet reports were a pass). Geometry belongs to the sweep;
declarations belong to preflight.

Tier 0-A blocks duplicate top-level definitions and overwritten required exports, aligns
TemplateDesign against `TEMPLATE_DOMAIN`, resolves declared derived-DAG cycles and binding
endpoints, requires every `category_anchors` author-check name to exist in the template source
(string literal or f-string skeleton), and warns when a Design-declared numeric range has no
`TEMPLATE_CORNERS` override pinned to a declared edge. Role counts and role/joint relations are
enforced by those named author checks, which the sweep runs on real geometry in every built
sample — preflight no longer recomputes them against a stub mesh.

Tier 0-B walks the sampler until the declared domain saturates and resolves every config. It
blocks a declared candidate the sampler never produces, any sampled config that makes
`resolve_config` raise, a missing multiplicity `candidate × N` combination, and an independent
parameter pinned to a single value across the seeds that select its owning candidate (which is
what a silent coercion inside `resolve_config` looks like from outside). Cross-slot candidate
pair reachability is reported as a warning, since a bounded scan cannot separate unreachable
from rare. Enumerate through the sampler, never through `dataclasses.replace`: candidate-local
parameter bands make a hand-assembled combination something no seed can reproduce.

Numeric boundaries are now the corner stage's, and corner overrides accept any Config field, not
just domain slots. Author them in `TEMPLATE_CORNERS`.

When a Design interface declares `surface_relation`, its subject role and named author check must
also belong to the applicable category anchor. Implement that check with the SDK's generic
`expect_footprint_within` and `expect_normal_band` projections; keep category-specific openings,
clearances, recess depths, and exposure thresholds in the Design and template. For a pressable
control, use `expect_normal_band(..., minimum_protrusion=...)` at maximum travel so a decorative
legend or a rest-pose-only check cannot hide a cap that disappears below the face. These run in
the sweep on real geometry; there is no preflight AABB approximation of them any more, and
templates no longer need a `_meshes_materialized()` guard.

If preflight fails, do not start random sweep. Repair its machine-readable finding and rerun the
same command until preflight passes. The checker does not edit templates automatically.

- `random-16` checks cumulative seeds 0–15 with pass threshold 0.90.
- **The whole-sweep threshold cannot absolve one candidate.** Both random stages split the same
  outcomes per declared candidate (`random_sweep.candidate_coverage`); a candidate that was drawn
  and has zero passing seeds blocks the stage outright. This costs no extra builds. The hole it
  closes is real: `open_plate` occurs in exactly 1 of seeds 0–15 on the guitar tuner, so a
  candidate broken everywhere it appears still scored 15/16 = 0.9375. A candidate never drawn is
  `uncovered`, not failed — corner owns those.
- `random-36` checks cumulative seeds 0–35. With an unchanged mechanical hash, it reuses valid
  0–15 artifacts and runs only missing seeds. Direct invocation also runs only missing seeds.
- `corner` first requires random-36 to meet its threshold. For Design-backed templates it builds
  only component candidates and N minimum/maximum values that did not occur in cumulative seeds
  0–35, then adds every authored high-risk cross-slot boundary. Joint motion edges remain mandatory
  in every built random or corner sample.

Every built sample runs CAD, authored tests, URDF, topology, disconnected geometry, collision, and
motion QC. By default every bounded revolute/prismatic joint must cover lower and upper; continuous
joints use global neutral plus -90/+90/180-degree representative poses, and mimic joints are covered
by their driver. Required single-joint poses are never truncated by the optional sampling budget.
Coverage comes from pose IDs actually executed by collision QC, never from URDF-limit inference.

An explicitly homogeneous repeated sibling mechanism may tag each articulation with the same
`motion_qc_repeat_group`. Every member still needs exhaustive authored checks for part/joint
existence, unique child ownership, axis, limits, and independent motion. Only the expensive
lower/upper collision poses are sampled: at most 12 deterministic spatial-edge, center, and
seed-rotated representatives per group. Untagged joints remain exhaustive, and the report must
record the group member count and the exact sampled joint names. Reject heterogeneous use of the
tag; never use it to replace joints with mimic motion.

For `TEMPLATE_DOMAIN` templates, the compiler baseline also verifies that every rotational joint
has a registered axis interface and still uses the solved origin/orientation/axis. This runs once
per built seed; motion poses continue to own swept collision rather than repeating the static axis
contract at every pose.

After corner passes, render representative seeds with the existing preview script and review a
contact sheet for category recognizability. This authoring review does not create a persistent
receipt or runtime dependency.

The cache is an execution optimization, not an authoring artifact. A changed mechanical hash
invalidates stale seed results. Repeating unchanged random-36 should report 36 hits and 0 misses.

## E. Optional post-mechanical portable asset export

Run this phase only when the user requests a portable PBR/material-physics package. It follows a
current strict corner pass by orchestration, but remains independent from the mechanical state
machine. Do not invent or require a new receipt transition.

Before export:

- confirm that the retained corner report belongs to the current mechanical hash;
- confirm that `pbr_material_library/unified/latest.json` exists and let the strict runtime loader
  verify its pointer, catalog, manifest, pool index, selected pools, physics sidecars, and PNGs;
- require every runtime visual to declare exactly one of scalar `material_class_id` or typed
  `AppearanceOnlySpec`; repair an untagged or conflicting visual before export;
- choose an output root outside authoring caches. Existing seed targets fail closed and are never
  overwritten.

Run from `arti-template`:

```bash
uv run python -m cli.main template --repo-root . export-assets <slug> \
  --seeds <seed-list> \
  --out <output-root> \
  [--material-seed <integer>] \
  [--library-pointer pbr_material_library/unified/latest.json]
```

The default material seed is each geometry seed. An explicit `--material-seed` applies the same
local RNG seed to every requested geometry seed. Selection is direct and deterministic: construct
one `random.Random(material_seed)`, visit canonical unique `(group, material_class_id)` pairs, and
call `choice()` once on each exact-class pool. Surfaces with the same authored material name and
the same class share that draw; otherwise selection is per `part::visual`. Snapshot and pool hashes
are provenance, never extra inputs to the random draw.

Each result is published as:

```text
<output-root>/<slug>/seed_<n>/
├── model.urdf
├── assets/
│   ├── meshes/*.obj
│   └── textures/
│       ├── mat_<material-id>/*.png
│       └── <appearance-only authored textures>
├── appearance.json
└── physics.json
```

`model.urdf` is an ordinary compile of the same template seed with physical collision meshes. The
exporter copies OBJ and authored texture dependencies, but PBR sampling does not alter its visual
materials. `appearance.json` is authoritative and records the selected concrete material, factors,
runtime channel bindings, copied PNG receipts, source provenance, selection provenance, and typed
appearance-only payload. Spatially constant channels remain factors; do not manufacture
placeholder textures.

`physics.json` has exactly one row for every `part::visual`. A physical row contains only these six
public material values: density, Young's modulus, Poisson's ratio, static friction, dynamic
friction, and restitution. An appearance-only row has null material ID/class and all six values
null.

The exporter builds all requested seeds in private staging and publishes them only after complete
validation. Require:

- identical, sorted `part::visual` surface sets in `appearance.json` and `physics.json`;
- copied asset containment, size, SHA-256, PNG decodability, declared dimensions, colorspace,
  decoded-pixel hash, and OpenGL tangent-normal convention where applicable;
- every PBR channel texture to join its file receipt, and every published material PNG to be
  referenced by a binding;
- only OBJ files below `assets/meshes`, no symlinks, and no `.blend` anywhere in a seed package;
- appearance-only rows to have no resolved material or physics values.

## F. Repair and completion

Use `random_sweep.repair_findings` (and corner repair findings when applicable) before reading the
full failed-seed payload. Each finding groups only the current cumulative run and reports affected
seeds/cases, structured parts/joints/elements/pairs/poses, correlated Config fields, the nearest
passing Config, and candidate function/source-line owners. Treat Config correlations as evidence,
not causality. Prefer the highest-impact finding, compare its representative failure with its
nearest pass, then edit the smallest relevant ordinary component or assembly function.

Obey each finding's `source_review_policy` before editing. When `blocking=true`, read every exact
record/revision `model.py` span in `source_targets` first. If a Design-backed finding requires source
review but its target mapping is unresolved, repair the SourceMap/TemplateDesign mapping before
changing the template. For `decision=recommended`, review the source whenever the Design does not
settle the mechanism intent or the proposed repair would alter source structure, interfaces, or
major proportions. Compile/runtime, asset-readiness, timeout, and missing exact-name failures do
not require source review. `repair_gate.status=source_review_required` is a hard repair prerequisite;
do not patch around it or continue to a later sweep stage.

Treat `preflight.quality_contract` as the approved seed-quality contract. Every sweep repair gate
includes its digest and the required random-16 rerun parameters. After changing the template, run:

```bash
uv run python -m cli.main template --repo-root . check <slug> --stage random-16 \
  --expected-quality-contract-sha256 <sha256>
```

The rerun must preserve slot candidates, multiplicity, independent parameter ranges, derived DAGs,
bindings, and interface semantics. Source `model.py` supplies structural and mechanism intent but
must not replace approved parameterization or derivations with fixed source constants. A digest
change is blocked as `quality_contract_changed_during_repair`; if the change is intentional, leave
the repair workflow, update the Design explicitly, and restart at preflight. Source navigation and
internal implementation refactors do not affect the digest.

For newly authored templates, prefer diagnostic entity names shaped as
`<slot>__<candidate>__<local_name>` so TemplateDesign's `implementation_function` can resolve a QC
part/joint directly to a function and line. Legacy templates fall back to traceback and exact entity
string ownership. Repair findings are ephemeral navigation: do not create persistent cluster/streak
state, alter the combination domain from failures, or make them part of seed construction.

After a mechanical edit, restart at random-16; valid cache behavior is automatic.

Both cumulative random stages use a 0.90 pass threshold; strict templates cannot lower it through
the CLI. Mechanical completion means random-16, random-36, and corner pass. A strict corner pass
with 36/36 random seeds is `strict_ready`; a pass with 33–35/36 is
`strict_ready_with_tolerance`. Corner cases themselves remain zero-tolerance. These statuses do not
claim Blender visual approval, source fidelity reconstruction, snapshot reproducibility, or
publication approval.

## Legacy fleet

Existing single-file templates remain legacy-compatible and do not need retroactive TemplateDesign
or `TEMPLATE_DOMAIN`. For release-wide maintenance, run import, `config_from_seed(0)`, and seed-0
compile smoke. Only deliberately selected templates enter the new authoring workflow.
