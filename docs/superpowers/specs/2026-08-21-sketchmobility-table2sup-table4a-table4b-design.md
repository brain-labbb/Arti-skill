# SketchMobility Table 2 Supplementary, Table 4a, and Table 4b Design

## Objective

Produce formal, fail-closed results for the SketchMobility rows in Table 2
supplementary, Table 4a, and Table 4b. All three evaluations use the exact
previously frozen N=800 SketchMobility cohort. Each result is independently
verified before its row and evidence paragraph are added to
`exp/URDF-Sim-Ready-Automatic-Evaluation.md`.

## Frozen Cohort And Upstream Evidence

The authority for selection identity and order is the existing formal Table 4
manifest:

`exp/runtime/urdf_table4_sketch_mobility_table1cohort_n800_20260821T090554Z/manifest.json`

The adapters must retain all 800 `items[]` in stored order and bind:

- ordered asset IDs SHA256
  `a88506e1da8e7e8b61a740965dea2faba4e9ab8280f47417e17550024b6dde17`;
- every `selection_rank`, asset ID, source/category, `mobility.urdf` SHA256,
  recursive package-content SHA256, and Table 4 joint specification;
- the formal Table 2 receipt for collision-coverage intent;
- the formal Table 3 receipt for joint-level executable correctness;
- the formal Table 4 manifest, asset records, and state records for historical
  strict results and exact K=21 state identities.

No failed asset may be dropped, replaced, or moved. All asset, joint, and state
denominators are intent-to-evaluate denominators.

## Architecture

Use three dataset-specific SketchMobility adapters rather than a combined
runner. Table 2 supplementary is a static XML/resource audit, Table 4a uses
Genesis contact penetration, and Table 4b uses exact surface-distance geometry
and isolated load timing. Separate adapters keep environment and recovery
boundaries explicit while sharing the already frozen metric atoms.

Each adapter has a standalone verifier that reads persisted bytes without
importing the live runner. Formal receipts are write-once, contain frozen source
snapshots and an artifact manifest, and support an atomic per-rank journal plus
strict resume where the runtime is material. A formal run may start only after
live read-only replay of its fixed N=5 smoke receipt succeeds.

## Table 2 Supplementary

The adapter evaluates all 800 `mobility.urdf` packages using the current frozen
`lam_supplementary_static` atom. The receipt pins both the declared evaluator
version and the exact module SHA256; it must not claim equivalence to an older
historical source hash.

Metrics follow the document definitions exactly:

- visual-bearing collision coverage is asset-level and fails closed on parser,
  extraction, resource, or load failure;
- joint-limit portability and joint dynamics coverage use all 1,824 intended
  movable joints;
- the placeholder registry remains the pre-registered empty registry, so
  placeholder-mass incidence is `N/E`, with complete-inertial coverage reported.

The verifier independently reloads the frozen cohort, recomputes package and
URDF bindings, reparses persisted asset records, reaggregates every numerator
and denominator, and checks the complete artifact closure.

## Table 4a

The collision oracle is the existing `genesis_contact_penetration_v1` adapter:
Genesis 1.3.1, CPU backend, precision 64, trimesh 5.0.0, rtree 1.4.1, penetration
strictly greater than `1e-6 m`, collision geometry only, and direct parent-child
pairs excluded under the common headline policy. Each joint uses the historical
Table 4 K=21 sweep including endpoints, with every other joint at q=0.

The formal denominator is N=800 and J=1,824. The 489 assets already failing the
frozen Table 2 collision-coverage gate are emitted as terminal fail-closed
records without starting Genesis. Genesis runs only for the remaining 311
assets. This is an execution optimization, not outcome filtering: all 489
assets, their joints, and their intended states remain failures in every
applicable denominator.

Every generated state vector is canonically hashed and matched to the Table 4
state receipt where a historical state exists. Missing historical executions
are regenerated only by the frozen K=21/q=0 rule and recorded as no-reference.
Table 3 joint-level pass is joined positionally and by asset/joint identity for
Executable CF DoF and Collision-safe DoF Retention. Historical Table 4 strict
pass is used only for the required declared-DoF bin table.

Genesis contact does not provide complete signed clearance for separated
pairs. `Normalized Clearance P5` therefore remains `N/E`, with measured state
and asset coverage reported as `PARTIAL` where appropriate. Load, mapping,
pair-policy, execution, child, and timeout failures all remain fail closed.

The frozen formal configuration is 16 workers, one private Genesis cache per
rank, 1.5 second launch staggering, and a 3,600 second per-asset timeout. The
expected wall time is approximately two to six hours, depending on mesh load
cost and host contention.

## Table 4b

The adapter evaluates all 800 packages using the existing exact geometry atom:
collision geometry only, no visual fallback, frozen surface sampling and weld
tolerance, and bidirectional nearest-surface queries implemented with
`trimesh.proximity.ProximityQuery.on_surface` plus rtree. Distances are
normalized by the q0 loadable visual-union AABB diagonal.

It reports analytic collision share, both surface P95 directions,
shapes/visual-bearing link, valid collision-mesh triangles/asset, intra-link
redundancy, and isolated collision load time. Timing uses one warmup plus five
measured loads in a fresh child with one numerical thread. Failures and missing
geometry produce `PARTIAL` or `N/E` coverage rather than shrinking N=800.

The frozen formal configuration is 16 workers and a 900 second per-asset
timeout. The expected wall time is approximately 20 to 40 minutes.

## Testing, Verification, And Publication

Implementation follows test-driven development. Tests first cover cohort
identity, path containment, recursive package binding, fail-closed records,
aggregate recomputation, source snapshots, write-once publication, interruption
and resume, smoke-gate replay, and semantic tamper rejection. Each test must be
observed failing for the intended missing behavior before production code is
added.

For each table, execution order is:

1. focused tests and syntax checks;
2. independent code review with all Critical and Important findings resolved;
3. fixed N=5 smoke and standalone verifier replay;
4. formal N=800 run;
5. standalone verifier replay with a before/after whole-receipt digest check;
6. backfill only the matching SketchMobility row, supporting macro/bin table
   when required, status text, and immutable evidence links;
7. verify that all reported values and hashes match persisted receipt bytes.

Table 2 supplementary runs first, Table 4a second, and Table 4b third. A failed
stage does not alter a later stage's protocol or cohort and is repaired or
resumed before publication.

