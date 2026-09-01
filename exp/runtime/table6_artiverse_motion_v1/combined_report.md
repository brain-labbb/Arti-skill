# Artiverse Table 6 reference audit

Status: paper-ready for the full-release static audit and the frozen discrete
penetration-free proxy, subject to the claim boundaries below. Artiverse is a
real-data release/export reference and is excluded from generated-method
rankings.

## Copy-ready results

| Evaluation layer | Denominator | Result | Paper status |
|---|---:|---:|---|
| Full-release core completeness | 3,544 assets | 3,543 / 3,544 (99.97%) | Reportable release audit |
| Semantic-to-URDF mapping completeness | 3,544 assets | 3,513 / 3,544 (99.13%) | Reportable release audit |
| Static motion eligibility | 3,544 assets | 3,494 / 3,544 (98.59%) | Reportable release audit |
| Native URDF load and drive | 100 intent assets | 99 / 100 (99.0%); 1 N/E timeout | Reportable operational proxy |
| Discrete penetration-free asset pass | 100 intent assets | 58 / 100 (58.0%); 41 FAIL, 1 N/E | Primary proxy cell |
| Discrete penetration-free asset pass | 99 evaluable assets | 58 / 99 (58.59%) | Secondary evaluable-only rate |
| Semantic-joint single-sweep pass | 598 evaluable semantic joints | 243 / 598 (40.64%) | Reportable path proxy |
| Strict continuous collision-free | 100 intent assets | N/E | Do not fill from the proxy |
| Joint type/recall/parent-child/axis/origin/limit accuracy | independent gold | N/A | No independent gold |

The full 3,544-asset release contains 16,471 raw annotation records, 16,355
semantic joints, 16,437 semantic DoFs, and 16,332 exported movable URDF joint
elements. Metadata validity is 16,355/16,355 for axis, 5,827/16,355 for
origin, and 16,344/16,355 for range. These are field-validity checks, not
accuracy against independent gold. The advertised
`mass_furniture_heuristic.json` payload is absent for all 3,544 assets and must
not be claimed as covered.

## Motion protocol

The outcome-blind frozen cohort contains 100 assets and uses no failure
replacement. Its intent denominators are 702 raw records, 694 semantic joints,
699 semantic DoFs, and 699 exported movable URDF elements. Every evaluable
asset is queried at its nominal midrange, at 11 states for each scalar
coordinate, and at 64 unscrambled Sobol states when it has more than one DoF.
The 99 completed assets therefore contribute 11,724 configuration queries:
6,633 single-coordinate states, 4,992 Sobol states, and 99 nominal-midrange
states.

A query fails only when a retained PyBullet signed `contactDistance` is below
`-1e-6 m`. Contact manifolds at or above that threshold are diagnostics and do
not fail the penetration-free proxy. Same-fixed-cluster pairs and semantic-pid
parent-child adjacency are excluded after transparent composite-proxy
collapse. The final run contains 4,979 penetration states and 494 states with
retained contacts only within tolerance. All 41 failing assets have first and
deepest signed-distance witnesses.

One 96-DoF piano exceeded the frozen 600-second per-asset limit. It remains in
the intent denominator as N/E; it was not replaced. The remaining 99 assets
loaded, drove, and completed evaluation. The 47 assets with 493 missing
prismatic annotation origins remain explicit metadata warnings because this
operational proxy executes the content-hashed native URDF origins; this does
not establish origin accuracy.

## Paper text

On the full Artiverse release (3,544 assets), 3,543 assets (99.97%) satisfied
the core completeness contract and 3,513 (99.13%) had complete
semantic-to-URDF mappings. On an outcome-blind frozen 100-asset cohort, 99
assets loaded and completed the prescribed motion queries. Fifty-eight assets
were penetration-free over all sampled paths, 41 exhibited retained
penetration deeper than 1 micrometer, and one 96-DoF asset exceeded the
600-second timeout, yielding 58.0% over the intent cohort (58/100) or 58.59%
over evaluable assets (58/99). At the semantic-joint level, 243/598 evaluable
joints (40.64%) passed all 11-state single-coordinate sweeps. These values are
discrete PyBullet penetration proxies; continuous collision certification and
independent articulation-semantic accuracy remain N/E and N/A, respectively.

## Superseded diagnostics

| State | Artifact | Reason excluded |
|---|---|---|
| `SUPERSEDED_SCORER_BUG` | summary SHA `61473fd59a05...` | Missing annotation origin was incorrectly fatal for native-URDF execution. |
| `SUPERSEDED_PROVENANCE_HARDENING` | summary SHA `c1b1d3f8f082...` | Pre-content-addressed diagnostic artifact. |
| `INVALIDATED_DIAGNOSTIC_EXCLUDED` | run `7f6321ba472d_8acc11a9cb1b_c966b6340a5d`, summary SHA `99e9a44c40ae...` | Any retained contact manifold was counted as failure; seven of 49 FAIL assets had zero penetration. |
| `ABORTED` | run `7f6321ba472d_1971a7a8cf7f` | Incomplete provenance-hardening run; no result. |

The invalidated 51 PASS / 49 FAIL diagnostic is preserved for audit only and
must not be cited. The signed-distance correction is recorded by amendment SHA
`7283305aeca28af8ba8469ab8c7ab1419dd1bb0068714d12f850ae460ce8bc35`;
the cohort identities, order, and selection hashes did not change.

## Canonical evidence

- Final run: `fd91a2d701b2_38be4cf7cd51_b31c87c13641`
- Protocol SHA: `663b4d62914372cc06a1d4f89519f35efd181e481388ea4716920af6fd96688d`
- Cohort SHA: `fd91a2d701b260517c70496a49c81f02799d235e82e670592608178bc10c3dce`
- Runner SHA: `38be4cf7cd512aaeab76d940147021e255224b25ca7440a5b704ea4a545818db`
- Run-config SHA: `b31c87c13641d1b52be5fe509583a577e9802b77c77cec1252d31c75a4388f7b`
- Final summary SHA: `da2d98fb44aa082ba2d21fdaae3e83c2a58fab93d9f46d58e54b264c66ccb48b`
- Final records SHA: `266afee5157c73b12799663aa6edc13e1b497216a12e5339d5f7fd5832027f36`

Claim boundary: this audit supports release/export completeness and a frozen
discrete penetration-free motion proxy. It does not support a strict
continuous collision claim or independent semantic-accuracy claims.
