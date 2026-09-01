# Table 7: Nova3D production-readiness audit

- Status: **BLOCKED / NOT_EVALUABLE**.
- Local evaluable cohort: **N=0**.
- Official checkout commit: `042ee613aa2fb745d287261eab029d42c704646e`.
- Final asset files in the checkout: **0**.
- Hosted API, paid generation, network, secret, and GPU use in this runner: **none**.

## Local common-protocol row

| Method | Watertight | Manifold | Open Edges | Degenerate Faces | Self-Intersection | Source KB | URDF KB | Mesh KB | Portable Package | Deterministic Build | Semantic Complete | Kinematic Complete | Physical Complete |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| Nova3D (official public checkout; local N=0) | not_evaluable | not_evaluable | not_evaluable | not_evaluable | not_evaluable | not_evaluable | not_evaluable | not_evaluable | not_evaluable | not_evaluable | not_evaluable | not_evaluable | not_evaluable |

`N=0` is an availability result, not a zero performance score. The public repository
contains open clients and integrations, while its README identifies the hosted generation
backend as closed-source and marks generated examples plus source programs as coming soon.
No final package exists on which geometry, copying, rebuilding, or field completeness can
be measured.

## Separate paper-only evidence

The existing Table 7 transcription reports N=54 GLB assets with watertight
`0.89`, manifold `0.91`, mean open edges
`42`, source `18.7 KB/asset`, and
GLB size `1.16 MB/asset`. It also transcribes the claims
`54/54 runtime-ready GLB`, `deterministic headless build reported`, and
`named/tree claims reported`. These values were not reproduced from this checkout and are
never used as local measurements.

## Blockers and denominators

- `CLOSED_BACKEND`: generation is dispatched to the proprietary hosted service.
- `PAPER_ASSETS_UNRELEASED`: no paper GLB/source packages or examples are in the checkout.
- requested assets: 0; available assets: 0; geometry-evaluable assets: 0;
  package-evaluable assets: 0.
- Every local Table 7 column is `not_evaluable`; paper-only N=54 remains a separate
  evidence layer.
