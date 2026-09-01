# Table 7 UniPhysGen Production Readiness Release Audit

Status: **RELEASE_BLOCKED**  
Interpretation: **NOT_AN_EXPERIMENTAL_ZERO**  
Evaluated final packages: **N=0**

The frozen official checkout remains a release placeholder at commit
`742d4e6170ee132144880afb374dac2c1bc46c8a`. Its exhaustive non-`.git` inventory contains only
`.gitignore`, `LICENSE`, and `README.md`; their byte sizes, Git blob IDs, and
SHA-256 hashes are recorded in `manifest.json`. There is no runnable code,
compatible checkpoint, output package contract, or attributable final
simulation package.

This is a release-readiness blocker, not a failed UniPhysGen experiment and not
evidence that the method generated zero successful assets. No inference,
generation, geometry scoring, package test, or deterministic rebuild was run,
and no paper value was substituted for a local measurement.

## Denominators

- requested assets: 0
- available assets: 0
- geometry-evaluable assets: 0
- package-evaluable assets: 0

## Table 7 row

| Method | Watertight | Manifold | Open Edges | Degenerate Faces | Self-Intersection | Source KB | URDF KB | Mesh KB | Portable Package | Deterministic Build | Semantic Complete | Kinematic Complete | Physical Complete |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| UniPhysGen (official release placeholder; N=0) | not_evaluable | not_evaluable | not_evaluable | not_evaluable | not_evaluable | not_evaluable | not_evaluable | not_evaluable | not_evaluable | not_evaluable | not_evaluable | not_evaluable | not_evaluable |

Every performance cell is `not_evaluable`: there is no final artifact on which
to measure a pass, failure, byte size, or error count.

## Blocking release items

1. Runnable official inference code and environment specification.
2. Compatible official checkpoint with provenance and license.
3. Official output/final-package schema and dependency contract.
4. At least one attributable final simulation package, released or generated.

Protocol SHA-256: `5fc86932f35f8b66514d5747be732b5c75fef7215c987628f5dd28522f710a7c`  
Manifest SHA-256: `c4a05b83cc2897745268c9541db9383f790ad98527320630bb38b82e8e35796c`
