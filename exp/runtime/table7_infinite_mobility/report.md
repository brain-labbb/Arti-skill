# Infinite Mobility Table 7 Production Readiness

## Frozen cohort

- Protocol: `nano3d_table7_production_readiness_v1`
- Protocol SHA-256: `5fc86932f35f8b66514d5747be732b5c75fef7215c987628f5dd28522f710a7c`
- Manifest SHA-256: `recorded after report generation in self_check.json`
- Requested: 720 (20 factories x 36 frozen seeds)
- Available strict final packages: 713
- Generation timeouts retained: 7
- Generated with movable joints: 658
- Generated without movable joints: 55

## Table 7 row evidence

| Metric | Locally measured result |
|---|---|
| Watertight | per-asset mean geometry fraction 0.000000; geometry-level 0/12408 = 0.000000 |
| Manifold | edge-manifold proxy per-asset mean 0.965990; geometry-level 12103/12408 = 0.975419; vertex-manifold not claimed |
| Open Edges | 590531.473/geometry-evaluable asset mean; 421048940 total |
| Degenerate Faces | 2721.063/geometry-evaluable asset mean; 1940118 total |
| Self-Intersection | N/A (`not_evaluable`: no exact backend with adjacent-face exclusion) |
| Source KB | per-asset N/A; 662.209 KiB across 20 unique shared factory-defining modules |
| URDF KB | 8.706 KiB/available asset; 6207.386 KiB total |
| Mesh KB | 25392.839 KiB/available asset; 18105094.213 KiB total packaged mesh payload |
| Portable Package | 713/713 available; 713/720 intent-to-run |
| Deterministic Build | N/A (`not_evaluable`: no second fresh same-factory same-seed build) |
| Semantic Complete | N/A strict; name/tree field proxy 0/713 available |
| Kinematic Complete | 658/658 native articulated packages; 55 generated packages have no movable joint; 7 unavailable |
| Physical Complete | 0/713 available native simulation packages; 0 native collision elements |

## Claim boundary

The 720 requested factory-seed identities are conserved without success-based reselection. Geometry load failures, if any, are excluded only from geometry-evaluable denominators and are not counted as clean meshes. Semantic strict completeness and deterministic rebuild remain N/A. Physical completeness is fail-closed: these exports contain zero native collision elements, so a simulator reporting no contact would be vacuous and is not used.
