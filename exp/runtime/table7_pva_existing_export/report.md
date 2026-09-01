# PV-A existing-export Table 7 manifold backfill

Status: **COMPLETE**

The cohort is the original frozen N=33 existing-export pilot. Ten original
`seed_exports_physics_10` paths are no longer present, so the audit uses the
preserved N=33 input-package copies. Identity and URDF hashes match the legacy
manifest 33/33, and the old geometry statistics reproduce exactly before the
new metric is accepted.

## Result

| Method | Manifold |
|---|---:|
| PV-A existing-export pilot (N=33) | 0.993295 edge-manifold proxy mean/asset; 382/387 geometries |

- Assets whose every geometry passes: 31/33.
- Nonmanifold edges (>2 incident faces): 261 total.
- Load errors: 0.
- Definition: every undirected edge has at most two incident faces. Boundary
  edges are allowed; vertex-manifold is not claimed.
- Legacy reproduction gate: **PASS**.
