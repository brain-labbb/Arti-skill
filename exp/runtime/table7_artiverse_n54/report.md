# Table 7: Artiverse production readiness (frozen N=54)

Status: **COMPLETE**

The cohort is the first 54 full asset IDs under a fixed salted SHA-256 rank over
the frozen 3,544-asset manifest. Selection reads no outcome field and failed or
unavailable selected assets would remain in the denominator. The selected cohort
covers 23 categories and 10 source repositories.

## Table 7 row

| Method | Watertight | Manifold | Open Edges | Degenerate Faces | Self-Intersection | Source KB | URDF KB | Mesh KB | Portable Package | Deterministic Build | Semantic Complete | Kinematic Complete | Physical Complete |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Artiverse (frozen N=54) | 0.021914 mean/asset; 6/547 mesh components | 0.997354 edge-manifold mean/asset; 546/547 mesh components | 12,347.19/asset; 666,748 total | 5.65/asset; 305 total | N/E | shared 48.47 KiB total; per-asset N/E | 36.19 KiB/asset; 1954.32 KiB total | 3259.31 KiB/asset; 176002.94 KiB total | 54/54 | N/E | N/E strict; field proxy 53/54 | 54/54 | 0/54 |

## Denominators and boundaries

- Asset denominator: requested/available/package-evaluable/geometry-evaluable = 54/54/54/54.
- A mesh component is one independently loaded triangle-mesh object inside an asset; one asset may contain several components. Readable mesh components: 547.
- Watertight and edge-manifold report an asset-macro mean plus a mesh-component numerator/denominator.
- Open edges and degenerate faces sum over the same mesh components, then divide by geometry-evaluable assets for the per-asset mean.
- Strict semantics remain N/E. The separately labelled name/tree field proxy is not semantic correctness.
- Physical Complete is a strict AND over native collision, mass, inertia, joint dynamics, and contact/friction metadata; runtime defaults do not count.

## Reproduction

```bash
python arti-skill/exp/scripts/run_table7_artiverse_n54.py
python arti-skill/exp/scripts/run_table7_artiverse_n54.py --verify-only
```
