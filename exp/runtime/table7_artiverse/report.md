# Table 7: Artiverse production readiness

Status: **COMPLETE**

This static audit retains all 3544 identities in the official
two-chunk manifest (84 categories, 10 source repositories). No asset was selected,
repaired, rebuilt, or dropped based on a Table 7 outcome.

## Cohort and provenance

- Requested/available: 3544/3544; unavailable=0.
- Geometry-evaluable: 3543; package-evaluable: 3543.
- Dataset revision: `8c4b120418e7cbdf9ac4c9580c5dbfdbf128a248`; official code commit: `44f3d41d015018e9b4dff2cbf01fd0892fe6b2c5`.
- Extraction matched 3,544 roots, 531,937 files, and 86,992,752,890 input bytes.
- Geometry scope is the canonical-deduplicated URDF visual-mesh dependency closure.
- Headline Mesh KB is the canonical-deduplicated union of all URDF-declared visual and collision mesh paths. Unreferenced packaged GLB, convenience, and decomposition representations are excluded; visual-only and all-packaged byte totals are retained as auxiliary evidence.

## Table 7 row

| Method | Watertight | Manifold | Open Edges | Degenerate Faces | Self-Intersection | Source KB | URDF KB | Mesh KB | Portable Package | Deterministic Build | Semantic Complete | Kinematic Complete | Physical Complete |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Artiverse frozen pre-release subset (N=3544) | 0.030429 mean/asset; 1493/43359 geom | 0.996074 edge-manifold mean/asset; 43258/43359 geom | 20229.299/asset; 71672407 total | 181.250/asset; 642170 total | N/E | shared 48.47 total; per-asset N/E | 20.33/asset; 72040.45 total | 3582.79/asset; 12693841.49 total | 3543/3544 | N/E (no two fresh builds) | N/E; field proxy 3526/3543 evaluable; 1 N/E | 3539/3541 applicable | 0/3544 applicable |

## Evidence-state notes

- Readable geometries: 43359; mesh-load-error assets: 0.
- Edge manifold is the <=2 incident-face proxy; vertex manifold is not claimed.
- Self-intersection is N/E because no exact adjacent-face-excluding backend ran.
- Source size is shared method code and is not divided into artificial per-asset source bytes.
- Portable Package copies the complete native simulation subpackage (`urdf_w_collider`) into fresh workspace-internal storage, then parses the copied URDF and recursively parses and resolves declared dependencies. Mesh readability is scored independently by the geometry audit. It does not claim to copy the entire Artiverse model root.
- Deterministic Build is N/E: existing hashes are provenance, not two fresh builds.
- Strict Semantic Complete is N/E for 3544/3544; the separately labelled name/tree field proxy is not semantic correctness.
- Physical completeness is fail-closed: simulator defaults do not satisfy native collision, mass, inertia, joint dynamics, or contact metadata gates.

## Reproduction

```bash
python arti-skill/exp/scripts/run_table7_artiverse.py --preflight-only
python arti-skill/exp/scripts/run_table7_artiverse.py --workers 2 --preflight-snapshot arti-skill/exp/artiverse/full_preflight.tmp.json
python arti-skill/exp/scripts/run_table7_artiverse.py --verify-only
```
