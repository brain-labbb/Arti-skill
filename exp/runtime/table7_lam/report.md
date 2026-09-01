# Table 7 LAM production readiness

Status: **COMPLETE**

This is a representation-aware static audit of the official released LAM
`viable` package cohort frozen for Table 6. It is not a generation rerun. The
100 requested categories were fixed before Table 7 scoring and no asset was
selected or dropped based on geometry or completeness outcomes.

## Cohort and artifact provenance

- Requested/available: 100/100; geometry-evaluable: 100; package-evaluable: 100.
- Scored layer: official **released** archive extracts only. Locally generated=0; recovered=0.
- Final geometry scope: unique mesh payloads declared by each top-level `generated.urdf`; intermediate package meshes outside that final dependency graph are not scored.
- A filename such as `generated.urdf` does not imply local generation; release hashes are checked against the frozen Table 6 inline URDF hashes.

## Static results

- Readable final geometries: 554 from 554 mesh files; load failures=0.
- Watertight: 0/554 geometries; per-asset mesh-fraction mean=0.000.
- Edge-manifold proxy: 554/554 geometries; per-asset mesh-fraction mean=1.000. Tool: NumPy undirected-edge incidence on trimesh `process=false` triangle faces. Vertex-manifold is explicitly not claimed.
- Open edges: 1021164 total; 10211.640/asset mean. Nonmanifold edges (>2 incident faces): 0 total.
- Degenerate faces (repeated index or area <=1e-12): 0 total; 0.000/asset mean.
- Self-intersection: **N/A**; no exact triangle-triangle backend with adjacent-face exclusion was recorded.

## Package and completeness

- Source: 3561.89 KiB total, 35.62 KiB/asset. This is per-package executable source (`.js/.mjs/.cjs/.py/.ts/.tsx`).
- URDF: 281.67 KiB total, 2.82 KiB/asset.
- Final referenced mesh: 147845.08 KiB total, 1478.45 KiB/asset, canonical-path deduplicated within asset.
- Portable package: 100/100 pass after complete-package copy to a fresh workspace-internal directory and recursive relative dependency resolution; fail=0, N/A=0.
- Deterministic build: **N/A (100/100)**. No second fresh build was run; release/content hashes are provenance evidence only.
- Semantic complete: **N/A (100/100)** without an output-independent required-part/role specification. The separately labelled name+tree field proxy passes 100/100.
- Kinematic complete: 35/100 pass, 65 fail, 0 N/A. Every joint must explicitly declare finite `origin xyz/rpy`; simulator identity defaults do not pass this gate.
- Physical complete: 0/100 pass, 100 fail, 0 N/A. Native collision, positive mass/inertia, joint damping/friction, and contact/friction metadata are all required; PyBullet defaults never count.

## Table 7 row

| Method | Watertight | Manifold | Open Edges | Degenerate Faces | Self-Intersection | Source KB | URDF KB | Mesh KB | Portable Package | Deterministic Build | Semantic Complete | Kinematic Complete | Physical Complete |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| LAM official release (frozen N=100) | 0.000 mean/asset; 0/554 geom | 1.000 edge-manifold mean/asset; 554/554 geom | 10211.640/asset; 1021164 total | 0.000/asset; 0 total | N/A | 35.62/asset; 3561.89 total | 2.82/asset; 281.67 total | 1478.45/asset; 147845.08 total | 100/100 | N/A (no two fresh builds) | N/A; field proxy 100/100 | 35/100 | 0/100 |

## Evidence

- `protocol_snapshot.json`: byte-for-byte frozen shared protocol.
- `manifest.json`: pre-score frozen identities and input evidence hashes.
- `asset_records.json`: per-asset, per-geometry, dependency, size, and field-gate evidence.
- `summary.json`: explicit denominators and aggregates.
- `self_check.json`: cohort/accounting/path/hash assertions.
- `hashes.sha256`: hashes for the runner and all required outputs.
