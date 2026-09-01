# Infinite Mobility Table 2 Naming direct evaluation

This is a public-factory supplementary cohort, not a common-category matched result.

## Frozen policy

- One URDF part is a link with at least one valid renderable visual geometry (mesh, box, cylinder, or sphere under protocol v1.1).
- Multiple renderable visuals on one link are merged into one part node; geometry-type and mesh-reference counts remain audit fields.
- The shared Nano3D placeholder regex is used unchanged. Therefore `l_<index>` passes lexical Nameability even though it is separately flagged as an opaque generated name.
- Original 300-second PASS packages enter directly; the seven original TIMEOUTs use separately recorded 900-second strict-PASS recovery packages only for Naming artifact coverage.
- Cross-seed metrics use raw names only. They are not semantic role consistency.

## Direct results

| Metric | Result | Scope |
|---|---:|---|
| Parts | 10.307 [9.461, 11.206] | asset bootstrap; 7421/720 renderable-geometry URDF links |
| Parts cluster sensitivity | [6.183, 15.175] | factory then observed-seed bootstrap |
| Named / Nameability | 1.000 | 7421/7421 |
| Opaque `l_<index>` names | 1.000 | supplementary audit; lexical Nameability is not semantic readability |
| Raw unique-name set Jaccard | 0.827 pair-micro / 0.827 factory-macro / 0.848 median [0.760, 0.890] | within factory |
| Raw name-multiset weighted Jaccard | 0.827 pair-micro / 0.827 factory-macro / 0.848 median [0.760, 0.891] | within factory |
| Exact raw-name-multiset mode rate | 0.507 factory-macro / 0.472 median [0.371, 0.647] | modal signature frequency / PASS seeds, then factory mean |
| Semantic Precision / Recall | N/A | no independent semantic gold or judges |
| Naming Richness | N/A | no independent semantic role inventory |
| Functional / Instance / Over-Segmentation | N/A | no independent functional, instance, or decomposition gold |

## Coverage and audit

- Original 300-second reliability: 713/720 PASS; Naming-evaluable after recovery overlay: 720/720; recovered: 7.
- Factories: 20; within-factory raw-name pairs: 12600.
- URDF part nodes: 7421; renderable visuals: 7421 (mesh 7421, primitives 0); multi-visual links: 0.
- Invalid or unsupported visual geometries excluded: 0.
- Bootstrap: 10000 deterministic resamples, seed 260811002.
