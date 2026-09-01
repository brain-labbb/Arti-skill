# Articraft Naming Baseline v1.1

Status: **COMPLETE**

## Frozen cohort

- Selection protocol: `nano3d_table2_baseline_naming_v1` (`962946bb309d77c8cee746e166762246e291cf9aa4491337bc4846ef978f5c94`).
- Pre-expansion frozen manifest SHA-256: `12cfac6822240f895ba90b68e7e275c136b6cecb85e59d40f2594f1fb403a3bf`. The exact 123-record base is retained; 119 newly hydrated categories are added.
- Eligible pool: 10066 hydrated retained records across 242 categories.
- Requested cohort: 242 records.
- Representation/unit: native Articraft URDF; one link with at least one valid renderable visual geometry.
- Shared protocol: `nano3d_table2_baseline_naming_v1.1` (`4e9d90300018ea47dd0473fc09cb866656b42cdc749377e5a46fb0a9c699cb1e`).
- Cohort manifest SHA-256: `7a49511679075c4f440b5cf35a7addac245c2a0ffdaca48a4a9618bb7c491fb4`.

## Direct results

- Artifact coverage: 242/242 = 1.000000.
- Status counts: `{"PASS": 242}`.
- Parts: 5.438 renderable-visual URDF links/asset; median 4.000; 95% bootstrap CI [4.843, 6.145]; total 1316.
- Named / Nameability: 1305/1316 = 0.991641.
- Protocol-placeholder audit: 11/1316 across 3 assets; names `{"link1": 2, "link2": 2, "link3": 2, "link4": 2, "link5": 1, "link_1": 1, "link_2": 1}`. Non-placeholder does not certify semantic correctness.
- Geometry link composition: mesh-only 166; primitive-only 766; mixed 384; invalid visual geometries 0.

## Fail-closed fields

Semantic Precision, Semantic Recall, Naming Richness, Functional Core Coverage,
Instance Discriminability, and Over-Segmentation Rate are **N/A**. This cohort has
no frozen output-independent role gold and no completed three-judge blind verdicts.
Readable source/link names are not used to certify semantic correctness.

Cross-Seed Consistency is **N/A** because Articraft is evaluated as a per-asset
method and the audited records expose no official frozen reusable seed interface.

## Scope

This is a local retained-category supplementary cohort, not the common hidden
prompt-matched authoring comparison. Visual compile was used only to produce
naming-evaluable native URDF packages; it is not full physical QC. URDF-link
counts are not relabeled as GLB-node counts without a converter-preservation audit.
