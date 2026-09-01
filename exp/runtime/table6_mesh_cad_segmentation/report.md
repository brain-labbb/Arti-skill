# Table 6 mesh-native / CAD / segmentation preflight

## Verdict

**PAPER_ONLY / N_A.** `mesh-native / CAD / segmentation (paper)` is an aggregate representation-control row, not one runnable method or one shared experimental cohort. It groups non-articulated output families whose native deliverables do not expose an articulated joint model.

The Table 6 entries `Articulable = no`, `Joints/Asset = 0`, and `Native Joint Exposure = 0` describe that representation boundary. Joint type, recall, parent-child, axis, origin, limit, geometry, collision-range, and generic-range metrics are therefore **N/A**, not zero-scored failures.

## Local evidence boundary

No local common articulated output can be attributed to this aggregate paper control. The available local articulated assets and motion checks belong to the PV-A existing-export cohorts documented separately in Table 6. Reusing those outputs for mesh-native, CAD, or segmentation papers would break method provenance and would not constitute a baseline rerun.

No execution was attempted for this row. Its status is `NOT_RUN_NOT_APPLICABLE`, not `FAILED`.

## What would be required for a real result

A future experiment must first split the aggregate into named paper methods, freeze a shared item manifest, and preserve each method's native output. If a post-hoc articulation adapter is introduced, adapter-derived joints must be reported in a separate row and must not be described as native joint exposure. Only attributable outputs with joint gold and the common Table 6 evaluator can populate the accuracy and motion columns.
