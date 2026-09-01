# Table 6 PartNet-Mobility real-data reference audit

Status: **COMPLETE**

PartNet-Mobility is a curated source/reference dataset, not a generated method. It must be reported outside generated-method rankings.

## Provenance and inventory

- Local v0 packages: 2347 assets / 46 exact `meta.model_cat` categories.
- Companion archive: `b47247a44246111e8d09f2c0e64b4012ae35e0dcf4bb55f68a05b604455119ff` (3268124298 bytes; 381018 ZIP entries).
- Audited schema paths with matched direct-root/archive presence: 16429/16429; presence/hash mismatches: 0; present files byte-compared: 15335.
- Official repository pin: `ee0aa3ef1df16181d76d83f7415aa8c94ed1da8f`; gated=manual; license='other'.
- Status remains `PROVENANCE_LIMITED`: local bytes match the frozen companion archive, but were not directly authenticated against gated per-ID objects at the pinned revision.

## Full-release annotation and URDF audit

- Six mandatory core files present: 2347/2347; optional `result_original.json`: 1253/2347 (absent from both direct root and companion archive for the remaining packages).
- Annotation parse: 2347/2347; URDF parse: 2347/2347; valid URDF trees: 2347/2347.
- Logical mobility annotations: 11753 (5.008/asset); axis origin/direction/range finite-field coverage: 11753/11753 / 11731/11753 / 11753/11753.
- URDF movable DoFs: 11971 (5.101/asset); motion-sweepable: 11970; bounded zero-width: 1.
- URDF finite-field coverage: endpoints 11971/11971; axes 11949/11971; origins 11971/11971; bounded limits 9618/9618.
- Collision packages: 2347/2347; missing collision mesh references: 120; valid inertial links: 0.
- Composite representation: 218 logical `slider+` annotations explain the DoF expansion from 11753 logical motions to 11971 URDF DoFs. URDF DoF count is not joint recall.

## PhysX same-ID boundary

- Shared PartNet/PhysX-finaljson IDs: 2024; PartNet-only IDs: 323; PhysX IDs missing from PartNet: 0.
- Shared subset logical/URDF joints: 9916/9992; PartNet-only: 1837/1979.
- The shared annotations are valid source-reference fields for same-ID representation preservation (type/parent/axis/origin/limit), not independent blind-test gold: PhysX-Mobility is derived from PartNet-Mobility. The 323 PartNet-only assets have no local PhysX finaljson pair.

## Frozen category functional proxy

- Cohort: 46 assets / 46 categories; load 46/46; reset/readback 46/46; complete 46/46.
- Declared/motion/zero-width DoFs: 286/286/0.
- Penetration-free states at 1e-6 m: 2190/4554; strict contact-free: 2169/4554.
- Joint single-sweep penetration-free: 128/286; asset proxy: 33/46.
- Protocol: 11 states per nonzero single DoF plus 64 unscrambled Sobol states for assets with more than one motion DoF; per-asset subprocess; self-collision plus exclude-parent flags; motors disabled; exact reset/readback and `performCollisionDetection`; no simulation step and no CCD.

## Table 6 boundary

The release provides reference type, parent graph, axis origin/direction, limits, part membership, names, collision meshes, and standard URDF exports. For the reference itself, Joint Type Accuracy, Joint Recall, Parent-Child Accuracy, Axis/Origin/Limit semantic accuracy, Joint Geom. Valid, and Asset Geom. Valid remain N/A. Annotation and URDF belong to the same release pipeline, metadata presence is not accuracy, and the discrete collision result is not continuous collision or physical correctness.

License boundary: the official card is gated and limits use to non-commercial research/education; ShapeNet terms also apply. Do not redistribute raw assets in supplementary material unless recipients have accepted the terms.
