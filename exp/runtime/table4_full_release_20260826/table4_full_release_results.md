# Table 4 Full-Release Results

Collision and mechanical-clearance diagnostics for eight full-release comparison cohorts; the measured or blocked status is shown per row. Ours rows are copied unchanged from the protocol source.

- Combined receipt: `/mnt/zsn/lyb/arti-skill/exp/runtime/table4_full_release_20260826/full_release_receipt.json`
- Source protocol: [URDF-Sim-Ready-Automatic-Evaluation.md](../../URDF-Sim-Ready-Automatic-Evaluation.md)
- This renderer is read-only and does not run an evaluator.

## Table 4. Collision and Mechanical Clearance

| Dataset / Outputs | N_eval | J_eval | Rest All-pair CF | Rest Non-adjacent CF | Single-joint Sweep CF | Multi-joint Sobol CF | Collision-state Rate | AOR | Max Penetration | Collision-free Range | Strict Collision Pass | Status | Evidence |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---|---|
| Ours-500K | 500 | 2,467 | 7 / 500 (1.400%) | 487 / 500 (97.400%) | 487 / 500 (97.400%) | 485 / 500 (97.000%) | 2,447 / 84,307 (2.902%) | N/E | 0.297148 (500 / 500 measured; COMPLETE) | 49,577 / 51,807 (95.696%) | 485 / 500 (97.000%) | source (unchanged) | protocol source |
| Articraft-10K | 9,996 | 37,144 | 169 / 9,996 (1.691%) | 2,532 / 9,996 (25.330%) | 2,065 / 9,996 (20.658%) | 2,018 / 9,996 (20.188%) | 1,065,315 / 1,429,700 (74.513%) | N/E | 0.667632 (3,058 / 9,996 measured; PARTIAL) | 204,039 / 780,024 (26.158%) | 1,988 / 9,996 (19.888%) | COMPLETE_WITH_RETAINED_FAILURES | [`summary`](articraft/summary.json) |
| LAM released outputs | 3,217 | 10,381 | 9 / 3,217 (0.280%) | 477 / 3,217 (14.827%) | 404 / 3,217 (12.558%) | 384 / 3,217 (11.937%) | 365,147 / 421,026 (86.728%) | N/E | 0.896712 (1,320 / 3,217 measured; PARTIAL) | 23,640 / 218,001 (10.844%) | 378 / 3,217 (11.750%) | COMPLETE_WITH_RETAINED_FAILURES | [`summary`](lam/summary.json) |
| Artiverse | 3,544 | 16,332 | 48 / 3,544 (1.354%) | 1,444 / 3,544 (40.745%) | 1,251 / 3,544 (35.299%) | 1,223 / 3,544 (34.509%) | 326,510 / 573,140 (56.969%) | N/E | 0.689776 (3,515 / 3,544 measured; PARTIAL) | 96,580 / 342,972 (28.160%) | 1,118 / 3,544 (31.546%) | COMPLETE_WITH_RETAINED_FAILURES | [`summary`](artiverse/summary.json) |
| PartNet-Mobility | 2,347 | 11,971 | 70 / 2,347 (2.983%) | 1,819 / 2,347 (77.503%) | 1,727 / 2,347 (73.583%) | 1,705 / 2,347 (72.646%) | 140,930 / 403,946 (34.888%) | N/E | 0.646960 (2,313 / 2,347 measured; PARTIAL) | 141,022 / 251,391 (56.097%) | 1,665 / 2,347 (70.942%) | COMPLETE_WITH_RETAINED_FAILURES | [`summary`](partnet/summary.json) |
| PhysX-Mobility | 2,024 | 9,883 | N/E | N/E | N/E | N/E | N/E | N/E | N/E | N/E | N/E | BLOCKED | [`summary`](physx/summary.json) |
| SketchMobility | 4,956 | 11,009 | 33 / 4,956 (0.666%) | 2,184 / 4,956 (44.068%) | 1,978 / 4,956 (39.911%) | 2,001 / 4,956 (40.375%) | 292,617 / 553,329 (52.883%) | N/E | 0.539062 (2,777 / 4,956 measured; PARTIAL) | 113,684 / 231,189 (49.174%) | 1,882 / 4,956 (37.974%) | COMPLETE_WITH_RETAINED_FAILURES | [`summary`](sketch/summary.json) |
| Infinite Mobility | 720 | 4,723 | N/E | N/E | N/E | N/E | N/E | N/E | N/E | N/E | N/E | BLOCKED | [`summary`](infinite/summary.json) |
| Infinigen-Sim | 8,226 | 31,975 | 94 / 8,226 (1.143%) | 6,836 / 8,226 (83.102%) | 5,045 / 8,226 (61.330%) | 5,100 / 8,226 (61.999%) | 298,941 / 1,200,149 (24.909%) | N/E | 0.809588 (8,226 / 8,226 measured; COMPLETE) | 476,993 / 671,475 (71.037%) | 4,746 / 8,226 (57.695%) | COMPLETE | [`summary`](infinigen/summary.json) |

N/E denotes not estimable under the frozen protocol. In particular, a release with no native collision geometry is blocked for collision-dependent metrics; an empty contact query is never treated as a collision-free pass.

The frozen states are rest q=0, K=21 single-joint sweeps, and R=64 Sobol multi-joint configurations (seed 20260813). Pair policy reports all-pair and non-adjacent results and treats penetration strictly greater than 1e-6 m as illegal.

Generated from the combined receipt; per-dataset records, checkpoints, manifests, and artifact hashes remain in the linked evidence directories.
