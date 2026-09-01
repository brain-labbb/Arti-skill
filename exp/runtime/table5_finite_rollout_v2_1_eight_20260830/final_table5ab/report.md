# Table 5: articulated-asset simulation readiness

Run classification: **COMPLETE**.

The 10 s simulation-validity test is not physical settling. Each asset is reset three times to its URDF neutral state (zero clamped to bounded ranges), then simulated for 10 s at 240 Hz with gravity, contacts, self-collision, a fixed base, manifest-bound physics, and zero applied joint force. Passing requires accurate finite resets, finite mapped states and observed poses for all 2,400 steps, and unchanged mapping in all repetitions. Table 5a-I reports this Genesis single-simulator readiness evaluation. Table 5a-II separately evaluates free-root center-of-mass stability with all articulated joints locked at canonical q=0. Table 5b reports exact Genesis/PyBullet/MuJoCo intersections for import, canonical DoF mapping, and 10 s numerical stability; these values are not averages of per-simulator rates. Joint speed and 0.5% limit compliance are retained only as strict sensitivity diagnostics because they depend on authored damping, armature, collision filtering, and functional multi-joint dependencies.

Complete Non-placeholder Inertials requires every non-root dynamic link to have a positive finite mass, finite center of mass, and positive-definite inertia satisfying the rigid-body triangle inequality. An asset is excluded when every required dynamic link uses the exact unit placeholder `mass=1, inertia=I` (and zero COM when published); simulator-generated defaults do not count. PV-A overlay plans are hash-checked and audited with the same rule. The unfiltered mathematical-validity rate and placeholder incidence remain in the supplementary report.

## Table 5a-I: Genesis single-simulator readiness

| Dataset | N | Import (%) ↑ | DoF Mapping (%) ↑ | Complete Non-placeholder Inertials (%) ↑ | Genesis 10 s Simulation Validity (%) ↑ | Trajectory Coverage (%) ↑ | Drift Pos P95 (% diag.) ↓ | Drift Rot P95 (deg) ↓ |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| Articraft-10K | 200 | 98.00 | 99.49 | 71.00 | 97.50 | **100.00** | 2.42e-05 (415/415) | 5.21e-05 (415/415) |
| LAM released outputs | 200 | 87.50 | 98.93 | 0.00 | 87.00 | 95.78 | 2.06e-05 (363/378) | 5.26e-05 (363/378) |
| Artiverse | 200 | 99.50 | 99.21 | **100.00** | 96.00 | 95.50 | 1.09e-05 (573/600) | 3.32e-05 (573/600) |
| PartNet-Mobility | 200 | 95.00 | 92.02 | 0.00 | 95.00 | 0.00 | 1.76e-06 (349/395) | 3.44e-05 (349/395) |
| PhysX-Mobility | 200 | **100.00** | **100.00** | 0.00 | 99.00 | **100.00** | 1.22e-05 (473/479) | 3.48e-05 (473/479) |
| SketchMobility | 200 | 99.50 | 99.02 | 40.00 | **99.50** | 80.14 | 2.06e-05 (288/292) | 4.43e-05 (288/292) |
| Infinigen-Sim | 200 | 93.00 | 90.28 | 0.00 | 93.00 | 0.00 | 1.81e-05 (734/813) | 3.70e-05 (734/813) |
| Ours (PV-A) | 200 | **100.00** | **100.00** | **100.00** | **99.50** | 99.36 | 1.31e-05 (624/628) | 6.53e-05 (624/628) |

## Table 5a-II: Genesis free-root center-of-mass stability

This panel uses only assets eligible for free-standing support evaluation, so `Eligible N` differs by dataset. The root is free, all movable joints are locked at canonical q=0, gravity is enabled, and each asset is simulated for 10 s. Physics Coverage requires dataset-provided valid mass and center-of-mass parameters. CoM Margin Coverage additionally requires a non-degenerate ground-contact support polygon. Missing attributable physics fails closed for CoM Static Stability. The runtime uses robust collision fallback and recomputed inertias for diagnostic execution; normalized margin magnitude is omitted from the main table.

| Dataset | Eligible N | Physics Coverage (%) ↑ | CoM Margin Coverage (%) ↑ | CoM Static Stability (%) ↑ |
|---|---:|---:|---:|---:|
| Articraft-10K | 159 | 71.70 | 69.81 | 63.52 |
| LAM released outputs | 75 | 5.33 | 5.33 | 4.00 |
| Artiverse | 160 | **100.00** | **99.38** | **91.88** |
| PartNet-Mobility | 158 | 0.00 | 0.00 | 0.00 |
| PhysX-Mobility | 130 | **100.00** | 99.23 | 84.62 |
| SketchMobility | 163 | 41.10 | 38.65 | 34.36 |
| Infinigen-Sim | 170 | 0.00 | 0.00 | 0.00 |
| Ours (PV-A) | 156 | **100.00** | 99.36 | 89.10 |

## Table 5b: Cross-simulator evaluation

`All-3` is the exact per-asset or per-joint intersection across Genesis, PyBullet, and MuJoCo. Stable denotes the same 10 s numerical-stability gate used in Table 5a, not physical settling.

| Dataset | All-3 Import (%) ↑ | All-3 DoF Mapping (%) ↑ | All-3 Stable (%) ↑ |
|---|---:|---:|---:|
| Articraft-10K | 91.50 | 94.60 | 91.50 |
| LAM released outputs | 41.50 | 54.39 | 41.00 |
| Artiverse | 85.00 | 85.51 | 81.50 |
| PartNet-Mobility | 42.00 | 25.82 | 42.00 |
| PhysX-Mobility | 99.00 | 98.75 | **99.00** |
| SketchMobility | 42.00 | 43.66 | 42.00 |
| Infinigen-Sim | 93.00 | 90.28 | 93.00 |
| Ours (PV-A) | **99.50** | **99.76** | **99.00** |

## Method basis

- NVIDIA PhysX treats 200-300 deg/s as a tuning recommendation and separately emphasizes timestep, mass/inertia ratios, armature, damping, and non-adjacent self-collision: https://nvidia-omniverse.github.io/PhysX/ovphysx/latest/guides/articulation_stability.html
- MuJoCo models joint limits, damping, friction loss, and armature as separate authored properties: https://mujoco.readthedocs.io/en/3.1.3/XMLreference.html
- Articraft's simulation guide reports standing stability, penetration, separation, residual velocity, and released-joint peak speed as distinct diagnostics: https://github.com/articraftresearch/Articraft/blob/main/docs/simulation.md
