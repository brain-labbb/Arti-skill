# Table 5: articulated-asset simulation readiness

Run classification: **COMPLETE**.

The 10 s simulation-validity test is not physical settling. Each asset is reset three times to its URDF neutral state (zero clamped to bounded ranges), then simulated for 10 s at 240 Hz with gravity, contacts, self-collision, a fixed base, manifest-bound physics, and zero applied joint force. Passing requires accurate finite resets, finite mapped states and observed poses for all 2,400 steps, and unchanged mapping in all repetitions. Table 5a reports the Genesis single-simulator evaluation. Table 5b reports exact Genesis/PyBullet/MuJoCo intersections for import, canonical DoF mapping, and 10 s numerical stability; these values are not averages of per-simulator rates. Joint speed and 0.5% limit compliance are retained only as strict sensitivity diagnostics because they depend on authored damping, armature, collision filtering, and functional multi-joint dependencies.

Complete Non-placeholder Inertials requires every non-root dynamic link to have a positive finite mass, finite center of mass, and positive-definite inertia satisfying the rigid-body triangle inequality. An asset is excluded when every required dynamic link uses the exact unit placeholder `mass=1, inertia=I` (and zero COM when published); simulator-generated defaults do not count. PV-A overlay plans are hash-checked and audited with the same rule. The unfiltered mathematical-validity rate and placeholder incidence remain in the supplementary report.

Complete Genesis Readiness is the exact per-asset intersection of native import, complete declared DoF mapping, complete non-placeholder inertials, and the three-repeat 10 s Genesis validity gate.

## Table 5a: Genesis single-simulator evaluation

| Dataset | N | Import (%) ↑ | DoF Mapping (%) ↑ | Complete Non-placeholder Inertials (%) ↑ | Genesis 10 s Simulation Validity (%) ↑ | Trajectory Coverage (%) ↑ | Complete Genesis Readiness (%) ↑ |
|---|---:|---:|---:|---:|---:|---:|---:|
| Articraft-10K | 200 | 98.00 | 99.49 | 71.00 | 97.50 | **100.00** | 71.00 |
| LAM released outputs | 200 | 87.50 | 98.93 | 0.00 | 87.00 | 95.78 | 0.00 |
| Artiverse | 200 | 99.50 | 99.21 | **100.00** | 96.00 | 95.50 | 96.00 |
| PartNet-Mobility | 200 | 95.00 | 92.02 | 0.00 | 95.00 | 0.00 | 0.00 |
| PhysX-Mobility | 200 | **100.00** | **100.00** | 0.00 | 99.00 | **100.00** | 0.00 |
| SketchMobility | 200 | 99.50 | 99.02 | 40.00 | **99.50** | 80.14 | 40.00 |
| Infinigen-Sim | 200 | 93.00 | 90.28 | 0.00 | 93.00 | 0.00 | 0.00 |
| Ours (PV-A) | 200 | **100.00** | **100.00** | **100.00** | **99.50** | 99.36 | **99.50** |

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
