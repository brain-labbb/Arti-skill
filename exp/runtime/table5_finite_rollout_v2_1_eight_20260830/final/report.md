# Table 5: articulated-asset simulation readiness

Run classification: **COMPLETE**.

The primary rollout is deliberately named **10 s Finite Rollout**, not physical settling. Each asset is reset three times to its URDF neutral state (zero clamped to bounded ranges), then simulated for 10 s at 240 Hz with gravity, contacts, self-collision, a fixed base, manifest-bound physics, and zero applied joint force. Passing requires accurate finite resets, finite mapped states and observed poses for all 2,400 steps, and unchanged mapping in all repetitions. The main-table rate is the per-asset intersection across Genesis, PyBullet, and MuJoCo; per-simulator rates are reported separately. Joint speed and 0.5% limit compliance are retained only as strict sensitivity diagnostics because they depend on authored damping, armature, collision filtering, and functional multi-joint dependencies.

Complete Non-placeholder Inertials requires every non-root dynamic link to have a positive finite mass, finite center of mass, and positive-definite inertia satisfying the rigid-body triangle inequality. An asset is excluded when every required dynamic link uses the exact unit placeholder `mass=1, inertia=I` (and zero COM when published); simulator-generated defaults do not count. PV-A overlay plans are hash-checked and audited with the same rule. The unfiltered mathematical-validity rate and placeholder incidence remain in the supplementary report.

## Main results (Genesis)

| Dataset | N | Import (%) ↑ | DoF Mapping (%) ↑ | Complete Non-placeholder Inertials (%) ↑ | 3-Sim 10 s Finite Rollout (%) ↑ | Trajectory Coverage (%) ↑ |
|---|---:|---:|---:|---:|---:|---:|
| Articraft-10K | 200 | 98.00 | 99.49 | 71.00 | 91.50 | **100.00** |
| LAM released outputs | 200 | 87.50 | 98.93 | 0.00 | 41.00 | 95.78 |
| Artiverse | 200 | 99.50 | 99.21 | **100.00** | 81.50 | 95.50 |
| PartNet-Mobility | 200 | 95.00 | 92.02 | 0.00 | 42.00 | 0.00 |
| PhysX-Mobility | 200 | **100.00** | **100.00** | 0.00 | **99.00** | **100.00** |
| SketchMobility | 200 | 99.50 | 99.02 | 40.00 | 42.00 | 80.14 |
| Infinigen-Sim | 200 | 93.00 | 90.28 | 0.00 | 93.00 | 0.00 |
| Ours (PV-A) | 200 | **100.00** | **100.00** | **100.00** | **99.00** | 99.36 |

## Cross-simulator finite rollout

| Dataset | Genesis (%) | PyBullet (%) | MuJoCo (%) | All three (%) |
|---|---:|---:|---:|---:|
| Articraft-10K | 97.50 | 98.00 | 92.00 | 91.50 |
| LAM released outputs | 87.00 | 88.00 | 42.00 | 41.00 |
| Artiverse | 96.00 | 100.00 | 84.00 | 81.50 |
| PartNet-Mobility | 95.00 | 98.50 | 42.00 | 42.00 |
| PhysX-Mobility | 99.00 | 99.50 | 99.50 | 99.00 |
| SketchMobility | 99.50 | 55.00 | 85.50 | 42.00 |
| Infinigen-Sim | 93.00 | 100.00 | 99.50 | 93.00 |
| Ours (PV-A) | 99.50 | 100.00 | 99.50 | 99.00 |

## Method basis

- NVIDIA PhysX treats 200-300 deg/s as a tuning recommendation and separately emphasizes timestep, mass/inertia ratios, armature, damping, and non-adjacent self-collision: https://nvidia-omniverse.github.io/PhysX/ovphysx/latest/guides/articulation_stability.html
- MuJoCo models joint limits, damping, friction loss, and armature as separate authored properties: https://mujoco.readthedocs.io/en/3.1.3/XMLreference.html
- Articraft's simulation guide reports standing stability, penetration, separation, residual velocity, and released-joint peak speed as distinct diagnostics: https://github.com/articraftresearch/Articraft/blob/main/docs/simulation.md
