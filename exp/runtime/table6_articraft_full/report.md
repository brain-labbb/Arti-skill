# Original Articraft Table 6 preflight and preliminary pilot

- Formal status: **BLOCKED_COMMON_TABLE6**.
- Local pilot: **COMPLETED_WITH_STRICT_FAILURES**; 12-category, 143-record frozen Original Articraft single-instance cohort; not the 54-item common Table 6 benchmark.
- Strict compile: 126/143.
- Exportable URDF after strict attempt: 143/143 (recovered strict-failure exports remain failures).
- Articulable: 143/143.
- Movable joints: 442; joints/attempted asset: 3.090909090909091.
- Declared movable joints: 442; bounded zero-width: 0/442 across 0/143; motion-sweep eligible: 442/442 joints and 143/143 assets.
- Rest states contact-free: 117/143; penetration-free at <= 1e-06 m: 118/143.
- Single-joint states contact-free: 3512/4862; penetration-free: 3576/4862.
- Multi-joint Sobol states contact-free: 5720/7488; penetration-free: 5786/7488.
- Combined swept states contact-free: 9232/12350; penetration-free: 9362/12350.
- Joint single sweeps contact-free: 286/442; penetration-free: 292/442.
- Asset swept proxy contact-free: 82/143; penetration-free: 84/143.

Collision policy: `PyBullet DIRECT, URDF_USE_SELF_COLLISION | URDF_USE_SELF_COLLISION_EXCLUDE_PARENT`. Pose evaluation: `disable movable-joint motors; resetJointState; performCollisionDetection; no stepSimulation`. Parent-child pairs are excluded; non-parent self-collision is enabled. Contact-free means zero returned contacts. Penetration-free permits contact only when maximum penetration depth is <= 1e-06 m. Continuous collision detection was not run.

The collision and metadata fields are operational proxies only. Semantic Table 6 accuracy/validity fields remain N/A, and this pilot must not replace the missing common 54-item benchmark.
