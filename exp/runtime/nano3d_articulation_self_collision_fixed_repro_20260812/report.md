# Nano3D Articulation paper-aligned evaluation

- assets: 33; movable joints: 186; mean joints/asset: 5.636
- single-joint sweep: 725/2046 states collision-free
- multi-joint Sobol sweep: 509/1536 states collision-free
- full functional proxy: 1234/3582 states collision-free
- per-joint single-sweep pass: 62/186
- asset full-range collision-free proxy: 10/33

Semantic type/recall/parent-child/axis-on-moving-part and rest-pose-frozen remain N/A because the local cohort has no frozen joint gold or pre-articulation pair.

All collision results use PyBullet discrete stepping with URDF_USE_SELF_COLLISION | URDF_USE_SELF_COLLISION_EXCLUDE_PARENT; they are not a CCD or full physical-validity claim.
