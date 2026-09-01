# Nano3D Articulation paper-aligned evaluation

- assets: 33; movable joints: 186; mean joints/asset: 5.636
- declared movable joints: 186; functional-motion joints: 186; zero-width bounded excluded: 0
- single-joint penetration-free: 727/2046; contact-free: 725/2046
- multi-joint penetration-free: 514/1536; contact-free: 509/1536
- full penetration-free proxy: 1241/3582; contact-free: 1234/3582
- per-joint single-sweep penetration-free: 62/186; contact-free: 62/186
- asset full-range penetration-free proxy: 10/33; contact-free: 10/33

Semantic type/recall/parent-child/axis-on-moving-part and rest-pose-frozen remain N/A because the local cohort has no frozen joint gold or pre-articulation pair.

All collision results disable motors, reset the requested pose, and call performCollisionDetection with URDF_USE_SELF_COLLISION | URDF_USE_SELF_COLLISION_EXCLUDE_PARENT; no simulation step is taken. Penetration-free means no depth above 1e-06 m; contact-free means zero contact points. These are not CCD or full physical-validity claims.
