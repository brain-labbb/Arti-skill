# Table 6 Artiverse pre-release reference audit

Status: **COMPLETE**

The manifest-defined scope (all) contains 3544 assets across 84 categories and 10 upstream sources. Selection used every listed root and did not depend on outcomes.

## Static release audit

- core annotation + segmented GLB + collider URDF: 3544/3544
- human-verified reference annotations: 16471 logical articulations (4.648/asset)
- simulator export: 16332 movable URDF DoFs (4.608/asset)
- parseable collider URDF: 3543/3544
- annotation axis metadata: 16429/16471; range metadata: 15714/16471
- valid URDF trees: 3526/3544; cycle assets: 17
- collision geometry: 3543/3544 assets; missing mesh references: 0
- material metadata: 3544/3544; README-declared mass_furniture_heuristic.json observed: 0/3544

## Functional proxy

- load: 84/84; reset/readback: 84/84
- complete measurements: 84/84
- declared/motion/zero-range DoFs: 538/526/12
- contact-free states (executed): 2959/9498
- penetration-free states (executed): 3587/9498
- per-joint penetration-free single sweep: 122/526
- per-asset penetration-free proxy: 33/84

## Claim boundary

Artiverse is a manually corrected and expert-verified dataset reference, not a prediction method. Its annotation JSON and URDF belong to the same release pipeline, so their agreement is export fidelity rather than independent joint accuracy. Complex joint annotations can be lossy in standard URDF. All PyBullet results are discrete resetJointState + performCollisionDetection proxies with motors disabled and direct parent-child collisions excluded; no dynamics step, CCD, or semantic axis-on-moving-part judgment was run.

This is a gated, license=other, pre-release snapshot whose README says cleanup is ongoing.
