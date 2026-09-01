# Artiverse Table 6 release reference

Status: STATIC_COMPLETE; MOTION_INTENT_FROZEN_NOT_EXECUTED

The frozen release contains 3544 assets across 84 categories and 10 sources.

## Distinct denominators

- raw annotation records: 16471
- semantic joints (unique motion-bearing pid): 16355
- semantic scalar DoFs: 16437
- exported movable URDF elements: 16332

## Static release audit

- core-complete packages: 3543/3544
- parseable annotation JSON: 3544/3544
- parseable URDF: 3543/3544
- complete semantic-to-export mapping: 3513/3544
- resolved native collision mesh references: 3543/3544
- heuristic mass sidecar present: 0/3544

## Frozen motion intent

The content-hashed motion cohort has 100 assets from 84 eligible categories. It was selected before any load, contact, or FCL result; failures are never replaced.

## Claim boundary

Artiverse is a real-data release reference, not a generated method. Annotation-to-URDF agreement is export self-consistency rather than independent semantic accuracy. Joint type accuracy, recall, parent-child accuracy, and semantic axis/origin/limit accuracy remain N/A without independent gold. Motion and collision have not yet been run.
