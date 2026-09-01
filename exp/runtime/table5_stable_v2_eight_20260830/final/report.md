# Table 5 primary results

Run classification: **COMPLETE**.

Stable v2 uses gravity [0, 0, -9.81], contacts, self-collision, a fixed base, and each asset's manifest-bound physics. Each asset is reset to 25%, 50%, and 75% of every mapped bounded hinge/slide range. Each trial runs the full 10 s at 240 Hz under zero applied joint force. Passing requires finite states and poses, unchanged mapping, <=0.5% normalized limit violation, revolute speed <=300 deg/s, and prismatic speed <=5 m/s for all three trials.

## Genesis simulation readiness

| Dataset | N | Import (%) | DoF Mapping (%) | Stable v2 (%) | Actuated Trajectory Coverage (%) | Drift Pos P95 (% diag.) | Drift Rot P95 (deg) |
|---|---:|---:|---:|---:|---:|---:|---:|
| Articraft-10K | 200 | 98.00 | 99.49 | 26.00 | 100.00 | 0.0000 (415/415) | 0.0001 (415/415) |
| LAM released outputs | 200 | 87.50 | 98.93 | 50.50 | 95.78 | 0.0000 (363/378) | 0.0001 (363/378) |
| Artiverse | 200 | 99.50 | 99.21 | 79.50 | 95.50 | 0.0000 (573/600) | 0.0000 (573/600) |
| PartNet-Mobility | 200 | 95.00 | 92.02 | 36.50 | 0.00 | 0.0000 (345/395) | 0.0000 (345/395) |
| PhysX-Mobility | 200 | 100.00 | 100.00 | 68.50 | 100.00 | 0.0000 (479/479) | 0.0000 (479/479) |
| SketchMobility | 200 | 99.50 | 99.02 | 49.50 | 80.14 | 0.0000 (288/292) | 0.0001 (288/292) |
| Infinigen-Sim | 200 | 93.00 | 90.28 | 71.50 | 0.00 | 0.0000 (734/813) | 0.0000 (734/813) |
| Ours (PV-A) | 200 | 100.00 | 100.00 | 34.00 | 99.36 | 0.0000 (628/628) | 0.0001 (628/628) |

## Cross-simulator Stable v2

| Dataset | Genesis (%) | PyBullet (%) | MuJoCo (%) | All three (%) |
|---|---:|---:|---:|---:|
| Articraft-10K | 26.00 | 34.50 | 25.00 | 15.50 |
| LAM released outputs | 50.50 | 50.00 | 4.00 | 2.00 |
| Artiverse | 79.50 | 77.00 | 10.50 | 8.00 |
| PartNet-Mobility | 36.50 | 51.50 | 9.50 | 5.00 |
| PhysX-Mobility | 68.50 | 74.00 | 44.00 | 44.00 |
| SketchMobility | 49.50 | 25.50 | 29.00 | 4.00 |
| Infinigen-Sim | 71.50 | 39.50 | 69.00 | 29.00 |
| Ours (PV-A) | 34.00 | 29.00 | 29.50 | 17.00 |

## Long-horizon constraint drift

Each cell is `P95 (evaluated/candidate joints)`. Translation is normalized by object bounding-box diagonal; rotation is quaternion geodesic error.

| Dataset | Genesis Pos (% diag.) | Genesis Rot (deg) | PyBullet Pos (% diag.) | PyBullet Rot (deg) | MuJoCo Pos (% diag.) | MuJoCo Rot (deg) |
|---|---:|---:|---:|---:|---:|---:|
| Articraft-10K | 0.0000 (415/415) | 0.0001 (415/415) | 0.0000 (415/415) | 0.0000 (415/415) | 1.3427 (387/415) | 5.7229 (387/415) |
| LAM released outputs | 0.0000 (363/378) | 0.0001 (363/378) | 0.0000 (375/378) | 0.0000 (376/378) | 1.7595 (200/378) | 7.5209 (200/378) |
| Artiverse | 0.0000 (573/600) | 0.0000 (573/600) | 0.0000 (600/600) | 0.0000 (600/600) | 2.3108 (508/600) | 13.3420 (508/600) |
| PartNet-Mobility | 0.0000 (345/395) | 0.0000 (345/395) | 0.0000 (392/395) | 0.0000 (392/395) | 0.5980 (96/395) | 5.7953 (96/395) |
| PhysX-Mobility | 0.0000 (479/479) | 0.0000 (479/479) | 0.0000 (479/479) | 0.0000 (479/479) | 0.5983 (479/479) | 0.8739 (479/479) |
| SketchMobility | 0.0000 (288/292) | 0.0001 (288/292) | 0.0000 (174/292) | 0.0000 (174/292) | 1.7611 (205/292) | 6.5772 (205/292) |
| Infinigen-Sim | 0.0000 (734/813) | 0.0000 (734/813) | 0.0000 (813/813) | 0.0000 (813/813) | 36.6354 (809/813) | 2.0702 (809/813) |
| Ours (PV-A) | 0.0000 (628/628) | 0.0001 (628/628) | 0.0000 (628/628) | 0.0000 (628/628) | 0.9138 (626/628) | 8.1949 (626/628) |
