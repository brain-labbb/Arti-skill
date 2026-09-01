# Table 5 supplementary diagnostics

These diagnostics do not participate in the primary ranking.

## Inertial audit detail

The mathematical-validity column accepts any complete positive-definite rigid-body inertia. The exact-unit-placeholder column counts assets for which every required dynamic link is exactly `mass=1, inertia=I` (and zero COM when published). Hash-bound PV-A overlay plans are parsed with the same rule.

| Dataset | Mathematically Valid (%) | Exact Unit Placeholder (%) | Complete Non-placeholder (%) |
|---|---:|---:|---:|
| Articraft-10K | 71.00 | 0.00 | 71.00 |
| LAM released outputs | 3.00 | 3.00 | 0.00 |
| Artiverse | 100.00 | 0.00 | 100.00 |
| PartNet-Mobility | 0.00 | 0.00 | 0.00 |
| PhysX-Mobility | 99.50 | 99.50 | 0.00 |
| SketchMobility | 40.00 | 0.00 | 40.00 |
| Infinigen-Sim | 0.00 | 0.00 | 0.00 |
| Ours (PV-A) | 100.00 | 0.00 | 100.00 |

## Neutral long-horizon physical diagnostics

This is the continuous limit-violation distribution from the same three neutral 10 s trials. No arbitrary pass/fail threshold is applied. Each cell is `P50 / P95 (evaluated/candidate bounded-joint assets)`; missing or failed rollouts reduce coverage rather than improving the statistic.

| Dataset | Genesis Limit Violation (%) | PyBullet Limit Violation (%) | MuJoCo Limit Violation (%) |
|---|---:|---:|---:|
| Articraft-10K | 0.1077 / 69.6030 (175/175) | 0.0000 / 29.0215 (175/175) | 0.1001 / 17.2588 (163/175) |
| LAM released outputs | 0.0000 / 35.3625 (159/163) | 0.0000 / 158.4178 (161/163) | 4.7913 / 138.9120 (75/163) |
| Artiverse | 0.0000 / 0.8656 (189/197) | 0.0000 / 15.6094 (197/197) | 13.7410 / 2361.2727 (166/197) |
| PartNet-Mobility | 0.0004 / 10.5011 (158/168) | 0.0000 / 19.9885 (165/168) | 0.0457 / 88.4958 (58/168) |
| PhysX-Mobility | 0.0025 / 9.1071 (198/199) | 0.0000 / 12.3298 (199/199) | 0.0063 / 29.9849 (199/199) |
| SketchMobility | 0.0002 / 1.5218 (166/167) | 0.0000 / 9.1560 (81/167) | 0.0519 / 10.9029 (138/167) |
| Infinigen-Sim | 0.0000 / 0.5565 (186/200) | 0.0492 / 65.7404 (200/200) | 0.0000 / 2.1082 (199/200) |
| Ours (PV-A) | 0.0240 / 70.8367 (179/180) | 0.0000 / 173.6109 (180/180) | 0.2082 / 138.5785 (179/180) |

## Long-horizon constraint drift

Each cell is `P95 (evaluated/candidate bounded joints)`. Translation is normalized by object bounding-box diagonal; rotation is quaternion geodesic error. Coverage is retained in every cell so missing evaluations cannot improve a score.

| Dataset | Genesis Pos | Genesis Rot | PyBullet Pos | PyBullet Rot | MuJoCo Pos | MuJoCo Rot |
|---|---:|---:|---:|---:|---:|---:|
| Articraft-10K | 0.0000 (415/415) | 0.0001 (415/415) | 0.0000 (415/415) | 0.0000 (415/415) | 0.8828 (387/415) | 2.7081 (387/415) |
| LAM released outputs | 0.0000 (363/378) | 0.0001 (363/378) | 0.0000 (375/378) | 0.0000 (376/378) | 1.9538 (200/378) | 6.6363 (200/378) |
| Artiverse | 0.0000 (573/600) | 0.0000 (573/600) | 0.0000 (600/600) | 0.0000 (600/600) | 2.1671 (503/600) | 12.4934 (503/600) |
| PartNet-Mobility | 0.0000 (349/395) | 0.0000 (349/395) | 0.0000 (392/395) | 0.0000 (392/395) | 0.3051 (96/395) | 2.4252 (96/395) |
| PhysX-Mobility | 0.0000 (473/479) | 0.0000 (473/479) | 0.0000 (479/479) | 0.0000 (479/479) | 0.2143 (479/479) | 0.4305 (479/479) |
| SketchMobility | 0.0000 (288/292) | 0.0000 (288/292) | 0.0000 (174/292) | 0.0000 (174/292) | 1.1609 (205/292) | 5.6934 (205/292) |
| Infinigen-Sim | 0.0000 (734/813) | 0.0000 (734/813) | 0.0000 (813/813) | 0.0000 (813/813) | 36.0376 (809/813) | 1.4883 (809/813) |
| Ours (PV-A) | 0.0000 (624/628) | 0.0001 (624/628) | 0.0000 (628/628) | 0.0000 (628/628) | 0.5347 (626/628) | 8.0368 (626/628) |

## Strict multi-pose sensitivity

This protocol synchronously places every bounded joint at 25%, 50%, and 75% of its range and requires <=0.5% limit violation plus <=300 deg/s revolute and <=5 m/s prismatic peak speed for all three 10 s trials. It is retained as a deliberately strict stress test, not as the primary stability construct.

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

## Existing active-control diagnostics

| Dataset | Tracking NRMSE P95 (%) | Limit Violation P95 (%) |
|---|---:|---:|
| Articraft-10K | 76.9149 (415/415) | 24.9453 (415/415) |
| LAM released outputs | 83.7739 (363/379) | 17.0934 (363/379) |
| Artiverse | 70.8101 (573/600) | 28.7315 (573/600) |
| PartNet-Mobility | N/E (0/396) | N/E (0/396) |
| PhysX-Mobility | 618.2454 (479/479) | 779.3137 (479/479) |
| SketchMobility | 1737.8529 (234/292) | 336.7446 (234/292) |
| Infinigen-Sim | N/E (0/813) | N/E (0/813) |
| Ours (PV-A) | 76.9378 (624/628) | 32.3548 (624/628) |
