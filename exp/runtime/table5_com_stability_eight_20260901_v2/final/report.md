# Genesis Center-of-Mass Stability (Exploratory)

Joints are hard-locked at canonical q=0; the root remains free. The support polygon is built from Genesis ground-contact positions during the initial contact window.

## Dataset-attributable metrics

| Dataset | Eligible N | Physics Coverage ↑ | CoM Margin Coverage ↑ | Normalized CoM Support Margin ↑ | CoM Static Stability ↑ |
|---|---:|---:|---:|---:|---:|
| Articraft-10K | 159 | 71.7% | 69.8% | 0.2236 | 63.5% |
| LAM released outputs | 75 | 5.3% | 5.3% | 0.2750 | 4.0% |
| Artiverse | 160 | 100.0% | 99.4% | 0.2106 | 91.9% |
| PartNet-Mobility | 158 | 0.0% | 0.0% | N/A | 0.0% |
| PhysX-Mobility | 130 | 100.0% | 99.2% | 0.2353 | 84.6% |
| SketchMobility | 163 | 41.1% | 38.7% | 0.2032 | 34.4% |
| Infinigen-Sim | 170 | 0.0% | 0.0% | N/A | 0.0% |
| Ours (PV-A) | 156 | 100.0% | 99.4% | 0.2251 | 89.1% |

The primary view requires dataset-provided valid physics. Missing physics fails closed for CoM Static Stability; margin summaries include only attributable, non-degenerate support measurements.
CoM Support Margin is the signed COM-to-support-boundary distance. The normalized value divides it by support-polygon diameter.
The primary table reports the normalized margin because the millimeter margin is sensitive to asset scale.

## Supplementary attributable diagnostics

| Dataset | CoM Support Margin median (mm) ↑ | Conditional Stability ↑ |
|---|---:|---:|
| Articraft-10K | 81.77 | 88.6% |
| LAM released outputs | 486.25 | 75.0% |
| Artiverse | 169.76 | 91.9% |
| PartNet-Mobility | N/A | N/A |
| PhysX-Mobility | 384.99 | 84.6% |
| SketchMobility | 96.27 | 83.6% |
| Infinigen-Sim | N/A | N/A |
| Ours (PV-A) | 97.94 | 89.1% |

Conditional Stability uses physics-ready assets as its denominator and separates physical completeness from dynamics.

## Genesis-finalized diagnostic

| Dataset | N | CoM Support Margin median (normalized) ↑ | CoM Support Margin median (mm) ↑ | CoM Static Stability ↑ | Support coverage ↑ |
|---|---:|---:|---:|---:|---:|
| Articraft-10K | 159 | 0.2428 | 83.14 | 89.3% | 97.5% |
| LAM released outputs | 75 | 0.2047 | 326.15 | 70.7% | 89.3% |
| Artiverse | 160 | 0.2106 | 169.76 | 91.9% | 99.4% |
| PartNet-Mobility | 158 | 0.2139 | 220.92 | 95.6% | 99.4% |
| PhysX-Mobility | 130 | 0.2353 | 384.99 | 84.6% | 99.2% |
| SketchMobility | 163 | 0.2059 | 108.12 | 88.3% | 96.9% |
| Infinigen-Sim | 170 | 0.2707 | 163.83 | 94.7% | 100.0% |
| Ours (PV-A) | 156 | 0.2251 | 97.94 | 89.1% | 99.4% |

The diagnostic view includes Genesis-native fallback mass and COM for assets whose released physics is missing; it is not attributed to the source dataset.
