# Genesis Center-of-Mass Stability (Exploratory)

Joints are hard-locked at canonical q=0; the root remains free. The support polygon is built from Genesis ground-contact positions during the initial contact window.

## Dataset-attributable metrics

| Dataset | Eligible N | Physics Coverage ↑ | CoM Margin Coverage ↑ | Normalized CoM Support Margin ↑ | CoM Static Stability ↑ |
|---|---:|---:|---:|---:|---:|
| Articraft-10K | 159 | 71.7% | 69.8% | 0.2548 | 64.8% |
| LAM released outputs | 75 | 5.3% | 5.3% | 0.2757 | 4.0% |
| Artiverse | 160 | 100.0% | 99.4% | 0.1742 | 88.8% |
| PartNet-Mobility | 158 | 0.0% | 0.0% | N/A | 0.0% |
| PhysX-Mobility | 130 | 100.0% | 0.0% | N/A | 0.0% |
| SketchMobility | 163 | 41.1% | 0.0% | N/A | 0.0% |
| Infinigen-Sim | 170 | 0.0% | 0.0% | N/A | 0.0% |
| Ours (PV-A) | 156 | 100.0% | 99.4% | 0.2296 | 90.4% |

The primary view requires dataset-provided valid physics. Missing physics fails closed for CoM Static Stability; margin summaries include only attributable, non-degenerate support measurements.
CoM Support Margin is the signed COM-to-support-boundary distance. The normalized value divides it by support-polygon diameter.
The primary table reports the normalized margin because the millimeter margin is sensitive to asset scale.

## Supplementary attributable diagnostics

| Dataset | CoM Support Margin median (mm) ↑ | Conditional Stability ↑ |
|---|---:|---:|
| Articraft-10K | 83.55 | 90.4% |
| LAM released outputs | 487.49 | 75.0% |
| Artiverse | 133.90 | 88.8% |
| PartNet-Mobility | N/A | N/A |
| PhysX-Mobility | N/A | 0.0% |
| SketchMobility | N/A | 0.0% |
| Infinigen-Sim | N/A | N/A |
| Ours (PV-A) | 100.34 | 90.4% |

Conditional Stability uses physics-ready assets as its denominator and separates physical completeness from dynamics.

## Genesis-finalized diagnostic

| Dataset | N | CoM Support Margin median (normalized) ↑ | CoM Support Margin median (mm) ↑ | CoM Static Stability ↑ | Support coverage ↑ |
|---|---:|---:|---:|---:|---:|
| Articraft-10K | 159 | 0.2535 | 85.48 | 90.6% | 97.5% |
| LAM released outputs | 75 | 0.2254 | 374.99 | 38.7% | 46.7% |
| Artiverse | 160 | 0.1742 | 133.90 | 88.8% | 99.4% |
| PartNet-Mobility | 158 | 0.2023 | 218.74 | 91.8% | 94.9% |
| PhysX-Mobility | 130 | N/A | N/A | 0.0% | 0.0% |
| SketchMobility | 163 | 0.1955 | 125.21 | 47.2% | 53.4% |
| Infinigen-Sim | 170 | 0.1854 | 106.58 | 87.1% | 98.8% |
| Ours (PV-A) | 156 | 0.2296 | 100.34 | 90.4% | 99.4% |

The diagnostic view includes Genesis-native fallback mass and COM for assets whose released physics is missing; it is not attributed to the source dataset.
