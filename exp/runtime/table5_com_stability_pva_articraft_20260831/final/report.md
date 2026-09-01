# Genesis Center-of-Mass Stability (Exploratory)

Joints are hard-locked at canonical q=0; the root remains free. The support polygon is built from Genesis ground-contact positions during the initial contact window.

## Dataset-attributable metrics

| Dataset | N | Physics coverage ↑ | CoM Support Margin median (mm) ↑ | Normalized margin ↑ | CoM Static Stability ↑ | Conditional stability ↑ |
|---|---:|---:|---:|---:|---:|---:|
| Ours (PV-A) | 156 | 100.0% | 100.34 | 0.2296 | 90.4% | 90.4% |
| Articraft-10K | 159 | 71.7% | 83.55 | 0.2548 | 64.8% | 90.4% |

The primary view requires dataset-provided valid physics. Missing physics fails closed for CoM Static Stability; margin summaries include only attributable, non-degenerate support measurements.
CoM Support Margin is the signed COM-to-support-boundary distance. The normalized value divides it by support-polygon diameter.
Conditional stability uses physics-ready assets as its denominator and separates physical completeness from dynamics.

## Genesis-finalized diagnostic

| Dataset | N | CoM Support Margin median (normalized) ↑ | CoM Support Margin median (mm) ↑ | CoM Static Stability ↑ | Support coverage ↑ |
|---|---:|---:|---:|---:|---:|
| Ours (PV-A) | 156 | 0.2296 | 100.34 | 90.4% | 99.4% |
| Articraft-10K | 159 | 0.2535 | 85.48 | 90.6% | 97.5% |

The diagnostic view includes Genesis-native fallback mass and COM for assets whose released physics is missing; it is not attributed to the source dataset.
