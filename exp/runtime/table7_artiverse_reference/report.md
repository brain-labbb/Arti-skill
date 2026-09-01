# Table 7: Artiverse real-data production-readiness reference

Status: **COMPLETE**

The full reference retains all 3544 identities from the exact
official chunk manifest. No unavailable, malformed, or failed asset is replaced or
removed. Expensive mesh topology uses a separate content-blind frozen N=100
cohort; all other locally measurable axes use N=3544.

## Cohorts

- Full requested/available: 3544/3543.
- Portable-package evaluable: 3544.
- Geometry requested/evaluable: 100/100.
- Geometry selection: ascending SHA256(manifest root UTF-8), then manifest root; take first 100.

## Results

| Axis | Result |
|---|---:|
| Watertight | 11233/17804 readable geometries; 0.076988 mean/asset |
| Edge-manifold proxy | 17795/17804 readable geometries; 0.996567 mean/asset |
| Open edges | 6131016 total; 61310.160/evaluable geometry asset |
| Degenerate faces | 12051 total; 120.510/evaluable geometry asset |
| Portable Package | 3543/3544 evaluable |
| Semantic field proxy | 3526/3543 evaluable |
| Kinematic Complete | 3541/3543 applicable |
| Physical Complete | 0/3543 applicable |

## Evidence boundaries

- Strict semantic completeness is not evaluable without output-independent part/role gold; the named-part/tree result is only a field proxy.
- Deterministic Build is not evaluable because two fresh builds were not run.
- Self-intersection is not evaluable because no exact adjacent-face-excluding backend ran.
- Simulator-provided defaults do not count as native physical metadata.

## Reproduction

```bash
python exp/scripts/run_artiverse_production_readiness_reference.py --preflight-only
python exp/scripts/run_artiverse_production_readiness_reference.py --workers 2
python exp/scripts/run_artiverse_production_readiness_reference.py --verify-only
```
