# Ours / PV-A Artiverse Table 2

- Classification: **STRUCTURAL_PROXY**
- Mode: **structural-proxy**
- Paper reference: [Artiverse Table 2](https://arxiv.org/abs/2605.24403), page 6
- Evaluated assets: **302,440 / 302,440**
- Runner SHA-256: `6ab0e126335adc979cd7f12da88e602b4ad5b661beed57da70b0fb723c64cd94`
- Protocol SHA-256: `b908de0e6635e64b811a3941add143eaf6f037a5ece183944e7fe50cc4459071`

## Table 2-shaped output

| Dataset | # obj | Category total | Avg # obj | # Func. Parts total | Avg | # Arti. Parts total | Avg | # Joints 1-DoF | # Joints 2-DoF |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Ours / PV-A | 302,440 | 531 | 569.6 | 1,979,929 | 6.5 | 1,453,516 | 4.8 | 1,453,516 | 0 |

## Scope and diagnostics

This row is a **STRUCTURAL_PROXY**, not a semantic annotation result.
- Functional parts: URDF links with at least one visual element.
- Articulated parts: unique child links of non-fixed XML joints.
- Joints: non-fixed XML joint elements; XML types are mapped to DoF buckets.
- A paper-comparable semantic row requires a complete `--annotations` sidecar and `--mode semantic`.

- Logical joints (all DoF): 1,453,516; 3-DoF: 0; other: 0.
- Representation movable XML joints: 1,453,516.
- Semantic minus representation logical-joint count: 0.
- Roster joint-count mismatches: 0.
- Average denominator: 302,440 (null when any selected asset fails).
