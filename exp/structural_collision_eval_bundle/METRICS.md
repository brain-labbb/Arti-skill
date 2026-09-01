# Structural Integrity and Collision Protocol

This package evaluates URDF assets using meshes, the joint hierarchy, forward
kinematics, proximity queries, and FCL. It does not import a physics simulator.

All distances are normalized by the world-space asset bounding-box diagonal at
`q=0`. Dataset comparisons use asset-balanced macro aggregation unless the
metric is explicitly asset-binary or an asset-balanced P95.

## Structural Metrics

### 1. Rooted Assets (%) - higher is better

The original joint-origin surface ROI test is applied to every URDF edge. An
asset passes when it has one root, every joint is evaluable, and every joint has
bidirectional local support above `support_threshold`.

This is an asset-binary, fail-closed metric. Missing records and non-evaluable
assets remain in the full manifest denominator.

### 2. Joint Support Macro (%) - higher is better

For each asset, compute the fraction of joints passing the bidirectional
joint-origin ROI support test. Average those per-asset rates. Each asset has
equal weight, regardless of its joint count.

### 3. Joint Gap P95 (% diagonal) - lower is better

For each joint, collect bidirectional parent-child nearest-surface distances in
the joint-local ROI and use their Q5 as the robust mating gap. First compute the
P95 joint gap per asset, then compute P95 over the per-asset values. This gives
one contribution per asset before the dataset percentile.

### 4. Axis Rooted Assets (%) - higher is better

For every movable joint, both parent and child geometry must lie within the
axis tolerance of a bounded joint-axis segment. Fixed joints use a point probe.
An asset passes when it has one root and all joint axis-support tests pass.

Axis support measures whether the joint axis is geometrically carried. It is
not presented as proof of parent-child surface contact.

### 5. Axis Support Macro (%) - higher is better

Compute the fraction of axis-supported joints per asset and average the
per-asset rates.

### 6. K=9 Axis Pose Support Macro (%) - higher is better

Sweep each movable joint through nine uniformly spaced poses while all other
joints remain at `q=0`. For each asset, compute the fraction of evaluated poses
whose axis support remains valid, then average the per-asset rates.

The package reports asset-balanced macro, not pooled pose micro.

## Collision Metrics

Only bounded revolute and prismatic joints with finite `lower < upper` are
eligible for normalized motion-range metrics. Eligibility coverage is always
reported.

For joint `j`, collision candidates are link pairs crossing the cut between the
joint's child subtree and the rest of the asset. Pairs inside one fixed-joint
rigid cluster are excluded because they have no relative motion.

Direct parent-child pairs are not globally excluded. FCL contacts close to the
actuated joint axis are masked as connector contacts; contacts outside that
joint-local tube remain eligible. For every eligible pair:

```text
growth(q) = max(0, depth(q) - depth(q=0))
```

An event requires both absolute penetration and growth to exceed `0.2%` of the
asset diagonal. This tolerates unchanged initial overlap and small numerical or
closure contact while retaining parent-child penetration away from the joint.

### 7. Collision-Free Joint Motion Range (%) - higher is better

With K sampled states there are K-1 motion intervals. An interval is valid when
both endpoints are free of motion-induced collision. Compute valid intervals / 
all intervals per joint, average within each asset, then average across assets.

This is a sampled approximation, not continuous collision detection.

### 8. Premature Collision-Free Joints (%) - higher is better

A joint passes when all interior samples are collision-free. Endpoint samples
are omitted from this binary metric so normal closure at a declared limit does
not become a premature-collision failure. Severe endpoint penetration can still
reduce Collision-Free Joint Motion Range and Penetration Growth P95.

Compute the passing-joint rate per asset, then average across assets.

### 9. Penetration Growth P95 (% diagonal) - lower is better

For each joint, take the maximum penetration growth over all sampled poses and
eligible crossing pairs. Compute Q95 across joints inside each asset, then Q95
over the per-asset values.

## Frozen Defaults

| Parameter | Value |
|---|---:|
| Structural geometry | visual |
| Collision geometry | collision, visual fallback |
| Joint ROI radius | 2% asset diagonal |
| Support/gap threshold | 0.2% asset diagonal |
| Support ratio threshold | 10% |
| Collision absolute threshold | 0.2% asset diagonal |
| Collision growth threshold | 0.2% asset diagonal |
| Pose samples | K=9 |
| Structural surface samples | 128 per side |
| Maximum FCL contacts per pair | 128 |

These are evaluation settings, not community-standard constants. Calibrate them
on a frozen human-labeled valid/broken subset before publication, then do not
select thresholds based on which dataset wins.

## Known Boundaries

- K=9 can miss collisions between adjacent samples; use CCD only as an optional
  expensive audit.
- FCL triangle-mesh penetration depth is sensitive to collision-mesh quality.
- The parent-child contact mask depends on FCL contact positions and a bounded
  axis tube. Audit masked and retained contact counts on a labeled subset.
- Geometry load failures and unsupported mesh references must be reported via
  coverage and must never silently improve a dataset score.
