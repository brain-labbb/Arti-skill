# arti-skill Benchmark

## Benchmark Overview

`arti-skill Benchmark` evaluates whether an agent can convert source articulated assets into reusable, executable, and mechanically valid parametric templates. Unlike single-asset generation benchmarks, each benchmark item is a template with a generation domain, rather than one fixed mesh.

The benchmark evaluates five capabilities:

1. template construction reliability;
2. multi-seed generation validity;
3. articulation and collision correctness;
4. controllable and diverse asset generation;
5. simulation readiness and downstream utility.

The final runtime artifact is a self-contained template that can deterministically generate URDF, mesh, and validation reports without calling an LLM.

## Task Definition

Each task is defined by:

```text
T = (I, C, S, D, R)
```

where `I` denotes source images/assets, `C` denotes the target category, `S` denotes source-map evidence, `D` denotes the expected template domain, and `R` denotes the reproducibility configuration.

Given a task, a method must produce:

- an executable parameterized template;
- a machine-readable template domain and configuration interface;
- generated assets for the prescribed seeds;
- URDF and mesh artifacts;
- structural, motion, and validation reports.

The template must not import source runtime code, other templates, or seed-specific exceptions at generation time.

## Dataset and Splits

The benchmark manifest separates semantic categories, executable templates, source assets, and generated seeds. For every template we record:

- category and template identifier;
- SourceMap and TemplateDesign availability;
- component slots and candidate components;
- continuous parameter dimensions;
- discrete and effective combination-domain sizes;
- link/joint and articulation statistics;
- source and template version hashes.

Each template is evaluated on a fixed 36-seed suite:

- 32 coverage seeds sampled with a fixed pairwise/LHS or Sobol protocol;
- 4 boundary seeds covering minimum, maximum, compact, and expanded configurations;
- additional authored `corner` cases supplied by the template design.

For downstream learning experiments, templates—not seeds—are split into train, validation, and test sets. Unseen-seed evaluation is used for interpolation, unseen-component evaluation for compositional generalization, and unseen-template evaluation for category-level generalization.

## Compared Methods

| Method | SourceMap | TemplateDesign | Domain constraints | Multi-seed harness | Corner/regression checks |
|---|---:|---:|---:|---:|---:|
| Direct-Agent | ✗ | ✗ | ✗ | ✗ | ✗ |
| Source-backed | ✓ | ✗ | partial | ✗ | ✗ |
| Template-only | ✗/partial | ✓ | ✓ | partial | ✗ |
| arti-skill | ✓ | ✓ | ✓ | ✓ | ✓ |

All methods use the same source tasks, model version, execution environment, token/tool budget, repair budget, and evaluation seeds. Failed runs remain in the denominator.

## Evaluation Metrics

### Generation Reliability

We report:

- **Template Success**: fraction of tasks producing an executable template;
- **First-shot Success**: success before repair;
- **Final Success**: success after the fixed repair budget;
- **Seed Pass Rate**: fraction of generated seeds passing all required gates;
- **Template Full-pass Rate**: fraction of templates passing all 36 standard seeds and corner cases;
- **Repair Count** and **Human Intervention Rate**;
- **Regression Retention**: fraction of previously passing seeds that remain valid after repair;
- wall-clock time, compile/check count, token usage, and cost.

\[
R_{seed}=\frac{N_{passing\ seeds}}{N_{generated\ seeds}}
\]

\[
R_{retention}=1-\frac{N_{regressed\ old\ seeds}}{N_{rechecked\ old\ seeds}}
\]

### Structural and Production Validity

For every generated seed we evaluate:

- executable and artifact-saved rate;
- valid URDF tree, root count, reachability, and parent-child closure;
- visual/collision mesh reference completeness;
- isolated-part and static-overlap rate;
- mesh watertight/manifold/open-edge statistics;
- link/joint count and semantic-name coverage;
- mass, inertia, damping, friction, and collision-field completeness.

Semantic precision/recall is reported only when a frozen semantic gold annotation is available.

### Articulation and Motion Validity

For every movable joint we measure:

- joint type accuracy;
- axis angular error;
- pivot-to-axis distance;
- parent/child and joint-origin correctness;
- limit and realized-range plausibility;
- motion-state coverage;
- full-range collision-free rate;
- minimum clearance and maximum penetration;
- Average Overlapping Ratio (AOR).

Motion is tested at rest, boundary, intermediate, and multi-joint coupled states. Endpoint-only checks are not considered sufficient for the full-range metric.

### Diversity and Compositionality

At the template level we report:

- number of slots, candidates, continuous parameters, and effective combinations;
- topology-family count and topology entropy;
- link/joint/depth distributions across seeds;
- fraction of seeds with component, topology, or only scale changes;
- exact duplicate and geometric near-duplicate rates;
- normalized point-cloud/voxel distances;
- graph edit distance between articulation trees.

MMD, COV, and 1-NNA are used only when an independent reference distribution is available. Otherwise, diversity is reported as within-template and cross-template variation.

### Controllability and Reproducibility

Given target properties such as height, width, component count, joint angle, or travel range, we re-measure the generated geometry and kinematics instead of reading the input parameter directly. We report:

- target hit rate;
- normalized mean absolute error;
- monotonic response rate;
- no-effect parameter rate;
- non-target preservation.

For repeated compilation of the same template-seed pair, we report URDF hash agreement, mesh hash agreement, semantic agreement, and bit-identical rate.

### Simulation Readiness

On a fixed stratified subset, each asset is evaluated in MuJoCo, PyBullet, SAPIEN, and Genesis when available. We report:

1. parser/import success;
2. scene construction and first-step success;
3. fixed-duration gravity and articulation rollout;
4. NaN/explosion rate, penetration, drift, limit violation, and runtime.

Exact mesh, convex hull, and convex-decomposition collision representations are evaluated separately to expose the geometry–simulation trade-off.

## Evaluation Protocol

The benchmark uses three levels:

| Level | Scope | Purpose |
|---|---|---|
| L1: Full-corpus QC | all templates × 36 seeds + corner | executable, structural, parameter, and lightweight motion validity |
| L2: Stratified physics | fixed subset across complexity and articulation types | full-range collision, physical completeness, and stability |
| L3: Generalization | held-out templates/components/combinations | compositional and downstream generalization |

The authoritative template stages are:

```text
preflight → random-16 → random-36 → corner
```

All failures are categorized by stage and type: execution, artifact, mesh, structure, semantic, parameter response, static collision, motion collision, physics, simulator, and visual review.

## Reporting Protocol

Each run publishes:

```text
experiment_manifest.jsonl
seed_manifest.jsonl
template_results.jsonl
seed_results.jsonl
motion_results.jsonl
simulator_results.jsonl
diversity_results.jsonl
controllability_results.jsonl
failure_taxonomy.jsonl
summary.json
```

The main paper tables are:

| Table | Content |
|---|---|
| Table 1 | benchmark scale and template-domain statistics |
| Table 2 | reliability and ablation results |
| Table 3 | structure, articulation, collision, and AOR |
| Table 4 | collision representation and simulator readiness |
| Table 5 | diversity, controllability, and reproducibility |
| Table 6 | held-out generalization and optional downstream results |

## Validity Rules

Compilation success is not treated as physical correctness. Results are reported both before and after quality filtering. AOR=0 on a filtered release set is not sufficient evidence for collision-free generation. Vision scores do not replace kinematic checks, and reconstruction metrics such as PSNR or Chamfer are used only for paired ground-truth tasks.

## Minimal Benchmark Package

The minimal paper benchmark consists of:

1. Direct-Agent and arti-skill ablation;
2. full-corpus 36-seed and corner validation;
3. full-range motion and collision-representation comparison;
4. template-level diversity, controllability, and determinism;
5. stratified multi-simulator evaluation;
6. held-out downstream evaluation when a reproducible downstream task is available.

