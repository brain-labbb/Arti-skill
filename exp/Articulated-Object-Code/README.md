---
license: mit
language:
- en
task_categories:
- text-to-3d
tags:
- articulated-objects
- urdf
- robotics
- procedural-3d
- code-generation
size_categories:
- 1K<n<10K
configs:
- config_name: default
  data_files:
  - split: train
    path: articulated_code.parquet
- config_name: manifest
  data_files:
  - split: train
    path: manifest.parquet
---

# Articulated-Object-Code

3D **articulated objects** (objects with moving parts — drawers, hinges, wheels, scissors,
robot arms, …) generated from natural-language descriptions by the **LAM** pipeline
(*Language-driven Articulated Mesh Generation*): a multi-stage LLM pipeline that writes
geometry code and assembles it into a **URDF + meshes** robot description.

Each object ships its `generated.urdf` (links + joints), the part meshes it references,
materials, the generation configs (including the original text prompt), and the **full
generation logs** (VLM-feedback iterations kept verbatim).

**3217 objects** total, across **~660 categories**. Quality is graded by the `tier` column
of `manifest.parquet`.

## Contents

| File | What |
|---|---|
| `viable.tar.gz` | **2533 recommended objects** — load as valid articulated URDFs *and* pass deep checks (real geometry, parts assembled together, sane joints). |
| `loads_only.tar.gz` | 299 objects — valid loadable URDF but failed a deep check (a detached part, a joint with no limit, or a NaN mesh). |
| `imperfect.tar.gz` | 385 objects — failed structural validation (kept for completeness). |
| `articulated_code.parquet` | **Self-contained index + code.** One row per object with the generating **code inlined**: `threejs_code` (the Three.js geometry code), `urdf`, `articulation_json`, `links_hierarchy_json`, the caption, and every validation/metadata field — browse without unpacking the tars. |
| `manifest.parquet` / `manifest.csv` | Lightweight index (same rows, no code columns): tier, validation flags, caption, model, joint/part counts, `rel_path`. |

Extracting a tar gives `objects/<category>/<category>_NNN/` (or `imperfect/…`):

```
<category>_NNN/
├── generated.urdf          # articulation spec (links + joints)
├── links/ or obj_parts/    # the meshes the URDF references
├── *.mtl, *.png            # materials / textures
├── configs/*.json          # articulation.json, links_hierarchy.json, generation_config.yaml
└── pipeline_logs/ …        # full generation logs (VLM-feedback iterations)
```

## Quality tiers

| tier | count | meaning |
|---|---|---|
| **`viable`** | **2533** | Loads as a valid articulated URDF **and** passes deep checks: real non-degenerate geometry, all parts assembled together (no detached/floating part), sane joint axes + limits. |
| `loads_only` | 299 | Valid loadable URDF, failed a deep check. |
| `broken` | 385 | Failed structural validation. |

## Usage

```python
import pandas as pd
from huggingface_hub import hf_hub_download

# index + generating code, inlined (no download of meshes needed)
df = pd.read_parquet(hf_hub_download(
    "YipengGao/Articulated-Object-Code", "articulated_code.parquet", repo_type="dataset"))
print(df[df.tier == "viable"].category.value_counts())     # 2533 ready-to-use objects

row = df[df.tier == "viable"].iloc[0]
print(row.caption)         # the text prompt
print(row.threejs_code)    # the Three.js code that builds the geometry
print(row.urdf)            # the articulation spec (links + joints)

# to get the actual meshes, pull the tar:
hf_hub_download("YipengGao/Articulated-Object-Code", "viable.tar.gz", repo_type="dataset")
# tar xf viable.tar.gz  ->  objects/<category>/<id>/generated.urdf + meshes
```

Load any object in a URDF viewer / simulator (`yourdfpy`, `pybullet`, …):

```python
import pybullet as p
p.connect(p.DIRECT)
p.loadURDF("objects/<category>/<id>/generated.urdf")
```

## Manifest columns

`object_release_id`, `category`, **`tier`** (viable / loads_only / broken), `status`,
`viable`, `geom_ok`, `assembly_ok`, `motion_ok`, `n_floating_parts`, `max_gap_ratio`,
`reasons`, `deep_reasons`, `caption` (original prompt), `model`, `pipeline`, `gen_date`,
`n_links`, `n_joints`, `n_movable`, `n_meshes`, `total_verts`, `total_faces`, `has_material`,
`rel_path`.

## Validation

**Structural:** URDF parses · ≥2 links · ≥1 movable joint · joints link real links · kinematic
tree connected & acyclic · referenced meshes exist, non-empty, have geometry.
**Deep (viable):** meshes finite & non-degenerate · forward-kinematics at rest pose confirms
every child part is attached to its parent (no floating parts) · movable joints have non-zero
axis + sane limits. Cross-checked with `pybullet` (objects load and pose across joint ranges).

## Companion dataset

Blender-Python 3D code generation (a separate paper): [`YipengGao/3DCode`](https://huggingface.co/datasets/YipengGao/3DCode).
