---
language:
  - en
pretty_name: SketchMobility
tags:
  - 3D
  - articulated-objects
  - robotics
  - urdf
  - motion
size_categories:
  - 1K<n<10K
viewer: false
license: other
extra_gated_heading: Request access to SketchMobility
extra_gated_prompt: >-
  Please provide the information below and acknowledge the applicable
  SketchMobility and upstream license terms described in LICENSE.md and
  LICENSE_MAP.json. Access is granted automatically after submission.
extra_gated_fields:
  Name: text
  PI/Advisor: text
  Affiliation: text
  Purpose: text
  Country: country
  I have read and agree to the applicable licenses: checkbox
---

# SketchMobility

SketchMobility is a curated release of **4,956 articulated 3D objects** with
human-reviewed motion annotations, URDF descriptions, source-sensitive mesh
assets, and per-object normalization metadata.

This dataset accompanies
[Sketch2Arti: Sketch-based Articulation Modeling of CAD Objects](https://arxiv.org/abs/2604.25781).

## Dataset

| Source | Objects | License treatment |
| --- | ---: | --- |
| [Articraft](https://articraft3d.github.io/) (`Agentic`) | 205 | CC BY 4.0 |
| [Infinigen-Articulated](https://huggingface.co/datasets/princeton-vl/infinigen-articulated) | 726 | CC BY 4.0 |
| [PartNeXt](https://huggingface.co/datasets/AuWang/PartNeXt) | 2,177 | CC BY 4.0 |
| [Shape2Motion](https://github.com/wangxiaogang866/Shape2Motion) | 1,848 | GPL-3.0 |
| **Total** | **4,956** | **Mixed** |

After unpacking, each object has this layout:

```text
data/
  {source}/
    {category}/
      {display_id}/
        annotation.json
        mobility.urdf
        meshes/
```

`annotation.json` retains the original `upstreamId`, release ID, joints, mesh
links, and exact normalization transform. Mesh encodings remain
source-sensitive.

Sketch images are not included. The sketch-generation script will be released
with the Sketch2Arti system code.

## Download and unpack

The dataset follows the Artiverse release convention: verified `tar.gz` chunks
are stored under `dataset_chunks/`, and each object directory stays within one
chunk.

```bash
hf download Arlo397/SketchMobility \
  --repo-type dataset \
  --local-dir SketchMobility

cd SketchMobility
python pack_dataset_chunks.py unpack \
  --chunks-dir dataset_chunks \
  --overwrite \
  --verify-sha256
```

The command reconstructs `data/`. Chunk sizes and SHA-256 checksums are listed
in `dataset_chunks/manifest.json`.

## License and attribution

SketchMobility is a mixed-license compilation. SketchMobility-original
curation, annotations, corrections, normalization metadata, release
identifiers, and documentation are CC BY 4.0 to the extent these rights are
held by the contributors. Upstream terms continue to apply.

See [LICENSE.md](LICENSE.md), [LICENSE_MAP.json](LICENSE_MAP.json), and
[ATTRIBUTION.md](ATTRIBUTION.md).

## Citation

```bibtex
@misc{yang2026sketch2arti,
  title         = {Sketch2Arti: Sketch-based Articulation Modeling of CAD Objects},
  author        = {Yang, Yi and Pan, Hao and Cui, Yijing and Sheffer, Alla and Li, Changjian},
  year          = {2026},
  eprint        = {2604.25781},
  archivePrefix = {arXiv},
  primaryClass  = {cs.CV}
}
```
