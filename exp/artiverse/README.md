---
language:
  - en
pretty_name: Artiverse
tags:
  - 3D
viewer: false
license: other
extra_gated_heading: Acknowledge license to accept the repository
extra_gated_prompt: >-
  By using data from this repo, users agree to the corresponding licensing of the original data.

  To request access to Artiverse, you will need to provide your **full
  name** (please provide both your first and last name), the name of your
  **advisor or the principal investigator (PI)** of your lab (in the PI/Advisor)
  fields, and the **school or company** that you are affiliated with (the
  **Affiliation** field).   After requesting access to this repo, you
  will be considered for access approval. 

extra_gated_fields:
  Name: text
  PI/Advisor: text
  Affiliation: text
  Purpose: text
  Country: text
  I have read and agree to the corresponding licenses of the datasets: checkbox
  I would like to receive occasional emails with Artiverse updates: checkbox
---

# Artiverse: A Diverse and Physically Grounded Dataset for Articulated Objects

*Disclaimer: this repository contains data that is still under development and is continuously being verified and improved. We are committed to making it as clean and useful as possible for the community. If you find **any issues** with the data or have **any feedback/requests**, please file an issue on Hugging Face or the [data viewer interface](https://aspis.cmpt.sfu.ca/artiverse_viewer/#/), or send us an email. Your feedback is an important part of the community effort to bring more high-quality articulated objects data to everyone. Thank you!*

This repo contains a subset of pre-release Artiverse assets. We are actively working on finalizing the data. We will soon release the remaining models and then the cleanup pass resolving all the remaining problems. The [data viewer](https://aspis.cmpt.sfu.ca/artiverse_viewer/#/) is available online.

Our data structure is organized by semantic category, source, and model id:

```text
data/
  {category}/
    {source}/
      {model_id}/
        {model_id}.segmented.glb
        {model_id}.articulations.json
        material.json
        imgs/
        urdf_w_collider/
```

Where:

- `{category}` is the object class, for example `armoire`, `bookcase`, `chest_of_drawers`, `desk`, `sideboard`, `tv_stand`, or `wall_cabinet`.
- `{source}` is the upstream asset source. The current release uses `fpModel`.
- `{model_id}` is the unique model identifier within that source.
- Note - this structure will be updated in the full release.

Each model folder contains:

- `{model_id}.segmented.glb`: segmented geometry with part structure and metadata used by our processing pipeline.
- `{model_id}.articulations.json`: articulation annotations, including articulated part ids, joint types, axes, and motion ranges.
- `material.json`: inferred material assignments and densities for the segmented parts.
- `mass_furniture_heuristic.json`: per-part and total mass estimates.
- `imgs/`: reference renderings for the object.
- `urdf_w_collider/`: URDF export with visual meshes and collision geometry for simulation use.

The repository stores the dataset as tar.gz chunks. To reconstruct `data/` from the chunks:

```bash
python pack_dataset_chunks.py unpack --chunks-dir dataset_chunks --overwrite --verify-sha256
```

[pygltftoolkit](https://github.com/3dlg-hcvc/pygltftoolkit) can be used to load the GLBs and parse the segmentation annotations. The annotation formats largely follow S2O, so please check out the [documentation](https://huggingface.co/datasets/3dlg-hcvc/s2o/blob/main/ANNOTATIONS.md).
