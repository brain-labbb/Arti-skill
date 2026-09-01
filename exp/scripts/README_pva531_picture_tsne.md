# Joint PV-A / picture t-SNE

## Uniform Blender benchmark

For a controlled comparison, render one `seed_0000` asset from every CSV row
before extracting features.  The runner writes to a fresh root and records the
renderer hash, Blender version, full studio contract, per-image SHA-256, and
one status row per generator:

```bash
/root/miniconda3/envs/RoboDojo/bin/python \
  exp/scripts/render_pva531_uniform.py \
  --output-root /mnt/zsn/data/particulate/datasets/PV-A/renders/uniform531_studio_256_v1 \
  --gpu 7 --workers 4 --resolution 256 --samples 4
```

All renders use Cycles with denoising, AgX Medium High Contrast, the same
42-degree vertical FOV and camera direction, the same three area-light
directions and gains, and the same world/ground background.  Camera distance
and light size are scaled from each object's bounding sphere so differently
sized objects have the same framing policy and remain fully visible.  This is
the only per-object scene adaptation.

Extract fresh features from the audited render root with:

```bash
CUDA_VISIBLE_DEVICES=7 /root/miniconda3/envs/RoboDojo/bin/python \
  exp/scripts/visualize_pva531_picture_tsne.py \
  --uniform-render-root \
    /mnt/zsn/data/particulate/datasets/PV-A/renders/uniform531_studio_256_v1 \
  --output-dir exp/runtime/pva531_uniform_tsne \
  --device cuda:0 --force-extract
```

In this mode every generator has exactly one rendered image.  The original
99/432 roster origin and mapped picture category are retained only as plot
metadata; no source photograph is passed to DINOv2 or CLIP.  The render root
must contain a successful `render_manifest.csv` and the matching
`render_config.json`, and all 531 images must have one configured resolution.
Schema-v2 render configs also fingerprint every selected URDF, appearance
receipt, mesh, the PBR snapshot pointer, and the imported material helper.
Resume accepts a PNG only when it matches a prior successful content receipt;
new renders are atomically promoted from a temporary PNG after validation.

## Mixed reference-image benchmark

`visualize_pva531_picture_tsne.py` creates a class-level visualization for
the 531 rows in `template_maps/generator_picture_index.csv`:

- 99 `articraft_builtin_dataset_no_picture` rows use the PV-A
  `seed_0000` representative render.
- 432 `picture_backed` rows use every PNG in their mapped picture directory.
  Image embeddings are L2-normalized, averaged per generator row, and
  normalized again before t-SNE.  This prevents directories with more source
  photos from receiving more points.
- The two `Public toilet` generator rows share one source directory.  Both
  generator identities remain in the output, so the final class plot has 531
  points and records the alias in `dataset_manifest.json`.

Run both encoders with the local Hugging Face snapshots:

```bash
/root/miniconda3/envs/RoboDojo/bin/python \
  exp/scripts/visualize_pva531_picture_tsne.py \
  --device cuda:0 \
  --output-dir exp/runtime/pva531_picture_tsne
```

The script writes `dinov2/` and `clip/` subdirectories containing raw image
features, 531 class features, NumPy/CSV t-SNE coordinates, and three PNG
plots (`tsne_by_source.png`, `tsne_by_picture_category.png`, and
`tsne_builtin_labels.png`).  `dataset_manifest.json` and
`generator_roster_resolved.csv` describe the exact image membership.  DINOv2
and CLIP are reduced independently; their feature dimensions are 768 and
512 respectively.

Use `--force-extract` to invalidate the raw feature caches.  The default
model paths point to revisions `f9e44c814b77203eaa57a6bdbbd535f21ede1415`
(DINOv2-base) and `3d74acf9a28c67741b2f4f2ea7635f0aaf6f0268`
(CLIP ViT-B/32).  A Torch environment at version 2.6 or newer is required
for the CLIP `pytorch_model.bin` snapshot.
