# Cross-Dataset Comparison Under a Matched Five-Asset Rendering Protocol

## Scope

This document summarizes a class-wise feature-space comparison of PV-A,
Artiverse, Articraft-10K, PartNet-Mobility, and LAM. The quantitative results
use only the strict subset in which every retained category has exactly five
distinct successfully rendered assets. No category is padded with duplicate
images.

## Experimental protocol

All assets were rendered with a matched Blender studio contract: the same
image resolution, camera direction and field of view, lighting arrangement,
world/ground background, color-management settings, and Cycles sampling. The
camera distance is adapted to each object's bounding sphere only to keep the
object consistently framed and fully visible. This is a framing policy, not a
dataset-specific appearance adjustment.

The strict cohort contains the following numbers of categories and images:

| Dataset | All categories | All renders | Strict categories | Strict renders |
|---|---:|---:|---:|---:|
| PV-A | 531 | 2,655 | 531 | 2,655 |
| Artiverse | 84 | 368 | 65 | 325 |
| Articraft-10K | 244 | 1,193 | 236 | 1,180 |
| PartNet-Mobility | 46 | 230 | 46 | 230 |
| LAM | 660 | 1,279 | 93 | 465 |

For every strict category, five rendered images were embedded with DINOv2-base
(768 dimensions) and CLIP ViT-B/32 (512 dimensions). Feature vectors were
L2-normalized before computing similarities and metrics.

## Metrics

Let $z_i$ be a normalized feature vector and $y_i$ its category label. Cosine
similarity is $s_{ij}=z_i^\top z_j$.

- **Top-1 same-class nearest-neighbor rate.** For each image, find the most
  similar *other* image (the query itself is excluded) and report the fraction
  whose nearest neighbor has the same category:
  
  $\frac{1}{N}\sum_i\mathbf{1}\left[y_{\operatorname{argmax}_{j\ne i}s_{ij}}=y_i\right]$.

- **Cosine silhouette.** For image $i$, let $a_i$ be the mean cosine distance
  to the other four images in its category and let $b_i$ be the smallest mean
  cosine distance to any other category. We report the mean of
  $(b_i-a_i)/\max(a_i,b_i)$. Higher values indicate tighter within-category
  organization and better separation from the nearest competing category;
  negative values indicate substantial overlap.

## Main results

| Dataset | Strict classes (exactly 5 samples/class) | DINOv2 Top-1 same-class NN $\uparrow$ | DINOv2 cosine silhouette $\uparrow$ | CLIP ViT-B/32 Top-1 same-class NN $\uparrow$ | CLIP ViT-B/32 cosine silhouette $\uparrow$ |
|---|---:|---:|---:|---:|---:|
| PV-A | 531 | **0.881** | **0.289** | **0.592** | 0.064 |
| Artiverse | 65 | 0.735 | 0.259 | 0.569 | **0.092** |
| PartNet-Mobility | 46 | 0.696 | 0.196 | 0.552 | 0.056 |
| Articraft-10K | 236 | 0.527 | 0.075 | 0.407 | -0.041 |
| LAM | 93 | 0.432 | -0.019 | 0.320 | -0.099 |

## Results interpretation

Under the matched rendering protocol, PV-A obtains the highest DINOv2 Top-1
rate (0.881) and cosine silhouette (0.289). This indicates the strongest
class-consistent organization in DINOv2 feature space among the evaluated
strict cohorts. PV-A also has the highest CLIP Top-1 rate (0.592), although the
margin over Artiverse is small.

PV-A does not dominate every metric: Artiverse has the higher CLIP cosine
silhouette (0.092 versus 0.064). The apparent advantage is therefore dependent
on the feature encoder and should be described as stronger visual class
consistency under the tested conditions, rather than as proof of universally
superior asset quality.

The results are descriptive feature-space statistics. They may be affected by
taxonomy granularity, category definitions, the number of eligible categories,
rendering/style distributions, and the five-sample eligibility rule. In
particular, only 93 of LAM's 660 categories satisfy the strict criterion.
Because the strict category counts differ, the nearest-neighbor random
baseline and the difficulty of finding a nearest competing category also
differ across datasets. Thus, the table should be read as a descriptive
feature-space comparison rather than a strictly calibrated overall ranking.
Consequently, these measurements do not by themselves establish superiority in
semantic coverage, geometric accuracy, realism, diversity, or downstream task
performance.

## Suggested paper paragraph

> Under a matched Blender rendering protocol, we retained categories with five
> distinct rendered assets and extracted L2-normalized features using DINOv2-
> base and CLIP ViT-B/32. PV-A achieved the highest DINOv2 Top-1 same-class
> nearest-neighbor rate (0.881) and cosine silhouette (0.289), indicating the
> strongest class-consistent organization in DINOv2 space. It also achieved the
> highest CLIP Top-1 rate (0.592), while Artiverse obtained a higher CLIP
> cosine silhouette (0.092 versus 0.064). These results support a PV-A advantage
> in visual class consistency under the tested encoders, but do not imply that
> PV-A is globally superior in data quality or downstream utility.

## Coordinate alignment and figures

The figures below use a single joint t-SNE fit over all 4,855 strict `n=5`
images for each encoder. The five dataset-specific figures then reuse the
corresponding subset of those joint coordinates. Within a given encoder, all
five figures use identical square `x/y` limits, so locations and relative
spread can be compared directly. DINOv2 and CLIP remain separate feature
spaces and are therefore shown in separate panels. Points are rendered images,
colors identify categories, and outlined points are category centers.

### PV-A

![PV-A strict n=5 aligned joint t-SNE](individual_tsne/aligned_strict_n5/pva.png)

### Artiverse

![Artiverse strict n=5 aligned joint t-SNE](individual_tsne/aligned_strict_n5/artiverse.png)

### Articraft-10K

![Articraft-10K strict n=5 aligned joint t-SNE](individual_tsne/aligned_strict_n5/articraft10k.png)

### PartNet-Mobility

![PartNet-Mobility strict n=5 aligned joint t-SNE](individual_tsne/aligned_strict_n5/partnet_mobility.png)

### LAM

![LAM strict n=5 aligned joint t-SNE](individual_tsne/aligned_strict_n5/lam.png)

### Joint and metric summaries

![Joint strict n=5 t-SNE by dataset](tsne_joint_strict_n5_by_dataset.png)

![Strict n=5 feature-space metrics](feature_space_metrics_strict_n5.png)

## Artifact references

The aligned strict per-dataset PNGs and their shared-axis manifest are in
`individual_tsne/aligned_strict_n5/`. The source joint coordinate files are in
`coordinates/dinov2/joint_strict_n5.csv` and
`coordinates/clip/joint_strict_n5.csv`. The source metric table is
`strict_n5_feature_metrics.csv`, and the complete dataset/sample audit is
`audit.json`.
The analysis was produced by `exp/scripts/visualize_five_datasets_n5_tsne.py`.

## Reporting note

t-SNE is used for qualitative visualization only. The aligned figures use one
joint fit per encoder and shared limits within that encoder; DINOv2 and CLIP
coordinates are still not comparable to one another. Quantitative claims
should be based on the original feature-space metrics above. The earlier
independently fitted panels remain under `individual_tsne/strict_n5/` for
diagnostic reference, but should not be used for cross-dataset positional
comparisons.
