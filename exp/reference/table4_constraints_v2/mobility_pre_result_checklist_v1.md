# Mobility Table 4 pre-result checklist v1

This checklist was frozen by the independent protocol auditor before any formal
PhysX-Mobility or PartNet-Mobility render, selection, materialization, or score
output existed. After this point, review scope is limited to this checklist and
real regressions introduced while satisfying it.

## 0. Pre-result and freeze

- All formal private/render/snapshot/result/staging roots are absent, with no
  related formal process. A PASS audit root may contain only
  `protocol_audit_pre_result.json` and `report.md`.
- The effective amendment/addendum is `pre_result_frozen` and binds every
  implementation, contract, prompt, protocol, model, and evidence hash.
- The global Table 4 canonicalizer remains at SHA-256
  `e46308fe8f3b653967ede6d4a75c881c012f55732542806fea2dde5003281b31`;
  mobility uses a dedicated nonmetric canonicalizer.
- Before creating output, every stage revalidates the exact PASS audit closure,
  effective amendment, and complete implementation map. The full chain is
  propagated source -> render -> snapshot -> selection -> materialization.

## 1. Data, source, and provenance

- Bind archive bytes/SHA, revision/evidence, identity cardinality, and every ZIP
  regular member to the corresponding extracted regular file by stream SHA.
- Enforce exact archive/extracted path closure and reject symlinks, absolute or
  non-normalized paths, duplicate-normalized paths, backslashes, and traversal.
- Private roots have exact top-level closure and regular non-symlink entries.
- Candidate keys are unique 64-character lowercase hex. Source identity and
  category are private; render plans contain only key, relative scene, and hash.
- PartNet: 2,347 assets and 46 source-audit-only categories; all 2,347 URDFs
  must match the frozen exact root fixed transform; official-object binding
  remains `PROVENANCE_LIMITED`.
- PhysX: 2,024 assets, 14,096 mesh-bearing links, and 91,855 visuals; bind the
  pinned HF/archive evidence and use the official fixed rotation only for the
  camera frame, not as semantic-up evidence.

## 2. Numeric adapter and private geometry

- Parse only supported numeric OBJ vertex/face data; freeze homogeneous-vertex,
  polygon-triangulation, finite-value, and index rules with full-cohort scans.
- Apply frozen zero-pose URDF transforms, visual origins, and scales. Emit one
  deterministic label-free NPZ per candidate with exact `vertices.npy` and
  `faces.npy`, ZIP_STORED fixed metadata, `allow_pickle=False`, little-endian
  float64/int64 arrays, finite/in-range nonempty values, and content addressing.
- Instance order depends only on numeric vertices/faces/transform bytes, never
  source path, raw OBJ hash, names, materials, or comments.
- Bind source OBJ hashes, zero-pose transform hashes, array content/counts, and
  derived NPZ bytes. Scene and referenced blob sets must exactly equal actual
  regular non-symlink files with no extra or unreferenced file.
- Metamorphic path and OBJ label/material/comment edits must leave derived NPZ,
  eight PNGs, and render record byte-identical. Both full-cohort scans pass.

## 3. Shared renderer and axis

- Both arms call the same renderer and builder files. The worker reads only a
  private root, opaque key, relative scene/hash, and output. It cannot read any
  prompt, protocol, spec, amendment, metadata, category, identity, part name,
  dimension, score, URDF, or raw OBJ.
- Common canonical camera is +Z. PartNet uses identity camera frame; PhysX uses
  the pinned `Rx(+pi/2)` source-to-canonical camera/light mapping. Mesh vertices
  and score geometry are not modified. The paired algebra invariant passes.
- Eight 256x256 views use azimuths 0:45:315, elevation 22.5 degrees,
  orthographic half-frame `1.15 * AABB half-diagonal`, radius `3.2h`, near
  `0.01h`, far `8h`, fixed material/light/white background, double-sided
  rendering, and at least 32 foreground pixels per view.
- Run 1 uses one worker and run 2 four workers across the entire cohort. Every
  PNG, render record, and worker fingerprint is identical; no candidate drops.

## 4. Runtime and determinism

- Fix EGL GPU 1 and thread environment. The actual current GL context device
  UUID must equal physical GPU 1; bind driver, GL/EGL identity, Python, numpy,
  trimesh, pyrender, PyOpenGL, Pillow, Python zlib compile/runtime, and Pillow
  zlib/zlib-ng fingerprints. Builder pre/post and every worker fingerprint must
  be exact.
- Renderer/selector compile and forbidden-input scans pass, along with fixed
  smoke, camera algebra, numeric NPZ schema, and metamorphic tests.

## 5. Snapshot

- Render build locks before any prompt projection and independently revalidates
  private scenes, blobs, derivations, and NPZ files.
- The locker revalidates the full current chain and atomically publishes from a
  parent staging directory.
- Selector-visible closure is exactly candidate inventory, renders, model,
  prompt-only manifest, nonsemantic execution contract, and snapshot lock.
- The renders root and entries are regular non-symlinks; its direct-child set
  exactly equals candidate keys. Each candidate directory contains only
  `000.png` through `007.png`, with every byte hash matching the locked build.
- Inventory contains only opaque key/identity hash, relative render directory,
  eligibility/views, and render hashes. It cannot expose source, geometry,
  labels, category, bounds, or metadata.
- Prompt-only input is 18 sorted `{task_id,prompt}` rows projected exactly from
  frozen prompts. The pinned model file closure/hashes are exact. Contract data
  is nonsemantic and contains no arm, method, or source semantics.

## 6. Selector

- Both arms call the same selector/launcher/schema. The launcher validates the
  benchmark and audit in a parent phase; the child runs from the snapshot and
  can reach only its exact input closure.
- Fix model/revision, GPU 1, batch-assets 8, FP32 model features, float64
  per-view normalization/equal mean/renormalization/scoring, prompt tokenization,
  global top-1, and tie rule `(-exact_score, opaque_key)`.
- Two full replays each fresh-load model and processor. Asset/prompt embeddings,
  tokenization, index, and selection are exact. Repair and rank fallback are 0.
- Publish a complete `selection_bundle` with one parent-staging directory rename;
  interruption cannot leave a partial formal selection.

## 7. Materializer and reporting

- Read private source only after a complete selection lock. The manifest has
  exactly 18 unique task rows. Preserve the selected key; a selected-asset
  failure records that same key and failure status and cannot substitute another
  rank, repair, or target rescale.
- Publish a complete `materialization_bundle` by one parent-staging rename and
  report `COMPLETE` only for 18/18 terminal artifacts, otherwise `PARTIAL`.
- Use the dedicated nonmetric canonicalizer. Forbid `unit_scale_to_m` and all
  `*_m` fields; report only dataset-unit bounds/extents, unestablished metric
  binding, `metric_eligible=false`, numeric `N/A`, and applied scale 1.0.
- PhysX count is N/R. PartNet count may be only a supplementary name-matched
  renderable-node proxy with explicit artifact and evaluable denominators,
  never semantic ground truth.

## 8. Strict numeric N/A

- Both arms contain exactly the 20 protocol numeric constraints. Every row has
  non-applicable/non-evaluable status, `passed=null`, and a reason limited to
  unestablished metric-unit/dimension-axis mapping and forbidden target rescale.
- Summary reports 20 protocol constraints, zero evaluable, 20 N/A,
  `numeric_pass=null`, and display `N/A`. Artifact failure cannot convert N/A to
  zero or false.
- PartNet does not claim the local raw release itself is unit-sphere normalized.
  PhysX JSON dimensions are not used for materialization or scoring.

## 9. Verifiers and integrity

- Each dedicated verifier independently recomputes the current amendment and
  implementation chain, archive/source/NPZ closure, render replay/runtime,
  snapshot/model/contract, embeddings/top-1/tie, materialized artifacts,
  hierarchy, N/A rows, count boundary, and zero repair/fallback.
- Verifiers do not merely repeat lock assertions and write atomically without
  modifying locked results. Fresh verification, compile, and all frozen tests pass.
- Any generic artifact-integrity verifier or count scorer is used only within
  its declared applicability boundary and is bound to its frozen SHA.

## 10. License and paper interpretation

- PhysX evidence indicates CC-BY-NC-4.0 while the local archive lacks embedded
  license text; disclose this boundary. PartNet software MIT does not license
  the gated assets, whose noncommercial research/education and ShapeNet terms
  remain distinct; local official-object binding is provenance-limited.
- Both rows are fixed-dataset prompt-only retrieval references, not generation
  or same-prompt generation methods. Do not infer a dataset/retriever ranking
  from Artiverse 16 native views versus mobility eight synthetic views.
- Report actual completion denominators and preserve N/A, N/R, and proxy labels.

## PASS evidence

- The final pre-result audit binds all final paths and SHA-256 values, a fresh
  snapshot showing all formal roots absent and no formal process, complete
  full-corpus adapter scans, metamorphic tests, GL-context probe, compile/static
  gates, and every frozen self-test.
- The independent pre-result auditor does not run or read any formal render,
  retrieval, selection, score, or materialization output.
