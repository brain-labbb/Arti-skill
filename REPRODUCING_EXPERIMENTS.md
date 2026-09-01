# Reproducing `arti-skill` experiments

The GitHub repository contains the source code, experiment protocols, paper
tables, manifests, and checksums. It intentionally does not contain local
virtual environments, caches, rendered images, the full PV-A extraction, or
multi-gigabyte SQLite outputs. Those payloads are restored from pinned data
revisions and separately archived result files.

The GitHub pipeline snapshot also excludes the unrelated local workspaces
`blender_gallery/`, `configs/`, `data_check/`,
`embodied_gen_hinge_physics/`, and `embodied_gen_part_physics/`. The working
copy may still contain them, but they are outside this backup's scope.

## Repository and submodules

Clone the pipeline and its pinned submodule commits:

```bash
git clone --recurse-submodules <pipeline-repo-url> arti-skill
cd arti-skill
just bootstrap
just doctor
```

The top-level commit records these submodule commits:

| path | pinned commit |
| --- | --- |
| `arti-template` | `62d07cb2cb3b2fcc52aa2ca093d13c6958cc53f6` |
| `articraft_data` | `a657de7b5d14b2167dc0e453a1f3de6d9c3a05cb` |

The relative URLs in `.gitmodules` require both submodule repositories to be
published beside the pipeline repository in the same hosting namespace. If
their GitHub repository names differ, update `.gitmodules` before publishing.

## PV-A source data

The exact source contract used by the original Table4 run is the ModelScope
dataset `brain233333/PV-A` at revision:

```text
e878671bf7b063808b26994fba0fa6038d82a232
```

The current repair revision is `d49f44eba3692b6352ed14f55d1b46ebcb89635e`.
It changes `single_wheelbarrow` and must not be used when reproducing the old
Table4 contract. At the pinned revision its archive checksum is:

```text
single_wheelbarrow.tar.zst
3f94e109a7a97b39ba3bfbe490d2fcbad659e9e5b234eb7d65040f1ebb6a79db
```

Fetch the archives with Git LFS and extract them into a local package root.
The complete extraction is about 311 GB; the compressed archives are much
smaller and can be kept as the durable source backup.

```bash
DATA_REPO=/data/PV-A-modelscope
EXTRACTED=/data/PV-A-extracted
git clone --no-checkout https://www.modelscope.cn/datasets/brain233333/PV-A.git "$DATA_REPO"
git -C "$DATA_REPO" checkout e878671bf7b063808b26994fba0fa6038d82a232
git -C "$DATA_REPO" lfs pull --include='archives/**,manifest.csv,README.md,library_snapshot.json'
mkdir -p "$EXTRACTED"
for archive in "$DATA_REPO"/archives/*.tar.zst; do
  name=${archive##*/}
  slug=${name%.tar.zst}
  slug=${slug%_part??}
  mkdir -p "$EXTRACTED/$slug"
  tar --zstd -xf "$archive" -C "$EXTRACTED/$slug"
done
sha256sum "$DATA_REPO/archives/single_wheelbarrow.tar.zst"
```

## Table4 v4 continuation

The retained partial result is not a formal release, but it is resumable. The
tracked receipt and handoff metadata record the following immutable values:

```text
completed assets: 152064 / 302440
result database SHA-256: 234ec794d331926b22eb576db045332a1b93cba8e3e852b57344b419c4984b7d
partial archive SHA-256: 71423f8aec70b2d17ed8bc23db9c118471267c46f827dd95cb3d488292a963fb
```

The 5.4 GB archive is intentionally stored outside GitHub. Restore it into
the pipeline checkout and verify it before extracting:

```bash
sha256sum -c exp/backups/table4_v4_partial_20260901T091955Z.tar.zst.sha256
tar --zstd -xf exp/backups/table4_v4_partial_20260901T091955Z.tar.zst -C .
```

After the PV-A mirror exists at the path recorded by the run manifest, resume
with the original frozen parameters:

```bash
tmux -S /tmp/codex_table4.sock new-session -d -s pva_v4_full \
  'cd /path/to/arti-skill && exec arti-template/.venv/bin/python -u exp/scripts/run_pva_table4_simulator_free_full_release.py \
  --source-evaluation exp/runtime/pva_table1234_full_release_20260826/evaluation \
  --output exp/runtime/pva_table4_simulator_free_full_release_v4 \
  --workers 32 --batch-size 128 --timeout-seconds 600 --resume \
  --package-root-override /tmp/pva_table4_local_mirror_20260827/extracted \
  > exp/runtime/pva_table4_simulator_free_full_release_v4.tmux.log 2>&1'
```

Do not change the source evaluation, roster, code hashes, contact registry, or
package root contents during a resume. The runner fails closed on package
drift. The old `single_wheelbarrow` archive must be used for an exact run.

## Environment

Do not back up `.venv` directories. Recreate them from the committed lockfiles
and requirement files using `just setup-template`, `just setup-data`, and the
documented `uv.lock` files. Keep credentials only in a local `.env`; use
`.env.example` as the non-secret template.
