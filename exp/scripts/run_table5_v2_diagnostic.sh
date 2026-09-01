#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
DIAG_ROOT="${REPO_ROOT}/exp/runtime/table5_v2_articraft_diagnostic_resample_20260830"
PREPARED="${DIAG_ROOT}/prepared/manifest.json"
OUT_ROOT="${DIAG_ROOT}/runs"
GPUS="${TABLE5_GPUS:-1,4,7}"

cd "${REPO_ROOT}"

python exp/scripts/table5_v2_prepare_diagnostic.py --verify "${PREPARED}" --skip-file-hashes

python exp/scripts/table5_v2_runtime_r2.py run \
  --prepared "${PREPARED}" \
  --simulator genesis \
  --out "${OUT_ROOT}/genesis" \
  --datasets articraft_10k \
  --workers 3 \
  --gpus "${GPUS}" \
  --executable /mnt/zsn/miniconda3/envs/genesis-main/bin/python

python exp/scripts/table5_v2_runtime_r2.py run \
  --prepared "${PREPARED}" \
  --simulator pybullet \
  --out "${OUT_ROOT}/pybullet" \
  --datasets articraft_10k \
  --workers 5 \
  --executable "${REPO_ROOT}/exp/.venv_low_medium/bin/python"

python exp/scripts/table5_v2_runtime_r2.py run \
  --prepared "${PREPARED}" \
  --simulator mujoco \
  --out "${OUT_ROOT}/mujoco" \
  --datasets articraft_10k \
  --workers 5 \
  --executable /mnt/zsn/miniconda3/bin/python

python exp/scripts/table5_v2_aggregate_r2.py \
  --prepared "${PREPARED}" \
  --genesis "${OUT_ROOT}/genesis" \
  --pybullet "${OUT_ROOT}/pybullet" \
  --mujoco "${OUT_ROOT}/mujoco" \
  --out "${DIAG_ROOT}/final"
