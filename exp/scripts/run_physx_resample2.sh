#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
ROOT="${REPO_ROOT}/exp/runtime/table5_v2_physx_resample2_seed20260830"
PREPARED="${ROOT}/prepared/manifest.json"
OUT_ROOT="${ROOT}/runs"
GPUS="${TABLE5_GPUS:-0,3,5}"
cd "${REPO_ROOT}"

python exp/scripts/table5_v2_runtime_r2.py run \
  --prepared "${PREPARED}" --simulator genesis --out "${OUT_ROOT}/genesis" \
  --datasets physx_mobility --workers 3 --gpus "${GPUS}" \
  --executable /mnt/zsn/miniconda3/envs/genesis-main/bin/python

python exp/scripts/table5_v2_runtime_r2.py run \
  --prepared "${PREPARED}" --simulator pybullet --out "${OUT_ROOT}/pybullet" \
  --datasets physx_mobility --workers 5 \
  --executable "${REPO_ROOT}/exp/.venv_low_medium/bin/python"

python exp/scripts/table5_v2_runtime_r2.py run \
  --prepared "${PREPARED}" --simulator mujoco --out "${OUT_ROOT}/mujoco" \
  --datasets physx_mobility --workers 5 \
  --executable /mnt/zsn/miniconda3/bin/python

python exp/scripts/table5_v2_aggregate_r2.py \
  --prepared "${PREPARED}" --genesis "${OUT_ROOT}/genesis" \
  --pybullet "${OUT_ROOT}/pybullet" --mujoco "${OUT_ROOT}/mujoco" \
  --out "${ROOT}/final"
