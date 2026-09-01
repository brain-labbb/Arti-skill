#!/usr/bin/env bash
set -euo pipefail

BUNDLE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_BIN="${PYTHON_BIN:-python3}"
MANIFEST="${MANIFEST:-${1:-manifest.eligible.jsonl}}"
RESULTS_DIR="${RESULTS_DIR:-${2:-results}}"
WORKERS="${WORKERS:-32}"
TASK="${TASK:-both}"
SHARD_COUNT="${SHARD_COUNT:-1}"
SHARD_INDEX="${SHARD_INDEX:-0}"

export OMP_NUM_THREADS="${OMP_NUM_THREADS:-1}"
export OPENBLAS_NUM_THREADS="${OPENBLAS_NUM_THREADS:-1}"
export MKL_NUM_THREADS="${MKL_NUM_THREADS:-1}"
export NUMEXPR_NUM_THREADS="${NUMEXPR_NUM_THREADS:-1}"

EXTRA_ARGS=()
if [[ -n "${MAX_ASSETS:-}" ]]; then
  EXTRA_ARGS+=(--max-assets "${MAX_ASSETS}")
fi

"${PYTHON_BIN}" "${BUNDLE_DIR}/full_eval.py" \
  --manifest "${MANIFEST}" \
  --protocol "${BUNDLE_DIR}/protocol.json" \
  --out "${RESULTS_DIR}" \
  --task "${TASK}" \
  --workers "${WORKERS}" \
  --shard-count "${SHARD_COUNT}" \
  --shard-index "${SHARD_INDEX}" \
  "${EXTRA_ARGS[@]}"

if [[ "${AGGREGATE:-0}" == "1" ]]; then
  "${PYTHON_BIN}" "${BUNDLE_DIR}/aggregate.py" \
    --manifest "${MANIFEST}" \
    --results "${RESULTS_DIR}" \
    --out "${RESULTS_DIR}/aggregate"
fi
