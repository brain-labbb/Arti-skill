#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
PREPARED="${REPO_ROOT}/exp/runtime/table5_v2_core200_prepared_five_full_release_articraft10787_infinigen_paired_official_metrics_r2/manifest.json"
OUT_ROOT="${REPO_ROOT}/exp/runtime/table5_v2_r2_formal_eight_datasets"
GPUS="${TABLE5_GPUS:-0,1,2,3,4}"
SESSION_NAME="${TABLE5_TMUX_SESSION:-table5-v2-r2-formal}"
LOG_PATH="${OUT_ROOT}/tmux.log"

cd "${REPO_ROOT}"

python exp/scripts/table5_v2_prepare_r2.py --verify "${PREPARED}"

if [[ "${TABLE5_RUN_IN_TMUX:-0}" != "1" ]]; then
  if ! command -v tmux >/dev/null 2>&1; then
    echo "tmux is not installed" >&2
    exit 2
  fi
  if tmux has-session -t "${SESSION_NAME}" 2>/dev/null; then
    echo "tmux session already exists: ${SESSION_NAME}" >&2
    exit 2
  fi
  mkdir -p "${OUT_ROOT}"
  printf -v TMUX_COMMAND \
    'set -o pipefail; env TABLE5_RUN_IN_TMUX=1 TABLE5_GPUS=%q bash %q 2>&1 | tee %q' \
    "${GPUS}" "$0" "${LOG_PATH}"
  tmux new-session -d -s "${SESSION_NAME}" -c "${REPO_ROOT}" \
    "bash -lc $(printf '%q' "${TMUX_COMMAND}")"
  echo "Started tmux session: ${SESSION_NAME}"
  echo "Attach: tmux attach -t ${SESSION_NAME}"
  echo "Log: ${LOG_PATH}"
  exit 0
fi

# Verify again inside the detached session before any formal worker starts.
python exp/scripts/table5_v2_prepare_r2.py --verify "${PREPARED}"

python exp/scripts/run_table5_v2_native.py \
  --stage all \
  --prepared "${PREPARED}" \
  --out-root "${OUT_ROOT}" \
  --workers 5 \
  --gpus "${GPUS}"
