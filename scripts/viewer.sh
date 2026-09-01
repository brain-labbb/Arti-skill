#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ -f "$repo_root/.env" ]]; then
  set -a
  # shellcheck disable=SC1091
  source "$repo_root/.env"
  set +a
fi

command -v just >/dev/null 2>&1 || { echo "just is required to start the viewer" >&2; exit 1; }

case "${ARTI_RUNTIME_REPO:-template}" in
  template) runtime_repo="${ARTI_TEMPLATE_DIR:-arti-template}" ;;
  data) runtime_repo="${ARTICRAFT_DATA_REPO:-articraft_data}" ;;
  *) echo "ARTI_RUNTIME_REPO must be 'template' or 'data'" >&2; exit 2 ;;
esac

if [[ "$runtime_repo" != /* ]]; then
  runtime_repo="$repo_root/${runtime_repo#./}"
fi

exec just -d "$runtime_repo" viewer
