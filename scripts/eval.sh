#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ -f "$repo_root/.env" ]]; then
  set -a
  # shellcheck disable=SC1091
  source "$repo_root/.env"
  set +a
fi

command -v uv >/dev/null 2>&1 || { echo "uv is required to run eval_pilot" >&2; exit 1; }
[[ $# -gt 0 ]] || { echo "usage: $0 <pilot-command> [args...]" >&2; exit 2; }

template_dir="${ARTI_TEMPLATE_DIR:-arti-template}"
if [[ "$template_dir" != /* ]]; then
  template_dir="$repo_root/${template_dir#./}"
fi

cd "$template_dir"
exec uv run --frozen python "$repo_root/eval_pilot/pilot.py" "$@"
