#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ -f "$repo_root/.env" ]]; then
  set -a
  # shellcheck disable=SC1091
  source "$repo_root/.env"
  set +a
fi

git -C "$repo_root" submodule sync --recursive
GIT_LFS_SKIP_SMUDGE=1 git -C "$repo_root" submodule update --init --recursive

if command -v git-lfs >/dev/null 2>&1; then
  git -C "$repo_root" lfs install --local >/dev/null
  git -C "$repo_root/articraft_data" lfs install --local >/dev/null
elif [[ "${ARTI_PULL_LFS:-0}" == "1" ]]; then
  echo "git-lfs is required when ARTI_PULL_LFS=1" >&2
  exit 1
else
  echo "warning: git-lfs is not installed; data payloads remain unavailable" >&2
fi

if [[ "${ARTI_PULL_LFS:-0}" == "1" ]]; then
  lfs_include="${ARTI_LFS_INCLUDE:-data/records/**,picture/**}"
  git -C "$repo_root/articraft_data" -c lfs.fetchexclude= lfs pull --include="$lfs_include"
fi

if [[ "${ARTI_SETUP_TEMPLATE:-0}" == "1" ]]; then
  command -v just >/dev/null 2>&1 || { echo "just is required for template setup" >&2; exit 1; }
  just -d "$repo_root/arti-template" setup
fi

if [[ "${ARTI_SETUP_DATA:-0}" == "1" ]]; then
  command -v just >/dev/null 2>&1 || { echo "just is required for data setup" >&2; exit 1; }
  just -d "$repo_root/articraft_data" setup
fi

echo "pipeline checkout is ready"
