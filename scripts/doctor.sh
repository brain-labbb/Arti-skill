#!/usr/bin/env bash
set -u

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
failures=0

pass() { printf 'ok: %s\n' "$1"; }
warn() { printf 'warning: %s\n' "$1" >&2; }
fail() { printf 'error: %s\n' "$1" >&2; failures=$((failures + 1)); }

for command_name in git python3; do
  if command -v "$command_name" >/dev/null 2>&1; then
    pass "$command_name is available"
  else
    fail "$command_name is required"
  fi
done

for command_name in just uv; do
  if command -v "$command_name" >/dev/null 2>&1; then
    pass "$command_name is available"
  else
    warn "$command_name is optional until dependency setup is requested"
  fi
done

if command -v git-lfs >/dev/null 2>&1; then
  pass "git-lfs is available"
else
  fail "git-lfs is required to hydrate articraft_data"
fi

if git -C "$repo_root" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  pass "top-level Git repository is initialized"
else
  fail "top-level Git repository is not initialized"
fi

for submodule in arti-template articraft_data; do
  child="$repo_root/$submodule"
  if ! git -C "$child" rev-parse --verify HEAD >/dev/null 2>&1; then
    fail "$submodule is not initialized"
    continue
  fi

  entry="$(git -C "$repo_root" ls-files --stage -- "$submodule")"
  mode="$(awk '{print $1}' <<<"$entry")"
  expected="$(awk '{print $2}' <<<"$entry")"
  actual="$(git -C "$child" rev-parse HEAD)"

  if [[ "$mode" != "160000" ]]; then
    fail "$submodule is not recorded as a gitlink"
  elif [[ "$expected" != "$actual" ]]; then
    fail "$submodule HEAD differs from the top-level gitlink"
  else
    pass "$submodule matches ${actual:0:12}"
  fi
done

[[ -f "$repo_root/arti-template/pyproject.toml" ]] \
  && pass "template project marker exists" \
  || fail "arti-template/pyproject.toml is missing"
[[ -f "$repo_root/articraft_data/.gitattributes" ]] \
  && pass "data LFS policy exists" \
  || fail "articraft_data/.gitattributes is missing"
[[ -f "$repo_root/eval_pilot/pilot.py" ]] \
  && pass "evaluation pilot exists" \
  || fail "eval_pilot/pilot.py is missing"

if git -C "$repo_root" ls-files --error-unmatch .env >/dev/null 2>&1; then
  fail ".env must never be tracked"
else
  pass ".env is not tracked"
fi

if (( failures > 0 )); then
  printf '%d doctor check(s) failed\n' "$failures" >&2
  exit 1
fi

echo "all required pipeline checks passed"
