#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec python3 -m unittest discover -s "$repo_root/tests/e2e" -p 'test_*.py' -v
