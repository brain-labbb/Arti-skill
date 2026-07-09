#!/usr/bin/env bash
# Weekly report-only motion-QC census over the whole template registry.
#
# Produces reports/motion_census_<date>.jsonl and prints the ranked offender
# report. Report-only: never touches sweep streak state or any template.
# Compare against the previous dated JSONL to spot regressions (a template
# that was clean last week and fails now was broken by an SDK/template edit).
#
# Usage (from the repo root):
#   bash scripts/motion_census_weekly.sh [extra motion_census.py args]
set -euo pipefail

cd "$(dirname "$0")/.."
STAMP="$(date +%F)"
OUT="reports/motion_census_${STAMP}.jsonl"

OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 NUMEXPR_NUM_THREADS=1 \
    .venv/bin/python scripts/motion_census.py \
    --seeds 6 --max-workers 6 --compile-timeout 180 \
    --out "${OUT}" --resume "$@"

PREV="$(ls reports/motion_census_*.jsonl 2>/dev/null | grep -v "${STAMP}" | sort | tail -1 || true)"
if [ -n "${PREV}" ]; then
    echo ""
    echo "=== Delta vs ${PREV} ==="
    .venv/bin/python - "$PREV" "$OUT" <<'EOF'
import json, sys

def load(path):
    rows = {}
    for line in open(path, encoding="utf-8"):
        line = line.strip()
        if line:
            r = json.loads(line)
            rows[r["slug"]] = r
    return rows

prev, cur = load(sys.argv[1]), load(sys.argv[2])
def bad(r):
    return r.get("status") != "ok" or r.get("motion_qc_fail_count", 0) > 0 or r.get("other_fail_count", 0) > 0
regressed = sorted(s for s in cur if s in prev and not bad(prev[s]) and bad(cur[s]))
recovered = sorted(s for s in cur if s in prev and bad(prev[s]) and not bad(cur[s]))
print(f"regressed (clean -> failing): {len(regressed)}")
for s in regressed:
    print(f"  {s}")
print(f"recovered (failing -> clean): {len(recovered)}")
for s in recovered:
    print(f"  {s}")
EOF
fi
