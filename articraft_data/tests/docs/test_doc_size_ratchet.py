"""Size ratchet for agent-facing workflow docs.

These docs are loaded into every fork/variant agent's context: line count is
instruction memory. Growth must be a visible decision, not silent drift —
either trim elsewhere or raise the budget in doc_budget.json IN THE SAME
COMMIT (reviewable). Shrinking is always fine; lower the budget to lock gains.
"""

from __future__ import annotations

import json
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
BUDGET_FILE = Path(__file__).with_name("doc_budget.json")


def test_doc_size_ratchet() -> None:
    budgets: dict[str, int] = json.loads(BUDGET_FILE.read_text(encoding="utf-8"))
    over: list[str] = []
    for name, cap in sorted(budgets.items()):
        doc = REPO_ROOT / name
        assert doc.is_file(), f"budgeted doc missing: {name}"
        lines = len(doc.read_text(encoding="utf-8").splitlines())
        if lines > cap:
            over.append(f"{name}: {lines} > budget {cap}")
    assert not over, (
        "Agent-facing docs grew past their ratchet budget. Trim elsewhere or "
        "raise the budget in tests/docs/doc_budget.json in this same commit "
        "(reviewable):\n  " + "\n  ".join(over)
    )
