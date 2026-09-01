# T4 distributional editability status

`prepare_t4_distributional_protocol.py` resolves the mismatch between the existing 18-task
single-seed slice and the desired 18×16 propagation experiment. It freezes 16 seeds that satisfy a
real edit precondition for every task. Additive count edits increment the current count by one;
replacement edits exclude seeds already containing the target. This avoids scoring a no-op as a
successful edit.

Prepare the 288-case manifest without compiling geometry:

```bash
arti-template/.venv/bin/python \
  exp/scripts/prepare_t4_distributional_protocol.py
```

The resulting `readiness.json` intentionally remains blocked. Before compiling edited artifacts,
an independent reviewer must freeze, for every task, the target roles, allowed dependent roles, and
true non-target roles. A pre-edit regression manifest and the two-reviewer/adjudication workflow are
also required. Without these inputs, Target/Anchor/Scale can be partially proxied, but Non-Target
Preservation, Locality, Regression Preservation, and Final Pass cannot be claimed.
