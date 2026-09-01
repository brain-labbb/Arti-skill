# Table 4 Constraints v2: real LAM baseline

Status: `completed`

Backend: `official LAM pipeline + DashScope OpenAI-compatible backend` using `qwen3.8-max`; not the exact paper backend.
Official LAM commit: `0b3a87beb8c35273a5acf8681221791aff746d8e`.
Frozen prompt SHA-256: `0b46c15c65c46550cc25654ccca10e4de87f1b97e6bde89796ea3cc9b439065e`.
Benchmark repair budget: `0`; official native critique/fixer loops remained enabled.
Tasks: `18/18` attempted, `8` pipeline success, `10` timeout, `0` failed.
Canonical artifacts: `8/18`.
Pipeline-success tasks: `T4C001`, `T4C003`, `T4C004`, `T4C007`, `T4C008`, `T4C015`, `T4C017`, `T4C018`.
Timeout tasks: `T4C002`, `T4C005`, `T4C006`, `T4C009`, `T4C010`, `T4C011`, `T4C012`, `T4C013`, `T4C014`, `T4C016`.
Four timeout checkpoints (`T4C011`, `T4C012`, `T4C013`, `T4C016`) are supplementary only and were not canonicalized or scored.
Final manifest SHA-256: `80416d513345c76ee4c3c93bdf13172d227db6ed57d7aa7c0b020151c7e1796a`.
Structured scorer byte-identical across two runs: `True`.
Numeric-primary scorer byte-identical across two runs: `True`.
Integrity verifier byte-identical across two runs: `True`.
Formal input/output tokens: `925296` / `760693`.
The recorded `$32.073750` is only the official LAM fallback estimate because upstream has no Qwen pricing; it is not a measured DashScope bill.

## Numeric-primary score (paper main panel)

```json
{
  "artifact_count": 8,
  "benchmark_id": "table4_constraints_v2",
  "conditional_accuracy": 1.0,
  "constraints": 20,
  "count_pass": null,
  "coverage": 0.5,
  "measurable": 10,
  "method": "lam",
  "numeric_pass": "10/20",
  "panel": "cad_numeric",
  "passed": 10,
  "prompt_manifest_sha256": "0b46c15c65c46550cc25654ccca10e4de87f1b97e6bde89796ea3cc9b439065e",
  "satisfaction": 0.5,
  "schema_version": 2,
  "task_count": 18
}
```

## Structured score (named-node count proxy; supplementary)

```json
{
  "artifact_count": 8,
  "benchmark_id": "table4_constraints_v2",
  "conditional_accuracy": 1.0,
  "constraints": 52,
  "count_pass": "11/32",
  "coverage": 0.40384615384615385,
  "measurable": 21,
  "method": "lam",
  "numeric_pass": "10/20",
  "panel": "structured_main",
  "passed": 21,
  "prompt_manifest_sha256": "0b46c15c65c46550cc25654ccca10e4de87f1b97e6bde89796ea3cc9b439065e",
  "satisfaction": 0.40384615384615385,
  "schema_version": 2,
  "task_count": 18
}
```
