#!/usr/bin/env python3
"""Bind CPU affinity before importing and executing a Python child script."""

from __future__ import annotations

import os
import sys


CPU_AFFINITY_ENV = "LAM_GENESIS_CPU_AFFINITY"
AFFINITY_RECEIPT_ENV = "TABLE4A_EARLY_CPU_AFFINITY_RECEIPT"


def parse_cpu_affinity(raw: str) -> list[int]:
    if not raw or raw.strip() != raw:
        raise ValueError(f"invalid {CPU_AFFINITY_ENV}: {raw!r}")
    tokens = raw.split(",")
    if any(
        not token
        or not token.isascii()
        or not token.isdecimal()
        or token != str(int(token))
        for token in tokens
    ):
        raise ValueError(f"invalid {CPU_AFFINITY_ENV}: {raw!r}")
    cpus = [int(token) for token in tokens]
    if any(cpu < 0 for cpu in cpus) or cpus != sorted(set(cpus)):
        raise ValueError(f"non-canonical {CPU_AFFINITY_ENV}: {raw!r}")
    return cpus


def bind_from_environment() -> list[int]:
    requested = parse_cpu_affinity(os.environ.get(CPU_AFFINITY_ENV, ""))
    available = sorted(int(cpu) for cpu in os.sched_getaffinity(0))
    if any(cpu not in available for cpu in requested):
        raise RuntimeError(
            f"requested affinity is unavailable: requested={requested}, available={available}"
        )
    os.sched_setaffinity(0, set(requested))
    observed = sorted(int(cpu) for cpu in os.sched_getaffinity(0))
    if observed != requested:
        raise RuntimeError(f"CPU affinity readback mismatch: {observed} != {requested}")
    os.environ[AFFINITY_RECEIPT_ENV] = (
        f"pid={os.getpid()};cpus={','.join(str(cpu) for cpu in observed)}"
    )
    return observed


def main() -> int:
    if len(sys.argv) < 3 or sys.argv[1] != "--":
        print("usage: exec_with_cpu_affinity.py -- CHILD_SCRIPT [ARG ...]", file=sys.stderr)
        return 2
    try:
        bind_from_environment()
    except Exception as exc:  # noqa: BLE001
        print(f"early CPU affinity binding failed: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 2
    os.execv(sys.executable, [sys.executable, *sys.argv[2:]])
    raise AssertionError("os.execv returned unexpectedly")


if __name__ == "__main__":
    raise SystemExit(main())
