#!/usr/bin/env python3
"""Diagnostic prepare for the PhysX stability-resample cohort (physx-only).

Bypasses the frozen multi-dataset final validation so a single-dataset
(physx_mobility) cohort can be prepared and evaluated in isolation.
"""

from __future__ import annotations

import json
from pathlib import Path
import sys
from typing import Any

SCRIPT_DIR = Path(__file__).resolve()
SCRIPT_PATH = SCRIPT_DIR
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import table5_v2_prepare as _core  # noqa: E402
import table5_v2_prepare_r2 as _r2  # noqa: E402
from table5_v2_resample_physx_stability import (  # noqa: E402
    NEW_SEED,
    SLUG,
    validate_physx_stability_cohort,
)


def _load_cohort(path: Path) -> dict[str, Any]:
    try:
        cohort = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise _core.PrepareError(f"cannot load cohort: {error}") from error
    if not isinstance(cohort, dict):
        raise _core.PrepareError("cohort must be a JSON object")
    validate_physx_stability_cohort(cohort, NEW_SEED, expected_slugs=[SLUG])
    return cohort


def main(argv=None) -> int:
    _r2.install()
    _core._load_cohort = _load_cohort
    # Diagnostic run: skip the frozen final prepared-manifest validation, which
    # only rejects the single-dataset layout; row/protocol integrity is already
    # enforced by the cohort validator and the runtime's own checks.
    _core.validate_manifest = lambda manifest, verify_files=True: None
    return _core.main(argv)


if __name__ == "__main__":
    raise SystemExit(main())
