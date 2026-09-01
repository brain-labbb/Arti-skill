#!/usr/bin/env python3
"""Prepare a new Table 5 manifest with revision-2 metric semantics.

Asset preparation is delegated unchanged to the frozen v1 implementation.  This
entrypoint only versions and binds the revised runtime and aggregation contract.
"""

from __future__ import annotations

from copy import deepcopy
from pathlib import Path
import sys
from typing import Any, Mapping, Sequence


SCRIPT_PATH = Path(__file__).resolve()
SCRIPT_DIR = SCRIPT_PATH.parent
REPO_ROOT = SCRIPT_PATH.parents[2]
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import table5_v2_prepare as _core  # noqa: E402
import table5_v2_runtime_r2 as _runtime_r2  # noqa: E402


PROTOCOL_SCHEMA = _runtime_r2.PROTOCOL_SCHEMA
PROTOCOL_ID = _runtime_r2.PROTOCOL_ID
METRIC_SEMANTICS_ID = _runtime_r2.METRIC_SEMANTICS_ID
DEFAULT_COHORT = (
    REPO_ROOT
    / "exp/runtime/table5_v2_core200_five_full_release_articraft10787_infinigen_paired_official/cohort_manifest.json"
)
DEFAULT_OUTPUT = (
    REPO_ROOT
    / "exp/runtime/table5_v2_core200_prepared_five_full_release_articraft10787_infinigen_paired_official_metrics_r2"
)

_ORIGINAL_PROTOCOL = _core._protocol
_INSTALLED = False


def _protocol(cohort: Mapping[str, Any]) -> dict[str, Any]:
    protocol = deepcopy(_ORIGINAL_PROTOCOL(cohort))
    stable_steps = int(protocol["runtime"]["passive_settling"]["steps"])
    protocol["schema_version"] = PROTOCOL_SCHEMA
    protocol["protocol_id"] = PROTOCOL_ID
    protocol["v2_metrics"].update(
        {
            "metric_semantics_id": METRIC_SEMANTICS_ID,
            "import_success": (
                "manifest-bound source returns successfully from the simulator's "
                "native asset-load API; mapping, physics application, stepping, and "
                "canonical structure preservation are separate diagnostics"
            ),
            "stable_rollout": (
                "after physics policy application, every imported asset completes a "
                "fixed-step zero-force passive rollout with finite observed states "
                "and poses; no eligible joint is required"
            ),
            "passive_stable_rollout": {
                "steps": stable_steps,
                "control": "zero_applied_joint_force",
                "initial_joint_position": (
                    "declared finite bounded midpoint, otherwise zero"
                ),
                "requires_nonempty_observed_link_set": True,
                "requires_mapping_unchanged": True,
            },
            "trajectory_coverage": (
                "successfully evaluated bounded revolute/prismatic joints / all "
                "declared revolute/prismatic joints"
            ),
        }
    )
    runtime_script = SCRIPT_PATH.with_name("table5_v2_runtime_r2.py")
    runtime_core = SCRIPT_PATH.with_name("table5_v2_runtime.py")
    compat_script = SCRIPT_PATH.with_name("table5_v2_runtime_compat.py")
    aggregate_script = SCRIPT_PATH.with_name("table5_v2_aggregate_r2.py")
    for path in (runtime_script, runtime_core, compat_script, aggregate_script):
        if not path.is_file():
            raise _core.PrepareError(f"revision-2 implementation is missing: {path}")
    implementation = protocol["implementation"]
    implementation.update(
        {
            "prepare_script": str(SCRIPT_PATH),
            "prepare_script_sha256": _core.sha256_file(SCRIPT_PATH),
            "prepare_core_script": str(Path(_core.__file__).resolve()),
            "prepare_core_script_sha256": _core.sha256_file(
                Path(_core.__file__).resolve()
            ),
            "v2_runtime_script": str(runtime_script),
            "v2_runtime_script_sha256": _core.sha256_file(runtime_script),
            "v2_runtime_core_script": str(runtime_core),
            "v2_runtime_core_script_sha256": _core.sha256_file(runtime_core),
            "genesis_compat_script": str(compat_script),
            "genesis_compat_script_sha256": _core.sha256_file(compat_script),
            "aggregate_script": str(aggregate_script),
            "aggregate_script_sha256": _core.sha256_file(aggregate_script),
        }
    )
    protocol["protocol_sha256"] = _core.canonical_sha256(
        protocol, exclude_fields=("protocol_sha256",)
    )
    return protocol


def install() -> None:
    global _INSTALLED
    if _INSTALLED:
        return
    _core._protocol = _protocol
    _core.DEFAULT_COHORT = DEFAULT_COHORT
    _core.DEFAULT_OUTPUT = DEFAULT_OUTPUT
    _INSTALLED = True


def main(argv: Sequence[str] | None = None) -> int:
    install()
    return _core.main(argv)


if __name__ == "__main__":
    raise SystemExit(main())
