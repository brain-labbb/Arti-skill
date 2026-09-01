#!/usr/bin/env python3
"""Run Table 5 v2 with a receipt-bound Genesis fixed-root compatibility patch.

The frozen runtime and evaluator sources remain unchanged.  This entrypoint
loads them, validates their frozen hashes, and then applies a narrowly scoped
adapter patch for Genesis importers that omit a coincident dummy URDF root.
"""

from __future__ import annotations

from copy import deepcopy
import json
import math
from pathlib import Path
import sys
from typing import Any, Mapping, Sequence


SCRIPT_PATH = Path(__file__).resolve()
SCRIPT_DIR = SCRIPT_PATH.parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import table5_v2_runtime as _runtime  # noqa: E402


PATCH_ID = "genesis-coincident-fixed-root-mapping-v1"
FROZEN_RUNTIME_PATH = _runtime.SCRIPT_PATH
_ORIGINAL_VALIDATE_V2_PROTOCOL = _runtime._validate_v2_protocol
_ORIGINAL_V2_IDENTITY = _runtime._v2_identity
_ORIGINAL_MAPPING_RECEIPT = _runtime._mapping_receipt
_ORIGINAL_GENESIS_INIT = _runtime._legacy.GenesisAdapter.__init__
_ORIGINAL_DYNAMIC_GENESIS_INIT = _runtime._runtime.DynamicGenesisAdapter.__init__
_INSTALLED = False


def _zero_xyz_or_rpy(value: Any) -> bool:
    return bool(
        isinstance(value, list)
        and len(value) == 3
        and all(
            isinstance(item, (int, float))
            and not isinstance(item, bool)
            and math.isfinite(float(item))
            and abs(float(item)) <= 1.0e-12
            for item in value
        )
    )


def fixed_root_mapping(
    row: Mapping[str, Any], observed_link_names: Sequence[str]
) -> dict[str, Any] | None:
    """Return a deterministic mapping for one omitted coincident fixed root."""

    tree = row.get("joint_tree")
    if not isinstance(tree, Mapping):
        return None
    roots = tree.get("root_links")
    joints = tree.get("joints")
    if not isinstance(roots, list) or len(roots) != 1 or not isinstance(joints, list):
        return None
    root_name = roots[0]
    if not isinstance(root_name, str) or root_name in set(observed_link_names):
        return None
    outgoing = [
        joint
        for joint in joints
        if isinstance(joint, Mapping) and joint.get("parent") == root_name
    ]
    if len(outgoing) != 1:
        return None
    joint = outgoing[0]
    child_name = joint.get("child")
    if (
        joint.get("type") != "fixed"
        or not isinstance(joint.get("name"), str)
        or not isinstance(child_name, str)
        or child_name not in set(observed_link_names)
        or not _zero_xyz_or_rpy(joint.get("origin_xyz"))
        or not _zero_xyz_or_rpy(joint.get("origin_rpy"))
    ):
        return None
    receipt = {
        "patch_id": PATCH_ID,
        "canonical_root_link_name": root_name,
        "surrogate_observed_link_name": child_name,
        "canonical_fixed_joint_name": joint["name"],
        "criterion": "single coincident fixed child of an omitted canonical root",
    }
    receipt["receipt_sha256"] = _runtime._runtime.canonical_sha256(
        receipt, exclude_fields=("receipt_sha256",)
    )
    return receipt


def _bind_fixed_root(self: Any, row: Mapping[str, Any]) -> None:
    self.table5_v2_fixed_root_mapping = fixed_root_mapping(row, self.links)
    if self.root_name not in self.links and self.table5_v2_fixed_root_mapping is None:
        raise _runtime.RuntimeErrorV2(
            "Genesis omitted the canonical root without an eligible fixed-root mapping"
        )
    if self.table5_v2_fixed_root_mapping is not None:
        self.warnings.append(
            "reconstructed omitted coincident fixed root: "
            f"{self.table5_v2_fixed_root_mapping['canonical_root_link_name']}"
        )


def _genesis_init(
    self: Any,
    raw_urdf_path: Path,
    row: dict[str, Any],
    protocol: dict[str, Any],
    parent_gpu_receipt: dict[str, Any],
) -> None:
    _ORIGINAL_GENESIS_INIT(
        self, raw_urdf_path, row, protocol, parent_gpu_receipt
    )
    _bind_fixed_root(self, row)


def _dynamic_genesis_init(
    self: Any,
    raw_urdf_path: Path,
    row: dict[str, Any],
    protocol: dict[str, Any],
) -> None:
    _ORIGINAL_DYNAMIC_GENESIS_INIT(self, raw_urdf_path, row, protocol)
    _bind_fixed_root(self, row)


def _genesis_link_poses(self: Any) -> dict[str, dict[str, list[float]]]:
    world = {
        name: {
            "translation": self._values(link.get_pos(relative=False)),
            "rotation": self._values(link.get_quat(relative=False)),
        }
        for name, link in self.links.items()
    }
    mapping = getattr(self, "table5_v2_fixed_root_mapping", None)
    if self.root_name not in world and isinstance(mapping, Mapping):
        surrogate = str(mapping["surrogate_observed_link_name"])
        if surrogate not in world:
            raise _runtime.RuntimeErrorV2(
                f"Genesis fixed-root surrogate pose missing: {surrogate}"
            )
        world[self.root_name] = deepcopy(world[surrogate])
    return _runtime._legacy.relative_link_poses(world, self.root_name)


def _mapping_receipt(adapter: Any, row: Mapping[str, Any]) -> dict[str, Any]:
    receipt = _ORIGINAL_MAPPING_RECEIPT(adapter, row)
    fixed_root = getattr(adapter, "table5_v2_fixed_root_mapping", None)
    if isinstance(fixed_root, Mapping):
        receipt["fixed_root_mapping"] = deepcopy(dict(fixed_root))
        receipt["receipt_sha256"] = _runtime._runtime.canonical_sha256(
            receipt, exclude_fields=("receipt_sha256",)
        )
    return receipt


def _validate_v2_protocol(protocol: Mapping[str, Any]) -> None:
    # Validate the exact frozen implementation first.  SCRIPT_PATH is replaced
    # only so worker launch and identity receipts name this compatibility layer.
    active_path = _runtime.SCRIPT_PATH
    try:
        _runtime.SCRIPT_PATH = FROZEN_RUNTIME_PATH
        _ORIGINAL_VALIDATE_V2_PROTOCOL(protocol)
    finally:
        _runtime.SCRIPT_PATH = active_path


def _v2_identity(*args: Any, **kwargs: Any) -> dict[str, Any]:
    identity = _ORIGINAL_V2_IDENTITY(*args, **kwargs)
    identity["runtime_compatibility"] = {
        "patch_id": PATCH_ID,
        "patch_source_sha256": _runtime.sha256_file(SCRIPT_PATH),
        "frozen_runtime_source_sha256": _runtime.sha256_file(FROZEN_RUNTIME_PATH),
    }
    return identity


def install() -> None:
    global _INSTALLED
    if _INSTALLED:
        return
    _runtime._legacy.GenesisAdapter.__init__ = _genesis_init
    _runtime._legacy.GenesisAdapter.link_poses = _genesis_link_poses
    _runtime._runtime.DynamicGenesisAdapter.__init__ = _dynamic_genesis_init
    _runtime._mapping_receipt = _mapping_receipt
    _runtime._validate_v2_protocol = _validate_v2_protocol
    _runtime.SCRIPT_PATH = SCRIPT_PATH
    _runtime._runtime._identity = _v2_identity
    _INSTALLED = True


def main(argv: Sequence[str] | None = None) -> int:
    install()
    return _runtime.main(argv)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except _runtime.RuntimeErrorV2 as error:
        print(f"table5_v2_runtime_compat: {error}", file=sys.stderr)
        raise SystemExit(2)
