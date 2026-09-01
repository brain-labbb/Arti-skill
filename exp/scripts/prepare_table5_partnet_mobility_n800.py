#!/usr/bin/env python3
"""Materialize the frozen, source-bound PartNet-Mobility N=800 Table 5 manifest.

The cohort reuses all 800 items of the frozen Table 4 PartNet-Mobility manifest
in existing order (no resampling, no replacement, no outcome filtering).
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from table5_partnet_mobility_common import (
    ManifestError,
    build_manifest,
    output_lock,
    protocol_with_hash,
    publish_receipt_set,
    validate_canonical_protocol,
    validate_manifest,
    validate_output_path,
)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset-root", required=True, type=Path)
    parser.add_argument("--table4-manifest", required=True, type=Path)
    parser.add_argument("--table2-root", required=True, type=Path)
    parser.add_argument("--table3-root", required=True, type=Path)
    parser.add_argument("--table4-root", required=True, type=Path)
    parser.add_argument("--protocol", required=True, type=Path)
    parser.add_argument("--out", required=True, type=Path)
    arguments = parser.parse_args()
    upstream_roots = {
        "table2": arguments.table2_root,
        "table3": arguments.table3_root,
        "table4": arguments.table4_root,
    }
    try:
        protocol = validate_canonical_protocol(arguments.protocol)
        validate_output_path(
            arguments.dataset_root,
            [arguments.table4_manifest, *upstream_roots.values()],
            arguments.out,
        )
        with output_lock(arguments.out):
            protocol_receipt = protocol_with_hash(protocol)
            manifest = build_manifest(
                arguments.dataset_root,
                arguments.table4_manifest,
                upstream_roots,
                protocol=protocol,
            )
            validate_manifest(
                manifest,
                arguments.dataset_root,
                arguments.table4_manifest,
                upstream_roots,
                protocol=protocol,
            )
            publish_receipt_set(arguments.out, protocol_receipt, manifest)
    except (ManifestError, OSError, KeyError, json.JSONDecodeError) as error:
        print(f"prepare_table5_partnet_mobility_n800: {error}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
