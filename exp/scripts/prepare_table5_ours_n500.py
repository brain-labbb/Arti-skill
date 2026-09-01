#!/usr/bin/env python3
"""Materialize the frozen Artiverse Table 1 N=800 Table 5 manifest."""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

from table5_ours_common import (
    ManifestError,
    build_manifest,
    cleanup_new_prepare_output,
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
    parser.add_argument("--table1-manifest", required=True, type=Path)
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
    output_preexisted = True
    lock_receipt = None
    try:
        protocol = validate_canonical_protocol(arguments.protocol)
        validate_output_path(
            arguments.dataset_root,
            upstream_roots.values(),
            arguments.out,
            table1_manifest=arguments.table1_manifest,
        )
        output_preexisted = os.path.lexists(arguments.out)
        with output_lock(arguments.out, require_empty=True) as acquired_lock:
            lock_receipt = acquired_lock
            manifest = build_manifest(
                arguments.dataset_root,
                arguments.table1_manifest,
                upstream_roots,
                protocol=protocol,
                formal=True,
            )
            validate_manifest(
                manifest,
                arguments.dataset_root,
                arguments.table1_manifest,
                upstream_roots,
                protocol=protocol,
                formal=True,
            )
            publish_receipt_set(
                arguments.out,
                protocol_with_hash(protocol),
                manifest,
            )
    except (ManifestError, OSError, KeyError, json.JSONDecodeError) as error:
        if (
            not output_preexisted
            and lock_receipt is not None
            and lock_receipt.get("created_output") is True
        ):
            cleanup_new_prepare_output(arguments.out, lock_receipt)
        print(f"prepare_table5_artiverse_n800: {error}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
