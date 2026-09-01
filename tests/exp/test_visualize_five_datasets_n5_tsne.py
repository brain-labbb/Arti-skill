from __future__ import annotations

import importlib.util
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "exp/scripts/visualize_five_datasets_n5_tsne.py"


def _module():
    name = "_test_visualize_five_datasets_n5_tsne"
    spec = importlib.util.spec_from_file_location(name, SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def test_defaults_include_lam_and_five_dataset_totals() -> None:
    module = _module()
    args = module.build_argument_parser().parse_args([])
    specs = module._dataset_specs(args)
    assert [spec.key for spec in specs] == [
        "pva", "artiverse", "articraft10k", "partnet_mobility", "lam"
    ]
    lam = specs[-1]
    assert lam.name == "LAM"
    assert lam.expected_classes == 660
    assert lam.expected_samples == 1279
    assert lam.expected_strict_classes == 93
    assert args.output_dir == ROOT / "exp/pva_artiverse_articraft_partnet_lam_n5_tsne"


def test_lam_dataset_color_is_distinct() -> None:
    module = _module()
    assert module.DATASET_COLORS["lam"] not in {
        module.DATASET_COLORS[key]
        for key in module.DATASET_COLORS
        if key != "lam"
    }


def test_published_five_dataset_audit_is_complete() -> None:
    import json

    module = _module()
    audit_path = ROOT / "exp/pva_artiverse_articraft_partnet_lam_n5_tsne/audit.json"
    audit = json.loads(audit_path.read_text(encoding="utf-8"))
    assert audit["pass"] is True
    assert audit["counts"] == {
        "all_classes": 1565,
        "all_samples": 5725,
        "strict_n5_classes": 971,
        "strict_n5_samples": 4855,
        "unique_class_colors": 1565,
    }
