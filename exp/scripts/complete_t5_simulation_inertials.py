#!/usr/bin/env python3
"""Complete missing T5 simulator inertials with a documented local-AABB box prior."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
import xml.etree.ElementTree as ET
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np

for _name, _value in {"float": float, "int": int}.items():
    if not hasattr(np, _name):
        setattr(np, _name, _value)

SCRIPT_DIR = Path(__file__).resolve().parent
EXP_ROOT = SCRIPT_DIR.parent
DEFAULT_ROOT = EXP_ROOT / "runtime/t5_formal_v1/simulation_ready"
ALUMINUM_DENSITY = 2700.0
MIN_EXTENT_M = 0.001

sys.path.insert(0, str(SCRIPT_DIR))
from run_nano3d_articulation_paper import parse_physical_metadata  # noqa: E402
from run_nano3d_urdf_glb_pilot import build_link_mesh, material_colors  # noqa: E402


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def finite_positive(value: str | None) -> bool:
    try:
        number = float(value)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return False
    return math.isfinite(number) and number > 0.0


def inertial_valid(link: ET.Element) -> bool:
    inertial = link.find("inertial")
    if inertial is None:
        return False
    mass = inertial.find("mass")
    tensor = inertial.find("inertia")
    return bool(
        mass is not None
        and tensor is not None
        and finite_positive(mass.attrib.get("value"))
        and all(finite_positive(tensor.attrib.get(key)) for key in ("ixx", "iyy", "izz"))
    )


def fmt(value: float) -> str:
    return f"{float(value):.9g}"


def add_proxy_collision(link: ET.Element) -> None:
    collision = ET.SubElement(link, "collision", {"name": "standardized_empty_link_proxy"})
    geometry = ET.SubElement(collision, "geometry")
    ET.SubElement(geometry, "box", {"size": f"{MIN_EXTENT_M} {MIN_EXTENT_M} {MIN_EXTENT_M}"})


def replace_inertial(link: ET.Element, bounds: np.ndarray) -> dict[str, Any]:
    old = link.find("inertial")
    if old is not None:
        link.remove(old)
    low, high = bounds
    centre = (low + high) / 2.0
    extents = np.maximum(high - low, MIN_EXTENT_M)
    mass = float(ALUMINUM_DENSITY * np.prod(extents))
    x, y, z = (float(value) for value in extents)
    ixx = mass * (y * y + z * z) / 12.0
    iyy = mass * (x * x + z * z) / 12.0
    izz = mass * (x * x + y * y) / 12.0
    inertial = ET.Element("inertial")
    ET.SubElement(inertial, "origin", {"xyz": " ".join(fmt(value) for value in centre), "rpy": "0 0 0"})
    ET.SubElement(inertial, "mass", {"value": fmt(mass)})
    ET.SubElement(
        inertial,
        "inertia",
        {
            "ixx": fmt(ixx),
            "ixy": "0",
            "ixz": "0",
            "iyy": fmt(iyy),
            "iyz": "0",
            "izz": fmt(izz),
        },
    )
    link.insert(0, inertial)
    return {
        "centre": [float(value) for value in centre],
        "extents": [x, y, z],
        "mass": mass,
        "inertia_diagonal": [ixx, iyy, izz],
    }


def complete_package(package: Path) -> dict[str, Any]:
    urdf_path = package / "model.urdf"
    root = ET.parse(urdf_path).getroot()
    colors = material_colors(root)
    changed: list[dict[str, Any]] = []
    proxy_links: list[str] = []
    for node in root.findall("link"):
        name = node.attrib["name"]
        has_collision = bool(node.findall("collision"))
        if inertial_valid(node) and has_collision:
            continue
        mesh, _ = build_link_mesh(node, package, colors)
        if mesh is None or np.asarray(mesh.bounds).shape != (2, 3):
            bounds = np.asarray(
                [[-MIN_EXTENT_M / 2.0] * 3, [MIN_EXTENT_M / 2.0] * 3],
                dtype=float,
            )
            if not has_collision:
                add_proxy_collision(node)
                proxy_links.append(name)
        else:
            bounds = np.asarray(mesh.bounds, dtype=float)
            if not has_collision:
                add_proxy_collision(node)
                proxy_links.append(name)
        values = replace_inertial(node, bounds)
        changed.append({"link": name, **values})
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(urdf_path, encoding="unicode", xml_declaration=False)
    return {
        "asset_id": package.name,
        "changed_link_count": len(changed),
        "proxy_collision_links": proxy_links,
        "changed_links": changed,
        "model_urdf_sha256": sha256(urdf_path),
        **parse_physical_metadata(package),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=DEFAULT_ROOT)
    args = parser.parse_args()
    root = args.root.resolve()
    root.relative_to(EXP_ROOT.resolve())
    manifest_path = root / "simulation_input_manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    records = [complete_package(Path(row["copied_package"])) for row in manifest]
    by_id = {row["asset_id"]: row for row in records}
    for row in manifest:
        row["model_urdf_sha256"] = by_id[row["asset_id"]]["model_urdf_sha256"]
    manifest_path.write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    output = {
        "schema_version": 1,
        "protocol": "t5_local_aabb_inertial_completion_v1",
        "created_at": datetime.now(timezone.utc).isoformat(),
        "density_kg_m3": ALUMINUM_DENSITY,
        "minimum_extent_m": MIN_EXTENT_M,
        "changed_links": sum(row["changed_link_count"] for row in records),
        "proxy_collision_links": sum(len(row["proxy_collision_links"]) for row in records),
        "asset_count": len(records),
        "physical_metadata_complete_links": sum(row["physical_metadata_complete_link_count"] for row in records),
        "link_count": sum(row["link_count"] for row in records),
        "physical_metadata_complete_assets": sum(row["physical_metadata_complete_asset"] for row in records),
        "method": (
            "Keep trustworthy generated inertials; for missing/untrusted links use the local collision/visual AABB as a solid aluminum box. "
            "Links with no body geometry receive a 1 mm collision/inertial proxy."
        ),
        "records": records,
    }
    (root / "inertial_completion.json").write_text(
        json.dumps(output, indent=2, ensure_ascii=False, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps({key: value for key, value in output.items() if key != "records"}, indent=2))
    return 0 if output["physical_metadata_complete_assets"] == len(records) else 1


if __name__ == "__main__":
    raise SystemExit(main())
