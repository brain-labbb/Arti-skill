#!/usr/bin/env python3

from __future__ import annotations

from pathlib import Path
import sys
import tempfile
import unittest
from xml.etree import ElementTree


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import table5_tipover_eligibility as eligibility


class EligibilityTest(unittest.TestCase):
    def test_installation_semantics_are_fixed(self) -> None:
        self.assertIsNotNone(eligibility.semantic_reason("louvered_shutter_assembly"))
        self.assertIsNone(eligibility.semantic_reason("floor_lamp"))

    def test_filtered_manifest_keeps_row_hashes(self) -> None:
        manifest = {
            "manifest_sha256": "source",
            "datasets": [{
                "dataset_slug": "pva",
                "dataset_name": "PV-A",
                "rows": [{"dataset_id": "a"}, {"dataset_id": "b"}],
            }],
        }
        with tempfile.TemporaryDirectory() as directory:
            result = eligibility.make_filtered_manifest(manifest, {"a"}, Path(directory) / "manifest.json")
            self.assertEqual(result["datasets"][0]["rows"][0]["dataset_id"], "a")
            self.assertEqual(len(result["datasets"][0]["rows"]), 1)
            self.assertNotEqual(result["manifest_sha256"], "source")

    def test_visual_geometry_is_used_when_collision_is_absent(self) -> None:
        row = {
            "joint_tree": {
                "root_links": ["base"],
                "joints": [],
            }
        }
        robot = ElementTree.Element("robot")
        link = ElementTree.SubElement(robot, "link", name="base")
        visual = ElementTree.SubElement(link, "visual")
        geometry = ElementTree.SubElement(visual, "geometry")
        ElementTree.SubElement(geometry, "box", size="2 2 1")
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "model.urdf"
            ElementTree.ElementTree(robot).write(source, encoding="unicode")
            points = eligibility.collision_points(row, source)
        self.assertEqual(points.shape, (8, 3))


if __name__ == "__main__":
    unittest.main()
