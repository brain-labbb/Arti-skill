from __future__ import annotations

import configparser
import subprocess
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SUBMODULES = ("arti-template", "articraft_data")


def git(*args: str, cwd: Path = ROOT) -> str:
    return subprocess.run(
        ["git", "-C", str(cwd), *args],
        check=True,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    ).stdout.strip()


class PipelineCheckoutTests(unittest.TestCase):
    def test_exactly_two_submodules_are_declared(self) -> None:
        parser = configparser.ConfigParser()
        parser.read(ROOT / ".gitmodules")
        paths = {parser[section]["path"] for section in parser.sections()}
        self.assertEqual(paths, set(SUBMODULES))

    def test_submodules_are_gitlinks_at_the_checked_out_heads(self) -> None:
        for path in SUBMODULES:
            with self.subTest(path=path):
                fields = git("ls-files", "--stage", "--", path).split()
                self.assertGreaterEqual(len(fields), 4)
                self.assertEqual(fields[0], "160000")
                self.assertEqual(fields[1], git("rev-parse", "HEAD", cwd=ROOT / path))

    def test_child_project_markers_exist(self) -> None:
        expected = (
            ROOT / "arti-template" / "pyproject.toml",
            ROOT / "arti-template" / "justfile",
            ROOT / "articraft_data" / "pyproject.toml",
            ROOT / "articraft_data" / ".gitattributes",
            ROOT / "eval_pilot" / "pilot.py",
            ROOT / "configs" / "pipeline.toml",
        )
        for path in expected:
            with self.subTest(path=path.relative_to(ROOT)):
                self.assertTrue(path.is_file())

    def test_local_secret_file_is_not_tracked(self) -> None:
        tracked = subprocess.run(
            ["git", "-C", str(ROOT), "ls-files", "--error-unmatch", ".env"],
            text=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        self.assertNotEqual(tracked.returncode, 0)


if __name__ == "__main__":
    unittest.main()
