#!/usr/bin/env python3
"""CLI wrapper for the frozen PhysX-Mobility one-per-category render."""

from render_mobility_uniform import main


if __name__ == "__main__":
    raise SystemExit(main(["--dataset", "physx", *__import__("sys").argv[1:]]))
