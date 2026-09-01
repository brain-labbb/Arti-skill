#!/usr/bin/env python3
"""CLI wrapper for the frozen Infinigen-Sim one-per-category render."""

from render_mobility_uniform import main


if __name__ == "__main__":
    raise SystemExit(main(["--dataset", "infinigen", *__import__("sys").argv[1:]]))
