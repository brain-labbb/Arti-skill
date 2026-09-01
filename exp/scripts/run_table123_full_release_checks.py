"""Backward-compatible import name for the full-release acceptance checker."""

try:
    from check_table123_full_release import *  # noqa: F401,F403
except ImportError:  # pragma: no cover
    from .check_table123_full_release import *  # noqa: F401,F403
