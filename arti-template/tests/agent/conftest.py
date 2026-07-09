"""Auto-mark per-template asset tests so they are excluded from the default run.

Per-template tests (`test_<slug>_template.py`) lock a single template's
structure. They are optional regression nets — the authoritative signal for a
modular template is `articraft template compile-sweep <slug>`, not pytest (see
CLAUDE.md). To keep `just test-all` / CI from being dominated by per-template
drift, these tests are auto-tagged `template_asset` and excluded by default via
`-m "not template_asset"` in pyproject. Run them explicitly with
`pytest -m template_asset` (or by path) when you want them.

The sweep/coverage ENGINE tests (`test_template_sweep*.py`,
`test_split_templates.py`) are NOT per-template and must keep running by
default, so they are explicitly excluded from the tagging below even though
some share the `_template` suffix.
"""

from __future__ import annotations

import pytest

# Engine/infra tests that happen to contain "template" in their filename but
# are NOT per-template asset tests — they must run in the default set.
_ENGINE_TEST_STEMS = frozenset(
    {
        "test_template_sweep",
        "test_template_sweep_clusters",
        "test_template_sweep_coverage",
        "test_template_sweep_state",
        "test_split_templates",
    }
)


def pytest_collection_modifyitems(config, items):
    for item in items:
        stem = item.path.stem
        if stem in _ENGINE_TEST_STEMS:
            continue
        if stem.startswith("test_") and stem.endswith("_template"):
            item.add_marker(pytest.mark.template_asset)


@pytest.fixture(autouse=True)
def _clear_probe_cache():
    """The slot-choice probe cache is process-lived (a pipeline-run speedup);
    tests monkeypatch different choices onto the same slug, so isolate them."""
    from agent.template_sweep_coverage import _PROBE_CACHE

    _PROBE_CACHE.clear()
    yield
    _PROBE_CACHE.clear()
