#!/usr/bin/env python3
"""Run Articraft compile with one source-dataset compatibility alias.

Some frozen source records call ``warn_if_articulation_origin_near_geometry``.
The SDK exposes the same warning check under the corrected ``...far...`` name.
This wrapper aliases only that non-blocking diagnostic before invoking the
unchanged compiler; it does not edit source records or generated geometry.
"""

from __future__ import annotations

import sdk as sdk_package
from sdk import TestContext
from sdk._core.v0._testing.core import TestContextCoreMixin
from sdk._core.v0._testing.model_checks import TestContextModelCheckMixin


_test_context_init = TestContext.__init__
_knob_geometry = sdk_package.KnobGeometry
_part_world_aabb = TestContextCoreMixin.part_world_aabb
_warn_disconnected = (
    TestContextModelCheckMixin.warn_if_part_contains_disconnected_geometry_islands
)


def _test_context_init_compat(
    self: TestContext,
    *args: object,
    geometry_source: str | None = None,
    **kwargs: object,
) -> None:
    if geometry_source not in {None, "visual", "collision"}:
        raise ValueError(f"unsupported legacy geometry_source: {geometry_source}")
    _test_context_init(self, *args, **kwargs)


def _knob_geometry_compat(*args: object, **kwargs: object) -> object:
    try:
        return _knob_geometry(*args, **kwargs)
    except ValueError as error:
        if "Null TopoDS_Shape object" not in str(error) or kwargs.get("grip") is None:
            raise
        retry_kwargs = dict(kwargs)
        retry_kwargs["grip"] = None
        return _knob_geometry(*args, **retry_kwargs)


def _part_world_aabb_compat(
    self: TestContextCoreMixin,
    part: object,
    *,
    use: str | None = None,
) -> object:
    if use not in {None, "visual", "collision"}:
        raise ValueError(f"unsupported part_world_aabb source selector: {use}")
    return _part_world_aabb(self, part)
_fail_disconnected = (
    TestContextModelCheckMixin.fail_if_part_contains_disconnected_geometry_islands
)
_warn_coplanar = TestContextModelCheckMixin.warn_if_coplanar_surfaces


def _warn_disconnected_compat(
    self: TestContextModelCheckMixin,
    *,
    tol: float = 1e-6,
    name: str | None = None,
    use: str | None = None,
) -> bool:
    if use not in {None, "visual"}:
        raise ValueError(f"unsupported disconnected-geometry source selector: {use}")
    return _warn_disconnected(self, tol=tol, name=name)


def _fail_disconnected_compat(
    self: TestContextModelCheckMixin,
    *,
    tol: float = 1e-6,
    name: str | None = None,
    use: str | None = None,
) -> bool:
    if use not in {None, "visual"}:
        raise ValueError(f"unsupported disconnected-geometry source selector: {use}")
    return _fail_disconnected(self, tol=tol, name=name)


def _warn_coplanar_compat(
    self: TestContextModelCheckMixin,
    *,
    use: str | None = None,
    **kwargs: object,
) -> bool:
    if use not in {None, "visual"}:
        raise ValueError(f"unsupported coplanar-surface source selector: {use}")
    return _warn_coplanar(self, **kwargs)


if not hasattr(TestContextModelCheckMixin, "warn_if_articulation_origin_near_geometry"):
    TestContextModelCheckMixin.warn_if_articulation_origin_near_geometry = (  # type: ignore[attr-defined]
        TestContextModelCheckMixin.warn_if_articulation_origin_far_from_geometry
    )
if not hasattr(TestContextModelCheckMixin, "warn_if_part_geometry_disconnected"):
    TestContextModelCheckMixin.warn_if_part_geometry_disconnected = (  # type: ignore[attr-defined]
        _warn_disconnected_compat
    )
if not hasattr(TestContextModelCheckMixin, "check_articulation_overlaps"):
    TestContextModelCheckMixin.check_articulation_overlaps = (  # type: ignore[attr-defined]
        TestContextModelCheckMixin.fail_if_articulation_overlaps
    )
TestContextModelCheckMixin.warn_if_part_contains_disconnected_geometry_islands = (  # type: ignore[method-assign]
    _warn_disconnected_compat
)
TestContextModelCheckMixin.fail_if_part_contains_disconnected_geometry_islands = (  # type: ignore[method-assign]
    _fail_disconnected_compat
)
TestContextModelCheckMixin.warn_if_coplanar_surfaces = _warn_coplanar_compat  # type: ignore[method-assign]
TestContext.__init__ = _test_context_init_compat  # type: ignore[method-assign]
TestContextCoreMixin.part_world_aabb = _part_world_aabb_compat  # type: ignore[method-assign]
sdk_package.KnobGeometry = _knob_geometry_compat  # type: ignore[assignment]

from cli.compile_record import main  # noqa: E402  (alias must be installed first)


if __name__ == "__main__":
    raise SystemExit(main())
