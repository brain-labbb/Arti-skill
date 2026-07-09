"""Verified mechanism idioms for template authoring.

Corpus facts (2026-07 census + repair campaign over 289 templates): ~45% of
templates hinge a lid/door/leaf/hatch, ~20% slide something, ~6% fold a chain
— and the genuine motion-穿模 defect list concentrated in exactly three root
causes: hinges swinging the WRONG WAY into their own body (manhole cover,
turnstile door, kettle bail), physically-coupled chains modeled as INDEPENDENT
joints (folding door, folder sheets, folding arm), and hand-written limits
letting a subtree sweep into geometry (turret, tank, scale, telescope).

These constructors make those bugs unrepresentable at build time. They do NOT
create geometry — authors keep full control of appearance; the idiom wires the
kinematics: emits the joint, verifies the swing direction actually moves the
panel toward the declared clear side, solves the safe travel against the rest
of the model with the SDK clearance solver, and wires mimic coupling for
chains. Prefer these over raw ``model.articulation(...)`` for matching
mechanisms; drop to raw joints only when no idiom fits (and say why).
"""

from __future__ import annotations

import math
from typing import Dict, Iterable, Optional, Sequence, Tuple

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MatingContract,
    Mimic,
    MotionLimits,
    Origin,
    clamp_joint_limits,
    max_joint_value,
)
from sdk._core.v0.geometry_qc import compute_part_world_transforms
from sdk._core.v0.motion_solve import _DEFAULT_MARGIN

__all__ = [
    "coupled_chain",
    "hinged_panel",
    "sliding_member",
]

# Fraction of the requested travel below which a solved range counts as
# collapsed: the mechanism is unusable and the caller must fix geometry, not
# ship a joint that barely moves.
_MIN_SOLVED_FRACTION = 0.05


def _part_name(part: object) -> str:
    name = part if isinstance(part, str) else getattr(part, "name", None)
    if not isinstance(name, str):
        raise ValueError(f"expected a part or part name, got {part!r}")
    return name


def _panel_world_centroid(model: ArticulatedObject, part_name: str, pose: Dict[str, float]):
    transforms = compute_part_world_transforms(model, pose)
    tf = transforms[part_name]
    part = model.get_part(part_name)
    xs = ys = zs = 0.0
    count = 0
    for visual in part.visuals:
        x, y, z = visual.origin.xyz
        wx = tf[0][0] * x + tf[0][1] * y + tf[0][2] * z + tf[0][3]
        wy = tf[1][0] * x + tf[1][1] * y + tf[1][2] * z + tf[1][3]
        wz = tf[2][0] * x + tf[2][1] * y + tf[2][2] * z + tf[2][3]
        xs += wx
        ys += wy
        zs += wz
        count += 1
    if count == 0:
        raise ValueError(f"part {part_name!r} has no visuals to reason about")
    return (xs / count, ys / count, zs / count)


def hinged_panel(
    model: ArticulatedObject,
    *,
    joint_name: str,
    parent: object,
    panel: object,
    hinge_origin: Tuple[float, float, float],
    axis: Tuple[float, float, float],
    open_toward: Tuple[float, float, float],
    max_open: float = math.radians(110.0),
    margin: float = _DEFAULT_MARGIN,
    keepout: Optional[Iterable[str]] = None,
    allowed_pairs: Optional[Iterable[Tuple[str, str]]] = None,
    effort: float = 12.0,
    velocity: float = 1.2,
    mating: Optional[MatingContract] = None,
    solve_clearance: bool = True,
) -> MotionLimits:
    """Emit a lid/door/hatch/flap hinge that provably opens the right way.

    ``open_toward`` is a world-frame direction the panel's centroid must move
    when the joint opens (e.g. ``(0, -1, 0)`` for a cabinet door that must
    swing out the front). The constructor probes the FK at a small positive
    angle and RAISES if the panel moves against it — the reversed-swing bug
    family (cover into shaft, door into cabinet, bail into spout) becomes a
    build-time error instead of a shipped 穿模. The opening range is then
    clamped with the clearance solver so the panel's swept arc keeps
    ``margin`` from everything else (minus ``allowed_pairs``).

    Emit the panel's geometry BEFORE calling this; the joint is created here
    with ``[0, max_open]`` and tightened in place. Returns the final limits.
    """
    parent_part = model.get_part(parent)
    panel_part = model.get_part(panel)
    limits = MotionLimits(effort=effort, velocity=velocity, lower=0.0, upper=float(max_open))
    model.articulation(
        joint_name,
        ArticulationType.REVOLUTE,
        parent=parent_part,
        child=panel_part,
        origin=Origin(xyz=tuple(float(c) for c in hinge_origin)),
        axis=tuple(float(c) for c in axis),
        motion_limits=limits,
        mating=mating,
    )
    probe_angle = min(0.15, float(max_open) / 2.0)
    rest = _panel_world_centroid(model, panel_part.name, {joint_name: 0.0})
    opened = _panel_world_centroid(model, panel_part.name, {joint_name: probe_angle})
    motion = tuple(o - r for o, r in zip(opened, rest))
    magnitude = math.sqrt(sum(m * m for m in motion))
    direction_norm = math.sqrt(sum(d * d for d in open_toward)) or 1.0

    def _remove_joint() -> None:
        model.articulations[:] = [j for j in model.articulations if j.name != joint_name]

    if magnitude < 1e-9:
        _remove_joint()
        raise ValueError(
            f"hinged_panel({joint_name!r}): the panel's visual centroid sits on the "
            f"hinge axis, so the opening direction cannot be inferred — restructure "
            f"the panel visuals (or place the hinge on the panel edge) before mounting."
        )
    if sum(m * d for m, d in zip(motion, open_toward)) / (magnitude * direction_norm) < 0.6:
        _remove_joint()
        raise ValueError(
            f"hinged_panel({joint_name!r}): opening the joint moves {panel_part.name!r} "
            f"by {motion} which is NOT toward open_toward={open_toward} (needs cos>0.6). "
            f"The hinge axis sign or pivot edge is wrong — this is the reversed-swing "
            f"bug family; flip the axis or move the hinge to the opposite edge instead "
            f"of shipping it."
        )
    if not solve_clearance:
        return limits
    solved = clamp_joint_limits(
        model,
        joint_name,
        margin=margin,
        keepout=keepout,
        allowed_pairs=allowed_pairs,
    )
    if float(solved.upper or 0.0) < float(max_open) * _MIN_SOLVED_FRACTION:
        _remove_joint()
        raise ValueError(
            f"hinged_panel({joint_name!r}): the clearance solver collapsed the opening "
            f"range to {solved.upper:.4g} rad (requested {float(max_open):.4g}) — the "
            f"panel cannot open at all. Fix the geometry (something blocks the arc at "
            f"rest) instead of shipping a door that doesn't move."
        )
    return solved


def coupled_chain(
    model: ArticulatedObject,
    *,
    driver: str,
    followers: Sequence[str],
    multipliers: Optional[Sequence[float]] = None,
    fold_budget: Optional[float] = None,
    margin: float = _DEFAULT_MARGIN,
    keepout: Optional[Iterable[str]] = None,
    allowed_pairs: Optional[Iterable[Tuple[str, str]]] = None,
) -> MotionLimits:
    """Couple an already-emitted serial chain to one driver joint.

    A concertina/folding chain modeled as independent joints ALWAYS has
    self-intersecting pose combos (the QC samples joints independently, and
    physically the members constrain each other). This idiom wires every
    follower joint as a ``Mimic`` of ``driver`` (default multiplier 1.0, or
    the classic alternating ±2.0 for accordion leaves via ``multipliers``),
    then clamps the driver's limits with the clearance solver — on the coupled
    trajectory, so the poses that remain are exactly the physical ones.

    ``fold_budget`` optionally caps the all-joints cumulative rotation (e.g.
    ``4.2`` rad keeps a same-direction chain from wrapping back through its
    base) before the solver runs; the solver then only tightens further.
    """
    driver_joint = None
    follower_joints = []
    for joint in model.articulations:
        if joint.name == driver:
            driver_joint = joint
        elif joint.name in set(followers):
            follower_joints.append(joint)
    if driver_joint is None:
        raise ValueError(f"coupled_chain: unknown driver joint {driver!r}")
    missing = set(followers) - {j.name for j in follower_joints}
    if missing:
        raise ValueError(f"coupled_chain: unknown follower joint(s) {sorted(missing)}")
    # The clearance solve's moving set is the driver's child subtree; a
    # follower outside it would be mimic-coupled but NEVER clearance-checked.
    from sdk._core.v0.motion_solve import _child_subtree

    subtree = _child_subtree(model, driver)
    child_of = {}
    for joint in model.articulations:
        c = joint.child if isinstance(joint.child, str) else getattr(joint.child, "name", None)
        if isinstance(c, str):
            child_of[joint.name] = c
    outside = [name for name in followers if child_of.get(name) not in subtree]
    if outside:
        raise ValueError(
            f"coupled_chain: follower joint(s) {outside} move parts OUTSIDE the driver's "
            f"child subtree — they would be mimic-coupled but never clearance-checked "
            f"(the solver only sweeps the driver's subtree). Chain the parts serially "
            f"under the driver's child, or solve each branch separately."
        )
    if multipliers is None:
        multipliers = [1.0] * len(follower_joints)
    if len(multipliers) != len(follower_joints):
        raise ValueError("coupled_chain: multipliers must match followers 1:1")
    by_name = {j.name: j for j in follower_joints}
    for name, multiplier in zip(followers, multipliers):
        joint = by_name[name]
        joint.mimic = Mimic(joint=driver, multiplier=float(multiplier))
        limits = joint.motion_limits
        if limits is not None and driver_joint.motion_limits is not None:
            lo, hi = sorted(
                (
                    float(driver_joint.motion_limits.lower or 0.0) * float(multiplier),
                    float(driver_joint.motion_limits.upper or 0.0) * float(multiplier),
                )
            )
            joint.motion_limits = MotionLimits(
                effort=limits.effort, velocity=limits.velocity, lower=lo, upper=hi
            )
    if fold_budget is not None and driver_joint.motion_limits is not None:
        total_gain = 1.0 + sum(abs(float(m)) for m in multipliers)
        cap = float(fold_budget) / total_gain
        current = driver_joint.motion_limits
        driver_joint.motion_limits = MotionLimits(
            effort=current.effort,
            velocity=current.velocity,
            lower=max(float(current.lower or 0.0), -cap),
            upper=min(float(current.upper or 0.0), cap),
        )
    solved = clamp_joint_limits(
        model,
        driver,
        margin=margin,
        keepout=keepout,
        allowed_pairs=allowed_pairs,
    )
    # Re-sync follower limits from the driver's REALIZED range so exporters
    # and UIs reading follower limits agree with the coupled trajectory.
    for name, multiplier in zip(followers, multipliers):
        joint = by_name[name]
        if joint.motion_limits is not None:
            lo, hi = sorted(
                (
                    float(solved.lower or 0.0) * float(multiplier),
                    float(solved.upper or 0.0) * float(multiplier),
                )
            )
            joint.motion_limits = MotionLimits(
                effort=joint.motion_limits.effort,
                velocity=joint.motion_limits.velocity,
                lower=lo,
                upper=hi,
            )
    return solved


def sliding_member(
    model: ArticulatedObject,
    *,
    joint_name: str,
    parent: object,
    slider: object,
    origin: Tuple[float, float, float],
    axis: Tuple[float, float, float],
    max_travel: float,
    margin: float = _DEFAULT_MARGIN,
    keepout: Optional[Iterable[str]] = None,
    allowed_pairs: Optional[Iterable[Tuple[str, str]]] = None,
    effort: float = 30.0,
    velocity: float = 0.3,
    mating: Optional[MatingContract] = None,
) -> MotionLimits:
    """Emit a drawer/tray/telescoping prismatic with solver-clamped travel.

    ``max_travel`` is the author's intent (full extension); the realized upper
    limit is the largest travel keeping the slider subtree ``margin`` clear of
    everything else — a tray can no longer rise into a closed lid, a sluice
    gate can no longer drop into a spinning wheel. Nested containment that is
    intentional (rail inside sleeve) goes in ``allowed_pairs`` element-scoped
    allowances on the template side, or a restricted ``keepout``.
    """
    parent_part = model.get_part(parent)
    slider_part = model.get_part(slider)
    limits = MotionLimits(effort=effort, velocity=velocity, lower=0.0, upper=float(max_travel))
    model.articulation(
        joint_name,
        ArticulationType.PRISMATIC,
        parent=parent_part,
        child=slider_part,
        origin=Origin(xyz=tuple(float(c) for c in origin)),
        axis=tuple(float(c) for c in axis),
        motion_limits=limits,
        mating=mating,
    )
    solved_upper = max_joint_value(
        model,
        joint=joint_name,
        limit=float(max_travel),
        margin=margin,
        keepout=keepout,
        allowed_pairs=allowed_pairs,
    )
    if solved_upper < float(max_travel) * _MIN_SOLVED_FRACTION:
        model.articulations[:] = [j for j in model.articulations if j.name != joint_name]
        raise ValueError(
            f"sliding_member({joint_name!r}): the clearance solver collapsed the travel "
            f"to {solved_upper:.4g} m (requested {float(max_travel):.4g}) — the slider "
            f"cannot move. Common cause: the prismatic axis points INTO the carcass "
            f"(flipped sign) or something blocks the path at rest. Fix the geometry "
            f"instead of shipping a drawer that doesn't open."
        )
    joint = next(j for j in model.articulations if j.name == joint_name)
    joint.motion_limits = MotionLimits(
        effort=effort, velocity=velocity, lower=0.0, upper=solved_upper
    )
    return joint.motion_limits
