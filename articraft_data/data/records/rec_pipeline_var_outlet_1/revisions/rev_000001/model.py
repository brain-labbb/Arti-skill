from __future__ import annotations

# Fire-hydrant standpipe (pillar / above-ground hydrant on a riser pipe), built
# faithfully from picture/Equipment/Pipeline/002.png.
#
# What the reference shows:
#   - A tall orange/red riser pipe (the standpipe) standing vertically.
#   - A bulbous red cast-iron valve body / bonnet seated on top of the riser.
#   - A large spoked cast-iron HANDWHEEL on a brass stem at the very top; turning
#     it operates the internal valve (the primary user mechanism).
#   - Variant edit: a single brass side outlet nozzle (one tee take-off) on the
#     body, with one brass pull-off cap retained by one articulated oval-link
#     chain. The cap slides straight out along the nozzle axis to uncover the
#     outlet (the secondary mechanism).
#   - A small brass dome / packing nut at the very top under the wheel.
#
# Articulation:
#   - body_to_wheel : CONTINUOUS about +Z. Turning the handwheel operates the
#     valve. This is the hero mechanism.
#   - body_to_cap_0 : PRISMATIC along the outlet axis (+X). The brass cap pulls
#     straight off along its nozzle to uncover the outlet (it does NOT lift
#     vertically).
#   - chain_0_swing_{i}: REVOLUTE link joints (adapted from the gutter
#     downchain link-chain sample). Each retaining chain is ONE continuous serial
#     chain whose root link is jointed to the CAP at its small ball (so the chain
#     follows the cap and never detaches from it -- pull the cap off and the chain
#     comes with it) and which runs down to wrap the body eye. Being a single
#     chain, consecutive links always interlink, so it can never break.
#
# Frame convention (real meters, Z-up):
#   +Z  -> up; the riser pipe foot sits at z = 0.
#   The single outlet nozzle points along +X.
#   The handwheel axis is vertical (+Z), offset slightly toward +Y like the
#   reference where the wheel stem rises at the back of the bonnet.

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---------------------------------------------------------------------------
# Dimensions (meters)
# ---------------------------------------------------------------------------

# Riser standpipe (tall orange pipe at the bottom).
RISER_OD = 0.110  # outer diameter
RISER_ID = 0.092  # bore (hollow pipe)
RISER_LEN = 0.520  # length of the visible riser
RISER_TOP_Z = RISER_LEN  # top of the riser where the body collar seats

# Body collar / flange where the riser meets the cast-iron valve body.
COLLAR_OD = 0.150
COLLAR_H = 0.030
COLLAR_Z = RISER_TOP_Z  # bottom of the collar sits on the riser top

# Cast-iron valve body / bonnet (the bulbous red middle section).
BODY_BOTTOM_Z = COLLAR_Z + COLLAR_H
BODY_MAX_R = 0.092  # widest radius of the bulbous body
BODY_HEIGHT = 0.140  # height of the bulbous section
BODY_CENTER_Z = BODY_BOTTOM_Z + BODY_HEIGHT / 2.0
BODY_TOP_Z = BODY_BOTTOM_Z + BODY_HEIGHT

# Bonnet neck + brass packing dome + wheel stem above the body.
NECK_R = 0.040
NECK_H = 0.022
DOME_R = 0.030  # small brass dome / packing nut under the wheel
STEM_R = 0.009  # brass valve stem the wheel rides on
STEM_TOP_Z = BODY_TOP_Z + NECK_H + 0.060  # top of stem (wheel hub seats here)

# Outlet nozzle loop. Variant 2 fork is intentionally reduced to one capped side
# nozzle: N == 1, emitted through the same regular loop used for caps/chains.
OUTLET_COUNT = 1

# Outlet nozzle (brass), pointing along +X out of the body.
NOZZLE_R = 0.034  # nozzle outer radius
NOZZLE_BORE = 0.024  # nozzle bore radius
NOZZLE_LEN = 0.078  # how far the nozzle barrel runs out past the root
NOZZLE_Z = BODY_CENTER_Z + 0.006  # nozzles sit just above body mid-height
NOZZLE_ROOT_X = 0.050  # where the nozzle leaves the body (inside the bulb)
NOZZLE_TIP_X = NOZZLE_ROOT_X + NOZZLE_LEN  # outer face of the nozzle (~0.128)

# Brass screw caps closing the nozzles. The cap is authored as a cup along +X
# from its local origin, with the recessed mouth facing -X (toward the body).
CAP_R = 0.038
CAP_H = 0.030
CAP_RECESS_DEPTH = 0.020  # how deep the nozzle-receiving recess is
# The seated cap-mouth plane sits back over the nozzle tip so the tip is captured
# inside the recess (real seated fit; the resulting overlap is allowed).
CAP_HINGE_X = NOZZLE_TIP_X - 0.014
# The cap slides PRISMATICALLY along the outlet axis (not vertically). In the
# default pose it is shown pulled this far OUT along the nozzle so the mouth is
# uncovered; driving the joint to -CAP_DEFAULT_PULL re-seats it on the outlet.
CAP_DEFAULT_PULL = 0.035

# Retaining chain: oval links running from a boss on the cap to an eye on the
# body, so one end is tied to the hydrant and the other to the cap.
CHAIN_BOSS_LOCAL = (0.004, 0.026, -0.026)  # chain pin on the cap (right-cap frame)
CHAIN_LUG = (0.060, 0.034, 0.556)  # chain eye on the body shoulder (right side)
CHAIN_LUG_R = 0.009
# Oval link proportions: small links so the chain reads as several fine links.
# Consecutive links interlink at the natural pitch (one wire-diameter overlap),
# which is the proven clean interlink and is allowed as an intentional chain.
CHAIN_LINK_HALF_LEN = 0.0085
CHAIN_LINK_HALF_WID = 0.0040
CHAIN_LINK_WIRE_R = 0.0011
CHAIN_LINK_PITCH = 2.0 * CHAIN_LINK_HALF_LEN - 2.0 * CHAIN_LINK_WIRE_R
CHAIN_SWING = math.radians(35.0)

# Span from the cap boss (in the default pulled-out pose) to the body eye.
_cap_origin_r = (CAP_HINGE_X + CAP_DEFAULT_PULL, 0.0, NOZZLE_Z)
_chain_d_r = (
    CHAIN_LUG[0] - _cap_origin_r[0] - CHAIN_BOSS_LOCAL[0],
    CHAIN_LUG[1] - _cap_origin_r[1] - CHAIN_BOSS_LOCAL[1],
    CHAIN_LUG[2] - _cap_origin_r[2] - CHAIN_BOSS_LOCAL[2],
)
CHAIN_SPAN = math.sqrt(sum(c * c for c in _chain_d_r))
CHAIN_STEP = CHAIN_LINK_PITCH
# The chain is ONE continuous serial chain (every link interlinks with the next,
# so it can never break in the middle). It is riveted to the CAP at its small
# ball (that end is a kinematic joint, so the chain follows the cap) and runs
# down to wrap the body eye.
CHAIN_LINKS = max(6, round(CHAIN_SPAN / CHAIN_LINK_PITCH))

# Handwheel (spoked cast-iron operating wheel) at the top.
WHEEL_RIM_R = 0.072  # outer radius of the wheel rim
WHEEL_RIM_TUBE = 0.010  # rim cross-section (torus tube)
WHEEL_HUB_R = 0.018  # central hub radius
WHEEL_HUB_H = 0.026  # hub height
WHEEL_SPOKE_R = 0.004  # spoke rod radius
WHEEL_N_SPOKES = 6
WHEEL_Z = STEM_TOP_Z + WHEEL_HUB_H / 2.0  # hub center on the stem top
WHEEL_Y = 0.0

TOL = 0.0012
ATOL = 0.20

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------


def _materials(model: ArticulatedObject) -> None:
    model.material("hydrant_red", rgba=(0.82, 0.16, 0.10, 1.0))
    model.material("hydrant_orange", rgba=(0.90, 0.28, 0.10, 1.0))
    model.material("cast_iron", rgba=(0.18, 0.18, 0.19, 1.0))
    model.material("brass", rgba=(0.78, 0.60, 0.20, 1.0))
    model.material("brass_dark", rgba=(0.62, 0.46, 0.16, 1.0))


# ---------------------------------------------------------------------------
# Geometry builders (CadQuery, authored directly in meters)
# ---------------------------------------------------------------------------


def _riser_shape() -> cq.Workplane:
    """Tall hollow orange riser pipe standing on z = 0."""
    outer = cq.Workplane("XY").circle(RISER_OD / 2.0).extrude(RISER_LEN)
    bore = (
        cq.Workplane("XY")
        .workplane(offset=RISER_ID * 0.0)  # keep at z=0
        .circle(RISER_ID / 2.0)
        .extrude(RISER_LEN + 0.001)
    )
    pipe = outer.cut(bore)
    return pipe


def _collar_shape() -> cq.Workplane:
    """Flanged collar where the riser meets the valve body."""
    return cq.Workplane("XY").workplane(offset=COLLAR_Z).circle(COLLAR_OD / 2.0).extrude(COLLAR_H)


def _body_shape() -> cq.Workplane:
    """Bulbous cast-iron valve body via a revolved profile."""
    z0 = BODY_BOTTOM_Z
    h = BODY_HEIGHT
    # (radius, z) profile, revolved about Z. Bulged in the middle, necked top.
    pts = [
        (COLLAR_OD / 2.0 - 0.004, z0),
        (BODY_MAX_R * 0.82, z0 + h * 0.10),
        (BODY_MAX_R, z0 + h * 0.42),
        (BODY_MAX_R * 0.92, z0 + h * 0.70),
        (NECK_R + 0.018, z0 + h * 0.90),
        (NECK_R + 0.006, z0 + h),
    ]
    profile = cq.Workplane("XZ")
    profile = profile.moveTo(0.0, z0)
    for r, z in pts:
        profile = profile.lineTo(r, z)
    profile = profile.lineTo(0.0, z0 + h).close()
    return profile.revolve(360.0, (0, 0, 0), (0, 1, 0))


def _bonnet_neck_shape() -> cq.Workplane:
    """Neck + brass packing dome + valve stem above the body."""
    neck = cq.Workplane("XY").workplane(offset=BODY_TOP_Z).circle(NECK_R).extrude(NECK_H)
    return neck


def _dome_stem_shape() -> cq.Workplane:
    """Brass packing dome under the wheel plus the rising valve stem."""
    dome_base_z = BODY_TOP_Z + NECK_H
    dome = cq.Workplane("XY").workplane(offset=dome_base_z).sphere(DOME_R)
    # Keep only the upper hemisphere by translating a half-cut.
    dome = dome.translate((0, 0, 0))
    cut_box = (
        cq.Workplane("XY")
        .workplane(offset=dome_base_z - DOME_R)
        .box(4 * DOME_R, 4 * DOME_R, 2 * DOME_R, centered=(True, True, False))
    )
    dome = dome.intersect(cut_box)
    stem = (
        cq.Workplane("XY")
        .workplane(offset=dome_base_z)
        .circle(STEM_R)
        .extrude(STEM_TOP_Z - dome_base_z)
    )
    return dome.union(stem)


def _nozzle_shape(direction: int) -> cq.Workplane:
    """One brass outlet nozzle pointing along +X (direction=1) or -X (-1).

    The nozzle root starts inside the body wall so it is captured by the body;
    the tip carries a flange ring with male thread relief.
    """
    sign = 1.0 if direction > 0 else -1.0
    # Tube along X built on the YZ plane, then translated/rotated.
    tube = (
        cq.Workplane("YZ")
        .circle(NOZZLE_R)
        .circle(NOZZLE_BORE)
        .extrude(NOZZLE_LEN + NOZZLE_ROOT_X)  # extrudes along +X (plane normal)
    )
    # The extrude on a YZ workplane runs along +X starting at x=0.
    nozzle = tube.translate((0.0, 0.0, NOZZLE_Z))
    # Outer flange ring near the tip.
    flange = (
        cq.Workplane("YZ")
        .workplane(offset=NOZZLE_TIP_X - 0.010)
        .circle(NOZZLE_R + 0.006)
        .circle(NOZZLE_BORE)
        .extrude(0.010)
        .translate((0.0, 0.0, NOZZLE_Z))
    )
    nozzle = nozzle.union(flange)
    if sign < 0.0:
        nozzle = nozzle.rotate((0, 0, 0), (0, 0, 1), 180.0)
    return nozzle


def _chain_lug_shape(direction: int) -> cq.Workplane:
    """Small eye on the body shoulder (below the outlet) that anchors a chain.

    The chain's far link wraps this eye so the retaining chain is tied to the
    hydrant body instead of being left dangling in the air.
    """
    sign = 1.0 if direction > 0 else -1.0
    lx, ly, lz = CHAIN_LUG
    eye = (
        cq.Workplane("XY")
        .workplane(offset=lz)
        .center(sign * lx, sign * ly)
        .sphere(CHAIN_LUG_R)
    )
    return eye


def _cap_chain_anchor(direction: int) -> tuple[float, float, float]:
    """Cap-frame point where the first retaining-chain link is pinned (on the
    cap's chain boss). The left cap is the right cap rotated 180 deg about Z, so
    the boss mirrors in X and Y."""
    sign = 1.0 if direction > 0 else -1.0
    bx, by, bz = CHAIN_BOSS_LOCAL
    return (sign * bx, sign * by, bz)


def _rpy_aim_negz(direction: tuple[float, float, float]) -> tuple[float, float, float]:
    """Return rpy (roll=0) so a child's local -Z axis points along ``direction``
    (expressed in the parent frame). Used to aim the chain from the cap boss
    straight toward the body eye."""
    dx, dy, dz = direction
    n = math.sqrt(dx * dx + dy * dy + dz * dz)
    if n < 1e-9:
        return (0.0, 0.0, 0.0)
    # local +Z must point along -direction
    vx, vy, vz = -dx / n, -dy / n, -dz / n
    yaw = math.atan2(vy, vx)
    pitch = math.atan2(math.hypot(vx, vy), vz)
    return (0.0, pitch, yaw)


def _add_subchain(model, prefix, parent_part, root_origin, n_links):
    """Add a serial sub-chain of ``n_links`` oval links starting at
    ``root_origin`` (in ``parent_part``'s frame) and marching along its local
    -Z. The root link is jointed to ``parent_part`` (so that end is welded on).
    Returns the list of link parts (index 0 is the root link)."""
    parent = parent_part
    origin = root_origin
    links = []
    for i in range(n_links):
        link = model.part(f"{prefix}_{i}")
        in_yz = i % 2 == 1
        link.visual(
            _oval_chain_link_mesh(in_yz, f"{prefix}_{i}_oval"),
            material="brass_dark",
            name="oval_body",
        )
        model.articulation(
            f"{prefix}_swing_{i}",
            ArticulationType.REVOLUTE,
            parent=parent,
            child=link,
            origin=origin,
            axis=(1.0, 0.0, 0.0) if in_yz else (0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=0.8, velocity=2.0, lower=-CHAIN_SWING, upper=CHAIN_SWING
            ),
        )
        parent = link
        origin = Origin(xyz=(0.0, 0.0, -CHAIN_STEP))
        links.append(link)
    return links


def _oval_chain_link_mesh(in_yz_plane: bool, name: str):
    """Oval chain link adapted from gutter_downchain_var_link_chain.

    The local origin is the top contact point. The oval body extends downward,
    and adjacent links alternate between XZ and YZ planes like a real chain.
    """
    pts = []
    for j in range(36):
        t = 2.0 * math.pi * j / 36
        short = CHAIN_LINK_HALF_WID * math.cos(t)
        long = CHAIN_LINK_HALF_LEN * math.sin(t)
        if in_yz_plane:
            pts.append((0.0, short, -CHAIN_LINK_HALF_LEN + long))
        else:
            pts.append((short, 0.0, -CHAIN_LINK_HALF_LEN + long))
    geom = tube_from_spline_points(
        pts,
        radius=CHAIN_LINK_WIRE_R,
        samples_per_segment=10,
        closed_spline=True,
        radial_segments=14,
        cap_ends=False,
    )
    return mesh_from_geometry(geom, name)


def _cap_shape(direction: int) -> cq.Workplane:
    """Brass screw cap that closes one nozzle.

    Base form (direction>0): a cup extending +X from the local origin with the
    open recessed mouth at local x=0 (facing -X, toward the nozzle tip) and the
    closed outer disk at the far +X end. A small chain boss sits on the lower
    flank where the retaining chain's first link pins on. For the mirrored left
    cap (direction<0) the whole cap is rotated 180 deg about Z so it extends -X
    with its mouth facing +X (toward the left nozzle tip).

    The cap is authored at its SEATED position; the prismatic joint slides it
    out along the nozzle axis (see body_to_*_cap), so it never lifts vertically.
    """
    # Cup body: a cylinder with a closed outer face and a recessed mouth.
    cup = (
        cq.Workplane("YZ").circle(CAP_R).extrude(CAP_H)  # extrudes along +X
    )
    # Recess the mouth (the open end at local x=0 that seats over the nozzle
    # tip). The recess radius is slightly smaller than the nozzle so the cup wall
    # grips the nozzle tip with a small interference seat (captured, allowed).
    recess = cq.Workplane("YZ").circle(NOZZLE_R - 0.002).extrude(CAP_RECESS_DEPTH)
    cup = cup.cut(recess)
    # Knurled grip ring near the outer face.
    grip = cq.Workplane("YZ").workplane(offset=CAP_H - 0.006).circle(CAP_R + 0.004).extrude(0.006)
    cap = cup.union(grip)
    # Chain boss on the lower flank of the cap (right-cap frame). The chain's
    # first link is pinned here; the boss mirrors with the 180 deg cap rotation.
    bx, by, bz = CHAIN_BOSS_LOCAL
    boss = cq.Workplane("XY").workplane(offset=bz).center(bx, by).sphere(0.007)
    cap = cap.union(boss)
    if direction < 0:
        cap = cap.rotate((0, 0, 0), (0, 0, 1), 180.0)
    return cap


def _wheel_shape() -> cq.Workplane:
    """Spoked cast-iron handwheel: rim torus + hub + radial spokes.

    Authored centered at the origin in the hub-local frame (axis = +Z), so the
    articulation origin places the whole wheel at the stem top.
    """
    rim = cq.Workplane("XY").toPending()
    rim = cq.Solid.makeTorus(WHEEL_RIM_R, WHEEL_RIM_TUBE)
    rim = cq.Workplane(obj=rim)

    # Solid hub spanning the wheel plane. Its lower half reaches below the wheel
    # plane so it swallows the top of the brass valve stem (a real captured-stem
    # fit, allowed in tests).
    hub = (
        cq.Workplane("XY")
        .workplane(offset=-WHEEL_HUB_H * 0.6)
        .circle(WHEEL_HUB_R)
        .extrude(WHEEL_HUB_H * 1.2)
    )

    wheel = rim.union(hub)

    # Radial spokes connecting hub to rim, lying in the wheel plane (z=0).
    for i in range(WHEEL_N_SPOKES):
        ang = 2.0 * math.pi * i / WHEEL_N_SPOKES
        # A cylinder along +Z, rotated to lie radially, then placed at mid-radius.
        spoke_len = WHEEL_RIM_R - WHEEL_HUB_R + 0.004
        spoke = cq.Workplane("XY").circle(WHEEL_SPOKE_R).extrude(spoke_len)
        # Lay the spoke flat (along +X) then rotate about Z by ang.
        spoke = spoke.rotate((0, 0, 0), (0, 1, 0), 90.0)  # now along +X from origin
        spoke = spoke.translate((WHEEL_HUB_R - 0.002, 0.0, 0.0))
        spoke = spoke.rotate((0, 0, 0), (0, 0, 1), math.degrees(ang))
        wheel = wheel.union(spoke)

    return wheel


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fire_hydrant_standpipe")
    _materials(model)

    # ----- Root: standpipe body (riser + collar + cast body + bonnet + nozzles)
    body = model.part("standpipe_body")

    body.visual(
        mesh_from_cadquery(_riser_shape(), "riser_pipe", tolerance=TOL),
        material="hydrant_orange",
        name="riser_pipe",
    )
    body.visual(
        mesh_from_cadquery(_collar_shape(), "body_collar", tolerance=TOL),
        material="cast_iron",
        name="body_collar",
    )
    body.visual(
        mesh_from_cadquery(_body_shape(), "valve_body", tolerance=TOL),
        material="hydrant_red",
        name="valve_body",
    )
    body.visual(
        mesh_from_cadquery(_bonnet_neck_shape(), "bonnet_neck", tolerance=TOL),
        material="hydrant_red",
        name="bonnet_neck",
    )
    body.visual(
        mesh_from_cadquery(_dome_stem_shape(), "dome_stem", tolerance=TOL),
        material="brass",
        name="dome_stem",
    )
    for i in range(OUTLET_COUNT):
        # The edited variant has N=1: a single capped tee take-off on +X.  The
        # loop keeps the outlet/cap/chain policy regular and prevents the old
        # opposite-side branch from being authored.
        direction = 1
        body.visual(
            mesh_from_cadquery(_nozzle_shape(direction), f"nozzle_{i}", tolerance=TOL),
            material="brass",
            name=f"nozzle_{i}",
        )
        body.visual(
            mesh_from_cadquery(_chain_lug_shape(direction), f"chain_lug_{i}", tolerance=TOL),
            material="cast_iron",
            name=f"chain_lug_{i}",
        )

    # ----- Handwheel (hero mechanism)
    wheel = model.part("handwheel")
    wheel.visual(
        mesh_from_cadquery(_wheel_shape(), "handwheel", tolerance=TOL),
        material="cast_iron",
        name="handwheel",
    )

    model.articulation(
        "body_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=wheel,
        origin=Origin(xyz=(0.0, WHEEL_Y, WHEEL_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=4.0),
    )

    # ----- Brass outlet cap and retaining chain, emitted from the same single
    # outlet loop (N == 1). The cap is shown pulled out along the nozzle like
    # the parent mechanism; negative travel slides it back to the nozzle
    # mouth/contact surface.
    for i in range(OUTLET_COUNT):
        direction = 1
        cap_part = model.part(f"cap_{i}")
        cap_part.visual(
            mesh_from_cadquery(_cap_shape(direction), f"cap_{i}", tolerance=TOL),
            material="brass",
            name=f"cap_{i}",
        )

        model.articulation(
            f"body_to_cap_{i}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=cap_part,
            origin=Origin(xyz=(CAP_HINGE_X + CAP_DEFAULT_PULL, 0.0, NOZZLE_Z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=0.25, lower=-CAP_DEFAULT_PULL, upper=0.020),
        )

        # Retaining chain: ONE continuous oval-link chain, riveted to the CAP at
        # its small ball (root link jointed to the cap, so the chain follows the
        # cap and never detaches from it) and running down to wrap the body eye.
        # A single serial chain means consecutive links always interlink and it
        # can never break in the middle.
        sign = 1.0 if direction > 0 else -1.0
        eye_world = (sign * CHAIN_LUG[0], sign * CHAIN_LUG[1], CHAIN_LUG[2])
        boss_local = _cap_chain_anchor(direction)
        cap_origin_world = (sign * (CAP_HINGE_X + CAP_DEFAULT_PULL), 0.0, NOZZLE_Z)
        boss_world = (
            cap_origin_world[0] + boss_local[0],
            cap_origin_world[1] + boss_local[1],
            cap_origin_world[2] + boss_local[2],
        )
        dir_ball_to_eye = (
            eye_world[0] - boss_world[0],
            eye_world[1] - boss_world[1],
            eye_world[2] - boss_world[2],
        )
        _add_subchain(
            model,
            f"chain_{i}",
            cap_part,
            Origin(xyz=boss_local, rpy=_rpy_aim_negz(dir_ball_to_eye)),
            CHAIN_LINKS,
        )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("standpipe_body")
    wheel = object_model.get_part("handwheel")
    cap = object_model.get_part("cap_0")

    def _chain_links(prefix):
        return [object_model.get_part(f"{prefix}_{i}") for i in range(CHAIN_LINKS)]

    def _chain_joints(prefix):
        return [object_model.get_articulation(f"{prefix}_swing_{i}") for i in range(CHAIN_LINKS)]

    # One continuous retaining chain, riveted to the cap and wrapped on the body eye.
    chain_links = _chain_links("chain_0")

    wheel_joint = object_model.get_articulation("body_to_wheel")
    cap_joint = object_model.get_articulation("body_to_cap_0")

    # --- Joint type / axis contracts -------------------------------------
    ctx.check(
        "handwheel joint is continuous about Z",
        str(wheel_joint.joint_type).lower().endswith("continuous")
        and tuple(round(a, 3) for a in wheel_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={wheel_joint.joint_type}, axis={wheel_joint.axis}",
    )
    ctx.check(
        "single cap joint is prismatic along the nozzle axis (+X)",
        str(cap_joint.joint_type).lower().endswith("prismatic")
        and tuple(round(a, 3) for a in cap_joint.axis) == (1.0, 0.0, 0.0)
        and cap_joint.motion_limits is not None
        and abs(cap_joint.motion_limits.lower + CAP_DEFAULT_PULL) < 1e-6
        and abs(cap_joint.motion_limits.upper - 0.020) < 1e-6,
        details=f"type={cap_joint.joint_type}, axis={cap_joint.axis}, limits={cap_joint.motion_limits}",
    )
    for i, joint in enumerate(_chain_joints("chain_0")):
        expected_axis = (1.0, 0.0, 0.0) if i % 2 == 1 else (0.0, 1.0, 0.0)
        ctx.check(
            f"chain_0 link {i} is a revolute oval-link joint",
            str(joint.joint_type).lower().endswith("revolute")
            and tuple(round(a, 3) for a in joint.axis) == expected_axis
            and joint.motion_limits is not None
            and abs(joint.motion_limits.lower + CHAIN_SWING) < 1e-6
            and abs(joint.motion_limits.upper - CHAIN_SWING) < 1e-6,
            details=f"type={joint.joint_type}, axis={joint.axis}, limits={joint.motion_limits}",
        )

    # --- Hero geometry: tall riser, bulbous body, spoked wheel on top ------
    body_aabb = ctx.part_world_aabb(body)
    wheel_aabb = ctx.part_world_aabb(wheel)
    assert body_aabb is not None and wheel_aabb is not None

    body_height = body_aabb[1][2] - body_aabb[0][2]
    ctx.check(
        "standpipe is tall (riser + body)",
        body_height > 0.55,
        details=f"body height={body_height:.3f}",
    )

    # Wheel sits above the body (at the top), and is wide enough to read spoked.
    wheel_width = wheel_aabb[1][0] - wheel_aabb[0][0]
    ctx.check(
        "handwheel is a wide rim at the top",
        wheel_aabb[0][2] > body_aabb[1][2] - 0.06 and wheel_width > 0.12,
        details=f"wheel z0={wheel_aabb[0][2]:.3f}, body z1={body_aabb[1][2]:.3f}, "
        f"wheel width={wheel_width:.3f}",
    )

    # Riser bore: the riser is hollow (narrower part width than the body bulge).
    riser_aabb = ctx.part_element_world_aabb(body, elem="riser_pipe")
    valve_aabb = ctx.part_element_world_aabb(body, elem="valve_body")
    assert riser_aabb is not None and valve_aabb is not None
    riser_w = riser_aabb[1][0] - riser_aabb[0][0]
    valve_w = valve_aabb[1][0] - valve_aabb[0][0]
    ctx.check(
        "valve body bulges wider than the riser",
        valve_w > riser_w + 0.02,
        details=f"valve width={valve_w:.3f}, riser width={riser_w:.3f}",
    )

    # --- Outlet nozzle edit: exactly one brass side take-off, with one cap ----
    body_visual_names = [v.name for v in body.visuals]
    part_names = [p.name for p in object_model.parts]
    ctx.check(
        "outlet loop emits exactly one side nozzle",
        [name for name in body_visual_names if name.startswith("nozzle_")] == ["nozzle_0"]
        and OUTLET_COUNT == 1,
        details=f"outlet visuals={[name for name in body_visual_names if name.startswith('nozzle_')]}",
    )
    ctx.check(
        "outlet loop emits exactly one cap and one retaining chain",
        "cap_0" in part_names
        and not any(name in part_names for name in ("right_cap", "left_cap", "cap_1"))
        and all(f"chain_0_{i}" in part_names for i in range(CHAIN_LINKS))
        and not any(name.startswith("chain_1_") or name.startswith("right_chain_") or name.startswith("left_chain_") for name in part_names),
        details=f"parts={part_names}",
    )

    nz = ctx.part_element_world_aabb(body, elem="nozzle_0")
    assert nz is not None
    ctx.check(
        "single nozzle points +X as the one tee take-off",
        nz[1][0] > 0.08 and nz[0][0] > -0.01,
        details=f"nozzle_0 x_min={nz[0][0]:.3f}, x_max={nz[1][0]:.3f}",
    )

    # The cap defaults pulled out along the outlet axis (uncovering the nozzle),
    # not lifted vertically: the cap clears the nozzle tip in X and stays
    # centered on the outlet height in Z.
    cap_default = ctx.part_world_aabb(cap)
    assert cap_default is not None
    cap_cz = (cap_default[0][2] + cap_default[1][2]) / 2.0
    ctx.check(
        "single cap defaults pulled outboard past the nozzle tip (not lifted)",
        cap_default[0][0] > NOZZLE_TIP_X - 0.02 and abs(cap_cz - NOZZLE_Z) < 0.05,
        details=f"cap x=({cap_default[0][0]:.3f}, {cap_default[1][0]:.3f}), "
        f"tip_x={NOZZLE_TIP_X:.3f}, nozzle_z={NOZZLE_Z:.3f}",
    )
    ctx.expect_gap(
        cap,
        body,
        axis="x",
        min_gap=0.010,
        name="default-open cap clears the nozzle tip along X",
    )

    with ctx.pose({cap_joint: -CAP_DEFAULT_PULL}):
        seated_aabb = ctx.part_world_aabb(cap)
        assert seated_aabb is not None
        seated_cz = (seated_aabb[0][2] + seated_aabb[1][2]) / 2.0
        ctx.expect_overlap(
            cap,
            body,
            axes="yz",
            min_overlap=0.03,
            name="single cap seats over the single nozzle",
        )
        ctx.check(
            "single cap mouth reaches the nozzle tip when re-seated",
            seated_aabb[0][0] < NOZZLE_TIP_X and seated_aabb[1][0] > NOZZLE_TIP_X
            and abs(seated_cz - NOZZLE_Z) < 0.05,
            details=f"seated cap x=({seated_aabb[0][0]:.3f}, {seated_aabb[1][0]:.3f}), "
            f"tip_x={NOZZLE_TIP_X:.3f}, z_center={seated_cz:.3f}",
        )

    # One continuous chain: root link jointed to the CAP at its small ball
    # (chain follows the cap, never detaches there); consecutive links interlink
    # so the chain can never break in the middle; the far link wraps the body eye.
    root_joint = object_model.get_articulation("chain_0_swing_0")
    ctx.check(
        "single chain root is jointed to the cap (follows the cap, cannot detach)",
        root_joint.parent == "cap_0",
        details=f"root parent={root_joint.parent}",
    )
    eye_aabb = ctx.part_element_world_aabb(body, elem="chain_lug_0")
    first_aabb = ctx.part_world_aabb(chain_links[0])
    last_aabb = ctx.part_world_aabb(chain_links[-1])
    cap_aabb = ctx.part_world_aabb(cap)
    assert eye_aabb is not None and first_aabb is not None and last_aabb is not None
    assert cap_aabb is not None
    first_c = [(first_aabb[0][k] + first_aabb[1][k]) / 2.0 for k in range(3)]
    last_c = [(last_aabb[0][k] + last_aabb[1][k]) / 2.0 for k in range(3)]
    eye_c = [(eye_aabb[0][k] + eye_aabb[1][k]) / 2.0 for k in range(3)]
    first_on_cap = all(
        first_aabb[0][k] <= cap_aabb[1][k] + 0.006 and first_aabb[1][k] >= cap_aabb[0][k] - 0.006
        for k in range(3)
    )
    ctx.check(
        "single chain root link sits on the cap ball",
        first_on_cap,
        details=f"root={tuple(round(v,3) for v in first_c)}",
    )
    link_aabbs = [ctx.part_world_aabb(lk) for lk in chain_links]
    continuous = all(
        a is not None
        and b is not None
        and all(a[0][k] <= b[1][k] and a[1][k] >= b[0][k] for k in range(3))
        for a, b in zip(link_aabbs, link_aabbs[1:])
    )
    ctx.check(
        "single chain is one continuous interlinked piece (no break)",
        continuous,
        details=f"link count={len(chain_links)}",
    )
    ctx.check(
        "single chain far link reaches the body eye",
        math.dist(last_c, eye_c) < 0.05 and last_c[2] < first_c[2] - 0.02,
        details=f"far={tuple(round(v,3) for v in last_c)}, eye={tuple(round(v,3) for v in eye_c)}",
    )
    ctx.check(
        "single chain hangs on the same side as its cap",
        last_c[0] > 0.0,
        details=f"far_x={last_c[0]:.3f}",
    )

    # --- Mechanism actuation -------------------------------------------------
    # Turning the handwheel rotates a rim point around the Z axis.
    rim_rest = ctx.part_world_aabb(wheel)
    with ctx.pose({wheel_joint: math.pi / 2.0}):
        rim_turned = ctx.part_world_aabb(wheel)
    assert rim_rest is not None and rim_turned is not None
    # The wheel is not perfectly axisymmetric in tessellation; confirm the
    # mechanism is posable and the wheel stays centered on the axis.
    ctx.check(
        "handwheel stays centered on its axis when turned",
        abs((rim_turned[0][0] + rim_turned[1][0]) / 2.0) < 0.01
        and abs((rim_turned[0][1] + rim_turned[1][1]) / 2.0 - WHEEL_Y) < 0.01,
        details=f"turned center x/y=({(rim_turned[0][0] + rim_turned[1][0]) / 2.0:.4f}, "
        f"{(rim_turned[0][1] + rim_turned[1][1]) / 2.0:.4f})",
    )

    # Sliding the cap from seated to open moves it straight along the nozzle
    # axis (in X), with no vertical lift.
    with ctx.pose({cap_joint: -CAP_DEFAULT_PULL}):
        cap_closed_aabb = ctx.part_world_aabb(cap)
    with ctx.pose({cap_joint: 0.0}):
        cap_open_aabb = ctx.part_world_aabb(cap)
    assert cap_closed_aabb is not None and cap_open_aabb is not None
    closed_cz = (cap_closed_aabb[0][2] + cap_closed_aabb[1][2]) / 2.0
    open_cz = (cap_open_aabb[0][2] + cap_open_aabb[1][2]) / 2.0
    closed_cx = (cap_closed_aabb[0][0] + cap_closed_aabb[1][0]) / 2.0
    open_cx = (cap_open_aabb[0][0] + cap_open_aabb[1][0]) / 2.0
    ctx.check(
        "single cap slides along the nozzle axis when opened (not lifted)",
        open_cx > closed_cx + 0.02 and abs(open_cz - closed_cz) < 0.004,
        details=f"closed center=({closed_cx:.3f},{closed_cz:.3f}), "
        f"open center=({open_cx:.3f},{open_cz:.3f})",
    )

    # --- Connectivity / clearance -------------------------------------------
    # Handwheel hub seats on the stem (captured pin-in-hub fit), allowed.
    ctx.allow_overlap(
        wheel,
        body,
        elem_a="handwheel",
        elem_b="dome_stem",
        reason="The handwheel hub is bored onto the brass valve stem; the stem "
        "is intentionally captured inside the hub bore.",
    )
    # The brass cap seats over the nozzle tip (captured seating fit).
    ctx.allow_overlap(
        cap,
        body,
        elem_a="cap_0",
        elem_b="nozzle_0",
        reason="The closed brass cap is screwed over the nozzle tip; the tip is "
        "intentionally captured inside the cap recess.",
    )
    # Root links are riveted onto the cap's small ball (chain follows cap).
    for root_link in chain_links[:2]:
        ctx.allow_overlap(
            root_link,
            cap,
            reason="The chain's root is riveted onto the cap's small ball; it follows the cap.",
        )
    # Far links wrap the body eye.
    for tail_link in chain_links[-2:]:
        ctx.allow_overlap(
            tail_link,
            body,
            reason="The chain's far end wraps the body eye on the hydrant.",
        )
    # Consecutive oval links interlink (intentional continuous chain).
    for a, b in zip(chain_links, chain_links[1:]):
        ctx.allow_overlap(
            a, b, reason="Consecutive oval links interlink, as in a real chain."
        )

    return ctx.report()


object_model = build_object_model()
