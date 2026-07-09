from __future__ import annotations

# Fire-hydrant standpipe (pillar / above-ground hydrant on a riser pipe), built
# faithfully from picture/Equipment/Pipeline/002.png.
#
# What the reference shows:
#   - A tall orange/red riser pipe (the standpipe) standing vertically.
#   - A bulbous red cast-iron valve body / bonnet seated on top of the riser.
#   - A large spoked cast-iron HANDWHEEL on a brass stem at the very top; turning
#     it operates the internal valve (the primary user mechanism).
#   - Three equiangular brass outlet nozzles around the body (pumper manifold),
#     each with a brass pull-off cap retained by a short articulated oval-link
#     chain. The caps slide straight outward along their nozzle axes to uncover
#     the outlets (the secondary mechanism).
#   - A small brass dome / packing nut at the very top under the wheel.
#
# Articulation:
#   - body_to_wheel : CONTINUOUS about +Z. Turning the handwheel operates the
#     valve. This is the hero mechanism.
#   - body_to_cap_i : PRISMATIC along each outlet's local radial axis. Each
#     brass cap pulls straight off along its nozzle to uncover the outlet (it
#     does NOT lift vertically).
#   - {side}_chain_swing_{i}: REVOLUTE link joints (adapted from the gutter
#     downchain link-chain sample). Each retaining chain is ONE continuous serial
#     chain whose root link is jointed to the CAP at its small ball (so the chain
#     follows the cap and never detaches from it -- pull the cap off and the chain
#     comes with it) and which runs down to wrap the body eye. Being a single
#     chain, consecutive links always interlink, so it can never break.
#
# Frame convention (real meters, Z-up):
#   +Z  -> up; the riser pipe foot sits at z = 0.
#   The three outlet nozzles are equally spaced about +Z, with cap_0 along +X.
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

# Outlet nozzles (brass), equally spaced as a 3-port pumper manifold.
OUTLET_N = 3
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
# body, so one end is tied to the hydrant and the other to the cap. Coordinates
# are in an outlet-local frame: +X radial outboard, +Y tangential, +Z up.
CHAIN_BOSS_LOCAL = (0.004, 0.026, -0.026)  # chain pin on each cap
CHAIN_LUG_LOCAL = (0.060, 0.034, 0.556)  # chain eye on each body shoulder
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
    CHAIN_LUG_LOCAL[0] - _cap_origin_r[0] - CHAIN_BOSS_LOCAL[0],
    CHAIN_LUG_LOCAL[1] - _cap_origin_r[1] - CHAIN_BOSS_LOCAL[1],
    CHAIN_LUG_LOCAL[2] - _cap_origin_r[2] - CHAIN_BOSS_LOCAL[2],
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
# Repeated outlet layout helpers
# ---------------------------------------------------------------------------


def _outlet_angle(i: int) -> float:
    """Equiangular outlet yaw, with outlet 0 pointing along +X."""
    return 2.0 * math.pi * i / OUTLET_N


def _radial(angle: float) -> tuple[float, float, float]:
    return (math.cos(angle), math.sin(angle), 0.0)


def _tangent(angle: float) -> tuple[float, float, float]:
    return (-math.sin(angle), math.cos(angle), 0.0)


def _outlet_local_to_body(angle: float, local: tuple[float, float, float]) -> tuple[float, float, float]:
    """Map outlet-local (radial, tangential, z) coordinates into the body frame."""
    r = _radial(angle)
    t = _tangent(angle)
    return (
        r[0] * local[0] + t[0] * local[1],
        r[1] * local[0] + t[1] * local[1],
        local[2],
    )


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


def _nozzle_shape(angle: float) -> cq.Workplane:
    """One brass outlet nozzle rotated about +Z by ``angle``.

    The nozzle root starts inside the body wall so it is captured by the body;
    the tip carries a flange ring with male thread relief.
    """
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
    return nozzle.rotate((0, 0, 0), (0, 0, 1), math.degrees(angle))


def _chain_lug_shape(angle: float) -> cq.Workplane:
    """Small eye on the body shoulder (below the outlet) that anchors a chain.

    The chain's far link wraps this eye so the retaining chain is tied to the
    hydrant body instead of being left dangling in the air.
    """
    lx, ly, lz = _outlet_local_to_body(angle, CHAIN_LUG_LOCAL)
    eye = (
        cq.Workplane("XY")
        .workplane(offset=lz)
        .center(lx, ly)
        .sphere(CHAIN_LUG_R)
    )
    return eye


def _cap_chain_direction_local() -> tuple[float, float, float]:
    """Direction from each cap boss to its body eye in the cap/outlet frame."""
    return (
        CHAIN_LUG_LOCAL[0] - (CAP_HINGE_X + CAP_DEFAULT_PULL) - CHAIN_BOSS_LOCAL[0],
        CHAIN_LUG_LOCAL[1] - CHAIN_BOSS_LOCAL[1],
        CHAIN_LUG_LOCAL[2] - NOZZLE_Z - CHAIN_BOSS_LOCAL[2],
    )


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


def _cap_shape() -> cq.Workplane:
    """Brass screw cap that closes one nozzle.

    A cup extending local +X from the local origin, with the open recessed mouth
    at local x=0 (facing local -X, toward the nozzle tip) and the closed outer
    disk at the far +X end. A small chain boss sits on the lower/tangential flank
    where the retaining chain's first link pins on. Each cap part is rotated by
    its joint origin so local +X is the outlet's radial direction.

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
    # Three equiangular outlet nozzles and body chain eyes: the only forked
    # structural change from the two-port parent is OUTLET_N = 3.
    for i in range(OUTLET_N):
        angle = _outlet_angle(i)
        body.visual(
            mesh_from_cadquery(_nozzle_shape(angle), f"nozzle_{i}", tolerance=TOL),
            material="brass",
            name=f"nozzle_{i}",
        )
        body.visual(
            mesh_from_cadquery(_chain_lug_shape(angle), f"chain_lug_{i}", tolerance=TOL),
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

    # ----- Brass outlet caps (secondary mechanism): each slides along its own
    # radial outlet axis. The caps are displayed pulled out along their nozzles
    # (mouth uncovered); negative joint travel slides them back in to re-seat.
    cap_parts = []
    for i in range(OUTLET_N):
        angle = _outlet_angle(i)
        cap = model.part(f"cap_{i}")
        cap.visual(
            mesh_from_cadquery(_cap_shape(), f"cap_{i}", tolerance=TOL),
            material="brass",
            name=f"cap_{i}",
        )
        cap_parts.append(cap)
        cap_origin = _outlet_local_to_body(angle, (CAP_HINGE_X + CAP_DEFAULT_PULL, 0.0, NOZZLE_Z))
        model.articulation(
            f"body_to_cap_{i}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=cap,
            origin=Origin(xyz=cap_origin, rpy=(0.0, 0.0, angle)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=0.25, lower=-CAP_DEFAULT_PULL, upper=0.020),
        )

    # ----- Retaining chains: ONE continuous oval-link chain per outlet, riveted
    # to the CAP at its small ball (root link jointed to the cap, so the chain
    # follows the cap and never detaches from it -- pull the cap off and the chain
    # comes with it) and running down to wrap the body eye. A single serial chain,
    # so consecutive links always interlink and it can never break in the middle.
    dir_ball_to_eye = _cap_chain_direction_local()
    for i, cap_part in enumerate(cap_parts):
        _add_subchain(
            model,
            f"chain_{i}",
            cap_part,
            Origin(xyz=CHAIN_BOSS_LOCAL, rpy=_rpy_aim_negz(dir_ball_to_eye)),
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
    cap_parts = [object_model.get_part(f"cap_{i}") for i in range(OUTLET_N)]
    cap_joints = [object_model.get_articulation(f"body_to_cap_{i}") for i in range(OUTLET_N)]

    def _chain_links(prefix):
        return [object_model.get_part(f"{prefix}_{i}") for i in range(CHAIN_LINKS)]

    def _chain_joints(prefix):
        return [object_model.get_articulation(f"{prefix}_swing_{i}") for i in range(CHAIN_LINKS)]

    def _aabb_center(aabb):
        return [(aabb[0][k] + aabb[1][k]) / 2.0 for k in range(3)]

    def _radial_value(point, angle):
        return point[0] * math.cos(angle) + point[1] * math.sin(angle)

    def _angle_delta(a, b):
        return abs((a - b + math.pi) % (2.0 * math.pi) - math.pi)

    # One continuous chain per outlet, riveted to the body at the eye.
    chains = [_chain_links(f"chain_{i}") for i in range(OUTLET_N)]

    wheel_joint = object_model.get_articulation("body_to_wheel")

    # --- Joint type / axis contracts -------------------------------------
    ctx.check(
        "handwheel joint is continuous about Z",
        str(wheel_joint.joint_type).lower().endswith("continuous")
        and tuple(round(a, 3) for a in wheel_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={wheel_joint.joint_type}, axis={wheel_joint.axis}",
    )
    ctx.check(
        "pumper manifold has exactly three independent prismatic caps",
        len(cap_parts) == 3
        and len({cap.name for cap in cap_parts}) == 3
        and all(str(joint.joint_type).lower().endswith("prismatic") for joint in cap_joints),
        details=f"caps={[cap.name for cap in cap_parts]}, joints={[joint.name for joint in cap_joints]}",
    )
    for i, joint in enumerate(cap_joints):
        ctx.check(
            f"cap_{i} joint uses the shared radial prismatic policy",
            str(joint.joint_type).lower().endswith("prismatic")
            and tuple(round(a, 3) for a in joint.axis) == (1.0, 0.0, 0.0)
            and joint.motion_limits is not None
            and abs(joint.motion_limits.lower + CAP_DEFAULT_PULL) < 1e-6
            and abs(joint.motion_limits.upper - 0.020) < 1e-6,
            details=f"type={joint.joint_type}, axis={joint.axis}, limits={joint.motion_limits}",
        )
    for prefix in (f"chain_{i}" for i in range(OUTLET_N)):
        for i, joint in enumerate(_chain_joints(prefix)):
            expected_axis = (1.0, 0.0, 0.0) if i % 2 == 1 else (0.0, 1.0, 0.0)
            ctx.check(
                f"{prefix} link {i} is a revolute oval-link joint",
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

    # --- Outlet manifold: three equiangular nozzles, caps, and body chain eyes.
    nozzle_centers = []
    cap_centers = []
    for i, cap in enumerate(cap_parts):
        angle = _outlet_angle(i)
        nozzle_aabb = ctx.part_element_world_aabb(body, elem=f"nozzle_{i}")
        lug_aabb = ctx.part_element_world_aabb(body, elem=f"chain_lug_{i}")
        cap_aabb = ctx.part_world_aabb(cap)
        assert nozzle_aabb is not None and lug_aabb is not None and cap_aabb is not None
        nozzle_c = _aabb_center(nozzle_aabb)
        lug_c = _aabb_center(lug_aabb)
        cap_c = _aabb_center(cap_aabb)
        nozzle_centers.append(nozzle_c)
        cap_centers.append(cap_c)
        nozzle_angle = math.atan2(nozzle_c[1], nozzle_c[0])
        cap_angle = math.atan2(cap_c[1], cap_c[0])
        ctx.check(
            f"nozzle_{i} points on its equiangular radial",
            _angle_delta(nozzle_angle, angle) < math.radians(8.0)
            and _radial_value(nozzle_c, angle) > 0.050,
            details=f"center={tuple(round(v,3) for v in nozzle_c)}, expected_angle={angle:.3f}",
        )
        ctx.check(
            f"cap_{i} sits outboard on the same radial as nozzle_{i}",
            _angle_delta(cap_angle, angle) < math.radians(8.0)
            and _radial_value(cap_c, angle) > _radial_value(nozzle_c, angle) + 0.030
            and abs(cap_c[2] - NOZZLE_Z) < 0.05,
            details=f"cap={tuple(round(v,3) for v in cap_c)}, nozzle={tuple(round(v,3) for v in nozzle_c)}",
        )
        ctx.check(
            f"chain_lug_{i} is mounted below its outlet shoulder",
            _angle_delta(math.atan2(lug_c[1], lug_c[0]), angle) < math.radians(35.0)
            and lug_c[2] < cap_c[2],
            details=f"lug={tuple(round(v,3) for v in lug_c)}, cap={tuple(round(v,3) for v in cap_c)}",
        )

    outlet_angles = sorted(math.atan2(c[1], c[0]) % (2.0 * math.pi) for c in nozzle_centers)
    outlet_gaps = [
        (outlet_angles[(i + 1) % OUTLET_N] - outlet_angles[i]) % (2.0 * math.pi)
        for i in range(OUTLET_N)
    ]
    ctx.check(
        "three outlets are equally spaced at 120 degrees",
        all(abs(gap - 2.0 * math.pi / OUTLET_N) < math.radians(8.0) for gap in outlet_gaps),
        details=f"angles={[round(a,3) for a in outlet_angles]}, gaps={[round(g,3) for g in outlet_gaps]}",
    )

    # Caps default pulled OUT along the outlet axis (uncovering the nozzle), not
    # lifted vertically: each cap clears the nozzle tip radially and stays
    # centered on the outlet height in Z.
    for i, cap_c in enumerate(cap_centers):
        angle = _outlet_angle(i)
        ctx.check(
            f"cap_{i} defaults pulled outboard past nozzle_{i} (not lifted)",
            _radial_value(cap_c, angle) > NOZZLE_TIP_X + 0.025
            and abs(cap_c[2] - NOZZLE_Z) < 0.05,
            details=f"cap_center={tuple(round(v,3) for v in cap_c)}, tip_r={NOZZLE_TIP_X:.3f}",
        )

    # One continuous chain per outlet: root link jointed to the CAP at its small
    # ball (chain follows the cap, never detaches there); consecutive links
    # interlink so the chain can never break in the middle; the far link wraps the
    # body eye.
    for i, (chain_links, cap_part) in enumerate(zip(chains, cap_parts)):
        angle = _outlet_angle(i)
        root_joint = object_model.get_articulation(f"chain_{i}_swing_0")
        ctx.check(
            f"chain_{i} root is jointed to cap_{i} (follows the cap, cannot detach)",
            root_joint.parent == f"cap_{i}",
            details=f"root parent={root_joint.parent}",
        )
        eye_aabb = ctx.part_element_world_aabb(body, elem=f"chain_lug_{i}")
        first_aabb = ctx.part_world_aabb(chain_links[0])
        last_aabb = ctx.part_world_aabb(chain_links[-1])
        cap_aabb = ctx.part_world_aabb(cap_part)
        assert eye_aabb is not None and first_aabb is not None and last_aabb is not None
        assert cap_aabb is not None
        first_c = _aabb_center(first_aabb)
        last_c = _aabb_center(last_aabb)
        eye_c = _aabb_center(eye_aabb)
        # Root link sits on the cap's small ball (its AABB intersects the cap).
        first_on_cap = all(
            first_aabb[0][k] <= cap_aabb[1][k] + 0.006 and first_aabb[1][k] >= cap_aabb[0][k] - 0.006
            for k in range(3)
        )
        ctx.check(
            f"chain_{i} root link sits on cap_{i} ball",
            first_on_cap,
            details=f"root={tuple(round(v,3) for v in first_c)}",
        )
        # Consecutive links interlink (each pair's AABBs overlap) -> continuous.
        link_aabbs = [ctx.part_world_aabb(lk) for lk in chain_links]
        continuous = all(
            a is not None
            and b is not None
            and all(a[0][k] <= b[1][k] and a[1][k] >= b[0][k] for k in range(3))
            for a, b in zip(link_aabbs, link_aabbs[1:])
        )
        ctx.check(
            f"chain_{i} is one continuous interlinked piece (no break)",
            continuous,
            details=f"link count={len(chain_links)}",
        )
        # Far link reaches/wraps the body eye.
        ctx.check(
            f"chain_{i} far link reaches the body eye",
            math.dist(last_c, eye_c) < 0.05 and last_c[2] < first_c[2] - 0.02,
            details=f"far={tuple(round(v,3) for v in last_c)}, eye={tuple(round(v,3) for v in eye_c)}",
        )
        ctx.check(
            f"chain_{i} hangs on the same radial sector as cap_{i}",
            _angle_delta(math.atan2(last_c[1], last_c[0]), angle) < math.radians(35.0),
            details=f"far={tuple(round(v,3) for v in last_c)}, angle={angle:.3f}",
        )

    # Re-seated caps (slid back in along the outlet axis) overlap their nozzles
    # in projection (seated on the tip).
    with ctx.pose({joint: -CAP_DEFAULT_PULL for joint in cap_joints}):
        for i, cap in enumerate(cap_parts):
            ctx.expect_overlap(
                cap,
                body,
                axes="xyz",
                elem_a=f"cap_{i}",
                elem_b=f"nozzle_{i}",
                min_overlap=0.010,
                name=f"cap_{i} seats over nozzle_{i}",
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

    # Sliding each cap from seated to default-open moves it straight outward
    # along its nozzle radial, with no vertical lift.
    for i, (cap, joint) in enumerate(zip(cap_parts, cap_joints)):
        angle = _outlet_angle(i)
        with ctx.pose({joint: -CAP_DEFAULT_PULL}):
            cap_closed_aabb = ctx.part_world_aabb(cap)
        with ctx.pose({joint: 0.0}):
            cap_open_aabb = ctx.part_world_aabb(cap)
        assert cap_closed_aabb is not None and cap_open_aabb is not None
        closed_c = _aabb_center(cap_closed_aabb)
        open_c = _aabb_center(cap_open_aabb)
        radial_travel = _radial_value(open_c, angle) - _radial_value(closed_c, angle)
        tangent_drift = abs(
            (open_c[0] * -math.sin(angle) + open_c[1] * math.cos(angle))
            - (closed_c[0] * -math.sin(angle) + closed_c[1] * math.cos(angle))
        )
        ctx.check(
            f"cap_{i} slides along its nozzle radial when opened (not lifted)",
            radial_travel > 0.02 and tangent_drift < 0.004 and abs(open_c[2] - closed_c[2]) < 0.004,
            details=f"closed={tuple(round(v,3) for v in closed_c)}, "
            f"open={tuple(round(v,3) for v in open_c)}, travel={radial_travel:.3f}",
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
    # Lowered brass caps seat over the nozzle tips (captured seating fit).
    for i, (cap, chain_links) in enumerate(zip(cap_parts, chains)):
        ctx.allow_overlap(
            cap,
            body,
            elem_a=f"cap_{i}",
            elem_b=f"nozzle_{i}",
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
