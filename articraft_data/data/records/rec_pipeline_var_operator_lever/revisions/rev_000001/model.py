from __future__ import annotations

# Articulated flanged globe valve (industrial pipeline shut-off valve).
#
# Identity (from picture/Equipment/Pipeline/003.png): a blue cast-iron flanged
# globe valve with a rounded spherical body, two bolted RF flanges (inlet and
# outlet) on a horizontal run, a bolted bonnet flange, a blue bonnet/yoke neck
# carrying a square gland nut, a bright fixed steel stem, and a red straight
# quarter-turn lever operator on top.
#
# Primary real mechanism for this fork: a single straight lever handle turns
# about the vertical stem axis like a ball/butterfly operator.  The globe body,
# flanges, yoke, and stem remain the fixed parent; only the lever rotates about
# local +Z through a realistic quarter-turn range.

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
)

# ---------------------------------------------------------------------------
# Dimensions (meters). A nominal ~DN150 / 6" globe valve.
# ---------------------------------------------------------------------------

# Horizontal pipe run along X; valve centerline (port axis) at z = PORT_Z.
PORT_Z = 0.150  # height of the flange/port centerline above ground

# Flanges
FLANGE_R = 0.140  # outer radius of a port flange disc
FLANGE_T = 0.024  # flange disc thickness
FLANGE_FACE_R = 0.110  # raised-face radius
FLANGE_FACE_T = 0.004  # raised-face proud height
BORE_R = 0.078  # outer radius of the flange hub neck
BORE_INNER_R = 0.058  # inner pipe bore radius (hollow)
PORT_HALF_SPAN = 0.150  # distance from body center to each flange face (along X)
HUB_R = 0.066  # flange hub neck radius where it meets body

# Body (rounded globe)
BODY_R = 0.115  # spherical body radius
BODY_CENTER_Z = PORT_Z  # body centered on the port axis height

# Bonnet flange (bolted joint between body and bonnet), on top of the body
BONNET_FLANGE_R = 0.105
BONNET_FLANGE_T = 0.022
BONNET_FLANGE_Z = PORT_Z + 0.095  # top of body where bonnet bolts on

# Bonnet / yoke neck (blue) rising from the bonnet flange
YOKE_R = 0.052
YOKE_BASE_Z = BONNET_FLANGE_Z + BONNET_FLANGE_T
YOKE_TOP_Z = YOKE_BASE_Z + 0.085

# Square gland / packing nut on the yoke
GLAND_SIZE = 0.072  # across-flats of the square gland block
GLAND_H = 0.058
GLAND_Z = YOKE_BASE_Z + 0.018

# Stem-nut bushing on top of the yoke (the rising stem threads through this)
STEMNUT_R = 0.030
STEMNUT_H = 0.026
STEMNUT_Z = YOKE_TOP_Z  # base of the stem-nut sits on the yoke top

# Rising stem (bright steel) and straight lever operator
STEM_R = 0.011
STEM_BOTTOM_Z = GLAND_Z  # stem reaches down into the gland/packing
STEM_TOP_Z = STEMNUT_Z + STEMNUT_H + 0.060  # exposed stem above the stem nut
LEVER_HUB_R = 0.026
LEVER_HUB_H = 0.032
LEVER_SHORT_ARM = 0.045
LEVER_LONG_ARM = 0.165
LEVER_BAR_W = 0.030
LEVER_BAR_H = 0.014
LEVER_BAR_Z = 0.020

# Bolting
N_BOLTS = 8
BOLT_R = 0.0075
BOLT_HEAD_R = 0.012
BOLT_HEAD_H = 0.010

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------
VALVE_BLUE = (0.20, 0.42, 0.66, 1.0)
RED = (0.78, 0.13, 0.12, 1.0)
STEEL = (0.74, 0.76, 0.79, 1.0)
DARK_STEEL = (0.32, 0.34, 0.37, 1.0)


def _bolt_ring(plane: str, ring_r: float, center, count: int = N_BOLTS):
    """Return list of (x, y) or (y, z) etc. bolt centers on a circle."""
    pts = []
    for i in range(count):
        a = 2.0 * math.pi * i / count + math.pi / count
        pts.append((ring_r * math.cos(a), ring_r * math.sin(a)))
    return pts


def _make_body_mesh():
    """Blue cast-iron valve body: spherical globe, two flange hub necks, two
    bolted flange discs with raised faces and hollow bores, plus the bonnet
    flange that the bonnet bolts onto. Built around the port centerline along X
    at height PORT_Z, then the whole thing is authored in world meters."""

    # --- Spherical globe body, hollowed slightly along the flow path so the
    #     bore reads through. We model it as a solid sphere; flange bores cut
    #     the visible openings.
    body = (
        cq.Workplane("XY")
        .sphere(BODY_R)
        .translate((0.0, 0.0, BODY_CENTER_Z))
    )

    # Flatten the bottom a touch so the valve can read as seated (optional foot).
    # Keep it simple: leave the sphere.

    # --- Two flange hub necks + flange discs along +X and -X.
    def port(sign: float):
        # Hub neck: tapered cone from body hub (HUB_R, deep inside the sphere)
        # out to the flange back face (BORE_R). Absolute YZ planes are used so
        # the same logic works for the +X and -X ports.
        x_in = sign * (BODY_R - 0.055)  # neck start, well inside the sphere
        x_neck_end = sign * (PORT_HALF_SPAN - FLANGE_T)  # neck meets disc back
        x_face = sign * PORT_HALF_SPAN  # outer flange face plane
        a, b = sorted((x_in, x_neck_end))
        r_a = HUB_R if sign > 0 else BORE_R
        r_b = BORE_R if sign > 0 else HUB_R
        neck = (
            cq.Workplane("YZ")
            .workplane(offset=a)
            .circle(r_a)
            .workplane(offset=b - a)
            .circle(r_b)
            .loft(combine=True)
        )
        # Flange disc (full thickness, inner side toward the body).
        disc_lo = min(x_face, x_face - sign * FLANGE_T)
        disc = (
            cq.Workplane("YZ")
            .workplane(offset=disc_lo)
            .circle(FLANGE_R)
            .extrude(FLANGE_T)
        )
        # Raised face proud of the outer flange surface. It is extended a few
        # millimeters back INTO the disc so the boolean union merges them as one
        # volume (a flush coplanar touch would leave a separate tessellation
        # shell and read as a disconnected mesh island).
        rf_overlap = 0.003
        rf_off = (x_face - rf_overlap) if sign > 0 else (x_face - FLANGE_FACE_T)
        rf = (
            cq.Workplane("YZ")
            .workplane(offset=rf_off)
            .circle(FLANGE_FACE_R)
            .extrude(FLANGE_FACE_T + rf_overlap)
        )
        solid = neck.union(disc).union(rf).translate((0.0, 0.0, PORT_Z))
        return solid

    body = body.union(port(1.0)).union(port(-1.0))

    # --- Bonnet flange on top of the body (a thick disc).
    bonnet_flange = (
        cq.Workplane("XY")
        .workplane(offset=BONNET_FLANGE_Z)
        .circle(BONNET_FLANGE_R)
        .extrude(BONNET_FLANGE_T)
    )
    body = body.union(bonnet_flange)

    # --- Bore out the flow path along X so both flange faces read hollow.
    # The bore must fully clear the proud raised faces (which sit at
    # +/-(PORT_HALF_SPAN + FLANGE_FACE_T)); otherwise a thin capped ring of
    # raised-face material is left behind as a disconnected mesh island.
    bore_half = PORT_HALF_SPAN + FLANGE_FACE_T + 0.02
    bore = (
        cq.Workplane("YZ")
        .workplane(offset=-bore_half)
        .circle(BORE_INNER_R)
        .extrude(2.0 * bore_half)
        .translate((0.0, 0.0, PORT_Z))
    )
    body = body.cut(bore)

    # --- Bonnet-flange bolt holes (cosmetic shallow recesses around the ring).
    for (bx, by) in _bolt_ring("XY", BONNET_FLANGE_R - 0.018, None, count=N_BOLTS):
        hole = (
            cq.Workplane("XY")
            .workplane(offset=BONNET_FLANGE_Z - 0.001)
            .center(bx, by)
            .circle(BOLT_R)
            .extrude(BONNET_FLANGE_T + 0.002)
        )
        body = body.cut(hole)

    return mesh_from_cadquery(body, "valve_body")


def _make_bonnet_yoke_mesh():
    """The blue bonnet/yoke neck, square gland nut, and stem-nut bushing that
    rise from the bonnet flange and carry the rising stem. This is part of the
    fixed body assembly (does not rotate)."""

    # Bonnet neck: cylinder from bonnet flange up to yoke base.
    neck = (
        cq.Workplane("XY")
        .workplane(offset=BONNET_FLANGE_Z + BONNET_FLANGE_T - 0.004)
        .circle(YOKE_R)
        .extrude(YOKE_TOP_Z - (BONNET_FLANGE_Z + BONNET_FLANGE_T) + 0.004)
    )

    # Square gland / packing nut hugging the lower yoke.
    gland = (
        cq.Workplane("XY")
        .workplane(offset=GLAND_Z)
        .rect(GLAND_SIZE, GLAND_SIZE)
        .extrude(GLAND_H)
        .edges("|Z")
        .fillet(0.004)
    )
    yoke = neck.union(gland)

    # Stem-nut bushing (a short hex/round collar) on top of the yoke.
    stemnut = (
        cq.Workplane("XY")
        .workplane(offset=STEMNUT_Z)
        .polygon(6, 2.0 * STEMNUT_R)
        .extrude(STEMNUT_H)
    )
    yoke = yoke.union(stemnut)

    # Bore a vertical hole through the yoke for the stem to pass.
    stem_bore = (
        cq.Workplane("XY")
        .workplane(offset=BONNET_FLANGE_Z)
        .circle(STEM_R + 0.0015)
        .extrude(STEMNUT_Z + STEMNUT_H + 0.01)
    )
    yoke = yoke.cut(stem_bore)

    return mesh_from_cadquery(yoke, "bonnet_yoke")


def _make_bolts_mesh():
    """Eight hex bolt heads seated on the bonnet flange ring."""
    bolts = None
    for (bx, by) in _bolt_ring("XY", BONNET_FLANGE_R - 0.018, None, count=N_BOLTS):
        head = (
            cq.Workplane("XY")
            .workplane(offset=BONNET_FLANGE_Z + BONNET_FLANGE_T)
            .center(bx, by)
            .polygon(6, 2.0 * BOLT_HEAD_R)
            .extrude(BOLT_HEAD_H)
        )
        bolts = head if bolts is None else bolts.union(head)
    return mesh_from_cadquery(bolts, "bonnet_bolts")


def _make_stem_mesh():
    """Fixed bright rising stem and shoulder collar carried by the yoke.

    The stem is part of the parent valve assembly in this fork.  A small collar
    sits on the top face of the stem-nut bushing, so the steel stem is visibly
    mounted to the blue yoke instead of reading as a floating rod in the bore.
    """

    z_joint = STEM_TOP_Z
    stem_len = STEM_TOP_Z - STEM_BOTTOM_Z
    stem_center = 0.5 * (STEM_TOP_Z + STEM_BOTTOM_Z)
    stem = (
        cq.Workplane("XY")
        .workplane(offset=STEM_BOTTOM_Z)
        .circle(STEM_R)
        .extrude(stem_len)
    )

    # Shoulder washer/collar at the top of the stem-nut bushing.  It contacts
    # the yoke top annulus and ties the visible stem to the parent part.
    stemnut_top_z = STEMNUT_Z + STEMNUT_H
    collar = (
        cq.Workplane("XY")
        .workplane(offset=stemnut_top_z)
        .circle(LEVER_HUB_R * 0.72)
        .extrude(0.008)
    )

    # A slightly flattened top face gives the lever hub a real surface to bear
    # on at the joint origin.
    top_pad = (
        cq.Workplane("XY")
        .workplane(offset=z_joint - 0.006)
        .circle(LEVER_HUB_R * 0.62)
        .extrude(0.006)
    )

    stem = stem.union(collar).union(top_pad)
    return mesh_from_cadquery(stem, "fixed_stem")


def _make_lever_mesh():
    """Red straight quarter-turn lever operator in the local joint frame.

    Local z=0 is the bearing/contact plane on the top face of the fixed stem.
    The hub rises above that plane and a single straight bar runs across it,
    much like a ball- or butterfly-valve lever handle.
    """

    hub = cq.Workplane("XY").circle(LEVER_HUB_R).extrude(LEVER_HUB_H)

    total_len = LEVER_SHORT_ARM + LEVER_LONG_ARM
    center_x = 0.5 * (LEVER_LONG_ARM - LEVER_SHORT_ARM)
    bar = (
        cq.Workplane("XY")
        .workplane(offset=LEVER_BAR_Z)
        .center(center_x, 0.0)
        .rect(total_len, LEVER_BAR_W)
        .extrude(LEVER_BAR_H)
        .edges("|Z")
        .fillet(LEVER_BAR_W * 0.45)
    )

    # A rounded knob at the long end makes the operator read as a single handle,
    # not a wheel spoke.
    end_knob = (
        cq.Workplane("XY")
        .workplane(offset=LEVER_BAR_Z)
        .center(LEVER_LONG_ARM, 0.0)
        .circle(LEVER_BAR_W * 0.58)
        .extrude(LEVER_BAR_H)
    )

    lever = hub.union(bar).union(end_knob)
    return mesh_from_cadquery(lever, "lever_handle")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flanged_globe_valve")

    model.material("valve_blue", rgba=VALVE_BLUE)
    model.material("valve_red", rgba=RED)
    model.material("steel", rgba=STEEL)
    model.material("dark_steel", rgba=DARK_STEEL)

    # --- Fixed body assembly (root) ---------------------------------------
    body = model.part("valve_body")
    body.visual(_make_body_mesh(), material="valve_blue", name="body_shell")
    body.visual(_make_bonnet_yoke_mesh(), material="valve_blue", name="bonnet_yoke")
    body.visual(_make_bolts_mesh(), material="dark_steel", name="bonnet_bolts")
    body.visual(_make_stem_mesh(), material="steel", name="fixed_stem")

    # --- Rotating straight quarter-turn lever ------------------------------
    lever = model.part("lever_handle")
    lever.visual(_make_lever_mesh(), material="valve_red", name="lever_bar")

    z_joint = STEM_TOP_Z
    model.articulation(
        "body_to_lever",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lever,
        origin=Origin(xyz=(0.0, 0.0, z_joint)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=4.0, lower=0.0, upper=math.pi / 2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("valve_body")
    lever = object_model.get_part("lever_handle")
    joint = object_model.get_articulation("body_to_lever")

    # --- Joint identity: limited quarter-turn revolute about vertical Z ----
    ctx.check(
        "lever joint is revolute",
        joint.joint_type == ArticulationType.REVOLUTE,
        details=f"joint_type={joint.joint_type}",
    )
    axis = tuple(joint.axis)
    ctx.check(
        "lever axis is positive vertical Z",
        abs(axis[0]) < 1e-6 and abs(axis[1]) < 1e-6 and abs(axis[2]) > 0.999,
        details=f"axis={axis}",
    )
    limits = joint.motion_limits
    ctx.check(
        "lever has quarter-turn limits",
        limits is not None
        and abs(limits.lower - 0.0) < 1e-6
        and abs(limits.upper - math.pi / 2.0) < 1e-6,
        details=f"limits={limits}",
    )

    # --- Hero geometry placement (rest pose) ------------------------------
    body_aabb = ctx.part_world_aabb(body)
    lever_aabb = ctx.part_world_aabb(lever)
    assert body_aabb is not None and lever_aabb is not None

    # Lever sits directly on the fixed stem top and above the bonnet/yoke region.
    ctx.check(
        "lever above fixed yoke and stem",
        lever_aabb[0][2] >= body_aabb[1][2] - 0.001,
        details=f"lever_bottom={lever_aabb[0][2]:.3f}, body_top={body_aabb[1][2]:.3f}",
    )

    # The replacement operator is a single straight lever: long in X, narrow in
    # Y, with no wheel-like circular footprint.
    lever_dx = lever_aabb[1][0] - lever_aabb[0][0]
    lever_dy = lever_aabb[1][1] - lever_aabb[0][1]
    ctx.check(
        "operator is a straight narrow lever",
        lever_dx > 0.18 and lever_dy < 0.07 and lever_dx > 3.0 * lever_dy,
        details=f"lever_dx={lever_dx:.3f}, lever_dy={lever_dy:.3f}",
    )

    # The fixed parent still contains the visible steel rising stem.
    fixed_stem = body.get_visual("fixed_stem")
    stem_aabb = ctx.part_element_world_aabb(body, elem=fixed_stem)
    assert stem_aabb is not None
    ctx.check(
        "fixed stem reaches lever contact plane",
        abs(stem_aabb[1][2] - STEM_TOP_Z) < 0.002,
        details=f"stem_top={stem_aabb[1][2]:.4f}, expected={STEM_TOP_Z:.4f}",
    )
    ctx.expect_gap(
        lever,
        body,
        axis="z",
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem="lever_bar",
        negative_elem="fixed_stem",
        name="lever hub sits on stem top",
    )

    # Valve body spans the two flanges across X (port-to-port run).
    body_dx = body_aabb[1][0] - body_aabb[0][0]
    ctx.check(
        "body spans both flanges across X",
        body_dx > 2.0 * PORT_HALF_SPAN - 0.01,
        details=f"body_x_extent={body_dx:.3f}",
    )

    # Flange ports are hollow: a thin probe column along the port axis must pass
    # through the body bore. Check the body has the expected face radius via the
    # named shell visual extents on Y/Z at the port height.
    shell = body.get_visual("body_shell")
    shell_aabb = ctx.part_element_world_aabb(body, elem=shell)
    assert shell_aabb is not None
    ctx.check(
        "flange discs read at full diameter",
        (shell_aabb[1][1] - shell_aabb[0][1]) > 2.0 * FLANGE_R - 0.02,
        details=f"flange_y_extent={shell_aabb[1][1] - shell_aabb[0][1]:.3f}",
    )

    # --- Mechanism proof: the straight lever quarter-turns about +Z ----------
    rest_center_z = 0.5 * (lever_aabb[0][2] + lever_aabb[1][2])
    with ctx.pose({joint: math.pi / 2.0}):
        spun_aabb = ctx.part_world_aabb(lever)
        assert spun_aabb is not None
        spun_center_z = 0.5 * (spun_aabb[0][2] + spun_aabb[1][2])
        ctx.check(
            "quarter-turn lever does not move vertically",
            abs(spun_center_z - rest_center_z) < 1e-4,
            details=f"rest_z={rest_center_z:.4f}, spun_z={spun_center_z:.4f}",
        )
        spun_dx = spun_aabb[1][0] - spun_aabb[0][0]
        spun_dy = spun_aabb[1][1] - spun_aabb[0][1]
        ctx.check(
            "quarter-turn rotates lever footprint from X to Y",
            spun_dy > 0.18 and spun_dx < 0.07,
            details=f"spun_dx={spun_dx:.3f}, spun_dy={spun_dy:.3f}",
        )

    return ctx.report()


object_model = build_object_model()
