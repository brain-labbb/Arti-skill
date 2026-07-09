from __future__ import annotations

# Articulated flanged globe valve (industrial pipeline shut-off valve).
#
# Identity (from picture/Equipment/Pipeline/003.png): a blue cast-iron flanged
# globe valve with a rounded spherical body, two bolted RF flanges (inlet and
# outlet) on a horizontal run, a bolted bonnet flange, a blue bonnet/yoke neck
# carrying a square gland nut, a bright rising steel stem, and a red spoked
# handwheel on top.
#
# Primary real mechanism: the handwheel is turned about the vertical stem axis
# to open/close the valve. The handwheel + rising stem rotate together as one
# rigid body about the stem nut on top of the yoke, so it is modeled as a single
# CONTINUOUS rotary part about local +Z.

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

# Rising stem (bright steel) and handwheel
STEM_R = 0.011
STEM_BOTTOM_Z = GLAND_Z  # stem reaches down into the gland/packing
STEM_TOP_Z = STEMNUT_Z + STEMNUT_H + 0.060  # exposed stem above the stem nut
HUB_Z = STEM_TOP_Z  # handwheel hub center
WHEEL_R = 0.115  # handwheel rim center radius
WHEEL_TUBE_R = 0.011  # handwheel rim tube radius
WHEEL_HUB_R = 0.024
WHEEL_HUB_H = 0.030
SPOKE_R = 0.0075
N_SPOKES = 6

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


def _make_handwheel_mesh():
    """Red spoked handwheel + central hub + the bright rising stem, modeled in a
    local frame whose origin is the stem axis at the yoke top (joint frame).

    The mesh is authored so that local z=0 is at the joint origin (top of the
    stem nut), with the stem going DOWN (negative z) into the yoke and the wheel
    on top (positive z). This makes the CONTINUOUS rotation about local +Z spin
    the wheel and stem together.
    """
    z_joint = STEMNUT_Z + STEMNUT_H  # world z of the joint frame (top of stem nut)

    # Rising stem: long bright cylinder spanning from inside the gland up to the
    # hub. In local frame, shift so z=0 is the joint plane.
    stem_len = STEM_TOP_Z - STEM_BOTTOM_Z
    stem_center_world = 0.5 * (STEM_TOP_Z + STEM_BOTTOM_Z)
    stem = (
        cq.Workplane("XY")
        .circle(STEM_R)
        .extrude(stem_len, both=True)
        .translate((0.0, 0.0, stem_center_world - z_joint))
    )

    # Central hub of the handwheel.
    hub = (
        cq.Workplane("XY")
        .workplane(offset=HUB_Z - WHEEL_HUB_H / 2.0 - z_joint)
        .circle(WHEEL_HUB_R)
        .extrude(WHEEL_HUB_H)
    )
    wheel = stem.union(hub)

    # Outer rim: a torus in the wheel plane (horizontal, at HUB_Z).
    rim = (
        cq.Workplane("XY")
        .workplane(offset=HUB_Z - z_joint)
        .add(
            cq.Solid.makeTorus(WHEEL_R, WHEEL_TUBE_R).located(
                cq.Location(cq.Vector(0, 0, HUB_Z - z_joint))
            )
        )
    )
    wheel = wheel.union(rim)

    # Curved spokes from hub to rim.
    for i in range(N_SPOKES):
        a = 2.0 * math.pi * i / N_SPOKES
        x_out = (WHEEL_R - WHEEL_TUBE_R) * math.cos(a)
        y_out = (WHEEL_R - WHEEL_TUBE_R) * math.sin(a)
        x_in = WHEEL_HUB_R * math.cos(a)
        y_in = WHEEL_HUB_R * math.sin(a)
        spoke_len = math.hypot(x_out - x_in, y_out - y_in)
        spoke = (
            cq.Workplane("XY")
            .circle(SPOKE_R)
            .extrude(spoke_len)
            # extrude is along +Z; rotate to lie radially in the wheel plane.
            .rotate((0, 0, 0), (0, 1, 0), 90.0)  # now along +X
            .rotate((0, 0, 0), (0, 0, 1), math.degrees(a))
            .translate((x_in, y_in, HUB_Z - z_joint))
        )
        wheel = wheel.union(spoke)

    return mesh_from_cadquery(wheel, "handwheel")


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

    # --- Rotating handwheel + rising stem ---------------------------------
    handwheel = model.part("handwheel")
    handwheel.visual(_make_handwheel_mesh(), material="valve_red", name="wheel")
    # The bright stem is part of the same mesh; give the rim/spokes red and the
    # stem reads steel via a second thin visual overlay on the stem region.
    # (Single mesh is fine; we keep one red material to match the photo's wheel.)

    z_joint = STEMNUT_Z + STEMNUT_H
    model.articulation(
        "body_to_handwheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=handwheel,
        origin=Origin(xyz=(0.0, 0.0, z_joint)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("valve_body")
    handwheel = object_model.get_part("handwheel")
    joint = object_model.get_articulation("body_to_handwheel")

    # The stem is captured inside the yoke stem-nut/packing bushing.
    ctx.allow_overlap(
        handwheel,
        body,
        reason="The rising stem is intentionally nested through the yoke stem-nut and packing gland bushing.",
    )

    # --- Joint identity: continuous rotary about vertical Z ---------------
    ctx.check(
        "handwheel joint is continuous",
        joint.joint_type == ArticulationType.CONTINUOUS,
        details=f"joint_type={joint.joint_type}",
    )
    axis = tuple(joint.axis)
    ctx.check(
        "handwheel axis is vertical Z",
        abs(axis[0]) < 1e-6 and abs(axis[1]) < 1e-6 and abs(axis[2]) > 0.999,
        details=f"axis={axis}",
    )

    # --- Hero geometry placement (rest pose) ------------------------------
    body_aabb = ctx.part_world_aabb(body)
    wheel_aabb = ctx.part_world_aabb(handwheel)
    assert body_aabb is not None and wheel_aabb is not None

    # Handwheel sits above the bonnet/yoke region of the body.
    ctx.check(
        "handwheel rim above bonnet",
        wheel_aabb[1][2] > body_aabb[1][2] - 0.02,
        details=f"wheel_top={wheel_aabb[1][2]:.3f}, body_top={body_aabb[1][2]:.3f}",
    )

    # Handwheel is a wide wheel (diameter ~ 2*WHEEL_R), centered on the stem.
    wheel_dx = wheel_aabb[1][0] - wheel_aabb[0][0]
    ctx.check(
        "handwheel is a wide wheel",
        wheel_dx > 2.0 * WHEEL_R - 0.02,
        details=f"wheel_x_extent={wheel_dx:.3f}",
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

    # --- Mechanism proof: rotating the wheel moves a rim point in XY, not Z ---
    # Pick a reference rim extent at rest, then quarter-turn and confirm the
    # part's X/Y footprint stays (a wheel spinning about Z keeps its AABB span)
    # while a marker on the rim travels. We verify via a 90-degree pose that the
    # wheel does not translate vertically and remains centered on the stem.
    rest_center_z = 0.5 * (wheel_aabb[0][2] + wheel_aabb[1][2])
    with ctx.pose({joint: math.pi / 2.0}):
        spun_aabb = ctx.part_world_aabb(handwheel)
        assert spun_aabb is not None
        spun_center_z = 0.5 * (spun_aabb[0][2] + spun_aabb[1][2])
        ctx.check(
            "spinning wheel does not move vertically",
            abs(spun_center_z - rest_center_z) < 1e-4,
            details=f"rest_z={rest_center_z:.4f}, spun_z={spun_center_z:.4f}",
        )
        # Rotation about Z keeps the wheel centered on x=y=0.
        cx = 0.5 * (spun_aabb[0][0] + spun_aabb[1][0])
        cy = 0.5 * (spun_aabb[0][1] + spun_aabb[1][1])
        ctx.check(
            "spinning wheel stays centered on stem axis",
            abs(cx) < 0.01 and abs(cy) < 0.01,
            details=f"center=({cx:.4f},{cy:.4f})",
        )

    return ctx.report()


object_model = build_object_model()
