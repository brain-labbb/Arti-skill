from __future__ import annotations

# Realistic articulated desk pencil sharpener modeled on the reference image
# (a Caran d'Ache style hand-crank table sharpener). The primary user-facing
# mechanism is the side hand crank, modeled as a CONTINUOUS rotary joint about
# the horizontal crank axle. The dark charcoal housing is hollow at the front
# where the pencil port and helical cutter sit, and two clamp posts rise from
# the top deck.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Coordinate convention (housing/part frames, meters)
#   +X : toward the FRONT face (pencil port faces +X)
#   +Y : toward the RIGHT side where the crank lives
#   +Z : up
# ---------------------------------------------------------------------------

# Housing outer extents.
BODY_W = 0.090  # depth along X
BODY_D = 0.082  # width along Y
BODY_H = 0.078  # tall lower body
BODY_FILLET = 0.012

# Upper shoulder block that the crank axle and clamp posts attach to.
SHOULDER_H = 0.026
SHOULDER_W = 0.084
SHOULDER_D = 0.078

# Pencil port (front face) geometry.
PORT_CENTER_Z = 0.040
PORT_OUTER_R = 0.018
PORT_THROAT_R = 0.0075
PORT_DEPTH = 0.022

# Crank geometry.
AXLE_R = 0.0055
AXLE_LEN = 0.018  # exposed stub length outboard of the housing
CRANK_ARM_LEN = 0.034  # radial arm length (axle center to handle center)
CRANK_ARM_W = 0.0095
CRANK_ARM_T = 0.0055
HANDLE_R = 0.006
HANDLE_LEN = 0.022

# Right wall of the housing in part-local X-Y-Z; crank sits just outboard of it.
RIGHT_WALL_Y = BODY_D / 2.0
AXLE_Z = BODY_H + 0.006  # axle height (on the shoulder block, mid)


def _rounded_box(width: float, depth: float, height: float, fillet: float) -> cq.Workplane:
    """Solid rounded box centered on XY, base at z=0, vertical edges filleted."""
    return (
        cq.Workplane("XY")
        .box(width, depth, height, centered=(True, True, False))
        .edges("|Z")
        .fillet(fillet)
    )


def _build_housing() -> cq.Workplane:
    """Hollowed charcoal housing with a front pencil port and an upper shoulder."""
    body = _rounded_box(BODY_W, BODY_D, BODY_H, BODY_FILLET)

    # Upper shoulder block (slightly inset) that carries the crank + clamp posts.
    shoulder = (
        cq.Workplane("XY")
        .workplane(offset=BODY_H)
        .box(SHOULDER_W, SHOULDER_D, SHOULDER_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.010)
        .edges(">Z")
        .fillet(0.006)
    )
    body = body.union(shoulder)

    # Front pencil port: a conical insertion funnel cut into the +X face.
    port_axis = cq.Workplane("YZ").workplane(offset=BODY_W / 2.0 + 0.001)
    # Outer recess (shallow dish where the pencil rests).
    recess = (
        port_axis.center(0.0, PORT_CENTER_Z)
        .circle(PORT_OUTER_R)
        .extrude(-0.006)
    )
    body = body.cut(recess)
    # Tapered throat leading toward the cutter (funnel).
    throat = (
        cq.Workplane("YZ")
        .workplane(offset=BODY_W / 2.0 - 0.005)
        .center(0.0, PORT_CENTER_Z)
        .circle(PORT_OUTER_R - 0.004)
        .workplane(offset=-PORT_DEPTH)
        .circle(PORT_THROAT_R)
        .loft(combine=False)
    )
    body = body.cut(throat)

    # Axle bore through the right shoulder wall so the crank shaft reads as
    # entering the housing.
    axle_bore = (
        cq.Workplane("XZ")
        .workplane(offset=-(RIGHT_WALL_Y + 0.002))
        .center(0.0, AXLE_Z)
        .circle(AXLE_R + 0.0009)
        .extrude(0.020)
    )
    body = body.cut(axle_bore)

    return body


def _build_cutter() -> cq.Workplane:
    """Helical milling cutter visible inside the pencil port (along X).

    The cutter cylinder runs deep into the housing (rearward, -X) so it stays
    mechanically captured by the housing throat, and its visible front end sits
    a couple of mm recessed behind the front face.
    """
    # Cutter front end is recessed 0.004 behind the front face; the body runs
    # rearward (-X) deep into the housing interior.
    cutter_front_x = BODY_W / 2.0 - 0.004
    cutter_back_x = BODY_W / 2.0 - PORT_DEPTH - 0.012
    core = (
        cq.Workplane("YZ")
        .workplane(offset=cutter_back_x)
        .center(0.0, PORT_CENTER_Z)
        .circle(PORT_THROAT_R - 0.0008)
        .extrude(cutter_front_x - cutter_back_x)
    )
    # A few raised flutes (simplified helical milling teeth) on the front body.
    flutes = None
    for k in range(4):
        ang = k * math.pi / 2.0
        rib = (
            cq.Workplane("YZ")
            .workplane(offset=BODY_W / 2.0 - PORT_DEPTH - 0.001)
            .center(
                (PORT_THROAT_R - 0.0015) * math.cos(ang),
                PORT_CENTER_Z + (PORT_THROAT_R - 0.0015) * math.sin(ang),
            )
            .circle(0.0011)
            .extrude(0.013)
        )
        flutes = rib if flutes is None else flutes.union(rib)
    return core.union(flutes)


def _build_front_plate() -> cq.Workplane:
    """Thin badge plate on the lower front face (the CARAN D'ACHE logo strip)."""
    return (
        cq.Workplane("YZ")
        .workplane(offset=BODY_W / 2.0)
        .center(0.0, 0.016)
        .rect(0.044, 0.010)
        .extrude(0.0010)
        .edges("|X")
        .fillet(0.0015)
    )


def _build_clamp_post() -> cq.Workplane:
    """One cylindrical clamp/cover post that rises from the top deck."""
    deck_z = BODY_H + SHOULDER_H
    post = (
        cq.Workplane("XY")
        .workplane(offset=deck_z)
        .circle(0.0065)
        .extrude(0.014)
        .edges(">Z")
        .fillet(0.0035)
    )
    # Knurled cap groove (small recessed ring near the top).
    groove = (
        cq.Workplane("XY")
        .workplane(offset=deck_z + 0.0095)
        .circle(0.0066)
        .extrude(0.0012)
    )
    return post.cut(groove.intersect(post)).union(post)


def _build_crank() -> cq.Workplane:
    """Hand crank: axle stub + radial arm + perpendicular grip handle.

    Authored in a local frame whose ORIGIN is the axle center, with the axle
    running along local +Y. This frame is placed so it coincides with the
    articulation frame; rotation about +Y spins the arm + handle.
    """
    # Axle stub running along Y. It starts slightly INBOARD (-Y in this pre-mirror
    # frame) so that after mirroring + mounting it penetrates the housing axle
    # bore for a captured, connected fit; then it runs outboard to the hub/arm.
    axle = (
        cq.Workplane("XZ")
        .workplane(offset=-0.004)
        .circle(AXLE_R)
        .extrude(AXLE_LEN + 0.004)  # extrude along +Y (pre-mirror)
    )
    # Bearing flange that seats flat against the outboard housing wall face.
    # It is larger than the axle bore so it bears on the wall surface (this is
    # the real support path: the crank rides on the wall, not floating in the
    # clearance bore). Pre-mirror it spans Y in [0, 0.002]; the wall face lands
    # at crank-local Y=0 after mounting, so the flange contacts the wall.
    flange = (
        cq.Workplane("XZ")
        .workplane(offset=0.0)
        .circle(0.0105)
        .extrude(0.0022)
    )
    # Hub collar where the arm meets the axle.
    hub = (
        cq.Workplane("XZ")
        .workplane(offset=AXLE_LEN - 0.004)
        .circle(0.0085)
        .extrude(0.006)
    )
    # Radial arm: a flat bar from the axle outboard plane reaching radially in
    # +Z (the crank starts pointing up at q=0). Built in XY then placed.
    arm_y = AXLE_LEN + 0.001
    arm = (
        cq.Workplane("XZ")
        .workplane(offset=arm_y)
        .center(0.0, CRANK_ARM_LEN / 2.0)
        .rect(CRANK_ARM_W, CRANK_ARM_LEN)
        .extrude(CRANK_ARM_T)
        .edges("|Y")
        .fillet(0.0025)
    )
    # Grip handle: a knob/cylinder parallel to the axle (along +Y) at the arm tip.
    handle = (
        cq.Workplane("XZ")
        .workplane(offset=arm_y)
        .center(0.0, CRANK_ARM_LEN)
        .circle(HANDLE_R)
        .extrude(HANDLE_LEN)
        .faces(">Y")
        .fillet(0.004)
    )
    handle_cap = (
        cq.Workplane("XZ")
        .workplane(offset=arm_y + HANDLE_LEN)
        .center(0.0, CRANK_ARM_LEN)
        .circle(HANDLE_R + 0.0015)
        .extrude(0.003)
        .edges(">Y")
        .fillet(0.002)
    )
    crank = axle.union(flange).union(hub).union(arm).union(handle).union(handle_cap)
    # The "XZ" workplane extrudes toward -Y, so the assembly is authored on the
    # -Y side. Mirror across the XZ plane so the crank points OUTBOARD (+Y),
    # i.e. away from the housing once mounted on the right wall.
    return crank.mirror(mirrorPlane="XZ")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pencil_sharpener")

    charcoal = model.material("charcoal_plastic", rgba=(0.22, 0.23, 0.25, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.14, 0.14, 0.16, 1.0))
    steel = model.material("steel", rgba=(0.55, 0.56, 0.58, 1.0))
    badge = model.material("badge_silver", rgba=(0.78, 0.79, 0.80, 1.0))

    # --- Housing (root) ---------------------------------------------------
    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_build_housing(), "housing"),
        material=charcoal,
        name="housing_shell",
    )
    housing.visual(
        mesh_from_cadquery(_build_front_plate(), "front_plate"),
        material=badge,
        name="front_plate",
    )
    housing.inertial = Inertial.from_geometry(
        Box((BODY_W, BODY_D, BODY_H + SHOULDER_H)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, (BODY_H + SHOULDER_H) / 2.0)),
    )

    # --- Cutter (fixed inside the port) -----------------------------------
    cutter = model.part("cutter")
    cutter.visual(
        mesh_from_cadquery(_build_cutter(), "cutter"),
        material=steel,
        name="cutter_body",
    )
    cutter.inertial = Inertial.from_geometry(
        Cylinder(radius=PORT_THROAT_R, length=0.024),
        mass=0.02,
        origin=Origin(xyz=(BODY_W / 2.0 - PORT_DEPTH, 0.0, PORT_CENTER_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
    )
    model.articulation(
        "housing_to_cutter",
        ArticulationType.FIXED,
        parent=housing,
        child=cutter,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # --- Two top clamp posts (fixed) --------------------------------------
    deck_z = BODY_H + SHOULDER_H
    for idx, x_off in enumerate((-0.020, 0.020)):
        post = model.part(f"clamp_post_{idx + 1}")
        post.visual(
            mesh_from_cadquery(_build_clamp_post(), f"clamp_post_{idx + 1}"),
            material=dark_metal,
            name=f"clamp_post_shell_{idx + 1}",
        )
        post.inertial = Inertial.from_geometry(
            Cylinder(radius=0.0065, length=0.014),
            mass=0.01,
            origin=Origin(xyz=(x_off, 0.012, deck_z + 0.007)),
        )
        model.articulation(
            f"housing_to_clamp_post_{idx + 1}",
            ArticulationType.FIXED,
            parent=housing,
            child=post,
            origin=Origin(xyz=(x_off, 0.012, 0.0)),
        )

    # --- Hand crank (PRIMARY mechanism: CONTINUOUS rotary) ----------------
    crank = model.part("crank")
    crank.visual(
        mesh_from_cadquery(_build_crank(), "crank"),
        material=dark_metal,
        name="crank_body",
    )
    crank.inertial = Inertial.from_geometry(
        Box((0.014, AXLE_LEN + HANDLE_LEN, CRANK_ARM_LEN + 2 * HANDLE_R)),
        mass=0.04,
        origin=Origin(xyz=(0.0, (AXLE_LEN + HANDLE_LEN) / 2.0, CRANK_ARM_LEN / 2.0)),
    )

    # The crank is authored with its axle center at local origin and the axle
    # running along +Y. Place the articulation frame at the right housing wall,
    # at the axle height. Rotation about +Y spins the crank.
    model.articulation(
        "housing_to_crank",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=crank,
        origin=Origin(xyz=(0.0, RIGHT_WALL_Y, AXLE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=12.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    crank = object_model.get_part("crank")
    cutter = object_model.get_part("cutter")
    post1 = object_model.get_part("clamp_post_1")
    post2 = object_model.get_part("clamp_post_2")
    crank_joint = object_model.get_articulation("housing_to_crank")

    # --- Mechanism: crank joint type + axis -------------------------------
    ctx.check(
        "crank is a continuous rotary joint",
        str(crank_joint.joint_type).lower().endswith("continuous"),
        details=f"joint_type={crank_joint.joint_type}",
    )
    axis = tuple(crank_joint.axis)
    ctx.check(
        "crank axis is horizontal along Y",
        abs(axis[1]) > 0.99 and abs(axis[0]) < 0.01 and abs(axis[2]) < 0.01,
        details=f"axis={axis}",
    )

    # --- Crank lives on the +Y (right) side, outboard of the housing ------
    h_aabb = ctx.part_world_aabb(housing)
    c_aabb = ctx.part_world_aabb(crank)
    ctx.check(
        "crank sits on the +Y side of the housing",
        c_aabb is not None and h_aabb is not None and c_aabb[1][1] > h_aabb[1][1],
        details=f"crank_max_y={c_aabb[1][1] if c_aabb else None}, housing_max_y={h_aabb[1][1] if h_aabb else None}",
    )

    # --- Actuating the crank actually moves the handle (rotation works) ----
    rest_aabb = ctx.part_world_aabb(crank)
    with ctx.pose({crank_joint: math.pi / 2.0}):
        quarter_aabb = ctx.part_world_aabb(crank)
    with ctx.pose({crank_joint: math.pi}):
        half_aabb = ctx.part_world_aabb(crank)

    # At rest the arm/handle reach up (+Z). A quarter turn about +Y swings the
    # tip toward -X (front/back), and a half turn drops the tip below the axle.
    ctx.check(
        "crank handle starts above the axle at rest",
        rest_aabb is not None and rest_aabb[1][2] > AXLE_Z + CRANK_ARM_LEN * 0.5,
        details=f"rest_max_z={rest_aabb[1][2] if rest_aabb else None}",
    )
    ctx.check(
        "rotating the crank moves the handle (quarter turn swings in X)",
        rest_aabb is not None and quarter_aabb is not None
        and quarter_aabb[1][0] > rest_aabb[1][0] + 0.01,
        details=f"rest_max_x={rest_aabb[1][0] if rest_aabb else None}, quarter_max_x={quarter_aabb[1][0] if quarter_aabb else None}",
    )
    ctx.check(
        "half turn drops the crank tip below the rest height",
        rest_aabb is not None and half_aabb is not None
        and half_aabb[1][2] < rest_aabb[1][2] - 0.01,
        details=f"rest_max_z={rest_aabb[1][2] if rest_aabb else None}, half_max_z={half_aabb[1][2] if half_aabb else None}",
    )

    # --- Front pencil port + cutter present and located -------------------
    # The cutter is intentionally captured inside the housing throat (a nested
    # fit), and its axle stub of the crank is captured in the housing bore.
    ctx.allow_overlap(
        cutter,
        housing,
        elem_a="cutter_body",
        elem_b="housing_shell",
        reason="The milling cutter is intentionally seated/captured inside the front pencil port throat of the housing.",
    )
    ctx.allow_overlap(
        crank,
        housing,
        elem_a="crank_body",
        elem_b="housing_shell",
        reason="The crank axle stub is intentionally captured in the housing axle bore so the crank reads as mounted, not floating.",
    )

    cut_aabb = ctx.part_world_aabb(cutter)
    ctx.check(
        "cutter sits inside the front port region (near port center height)",
        cut_aabb is not None
        and (cut_aabb[0][2] < PORT_CENTER_Z < cut_aabb[1][2]),
        details=f"cutter_aabb={cut_aabb}",
    )
    # The cutter front end must be recessed behind the front housing face.
    front_face_x = h_aabb[1][0]
    ctx.check(
        "cutter front end is recessed behind the front housing face",
        cut_aabb is not None and cut_aabb[1][0] < front_face_x - 0.002,
        details=f"cutter_max_x={cut_aabb[1][0] if cut_aabb else None}, front_face_x={front_face_x}",
    )

    # --- Two clamp posts rise above the deck ------------------------------
    deck_z = BODY_H + SHOULDER_H
    for post, name in ((post1, "clamp_post_1"), (post2, "clamp_post_2")):
        p_aabb = ctx.part_world_aabb(post)
        ctx.check(
            f"{name} rises above the top deck",
            p_aabb is not None and p_aabb[1][2] > deck_z + 0.008,
            details=f"{name}_max_z={p_aabb[1][2] if p_aabb else None}",
        )
    # The two posts are distinct in X (a pair, not coincident).
    ctx.expect_origin_distance(post1, post2, axes="x", min_dist=0.03)

    # --- Connectivity: crank axle reaches the housing wall ----------------
    ctx.expect_overlap(
        crank,
        housing,
        axes="xz",
        min_overlap=0.004,
        name="crank axle aligns with the housing axle boss",
    )

    return ctx.report()


object_model = build_object_model()
