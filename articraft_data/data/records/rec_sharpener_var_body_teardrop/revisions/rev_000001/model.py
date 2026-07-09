from __future__ import annotations

# Realistic articulated desk pencil sharpener — streamlined teardrop variant.
# The body is a smooth egg/teardrop shell built as a CadQuery loft between
# elliptical cross-sections: wider and taller at the back (-X), tapering toward
# the front pencil port (+X). The primary user-facing mechanism is the side
# hand crank, modeled as a CONTINUOUS rotary joint about the horizontal axle.

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

# Overall size envelope (similar to original boxy variant).
BODY_W = 0.090  # front-to-back depth along X
BODY_D = 0.082  # max width along Y (at back)
BODY_MAX_H = 0.100  # max height at back of body shell

# Teardrop loft profiles: (x_position, y_half_width, z_half_height).
# Each ellipse is centered at (Y=0, Z=z_half_height) so its bottom touches Z=0.
# The body tapers from wide/tall at the back to narrow/short at the front.
TEARDROP_PROFILES = [
    (-0.045, 0.041, 0.050),   # back — widest/tallest
    (-0.015, 0.039, 0.048),
    (+0.015, 0.033, 0.043),
    (+0.035, 0.025, 0.036),
    (+0.045, 0.022, 0.030),   # front — narrowest
]

# Raised deck platform on top of the back portion for clamp posts.
DECK_CENTER_X = -0.005
DECK_BASE_Z = 0.090
DECK_H = 0.014
DECK_RX = 0.030  # X semi-axis
DECK_RY = 0.024  # Y semi-axis
DECK_TOP_Z = DECK_BASE_Z + DECK_H  # 0.104

# Pencil port (front face) geometry.
PORT_CENTER_Z = 0.040
PORT_OUTER_R = 0.018
PORT_THROAT_R = 0.0075
PORT_DEPTH = 0.022

# Crank geometry (unchanged from original).
AXLE_R = 0.0055
AXLE_LEN = 0.018
CRANK_ARM_LEN = 0.034
CRANK_ARM_W = 0.0095
CRANK_ARM_T = 0.0055
HANDLE_R = 0.006
HANDLE_LEN = 0.022

# Axle position adjusted for the teardrop body.
AXLE_X = 0.0
AXLE_Z = 0.072
# Body surface Y at the axle height (approximate from profile interpolation).
RIGHT_WALL_Y = 0.031


def _build_teardrop_shell() -> cq.Workplane:
    """Smooth teardrop/egg shell built as a loft between elliptical sections.

    The body is wider and taller at the back (-X) and tapers smoothly toward
    the front (+X), creating a streamlined compound-curved shell.
    """
    profiles = TEARDROP_PROFILES
    # CadQuery .center() is additive across chained workplanes, so we track
    # the accumulated Z offset and only apply the delta for each section.
    x0, ry0, rz0 = profiles[0]
    wp = cq.Workplane("YZ").workplane(offset=x0)
    accum_z = 0.0

    wp = wp.center(0.0, rz0 - accum_z).ellipse(ry0, rz0)
    accum_z = rz0

    for i in range(1, len(profiles)):
        xi, ryi, rzi = profiles[i]
        dx = xi - profiles[i - 1][0]
        dz = rzi - accum_z
        wp = wp.workplane(offset=dx).center(0.0, dz).ellipse(ryi, rzi)
        accum_z = rzi

    body = wp.loft()

    # Flatten the bottom at Z=0 for stable desk sitting.
    bottom_cut = (
        cq.Workplane("XY")
        .workplane(offset=-0.012)
        .box(0.200, 0.200, 0.012, centered=(True, True, False))
    )
    body = body.cut(bottom_cut)

    # Smooth deck platform on top for clamp post mounting — a rounded box
    # embedded slightly into the body top for a secure connection.
    deck = (
        cq.Workplane("XY")
        .workplane(offset=DECK_BASE_Z)
        .center(DECK_CENTER_X, 0.0)
        .box(DECK_RX * 2.0, DECK_RY * 2.0, DECK_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.006)
    )
    try:
        deck = deck.edges(">Z").fillet(0.003)
    except Exception:
        pass
    body = body.union(deck)

    # Front pencil port: conical insertion funnel cut into the +X face.
    front_x = BODY_W / 2.0 + 0.001
    recess = (
        cq.Workplane("YZ")
        .workplane(offset=front_x)
        .center(0.0, PORT_CENTER_Z)
        .circle(PORT_OUTER_R)
        .extrude(-0.006)
    )
    body = body.cut(recess)

    # Tapered throat leading toward the cutter.
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

    # Axle bore through the right body wall for the crank shaft.
    axle_bore = (
        cq.Workplane("XZ")
        .workplane(offset=-(RIGHT_WALL_Y + 0.003))
        .center(AXLE_X, AXLE_Z)
        .circle(AXLE_R + 0.001)
        .extrude(0.018)
    )
    body = body.cut(axle_bore)

    return body


def _build_cutter() -> cq.Workplane:
    """Helical milling cutter visible inside the pencil port (along X)."""
    cutter_front_x = BODY_W / 2.0 - 0.004
    cutter_back_x = BODY_W / 2.0 - PORT_DEPTH - 0.012
    core = (
        cq.Workplane("YZ")
        .workplane(offset=cutter_back_x)
        .center(0.0, PORT_CENTER_Z)
        .circle(PORT_THROAT_R - 0.0008)
        .extrude(cutter_front_x - cutter_back_x)
    )
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
    """Thin badge plate on the lower front face."""
    return (
        cq.Workplane("YZ")
        .workplane(offset=BODY_W / 2.0)
        .center(0.0, 0.016)
        .rect(0.030, 0.008)
        .extrude(0.0010)
        .edges("|X")
        .fillet(0.0012)
    )


def _build_clamp_post() -> cq.Workplane:
    """One cylindrical clamp/cover post that rises from the deck."""
    post = (
        cq.Workplane("XY")
        .workplane(offset=DECK_TOP_Z)
        .circle(0.0065)
        .extrude(0.014)
        .edges(">Z")
        .fillet(0.0035)
    )
    groove = (
        cq.Workplane("XY")
        .workplane(offset=DECK_TOP_Z + 0.0095)
        .circle(0.0066)
        .extrude(0.0012)
    )
    return post.cut(groove.intersect(post)).union(post)


def _build_crank() -> cq.Workplane:
    """Hand crank: axle stub + radial arm + perpendicular grip handle.

    Authored with origin at axle center, axle along local +Y after mirror.
    """
    # Axle stub (extrude goes -Y on XZ workplane).
    axle = (
        cq.Workplane("XZ")
        .workplane(offset=-0.004)
        .circle(AXLE_R)
        .extrude(AXLE_LEN + 0.004)
    )
    # Bearing flange that seats against the housing wall.
    flange = (
        cq.Workplane("XZ")
        .workplane(offset=0.0)
        .circle(0.0105)
        .extrude(0.0022)
    )
    # Hub collar.
    hub = (
        cq.Workplane("XZ")
        .workplane(offset=AXLE_LEN - 0.004)
        .circle(0.0085)
        .extrude(0.006)
    )
    # Radial arm.
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
    # Grip handle.
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
    return crank.mirror(mirrorPlane="XZ")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pencil_sharpener_teardrop")

    charcoal = model.material("charcoal_plastic", rgba=(0.22, 0.23, 0.25, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.14, 0.14, 0.16, 1.0))
    steel = model.material("steel", rgba=(0.55, 0.56, 0.58, 1.0))
    badge = model.material("badge_silver", rgba=(0.78, 0.79, 0.80, 1.0))

    # --- Housing (root) ---------------------------------------------------
    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_build_teardrop_shell(), "housing"),
        material=charcoal,
        name="housing_shell",
    )
    housing.visual(
        mesh_from_cadquery(_build_front_plate(), "front_plate"),
        material=badge,
        name="front_plate",
    )
    housing.inertial = Inertial.from_geometry(
        Box((BODY_W, BODY_D, DECK_TOP_Z)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, DECK_TOP_Z / 2.0)),
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
        origin=Origin(
            xyz=(BODY_W / 2.0 - PORT_DEPTH, 0.0, PORT_CENTER_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
    )
    model.articulation(
        "housing_to_cutter",
        ArticulationType.FIXED,
        parent=housing,
        child=cutter,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # --- Two top clamp posts (fixed, built via loop) ----------------------
    post_x_offsets = (-0.018, 0.018)
    for idx, x_off in enumerate(post_x_offsets):
        post = model.part(f"clamp_post_{idx}")
        post.visual(
            mesh_from_cadquery(_build_clamp_post(), f"clamp_post_{idx}"),
            material=dark_metal,
            name=f"clamp_post_shell_{idx}",
        )
        post.inertial = Inertial.from_geometry(
            Cylinder(radius=0.0065, length=0.014),
            mass=0.01,
            origin=Origin(xyz=(x_off, 0.0, DECK_TOP_Z + 0.007)),
        )
        model.articulation(
            f"housing_to_clamp_post_{idx}",
            ArticulationType.FIXED,
            parent=housing,
            child=post,
            origin=Origin(xyz=(x_off, 0.0, 0.0)),
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

    model.articulation(
        "housing_to_crank",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=crank,
        origin=Origin(xyz=(AXLE_X, RIGHT_WALL_Y, AXLE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=12.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    crank = object_model.get_part("crank")
    cutter = object_model.get_part("cutter")
    post0 = object_model.get_part("clamp_post_0")
    post1 = object_model.get_part("clamp_post_1")
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

    # --- Actuating the crank actually moves the handle --------------------
    rest_aabb = ctx.part_world_aabb(crank)
    with ctx.pose({crank_joint: math.pi / 2.0}):
        quarter_aabb = ctx.part_world_aabb(crank)
    with ctx.pose({crank_joint: math.pi}):
        half_aabb = ctx.part_world_aabb(crank)

    ctx.check(
        "crank handle starts above the axle at rest",
        rest_aabb is not None and rest_aabb[1][2] > AXLE_Z + CRANK_ARM_LEN * 0.5,
        details=f"rest_max_z={rest_aabb[1][2] if rest_aabb else None}",
    )
    ctx.check(
        "quarter turn swings the handle in X",
        rest_aabb is not None and quarter_aabb is not None
        and quarter_aabb[1][0] > rest_aabb[1][0] + 0.01,
        details=f"rest_max_x={rest_aabb[1][0] if rest_aabb else None}, quarter_max_x={quarter_aabb[1][0] if quarter_aabb else None}",
    )
    ctx.check(
        "half turn drops the crank tip below rest height",
        rest_aabb is not None and half_aabb is not None
        and half_aabb[1][2] < rest_aabb[1][2] - 0.01,
        details=f"rest_max_z={rest_aabb[1][2] if rest_aabb else None}, half_max_z={half_aabb[1][2] if half_aabb else None}",
    )

    # --- Front pencil port + cutter present and located -------------------
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
        "cutter sits inside the front port region",
        cut_aabb is not None and (cut_aabb[0][2] < PORT_CENTER_Z < cut_aabb[1][2]),
        details=f"cutter_aabb={cut_aabb}",
    )
    front_face_x = h_aabb[1][0]
    ctx.check(
        "cutter front end is recessed behind the front housing face",
        cut_aabb is not None and cut_aabb[1][0] < front_face_x - 0.002,
        details=f"cutter_max_x={cut_aabb[1][0] if cut_aabb else None}, front_face_x={front_face_x}",
    )

    # --- Two clamp posts rise above the deck ------------------------------
    for post, name in ((post0, "clamp_post_0"), (post1, "clamp_post_1")):
        p_aabb = ctx.part_world_aabb(post)
        ctx.check(
            f"{name} rises above the top deck",
            p_aabb is not None and p_aabb[1][2] > DECK_TOP_Z + 0.008,
            details=f"{name}_max_z={p_aabb[1][2] if p_aabb else None}",
        )
    ctx.expect_origin_distance(post0, post1, axes="x", min_dist=0.025)

    # --- Connectivity: crank axle reaches the housing wall ----------------
    ctx.expect_overlap(
        crank,
        housing,
        axes="xz",
        min_overlap=0.004,
        name="crank axle aligns with the housing axle boss",
    )

    # --- Teardrop shape: housing is taller than wide (streamlined) --------
    h_dy = h_aabb[1][1] - h_aabb[0][1]
    h_dz = h_aabb[1][2] - h_aabb[0][2]
    h_dx = h_aabb[1][0] - h_aabb[0][0]
    ctx.check(
        "housing has teardrop proportions (taller than wide, tapered)",
        h_dz > h_dy * 1.10 and h_dx > 0.080,
        details=f"dx={h_dx:.4f}, dy={h_dy:.4f}, dz={h_dz:.4f}",
    )

    # Front plate narrower than housing max width (proves taper).
    ctx.expect_within(
        housing,
        housing,
        axes="y",
        inner_elem="front_plate",
        outer_elem="housing_shell",
        margin=0.002,
        name="front badge fits within the tapered housing width",
    )

    return ctx.report()


object_model = build_object_model()
