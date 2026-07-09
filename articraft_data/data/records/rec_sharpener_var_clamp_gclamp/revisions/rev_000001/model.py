from __future__ import annotations

# Realistic articulated desk pencil sharpener with a table-edge G-clamp mount.
# The primary user-facing mechanism is the side hand crank (CONTINUOUS rotary
# about the horizontal crank axle). The secondary mechanism is the under-body
# G-clamp thumbscrew: a PRISMATIC vertical joint that drives the clamp pad
# upward against the underside of a table. The dark charcoal housing is hollow
# at the front pencil port where the helical cutter sits. A cast-metal C-frame
# hangs beneath the housing with the spine at the rear, and the thumbscrew
# travels through the lower arm.

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

# Upper shoulder block that the crank axle attaches to.
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

# ---------------------------------------------------------------------------
# G-clamp frame (under-body table-edge mount)
# ---------------------------------------------------------------------------
CF_SPINE_X = -0.043       # spine outer face X (near housing rear at -0.045)
CF_SPINE_T = 0.009        # spine thickness along X
CF_ARM_T = 0.009          # arm plate thickness along Z
CF_SPINE_H = 0.042        # clear height between arms (table capacity ~42 mm)
CF_ARM_DEPTH = 0.064      # arm total X extent from spine outer face
CF_FRONT_X = CF_SPINE_X + CF_ARM_DEPTH  # arm front edge X = 0.021
CF_INNER_X = CF_SPINE_X + CF_SPINE_T    # spine inner face X = -0.034
CF_WIDTH = 0.052          # frame width along Y
CF_LOWER_TOP = -(CF_ARM_T + CF_SPINE_H)          # lower arm top Z = -0.051
CF_LOWER_BOT = -(CF_ARM_T + CF_SPINE_H + CF_ARM_T)  # lower arm bottom Z = -0.060
CF_FRAME_H = CF_ARM_T + CF_SPINE_H + CF_ARM_T     # total frame height = 0.060

# Thumbscrew (prismatic clamp screw).
SCREW_X = 0.006           # screw center X on the lower arm
SCREW_R = 0.005           # screw shaft radius
SCREW_SHAFT_LEN = 0.042   # shaft length below the pad
PAD_R = 0.014             # clamp pad radius
PAD_T = 0.005             # clamp pad thickness
HANDLE_R_BAR = 0.003      # T-handle bar radius
HANDLE_LEN = 0.038        # T-handle crossbar length
SCREW_TRAVEL = 0.028      # max prismatic travel (m)


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

    # Upper shoulder block (slightly inset) that carries the crank.
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


def _build_clamp_frame() -> cq.Workplane:
    """C-shaped G-clamp frame that hangs below the housing.

    Built in the housing coordinate frame with the upper arm top surface at
    z=0 (flush with housing bottom). The spine is at the rear (-X) side and
    the arms extend forward (+X).
    """
    sx = CF_SPINE_X
    ix = CF_INNER_X
    fx = CF_FRONT_X
    t = CF_ARM_T
    h = CF_SPINE_H
    w = CF_WIDTH

    # Upper arm: plate from spine inner face to front, top at z=0.
    ua_cx = (ix + fx) / 2.0
    ua_dx = fx - ix
    upper = (
        cq.Workplane("XY")
        .workplane(offset=-t)
        .center(ua_cx, 0.0)
        .box(ua_dx, w, t, centered=(True, True, False))
    )

    # Spine: full-height vertical member at the rear.
    spine_cx = sx + CF_SPINE_T / 2.0
    spine_dz = t + h + t  # total height = CF_FRAME_H
    spine = (
        cq.Workplane("XY")
        .workplane(offset=CF_LOWER_BOT)
        .center(spine_cx, 0.0)
        .box(CF_SPINE_T, w, spine_dz, centered=(True, True, False))
    )

    # Lower arm: plate from spine inner face to front, top at z=CF_LOWER_TOP.
    lower = (
        cq.Workplane("XY")
        .workplane(offset=CF_LOWER_BOT)
        .center(ua_cx, 0.0)
        .box(ua_dx, w, t, centered=(True, True, False))
    )

    frame = upper.union(spine).union(lower)

    # Fillet outer vertical edges for a cast-metal look.
    try:
        frame = frame.edges("|Z").fillet(0.0025)
    except Exception:
        pass  # fillet may fail on complex topology; frame still valid

    # Inner corner fillets where arms meet spine (softer cast look).
    try:
        frame = frame.edges("|Y").fillet(0.002)
    except Exception:
        pass

    # Screw hole through the lower arm (slightly oversize for clearance).
    hole = (
        cq.Workplane("XY")
        .workplane(offset=CF_LOWER_BOT - 0.001)
        .center(SCREW_X, 0.0)
        .circle(SCREW_R + 0.0012)
        .extrude(t + 0.002)
    )
    frame = frame.cut(hole)

    # Reinforcement boss on the underside of the lower arm around the hole.
    boss = (
        cq.Workplane("XY")
        .workplane(offset=CF_LOWER_BOT - 0.003)
        .center(SCREW_X, 0.0)
        .circle(SCREW_R + 0.005)
        .extrude(0.003)
    )
    frame = frame.union(boss)

    # Two mounting bolt heads on the upper arm top (visual detail).
    for bx in (ix + 0.010, fx - 0.010):
        bolt = (
            cq.Workplane("XY")
            .workplane(offset=0.0)
            .center(bx, 0.0)
            .circle(0.003)
            .extrude(0.002)
        )
        bolt = bolt.edges(">Z").fillet(0.001)
        frame = frame.union(bolt)

    return frame


def _build_thumbscrew() -> cq.Workplane:
    """Thumbscrew with pad, threaded shaft, and T-handle.

    Authored with the local origin at the pad BOTTOM center (the face that
    contacts the clamp arm / table underside). Pad extends upward (+Z), shaft
    and handle extend downward (-Z).
    """
    # Pad: flat disk from z=0 to z=PAD_T.
    pad = (
        cq.Workplane("XY")
        .circle(PAD_R)
        .extrude(PAD_T)
    )
    # Chamfer top edge of pad for a finished look.
    try:
        pad = pad.edges(">Z").chamfer(0.001)
    except Exception:
        pass

    # Shaft: cylinder below the pad.
    shaft = (
        cq.Workplane("XY")
        .workplane(offset=-SCREW_SHAFT_LEN)
        .circle(SCREW_R)
        .extrude(SCREW_SHAFT_LEN)
    )

    # Simplified thread ridges on the lower portion of the shaft.
    screw_body = pad.union(shaft)
    n_threads = 6
    thread_zone = SCREW_SHAFT_LEN * 0.55  # threads on lower 55% of shaft
    for i in range(n_threads):
        tz = -SCREW_SHAFT_LEN + (i + 0.5) * (thread_zone / n_threads)
        ring = (
            cq.Workplane("XY")
            .workplane(offset=tz)
            .circle(SCREW_R + 0.0007)
            .extrude(0.0009)
        )
        screw_body = screw_body.union(ring)

    # T-handle crossbar at the shaft bottom (along Y).
    hz = -SCREW_SHAFT_LEN  # handle center Z in local frame
    # YZ workplane: normal is +X; workplane(offset=d) goes to x=d.
    handle = (
        cq.Workplane("YZ")
        .workplane(offset=-HANDLE_LEN / 2.0)
        .center(0.0, hz)
        .circle(HANDLE_R_BAR)
        .extrude(HANDLE_LEN)
    )
    # Ball ends on the handle.
    ball_l = (
        cq.Workplane("XY")
        .transformed(offset=(-HANDLE_LEN / 2.0, 0.0, hz))
        .sphere(HANDLE_R_BAR * 1.8)
    )
    ball_r = (
        cq.Workplane("XY")
        .transformed(offset=(HANDLE_LEN / 2.0, 0.0, hz))
        .sphere(HANDLE_R_BAR * 1.8)
    )

    return screw_body.union(handle).union(ball_l).union(ball_r)


def _build_crank() -> cq.Workplane:
    """Hand crank: axle stub + radial arm + perpendicular grip handle.

    Authored in a local frame whose ORIGIN is the axle center, with the axle
    running along local +Y. This frame is placed so it coincides with the
    articulation frame; rotation about +Y spins the arm + handle.
    """
    # Axle stub running along Y.
    axle = (
        cq.Workplane("XZ")
        .workplane(offset=-0.004)
        .circle(AXLE_R)
        .extrude(AXLE_LEN + 0.004)
    )
    # Bearing flange that seats flat against the outboard housing wall face.
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
    # Radial arm: a flat bar from the axle reaching radially in +Z.
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
    # Grip handle: a knob/cylinder parallel to the axle at the arm tip.
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
    # Mirror so the crank points OUTBOARD (+Y).
    return crank.mirror(mirrorPlane="XZ")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pencil_sharpener")

    charcoal = model.material("charcoal_plastic", rgba=(0.22, 0.23, 0.25, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.14, 0.14, 0.16, 1.0))
    steel = model.material("steel", rgba=(0.55, 0.56, 0.58, 1.0))
    badge = model.material("badge_silver", rgba=(0.78, 0.79, 0.80, 1.0))
    cast_iron = model.material("cast_iron", rgba=(0.28, 0.27, 0.26, 1.0))
    zinc = model.material("zinc_screw", rgba=(0.62, 0.63, 0.60, 1.0))

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

    # --- G-clamp frame (fixed beneath housing) ----------------------------
    clamp_frame = model.part("clamp_frame")
    clamp_frame.visual(
        mesh_from_cadquery(_build_clamp_frame(), "clamp_frame"),
        material=cast_iron,
        name="clamp_frame_body",
    )
    clamp_frame.inertial = Inertial.from_geometry(
        Box((CF_ARM_DEPTH, CF_WIDTH, CF_FRAME_H)),
        mass=0.22,
        origin=Origin(xyz=((CF_SPINE_X + CF_FRONT_X) / 2.0, 0.0, -CF_FRAME_H / 2.0)),
    )
    model.articulation(
        "housing_to_clamp_frame",
        ArticulationType.FIXED,
        parent=housing,
        child=clamp_frame,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # --- Thumbscrew (prismatic: drives pad upward to clamp table) ---------
    thumbscrew = model.part("thumbscrew")
    thumbscrew.visual(
        mesh_from_cadquery(_build_thumbscrew(), "thumbscrew"),
        material=zinc,
        name="thumbscrew_body",
    )
    thumbscrew.inertial = Inertial.from_geometry(
        Cylinder(radius=PAD_R, length=PAD_T + SCREW_SHAFT_LEN),
        mass=0.035,
        origin=Origin(xyz=(0.0, 0.0, -(SCREW_SHAFT_LEN - PAD_T) / 2.0)),
    )
    # Joint origin at the lower arm top surface where the pad rests at q=0.
    # Positive q (axis +Z) drives the pad upward toward the housing.
    model.articulation(
        "clamp_frame_to_thumbscrew",
        ArticulationType.PRISMATIC,
        parent=clamp_frame,
        child=thumbscrew,
        origin=Origin(xyz=(SCREW_X, 0.0, CF_LOWER_TOP)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=15.0,
            velocity=0.05,
            lower=0.0,
            upper=SCREW_TRAVEL,
        ),
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
    clamp_frame = object_model.get_part("clamp_frame")
    thumbscrew = object_model.get_part("thumbscrew")
    crank_joint = object_model.get_articulation("housing_to_crank")
    screw_joint = object_model.get_articulation("clamp_frame_to_thumbscrew")

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
    front_face_x = h_aabb[1][0]
    ctx.check(
        "cutter front end is recessed behind the front housing face",
        cut_aabb is not None and cut_aabb[1][0] < front_face_x - 0.002,
        details=f"cutter_max_x={cut_aabb[1][0] if cut_aabb else None}, front_face_x={front_face_x}",
    )

    # --- G-clamp frame hangs below the housing ----------------------------
    cf_aabb = ctx.part_world_aabb(clamp_frame)
    ctx.check(
        "clamp frame extends below the housing bottom",
        cf_aabb is not None and cf_aabb[0][2] < -0.030,
        details=f"clamp_frame_min_z={cf_aabb[0][2] if cf_aabb else None}",
    )
    # Spine at the rear (-X side) of the frame.
    ctx.check(
        "clamp frame spine is at the rear (-X) side",
        cf_aabb is not None and cf_aabb[0][0] < -0.030,
        details=f"clamp_frame_min_x={cf_aabb[0][0] if cf_aabb else None}",
    )
    # Frame C-shape: the gap between upper and lower arms is visible.
    # Upper arm top is at z~0, lower arm top at z=CF_LOWER_TOP=-0.051.
    # The frame should span from near z=0 to z~-0.060.
    ctx.check(
        "clamp frame has C-shape vertical span",
        cf_aabb is not None and (cf_aabb[1][2] - cf_aabb[0][2]) > 0.045,
        details=f"clamp_frame_z_span={(cf_aabb[1][2] - cf_aabb[0][2]) if cf_aabb else None}",
    )

    # --- Thumbscrew is prismatic with vertical axis -----------------------
    ctx.check(
        "thumbscrew is a prismatic joint",
        str(screw_joint.joint_type).lower().endswith("prismatic"),
        details=f"joint_type={screw_joint.joint_type}",
    )
    saxis = tuple(screw_joint.axis)
    ctx.check(
        "thumbscrew axis is vertical (+Z)",
        abs(saxis[2]) > 0.99 and abs(saxis[0]) < 0.01 and abs(saxis[1]) < 0.01,
        details=f"axis={saxis}",
    )

    # --- Thumbscrew pad travels upward when actuated ----------------------
    ts_aabb = ctx.part_world_aabb(thumbscrew)
    with ctx.pose({screw_joint: SCREW_TRAVEL}):
        ts_ext_aabb = ctx.part_world_aabb(thumbscrew)
    ctx.check(
        "thumbscrew at rest sits near the lower arm",
        ts_aabb is not None and ts_aabb[1][2] < CF_LOWER_TOP + PAD_T + 0.005,
        details=f"thumbscrew_max_z_rest={ts_aabb[1][2] if ts_aabb else None}",
    )
    ctx.check(
        "thumbscrew extends upward when actuated",
        ts_aabb is not None and ts_ext_aabb is not None
        and ts_ext_aabb[1][2] > ts_aabb[1][2] + SCREW_TRAVEL * 0.8,
        details=f"rest_max_z={ts_aabb[1][2] if ts_aabb else None}, ext_max_z={ts_ext_aabb[1][2] if ts_ext_aabb else None}",
    )

    # --- Thumbscrew lives below the housing (under-body mount) ------------
    ctx.check(
        "thumbscrew is below the housing bottom",
        ts_aabb is not None and ts_aabb[1][2] < 0.0,
        details=f"thumbscrew_max_z={ts_aabb[1][2] if ts_aabb else None}",
    )

    # --- Connectivity: crank axle reaches the housing wall ----------------
    ctx.expect_overlap(
        crank,
        housing,
        axes="xz",
        min_overlap=0.004,
        name="crank axle aligns with the housing axle boss",
    )

    # --- Thumbscrew shaft passes through clamp frame lower arm ------------
    # The shaft is intentionally captured through the hole in the lower arm.
    ctx.allow_overlap(
        thumbscrew,
        clamp_frame,
        elem_a="thumbscrew_body",
        elem_b="clamp_frame_body",
        reason="The thumbscrew shaft passes through the clearance hole in the clamp frame lower arm; the pad seats on the arm top surface.",
    )
    # Prove the thumbscrew is centered on the clamp frame screw position.
    ctx.expect_overlap(
        thumbscrew,
        clamp_frame,
        axes="xy",
        min_overlap=0.004,
        name="thumbscrew overlaps clamp frame in XY (passes through arm)",
    )

    return ctx.report()


object_model = build_object_model()
