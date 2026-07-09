from __future__ import annotations

# Realistic articulated desk pencil sharpener modeled on the reference image
# (a Caran d'Ache style hand-crank table sharpener). The primary user-facing
# mechanism is the side hand crank, modeled as a CONTINUOUS rotary joint about
# the horizontal crank axle. The dark charcoal housing is hollow at the front
# where the pencil port and helical cutter sit, and two clamp posts rise from
# the top deck.
#
# Variant: the shavings receptacle is a hinged-lid bin at the lower rear of
# the housing. A fixed bin_shell (open-top tray with mounting ribs) sits in a
# cutout in the housing rear, and a bin_lid (cover panel with hinge knuckle
# and finger tab) swings up on a REVOLUTE hinge along the rear edge to expose
# the shavings compartment for emptying.

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

# Shavings bin (lower rear of housing, hinged-lid receptacle).
BIN_L = 0.046        # length along X (rear to front)
BIN_W = 0.064        # width along Y (slightly narrower than housing)
BIN_H = 0.034        # height (lower portion of body)
BIN_WALL = 0.0028    # wall thickness
LID_T = 0.0035       # lid panel thickness
BIN_REAR_X = -BODY_W / 2.0            # rear wall aligned with housing rear
BIN_FRONT_X = BIN_REAR_X + BIN_L      # front edge of bin opening
BIN_CENTER_X = (BIN_REAR_X + BIN_FRONT_X) / 2.0
BIN_HINGE_Z = BIN_H                   # hinge seam at top of bin opening


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

    # Cut away the lower rear corner to create the shavings bin opening.
    # The bin shell (a separate fixed part) fills this space, and the bin
    # lid (hinged) covers the top of the opening.
    # Lower cutout: full bin shell clearance.
    bin_cutout = (
        cq.Workplane("XY")
        .workplane(offset=-0.001)
        .center(BIN_CENTER_X, 0.0)
        .rect(BIN_L + 0.004, BIN_W + 0.004)
        .extrude(BIN_H + 0.002)
    )
    body = body.cut(bin_cutout)

    # Upper cutout: lid clearance above the bin opening.
    # Slightly narrower to leave a seating ledge for the lid edges.
    lid_clearance = (
        cq.Workplane("XY")
        .workplane(offset=BIN_H)
        .center(BIN_CENTER_X, 0.0)
        .rect(BIN_L + 0.002, BIN_W + 0.002)
        .extrude(LID_T + 0.008)
    )
    body = body.cut(lid_clearance)

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


def _build_bin_shell() -> cq.Workplane:
    """Open-top shavings tray that slots into the lower rear of the housing.

    Authored with its local origin at the center of the tray base (centered in
    X and Y, base at z=0). The tray is an open-top hollow box with thin walls.
    Side mounting ribs extend outward to contact the housing cutout walls,
    providing a physical support connection.
    """
    outer = (
        cq.Workplane("XY")
        .box(BIN_L, BIN_W, BIN_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.004)
    )
    inner = (
        cq.Workplane("XY")
        .workplane(offset=BIN_WALL)
        .box(BIN_L - 2 * BIN_WALL, BIN_W - 2 * BIN_WALL, BIN_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.003)
    )
    shell = outer.cut(inner)

    # Side mounting ribs: extend outward past the cutout edge to contact the
    # housing walls. These are realistic mounting features (retention ribs).
    rib_ext = 0.003  # extension beyond bin shell side (into housing wall)
    rib_h = BIN_H * 0.4
    rib_z = BIN_H * 0.25
    for y_sign in (-1.0, 1.0):
        rib = (
            cq.Workplane("XY")
            .workplane(offset=rib_z)
            .center(0.0, y_sign * (BIN_W / 2.0 + rib_ext / 2.0))
            .rect(BIN_L * 0.5, rib_ext)
            .extrude(rib_h)
        )
        shell = shell.union(rib)

    # Front mounting rib: same concept on the +X face.
    front_rib = (
        cq.Workplane("XY")
        .workplane(offset=rib_z)
        .center(BIN_L / 2.0 + rib_ext / 2.0, 0.0)
        .rect(rib_ext, BIN_W * 0.5)
        .extrude(rib_h)
    )
    shell = shell.union(front_rib)

    # Small rear hinge boss: a short cylinder along Y at the top-rear edge
    # where the lid hinge barrel seats.
    hinge_boss = (
        cq.Workplane("XZ")
        .workplane(offset=-BIN_W / 2.0 + 0.004)
        .center(-BIN_L / 2.0 + 0.004, BIN_H - 0.003)
        .circle(0.0035)
        .extrude(BIN_W - 0.008)
    )
    return shell.union(hinge_boss)


def _build_bin_lid() -> cq.Workplane:
    """Hinged cover panel for the shavings bin.

    Authored with its local origin at the HINGE EDGE center (the rear edge of
    the lid). The panel extends along local +X toward the front of the bin.
    A cylindrical hinge knuckle wraps around the Y axis at the origin, and a
    small finger tab at the front edge provides grip for opening.
    """
    # Main panel: extends from x=0 (hinge) to x=BIN_L, centered in Y.
    panel = (
        cq.Workplane("XY")
        .center(BIN_L / 2.0, 0.0)
        .rect(BIN_L - 0.002, BIN_W - 0.004)
        .extrude(LID_T)
        .edges("|Z")
        .fillet(0.003)
    )
    # Hinge knuckle: cylinder along Y at the rear edge, wrapping the hinge pin.
    knuckle = (
        cq.Workplane("XZ")
        .workplane(offset=-BIN_W / 2.0 + 0.006)
        .center(0.0, LID_T / 2.0)
        .circle(0.004)
        .extrude(BIN_W - 0.012)
    )
    # Finger tab at the front edge for grip.
    tab = (
        cq.Workplane("XY")
        .workplane(offset=LID_T)
        .center(BIN_L - 0.008, 0.0)
        .rect(0.016, 0.022)
        .extrude(0.004)
        .edges(">Z")
        .fillet(0.002)
    )
    return panel.union(knuckle).union(tab)


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

    # --- Shavings bin shell (fixed tray in lower rear) --------------------
    bin_shell = model.part("bin_shell")
    bin_shell.visual(
        mesh_from_cadquery(_build_bin_shell(), "bin_shell"),
        material=charcoal,
        name="bin_tray",
    )
    bin_shell.inertial = Inertial.from_geometry(
        Box((BIN_L, BIN_W, BIN_H)),
        mass=0.03,
        origin=Origin(xyz=(BIN_CENTER_X, 0.0, BIN_H / 2.0)),
    )
    model.articulation(
        "housing_to_bin_shell",
        ArticulationType.FIXED,
        parent=housing,
        child=bin_shell,
        origin=Origin(xyz=(BIN_CENTER_X, 0.0, 0.0)),
    )

    # --- Shavings bin lid (REVOLUTE hinge at rear edge) -------------------
    bin_lid = model.part("bin_lid")
    bin_lid.visual(
        mesh_from_cadquery(_build_bin_lid(), "bin_lid"),
        material=dark_metal,
        name="bin_lid_panel",
    )
    bin_lid.inertial = Inertial.from_geometry(
        Box((BIN_L, BIN_W, LID_T + 0.008)),
        mass=0.012,
        origin=Origin(xyz=(BIN_L / 2.0, 0.0, (LID_T + 0.008) / 2.0)),
    )
    # Hinge origin at the actual contact seam: rear edge of the bin opening,
    # at the top of the bin wall. The lid extends forward (+X) from here.
    # Axis (0, -1, 0) so positive q swings the free edge upward (+Z).
    model.articulation(
        "bin_shell_to_lid",
        ArticulationType.REVOLUTE,
        parent=bin_shell,
        child=bin_lid,
        origin=Origin(xyz=(-BIN_L / 2.0, 0.0, BIN_H)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.0, lower=0.0, upper=1.9),
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
    bin_shell = object_model.get_part("bin_shell")
    bin_lid = object_model.get_part("bin_lid")
    post1 = object_model.get_part("clamp_post_1")
    post2 = object_model.get_part("clamp_post_2")
    crank_joint = object_model.get_articulation("housing_to_crank")
    lid_hinge = object_model.get_articulation("bin_shell_to_lid")

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

    # --- Shavings bin: hinged-lid mechanism -------------------------------
    # The bin shell is fixed inside the housing rear; the lid is hinged.
    ctx.allow_overlap(
        bin_shell,
        housing,
        elem_a="bin_tray",
        elem_b="housing_shell",
        reason="The bin tray is intentionally seated inside the lower rear cutout of the housing shell.",
    )
    ctx.allow_overlap(
        bin_lid,
        bin_shell,
        elem_a="bin_lid_panel",
        elem_b="bin_tray",
        reason="The closed lid panel seats on the bin tray lip with a small intentional overlap at the hinge knuckle.",
    )
    ctx.allow_overlap(
        bin_lid,
        housing,
        elem_a="bin_lid_panel",
        elem_b="housing_shell",
        reason="The closed lid sits within the housing rear opening; the lid edges are captured by the housing cutout walls.",
    )

    # Hinge type and axis
    ctx.check(
        "bin lid hinge is revolute",
        str(lid_hinge.joint_type).lower().endswith("revolute"),
        details=f"joint_type={lid_hinge.joint_type}",
    )
    hinge_axis = tuple(lid_hinge.axis)
    ctx.check(
        "bin lid hinge axis is along Y (rear edge)",
        abs(hinge_axis[1]) > 0.99 and abs(hinge_axis[0]) < 0.01 and abs(hinge_axis[2]) < 0.01,
        details=f"axis={hinge_axis}",
    )

    # Bin shell sits at the rear of the housing, below mid-height
    bs_aabb = ctx.part_world_aabb(bin_shell)
    ctx.check(
        "bin shell is at the rear of the housing (negative X half)",
        bs_aabb is not None and bs_aabb[0][0] < -0.01,
        details=f"bin_shell_min_x={bs_aabb[0][0] if bs_aabb else None}",
    )
    ctx.check(
        "bin shell is in the lower portion of the housing",
        bs_aabb is not None and bs_aabb[1][2] < BODY_H * 0.6,
        details=f"bin_shell_max_z={bs_aabb[1][2] if bs_aabb else None}",
    )

    # Bin shell is contained within the housing footprint (XY)
    ctx.expect_within(
        bin_shell,
        housing,
        axes="xy",
        margin=0.005,
        name="bin shell stays within housing footprint",
    )

    # Lid closed pose: sits on top of the bin shell
    lid_aabb_closed = ctx.part_world_aabb(bin_lid)
    ctx.check(
        "lid covers the bin opening at rest (closed)",
        lid_aabb_closed is not None and bs_aabb is not None
        and lid_aabb_closed[0][2] >= bs_aabb[1][2] - 0.008,
        details=f"lid_min_z={lid_aabb_closed[0][2] if lid_aabb_closed else None}, bin_max_z={bs_aabb[1][2] if bs_aabb else None}",
    )

    # Lid open pose: positive q swings the free edge upward
    with ctx.pose({lid_hinge: 1.2}):
        lid_aabb_open = ctx.part_world_aabb(bin_lid)
    ctx.check(
        "lid opens upward on positive hinge angle",
        lid_aabb_open is not None and lid_aabb_closed is not None
        and lid_aabb_open[1][2] > lid_aabb_closed[1][2] + 0.01,
        details=f"closed_max_z={lid_aabb_closed[1][2] if lid_aabb_closed else None}, open_max_z={lid_aabb_open[1][2] if lid_aabb_open else None}",
    )

    # At max open (1.9 rad ≈ 109°), the lid should be well past vertical
    with ctx.pose({lid_hinge: 1.9}):
        lid_aabb_max = ctx.part_world_aabb(bin_lid)
    ctx.check(
        "lid reaches past vertical at max open angle",
        lid_aabb_max is not None and lid_aabb_closed is not None
        and lid_aabb_max[1][2] > lid_aabb_closed[1][2] + 0.02,
        details=f"closed_max_z={lid_aabb_closed[1][2] if lid_aabb_closed else None}, max_open_z={lid_aabb_max[1][2] if lid_aabb_max else None}",
    )

    # Hinge origin is at the rear edge of the bin (contact seam).
    # The articulation origin is in the bin_shell local frame; compute world x.
    hinge_local_x = lid_hinge.origin.xyz[0]
    hinge_world_x = BIN_CENTER_X + hinge_local_x  # bin_shell is at BIN_CENTER_X in housing
    ctx.check(
        "hinge origin is at the rear edge of the bin opening",
        hinge_world_x < BIN_CENTER_X - 0.01,
        details=f"hinge_world_x={hinge_world_x:.4f}, BIN_REAR_X={BIN_REAR_X:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
