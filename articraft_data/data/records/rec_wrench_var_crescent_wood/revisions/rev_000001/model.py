from __future__ import annotations

# Adjustable wrench ("crescent" / adjustable spanner), variant 1 — wooden grip fork.
#
# Reference: picture/Handtools/Wrench/001.png
#   - A crescent-style head, tilted from the handle line, much wider than
#     the handle, with a FIXED jaw at the tip and a MOVABLE jaw that slides
#     along an angled jaw channel.
#   - A knurled worm-screw set in a pocket in the head, its axis parallel to
#     the slide direction, rim exposed through windows in both flat faces so a
#     thumb can spin it. The worm meshes with rack teeth on the movable jaw's
#     shank, which rides fully ENCLOSED inside a slot through the head.
#   - TAPERED WOODEN HANDLE: a lathe-revolved teardrop wood grip (fattest
#     mid-section, narrower toward the butt), a brass ferrule collar where the
#     grip meets the head, and a small steel butt cap at the end.
#
# Coordinate convention (meters):
#   +X : along the handle, from the ring butt (-X) toward the head (+X).
#   +Y : wrench width, the side toward which the angled jaw mouth opens.
#   +Z : up; the wrench lies flat on the ground plane (z_min ~= 0).
#
# Head-local frame: the head is authored in a local frame whose +x axis is the
# SLIDE direction (perpendicular to the jaw gripping faces) and +y points out
# of the jaw mouth. The whole head is rotated by HEAD_TILT about Z at the
# handle/head junction, giving the classic angled crescent-wrench opening.
#
# Mechanism:
#   - PRIMARY user mechanism: the movable jaw SLIDES (PRISMATIC) along the
#     tilted slide axis to open/close the jaw gap. Positive travel opens it.
#   - The worm thumb-wheel ROTATES (CONTINUOUS) in its pocket about the slide
#     axis; it is the part the user actually spins to drive the jaw.

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
# Real-world dimensions (a ~12 inch adjustable wrench), all in meters.
# ---------------------------------------------------------------------------
HANDLE_LEN = 0.235          # length of the handle span
HANDLE_X0_BASE = 0.0        # ring center at origin

RING_R_OUTER = 0.024        # outer radius of the box-ring butt
RING_HEX_AF = 0.020         # hex through-hole across-flats
RING_HALF_T = 0.009         # ring half-thickness (Z)

HEAD_T = 0.013              # head plate thickness (Z)
HEAD_TILT_DEG = 15.0        # head/slide tilt from the handle axis (about Z)

# Head-local landmarks (slide axis = local +x, mouth opens toward local +y).
FIXED_JAW_FACE_X = 0.058    # fixed jaw gripping face plane (local x)
MOUTH_X0 = 0.012            # back wall of the jaw mouth cut (local x)
MOUTH_FLOOR_Y = 0.004       # floor of the jaw mouth (local y)
SLOT_X0 = 0.002             # slide slot, back end (closed inside the head)
SLOT_X1 = 0.048             # slide slot, front end (opens into the mouth)
SLOT_Y0 = -0.009            # slide slot bottom (local y)
SLOT_Y1 = 0.007             # slide slot top (local y)

JAW_NOMINAL_GAP = 0.008     # jaw gap at q = 0 (m)
JAW_TRAVEL = 0.018          # prismatic travel; gap_open = 0.026 m
JAW_ORIGIN_LX = FIXED_JAW_FACE_X - JAW_NOMINAL_GAP  # jaw joint origin, local x

# Movable jaw (authored in its OWN frame: gripping face at local x = 0).
JAW_BLOCK_LEN = 0.016       # jaw block length along the slide axis
JAW_BLOCK_TOP = 0.034       # jaw block top (local y)
SHANK_X0, SHANK_X1 = -0.026, -0.006   # rack shank span (jaw-local x)
SHANK_Y0, SHANK_Y1 = -0.007, 0.006    # rack shank span (jaw-local y)
SHANK_HALF_T = 0.005        # rack shank half-thickness (Z)

WORM_R = 0.008              # worm thumb-wheel radius
WORM_HALF_LEN = 0.009       # worm half-length (along the slide axis)
WORM_LCX = 0.025            # worm center, head-local x
WORM_LCY = -0.0135          # worm center, head-local y (below the slot)

# ---------------------------------------------------------------------------
# Wooden grip, ferrule, and butt cap dimensions.
# ---------------------------------------------------------------------------
# The grip spans from near the ring to the head junction.
GRIP_X0 = RING_R_OUTER * 0.55  # grip start (inside ring footprint)
GRIP_X1 = GRIP_X0 + HANDLE_LEN # grip end (head junction)

GRIP_MAX_R = 0.014          # max grip radius (mid-section, 28 mm diameter)
GRIP_BUTT_R = 0.006         # grip radius at butt end
GRIP_HEAD_R = 0.0085        # grip radius at head end (under ferrule)

# Ferrule: brass collar at the head junction.
FERRULE_LEN = 0.014
FERRULE_R = 0.0105          # outer radius (slightly wider than grip)
FERRULE_X0 = GRIP_X1 - FERRULE_LEN
FERRULE_X1 = GRIP_X1

# Butt cap: small steel disk at the butt end of the grip.
BUTT_CAP_LEN = 0.006
BUTT_CAP_R = 0.0075         # slightly wider than grip butt
BUTT_CAP_X0 = GRIP_X0
BUTT_CAP_X1 = GRIP_X0 + BUTT_CAP_LEN

# Tang: thin hidden steel rod connecting ring to head through the grip.
TANG_R = 0.003

# Z lift: the round grip is the lowest element when lying flat.
Z_LIFT = GRIP_MAX_R + 0.001  # grip centerline height, small ground clearance

STEEL = (0.74, 0.75, 0.77, 1.0)         # bright chromed alloy steel
STEEL_DARK = (0.58, 0.59, 0.62, 1.0)    # slightly darker machined surfaces
KNURL = (0.50, 0.51, 0.54, 1.0)         # darker knurled worm
WOOD = (0.52, 0.32, 0.15, 1.0)          # warm brown hardwood
BRASS = (0.72, 0.58, 0.28, 1.0)         # brass ferrule

# Geometry landmarks along +X.
RING_CX = 0.0                                  # ring center at origin
HANDLE_X0 = GRIP_X0                            # handle/grip start
HANDLE_X1 = GRIP_X1                            # handle/grip end (head anchor)

# Crescent head outline (head-local XY).
HEAD_PROFILE: list[tuple[float, float]] = [
    (-0.012, -0.013),   # neck bottom (overlaps the handle end)
    (0.022, -0.035),    # lobe bottom, rear
    (0.048, -0.035),    # lobe bottom, front
    (0.070, -0.016),    # tip underside rise
    (0.0775, 0.004),    # head tip (max local x)
    (0.073, 0.020),     # fixed jaw outer flank
    (0.062, 0.0355),    # fixed jaw tip (4 mm thick at the top)
    (0.010, 0.0355),    # straight top edge behind the mouth
    (-0.012, 0.013),    # neck top
]


def _hex_profile(across_flats: float) -> list[tuple[float, float]]:
    """Flat-top hexagon profile (across-flats given), centered at origin."""
    r = across_flats / math.sqrt(3.0)
    pts: list[tuple[float, float]] = []
    for i in range(6):
        a = math.pi / 6.0 + i * math.pi / 3.0
        pts.append((r * math.cos(a), r * math.sin(a)))
    return pts


def _build_head_local() -> cq.Workplane:
    """Crescent head plate in the head-local frame (slide along +x)."""
    plate = (
        cq.Workplane("XY")
        .polyline(HEAD_PROFILE)
        .close()
        .extrude(HEAD_T * 0.5, both=True)
    )
    try:
        plate = plate.edges("|Z").fillet(0.0015)
    except Exception:
        pass

    # Jaw mouth opening.
    mouth = (
        cq.Workplane("XY")
        .center((MOUTH_X0 + FIXED_JAW_FACE_X) * 0.5, (MOUTH_FLOOR_Y + 0.065) * 0.5)
        .rect(FIXED_JAW_FACE_X - MOUTH_X0, 0.065 - MOUTH_FLOOR_Y)
        .extrude(HEAD_T, both=True)
    )
    plate = plate.cut(mouth)

    # Slide slot for the movable jaw's rack shank.
    slot = (
        cq.Workplane("XY")
        .center((SLOT_X0 + SLOT_X1) * 0.5, (SLOT_Y0 + SLOT_Y1) * 0.5)
        .rect(SLOT_X1 - SLOT_X0, SLOT_Y1 - SLOT_Y0)
        .extrude(HEAD_T, both=True)
    )
    plate = plate.cut(slot)

    # Worm pocket bore.
    pocket = (
        cq.Workplane("YZ")
        .workplane(offset=WORM_LCX - WORM_HALF_LEN - 0.002)
        .center(WORM_LCY, 0.0)
        .circle(WORM_R + 0.0015)
        .extrude(2.0 * (WORM_HALF_LEN + 0.002))
    )
    plate = plate.cut(pocket)
    return plate


def _build_body_core() -> cq.Workplane:
    """Steel core: box-ring butt + hidden tang shank + tilted crescent head.

    The tang is a thin rod running through the wooden grip interior,
    connecting the ring to the head (like a real tool's tang).
    """
    # --- Box ring with hex through-hole. ---
    ring = (
        cq.Workplane("XY")
        .center(RING_CX, 0.0)
        .circle(RING_R_OUTER)
        .extrude(RING_HALF_T, both=True)
    )
    hole = (
        cq.Workplane("XY")
        .center(RING_CX, 0.0)
        .polyline(_hex_profile(RING_HEX_AF))
        .close()
        .extrude(RING_HALF_T * 1.4, both=True)
    )
    ring = ring.cut(hole)

    # --- Tang: thin cylindrical steel rod through the grip. ---
    tang = (
        cq.Workplane("YZ")
        .workplane(offset=0.0)
        .circle(TANG_R)
        .extrude(GRIP_X1 + 0.005)
    )

    # --- Crescent head, tilted about Z at the handle/head junction. ---
    head = (
        _build_head_local()
        .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), HEAD_TILT_DEG)
        .translate((GRIP_X1, 0.0, 0.0))
    )

    core = ring.union(tang).union(head)
    return core.translate((0.0, 0.0, Z_LIFT))


def _build_wood_grip() -> cq.Workplane:
    """Lathe-revolved teardrop wooden grip.

    A spline half-profile in the XY plane (X along the handle, Y = radius)
    is revolved 360 degrees around the X axis to create a round turned grip.
    Fattest at mid-section, narrower toward the butt, moderate at the head end.
    """
    # Half-profile: (x along handle, y = radius from axis).
    # The spline defines the outer surface of the turned grip.
    hx0, hx1 = GRIP_X0, GRIP_X1
    hlen = hx1 - hx0

    profile: list[tuple[float, float]] = [
        (hx0, 0.0),                                   # on axis at butt
        (hx0 + 0.004, GRIP_BUTT_R * 0.6),             # gentle rise from butt
        (hx0 + 0.012, GRIP_BUTT_R),                   # reach butt radius
        (hx0 + 0.030, GRIP_BUTT_R + 0.003),           # widening
        (hx0 + 0.055, GRIP_MAX_R * 0.80),             # approaching max
        (hx0 + 0.085, GRIP_MAX_R * 0.95),             # near max
        (hx0 + 0.110, GRIP_MAX_R),                    # max radius (slightly fwd of center)
        (hx0 + 0.140, GRIP_MAX_R * 0.97),             # staying wide
        (hx0 + 0.170, GRIP_MAX_R * 0.88),             # tapering
        (hx0 + 0.200, GRIP_HEAD_R + 0.002),           # narrowing toward head
        (hx1 - 0.005, GRIP_HEAD_R),                   # at ferrule seat
        (hx1, GRIP_HEAD_R),                            # end at head junction
        (hx1, 0.0),                                    # back to axis
    ]

    grip = (
        cq.Workplane("XY")
        .spline(profile)
        .close()
        .revolve(360, (hx0, 0.0), (hx1, 0.0))
    )
    return grip.translate((0.0, 0.0, Z_LIFT))


def _build_ferrule() -> cq.Workplane:
    """Brass ferrule collar: short cylinder at the head end of the grip."""
    ferrule = (
        cq.Workplane("YZ")
        .workplane(offset=FERRULE_X0)
        .circle(FERRULE_R)
        .extrude(FERRULE_LEN)
    )
    # Chamfer the outer edges for a finished look.
    try:
        ferrule = ferrule.edges("|X").chamfer(0.0008)
    except Exception:
        pass
    return ferrule.translate((0.0, 0.0, Z_LIFT))


def _build_butt_cap() -> cq.Workplane:
    """Small steel butt cap: short cylinder at the butt end of the grip."""
    cap = (
        cq.Workplane("YZ")
        .workplane(offset=BUTT_CAP_X0)
        .circle(BUTT_CAP_R)
        .extrude(BUTT_CAP_LEN)
    )
    try:
        cap = cap.edges("|X").chamfer(0.0006)
    except Exception:
        pass
    return cap.translate((0.0, 0.0, Z_LIFT))


def _build_movable_jaw() -> cq.Workplane:
    """Movable jaw: exposed jaw block + enclosed rack shank.

    Authored in its OWN local frame, slide axis along x, gripping face at
    local x = 0 facing +x toward the fixed jaw. The shank extends backward
    (-x) below the mouth floor and carries rack teeth on its -y face.
    """
    block = (
        cq.Workplane("XY")
        .moveTo(0.0, MOUTH_FLOOR_Y)
        .lineTo(0.0, JAW_BLOCK_TOP)
        .lineTo(-0.010, JAW_BLOCK_TOP)
        .lineTo(-JAW_BLOCK_LEN, 0.024)
        .lineTo(-JAW_BLOCK_LEN, MOUTH_FLOOR_Y)
        .close()
        .extrude(0.006, both=True)
    )
    try:
        block = block.edges("|Z").fillet(0.0012)
    except Exception:
        pass

    shank = (
        cq.Workplane("XY")
        .center((SHANK_X0 + SHANK_X1) * 0.5, (SHANK_Y0 + SHANK_Y1) * 0.5)
        .rect(SHANK_X1 - SHANK_X0, SHANK_Y1 - SHANK_Y0)
        .extrude(SHANK_HALF_T, both=True)
    )

    rack = None
    for i in range(5):
        x = -0.008 - i * 0.004
        tooth = (
            cq.Workplane("XY")
            .center(x, SHANK_Y0 - 0.00075)
            .rect(0.002, 0.002)
            .extrude(0.004, both=True)
        )
        rack = tooth if rack is None else rack.union(tooth)

    jaw = block.union(shank)
    if rack is not None:
        try:
            jaw = jaw.union(rack)
        except Exception:
            pass
    return jaw


def _build_worm() -> cq.Workplane:
    """Knurled worm thumb-wheel: a short cylinder along local X (slide axis)."""
    wheel = (
        cq.Workplane("YZ")
        .circle(WORM_R)
        .extrude(WORM_HALF_LEN, both=True)
    )
    grooves = cq.Workplane("YZ")
    for i in range(16):
        a = i * (2.0 * math.pi / 16.0)
        slot = (
            cq.Workplane("YZ")
            .center(WORM_R * math.cos(a), WORM_R * math.sin(a))
            .circle(0.0008)
            .extrude(WORM_HALF_LEN + 0.0005, both=True)
        )
        grooves = grooves.add(slot)
    try:
        wheel = wheel.cut(grooves.combine())
    except Exception:
        pass
    return wheel


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="adjustable_wrench")

    steel = model.material("steel", rgba=STEEL)
    steel_dark = model.material("steel_dark", rgba=STEEL_DARK)
    knurl = model.material("knurl", rgba=KNURL)
    wood = model.material("wood", rgba=WOOD)
    brass = model.material("brass", rgba=BRASS)

    tilt = math.radians(HEAD_TILT_DEG)
    c, s = math.cos(tilt), math.sin(tilt)

    # --- Root body: steel core + wooden grip + ferrule + butt cap. ---
    body = model.part("wrench_body")
    body.visual(
        mesh_from_cadquery(_build_body_core(), "body_core"),
        material=steel,
        name="body_core",
    )
    body.visual(
        mesh_from_cadquery(_build_wood_grip(), "wood_grip"),
        material=wood,
        name="wood_grip",
    )
    body.visual(
        mesh_from_cadquery(_build_ferrule(), "ferrule"),
        material=brass,
        name="ferrule_collar",
    )
    body.visual(
        mesh_from_cadquery(_build_butt_cap(), "butt_cap"),
        material=steel,
        name="butt_cap",
    )

    # --- Movable jaw (prismatic along the tilted slide axis). ---
    movable = model.part("movable_jaw")
    movable.visual(
        mesh_from_cadquery(_build_movable_jaw(), "movable_jaw"),
        material=steel_dark,
        name="jaw_shell",
    )

    # --- Worm thumb-wheel (continuous rotation about the slide axis). ---
    worm = model.part("worm_screw")
    worm.visual(
        mesh_from_cadquery(_build_worm(), "worm_screw"),
        material=knurl,
        name="worm_wheel",
    )

    # Prismatic jaw slide (unchanged from parent).
    model.articulation(
        "jaw_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=movable,
        origin=Origin(
            xyz=(HANDLE_X1 + JAW_ORIGIN_LX * c, JAW_ORIGIN_LX * s, Z_LIFT),
            rpy=(0.0, 0.0, tilt),
        ),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.05, lower=0.0, upper=JAW_TRAVEL),
    )

    # Worm wheel spins in its head pocket about the slide axis (unchanged).
    model.articulation(
        "worm_turn",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=worm,
        origin=Origin(
            xyz=(
                HANDLE_X1 + WORM_LCX * c - WORM_LCY * s,
                WORM_LCX * s + WORM_LCY * c,
                Z_LIFT,
            ),
            rpy=(0.0, 0.0, tilt),
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=10.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("wrench_body")
    movable = object_model.get_part("movable_jaw")
    worm = object_model.get_part("worm_screw")
    jaw_slide = object_model.get_articulation("jaw_slide")
    worm_turn = object_model.get_articulation("worm_turn")

    jaw_shell = movable.get_visual("jaw_shell")
    body_core = body.get_visual("body_core")
    worm_wheel = worm.get_visual("worm_wheel")
    wood_grip = body.get_visual("wood_grip")
    ferrule = body.get_visual("ferrule_collar")
    butt_cap = body.get_visual("butt_cap")

    # --- Intentional mechanism overlaps (captured slide + worm/rack mesh). ---
    ctx.allow_overlap(
        movable,
        body,
        elem_a=jaw_shell,
        elem_b=body_core,
        reason=(
            "The movable-jaw rack shank rides captured inside the head's "
            "slide slot; the shank/slot interpenetration is the intended "
            "prismatic slide fit, proved by expect_within/expect_overlap below."
        ),
    )
    ctx.allow_overlap(
        movable,
        worm,
        elem_a=jaw_shell,
        elem_b=worm_wheel,
        reason=(
            "The worm thumb-wheel meshes with the jaw's rack teeth to drive "
            "the slide; this tooth engagement is an intentional small overlap."
        ),
    )

    # --- Joint type / axis claims. ---
    ctx.check(
        "jaw slide is prismatic",
        str(jaw_slide.joint_type).lower().endswith("prismatic"),
        details=f"type={jaw_slide.joint_type}",
    )
    ctx.check(
        "worm turn is continuous",
        str(worm_turn.joint_type).lower().endswith("continuous"),
        details=f"type={worm_turn.joint_type}",
    )
    ctx.check(
        "worm spins about the slide axis (joint-frame X)",
        abs(worm_turn.axis[0]) > 0.99 and abs(worm_turn.axis[2]) < 0.01,
        details=f"axis={worm_turn.axis}",
    )
    ctx.check(
        "jaw slides along the joint-frame slide axis (-X)",
        jaw_slide.axis[0] < -0.99 and abs(jaw_slide.axis[2]) < 0.01,
        details=f"axis={jaw_slide.axis}",
    )

    # --- Body proportions: full length, wide crescent head, lies flat. ---
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "wrench body spans handle length",
        body_aabb is not None and (body_aabb[1][0] - body_aabb[0][0]) > 0.30,
        details=f"aabb={body_aabb}",
    )
    head_width = None if body_aabb is None else body_aabb[1][1] - body_aabb[0][1]
    grip_diameter = GRIP_MAX_R * 2.0
    ctx.check(
        "crescent head is 2.3-3.5x wider than the grip diameter",
        head_width is not None and 2.3 * grip_diameter < head_width < 3.5 * grip_diameter,
        details=f"body_y_span={head_width} grip_diameter={grip_diameter}",
    )
    ctx.check(
        "wrench lies flat on the ground (z_min ~ 0)",
        body_aabb is not None and abs(body_aabb[0][2]) < 0.003,
        details=f"z_min={None if body_aabb is None else body_aabb[0][2]}",
    )

    # --- Wooden grip: round cross-section, lathe-revolved teardrop. ---
    grip_aabb = ctx.part_element_world_aabb(body, elem=wood_grip)
    ctx.check(
        "wood grip exists with substantial extent",
        grip_aabb is not None,
        details=f"grip_aabb={grip_aabb}",
    )
    if grip_aabb is not None:
        grip_dx = grip_aabb[1][0] - grip_aabb[0][0]
        grip_dy = grip_aabb[1][1] - grip_aabb[0][1]
        grip_dz = grip_aabb[1][2] - grip_aabb[0][2]
        ctx.check(
            "wood grip spans most of the handle length",
            grip_dx > 0.18,
            details=f"grip_dx={grip_dx}",
        )
        ctx.check(
            "wood grip is approximately round (Y ~ Z extent)",
            abs(grip_dy - grip_dz) < 0.004,
            details=f"grip_dy={grip_dy} grip_dz={grip_dz}",
        )
        ctx.check(
            "wood grip max diameter is 22-32 mm",
            0.022 < max(grip_dy, grip_dz) < 0.032,
            details=f"max_span={max(grip_dy, grip_dz)}",
        )

    # --- Ferrule collar at the head end. ---
    ferrule_aabb = ctx.part_element_world_aabb(body, elem=ferrule)
    ctx.check(
        "ferrule collar exists",
        ferrule_aabb is not None,
        details=f"ferrule_aabb={ferrule_aabb}",
    )
    if ferrule_aabb is not None and grip_aabb is not None:
        ctx.check(
            "ferrule sits at the head end of the grip",
            ferrule_aabb[1][0] > grip_aabb[0][0] + grip_dx * 0.85,
            details=f"ferrule_x_max={ferrule_aabb[1][0]} grip_x_range=[{grip_aabb[0][0]}, {grip_aabb[1][0]}]",
        )

    # --- Butt cap at the butt end. ---
    cap_aabb = ctx.part_element_world_aabb(body, elem=butt_cap)
    ctx.check(
        "butt cap exists",
        cap_aabb is not None,
        details=f"cap_aabb={cap_aabb}",
    )
    if cap_aabb is not None and grip_aabb is not None:
        ctx.check(
            "butt cap sits at the butt end of the grip",
            cap_aabb[0][0] < grip_aabb[0][0] + grip_dx * 0.15,
            details=f"cap_x_min={cap_aabb[0][0]} grip_x_range=[{grip_aabb[0][0]}, {grip_aabb[1][0]}]",
        )

    # --- Worm wheel seated in its pocket. ---
    ctx.expect_overlap(worm, body, axes="xz", min_overlap=0.004,
                       name="worm seated in head pocket")
    ctx.expect_within(worm, body, axes="yz", margin=0.0005,
                      name="worm wheel within body envelope")
    worm_aabb = ctx.part_world_aabb(worm)
    worm_zspan = None if worm_aabb is None else worm_aabb[1][2] - worm_aabb[0][2]
    ctx.check(
        "worm rim protrudes 1-3 mm from the head-face windows",
        worm_zspan is not None and HEAD_T + 0.002 < worm_zspan < HEAD_T + 0.006,
        details=f"worm_z_span={worm_zspan} head_t={HEAD_T}",
    )

    # --- Movable jaw faces the fixed jaw across the angled mouth. ---
    ctx.expect_overlap(movable, body, axes="y", min_overlap=0.010,
                       name="movable jaw overlaps body width")

    # Captured slide: rack shank stays inside the head at rest.
    ctx.expect_within(movable, body, axes="yz", margin=0.0005,
                      inner_elem=jaw_shell, outer_elem=body_core,
                      name="jaw + rack inside head envelope at rest")
    ctx.expect_overlap(movable, body, axes="x", min_overlap=0.006,
                       elem_a=jaw_shell, elem_b=body_core,
                       name="rack shank retained in head at rest")

    # ... and across the full prismatic travel.
    rest_pos = ctx.part_world_position(movable)
    with ctx.pose({jaw_slide: JAW_TRAVEL}):
        open_pos = ctx.part_world_position(movable)
        ctx.expect_within(movable, body, axes="yz", margin=0.0005,
                          inner_elem=jaw_shell, outer_elem=body_core,
                          name="jaw + rack inside head envelope when open")
        ctx.expect_overlap(movable, body, axes="x", min_overlap=0.004,
                           elem_a=jaw_shell, elem_b=body_core,
                           name="rack shank retained in head when open")
    ctx.check(
        "opening slide retracts jaw along the slide axis (widens gap)",
        rest_pos is not None and open_pos is not None
        and (rest_pos[0] - open_pos[0]) > 0.015,
        details=f"rest={rest_pos} open={open_pos}",
    )

    # --- Mechanism: turning the worm rotates the wheel. ---
    with ctx.pose({worm_turn: 1.0}):
        turned = ctx.part_world_aabb(worm)
        ctx.check(
            "worm wheel poses under rotation",
            turned is not None,
            details=f"turned_aabb={turned}",
        )

    return ctx.report()


object_model = build_object_model()
