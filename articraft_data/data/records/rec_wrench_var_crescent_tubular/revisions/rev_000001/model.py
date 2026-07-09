from __future__ import annotations

# Adjustable wrench ("crescent" / adjustable spanner), variant 1 — tubular-shank fork.
#
# Reference: picture/Handtools/Wrench/001.png
#   - Round tubular steel shank (hollow cylindrical tube) running back from
#     the head, with a short end ferrule collar at the butt.
#   - A wide crescent-style head, tilted from the handle line, much wider than
#     the shank, with a FIXED jaw at the tip and a MOVABLE jaw that slides
#     along an angled jaw channel.
#   - A knurled worm-screw set in a pocket in the head, its axis parallel to
#     the slide direction, rim exposed through windows in both flat faces so a
#     thumb can spin it. The worm meshes with rack teeth on the movable jaw's
#     shank, which rides fully ENCLOSED inside a slot through the head.
#
# Coordinate convention (meters):
#   +X : along the shank, from the ferrule butt (-X) toward the head (+X).
#   +Y : wrench width, the side toward which the angled jaw mouth opens.
#   +Z : up; the wrench lies flat on the ground plane (z_min ~= 0).
#
# Head-local frame: the head is authored in a local frame whose +x axis is the
# SLIDE direction (perpendicular to the jaw gripping faces) and +y points out
# of the jaw mouth. The whole head is rotated by HEAD_TILT about Z at the
# shank/head junction, giving the classic angled crescent-wrench opening.
#
# Mechanism:
#   - PRIMARY user mechanism: the movable jaw SLIDES (PRISMATIC) along the
#     tilted slide axis to open/close the jaw gap. Positive travel opens it.
#     Over the FULL travel range the rack shank stays inside the head's slot;
#     only the jaw block is exposed in the mouth opening.
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
HANDLE_LEN = 0.235          # length of the tubular shank (along X)
SHANK_R = 0.013             # outer radius of tubular shank (~26 mm OD)
SHANK_WALL = 0.002          # tube wall thickness
SHANK_R_INNER = SHANK_R - SHANK_WALL  # inner bore radius (~22 mm ID)

FERRULE_R = 0.0145          # end ferrule collar outer radius
FERRULE_LEN = 0.010         # ferrule collar length

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

Z_LIFT = FERRULE_R           # lift so the ferrule underside rests at z = 0

STEEL = (0.74, 0.75, 0.77, 1.0)         # bright chromed alloy steel
STEEL_DARK = (0.58, 0.59, 0.62, 1.0)    # slightly darker machined surfaces
KNURL = (0.50, 0.51, 0.54, 1.0)         # darker knurled worm
FERRULE_CLR = (0.62, 0.63, 0.66, 1.0)   # slightly blue-grey ferrule collar

# Geometry landmarks along +X.
HANDLE_X0 = FERRULE_LEN + 0.003          # tube begins after ferrule collar
HANDLE_X1 = HANDLE_X0 + HANDLE_LEN       # shank/head junction (head anchor)

# Crescent head outline (head-local XY). Lens/teardrop plate: a wide round
# lower lobe, a rising hooked tip carrying the fixed jaw, and a straight top
# edge along the mouth side. Width spans y in [-0.035, 0.0355] = 0.0705 m,
# ~2.7x the 0.026 m shank diameter.
HEAD_PROFILE: list[tuple[float, float]] = [
    (-0.012, -0.013),   # neck bottom (overlaps the shank end)
    (0.022, -0.035),    # lobe bottom, rear
    (0.048, -0.035),    # lobe bottom, front
    (0.070, -0.016),    # tip underside rise
    (0.0775, 0.004),    # head tip (max local x)
    (0.073, 0.020),     # fixed jaw outer flank
    (0.062, 0.0355),    # fixed jaw tip (4 mm thick at the top)
    (0.010, 0.0355),    # straight top edge behind the mouth
    (-0.012, 0.013),    # neck top
]


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

    # Jaw mouth: angled opening between the fixed jaw face (x = 0.058) and the
    # mouth back wall (x = 0.012), open toward +y past the head edge.
    mouth = (
        cq.Workplane("XY")
        .center((MOUTH_X0 + FIXED_JAW_FACE_X) * 0.5, (MOUTH_FLOOR_Y + 0.065) * 0.5)
        .rect(FIXED_JAW_FACE_X - MOUTH_X0, 0.065 - MOUTH_FLOOR_Y)
        .extrude(HEAD_T, both=True)
    )
    plate = plate.cut(mouth)

    # Slide slot for the movable jaw's rack shank. Fully internal in plan
    # view: closed at the back end (x = SLOT_X0), >20 mm of lobe material
    # below it, and it only opens forward into the mouth where the jaw neck
    # passes. The rack rides inside this slot at every joint value.
    slot = (
        cq.Workplane("XY")
        .center((SLOT_X0 + SLOT_X1) * 0.5, (SLOT_Y0 + SLOT_Y1) * 0.5)
        .rect(SLOT_X1 - SLOT_X0, SLOT_Y1 - SLOT_Y0)
        .extrude(HEAD_T, both=True)
    )
    plate = plate.cut(slot)

    # Worm pocket: a bore along the slide axis just below the slot. Its
    # radius exceeds the plate half-thickness, so it breaks through both flat
    # faces as the classic thumb windows, and it breaches the slot bottom so
    # the worm rim can mesh with the rack teeth.
    pocket = (
        cq.Workplane("YZ")
        .workplane(offset=WORM_LCX - WORM_HALF_LEN - 0.002)
        .center(WORM_LCY, 0.0)
        .circle(WORM_R + 0.0015)
        .extrude(2.0 * (WORM_HALF_LEN + 0.002))
    )
    plate = plate.cut(pocket)
    return plate


def _build_tubular_shank() -> cq.Workplane:
    """Round tubular steel shank via lathe-revolve: hollow cylinder + ferrule.

    The shank is a straight hollow tube (constant outer/inner radii) running
    from HANDLE_X0 to HANDLE_X1 along +X, built by revolving an annular
    cross-section about the X axis. A short ferrule collar (slightly wider
    ring) wraps the butt end, with the bore open to show the hollow interior.
    """
    # --- Main tube: revolve an annular profile about the X axis. ---
    # Profile in XY plane, above X axis (positive Y = radial distance).
    # Annulus cross-section: inner radius to outer radius, full handle length.
    tube_profile = (
        cq.Workplane("XZ")
        .moveTo(HANDLE_X0, SHANK_R_INNER)
        .lineTo(HANDLE_X1, SHANK_R_INNER)
        .lineTo(HANDLE_X1, SHANK_R)
        .lineTo(HANDLE_X0, SHANK_R)
        .close()
    )
    tube = tube_profile.revolve(360, (0, 0), (HANDLE_X1 + 1, 0))

    # --- Ferrule collar: slightly wider ring at the butt end. ---
    ferrule_profile = (
        cq.Workplane("XZ")
        .moveTo(HANDLE_X0 - FERRULE_LEN, SHANK_R_INNER)
        .lineTo(HANDLE_X0, SHANK_R_INNER)
        .lineTo(HANDLE_X0, FERRULE_R)
        .lineTo(HANDLE_X0 - FERRULE_LEN, FERRULE_R)
        .close()
    )
    ferrule = ferrule_profile.revolve(360, (0, 0), (HANDLE_X0 + 1, 0))

    # Chamfer the butt-end outer edge for a finished look.
    try:
        ferrule = ferrule.edges("|X").edges(cq.selectors.RadiusNthSelector(-1)).chamfer(0.001)
    except Exception:
        pass

    shank = tube.union(ferrule)
    return shank


def _build_body() -> cq.Workplane:
    """Fixed body: tubular shank + tilted crescent head with fixed jaw."""
    shank = _build_tubular_shank()

    # --- Crescent head, tilted about Z at the shank/head junction. ---
    head = (
        _build_head_local()
        .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), HEAD_TILT_DEG)
        .translate((HANDLE_X1, 0.0, 0.0))
    )

    body = shank.union(head)
    return body.translate((0.0, 0.0, Z_LIFT))


def _build_movable_jaw() -> cq.Workplane:
    """Movable jaw: exposed jaw block + enclosed rack shank.

    Authored in its OWN local frame, slide axis along x, gripping face at
    local x = 0 facing +x toward the fixed jaw. The shank extends backward
    (-x) below the mouth floor and carries rack teeth on its -y face; the
    shank/teeth stay inside the head's slide slot over the full travel.
    """
    # Jaw block: the only part exposed in the mouth opening.
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

    # Rack shank, riding inside the head's slot (1 mm clearance all around).
    shank = (
        cq.Workplane("XY")
        .center((SHANK_X0 + SHANK_X1) * 0.5, (SHANK_Y0 + SHANK_Y1) * 0.5)
        .rect(SHANK_X1 - SHANK_X0, SHANK_Y1 - SHANK_Y0)
        .extrude(SHANK_HALF_T, both=True)
    )

    # Rack teeth on the underside (-y) of the shank, engaged by the worm.
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
        cq.Workplane("YZ")  # cylinder axis along +X
        .circle(WORM_R)
        .extrude(WORM_HALF_LEN, both=True)
    )
    # Knurl grooves: a ring of thin slots around the circumference.
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

    tilt = math.radians(HEAD_TILT_DEG)
    c, s = math.cos(tilt), math.sin(tilt)

    # --- Root body: tubular shank + tilted crescent head with fixed jaw. ---
    body = model.part("wrench_body")
    body.visual(
        mesh_from_cadquery(_build_body(), "wrench_body"),
        material=steel,
        name="body_shell",
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

    # Prismatic jaw slide. The joint frame is the head-local frame (rpy yaw =
    # head tilt); at q = 0 the movable jaw face sits JAW_NOMINAL_GAP from the
    # fixed jaw face. Positive q drives the jaw along local -x, widening the
    # gap while the rack shank stays inside the head's slot (travel sized to
    # the slot: shank spans head-local x in [0.006, 0.044] over q in
    # [0, 0.018], inside the [0.002, 0.048] slot).
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

    # Worm wheel spins in its head pocket about the slide axis.
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
    body_shell = body.get_visual("body_shell")
    worm_wheel = worm.get_visual("worm_wheel")

    # --- Intentional mechanism overlaps (captured slide + worm/rack mesh). ---
    ctx.allow_overlap(
        movable,
        body,
        elem_a=jaw_shell,
        elem_b=body_shell,
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
    shank_d = 2.0 * SHANK_R
    ctx.check(
        "crescent head is 2-3.5x wider than the shank diameter",
        head_width is not None and 2.0 * shank_d < head_width < 3.5 * shank_d,
        details=f"body_y_span={head_width} shank_d={shank_d}",
    )
    ctx.check(
        "wrench lies flat on the ground (z_min ~ 0)",
        body_aabb is not None and abs(body_aabb[0][2]) < 0.001,
        details=f"z_min={None if body_aabb is None else body_aabb[0][2]}",
    )

    # --- Tubular shank geometry: round cross-section, not flat. ---
    body_z_span = None if body_aabb is None else body_aabb[1][2] - body_aabb[0][2]
    ctx.check(
        "tubular shank makes body taller than flat-handle thickness",
        body_z_span is not None and body_z_span > 2.0 * SHANK_R - 0.001,
        details=f"z_span={body_z_span} expected~={2.0 * Z_LIFT + 2.0 * SHANK_R:.4f}",
    )

    # Ferrule collar extends wider than the tube.
    body_y_span = None if body_aabb is None else body_aabb[1][1] - body_aabb[0][1]
    ctx.check(
        "body y-span exceeds shank diameter (head dominates width)",
        body_y_span is not None and body_y_span > shank_d + 0.02,
        details=f"y_span={body_y_span} shank_d={shank_d}",
    )

    # Worm wheel seated in its pocket, rim poking through the flat-face
    # windows for thumb access (z-span exceeds the head plate thickness).
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

    # Movable jaw faces the fixed jaw across the angled mouth.
    ctx.expect_overlap(movable, body, axes="y", min_overlap=0.010,
                       name="movable jaw overlaps body width")

    # Captured slide: the rack shank stays inside the head at rest...
    ctx.expect_within(movable, body, axes="yz", margin=0.0005,
                      inner_elem=jaw_shell, outer_elem=body_shell,
                      name="jaw + rack inside head envelope at rest")
    ctx.expect_overlap(movable, body, axes="x", min_overlap=0.006,
                       elem_a=jaw_shell, elem_b=body_shell,
                       name="rack shank retained in head at rest")

    # ... and across the full prismatic travel.
    rest_pos = ctx.part_world_position(movable)
    with ctx.pose({jaw_slide: JAW_TRAVEL}):
        open_pos = ctx.part_world_position(movable)
        ctx.expect_within(movable, body, axes="yz", margin=0.0005,
                          inner_elem=jaw_shell, outer_elem=body_shell,
                          name="jaw + rack inside head envelope when open")
        ctx.expect_overlap(movable, body, axes="x", min_overlap=0.004,
                           elem_a=jaw_shell, elem_b=body_shell,
                           name="rack shank retained in head when open")
    ctx.check(
        "opening slide retracts jaw along the slide axis (widens gap)",
        rest_pos is not None and open_pos is not None
        and (rest_pos[0] - open_pos[0]) > 0.015,
        details=f"rest={rest_pos} open={open_pos}",
    )

    # --- Mechanism: turning the worm rotates the wheel (pose applies cleanly).
    with ctx.pose({worm_turn: 1.0}):
        turned = ctx.part_world_aabb(worm)
        ctx.check(
            "worm wheel poses under rotation",
            turned is not None,
            details=f"turned_aabb={turned}",
        )

    return ctx.report()


object_model = build_object_model()
