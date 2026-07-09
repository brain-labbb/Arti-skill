from __future__ import annotations

# Monkey wrench (adjustable spanner with square jaws), fork variant.
#
# Parent: crescent wrench (angled jaws, tilted head).
# Changed property: head replaced with a monkey-wrench style head whose two
# flat parallel gripping jaws run square (perpendicular) across the tool axis,
# with the lower jaw sliding open/closed along the head channel (+Y axis).
#
# Coordinate convention (meters):
#   +X : along the handle, from the ring butt (-X) toward the head (+X).
#   +Y : perpendicular to handle; jaw slide direction. Upper jaw at +Y.
#   +Z : thickness; wrench lies flat on the ground plane (z_min ~= 0).
#
# Mechanism:
#   - PRIMARY: lower jaw SLIDES (PRISMATIC) along Y to open/close the gap.
#     Positive travel opens the gap (jaw moves in -Y direction).
#   - Worm thumb-wheel ROTATES (CONTINUOUS) about X axis in the head's back
#     wall; its rim engages rack teeth on the movable jaw shank.

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
# Real-world dimensions (~12 inch monkey wrench), all in meters.
# ---------------------------------------------------------------------------
HANDLE_LEN = 0.235
HANDLE_W = 0.026
HANDLE_T = 0.0085

RING_R_OUTER = 0.024
RING_HEX_AF = 0.020
RING_HALF_T = 0.009

# Head frame (rectangular block, no tilt — jaws square to handle axis).
HEAD_X0_OFF = -0.006          # overlap behind handle end
HEAD_LEN = 0.050              # head length along X
HEAD_Y_BOT = -0.042           # bottom of head frame
HEAD_Y_TOP = 0.050            # top of head frame (fixed jaw tip)
HEAD_T = 0.014                # head plate thickness (Z)

# Fixed jaw (upper, integral with frame).
FIXED_JAW_FACE_Y = 0.034      # gripping face (inner face, faces -Y)

# Main cutout (single window: mouth above + channel below).
CUT_BACK = 0.020              # back wall thickness behind cutout
CUT_FRONT = 0.004             # front wall thickness ahead of cutout
CUT_Y_BOT = -0.036            # cutout bottom (channel floor)
CUT_Y_TOP = FIXED_JAW_FACE_Y  # cutout top = fixed jaw face

# Movable jaw (authored in own frame: gripping face at local y = 0).
JAW_SHOE_H = 0.006            # shoe height (block below face, in -Y)
JAW_SHOE_XW = 0.027           # shoe width (X, fills cutout for wall contact)
JAW_SHANK_H = 0.016           # shank height below shoe
JAW_SHANK_XW = 0.014          # shank width (X, narrower than shoe)
SHANK_HALF_T = 0.005          # shank half-thickness (Z)

# Joint parameters.
JAW_FACE_Y_REST = 0.004       # movable jaw face position at q = 0
JAW_TRAVEL = 0.016            # prismatic travel; max gap = 0.030 + 0.016

# Worm thumb-wheel (cylinder along X, in the head back wall).
WORM_R = 0.007
WORM_HALF_LEN = 0.006

# Z lift so the ring underside rests at z = 0.
Z_LIFT = RING_HALF_T + 0.0005

# Derived world-space landmarks.
RING_CX = 0.0
HANDLE_X0 = RING_R_OUTER * 0.55
HANDLE_X1 = HANDLE_X0 + HANDLE_LEN
HEAD_X0 = HANDLE_X1 + HEAD_X0_OFF
HEAD_X1 = HEAD_X0 + HEAD_LEN
CUT_X0 = HEAD_X0 + CUT_BACK
CUT_X1 = HEAD_X1 - CUT_FRONT

# Worm center (in the back wall of the head, past the handle end, at shank height).
WORM_CX = HANDLE_X1 + 0.010
WORM_CY = JAW_FACE_Y_REST - JAW_SHOE_H - JAW_SHANK_H * 0.5  # shank mid-height

# Rack teeth on the movable jaw shank (-X face, facing the worm).
RACK_N = 5
RACK_TOOTH_DX = 0.003
RACK_TOOTH_DY = 0.002
RACK_TOOTH_DZ = 0.003

# Materials.
STEEL = (0.72, 0.73, 0.76, 1.0)
STEEL_DARK = (0.55, 0.56, 0.60, 1.0)
STEEL_BLUE = (0.42, 0.47, 0.58, 1.0)
KNURL = (0.48, 0.49, 0.52, 1.0)
GRIP_RUBBER = (0.18, 0.18, 0.20, 1.0)


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _hex_profile(across_flats: float) -> list[tuple[float, float]]:
    """Flat-top hexagon profile (across-flats given), centered at origin."""
    r = across_flats / math.sqrt(3.0)
    pts: list[tuple[float, float]] = []
    for i in range(6):
        a = math.pi / 6.0 + i * math.pi / 3.0
        pts.append((r * math.cos(a), r * math.sin(a)))
    return pts


def _build_head_frame() -> cq.Workplane:
    """Monkey wrench head frame: rectangular block with cutout + worm pocket."""
    # Solid rectangular block.
    plate = (
        cq.Workplane("XY")
        .center((HEAD_X0 + HEAD_X1) * 0.5, (HEAD_Y_BOT + HEAD_Y_TOP) * 0.5)
        .rect(HEAD_LEN, HEAD_Y_TOP - HEAD_Y_BOT)
        .extrude(HEAD_T * 0.5, both=True)
    )
    try:
        plate = plate.edges("|Z").fillet(0.002)
    except Exception:
        pass

    # Main cutout: mouth + channel as one rectangular window.
    cutout = (
        cq.Workplane("XY")
        .center((CUT_X0 + CUT_X1) * 0.5, (CUT_Y_BOT + CUT_Y_TOP) * 0.5)
        .rect(CUT_X1 - CUT_X0, CUT_Y_TOP - CUT_Y_BOT)
        .extrude(HEAD_T, both=True)
    )
    plate = plate.cut(cutout)

    # Worm pocket: bore along X through the back wall, starting past the
    # handle end to avoid residual handle material. Breaks through both Z
    # faces for thumb access and breaches the cutout back wall for rack
    # engagement.
    pocket_start = HANDLE_X1
    pocket_end = CUT_X0 + 0.004
    pocket = (
        cq.Workplane("YZ")
        .workplane(offset=pocket_start)
        .center(WORM_CY, 0.0)
        .circle(WORM_R + 0.001)
        .extrude(pocket_end - pocket_start)
    )
    plate = plate.cut(pocket)
    return plate


def _build_body() -> cq.Workplane:
    """Fixed body: box-ring butt + flat handle + monkey wrench head frame."""
    # --- Box ring (closed end) with hex through-hole. ---
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

    # --- Flat handle shank (slight taper toward head). ---
    handle = (
        cq.Workplane("XY")
        .moveTo(HANDLE_X0, -HANDLE_W * 0.42)
        .lineTo(HANDLE_X0, HANDLE_W * 0.42)
        .lineTo(HANDLE_X1, HANDLE_W * 0.5)
        .lineTo(HANDLE_X1, -HANDLE_W * 0.5)
        .close()
        .extrude(HANDLE_T * 0.5, both=True)
    )
    try:
        handle = handle.edges("|Z").fillet(0.003)
    except Exception:
        pass

    # --- Monkey wrench head frame (no tilt, jaws square to handle). ---
    head = _build_head_frame()

    body = ring.union(handle).union(head)
    return body.translate((0.0, 0.0, Z_LIFT))


def _rack_tooth(x_center: float, y_center: float) -> cq.Workplane:
    """Single rack tooth on the shank -X face (shared geometry helper)."""
    return (
        cq.Workplane("XY")
        .center(x_center, y_center)
        .rect(RACK_TOOTH_DX, RACK_TOOTH_DY)
        .extrude(RACK_TOOTH_DZ, both=True)
    )


def _build_movable_jaw() -> cq.Workplane:
    """Movable jaw: shoe block + shank + rack teeth.

    Authored in its own frame: gripping face at local y = 0 (faces +Y).
    The shoe extends below (-Y) from the face, and the shank extends
    further below the shoe.
    """
    # Shoe: thin wide block (the visible gripping jaw).
    shoe = (
        cq.Workplane("XY")
        .center(0.0, -JAW_SHOE_H * 0.5)
        .rect(JAW_SHOE_XW, JAW_SHOE_H)
        .extrude(HEAD_T * 0.5 - 0.001, both=True)
    )
    try:
        shoe = shoe.edges("|Z").fillet(0.001)
    except Exception:
        pass

    # Shank: narrower block below the shoe, sliding in the channel.
    shank = (
        cq.Workplane("XY")
        .center(0.0, -JAW_SHOE_H - JAW_SHANK_H * 0.5)
        .rect(JAW_SHANK_XW, JAW_SHANK_H)
        .extrude(SHANK_HALF_T, both=True)
    )

    jaw = shoe.union(shank)

    # Rack teeth on the -X face of the shank (facing the worm).
    rack_y_start = -JAW_SHOE_H - JAW_SHANK_H * 0.8
    for i in range(RACK_N):
        ty = rack_y_start + i * (JAW_SHANK_H * 0.6 / max(RACK_N - 1, 1))
        tooth = _rack_tooth(-JAW_SHANK_XW * 0.5 - RACK_TOOTH_DX * 0.5, ty)
        try:
            jaw = jaw.union(tooth)
        except Exception:
            pass

    return jaw


def _build_worm() -> cq.Workplane:
    """Knurled worm thumb-wheel: short cylinder along X (handle axis)."""
    wheel = (
        cq.Workplane("YZ")
        .circle(WORM_R)
        .extrude(WORM_HALF_LEN, both=True)
    )
    # Knurl grooves around the circumference.
    grooves = cq.Workplane("YZ")
    for i in range(14):
        a = i * (2.0 * math.pi / 14.0)
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


def _build_grip_ridge(x_pos: float) -> cq.Workplane:
    """Small raised grip ridge on the handle surface (shared helper)."""
    return (
        cq.Workplane("XY")
        .center(x_pos, 0.0)
        .rect(0.006, HANDLE_W * 0.85)
        .extrude(HANDLE_T * 0.5 + 0.0008, both=True)
    )


# ---------------------------------------------------------------------------
# Object model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="monkey_wrench")

    steel = model.material("steel", rgba=STEEL)
    steel_dark = model.material("steel_dark", rgba=STEEL_DARK)
    steel_blue = model.material("steel_blue", rgba=STEEL_BLUE)
    knurl = model.material("knurl", rgba=KNURL)
    rubber = model.material("rubber", rgba=GRIP_RUBBER)

    # --- Root body: handle + ring + rectangular head frame with fixed jaw. ---
    body = model.part("wrench_body")
    body.visual(
        mesh_from_cadquery(_build_body(), "wrench_body"),
        material=steel,
        name="body_shell",
    )

    # Handle grip ridges (repeated decoration, inline as parent visuals).
    grip_x_start = HANDLE_X0 + 0.040
    grip_spacing = 0.018
    for i in range(7):
        x = grip_x_start + i * grip_spacing
        body.visual(
            mesh_from_cadquery(
                _build_grip_ridge(x).translate((0.0, 0.0, Z_LIFT)),
                f"grip_ridge_{i}",
            ),
            material=rubber,
            name=f"grip_ridge_{i}",
        )

    # --- Movable jaw (prismatic along Y, lower jaw slides to open/close). ---
    movable = model.part("movable_jaw")
    movable.visual(
        mesh_from_cadquery(_build_movable_jaw(), "movable_jaw"),
        material=steel_blue,
        name="jaw_shell",
    )

    # --- Worm thumb-wheel (continuous rotation about X axis). ---
    worm = model.part("worm_screw")
    worm.visual(
        mesh_from_cadquery(_build_worm(), "worm_screw"),
        material=knurl,
        name="worm_wheel",
    )

    # Prismatic jaw slide along Y. The joint frame sits at the movable jaw's
    # gripping face position at rest. Positive q drives the jaw in -Y
    # (opening the gap between the parallel jaw faces).
    jaw_origin_x = (CUT_X0 + CUT_X1) * 0.5
    model.articulation(
        "jaw_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=movable,
        origin=Origin(
            xyz=(jaw_origin_x, JAW_FACE_Y_REST, Z_LIFT),
        ),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0, velocity=0.05, lower=0.0, upper=JAW_TRAVEL,
        ),
    )

    # Worm wheel in the back wall, spins about X to drive the jaw.
    model.articulation(
        "worm_turn",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=worm,
        origin=Origin(
            xyz=(WORM_CX, WORM_CY, Z_LIFT),
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=10.0),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

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

    # --- Intentional mechanism overlaps. ---
    ctx.allow_overlap(
        movable,
        body,
        elem_a=jaw_shell,
        elem_b=body_shell,
        reason=(
            "The movable jaw shank rides captured inside the head frame's "
            "channel cutout; the shank/cutout interpenetration is the intended "
            "prismatic slide fit, proved by expect_within/expect_overlap below."
        ),
    )
    ctx.allow_overlap(
        movable,
        worm,
        elem_a=jaw_shell,
        elem_b=worm_wheel,
        reason=(
            "The worm thumb-wheel engages rack teeth on the jaw shank to "
            "drive the slide; this tooth engagement is an intentional small "
            "local overlap."
        ),
    )

    # The worm is captured inside the head's pocket bore (a cylindrical void
    # in the back wall). It rotates freely within the pocket with small
    # radial clearance; the pocket fully encloses it on all sides except
    # the Z-face thumb windows.
    ctx.allow_isolated_part(
        worm,
        reason=(
            "The worm thumb-wheel is captured inside the head's pocket bore; "
            "it rotates within the cylindrical void with small radial clearance "
            "and is retained by the pocket walls on all sides."
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
        "jaw slides along Y axis (perpendicular to handle)",
        abs(jaw_slide.axis[1]) > 0.99 and abs(jaw_slide.axis[0]) < 0.01,
        details=f"axis={jaw_slide.axis}",
    )
    ctx.check(
        "worm spins about X axis (handle axis)",
        abs(worm_turn.axis[0]) > 0.99 and abs(worm_turn.axis[2]) < 0.01,
        details=f"axis={worm_turn.axis}",
    )

    # --- Jaws are square (perpendicular) to the handle axis. ---
    ctx.check(
        "jaw slide axis is perpendicular to handle (no tilt)",
        abs(jaw_slide.axis[0]) < 0.01,
        details=f"axis_x_component={jaw_slide.axis[0]}",
    )

    # --- Body proportions: full length, rectangular head, lies flat. ---
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "wrench body spans handle length (>0.30 m)",
        body_aabb is not None and (body_aabb[1][0] - body_aabb[0][0]) > 0.30,
        details=f"aabb={body_aabb}",
    )
    head_height = None if body_aabb is None else body_aabb[1][1] - body_aabb[0][1]
    ctx.check(
        "monkey wrench head is taller than handle width (rectangular block)",
        head_height is not None and head_height > 2.5 * HANDLE_W,
        details=f"body_y_span={head_height} handle_w={HANDLE_W}",
    )
    ctx.check(
        "wrench lies flat on the ground (z_min ~ 0)",
        body_aabb is not None and abs(body_aabb[0][2]) < 0.002,
        details=f"z_min={None if body_aabb is None else body_aabb[0][2]}",
    )

    # --- Worm wheel captured in back wall pocket. ---
    ctx.expect_within(worm, body, axes="yz", margin=0.002,
                      name="worm within body pocket region (YZ)")
    ctx.expect_overlap(worm, body, axes="x", min_overlap=0.008,
                       name="worm overlaps body in X (inside back wall)")

    # --- Movable jaw overlaps body (jaw is inside the head cutout). ---
    ctx.expect_overlap(movable, body, axes="x", min_overlap=0.008,
                       name="movable jaw overlaps body in X (inside cutout)")

    # Captured slide: shank stays inside the head at rest.
    ctx.expect_overlap(movable, body, axes="y", min_overlap=0.004,
                       elem_a=jaw_shell, elem_b=body_shell,
                       name="jaw shank retained in head at rest (Y overlap)")

    # Across the full prismatic travel.
    rest_pos = ctx.part_world_position(movable)
    with ctx.pose({jaw_slide: JAW_TRAVEL}):
        open_pos = ctx.part_world_position(movable)
        ctx.expect_overlap(movable, body, axes="y", min_overlap=0.002,
                           elem_a=jaw_shell, elem_b=body_shell,
                           name="jaw shank retained in head when open")
    ctx.check(
        "opening slide moves jaw in -Y (widens gap)",
        rest_pos is not None and open_pos is not None
        and (rest_pos[1] - open_pos[1]) > 0.012,
        details=f"rest={rest_pos} open={open_pos}",
    )

    # --- Grip ridges exist as body visuals (repeated pattern). ---
    body_visuals = [v for v in body.visuals if v.name.startswith("grip_ridge_")]
    ctx.check(
        "handle has repeated grip ridge visuals",
        len(body_visuals) >= 5,
        details=f"found {len(body_visuals)} grip ridges",
    )

    # --- Mechanism: worm poses under rotation. ---
    with ctx.pose({worm_turn: 1.0}):
        turned = ctx.part_world_aabb(worm)
        ctx.check(
            "worm wheel poses under rotation",
            turned is not None,
            details=f"turned_aabb={turned}",
        )

    return ctx.report()


object_model = build_object_model()
