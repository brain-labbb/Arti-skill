from __future__ import annotations

# Adjustable wrench ("crescent" / adjustable spanner), QUICK-ADJUST variant.
#
# Reference: picture/Handtools/Wrench/001.png (parent silhouette retained).
# Variant change: the worm-screw jaw driver is replaced by a thumb-lever
# quick-adjust slide.  The movable jaw still rides the same head channel
# (prismatic) but is pushed open/closed by a finger-operated lever that
# pivots on the head about the Z axis (perpendicular to the flat face).
#
# Coordinate convention (meters):
#   +X : along the handle, from the ring butt (-X) toward the head (+X).
#   +Y : wrench width, the side toward which the angled jaw mouth opens.
#   +Z : up; the wrench lies flat on the ground plane (z_min ~= 0).
#
# Head-local frame: the head is authored in a local frame whose +x axis is
# the SLIDE direction (perpendicular to the jaw gripping faces) and +y points
# out of the jaw mouth.  The whole head is rotated by HEAD_TILT about Z at
# the handle/head junction.
#
# Mechanism:
#   - jaw_slide (PRISMATIC): movable jaw slides along the tilted slide axis
#     to open/close the jaw gap.  Positive travel opens it.
#   - lever_pivot (REVOLUTE): thumb lever pivots on the head about Z; its
#     engagement tab sweeps through the slot and pushes the jaw shank.
#     Positive rotation (CCW from above) drives the jaw open.

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
HANDLE_LEN = 0.235
HANDLE_W = 0.026
HANDLE_T = 0.0085

RING_R_OUTER = 0.024
RING_HEX_AF = 0.020
RING_HALF_T = 0.009

HEAD_T = 0.013
HEAD_TILT_DEG = 15.0

# Head-local landmarks (slide axis = local +x, mouth opens toward local +y).
FIXED_JAW_FACE_X = 0.058
MOUTH_X0 = 0.012
MOUTH_FLOOR_Y = 0.004
SLOT_X0 = 0.002
SLOT_X1 = 0.048
SLOT_Y0 = -0.009
SLOT_Y1 = 0.007

JAW_NOMINAL_GAP = 0.008
JAW_TRAVEL = 0.018
JAW_ORIGIN_LX = FIXED_JAW_FACE_X - JAW_NOMINAL_GAP

# Movable jaw (authored in its own frame: gripping face at local x = 0).
JAW_BLOCK_LEN = 0.016
JAW_BLOCK_TOP = 0.034
SHANK_X0, SHANK_X1 = -0.026, -0.006
SHANK_Y0, SHANK_Y1 = -0.007, 0.006
SHANK_HALF_T = 0.005

# Thumb-lever mechanism (head-local frame, pivot on the head plate).
LEVER_PIVOT_LX = 0.020       # pivot X in head-local
LEVER_PIVOT_LY = -0.010      # pivot Y in head-local (just below the slot)
LEVER_ARM_LEN = 0.025        # thumb arm length from pivot
LEVER_PAD_LEN = 0.010        # thumb pad extra length
LEVER_PAD_HW = 0.0065        # thumb pad half-width
LEVER_ARM_HW = 0.0045        # arm half-width
LEVER_T = 0.004              # lever plate thickness
PIVOT_BOSS_R = 0.003         # pivot boss pin radius
PIVOT_BOSS_H = 0.005         # boss height above head face
LEVER_TAB_TIP_Y = 0.014      # engagement tab tip Y from pivot (lever-local)
LEVER_TRAVEL = 0.50          # max lever rotation (rad, ~29 deg)
N_GRIP_RIDGES = 4            # grip ridges on thumb pad

Z_LIFT = 0.009

STEEL = (0.74, 0.75, 0.77, 1.0)
STEEL_DARK = (0.58, 0.59, 0.62, 1.0)
LEVER_OXIDE = (0.38, 0.40, 0.42, 1.0)    # dark oxide lever finish
GRIP_RUBBER = (0.22, 0.24, 0.26, 1.0)    # thumb-pad grip ridges

# Geometry landmarks along +X.
RING_CX = 0.0
HANDLE_X0 = RING_R_OUTER * 0.55
HANDLE_X1 = HANDLE_X0 + HANDLE_LEN

HEAD_PROFILE: list[tuple[float, float]] = [
    (-0.012, -0.013),
    (0.022, -0.035),
    (0.048, -0.035),
    (0.070, -0.016),
    (0.0775, 0.004),
    (0.073, 0.020),
    (0.062, 0.0355),
    (0.010, 0.0355),
    (-0.012, 0.013),
]


def _hex_profile(across_flats: float) -> list[tuple[float, float]]:
    r = across_flats / math.sqrt(3.0)
    pts: list[tuple[float, float]] = []
    for i in range(6):
        a = math.pi / 6.0 + i * math.pi / 3.0
        pts.append((r * math.cos(a), r * math.sin(a)))
    return pts


# ---- shared helper for repeated grip ridges on the thumb pad ----------
def _grip_ridge(width: float, depth: float, height: float) -> cq.Workplane:
    """Small raised ridge, bottom face at z = 0 (sits on the lever top)."""
    return (
        cq.Workplane("XY")
        .rect(width, depth)
        .extrude(height)
    )


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

    # Jaw mouth cut.
    mouth = (
        cq.Workplane("XY")
        .center((MOUTH_X0 + FIXED_JAW_FACE_X) * 0.5,
                (MOUTH_FLOOR_Y + 0.065) * 0.5)
        .rect(FIXED_JAW_FACE_X - MOUTH_X0, 0.065 - MOUTH_FLOOR_Y)
        .extrude(HEAD_T, both=True)
    )
    plate = plate.cut(mouth)

    # Slide slot for the movable jaw shank (fully enclosed).
    slot = (
        cq.Workplane("XY")
        .center((SLOT_X0 + SLOT_X1) * 0.5, (SLOT_Y0 + SLOT_Y1) * 0.5)
        .rect(SLOT_X1 - SLOT_X0, SLOT_Y1 - SLOT_Y0)
        .extrude(HEAD_T, both=True)
    )
    plate = plate.cut(slot)

    # Pivot boss pin on the +Z face for the thumb lever.
    boss = (
        cq.Workplane("XY")
        .workplane(offset=HEAD_T * 0.5)
        .center(LEVER_PIVOT_LX, LEVER_PIVOT_LY)
        .circle(PIVOT_BOSS_R)
        .extrude(PIVOT_BOSS_H)
    )
    plate = plate.union(boss)
    return plate


def _build_body() -> cq.Workplane:
    """Fixed body: box-ring butt + flat handle + tilted crescent head."""
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

    handle = (
        cq.Workplane("XY")
        .moveTo(HANDLE_X0, -HANDLE_W * 0.42)
        .lineTo(HANDLE_X0, HANDLE_W * 0.42)
        .lineTo(HANDLE_X1, HANDLE_W * 0.5)
        .lineTo(HANDLE_X1, -HANDLE_W * 0.5)
        .close()
        .extrude(HANDLE_T * 0.5, both=True)
        .edges("|Z")
        .fillet(0.003)
    )

    head = (
        _build_head_local()
        .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), HEAD_TILT_DEG)
        .translate((HANDLE_X1, 0.0, 0.0))
    )

    body = ring.union(handle).union(head)
    return body.translate((0.0, 0.0, Z_LIFT))


def _build_movable_jaw() -> cq.Workplane:
    """Movable jaw: exposed jaw block + enclosed slide shank (no rack teeth)."""
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

    jaw = block.union(shank)
    return jaw


def _build_thumb_lever() -> cq.Workplane:
    """Thumb lever: engagement tab, arm, and thumb pad.  Pivot at origin.

    Authored with bottom face at z = 0 (lever sits on the head face).
    Arm extends along -Y; engagement tab extends along +Y into the slot.
    """
    tab_hw = 0.003  # tab half-width (X)

    pts: list[tuple[float, float]] = [
        # Engagement tab (top of lever, extending +Y)
        (-tab_hw, LEVER_TAB_TIP_Y),
        (tab_hw + 0.001, LEVER_TAB_TIP_Y),
        # Right shoulder tapering to arm
        (LEVER_ARM_HW, 0.003),
        # Arm right side
        (LEVER_ARM_HW, -LEVER_ARM_LEN + 0.002),
        # Pad right side
        (LEVER_PAD_HW, -LEVER_ARM_LEN),
        (LEVER_PAD_HW, -LEVER_ARM_LEN - LEVER_PAD_LEN),
        # Pad bottom (rounded via arc approximated as line)
        (-LEVER_PAD_HW, -LEVER_ARM_LEN - LEVER_PAD_LEN),
        # Pad left side
        (-LEVER_PAD_HW, -LEVER_ARM_LEN),
        # Arm left side
        (-LEVER_ARM_HW, -LEVER_ARM_LEN + 0.002),
        # Left shoulder
        (-LEVER_ARM_HW, 0.003),
        # Back to tab
        (-tab_hw - 0.001, LEVER_TAB_TIP_Y - 0.001),
    ]

    body = (
        cq.Workplane("XY")
        .polyline(pts)
        .close()
        .extrude(LEVER_T)
    )
    try:
        body = body.edges("|Z").fillet(0.001)
    except Exception:
        pass

    # Pivot bore: through-hole for the boss pin.
    bore = (
        cq.Workplane("XY")
        .circle(PIVOT_BOSS_R + 0.0005)
        .extrude(LEVER_T * 3, both=True)
    )
    body = body.cut(bore)
    return body


# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="adjustable_wrench")

    steel = model.material("steel", rgba=STEEL)
    steel_dark = model.material("steel_dark", rgba=STEEL_DARK)
    lever_mat = model.material("lever_oxide", rgba=LEVER_OXIDE)
    grip_mat = model.material("grip_rubber", rgba=GRIP_RUBBER)

    tilt = math.radians(HEAD_TILT_DEG)
    c, s = math.cos(tilt), math.sin(tilt)

    # --- Root body: handle + ring + tilted crescent head + pivot boss. ---
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

    # --- Thumb lever (revolute on the head, engagement tab in slot). ---
    lever = model.part("thumb_lever")
    lever.visual(
        mesh_from_cadquery(_build_thumb_lever(), "thumb_lever"),
        material=lever_mat,
        name="lever_shell",
    )

    # Grip ridges on the thumb pad (repeated sub-visuals via for-loop).
    ridge_w = LEVER_PAD_HW * 1.50
    ridge_d = 0.0015
    ridge_h = 0.0008
    for i in range(N_GRIP_RIDGES):
        y_pos = -(LEVER_ARM_LEN + (i + 0.5) * (LEVER_PAD_LEN / N_GRIP_RIDGES))
        ridge = _grip_ridge(ridge_w, ridge_d, ridge_h).translate(
            (0.0, y_pos, LEVER_T)
        )
        lever.visual(
            mesh_from_cadquery(ridge, f"grip_ridge_{i}"),
            material=grip_mat,
            name=f"grip_ridge_{i}",
        )

    # ---- Articulations ----

    # Prismatic jaw slide (same as parent: head-local frame, -X opens gap).
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
        motion_limits=MotionLimits(
            effort=120.0, velocity=0.05, lower=0.0, upper=JAW_TRAVEL
        ),
    )

    # Thumb-lever pivot (revolute about Z on the head face, at the boss pin).
    model.articulation(
        "lever_pivot",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lever,
        origin=Origin(
            xyz=(
                HANDLE_X1 + LEVER_PIVOT_LX * c - LEVER_PIVOT_LY * s,
                LEVER_PIVOT_LX * s + LEVER_PIVOT_LY * c,
                Z_LIFT + HEAD_T * 0.5,
            ),
            rpy=(0.0, 0.0, tilt),
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=15.0, velocity=3.0, lower=0.0, upper=LEVER_TRAVEL
        ),
    )

    return model


# ---------------------------------------------------------------------------
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("wrench_body")
    movable = object_model.get_part("movable_jaw")
    lever = object_model.get_part("thumb_lever")
    jaw_slide = object_model.get_articulation("jaw_slide")
    lever_pivot = object_model.get_articulation("lever_pivot")

    body_shell = body.get_visual("body_shell")
    jaw_shell = movable.get_visual("jaw_shell")
    lever_shell = lever.get_visual("lever_shell")

    # ---- Intentional mechanism overlaps ----

    ctx.allow_overlap(
        movable,
        body,
        elem_a=jaw_shell,
        elem_b=body_shell,
        reason=(
            "The movable-jaw slide shank rides captured inside the head's "
            "enclosed slide slot; the shank/slot interpenetration is the "
            "intended prismatic slide fit."
        ),
    )
    ctx.allow_overlap(
        lever,
        body,
        elem_a=lever_shell,
        elem_b=body_shell,
        reason=(
            "The lever bore fits over the head's pivot boss pin; the boss "
            "enters the lever bore as a captured pin joint."
        ),
    )
    ctx.allow_overlap(
        lever,
        movable,
        elem_a=lever_shell,
        elem_b=jaw_shell,
        reason=(
            "The lever engagement tab sweeps into the slot to push the jaw "
            "shank; this tab/shank contact is the intended quick-adjust drive."
        ),
    )
    ctx.allow_coplanar_surfaces(
        body,
        lever,
        reason=(
            "The thumb lever sits flush on the head face and pivots on the "
            "boss pin; the mating flat surfaces are intentional."
        ),
    )

    # ---- Joint type / axis claims ----
    ctx.check(
        "jaw slide is prismatic",
        str(jaw_slide.joint_type).lower().endswith("prismatic"),
        details=f"type={jaw_slide.joint_type}",
    )
    ctx.check(
        "lever pivot is revolute",
        str(lever_pivot.joint_type).lower().endswith("revolute"),
        details=f"type={lever_pivot.joint_type}",
    )
    ctx.check(
        "lever pivots about Z (perpendicular to wrench face)",
        abs(lever_pivot.axis[2]) > 0.99
        and abs(lever_pivot.axis[0]) < 0.01
        and abs(lever_pivot.axis[1]) < 0.01,
        details=f"axis={lever_pivot.axis}",
    )
    ctx.check(
        "jaw slides along the joint-frame slide axis (-X)",
        jaw_slide.axis[0] < -0.99 and abs(jaw_slide.axis[2]) < 0.01,
        details=f"axis={jaw_slide.axis}",
    )

    # ---- Body proportions ----
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "wrench body spans handle length",
        body_aabb is not None and (body_aabb[1][0] - body_aabb[0][0]) > 0.30,
        details=f"aabb={body_aabb}",
    )
    head_width = None if body_aabb is None else body_aabb[1][1] - body_aabb[0][1]
    ctx.check(
        "crescent head is 2.5-3.5x wider than the handle",
        head_width is not None and 2.5 * HANDLE_W < head_width < 3.5 * HANDLE_W,
        details=f"body_y_span={head_width} handle_w={HANDLE_W}",
    )
    ctx.check(
        "wrench lies flat on the ground (z_min ~ 0)",
        body_aabb is not None and abs(body_aabb[0][2]) < 0.001,
        details=f"z_min={None if body_aabb is None else body_aabb[0][2]}",
    )

    # ---- Thumb lever seated on head face ----
    ctx.expect_overlap(lever, body, axes="xy", min_overlap=0.003,
                       name="lever overlaps head footprint in XY")
    lever_aabb = ctx.part_world_aabb(lever)
    head_z_top = Z_LIFT + HEAD_T * 0.5
    ctx.check(
        "lever sits on or just above the head top face",
        lever_aabb is not None
        and abs(lever_aabb[0][2] - head_z_top) < 0.002,
        details=f"lever_z_min={None if lever_aabb is None else lever_aabb[0][2]} "
                f"head_z_top={head_z_top}",
    )
    lever_z_span = (
        None if lever_aabb is None else lever_aabb[1][2] - lever_aabb[0][2]
    )
    ctx.check(
        "lever thickness matches design (3-6 mm)",
        lever_z_span is not None and 0.003 < lever_z_span < 0.007,
        details=f"lever_z_span={lever_z_span}",
    )

    # Lever extends below the head profile (thumb access).
    ctx.check(
        "lever arm extends past the head lobe for thumb access",
        lever_aabb is not None and body_aabb is not None
        and lever_aabb[0][1] < body_aabb[0][1] - 0.005,
        details=f"lever_y_min={None if lever_aabb is None else lever_aabb[0][1]} "
                f"body_y_min={None if body_aabb is None else body_aabb[0][1]}",
    )

    # ---- Grip ridges exist on lever ----
    for i in range(N_GRIP_RIDGES):
        rv = lever.get_visual(f"grip_ridge_{i}")
        ctx.check(
            f"grip ridge {i} exists on lever",
            rv is not None,
            details=f"visual grip_ridge_{i} not found",
        )

    # ---- Movable jaw ----
    ctx.expect_overlap(movable, body, axes="y", min_overlap=0.010,
                       name="movable jaw overlaps body width")
    ctx.expect_within(movable, body, axes="yz", margin=0.0005,
                      inner_elem=jaw_shell, outer_elem=body_shell,
                      name="jaw + shank inside head envelope at rest")
    ctx.expect_overlap(movable, body, axes="x", min_overlap=0.006,
                       elem_a=jaw_shell, elem_b=body_shell,
                       name="slide shank retained in head at rest")

    # Captured slide across full travel.
    rest_pos = ctx.part_world_position(movable)
    with ctx.pose({jaw_slide: JAW_TRAVEL}):
        open_pos = ctx.part_world_position(movable)
        ctx.expect_within(movable, body, axes="yz", margin=0.0005,
                          inner_elem=jaw_shell, outer_elem=body_shell,
                          name="jaw + shank inside head envelope when open")
        ctx.expect_overlap(movable, body, axes="x", min_overlap=0.004,
                           elem_a=jaw_shell, elem_b=body_shell,
                           name="slide shank retained in head when open")
    ctx.check(
        "opening slide retracts jaw along the slide axis (widens gap)",
        rest_pos is not None and open_pos is not None
        and (rest_pos[0] - open_pos[0]) > 0.015,
        details=f"rest={rest_pos} open={open_pos}",
    )

    # ---- Lever pivot mechanism ----
    lever_aabb_rest = ctx.part_world_aabb(lever)
    with ctx.pose({lever_pivot: LEVER_TRAVEL}):
        lever_aabb_push = ctx.part_world_aabb(lever)
        ctx.check(
            "lever poses under rotation",
            lever_aabb_push is not None,
            details=f"push_aabb={lever_aabb_push}",
        )
    ctx.check(
        "lever pivot sweeps the arm (AABB shifts with q)",
        lever_aabb_rest is not None and lever_aabb_push is not None
        and (
            abs(lever_aabb_rest[0][1] - lever_aabb_push[0][1]) > 0.001
            or abs(lever_aabb_rest[1][0] - lever_aabb_push[1][0]) > 0.001
        ),
        details=f"rest_aabb={lever_aabb_rest} push_aabb={lever_aabb_push}",
    )

    # Pivot origin sits on the head's upper face near the slot.
    pivot_world_z = lever_pivot.origin.xyz[2] if lever_pivot.origin else None
    ctx.check(
        "lever pivot origin is at the head top face",
        pivot_world_z is not None
        and abs(pivot_world_z - (Z_LIFT + HEAD_T * 0.5)) < 0.001,
        details=f"pivot_z={pivot_world_z}",
    )

    return ctx.report()


object_model = build_object_model()
