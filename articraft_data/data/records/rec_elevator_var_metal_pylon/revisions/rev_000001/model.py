from __future__ import annotations

# Passenger elevator landing entrance – narrow metal-framed pylon variant.
#
# Coordinate convention (Z-up, meters):
#   +X = frame width       (doors slide along X)
#   +Y = frame depth, going back into the shaft
#   +Z = height            (ground/floor at z = 0)
#
# Y layout (front -> back):
#   buttons/digits proud    y < 0
#   door leaves             y in [-0.040, 0.000]
#   frame front plane       y = 0
#   frame body              y in [0, FRAME_D]
#   shaft recess            y in [0, SHAFT_BACK_Y]
#
# The narrow steel-framed pylon surround is the fixed root: two vertical
# mullions, a head beam, thin metal infill panels, and the shaft recess
# behind.  Indicator, call panel, and sill are inline visuals on the root.
# Two brushed-stainless center-opening leaves slide apart along X on
# prismatic joints.

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
# Dimensions
# ---------------------------------------------------------------------------

# Doorway opening.
OPEN_W = 1.15
OPEN_H = 2.15
OPEN_HALF = OPEN_W / 2.0  # 0.575

# Steel pylon frame (narrow surround).
FRAME_H = 2.70
FRAME_D = 0.12
MULLION_W = 0.10
HEAD_BEAM_H = 0.10
INFILL_STRIP_W = 0.06
INFILL_T = 0.012
UPPER_INFILL_T = 0.035

# Outer frame width (narrow: just mullions + thin side infill strips).
FRAME_W = OPEN_W + 2.0 * MULLION_W + 2.0 * INFILL_STRIP_W  # 1.47

# Door leaves (brushed stainless, unchanged from parent).
LEAF_W = OPEN_HALF  # 0.575
LEAF_BOTTOM = 0.018  # rests at the sill-track top
LEAF_TOP = OPEN_H
LEAF_H = LEAF_TOP - LEAF_BOTTOM
LEAF_T = 0.040
DOOR_FRONT_Y = -0.040
DOOR_BACK_Y = 0.0
DOOR_CY = (DOOR_FRONT_Y + DOOR_BACK_Y) / 2.0  # -0.020

# Prismatic travel.
DOOR_TRAVEL = 0.52

# Indicator (surface-mounted on upper infill above head beam).
IND_W = 0.34
IND_H = 0.13
IND_D = 0.05
IND_CZ = OPEN_H + HEAD_BEAM_H + 0.08  # 2.33

# Hall call-button plate (surface-mounted on right mullion).
PLATE_W = 0.085
PLATE_H = 0.16
PLATE_D = 0.012
PLATE_CX = OPEN_HALF + MULLION_W / 2.0  # 0.625, right mullion centre
PLATE_CZ = 1.10

# Threshold sill at the floor (grooved door track).
SILL_W = OPEN_W + 0.10
SILL_FRONT_Y = -0.14
SILL_BACK_Y = 0.03
SILL_TOP = LEAF_BOTTOM

# Shaft recess behind the doors.
SHAFT_W = OPEN_W + 2.0 * MULLION_W  # 1.35 (wider than opening for coverage)
SHAFT_H = OPEN_H
SHAFT_BACK_Y = 0.35

# ---------------------------------------------------------------------------
# Colours / materials
# ---------------------------------------------------------------------------
STEEL_FRAME = (0.42, 0.44, 0.48, 1.0)   # dark brushed steel for frame
STEEL = (0.72, 0.73, 0.75, 1.0)         # brushed stainless for doors/sill
INFILL = (0.32, 0.34, 0.38, 1.0)        # dark grey sheet metal
INDICATOR_BLACK = (0.04, 0.04, 0.05, 1.0)
RED_LED = (0.88, 0.10, 0.10, 1.0)
SHAFT_DARK = (0.05, 0.05, 0.06, 1.0)
BUTTON_STEEL = (0.80, 0.81, 0.83, 1.0)


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------
def _box_xyz(xc: float, xs: float, yc: float, ys: float,
             zc: float, zs: float) -> cq.Workplane:
    """Axis-aligned box from explicit centre + size on each axis."""
    return cq.Workplane("XY").box(xs, ys, zs).translate((xc, yc, zc))


def _frame_member(xc: float, w: float, h: float,
                  z_bottom: float = 0.0) -> cq.Workplane:
    """Rectangular-section steel frame member (mullion or beam).

    Front face at y = 0, back face at y = FRAME_D.
    """
    zc = z_bottom + h / 2.0
    body = _box_xyz(xc, w, FRAME_D / 2.0, FRAME_D, zc, h)
    # Subtle inside-fillet on the long edges for a rolled-steel look.
    return body.edges("|Z").fillet(min(0.003, w / 6.0))


def _infill_panel(xc: float, w: float, h: float, t: float,
                  z_bottom: float = 0.0) -> cq.Workplane:
    """Thin sheet-metal infill panel.  Front face at y = 0, back at y = t."""
    zc = z_bottom + h / 2.0
    return _box_xyz(xc, w, t / 2.0, t, zc, h)


def _leaf_shape(sign: float) -> cq.Workplane:
    """Brushed-stainless door leaf with a recessed panel.

    ``sign`` = −1 (left) or +1 (right).  Inner edge at x = 0 (centre seam).
    Leaf local-Z bottom is at z = 0 (joint origin sits at the sill track).
    """
    cx = sign * LEAF_W / 2.0
    cz = LEAF_H / 2.0
    body = _box_xyz(cx, LEAF_W, DOOR_CY, LEAF_T, cz, LEAF_H)
    # Recessed panel on the front face.
    pocket_w = LEAF_W - 0.12
    pocket_h = LEAF_H - 0.20
    pocket = _box_xyz(cx, pocket_w, DOOR_FRONT_Y + 0.004, 0.010, cz, pocket_h)
    leaf = body.cut(pocket)
    leaf = leaf.edges("|Z").fillet(0.004)
    return leaf


def _shaft_recess_shape() -> cq.Workplane:
    """Hollow elevator-car interior visible through the doorway.

    Five thin panels forming a box open at the front (y = 0):
    back wall, left wall, right wall, ceiling, and floor.
    """
    depth = SHAFT_BACK_Y
    t = 0.018
    w = SHAFT_W
    h = SHAFT_H

    back = _box_xyz(0.0, w, depth - t / 2.0, t, h / 2.0, h)
    left = _box_xyz(-w / 2.0 + t / 2.0, t, depth / 2.0, depth, h / 2.0, h)
    right = _box_xyz(w / 2.0 - t / 2.0, t, depth / 2.0, depth, h / 2.0, h)
    ceil_ = _box_xyz(0.0, w, depth / 2.0, depth, h - t / 2.0, t)
    floor_ = _box_xyz(0.0, w, depth / 2.0, depth, t / 2.0, t)

    return back.union(left).union(right).union(ceil_).union(floor_)


def _sill_shape() -> cq.Workplane:
    """Stainless threshold sill with longitudinal door-track grooves."""
    yc = (SILL_FRONT_Y + SILL_BACK_Y) / 2.0
    yd = SILL_BACK_Y - SILL_FRONT_Y
    base = _box_xyz(0.0, SILL_W, yc, yd, SILL_TOP / 2.0, SILL_TOP)
    n_grooves = 5
    groove_w = 0.010
    groove_depth = 0.008
    spacing = 0.022
    for i in range(n_grooves):
        gy = DOOR_CY + (i - (n_grooves - 1) / 2.0) * spacing
        groove = _box_xyz(
            0.0, SILL_W + 0.02, gy, groove_w,
            SILL_TOP - groove_depth / 2.0 + 0.0005, groove_depth,
        )
        base = base.cut(groove)
    return base


def _indicator_box_shape() -> cq.Workplane:
    """Surface-mounted indicator housing on the upper infill panel."""
    y_front = -IND_D + 0.005  # protrudes forward, 5 mm seated into infill
    y_back = 0.005
    yc = (y_front + y_back) / 2.0
    yd = y_back - y_front
    box = _box_xyz(0.0, IND_W, yc, yd, IND_CZ, IND_H)
    box = box.edges("|Y").fillet(0.004)
    # Recessed pocket on the front face for the display.
    p_front = y_front + 0.004
    p_back = y_front + 0.016
    p_yc = (p_front + p_back) / 2.0
    p_yd = p_back - p_front
    pocket = _box_xyz(0.0, IND_W - 0.04, p_yc, p_yd, IND_CZ, IND_H - 0.04)
    return box.cut(pocket)


def _digit_shape() -> cq.Workplane:
    """Red digit '1' plus up-arrow on the indicator face."""
    # Plates sit in the indicator pocket, slightly proud toward -Y.
    y_front_box = -IND_D + 0.005
    plate_y_front = y_front_box - 0.002
    plate_y_back = y_front_box + 0.014
    plate_y_c = (plate_y_front + plate_y_back) / 2.0
    plate_y_d = plate_y_back - plate_y_front

    # Vertical stroke of "1".
    digit = _box_xyz(0.060, 0.014, plate_y_c, plate_y_d, IND_CZ, 0.070)
    serif = _box_xyz(0.060, 0.034, plate_y_c, plate_y_d, IND_CZ - 0.035, 0.012)
    digit = digit.union(serif)

    # Top-left flag of "1".
    top_flag = (
        cq.Workplane("XZ")
        .box(0.032, plate_y_d, 0.011)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -28)
        .translate((0.048, plate_y_c, IND_CZ + 0.029))
    )
    digit = digit.union(top_flag)

    # Up-pointing direction arrow.
    arrow = (
        cq.Workplane("XZ")
        .polyline([(-0.085, -0.030), (-0.035, -0.030), (-0.060, 0.030)])
        .close()
        .extrude(plate_y_d)
        .translate((0.0, plate_y_back, IND_CZ))
    )
    return digit.union(arrow)


def _call_plate_shape() -> cq.Workplane:
    """Surface-mounted call-button plate on the right mullion."""
    y_front = -PLATE_D + 0.002
    y_back = 0.002
    yc = (y_front + y_back) / 2.0
    yd = y_back - y_front
    plate = _box_xyz(PLATE_CX, PLATE_W, yc, yd, PLATE_CZ, PLATE_H)
    plate = plate.edges("|Y").fillet(0.006)
    return plate


def _call_buttons_shape() -> cq.Workplane:
    """Two round call buttons (up / down) proud of the call plate."""
    btn_r = 0.016
    btn_len = 0.010
    # Buttons rooted 3 mm into the plate face, protruding toward -Y.
    y_back = -PLATE_D + 0.002 + 0.003  # -0.007
    up = (
        cq.Workplane("XZ")
        .circle(btn_r)
        .extrude(btn_len)
        .translate((PLATE_CX, y_back, PLATE_CZ + 0.038))
    )
    down = (
        cq.Workplane("XZ")
        .circle(btn_r)
        .extrude(btn_len)
        .translate((PLATE_CX, y_back, PLATE_CZ - 0.038))
    )
    return up.union(down)


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="elevator_landing_entrance")

    model.material("steel_frame", rgba=STEEL_FRAME)
    model.material("steel", rgba=STEEL)
    model.material("infill", rgba=INFILL)
    model.material("indicator_black", rgba=INDICATOR_BLACK)
    model.material("red_led", rgba=RED_LED)
    model.material("shaft_dark", rgba=SHAFT_DARK)
    model.material("button_steel", rgba=BUTTON_STEEL)

    # ---- Root: narrow metal-framed pylon surround. ----
    frame = model.part("frame")

    # Vertical mullions (repeated sub-parts via loop).
    mullion_signs = [-1.0, 1.0]
    for i in range(2):
        xc = mullion_signs[i] * (OPEN_HALF + MULLION_W / 2.0)
        frame.visual(
            mesh_from_cadquery(_frame_member(xc, MULLION_W, FRAME_H),
                               f"mullion_{i}"),
            material="steel_frame",
        )

    # Head beam spanning between mullion outer edges.
    head_w = OPEN_W + 2.0 * MULLION_W  # 1.35
    frame.visual(
        mesh_from_cadquery(_frame_member(0.0, head_w, HEAD_BEAM_H, OPEN_H),
                           "head_beam"),
        material="steel_frame",
    )

    # Thin side infill panels (repeated via loop).
    for i in range(2):
        sign = mullion_signs[i]
        # 5 mm X-overlap with the mullion for geometric connectivity.
        inner_abs = OPEN_HALF + MULLION_W - 0.005
        outer_abs = OPEN_HALF + MULLION_W + INFILL_STRIP_W
        w = outer_abs - inner_abs  # 0.065
        xc = sign * (inner_abs + outer_abs) / 2.0
        frame.visual(
            mesh_from_cadquery(_infill_panel(xc, w, FRAME_H, INFILL_T),
                               f"infill_panel_{i}"),
            material="infill",
        )

    # Upper infill panel (above head beam, between mullions).
    upper_bottom = OPEN_H + HEAD_BEAM_H - 0.005  # 5 mm Z-overlap with head beam
    upper_h = FRAME_H - upper_bottom
    upper_w = OPEN_W + 0.010  # 5 mm X-overlap on each mullion inner face
    frame.visual(
        mesh_from_cadquery(
            _infill_panel(0.0, upper_w, upper_h, UPPER_INFILL_T, upper_bottom),
            "upper_infill",
        ),
        material="infill",
    )

    # Shaft recess behind the doors.
    frame.visual(
        mesh_from_cadquery(_shaft_recess_shape(), "shaft_recess"),
        material="shaft_dark",
    )

    # Indicator (surface-mounted on upper infill).
    frame.visual(
        mesh_from_cadquery(_indicator_box_shape(), "indicator_box"),
        material="indicator_black",
        name="indicator_box",
    )
    frame.visual(
        mesh_from_cadquery(_digit_shape(), "indicator_digit"),
        material="red_led",
        name="indicator_digit",
    )

    # Hall call panel (surface-mounted on right mullion).
    frame.visual(
        mesh_from_cadquery(_call_plate_shape(), "call_plate"),
        material="steel",
        name="call_plate",
    )
    frame.visual(
        mesh_from_cadquery(_call_buttons_shape(), "call_buttons"),
        material="button_steel",
        name="call_buttons",
    )

    # Threshold sill at the floor.
    frame.visual(
        mesh_from_cadquery(_sill_shape(), "sill_track"),
        material="steel",
        name="sill_track",
    )

    # ---- Door leaves (prismatic, centre-opening). ----
    door_signs = [-1.0, 1.0]
    door_axes = [(-1.0, 0.0, 0.0), (1.0, 0.0, 0.0)]
    for i in range(2):
        door = model.part(f"door_{i}")
        door.visual(
            mesh_from_cadquery(_leaf_shape(door_signs[i]), f"leaf_{i}"),
            material="steel",
        )
        model.articulation(
            f"frame_to_door_{i}",
            ArticulationType.PRISMATIC,
            parent=frame,
            child=door,
            # Joint origin at the sill-track surface (actual contact plane).
            origin=Origin(xyz=(0.0, 0.0, LEAF_BOTTOM)),
            axis=door_axes[i],
            motion_limits=MotionLimits(
                lower=0.0, upper=DOOR_TRAVEL, effort=400.0, velocity=0.5
            ),
        )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    door_0 = object_model.get_part("door_0")
    door_1 = object_model.get_part("door_1")
    j_0 = object_model.get_articulation("frame_to_door_0")
    j_1 = object_model.get_articulation("frame_to_door_1")

    # ---- Pylon frame geometry checks (the changed structural property). ----
    frame_aabb = ctx.part_world_aabb(frame)
    frame_w = frame_aabb[1][0] - frame_aabb[0][0]
    frame_h = frame_aabb[1][2] - frame_aabb[0][2]
    frame_d = frame_aabb[1][1] - frame_aabb[0][1]

    ctx.check(
        "pylon frame is narrow (not a broad wall)",
        frame_w < 1.80 and frame_w > OPEN_W,
        details=f"frame_width={frame_w:.3f} (expect ~{FRAME_W:.2f})",
    )
    ctx.check(
        "pylon frame is tall and grounded",
        frame_h > OPEN_H and frame_aabb[0][2] < 0.01,
        details=f"frame_h={frame_h:.3f} z_min={frame_aabb[0][2]:.4f}",
    )
    ctx.check(
        "pylon frame is thin metal (not deep stone)",
        frame_d < 0.50,
        details=f"frame_depth={frame_d:.3f}",
    )

    # ---- Closed pose (q = 0): leaves meet at centre seam. ----
    with ctx.pose({j_0: 0.0, j_1: 0.0}):
        ctx.expect_gap(
            door_1, door_0, axis="x",
            min_gap=-0.002, max_gap=0.004,
            name="closed leaves meet at centre seam",
        )
        la = ctx.part_world_aabb(door_0)
        ra = ctx.part_world_aabb(door_1)
        lcx = (la[0][0] + la[1][0]) / 2.0
        rcx = (ra[0][0] + ra[1][0]) / 2.0
        ctx.check(
            "closed leaves symmetric about centre",
            abs(lcx + rcx) < 0.01 and abs(lcx) > 0.2,
            details=f"left_cx={lcx:.4f}, right_cx={rcx:.4f}",
        )
        ctx.check(
            "leaves share z-span",
            abs(la[0][2] - ra[0][2]) < 0.002 and abs(la[1][2] - ra[1][2]) < 0.002,
            details=f"L_z=({la[0][2]:.3f},{la[1][2]:.3f}) "
                    f"R_z=({ra[0][2]:.3f},{ra[1][2]:.3f})",
        )
        total_w = ra[1][0] - la[0][0]
        ctx.check(
            "closed leaves cover opening width",
            total_w >= OPEN_W - 0.01,
            details=f"covered={total_w:.4f} vs opening={OPEN_W}",
        )
        ctx.check(
            "closed leaves span opening height",
            la[0][2] < 0.04 and la[1][2] > OPEN_H - 0.02,
            details=f"L_z=({la[0][2]:.3f},{la[1][2]:.3f})",
        )

    # ---- Open pose: leaves retract clear of the opening centre. ----
    with ctx.pose({j_0: DOOR_TRAVEL, j_1: DOOR_TRAVEL}):
        la_o = ctx.part_world_aabb(door_0)
        ra_o = ctx.part_world_aabb(door_1)
        centre_gap = ra_o[0][0] - la_o[1][0]
        ctx.check(
            "open leaves clear the opening centre",
            centre_gap > 0.9 * OPEN_W,
            details=f"centre_gap={centre_gap:.4f} vs opening={OPEN_W}",
        )

    # Decisive directional check: each leaf moves outward when opened.
    with ctx.pose({j_0: 0.0, j_1: 0.0}):
        lc = ctx.part_world_position(door_0)
        rc = ctx.part_world_position(door_1)
    with ctx.pose({j_0: DOOR_TRAVEL, j_1: DOOR_TRAVEL}):
        lo = ctx.part_world_position(door_0)
        ro = ctx.part_world_position(door_1)
    ctx.check(
        "door_0 opens toward -X, door_1 toward +X",
        lo[0] < lc[0] - 0.3 and ro[0] > rc[0] + 0.3,
        details=f"left {lc[0]:.3f}->{lo[0]:.3f}, right {rc[0]:.3f}->{ro[0]:.3f}",
    )

    # ---- Inline visuals in the right positions. ----
    ind_aabb = ctx.part_element_world_aabb(frame, elem="indicator_box")
    ctx.check(
        "indicator is above the doorway opening",
        ind_aabb[0][2] > OPEN_H,
        details=f"indicator_zmin={ind_aabb[0][2]:.4f} vs opening_h={OPEN_H}",
    )

    plate_aabb = ctx.part_element_world_aabb(frame, elem="call_plate")
    plate_cx = (plate_aabb[0][0] + plate_aabb[1][0]) / 2.0
    plate_cz = (plate_aabb[0][2] + plate_aabb[1][2]) / 2.0
    ctx.check(
        "call panel is beside the opening at hand height",
        plate_cx > OPEN_HALF and 0.9 < plate_cz < 1.3,
        details=f"plate_cx={plate_cx:.3f}, plate_cz={plate_cz:.3f}",
    )

    sill_aabb = ctx.part_element_world_aabb(frame, elem="sill_track")
    ctx.check(
        "sill sits at the floor",
        sill_aabb[0][2] < 0.005 and sill_aabb[1][2] < 0.05,
        details=f"sill_z=({sill_aabb[0][2]:.4f},{sill_aabb[1][2]:.4f})",
    )

    return ctx.report()


object_model = build_object_model()
