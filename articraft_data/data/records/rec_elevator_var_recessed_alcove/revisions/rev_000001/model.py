from __future__ import annotations

# Passenger elevator landing entrance — RECESSED ALCOVE variant.
#
# Coordinate convention (Z-up, meters):
#   +X = wall width        (doors slide along X)
#   +Y = wall thickness / depth, going back into the shaft
#   +Z = height            (ground/floor at z = 0)
#
# Y layout (front -> back):
#   wall front face          y = 0
#   alcove pocket            y in [0, ALCOVE_D]
#   door leaves              y in [ALCOVE_D-LEAF_T, ALCOVE_D]
#   back reveal / jamb       y in [ALCOVE_D, ALCOVE_D+JAMB_D]
#   granite wall body        y in [0, WALL_D]
#   shaft recess             y in [ALCOVE_D, ALCOVE_D+0.35]
#
# The granite wall has a rectangular alcove pocket cut into it from the front
# face. Reveal panels (brushed steel) line the pocket interior on the left,
# right, top, and back (header above the door). The door plane sits at the
# back of the alcove. Indicator is on the back reveal header; call panel is on
# the right reveal wall. Sill sits at the alcove floor.

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

# Granite wall slab: front face at y=0, body into +Y.
WALL_W = 2.60
WALL_H = 2.70
WALL_D = 0.25

# Doorway opening cut through the wall (at the back of the alcove).
OPEN_W = 1.15
OPEN_H = 2.15
OPEN_HALF = OPEN_W / 2.0  # 0.575

# Alcove pocket dimensions (the recessed niche).
ALCOVE_W = 1.50   # mouth width (wider than door opening)
ALCOVE_H = 2.40   # mouth height (taller than door opening)
ALCOVE_D = 0.18   # pocket depth from wall front

# Reveal panel thickness (brushed-steel cladding on pocket faces).
REVEAL_T = 0.015

# Door leaves (brushed stainless). Each covers half the opening when closed.
LEAF_W = OPEN_HALF  # 0.575
LEAF_BOTTOM = 0.018  # rests at the sill-track top
LEAF_TOP = OPEN_H  # reaches the opening head
LEAF_H = LEAF_TOP - LEAF_BOTTOM
LEAF_T = 0.040
# Doors sit at the back of the alcove.
DOOR_FRONT_Y = ALCOVE_D - LEAF_T  # 0.140
DOOR_BACK_Y = ALCOVE_D             # 0.180
DOOR_CY = (DOOR_FRONT_Y + DOOR_BACK_Y) / 2.0  # 0.160

# Jamb / steel frame lining the opening (at the alcove back, into +Y).
JAMB_T = 0.045  # how far the jamb reaches inward over the opening edge
JAMB_D = 0.08   # jamb depth along +Y from the door plane

# Prismatic travel: each leaf retracts into its pocket behind the reveal.
DOOR_TRAVEL = 0.52

# Indicator box on the back reveal header above the doorway.
IND_W = 0.34
IND_H = 0.13
IND_D = 0.04
IND_CZ = OPEN_H + 0.12  # 2.27, above the door on the back reveal

# Hall call-button plate on the right reveal wall.
PLATE_W = 0.085   # along Y (pocket depth direction)
PLATE_H = 0.160   # along Z (vertical)
PLATE_EMBED = 0.012
PLATE_CY = ALCOVE_D / 2.0   # 0.09, centered in pocket depth
PLATE_CZ = 1.10              # hand height

# Threshold sill at the alcove floor.
SILL_W = OPEN_W + 0.10
SILL_FRONT_Y = ALCOVE_D - 0.12   # 0.06, extends into the alcove
SILL_BACK_Y = ALCOVE_D + 0.03    # 0.21, slightly behind door plane
SILL_TOP = LEAF_BOTTOM            # 0.018

# Shaft recess behind the doors.
SHAFT_W = OPEN_W
SHAFT_H = OPEN_H
SHAFT_FRONT_Y = ALCOVE_D
SHAFT_BACK_Y = ALCOVE_D + 0.35

# ---------------------------------------------------------------------------
# Colors / materials
# ---------------------------------------------------------------------------
GRANITE = (0.17, 0.17, 0.20, 1.0)
STEEL = (0.72, 0.73, 0.75, 1.0)
STEEL_DARK = (0.55, 0.56, 0.58, 1.0)
REVEAL_STEEL = (0.45, 0.46, 0.50, 1.0)  # darker brushed steel for reveals
INDICATOR_BLACK = (0.04, 0.04, 0.05, 1.0)
RED_LED = (0.88, 0.10, 0.10, 1.0)
SHAFT_DARK = (0.05, 0.05, 0.06, 1.0)
BUTTON_STEEL = (0.80, 0.81, 0.83, 1.0)


# ---------------------------------------------------------------------------
# Geometry helpers (CadQuery, authored directly in meters)
# ---------------------------------------------------------------------------
def _box_xyz(xc: float, xs: float, yc: float, ys: float, zc: float, zs: float) -> cq.Workplane:
    """Axis-aligned box from explicit center+size on each axis."""
    return cq.Workplane("XY").box(xs, ys, zs).translate((xc, yc, zc))


def _granite_wall_shape() -> cq.Workplane:
    """Dark granite slab with alcove pocket and doorway tunnel cut through it.

    Front face at y=0, back face at y=WALL_D. The alcove pocket is a
    rectangular cutout from the front face (ALCOVE_W × ALCOVE_H × ALCOVE_D).
    The doorway tunnel continues through the remaining wall behind the pocket.
    """
    slab = _box_xyz(0.0, WALL_W, WALL_D / 2.0, WALL_D, WALL_H / 2.0, WALL_H)
    # Cut alcove pocket from front face (slightly oversized in Y for clean cut).
    pocket = _box_xyz(
        0.0, ALCOVE_W, ALCOVE_D / 2.0, ALCOVE_D + 0.002,
        ALCOVE_H / 2.0, ALCOVE_H,
    )
    slab = slab.cut(pocket)
    # Cut door opening through remaining wall behind pocket.
    rem_start = ALCOVE_D
    rem_end = WALL_D + 0.002
    door_hole = _box_xyz(
        0.0, OPEN_W, (rem_start + rem_end) / 2.0, rem_end - rem_start,
        OPEN_H / 2.0, OPEN_H,
    )
    slab = slab.cut(door_hole)
    return slab


def _side_reveal_shape(sign: float) -> cq.Workplane:
    """Side reveal panel lining the left (sign=-1) or right (sign=+1) pocket wall."""
    cx = sign * (ALCOVE_W / 2.0 - REVEAL_T / 2.0)
    return _box_xyz(cx, REVEAL_T, ALCOVE_D / 2.0, ALCOVE_D, ALCOVE_H / 2.0, ALCOVE_H)


def _top_reveal_shape() -> cq.Workplane:
    """Top reveal panel lining the top of the alcove pocket."""
    return _box_xyz(
        0.0, ALCOVE_W - 2.0 * REVEAL_T,
        ALCOVE_D / 2.0, ALCOVE_D,
        ALCOVE_H - REVEAL_T / 2.0, REVEAL_T,
    )


def _back_reveal_shape() -> cq.Workplane:
    """Back reveal header panel above the door opening at the alcove back.

    A thin panel at y ≈ ALCOVE_D spanning the full alcove width, with the
    door opening cut through it. Only the header (above the door) remains
    visible; the side returns are part of the granite cross-section.
    """
    header_h = ALCOVE_H - OPEN_H
    cz = OPEN_H + header_h / 2.0
    panel = _box_xyz(
        0.0, ALCOVE_W - 2.0 * REVEAL_T,
        ALCOVE_D - REVEAL_T / 2.0, REVEAL_T,
        cz, header_h,
    )
    return panel


def _jamb_shape() -> cq.Workplane:
    """Brushed-steel U-jamb lining the doorway opening at the alcove back."""
    outer_w = OPEN_W + 2.0 * JAMB_T
    outer_top = OPEN_H + JAMB_T
    jamb_y_c = ALCOVE_D + JAMB_D / 2.0
    frame = _box_xyz(0.0, outer_w, jamb_y_c, JAMB_D, outer_top / 2.0, outer_top)
    clear = _box_xyz(
        0.0, OPEN_W, jamb_y_c, JAMB_D + 0.04,
        OPEN_H / 2.0 + 0.05, OPEN_H + 0.10,
    )
    band = frame.cut(clear)
    below = _box_xyz(0.0, outer_w + 0.05, jamb_y_c, JAMB_D + 0.05, -0.05, 0.10)
    return band.cut(below)


def _leaf_shape(sign: float) -> cq.Workplane:
    """A single brushed-stainless door leaf with a subtle recessed panel.

    `sign` = -1 (left) or +1 (right). The inner edge sits at x=0 (the center
    seam). Leaf spans z in [LEAF_BOTTOM, LEAF_TOP] and y in [DOOR_FRONT_Y, DOOR_BACK_Y].
    """
    cx = sign * LEAF_W / 2.0
    cz = (LEAF_BOTTOM + LEAF_TOP) / 2.0
    body = _box_xyz(cx, LEAF_W, DOOR_CY, LEAF_T, cz, LEAF_H)
    # Subtle recessed panel on the front face (shallow pocket).
    pocket_w = LEAF_W - 0.12
    pocket_h = LEAF_H - 0.20
    pocket = _box_xyz(
        cx, pocket_w, DOOR_FRONT_Y + 0.004, 0.010, cz, pocket_h,
    )
    leaf = body.cut(pocket)
    leaf = leaf.edges("|Z").fillet(0.004)
    return leaf


def _shaft_recess_shape() -> cq.Workplane:
    """Hollow elevator-car interior visible through the doorway."""
    depth = SHAFT_BACK_Y - SHAFT_FRONT_Y
    t = 0.018
    w = SHAFT_W
    h = SHAFT_H
    y_c = (SHAFT_FRONT_Y + SHAFT_BACK_Y) / 2.0

    back = _box_xyz(0.0, w, SHAFT_BACK_Y - t / 2.0, t, h / 2.0, h)
    left = _box_xyz(-w / 2.0 + t / 2.0, t, y_c, depth, h / 2.0, h)
    right = _box_xyz(w / 2.0 - t / 2.0, t, y_c, depth, h / 2.0, h)
    ceil_ = _box_xyz(0.0, w, y_c, depth, h - t / 2.0, t)
    floor_ = _box_xyz(0.0, w, y_c, depth, t / 2.0, t)
    return back.union(left).union(right).union(ceil_).union(floor_)


def _sill_shape() -> cq.Workplane:
    """Stainless threshold sill at the alcove floor with longitudinal door-track grooves."""
    yc = (SILL_FRONT_Y + SILL_BACK_Y) / 2.0
    yd = SILL_BACK_Y - SILL_FRONT_Y
    base = _box_xyz(0.0, SILL_W, yc, yd, SILL_TOP / 2.0, SILL_TOP)
    # Longitudinal grooves running along X (the slide direction), in the top face.
    n_grooves = 5
    groove_w = 0.010
    groove_depth = 0.008
    spacing = 0.022
    for i in range(n_grooves):
        gy = DOOR_CY + (i - (n_grooves - 1) / 2.0) * spacing
        groove = _box_xyz(
            0.0,
            SILL_W + 0.02,
            gy,
            groove_w,
            SILL_TOP - groove_depth / 2.0 + 0.0005,
            groove_depth,
        )
        base = base.cut(groove)
    return base


def _indicator_box_shape() -> cq.Workplane:
    """Recessed indicator housing on the back reveal header above the door."""
    yc = ALCOVE_D - IND_D / 2.0
    box = _box_xyz(0.0, IND_W, yc, IND_D, IND_CZ, IND_H)
    # Shallow front pocket (recessed glass face) opening toward -Y.
    pocket = _box_xyz(
        0.0, IND_W - 0.04, ALCOVE_D - 0.006, 0.014, IND_CZ, IND_H - 0.04,
    )
    return box.cut(pocket)


def _digit_shape() -> cq.Workplane:
    """Red digit '1' plus a small up-arrow on the indicator face."""
    plate_y_front = ALCOVE_D - IND_D + 0.002
    plate_y_back = ALCOVE_D - 0.003
    plate_y_c = (plate_y_front + plate_y_back) / 2.0
    plate_y_d = plate_y_back - plate_y_front
    # Vertical stroke of the digit "1".
    digit = _box_xyz(0.060, 0.014, plate_y_c, plate_y_d, IND_CZ, 0.070)
    # Small base serif for the "1".
    serif = _box_xyz(0.060, 0.034, plate_y_c, plate_y_d, IND_CZ - 0.035, 0.012)
    digit = digit.union(serif)
    # Top-left angled flag of the "1".
    top_flag = (
        cq.Workplane("XZ")
        .box(0.032, plate_y_d, 0.011)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -28)
        .translate((0.048, plate_y_c, IND_CZ + 0.029))
    )
    digit = digit.union(top_flag)
    # Up-pointing direction arrow to the left of the digit.
    arrow = (
        cq.Workplane("XZ")
        .polyline([(-0.085, -0.030), (-0.035, -0.030), (-0.060, 0.030)])
        .close()
        .extrude(plate_y_d)
        .translate((0.0, plate_y_back, IND_CZ))
    )
    return digit.union(arrow)


def _call_plate_shape() -> cq.Workplane:
    """Hall call plate on the right reveal wall, facing inward (-X)."""
    x_face = ALCOVE_W / 2.0 - REVEAL_T  # reveal inner face
    x_front = x_face + 0.002  # slightly proud of reveal
    x_back = x_face - PLATE_EMBED  # embedded into reveal
    xc = (x_front + x_back) / 2.0
    xd = x_front - x_back
    plate = _box_xyz(xc, xd, PLATE_CY, PLATE_W, PLATE_CZ, PLATE_H)
    plate = plate.edges("|X").fillet(0.006)
    return plate


def _call_buttons_shape() -> cq.Workplane:
    """Two round call buttons protruding from the right reveal plate toward -X."""
    btn_r = 0.016
    btn_len = 0.020
    # On YZ workplane, extrude(+d) goes in +X direction.
    # Place the start at the front (protruding) end; extrude goes +X (into plate).
    x_front = ALCOVE_W / 2.0 - REVEAL_T - 0.008  # protruding into alcove

    up = (
        cq.Workplane("YZ")
        .circle(btn_r)
        .extrude(btn_len)
        .translate((x_front, PLATE_CY, PLATE_CZ + 0.038))
    )
    down = (
        cq.Workplane("YZ")
        .circle(btn_r)
        .extrude(btn_len)
        .translate((x_front, PLATE_CY, PLATE_CZ - 0.038))
    )
    # Connecting backing plate (hidden behind call plate, embedded in reveal).
    # Ensures the two button cylinders form one connected mesh component.
    x_back = x_front + btn_len
    backing = _box_xyz(
        x_back - 0.003, 0.010,
        PLATE_CY, PLATE_W * 0.5,
        PLATE_CZ, 2.0 * 0.038 + 0.010,
    )
    return up.union(down).union(backing)


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="elevator_landing_entrance_alcove")

    model.material("granite", rgba=GRANITE)
    model.material("steel", rgba=STEEL)
    model.material("steel_dark", rgba=STEEL_DARK)
    model.material("reveal_steel", rgba=REVEAL_STEEL)
    model.material("indicator_black", rgba=INDICATOR_BLACK)
    model.material("red_led", rgba=RED_LED)
    model.material("shaft_dark", rgba=SHAFT_DARK)
    model.material("button_steel", rgba=BUTTON_STEEL)

    # --- Fixed root: the granite wall surround with alcove reveals. ---
    wall = model.part("wall_surround")
    wall.visual(
        mesh_from_cadquery(_granite_wall_shape(), "granite_slab"),
        material="granite",
        name="granite_slab",
    )
    wall.visual(
        mesh_from_cadquery(_jamb_shape(), "door_jamb"),
        material="steel_dark",
        name="door_jamb",
    )
    wall.visual(
        mesh_from_cadquery(_shaft_recess_shape(), "shaft_recess"),
        material="shaft_dark",
        name="shaft_recess",
    )
    # Alcove reveal panels (repeated side reveals via loop).
    for i in range(2):
        sign = -1.0 + 2.0 * i  # -1 for i=0 (left), +1 for i=1 (right)
        wall.visual(
            mesh_from_cadquery(_side_reveal_shape(sign), f"reveal_side_{i}"),
            material="reveal_steel",
            name=f"reveal_side_{i}",
        )
    wall.visual(
        mesh_from_cadquery(_top_reveal_shape(), "reveal_top"),
        material="reveal_steel",
        name="reveal_top",
    )
    wall.visual(
        mesh_from_cadquery(_back_reveal_shape(), "reveal_back"),
        material="reveal_steel",
        name="reveal_back",
    )

    # Indicator: on the back reveal header above the door.
    indicator = model.part("indicator")
    indicator.visual(
        mesh_from_cadquery(_indicator_box_shape(), "indicator_box"),
        material="indicator_black",
        name="indicator_box",
    )
    indicator.visual(
        mesh_from_cadquery(_digit_shape(), "indicator_digit"),
        material="red_led",
        name="indicator_digit",
    )

    # Hall call panel: on the right reveal wall.
    call_panel = model.part("call_panel")
    call_panel.visual(
        mesh_from_cadquery(_call_plate_shape(), "call_plate"),
        material="steel",
        name="call_plate",
    )
    call_panel.visual(
        mesh_from_cadquery(_call_buttons_shape(), "call_buttons"),
        material="button_steel",
        name="call_buttons",
    )

    # Threshold sill: at the alcove floor.
    sill = model.part("sill")
    sill.visual(
        mesh_from_cadquery(_sill_shape(), "sill_track"),
        material="steel",
        name="sill_track",
    )

    # --- Door leaves (brushed stainless), created via loop. ---
    door_parts = []
    for i in range(2):
        sign = -1.0 + 2.0 * i  # -1 (left) or +1 (right)
        door = model.part(f"door_{i}")
        door.visual(
            mesh_from_cadquery(_leaf_shape(sign), f"leaf_{i}"),
            material="steel",
            name=f"leaf_{i}",
        )
        door_parts.append(door)

    # --- Fixed mounts of surround sub-parts to the wall root. ---
    model.articulation(
        "wall_to_indicator",
        ArticulationType.FIXED,
        parent=wall,
        child=indicator,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    model.articulation(
        "wall_to_call_panel",
        ArticulationType.FIXED,
        parent=wall,
        child=call_panel,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    model.articulation(
        "wall_to_sill",
        ArticulationType.FIXED,
        parent=wall,
        child=sill,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # --- Prismatic door joints: lower = CLOSED, upper = OPEN. ---
    # Uniform joint policy: both leaves use the same origin and limits.
    door_axes = [(-1.0, 0.0, 0.0), (1.0, 0.0, 0.0)]
    for i in range(2):
        model.articulation(
            f"wall_to_door_{i}",
            ArticulationType.PRISMATIC,
            parent=wall,
            child=door_parts[i],
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
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

    wall = object_model.get_part("wall_surround")
    indicator = object_model.get_part("indicator")
    call_panel = object_model.get_part("call_panel")
    sill = object_model.get_part("sill")
    door_0 = object_model.get_part("door_0")
    door_1 = object_model.get_part("door_1")
    j_0 = object_model.get_articulation("wall_to_door_0")
    j_1 = object_model.get_articulation("wall_to_door_1")

    # Intentional overlaps: surround sub-parts are seated/embedded in the wall.
    ctx.allow_overlap(
        sill, wall,
        reason="The threshold sill butts into the wall at the alcove floor.",
    )
    ctx.allow_overlap(
        indicator, wall,
        reason="The indicator housing is recessed into the back reveal header above the doorway.",
    )
    ctx.allow_overlap(
        call_panel, wall,
        reason="The call plate is flush-mounted into the right alcove reveal wall.",
    )
    # Prove each embed actually contacts the wall.
    ctx.expect_contact(indicator, wall, name="indicator is seated in the back reveal")
    ctx.expect_contact(call_panel, wall, name="call panel is seated in the right reveal")
    ctx.expect_contact(sill, wall, name="sill is seated at the alcove floor")

    # --- Alcove geometry: doors are recessed behind the wall front face. ---
    wall_aabb = ctx.part_world_aabb(wall)
    door_0_aabb = ctx.part_world_aabb(door_0)
    alcove_setback = door_0_aabb[0][1] - wall_aabb[0][1]
    ctx.check(
        "doors are recessed into the alcove (setback from wall front)",
        alcove_setback > 0.10,
        details=f"door_min_y={door_0_aabb[0][1]:.3f}, wall_min_y={wall_aabb[0][1]:.3f}, setback={alcove_setback:.3f}",
    )

    # Wall front face is at y ≈ 0 and extends well beyond the alcove mouth.
    ctx.check(
        "wall surround is large and grounded",
        wall_aabb[0][2] < 0.01
        and (wall_aabb[1][0] - wall_aabb[0][0]) > ALCOVE_W + 0.5
        and (wall_aabb[1][2] - wall_aabb[0][2]) > ALCOVE_H,
        details=f"wall_x=({wall_aabb[0][0]:.2f},{wall_aabb[1][0]:.2f}) wall_z=({wall_aabb[0][2]:.2f},{wall_aabb[1][2]:.2f})",
    )

    # --- Reveals exist as named visuals of wall_surround. ---
    reveal_names = {"reveal_side_0", "reveal_side_1", "reveal_top", "reveal_back"}
    wall_visual_names = {v.name for v in wall.visuals}
    ctx.check(
        "alcove reveal panels are named visuals of wall_surround",
        reveal_names.issubset(wall_visual_names),
        details=f"expected={reveal_names}, found={wall_visual_names}",
    )

    # --- Indicator is above the doorway, at the alcove back. ---
    ind_aabb = ctx.part_world_aabb(indicator)
    ind_cy = (ind_aabb[0][1] + ind_aabb[1][1]) / 2.0
    ctx.check(
        "indicator is above the doorway opening",
        ind_aabb[0][2] > OPEN_H,
        details=f"indicator_zmin={ind_aabb[0][2]:.4f} vs opening_h={OPEN_H}",
    )
    ctx.check(
        "indicator is at the alcove back plane",
        ind_aabb[1][1] > ALCOVE_D - 0.03 and ind_aabb[0][1] > ALCOVE_D - IND_D - 0.02,
        details=f"indicator_y=({ind_aabb[0][1]:.3f},{ind_aabb[1][1]:.3f}), alcove_d={ALCOVE_D}",
    )

    # --- Call panel is on the right reveal wall at hand height. ---
    plate_aabb = ctx.part_world_aabb(call_panel)
    plate_cx = (plate_aabb[0][0] + plate_aabb[1][0]) / 2.0
    plate_cz = (plate_aabb[0][2] + plate_aabb[1][2]) / 2.0
    ctx.check(
        "call panel is on the right reveal at hand height",
        plate_cx > ALCOVE_W / 2.0 - 0.05 and 0.9 < plate_cz < 1.3,
        details=f"plate_cx={plate_cx:.3f}, plate_cz={plate_cz:.3f}",
    )

    # --- Sill sits at the floor. ---
    sill_aabb = ctx.part_world_aabb(sill)
    ctx.check(
        "sill sits at the floor (z ~ 0)",
        sill_aabb[0][2] < 0.005 and sill_aabb[1][2] < 0.05,
        details=f"sill_z=({sill_aabb[0][2]:.4f},{sill_aabb[1][2]:.4f})",
    )

    # --- Closed pose (q=0): leaves meet at the center seam and cover opening. ---
    with ctx.pose({j_0: 0.0, j_1: 0.0}):
        ctx.expect_gap(
            door_1, door_0,
            axis="x",
            min_gap=-0.002,
            max_gap=0.004,
            name="closed leaves meet at center seam",
        )
        la = ctx.part_world_aabb(door_0)
        ra = ctx.part_world_aabb(door_1)
        lcx = (la[0][0] + la[1][0]) / 2.0
        rcx = (ra[0][0] + ra[1][0]) / 2.0
        ctx.check(
            "closed leaves are symmetric about center",
            abs(lcx + rcx) < 0.01 and abs(lcx) > 0.2,
            details=f"left_cx={lcx:.4f}, right_cx={rcx:.4f}",
        )
        ctx.check(
            "leaves share z-span",
            abs(la[0][2] - ra[0][2]) < 0.002 and abs(la[1][2] - ra[1][2]) < 0.002,
            details=f"L_z=({la[0][2]:.3f},{la[1][2]:.3f}) R_z=({ra[0][2]:.3f},{ra[1][2]:.3f})",
        )
        total_w = ra[1][0] - la[0][0]
        ctx.check(
            "closed leaves cover the opening width",
            total_w >= OPEN_W - 0.01,
            details=f"covered_width={total_w:.4f} vs opening={OPEN_W}",
        )
        ctx.check(
            "closed leaves span the opening height",
            la[0][2] < 0.03 and la[1][2] > OPEN_H - 0.02,
            details=f"L_z=({la[0][2]:.3f},{la[1][2]:.3f})",
        )

    # --- Open pose (upper): leaves retract clear of the opening center. ---
    with ctx.pose({j_0: DOOR_TRAVEL, j_1: DOOR_TRAVEL}):
        la_o = ctx.part_world_aabb(door_0)
        ra_o = ctx.part_world_aabb(door_1)
        center_gap = ra_o[0][0] - la_o[1][0]
        ctx.check(
            "open leaves clear the opening center",
            center_gap > 0.9 * OPEN_W,
            details=f"center_gap={center_gap:.4f} vs opening={OPEN_W}",
        )

    # Decisive directional check: each leaf moves outward when opened.
    with ctx.pose({j_0: 0.0, j_1: 0.0}):
        lc = ctx.part_world_position(door_0)
        rc = ctx.part_world_position(door_1)
    with ctx.pose({j_0: DOOR_TRAVEL, j_1: DOOR_TRAVEL}):
        lo = ctx.part_world_position(door_0)
        ro = ctx.part_world_position(door_1)
    ctx.check(
        "left leaf opens toward -X, right toward +X",
        lo[0] < lc[0] - 0.3 and ro[0] > rc[0] + 0.3,
        details=f"left {lc[0]:.3f}->{lo[0]:.3f}, right {rc[0]:.3f}->{ro[0]:.3f}",
    )

    return ctx.report()


object_model = build_object_model()
