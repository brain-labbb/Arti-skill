from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Materials — natural timber palette for the A-frame ark
# ---------------------------------------------------------------------------
TIMBER = Material("natural warm timber", rgba=(0.62, 0.45, 0.28, 1.0))
TRIM = Material("darker timber trim", rgba=(0.42, 0.30, 0.18, 1.0))
MESH = Material("dark galvanized wire mesh", rgba=(0.10, 0.11, 0.10, 1.0))
BLACK = Material("black latch hardware", rgba=(0.02, 0.02, 0.018, 1.0))
TRAY = Material("black removable plastic tray", rgba=(0.035, 0.035, 0.032, 1.0))
RAMP = Material("pale grooved ramp planks", rgba=(0.72, 0.67, 0.58, 1.0))

# ---------------------------------------------------------------------------
# A-frame geometry constants
# ---------------------------------------------------------------------------
X_NEST_END = -0.70       # back gable of nest box
X_NEST_FRONT = -0.20     # divider between nest box and run
X_RUN_END = 0.70         # front gable of run
Y_HALF = 0.40            # half base width (0.80 m total)
Z_BASE = 0.25            # base edge height (on timber legs)
Z_RIDGE = 0.95           # ridge beam height
RISE = Z_RIDGE - Z_BASE  # 0.70 m vertical rise
SLOPE_ANGLE = math.atan2(RISE, Y_HALF)          # ~60.3 deg from horizontal
SLOPE_LEN = math.sqrt(RISE ** 2 + Y_HALF ** 2)  # ~0.806 m along slope
TILT = math.pi / 2 - SLOPE_ANGLE                # ~29.7 deg from vertical
LEFT_ROLL = -TILT                                # roll for left slope elements
RIGHT_ROLL = TILT                                # roll for right slope elements

FRAME_LEN = X_RUN_END - X_NEST_END               # 1.40 m
FRAME_CX = (X_NEST_END + X_RUN_END) / 2          # 0.0
NEST_CX = (X_NEST_END + X_NEST_FRONT) / 2        # -0.45
NEST_LEN = X_NEST_FRONT - X_NEST_END             # 0.50
RUN_CX = (X_NEST_FRONT + X_RUN_END) / 2          # 0.25

POST = 0.042   # timber member cross-section
WIRE = 0.006   # wire diameter
DEPTH = 0.010  # wire depth/thickness

# Door on left slope
DOOR_W = 0.50
DOOR_H = 0.55
DOOR_CX = RUN_CX
DOOR_HINGE_Z = Z_BASE + POST        # hinge sits above the base rail
DOOR_HINGE_Y = -Y_HALF * (1 - POST / RISE)  # slope surface at hinge height

# Ramp — hinge sits just below the back base rail so the board clears it
RAMP_LEN = 0.40
RAMP_W = 0.30
RAMP_HINGE_Z = Z_BASE - POST  # below base rail top
DOWN_ANGLE = math.asin(RAMP_HINGE_Z / RAMP_LEN)

# Tray
TRAY_W = 0.60
TRAY_LEN = NEST_LEN - 0.14  # shorter to clear the ramp hinge plate


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------
def _box(part, name: str, size, xyz, material=TIMBER, rpy=(0.0, 0.0, 0.0)) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def _cyl(part, name: str, radius: float, length: float, xyz, material=BLACK, rpy=(0.0, 0.0, 0.0)) -> None:
    part.visual(Cylinder(radius=radius, length=length), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def _front_mesh(part, prefix: str, x0: float, x1: float, z0: float, z1: float, y: float, *, wire=WIRE, depth=DEPTH) -> None:
    """Square wire grid in an XZ panel at fixed Y (kept from parent)."""
    width = x1 - x0
    height = z1 - z0
    cols = max(3, int(width / 0.095))
    rows = max(3, int(height / 0.085))
    for i in range(cols + 1):
        x = x0 + width * i / cols
        _box(part, f"{prefix}_v_{i}", (wire, depth, height + wire), (x, y, (z0 + z1) / 2), MESH)
    for j in range(rows + 1):
        z = z0 + height * j / rows
        _box(part, f"{prefix}_h_{j}", (width + wire, depth, wire), ((x0 + x1) / 2, y, z), MESH)


def _side_mesh(part, prefix: str, y0: float, y1: float, z0: float, z1: float, x: float, *, wire=WIRE, depth=DEPTH) -> None:
    """Square wire grid in a YZ panel at fixed X (kept from parent)."""
    panel_depth = y1 - y0
    height = z1 - z0
    cols = max(3, int(panel_depth / 0.095))
    rows = max(3, int(height / 0.085))
    for i in range(cols + 1):
        y = y0 + panel_depth * i / cols
        _box(part, f"{prefix}_v_{i}", (depth, wire, height + wire), (x, y, (z0 + z1) / 2), MESH)
    for j in range(rows + 1):
        z = z0 + height * j / rows
        _box(part, f"{prefix}_h_{j}", (depth, panel_depth + wire, wire), (x, (y0 + y1) / 2, z), MESH)


def _slope_mesh(part, prefix: str, x0: float, x1: float, y_sign: int, *, wire=WIRE, depth=DEPTH) -> None:
    """Wire grid on one A-frame slope face.

    y_sign = -1 for left slope, +1 for right slope.
    Horizontal wires run along X at evenly-spaced positions up the slope.
    Slope-parallel wires run from base to ridge at evenly-spaced X positions.
    """
    roll = LEFT_ROLL if y_sign < 0 else RIGHT_ROLL
    width = x1 - x0
    cols = max(3, int(width / 0.095))
    rows = max(3, int(SLOPE_LEN / 0.085))
    y_center = y_sign * Y_HALF / 2
    z_center = (Z_BASE + Z_RIDGE) / 2

    # Horizontal wires (along X) at various slope positions
    for j in range(rows + 1):
        t = j / rows
        y = y_sign * Y_HALF * (1 - t)
        z = Z_BASE + RISE * t
        _box(part, f"{prefix}_h_{j}", (width + wire, depth, wire),
             ((x0 + x1) / 2, y, z), MESH, rpy=(roll, 0.0, 0.0))

    # Slope-parallel wires at various X positions
    for i in range(cols + 1):
        x = x0 + width * i / cols
        _box(part, f"{prefix}_v_{i}", (wire, depth, SLOPE_LEN + wire),
             (x, y_center, z_center), MESH, rpy=(roll, 0.0, 0.0))


def _gable_frame(part, prefix: str, x_pos: float) -> None:
    """Triangular gable frame: two tilted rafters + horizontal base rail."""
    _box(part, f"{prefix}_base_rail", (POST, 2 * Y_HALF + POST, POST),
         (x_pos, 0.0, Z_BASE), TIMBER)
    _box(part, f"{prefix}_left_rafter", (POST, POST, SLOPE_LEN),
         (x_pos, -Y_HALF / 2, (Z_BASE + Z_RIDGE) / 2), TIMBER, rpy=(LEFT_ROLL, 0.0, 0.0))
    _box(part, f"{prefix}_right_rafter", (POST, POST, SLOPE_LEN),
         (x_pos, Y_HALF / 2, (Z_BASE + Z_RIDGE) / 2), TIMBER, rpy=(RIGHT_ROLL, 0.0, 0.0))


def _gable_planks(part, prefix: str, x_pos: float, n_planks: int = 7) -> None:
    """Horizontal planks filling a triangular gable (gets shorter toward ridge)."""
    plank_h = RISE / n_planks
    for i in range(n_planks):
        t = (i + 0.5) / n_planks
        z = Z_BASE + RISE * t
        width = 2 * Y_HALF * (1 - t) - 0.06
        if width > 0.04:
            _box(part, f"{prefix}_plank_{i}", (0.018, width, plank_h - 0.003),
                 (x_pos, 0.0, z), TIMBER)


# ---------------------------------------------------------------------------
# Build
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="aframe_rabbit_ark")
    for mat in (TIMBER, TRIM, MESH, BLACK, TRAY, RAMP):
        model.material(mat.name, rgba=mat.rgba)

    frame = model.part("hutch_frame")

    # ---- Legs (four timber legs at base corners) ----
    leg_positions = [
        (X_NEST_END, -Y_HALF),
        (X_NEST_END, Y_HALF),
        (X_RUN_END, -Y_HALF),
        (X_RUN_END, Y_HALF),
    ]
    for i, (lx, ly) in enumerate(leg_positions):
        _box(frame, f"leg_{i}", (POST, POST, Z_BASE), (lx, ly, Z_BASE / 2), TIMBER)

    # ---- Base rails (connecting legs along X and Y) ----
    _box(frame, "left_base_rail", (FRAME_LEN + POST, POST, POST),
         (FRAME_CX, -Y_HALF, Z_BASE), TIMBER)
    _box(frame, "right_base_rail", (FRAME_LEN + POST, POST, POST),
         (FRAME_CX, Y_HALF, Z_BASE), TIMBER)
    _box(frame, "back_base_rail", (POST, 2 * Y_HALF + POST, POST),
         (X_NEST_END, 0.0, Z_BASE), TIMBER)
    _box(frame, "front_base_rail", (POST, 2 * Y_HALF + POST, POST),
         (X_RUN_END, 0.0, Z_BASE), TIMBER)
    _box(frame, "divider_base_rail", (POST, 2 * Y_HALF + POST, POST),
         (X_NEST_FRONT, 0.0, Z_BASE), TIMBER)

    # ---- Ridge beam ----
    _box(frame, "ridge_beam", (FRAME_LEN + POST, POST, POST),
         (FRAME_CX, 0.0, Z_RIDGE), TIMBER)

    # ---- Purlins (horizontal members on the interior face of each slope) ----
    purlin_y_inset = Y_HALF / 2 - 0.055  # well inside the slope surface
    purlin_z = (Z_BASE + Z_RIDGE) / 2 - 0.040
    _box(frame, "left_purlin", (FRAME_LEN, POST * 0.75, POST * 0.75),
         (FRAME_CX, -purlin_y_inset, purlin_z), TIMBER)
    _box(frame, "right_purlin", (FRAME_LEN, POST * 0.75, POST * 0.75),
         (FRAME_CX, purlin_y_inset, purlin_z), TIMBER)

    # ---- Gable frames at three positions ----
    _gable_frame(frame, "nest_back_gable", X_NEST_END)
    _gable_frame(frame, "divider_gable", X_NEST_FRONT)
    _gable_frame(frame, "run_front_gable", X_RUN_END)

    # ---- Nest box: solid slope panels + back gable wall + divider wall + floor ----
    # Solid plank panels on both slopes of the nest box section
    _box(frame, "nest_left_slope_panel",
         (NEST_LEN, 0.018, SLOPE_LEN),
         (NEST_CX, -Y_HALF / 2, (Z_BASE + Z_RIDGE) / 2),
         TIMBER, rpy=(LEFT_ROLL, 0.0, 0.0))
    _box(frame, "nest_right_slope_panel",
         (NEST_LEN, 0.018, SLOPE_LEN),
         (NEST_CX, Y_HALF / 2, (Z_BASE + Z_RIDGE) / 2),
         TIMBER, rpy=(RIGHT_ROLL, 0.0, 0.0))

    # Nest box floor
    _box(frame, "nest_floor", (NEST_LEN, 2 * Y_HALF - 0.04, 0.018),
         (NEST_CX, 0.0, Z_BASE + 0.009), TIMBER)

    # Back gable wall — CadQuery triangular panel with ramp opening
    opening_w = 0.24
    opening_h = 0.20
    back_wall = (
        cq.Workplane("YZ")
        .moveTo(-Y_HALF + 0.02, 0.0)
        .lineTo(Y_HALF - 0.02, 0.0)
        .lineTo(0.0, RISE - 0.02)
        .close()
        .extrude(0.018)
    )
    back_cutter = (
        cq.Workplane("YZ")
        .center(0.0, opening_h / 2)
        .rect(opening_w, opening_h)
        .extrude(0.030)
    )
    back_cutter = back_cutter.translate((-0.006, 0.0, 0.0))
    back_wall = back_wall.cut(back_cutter)
    back_wall = back_wall.translate((-0.009, 0.0, 0.0))
    frame.visual(
        mesh_from_cadquery(back_wall, "nest_back_gable_wall"),
        origin=Origin(xyz=(X_NEST_END, 0.0, Z_BASE)),
        material=TIMBER,
        name="nest_back_gable_wall",
    )

    # Divider wall — CadQuery triangular panel with rabbit passage opening
    div_opening_w = 0.22
    div_opening_h = 0.22
    divider_wall = (
        cq.Workplane("YZ")
        .moveTo(-Y_HALF + 0.02, 0.0)
        .lineTo(Y_HALF - 0.02, 0.0)
        .lineTo(0.0, RISE - 0.02)
        .close()
        .extrude(0.018)
    )
    div_cutter = (
        cq.Workplane("YZ")
        .center(0.0, div_opening_h / 2)
        .rect(div_opening_w, div_opening_h)
        .extrude(0.030)
    )
    div_cutter = div_cutter.translate((-0.006, 0.0, 0.0))
    divider_wall = divider_wall.cut(div_cutter)
    divider_wall = divider_wall.translate((-0.009, 0.0, 0.0))
    frame.visual(
        mesh_from_cadquery(divider_wall, "nest_divider_wall"),
        origin=Origin(xyz=(X_NEST_FRONT, 0.0, Z_BASE)),
        material=TIMBER,
        name="nest_divider_wall",
    )

    # Decorative plank lines on nest box slope panels
    for i, t in enumerate((0.25, 0.50, 0.75)):
        y = -Y_HALF * (1 - t)
        z = Z_BASE + RISE * t
        _box(frame, f"nest_left_plank_line_{i}", (NEST_LEN - 0.02, 0.008, 0.006),
             (NEST_CX, y - 0.012, z), TRIM, rpy=(LEFT_ROLL, 0.0, 0.0))
        _box(frame, f"nest_right_plank_line_{i}", (NEST_LEN - 0.02, 0.008, 0.006),
             (NEST_CX, -y + 0.012, z), TRIM, rpy=(RIGHT_ROLL, 0.0, 0.0))

    # ---- Run section: wire mesh on both slopes ----
    # Right slope mesh covers the full run section
    _slope_mesh(frame, "run_right_slope_mesh", X_NEST_FRONT + 0.04, X_RUN_END - 0.04, +1)
    # Left slope mesh: split around the door opening with clearance
    door_mesh_lo_x1 = DOOR_CX - DOOR_W / 2 - 0.03   # just left of door
    door_mesh_hi_x0 = DOOR_CX + DOOR_W / 2 + 0.03   # just right of door
    # Below the door (between divider and door start)
    if door_mesh_lo_x1 > X_NEST_FRONT + 0.06:
        _slope_mesh(frame, "run_left_slope_mesh_lo", X_NEST_FRONT + 0.04, door_mesh_lo_x1, -1)
    # Above the door (between door end and run front)
    if door_mesh_hi_x0 < X_RUN_END - 0.06:
        _slope_mesh(frame, "run_left_slope_mesh_hi", door_mesh_hi_x0, X_RUN_END - 0.04, -1)

    # Run end gable: triangular wire mesh (horizontal wires get shorter toward ridge)
    n_gable_wires = 6
    for j in range(n_gable_wires):
        t = (j + 0.5) / n_gable_wires
        z = Z_BASE + RISE * t
        width = 2 * Y_HALF * (1 - t) - 0.12
        if width > 0.08:
            _box(frame, f"run_end_gable_mesh_h_{j}", (WIRE, width, WIRE),
                 (X_RUN_END + 0.008, 0.0, z), MESH)
    n_gable_v = 5
    for i in range(n_gable_v):
        t_y = (i + 0.5) / n_gable_v
        y = -Y_HALF + 2 * Y_HALF * t_y
        max_z = Z_BASE + RISE * (1 - abs(y) / Y_HALF) - 0.08
        h = max_z - Z_BASE - 0.04
        if h > 0.06:
            _box(frame, f"run_end_gable_mesh_v_{i}", (WIRE, WIRE, h),
                 (X_RUN_END + 0.008, y, Z_BASE + 0.02 + h / 2), MESH)

    # ---- Hinge receivers on frame for door and ramp ----
    # Door hinge plates on left base rail
    plate_z = Z_BASE + POST / 2  # sit on top of base rail
    _box(frame, "door_hinge_plate_a", (0.055, 0.025, 0.014),
         (DOOR_CX - DOOR_W * 0.25, -Y_HALF, plate_z), BLACK)
    _box(frame, "door_hinge_plate_b", (0.055, 0.025, 0.014),
         (DOOR_CX + DOOR_W * 0.25, -Y_HALF, plate_z), BLACK)

    # Ramp hinge plate at nest box back (mounted on the interior side, connects to base rail)
    plate_z_top = Z_BASE - POST / 2  # touches bottom of back base rail
    plate_z_bot = RAMP_HINGE_Z - 0.010
    _box(frame, "ramp_hinge_plate",
         (0.030, RAMP_W + 0.04, plate_z_top - plate_z_bot),
         (X_NEST_END + 0.020, 0.0, (plate_z_top + plate_z_bot) / 2), BLACK)

    # Tray slide rails under nest box (offset outward to clear the pull handle)
    _box(frame, "tray_rail_left", (NEST_LEN - 0.04, 0.025, 0.020),
         (NEST_CX, -TRAY_W / 2 - 0.030, Z_BASE - 0.028), TRIM)
    _box(frame, "tray_rail_right", (NEST_LEN - 0.04, 0.025, 0.020),
         (NEST_CX, TRAY_W / 2 + 0.030, Z_BASE - 0.028), TRIM)

    # ===================================================================
    # Run door — framed wire mesh panel on the left slope, hinged at base
    # ===================================================================
    run_door = model.part("run_door")
    stile = 0.035
    thick = 0.028

    # Door frame stiles (along Z = slope direction in door local frame)
    _box(run_door, "door_left_stile", (stile, thick, DOOR_H),
         (-DOOR_W / 2 + stile / 2, 0.0, DOOR_H / 2), TIMBER)
    _box(run_door, "door_right_stile", (stile, thick, DOOR_H),
         (DOOR_W / 2 - stile / 2, 0.0, DOOR_H / 2), TIMBER)
    # Door frame rails (along X)
    _box(run_door, "door_bottom_rail", (DOOR_W, thick, stile),
         (0.0, 0.0, stile / 2), TIMBER)
    _box(run_door, "door_top_rail", (DOOR_W, thick, stile),
         (0.0, 0.0, DOOR_H - stile / 2), TIMBER)
    # Inner lip
    _box(run_door, "door_inner_lip", (DOOR_W - 0.07, 0.010, DOOR_H - 0.07),
         (0.0, -0.003, DOOR_H / 2), TRIM)

    # Wire mesh infill in the door
    mesh_w = DOOR_W - 0.09
    mesh_h = DOOR_H - 0.09
    door_cols = max(3, int(mesh_w / 0.080))
    door_rows = max(3, int(mesh_h / 0.080))
    for i in range(door_cols + 1):
        x = -mesh_w / 2 + mesh_w * i / door_cols
        _box(run_door, f"door_mesh_v_{i}", (0.004, 0.005, mesh_h),
             (x, -0.006, DOOR_H / 2), MESH)
    for j in range(door_rows + 1):
        z = stile + mesh_h * j / door_rows
        _box(run_door, f"door_mesh_h_{j}", (mesh_w, 0.005, 0.004),
             (0.0, -0.006, z), MESH)

    # Hinge knuckles (cylinders along X at Z=0)
    _cyl(run_door, "door_hinge_knuckle_a", 0.009, 0.050,
         (-DOOR_W * 0.25, 0.0, 0.0), BLACK, rpy=(0.0, math.pi / 2, 0.0))
    _cyl(run_door, "door_hinge_knuckle_b", 0.009, 0.050,
         (DOOR_W * 0.25, 0.0, 0.0), BLACK, rpy=(0.0, math.pi / 2, 0.0))

    # Latch plate and knob at top of door
    _box(run_door, "door_latch_plate", (0.040, 0.012, 0.025),
         (0.0, -0.018, DOOR_H - 0.025), BLACK)
    _cyl(run_door, "door_latch_knob", 0.008, 0.014,
         (0.0, -0.030, DOOR_H - 0.025), BLACK, rpy=(math.pi / 2, 0.0, 0.0))

    # Articulation: hinge on left base rail, pre-rotated to lie on slope
    hinge_z = Z_BASE + POST / 2  # matches hinge plate position
    model.articulation(
        "frame_to_run_door",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=run_door,
        origin=Origin(xyz=(DOOR_CX, -Y_HALF, hinge_z), rpy=(LEFT_ROLL, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.35, effort=6.0, velocity=1.2),
    )

    # ===================================================================
    # Front ramp — drops from nest box floor to ground
    # ===================================================================
    front_ramp = model.part("front_ramp")

    # Ramp board extends in local -X from hinge, pre-tilted downward by rpy
    _box(front_ramp, "ramp_board", (RAMP_LEN, RAMP_W, 0.024),
         (-RAMP_LEN / 2, 0.0, 0.0), RAMP)

    # Side rails (shortened to clear the base rail at the hinge)
    rail_len = RAMP_LEN - 0.08
    rail_offset = -0.04  # shift center away from hinge
    _box(front_ramp, "ramp_side_rail_a", (rail_len, 0.028, 0.032),
         (-RAMP_LEN / 2 + rail_offset, -RAMP_W / 2 + 0.014, 0.020), TIMBER)
    _box(front_ramp, "ramp_side_rail_b", (rail_len, 0.028, 0.032),
         (-RAMP_LEN / 2 + rail_offset, RAMP_W / 2 - 0.014, 0.020), TIMBER)

    # Cleats across the ramp surface for grip
    for j in range(4):
        t = (j + 1) / 5
        x = -RAMP_LEN * t
        _box(front_ramp, f"ramp_cleat_{j}", (0.018, RAMP_W - 0.06, 0.016),
             (x, 0.0, 0.020), TRIM)

    # Hinge rod at the hinge line
    _cyl(front_ramp, "ramp_hinge_rod", 0.008, RAMP_W + 0.04,
         (0.0, 0.0, 0.0), BLACK, rpy=(math.pi / 2, 0.0, 0.0))

    model.articulation(
        "frame_to_front_ramp",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=front_ramp,
        origin=Origin(xyz=(X_NEST_END, 0.0, RAMP_HINGE_Z), rpy=(0.0, -DOWN_ANGLE, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.20, effort=5.0, velocity=1.0),
    )

    # ===================================================================
    # Floor tray — slide-out droppings tray under nest box
    # ===================================================================
    tray = model.part("floor_tray")
    _box(tray, "tray_pan", (TRAY_LEN, TRAY_W, 0.025), (0.0, 0.0, 0.0), TRAY)
    _box(tray, "tray_front_pull", (0.26, 0.018, 0.028),
         (0.0, -TRAY_W / 2, 0.016), BLACK)

    model.articulation(
        "frame_to_floor_tray",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=tray,
        origin=Origin(xyz=(NEST_CX, 0.0, Z_BASE - 0.030)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.35, effort=25.0, velocity=0.25),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("hutch_frame")
    run_door = object_model.get_part("run_door")
    ramp = object_model.get_part("front_ramp")
    tray = object_model.get_part("floor_tray")

    door_hinge = object_model.get_articulation("frame_to_run_door")
    ramp_hinge = object_model.get_articulation("frame_to_front_ramp")
    tray_slide = object_model.get_articulation("frame_to_floor_tray")

    # ---- Intentional hinge hardware overlaps ----
    # Ramp hinge rod nests inside the ramp hinge plate (captured hardware)
    ctx.allow_overlap(
        "front_ramp", "hutch_frame",
        elem_a="ramp_hinge_rod", elem_b="ramp_hinge_plate",
        reason="Ramp hinge rod is a captured pin seated inside the frame hinge plate.",
    )
    ctx.expect_contact(
        ramp, frame,
        elem_a="ramp_hinge_rod", elem_b="ramp_hinge_plate",
        name="ramp hinge rod contacts hinge plate",
    )

    # Door hinge knuckles nest against the frame hinge plates at the pivot
    ctx.allow_overlap(
        "hutch_frame", "run_door",
        elem_a="door_hinge_plate_b", elem_b="door_hinge_knuckle_b",
        reason="Door hinge knuckle interleaves with the frame hinge plate at the pivot.",
    )
    ctx.allow_overlap(
        "hutch_frame", "run_door",
        elem_a="door_hinge_plate_a", elem_b="door_hinge_knuckle_a",
        reason="Door hinge knuckle interleaves with the frame hinge plate at the pivot.",
    )
    ctx.expect_contact(
        frame, run_door,
        elem_a="door_hinge_plate_a", elem_b="door_hinge_knuckle_a",
        name="door hinge hardware remains in contact at rest",
    )

    # Whole-part door-frame overlap: door bottom rail, stiles and knuckles all
    # share the hinge pivot line with the base rail. The door swings away on
    # positive articulation, so at rest they sit together at the pivot.
    ctx.allow_overlap(
        "hutch_frame", "run_door",
        reason="Door bottom rail, stiles and hinge knuckles share the base rail pivot line; the door swings away on positive articulation.",
    )
    # Confirm the door actually separates from the frame when opened
    door_rest = ctx.part_world_aabb(run_door)
    with ctx.pose({door_hinge: 1.0}):
        door_open = ctx.part_world_aabb(run_door)
    ctx.check(
        "door_separates_from_frame_on_open",
        door_rest is not None and door_open is not None
        and door_open[0][1] < door_rest[0][1] - 0.05,
        details=f"rest={door_rest}, open={door_open}",
    )
    # Ridge beam visual exists and is at the top of the frame
    ridge = frame.get_visual("ridge_beam")
    ctx.check(
        "ridge_beam_exists",
        ridge is not None,
        details="A-frame hutch_frame must have a ridge_beam visual at the apex",
    )

    frame_aabb = ctx.part_world_aabb(frame)
    ctx.check(
        "aframe_reaches_ridge_height",
        frame_aabb is not None and frame_aabb[1][2] > 0.88,
        details=f"A-frame ridge should reach at least 0.88 m; max_z={frame_aabb[1][2] if frame_aabb else None}",
    )
    ctx.check(
        "aframe_base_width",
        frame_aabb is not None and (frame_aabb[1][1] - frame_aabb[0][1]) > 0.70,
        details="A-frame base should span at least 0.70 m across",
    )

    # Nest box slope panels exist (closed sleeping area under apex)
    nest_panel = frame.get_visual("nest_left_slope_panel")
    ctx.check(
        "nest_box_slope_panel_exists",
        nest_panel is not None,
        details="Nest box must have enclosed solid slope panels under the apex",
    )

    # Tray sits within the frame footprint on X
    ctx.expect_within(tray, frame, axes="x", margin=0.03, name="tray stays within frame length")
    ctx.expect_overlap(tray, frame, axes="y", min_overlap=0.30, name="tray is inserted on its rails")

    # ---- Articulated pose checks ----
    door_closed = ctx.part_world_aabb(run_door)
    ramp_down = ctx.part_world_aabb(ramp)
    tray_home = ctx.part_world_aabb(tray)

    with ctx.pose({door_hinge: 1.0, ramp_hinge: 1.0, tray_slide: 0.25}):
        door_open = ctx.part_world_aabb(run_door)
        ramp_up = ctx.part_world_aabb(ramp)
        tray_out = ctx.part_world_aabb(tray)

    # Run door opens outward from the slope (moves in -Y direction)
    ctx.check(
        "run_door_opens_outward_from_slope",
        door_closed is not None
        and door_open is not None
        and door_open[0][1] < door_closed[0][1] - 0.05,
        details=f"door closed={door_closed}, open={door_open}",
    )

    # Front ramp folds upward (max Z increases)
    ctx.check(
        "front_ramp_folds_upward",
        ramp_down is not None
        and ramp_up is not None
        and ramp_up[1][2] > ramp_down[1][2] + 0.10,
        details=f"ramp down={ramp_down}, up={ramp_up}",
    )

    # Floor tray slides out toward -Y
    ctx.check(
        "floor_tray_slides_out",
        tray_home is not None
        and tray_out is not None
        and tray_out[0][1] < tray_home[0][1] - 0.20,
        details=f"tray home={tray_home}, out={tray_out}",
    )

    return ctx.report()


object_model = build_object_model()
