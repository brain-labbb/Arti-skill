from __future__ import annotations

"""Gray steel single-toggle jaw crusher.

Reference-image driven asset:
- gray welded steel frame (two side plates, tilted front wall, rear wall, base rails)
- orange corrugated fixed jaw plate bolted to the tilted front wall
- orange corrugated movable jaw plate on a swinging pitman hung from the eccentric shaft
- eccentric shaft with a large spoked flywheel (one side) and a V-belt drive pulley (other side)
- rear toggle plate seated between the pitman toggle pocket and the rear frame seat
- tension rod with a coil spring below the toggle plate
- side-mounted electric drive motor with its own small pulley (belt-plane aligned)

Articulations:
- frame_to_swing_jaw        REVOLUTE   (small crushing stroke; positive q opens the jaw)
- frame_to_eccentric_shaft  CONTINUOUS (flywheel / eccentric shaft spin)
- swing_jaw_to_toggle_plate REVOLUTE mimic (-1 x jaw) so the toggle stays level while it
  follows the pitman toggle pocket
- frame_to_motor_pulley     CONTINUOUS mimic (belt ratio x shaft) for the motor pulley
"""

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoxGeometry,
    CylinderGeometry,
    ExtrudeGeometry,
    LatheGeometry,
    MeshGeometry,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)

# ---------------------------------------------------------------------------
# Global layout constants (meters).  +X = rear (motor side), -X = front
# (fixed jaw), +Y = drive-pulley side, -Y = flywheel side, +Z = up.
# ---------------------------------------------------------------------------

SIDE_PLATE_THICK = 0.07
SIDE_PLATE_Y = 0.635  # center of each side plate (inner faces at +-0.60)
SIDE_INNER = SIDE_PLATE_Y - SIDE_PLATE_THICK / 2.0  # 0.60
SIDE_TOP_Z = 1.55

SHAFT_X = 0.25
SHAFT_Z = 1.66
SHAFT_R = 0.095
ECC_BOSS_R = 0.115
HUB_OUTER_R = 0.20
HUB_BORE_R = 0.105
HUB_WIDTH = 1.00

JAW_STROKE = 0.025  # rad, opening swing of the pitman

# Fixed jaw plate centerline: top (-0.661, 1.523) -> bottom (-0.154, 0.367)
FIXED_PLATE_CENTER = (-0.4075, 0.945)
FIXED_PLATE_LEN_HALF = 0.631
FIXED_PLATE_ANGLE = math.atan2(0.505, 1.15)  # ~0.4136 rad lean (top toward -X)
FIXED_PLATE_THICK = 0.09

FRONT_WALL_THICK = 0.10
# front wall mid-plane sits behind the jaw plate with a 4 mm seating embed
FRONT_WALL_SHIFT = FIXED_PLATE_THICK / 2.0 + FRONT_WALL_THICK / 2.0 - 0.004

RIB_R = 0.028

# Toggle geometry (in swing-jaw local frame; jaw local frame at the shaft axis)
TOGGLE_SEAT_LOCAL = (0.26, -1.14)  # (x, z) of the toggle joint on the pitman
TOGGLE_LEN = 0.34  # rocker-to-rocker distance
TOGGLE_ROCKER_R = 0.032
TOGGLE_HALF_W = 0.26

MOTOR_PULLEY_POS = (1.02, 0.85, 0.585)
BELT_RATIO = 3.5  # motor pulley turns ~3.5x per eccentric-shaft turn


def _circle(radius: float, segments: int = 48) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * i / segments),
            radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _box_at(size: tuple[float, float, float], center: tuple[float, float, float]) -> MeshGeometry:
    return BoxGeometry(size).translate(*center)


def _cyl_y(
    radius: float, length: float, center: tuple[float, float, float], segments: int = 32
) -> MeshGeometry:
    """Cylinder with its axis along +Y, centered at `center`."""
    geo = CylinderGeometry(radius, length, radial_segments=segments)
    geo.rotate_x(math.pi / 2.0)
    return geo.translate(*center)


def _side_plate_mesh(y_center: float) -> MeshGeometry:
    profile = [
        (-1.05, 0.115),
        (1.05, 0.115),
        (1.05, 0.85),
        (0.60, SIDE_TOP_Z),
        (-1.05, SIDE_TOP_Z),
    ]
    geo = ExtrudeGeometry(profile, SIDE_PLATE_THICK, cap=True, center=True)
    geo.rotate_x(math.pi / 2.0)  # profile Y -> world Z, thickness along Y
    return geo.translate(0.0, y_center, 0.0)


def _base_frame_mesh() -> MeshGeometry:
    geo = MeshGeometry()
    # longitudinal rails under the side plates
    geo.merge(_box_at((2.10, 0.16, 0.12), (0.0, -SIDE_PLATE_Y, 0.06)))
    geo.merge(_box_at((2.10, 0.16, 0.12), (0.0, SIDE_PLATE_Y, 0.06)))
    # cross members
    geo.merge(_box_at((0.14, 1.28, 0.10), (-0.88, 0.0, 0.05)))
    geo.merge(_box_at((0.14, 1.28, 0.10), (0.88, 0.0, 0.05)))
    # rear-side ground plate that carries the motor pedestal
    geo.merge(_box_at((0.55, 0.90, 0.10), (1.025, 1.05, 0.05)))
    return geo


def _front_wall_mesh() -> MeshGeometry:
    theta = FIXED_PLATE_ANGLE
    n = (math.cos(theta), 0.0, math.sin(theta))
    cx = FIXED_PLATE_CENTER[0] - FRONT_WALL_SHIFT * n[0]
    cz = FIXED_PLATE_CENTER[1] - FRONT_WALL_SHIFT * n[2]
    geo = BoxGeometry((FRONT_WALL_THICK, 2.0 * SIDE_INNER + 0.04, 1.34))
    geo.rotate_y(-theta)
    return geo.translate(cx, 0.0, cz)


def _rear_wall_mesh() -> MeshGeometry:
    # main rear wall between the side plates
    return _box_at((0.10, 1.24, 0.755), (1.0, 0.0, 0.4925))


def _toggle_seat_mesh() -> MeshGeometry:
    # open-front pocket flanges that bracket the toggle rear end
    geo = MeshGeometry()
    geo.merge(_box_at((0.22, 0.56, 0.05), (0.89, 0.0, 0.59)))
    geo.merge(_box_at((0.22, 0.56, 0.045), (0.89, 0.0, 0.4575)))
    return geo


def _rod_bracket_mesh() -> MeshGeometry:
    # tension-rod guide bracket (two lugs flanking the rod path)
    geo = MeshGeometry()
    geo.merge(_box_at((0.16, 0.02, 0.18), (0.92, -0.04, 0.21)))
    geo.merge(_box_at((0.16, 0.02, 0.18), (0.92, 0.04, 0.21)))
    return geo


def _bearing_housings_mesh() -> MeshGeometry:
    geo = MeshGeometry()
    for sign in (-1.0, 1.0):
        yc = sign * SIDE_PLATE_Y
        geo.merge(_box_at((0.34, 0.18, 0.115), (SHAFT_X, yc, 1.6025)))
        geo.merge(_cyl_y(0.13, 0.18, (SHAFT_X, yc, SHAFT_Z)))
    return geo


def _motor_pedestal_mesh() -> MeshGeometry:
    geo = MeshGeometry()
    geo.merge(_box_at((0.36, 0.40, 0.29), (1.02, 1.19, 0.24)))  # pedestal
    geo.merge(_box_at((0.46, 0.56, 0.05), (1.02, 1.19, 0.40)))  # platform
    return geo


def _motor_mesh() -> MeshGeometry:
    x, z = 1.02, 0.585
    geo = MeshGeometry()
    geo.merge(_cyl_y(0.17, 0.50, (x, 1.19, z)))  # body
    for yf in (1.02, 1.10, 1.18, 1.26, 1.34):  # cooling fins
        geo.merge(_cyl_y(0.185, 0.012, (x, yf, z)))
    geo.merge(_cyl_y(0.145, 0.04, (x, 0.94, z)))  # front end cap
    geo.merge(_cyl_y(0.145, 0.04, (x, 1.44, z)))  # rear end cap
    geo.merge(_box_at((0.14, 0.16, 0.10), (x, 1.19, 0.78)))  # junction box
    geo.merge(_cyl_y(0.028, 0.07, (x, 0.915, z), segments=20))  # shaft stub
    return geo


def _fixed_jaw_plate_mesh() -> MeshGeometry:
    geo = MeshGeometry()
    geo.merge(BoxGeometry((FIXED_PLATE_THICK, 1.16, 2.0 * FIXED_PLATE_LEN_HALF)))
    # vertical corrugation ribs on the crushing face (local +X side)
    for i in range(9):
        yk = -0.52 + i * 0.13
        rib = CylinderGeometry(RIB_R, 1.20, radial_segments=20)
        rib.translate(FIXED_PLATE_THICK / 2.0, yk, 0.0)
        geo.merge(rib)
    geo.rotate_y(-FIXED_PLATE_ANGLE)
    return geo.translate(FIXED_PLATE_CENTER[0], 0.0, FIXED_PLATE_CENTER[1])


def _pitman_body_mesh() -> MeshGeometry:
    # side profile in jaw-local XZ (origin at the eccentric shaft axis), CCW
    profile = [
        (-0.185, -1.20),
        (-0.10, -1.30),
        (0.10, -1.30),
        (0.24, -1.18),
        (0.30, -0.78),
        (0.26, -0.40),
        (0.16, -0.14),
        (-0.185, -0.14),
    ]
    geo = ExtrudeGeometry(profile, 1.08, cap=True, center=True)
    geo.rotate_x(math.pi / 2.0)
    # toggle seat block on the lower rear face of the pitman
    geo.merge(
        _box_at(
            (0.085, 2.0 * (TOGGLE_HALF_W + 0.01), 0.12),
            (0.2425, 0.0, TOGGLE_SEAT_LOCAL[1]),
        )
    )
    return geo


def _pitman_hub_mesh() -> MeshGeometry:
    # true hollow bearing hub: revolved shell with an open bore for the shaft
    geo = LatheGeometry.from_shell_profiles(
        [(HUB_OUTER_R, -HUB_WIDTH / 2.0), (HUB_OUTER_R, HUB_WIDTH / 2.0)],
        [(HUB_BORE_R, -HUB_WIDTH / 2.0), (HUB_BORE_R, HUB_WIDTH / 2.0)],
        segments=48,
    )
    geo.rotate_x(math.pi / 2.0)
    return geo


def _movable_jaw_plate_mesh() -> MeshGeometry:
    geo = MeshGeometry()
    geo.merge(_box_at((0.07, 1.08, 1.22), (-0.21, 0.0, -0.63)))
    for i in range(9):
        yk = -0.48 + i * 0.12
        rib = CylinderGeometry(RIB_R, 1.18, radial_segments=20)
        rib.translate(-0.245, yk, -0.63)
        geo.merge(rib)
    return geo


def _tension_rod_mesh() -> MeshGeometry:
    """Tension rod + coil spring, in swing-jaw local frame."""
    start = (0.14, -1.24)  # anchored inside the pitman bottom
    direction = (0.9246, -0.3807)  # unit vector in local XZ
    length = 0.46
    theta = math.atan2(direction[0], direction[1])  # rotate +Z onto direction

    geo = MeshGeometry()
    # anchor lug embedded in the pitman bottom
    geo.merge(_box_at((0.10, 0.10, 0.10), (start[0], 0.0, start[1])))
    rod = CylinderGeometry(0.02, length, radial_segments=16)
    rod.translate(0.0, 0.0, length / 2.0)
    rod.rotate_y(theta)
    rod.translate(start[0], 0.0, start[1])
    geo.merge(rod)
    # coil spring rendered as a stack of tori around the rod
    for i in range(6):
        d = 0.23 + i * 0.028
        coil = TorusGeometry(0.034, 0.016, radial_segments=10, tubular_segments=24)
        coil.rotate_y(theta)
        coil.translate(start[0] + direction[0] * d, 0.0, start[1] + direction[1] * d)
        geo.merge(coil)
    # spring stop washer near the rod end
    washer = CylinderGeometry(0.055, 0.02, radial_segments=24)
    washer.rotate_y(theta)
    washer.translate(start[0] + direction[0] * 0.41, 0.0, start[1] + direction[1] * 0.41)
    geo.merge(washer)
    return geo


def _flywheel_mesh() -> MeshGeometry:
    """Spoked flywheel, built about the shaft axis then moved to y=-0.95."""
    geo = MeshGeometry()
    geo.merge(
        LatheGeometry.from_shell_profiles(
            [(0.55, -0.08), (0.55, 0.08)],
            [(0.44, -0.08), (0.44, 0.08)],
            segments=56,
        )
    )
    for i in range(6):
        spoke = BoxGeometry((0.38, 0.075, 0.09))
        spoke.translate(0.28, 0.0, 0.0)
        spoke.rotate_z(i * math.pi / 3.0)
        geo.merge(spoke)
    geo.merge(CylinderGeometry(0.14, 0.20, radial_segments=32))
    geo.rotate_x(math.pi / 2.0)
    return geo.translate(0.0, -0.95, 0.0)


def _drive_pulley_mesh() -> MeshGeometry:
    geo = MeshGeometry()
    geo.merge(CylinderGeometry(0.34, 0.14, radial_segments=48))  # core
    for zo in (-0.0575, 0.0575):  # V-belt rim cheeks
        cheek = CylinderGeometry(0.40, 0.025, radial_segments=48)
        cheek.translate(0.0, 0.0, zo)
        geo.merge(cheek)
    geo.merge(CylinderGeometry(0.12, 0.16, radial_segments=32))  # hub
    geo.rotate_x(math.pi / 2.0)
    return geo.translate(0.0, 0.85, 0.0)


def _eccentric_shaft_mesh() -> MeshGeometry:
    return _cyl_y(SHAFT_R, 2.10, (0.0, -0.05, 0.0))


def _eccentric_boss_mesh() -> MeshGeometry:
    return _cyl_y(ECC_BOSS_R, 0.88, (0.0, 0.0, 0.0))


def _toggle_plate_mesh() -> MeshGeometry:
    geo = MeshGeometry()
    geo.merge(_box_at((0.32, 2.0 * TOGGLE_HALF_W, 0.056), (TOGGLE_LEN / 2.0, 0.0, 0.0)))
    for xr in (0.0, TOGGLE_LEN):
        rocker = CylinderGeometry(TOGGLE_ROCKER_R, 0.50, radial_segments=20)
        rocker.rotate_x(math.pi / 2.0)
        rocker.translate(xr, 0.0, 0.0)
        geo.merge(rocker)
    return geo


def _motor_pulley_mesh() -> MeshGeometry:
    geo = MeshGeometry()
    geo.merge(CylinderGeometry(0.115, 0.10, radial_segments=32))
    for zo in (-0.045, 0.045):
        cheek = CylinderGeometry(0.135, 0.02, radial_segments=32)
        cheek.translate(0.0, 0.0, zo)
        geo.merge(cheek)
    geo.rotate_x(math.pi / 2.0)
    return geo


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_toggle_jaw_crusher")

    frame_gray = model.material("frame_gray", rgba=(0.58, 0.60, 0.63, 1.0))
    steel_light = model.material("steel_light", rgba=(0.74, 0.76, 0.78, 1.0))
    jaw_orange = model.material("jaw_orange", rgba=(0.86, 0.32, 0.10, 1.0))
    wheel_gray = model.material("wheel_gray", rgba=(0.42, 0.44, 0.47, 1.0))
    shaft_steel = model.material("shaft_steel", rgba=(0.55, 0.56, 0.58, 1.0))
    motor_gray = model.material("motor_gray", rgba=(0.62, 0.64, 0.67, 1.0))
    toggle_steel = model.material("toggle_steel", rgba=(0.50, 0.52, 0.55, 1.0))

    # ------------------------------------------------------------------ frame
    frame = model.part("frame")
    frame.visual(
        mesh_from_geometry(_base_frame_mesh(), "base_frame"),
        material=frame_gray,
        name="base_frame",
    )
    frame.visual(
        mesh_from_geometry(_side_plate_mesh(-SIDE_PLATE_Y), "flywheel_side_plate"),
        material=frame_gray,
        name="flywheel_side_plate",
    )
    frame.visual(
        mesh_from_geometry(_side_plate_mesh(SIDE_PLATE_Y), "drive_side_plate"),
        material=frame_gray,
        name="drive_side_plate",
    )
    frame.visual(
        mesh_from_geometry(_front_wall_mesh(), "front_wall"),
        material=frame_gray,
        name="front_wall",
    )
    frame.visual(
        mesh_from_geometry(_rear_wall_mesh(), "rear_wall"),
        material=frame_gray,
        name="rear_wall",
    )
    frame.visual(
        mesh_from_geometry(_toggle_seat_mesh(), "toggle_seat"),
        material=frame_gray,
        name="toggle_seat",
    )
    frame.visual(
        mesh_from_geometry(_rod_bracket_mesh(), "rod_bracket"),
        material=frame_gray,
        name="rod_bracket",
    )
    frame.visual(
        mesh_from_geometry(_bearing_housings_mesh(), "bearing_housings"),
        material=steel_light,
        name="bearing_housings",
    )
    frame.visual(
        mesh_from_geometry(_motor_pedestal_mesh(), "motor_pedestal"),
        material=frame_gray,
        name="motor_pedestal",
    )
    frame.visual(
        mesh_from_geometry(_motor_mesh(), "motor_assembly"),
        material=motor_gray,
        name="motor_assembly",
    )

    # -------------------------------------------------------- fixed jaw plate
    fixed_plate = model.part("fixed_jaw_plate")
    fixed_plate.visual(
        mesh_from_geometry(_fixed_jaw_plate_mesh(), "fixed_plate_corrugated"),
        material=jaw_orange,
        name="fixed_plate_corrugated",
    )
    model.articulation(
        "frame_to_fixed_jaw_plate",
        ArticulationType.FIXED,
        parent=frame,
        child=fixed_plate,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # -------------------------------------------------------------- swing jaw
    swing_jaw = model.part("swing_jaw")
    swing_jaw.visual(
        mesh_from_geometry(_pitman_body_mesh(), "pitman_body"),
        material=frame_gray,
        name="pitman_body",
    )
    swing_jaw.visual(
        mesh_from_geometry(_pitman_hub_mesh(), "pitman_hub"),
        material=steel_light,
        name="pitman_hub",
    )
    swing_jaw.visual(
        mesh_from_geometry(_movable_jaw_plate_mesh(), "movable_plate_corrugated"),
        material=jaw_orange,
        name="movable_plate_corrugated",
    )
    swing_jaw.visual(
        mesh_from_geometry(_tension_rod_mesh(), "tension_rod_spring"),
        material=toggle_steel,
        name="tension_rod_spring",
    )
    model.articulation(
        "frame_to_swing_jaw",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=swing_jaw,
        origin=Origin(xyz=(SHAFT_X, 0.0, SHAFT_Z)),
        # Pitman hangs along local -Z; -Y makes positive q swing the jaw
        # bottom toward +X (away from the fixed jaw = opening stroke).
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=60000.0, velocity=2.0, lower=0.0, upper=JAW_STROKE),
    )

    # -------------------------------------------------------- eccentric shaft
    shaft = model.part("eccentric_shaft")
    shaft.visual(
        mesh_from_geometry(_eccentric_shaft_mesh(), "shaft"),
        material=shaft_steel,
        name="shaft",
    )
    shaft.visual(
        mesh_from_geometry(_eccentric_boss_mesh(), "eccentric_boss"),
        material=shaft_steel,
        name="eccentric_boss",
    )
    shaft.visual(
        mesh_from_geometry(_flywheel_mesh(), "flywheel"),
        material=wheel_gray,
        name="flywheel",
    )
    shaft.visual(
        mesh_from_geometry(_drive_pulley_mesh(), "drive_pulley"),
        material=wheel_gray,
        name="drive_pulley",
    )
    model.articulation(
        "frame_to_eccentric_shaft",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=shaft,
        origin=Origin(xyz=(SHAFT_X, 0.0, SHAFT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=30000.0, velocity=35.0),
    )

    # ----------------------------------------------------------- toggle plate
    toggle = model.part("toggle_plate")
    toggle.visual(
        mesh_from_geometry(_toggle_plate_mesh(), "toggle_body"),
        material=toggle_steel,
        name="toggle_body",
    )
    model.articulation(
        "swing_jaw_to_toggle_plate",
        ArticulationType.REVOLUTE,
        parent=swing_jaw,
        child=toggle,
        origin=Origin(xyz=(TOGGLE_SEAT_LOCAL[0], 0.0, TOGGLE_SEAT_LOCAL[1])),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8000.0, velocity=2.0, lower=-0.06, upper=0.06),
        mimic=Mimic(joint="frame_to_swing_jaw", multiplier=-1.0),
    )

    # ----------------------------------------------------------- motor pulley
    motor_pulley = model.part("motor_pulley")
    motor_pulley.visual(
        mesh_from_geometry(_motor_pulley_mesh(), "pulley_body"),
        material=wheel_gray,
        name="pulley_body",
    )
    model.articulation(
        "frame_to_motor_pulley",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=motor_pulley,
        origin=Origin(xyz=MOTOR_PULLEY_POS),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4000.0, velocity=120.0),
        mimic=Mimic(joint="frame_to_eccentric_shaft", multiplier=BELT_RATIO),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    fixed_plate = object_model.get_part("fixed_jaw_plate")
    swing_jaw = object_model.get_part("swing_jaw")
    shaft = object_model.get_part("eccentric_shaft")
    toggle = object_model.get_part("toggle_plate")
    motor_pulley = object_model.get_part("motor_pulley")
    jaw_joint = object_model.get_articulation("frame_to_swing_jaw")

    # ---------------------------------------------------- intentional overlaps
    ctx.allow_overlap(
        "eccentric_shaft",
        "frame",
        elem_a="shaft",
        elem_b="bearing_housings",
        reason="Eccentric shaft is intentionally captured inside the frame pillow-block bearings.",
    )
    ctx.allow_overlap(
        "swing_jaw",
        "eccentric_shaft",
        elem_a="pitman_hub",
        elem_b="eccentric_boss",
        reason="Pitman hub is pressed around the eccentric boss; both rotate about the same axis.",
    )
    ctx.allow_overlap(
        "fixed_jaw_plate",
        "frame",
        elem_a="fixed_plate_corrugated",
        elem_b="front_wall",
        reason="Fixed jaw plate is bolted flat against the tilted front wall (4 mm seating embed).",
    )
    ctx.allow_overlap(
        "toggle_plate",
        "swing_jaw",
        elem_a="toggle_body",
        elem_b="pitman_body",
        reason="Toggle front rocker seats in the pitman toggle pocket (captured rocker end).",
    )
    ctx.allow_overlap(
        "motor_pulley",
        "frame",
        elem_a="pulley_body",
        elem_b="motor_assembly",
        reason="Motor pulley is pressed onto the motor shaft stub; both are coaxial.",
    )

    # -------------------------------------------------------- support contacts
    ctx.expect_contact(
        shaft,
        frame,
        elem_a="shaft",
        elem_b="bearing_housings",
        name="shaft seats in the frame bearings",
    )
    ctx.expect_contact(
        swing_jaw,
        shaft,
        elem_a="pitman_hub",
        elem_b="eccentric_boss",
        name="pitman hub hangs on the eccentric boss",
    )
    ctx.expect_contact(
        fixed_plate,
        frame,
        elem_a="fixed_plate_corrugated",
        elem_b="front_wall",
        name="fixed jaw plate seats on the front wall",
    )
    ctx.expect_contact(
        toggle,
        swing_jaw,
        elem_a="toggle_body",
        elem_b="pitman_body",
        name="toggle rocker seats in the pitman pocket",
    )
    ctx.expect_contact(
        motor_pulley,
        frame,
        elem_a="pulley_body",
        elem_b="motor_assembly",
        name="motor pulley is mounted on the motor shaft stub",
    )

    # ----------------------------------------------------- crushing chamber fit
    ctx.expect_gap(
        swing_jaw,
        fixed_plate,
        axis="x",
        min_gap=0.03,
        max_gap=0.12,
        positive_elem="movable_plate_corrugated",
        negative_elem="fixed_plate_corrugated",
        name="closed discharge gap between the jaw plates",
    )
    ctx.expect_within(
        swing_jaw,
        frame,
        axes="y",
        margin=0.0,
        name="pitman assembly stays between the side plates",
    )
    ctx.expect_within(
        toggle,
        frame,
        axes="y",
        inner_elem="toggle_body",
        outer_elem="rear_wall",
        margin=0.0,
        name="toggle plate stays inside the rear seat width",
    )
    ctx.expect_overlap(
        toggle,
        frame,
        axes="x",
        elem_a="toggle_body",
        elem_b="toggle_seat",
        min_overlap=0.02,
        name="toggle rear end reaches into the rear seat pocket",
    )

    # flywheel on the -Y side, drive pulley outboard on the +Y side
    fw = ctx.part_element_world_aabb(shaft, elem="flywheel")
    dp = ctx.part_element_world_aabb(shaft, elem="drive_pulley")
    ctx.check(
        "flywheel sits outboard on the flywheel side",
        fw is not None and fw[1][1] < -SIDE_PLATE_Y and (fw[1][0] - fw[0][0]) > 1.0,
        details=f"flywheel aabb={fw}",
    )
    ctx.check(
        "drive pulley sits outboard on the drive side",
        dp is not None and dp[0][1] > SIDE_PLATE_Y,
        details=f"drive pulley aabb={dp}",
    )
    # motor pulley shares the drive-pulley belt plane
    mp = ctx.part_element_world_aabb(motor_pulley, elem="pulley_body")
    ctx.check(
        "motor pulley is aligned with the drive pulley belt plane",
        mp is not None
        and dp is not None
        and abs((mp[0][1] + mp[1][1]) / 2.0 - (dp[0][1] + dp[1][1]) / 2.0) < 0.02,
        details=f"motor pulley aabb={mp}",
    )

    # ------------------------------------------------------- decisive open pose
    toggle_rest = ctx.part_world_position(toggle)
    with ctx.pose({jaw_joint: JAW_STROKE}):
        toggle_open = ctx.part_world_position(toggle)
        ctx.expect_gap(
            swing_jaw,
            fixed_plate,
            axis="x",
            min_gap=0.05,
            positive_elem="movable_plate_corrugated",
            negative_elem="fixed_plate_corrugated",
            name="discharge gap widens at the open stroke",
        )
        ctx.expect_gap(
            frame,
            toggle,
            axis="x",
            min_gap=0.005,
            positive_elem="rear_wall",
            negative_elem="toggle_body",
            name="toggle still clears the rear wall face at full stroke",
        )
    ctx.check(
        "positive jaw stroke swings the pitman bottom rearward",
        toggle_rest is not None
        and toggle_open is not None
        and toggle_open[0] > toggle_rest[0] + 0.015,
        details=f"toggle rest={toggle_rest}, open={toggle_open}",
    )

    return ctx.report()


object_model = build_object_model()
