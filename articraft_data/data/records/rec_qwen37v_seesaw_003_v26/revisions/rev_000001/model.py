from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ConeGeometry,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sample_catmull_rom_spline_2d,
    tube_from_spline_points,
)

# ---------------------------------------------------------------------------
# Shared dimensions (meters). World: X along the seesaw length, Z up.
# ---------------------------------------------------------------------------
PIVOT_Z = 0.34          # world height of the rocking axis (inside the bracket)
BEAM_R = 0.06           # main tube radius (~120 mm diameter)
BEAM_HALF = 1.15        # half-length of the curved main tube
CURVE_C = 0.1285        # parabolic curvature of the banana beam
BEAM_CENTER_Z = 0.16    # beam centerline height at x=0, relative to the pivot

COLLAR_X = 0.97         # clamp collar position along the beam
SEAT_CENTER_X = 1.14
SEAT_Z = 0.062          # seat plate mid-plane, relative to the pivot
PLATE_T = 0.012
HANDLE_X = 1.03
HANDLE_Z = 0.552        # handle plate mid-plane, relative to the pivot

ROCK_LIMIT = 0.262      # ~15 degrees each way

PEDESTAL_R = 0.075
PEDESTAL_H = 0.22
BRACKET_SIZE = (0.16, 0.13, 0.17)
BRACKET_CZ = 0.295      # bracket box center height (spans 0.21 .. 0.38)

# Support legs: flat bars radiating from pedestal base to rubber pads
LEG_COUNT = 4
LEG_BAR_LENGTH = 0.14
LEG_BAR_WIDTH = 0.030
LEG_BAR_HEIGHT = 0.018
LEG_OUTER_R = 0.16       # center-to-outer-edge distance for pad
PAD_RADIUS = 0.040
PAD_THICKNESS = 0.010

# Locking pin
PIN_RADIUS = 0.012
PIN_LENGTH = 0.08
PIN_KNOB_RADIUS = 0.022
PIN_KNOB_LENGTH = 0.018
PIN_SLIDE_RANGE = 0.055  # prismatic travel in meters

# Footrest dimensions
FOOTREST_LENGTH = 0.14
FOOTREST_WIDTH = 0.10
FOOTREST_THICKNESS = 0.008
RIB_COUNT = 5
RIB_HEIGHT = 0.006
RIB_WIDTH = 0.008


def _beam_z(x: float) -> float:
    """Beam centerline height (relative to the pivot frame) at station x."""
    return BEAM_CENTER_Z + CURVE_C * x * x


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="playground_seesaw")

    model.material("gloss_red_orange", rgba=(0.88, 0.20, 0.06, 1.0))
    model.material("matte_black", rgba=(0.09, 0.09, 0.10, 1.0))
    model.material("light_gray", rgba=(0.78, 0.78, 0.79, 1.0))
    model.material("dark_gray_steel", rgba=(0.34, 0.36, 0.38, 1.0))
    model.material("silver_rivet", rgba=(0.74, 0.75, 0.78, 1.0))
    model.material("rubber_black", rgba=(0.12, 0.12, 0.11, 1.0))
    model.material("pin_zinc", rgba=(0.72, 0.73, 0.70, 1.0))
    model.material("footrest_gray", rgba=(0.30, 0.32, 0.34, 1.0))

    # -----------------------------------------------------------------
    # Fixed base: short light-gray ground pedestal + black cast bracket
    # + 4 support legs with rubber ground pads.
    # -----------------------------------------------------------------
    base = model.part("pedestal_mount")
    base.visual(
        Cylinder(radius=PEDESTAL_R, length=PEDESTAL_H),
        origin=Origin(xyz=(0.0, 0.0, PEDESTAL_H / 2.0)),
        material="light_gray",
        name="ground_pedestal",
    )
    base.visual(
        Box(BRACKET_SIZE),
        origin=Origin(xyz=(0.0, 0.0, BRACKET_CZ)),
        material="matte_black",
        name="pivot_bracket",
    )
    # Round pivot bosses on both bracket cheeks, with visible bolt heads.
    for i, sy in enumerate((1.0, -1.0)):
        base.visual(
            Cylinder(radius=0.055, length=0.022),
            origin=Origin(xyz=(0.0, sy * 0.0755, PIVOT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="matte_black",
            name=f"pivot_boss_{i}",
        )
        for j, ang in enumerate((0.25, 0.75, 1.25, 1.75)):
            dx = 0.034 * math.cos(ang * math.pi)
            dz = 0.034 * math.sin(ang * math.pi)
            base.visual(
                Cylinder(radius=0.0085, length=0.012),
                origin=Origin(
                    xyz=(dx, sy * 0.0895, PIVOT_Z + dz),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material="silver_rivet",
                name=f"bracket_bolt_{i}_{j}",
            )

    # Support legs radiating from pedestal base, each with a rubber ground pad.
    # Each leg is a flat horizontal bar from the pedestal side outward at ground
    # level; the pad sits directly under the outer end so it contacts the leg.
    leg_z_center = LEG_BAR_HEIGHT / 2.0 + PAD_THICKNESS  # leg bottom rests on pad top
    for i in range(LEG_COUNT):
        angle = (2.0 * math.pi * i) / LEG_COUNT + math.pi / 4.0
        dx = math.cos(angle)
        dy = math.sin(angle)
        # Leg bar center: halfway along its horizontal reach from pedestal edge
        inner_r = PEDESTAL_R - 0.01  # slight overlap with pedestal
        bar_center_r = inner_r + LEG_BAR_LENGTH / 2.0
        leg_cx = dx * bar_center_r
        leg_cy = dy * bar_center_r
        yaw = math.atan2(dy, dx)
        base.visual(
            Box((LEG_BAR_LENGTH, LEG_BAR_WIDTH, LEG_BAR_HEIGHT)),
            origin=Origin(
                xyz=(leg_cx, leg_cy, leg_z_center),
                rpy=(0.0, 0.0, yaw),
            ),
            material="light_gray",
            name=f"support_leg_{i}",
        )
        # Rubber ground pad at the outer end of each leg
        pad_r = inner_r + LEG_BAR_LENGTH * 0.85
        pad_x = dx * pad_r
        pad_y = dy * pad_r
        base.visual(
            Cylinder(radius=PAD_RADIUS, length=PAD_THICKNESS),
            origin=Origin(xyz=(pad_x, pad_y, PAD_THICKNESS / 2.0)),
            material="rubber_black",
            name=f"rubber_pad_{i}",
        )

    # Pin guide bushing on the bracket cheek (where the locking pin slides through)
    base.visual(
        Cylinder(radius=0.025, length=0.030),
        origin=Origin(
            xyz=(0.0, 0.095, PIVOT_Z - 0.04),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material="matte_black",
        name="pin_bushing",
    )

    # -----------------------------------------------------------------
    # Locking pin: prismatic slide near the central bracket.
    # Part frame at the retracted position; pin extends along +Y.
    # -----------------------------------------------------------------
    lock_pin = model.part("locking_pin")

    # Pin shaft
    lock_pin.visual(
        Cylinder(radius=PIN_RADIUS, length=PIN_LENGTH),
        origin=Origin(xyz=(0.0, PIN_LENGTH / 2.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="pin_zinc",
        name="pin_shaft",
    )
    # Knob/handle at the outer end
    lock_pin.visual(
        Cylinder(radius=PIN_KNOB_RADIUS, length=PIN_KNOB_LENGTH),
        origin=Origin(
            xyz=(0.0, PIN_LENGTH + PIN_KNOB_LENGTH / 2.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material="matte_black",
        name="pin_knob",
    )
    # Small retaining ring at the inner end
    lock_pin.visual(
        Cylinder(radius=PIN_RADIUS + 0.005, length=0.005),
        origin=Origin(xyz=(0.0, -0.002, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="pin_zinc",
        name="pin_retainer",
    )

    # -----------------------------------------------------------------
    # Rocker: curved red beam + pivot stub + mirrored seat/handle ends.
    # Part frame sits on the pivot axis; geometry is authored relative
    # to that frame so the revolute joint needs no extra offset.
    # -----------------------------------------------------------------
    rocker = model.part("rocker")

    # Thick glossy banana beam, swept along a shallow parabola.
    n = 12
    beam_pts = []
    for k in range(-n, n + 1):
        x = BEAM_HALF * k / n
        beam_pts.append((x, 0.0, _beam_z(x)))
    rocker.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                beam_pts,
                radius=BEAM_R,
                samples_per_segment=4,
                radial_segments=28,
                cap_ends=True,
            ),
            "beam_tube",
        ),
        material="gloss_red_orange",
        name="beam_tube",
    )

    # Red flare wedge under the beam center, blending into the pivot stub.
    wedge = ConeGeometry(0.085, 0.09, radial_segments=28).rotate_x(math.pi)
    wedge.translate(0.0, 0.0, 0.110)
    rocker.visual(
        mesh_from_geometry(wedge, "pivot_wedge"),
        material="gloss_red_orange",
        name="pivot_wedge",
    )

    # Short red stub descending from the beam into the black bracket.
    rocker.visual(
        Cylinder(radius=0.048, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material="gloss_red_orange",
        name="pivot_stub",
    )

    # Lock hole indicator on the stub (where the pin engages)
    rocker.visual(
        Cylinder(radius=PIN_RADIUS + 0.003, length=0.020),
        origin=Origin(
            xyz=(0.0, 0.0, -0.04),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material="matte_black",
        name="lock_hole",
    )

    seat_profile = sample_catmull_rom_spline_2d(
        [
            (0.21, 0.0),
            (0.05, 0.115),
            (-0.10, 0.145),
            (-0.185, 0.10),
            (-0.21, 0.0),
            (-0.185, -0.10),
            (-0.10, -0.145),
            (0.05, -0.115),
        ],
        samples_per_segment=8,
        closed=True,
    )
    grip_outer = rounded_rect_profile(0.18, 0.30, 0.05)
    grip_hole = rounded_rect_profile(0.06, 0.09, 0.02)
    grip_holes = [
        [(hx, hy + 0.075) for hx, hy in grip_hole],
        [(hx, hy - 0.075) for hx, hy in grip_hole],
    ]

    collar_z = _beam_z(COLLAR_X)
    slope = 2.0 * CURVE_C * COLLAR_X
    tangent = math.atan(slope)

    for i, s in enumerate((1.0, -1.0)):
        # Black clamp collar ring around the beam, aligned to the local tangent.
        rocker.visual(
            Cylinder(radius=0.080, length=0.085),
            origin=Origin(
                xyz=(s * COLLAR_X, 0.0, collar_z),
                rpy=(0.0, math.pi / 2.0 - s * tangent, 0.0),
            ),
            material="matte_black",
            name=f"clamp_collar_{i}",
        )
        for j, sy in enumerate((1.0, -1.0)):
            rocker.visual(
                Cylinder(radius=0.011, length=0.032),
                origin=Origin(
                    xyz=(s * COLLAR_X, sy * 0.082, collar_z),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material="silver_rivet",
                name=f"collar_bolt_{i}_{j}",
            )

        # Thin red tube branching downward-outboard from the collar to the seat.
        drop_pts = [
            (s * COLLAR_X, 0.0, collar_z),
            (s * 1.05, 0.0, 0.185),
            (s * 1.12, 0.0, 0.105),
            (s * 1.15, 0.0, 0.066),
        ]
        rocker.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    drop_pts, radius=0.026, samples_per_segment=10, radial_segments=18
                ),
                f"drop_tube_{i}",
            ),
            material="gloss_red_orange",
            name=f"drop_tube_{i}",
        )

        # Flat dark-gray rounded-triangular seat plate with rivets.
        seat = ExtrudeGeometry(seat_profile, PLATE_T, cap=True, center=True)
        if s < 0:
            seat.rotate_z(math.pi)
        seat.translate(s * SEAT_CENTER_X, 0.0, SEAT_Z)
        rocker.visual(
            mesh_from_geometry(seat, f"seat_plate_{i}"),
            material="dark_gray_steel",
            name=f"seat_plate_{i}",
        )
        rivet_xy = [(0.13, 0.0), (0.0, 0.10), (0.0, -0.10), (-0.13, 0.075), (-0.13, -0.075)]
        for j, (lx, ly) in enumerate(rivet_xy):
            rocker.visual(
                Cylinder(radius=0.008, length=0.010),
                origin=Origin(xyz=(s * (SEAT_CENTER_X + lx), ly, 0.070)),
                material="silver_rivet",
                name=f"seat_rivet_{i}_{j}",
            )
        # Small black stop fin under the seat nose (as in the reference photo).
        rocker.visual(
            Box((0.045, 0.022, 0.04)),
            origin=Origin(xyz=(s * 1.26, 0.0, 0.038)),
            material="matte_black",
            name=f"seat_fin_{i}",
        )

        # Textured footrest plate near each seat, with raised grip ribs.
        # Placed between collar and seat so it doesn't extend past the beam tips.
        footrest_x = s * (COLLAR_X + 0.12)
        footrest_z = SEAT_Z + 0.005
        # Base plate
        rocker.visual(
            Box((FOOTREST_LENGTH, FOOTREST_WIDTH, FOOTREST_THICKNESS)),
            origin=Origin(xyz=(footrest_x, 0.0, footrest_z)),
            material="footrest_gray",
            name=f"footrest_plate_{i}",
        )
        # Raised anti-slip ribs across the footrest
        for r in range(RIB_COUNT):
            rib_x_offset = -FOOTREST_LENGTH / 2.0 + FOOTREST_LENGTH * (r + 0.5) / RIB_COUNT
            rocker.visual(
                Box((RIB_WIDTH, FOOTREST_WIDTH * 0.85, RIB_HEIGHT)),
                origin=Origin(
                    xyz=(footrest_x + rib_x_offset, 0.0, footrest_z + FOOTREST_THICKNESS / 2.0 + RIB_HEIGHT / 2.0),
                ),
                material="matte_black",
                name=f"footrest_rib_{i}_{r}",
            )
        # Footrest mounting bracket connecting footrest to the drop tube area
        bracket_z_mid = (footrest_z + collar_z) / 2.0
        rocker.visual(
            Box((0.022, 0.030, collar_z - footrest_z)),
            origin=Origin(xyz=(footrest_x, 0.0, bracket_z_mid)),
            material="dark_gray_steel",
            name=f"footrest_bracket_{i}",
        )

        # Thin red post rising from the beam to the gray handlebar grip plate.
        post_pts = [
            (s * COLLAR_X, 0.0, 0.285),
            (s * 0.985, 0.0, 0.40),
            (s * 1.01, 0.0, 0.48),
            (s * HANDLE_X, 0.0, 0.550),
        ]
        rocker.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    post_pts, radius=0.021, samples_per_segment=10, radial_segments=18
                ),
                f"handle_post_{i}",
            ),
            material="gloss_red_orange",
            name=f"handle_post_{i}",
        )
        grip = ExtrudeWithHolesGeometry(grip_outer, grip_holes, PLATE_T, cap=True, center=True)
        grip.translate(s * HANDLE_X, 0.0, HANDLE_Z)
        rocker.visual(
            mesh_from_geometry(grip, f"handle_plate_{i}"),
            material="dark_gray_steel",
            name=f"handle_plate_{i}",
        )

    # Single rocking pivot: horizontal axis across the seesaw length.
    model.articulation(
        "rocker_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=rocker,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=400.0, velocity=1.5, lower=-ROCK_LIMIT, upper=ROCK_LIMIT
        ),
    )

    # Locking pin: prismatic slide through the bracket bushing.
    # At q=0 the pin is retracted (flush with bushing outer face).
    # Positive q slides it outward along +Y to disengage from the rocker stub.
    model.articulation(
        "lock_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=lock_pin,
        origin=Origin(xyz=(0.0, 0.095, PIVOT_Z - 0.04)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=50.0, velocity=0.3, lower=0.0, upper=PIN_SLIDE_RANGE
        ),
    )

    return model


def _intersects(a, b, tol: float = 1e-4) -> bool:
    if a is None or b is None:
        return False
    return all(a[0][i] <= b[1][i] + tol and b[0][i] <= a[1][i] + tol for i in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("pedestal_mount")
    rocker = object_model.get_part("rocker")
    lock_pin = object_model.get_part("locking_pin")
    pivot = object_model.get_articulation("rocker_pivot")
    lock_slide = object_model.get_articulation("lock_slide")

    # The red pivot stub is intentionally captured inside the black bracket.
    ctx.allow_overlap(
        rocker,
        base,
        elem_a="pivot_stub",
        elem_b="pivot_bracket",
        reason="The red center stub descends into the cast pivot bracket that captures the rocking axle.",
    )
    ctx.expect_overlap(
        rocker,
        base,
        axes="z",
        elem_a="pivot_stub",
        elem_b="pivot_bracket",
        min_overlap=0.04,
        name="pivot stub inserted into bracket",
    )
    ctx.expect_within(
        rocker,
        base,
        axes="xy",
        inner_elem="pivot_stub",
        outer_elem="pivot_bracket",
        margin=0.0,
        name="pivot stub centered in bracket",
    )

    # Locking pin shaft is intentionally nested inside the pin bushing.
    ctx.allow_overlap(
        lock_pin,
        base,
        elem_a="pin_shaft",
        elem_b="pin_bushing",
        reason="The locking pin shaft slides through the bracket bushing as a captured prismatic member.",
    )
    # Pin retaining ring is captured inside the bushing bore.
    ctx.allow_overlap(
        lock_pin,
        base,
        elem_a="pin_retainer",
        elem_b="pin_bushing",
        reason="The retaining ring sits inside the bushing bore to capture the pin against pull-out.",
    )
    ctx.expect_contact(
        lock_pin,
        base,
        elem_a="pin_shaft",
        elem_b="pin_bushing",
        name="pin shaft passes through bushing",
    )

    # Lock hole indicator on the rocker stub is inside the bracket (the stub
    # is already allowed to overlap the bracket; the lock hole is part of that
    # captured geometry).
    ctx.allow_overlap(
        rocker,
        base,
        elem_a="lock_hole",
        elem_b="pivot_bracket",
        reason="The lock-hole indicator is on the pivot stub which descends into the bracket.",
    )

    # Bracket seated on the pedestal (both visuals on the fixed base part).
    bracket = ctx.part_element_world_aabb(base, elem="pivot_bracket")
    pedestal = ctx.part_element_world_aabb(base, elem="ground_pedestal")
    ctx.check(
        "bracket sits atop ground pedestal",
        _intersects(bracket, pedestal),
        details=f"bracket={bracket}, pedestal={pedestal}",
    )

    # Rubber ground pads exist at ground level under support legs.
    pad0 = ctx.part_element_world_aabb(base, elem="rubber_pad_0")
    pad1 = ctx.part_element_world_aabb(base, elem="rubber_pad_1")
    pad2 = ctx.part_element_world_aabb(base, elem="rubber_pad_2")
    pad3 = ctx.part_element_world_aabb(base, elem="rubber_pad_3")
    ctx.check(
        "rubber ground pads at ground level",
        pad0 is not None and pad1 is not None and pad2 is not None and pad3 is not None
        and pad0[0][2] < 0.02 and pad1[0][2] < 0.02
        and pad2[0][2] < 0.02 and pad3[0][2] < 0.02,
        details=f"pad0={pad0}, pad1={pad1}, pad2={pad2}, pad3={pad3}",
    )
    # Pads spread outward from center (not stacked on pedestal)
    ctx.check(
        "rubber pads spread under support legs",
        pad0 is not None and pad1 is not None and pad2 is not None and pad3 is not None
        and (pad0[1][0] - pad0[0][0]) > 0.02
        and (pad2[1][0] - pad2[0][0]) > 0.02,
        details=f"pads spread from center",
    )

    # Locking pin near the central bracket
    pin_shaft_aabb = ctx.part_element_world_aabb(lock_pin, elem="pin_shaft")
    pin_knob_aabb = ctx.part_element_world_aabb(lock_pin, elem="pin_knob")
    ctx.check(
        "locking pin near central bracket",
        pin_shaft_aabb is not None and pin_knob_aabb is not None
        and abs(pin_shaft_aabb[0][0]) < 0.10
        and abs(pin_shaft_aabb[1][0]) < 0.10,
        details=f"pin_shaft={pin_shaft_aabb}",
    )

    # Locking pin prismatic joint exists with correct range
    lock_lim = lock_slide.motion_limits
    ctx.check(
        "lock slide has prismatic range",
        lock_lim is not None
        and lock_slide.articulation_type == ArticulationType.PRISMATIC
        and abs(lock_lim.lower) < 0.001
        and lock_lim.upper > 0.03,
        details=f"limits=({lock_lim.lower}, {lock_lim.upper}), type={lock_slide.articulation_type}",
    )

    # Locking pin slides outward when actuated
    pin_rest_pos = ctx.part_world_position(lock_pin)
    with ctx.pose({lock_slide: PIN_SLIDE_RANGE}):
        pin_ext_pos = ctx.part_world_position(lock_pin)
        ctx.check(
            "locking pin slides outward along Y",
            pin_rest_pos is not None and pin_ext_pos is not None
            and pin_ext_pos[1] > pin_rest_pos[1] + 0.03,
            details=f"rest={pin_rest_pos}, extended={pin_ext_pos}",
        )

    # Footrests exist near each seat
    fr0 = ctx.part_element_world_aabb(rocker, elem="footrest_plate_0")
    fr1 = ctx.part_element_world_aabb(rocker, elem="footrest_plate_1")
    ctx.check(
        "textured footrests near each seat",
        fr0 is not None and fr1 is not None
        and abs(fr0[0][0]) > 0.9
        and abs(fr1[0][0]) > 0.9,
        details=f"footrest0={fr0}, footrest1={fr1}",
    )
    # Footrests are mirrored about center
    ctx.check(
        "footrests mirrored about pivot",
        fr0 is not None and fr1 is not None
        and abs((fr0[0][0] + fr0[1][0]) / 2.0 + (fr1[0][0] + fr1[1][0]) / 2.0) < 0.05,
        details=f"footrest0={fr0}, footrest1={fr1}",
    )
    # Footrest ribs exist (textured surface)
    rib00 = ctx.part_element_world_aabb(rocker, elem="footrest_rib_0_0")
    rib04 = ctx.part_element_world_aabb(rocker, elem="footrest_rib_0_4")
    ctx.check(
        "footrest has raised texture ribs",
        rib00 is not None and rib04 is not None
        and fr0 is not None
        and rib00[1][2] > fr0[1][2] - 0.001
        and rib04[1][2] > fr0[1][2] - 0.001,
        details=f"rib0={rib00}, rib4={rib04}, plate={fr0}",
    )

    # Hero beam: ~2.6 m long banana tube that dips at center and rises at ends.
    beam = ctx.part_element_world_aabb(rocker, elem="beam_tube")
    ctx.check(
        "beam tube spans the seesaw length",
        beam is not None and (beam[1][0] - beam[0][0]) >= 2.2,
        details=f"beam={beam}",
    )
    ctx.check(
        "beam sweeps upward toward both ends",
        beam is not None and (beam[1][2] - beam[0][2]) >= 0.25,
        details=f"beam z-range={None if beam is None else beam[1][2] - beam[0][2]}",
    )

    # Overall envelope: about 2.6 m long, about 0.9 m tall.
    ra = ctx.part_world_aabb(rocker)
    ba = ctx.part_world_aabb(base)
    ctx.check(
        "overall length about 2.6 m",
        ra is not None and 2.4 <= (ra[1][0] - ra[0][0]) <= 2.8,
        details=f"rocker aabb={ra}",
    )
    ctx.check(
        "overall height about 0.9 m",
        ra is not None and ba is not None and 0.82 <= max(ra[1][2], ba[1][2]) <= 0.98,
        details=f"rocker={ra}, base={ba}",
    )

    # End assemblies: seats hang below the beam, grip plates rise above it.
    seat0 = ctx.part_element_world_aabb(rocker, elem="seat_plate_0")
    seat1 = ctx.part_element_world_aabb(rocker, elem="seat_plate_1")
    grip0 = ctx.part_element_world_aabb(rocker, elem="handle_plate_0")
    grip1 = ctx.part_element_world_aabb(rocker, elem="handle_plate_1")
    ctx.check(
        "seats at sitting height below the beam",
        seat0 is not None
        and seat1 is not None
        and 0.35 <= seat0[1][2] <= 0.46
        and 0.35 <= seat1[1][2] <= 0.46,
        details=f"seat0={seat0}, seat1={seat1}",
    )
    ctx.check(
        "grip plates above the beam ends",
        grip0 is not None
        and grip1 is not None
        and beam is not None
        and grip0[0][2] > beam[1][2]
        and grip1[0][2] > beam[1][2],
        details=f"grip0={grip0}, grip1={grip1}, beam={beam}",
    )

    # The two ends mirror each other across the pivot.
    def _cx(aabb):
        return 0.5 * (aabb[0][0] + aabb[1][0])

    ctx.check(
        "seat assemblies mirrored about the pivot",
        seat0 is not None
        and seat1 is not None
        and _cx(seat0) > 0.9
        and _cx(seat1) < -0.9
        and abs(_cx(seat0) + _cx(seat1)) < 0.02
        and abs(seat0[1][2] - seat1[1][2]) < 0.01,
        details=f"seat0={seat0}, seat1={seat1}",
    )
    ctx.check(
        "grip plates mirrored about the pivot",
        grip0 is not None and grip1 is not None and abs(_cx(grip0) + _cx(grip1)) < 0.02,
        details=f"grip0={grip0}, grip1={grip1}",
    )

    # Mounted, not floating: drop tubes reach seats, posts reach grip plates,
    # clamp collars ring the beam.
    drop0 = ctx.part_element_world_aabb(rocker, elem="drop_tube_0")
    drop1 = ctx.part_element_world_aabb(rocker, elem="drop_tube_1")
    post0 = ctx.part_element_world_aabb(rocker, elem="handle_post_0")
    post1 = ctx.part_element_world_aabb(rocker, elem="handle_post_1")
    collar0 = ctx.part_element_world_aabb(rocker, elem="clamp_collar_0")
    collar1 = ctx.part_element_world_aabb(rocker, elem="clamp_collar_1")
    ctx.check(
        "drop tubes connect beam to seats",
        _intersects(drop0, beam)
        and _intersects(drop0, seat0)
        and _intersects(drop1, beam)
        and _intersects(drop1, seat1),
        details=f"drop0={drop0}, drop1={drop1}",
    )
    ctx.check(
        "handle posts connect beam to grip plates",
        _intersects(post0, beam)
        and _intersects(post0, grip0)
        and _intersects(post1, beam)
        and _intersects(post1, grip1),
        details=f"post0={post0}, post1={post1}",
    )
    ctx.check(
        "clamp collars ring the beam near its ends",
        _intersects(collar0, beam)
        and _intersects(collar1, beam)
        and collar0 is not None
        and collar1 is not None
        and abs(_cx(collar0)) > 0.85
        and abs(_cx(collar1)) > 0.85,
        details=f"collar0={collar0}, collar1={collar1}",
    )

    # Joint limits: about +/- 15 degrees of rocking.
    lim = pivot.motion_limits
    ctx.check(
        "rocking range about +/- 15 degrees",
        lim is not None
        and abs(lim.lower + ROCK_LIMIT) < 0.02
        and abs(lim.upper - ROCK_LIMIT) < 0.02,
        details=f"limits=({lim.lower}, {lim.upper})",
    )

    # Decisive pose checks: the whole rocker tilts as one body; seats swap
    # height, everything clears the ground, the base stays put.
    base_rest = ctx.part_world_aabb(base)
    with ctx.pose({pivot: ROCK_LIMIT}):
        seat0_dn = ctx.part_element_world_aabb(rocker, elem="seat_plate_0")
        seat1_up = ctx.part_element_world_aabb(rocker, elem="seat_plate_1")
        rocker_dn = ctx.part_world_aabb(rocker)
        base_posed = ctx.part_world_aabb(base)
        ctx.check(
            "positive rock lowers seat_0 and raises seat_1",
            seat0_dn is not None
            and seat1_up is not None
            and seat0 is not None
            and seat1 is not None
            and seat0_dn[1][2] < seat0[1][2] - 0.15
            and seat1_up[1][2] > seat1[1][2] + 0.15,
            details=f"seat0_dn={seat0_dn}, seat1_up={seat1_up}",
        )
        ctx.check(
            "rocker clears the ground at full tilt",
            rocker_dn is not None and rocker_dn[0][2] > 0.005,
            details=f"rocker={rocker_dn}",
        )
        ctx.check(
            "pedestal and bracket stay fixed while rocking",
            base_rest is not None and base_posed is not None and _intersects(base_rest, base_posed)
            and abs(base_rest[1][2] - base_posed[1][2]) < 1e-6,
            details=f"rest={base_rest}, posed={base_posed}",
        )
    with ctx.pose({pivot: -ROCK_LIMIT}):
        seat0_up = ctx.part_element_world_aabb(rocker, elem="seat_plate_0")
        rocker_up = ctx.part_world_aabb(rocker)
        ctx.check(
            "negative rock raises seat_0",
            seat0_up is not None and seat0 is not None and seat0_up[0][2] > seat0[0][2] + 0.15,
            details=f"seat0_up={seat0_up}",
        )
        ctx.check(
            "rocker clears the ground at opposite tilt",
            rocker_up is not None and rocker_up[0][2] > 0.005,
            details=f"rocker={rocker_up}",
        )

    return ctx.report()


object_model = build_object_model()
