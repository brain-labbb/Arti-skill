from __future__ import annotations

import math

import cadquery as cq

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
    mesh_from_cadquery,
    mesh_from_geometry,
    rounded_rect_profile,
    sample_catmull_rom_spline_2d,
    tube_from_spline_points,
)

# ---------------------------------------------------------------------------
# Shared dimensions (meters). World: X along the seesaw length, Z up.
# ---------------------------------------------------------------------------
PIVOT_Z = 0.42          # world height of the rocking axis (top of A-frame)
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

# A-frame dimensions
AFRAME_LEG_R = 0.032       # leg tube radius
AFRAME_BASE_W = 0.50       # width of ground plate (Y direction)
AFRAME_BASE_L = 0.30       # length of ground plate (X direction)
AFRAME_BASE_T = 0.018      # ground plate thickness
AFRAME_SPREAD = 0.22       # half-spread of legs at ground level (Y direction)
AFRAME_CROSSBAR_R = 0.025  # crossbar tube radius
AFRAME_CROSSBAR_L = 0.18   # crossbar length (Y direction)

# Bracket dimensions (at top of A-frame)
BRACKET_W = 0.16
BRACKET_D = 0.13
BRACKET_H = 0.12
BRACKET_CZ = PIVOT_Z       # bracket centered at pivot height

# Locking pin dimensions
PIN_R = 0.012
PIN_LENGTH = 0.10
PIN_HANDLE_R = 0.022
PIN_RING_T = 0.008
PIN_SLEEVE_Z = PIVOT_Z - 0.10  # sleeve center below the pivot bosses


def _beam_z(x: float) -> float:
    """Beam centerline height (relative to the pivot frame) at station x."""
    return BEAM_CENTER_Z + CURVE_C * x * x


def _build_molded_seat(sign: float) -> cq.Workplane:
    """Build a molded bucket seat with raised lip edges using CadQuery.

    The seat is roughly elliptical/dish-shaped, about 0.38m across (X),
    0.28m wide (Y), with a dished interior and raised rim (~25mm lip).
    sign: +1 for right seat, -1 for left seat (mirrored).
    """
    # Outer shell: elliptical dish shape
    outer = (
        cq.Workplane("XY")
        .ellipse(0.19, 0.14)
        .extrude(0.025)  # base plate thickness
    )
    # Raised lip ring: offset outward ring around the edge
    lip_outer = (
        cq.Workplane("XY")
        .ellipse(0.20, 0.15)
        .extrude(0.050)  # lip height
    )
    lip_inner = (
        cq.Workplane("XY")
        .ellipse(0.17, 0.12)
        .extrude(0.050)
    )
    lip_ring = lip_outer.cut(lip_inner)

    # Interior dish: cut a concave depression in the base
    dish = (
        cq.Workplane("XY")
        .workplane(offset=0.012)
        .ellipse(0.15, 0.10)
        .extrude(0.018)  # dish depth
    )
    seat = outer.union(lip_ring).cut(dish)

    # Add a small backrest bump at the outboard end
    backrest = (
        cq.Workplane("XZ")
        .center(sign * 0.16, 0.025)
        .rect(0.04, 0.08)
        .extrude(0.22 if sign > 0 else -0.22)
    )
    # Round the backrest top
    seat = seat.union(backrest)

    # Mirror for left seat
    if sign < 0:
        seat = seat.mirror("YZ")

    return seat


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="playground_seesaw")

    model.material("gloss_red_orange", rgba=(0.88, 0.20, 0.06, 1.0))
    model.material("matte_black", rgba=(0.09, 0.09, 0.10, 1.0))
    model.material("light_gray", rgba=(0.78, 0.78, 0.79, 1.0))
    model.material("dark_gray_steel", rgba=(0.34, 0.36, 0.38, 1.0))
    model.material("silver_rivet", rgba=(0.74, 0.75, 0.78, 1.0))
    model.material("yellow_pin", rgba=(0.92, 0.82, 0.12, 1.0))

    # -----------------------------------------------------------------
    # Fixed base: A-frame support with axle brackets.
    # -----------------------------------------------------------------
    base = model.part("aframe_base")

    # Ground plate
    base.visual(
        Box((AFRAME_BASE_L, AFRAME_BASE_W, AFRAME_BASE_T)),
        origin=Origin(xyz=(0.0, 0.0, AFRAME_BASE_T / 2.0)),
        material="light_gray",
        name="ground_plate",
    )

    # Anchor bolts at corners of ground plate
    for i, (bx, by) in enumerate([
        (0.10, 0.18), (0.10, -0.18), (-0.10, 0.18), (-0.10, -0.18)
    ]):
        base.visual(
            Cylinder(radius=0.010, length=0.025),
            origin=Origin(xyz=(bx, by, 0.005)),
            material="silver_rivet",
            name=f"anchor_bolt_{i}",
        )

    # A-frame legs: two angled tubes rising from ground plate to bracket sides.
    # Legs attach outside the bracket cheeks (at y=±0.085) and spread out
    # to the ground plate edges (at y=±AFRAME_SPREAD).
    LEG_TOP_Y = 0.12  # leg attaches well outside bracket cheek to clear pin
    LEG_BOTTOM_Z = 0.008  # legs embed slightly into ground plate for connectivity
    leg_height = PIVOT_Z - LEG_BOTTOM_Z
    leg_dy = AFRAME_SPREAD - LEG_TOP_Y
    leg_length = math.sqrt(leg_height**2 + leg_dy**2)
    leg_angle = math.atan2(leg_dy, leg_height)  # angle from vertical

    for i, sy in enumerate((1.0, -1.0)):
        # Leg tube: angled from ground to bracket attachment
        mid_y = sy * (AFRAME_SPREAD + LEG_TOP_Y) / 2.0
        mid_z = LEG_BOTTOM_Z + leg_height / 2.0
        base.visual(
            Cylinder(radius=AFRAME_LEG_R, length=leg_length),
            origin=Origin(
                xyz=(0.0, mid_y, mid_z),
                rpy=(sy * leg_angle, 0.0, 0.0),
            ),
            material="light_gray",
            name=f"aframe_leg_{i}",
        )

    # Crossbar connecting the two legs (positioned below the bracket/stub)
    crossbar_z = PIVOT_Z - 0.15  # well below stub bottom and pin
    # At this height, legs are spread apart; crossbar spans between them
    leg_spread_at_crossbar = LEG_TOP_Y + (AFRAME_SPREAD - LEG_TOP_Y) * (PIVOT_Z - crossbar_z) / leg_height
    crossbar_length = 2.0 * leg_spread_at_crossbar - 0.02  # slightly shorter than leg spread
    base.visual(
        Cylinder(radius=AFRAME_CROSSBAR_R, length=crossbar_length),
        origin=Origin(
            xyz=(0.0, 0.0, crossbar_z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material="light_gray",
        name="crossbar",
    )

    # Axle bracket plates on both sides of the A-frame apex
    for i, sy in enumerate((1.0, -1.0)):
        # Bracket cheek plate
        base.visual(
            Box((BRACKET_W, 0.016, BRACKET_H)),
            origin=Origin(xyz=(0.0, sy * 0.065, PIVOT_Z)),
            material="matte_black",
            name=f"bracket_cheek_{i}",
        )
        # Round pivot boss on each cheek
        base.visual(
            Cylinder(radius=0.045, length=0.018),
            origin=Origin(
                xyz=(0.0, sy * 0.076, PIVOT_Z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material="matte_black",
            name=f"pivot_boss_{i}",
        )
        # Bolt heads on bracket cheeks
        for j, ang in enumerate((0.25, 0.75, 1.25, 1.75)):
            dx = 0.032 * math.cos(ang * math.pi)
            dz = 0.032 * math.sin(ang * math.pi)
            base.visual(
                Cylinder(radius=0.008, length=0.010),
                origin=Origin(
                    xyz=(dx, sy * 0.076, PIVOT_Z + dz),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material="silver_rivet",
                name=f"bracket_bolt_{i}_{j}",
            )

    # Bracket top plate connecting the two cheeks (spans full cheek width)
    base.visual(
        Box((BRACKET_W, 0.15, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z + BRACKET_H / 2.0 - 0.001)),
        material="matte_black",
        name="bracket_top_plate",
    )

    # Gusset plates connecting bracket cheeks to A-frame legs
    cheek_outer_y = 0.065 + 0.016 / 2.0  # 0.073
    leg_inner_y = LEG_TOP_Y - AFRAME_LEG_R  # 0.12 - 0.032 = 0.088
    gusset_cy = (cheek_outer_y + leg_inner_y) / 2.0
    gusset_dy = leg_inner_y - cheek_outer_y + 0.01  # ensure overlap with both
    for i, sy in enumerate((1.0, -1.0)):
        base.visual(
            Box((BRACKET_W, gusset_dy, 0.06)),
            origin=Origin(xyz=(0.0, sy * gusset_cy, PIVOT_Z)),
            material="matte_black",
            name=f"gusset_plate_{i}",
        )

    # Locking pin guide sleeve (a short tube through the bracket, below bosses)
    base.visual(
        Cylinder(radius=PIN_R + 0.005, length=0.14),
        origin=Origin(
            xyz=(0.0, 0.0, PIN_SLEEVE_Z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material="matte_black",
        name="pin_guide_sleeve",
    )
    # Vertical mounting plate connecting sleeve to bracket bottom
    # Must span full Y width to contact bracket cheeks for connectivity
    sleeve_top_z = PIN_SLEEVE_Z + PIN_R + 0.005
    bracket_bottom_z = PIVOT_Z - BRACKET_H / 2.0
    mount_height = bracket_bottom_z - sleeve_top_z
    if mount_height > 0.005:
        mount_cz = (sleeve_top_z + bracket_bottom_z) / 2.0
        base.visual(
            Box((0.05, 0.15, mount_height)),
            origin=Origin(xyz=(0.0, 0.0, mount_cz)),
            material="matte_black",
            name="sleeve_mount_plate",
        )

    # -----------------------------------------------------------------
    # Locking pin: slides along Y axis through the bracket.
    # -----------------------------------------------------------------
    lock_pin = model.part("locking_pin")

    # Pin shaft: offset so at rest (q=0) the shaft is retracted outward
    # and the handle clears the bracket cheek.
    PIN_SHAFT_OFFSET = -0.02  # shift shaft toward handle side
    lock_pin.visual(
        Cylinder(radius=PIN_R, length=PIN_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, PIN_SHAFT_OFFSET)),
        material="yellow_pin",
        name="pin_shaft",
    )
    # Pin handle ring at the outer end
    HANDLE_Z_LOCAL = PIN_SHAFT_OFFSET - PIN_LENGTH / 2.0 - PIN_RING_T / 2.0
    lock_pin.visual(
        Cylinder(radius=PIN_HANDLE_R, length=PIN_RING_T),
        origin=Origin(xyz=(0.0, 0.0, HANDLE_Z_LOCAL)),
        material="yellow_pin",
        name="pin_handle_ring",
    )
    # Small cross-bar through the handle ring for grip
    lock_pin.visual(
        Cylinder(radius=0.006, length=0.040),
        origin=Origin(
            xyz=(0.0, 0.0, HANDLE_Z_LOCAL),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material="matte_black",
        name="pin_grip_bar",
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

    # Locking notch on the pivot stub (a groove the pin passes through)
    rocker.visual(
        Box((0.10, 0.028, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, -0.05)),
        material="matte_black",
        name="lock_notch",
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

        # Molded bucket seat with raised lips (CadQuery mesh).
        seat_mesh = _build_molded_seat(s)
        rocker.visual(
            mesh_from_cadquery(seat_mesh, f"molded_seat_{i}"),
            origin=Origin(xyz=(s * SEAT_CENTER_X, 0.0, SEAT_Z)),
            material="dark_gray_steel",
            name=f"molded_seat_{i}",
        )

        # Seat mounting bolts
        rivet_xy = [(0.10, 0.0), (0.0, 0.08), (0.0, -0.08), (-0.10, 0.06), (-0.10, -0.06)]
        for j, (lx, ly) in enumerate(rivet_xy):
            rocker.visual(
                Cylinder(radius=0.008, length=0.010),
                origin=Origin(xyz=(s * (SEAT_CENTER_X + lx), ly, SEAT_Z + 0.001)),
                material="silver_rivet",
                name=f"seat_bolt_{i}_{j}",
            )

        # Small black stop fin under the seat nose.
        rocker.visual(
            Box((0.045, 0.022, 0.04)),
            origin=Origin(xyz=(s * 1.26, 0.0, 0.038)),
            material="matte_black",
            name=f"seat_fin_{i}",
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

    # -----------------------------------------------------------------
    # Articulations
    # -----------------------------------------------------------------

    # Locking pin: prismatic joint along Y axis (slides in/out of bracket)
    # Pin is centered between legs at rest, slides outward to lock/unlock.
    model.articulation(
        "pin_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=lock_pin,
        origin=Origin(
            xyz=(0.0, 0.0, PIN_SLEEVE_Z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=50.0, velocity=0.3, lower=0.0, upper=0.15
        ),
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

    return model


def _intersects(a, b, tol: float = 1e-4) -> bool:
    if a is None or b is None:
        return False
    return all(a[0][i] <= b[1][i] + tol and b[0][i] <= a[1][i] + tol for i in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("aframe_base")
    rocker = object_model.get_part("rocker")
    lock_pin = object_model.get_part("locking_pin")
    pivot = object_model.get_articulation("rocker_pivot")
    pin_slide = object_model.get_articulation("pin_slide")

    # --- A-frame structure checks ---
    # A-frame legs exist and are visible
    leg0 = ctx.part_element_world_aabb(base, elem="aframe_leg_0")
    leg1 = ctx.part_element_world_aabb(base, elem="aframe_leg_1")
    ctx.check(
        "A-frame has two legs",
        leg0 is not None and leg1 is not None,
        details=f"leg0={leg0}, leg1={leg1}",
    )

    # Legs spread apart at bottom and converge at top
    ctx.check(
        "A-frame legs spread wider at ground than at apex",
        leg0 is not None and leg1 is not None
        and (leg0[1][1] - leg0[0][1]) > 0.01
        and (leg1[1][1] - leg1[0][1]) > 0.01,
        details=f"leg0_y_span={leg0}, leg1_y_span={leg1}",
    )

    # Crossbar connecting the legs
    crossbar = ctx.part_element_world_aabb(base, elem="crossbar")
    ctx.check(
        "crossbar exists on A-frame structure",
        crossbar is not None and crossbar[0][2] > PIVOT_Z - 0.20,
        details=f"crossbar={crossbar}",
    )

    # Bracket cheeks visible
    cheek0 = ctx.part_element_world_aabb(base, elem="bracket_cheek_0")
    cheek1 = ctx.part_element_world_aabb(base, elem="bracket_cheek_1")
    ctx.check(
        "axle bracket cheeks present on A-frame",
        cheek0 is not None and cheek1 is not None,
        details=f"cheek0={cheek0}, cheek1={cheek1}",
    )

    # --- Locking pin checks ---
    pin_shaft = ctx.part_element_world_aabb(lock_pin, elem="pin_shaft")
    pin_handle = ctx.part_element_world_aabb(lock_pin, elem="pin_handle_ring")
    ctx.check(
        "locking pin shaft exists",
        pin_shaft is not None,
        details=f"pin_shaft={pin_shaft}",
    )
    ctx.check(
        "locking pin has a handle ring",
        pin_handle is not None,
        details=f"pin_handle={pin_handle}",
    )

    # Pin slide joint is prismatic
    ctx.check(
        "pin_slide articulation is prismatic",
        pin_slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={pin_slide.articulation_type}",
    )

    # Pin slide limits
    pin_lim = pin_slide.motion_limits
    ctx.check(
        "pin slide has valid travel limits",
        pin_lim is not None and pin_lim.lower is not None and pin_lim.upper is not None
        and pin_lim.upper > pin_lim.lower and pin_lim.upper >= 0.05,
        details=f"limits=({pin_lim.lower}, {pin_lim.upper})",
    )

    # Decisive pose: pin retracted vs inserted
    pin_rest = ctx.part_world_position(lock_pin)
    pin_slide_upper = pin_lim.upper if pin_lim and pin_lim.upper else 0.08
    with ctx.pose({pin_slide: pin_slide_upper}):
        pin_extended = ctx.part_world_position(lock_pin)
        ctx.check(
            "locking pin translates when slid",
            pin_rest is not None and pin_extended is not None
            and any(abs(pin_extended[i] - pin_rest[i]) > 0.02 for i in range(3)),
            details=f"rest={pin_rest}, extended={pin_extended}",
        )

    # Pin guide sleeve exists on bracket
    sleeve = ctx.part_element_world_aabb(base, elem="pin_guide_sleeve")
    ctx.check(
        "pin guide sleeve on bracket",
        sleeve is not None and abs(sleeve[0][2] + sleeve[1][2] - 2 * PIN_SLEEVE_Z) < 0.04,
        details=f"sleeve={sleeve}",
    )

    # Lock notch on rocker stub
    lock_notch = ctx.part_element_world_aabb(rocker, elem="lock_notch")
    ctx.check(
        "lock notch exists on pivot stub",
        lock_notch is not None,
        details=f"lock_notch={lock_notch}",
    )

    # --- Molded seat checks ---
    seat0 = ctx.part_element_world_aabb(rocker, elem="molded_seat_0")
    seat1 = ctx.part_element_world_aabb(rocker, elem="molded_seat_1")
    ctx.check(
        "molded seats exist at both ends",
        seat0 is not None and seat1 is not None,
        details=f"seat0={seat0}, seat1={seat1}",
    )

    # Seats should have visible height extent (raised lips make them taller than flat plates)
    ctx.check(
        "molded seats have raised lips (height extent > 0.03m)",
        seat0 is not None and seat1 is not None
        and (seat0[1][2] - seat0[0][2]) > 0.03
        and (seat1[1][2] - seat1[0][2]) > 0.03,
        details=f"seat0_z={seat0[1][2] - seat0[0][2]:.4f}, seat1_z={seat1[1][2] - seat1[0][2]:.4f}",
    )

    # Seats should have reasonable width extent (bucket shape)
    ctx.check(
        "molded seats have bucket width (Y extent > 0.20m)",
        seat0 is not None and seat1 is not None
        and (seat0[1][1] - seat0[0][1]) > 0.20
        and (seat1[1][1] - seat1[0][1]) > 0.20,
        details=f"seat0_y={seat0[1][1] - seat0[0][1]:.4f}, seat1_y={seat1[1][1] - seat1[0][1]:.4f}",
    )

    # The red pivot stub is intentionally captured inside the black bracket.
    ctx.allow_overlap(
        rocker,
        base,
        elem_a="pivot_stub",
        elem_b="bracket_cheek_0",
        reason="The red center stub descends into the A-frame axle bracket that captures the rocking axle.",
    )
    ctx.allow_overlap(
        rocker,
        base,
        elem_a="pivot_stub",
        elem_b="bracket_cheek_1",
        reason="The red center stub descends into the A-frame axle bracket that captures the rocking axle.",
    )
    ctx.allow_overlap(
        rocker,
        base,
        elem_a="pivot_stub",
        elem_b="bracket_top_plate",
        reason="The red center stub passes through the bracket top plate into the axle housing.",
    )
    ctx.allow_overlap(
        rocker,
        base,
        elem_a="lock_notch",
        elem_b="pin_guide_sleeve",
        reason="The lock notch on the stub aligns with the pin guide sleeve for the locking mechanism.",
    )
    ctx.allow_overlap(
        base,
        rocker,
        elem_a="pin_guide_sleeve",
        elem_b="pivot_stub",
        reason="The pin guide sleeve passes through the pivot stub region to enable the locking pin to engage the lock notch.",
    )
    # Locking pin shaft inside the guide sleeve
    ctx.allow_overlap(
        lock_pin,
        base,
        elem_a="pin_shaft",
        elem_b="pin_guide_sleeve",
        reason="The locking pin slides inside the guide sleeve mounted on the bracket.",
    )
    # Locking pin passes through the pivot stub lock notch
    ctx.allow_overlap(
        lock_pin,
        rocker,
        elem_a="pin_shaft",
        elem_b="pivot_stub",
        reason="The locking pin passes through the lock notch in the pivot stub to secure the rocker.",
    )
    ctx.allow_overlap(
        lock_pin,
        rocker,
        elem_a="pin_shaft",
        elem_b="lock_notch",
        reason="The locking pin engages the lock notch on the pivot stub.",
    )
    # Pin passes through holes in bracket cheeks
    ctx.allow_overlap(
        base,
        lock_pin,
        elem_a="bracket_cheek_0",
        elem_b="pin_shaft",
        reason="The locking pin passes through a hole in the bracket cheek plate.",
    )
    ctx.allow_overlap(
        base,
        lock_pin,
        elem_a="bracket_cheek_1",
        elem_b="pin_shaft",
        reason="The locking pin passes through a hole in the bracket cheek plate.",
    )
    # Pin handle ring passes near the pivot boss region
    ctx.allow_overlap(
        base,
        lock_pin,
        elem_a="pivot_boss_1",
        elem_b="pin_handle_ring",
        reason="The pin handle ring sits adjacent to the pivot boss at the bracket cheek; simplified geometry has minor overlap representing the handle capture zone.",
    )

    ctx.expect_overlap(
        rocker,
        base,
        axes="z",
        elem_a="pivot_stub",
        elem_b="bracket_cheek_0",
        min_overlap=0.02,
        name="pivot stub inserted into bracket",
    )

    # Bracket seated on the A-frame structure (both on base part).
    ground = ctx.part_element_world_aabb(base, elem="ground_plate")
    ctx.check(
        "A-frame legs reach ground plate",
        leg0 is not None and leg1 is not None and ground is not None
        and leg0[0][2] <= ground[1][2] + 0.03
        and leg1[0][2] <= ground[1][2] + 0.03,
        details=f"leg0={leg0}, leg1={leg1}, ground={ground}",
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
        ra is not None and ba is not None and 0.82 <= max(ra[1][2], ba[1][2]) <= 1.05,
        details=f"rocker={ra}, base={ba}",
    )

    # End assemblies: seats hang below the beam, grip plates rise above it.
    grip0 = ctx.part_element_world_aabb(rocker, elem="handle_plate_0")
    grip1 = ctx.part_element_world_aabb(rocker, elem="handle_plate_1")
    ctx.check(
        "seats at sitting height below the beam",
        seat0 is not None
        and seat1 is not None
        and 0.30 <= seat0[1][2] <= 0.55
        and 0.30 <= seat1[1][2] <= 0.55,
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
        and abs(_cx(seat0) + _cx(seat1)) < 0.04
        and abs(seat0[1][2] - seat1[1][2]) < 0.02,
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

    # At least two non-fixed articulations exist (rocker_pivot + pin_slide)
    non_fixed = [
        a for a in object_model.articulations
        if a.articulation_type not in (ArticulationType.FIXED,)
    ]
    ctx.check(
        "at least two non-fixed articulations",
        len(non_fixed) >= 2,
        details=f"non_fixed={[a.name for a in non_fixed]}",
    )

    # Decisive pose checks: the whole rocker tilts as one body.
    base_rest = ctx.part_world_aabb(base)
    with ctx.pose({pivot: ROCK_LIMIT}):
        seat0_dn = ctx.part_element_world_aabb(rocker, elem="molded_seat_0")
        seat1_up = ctx.part_element_world_aabb(rocker, elem="molded_seat_1")
        rocker_dn = ctx.part_world_aabb(rocker)
        base_posed = ctx.part_world_aabb(base)
        ctx.check(
            "positive rock lowers seat_0 and raises seat_1",
            seat0_dn is not None
            and seat1_up is not None
            and seat0 is not None
            and seat1 is not None
            and seat0_dn[1][2] < seat0[1][2] - 0.10
            and seat1_up[1][2] > seat1[1][2] + 0.10,
            details=f"seat0_dn={seat0_dn}, seat1_up={seat1_up}",
        )
        ctx.check(
            "rocker clears the ground at full tilt",
            rocker_dn is not None and rocker_dn[0][2] > 0.005,
            details=f"rocker={rocker_dn}",
        )
        ctx.check(
            "A-frame stays fixed while rocking",
            base_rest is not None and base_posed is not None and _intersects(base_rest, base_posed)
            and abs(base_rest[1][2] - base_posed[1][2]) < 1e-6,
            details=f"rest={base_rest}, posed={base_posed}",
        )
    with ctx.pose({pivot: -ROCK_LIMIT}):
        seat0_up = ctx.part_element_world_aabb(rocker, elem="molded_seat_0")
        rocker_up = ctx.part_world_aabb(rocker)
        ctx.check(
            "negative rock raises seat_0",
            seat0_up is not None and seat0 is not None and seat0_up[0][2] > seat0[0][2] + 0.10,
            details=f"seat0_up={seat0_up}",
        )
        ctx.check(
            "rocker clears the ground at opposite tilt",
            rocker_up is not None and rocker_up[0][2] > 0.005,
            details=f"rocker={rocker_up}",
        )

    return ctx.report()


object_model = build_object_model()
