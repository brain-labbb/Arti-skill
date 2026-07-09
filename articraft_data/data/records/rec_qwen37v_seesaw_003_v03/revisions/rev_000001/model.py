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
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
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
PLATE_T = 0.015         # thicker molded seat base
LIP_H = 0.035           # raised lip height above the seat base
HANDLE_X = 1.03
HANDLE_Z = 0.552        # handle plate mid-plane, relative to the pivot

ROCK_LIMIT = 0.262      # ~15 degrees each way

PEDESTAL_R = 0.075
PEDESTAL_H = 0.22
BRACKET_SIZE = (0.16, 0.13, 0.17)
BRACKET_CZ = 0.295      # bracket box center height (spans 0.21 .. 0.38)

# Bumper dimensions and positions
BUMPER_X = 0.80         # bumper station along beam (inboard of collar)
BUMPER_R = 0.035        # bumper cylinder radius
BUMPER_H = 0.050        # bumper cylinder height
BUMPER_COMPRESS = 0.028 # max compression travel (meters)

# Spring dimensions
SPRING_R = 0.042        # coil mean radius
SPRING_WIRE_R = 0.007   # wire cross-section radius
SPRING_COILS = 6
SPRING_PITCH = 0.016    # vertical distance per coil
SPRING_BASE_Z = 0.04    # bottom of spring in rocker frame (above bracket top)


def _beam_z(x: float) -> float:
    """Beam centerline height (relative to the pivot frame) at station x."""
    return BEAM_CENTER_Z + CURVE_C * x * x


def _build_spring_mesh() -> MeshGeometry:
    """Build a continuous helical coil spring as a swept tube."""
    n_pts = SPRING_COILS * 16
    pts = []
    for k in range(n_pts + 1):
        t = k / n_pts
        angle = 2.0 * math.pi * SPRING_COILS * t
        x = SPRING_R * math.cos(angle)
        y = SPRING_R * math.sin(angle)
        z = SPRING_BASE_Z + SPRING_PITCH * SPRING_COILS * t
        pts.append((x, y, z))
    return tube_from_spline_points(
        pts,
        radius=SPRING_WIRE_R,
        samples_per_segment=6,
        radial_segments=12,
        cap_ends=True,
    )


def _build_molded_seat_base(sign: float) -> MeshGeometry:
    """Build a molded bucket seat: flat base plate + raised lip frame."""
    # Outer seat profile (rounded triangular plate, same as parent)
    outer = sample_catmull_rom_spline_2d(
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
    # Inner profile: scaled down to create the lip wall thickness
    inner_scale = 0.80
    inner = [(x * inner_scale, y * inner_scale) for x, y in outer]

    # Base plate (slightly thicker than parent for molded feel)
    base = ExtrudeGeometry(outer, PLATE_T, cap=True, center=True)

    # Raised lip: a frame ring on top of the base plate
    lip = ExtrudeWithHolesGeometry(outer, [inner], LIP_H, cap=True, center=True)
    lip.translate(0.0, 0.0, (PLATE_T + LIP_H) / 2.0)

    seat_mesh = base.merge(lip)
    if sign < 0:
        seat_mesh.rotate_z(math.pi)
    seat_mesh.translate(sign * SEAT_CENTER_X, 0.0, SEAT_Z)
    return seat_mesh


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="playground_seesaw_spring")

    model.material("gloss_red_orange", rgba=(0.88, 0.20, 0.06, 1.0))
    model.material("matte_black", rgba=(0.09, 0.09, 0.10, 1.0))
    model.material("light_gray", rgba=(0.78, 0.78, 0.79, 1.0))
    model.material("dark_gray_steel", rgba=(0.34, 0.36, 0.38, 1.0))
    model.material("silver_rivet", rgba=(0.74, 0.75, 0.78, 1.0))
    model.material("rubber_black", rgba=(0.12, 0.12, 0.13, 1.0))
    model.material("spring_steel", rgba=(0.62, 0.64, 0.66, 1.0))
    model.material("molded_seat", rgba=(0.22, 0.22, 0.24, 1.0))

    # -----------------------------------------------------------------
    # Fixed base: short light-gray ground pedestal + black cast bracket.
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

    # Spring anchor plate on top of bracket (where the spring seats)
    base.visual(
        Cylinder(radius=SPRING_R + 0.008, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, BRACKET_SIZE[2] / 2.0 + BRACKET_CZ + 0.004)),
        material="matte_black",
        name="spring_anchor_plate",
    )

    # -----------------------------------------------------------------
    # Rocker: curved red beam + pivot stub + spring + mirrored seat/handle ends.
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

    # Helical coil spring around the pivot stub (spring assist mechanism).
    rocker.visual(
        mesh_from_geometry(_build_spring_mesh(), "spring_coil"),
        material="spring_steel",
        name="spring_coil",
    )

    # Spring seat plate on top of spring (attached to rocker beam underside)
    spring_top_z = SPRING_BASE_Z + SPRING_PITCH * SPRING_COILS
    rocker.visual(
        Cylinder(radius=SPRING_R + 0.006, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, spring_top_z + 0.003)),
        material="gloss_red_orange",
        name="spring_seat_plate",
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

        # Molded seat with raised lips (bucket-style seat)
        seat_mesh = _build_molded_seat_base(s)
        rocker.visual(
            mesh_from_geometry(seat_mesh, f"molded_seat_{i}"),
            material="molded_seat",
            name=f"molded_seat_{i}",
        )

        # Rivets on the seat base (connecting to drop tube)
        rivet_xy = [(0.13, 0.0), (0.0, 0.10), (0.0, -0.10), (-0.13, 0.075), (-0.13, -0.075)]
        for j, (lx, ly) in enumerate(rivet_xy):
            rocker.visual(
                Cylinder(radius=0.008, length=0.010),
                origin=Origin(xyz=(s * (SEAT_CENTER_X + lx), ly, SEAT_Z - PLATE_T / 2.0 - 0.003)),
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
    # Rubber end bumpers: compress vertically on prismatic joints.
    # -----------------------------------------------------------------
    for i, s in enumerate((1.0, -1.0)):
        bumper = model.part(f"bumper_{i}")
        # Rubber cylinder bumper with rounded feel
        bumper.visual(
            Cylinder(radius=BUMPER_R, length=BUMPER_H),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material="rubber_black",
            name=f"bumper_body_{i}",
        )
        # Small metal mounting plate on top of bumper (connects to beam)
        bumper.visual(
            Cylinder(radius=BUMPER_R + 0.008, length=0.006),
            origin=Origin(xyz=(0.0, 0.0, BUMPER_H / 2.0 + 0.003)),
            material="dark_gray_steel",
            name=f"bumper_mount_plate_{i}",
        )
        # Compression indicator ring (visible rubber ridge)
        bumper.visual(
            mesh_from_geometry(
                TorusGeometry(radius=BUMPER_R - 0.004, tube=0.005),
                f"bumper_ring_{i}",
            ),
            origin=Origin(xyz=(0.0, 0.0, -BUMPER_H / 2.0 + 0.008)),
            material="rubber_black",
            name=f"bumper_ring_{i}",
        )

        # Bumper position in rocker frame: hanging below beam with clearance
        bumper_beam_z = _beam_z(BUMPER_X)
        beam_bottom = bumper_beam_z - BEAM_R
        bumper_gap = 0.010  # 10mm clearance below beam
        bumper_center_z = beam_bottom - bumper_gap - BUMPER_H / 2.0

        model.articulation(
            f"bumper_{i}_slide",
            ArticulationType.PRISMATIC,
            parent=rocker,
            child=bumper,
            origin=Origin(xyz=(s * BUMPER_X, 0.0, bumper_center_z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=800.0, velocity=0.5, lower=0.0, upper=BUMPER_COMPRESS
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
    base = object_model.get_part("pedestal_mount")
    rocker = object_model.get_part("rocker")
    pivot = object_model.get_articulation("rocker_pivot")
    bumper_0 = object_model.get_part("bumper_0")
    bumper_1 = object_model.get_part("bumper_1")
    bumper_0_slide = object_model.get_articulation("bumper_0_slide")
    bumper_1_slide = object_model.get_articulation("bumper_1_slide")

    # The red pivot stub is intentionally captured inside the black bracket.
    ctx.allow_overlap(
        rocker,
        base,
        elem_a="pivot_stub",
        elem_b="pivot_bracket",
        reason="The red center stub descends into the cast pivot bracket that captures the rocking axle.",
    )
    # Spring coil wraps around the pivot stub (intentional nesting)
    ctx.allow_overlap(
        rocker,
        base,
        elem_a="spring_coil",
        elem_b="pivot_bracket",
        reason="The spring coil surrounds the pivot stub and passes through the bracket zone as part of the spring-assist mechanism.",
    )
    # Spring seat plate overlaps the spring coil (seated on top)
    ctx.allow_overlap(
        rocker,
        rocker,
        elem_a="spring_seat_plate",
        elem_b="spring_coil",
        reason="The spring seat plate sits on top of the spring coil as a compression surface.",
    )
    # Bumper bodies and mount plates overlap the beam tube (mounted to beam underside)
    ctx.allow_overlap(
        bumper_0,
        rocker,
        elem_a="bumper_mount_plate_0",
        elem_b="beam_tube",
        reason="The bumper mount plate is fastened to the underside of the beam tube.",
    )
    ctx.allow_overlap(
        bumper_1,
        rocker,
        elem_a="bumper_mount_plate_1",
        elem_b="beam_tube",
        reason="The bumper mount plate is fastened to the underside of the beam tube.",
    )
    ctx.allow_overlap(
        bumper_0,
        rocker,
        elem_a="bumper_body_0",
        elem_b="beam_tube",
        reason="The rubber bumper body is seated against the underside of the beam tube as a compression stop.",
    )
    ctx.allow_overlap(
        bumper_1,
        rocker,
        elem_a="bumper_body_1",
        elem_b="beam_tube",
        reason="The rubber bumper body is seated against the underside of the beam tube as a compression stop.",
    )
    # Spring anchor plate on base overlaps the spring coil on rocker
    ctx.allow_overlap(
        base,
        rocker,
        elem_a="spring_anchor_plate",
        elem_b="spring_coil",
        reason="The spring anchor plate on the bracket seats against the bottom of the spring coil.",
    )
    # Spring anchor plate on base overlaps the pivot stub (stub passes through bracket zone)
    ctx.allow_overlap(
        base,
        rocker,
        elem_a="spring_anchor_plate",
        elem_b="pivot_stub",
        reason="The pivot stub passes through the spring anchor plate zone as part of the captured axle mechanism.",
    )
    # Bumper bodies overlap drop tubes (both mounted to beam assembly at same station)
    ctx.allow_overlap(
        bumper_0,
        rocker,
        elem_a="bumper_body_0",
        elem_b="drop_tube_0",
        reason="The rubber bumper body is positioned near the drop tube at the beam end, both part of the end assembly.",
    )
    ctx.allow_overlap(
        bumper_1,
        rocker,
        elem_a="bumper_body_1",
        elem_b="drop_tube_1",
        reason="The rubber bumper body is positioned near the drop tube at the beam end, both part of the end assembly.",
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

    # Bracket seated on the pedestal (both visuals on the fixed base part).
    bracket = ctx.part_element_world_aabb(base, elem="pivot_bracket")
    pedestal = ctx.part_element_world_aabb(base, elem="ground_pedestal")
    ctx.check(
        "bracket sits atop ground pedestal",
        _intersects(bracket, pedestal),
        details=f"bracket={bracket}, pedestal={pedestal}",
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

    # Spring coil is present and centered near the pivot
    spring = ctx.part_element_world_aabb(rocker, elem="spring_coil")
    ctx.check(
        "spring coil present at pivot area",
        spring is not None
        and abs(0.5 * (spring[0][0] + spring[1][0])) < 0.08
        and abs(0.5 * (spring[0][1] + spring[1][1])) < 0.08,
        details=f"spring={spring}",
    )
    ctx.check(
        "spring coil has vertical extent (helix shape)",
        spring is not None and (spring[1][2] - spring[0][2]) >= 0.06,
        details=f"spring z-range={None if spring is None else spring[1][2] - spring[0][2]}",
    )

    # Molded seats with raised lips exist and are at sitting height
    seat0 = ctx.part_element_world_aabb(rocker, elem="molded_seat_0")
    seat1 = ctx.part_element_world_aabb(rocker, elem="molded_seat_1")
    ctx.check(
        "molded seats at sitting height below the beam",
        seat0 is not None
        and seat1 is not None
        and 0.30 <= seat0[1][2] <= 0.48
        and 0.30 <= seat1[1][2] <= 0.48,
        details=f"seat0={seat0}, seat1={seat1}",
    )
    # Molded seats have vertical extent (raised lips give extra height vs flat plate)
    ctx.check(
        "molded seats have raised lip height",
        seat0 is not None
        and seat1 is not None
        and (seat0[1][2] - seat0[0][2]) >= (PLATE_T + LIP_H * 0.7)
        and (seat1[1][2] - seat1[0][2]) >= (PLATE_T + LIP_H * 0.7),
        details=f"seat0_z_range={None if seat0 is None else seat0[1][2] - seat0[0][2]}, seat1_z_range={None if seat1 is None else seat1[1][2] - seat1[0][2]}",
    )

    # Grip plates above the beam ends
    grip0 = ctx.part_element_world_aabb(rocker, elem="handle_plate_0")
    grip1 = ctx.part_element_world_aabb(rocker, elem="handle_plate_1")
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

    # Bumpers exist at both beam ends (inboard of collars)
    bump0 = ctx.part_world_aabb(bumper_0)
    bump1 = ctx.part_world_aabb(bumper_1)
    ctx.check(
        "bumper_0 positioned at positive beam end",
        bump0 is not None and _cx(bump0) > 0.6,
        details=f"bumper_0={bump0}",
    )
    ctx.check(
        "bumper_1 positioned at negative beam end",
        bump1 is not None and _cx(bump1) < -0.6,
        details=f"bumper_1={bump1}",
    )
    # Bumpers hang below the local beam bottom (beam curves up at ends,
    # so check that the bumper bottom is below the beam center dip zone)
    beam_bottom_center = beam[0][2] if beam is not None else None
    ctx.check(
        "bumpers positioned below beam centerline dip",
        bump0 is not None and bump1 is not None and beam_bottom_center is not None
        and bump0[1][2] < beam_bottom_center + 0.10
        and bump1[1][2] < beam_bottom_center + 0.10,
        details=f"bump0={bump0}, bump1={bump1}, beam_bottom={beam_bottom_center}",
    )

    # Bumper prismatic joints have correct limits
    b0_lim = bumper_0_slide.motion_limits
    b1_lim = bumper_1_slide.motion_limits
    ctx.check(
        "bumper_0 prismatic joint has compression range",
        b0_lim is not None
        and abs(b0_lim.lower) < 0.001
        and 0.015 <= b0_lim.upper <= 0.05,
        details=f"bumper_0 limits=({b0_lim.lower}, {b0_lim.upper})",
    )
    ctx.check(
        "bumper_1 prismatic joint has compression range",
        b1_lim is not None
        and abs(b1_lim.lower) < 0.001
        and 0.015 <= b1_lim.upper <= 0.05,
        details=f"bumper_1 limits=({b1_lim.lower}, {b1_lim.upper})",
    )

    # Mounted, not floating: drop tubes reach seats, posts reach grip plates
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

    # Overall envelope
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

    # Decisive pose checks: rocker tilts, seats swap height, bumpers compress
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

    # Bumper compression pose check
    bump0_rest = ctx.part_world_aabb(bumper_0)
    with ctx.pose({bumper_0_slide: BUMPER_COMPRESS}):
        bump0_compressed = ctx.part_world_aabb(bumper_0)
        ctx.check(
            "bumper_0 compresses upward on prismatic joint",
            bump0_rest is not None
            and bump0_compressed is not None
            and bump0_compressed[0][2] > bump0_rest[0][2] + 0.01,
            details=f"rest={bump0_rest}, compressed={bump0_compressed}",
        )

    with ctx.pose({pivot: -ROCK_LIMIT}):
        seat0_up = ctx.part_element_world_aabb(rocker, elem="molded_seat_0")
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
