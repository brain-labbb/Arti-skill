from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ConeGeometry,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
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
HANDLE_X = 1.03
HANDLE_Z = 0.552        # handle grip mid-plane, relative to the pivot

ROCK_LIMIT = 0.262      # ~15 degrees each way

PEDESTAL_R = 0.075
PEDESTAL_H = 0.22
BRACKET_SIZE = (0.16, 0.13, 0.17)
BRACKET_CZ = 0.295      # bracket box center height (spans 0.21 .. 0.38)

# Spring dimensions
SPRING_X = 0.07         # spring anchor position along beam (within bracket footprint)
SPRING_COIL_R = 0.022   # spring coil radius
SPRING_WIRE_R = 0.005   # spring wire radius
SPRING_HEIGHT = 0.040   # spring free height (fits between bracket top and beam underside)
SPRING_TURNS = 5
SPRING_ANCHOR_H = 0.012 # spring anchor lug height

# Locking pin dimensions
PIN_R = 0.006           # pin shaft radius
PIN_LENGTH = 0.14       # pin shaft length
PIN_HANDLE_R = 0.014    # pin T-handle bar radius
PIN_HANDLE_L = 0.055    # pin T-handle bar length
PIN_SLIDE = 0.08        # pin retraction travel


def _beam_z(x: float) -> float:
    """Beam centerline height (relative to the pivot frame) at station x."""
    return BEAM_CENTER_Z + CURVE_C * x * x


def _helix_points(coil_r: float, height: float, turns: int, n_per_turn: int = 16):
    """Generate helix centerline points for a compression spring."""
    pts = []
    total = turns * n_per_turn
    for i in range(total + 1):
        t = i / total
        angle = 2.0 * math.pi * turns * t
        pts.append((coil_r * math.cos(angle), coil_r * math.sin(angle), height * t))
    return pts


def _make_molded_seat(mirror: bool = False):
    """Build a molded bucket seat with raised lip walls using CadQuery."""
    # Outer shell: 300 x 260 mm footprint, 38 mm total height
    outer = (
        cq.Workplane("XY")
        .rect(0.30, 0.26)
        .extrude(0.038)
        .edges("|Z").fillet(0.055)
        .edges(">Z").fillet(0.004)
    )
    # Interior cavity: leaves 12 mm floor and ~6 mm walls, 26 mm deep
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=0.012)
        .rect(0.278, 0.238)
        .extrude(0.027)
        .edges("|Z").fillet(0.049)
    )
    seat = outer.cut(cavity)
    if mirror:
        seat = seat.mirror("XZ")
    return seat


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="spring_seesaw")

    model.material("gloss_red_orange", rgba=(0.88, 0.20, 0.06, 1.0))
    model.material("matte_black", rgba=(0.09, 0.09, 0.10, 1.0))
    model.material("light_gray", rgba=(0.78, 0.78, 0.79, 1.0))
    model.material("dark_gray_steel", rgba=(0.34, 0.36, 0.38, 1.0))
    model.material("silver_rivet", rgba=(0.74, 0.75, 0.78, 1.0))
    model.material("spring_steel", rgba=(0.55, 0.58, 0.62, 1.0))
    model.material("rubber_grip", rgba=(0.14, 0.14, 0.16, 1.0))
    model.material("pin_yellow", rgba=(0.92, 0.78, 0.10, 1.0))

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

    # Spring anchor lugs on bracket top (small black bosses).
    for i, sx in enumerate((1.0, -1.0)):
        base.visual(
            Cylinder(radius=0.018, length=SPRING_ANCHOR_H),
            origin=Origin(
                xyz=(sx * SPRING_X, 0.0, BRACKET_CZ + BRACKET_SIZE[2] / 2.0 + SPRING_ANCHOR_H / 2.0)
            ),
            material="matte_black",
            name=f"spring_anchor_{i}",
        )

    # Pin bore indicator on bracket +Y face (where locking pin enters).
    base.visual(
        Cylinder(radius=0.010, length=0.004),
        origin=Origin(
            xyz=(0.0, BRACKET_SIZE[1] / 2.0 + 0.002, PIVOT_Z - 0.08),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material="silver_rivet",
        name="pin_bore_mark",
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

    # Lock notch indicator ring on the pivot stub (where pin engages at q=0).
    rocker.visual(
        Cylinder(radius=0.052, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="silver_rivet",
        name="pin_notch_ring",
    )

    # Compression springs on each side of center, between bracket and beam.
    bracket_top_local = BRACKET_CZ + BRACKET_SIZE[2] / 2.0 - PIVOT_Z
    spring_z_base = bracket_top_local + SPRING_ANCHOR_H  # start above anchor top
    for i, sx in enumerate((1.0, -1.0)):
        spring_pts = _helix_points(SPRING_COIL_R, SPRING_HEIGHT, SPRING_TURNS)
        # Translate spring to sit between bracket top and beam underside
        translated_pts = [
            (px + sx * SPRING_X, py, pz + spring_z_base)
            for px, py, pz in spring_pts
        ]
        rocker.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    translated_pts,
                    radius=SPRING_WIRE_R,
                    samples_per_segment=2,
                    radial_segments=10,
                    cap_ends=False,
                ),
                f"spring_coil_{i}",
            ),
            material="spring_steel",
            name=f"spring_coil_{i}",
        )
        # Spring anchor plate bridging to beam underside
        rocker.visual(
            Cylinder(radius=0.018, length=0.012),
            origin=Origin(xyz=(sx * SPRING_X, 0.0, spring_z_base + SPRING_HEIGHT + 0.006)),
            material="matte_black",
            name=f"spring_cap_{i}",
        )

    # Collars, drop tubes, molded seats, and handle assemblies at each end.
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

        # Molded bucket seat with raised lip walls (CadQuery).
        seat_mesh = mesh_from_cadquery(
            _make_molded_seat(mirror=(s < 0)),
            f"molded_seat_{i}",
        )
        rocker.visual(
            seat_mesh,
            origin=Origin(xyz=(s * SEAT_CENTER_X, 0.0, SEAT_Z)),
            material="dark_gray_steel",
            name=f"molded_seat_{i}",
        )

        # Small black stop fin under the seat nose (contacts seat bottom).
        rocker.visual(
            Box((0.045, 0.022, 0.04)),
            origin=Origin(xyz=(s * 1.18, 0.0, SEAT_Z - 0.002)),
            material="matte_black",
            name=f"seat_fin_{i}",
        )

        # Thin red post rising from the beam to the handle grip.
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

        # Rounded handle grip: horizontal bar along Y with rubber sleeve.
        # Inner steel crossbar
        rocker.visual(
            Cylinder(radius=0.012, length=0.20),
            origin=Origin(
                xyz=(s * HANDLE_X, 0.0, HANDLE_Z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material="silver_rivet",
            name=f"grip_bar_{i}",
        )
        # Rubber grip sleeve over the center of the bar
        rocker.visual(
            Cylinder(radius=0.019, length=0.14),
            origin=Origin(
                xyz=(s * HANDLE_X, 0.0, HANDLE_Z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material="rubber_grip",
            name=f"grip_sleeve_{i}",
        )
        # End caps on the grip bar
        for k, ey in enumerate((-0.10, 0.10)):
            rocker.visual(
                Sphere(radius=0.014),
                origin=Origin(xyz=(s * HANDLE_X, ey, HANDLE_Z)),
                material="matte_black",
                name=f"grip_cap_{i}_{k}",
            )

    # -----------------------------------------------------------------
    # Locking pin: slides through bracket to engage/disengage rocker.
    # -----------------------------------------------------------------
    pin = model.part("locking_pin")

    # Pin shaft: oriented along Y, extends inward (toward -Y) from the part frame.
    pin.visual(
        Cylinder(radius=PIN_R, length=PIN_LENGTH),
        origin=Origin(
            xyz=(0.0, 0.022 - PIN_LENGTH / 2.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material="silver_rivet",
        name="pin_shaft",
    )
    # Pin collar at the shaft/handle junction for structural continuity.
    pin.visual(
        Cylinder(radius=PIN_HANDLE_R + 0.002, length=0.010),
        origin=Origin(
            xyz=(0.0, 0.017, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material="silver_rivet",
        name="pin_collar",
    )
    # Pin T-handle crossbar (perpendicular to shaft, along X).
    pin.visual(
        Cylinder(radius=PIN_HANDLE_R, length=PIN_HANDLE_L),
        origin=Origin(xyz=(0.0, 0.022, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="pin_yellow",
        name="pin_handle_bar",
    )
    # T-handle grip knobs at each end.
    for k, sx in enumerate((-1.0, 1.0)):
        pin.visual(
            Sphere(radius=0.012),
            origin=Origin(xyz=(sx * PIN_HANDLE_L / 2.0, 0.022, 0.0)),
            material="pin_yellow",
            name=f"pin_handle_knob_{k}",
        )

    # -----------------------------------------------------------------
    # Articulations
    # -----------------------------------------------------------------

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

    # Locking pin slide: prismatic joint along Y (perpendicular to seesaw).
    # At q=0 pin is inserted (locked); at q=PIN_SLIDE pin is retracted (free).
    model.articulation(
        "pin_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=pin,
        origin=Origin(
            xyz=(0.0, BRACKET_SIZE[1] / 2.0 + 0.018, PIVOT_Z - 0.08),
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=50.0, velocity=0.15, lower=0.0, upper=PIN_SLIDE
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
    pin = object_model.get_part("locking_pin")
    pivot = object_model.get_articulation("rocker_pivot")
    pin_joint = object_model.get_articulation("pin_slide")

    # The red pivot stub is intentionally captured inside the black bracket.
    ctx.allow_overlap(
        rocker,
        base,
        elem_a="pivot_stub",
        elem_b="pivot_bracket",
        reason="The red center stub descends into the cast pivot bracket that captures the rocking axle.",
    )
    ctx.allow_overlap(
        rocker,
        base,
        elem_a="pin_notch_ring",
        elem_b="pivot_bracket",
        reason="The lock-notch ring sits on the pivot stub inside the bracket, marking the pin engagement zone.",
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

    # The locking pin shaft passes through the bracket and engages the rocker stub.
    ctx.allow_overlap(
        pin,
        base,
        elem_a="pin_shaft",
        elem_b="pivot_bracket",
        reason="The locking pin shaft slides through a bore in the pivot bracket for locking engagement.",
    )
    ctx.allow_overlap(
        pin,
        base,
        elem_a="pin_shaft",
        elem_b="pin_bore_mark",
        reason="The pin shaft passes through the bore mark on the bracket face that directs it into the bore.",
    )
    ctx.allow_overlap(
        pin,
        rocker,
        elem_a="pin_shaft",
        elem_b="pivot_stub",
        reason="When locked (q=0), the pin shaft engages a cross-bore in the rocker pivot stub to prevent rocking.",
    )
    # Proof: pin remains near the bracket at rest and retracts along +Y.
    ctx.expect_overlap(
        pin,
        base,
        axes="z",
        elem_a="pin_shaft",
        elem_b="pivot_bracket",
        min_overlap=0.005,
        name="pin shaft passes through bracket height band",
    )

    # Bracket seated on the pedestal.
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

    # Molded seats: have raised lips (taller than a flat plate would be).
    seat0 = ctx.part_element_world_aabb(rocker, elem="molded_seat_0")
    seat1 = ctx.part_element_world_aabb(rocker, elem="molded_seat_1")
    ctx.check(
        "molded seats exist at both ends",
        seat0 is not None and seat1 is not None,
        details=f"seat0={seat0}, seat1={seat1}",
    )
    ctx.check(
        "molded seats have raised lips (height > 0.030 m)",
        seat0 is not None
        and seat1 is not None
        and (seat0[1][2] - seat0[0][2]) > 0.030
        and (seat1[1][2] - seat1[0][2]) > 0.030,
        details=f"seat0_z_range={None if seat0 is None else seat0[1][2]-seat0[0][2]}, seat1_z_range={None if seat1 is None else seat1[1][2]-seat1[0][2]}",
    )

    # Seats at sitting height below the beam.
    ctx.check(
        "seats at sitting height below the beam",
        seat0 is not None
        and seat1 is not None
        and 0.33 <= seat0[1][2] <= 0.48
        and 0.33 <= seat1[1][2] <= 0.48,
        details=f"seat0={seat0}, seat1={seat1}",
    )

    # Rounded handle grips: grip sleeves exist above the beam ends.
    grip0 = ctx.part_element_world_aabb(rocker, elem="grip_sleeve_0")
    grip1 = ctx.part_element_world_aabb(rocker, elem="grip_sleeve_1")
    ctx.check(
        "rounded grip sleeves exist above beam ends",
        grip0 is not None
        and grip1 is not None
        and beam is not None
        and grip0[0][2] > beam[1][2] - 0.05
        and grip1[0][2] > beam[1][2] - 0.05,
        details=f"grip0={grip0}, grip1={grip1}, beam={beam}",
    )
    ctx.check(
        "grip sleeves are cylindrical (longer in Y than X or Z)",
        grip0 is not None
        and grip1 is not None
        and (grip0[1][1] - grip0[0][1]) > (grip0[1][0] - grip0[0][0])
        and (grip1[1][1] - grip1[0][1]) > (grip1[1][0] - grip1[0][0]),
        details=f"grip0={grip0}, grip1={grip1}",
    )

    # Spring coils exist near center of beam.
    spring0 = ctx.part_element_world_aabb(rocker, elem="spring_coil_0")
    spring1 = ctx.part_element_world_aabb(rocker, elem="spring_coil_1")
    ctx.check(
        "spring coils present near beam center",
        spring0 is not None and spring1 is not None,
        details=f"spring0={spring0}, spring1={spring1}",
    )
    ctx.check(
        "springs positioned between bracket and beam",
        spring0 is not None
        and spring1 is not None
        and abs(0.5 * (spring0[0][0] + spring0[1][0])) < 0.30
        and abs(0.5 * (spring1[0][0] + spring1[1][0])) < 0.30,
        details=f"spring0_center={None if spring0 is None else 0.5*(spring0[0][0]+spring0[1][0])}, spring1_center={None if spring1 is None else 0.5*(spring1[0][0]+spring1[1][0])}",
    )

    # Locking pin: prismatic articulation exists and has correct type.
    ctx.check(
        "locking pin has prismatic articulation",
        pin_joint is not None
        and pin_joint.articulation_type == ArticulationType.PRISMATIC,
        details=f"joint={pin_joint}",
    )
    pin_lim = pin_joint.motion_limits
    ctx.check(
        "locking pin travel about 0.08 m",
        pin_lim is not None
        and abs(pin_lim.lower) < 0.005
        and 0.06 <= pin_lim.upper <= 0.10,
        details=f"limits=({pin_lim.lower}, {pin_lim.upper})",
    )

    # Locking pin geometry near the bracket.
    pin_aabb = ctx.part_world_aabb(pin)
    ctx.check(
        "locking pin near the central bracket",
        pin_aabb is not None
        and abs(0.5 * (pin_aabb[0][0] + pin_aabb[1][0])) < 0.15
        and pin_aabb[0][2] > 0.20,
        details=f"pin_aabb={pin_aabb}",
    )

    # Pin slide pose check: pin retracts along +Y.
    pin_rest_pos = ctx.part_world_position(pin)
    with ctx.pose({pin_joint: PIN_SLIDE}):
        pin_retracted_pos = ctx.part_world_position(pin)
        ctx.check(
            "locking pin retracts outward along Y",
            pin_rest_pos is not None
            and pin_retracted_pos is not None
            and pin_retracted_pos[1] > pin_rest_pos[1] + 0.05,
            details=f"rest={pin_rest_pos}, retracted={pin_retracted_pos}",
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
        "grip sleeves mirrored about the pivot",
        grip0 is not None and grip1 is not None and abs(_cx(grip0) + _cx(grip1)) < 0.02,
        details=f"grip0={grip0}, grip1={grip1}",
    )

    # Mounted, not floating: drop tubes reach seats, posts reach grip bars,
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
    grip_bar0 = ctx.part_element_world_aabb(rocker, elem="grip_bar_0")
    grip_bar1 = ctx.part_element_world_aabb(rocker, elem="grip_bar_1")
    ctx.check(
        "handle posts connect beam to grip bars",
        _intersects(post0, beam)
        and _intersects(post0, grip_bar0)
        and _intersects(post1, beam)
        and _intersects(post1, grip_bar1),
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

    # Decisive pose checks: rocker tilts, seats swap height, base stays put.
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
            base_rest is not None
            and base_posed is not None
            and _intersects(base_rest, base_posed)
            and abs(base_rest[1][2] - base_posed[1][2]) < 1e-6,
            details=f"rest={base_rest}, posed={base_posed}",
        )
    with ctx.pose({pivot: -ROCK_LIMIT}):
        seat0_up = ctx.part_element_world_aabb(rocker, elem="molded_seat_0")
        rocker_up = ctx.part_world_aabb(rocker)
        ctx.check(
            "negative rock raises seat_0",
            seat0_up is not None
            and seat0 is not None
            and seat0_up[0][2] > seat0[0][2] + 0.15,
            details=f"seat0_up={seat0_up}",
        )
        ctx.check(
            "rocker clears the ground at opposite tilt",
            rocker_up is not None and rocker_up[0][2] > 0.005,
            details=f"rocker={rocker_up}",
        )

    return ctx.report()


object_model = build_object_model()
