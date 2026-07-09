from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoxGeometry,
    CylinderGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    SphereGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---------------------------------------------------------------------------
# Spring-assisted modern playground seesaw.
#
# Layout (world frame, Z up, base centered on origin):
# - Forest-green pedestal base with ground plate, two pivot bracket plates,
#   and a visible coil spring between the brackets.
# - Single cherry-red rocking beam (~2.5 m) with central pivot hub.
# - Molded blue seats with raised lips at each beam end.
# - Upright handle grips with rounded end caps and rubber sleeves, inboard
#   of each seat.
# - Safety-yellow locking pin that slides horizontally near the bracket.
#
# Articulation:
# - beam_pivot: REVOLUTE, horizontal Y axis, +/- 18 degrees.
# - locking_pin_slide: PRISMATIC, Y axis, 0-60 mm travel.
# ---------------------------------------------------------------------------

TUBE_R = 0.025
BEAM_LEN = 2.50
TILT = math.radians(18.0)

GROUND_PLATE_R = 0.28
GROUND_PLATE_H = 0.025

PEDESTAL_W = 0.38
PEDESTAL_D = 0.28
PEDESTAL_H = 0.52

BRACKET_W = 0.32
BRACKET_H = 0.14
BRACKET_T = 0.012
BRACKET_Y = 0.060

PIVOT_Z = GROUND_PLATE_H + PEDESTAL_H + BRACKET_H / 2  # ~0.615 m

SEAT_X = 1.08
HANDLE_X = 0.80

# Materials
FOREST_GREEN = Material("forest_green", rgba=(0.15, 0.40, 0.22, 1.0))
CHERRY_RED = Material("cherry_red", rgba=(0.78, 0.14, 0.11, 1.0))
STEEL_GRAY = Material("steel_gray", rgba=(0.35, 0.35, 0.37, 1.0))
MOLDED_BLUE = Material("molded_blue", rgba=(0.18, 0.33, 0.62, 1.0))
RUBBER_BLACK = Material("rubber_black", rgba=(0.10, 0.10, 0.11, 1.0))
SPRING_STEEL = Material("spring_steel", rgba=(0.55, 0.55, 0.48, 1.0))
SAFETY_YELLOW = Material("safety_yellow", rgba=(0.88, 0.78, 0.10, 1.0))


def _helix_spring(
    radius: float, height: float, coils: int, wire_r: float
) -> MeshGeometry:
    """Helical coil spring built as a tube along a spiral path."""
    pts: list[tuple[float, float, float]] = []
    n = coils * 20
    for i in range(n + 1):
        t = i / n
        a = t * coils * 2.0 * math.pi
        pts.append((radius * math.cos(a), radius * math.sin(a), t * height))
    return tube_from_spline_points(
        pts,
        radius=wire_r,
        samples_per_segment=4,
        radial_segments=8,
        cap_ends=True,
    )


def _molded_seat_mesh() -> MeshGeometry:
    """Molded seat pan with raised lips on all four sides."""
    seat = BoxGeometry((0.28, 0.26, 0.012))

    lip_h = 0.032
    lip_t = 0.010
    half_w = 0.14
    half_d = 0.13

    # Back lip (taller for child retention)
    back = BoxGeometry((0.28, lip_t, 0.055))
    back.translate(0.0, -(half_d - lip_t / 2), 0.055 / 2 - 0.006)
    seat.merge(back)

    # Front lip
    front = BoxGeometry((0.28, lip_t, lip_h))
    front.translate(0.0, half_d - lip_t / 2, lip_h / 2 - 0.006)
    seat.merge(front)

    # Side lips
    for sx in (1.0, -1.0):
        side = BoxGeometry((lip_t, 0.26, lip_h))
        side.translate(sx * (half_w - lip_t / 2), 0.0, lip_h / 2 - 0.006)
        seat.merge(side)

    return seat


def _handle_grip_mesh() -> MeshGeometry:
    """Upright handle post with horizontal grip bar and rounded end caps."""
    h = MeshGeometry()

    # Vertical post
    post = CylinderGeometry(0.014, 0.28, radial_segments=14)
    post.translate(0.0, 0.0, 0.14)
    h.merge(post)

    # Horizontal grip bar
    bar = CylinderGeometry(0.012, 0.20, radial_segments=14)
    bar.rotate_x(math.pi / 2.0)
    bar.translate(0.0, 0.0, 0.28)
    h.merge(bar)

    # Rounded end caps (spheres at each grip end)
    for sy in (1.0, -1.0):
        cap = SphereGeometry(0.018, width_segments=10, height_segments=8)
        cap.translate(0.0, sy * 0.10, 0.28)
        h.merge(cap)

    # Rubber grip sleeve (thicker section in the middle of the bar)
    sleeve = CylinderGeometry(0.017, 0.12, radial_segments=14)
    sleeve.rotate_x(math.pi / 2.0)
    sleeve.translate(0.0, 0.0, 0.28)
    h.merge(sleeve)

    return h


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="spring_seesaw")

    # ── Base (static central pedestal) ──────────────────────────────────
    base = model.part("base")

    # Ground plate
    base.visual(
        mesh_from_geometry(
            CylinderGeometry(GROUND_PLATE_R, GROUND_PLATE_H, radial_segments=24),
            "ground_plate",
        ),
        origin=Origin(xyz=(0.0, 0.0, GROUND_PLATE_H / 2)),
        material=STEEL_GRAY,
        name="ground_plate",
    )

    # Pedestal column
    ped_z0 = GROUND_PLATE_H
    base.visual(
        mesh_from_geometry(
            BoxGeometry((PEDESTAL_W, PEDESTAL_D, PEDESTAL_H)),
            "pedestal",
        ),
        origin=Origin(xyz=(0.0, 0.0, ped_z0 + PEDESTAL_H / 2)),
        material=FOREST_GREEN,
        name="pedestal",
    )

    # Pivot bracket plates (two uprights flanking the beam)
    bracket_z0 = ped_z0 + PEDESTAL_H
    base.visual(
        mesh_from_geometry(
            BoxGeometry((BRACKET_W, BRACKET_T, BRACKET_H)),
            "bracket_0",
        ),
        origin=Origin(xyz=(0.0, BRACKET_Y, bracket_z0 + BRACKET_H / 2)),
        material=FOREST_GREEN,
        name="bracket_0",
    )
    base.visual(
        mesh_from_geometry(
            BoxGeometry((BRACKET_W, BRACKET_T, BRACKET_H)),
            "bracket_1",
        ),
        origin=Origin(xyz=(0.0, -BRACKET_Y, bracket_z0 + BRACKET_H / 2)),
        material=FOREST_GREEN,
        name="bracket_1",
    )

    # Cross-tie between bracket plates at the top (stiffener)
    tie = BoxGeometry((0.06, BRACKET_Y * 2 - BRACKET_T, 0.018))
    base.visual(
        mesh_from_geometry(tie, "bracket_tie"),
        origin=Origin(xyz=(0.0, 0.0, bracket_z0 + BRACKET_H - 0.009)),
        material=FOREST_GREEN,
        name="bracket_tie",
    )

    # Visible coil spring between bracket plates (spring-assist mechanism).
    # Spring sits on the pedestal top so it is connected to the base structure.
    # Height is limited so the spring stays below the beam pivot hub.
    spring_z0 = bracket_z0 - 0.002
    spring_height = 0.028  # stays below hub at PIVOT_Z
    spring_mesh = _helix_spring(0.030, spring_height, 4, 0.004)
    base.visual(
        mesh_from_geometry(spring_mesh, "coil_spring"),
        origin=Origin(xyz=(0.0, 0.0, spring_z0)),
        material=SPRING_STEEL,
        name="coil_spring",
    )

    # Spring anchor disc (connects spring base firmly to pedestal top)
    anchor = CylinderGeometry(0.036, 0.006, radial_segments=16)
    base.visual(
        mesh_from_geometry(anchor, "spring_anchor"),
        origin=Origin(xyz=(0.0, 0.0, bracket_z0 - 0.003)),
        material=STEEL_GRAY,
        name="spring_anchor",
    )

    # ── Beam (rocking beam with seats and handles) ──────────────────────
    beam = model.part("beam")

    # Main beam tube
    tube = CylinderGeometry(TUBE_R, BEAM_LEN, radial_segments=18)
    tube.rotate_y(math.pi / 2.0)
    beam.visual(
        mesh_from_geometry(tube, "beam_tube"),
        material=CHERRY_RED,
        name="beam_tube",
    )

    # Central pivot hub (wraps through the bracket plates as a bearing surface)
    hub = CylinderGeometry(0.036, 0.112, radial_segments=20)
    hub.rotate_x(math.pi / 2.0)
    beam.visual(
        mesh_from_geometry(hub, "pivot_hub"),
        material=STEEL_GRAY,
        name="pivot_hub",
    )

    # End caps on the beam tube (finished tube ends)
    for sx in (1.0, -1.0):
        cap = SphereGeometry(TUBE_R, width_segments=10, height_segments=6)
        cap.translate(sx * BEAM_LEN / 2, 0.0, 0.0)
        beam.visual(
            mesh_from_geometry(cap, f"end_cap_{0 if sx > 0 else 1}"),
            material=CHERRY_RED,
            name=f"end_cap_{0 if sx > 0 else 1}",
        )

    # Molded seats at beam ends
    for idx, sx in enumerate((1.0, -1.0)):
        seat_mesh = _molded_seat_mesh()
        beam.visual(
            mesh_from_geometry(seat_mesh, f"seat_{idx}"),
            origin=Origin(xyz=(sx * SEAT_X, 0.0, TUBE_R)),
            material=MOLDED_BLUE,
            name=f"seat_{idx}",
        )

    # Handle grips inboard of each seat
    for idx, sx in enumerate((1.0, -1.0)):
        handle_mesh = _handle_grip_mesh()
        beam.visual(
            mesh_from_geometry(handle_mesh, f"handle_{idx}"),
            origin=Origin(xyz=(sx * HANDLE_X, 0.0, TUBE_R - 0.003)),
            material=STEEL_GRAY,
            name=f"handle_{idx}",
        )
        # Rubber grip overlay for distinct material
        grip = CylinderGeometry(0.019, 0.12, radial_segments=14)
        grip.rotate_x(math.pi / 2.0)
        beam.visual(
            mesh_from_geometry(grip, f"rubber_grip_{idx}"),
            origin=Origin(xyz=(sx * HANDLE_X, 0.0, TUBE_R - 0.003 + 0.28)),
            material=RUBBER_BLACK,
            name=f"rubber_grip_{idx}",
        )

    # ── Locking Pin ─────────────────────────────────────────────────────
    pin = model.part("locking_pin")

    # Pin shaft (aligned along Y for sliding in/out)
    shaft = CylinderGeometry(0.010, 0.14, radial_segments=12)
    shaft.rotate_x(math.pi / 2.0)
    pin.visual(
        mesh_from_geometry(shaft, "pin_shaft"),
        material=SAFETY_YELLOW,
        name="pin_shaft",
    )

    # Pin grip knob (sphere at the outer end)
    knob = SphereGeometry(0.022, width_segments=12, height_segments=8)
    knob.translate(0.0, -0.08, 0.0)
    pin.visual(
        mesh_from_geometry(knob, "pin_knob"),
        material=SAFETY_YELLOW,
        name="pin_knob",
    )

    # ── Articulations ───────────────────────────────────────────────────

    # Beam pivot: revolute, horizontal Y axis, +/- 18 degrees
    model.articulation(
        "beam_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=beam,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=150.0, velocity=2.5, lower=-TILT, upper=TILT
        ),
    )

    # Locking pin: prismatic, slides along +Y (into bracket), 0-60 mm.
    # Placed near the top of the brackets, above the spring.
    pin_z = bracket_z0 + BRACKET_H - 0.025
    model.articulation(
        "locking_pin_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=pin,
        origin=Origin(
            xyz=(0.0, -(BRACKET_Y + BRACKET_T / 2 + 0.020), pin_z)
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0, velocity=0.5, lower=0.0, upper=0.06
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    beam = object_model.get_part("beam")
    pin = object_model.get_part("locking_pin")
    pivot = object_model.get_articulation("beam_pivot")
    pin_slide = object_model.get_articulation("locking_pin_slide")

    # ── Joint type and limit checks ────────────────────────────────────

    ctx.check(
        "beam pivot is revolute",
        pivot.articulation_type == ArticulationType.REVOLUTE,
    )
    lim = pivot.motion_limits
    ctx.check(
        "beam pivot rocks ±18 degrees",
        lim is not None
        and lim.lower is not None
        and lim.upper is not None
        and abs(lim.lower + TILT) < 1e-6
        and abs(lim.upper - TILT) < 1e-6,
        details=f"limits=({lim.lower}, {lim.upper})",
    )

    ctx.check(
        "locking pin is prismatic (non-fixed joint)",
        pin_slide.articulation_type == ArticulationType.PRISMATIC,
    )
    plim = pin_slide.motion_limits
    ctx.check(
        "locking pin has positive slide range ≥ 30 mm",
        plim is not None
        and plim.lower is not None
        and plim.upper is not None
        and plim.upper - plim.lower >= 0.03,
        details=f"pin limits=({plim.lower}, {plim.upper})",
    )

    # ── Molded seats with raised lips ──────────────────────────────────

    for idx in (0, 1):
        seat_aabb = ctx.part_element_world_aabb(beam, elem=f"seat_{idx}")
        ctx.check(
            f"molded seat {idx} exists on beam",
            seat_aabb is not None,
            details=f"seat_{idx} aabb={seat_aabb}",
        )
        if seat_aabb is not None:
            seat_height = seat_aabb[1][2] - seat_aabb[0][2]
            ctx.check(
                f"seat {idx} has raised lips (height > 25 mm)",
                seat_height > 0.025,
                details=f"seat height={seat_height:.4f} m",
            )

    # ── Rounded handle grips ───────────────────────────────────────────

    for idx in (0, 1):
        handle_aabb = ctx.part_element_world_aabb(beam, elem=f"handle_{idx}")
        ctx.check(
            f"handle grip {idx} exists on beam",
            handle_aabb is not None,
            details=f"handle_{idx} aabb={handle_aabb}",
        )
        if handle_aabb is not None:
            handle_height = handle_aabb[1][2] - handle_aabb[0][2]
            ctx.check(
                f"handle {idx} is tall enough for gripping (> 200 mm)",
                handle_height > 0.20,
                details=f"handle height={handle_height:.4f} m",
            )
            handle_width = handle_aabb[1][1] - handle_aabb[0][1]
            ctx.check(
                f"handle {idx} has rounded end caps (width > 150 mm)",
                handle_width > 0.15,
                details=f"handle width={handle_width:.4f} m",
            )

    # ── Spring mechanism visible on base ───────────────────────────────

    spring_aabb = ctx.part_element_world_aabb(base, elem="coil_spring")
    ctx.check(
        "base has visible coil spring (spring-assisted mechanism)",
        spring_aabb is not None,
    )

    # ── Locking pin near central bracket ───────────────────────────────

    pin_aabb = ctx.part_world_aabb(pin)
    bracket_aabb = ctx.part_element_world_aabb(base, elem="bracket_0")
    ctx.check(
        "locking pin is near the central bracket area",
        pin_aabb is not None
        and bracket_aabb is not None
        and abs(
            (pin_aabb[0][2] + pin_aabb[1][2]) / 2
            - (bracket_aabb[0][2] + bracket_aabb[1][2]) / 2
        )
        < 0.20,
        details=f"pin z center={(pin_aabb[0][2] + pin_aabb[1][2]) / 2:.3f}, "
        f"bracket z center={(bracket_aabb[0][2] + bracket_aabb[1][2]) / 2:.3f}",
    )

    # ── Base rests on ground ──────────────────────────────────────────

    base_aabb = ctx.part_world_aabb(base)
    ctx.check(
        "base rests on ground plane",
        base_aabb is not None and -0.01 <= base_aabb[0][2] <= 0.02,
        details=f"base aabb={base_aabb}",
    )

    # ── Decisive pose: beam seesaws ───────────────────────────────────

    rest_s0 = ctx.part_element_world_aabb(beam, elem="seat_0")
    rest_s1 = ctx.part_element_world_aabb(beam, elem="seat_1")
    with ctx.pose({pivot: TILT}):
        tilt_s0 = ctx.part_element_world_aabb(beam, elem="seat_0")
        tilt_s1 = ctx.part_element_world_aabb(beam, elem="seat_1")
        ctx.check(
            "beam seesaws: seat 0 drops and seat 1 rises at positive tilt",
            rest_s0 is not None
            and tilt_s0 is not None
            and rest_s1 is not None
            and tilt_s1 is not None
            and tilt_s0[0][2] < rest_s0[0][2] - 0.20
            and tilt_s1[0][2] > rest_s1[0][2] + 0.20,
            details=f"seat0 {rest_s0}->{tilt_s0}, seat1 {rest_s1}->{tilt_s1}",
        )
        beam_aabb = ctx.part_world_aabb(beam)
        ctx.check(
            "tilted beam stays above ground",
            beam_aabb is not None and beam_aabb[0][2] > 0.02,
            details=f"beam aabb={beam_aabb}",
        )

    # ── Decisive pose: locking pin slides ─────────────────────────────

    rest_pin_pos = ctx.part_world_position(pin)
    with ctx.pose({pin_slide: 0.05}):
        ext_pin_pos = ctx.part_world_position(pin)
        ctx.check(
            "locking pin slides inward when actuated",
            rest_pin_pos is not None
            and ext_pin_pos is not None
            and ext_pin_pos[1] > rest_pin_pos[1] + 0.02,
            details=f"rest={rest_pin_pos}, extended={ext_pin_pos}",
        )

    # ── Overlap allowances ─────────────────────────────────────────────

    # The pivot hub wraps through the bracket plates (bearing surface).
    ctx.allow_overlap(
        beam,
        base,
        elem_a="pivot_hub",
        elem_b="bracket_0",
        reason="Pivot hub intentionally wraps through bracket plate as bearing surface.",
    )
    ctx.allow_overlap(
        beam,
        base,
        elem_a="pivot_hub",
        elem_b="bracket_1",
        reason="Pivot hub intentionally wraps through bracket plate as bearing surface.",
    )
    ctx.expect_contact(
        beam,
        base,
        elem_a="pivot_hub",
        elem_b="bracket_0",
        name="beam hub contacts bracket plate (bearing support)",
    )

    # The locking pin shaft slides through the bracket plate — intentional.
    ctx.allow_overlap(
        base,
        pin,
        elem_a="bracket_1",
        elem_b="pin_shaft",
        reason="Locking pin shaft intentionally slides through the bracket plate hole.",
    )
    ctx.expect_contact(
        pin,
        base,
        elem_a="pin_shaft",
        elem_b="bracket_1",
        name="locking pin shaft passes through the bracket plate",
    )

    return ctx.report()


object_model = build_object_model()
