from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Shared dimensions (meters). World: X along seesaw length, Z up.
# ---------------------------------------------------------------------------
PIVOT_Z = 0.40          # world height of the rocking axis
BEAM_LEN = 2.40         # main beam length
BEAM_W = 0.120          # beam width (Y)
BEAM_H = 0.080          # beam height (Z)
BEAM_CZ = 0.04          # beam centerline z-offset in rocker frame

SEAT_X = 1.00           # seat center x-position in rocker frame
SEAT_W = 0.360          # seat width (along beam length)
SEAT_D = 0.300          # seat depth (across beam)
SEAT_T = 0.015          # seat base plate thickness
SEAT_LIP_H = 0.030     # raised-lip height above base
SEAT_LIP_W = 0.022     # lip wall width

BUMPER_X = 1.05         # bumper center x-position in rocker frame
BUMPER_R = 0.030        # bumper radius
BUMPER_H = 0.050        # bumper height

HANDLE_X = 0.90         # handle post x-position in rocker frame
HANDLE_TOP_Z = 0.50    # grip plate center z in rocker frame

ROCK_LIMIT = 0.262      # ~15 degrees each way

PED_R = 0.080           # pedestal radius
PED_H = 0.22            # pedestal height
BRK_W = 0.18            # bracket width (X)
BRK_D = 0.15            # bracket depth (Y)
BRK_H = 0.19            # bracket height (Z)
BRK_CZ = 0.295          # bracket center z in world

STUB_R = 0.042          # pivot stub radius
STUB_LEN = 0.12         # pivot stub length


def _make_molded_seat() -> object:
    """Build a molded seat with raised lips using CadQuery.

    The seat is a solid block with a rectangular pocket cut from the top face,
    leaving raised perimeter walls (lips) that help keep the rider seated.
    """
    total_h = SEAT_T + SEAT_LIP_H
    inner_w = SEAT_W - 2.0 * SEAT_LIP_W
    inner_d = SEAT_D - 2.0 * SEAT_LIP_W
    seat = (
        cq.Workplane("XY")
        .box(SEAT_W, SEAT_D, total_h)
        .faces(">Z").workplane()
        .rect(inner_w, inner_d)
        .cutBlind(-SEAT_LIP_H)
    )
    # Fillet outer top edges for a smooth molded appearance
    seat = seat.edges(">Z").fillet(0.005)
    return seat


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="playground_seesaw")

    model.material("steel_beam", rgba=(0.35, 0.37, 0.40, 1.0))
    model.material("rubber_black", rgba=(0.05, 0.05, 0.06, 1.0))
    model.material("matte_black", rgba=(0.09, 0.09, 0.10, 1.0))
    model.material("light_gray", rgba=(0.78, 0.78, 0.79, 1.0))
    model.material("molded_green", rgba=(0.15, 0.30, 0.18, 1.0))
    model.material("silver_bolt", rgba=(0.74, 0.75, 0.78, 1.0))
    model.material("grip_dark", rgba=(0.22, 0.24, 0.26, 1.0))

    # -----------------------------------------------------------------
    # Fixed base: light-gray ground pedestal + black cast pivot bracket.
    # -----------------------------------------------------------------
    base = model.part("pedestal_mount")
    base.visual(
        Cylinder(radius=PED_R, length=PED_H),
        origin=Origin(xyz=(0.0, 0.0, PED_H / 2.0)),
        material="light_gray",
        name="ground_pedestal",
    )
    base.visual(
        Box((BRK_W, BRK_D, BRK_H)),
        origin=Origin(xyz=(0.0, 0.0, BRK_CZ)),
        material="matte_black",
        name="pivot_bracket",
    )
    # Pivot bosses on bracket cheeks with visible bolts
    for i, sy in enumerate((1.0, -1.0)):
        base.visual(
            Cylinder(radius=0.048, length=0.018),
            origin=Origin(
                xyz=(0.0, sy * (BRK_D / 2.0 + 0.009), PIVOT_Z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material="matte_black",
            name=f"pivot_boss_{i}",
        )
        for j, ang in enumerate((0.25, 0.75, 1.25, 1.75)):
            dx = 0.028 * math.cos(ang * math.pi)
            dz = 0.028 * math.sin(ang * math.pi)
            base.visual(
                Cylinder(radius=0.007, length=0.010),
                origin=Origin(
                    xyz=(dx, sy * (BRK_D / 2.0 + 0.020), PIVOT_Z + dz),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material="silver_bolt",
                name=f"bracket_bolt_{i}_{j}",
            )

    # -----------------------------------------------------------------
    # Rocker: straight heavy steel beam + pivot stub + mirrored
    # seat/bumper/handle assemblies at each end.
    # Part frame sits on the pivot axis so the revolute joint needs
    # no extra offset.
    # -----------------------------------------------------------------
    rocker = model.part("rocker")

    # Heavy straight steel beam with filleted longitudinal edges.
    beam_cq = (
        cq.Workplane("XY")
        .box(BEAM_LEN, BEAM_W, BEAM_H)
        .edges("|X").fillet(0.004)
    )
    rocker.visual(
        mesh_from_cadquery(beam_cq, "main_beam"),
        origin=Origin(xyz=(0.0, 0.0, BEAM_CZ)),
        material="steel_beam",
        name="main_beam",
    )

    # End cap plates at beam extremities
    for i, s in enumerate((1.0, -1.0)):
        rocker.visual(
            Box((0.006, BEAM_W - 0.006, BEAM_H - 0.006)),
            origin=Origin(xyz=(s * (BEAM_LEN / 2.0 - 0.003), 0.0, BEAM_CZ)),
            material="steel_beam",
            name=f"beam_endcap_{i}",
        )

    # Short steel stub descending from beam into the black bracket.
    stub_cz = BEAM_CZ - BEAM_H / 2.0 - STUB_LEN / 2.0
    rocker.visual(
        Cylinder(radius=STUB_R, length=STUB_LEN),
        origin=Origin(xyz=(0.0, 0.0, stub_cz)),
        material="steel_beam",
        name="pivot_stub",
    )

    # Pre-build shared CadQuery shapes
    seat_cq = _make_molded_seat()
    seat_total_h = SEAT_T + SEAT_LIP_H
    beam_top_z = BEAM_CZ + BEAM_H / 2.0
    beam_bottom_z = BEAM_CZ - BEAM_H / 2.0

    # Seat center: slightly embedded into beam top for connectivity
    seat_center_z = beam_top_z + seat_total_h / 2.0 - 0.003

    # Bumper center: slightly embedded into beam bottom for connectivity
    bumper_cz = beam_bottom_z - BUMPER_H / 2.0 + 0.005

    # Handle post: from beam top to grip height
    post_bottom_z = beam_top_z - 0.005  # slight embed
    post_len = HANDLE_TOP_Z - post_bottom_z

    # Grip plate CadQuery shape (rounded rectangle)
    grip_cq = (
        cq.Workplane("XY")
        .box(0.140, 0.220, 0.012)
        .edges("|Z").fillet(0.018)
    )

    for i, s in enumerate((1.0, -1.0)):
        # --- Molded seat with raised lips on beam top ---
        rocker.visual(
            mesh_from_cadquery(seat_cq, f"molded_seat_{i}"),
            origin=Origin(xyz=(s * SEAT_X, 0.0, seat_center_z)),
            material="molded_green",
            name=f"molded_seat_{i}",
        )

        # --- Rubber bumper under beam near end ---
        rocker.visual(
            Cylinder(radius=BUMPER_R, length=BUMPER_H),
            origin=Origin(xyz=(s * BUMPER_X, 0.0, bumper_cz)),
            material="rubber_black",
            name=f"rubber_bumper_{i}",
        )

        # --- Handle post rising from beam to grip plate ---
        rocker.visual(
            Cylinder(radius=0.016, length=post_len),
            origin=Origin(xyz=(s * HANDLE_X, 0.0, post_bottom_z + post_len / 2.0)),
            material="steel_beam",
            name=f"handle_post_{i}",
        )

        # --- Grip plate at top of handle post ---
        grip_z = HANDLE_TOP_Z - 0.003  # slight embed into post top
        rocker.visual(
            mesh_from_cadquery(grip_cq, f"grip_plate_{i}"),
            origin=Origin(xyz=(s * HANDLE_X, 0.0, grip_z)),
            material="grip_dark",
            name=f"grip_plate_{i}",
        )

        # --- Seat mounting bolts on the lip surface ---
        lip_cx = SEAT_W / 2.0 - SEAT_LIP_W / 2.0
        lip_cy = SEAT_D / 2.0 - SEAT_LIP_W / 2.0
        bolt_z = seat_center_z + seat_total_h / 2.0 - 0.001
        for j, (bx, by) in enumerate(
            [(lip_cx, lip_cy), (lip_cx, -lip_cy), (-lip_cx, lip_cy), (-lip_cx, -lip_cy)]
        ):
            rocker.visual(
                Cylinder(radius=0.005, length=0.006),
                origin=Origin(xyz=(s * SEAT_X + s * bx, by, bolt_z)),
                material="silver_bolt",
                name=f"seat_bolt_{i}_{j}",
            )

    # Single rocking pivot: horizontal axis perpendicular to seesaw length.
    model.articulation(
        "rocker_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=rocker,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=400.0, velocity=1.5, lower=-ROCK_LIMIT, upper=ROCK_LIMIT,
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

    # The steel pivot stub is intentionally captured inside the black bracket.
    ctx.allow_overlap(
        rocker, base,
        elem_a="pivot_stub", elem_b="pivot_bracket",
        reason="The steel center stub descends into the cast pivot bracket that captures the rocking axle.",
    )
    ctx.expect_overlap(
        rocker, base, axes="z",
        elem_a="pivot_stub", elem_b="pivot_bracket",
        min_overlap=0.03,
        name="pivot stub inserted into bracket",
    )
    ctx.expect_within(
        rocker, base, axes="xy",
        inner_elem="pivot_stub", outer_elem="pivot_bracket",
        margin=0.0,
        name="pivot stub centered in bracket",
    )

    # Bracket seated on the pedestal.
    bracket = ctx.part_element_world_aabb(base, elem="pivot_bracket")
    pedestal = ctx.part_element_world_aabb(base, elem="ground_pedestal")
    ctx.check(
        "bracket sits atop ground pedestal",
        _intersects(bracket, pedestal),
        details=f"bracket={bracket}, pedestal={pedestal}",
    )

    # --- Heavy straight beam checks ---
    beam = ctx.part_element_world_aabb(rocker, elem="main_beam")
    ctx.check(
        "steel beam spans the seesaw length",
        beam is not None and (beam[1][0] - beam[0][0]) >= 2.2,
        details=f"beam={beam}",
    )
    ctx.check(
        "beam is straight (not a curved banana tube)",
        beam is not None and (beam[1][2] - beam[0][2]) < 0.12,
        details=f"beam z-range={None if beam is None else beam[1][2] - beam[0][2]:.4f}",
    )
    ctx.check(
        "beam has heavy cross-section (wide and tall)",
        beam is not None
        and (beam[1][1] - beam[0][1]) >= 0.10
        and (beam[1][2] - beam[0][2]) >= 0.06,
        details=f"beam y={None if beam is None else beam[1][1] - beam[0][1]:.3f}, "
                f"z={None if beam is None else beam[1][2] - beam[0][2]:.3f}",
    )

    # --- Rubber bumper checks ---
    bumper0 = ctx.part_element_world_aabb(rocker, elem="rubber_bumper_0")
    bumper1 = ctx.part_element_world_aabb(rocker, elem="rubber_bumper_1")
    ctx.check(
        "rubber bumpers present under beam near both ends",
        bumper0 is not None and bumper1 is not None,
        details=f"bumper0={bumper0}, bumper1={bumper1}",
    )
    ctx.check(
        "bumpers are below beam bottom surface",
        bumper0 is not None and bumper1 is not None and beam is not None
        and bumper0[1][2] <= beam[0][2] + 0.010
        and bumper1[1][2] <= beam[0][2] + 0.010,
        details=f"bumper0 top={None if bumper0 is None else bumper0[1][2]:.3f}, "
                f"bumper1 top={None if bumper1 is None else bumper1[1][2]:.3f}, "
                f"beam bottom={None if beam is None else beam[0][2]:.3f}",
    )

    # --- Molded seat with raised-lip checks ---
    seat0 = ctx.part_element_world_aabb(rocker, elem="molded_seat_0")
    seat1 = ctx.part_element_world_aabb(rocker, elem="molded_seat_1")
    ctx.check(
        "molded seats present at beam ends",
        seat0 is not None and seat1 is not None,
        details=f"seat0={seat0}, seat1={seat1}",
    )
    ctx.check(
        "molded seats have raised lips (vertical extent > 25mm)",
        seat0 is not None and seat1 is not None
        and (seat0[1][2] - seat0[0][2]) > 0.025
        and (seat1[1][2] - seat1[0][2]) > 0.025,
        details=f"seat0 h={None if seat0 is None else seat0[1][2] - seat0[0][2]:.3f}, "
                f"seat1 h={None if seat1 is None else seat1[1][2] - seat1[0][2]:.3f}",
    )
    ctx.check(
        "seats sit on top of beam",
        seat0 is not None and seat1 is not None and beam is not None
        and seat0[0][2] >= beam[1][2] - 0.015
        and seat1[0][2] >= beam[1][2] - 0.015,
        details=f"seat0 bottom={None if seat0 is None else seat0[0][2]:.3f}, "
                f"beam top={None if beam is None else beam[1][2]:.3f}",
    )

    # Mirrored ends
    def _cx(aabb):
        return 0.5 * (aabb[0][0] + aabb[1][0])

    ctx.check(
        "seats mirrored about pivot",
        seat0 is not None and seat1 is not None
        and _cx(seat0) > 0.8 and _cx(seat1) < -0.8
        and abs(_cx(seat0) + _cx(seat1)) < 0.05,
        details=f"seat0 cx={None if seat0 is None else _cx(seat0):.3f}, "
                f"seat1 cx={None if seat1 is None else _cx(seat1):.3f}",
    )
    ctx.check(
        "bumpers mirrored about pivot",
        bumper0 is not None and bumper1 is not None
        and abs(_cx(bumper0) + _cx(bumper1)) < 0.05,
        details=f"bumper0 cx={None if bumper0 is None else _cx(bumper0):.3f}, "
                f"bumper1 cx={None if bumper1 is None else _cx(bumper1):.3f}",
    )

    # --- Joint checks: non-fixed revolute ---
    lim = pivot.motion_limits
    ctx.check(
        "revolute joint is non-fixed (has motion range)",
        lim is not None and (lim.upper - lim.lower) > 0.1,
        details=f"range={None if lim is None else lim.upper - lim.lower:.3f}",
    )
    ctx.check(
        "rocking range about +/- 15 degrees",
        lim is not None
        and abs(lim.lower + ROCK_LIMIT) < 0.02
        and abs(lim.upper - ROCK_LIMIT) < 0.02,
        details=f"limits=({lim.lower}, {lim.upper})",
    )

    # Overall envelope
    ra = ctx.part_world_aabb(rocker)
    ctx.check(
        "overall length about 2.4 m",
        ra is not None and 2.2 <= (ra[1][0] - ra[0][0]) <= 2.8,
        details=f"rocker aabb={ra}",
    )

    # --- Decisive pose checks ---
    base_rest = ctx.part_world_aabb(base)
    with ctx.pose({pivot: ROCK_LIMIT}):
        seat0_dn = ctx.part_element_world_aabb(rocker, elem="molded_seat_0")
        seat1_up = ctx.part_element_world_aabb(rocker, elem="molded_seat_1")
        rocker_dn = ctx.part_world_aabb(rocker)
        base_posed = ctx.part_world_aabb(base)

        ctx.check(
            "positive rock lowers seat_0 and raises seat_1",
            seat0_dn is not None and seat1_up is not None
            and seat0 is not None and seat1 is not None
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
            base_rest is not None and base_posed is not None
            and abs(base_rest[1][2] - base_posed[1][2]) < 1e-6,
            details=f"rest={base_rest}, posed={base_posed}",
        )

    with ctx.pose({pivot: -ROCK_LIMIT}):
        seat0_up = ctx.part_element_world_aabb(rocker, elem="molded_seat_0")
        rocker_up = ctx.part_world_aabb(rocker)
        ctx.check(
            "negative rock raises seat_0",
            seat0_up is not None and seat0 is not None
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
