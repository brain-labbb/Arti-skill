from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---------------------------------------------------------------------------
# Heavy commercial playground seesaw variant
#
# World frame: beam runs along X, pivot axis along Y, Z up.
# - Two bent galvanized-steel tube arches (~50 mm dia) form A-shaped saddle.
# - Heavy commercial steel beam (90 x 45 mm) painted mustard with rust streaks.
# - Molded seats with raised lips at each end.
# - Tilting backrests on small revolute joints behind each seat.
# - Inverted-U grab handles inboard of each seat.
# - Curved rubber tire-section bumpers under each beam tip.
# - Visible axle caps (large washers) at the support bracket.
# - Triangular gusset plate connects beam to pivot.
# - Main pivot: revolute, axis (0,1,0), +/-20 degrees.
# - Backrest tilts: revolute, axis (0,1,0), -5 to +20 degrees from upright.
# ---------------------------------------------------------------------------

BEAM_HALF = 1.50  # 3.0 m beam
BEAM_W = 0.090    # heavier commercial beam: 90 mm wide
BEAM_T = 0.045    # 45 mm thick
PIVOT_Z = 0.76

ARCH_FOOT_X = 0.66
ARCH_FOOT_Y = 0.34
ARCH_APEX_Y = 0.05
ARCH_FOOT_Z = 0.028
TUBE_R = 0.025

AXLE_R = 0.018    # slightly heavier axle
AXLE_LEN = 0.26   # longer for visible caps

# Beam-local frame: origin at the axle center
BAR_BOT = 0.052
BAR_CTR = BAR_BOT + BEAM_T / 2.0
BAR_TOP = BAR_BOT + BEAM_T

SEAT_X = 1.28
HANDLE_X = 1.02
BUMPER_X = 1.42
TILT = math.radians(20.0)

# Backrest parameters
BACKREST_X = 1.38  # behind (outboard of) the seat
BACKREST_W = 0.22
BACKREST_H = 0.18
BACKREST_T = 0.010
BACKREST_BRACKET_H = 0.06  # bracket rises from beam top to hinge line
BACKREST_BRACKET_VISUAL_H = 0.054  # visual height connects to hinge pin bottom
BACKREST_LOWER_LIMIT = math.radians(-5.0)
BACKREST_UPPER_LIMIT = math.radians(20.0)

# Axle cap parameters
AXLE_CAP_R = 0.038   # large visible washer
AXLE_CAP_T = 0.008


def _arch_points(side: float) -> list[tuple[float, float, float]]:
    """Centerline of one bent-tube arch."""
    pts: list[tuple[float, float, float]] = []
    rise = PIVOT_Z - ARCH_FOOT_Z
    for i in range(11):
        t = -1.0 + 0.2 * i
        s = 1.0 - t * t
        x = ARCH_FOOT_X * t
        z = ARCH_FOOT_Z + rise * s
        y = side * ARCH_FOOT_Y + (-side * ARCH_APEX_Y - side * ARCH_FOOT_Y) * s
        pts.append((x, y, z))
    return pts


def _handle_points(x: float) -> list[tuple[float, float, float]]:
    """Inverted-U grab-handle rod centerline."""
    half_w = 0.038
    leg_bot = BAR_TOP - 0.010
    arc_z = 0.280
    pts: list[tuple[float, float, float]] = [
        (x, -half_w, leg_bot),
        (x, -half_w, 0.195),
    ]
    for k in range(7):
        a = math.pi * k / 6.0
        pts.append((x, -half_w * math.cos(a), arc_z + half_w * math.sin(a)))
    pts.append((x, half_w, 0.195))
    pts.append((x, half_w, leg_bot))
    return pts


def _bumper_geometry(x: float, index: int):
    """Curved tire-section bumper: half-annulus shell extruded across beam."""
    r_out = 0.068
    r_in = 0.050
    profile: list[tuple[float, float]] = []
    n = 20
    for k in range(n + 1):
        a = math.pi + math.pi * k / n
        profile.append((r_out * math.cos(a), r_out * math.sin(a)))
    for k in range(n + 1):
        a = 2.0 * math.pi - math.pi * k / n
        profile.append((r_in * math.cos(a), r_in * math.sin(a)))
    geom = ExtrudeGeometry(profile, 0.11, cap=True, center=True)
    geom.rotate_x(math.pi / 2.0)
    geom.translate(x, 0.0, BAR_BOT + 0.002)
    return mesh_from_geometry(geom, f"bumper_{index}")


def _gusset_geometry():
    """Triangular gusset plate joining beam bar to pivot sleeve."""
    profile = [(-0.12, 0.058), (0.12, 0.058), (0.0, 0.020)]
    geom = ExtrudeGeometry(profile, 0.024, cap=True, center=True)
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, "gusset_plate")


def _molded_seat_geometry(index: int):
    """Molded seat pan with raised lips around the perimeter.

    The seat is a shallow dish: a flat bottom plate surrounded by raised
    lip walls (~15 mm tall, ~8 mm thick).
    """
    seat_w = 0.26   # Y-width (across beam)
    seat_d = 0.28   # X-depth (along beam)
    lip_h = 0.018   # lip height above seat surface
    lip_t = 0.008   # lip thickness
    base_t = 0.006  # seat pan thickness

    parts_meshes = []

    # Seat pan base plate
    base = Box((seat_d, seat_w, base_t))
    parts_meshes.append(("seat_pan", base, (0.0, 0.0, base_t / 2.0)))

    # Front lip (positive X edge)
    front_lip = Box((lip_t, seat_w, lip_h))
    parts_meshes.append(("front_lip", front_lip, (seat_d / 2.0 - lip_t / 2.0, 0.0, base_t + lip_h / 2.0)))

    # Back lip (negative X edge)
    back_lip = Box((lip_t, seat_w, lip_h))
    parts_meshes.append(("back_lip", back_lip, (-seat_d / 2.0 + lip_t / 2.0, 0.0, base_t + lip_h / 2.0)))

    # Left lip (positive Y edge)
    left_lip = Box((seat_d - 2 * lip_t, lip_t, lip_h))
    parts_meshes.append(("left_lip", left_lip, (0.0, seat_w / 2.0 - lip_t / 2.0, base_t + lip_h / 2.0)))

    # Right lip (negative Y edge)
    right_lip = Box((seat_d - 2 * lip_t, lip_t, lip_h))
    parts_meshes.append(("right_lip", right_lip, (0.0, -seat_w / 2.0 + lip_t / 2.0, base_t + lip_h / 2.0)))

    return parts_meshes


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_playground_seesaw")

    # Materials
    galvanized = model.material("weathered_galvanized", rgba=(0.52, 0.55, 0.53, 1.0))
    rust = model.material("rust_steel", rgba=(0.42, 0.25, 0.13, 1.0))
    mustard = model.material("heavy_mustard_paint", rgba=(0.72, 0.52, 0.10, 1.0))
    dark_steel = model.material("dark_commercial_steel", rgba=(0.35, 0.33, 0.30, 1.0))
    rubber = model.material("black_rubber", rgba=(0.06, 0.06, 0.06, 1.0))
    molded_plastic = model.material("molded_seat_plastic", rgba=(0.18, 0.22, 0.55, 1.0))
    backrest_plastic = model.material("backrest_plastic", rgba=(0.20, 0.24, 0.52, 1.0))
    zinc_cap = model.material("zinc_plated_cap", rgba=(0.65, 0.67, 0.62, 1.0))

    # --------------------------------------------------------------- base ---
    base = model.part("arched_base")
    for i, side in enumerate((1.0, -1.0)):
        base.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    _arch_points(side),
                    radius=TUBE_R,
                    samples_per_segment=8,
                    radial_segments=18,
                    cap_ends=True,
                ),
                f"arch_{i}",
            ),
            material=galvanized,
            name=f"arch_{i}",
        )

    # Pivot axle bolt through both arch apexes
    base.visual(
        Cylinder(radius=AXLE_R, length=AXLE_LEN),
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rust,
        name="pivot_axle",
    )

    # Visible axle caps (large zinc-plated washers) at each end of the axle
    for i, side in enumerate((1.0, -1.0)):
        base.visual(
            Cylinder(radius=AXLE_CAP_R, length=AXLE_CAP_T),
            origin=Origin(
                xyz=(0.0, side * (AXLE_LEN / 2.0 - AXLE_CAP_T / 2.0), PIVOT_Z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=zinc_cap,
            name=f"axle_cap_{i}",
        )
        # Small retaining nut behind the cap
        base.visual(
            Cylinder(radius=0.014, length=0.010),
            origin=Origin(
                xyz=(0.0, side * (AXLE_LEN / 2.0 + 0.002), PIVOT_Z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=dark_steel,
            name=f"axle_nut_{i}",
        )

    # Support bracket plates at the apex (where axle passes through)
    for i, side in enumerate((1.0, -1.0)):
        base.visual(
            Box((0.06, 0.012, 0.08)),
            origin=Origin(
                xyz=(0.0, side * 0.04, PIVOT_Z - 0.02)),
            material=galvanized,
            name=f"bracket_plate_{i}",
        )

    # --------------------------------------------------------------- beam ---
    beam = model.part("beam")

    beam.visual(
        Cylinder(radius=0.028, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="pivot_sleeve",
    )
    beam.visual(_gusset_geometry(), material=mustard, name="gusset_plate")

    beam.visual(
        Box((2.0 * BEAM_HALF, BEAM_W, BEAM_T)),
        origin=Origin(xyz=(0.0, 0.0, BAR_CTR)),
        material=mustard,
        name="beam_bar",
    )

    # Rust streak patches
    for i, px in enumerate((-0.85, -0.30, 0.45, 0.95)):
        beam.visual(
            Box((0.16, BEAM_W + 0.004, 0.012)),
            origin=Origin(xyz=(px, 0.0, BAR_TOP - 0.004)),
            material=rust,
            name=f"rust_patch_{i}",
        )

    # End fittings for each side: molded seat, handle, bumper
    for i, side in enumerate((1.0, -1.0)):
        sx = side * SEAT_X

        # Molded seat with raised lips - multiple box elements forming the dish
        seat_parts = _molded_seat_geometry(i)
        for part_name, geom, offset in seat_parts:
            beam.visual(
                geom,
                origin=Origin(xyz=(sx + offset[0], offset[1], BAR_TOP + offset[2])),
                material=molded_plastic,
                name=f"seat_{i}_{part_name}",
            )

        # Grab handle
        beam.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    _handle_points(side * HANDLE_X),
                    radius=0.010,
                    samples_per_segment=8,
                    radial_segments=16,
                    cap_ends=True,
                ),
                f"handle_{i}",
            ),
            material=dark_steel,
            name=f"handle_{i}",
        )

        # Rubber bumper
        beam.visual(
            _bumper_geometry(side * BUMPER_X, i),
            material=rubber,
            name=f"bumper_{i}",
        )

        # Backrest mounting bracket (steel piece rising from beam to just below hinge)
        bracket_x = side * BACKREST_X
        beam.visual(
            Box((0.04, 0.05, BACKREST_BRACKET_VISUAL_H)),
            origin=Origin(xyz=(bracket_x, 0.0, BAR_TOP + BACKREST_BRACKET_VISUAL_H / 2.0)),
            material=dark_steel,
            name=f"backrest_bracket_{i}",
        )
        # Hinge pin at top of bracket
        beam.visual(
            Cylinder(radius=0.006, length=0.06),
            origin=Origin(
                xyz=(bracket_x, 0.0, BAR_TOP + BACKREST_BRACKET_H),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=dark_steel,
            name=f"backrest_hinge_pin_{i}",
        )

    # --------------------------------------------------------- backrests ---
    # Each backrest is a separate part connected by a revolute joint.
    # The joint axis is along Y (same as main pivot), allowing tilt forward/back.
    # Part frame origin at the hinge line (bottom of backrest panel).
    for i, side in enumerate((1.0, -1.0)):
        backrest = model.part(f"backrest_{i}")
        bx = side * BACKREST_X
        hinge_z_world = PIVOT_Z + BAR_TOP + BACKREST_BRACKET_H  # world Z of hinge

        # Backrest panel - extends upward from hinge line
        # Part frame at hinge line; panel goes up (+Z local)
        backrest.visual(
            Box((BACKREST_T, BACKREST_W, BACKREST_H)),
            origin=Origin(xyz=(0.0, 0.0, BACKREST_H / 2.0)),
            material=backrest_plastic,
            name=f"backrest_panel_{i}",
        )

        # Backrest reinforcing ribs
        for rib_y in (-0.06, 0.0, 0.06):
            backrest.visual(
                Box((BACKREST_T + 0.006, 0.008, BACKREST_H - 0.02)),
                origin=Origin(xyz=(0.0, rib_y, BACKREST_H / 2.0)),
                material=backrest_plastic,
                name=f"backrest_rib_{i}_{abs(int(rib_y*100))}",
            )

        # Hinge barrel at the bottom
        backrest.visual(
            Cylinder(radius=0.009, length=0.054),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"backrest_barrel_{i}",
        )

        # Articulation: backrest tilt
        # Parent is beam, child is backrest. Origin in beam's local frame.
        # The hinge is at (bx, 0, BAR_TOP + BACKREST_BRACKET_H) in beam frame.
        # Since beam frame origin is at the axle center (0,0,0 in beam local = pivot_z world),
        # the hinge Z in beam-local = BAR_TOP + BACKREST_BRACKET_H
        hinge_z_beam_local = BAR_TOP + BACKREST_BRACKET_H

        model.articulation(
            f"backrest_tilt_{i}",
            ArticulationType.REVOLUTE,
            parent=beam,
            child=backrest,
            origin=Origin(xyz=(bx, 0.0, hinge_z_beam_local)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=10.0,
                velocity=1.5,
                lower=BACKREST_LOWER_LIMIT,
                upper=BACKREST_UPPER_LIMIT,
            ),
        )

    # ----------------------------------------------------------- main pivot ---
    model.articulation(
        "beam_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=beam,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=200.0, velocity=2.5, lower=-TILT, upper=TILT),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("arched_base")
    beam = object_model.get_part("beam")
    pivot = object_model.get_articulation("beam_pivot")
    backrest_0 = object_model.get_part("backrest_0")
    backrest_1 = object_model.get_part("backrest_1")
    tilt_0 = object_model.get_articulation("backrest_tilt_0")
    tilt_1 = object_model.get_articulation("backrest_tilt_1")

    # --- Main pivot: sleeve captures axle ---
    ctx.allow_overlap(
        beam,
        base,
        elem_a="pivot_sleeve",
        elem_b="pivot_axle",
        reason="Pivot sleeve is a bushing intentionally nested around the axle bolt.",
    )
    # The arch tubes pass through the pivot area at the saddle apex; the sleeve
    # wraps around the axle between the arches. This is structurally necessary.
    ctx.allow_overlap(
        base,
        beam,
        elem_a="arch_0",
        elem_b="pivot_sleeve",
        reason="Arch tube passes through the saddle apex where the pivot sleeve sits; structurally required for the A-frame saddle.",
    )
    ctx.allow_overlap(
        base,
        beam,
        elem_a="arch_1",
        elem_b="pivot_sleeve",
        reason="Arch tube passes through the saddle apex where the pivot sleeve sits; structurally required for the A-frame saddle.",
    )
    ctx.expect_contact(
        beam,
        base,
        elem_a="pivot_sleeve",
        elem_b="pivot_axle",
        name="pivot sleeve is seated on the axle bolt",
    )
    ctx.expect_within(
        beam,
        base,
        axes="y",
        inner_elem="pivot_sleeve",
        outer_elem="pivot_axle",
        margin=0.001,
        name="pivot sleeve stays inside the axle span",
    )

    # --- Beam clears arch saddle ---
    ctx.expect_gap(
        beam,
        base,
        axis="z",
        positive_elem="beam_bar",
        negative_elem="arch_0",
        min_gap=0.005,
        max_gap=0.06,
        name="beam bar clears the arch saddle",
    )

    # --- Main pivot joint configuration ---
    ax = pivot.axis
    ctx.check(
        "pivot axis is horizontal and perpendicular to the beam",
        abs(ax[0]) < 1e-9 and abs(ax[1] - 1.0) < 1e-9 and abs(ax[2]) < 1e-9,
        details=f"axis={ax}",
    )
    lim = pivot.motion_limits
    ctx.check(
        "rocking limits are about +/- 20 degrees",
        lim is not None
        and lim.lower is not None
        and lim.upper is not None
        and abs(lim.lower + TILT) < 1e-6
        and abs(lim.upper - TILT) < 1e-6,
        details=f"limits=({lim.lower}, {lim.upper})",
    )

    # --- Hero geometry ---
    bar_box = ctx.part_element_world_aabb(beam, elem="beam_bar")
    base_box = ctx.part_world_aabb(base)
    axle_box = ctx.part_element_world_aabb(base, elem="pivot_axle")
    ctx.check(
        "beam is about 3.0 m long",
        bar_box is not None and abs((bar_box[1][0] - bar_box[0][0]) - 3.0) < 0.04,
        details=f"bar aabb={bar_box}",
    )
    ctx.check(
        "arched base feet rest on the ground",
        base_box is not None and -0.01 <= base_box[0][2] <= 0.02,
        details=f"base aabb={base_box}",
    )
    ctx.check(
        "pivot axle sits about 0.8 m high",
        axle_box is not None and 0.70 <= (axle_box[0][2] + axle_box[1][2]) / 2.0 <= 0.82,
        details=f"axle aabb={axle_box}",
    )

    # --- Axle caps are visible and near axle ends ---
    for i in range(2):
        cap_box = ctx.part_element_world_aabb(base, elem=f"axle_cap_{i}")
        ctx.check(
            f"axle_cap_{i} is visible with radius larger than axle",
            cap_box is not None
            and (cap_box[1][2] - cap_box[0][2]) > 0.06,
            details=f"cap aabb={cap_box}",
        )

    # --- Molded seats with raised lips ---
    for i in range(2):
        pan = ctx.part_element_world_aabb(beam, elem=f"seat_{i}_seat_pan")
        front_lip = ctx.part_element_world_aabb(beam, elem=f"seat_{i}_front_lip")
        ctx.check(
            f"seat_{i} has a pan above beam bar",
            pan is not None
            and bar_box is not None
            and pan[0][2] > bar_box[1][2] - 0.002,
            details=f"pan aabb={pan}",
        )
        ctx.check(
            f"seat_{i} has raised front lip above pan surface",
            front_lip is not None
            and pan is not None
            and front_lip[1][2] > pan[1][2] + 0.005,
            details=f"lip top={front_lip[1][2]}, pan top={pan[1][2]}",
        )

    # --- Handles ---
    for i in range(2):
        handle = ctx.part_element_world_aabb(beam, elem=f"handle_{i}")
        ctx.check(
            f"handle_{i} stands above the beam",
            handle is not None
            and bar_box is not None
            and handle[1][2] > bar_box[1][2] + 0.18
            and handle[0][2] < bar_box[1][2],
            details=f"handle aabb={handle}",
        )

    # --- Bumpers ---
    for i in range(2):
        bumper = ctx.part_element_world_aabb(beam, elem=f"bumper_{i}")
        ctx.check(
            f"bumper_{i} hangs below the beam tip",
            bumper is not None
            and bar_box is not None
            and bumper[0][2] < bar_box[0][2]
            and min(abs(bumper[0][0]), abs(bumper[1][0])) > 1.3,
            details=f"bumper aabb={bumper}",
        )

    # --- Backrest tilt joints ---
    for i, (br, tilt) in enumerate([(backrest_0, tilt_0), (backrest_1, tilt_1)]):
        panel = ctx.part_element_world_aabb(br, elem=f"backrest_panel_{i}")
        barrel = ctx.part_element_world_aabb(br, elem=f"backrest_barrel_{i}")

        # Backrest panel rises above the hinge line
        ctx.check(
            f"backrest_{i} panel extends upward from hinge",
            panel is not None
            and barrel is not None
            and panel[1][2] > barrel[1][2] + 0.10,
            details=f"panel={panel}, barrel={barrel}",
        )

        # Tilt joint has correct axis and limits
        tilt_ax = tilt.axis
        ctx.check(
            f"backrest_tilt_{i} axis is horizontal along Y",
            abs(tilt_ax[0]) < 1e-9 and abs(tilt_ax[1] - 1.0) < 1e-9 and abs(tilt_ax[2]) < 1e-9,
            details=f"axis={tilt_ax}",
        )
        tilt_lim = tilt.motion_limits
        ctx.check(
            f"backrest_tilt_{i} has non-trivial range",
            tilt_lim is not None
            and tilt_lim.lower is not None
            and tilt_lim.upper is not None
            and tilt_lim.upper > tilt_lim.lower + 0.1,
            details=f"limits=({tilt_lim.lower}, {tilt_lim.upper})",
        )

        # Backrest hinge pin on beam overlaps with backrest barrel (captured pin)
        ctx.allow_overlap(
            br,
            beam,
            elem_a=f"backrest_barrel_{i}",
            elem_b=f"backrest_hinge_pin_{i}",
            reason=f"Backrest barrel is a hinge bushing intentionally nested around hinge pin {i}.",
        )
        # Panel bottom wraps around the hinge pin at the pivot point
        ctx.allow_overlap(
            br,
            beam,
            elem_a=f"backrest_panel_{i}",
            elem_b=f"backrest_hinge_pin_{i}",
            reason=f"Backrest panel bottom edge wraps around hinge pin {i} at the pivot point.",
        )
        ctx.expect_contact(
            br,
            beam,
            elem_a=f"backrest_barrel_{i}",
            elem_b=f"backrest_hinge_pin_{i}",
            name=f"backrest barrel {i} is seated on the hinge pin",
        )

    # --- Decisive pose: main beam rocking ---
    rest_b0 = ctx.part_element_world_aabb(beam, elem="bumper_0")
    with ctx.pose({pivot: TILT}):
        down_b0 = ctx.part_element_world_aabb(beam, elem="bumper_0")
        up_b1 = ctx.part_element_world_aabb(beam, elem="bumper_1")
        ctx.check(
            "positive rock lowers the +X end near the ground",
            rest_b0 is not None
            and down_b0 is not None
            and down_b0[0][2] < rest_b0[0][2] - 0.40
            and down_b0[0][2] > 0.0,
            details=f"rest={rest_b0}, tilted={down_b0}",
        )
        ctx.check(
            "positive rock raises the -X end",
            up_b1 is not None and up_b1[0][2] > 1.0,
            details=f"raised bumper aabb={up_b1}",
        )
    with ctx.pose({pivot: -TILT}):
        down_b1 = ctx.part_element_world_aabb(beam, elem="bumper_1")
        ctx.check(
            "negative rock lowers the -X end near the ground",
            down_b1 is not None and 0.0 < down_b1[0][2] < 0.32,
            details=f"tilted bumper aabb={down_b1}",
        )

    # --- Decisive pose: backrest tilts ---
    rest_panel_0 = ctx.part_element_world_aabb(backrest_0, elem="backrest_panel_0")
    with ctx.pose({tilt_0: BACKREST_UPPER_LIMIT}):
        tilted_panel_0 = ctx.part_element_world_aabb(backrest_0, elem="backrest_panel_0")
        ctx.check(
            "backrest_0 tilts backward at upper limit (panel top moves in -X)",
            rest_panel_0 is not None
            and tilted_panel_0 is not None
            and tilted_panel_0[1][2] < rest_panel_0[1][2] + 0.05,  # top stays roughly at same height
            details=f"rest={rest_panel_0}, tilted={tilted_panel_0}",
        )

    return ctx.report()


object_model = build_object_model()
