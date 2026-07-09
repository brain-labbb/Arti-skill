from __future__ import annotations

"""Horizontal wooden venetian blind.

Reference: picture/Curtain/blind/002.png — dark-wood horizontal slats with
lighter wood-tone ladder tapes at two vertical stations, a wooden headrail
box, a heavier bottom rail, thin pull cords with small tassels at the front,
and a tilt wand.

Articulations:
- slat_tilt (REVOLUTE driver) + 11 mimic followers: all slats tilt in unison
  about their long horizontal (X) axes, limits -1.3..1.3 rad.
- lift (PRISMATIC driver on the bottom rail) + 12 per-slat mimic lift joints:
  pulling the lift cord raises the bottom rail, and the slats themselves rise
  and GATHER into a tight stack under the headrail. Each slat rides its own
  prismatic carrier that mimics `lift`; the multiplier grows toward the bottom
  of the blind, so the lowest slats travel farthest and the gather is led from
  the bottom upward, collapsing the open ~100 mm slat pitch down to a ~10 mm
  stacked pitch directly beneath the headrail.

  Engine note: the SDK's mimic relation is strictly linear (value =
  multiplier * driver + offset) with no per-joint clamping, so the gather is a
  smooth, proportional, bottom-led collection. A literal one-slat-at-a-time
  plateau (where each slat stays put until the rising stack reaches it) would
  require clamped/piecewise mimic the simulator does not provide; modeling it
  with linear mimic would force the lower slats to interpenetrate the stack.
  The proportional gather is the closest single-cord representation.

Kinematic structure: each slat is split into an invisible lift carrier
(headrail -> PRISMATIC slat_lift_i -> carrier_i) and the wood blade
(carrier_i -> REVOLUTE slat_tilt_i -> blade_i), giving every slat both a lift
and a tilt degree of freedom.

Support strategy: the ladder tapes are thin vertical strips owned by the
headrail, running the full drop from inside the headrail down to the bottom
rail. Every slat blade embeds the tape strips within its wider body at any lift
height — enough for the physical-contact support check, representing the real
ladder-tape slot through each slat.
"""

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

# ---------------------------------------------------------------- dimensions
BLIND_WIDTH = 0.80  # headrail width (X)
SLAT_WIDTH = 0.78
SLAT_DEPTH = 0.080  # ~80 mm wider slat (Y)
SLAT_THICK = 0.003
SLAT_COUNT = 12
SLAT_PITCH = 0.100  # open (lowered) slat-to-slat pitch
TOP_SLAT_Z = 1.215

HEADRAIL_SIZE = (BLIND_WIDTH, 0.062, 0.060)
HEADRAIL_Z = 1.27  # center; spans 1.24 .. 1.30

BOTTOM_RAIL_SIZE = (SLAT_WIDTH, 0.055, 0.022)
BOTTOM_RAIL_Z = 0.068  # center; spans 0.057 .. 0.079

TAPE_STATIONS = (-0.20, 0.20)  # ladder tape X positions
TAPE_WIDTH = 0.024
TAPE_THICK = 0.002
TAPE_Y = 0.0280  # inner face at 0.0270 -> embeds inside wider 0.040 slat half-edge
TAPE_TOP_Z = 1.27  # buried inside the headrail box
TAPE_BOTTOM_Z = 0.060  # buried inside the bottom rail z-range

# ---------------------------------------------------------------- gather (lift)
STACK_PITCH = 0.010  # tight slat pitch when fully raised
STACK_TOP_Z = TOP_SLAT_Z + 0.010  # gathered stack top; top slat lifts ~10 mm
RAIL_GATHER_GAP = 0.006  # clearance between gathered bottom slat and rail top

TILT_LIMIT = 1.3

DARK_WOOD = Material("dark_wood", rgba=(0.23, 0.155, 0.105, 1.0))
DARK_WOOD_RAIL = Material("dark_wood_rail", rgba=(0.20, 0.135, 0.09, 1.0))
TAPE_TAN = Material("tape_tan", rgba=(0.72, 0.60, 0.44, 1.0))
CORD_TAN = Material("cord_tan", rgba=(0.66, 0.55, 0.40, 1.0))


def slat_down_z(index: int) -> float:
    """Headrail-frame Z of a slat center when lowered (open). 1-based from top."""
    return TOP_SLAT_Z - (index - 1) * SLAT_PITCH


def slat_up_z(index: int) -> float:
    """Headrail-frame Z of a slat center when fully gathered under the headrail."""
    return STACK_TOP_Z - (index - 1) * STACK_PITCH


def slat_displacement(index: int) -> float:
    """How far a slat rises between lowered and fully gathered (always > 0)."""
    return slat_up_z(index) - slat_down_z(index)


# Bottom rail sits just under the lowest gathered slat at full lift; its travel
# is the master lift stroke that every slat carrier mimics.
RAIL_UP_Z = slat_up_z(SLAT_COUNT) - (0.5 * SLAT_THICK + RAIL_GATHER_GAP + 0.5 * BOTTOM_RAIL_SIZE[2])
RAIL_TRAVEL = RAIL_UP_Z - BOTTOM_RAIL_Z


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wooden_venetian_blind")

    # ------------------------------------------------------------- headrail
    headrail = model.part("headrail")
    headrail.visual(
        Box(HEADRAIL_SIZE),
        origin=Origin(xyz=(0.0, 0.0, HEADRAIL_Z)),
        material=DARK_WOOD_RAIL,
        name="headrail_box",
    )

    # Ladder tapes: front + back strips at two stations, headrail -> bottom rail.
    tape_len = TAPE_TOP_Z - TAPE_BOTTOM_Z
    tape_mid_z = 0.5 * (TAPE_TOP_Z + TAPE_BOTTOM_Z)
    for station_x, side in zip(TAPE_STATIONS, ("a", "b")):
        for tape_y, face in ((TAPE_Y, "front"), (-TAPE_Y, "rear")):
            headrail.visual(
                Box((TAPE_WIDTH, TAPE_THICK, tape_len)),
                origin=Origin(xyz=(station_x, tape_y, tape_mid_z)),
                material=TAPE_TAN,
                name=f"ladder_tape_{side}_{face}",
            )

    # Cord lock on the front-right of the headrail; pull cords exit here.
    headrail.visual(
        Box((0.07, 0.014, 0.030)),
        origin=Origin(xyz=(0.32, 0.036, 1.252)),
        material=DARK_WOOD_RAIL,
        name="cord_lock",
    )
    for cord_x, idx in ((0.305, 0), (0.335, 1)):
        cord_top = 1.260
        cord_bottom = 0.500
        headrail.visual(
            Cylinder(radius=0.0022, length=cord_top - cord_bottom),
            origin=Origin(xyz=(cord_x, 0.038, 0.5 * (cord_top + cord_bottom))),
            material=CORD_TAN,
            name=f"lift_cord_{idx}",
        )
        headrail.visual(
            Cylinder(radius=0.008, length=0.055),
            origin=Origin(xyz=(cord_x, 0.038, 0.475)),
            material=DARK_WOOD,
            name=f"cord_tassel_{idx}",
        )

    # Tilt wand hanging from a small hook on the front-left of the headrail.
    headrail.visual(
        Box((0.018, 0.012, 0.024)),
        origin=Origin(xyz=(-0.34, 0.035, 1.250)),
        material=DARK_WOOD_RAIL,
        name="wand_hook",
    )
    headrail.visual(
        Cylinder(radius=0.006, length=0.550),
        origin=Origin(xyz=(-0.34, 0.035, 0.980)),
        material=CORD_TAN,
        name="tilt_wand",
    )

    # ----------------------------------------------------------- bottom rail
    # The bottom rail is the master lift driver: pulling the cord raises it, and
    # all slat carriers mimic this stroke.
    bottom_rail = model.part("bottom_rail")
    bottom_rail.visual(
        Box(BOTTOM_RAIL_SIZE),
        origin=Origin(),
        material=DARK_WOOD_RAIL,
        name="bottom_rail_bar",
    )
    model.articulation(
        "lift",
        ArticulationType.PRISMATIC,
        parent=headrail,
        child=bottom_rail,
        origin=Origin(xyz=(0.0, 0.0, BOTTOM_RAIL_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.5, lower=0.0, upper=RAIL_TRAVEL),
    )

    # ---------------------------------------------------------------- slats
    # Each slat = invisible lift carrier (mimics `lift`) + wood blade (tilt).
    tilt_limits = MotionLimits(effort=2.0, velocity=2.0, lower=-TILT_LIMIT, upper=TILT_LIMIT)
    for i in range(1, SLAT_COUNT + 1):
        carrier = model.part(f"slat_carrier_{i:02d}")  # no visual: pure lift stage

        disp = slat_displacement(i)
        lift_mult = disp / RAIL_TRAVEL
        model.articulation(
            f"slat_lift_{i:02d}",
            ArticulationType.PRISMATIC,
            parent=headrail,
            child=carrier,
            origin=Origin(xyz=(0.0, 0.0, slat_down_z(i))),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=20.0, velocity=0.5, lower=0.0, upper=disp),
            mimic=Mimic(joint="lift", multiplier=lift_mult, offset=0.0),
        )

        blade = model.part(f"slat_{i:02d}")
        blade.visual(
            Box((SLAT_WIDTH, SLAT_DEPTH, SLAT_THICK)),
            origin=Origin(),
            material=DARK_WOOD,
            name=f"slat_{i:02d}_blade",
        )
        tilt_name = "slat_tilt" if i == 1 else f"slat_tilt_{i:02d}"
        model.articulation(
            tilt_name,
            ArticulationType.REVOLUTE,
            parent=carrier,
            child=blade,
            origin=Origin(),
            axis=(1.0, 0.0, 0.0),
            motion_limits=tilt_limits,
            mimic=None if i == 1 else Mimic(joint="slat_tilt", multiplier=1.0, offset=0.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    headrail = object_model.get_part("headrail")
    bottom_rail = object_model.get_part("bottom_rail")
    slats = [object_model.get_part(f"slat_{i:02d}") for i in range(1, SLAT_COUNT + 1)]
    tilt = object_model.get_articulation("slat_tilt")
    lift = object_model.get_articulation("lift")

    # 1. Slat count.
    slat_parts = [
        p
        for p in object_model.parts
        if p.name.startswith("slat_") and not p.name.startswith("slat_carrier_")
    ]
    ctx.check(
        "slat_count_is_12",
        len(slat_parts) == SLAT_COUNT,
        f"expected {SLAT_COUNT} slats, found {len(slat_parts)}",
    )

    # 2. Tilt mimic chain: every non-driver slat tilt joint mimics the driver 1:1.
    tilt_followers = [a for a in object_model.articulations if a.name.startswith("slat_tilt_")]
    ok_tilt_chain = len(tilt_followers) == SLAT_COUNT - 1 and all(
        a.mimic is not None
        and a.mimic.joint == "slat_tilt"
        and abs(a.mimic.multiplier - 1.0) < 1e-9
        and abs(a.mimic.offset) < 1e-9
        for a in tilt_followers
    )
    ctx.check(
        "tilt_followers_mimic_driver_1to1",
        ok_tilt_chain,
        f"{len(tilt_followers)} tilt followers; all must mimic 'slat_tilt' with multiplier 1.0",
    )

    # 3. Tilt driver axis/limits: long horizontal X axis, about -1.3..1.3 rad.
    ctx.check(
        "tilt_axis_is_long_horizontal_x",
        tuple(tilt.axis) == (1.0, 0.0, 0.0),
        f"tilt axis={tilt.axis}",
    )
    ctx.check(
        "tilt_limits_about_pm_1p3_rad",
        tilt.motion_limits is not None
        and abs(tilt.motion_limits.lower + TILT_LIMIT) < 1e-9
        and abs(tilt.motion_limits.upper - TILT_LIMIT) < 1e-9,
        f"tilt limits={tilt.motion_limits}",
    )

    # 4. Lift driver: a single prismatic on the bottom rail, vertical, positive stroke.
    ctx.check(
        "lift_driver_axis_is_vertical_z",
        tuple(lift.axis) == (0.0, 0.0, 1.0)
        and lift.articulation_type == ArticulationType.PRISMATIC
        and lift.mimic is None,
        f"lift axis={lift.axis} type={lift.articulation_type} mimic={lift.mimic}",
    )
    ctx.check(
        "lift_stroke_positive",
        lift.motion_limits is not None and lift.motion_limits.upper > 0.3,
        f"lift travel={RAIL_TRAVEL:.3f} m",
    )

    # 5. Per-slat lift mimic chain: 30 carriers mimic `lift`, multipliers strictly
    #    increase from the top slat to the bottom slat (the bottom leads the gather),
    #    and every multiplier is a positive fraction of the master stroke.
    lift_followers = [
        object_model.get_articulation(f"slat_lift_{i:02d}") for i in range(1, SLAT_COUNT + 1)
    ]
    mults = [a.mimic.multiplier for a in lift_followers if a.mimic is not None]
    ok_lift_chain = (
        len(mults) == SLAT_COUNT
        and all(a.mimic is not None and a.mimic.joint == "lift" for a in lift_followers)
        and all(0.0 < mm <= 1.0 for mm in mults)
        and all(mults[k] < mults[k + 1] for k in range(len(mults) - 1))
    )
    ctx.check(
        "lift_followers_mimic_lift_bottom_leads",
        ok_lift_chain,
        f"lift multipliers (top->bottom)={[round(mm, 3) for mm in mults]}",
    )

    # 6. Two ladder tape stations, each with front and rear strips.
    tape_xs = set()
    for side in ("a", "b"):
        for face in ("front", "rear"):
            tape = headrail.get_visual(f"ladder_tape_{side}_{face}")
            ctx.check(
                f"ladder_tape_{side}_{face}_exists",
                tape is not None,
                "missing ladder tape strip",
            )
            aabb = ctx.part_element_world_aabb(headrail, elem=tape)
            assert aabb is not None
            tape_xs.add(round(0.5 * (aabb[0][0] + aabb[1][0]), 4))
    ctx.check(
        "ladder_tapes_at_two_stations",
        len(tape_xs) == 2,
        f"tape stations at x={sorted(tape_xs)}",
    )

    # 7. Slats span both tape stations (the tapes carry the slats).
    slat_aabb = ctx.part_world_aabb(slats[0])
    assert slat_aabb is not None
    ctx.check(
        "slats_span_both_tape_stations",
        all(slat_aabb[0][0] < x < slat_aabb[1][0] for x in tape_xs),
        f"slat x-range=({slat_aabb[0][0]:.3f},{slat_aabb[1][0]:.3f}), stations={sorted(tape_xs)}",
    )

    headrail_box = headrail.get_visual("headrail_box")
    headrail_box_aabb = ctx.part_element_world_aabb(headrail, elem=headrail_box)
    assert headrail_box_aabb is not None

    # 8. Lowered pose: bottom rail sits below the lowest slat with a clear gap,
    #    headrail sits above the top slat, and the tapes contact the bottom rail.
    with ctx.pose({tilt: 0.0, lift: 0.0}):
        ctx.expect_gap(
            slats[-1],
            bottom_rail,
            axis="z",
            min_gap=0.01,
            name="bottom_rail_below_lowest_slat",
        )
        ctx.expect_gap(
            headrail,
            slats[0],
            axis="z",
            min_gap=0.01,
            positive_elem=headrail_box,
            name="headrail_above_top_slat",
        )
        tape_front = headrail.get_visual("ladder_tape_a_front")
        ctx.expect_contact(
            headrail,
            bottom_rail,
            elem_a=tape_front,
            name="ladder_tape_reaches_bottom_rail",
        )

    # 9. Raised pose: pulling the lift cord raises the SLATS (not just a rail) and
    #    gathers them into a tight stack under the headrail.
    travel = float(lift.motion_limits.upper)
    z_lowered = [ctx.part_world_position(s)[2] for s in slats]
    with ctx.pose({lift: travel}):
        z_raised = [ctx.part_world_position(s)[2] for s in slats]
        rises = [zr - zl for zr, zl in zip(z_raised, z_lowered)]

        # Every slat actually rises.
        ctx.check(
            "all_slats_rise_on_lift",
            all(r > 1e-4 for r in rises),
            f"min rise={min(rises):.4f} m",
        )
        # Bottom-led gather: each slat rises more than the one above it.
        ctx.check(
            "lower_slats_lead_the_gather",
            all(rises[k] < rises[k + 1] for k in range(len(rises) - 1)),
            f"rises top->bottom={[round(r, 3) for r in rises]}",
        )
        # Gathered stack is tight: span collapses from open pitch to ~stack pitch.
        stack_span = z_raised[0] - z_raised[-1]
        expected_span = (SLAT_COUNT - 1) * STACK_PITCH
        ctx.check(
            "gathered_stack_is_tight",
            abs(stack_span - expected_span) < 0.02,
            f"stack span={stack_span:.3f} m, expected ~{expected_span:.3f} m",
        )
        # Stack hangs under the headrail (top slat below the headrail box).
        top_slat_aabb = ctx.part_world_aabb(slats[0])
        assert top_slat_aabb is not None
        ctx.check(
            "gathered_stack_under_headrail",
            top_slat_aabb[1][2] < headrail_box_aabb[0][2],
            f"top slat top={top_slat_aabb[1][2]:.3f}, headrail bottom={headrail_box_aabb[0][2]:.3f}",
        )
        # Bottom rail rose to just beneath the gathered stack.
        ctx.expect_gap(
            slats[-1],
            bottom_rail,
            axis="z",
            min_gap=0.002,
            name="bottom_rail_under_gathered_stack",
        )

    # 10. Tilt pose: driving only the driver tilts the driver slat AND a mimic
    #     follower mid-stack (blade z-extent grows from 3 mm to ~45 mm).
    with ctx.pose({tilt: 1.0}):
        for probe, label in ((slats[0], "driver_slat"), (slats[5], "mid_follower_slat")):
            aabb = ctx.part_world_aabb(probe)
            assert aabb is not None
            z_extent = aabb[1][2] - aabb[0][2]
            ctx.check(
                f"tilt_rotates_{label}",
                z_extent > 0.030,
                f"z-extent at q=1.0 is {z_extent:.4f} m (flat would be {SLAT_THICK} m)",
            )

    # 11. Pull cords + tassels hang at the front, below the headrail.
    for idx in (0, 1):
        tassel = headrail.get_visual(f"cord_tassel_{idx}")
        aabb = ctx.part_element_world_aabb(headrail, elem=tassel)
        assert aabb is not None
        ctx.check(
            f"tassel_{idx}_hangs_below_headrail_front",
            aabb[1][2] < headrail_box_aabb[0][2] - 0.5 and aabb[0][1] > 0.0,
            f"tassel aabb={aabb}",
        )

    return ctx.report()


object_model = build_object_model()
