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
# Four-seat cross playground seesaw
#
# World frame: X and Y beams cross at center, pivot axis along Y for primary
# rocking, Z up.
#
# Base: two bent galvanized-steel tube arches (~50 mm dia) form an A-shaped
#   saddle; the apex carries a horizontal pivot axle.  Four rubber ground pads
#   sit under the arch feet.  A catch-plate bracket for the locking pin is
#   welded beside the apex.
#
# Cross beam: two perpendicular 3.0 m mustard-yellow steel bars (80 × 40 mm)
#   cross at the center.  Each of the four ends carries a wooden seat plate,
#   an inverted-U grab handle, and a curved rubber tire-section bumper.
#   A pivot sleeve + gusset plates connect the cross to the axle.
#   A pin-guide bracket is welded beside the crossing for the locking pin.
#
# Locking pin: a bright-steel shaft with a rubber grip head slides vertically
#   through the beam bracket (prismatic, axis (0,0,-1), 0 … 50 mm travel) to
#   engage the base catch plate and lock the seesaw level.
#
# Articulations:
#   beam_pivot  – REVOLUTE, axis (0,1,0), ±20°
#   pin_slide   – PRISMATIC, axis (0,0,-1), 0 … 0.05 m
# ---------------------------------------------------------------------------

BEAM_HALF = 1.50          # 3.0 m total beam
BEAM_W = 0.08
BEAM_T = 0.04
PIVOT_Z = 0.76            # axle height (~0.8 m)

ARCH_FOOT_X = 0.66
ARCH_FOOT_Y = 0.34
ARCH_APEX_Y = 0.05
ARCH_FOOT_Z = 0.028
TUBE_R = 0.025            # ~50 mm diameter bent tube

AXLE_R = 0.016
AXLE_LEN = 0.22

BAR_BOT = 0.05
BAR_CTR = BAR_BOT + BEAM_T / 2.0   # 0.07
BAR_TOP = BAR_BOT + BEAM_T         # 0.09

SEAT_DIST = 1.30         # seat distance from center
HANDLE_DIST = 1.04
BUMPER_DIST = 1.42
TILT = math.radians(20.0)

# Rubber ground pads
PAD_R = 0.060
PAD_T = 0.020

# Locking pin
PIN_R = 0.010
PIN_LEN = 0.080
PIN_HEAD_R = 0.020
PIN_HEAD_T = 0.012
PIN_TRAVEL = 0.050


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


def _handle_points(pos: float, along_y: bool = False) -> list[tuple[float, float, float]]:
    """Inverted-U grab-handle rod centerline."""
    half_w = 0.035
    leg_bot = BAR_TOP - 0.010
    arc_z = 0.275
    if not along_y:
        # Handle in YZ plane at x = pos
        pts: list[tuple[float, float, float]] = [
            (pos, -half_w, leg_bot),
            (pos, -half_w, 0.190),
        ]
        for k in range(7):
            a = math.pi * k / 6.0
            pts.append((pos, -half_w * math.cos(a), arc_z + half_w * math.sin(a)))
        pts.append((pos, half_w, 0.190))
        pts.append((pos, half_w, leg_bot))
    else:
        # Handle in XZ plane at y = pos
        pts = [
            (-half_w, pos, leg_bot),
            (-half_w, pos, 0.190),
        ]
        for k in range(7):
            a = math.pi * k / 6.0
            pts.append((-half_w * math.cos(a), pos, arc_z + half_w * math.sin(a)))
        pts.append((half_w, pos, 0.190))
        pts.append((half_w, pos, leg_bot))
    return pts


def _bumper_geometry(pos: float, index: int, along_y: bool = False):
    """Curved tire-section bumper: half-annulus shell across the beam."""
    r_out = 0.065
    r_in = 0.048
    profile: list[tuple[float, float]] = []
    n = 20
    for k in range(n + 1):
        a = math.pi + math.pi * k / n
        profile.append((r_out * math.cos(a), r_out * math.sin(a)))
    for k in range(n + 1):
        a = 2.0 * math.pi - math.pi * k / n
        profile.append((r_in * math.cos(a), r_in * math.sin(a)))
    geom = ExtrudeGeometry(profile, 0.10, cap=True, center=True)
    geom.rotate_x(math.pi / 2.0)
    if along_y:
        geom.rotate_z(math.pi / 2.0)
        geom.translate(0.0, pos, BAR_BOT + 0.002)
    else:
        geom.translate(pos, 0.0, BAR_BOT + 0.002)
    return mesh_from_geometry(geom, f"cross_bumper_{index}")


def _gusset_geometry(name: str, along_y: bool = False):
    """Triangular gusset plate joining beam bar to pivot sleeve."""
    profile = [(-0.10, 0.055), (0.10, 0.055), (0.0, 0.020)]
    geom = ExtrudeGeometry(profile, 0.020, cap=True, center=True)
    geom.rotate_x(math.pi / 2.0)
    if along_y:
        geom.rotate_z(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cross_playground_seesaw")

    galvanized = model.material("weathered_galvanized", rgba=(0.55, 0.58, 0.56, 1.0))
    rust = model.material("rust_steel", rgba=(0.42, 0.25, 0.13, 1.0))
    mustard = model.material("rusty_mustard_paint", rgba=(0.74, 0.53, 0.12, 1.0))
    pale_steel = model.material("pale_weathered_steel", rgba=(0.70, 0.66, 0.58, 1.0))
    wood = model.material("worn_wood", rgba=(0.60, 0.45, 0.28, 1.0))
    rubber = model.material("black_rubber", rgba=(0.08, 0.08, 0.08, 1.0))
    pad_rubber = model.material("ground_pad_rubber", rgba=(0.10, 0.10, 0.08, 1.0))
    pin_steel = model.material("bright_steel", rgba=(0.72, 0.72, 0.70, 1.0))
    grip_rubber = model.material("grip_rubber", rgba=(0.14, 0.06, 0.04, 1.0))

    # =============================================================== base ===
    base = model.part("arched_base")

    # Two crossing arch tubes
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
                f"seesaw_arch_{i}",
            ),
            material=galvanized,
            name=f"arch_{i}",
        )

    # Pivot axle bolt through both arch apexes, axis along Y
    base.visual(
        Cylinder(radius=AXLE_R, length=AXLE_LEN),
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rust,
        name="pivot_axle",
    )
    for i, side in enumerate((1.0, -1.0)):
        base.visual(
            Cylinder(radius=0.024, length=0.014),
            origin=Origin(
                xyz=(0.0, side * (AXLE_LEN / 2.0 - 0.006), PIVOT_Z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=rust,
            name=f"axle_nut_{i}",
        )

    # Catch plate bracket for the locking pin (welded outboard of the apex)
    base.visual(
        Box((0.040, 0.080, 0.060)),
        origin=Origin(xyz=(0.25, 0.0, PIVOT_Z - 0.030)),
        material=galvanized,
        name="pin_catch_plate",
    )

    # Rubber ground pads under each arch foot (4 feet)
    foot_positions = [
        (ARCH_FOOT_X, ARCH_FOOT_Y),
        (-ARCH_FOOT_X, ARCH_FOOT_Y),
        (ARCH_FOOT_X, -ARCH_FOOT_Y),
        (-ARCH_FOOT_X, -ARCH_FOOT_Y),
    ]
    for i, (fx, fy) in enumerate(foot_positions):
        base.visual(
            Cylinder(radius=PAD_R, length=PAD_T),
            origin=Origin(xyz=(fx, fy, PAD_T / 2.0)),
            material=pad_rubber,
            name=f"ground_pad_{i}",
        )

    # =========================================================== cross beam ===
    beam = model.part("cross_beam")

    # Pivot sleeve at the axle center
    beam.visual(
        Cylinder(radius=0.026, length=0.044),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rust,
        name="pivot_sleeve",
    )

    # X-direction beam bar
    beam.visual(
        Box((2.0 * BEAM_HALF, BEAM_W, BEAM_T)),
        origin=Origin(xyz=(0.0, 0.0, BAR_CTR)),
        material=mustard,
        name="beam_bar_x",
    )
    # Y-direction beam bar (perpendicular cross)
    beam.visual(
        Box((BEAM_W, 2.0 * BEAM_HALF, BEAM_T)),
        origin=Origin(xyz=(0.0, 0.0, BAR_CTR)),
        material=mustard,
        name="beam_bar_y",
    )

    # Gusset plates (one per beam direction)
    beam.visual(_gusset_geometry("gusset_x", along_y=False), material=mustard, name="gusset_x")
    beam.visual(_gusset_geometry("gusset_y", along_y=True), material=mustard, name="gusset_y")

    # Pin guide bracket on beam (welded outboard of the crossing, receives the pin)
    beam.visual(
        Box((0.040, 0.044, 0.070)),
        origin=Origin(xyz=(0.25, 0.0, 0.025)),
        material=galvanized,
        name="pin_guide_bracket",
    )

    # Rust streak patches on X beam
    for i, px in enumerate((-0.85, -0.30, 0.45, 0.95)):
        beam.visual(
            Box((0.16, BEAM_W + 0.004, 0.012)),
            origin=Origin(xyz=(px, 0.0, BAR_TOP - 0.004)),
            material=rust,
            name=f"rust_patch_x{i}",
        )
    # Rust streak patches on Y beam
    for i, py in enumerate((-0.85, -0.30, 0.45, 0.95)):
        beam.visual(
            Box((BEAM_W + 0.004, 0.16, 0.012)),
            origin=Origin(xyz=(0.0, py, BAR_TOP - 0.004)),
            material=rust,
            name=f"rust_patch_y{i}",
        )

    # ---- X-beam end fittings (seats 0,1 at +X, -X) ----
    for i, side in enumerate((1.0, -1.0)):
        beam.visual(
            Box((0.30, 0.24, 0.022)),
            origin=Origin(xyz=(side * SEAT_DIST, 0.0, BAR_TOP + 0.008)),
            material=wood,
            name=f"seat_{i}",
        )
        beam.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    _handle_points(side * HANDLE_DIST, along_y=False),
                    radius=0.009,
                    samples_per_segment=8,
                    radial_segments=16,
                    cap_ends=True,
                ),
                f"cross_handle_{i}",
            ),
            material=pale_steel,
            name=f"handle_{i}",
        )
        beam.visual(
            _bumper_geometry(side * BUMPER_DIST, i, along_y=False),
            material=rubber,
            name=f"bumper_{i}",
        )

    # ---- Y-beam end fittings (seats 2,3 at +Y, -Y) ----
    for i, side in enumerate((1.0, -1.0)):
        beam.visual(
            Box((0.24, 0.30, 0.022)),
            origin=Origin(xyz=(0.0, side * SEAT_DIST, BAR_TOP + 0.008)),
            material=wood,
            name=f"seat_{i + 2}",
        )
        beam.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    _handle_points(side * HANDLE_DIST, along_y=True),
                    radius=0.009,
                    samples_per_segment=8,
                    radial_segments=16,
                    cap_ends=True,
                ),
                f"cross_handle_{i + 2}",
            ),
            material=pale_steel,
            name=f"handle_{i + 2}",
        )
        beam.visual(
            _bumper_geometry(side * BUMPER_DIST, i + 2, along_y=True),
            material=rubber,
            name=f"bumper_{i + 2}",
        )

    # ========================================================= locking pin ===
    pin = model.part("locking_pin")
    # Pin shaft (vertical cylinder, extends upward from frame origin)
    pin.visual(
        Cylinder(radius=PIN_R, length=PIN_LEN),
        origin=Origin(xyz=(0.0, 0.0, PIN_LEN / 2.0)),
        material=pin_steel,
        name="pin_shaft",
    )
    # Pin grip head (wider rubber cap on top)
    pin.visual(
        Cylinder(radius=PIN_HEAD_R, length=PIN_HEAD_T),
        origin=Origin(xyz=(0.0, 0.0, PIN_LEN + PIN_HEAD_T / 2.0)),
        material=grip_rubber,
        name="pin_head",
    )

    # ============================================================= joints ===
    # Main beam pivot: revolute about Y axis, ±20°
    model.articulation(
        "beam_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=beam,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=200.0, velocity=2.5, lower=-TILT, upper=TILT),
    )

    # Locking pin: prismatic, slides downward to engage catch plate
    # Parent = beam (pin moves with beam), child = pin
    # At q=0 pin is retracted (bottom at beam z=0 → world z=PIVOT_Z)
    # At q=PIN_TRAVEL pin bottom drops to beam z=-PIN_TRAVEL
    model.articulation(
        "pin_slide",
        ArticulationType.PRISMATIC,
        parent=beam,
        child=pin,
        origin=Origin(xyz=(0.25, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.5, lower=0.0, upper=PIN_TRAVEL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("arched_base")
    beam = object_model.get_part("cross_beam")
    pin = object_model.get_part("locking_pin")
    pivot = object_model.get_articulation("beam_pivot")
    slide = object_model.get_articulation("pin_slide")

    # ---- Intentional overlaps ----

    # Pivot sleeve captures the axle bolt
    ctx.allow_overlap(
        beam, base,
        elem_a="pivot_sleeve", elem_b="pivot_axle",
        reason="Pivot sleeve is a bushing intentionally nested around the axle bolt.",
    )
    ctx.expect_contact(
        beam, base,
        elem_a="pivot_sleeve", elem_b="pivot_axle",
        name="pivot sleeve is seated on the axle bolt",
    )
    ctx.expect_within(
        beam, base,
        axes="y",
        inner_elem="pivot_sleeve", outer_elem="pivot_axle",
        margin=0.001,
        name="pivot sleeve stays inside the axle span",
    )

    # Cross beam bars intentionally cross at center (welded joint)
    ctx.allow_overlap(
        beam, beam,
        elem_a="beam_bar_x", elem_b="beam_bar_y",
        reason="Two perpendicular beam bars are welded together at the crossing point.",
    )

    # Pin shaft slides through the beam bracket
    ctx.allow_overlap(
        beam, pin,
        elem_a="pin_guide_bracket", elem_b="pin_shaft",
        reason="Locking pin shaft slides through the guide bracket on the beam.",
    )

    # Catch plate (base) and guide bracket (beam) align for pin engagement
    ctx.allow_overlap(
        base, beam,
        elem_a="pin_catch_plate", elem_b="pin_guide_bracket",
        reason="Catch plate and guide bracket are aligned at the same station for the pin to pass through both.",
    )

    # Pin shaft passes through a hole in the beam bar
    ctx.allow_overlap(
        beam, pin,
        elem_a="beam_bar_x", elem_b="pin_shaft",
        reason="Locking pin shaft is inserted through a bore hole in the beam bar.",
    )

    # Pin head rests on the beam bar top surface when retracted (captured pin)
    ctx.allow_overlap(
        beam, pin,
        elem_a="beam_bar_x", elem_b="pin_head",
        reason="Pin head seats flush on the beam bar top when the pin is in the retracted position.",
    )

    # ---- Pivot joint configuration ----
    ax = pivot.axis
    ctx.check(
        "pivot axis is horizontal and perpendicular to the X-beam",
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

    # ---- Pin slide joint configuration ----
    ctx.check(
        "pin_slide is a prismatic joint",
        slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={slide.articulation_type}",
    )
    slide_ax = slide.axis
    ctx.check(
        "pin slide axis is vertical (downward engagement)",
        abs(slide_ax[0]) < 1e-9 and abs(slide_ax[1]) < 1e-9 and abs(slide_ax[2] + 1.0) < 1e-9,
        details=f"axis={slide_ax}",
    )
    slide_lim = slide.motion_limits
    ctx.check(
        "pin slide has 0 to 50 mm travel",
        slide_lim is not None
        and slide_lim.lower is not None
        and slide_lim.upper is not None
        and abs(slide_lim.lower) < 1e-6
        and abs(slide_lim.upper - PIN_TRAVEL) < 1e-6,
        details=f"limits=({slide_lim.lower}, {slide_lim.upper})",
    )

    # ---- Hero geometry: scale, saddle height, grounded feet ----
    bar_x_box = ctx.part_element_world_aabb(beam, elem="beam_bar_x")
    bar_y_box = ctx.part_element_world_aabb(beam, elem="beam_bar_y")
    base_box = ctx.part_world_aabb(base)
    axle_box = ctx.part_element_world_aabb(base, elem="pivot_axle")

    ctx.check(
        "X-beam is about 3.0 m long",
        bar_x_box is not None and abs((bar_x_box[1][0] - bar_x_box[0][0]) - 3.0) < 0.02,
        details=f"bar_x aabb={bar_x_box}",
    )
    ctx.check(
        "Y-beam is about 3.0 m long",
        bar_y_box is not None and abs((bar_y_box[1][1] - bar_y_box[0][1]) - 3.0) < 0.02,
        details=f"bar_y aabb={bar_y_box}",
    )
    ctx.check(
        "arched base feet rest on the ground",
        base_box is not None and -0.02 <= base_box[0][2] <= 0.02,
        details=f"base aabb={base_box}",
    )
    ctx.check(
        "pivot axle sits about 0.8 m high",
        axle_box is not None and 0.70 <= (axle_box[0][2] + axle_box[1][2]) / 2.0 <= 0.82,
        details=f"axle aabb={axle_box}",
    )

    # ---- Four seats exist and are on the beam bar tops ----
    for i in range(4):
        seat = ctx.part_element_world_aabb(beam, elem=f"seat_{i}")
        ctx.check(
            f"seat_{i} exists and is seated on the beam bar top",
            seat is not None
            and bar_x_box is not None
            and seat[0][2] > bar_x_box[0][2]
            and seat[1][2] > bar_x_box[1][2],
            details=f"seat_{i} aabb={seat}",
        )

    # ---- Four handles exist and stand above the beam ----
    for i in range(4):
        handle = ctx.part_element_world_aabb(beam, elem=f"handle_{i}")
        ctx.check(
            f"handle_{i} stands above the beam",
            handle is not None
            and bar_x_box is not None
            and handle[1][2] > bar_x_box[1][2] + 0.18,
            details=f"handle_{i} aabb={handle}",
        )

    # ---- Four bumpers hang below the beam tips ----
    for i in range(4):
        bumper = ctx.part_element_world_aabb(beam, elem=f"bumper_{i}")
        ctx.check(
            f"bumper_{i} hangs below the beam tip",
            bumper is not None
            and bar_x_box is not None
            and bumper[0][2] < bar_x_box[0][2],
            details=f"bumper_{i} aabb={bumper}",
        )

    # ---- Rubber ground pads exist under the support legs ----
    for i in range(4):
        pad = ctx.part_element_world_aabb(base, elem=f"ground_pad_{i}")
        ctx.check(
            f"ground_pad_{i} exists on the ground",
            pad is not None and pad[0][2] < 0.02,
            details=f"ground_pad_{i} aabb={pad}",
        )

    # ---- Beam bar clears the arch saddle ----
    ctx.expect_gap(
        beam, base,
        axis="z",
        positive_elem="beam_bar_x",
        negative_elem="arch_0",
        min_gap=0.005,
        max_gap=0.06,
        name="beam bar clears the arch saddle",
    )

    # ---- Locking pin proof checks ----
    # At rest (retracted), pin bottom should be near the axle height
    pin_pos_retracted = ctx.part_world_position(pin)
    ctx.check(
        "locking pin is near the pivot when retracted",
        pin_pos_retracted is not None
        and abs(pin_pos_retracted[2] - PIVOT_Z) < 0.12,
        details=f"pin position={pin_pos_retracted}",
    )

    # Catch plate and guide bracket are aligned at the same X station
    ctx.expect_overlap(
        base, beam,
        axes="x",
        elem_a="pin_catch_plate", elem_b="pin_guide_bracket",
        min_overlap=0.030,
        name="catch plate and guide bracket share the same X station",
    )

    # Pin shaft stays within the beam bar footprint on the non-slide axes
    ctx.expect_within(
        pin, beam,
        axes="xy",
        inner_elem="pin_shaft", outer_elem="beam_bar_x",
        margin=0.005,
        name="pin shaft stays centered in the beam bar bore",
    )

    # Pin head is seated near the beam bar top when retracted
    ctx.expect_contact(
        beam, pin,
        elem_a="beam_bar_x", elem_b="pin_head",
        contact_tol=0.015,
        name="pin head is seated near the beam bar top",
    )

    # At max engagement, pin should move downward
    with ctx.pose({slide: PIN_TRAVEL}):
        pin_pos_engaged = ctx.part_world_position(pin)
        ctx.check(
            "pin slide moves the pin downward",
            pin_pos_retracted is not None
            and pin_pos_engaged is not None
            and pin_pos_engaged[2] < pin_pos_retracted[2] - 0.02,
            details=f"retracted={pin_pos_retracted}, engaged={pin_pos_engaged}",
        )

    # ---- Decisive pose checks: rocking alternately lowers each end ----
    rest_b0 = ctx.part_element_world_aabb(beam, elem="bumper_0")
    rest_b2 = ctx.part_element_world_aabb(beam, elem="bumper_2")
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

    # ---- Cross beam perpendicularity check ----
    ctx.check(
        "X-beam and Y-beam are perpendicular",
        bar_x_box is not None and bar_y_box is not None
        and (bar_x_box[1][0] - bar_x_box[0][0]) > 2.0  # X-beam spans > 2m in X
        and (bar_y_box[1][1] - bar_y_box[0][1]) > 2.0,  # Y-beam spans > 2m in Y
        details=f"bar_x={bar_x_box}, bar_y={bar_y_box}",
    )

    return ctx.report()


object_model = build_object_model()
