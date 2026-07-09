from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    CylinderGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ----------------------------------------------------------------------------
# Variant 26: Curved-beam four-seat playground seesaw with raised ends.
#
# Changes from parent:
# - Curved main beam tubes that sweep upward at both ends (raised ends).
# - Locking pin near the central bracket that slides on a prismatic joint.
# - Rubber ground pads under each support leg foot.
# - Textured footrest plates near each seat.
#
# Layout (world frame, Z up, base centered on the origin):
# - Sky-blue base: two arched inverted-U tube legs joined by cross members,
#   with dark rubber pads at each foot.
# - Two independent yellow rocking beams (~2.6 m), arranged in a shallow X,
#   each a curved tube truss with raised ends.
# - Each beam carries seat plates, T-handlebars, and textured footrests.
# - A locking pin slides vertically near the upper arch bracket.
# - Articulation: each beam has its own revolute pivot (+/- 18 degrees),
#   and the locking pin has a prismatic joint (0 to 0.08 m travel).
# ----------------------------------------------------------------------------

TUBE_R = 0.020  # ~40 mm diameter main tubing
BRACE_R = 0.016
SUPPORT_R = 0.018
HANDLE_R = 0.016

YAW = math.radians(10.0)  # half angle of the shallow X between the beams
TILT = math.radians(18.0)  # rocking range of each beam

LOW_ARCH_TOP = 0.56  # pivot height of the lower beam
HIGH_ARCH_TOP = 0.74  # pivot height of the upper beam
ARCH_HALF_SPAN = 0.36  # ground half-span of each arch
CROSS_BRACE_Z = 0.28  # height of the short cross members joining the arches
CROSS_BRACE_U = 0.315  # arch-plane coordinate of the legs at CROSS_BRACE_Z

BEAM_LEN = 2.60
BEAM_HALF = BEAM_LEN / 2.0
MAIN_Z = 0.08  # main top tube height above the pivot axis at center
CURVE_RISE = 0.18  # how much the beam ends rise above center
SLEEVE_R = 0.032
SLEEVE_LEN = 0.13
SEAT_X = 1.43  # seat plate center along beam from pivot
SEAT_Z = 0.038  # seat plate center height above pivot axis (at ends)
SEAT_SIZE = (0.26, 0.30, 0.012)
HANDLE_X = 1.04  # T-handlebar post, just inboard of the seat
HANDLE_TOP_Z = 0.34  # crossbar height above the pivot axis

# Footrest dimensions
FOOTREST_X = 1.18  # footrest center along beam from pivot
FOOTREST_SIZE = (0.18, 0.12, 0.008)
FOOTREST_RIB_COUNT = 5

# Ground pad dimensions
PAD_SIZE = (0.12, 0.12, 0.015)

# Locking pin dimensions
PIN_R = 0.012
PIN_LEN = 0.14
PIN_BRACKET_SIZE = (0.06, 0.06, 0.04)

SKY_BLUE = Material("sky_blue_paint", rgba=(0.33, 0.62, 0.84, 1.0))
WORN_YELLOW = Material("worn_yellow_paint", rgba=(0.87, 0.74, 0.12, 1.0))
RUST_BROWN = Material("rust_brown_steel", rgba=(0.42, 0.21, 0.13, 1.0))
RUBBER_BLACK = Material("rubber_black", rgba=(0.12, 0.12, 0.10, 1.0))
STEEL_GRAY = Material("steel_gray", rgba=(0.55, 0.55, 0.52, 1.0))
FOOTREST_YELLOW = Material("footrest_yellow", rgba=(0.80, 0.68, 0.10, 1.0))


def _tube_between(
    p0: tuple[float, float, float],
    p1: tuple[float, float, float],
    radius: float,
    *,
    radial_segments: int = 16,
) -> MeshGeometry:
    """Straight capped tube between two 3D points."""
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    geom = CylinderGeometry(radius, length, radial_segments=radial_segments)
    ux, uy, uz = dx / length, dy / length, dz / length
    ax, ay, az = -uy, ux, 0.0
    s = math.sqrt(ax * ax + ay * ay + az * az)
    if s > 1e-9:
        geom.rotate((ax / s, ay / s, az / s), math.atan2(s, uz))
    elif uz < 0.0:
        geom.rotate_x(math.pi)
    geom.translate(
        (p0[0] + p1[0]) / 2.0,
        (p0[1] + p1[1]) / 2.0,
        (p0[2] + p1[2]) / 2.0,
    )
    return geom


def _arch_mesh(axis_xy: tuple[float, float], top_z: float) -> MeshGeometry:
    """Inverted-U arch tube in the vertical plane spanned by axis_xy."""
    ax, ay = axis_xy
    shoulder = 0.52 if top_z < 0.65 else 0.66
    profile_uz = [
        (-ARCH_HALF_SPAN - 0.055, 0.022),
        (-ARCH_HALF_SPAN - 0.03, 0.028),
        (-0.35, 0.10),
        (-CROSS_BRACE_U, CROSS_BRACE_Z),
        (-0.27, 0.44),
        (-0.18, shoulder),
        (-0.07, top_z),
        (0.0, top_z),
        (0.07, top_z),
        (0.18, shoulder),
        (0.27, 0.44),
        (CROSS_BRACE_U, CROSS_BRACE_Z),
        (0.35, 0.10),
        (ARCH_HALF_SPAN + 0.03, 0.028),
        (ARCH_HALF_SPAN + 0.055, 0.022),
    ]
    points = [(u * ax, u * ay, z) for (u, z) in profile_uz]
    return tube_from_spline_points(
        points,
        radius=TUBE_R,
        samples_per_segment=10,
        radial_segments=16,
        cap_ends=True,
    )


def _curved_beam_tube() -> MeshGeometry:
    """Build a curved main beam tube that rises at both ends.

    The beam follows a gentle upward curve: low at center (MAIN_Z above pivot),
    rising to MAIN_Z + CURVE_RISE at each end. Uses spline points along X.
    """
    n_pts = 11
    points = []
    for i in range(n_pts):
        t = i / (n_pts - 1)  # 0 to 1
        x = -BEAM_HALF + t * BEAM_LEN
        # Parabolic rise: highest at ends, lowest at center
        normalized = (2.0 * t - 1.0)  # -1 to +1
        z_rise = CURVE_RISE * normalized * normalized
        z = MAIN_Z + z_rise
        points.append((x, 0.0, z))

    return tube_from_spline_points(
        points,
        radius=TUBE_R,
        samples_per_segment=12,
        radial_segments=18,
        cap_ends=True,
    )


def _beam_end_z() -> float:
    """Z height of the main tube at the beam ends (for seat/footrest placement)."""
    return MAIN_Z + CURVE_RISE


def _footrest_mesh() -> MeshGeometry:
    """Build a textured footrest: a flat plate with raised ribs across it."""
    base = BoxGeometry(FOOTREST_SIZE)
    # Add ribs across the plate surface for texture
    rib_w = FOOTREST_SIZE[0] * 0.85
    rib_d = 0.006
    rib_h = 0.005
    spacing = FOOTREST_SIZE[1] / (FOOTREST_RIB_COUNT + 1)
    for i in range(FOOTREST_RIB_COUNT):
        y_pos = -FOOTREST_SIZE[1] / 2.0 + spacing * (i + 1)
        rib = BoxGeometry((rib_w, rib_d, rib_h))
        rib.translate(0.0, y_pos, FOOTREST_SIZE[2] / 2.0 + rib_h / 2.0)
        base.merge(rib)
    return base


def _beam_meshes() -> tuple[MeshGeometry, MeshGeometry, MeshGeometry, MeshGeometry, MeshGeometry, MeshGeometry]:
    """Build one rocking beam in its local frame (X along beam, pivot at origin).

    Returns (truss_tube, axle_sleeve, handlebar_pos_x, handlebar_neg_x,
             footrest_pos_x, footrest_neg_x).
    """
    end_z = _beam_end_z()

    # Curved main top tube
    truss = _curved_beam_tube()

    for sx in (1.0, -1.0):
        # Diagonal brace from the axle sleeve up to the main tube
        truss.merge(
            _tube_between(
                (sx * 0.04, 0.0, 0.005),
                (sx * 0.60, 0.0, MAIN_Z + CURVE_RISE * 0.15),
                BRACE_R,
            )
        )
        # Short bent seat support dropping from the curved tube end under the seat
        # The support starts from the raised end of the beam
        truss.merge(
            tube_from_spline_points(
                [
                    (sx * 1.24, 0.0, MAIN_Z + CURVE_RISE * 0.72),
                    (sx * 1.34, 0.0, MAIN_Z + CURVE_RISE * 0.45),
                    (sx * 1.42, 0.0, MAIN_Z + CURVE_RISE * 0.15),
                    (sx * 1.49, 0.0, MAIN_Z + CURVE_RISE * 0.05),
                ],
                radius=SUPPORT_R,
                samples_per_segment=10,
                radial_segments=14,
                cap_ends=True,
            )
        )

    # Axle sleeve wrapping the arch-top pivot tube
    sleeve = (
        CylinderGeometry(SLEEVE_R, SLEEVE_LEN, radial_segments=20)
        .rotate_x(math.pi / 2.0)
    )
    weld_post = CylinderGeometry(0.014, MAIN_Z - 0.024, radial_segments=14).translate(
        0.0, 0.0, (MAIN_Z + 0.024) / 2.0
    )
    sleeve.merge(weld_post)

    handlebars: list[MeshGeometry] = []
    for sx in (1.0, -1.0):
        # Handle post height adjusts for the curve at that X position
        local_curve_z = CURVE_RISE * (abs(HANDLE_X) / BEAM_HALF) ** 2
        post_base_z = MAIN_Z + local_curve_z
        post = CylinderGeometry(HANDLE_R, 0.28, radial_segments=14).translate(
            sx * HANDLE_X, 0.0, post_base_z + 0.13
        )
        bar = (
            CylinderGeometry(HANDLE_R, 0.30, radial_segments=14)
            .rotate_x(math.pi / 2.0)
            .translate(sx * HANDLE_X, 0.0, HANDLE_TOP_Z + local_curve_z * 0.3)
        )
        handlebars.append(post.merge(bar))

    # Footrests near each seat (inboard of seat, on the beam)
    footrests: list[MeshGeometry] = []
    for sx in (1.0, -1.0):
        local_curve_z = CURVE_RISE * (abs(FOOTREST_X) / BEAM_HALF) ** 2
        fr_z = MAIN_Z + local_curve_z - 0.02  # slightly below main tube
        fr = _footrest_mesh()
        fr.translate(sx * FOOTREST_X, 0.0, fr_z)
        footrests.append(fr)

    return truss, sleeve, handlebars[0], handlebars[1], footrests[0], footrests[1]


def _add_beam_part(model: ArticulatedObject, part_name: str):
    truss, sleeve, hb0, hb1, fr0, fr1 = _beam_meshes()
    beam = model.part(part_name)
    beam.visual(
        mesh_from_geometry(truss, f"{part_name}_truss"),
        material=WORN_YELLOW,
        name="truss_tube",
    )
    beam.visual(
        mesh_from_geometry(sleeve, f"{part_name}_sleeve"),
        material=WORN_YELLOW,
        name="axle_sleeve",
    )
    beam.visual(
        mesh_from_geometry(hb0, f"{part_name}_handlebar_0"),
        material=WORN_YELLOW,
        name="handlebar_0",
    )
    beam.visual(
        mesh_from_geometry(hb1, f"{part_name}_handlebar_1"),
        material=WORN_YELLOW,
        name="handlebar_1",
    )
    beam.visual(
        mesh_from_geometry(fr0, f"{part_name}_footrest_0"),
        material=FOOTREST_YELLOW,
        name="footrest_0",
    )
    beam.visual(
        mesh_from_geometry(fr1, f"{part_name}_footrest_1"),
        material=FOOTREST_YELLOW,
        name="footrest_1",
    )
    # Seat plates at raised beam ends
    beam.visual(
        Box(SEAT_SIZE),
        origin=Origin(xyz=(SEAT_X, 0.0, SEAT_Z + CURVE_RISE * 0.85)),
        material=RUST_BROWN,
        name="seat_plate_0",
    )
    beam.visual(
        Box(SEAT_SIZE),
        origin=Origin(xyz=(-SEAT_X, 0.0, SEAT_Z + CURVE_RISE * 0.85)),
        material=RUST_BROWN,
        name="seat_plate_1",
    )
    return beam


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="curved_beam_seesaw")

    # --- static sky-blue base with rubber ground pads -------------------------
    base = model.part("base")
    low_axis = (-math.sin(YAW), math.cos(YAW))
    high_axis = (math.sin(YAW), math.cos(YAW))
    base.visual(
        mesh_from_geometry(_arch_mesh(low_axis, LOW_ARCH_TOP), "low_arch"),
        material=SKY_BLUE,
        name="low_arch",
    )
    base.visual(
        mesh_from_geometry(_arch_mesh(high_axis, HIGH_ARCH_TOP), "high_arch"),
        material=SKY_BLUE,
        name="high_arch",
    )
    # Short cross members tying the four legs into one rigid stand.
    leg_y = CROSS_BRACE_U * math.cos(YAW)
    leg_x = CROSS_BRACE_U * math.sin(YAW)
    for idx, sy in enumerate((1.0, -1.0)):
        brace = _tube_between(
            (-leg_x - 0.012, sy * leg_y, CROSS_BRACE_Z),
            (leg_x + 0.012, sy * leg_y, CROSS_BRACE_Z),
            SUPPORT_R,
        )
        base.visual(
            mesh_from_geometry(brace, f"cross_brace_{idx}"),
            material=SKY_BLUE,
            name=f"cross_brace_{idx}",
        )

    # Rubber ground pads under each arch foot (4 feet total: 2 arches x 2 legs)
    # Arch feet are at the ends of each arch profile in the ground plane
    for arch_axis in (low_axis, high_axis):
        ax, ay = arch_axis
        for sign in (1.0, -1.0):
            foot_u = sign * (ARCH_HALF_SPAN + 0.04)
            fx = foot_u * ax
            fy = foot_u * ay
            pad = BoxGeometry(PAD_SIZE)
            pad.translate(fx, fy, PAD_SIZE[2] / 2.0)
            base.visual(
                mesh_from_geometry(pad, f"ground_pad_{int(sign)}_{int(ax*100)}"),
                material=RUBBER_BLACK,
                name=f"ground_pad_{int(sign > 0)}_{int(ax > 0)}",
            )

    # --- two independent yellow rocking beams with curved raised ends ----------
    lower_beam = _add_beam_part(model, "lower_beam")
    upper_beam = _add_beam_part(model, "upper_beam")

    # --- locking pin bracket on the base (welded to the high arch) -----------
    # A bracket arm extends upward from the high arch tube to hold a locking pin.
    # The bracket is part of the base; the pin slides through it.
    pin_mount_y = 0.08  # offset in Y from arch center, beyond sleeve half-extent (0.07)
    pin_mount_z = HIGH_ARCH_TOP  # bracket starts at arch tube top
    bracket_height = 0.10  # bracket extends from arch top upward
    # Vertical arm from arch tube surface upward
    bracket_arm = BoxGeometry((0.025, 0.015, bracket_height))
    bracket_arm.translate(0.0, pin_mount_y, pin_mount_z + bracket_height / 2.0)
    base.visual(
        mesh_from_geometry(bracket_arm, "pin_bracket"),
        material=STEEL_GRAY,
        name="pin_bracket",
    )
    # Small tab at top of bracket with a hole for the pin
    bracket_tab = BoxGeometry((0.04, 0.04, 0.010))
    bracket_tab.translate(0.0, pin_mount_y, pin_mount_z + bracket_height + 0.005)
    base.visual(
        mesh_from_geometry(bracket_tab, "pin_bracket_tab"),
        material=STEEL_GRAY,
        name="pin_bracket_tab",
    )

    # --- locking pin (slides through the bracket) -----------------------------
    locking_pin = model.part("locking_pin")
    pin_len = 0.08
    pin_body = CylinderGeometry(PIN_R, pin_len, radial_segments=14)
    # Small handle ring at top
    pin_handle = (
        CylinderGeometry(PIN_R * 1.8, 0.008, radial_segments=14)
        .translate(0.0, 0.0, pin_len / 2.0 - 0.004)
    )
    pin_body.merge(pin_handle)
    locking_pin.visual(
        mesh_from_geometry(pin_body, "pin_body"),
        material=STEEL_GRAY,
        name="pin_shaft",
    )

    # --- articulations --------------------------------------------------------
    limits = MotionLimits(effort=150.0, velocity=2.5, lower=-TILT, upper=TILT)
    model.articulation(
        "lower_beam_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_beam,
        origin=Origin(xyz=(0.0, 0.0, LOW_ARCH_TOP), rpy=(0.0, 0.0, YAW)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=limits,
    )
    model.articulation(
        "upper_beam_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper_beam,
        origin=Origin(xyz=(0.0, 0.0, HIGH_ARCH_TOP), rpy=(0.0, 0.0, -YAW)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=limits,
    )

    # Locking pin: prismatic joint sliding vertically through the bracket tab.
    # At q=0 the pin is retracted (up); at q=0.05 it slides down to engage.
    pin_origin_z = HIGH_ARCH_TOP + 0.10 + 0.005  # center of bracket tab
    model.articulation(
        "locking_pin_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=locking_pin,
        origin=Origin(xyz=(0.0, pin_mount_y, pin_origin_z)),
        axis=(0.0, 0.0, -1.0),  # slides downward to engage
        motion_limits=MotionLimits(effort=50.0, velocity=0.5, lower=0.0, upper=0.05),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    lower_beam = object_model.get_part("lower_beam")
    upper_beam = object_model.get_part("upper_beam")
    locking_pin = object_model.get_part("locking_pin")
    lower_pivot = object_model.get_articulation("lower_beam_pivot")
    upper_pivot = object_model.get_articulation("upper_beam_pivot")
    pin_slide = object_model.get_articulation("locking_pin_slide")

    # --- Captured-axle fits (same as parent) ----------------------------------
    ctx.allow_overlap(
        lower_beam,
        base,
        elem_a="axle_sleeve",
        elem_b="low_arch",
        reason="Lower beam axle sleeve intentionally wraps the low arch top tube, its pivot axle.",
    )
    ctx.allow_overlap(
        upper_beam,
        base,
        elem_a="axle_sleeve",
        elem_b="high_arch",
        reason="Upper beam axle sleeve intentionally wraps the high arch top tube, its pivot axle.",
    )
    # Locking pin passes through its bracket tab (captured pin fit).
    ctx.allow_overlap(
        base,
        locking_pin,
        elem_a="pin_bracket",
        elem_b="pin_shaft",
        reason="Locking pin shaft slides through the bracket arm, a captured-pin fit.",
    )
    ctx.allow_overlap(
        base,
        locking_pin,
        elem_a="pin_bracket_tab",
        elem_b="pin_shaft",
        reason="Locking pin shaft slides through the bracket tab hole, a captured-pin fit.",
    )
    ctx.expect_contact(
        lower_beam,
        base,
        elem_a="axle_sleeve",
        elem_b="low_arch",
        name="lower beam sleeve rides on the low arch axle",
    )
    ctx.expect_contact(
        upper_beam,
        base,
        elem_a="axle_sleeve",
        elem_b="high_arch",
        name="upper beam sleeve rides on the high arch axle",
    )
    ctx.expect_contact(
        locking_pin,
        base,
        elem_a="pin_shaft",
        elem_b="pin_bracket",
        name="locking pin shaft passes through the bracket tab",
    )

    # --- Variant 26: curved beam with raised ends ----------------------------
    # The beam truss tube ends should be significantly higher than the center.
    for beam_name in ("lower_beam", "upper_beam"):
        beam = object_model.get_part(beam_name)
        truss_aabb = ctx.part_element_world_aabb(beam, elem="truss_tube")
        sleeve_aabb = ctx.part_element_world_aabb(beam, elem="axle_sleeve")
        ctx.check(
            f"{beam_name} has curved raised ends (truss extends well above pivot)",
            truss_aabb is not None
            and sleeve_aabb is not None
            and truss_aabb[1][2] > sleeve_aabb[1][2] + 0.08,
            details=f"truss top={truss_aabb[1][2] if truss_aabb else None}, sleeve top={sleeve_aabb[1][2] if sleeve_aabb else None}",
        )

    # --- Variant 26: rubber ground pads under support legs --------------------
    pad_count = 0
    for v in base.visuals:
        if v.name.startswith("ground_pad_"):
            pad_count += 1
    ctx.check(
        "base has rubber ground pads under the support legs",
        pad_count >= 4,
        details=f"found {pad_count} ground pads",
    )
    # Pads should be at ground level
    for v in base.visuals:
        if v.name.startswith("ground_pad_"):
            pad_aabb = ctx.part_element_world_aabb(base, elem=v.name)
            if pad_aabb is not None:
                ctx.check(
                    f"ground pad {v.name} sits at ground level",
                    pad_aabb[0][2] < 0.02,
                    details=f"pad bottom z={pad_aabb[0][2]:.4f}",
                )

    # --- Variant 26: textured footrests near each seat -----------------------
    for beam_name in ("lower_beam", "upper_beam"):
        beam = object_model.get_part(beam_name)
        for end in (0, 1):
            fr_aabb = ctx.part_element_world_aabb(beam, elem=f"footrest_{end}")
            seat_aabb = ctx.part_element_world_aabb(beam, elem=f"seat_plate_{end}")
            ctx.check(
                f"{beam_name} has textured footrest {end} near its seat",
                fr_aabb is not None and seat_aabb is not None,
                details=f"footrest={fr_aabb}, seat={seat_aabb}",
            )
            if fr_aabb is not None and seat_aabb is not None:
                # Footrest should be inboard of seat (closer to pivot)
                fr_cx = (fr_aabb[0][0] + fr_aabb[1][0]) / 2.0
                seat_cx = (seat_aabb[0][0] + seat_aabb[1][0]) / 2.0
                ctx.check(
                    f"{beam_name} footrest {end} is inboard of its seat",
                    abs(fr_cx) < abs(seat_cx),
                    details=f"footrest x={fr_cx:.3f}, seat x={seat_cx:.3f}",
                )

    # --- Variant 26: locking pin with prismatic joint ------------------------
    ctx.check(
        "locking pin part exists",
        locking_pin is not None,
        details="locking_pin part not found",
    )
    ctx.check(
        "locking pin has a prismatic joint",
        pin_slide is not None and pin_slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"pin_slide type={pin_slide.articulation_type if pin_slide else None}",
    )
    pin_lim = pin_slide.motion_limits if pin_slide else None
    ctx.check(
        "locking pin slides with nonzero travel",
        pin_lim is not None and pin_lim.upper > pin_lim.lower + 0.03,
        details=f"limits=({pin_lim.lower if pin_lim else None}, {pin_lim.upper if pin_lim else None})",
    )
    # Pin should be near the central bracket (high arch top area)
    pin_pos = ctx.part_world_position(locking_pin)
    ctx.check(
        "locking pin is mounted near the central bracket",
        pin_pos is not None and pin_pos[2] > HIGH_ARCH_TOP - 0.10,
        details=f"pin position={pin_pos}",
    )

    # Pin pose check: sliding the pin should move it downward
    rest_pin_z = pin_pos[2] if pin_pos else None
    with ctx.pose({pin_slide: 0.04}):
        extended_pin_pos = ctx.part_world_position(locking_pin)
        ctx.check(
            "locking pin slides downward when engaged",
            rest_pin_z is not None
            and extended_pin_pos is not None
            and extended_pin_pos[2] < rest_pin_z - 0.02,
            details=f"rest z={rest_pin_z}, engaged z={extended_pin_pos[2] if extended_pin_pos else None}",
        )

    # --- Base structure checks (preserved from parent) -----------------------
    base_aabb = ctx.part_world_aabb(base)
    ctx.check(
        "base is an arched stand about 0.7 m tall (including pin bracket)",
        base_aabb is not None and 0.70 <= base_aabb[1][2] <= 0.90,
        details=f"base aabb={base_aabb}",
    )
    ctx.check(
        "base feet rest on the ground plane",
        base_aabb is not None and -0.01 <= base_aabb[0][2] <= 0.015,
        details=f"base aabb={base_aabb}",
    )

    # Seats and handlebars exist at beam ends
    for beam, lo_z, hi_z in ((lower_beam, 0.50, 0.88), (upper_beam, 0.68, 1.06)):
        for end in (0, 1):
            seat = ctx.part_element_world_aabb(beam, elem=f"seat_plate_{end}")
            handle = ctx.part_element_world_aabb(beam, elem=f"handlebar_{end}")
            ok = seat is not None and handle is not None
            ctx.check(
                f"{beam.name} end {end} carries a seat plate and a handlebar",
                ok,
                details=f"seat={seat}, handle={handle}",
            )
            if not ok:
                continue
            scx = (seat[0][0] + seat[1][0]) / 2.0
            scy = (seat[0][1] + seat[1][1]) / 2.0
            scz = (seat[0][2] + seat[1][2]) / 2.0
            ctx.check(
                f"{beam.name} seat {end} sits near the beam end at sit height",
                math.hypot(scx, scy) > 1.25 and lo_z <= scz <= hi_z,
                details=f"seat center=({scx:.3f},{scy:.3f},{scz:.3f})",
            )
            hcx = (handle[0][0] + handle[1][0]) / 2.0
            hcy = (handle[0][1] + handle[1][1]) / 2.0
            inboard = math.hypot(scx - hcx, scy - hcy)
            ctx.check(
                f"{beam.name} handlebar {end} stands upright just inboard of its seat",
                handle[1][2] > seat[1][2] + 0.15 and 0.20 <= inboard <= 0.55,
                details=f"handle top={handle[1][2]:.3f}, seat top={seat[1][2]:.3f}, inboard={inboard:.3f}",
            )

    # Shallow X: beams cross above the base
    ctx.expect_overlap(
        lower_beam,
        upper_beam,
        axes="xy",
        min_overlap=0.5,
        name="beams cross above the base in plan view",
    )

    # Upper beam pivots above lower
    lo_sleeve = ctx.part_element_world_aabb(lower_beam, elem="axle_sleeve")
    up_sleeve = ctx.part_element_world_aabb(upper_beam, elem="axle_sleeve")
    ctx.check(
        "upper beam pivots above the lower beam",
        lo_sleeve is not None
        and up_sleeve is not None
        and (up_sleeve[0][2] + up_sleeve[1][2]) / 2.0
        > (lo_sleeve[0][2] + lo_sleeve[1][2]) / 2.0 + 0.10,
        details=f"lower sleeve={lo_sleeve}, upper sleeve={up_sleeve}",
    )

    # Rocking range is +/- 18 degrees on both pivots
    for pivot in (lower_pivot, upper_pivot):
        lim = pivot.motion_limits
        ctx.check(
            f"{pivot.name} rocks +/- 18 degrees",
            lim is not None
            and abs(lim.lower + TILT) < 1e-6
            and abs(lim.upper - TILT) < 1e-6,
            details=f"limits=({lim.lower if lim else None}, {lim.upper if lim else None})",
        )

    # Decisive pose: lower beam seesaws correctly
    rest_lo0 = ctx.part_element_world_aabb(lower_beam, elem="seat_plate_0")
    rest_lo1 = ctx.part_element_world_aabb(lower_beam, elem="seat_plate_1")
    rest_up0 = ctx.part_element_world_aabb(upper_beam, elem="seat_plate_0")
    with ctx.pose({lower_pivot: TILT}):
        tilt_lo0 = ctx.part_element_world_aabb(lower_beam, elem="seat_plate_0")
        tilt_lo1 = ctx.part_element_world_aabb(lower_beam, elem="seat_plate_1")
        tilt_up0 = ctx.part_element_world_aabb(upper_beam, elem="seat_plate_0")
        beam_aabb = ctx.part_world_aabb(lower_beam)
        ctx.check(
            "lower beam seesaws: one seat drops, the opposite seat rises",
            rest_lo0 is not None
            and tilt_lo0 is not None
            and rest_lo1 is not None
            and tilt_lo1 is not None
            and tilt_lo0[0][2] < rest_lo0[0][2] - 0.35
            and tilt_lo1[0][2] > rest_lo1[0][2] + 0.35,
            details=f"seat0 {rest_lo0} -> {tilt_lo0}, seat1 {rest_lo1} -> {tilt_lo1}",
        )
        ctx.check(
            "fully tilted lower beam stays clear of the ground",
            beam_aabb is not None and beam_aabb[0][2] > 0.02,
            details=f"lower beam aabb={beam_aabb}",
        )
        ctx.check(
            "beams rock independently: upper beam holds still while lower rocks",
            rest_up0 is not None
            and tilt_up0 is not None
            and abs(tilt_up0[0][2] - rest_up0[0][2]) < 1e-6,
            details=f"upper seat0 {rest_up0} -> {tilt_up0}",
        )
        ctx.expect_contact(
            lower_beam,
            base,
            elem_a="axle_sleeve",
            elem_b="low_arch",
            name="tilted lower beam sleeve stays on its axle",
        )

    with ctx.pose({upper_pivot: -TILT}):
        tilt_up0 = ctx.part_element_world_aabb(upper_beam, elem="seat_plate_0")
        beam_aabb = ctx.part_world_aabb(upper_beam)
        ctx.check(
            "upper beam seesaws the opposite way: its near seat rises",
            rest_up0 is not None
            and tilt_up0 is not None
            and tilt_up0[0][2] > rest_up0[0][2] + 0.35,
            details=f"upper seat0 {rest_up0} -> {tilt_up0}",
        )
        ctx.check(
            "fully tilted upper beam stays clear of the ground",
            beam_aabb is not None and beam_aabb[0][2] > 0.02,
            details=f"upper beam aabb={beam_aabb}",
        )
        ctx.expect_contact(
            upper_beam,
            base,
            elem_a="axle_sleeve",
            elem_b="high_arch",
            name="tilted upper beam sleeve stays on its axle",
        )

    return ctx.report()


object_model = build_object_model()
