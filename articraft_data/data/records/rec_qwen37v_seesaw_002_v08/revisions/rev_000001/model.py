from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
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
# Compact backyard playground seesaw with triangular A-frame supports.
#
# Layout (world frame, Z up, base centered on the origin):
# - Base: two triangular A-frame tube supports (front/back along Y), each an
#   inverted-V of sky-blue painted steel tubing, connected by a horizontal
#   axle tube at the top. Dark rubber ground pads sit under each foot.
# - Beam: a single yellow tube beam (~1.6 m) pivots on the axle, with:
#   - rust-brown seat plates at each end
#   - yellow T-handlebars just inboard of each seat
#   - textured footrest plates (ridged dark gray) below each seat
#   - tilting backrest plates on small revolute joints behind each seat
# - Articulation: beam pivots +/- 18 degrees; each backrest tilts +/- 15 deg.
# ----------------------------------------------------------------------------

TUBE_R = 0.020        # ~40 mm diameter main tubing
BRACE_R = 0.016       # diagonal brace tubing
SUPPORT_R = 0.018     # secondary support tubing
HANDLE_R = 0.014      # handlebar tubing

# A-frame support dimensions
FRAME_HEIGHT = 0.55   # pivot height above ground
FRAME_HALF_SPREAD = 0.32  # half-width of each A-frame at ground
FRAME_SEP = 0.22      # front/back separation of the two A-frames (Y direction)
AXLE_LEN = FRAME_SEP + 0.08  # axle tube extending slightly past both frames
AXLE_R = 0.022        # slightly thicker axle tube

# Beam dimensions
BEAM_LEN = 1.60
MAIN_Z = 0.06         # main tube center height above pivot axis
SLEEVE_R = 0.030
SLEEVE_LEN = 0.10

# Seat geometry
SEAT_X = 0.72         # seat center distance from pivot along beam
SEAT_Z = 0.025        # seat plate height above pivot axis
SEAT_SIZE = (0.24, 0.26, 0.010)

# Handlebar
HANDLE_X = 0.50       # handlebar post position along beam
HANDLE_TOP_Z = 0.30   # crossbar height above pivot axis

# Footrest
FOOTREST_X = 0.58     # footrest center along beam
FOOTREST_Z = -0.04    # below pivot axis
FOOTREST_SIZE = (0.14, 0.12, 0.008)

# Backrest
BACKREST_X = 0.82     # backrest hinge position along beam (behind seat)
BACKREST_Z = 0.12     # hinge height above pivot axis
BACKREST_SIZE = (0.20, 0.22, 0.008)
BACKREST_TILT = math.radians(15.0)

# Rubber pad
PAD_SIZE = (0.10, 0.08, 0.012)
PAD_Z = 0.006         # pad center height (half thickness on ground)

# Rocking range
TILT = math.radians(18.0)

# Materials
SKY_BLUE = Material("sky_blue_paint", rgba=(0.33, 0.62, 0.84, 1.0))
WORN_YELLOW = Material("worn_yellow_paint", rgba=(0.85, 0.72, 0.10, 1.0))
RUST_BROWN = Material("rust_brown_steel", rgba=(0.42, 0.21, 0.13, 1.0))
RUBBER_BLACK = Material("rubber_black", rgba=(0.12, 0.12, 0.12, 1.0))
DARK_GRAY = Material("dark_gray_grip", rgba=(0.28, 0.28, 0.30, 1.0))


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


def _aframe_mesh(y_pos: float) -> MeshGeometry:
    """Build one triangular A-frame support tube at given Y position.

    The A-frame is an inverted V: two legs rise from spread ground feet to
    meet at the apex (the pivot height). A horizontal cross-brace ties the
    legs together partway up for rigidity.
    """
    apex = (0.0, y_pos, FRAME_HEIGHT)
    foot_neg = (-FRAME_HALF_SPREAD, y_pos, 0.0)
    foot_pos = (FRAME_HALF_SPREAD, y_pos, 0.0)

    # Left leg (negative X foot to apex)
    geom = _tube_between(foot_neg, apex, TUBE_R)
    # Right leg (positive X foot to apex)
    geom.merge(_tube_between(foot_pos, apex, TUBE_R))

    # Cross brace at ~40% height
    brace_z = FRAME_HEIGHT * 0.40
    frac = brace_z / FRAME_HEIGHT
    bx_neg = -FRAME_HALF_SPREAD * (1.0 - frac)
    bx_pos = FRAME_HALF_SPREAD * (1.0 - frac)
    geom.merge(_tube_between(
        (bx_neg, y_pos, brace_z),
        (bx_pos, y_pos, brace_z),
        BRACE_R,
    ))

    return geom


def _rubber_pad_mesh(y_pos: float, x_sign: float) -> MeshGeometry:
    """Rubber ground pad at a foot position."""
    geom = CylinderGeometry(0.045, PAD_SIZE[2], radial_segments=18)
    geom.translate(x_sign * FRAME_HALF_SPREAD, y_pos, PAD_Z)
    return geom


def _footrest_mesh(sx: float) -> MeshGeometry:
    """Textured footrest plate with grip ridges near a seat."""
    # Base plate
    plate = CylinderGeometry(0.06, FOOTREST_SIZE[2], radial_segments=6)
    plate.translate(sx * FOOTREST_X, 0.0, FOOTREST_Z)

    # Add grip ridges (small raised bars across the plate)
    for i in range(4):
        offset = -0.04 + i * 0.027
        ridge = CylinderGeometry(0.004, 0.10, radial_segments=8)
        ridge.rotate_x(math.pi / 2.0)
        ridge.translate(sx * FOOTREST_X, offset, FOOTREST_Z + 0.006)
        plate.merge(ridge)

    return plate


def _backrest_mesh(sx: float) -> MeshGeometry:
    """Backrest plate in its own local frame (origin at hinge point).

    The stem extends slightly below the hinge origin (negative Z) so it
    wraps the hinge bracket on the beam (captured-pin hinge).
    """
    # Flat plate extending upward from the hinge
    plate = (
        CylinderGeometry(0.09, BACKREST_SIZE[2], radial_segments=6)
        .rotate_x(math.pi / 2.0)
        .translate(0.0, 0.0, 0.10)  # extends upward from hinge
    )
    # Stem wrapping below the hinge (captured pin) and extending up to plate
    stem_length = 0.10  # from z=-0.02 to z=+0.08
    stem = CylinderGeometry(0.012, stem_length, radial_segments=12).translate(
        0.0, 0.0, stem_length / 2.0 - 0.02
    )
    plate.merge(stem)
    return plate


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_backyard_seesaw")

    # --- Base: triangular A-frame supports with rubber pads -------------------
    base = model.part("base")

    # Two A-frames: front (+Y) and back (-Y)
    for idx, sy in enumerate((1.0, -1.0)):
        y_pos = sy * FRAME_SEP / 2.0
        base.visual(
            mesh_from_geometry(_aframe_mesh(y_pos), f"aframe_{idx}"),
            material=SKY_BLUE,
            name=f"aframe_{idx}",
        )

    # Horizontal axle tube connecting both frame apexes
    axle_geom = (
        CylinderGeometry(AXLE_R, AXLE_LEN, radial_segments=20)
        .rotate_x(math.pi / 2.0)
        .translate(0.0, 0.0, FRAME_HEIGHT)
    )
    base.visual(
        mesh_from_geometry(axle_geom, "axle_bar"),
        material=SKY_BLUE,
        name="axle_bar",
    )

    # Longitudinal tie bar connecting the two A-frames at mid-height for rigidity
    tie_z = FRAME_HEIGHT * 0.40
    tie_geom = (
        CylinderGeometry(BRACE_R, FRAME_SEP + 0.02, radial_segments=12)
        .rotate_x(math.pi / 2.0)
        .translate(0.0, 0.0, tie_z)
    )
    base.visual(
        mesh_from_geometry(tie_geom, "tie_bar"),
        material=SKY_BLUE,
        name="tie_bar",
    )

    # Rubber ground pads under all four feet
    for idx, (sy, sx_sign) in enumerate(
        [(1.0, 1.0), (1.0, -1.0), (-1.0, 1.0), (-1.0, -1.0)]
    ):
        pad = _rubber_pad_mesh(sy * FRAME_SEP / 2.0, sx_sign)
        base.visual(
            mesh_from_geometry(pad, f"rubber_pad_{idx}"),
            material=RUBBER_BLACK,
            name=f"rubber_pad_{idx}",
        )

    # --- Main rocking beam ----------------------------------------------------
    beam = model.part("beam")

    # Main beam tube
    truss = (
        CylinderGeometry(TUBE_R, BEAM_LEN, radial_segments=18)
        .rotate_y(math.pi / 2.0)
        .translate(0.0, 0.0, MAIN_Z)
    )

    # Diagonal braces from sleeve to main tube (triangulated)
    for sx in (1.0, -1.0):
        truss.merge(_tube_between(
            (sx * 0.04, 0.0, 0.005),
            (sx * 0.45, 0.0, MAIN_Z),
            BRACE_R,
        ))
        # Seat support tube dropping from main tube to seat plate
        truss.merge(tube_from_spline_points(
            [
                (sx * 0.58, 0.0, MAIN_Z),
                (sx * 0.65, 0.0, 0.04),
                (sx * SEAT_X, 0.0, SEAT_Z + SEAT_SIZE[2] / 2.0),
            ],
            radius=SUPPORT_R,
            samples_per_segment=8,
            radial_segments=14,
            cap_ends=True,
        ))
        # Backrest hinge bracket: spline from main tube center up to hinge
        truss.merge(tube_from_spline_points(
            [
                (sx * BACKREST_X, 0.0, MAIN_Z),
                (sx * BACKREST_X, 0.0, MAIN_Z + TUBE_R),
                (sx * BACKREST_X, 0.0, BACKREST_Z + MAIN_Z),
            ],
            radius=BRACE_R,
            samples_per_segment=6,
            radial_segments=12,
            cap_ends=True,
        ))
        # Footrest support: spline from main tube center down to footrest plate
        truss.merge(tube_from_spline_points(
            [
                (sx * FOOTREST_X, 0.0, MAIN_Z),
                (sx * FOOTREST_X, 0.0, MAIN_Z - TUBE_R),
                (sx * FOOTREST_X, 0.0, FOOTREST_Z + FOOTREST_SIZE[2] / 2.0),
            ],
            radius=BRACE_R,
            samples_per_segment=6,
            radial_segments=12,
            cap_ends=True,
        ))

    beam.visual(
        mesh_from_geometry(truss, "beam_truss"),
        material=WORN_YELLOW,
        name="truss_tube",
    )

    # Axle sleeve at center (wraps the axle bar)
    sleeve = (
        CylinderGeometry(SLEEVE_R, SLEEVE_LEN, radial_segments=20)
        .rotate_x(math.pi / 2.0)
    )
    # Weld post connecting sleeve to main tube
    weld_post = CylinderGeometry(0.012, MAIN_Z - 0.022, radial_segments=12).translate(
        0.0, 0.0, (MAIN_Z + 0.022) / 2.0
    )
    sleeve.merge(weld_post)
    beam.visual(
        mesh_from_geometry(sleeve, "beam_sleeve"),
        material=WORN_YELLOW,
        name="axle_sleeve",
    )

    # Seat plates
    for idx, sx in enumerate((1.0, -1.0)):
        beam.visual(
            Box(SEAT_SIZE),
            origin=Origin(xyz=(sx * SEAT_X, 0.0, SEAT_Z)),
            material=RUST_BROWN,
            name=f"seat_plate_{idx}",
        )

    # Handlebars (T-shaped, post connects truss tube to crossbar)
    handle_post_bottom = MAIN_Z + TUBE_R  # top of main truss tube
    handle_post_length = HANDLE_TOP_Z - handle_post_bottom
    handle_post_center = (handle_post_bottom + HANDLE_TOP_Z) / 2.0
    for idx, sx in enumerate((1.0, -1.0)):
        post = CylinderGeometry(HANDLE_R, handle_post_length, radial_segments=12).translate(
            sx * HANDLE_X, 0.0, handle_post_center
        )
        bar = (
            CylinderGeometry(HANDLE_R, 0.24, radial_segments=12)
            .rotate_x(math.pi / 2.0)
            .translate(sx * HANDLE_X, 0.0, HANDLE_TOP_Z)
        )
        handlebar = post.merge(bar)
        beam.visual(
            mesh_from_geometry(handlebar, f"handlebar_{idx}"),
            material=WORN_YELLOW,
            name=f"handlebar_{idx}",
        )

    # Textured footrests near each seat
    for idx, sx in enumerate((1.0, -1.0)):
        footrest = _footrest_mesh(sx)
        beam.visual(
            mesh_from_geometry(footrest, f"footrest_{idx}"),
            material=DARK_GRAY,
            name=f"footrest_{idx}",
        )

    # --- Tilting backrests (separate parts on revolute joints) ----------------
    for idx, sx in enumerate((1.0, -1.0)):
        backrest = model.part(f"backrest_{idx}")
        backrest.visual(
            mesh_from_geometry(_backrest_mesh(sx), f"backrest_plate_{idx}"),
            material=WORN_YELLOW,
            name="backrest_plate",
        )

    # --- Articulations --------------------------------------------------------
    # Main beam pivot (revolute, horizontal axis along Y perpendicular to beam)
    beam_pivot_limits = MotionLimits(
        effort=80.0, velocity=2.0, lower=-TILT, upper=TILT
    )
    model.articulation(
        "beam_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=beam,
        origin=Origin(xyz=(0.0, 0.0, FRAME_HEIGHT)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=beam_pivot_limits,
    )

    # Backrest tilt joints (revolute, axis along Y so backrest tilts forward/back)
    backrest_limits = MotionLimits(
        effort=5.0, velocity=1.5, lower=-BACKREST_TILT, upper=BACKREST_TILT
    )
    for idx, sx in enumerate((1.0, -1.0)):
        backrest_part = model.get_part(f"backrest_{idx}")
        model.articulation(
            f"backrest_tilt_{idx}",
            ArticulationType.REVOLUTE,
            parent=beam,
            child=backrest_part,
            origin=Origin(xyz=(sx * BACKREST_X, 0.0, BACKREST_Z + MAIN_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=backrest_limits,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    beam = object_model.get_part("beam")
    backrest_0 = object_model.get_part("backrest_0")
    backrest_1 = object_model.get_part("backrest_1")
    beam_pivot = object_model.get_articulation("beam_pivot")
    backrest_tilt_0 = object_model.get_articulation("backrest_tilt_0")
    backrest_tilt_1 = object_model.get_articulation("backrest_tilt_1")

    # --- Triangular support structure checks ---
    aframe_0 = ctx.part_element_world_aabb(base, elem="aframe_0")
    aframe_1 = ctx.part_element_world_aabb(base, elem="aframe_1")
    ctx.check(
        "base has two triangular A-frame supports",
        aframe_0 is not None and aframe_1 is not None,
        details=f"aframe_0={aframe_0}, aframe_1={aframe_1}",
    )

    # A-frames should be roughly at the designed height
    if aframe_0 is not None:
        frame_top = aframe_0[1][2]
        ctx.check(
            "A-frame supports reach the designed pivot height (~0.55 m)",
            0.50 <= frame_top <= 0.62,
            details=f"aframe top z={frame_top:.3f}",
        )

    # A-frames have spread feet (wider at ground than at top)
    if aframe_0 is not None:
        frame_width = aframe_0[1][0] - aframe_0[0][0]
        ctx.check(
            "A-frame feet spread wider than the pivot point (~0.64 m span)",
            frame_width > 0.50,
            details=f"aframe width={frame_width:.3f}",
        )

    # --- Rubber ground pads ---
    pad_count = 0
    for i in range(4):
        pad = ctx.part_element_world_aabb(base, elem=f"rubber_pad_{i}")
        if pad is not None:
            pad_count += 1
            ctx.check(
                f"rubber pad {i} sits on the ground",
                pad[0][2] < 0.02 and pad[1][2] < 0.025,
                details=f"pad aabb={pad}",
            )
    ctx.check(
        "all four rubber ground pads present under support legs",
        pad_count == 4,
        details=f"found {pad_count}/4 pads",
    )

    # --- Axle sleeve wraps axle bar (intentional overlap) ---
    ctx.allow_overlap(
        beam,
        base,
        elem_a="axle_sleeve",
        elem_b="axle_bar",
        reason="Beam axle sleeve intentionally wraps the axle bar as its pivot bearing.",
    )
    ctx.expect_contact(
        beam,
        base,
        elem_a="axle_sleeve",
        elem_b="axle_bar",
        name="beam sleeve rides on the axle bar",
    )

    # --- Backrest stems wrap beam hinge brackets (captured-pin hinge) ---
    for idx in (0, 1):
        br_part = object_model.get_part(f"backrest_{idx}")
        ctx.allow_overlap(
            br_part,
            beam,
            elem_a="backrest_plate",
            elem_b="truss_tube",
            reason=f"Backrest {idx} stem wraps the beam hinge bracket as a captured-pin hinge.",
        )
        ctx.expect_contact(
            br_part,
            beam,
            elem_a="backrest_plate",
            elem_b="truss_tube",
            name=f"backrest_{idx} hinge pin contacts the beam bracket",
        )

    # --- Beam pivot checks ---
    pivot_lim = beam_pivot.motion_limits
    ctx.check(
        "beam pivot rocks +/- 18 degrees",
        pivot_lim is not None
        and abs(pivot_lim.lower + TILT) < 1e-6
        and abs(pivot_lim.upper - TILT) < 1e-6,
        details=f"limits=({pivot_lim.lower if pivot_lim else None}, {pivot_lim.upper if pivot_lim else None})",
    )

    # --- Backrest tilt joints ---
    for tilt_joint, name in [
        (backrest_tilt_0, "backrest_tilt_0"),
        (backrest_tilt_1, "backrest_tilt_1"),
    ]:
        lim = tilt_joint.motion_limits
        ctx.check(
            f"{name} tilts +/- 15 degrees",
            lim is not None
            and abs(lim.lower + BACKREST_TILT) < 1e-3
            and abs(lim.upper - BACKREST_TILT) < 1e-3,
            details=f"limits=({lim.lower if lim else None}, {lim.upper if lim else None})",
        )

    # Backrests exist as separate articulated parts
    for idx in (0, 1):
        br = ctx.part_element_world_aabb(
            object_model.get_part(f"backrest_{idx}"), elem="backrest_plate"
        )
        ctx.check(
            f"backrest_{idx} exists as a separate tilting part",
            br is not None,
            details=f"backrest aabb={br}",
        )

    # --- Textured footrests ---
    for idx in (0, 1):
        fr = ctx.part_element_world_aabb(beam, elem=f"footrest_{idx}")
        ctx.check(
            f"footrest_{idx} exists near its seat on the beam",
            fr is not None,
            details=f"footrest aabb={fr}",
        )
        if fr is not None:
            seat = ctx.part_element_world_aabb(beam, elem=f"seat_plate_{idx}")
            if seat is not None:
                seat_cx = (seat[0][0] + seat[1][0]) / 2.0
                fr_cx = (fr[0][0] + fr[1][0]) / 2.0
                ctx.check(
                    f"footrest_{idx} is positioned inboard of seat_{idx}",
                    abs(fr_cx) < abs(seat_cx),
                    details=f"footrest_cx={fr_cx:.3f}, seat_cx={seat_cx:.3f}",
                )

    # --- Seat plates and handlebars exist ---
    for idx in (0, 1):
        seat = ctx.part_element_world_aabb(beam, elem=f"seat_plate_{idx}")
        handle = ctx.part_element_world_aabb(beam, elem=f"handlebar_{idx}")
        ctx.check(
            f"beam end {idx} carries a seat plate and handlebar",
            seat is not None and handle is not None,
            details=f"seat={seat}, handle={handle}",
        )

    # --- Decisive pose: beam seesaws ---
    rest_seat0 = ctx.part_element_world_aabb(beam, elem="seat_plate_0")
    rest_seat1 = ctx.part_element_world_aabb(beam, elem="seat_plate_1")
    with ctx.pose({beam_pivot: TILT}):
        tilt_seat0 = ctx.part_element_world_aabb(beam, elem="seat_plate_0")
        tilt_seat1 = ctx.part_element_world_aabb(beam, elem="seat_plate_1")
        beam_aabb = ctx.part_world_aabb(beam)
        ctx.check(
            "beam seesaws: one seat drops while the other rises",
            rest_seat0 is not None
            and tilt_seat0 is not None
            and rest_seat1 is not None
            and tilt_seat1 is not None
            and tilt_seat0[0][2] < rest_seat0[0][2] - 0.15
            and tilt_seat1[0][2] > rest_seat1[0][2] + 0.15,
            details=f"seat0 {rest_seat0} -> {tilt_seat0}, seat1 {rest_seat1} -> {tilt_seat1}",
        )
        ctx.check(
            "fully tilted beam stays clear of the ground",
            beam_aabb is not None and beam_aabb[0][2] > 0.01,
            details=f"beam aabb={beam_aabb}",
        )
        ctx.expect_contact(
            beam,
            base,
            elem_a="axle_sleeve",
            elem_b="axle_bar",
            name="tilted beam sleeve stays on axle",
        )

    # --- Decisive pose: backrest tilts ---
    br0_rest = ctx.part_element_world_aabb(backrest_0, elem="backrest_plate")
    with ctx.pose({backrest_tilt_0: BACKREST_TILT}):
        br0_tilted = ctx.part_element_world_aabb(backrest_0, elem="backrest_plate")
        ctx.check(
            "backrest_0 visibly tilts when its joint is actuated",
            br0_rest is not None
            and br0_tilted is not None
            and (abs(br0_tilted[0][0] - br0_rest[0][0]) > 0.005
                 or abs(br0_tilted[0][2] - br0_rest[0][2]) > 0.005),
            details=f"rest={br0_rest}, tilted={br0_tilted}",
        )

    return ctx.report()


object_model = build_object_model()
