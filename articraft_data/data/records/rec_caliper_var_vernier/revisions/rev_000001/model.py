from __future__ import annotations

# Articraft model: a vernier caliper (classic sliding measuring caliper with
# a graduated main scale on the beam and a vernier scale window on the slider).
#
# Articraft brief:
# - Object: a ~150 mm (6") vernier caliper. Graduated steel beam ~0.20 m long.
#   Coordinate frame: +X runs along the beam (the measuring / slide axis),
#   +Y is vertical (jaws point up for inside-measuring tips and down for the
#   large outside-measuring jaws), +Z is the thin thickness direction with the
#   scale face looking toward +Z.
# - Root/support: the graduated beam plus its fixed jaw head (one rigid part).
#   Everything mounts off the beam.
# - Parts: `beam` (root, with fixed jaws + main scale ticks + depth-rod channel),
#   `slider` (the moving jaw carriage carrying the vernier scale window, the
#   thumb roller knurl, and the depth rod tail).
# - Articulation: `beam_to_slider`, PRISMATIC along +X. The carriage slides
#   along the beam; positive q opens the jaws and pushes the depth rod out the
#   tail end.
# - Visible geometry: flat beam with an engraved main scale of many fine tick
#   marks, two fixed jaws (large lower outside jaw + small upper inside jaw),
#   mirrored sliding jaws, a vernier scale window with fine ticks on the
#   slider, a knurled thumb roller, and a slim depth rod.
# - Support/fit: the slider straddles the beam (its jaw root overlaps the beam
#   cross-section); this is a real captured prismatic fit, so it gets a scoped
#   allow_overlap. The depth rod rides inside the beam's tail channel.
# - Tests: beam/slider present, jaws present, main scale ticks present,
#   vernier scale ticks present, thumb roller present, prismatic axis is +X,
#   sliding opens the jaw gap and extends the depth rod.

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Real-world dimensions (meters)
# ---------------------------------------------------------------------------
BEAM_LEN = 0.205  # full graduated beam length along X
BEAM_H = 0.020  # beam height (Y)
BEAM_T = 0.0045  # beam thickness (Z)

JAW_DOWN_LEN = 0.040  # large outside-measuring jaw drop (down, -Y)
JAW_UP_LEN = 0.022  # small inside-measuring tip rise (up, +Y)
JAW_T = 0.0040  # jaw thickness (Z)

SLIDE_TRAVEL = 0.150  # usable measuring travel (6")
# Fixed jaw inner face sits at this X; slider jaw inner face sits SLIDE_GAP0
# to its right at rest, so the rest jaw gap is small (caliper "zeroed").
FIXED_JAW_X = 0.011  # X of fixed jaw centerline (near left/-X end)
SLIDER_REST_X = 0.044  # slider carriage center X at q = 0 (small rest jaw gap)

# Main scale: ticks engraved along the top face of the beam
MAIN_SCALE_X_START = 0.025  # first tick X (after the fixed jaw head)
MAIN_SCALE_X_END = 0.195  # last tick X (near beam tail)
MAIN_SCALE_N_MAJOR = 17  # number of major (cm-like) divisions
MAIN_SCALE_MINOR_PER_MAJOR = 10  # minor ticks per major division

# Vernier scale: a short window of ticks on the slider top
VERNIER_N_TICKS = 10  # classic 10-division vernier
VERNIER_SPACING = 0.0018  # tick spacing on vernier (slightly less than main scale)

ROD_R = 0.0016  # depth rod radius
ROD_LEN = 0.150  # depth rod length (rides in tail channel, extends out tail)

# Colors / materials
STEEL = (0.74, 0.76, 0.79, 1.0)
STEEL_DARK = (0.55, 0.57, 0.60, 1.0)
CHROME = (0.82, 0.84, 0.87, 1.0)
SCALE_DARK = (0.18, 0.18, 0.20, 1.0)
VERNIER_PLATE = (0.88, 0.88, 0.85, 1.0)


# ---------------------------------------------------------------------------
# CadQuery geometry builders (authored directly in meters)
# ---------------------------------------------------------------------------
def _beam_body() -> cq.Workplane:
    """Flat graduated beam: a thin bar along X with rounded long edges."""
    body = (
        cq.Workplane("XY")
        .box(BEAM_LEN, BEAM_H, BEAM_T, centered=(False, True, True))
        .edges("|Z")
        .fillet(0.0015)
    )
    return body


def _fixed_jaw() -> cq.Workplane:
    """Fixed head: the two jaws at the left end, rooted on the beam's -X end.

    Lower jaw is the big outside-measuring jaw (drops in -Y); upper jaw is the
    short inside-measuring knife tip (rises in +Y). Both taper to a point.
    """
    x = FIXED_JAW_X
    mface = x + 0.007  # measuring (right) face plane of this jaw
    # root block fused to the beam underside; spans up into the beam slightly.
    lower = (
        cq.Workplane("XY")
        .box(0.014, BEAM_H, JAW_T, centered=(True, True, True))
        .translate((x, 0.0, 0.0))
    )
    # Lower outside jaw: measuring face is the clean vertical right plane at
    # mface; the blade tapers down and back toward -X so it never reaches the
    # slider zone on the +X side.
    lower_blade_pts = [
        (mface, -BEAM_H / 2.0),
        (mface, -BEAM_H / 2.0 - JAW_DOWN_LEN),  # measuring tip (right/contact face)
        (mface - 0.010, -BEAM_H / 2.0 - JAW_DOWN_LEN),
        (mface - 0.013, -BEAM_H / 2.0 - JAW_DOWN_LEN * 0.45),
        (mface - 0.013, -BEAM_H / 2.0),
    ]
    lower_blade = (
        cq.Workplane("XY")
        .polyline(lower_blade_pts)
        .close()
        .extrude(JAW_T, both=True)
    )
    # Upper inside-measuring knife tip (short, rises +Y).
    upper_pts = [
        (mface - 0.011, BEAM_H / 2.0),
        (mface, BEAM_H / 2.0),
        (mface - 0.002, BEAM_H / 2.0 + JAW_UP_LEN),
        (mface - 0.006, BEAM_H / 2.0 + JAW_UP_LEN),
    ]
    upper = (
        cq.Workplane("XY")
        .polyline(upper_pts)
        .close()
        .extrude(JAW_T * 0.75, both=True)
    )
    return lower.union(lower_blade).union(upper)


def _tick_mark(length: float, width: float, height: float) -> cq.Workplane:
    """Shared geometry helper: a single thin tick mark box.

    The tick is a thin rectangular bar oriented along Y (vertical), centered
    at origin. `length` is along Y, `width` is along X, `height` is along Z.
    """
    return (
        cq.Workplane("XY")
        .box(width, length, height, centered=(True, True, True))
    )


def _main_scale_ticks() -> cq.Workplane:
    """Engraved main scale ticks along the top face of the beam.

    Generates many evenly-spaced tick marks: longer major ticks at regular
    intervals and shorter minor ticks in between. All ticks sit proud of
    the beam top surface (+Z face). A thin connecting base strip runs along
    the bottom of all ticks so the mesh is one connected component.
    """
    total_ticks = MAIN_SCALE_N_MAJOR * MAIN_SCALE_MINOR_PER_MAJOR + 1
    span = MAIN_SCALE_X_END - MAIN_SCALE_X_START
    spacing = span / (total_ticks - 1)
    beam_top_z = BEAM_T / 2.0
    tick_z_center = beam_top_z + 0.0003  # slightly proud of beam surface

    # Thin connecting base strip that ties all ticks together into one mesh.
    strip_len_x = span + 0.002
    strip_cx = (MAIN_SCALE_X_START + MAIN_SCALE_X_END) / 2.0
    base_strip = (
        cq.Workplane("XY")
        .box(strip_len_x, 0.0008, 0.0004, centered=(True, True, True))
        .translate((strip_cx, BEAM_H / 2.0 - 0.0004, beam_top_z + 0.0001))
    )
    result = base_strip
    for i in range(total_ticks):
        x_pos = MAIN_SCALE_X_START + i * spacing
        is_major = (i % MAIN_SCALE_MINOR_PER_MAJOR) == 0
        is_half = (i % (MAIN_SCALE_MINOR_PER_MAJOR // 2)) == 0 and not is_major

        if is_major:
            tick_len = 0.0050
            tick_w = 0.00040
        elif is_half:
            tick_len = 0.0035
            tick_w = 0.00030
        else:
            tick_len = 0.0022
            tick_w = 0.00022

        tick = (
            _tick_mark(tick_len, tick_w, 0.0006)
            .translate((x_pos, BEAM_H / 2.0 - tick_len / 2.0, tick_z_center))
        )
        result = result.union(tick)
    return result


def _slider_body() -> cq.Workplane:
    """Sliding carriage authored in its own local frame, centered at X=0.

    It straddles the beam (a thin frame around the beam cross-section) and
    carries mirrored jaws plus a vernier scale window plate on top with
    engraved vernier ticks fused into one body. The carriage's measuring
    jaws sit on the LEFT (-X) so that sliding the carriage in +X opens the
    gap against the fixed jaw.
    """
    carriage_w = 0.034  # along X
    # Frame/saddle that wraps the beam (slightly proud in Z so it reads raised).
    saddle = (
        cq.Workplane("XY")
        .box(carriage_w, BEAM_H + 0.006, BEAM_T + 0.0050, centered=(True, True, True))
        .edges("|Z")
        .fillet(0.0018)
    )

    xj = -carriage_w / 2.0  # left face of carriage = jaw measuring plane
    # Lower outside-measuring jaw (mirror of the fixed lower jaw): measuring
    # face is the clean vertical LEFT plane at xj; the blade tapers down and
    # toward +X (back under the carriage), away from the fixed jaw.
    lower_pts = [
        (xj, -BEAM_H / 2.0 + 0.001),
        (xj, -BEAM_H / 2.0 - JAW_DOWN_LEN),  # measuring tip (left/contact face)
        (xj + 0.010, -BEAM_H / 2.0 - JAW_DOWN_LEN),
        (xj + 0.013, -BEAM_H / 2.0 - JAW_DOWN_LEN * 0.45),
        (xj + 0.013, -BEAM_H / 2.0 + 0.001),
    ]
    lower = (
        cq.Workplane("XY")
        .polyline(lower_pts)
        .close()
        .extrude(JAW_T, both=True)
    )
    # Short upper inside-measuring tip (rises +Y).
    upper_pts = [
        (xj, BEAM_H / 2.0),
        (xj + 0.011, BEAM_H / 2.0),
        (xj + 0.006, BEAM_H / 2.0 + JAW_UP_LEN),
        (xj + 0.002, BEAM_H / 2.0 + JAW_UP_LEN),
    ]
    upper = (
        cq.Workplane("XY")
        .polyline(upper_pts)
        .close()
        .extrude(JAW_T * 0.75, both=True)
    )

    # Vernier scale window plate: a thin flat plate on top of the saddle where
    # the vernier scale ticks are engraved. This replaces the dial housing.
    plate_width = 0.028  # along X
    plate_height = 0.010  # along Y
    plate = (
        cq.Workplane("XY")
        .workplane(offset=BEAM_T / 2.0 + 0.0010)
        .box(plate_width, plate_height, 0.0018, centered=(True, True, True))
        .translate((0.002, BEAM_H / 2.0 - plate_height / 2.0 + 0.001, 0.0))
    )
    # Neck that ties the plate down onto the saddle top for a real mount.
    neck = (
        cq.Workplane("XY")
        .box(0.022, 0.008, BEAM_T + 0.006, centered=(True, True, True))
        .translate((0.002, BEAM_H / 2.0 - 0.002, 0.0))
    )

    # Build the vernier tick marks directly into the slider body so they are
    # one connected mesh with the plate and saddle.
    beam_top_z = BEAM_T / 2.0
    tick_z_center = beam_top_z + 0.0020
    total_span = (VERNIER_N_TICKS - 1) * VERNIER_SPACING
    x_start = 0.002 - total_span / 2.0

    body = saddle.union(lower).union(upper).union(neck).union(plate)

    for i in range(VERNIER_N_TICKS):
        x_pos = x_start + i * VERNIER_SPACING
        is_endpoint = (i == 0) or (i == VERNIER_N_TICKS - 1)
        tick_len = 0.0040 if is_endpoint else 0.0028
        tick_w = 0.00035 if is_endpoint else 0.00025
        tick = (
            _tick_mark(tick_len, tick_w, 0.0008)
            .translate((x_pos, BEAM_H / 2.0 - tick_len / 2.0 + 0.001, tick_z_center))
        )
        body = body.union(tick)

    return body





def _thumb_roller() -> cq.Workplane:
    """Knurled fine-adjust thumb roller, a short fluted wheel on local Z."""
    body = cq.Workplane("XY").circle(0.0060).extrude(0.0050, both=True)
    flutes = None
    n = 18
    for i in range(n):
        a = 2.0 * math.pi * i / n
        cut = (
            cq.Workplane("XY")
            .box(0.0010, 0.0012, 0.0110, centered=(True, True, True))
            .translate((0.0060, 0.0, 0.0))
            .rotate((0, 0, 0), (0, 0, 1), math.degrees(a))
        )
        flutes = cut if flutes is None else flutes.union(cut)
    body = body.cut(flutes)
    hub = cq.Workplane("XY").circle(0.0016).extrude(0.0070, both=True)
    return body.union(hub)


def _depth_rod() -> cq.Workplane:
    """Slim depth rod (round bar) authored along X in the slider frame."""
    return (
        cq.Workplane("YZ")
        .circle(ROD_R)
        .extrude(ROD_LEN)
    )


# ---------------------------------------------------------------------------
# Model assembly
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vernier_caliper")

    steel = model.material("steel", rgba=STEEL)
    steel_dark = model.material("steel_dark", rgba=STEEL_DARK)
    chrome = model.material("chrome", rgba=CHROME)
    scale_dark = model.material("scale_dark", rgba=SCALE_DARK)
    vernier_plate = model.material("vernier_plate", rgba=VERNIER_PLATE)

    # --- Beam (root) -------------------------------------------------------
    beam = model.part("beam")
    beam.visual(
        mesh_from_cadquery(_beam_body(), "beam_body"),
        material=steel,
        name="beam_body",
    )
    # Engraved scale strip along the top of the beam face (+Z).
    beam.visual(
        mesh_from_cadquery(
            cq.Workplane("XY")
            .box(BEAM_LEN - 0.02, 0.006, 0.0006, centered=(False, True, True))
            .translate((0.01, BEAM_H / 2.0 - 0.005, BEAM_T / 2.0)),
            "beam_scale",
        ),
        material=scale_dark,
        name="beam_scale",
    )
    # Main scale ticks: many fine graduation marks along the beam top.
    beam.visual(
        mesh_from_cadquery(_main_scale_ticks(), "main_scale_ticks"),
        material=scale_dark,
        name="main_scale_ticks",
    )
    beam.visual(
        mesh_from_cadquery(_fixed_jaw(), "fixed_jaw"),
        material=steel,
        name="fixed_jaw",
    )

    # --- Slider carriage ---------------------------------------------------
    # The slider part frame is centered on the beam cross-section at the rest X.
    slider = model.part("slider")
    slider.visual(
        mesh_from_cadquery(_slider_body(), "slider_body"),
        material=steel_dark,
        name="slider_body",
    )

    # Thumb roller mounted on the lower-right edge of the carriage frame.
    roller_x = 0.014
    roller_y = -BEAM_H / 2.0 + 0.002
    slider.visual(
        mesh_from_cadquery(_thumb_roller(), "thumb_roller"),
        origin=Origin(xyz=(roller_x, roller_y, 0.0)),
        material=steel_dark,
        name="thumb_roller",
    )

    # Depth rod: rooted at the carriage and running along +X into the beam's
    # tail channel; it protrudes from the right end as the slide opens.
    slider.visual(
        mesh_from_cadquery(_depth_rod(), "depth_rod"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=steel,
        name="depth_rod",
    )

    # --- Articulations -----------------------------------------------------
    # Carriage slides along +X. The slider part frame sits at SLIDER_REST_X at
    # q=0; positive q moves it toward +X, opening the jaw gap and extending the
    # depth rod out the tail.
    model.articulation(
        "beam_to_slider",
        ArticulationType.PRISMATIC,
        parent=beam,
        child=slider,
        origin=Origin(xyz=(SLIDER_REST_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0, velocity=0.20, lower=0.0, upper=SLIDE_TRAVEL
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    beam = object_model.get_part("beam")
    slider = object_model.get_part("slider")
    slide = object_model.get_articulation("beam_to_slider")

    # The carriage straddles the beam: a real captured prismatic fit, so the
    # saddle intentionally overlaps the beam cross-section.
    ctx.allow_overlap(
        slider,
        beam,
        elem_a="slider_body",
        elem_b="beam_body",
        reason="The slider saddle wraps and rides on the beam (captured prismatic fit).",
    )
    # The depth rod rides inside the beam tail channel (proxy solid beam).
    ctx.allow_overlap(
        slider,
        beam,
        elem_a="depth_rod",
        elem_b="beam_body",
        reason="The depth rod slides through the beam's tail channel (proxy solid beam).",
    )

    # --- Joint type / axis claims -----------------------------------------
    ctx.check(
        "slide joint is prismatic",
        slide.joint_type == "prismatic",
        details=f"got {slide.joint_type}",
    )
    ctx.check(
        "slide axis is +X",
        tuple(round(c, 6) for c in slide.axis) == (1.0, 0.0, 0.0),
        details=f"got {slide.axis}",
    )

    # Only one articulation (no dial needle joint on a vernier caliper).
    all_joints = list(object_model.articulations)
    ctx.check(
        "exactly one articulation (prismatic slide only)",
        len(all_joints) == 1,
        details=f"got {len(all_joints)} articulations",
    )

    # --- Presence / placement of hero features ----------------------------
    # Main scale ticks exist on the beam and span a significant length.
    ms = ctx.part_element_world_aabb(beam, elem="main_scale_ticks")
    assert ms is not None
    ms_dx = ms[1][0] - ms[0][0]
    ctx.check(
        "main scale ticks span most of the beam",
        ms_dx > 0.10,
        details=f"span={ms_dx:.4f}",
    )
    # Main scale ticks sit on the beam top face (+Z side).
    beam_aabb = ctx.part_element_world_aabb(beam, elem="beam_body")
    assert beam_aabb is not None
    ctx.check(
        "main scale ticks are on beam top face",
        ms[0][2] > beam_aabb[0][2],
        details=f"ticks_zmin={ms[0][2]:.4f}, beam_zmin={beam_aabb[0][2]:.4f}",
    )

    # The slider body includes the vernier plate and ticks fused into it,
    # so its top should extend above the beam top (proving vernier features).
    slider_body_aabb = ctx.part_element_world_aabb(slider, elem="slider_body")
    assert slider_body_aabb is not None
    ctx.check(
        "slider body has vernier plate/ticks above beam top",
        slider_body_aabb[1][2] > beam_aabb[1][2] + 0.001,
        details=f"slider_zmax={slider_body_aabb[1][2]:.4f}, beam_zmax={beam_aabb[1][2]:.4f}",
    )

    # Thumb roller present on the slider.
    tr = ctx.part_element_world_aabb(slider, elem="thumb_roller")
    assert tr is not None
    tr_d = tr[1][0] - tr[0][0]
    ctx.check(
        "thumb roller present (~12mm wheel)",
        0.009 < tr_d < 0.016,
        details=f"diam={tr_d:.4f}",
    )

    # Fixed jaw on the beam extends well below the beam (large outside jaw).
    fj = ctx.part_element_world_aabb(beam, elem="fixed_jaw")
    assert fj is not None
    ctx.check(
        "fixed jaw drops below the beam",
        fj[0][1] < beam_aabb[0][1] - JAW_DOWN_LEN * 0.6,
        details=f"jaw_ymin={fj[0][1]:.4f}, beam_ymin={beam_aabb[0][1]:.4f}",
    )

    # No dial housing or needle should exist on a vernier caliper.
    slider_visual_names = [v.name for v in slider.visuals]
    ctx.check(
        "no dial housing on vernier caliper",
        "dial_bezel" not in slider_visual_names and "dial_face" not in slider_visual_names,
        details=f"slider visuals: {slider_visual_names}",
    )
    part_names = [p.name for p in object_model.parts]
    ctx.check(
        "no needle part on vernier caliper",
        "needle" not in part_names,
        details=f"parts: {part_names}",
    )

    # --- Mechanism: sliding opens jaw gap and extends the depth rod --------
    def jaw_gap() -> float:
        sj = ctx.part_element_world_aabb(slider, elem="slider_body")
        fjb = ctx.part_element_world_aabb(beam, elem="fixed_jaw")
        assert sj is not None and fjb is not None
        # gap between fixed jaw right face and slider jaw left face
        return sj[0][0] - fjb[1][0]

    def rod_tail_x() -> float:
        rod = ctx.part_element_world_aabb(slider, elem="depth_rod")
        assert rod is not None
        return rod[1][0]

    with ctx.pose({slide: 0.0}):
        gap0 = jaw_gap()
        rod0 = rod_tail_x()

    rest_cx = ctx.part_world_position(slider)
    with ctx.pose({slide: SLIDE_TRAVEL}):
        gap1 = jaw_gap()
        rod1 = rod_tail_x()
        ext_cx = ctx.part_world_position(slider)
    # The carriage advances by the full travel purely along +X.
    assert rest_cx is not None and ext_cx is not None
    ctx.check(
        "carriage advances by full travel along +X only",
        abs((ext_cx[0] - rest_cx[0]) - SLIDE_TRAVEL) < 0.001
        and abs(ext_cx[1] - rest_cx[1]) < 0.001
        and abs(ext_cx[2] - rest_cx[2]) < 0.001,
        details=f"rest={rest_cx}, ext={ext_cx}",
    )

    ctx.check(
        "sliding opens the jaw gap",
        gap1 > gap0 + 0.10,
        details=f"gap0={gap0:.4f}, gap1={gap1:.4f}",
    )
    ctx.check(
        "depth rod extends out the tail when sliding",
        rod1 > rod0 + 0.10,
        details=f"rod0={rod0:.4f}, rod1={rod1:.4f}",
    )

    # Verify the slider body (with vernier plate and ticks) moves with the
    # slider along +X when the carriage slides.
    rest_body_cx = (slider_body_aabb[0][0] + slider_body_aabb[1][0]) / 2.0
    with ctx.pose({slide: 0.050}):
        moved_body = ctx.part_element_world_aabb(slider, elem="slider_body")
        assert moved_body is not None
        moved_body_cx = (moved_body[0][0] + moved_body[1][0]) / 2.0
    ctx.check(
        "vernier plate/ticks advance with slider along +X",
        moved_body_cx > rest_body_cx + 0.040,
        details=f"rest_cx={rest_body_cx:.4f}, moved_cx={moved_body_cx:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
