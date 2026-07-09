from __future__ import annotations

# Articraft model: a vernier caliper (sliding measuring caliper with an
# engraved vernier scale plate on the carriage instead of a round dial gauge).
#
# Articraft brief:
# - Object: a ~150 mm (6") vernier caliper. Graduated steel beam ~0.20 m long.
#   Coordinate frame: +X runs along the beam (the measuring / slide axis),
#   +Y is vertical (jaws point up for inside-measuring tips and down for the
#   large outside-measuring jaws), +Z is the thin thickness direction with the
#   vernier scale face looking toward +Z.
# - Root/support: the graduated beam plus its fixed jaw head (one rigid part).
#   Everything mounts off the beam.
# - Parts: `beam` (root, with fixed jaws + beam scale strip),
#   `slider` (the moving jaw carriage carrying the vernier scale plate,
#   the thumb roller knurl, and the depth rod tail).
# - Articulation: `beam_to_slider`, PRISMATIC along +X. The carriage slides
#   along the beam; positive q opens the jaws and pushes the depth rod out the
#   tail end. The vernier scale plate on the carriage aligns with the beam
#   scale so the measurement is read where the vernier lines match the beam
#   lines.
# - Visible geometry: flat beam with an engraved scale strip, two fixed jaws
#   (large lower outside jaw + small upper inside jaw), mirrored sliding jaws,
#   an engraved vernier scale plate on the carriage front face, a knurled
#   thumb roller, and a slim depth rod.
# - Support/fit: the slider straddles the beam (its jaw root overlaps the beam
#   cross-section); this is a real captured prismatic fit, so it gets a scoped
#   allow_overlap. The depth rod rides inside the beam's tail channel.
# - Tests: beam/slider present, jaws present, vernier plate + ticks present
#   and adjacent to the beam scale, prismatic axis is +X, sliding opens the
#   jaw gap and extends the depth rod.

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

ROD_R = 0.0016  # depth rod radius
ROD_LEN = 0.150  # depth rod length (rides in tail channel, extends out tail)

# Vernier scale plate dimensions
VERNIER_W = 0.026  # plate width along X (26 mm)
VERNIER_H = 0.008  # plate height along Y (8 mm)
VERNIER_T = 0.0008  # plate thickness along Z (0.8 mm)
VERNIER_SPAN = 0.024  # tick span along X (24 mm for 25 divisions)
VERNIER_N = 25  # number of vernier divisions

# Colors / materials
STEEL = (0.74, 0.76, 0.79, 1.0)
STEEL_DARK = (0.55, 0.57, 0.60, 1.0)
CHROME = (0.82, 0.84, 0.87, 1.0)
BRASS = (0.78, 0.66, 0.36, 1.0)
NEEDLE_BLACK = (0.10, 0.10, 0.11, 1.0)
SCALE_DARK = (0.30, 0.31, 0.33, 1.0)


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


def _slider_body() -> cq.Workplane:
    """Sliding carriage authored in its own local frame, centered at X=0.

    It straddles the beam (a thin frame around the beam cross-section) and
    carries mirrored jaws plus a raised vernier plate mounting pad. The
    carriage's measuring jaws sit on the LEFT (-X) so that sliding the
    carriage in +X opens the gap against the fixed jaw.
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

    # Vernier plate mounting pad: a raised rectangular pad on the front (+Z)
    # face of the saddle, positioned at the same Y height as the beam's
    # graduated scale strip so the vernier markings align with the main scale.
    pad_w = 0.028
    pad_h = 0.010
    pad_t = 0.0015
    saddle_front_z = (BEAM_T + 0.005) / 2.0
    pad_y = BEAM_H / 2.0 - 0.005  # matches beam scale Y center
    vernier_pad = (
        cq.Workplane("XY")
        .box(pad_w, pad_h, pad_t, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.001)
        .translate((0.0, pad_y, saddle_front_z))
    )

    return saddle.union(lower).union(upper).union(vernier_pad)


def _vernier_plate() -> cq.Workplane:
    """Thin vernier scale plate with rounded corners.

    Authored in its own local frame centered at origin. This is the brass-
    colored plate that carries the engraved vernier graduations.
    """
    return (
        cq.Workplane("XY")
        .box(VERNIER_W, VERNIER_H, VERNIER_T, centered=(True, True, True))
        .edges("|Z")
        .fillet(0.001)
    )


def _vernier_ticks() -> cq.Workplane:
    """Engraved vernier graduation lines: 25 divisions over 24 mm.

    Authored in local frame centered at origin. Major ticks every 5 divisions
    are taller and wider; minor ticks fill the gaps.
    """
    ticks = None
    for i in range(VERNIER_N + 1):
        x = -VERNIER_SPAN / 2.0 + i * (VERNIER_SPAN / VERNIER_N)
        major = (i % 5) == 0
        h = 0.005 if major else 0.003
        w = 0.00040 if major else 0.00020
        tick = (
            cq.Workplane("XY")
            .box(w, h, 0.0005, centered=(True, True, True))
            .translate((x, 0.0, 0.0))
        )
        ticks = tick if ticks is None else ticks.union(tick)
    return ticks


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
    brass = model.material("brass", rgba=BRASS)
    scale_dark = model.material("scale_dark", rgba=SCALE_DARK)

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

    # Vernier scale plate: mounted on the front (+Z) face of the carriage,
    # aligned with the beam's graduated scale strip so the measurement is read
    # where the vernier lines coincide with the beam lines.
    # Position: the plate sits on top of the vernier pad (pad top at
    # saddle_front_z + pad_t). The plate base embeds slightly into the pad
    # for a seated connection.
    vernier_cx = 0.0
    vernier_cy = BEAM_H / 2.0 - 0.005  # same Y as beam scale center
    saddle_front_z = (BEAM_T + 0.005) / 2.0
    pad_t = 0.0015
    vernier_cz = saddle_front_z + pad_t  # plate center sits at pad top
    slider.visual(
        mesh_from_cadquery(_vernier_plate(), "vernier_plate"),
        origin=Origin(xyz=(vernier_cx, vernier_cy, vernier_cz)),
        material=brass,
        name="vernier_plate",
    )
    # Engraved vernier tick marks on the plate surface (slightly proud).
    slider.visual(
        mesh_from_cadquery(_vernier_ticks(), "vernier_ticks"),
        origin=Origin(xyz=(vernier_cx, vernier_cy, vernier_cz + VERNIER_T / 2.0)),
        material=scale_dark,
        name="vernier_ticks",
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

    # --- Presence / placement of hero features ----------------------------
    beam_aabb = ctx.part_element_world_aabb(beam, elem="beam_body")
    assert beam_aabb is not None

    # Vernier plate exists on the slider and has plausible dimensions.
    vp = ctx.part_element_world_aabb(slider, elem="vernier_plate")
    assert vp is not None
    vp_dx = vp[1][0] - vp[0][0]
    vp_dy = vp[1][1] - vp[0][1]
    ctx.check(
        "vernier plate is a flat rectangular plate (~26 x 8 mm)",
        0.020 < vp_dx < 0.032 and 0.006 < vp_dy < 0.012,
        details=f"dx={vp_dx:.4f}, dy={vp_dy:.4f}",
    )

    # Vernier plate is adjacent to the beam scale (same Y height band).
    bs = ctx.part_element_world_aabb(beam, elem="beam_scale")
    assert bs is not None
    beam_scale_cy = (bs[0][1] + bs[1][1]) / 2.0
    vernier_cy = (vp[0][1] + vp[1][1]) / 2.0
    ctx.check(
        "vernier plate aligns with beam scale in Y",
        abs(vernier_cy - beam_scale_cy) < 0.003,
        details=f"vernier_cy={vernier_cy:.4f}, beam_scale_cy={beam_scale_cy:.4f}",
    )

    # Vernier ticks present on the plate surface.
    vt = ctx.part_element_world_aabb(slider, elem="vernier_ticks")
    assert vt is not None
    vt_dx = vt[1][0] - vt[0][0]
    ctx.check(
        "vernier ticks span the vernier plate width (~24 mm)",
        0.018 < vt_dx < 0.030,
        details=f"span={vt_dx:.4f}",
    )

    # Vernier plate is on the front (+Z) face of the carriage, proud of beam.
    ctx.check(
        "vernier plate on the front face (Z above beam center)",
        vp[0][2] > -0.001,
        details=f"vp_zmin={vp[0][2]:.4f}",
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

    # No dial gauge or needle should exist on this variant.
    slider_visual_names = {v.name for v in slider.visuals}
    ctx.check(
        "no dial gauge on the vernier variant",
        "dial_bezel" not in slider_visual_names
        and "dial_face" not in slider_visual_names
        and "needle" not in slider_visual_names,
        details=f"slider visuals: {sorted(slider_visual_names)}",
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
        # Vernier plate travels with the slider along +X.
        vp_ext = ctx.part_element_world_aabb(slider, elem="vernier_plate")
        assert vp_ext is not None

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

    # Vernier plate travels with the carriage: its X position shifts by the
    # full travel, proving it is mounted on the slider (not the beam).
    vp_rest_cx = (vp[0][0] + vp[1][0]) / 2.0
    vp_ext_cx = (vp_ext[0][0] + vp_ext[1][0]) / 2.0
    ctx.check(
        "vernier plate moves with the slider along +X",
        abs((vp_ext_cx - vp_rest_cx) - SLIDE_TRAVEL) < 0.002,
        details=f"rest_cx={vp_rest_cx:.4f}, ext_cx={vp_ext_cx:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
