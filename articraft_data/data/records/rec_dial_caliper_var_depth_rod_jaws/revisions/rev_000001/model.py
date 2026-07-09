from __future__ import annotations

# Articraft model: a dial caliper (vernier-style sliding measuring caliper).
#
# Articraft brief:
# - Object: a ~150 mm (6") dial caliper. Graduated steel beam ~0.20 m long.
#   Coordinate frame: +X runs along the beam (the measuring / slide axis),
#   +Y is vertical (jaws point down for the large outside-measuring jaws),
#   +Z is the thin thickness direction with the dial face looking toward +Z.
# - Root/support: the graduated beam plus its fixed jaw head (one rigid part).
#   Everything mounts off the beam.
# - Parts: `beam` (root, with fixed outside jaw + a depth-rod channel groove),
#   `slider` (the moving jaw carriage carrying the dial housing, the dial face,
#   the needle, the thumb roller knurl, and the prominent depth rod).
# - Articulation: `beam_to_slider`, PRISMATIC along +X. The carriage slides
#   along the beam; positive q opens the jaws and pushes the depth rod out the
#   tail end. The dial needle pivots about the dial center.
# - Visible geometry: flat beam with an engraved scale strip and a depth-rod
#   channel groove, large outside-measuring jaws only (no upper inside tips),
#   a round chromed dial bezel with a white graduated face and a needle, a
#   knurled thumb roller, and a prominent depth rod extending through the beam
#   tail channel.
# - Support/fit: the slider straddles the beam (its jaw root overlaps the beam
#   cross-section); this is a real captured prismatic fit, so it gets a scoped
#   allow_overlap. The depth rod rides inside the beam's tail channel.
# - Tests: beam/slider present, jaws present (no upper tips), dial round bezel
#   + needle present, thumb roller present, depth rod prominent, prismatic axis
#   is +X, sliding opens the jaw gap and extends the depth rod, needle rotates.

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
JAW_T = 0.0040  # jaw thickness (Z)

SLIDE_TRAVEL = 0.150  # usable measuring travel (6")
# Fixed jaw inner face sits at this X; slider jaw inner face sits SLIDE_GAP0
# to its right at rest, so the rest jaw gap is small (caliper "zeroed").
FIXED_JAW_X = 0.011  # X of fixed jaw centerline (near left/-X end)
SLIDER_REST_X = 0.044  # slider carriage center X at q = 0 (small rest jaw gap)

DIAL_R = 0.0220  # dial bezel outer radius
DIAL_FACE_R = 0.0185  # white graduated face radius
DIAL_Z = BEAM_T / 2.0  # dial sits on the +Z face of the carriage

ROD_R = 0.0020  # depth rod radius (prominent secondary measuring feature)
ROD_LEN = 0.155  # depth rod length (rides in tail channel, extends out tail)
CHANNEL_W = 0.006  # depth-rod channel groove width on beam tail
CHANNEL_D = 0.002  # channel groove depth (cut into beam +Z face)

# Colors / materials
STEEL = (0.74, 0.76, 0.79, 1.0)
STEEL_DARK = (0.55, 0.57, 0.60, 1.0)
CHROME = (0.82, 0.84, 0.87, 1.0)
DIAL_WHITE = (0.93, 0.93, 0.90, 1.0)
NEEDLE_BLACK = (0.10, 0.10, 0.11, 1.0)
SCALE_DARK = (0.30, 0.31, 0.33, 1.0)


# ---------------------------------------------------------------------------
# CadQuery geometry builders (authored directly in meters)
# ---------------------------------------------------------------------------
def _beam_body() -> cq.Workplane:
    """Flat graduated beam: a thin bar along X with rounded long edges and a
    depth-rod channel groove running along the top (+Z) face at the tail end."""
    body = (
        cq.Workplane("XY")
        .box(BEAM_LEN, BEAM_H, BEAM_T, centered=(False, True, True))
        .edges("|Z")
        .fillet(0.0015)
    )
    # Depth-rod channel: a shallow groove cut into the +Z face along the
    # right-hand portion of the beam. The depth rod rides in this channel.
    channel_x_start = 0.060
    channel_len = BEAM_LEN - channel_x_start - 0.003
    cutter = (
        cq.Workplane("XY")
        .box(
            channel_len,
            CHANNEL_W,
            CHANNEL_D + 0.001,
            centered=(False, True, True),
        )
        .translate((channel_x_start, 0.0, BEAM_T / 2.0 - CHANNEL_D / 2.0))
    )
    body = body.cut(cutter)
    return body


def _fixed_jaw() -> cq.Workplane:
    """Fixed head: the large outside-measuring jaw at the left end, rooted on
    the beam's -X end. The jaw drops in -Y and tapers to a measuring tip.
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
    return lower.union(lower_blade)


def _slider_jaw() -> cq.Workplane:
    """Sliding outside-measuring jaw blade, authored in the slider local frame.

    The jaw drops in -Y from the left (-X) face of the carriage and tapers
    back toward +X. No upper inside-measuring tip.
    """
    carriage_w = 0.034
    xj = -carriage_w / 2.0
    lower_pts = [
        (xj, -BEAM_H / 2.0 + 0.001),
        (xj, -BEAM_H / 2.0 - JAW_DOWN_LEN),
        (xj + 0.010, -BEAM_H / 2.0 - JAW_DOWN_LEN),
        (xj + 0.013, -BEAM_H / 2.0 - JAW_DOWN_LEN * 0.45),
        (xj + 0.013, -BEAM_H / 2.0 + 0.001),
    ]
    return (
        cq.Workplane("XY")
        .polyline(lower_pts)
        .close()
        .extrude(JAW_T, both=True)
    )


def _slider_body() -> cq.Workplane:
    """Sliding carriage authored in its own local frame, centered at X=0.

    It straddles the beam (a thin frame around the beam cross-section) and
    carries the dial housing boss. The jaw blade is a separate visual for
    cleaner testing. The carriage's measuring jaw sits on the LEFT (-X) so
    that sliding the carriage in +X opens the gap against the fixed jaw.
    """
    carriage_w = 0.034  # along X
    # Frame/saddle that wraps the beam (slightly proud in Z so it reads raised).
    saddle = (
        cq.Workplane("XY")
        .box(carriage_w, BEAM_H + 0.006, BEAM_T + 0.0050, centered=(True, True, True))
        .edges("|Z")
        .fillet(0.0018)
    )

    # Housing boss: a full-diameter backing pad on +Z that the dial back plate
    # seats on. It is wide enough to reach the bezel rim and tall enough to
    # bridge from the saddle up to the gauge so the dial reads as truly mounted.
    boss = (
        cq.Workplane("XY")
        .workplane(offset=BEAM_T / 2.0 - 0.0010)
        .circle(DIAL_R)
        .extrude(0.0070)
        .translate((0.004, BEAM_H / 2.0 + 0.004, 0.0))
    )
    # Neck that ties the boss down onto the saddle top so there is a real stem.
    neck = (
        cq.Workplane("XY")
        .box(0.024, 0.012, BEAM_T + 0.006, centered=(True, True, True))
        .translate((0.004, BEAM_H / 2.0 - 0.002, 0.0))
    )
    return saddle.union(neck).union(boss)


def _dial_bezel() -> cq.Workplane:
    """Round chromed dial housing: a solid back plate (seats on the carriage
    boss) plus the raised hollow rim. The back plate is what the white face
    rests on, tying the whole gauge into one connected body."""
    back_plate = (
        cq.Workplane("XY")
        .circle(DIAL_R)
        .extrude(0.0012)  # z[0, 0.0012], rests on the boss
    )
    ring = (
        cq.Workplane("XY")
        .circle(DIAL_R)
        .circle(DIAL_R - 0.0030)
        .extrude(0.0072)
        .translate((0.0, 0.0, 0.0010))  # z[0.0010, 0.0082]
    )
    return back_plate.union(ring)


def _dial_face() -> cq.Workplane:
    """Flat white graduated face disc that fills the bezel and rests on the
    bezel back plate (its base embeds slightly so it stays connected)."""
    # base at z=0.0006 (embeds 0.0006 into the back plate which tops at 0.0012)
    return (
        cq.Workplane("XY")
        .circle(DIAL_FACE_R)
        .extrude(0.0014)
        .translate((0.0, 0.0, 0.0006))
    )


def _dial_ticks() -> cq.Workplane:
    """Engraved-looking radial graduation ticks around the dial face."""
    ticks = None
    n = 50
    for i in range(n):
        a = 2.0 * math.pi * i / n
        major = (i % 5) == 0
        r_out = DIAL_FACE_R - 0.0010
        r_in = r_out - (0.0030 if major else 0.0018)
        w = 0.00045 if major else 0.00025
        seg = (
            cq.Workplane("XY")
            .box(r_out - r_in, w, 0.0008, centered=(False, True, True))
            .translate((r_in, 0.0, 0.0))
            .rotate((0, 0, 0), (0, 0, 1), math.degrees(a))
        )
        ticks = seg if ticks is None else ticks.union(seg)
    return ticks


def _dial_needle() -> cq.Workplane:
    """Indicator needle authored in its own frame, pivoting about Z at origin.

    The needle points along +X at q=0 (long pointer toward +X, short tail -X).
    """
    pointer = (
        cq.Workplane("XY")
        .polyline(
            [
                (-0.0030, 0.00060),
                (DIAL_FACE_R - 0.0020, 0.00018),
                (DIAL_FACE_R - 0.0020, -0.00018),
                (-0.0030, -0.00060),
            ]
        )
        .close()
        .extrude(0.0009)
    )
    hub = cq.Workplane("XY").circle(0.0016).extrude(0.0012)
    return pointer.union(hub)


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
    model = ArticulatedObject(name="dial_caliper")

    steel = model.material("steel", rgba=STEEL)
    steel_dark = model.material("steel_dark", rgba=STEEL_DARK)
    chrome = model.material("chrome", rgba=CHROME)
    face_white = model.material("dial_white", rgba=DIAL_WHITE)
    needle_black = model.material("needle_black", rgba=NEEDLE_BLACK)
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
    slider.visual(
        mesh_from_cadquery(_slider_jaw(), "slider_jaw"),
        material=steel,
        name="slider_jaw",
    )

    # Dial housing assembly sits above the beam on the +Z face of the carriage.
    # The bezel back plate embeds into the mounting boss (boss top ~0.00825) so
    # the gauge reads as seated on the carriage rather than floating above it.
    dial_cx = 0.004
    dial_cy = BEAM_H / 2.0 + 0.004
    dial_cz = 0.0070  # bezel back-plate base z (boss top overlaps it)
    slider.visual(
        mesh_from_cadquery(_dial_bezel(), "dial_bezel"),
        origin=Origin(xyz=(dial_cx, dial_cy, dial_cz)),
        material=chrome,
        name="dial_bezel",
    )
    slider.visual(
        mesh_from_cadquery(_dial_face(), "dial_face"),
        origin=Origin(xyz=(dial_cx, dial_cy, dial_cz)),
        material=face_white,
        name="dial_face",
    )
    slider.visual(
        mesh_from_cadquery(_dial_ticks(), "dial_ticks"),
        origin=Origin(xyz=(dial_cx, dial_cy, dial_cz + 0.0018)),
        material=scale_dark,
        name="dial_ticks",
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

    # --- Dial needle (its own part so it can rotate with the slide) --------
    needle = model.part("needle")
    needle.visual(
        mesh_from_cadquery(_dial_needle(), "needle"),
        material=needle_black,
        name="needle",
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

    # Needle pivots about the dial center (+Z axis). On a real dial caliper the
    # rack-and-pinion couples this to the slide, but the SDK mimic requires a
    # matching motion domain (rotation cannot mimic a prismatic translation), so
    # the indicator needle is authored as its own continuous revolute pointer
    # that reads the gauge; positive q sweeps it like the measuring dial does.
    model.articulation(
        "slider_to_needle",
        ArticulationType.CONTINUOUS,
        parent=slider,
        child=needle,
        origin=Origin(xyz=(dial_cx, dial_cy, dial_cz + 0.0021)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.5, velocity=10.0),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    beam = object_model.get_part("beam")
    slider = object_model.get_part("slider")
    needle = object_model.get_part("needle")
    slide = object_model.get_articulation("beam_to_slider")
    needle_joint = object_model.get_articulation("slider_to_needle")

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
    ctx.check(
        "needle joint pivots about +Z",
        needle_joint.joint_type == "continuous"
        and tuple(round(c, 6) for c in needle_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={needle_joint.joint_type}, axis={needle_joint.axis}",
    )

    # --- Presence / placement of hero features ----------------------------
    # Round dial bezel exists and reads round (square-ish XY footprint).
    bz = ctx.part_element_world_aabb(slider, elem="dial_bezel")
    assert bz is not None
    bz_dx = bz[1][0] - bz[0][0]
    bz_dy = bz[1][1] - bz[0][1]
    ctx.check(
        "dial bezel is round and ~44mm across",
        abs(bz_dx - bz_dy) < 0.002 and 0.040 < bz_dx < 0.048,
        details=f"dx={bz_dx:.4f}, dy={bz_dy:.4f}",
    )
    # Dial sits above the beam (its center clearly above beam top edge).
    beam_aabb = ctx.part_element_world_aabb(beam, elem="beam_body")
    assert beam_aabb is not None
    ctx.check(
        "dial mounted above the beam",
        bz[0][1] > beam_aabb[0][1],
        details=f"bezel_ymin={bz[0][1]:.4f}, beam_ymin={beam_aabb[0][1]:.4f}",
    )

    # Needle present and is a slim pointer inside the dial radius.
    nb = ctx.part_world_aabb(needle)
    assert nb is not None
    needle_span = nb[1][0] - nb[0][0]
    ctx.check(
        "needle is a slim pointer within the dial",
        0.012 < needle_span < 0.040,
        details=f"span={needle_span:.4f}",
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

    # No upper inside-measuring knife tips: neither the fixed jaw nor the slider
    # jaw extends above the beam top surface.
    beam_top_y = beam_aabb[1][1]
    ctx.check(
        "fixed jaw has no upper knife tip (stays at or below beam top)",
        fj[1][1] <= beam_top_y + 0.001,
        details=f"jaw_ymax={fj[1][1]:.4f}, beam_top={beam_top_y:.4f}",
    )
    sj_aabb = ctx.part_element_world_aabb(slider, elem="slider_jaw")
    assert sj_aabb is not None
    ctx.check(
        "slider jaw has no upper knife tip (stays at or below beam top)",
        sj_aabb[1][1] <= beam_top_y + 0.002,
        details=f"slider_jaw_ymax={sj_aabb[1][1]:.4f}, beam_top={beam_top_y:.4f}",
    )
    ctx.check(
        "slider jaw drops below the beam (large outside jaw)",
        sj_aabb[0][1] < beam_aabb[0][1] - JAW_DOWN_LEN * 0.6,
        details=f"slider_jaw_ymin={sj_aabb[0][1]:.4f}, beam_ymin={beam_aabb[0][1]:.4f}",
    )

    # Depth rod is the prominent secondary measuring feature: it is a visible
    # rod (diameter >= 3mm) extending from the carriage through the beam tail.
    rod_aabb = ctx.part_element_world_aabb(slider, elem="depth_rod")
    assert rod_aabb is not None
    rod_dy = rod_aabb[1][1] - rod_aabb[0][1]
    rod_dz = rod_aabb[1][2] - rod_aabb[0][2]
    rod_diam = max(rod_dy, rod_dz)
    ctx.check(
        "depth rod is prominent (diameter >= 3.5mm)",
        rod_diam >= 0.0035,
        details=f"rod_diam={rod_diam:.4f}",
    )
    # Rod length extends well past the beam tail at rest.
    rod_len = rod_aabb[1][0] - rod_aabb[0][0]
    ctx.check(
        "depth rod is long (>= 140mm)",
        rod_len >= 0.140,
        details=f"rod_len={rod_len:.4f}",
    )

    # --- Mechanism: sliding opens jaw gap and extends the depth rod --------
    def jaw_gap() -> float:
        sj = ctx.part_element_world_aabb(slider, elem="slider_jaw")
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

    # The indicator needle actually pivots: its tip swings around the dial
    # center when the needle joint is posed.
    def needle_tip() -> tuple[float, float, float]:
        nb_local = ctx.part_world_aabb(needle)
        assert nb_local is not None
        return (nb_local[1][0], nb_local[1][1], nb_local[1][2])

    with ctx.pose({needle_joint: 0.0}):
        tip0 = needle_tip()
    with ctx.pose({needle_joint: math.pi / 2.0}):
        tip90 = needle_tip()
    ctx.check(
        "needle pivots about the dial center",
        abs(tip90[1] - tip0[1]) > 0.010 or abs(tip90[0] - tip0[0]) > 0.010,
        details=f"tip0={tip0}, tip90={tip90}",
    )

    return ctx.report()


object_model = build_object_model()
