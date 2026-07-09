from __future__ import annotations

# Articraft model: a digital caliper (vernier-style sliding measuring caliper
# with a flat rectangular LCD readout instead of a round dial gauge).
#
# Articraft brief:
# - Object: a ~150 mm (6") digital caliper. Graduated steel beam ~0.20 m long.
#   Coordinate frame: +X runs along the beam (the measuring / slide axis),
#   +Y is vertical (jaws point up for inside-measuring tips and down for the
#   large outside-measuring jaws), +Z is the thin thickness direction with the
#   LCD screen facing toward +Z.
# - Root/support: the graduated beam plus its fixed jaw head (one rigid part).
#   Everything mounts off the beam.
# - Parts: `beam` (root, with fixed jaws + a long depth-rod channel),
#   `slider` (the moving jaw carriage carrying the LCD module housing, the
#   screen, segment readout bars, button pads, the thumb roller knurl, and the
#   depth rod tail).
# - Articulation: `beam_to_slider`, PRISMATIC along +X. The carriage slides
#   along the beam; positive q opens the jaws and pushes the depth rod out the
#   tail end. The LCD readout is fixed to the slider (no needle joint).
# - Visible geometry: flat beam with an engraved scale strip, two fixed jaws
#   (large lower outside jaw + small upper inside jaw), mirrored sliding jaws,
#   a flat rectangular LCD module with recessed dark screen and light segment
#   bars, small button pads, a knurled thumb roller, and a slim depth rod.
# - Support/fit: the slider straddles the beam (its jaw root overlaps the beam
#   cross-section); this is a real captured prismatic fit, so it gets a scoped
#   allow_overlap. The depth rod rides inside the beam's tail channel.
# - Tests: beam/slider present, jaws present, LCD screen present and
#   rectangular (not round), buttons present, thumb roller present, prismatic
#   axis is +X, sliding opens the jaw gap and extends the depth rod.

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

# LCD module dimensions (flat rectangular digital readout)
LCD_W = 0.042  # module width along X
LCD_H = 0.026  # module height along Y
LCD_D = 0.008  # module depth (thickness along Z)
SCREEN_W = 0.034  # screen window width
SCREEN_H = 0.014  # screen window height
LCD_Z_BASE = BEAM_T / 2.0 - 0.001  # module base sits on/just below beam top

ROD_R = 0.0016  # depth rod radius
ROD_LEN = 0.150  # depth rod length (rides in tail channel, extends out tail)

# Colors / materials
STEEL = (0.74, 0.76, 0.79, 1.0)
STEEL_DARK = (0.55, 0.57, 0.60, 1.0)
CHROME = (0.82, 0.84, 0.87, 1.0)
LCD_BG = (0.18, 0.22, 0.20, 1.0)  # dark greenish-grey LCD background
LCD_SEG = (0.72, 0.78, 0.65, 1.0)  # light segment bars (backlit LCD look)
BUTTON_DARK = (0.15, 0.15, 0.17, 1.0)
SCALE_DARK = (0.30, 0.31, 0.33, 1.0)
HOUSING_DARK = (0.22, 0.23, 0.25, 1.0)


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
    carries mirrored jaws plus a flat rectangular platform for the LCD module.
    The carriage's measuring jaws sit on the LEFT (-X) so that sliding the
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

    # LCD platform: a flat rectangular boss on +Z that the LCD module seats on.
    # Replaces the old round dial boss. Wide enough to support the full module.
    platform = (
        cq.Workplane("XY")
        .workplane(offset=BEAM_T / 2.0 - 0.0010)
        .rect(LCD_W * 0.85, LCD_H * 0.80)
        .extrude(0.0050)  # top at z ~ 0.00625
        .translate((0.004, BEAM_H / 2.0 + LCD_H / 2.0 - 0.002, 0.0))
    )
    # Neck that ties the platform down onto the saddle top so there is a real
    # structural connection.
    neck = (
        cq.Workplane("XY")
        .box(0.026, 0.014, BEAM_T + 0.006, centered=(True, True, True))
        .translate((0.004, BEAM_H / 2.0 - 0.002, 0.0))
    )
    return saddle.union(lower).union(upper).union(neck).union(platform)


def _lcd_housing() -> cq.Workplane:
    """Flat rectangular LCD module housing: a slim box with a recessed screen
    window on the +Z face. The housing sits on the slider's platform."""
    outer = (
        cq.Workplane("XY")
        .box(LCD_W, LCD_H, LCD_D, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.002)
    )
    # Recessed screen cavity on the top (+Z) face
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=LCD_D - 0.0015)
        .rect(SCREEN_W, SCREEN_H)
        .extrude(0.002)  # cut downward through the top
    )
    return outer.cut(cavity)


def _lcd_screen() -> cq.Workplane:
    """The recessed dark LCD screen panel that fills the housing window."""
    return (
        cq.Workplane("XY")
        .rect(SCREEN_W - 0.001, SCREEN_H - 0.001)
        .extrude(0.0012)
    )


def _lcd_segments() -> cq.Workplane:
    """Light-colored segment bars representing a digital readout on the LCD.

    A thin backing plate connects all segments into one body so the mesh is
    a single connected component.
    """
    # Thin backing plate that spans all digits and ties them together.
    backing = (
        cq.Workplane("XY")
        .box(SCREEN_W - 0.004, SCREEN_H - 0.004, 0.0003, centered=(True, True, True))
        .translate((0.0, 0.0, -0.0003))
    )
    bars = backing
    # Create 4 digit-like groups of horizontal segment bars
    digit_positions_x = [-0.012, -0.004, 0.004, 0.012]
    for dx in digit_positions_x:
        # Each "digit" is 3 horizontal bars stacked vertically (like a 7-seg)
        for i in range(3):
            dy = -0.004 + i * 0.004
            seg = (
                cq.Workplane("XY")
                .box(0.005, 0.0008, 0.0006, centered=(True, True, True))
                .translate((dx, dy, 0.0))
            )
            bars = bars.union(seg)
        # Vertical bars on sides of each digit
        for side in (-1, 1):
            vseg = (
                cq.Workplane("XY")
                .box(0.0006, 0.008, 0.0006, centered=(True, True, True))
                .translate((dx + side * 0.003, 0.0, 0.0))
            )
            bars = bars.union(vseg)
    # Decimal point
    dot = (
        cq.Workplane("XY")
        .box(0.0012, 0.0012, 0.0006, centered=(True, True, True))
        .translate((0.000, -0.005, 0.0))
    )
    bars = bars.union(dot)
    return bars


def _button_pad() -> cq.Workplane:
    """A small round button pad for the digital caliper controls."""
    base = cq.Workplane("XY").circle(0.0025).extrude(0.0018)
    # Small domed top for tactile look
    cap = (
        cq.Workplane("XY")
        .workplane(offset=0.0018)
        .circle(0.0022)
        .extrude(0.0006)
    )
    return base.union(cap)


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
    model = ArticulatedObject(name="digital_caliper")

    steel = model.material("steel", rgba=STEEL)
    steel_dark = model.material("steel_dark", rgba=STEEL_DARK)
    chrome = model.material("chrome", rgba=CHROME)
    lcd_bg = model.material("lcd_bg", rgba=LCD_BG)
    lcd_seg = model.material("lcd_seg", rgba=LCD_SEG)
    button_dark = model.material("button_dark", rgba=BUTTON_DARK)
    scale_dark = model.material("scale_dark", rgba=SCALE_DARK)
    housing_dark = model.material("housing_dark", rgba=HOUSING_DARK)

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

    # LCD module assembly sits above the beam on the +Z face of the carriage.
    # The housing base embeds slightly into the mounting platform so the module
    # reads as seated on the carriage rather than floating above it.
    lcd_cx = 0.004
    lcd_cy = BEAM_H / 2.0 + LCD_H / 2.0 - 0.002
    lcd_cz = 0.0050  # housing base z (platform top overlaps slightly)
    slider.visual(
        mesh_from_cadquery(_lcd_housing(), "lcd_housing"),
        origin=Origin(xyz=(lcd_cx, lcd_cy, lcd_cz)),
        material=housing_dark,
        name="lcd_housing",
    )
    # Screen sits inside the housing recess, slightly proud of cavity bottom.
    slider.visual(
        mesh_from_cadquery(_lcd_screen(), "lcd_screen"),
        origin=Origin(xyz=(lcd_cx, lcd_cy, lcd_cz + LCD_D - 0.0018)),
        material=lcd_bg,
        name="lcd_screen",
    )
    # Segment bars embedded slightly into the screen surface so they read as
    # printed/markings on the display and form a connected geometry group.
    slider.visual(
        mesh_from_cadquery(_lcd_segments(), "lcd_segments"),
        origin=Origin(xyz=(lcd_cx, lcd_cy, lcd_cz + LCD_D - 0.0012)),
        material=lcd_seg,
        name="lcd_segments",
    )

    # Button pads: two small round buttons below the screen on the housing face.
    for i in range(2):
        bx = lcd_cx + (-0.010 + i * 0.020)
        by = lcd_cy - LCD_H / 2.0 + 0.004
        bz = lcd_cz + LCD_D - 0.0002
        slider.visual(
            mesh_from_cadquery(_button_pad(), f"button_{i}"),
            origin=Origin(xyz=(bx, by, bz)),
            material=button_dark,
            name=f"button_{i}",
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

    # --- Only one articulation (no needle joint on a digital caliper) -----
    all_joints = list(object_model.articulations)
    ctx.check(
        "exactly one articulation (prismatic slide only)",
        len(all_joints) == 1,
        details=f"got {len(all_joints)} articulations: {[j.name for j in all_joints]}",
    )

    # --- Presence / placement of hero features ----------------------------
    # LCD housing exists and reads rectangular (not round).
    lcd_aabb = ctx.part_element_world_aabb(slider, elem="lcd_housing")
    assert lcd_aabb is not None
    lcd_dx = lcd_aabb[1][0] - lcd_aabb[0][0]
    lcd_dy = lcd_aabb[1][1] - lcd_aabb[0][1]
    lcd_dz = lcd_aabb[1][2] - lcd_aabb[0][2]
    ctx.check(
        "LCD housing is rectangular and flat (not round)",
        abs(lcd_dx - LCD_W) < 0.004 and abs(lcd_dy - LCD_H) < 0.004,
        details=f"dx={lcd_dx:.4f}, dy={lcd_dy:.4f}, dz={lcd_dz:.4f}",
    )
    ctx.check(
        "LCD housing is wider than tall (landscape orientation)",
        lcd_dx > lcd_dy,
        details=f"dx={lcd_dx:.4f}, dy={lcd_dy:.4f}",
    )
    ctx.check(
        "LCD housing is slim (thin panel, not a deep box)",
        lcd_dz < 0.015,
        details=f"dz={lcd_dz:.4f}",
    )

    # LCD screen present and recessed within housing.
    screen_aabb = ctx.part_element_world_aabb(slider, elem="lcd_screen")
    assert screen_aabb is not None
    scr_dx = screen_aabb[1][0] - screen_aabb[0][0]
    scr_dy = screen_aabb[1][1] - screen_aabb[0][1]
    ctx.check(
        "LCD screen is present and smaller than the housing",
        scr_dx < lcd_dx - 0.002 and scr_dy < lcd_dy - 0.002,
        details=f"screen dx={scr_dx:.4f}, dy={scr_dy:.4f}",
    )

    # Segment bars present on the screen.
    seg_aabb = ctx.part_element_world_aabb(slider, elem="lcd_segments")
    assert seg_aabb is not None
    ctx.check(
        "LCD segment bars are within the screen area",
        seg_aabb[0][0] >= screen_aabb[0][0] - 0.001
        and seg_aabb[1][0] <= screen_aabb[1][0] + 0.001
        and seg_aabb[0][1] >= screen_aabb[0][1] - 0.001
        and seg_aabb[1][1] <= screen_aabb[1][1] + 0.001,
        details=f"seg=[{seg_aabb[0]}, {seg_aabb[1]}], scr=[{screen_aabb[0]}, {screen_aabb[1]}]",
    )

    # Button pads present on the slider.
    btn0_aabb = ctx.part_element_world_aabb(slider, elem="button_0")
    btn1_aabb = ctx.part_element_world_aabb(slider, elem="button_1")
    assert btn0_aabb is not None and btn1_aabb is not None
    ctx.check(
        "two button pads present on the slider",
        (btn0_aabb[1][0] - btn0_aabb[0][0]) > 0.001
        and (btn1_aabb[1][0] - btn1_aabb[0][0]) > 0.001,
        details=f"btn0_dx={btn0_aabb[1][0] - btn0_aabb[0][0]:.4f}, btn1_dx={btn1_aabb[1][0] - btn1_aabb[0][0]:.4f}",
    )

    # LCD sits above the beam (its center clearly above beam top edge).
    beam_aabb = ctx.part_element_world_aabb(beam, elem="beam_body")
    assert beam_aabb is not None
    ctx.check(
        "LCD module mounted above the beam",
        lcd_aabb[0][1] > beam_aabb[0][1],
        details=f"lcd_ymin={lcd_aabb[0][1]:.4f}, beam_ymin={beam_aabb[0][1]:.4f}",
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

    # Confirm the LCD module moves with the slider (it is fixed to the slider,
    # not independently articulated).
    with ctx.pose({slide: 0.0}):
        lcd_pos_rest = ctx.part_element_world_aabb(slider, elem="lcd_housing")
    with ctx.pose({slide: SLIDE_TRAVEL}):
        lcd_pos_ext = ctx.part_element_world_aabb(slider, elem="lcd_housing")
    assert lcd_pos_rest is not None and lcd_pos_ext is not None
    lcd_travel = lcd_pos_ext[0][0] - lcd_pos_rest[0][0]
    ctx.check(
        "LCD module translates with the slider along +X",
        abs(lcd_travel - SLIDE_TRAVEL) < 0.002,
        details=f"lcd_travel={lcd_travel:.4f}, expected={SLIDE_TRAVEL:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
