from __future__ import annotations

# Inline cord power switch barrel fork.
#
# Targeted edit from the parent asset:
#   Keep the same prismatic slide actuator, but replace the pendant wall box /
#   top conduit arrangement with a small rounded inline cord-switch barrel and a
#   cord-entry boss at EACH end. The switch remains a workbench-scale equipment
#   power switch with one real user-facing sliding mechanism.
#
# Coordinate convention (meters):
#   X = body width across the cord, Y = cord / switch length, Z = height.
#   The actuator sits on the top face (+Z) and slides along +Y.
#
# Articulation:
#   * housing_to_slider -- PRISMATIC along +Y. The thumb slide rides in the
#                          raised top track and moves from OFF/low to ON/high.

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ----------------------------------------------------------------------------
# Real-world dimensions (meters): a compact inline lamp/cord-switch-sized body.
# ----------------------------------------------------------------------------
BARREL_W = 0.038          # width across the molded switch body
BARREL_L = 0.112          # length along the cord
BARREL_T = 0.017          # body height / thickness
BARREL_CORNER_R = 0.016   # large rounded ends/corners

CORD_R = 0.0042           # flexible cord radius
CORD_STUB = 0.040         # visible fixed cord segment beyond each boss
BOSS_R_X = 0.0120         # oval cord-entry boss half-width
BOSS_R_Z = 0.0068         # oval cord-entry boss half-height
BOSS_LEN = 0.014
BOSS_OVERLAP = 0.0020

# Top cover seam and fasteners.
SEAM_INSET_X = 0.0045
SEAM_INSET_Y = 0.0100
SCREW_Y = 0.037
SCREW_R = 0.0025

# Raised top slide track and the same prismatic slide actuator.
TRACK_W = 0.019
TRACK_L = 0.055
TRACK_PROUD = 0.0018
SLOT_W = 0.0080
SLOT_H = 0.0390          # slot length along Y
SLIDER_W = 0.0135        # thumb cap width
SLIDER_H = 0.0115        # thumb cap length along slide axis
SLIDER_CLEAR = 0.0015
SLIDER_TRAVEL = SLOT_H - SLIDER_H - 2.0 * SLIDER_CLEAR
SLIDER_REST_CY = -SLOT_H / 2.0 + SLIDER_CLEAR + SLIDER_H / 2.0


def _rounded_plate(width: float, length: float, height: float, radius: float):
    """Rounded-rectangle solid in the XY plane, extruded in +Z."""
    radius = min(radius, width / 2.0 - 0.001, length / 2.0 - 0.001)
    return (
        cq.Workplane("XY")
        .rect(width, length)
        .extrude(height)
        .edges("|Z")
        .fillet(radius)
    )


def _build_shell_mesh():
    """One-piece rounded molded inline barrel with shallow screw dimples."""
    shell = _rounded_plate(BARREL_W, BARREL_L, BARREL_T, BARREL_CORNER_R)
    # Soften the manufactured top and bottom edges.
    shell = shell.edges(">Z").fillet(0.0012)
    shell = shell.edges("<Z").fillet(0.0008)

    # Real slot/cavity under the thumb slide so the moving shoe is not buried in
    # a solid proxy housing.
    slide_cavity = (
        cq.Workplane("XY")
        .workplane(offset=BARREL_T + 0.0005)
        .rect(SLOT_W + 0.0018, SLOT_H + 0.0020)
        .extrude(-0.0045)
        .edges("|Z")
        .fillet(0.0012)
    )
    shell = shell.cut(slide_cavity)

    # Two shallow cover-screw dimples on the top shell.
    for sy in (-1, 1):
        dimple = (
            cq.Workplane("XY")
            .center(0.0, sy * SCREW_Y)
            .workplane(offset=BARREL_T + 0.0004)
            .circle(SCREW_R + 0.0007)
            .extrude(-0.0018)
        )
        shell = shell.cut(dimple)
    return shell


def _end_boss(sign: int):
    """Oval cord-entry bushing protruding from one end of the barrel."""
    start_y = sign * (BARREL_L / 2.0 - BOSS_OVERLAP)
    length = sign * (BOSS_LEN + BOSS_OVERLAP)
    boss = (
        cq.Workplane("XZ")
        .workplane(offset=start_y)
        .center(0.0, BARREL_T / 2.0)
        .ellipse(BOSS_R_X, BOSS_R_Z)
        .extrude(length)
    )
    hole = (
        cq.Workplane("XZ")
        .workplane(offset=start_y - sign * 0.0005)
        .center(0.0, BARREL_T / 2.0)
        # Slight compression fit around the cord keeps the visible cord supported
        # by the boss rather than floating in an oversized clearance hole.
        .circle(CORD_R - 0.0003)
        .extrude(length + sign * 0.0010)
    )
    return boss.cut(hole)


def _build_cord_bosses_mesh():
    return _end_boss(1).union(_end_boss(-1))


def _cord_stub(sign: int):
    """Short fixed cord segment exiting one boss."""
    start_y = sign * (BARREL_L / 2.0 + BOSS_LEN - 0.0030)
    return (
        cq.Workplane("XZ")
        .workplane(offset=start_y)
        .center(0.0, BARREL_T / 2.0)
        .circle(CORD_R)
        .extrude(sign * CORD_STUB)
    )


def _build_cord_stubs_mesh():
    return _cord_stub(1).union(_cord_stub(-1))


def _build_top_cover_mesh():
    """Slightly darker inset top cover panel showing the case seam line."""
    cover = (
        _rounded_plate(
            BARREL_W - 2.0 * SEAM_INSET_X,
            BARREL_L - 2.0 * SEAM_INSET_Y,
            0.0005,
            0.010,
        )
        .translate((0.0, 0.0, BARREL_T - 0.00015))
    )
    slot = (
        cq.Workplane("XY")
        .workplane(offset=BARREL_T - 0.0005)
        .rect(SLOT_W + 0.0015, SLOT_H + 0.0015)
        .extrude(0.0013)
        .edges("|Z")
        .fillet(0.0012)
    )
    return cover.cut(slot)


def _build_case_screws_mesh():
    """Two slotted screw heads that clamp the inline switch halves."""
    heads = None
    for sy in (-1, 1):
        head = (
            cq.Workplane("XY")
            .center(0.0, sy * SCREW_Y)
            .workplane(offset=BARREL_T + 0.00015)
            .circle(SCREW_R)
            .extrude(0.0009)
            .edges(">Z")
            .fillet(0.00035)
        )
        slit = (
            cq.Workplane("XY")
            .center(0.0, sy * SCREW_Y)
            .workplane(offset=BARREL_T + 0.00085)
            .box(SCREW_R * 1.55, 0.00055, 0.0008, centered=(True, True, False))
        )
        head = head.cut(slit)
        heads = head if heads is None else heads.union(head)
    return heads


def _build_slider_track_mesh():
    """Raised top track with a real long slot through which the slider shoe runs."""
    track = (
        _rounded_plate(TRACK_W, TRACK_L, TRACK_PROUD, 0.0045)
        .translate((0.0, 0.0, BARREL_T))
    )
    slot = (
        cq.Workplane("XY")
        .workplane(offset=BARREL_T - 0.0005)
        .rect(SLOT_W, SLOT_H)
        .extrude(TRACK_PROUD + 0.0012)
        .edges("|Z")
        .fillet(0.0016)
    )
    return track.cut(slot)


def _build_track_ticks_mesh():
    """Two small molded end ticks marking OFF/ON travel positions."""
    ticks = None
    for sy in (-1, 1):
        tick = (
            cq.Workplane("XY")
            .center(TRACK_W / 2.0 + 0.0030, sy * (SLOT_H / 2.0 - 0.0035))
            .workplane(offset=BARREL_T + 0.00025)
            .rect(0.0032, 0.0009)
            .extrude(0.00045)
        )
        ticks = tick if ticks is None else ticks.union(tick)
    return ticks


def _build_slider_mesh():
    """Thumb slide authored in its joint frame.

    The joint origin is on the actual top contact surface of the housing at
    BARREL_T. Geometry uses local Z: shoe descends into the slot, while the
    ribbed thumb cap sits proud above the track.
    """
    shoe = (
        cq.Workplane("XY")
        .workplane(offset=-0.0014)
        .rect(SLOT_W - 0.0010, SLIDER_H)
        .extrude(TRACK_PROUD + 0.0022)
        .edges("|Z")
        .fillet(0.0008)
    )
    cap = (
        cq.Workplane("XY")
        .workplane(offset=TRACK_PROUD - 0.00010)
        .rect(SLIDER_W, SLIDER_H + 0.0025)
        .extrude(0.0062)
        .edges("|Z")
        .fillet(0.0020)
        .edges(">Z")
        .fillet(0.0010)
    )
    # Three low grip ribs across the thumb cap.
    ribs = None
    for i in range(3):
        ry = (i - 1) * 0.0032
        rib = (
            cq.Workplane("XY")
            .center(0.0, ry)
            .workplane(offset=TRACK_PROUD + 0.0060)
            .rect(SLIDER_W * 0.78, 0.00065)
            .extrude(0.0007)
        )
        ribs = rib if ribs is None else ribs.union(rib)
    return shoe.union(cap).union(ribs)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="inline_cord_power_switch")

    abs_gray = Material(name="abs_gray", rgba=(0.55, 0.56, 0.58, 1.0))
    cover_gray = Material(name="cover_gray", rgba=(0.46, 0.47, 0.48, 1.0))
    dark_gray = Material(name="dark_gray", rgba=(0.20, 0.21, 0.22, 1.0))
    black_rubber = Material(name="black_rubber", rgba=(0.025, 0.026, 0.028, 1.0))
    screw_zinc = Material(name="screw_zinc", rgba=(0.76, 0.78, 0.80, 1.0))
    anodized = Material(name="anodized_dark", rgba=(0.19, 0.20, 0.22, 1.0))

    # ---- Inline rounded barrel housing (root) -------------------------------
    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_build_shell_mesh(), "housing_shell"),
        material=abs_gray,
        name="housing_shell",
    )
    housing.visual(
        mesh_from_cadquery(_build_top_cover_mesh(), "top_cover"),
        material=cover_gray,
        name="top_cover",
    )
    housing.visual(
        mesh_from_cadquery(_build_cord_bosses_mesh(), "cord_entry_bosses"),
        material=abs_gray,
        name="cord_entry_bosses",
    )
    housing.visual(
        mesh_from_cadquery(_build_cord_stubs_mesh(), "cord_stubs"),
        material=black_rubber,
        name="cord_stubs",
    )
    housing.visual(
        mesh_from_cadquery(_build_slider_track_mesh(), "slider_track"),
        material=dark_gray,
        name="slider_track",
    )
    housing.visual(
        mesh_from_cadquery(_build_track_ticks_mesh(), "track_ticks"),
        material=dark_gray,
        name="track_ticks",
    )
    housing.visual(
        mesh_from_cadquery(_build_case_screws_mesh(), "case_screws"),
        material=screw_zinc,
        name="case_screws",
    )

    # ---- Central thumb slider (prismatic along the inline body length +Y) ---
    slider = model.part("slider")
    slider.visual(
        mesh_from_cadquery(_build_slider_mesh(), "slide_thumb"),
        material=anodized,
        name="slide_thumb",
    )
    model.articulation(
        "housing_to_slider",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=slider,
        # Origin lies on the visible top face/slot contact surface, not on a
        # hidden anchor pad.
        origin=Origin(xyz=(0.0, SLIDER_REST_CY, BARREL_T)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0, velocity=0.1, lower=0.0, upper=SLIDER_TRAVEL
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    slider = object_model.get_part("slider")
    slide_joint = object_model.get_articulation("housing_to_slider")

    # The slide shoe is captured by the slotted top track. This local seated fit
    # can share tiny contact/mesh overlap at the slot lips, but it is not a broad
    # body collision.
    ctx.allow_overlap(
        housing,
        slider,
        elem_a="slider_track",
        elem_b="slide_thumb",
        reason="The thumb slide shoe is intentionally captured in the slotted top track.",
    )
    ctx.expect_overlap(
        housing,
        slider,
        axes="xy",
        elem_a="slider_track",
        elem_b="slide_thumb",
        min_overlap=0.006,
        name="thumb slide remains captured by the track footprint",
    )
    ctx.expect_within(
        slider,
        housing,
        axes="x",
        inner_elem="slide_thumb",
        outer_elem="slider_track",
        margin=0.0003,
        name="thumb slide is laterally retained by the track",
    )

    # --- Cord-switch barrel geometry is fixed housing geometry ---------------
    joint_names = {a.name for a in object_model.articulations}
    ctx.check(
        "only the thumb slide is jointed",
        joint_names == {"housing_to_slider"},
        details=f"joints={sorted(joint_names)}",
    )
    ctx.check(
        "cords and entry bosses belong to the fixed housing part",
        {"cord_entry_bosses", "cord_stubs"} <= {v.name for v in housing.visuals},
        details=str(sorted(v.name for v in housing.visuals)),
    )
    ctx.check(
        "housing has rounded barrel shell, top cover and slotted track",
        {"housing_shell", "top_cover", "slider_track", "case_screws"} <= {v.name for v in housing.visuals},
        details=str(sorted(v.name for v in housing.visuals)),
    )

    # --- Central slider joint contract (inline lengthwise slide) -------------
    ctx.check(
        "central switch is a prismatic slider (not a rotary)",
        str(slide_joint.articulation_type).lower().endswith("prismatic"),
        details=f"type={slide_joint.articulation_type}",
    )
    sax = tuple(float(a) for a in slide_joint.axis)
    ctx.check(
        "slider travels lengthwise along the inline body",
        abs(sax[1]) > 0.99 and abs(sax[0]) < 1e-6 and abs(sax[2]) < 1e-6,
        details=f"axis={sax}",
    )
    ctx.check(
        "slider has a usable up/down stroke",
        SLIDER_TRAVEL > 0.008,
        details=f"travel={SLIDER_TRAVEL:.4f}",
    )

    # Pushing the thumb cap forward actually moves it along the barrel.
    def _slider_y():
        box = ctx.part_world_aabb(slider)
        return None if box is None else (float(box[0][1]), float(box[1][1]))

    with ctx.pose({slide_joint: 0.0}):
        rear = _slider_y()
    with ctx.pose({slide_joint: SLIDER_TRAVEL}):
        forward = _slider_y()
    ctx.check("slider span present", rear is not None and forward is not None, details=str(rear))
    if rear is not None and forward is not None:
        advanced = (forward[0] - rear[0]) > 0.010 and (forward[1] - rear[1]) > 0.010
        ctx.check(
            "sliding the thumb cap moves it forward in the slot",
            advanced,
            details=f"rear_y={rear}, forward_y={forward}",
        )

    # The slider stays within the raised top track width (no sideways escape).
    track_aabb = ctx.part_element_world_aabb(housing, elem="slider_track")
    sl_aabb = ctx.part_world_aabb(slider)
    if track_aabb is not None and sl_aabb is not None:
        ctx.check(
            "slider cap fits inside the track width",
            sl_aabb[0][0] > track_aabb[0][0] - 1e-4 and sl_aabb[1][0] < track_aabb[1][0] + 1e-4,
            details=f"slider_x=({sl_aabb[0][0]:.4f},{sl_aabb[1][0]:.4f}), "
                    f"track_x=({track_aabb[0][0]:.4f},{track_aabb[1][0]:.4f})",
        )

    # --- Rounded inline barrel proportions ----------------------------------
    shell_aabb = ctx.part_element_world_aabb(housing, elem="housing_shell")
    ctx.check("shell aabb present", shell_aabb is not None, details=str(shell_aabb))
    if shell_aabb is not None:
        pmin, pmax = shell_aabb
        pw = float(pmax[0] - pmin[0])
        pl = float(pmax[1] - pmin[1])
        pt = float(pmax[2] - pmin[2])
        ctx.check("barrel width matches compact cord switch scale", abs(pw - BARREL_W) < 0.004, details=f"w={pw}")
        ctx.check("barrel length matches inline switch scale", abs(pl - BARREL_L) < 0.004, details=f"l={pl}")
        ctx.check("barrel is a small rounded inline body", 0.014 < pt < 0.021, details=f"t={pt}")

    # Cord-entry bosses sit at both lengthwise ends, and the cord is longer than
    # the molded barrel so the asset reads as an inline cord switch.
    boss_aabb = ctx.part_element_world_aabb(housing, elem="cord_entry_bosses")
    cord_aabb = ctx.part_element_world_aabb(housing, elem="cord_stubs")
    if boss_aabb is not None and shell_aabb is not None:
        ctx.check(
            "cord-entry bosses protrude from both ends of the barrel",
            float(boss_aabb[0][1]) < -BARREL_L / 2.0 - 0.010
            and float(boss_aabb[1][1]) > BARREL_L / 2.0 + 0.010,
            details=f"boss_y=({float(boss_aabb[0][1]):.4f},{float(boss_aabb[1][1]):.4f})",
        )
    if cord_aabb is not None:
        ctx.check(
            "fixed cord stubs extend beyond both entry bosses",
            float(cord_aabb[0][1]) < -BARREL_L / 2.0 - BOSS_LEN - 0.020
            and float(cord_aabb[1][1]) > BARREL_L / 2.0 + BOSS_LEN + 0.020,
            details=f"cord_y=({float(cord_aabb[0][1]):.4f},{float(cord_aabb[1][1]):.4f})",
        )

    return ctx.report()


object_model = build_object_model()
