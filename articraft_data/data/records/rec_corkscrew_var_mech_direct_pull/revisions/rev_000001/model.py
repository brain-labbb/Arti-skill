from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Direct T-handle pull corkscrew, modeled upright at true scale.
#   - matte black body: twin flat plates + top bridge + four legs + hollow
#     bell skirt (open underneath, seats on bottle neck)
#   - chrome rack spindle (annular rack rings + guide collar) sliding through
#     the body slot: PRISMATIC travel ~0.04 m
#   - chrome T-bar handle + helical worm spinning CONTINUOUSLY on the shaft
#   - no wings: user drives the worm by spinning the T-bar, then pulls
#     straight up to extract the cork
# ---------------------------------------------------------------------------

# Bell skirt (hollow, open underneath, tapering outward toward the table).
BELL_H = 0.028
BELL_R_BOT_OUT = 0.0240
BELL_R_BOT_IN = 0.0205
BELL_R_TOP_OUT = 0.0195
BELL_R_TOP_IN = 0.0160

# Twin vertical body plates (the flat black frame) and their support legs.
PLATE_W = 0.050
PLATE_T = 0.003
PLATE_Y_IN = 0.007  # inner face of each plate -> central mechanism slot
PLATE_Z0 = 0.063
PLATE_Z1 = 0.123
LEG_W = 0.009
LEG_X = 0.016

# Top bridge connecting the two plates (gives the user something to pull
# against and closes the top of the frame slot).
BRIDGE_T = 0.004

# Shaft stack.
SHAFT_R = 0.0045
SLEEVE_BORE_R = 0.0042  # light running fit: shaft core seats in this bore
SLEEVE_R = 0.0062
RING_R = 0.0068
TRAVEL = 0.038
JOINT_TOP_Z = 0.123  # prismatic joint frame: top plane of the body plates
# Guide collar on the rack spindle: rides in the slot between the two body
# plates with a light press fit so the sliding stage is genuinely supported.
GUIDE_X = 0.012
GUIDE_HALF_Y = 0.0073  # 0.3 mm interference against each plate inner face
GUIDE_Z0 = -0.024
GUIDE_Z1 = 0.004

# T-handle / worm locals (expressed in the spinning shaft's frame, whose
# origin coincides with the prismatic frame at rest: world z = JOINT_TOP_Z).
WORM_COIL_R = 0.0042
WORM_WIRE_R = 0.0017
WORM_Z0 = -0.113  # world 0.012 at rest
WORM_HEIGHT = 0.046
CORE_Z0 = -0.071
CORE_Z1 = 0.047
TBAR_Z = 0.046  # high enough that the descended bar clears the plate tops
TBAR_LEN = 0.060
TBAR_R = 0.0065


def _build_body_frame() -> cq.Workplane:
    """Black frame: bell skirt + four legs + twin plates + top bridge."""
    # Hollow bell skirt band, open at top and bottom (revolved cone shell).
    bell = (
        cq.Workplane("XZ")
        .polyline(
            [
                (BELL_R_BOT_IN, 0.0),
                (BELL_R_BOT_OUT, 0.0),
                (BELL_R_TOP_OUT, BELL_H),
                (BELL_R_TOP_IN, BELL_H),
            ]
        )
        .close()
        .revolve(360.0, (0, 0), (0, 1))
    )

    body = bell
    plate_h = PLATE_Z1 - PLATE_Z0
    for sy in (-1.0, 1.0):
        y_c = sy * (PLATE_Y_IN + 0.5 * PLATE_T)
        plate = (
            cq.Workplane("XY")
            .box(PLATE_W, PLATE_T, plate_h)
            .edges("|Y and >Z")
            .fillet(0.012)
            .translate((0.0, y_c, PLATE_Z0 + 0.5 * plate_h))
        )
        body = body.union(plate)
        for sx in (-1.0, 1.0):
            leg = (
                cq.Workplane("XY")
                .box(LEG_W, PLATE_T, 0.044)
                .translate((sx * LEG_X, y_c, 0.045))
            )
            body = body.union(leg)

    # Top bridge: connects the two plates across the Y gap, providing a
    # structural cap that the user pulls against during cork extraction.
    bridge_gap = 2.0 * PLATE_Y_IN
    bridge = (
        cq.Workplane("XY")
        .box(PLATE_W - 0.008, bridge_gap + 2.0 * PLATE_T, BRIDGE_T)
        .edges("|Z")
        .fillet(0.004)
        .translate((0.0, 0.0, PLATE_Z1 + 0.5 * BRIDGE_T))
    )
    body = body.union(bridge)

    # Central bore through the bridge for the shaft to pass.
    shaft_bore = (
        cq.Workplane("XY")
        .circle(SHAFT_R + 0.001)
        .extrude(BRIDGE_T + 0.004)
        .translate((0.0, 0.0, PLATE_Z1 - 0.002))
    )
    body = body.cut(shaft_bore)

    return body


def _build_rack_sleeve() -> cq.Workplane:
    """Chrome rack spindle: knurled ring stack + slot guide collar."""
    sleeve = cq.Workplane("XY").cylinder(0.025, SLEEVE_R).translate((0.0, 0.0, 0.0145))
    for i, z_ring in enumerate((0.0065, 0.0125, 0.0185)):
        ring = cq.Workplane("XY").cylinder(0.0035, RING_R).translate((0.0, 0.0, z_ring))
        sleeve = sleeve.union(ring)
    # Guide collar sliding in the slot between the body plates (light press
    # fit against the plate inner faces; carries the moving rack stage).
    guide = (
        cq.Workplane("XY")
        .box(GUIDE_X, 2.0 * GUIDE_HALF_Y, GUIDE_Z1 - GUIDE_Z0)
        .translate((0.0, 0.0, 0.5 * (GUIDE_Z0 + GUIDE_Z1)))
    )
    sleeve = sleeve.union(guide)
    bore = cq.Workplane("XY").cylinder(0.064, SLEEVE_BORE_R).translate((0.0, 0.0, 0.0045))
    return sleeve.cut(bore)


def _build_worm() -> cq.Workplane:
    """Helical worm screw: a round wire swept along a true helix."""
    helix = cq.Wire.makeHelix(pitch=0.0115, height=WORM_HEIGHT, radius=WORM_COIL_R)
    path = cq.Workplane("XY").newObject([helix])
    worm = (
        cq.Workplane("XZ")
        .center(WORM_COIL_R, 0.0)
        .circle(WORM_WIRE_R)
        .sweep(path, isFrenet=True)
    )
    return worm.translate((0.0, 0.0, WORM_Z0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="direct_pull_corkscrew")

    matte_black = model.material("matte_black", rgba=(0.07, 0.07, 0.08, 1.0))
    chrome = model.material("polished_chrome", rgba=(0.76, 0.78, 0.81, 1.0))
    bright_steel = model.material("bright_steel", rgba=(0.86, 0.87, 0.89, 1.0))

    # ----------------------------------------------------------------- body
    body = model.part("body_frame")
    body.visual(
        mesh_from_cadquery(_build_body_frame(), "body_frame"),
        material=matte_black,
        name="frame_shell",
    )

    # ----------------------------------------------------- rack spindle stage
    sleeve = model.part("rack_spindle")
    sleeve.visual(
        mesh_from_cadquery(_build_rack_sleeve(), "rack_spindle"),
        material=chrome,
        name="rack_sleeve",
    )

    # ------------------------------------------------- spinning shaft stage
    handle = model.part("t_handle_worm")
    handle.visual(
        mesh_from_cadquery(_build_worm(), "worm_screw"),
        material=chrome,
        name="worm_helix",
    )
    handle.visual(
        Cylinder(radius=SHAFT_R, length=CORE_Z1 - CORE_Z0),
        origin=Origin(xyz=(0.0, 0.0, 0.5 * (CORE_Z0 + CORE_Z1))),
        material=chrome,
        name="shaft_core",
    )
    handle.visual(
        Cylinder(radius=TBAR_R, length=TBAR_LEN),
        origin=Origin(xyz=(0.0, 0.0, TBAR_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bright_steel,
        name="t_bar",
    )
    for i, sx in enumerate((-1.0, 1.0)):
        handle.visual(
            Sphere(radius=TBAR_R),
            origin=Origin(xyz=(sx * 0.5 * TBAR_LEN, 0.0, TBAR_Z)),
            material=bright_steel,
            name=f"t_bar_tip_{i}",
        )

    # ----------------------------------------------------------- articulation
    # Rack spindle descends through the slot between the body plates.
    model.articulation(
        "spindle_travel",
        ArticulationType.PRISMATIC,
        parent=body,
        child=sleeve,
        origin=Origin(xyz=(0.0, 0.0, JOINT_TOP_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.10, lower=0.0, upper=TRAVEL),
    )
    # T-handle + worm spin continuously about the vertical shaft axis.
    model.articulation(
        "worm_spin",
        ArticulationType.CONTINUOUS,
        parent=sleeve,
        child=handle,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body_frame")
    sleeve = object_model.get_part("rack_spindle")
    handle = object_model.get_part("t_handle_worm")
    j_travel = object_model.get_articulation("spindle_travel")
    j_spin = object_model.get_articulation("worm_spin")

    # --- mechanism type: direct T-handle pull --------------------------------
    # No wings exist; the defining motion is the continuous worm spin.
    all_parts = [p.name for p in object_model.parts]
    ctx.check(
        "no wing parts present (direct pull mechanism)",
        not any("wing" in n for n in all_parts),
        details=f"parts={all_parts}",
    )
    all_joints = [a.name for a in object_model.articulations]
    ctx.check(
        "no wing pivot articulations",
        not any("wing" in n for n in all_joints),
        details=f"joints={all_joints}",
    )

    # --- joint plan conformance ------------------------------------------
    ctx.check(
        "worm spin is continuous about the vertical axis",
        j_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(j_spin.axis) == (0.0, 0.0, 1.0)
        and j_spin.motion_limits is not None
        and j_spin.motion_limits.lower is None
        and j_spin.motion_limits.upper is None,
    )
    ctx.check(
        "shaft travel is a 0.04 m descending prismatic",
        j_travel.articulation_type == ArticulationType.PRISMATIC
        and abs(j_travel.motion_limits.upper - TRAVEL) < 1e-9
        and tuple(j_travel.axis) == (0.0, 0.0, -1.0),
    )
    ctx.check(
        "at least one non-fixed joint exists",
        any(
            a.articulation_type != ArticulationType.FIXED
            for a in object_model.articulations
        ),
    )

    # --- scale and grounding ---------------------------------------------
    bb = ctx.part_world_aabb(body)
    ctx.check(
        "bell skirt grounded at z=0",
        bb is not None and abs(bb[0][2]) <= 1.5e-3,
        details=f"body aabb={bb}",
    )
    ctx.check(
        "body ~0.05 m wide",
        bb is not None and 0.046 <= (bb[1][0] - bb[0][0]) <= 0.054,
    )
    hb = ctx.part_world_aabb(handle)
    ctx.check(
        "overall height ~0.17 m at the T-bar top",
        hb is not None and 0.160 <= hb[1][2] <= 0.178,
        details=f"handle aabb={hb}",
    )
    ctx.check(
        "worm hangs above the table at rest",
        hb is not None and hb[0][2] > 0.004,
    )

    # --- worm coil legibility ---------------------------------------------
    wa = ctx.part_element_world_aabb(handle, elem="worm_helix")
    ctx.check(
        "helical worm coil fills the open frame window above the bell",
        wa is not None
        and wa[0][2] > 0.005
        and wa[1][2] < PLATE_Z0
        and (wa[1][0] - wa[0][0]) > 2.2 * SHAFT_R,
        details=f"worm aabb={wa}",
    )

    # --- continuous spin: off-axis T-bar proves real rotation --------------
    ctx.check(
        "T-bar spans X at rest",
        hb is not None and (hb[1][0] - hb[0][0]) >= 0.065,
    )
    with ctx.pose({j_spin: math.pi / 2.0}):
        hq = ctx.part_world_aabb(handle)
        ctx.check(
            "quarter turn swings the T-bar from X to Y",
            hq is not None
            and (hq[1][0] - hq[0][0]) <= 0.035
            and (hq[1][1] - hq[0][1]) >= 0.065,
            details=f"posed handle aabb={hq}",
        )

    # --- prismatic descent (pull stroke) ----------------------------------
    with ctx.pose({j_travel: TRAVEL}):
        hd = ctx.part_world_aabb(handle)
        ctx.check(
            "full travel drives the worm down through the bell mouth",
            hd is not None and hd[0][2] < hb[0][2] - 0.035,
            details=f"descended handle aabb={hd}",
        )
        ctx.expect_overlap(
            sleeve,
            body,
            axes="z",
            min_overlap=0.020,
            name="descended rack rides inside the body slot",
        )

    # --- rest-pose fits and declared bearing interfaces ---------------------
    # 1. Spindle guide collar press-fit in the slot between the body plates.
    ctx.allow_overlap(
        sleeve,
        body,
        elem_a="rack_sleeve",
        elem_b="frame_shell",
        reason="Guide collar slides in the body plate slot with a light press fit that carries the rack stage.",
    )
    ctx.expect_overlap(
        sleeve,
        body,
        axes="z",
        min_overlap=0.015,
        name="guide collar rides inside the frame slot at rest",
    )
    # 2. Shaft core running fit inside the rack sleeve bore.
    ctx.allow_overlap(
        sleeve,
        handle,
        elem_a="rack_sleeve",
        elem_b="shaft_core",
        reason="Polished shaft runs inside the rack sleeve bore as the spin bearing (light running fit).",
    )
    ctx.expect_overlap(
        handle,
        sleeve,
        axes="z",
        elem_a="shaft_core",
        elem_b="rack_sleeve",
        min_overlap=0.020,
        name="shaft core stays inserted through the rack sleeve bore",
    )
    ctx.expect_within(
        handle,
        sleeve,
        axes="xy",
        inner_elem="shaft_core",
        outer_elem="rack_sleeve",
        margin=0.001,
        name="shaft core runs inside the rack sleeve bore",
    )
    ctx.expect_overlap(
        handle,
        body,
        axes="xy",
        min_overlap=0.004,
        name="shaft passes through the central body channel",
    )

    return ctx.report()


object_model = build_object_model()
