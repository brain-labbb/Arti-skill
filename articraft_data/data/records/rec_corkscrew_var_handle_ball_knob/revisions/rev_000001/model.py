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
# Classic winged ("butterfly") corkscrew, modeled upright at true scale.
#   - matte black body: twin flat plates + two legs + hollow bell skirt
#   - two chrome lever wings pivoting on rivet bosses (toothed sectors inside)
#   - chrome rack spindle (annular rack rings) sliding down through the body
#   - chrome T-bar handle + helical worm spinning continuously on the spindle
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

# Wing pivots (rivet bosses near the top of the plates).
PIVOT_X = 0.0185
PIVOT_Z = 0.113
RIVET_PIN_R = 0.0033
RIVET_CAP_R = 0.0055

# Lever wings.
WING_LEN = 0.070
WING_HALF_T = 0.0025
WING_RANGE = math.radians(100.0)

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

# Ball-knob / worm locals (expressed in the spinning shaft's frame, whose
# origin coincides with the prismatic frame at rest: world z = JOINT_TOP_Z).
WORM_COIL_R = 0.0042
WORM_WIRE_R = 0.0017
WORM_Z0 = -0.113  # world 0.012 at rest
WORM_HEIGHT = 0.046
CORE_Z0 = -0.071
CORE_Z1 = 0.025   # shaft core top (lowered to seat the ball knob above)
BALL_R = 0.0105   # ball knob sphere radius (~21 mm diameter grip)
BALL_NECK_R = 0.005
BALL_NECK_H = 0.006


def _build_body_frame() -> cq.Workplane:
    """Black frame: bell skirt + four legs + twin plates with rounded tops."""
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
    return body


def _build_wing(inner_sign: float) -> cq.Workplane:
    """Flat tapered lever wing with forked spade tip and toothed sector.

    Local frame: pivot at the origin, blade hanging along -Z at q=0.
    `inner_sign` is the local X direction that faces the central shaft.
    """
    hub = cq.Workplane("XZ").circle(0.0095).extrude(WING_HALF_T, both=True)

    arm = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.0080, -0.004),
                (0.0080, -0.004),
                (0.0055, -0.046),
                (0.0095, -0.062),
                (0.0095, -0.070),
                (0.0040, -0.070),
                (0.0000, -0.0585),
                (-0.0040, -0.070),
                (-0.0095, -0.070),
                (-0.0095, -0.062),
                (-0.0055, -0.046),
            ]
        )
        .close()
        .extrude(WING_HALF_T, both=True)
    )

    wing = hub.union(arm)

    # Toothed sector: the teeth face the rack over the full 0..100 deg swing
    # (local directions from inner-horizontal up to vertical).
    for phi_deg in (12.0, 38.0, 64.0, 90.0):
        phi = math.radians(phi_deg)
        cx = inner_sign * 0.0093 * math.sin(phi)
        cz = 0.0093 * math.cos(phi)
        tooth = (
            cq.Workplane("XZ", origin=(cx, 0.0, cz))
            .circle(0.0019)
            .extrude(0.0022, both=True)
        )
        wing = wing.union(tooth)

    # Pivot bore that journals on the rivet pin (light interference so the
    # wing is genuinely carried by the pin, declared in run_tests()).
    pivot_hole = cq.Workplane("XZ").circle(0.0030).extrude(0.02, both=True)
    return wing.cut(pivot_hole)


def _build_rack_sleeve() -> cq.Workplane:
    """Chrome rack spindle: knurled ring stack + slot guide collar."""
    sleeve = cq.Workplane("XY").cylinder(0.025, SLEEVE_R).translate((0.0, 0.0, 0.0145))
    for z_ring in (0.0065, 0.0125, 0.0185):
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


def _build_ball_knob() -> cq.Workplane:
    """Spherical ball knob with a short neck/stem that seats onto the shaft top.

    Local frame: origin at the worm_spin joint (prismatic top, world JOINT_TOP_Z
    at rest). The ball sits above the shaft core and the neck bridges them.
    """
    # Sphere center sits at the top of the core + neck + ball radius.
    sphere_center_z = CORE_Z1 + BALL_NECK_H + BALL_R
    # Stem/neck: short cylinder connecting shaft top to the sphere base.
    neck = (
        cq.Workplane("XY")
        .cylinder(BALL_NECK_H, BALL_NECK_R)
        .translate((0.0, 0.0, CORE_Z1 + 0.5 * BALL_NECK_H))
    )
    # Sphere: the main grip.
    ball = cq.Workplane("XY").sphere(BALL_R).translate((0.0, 0.0, sphere_center_z))
    return neck.union(ball)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="winged_butterfly_corkscrew")

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
    # Rivet bosses: pin through both plates + proud caps, one per wing pivot.
    for i, sx in enumerate((-1.0, 1.0)):
        body.visual(
            Cylinder(radius=RIVET_PIN_R, length=0.025),
            origin=Origin(xyz=(sx * PIVOT_X, 0.0, PIVOT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=bright_steel,
            name=f"rivet_pin_{i}",
        )
        for sy in (-1.0, 1.0):
            body.visual(
                Cylinder(radius=RIVET_CAP_R, length=0.0025),
                origin=Origin(
                    xyz=(sx * PIVOT_X, sy * 0.0112, PIVOT_Z),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=bright_steel,
                name=f"rivet_cap_{i}_{sy > 0:d}",
            )

    # ---------------------------------------------------------------- wings
    wing_0 = model.part("wing_lever_0")  # mounted on the -X rivet
    wing_0.visual(
        mesh_from_cadquery(_build_wing(inner_sign=1.0), "wing_lever_0"),
        material=chrome,
        name="wing_blade",
    )
    wing_1 = model.part("wing_lever_1")  # mounted on the +X rivet
    wing_1.visual(
        mesh_from_cadquery(_build_wing(inner_sign=-1.0), "wing_lever_1"),
        material=chrome,
        name="wing_blade",
    )

    # ----------------------------------------------------- rack spindle stage
    sleeve = model.part("rack_spindle")
    sleeve.visual(
        mesh_from_cadquery(_build_rack_sleeve(), "rack_spindle"),
        material=chrome,
        name="rack_sleeve",
    )

    # ------------------------------------------------- spinning shaft stage
    handle = model.part("ball_knob_worm")
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
    # Ball knob: lathe sphere with short neck onto the shaft top.
    handle.visual(
        mesh_from_cadquery(_build_ball_knob(), "ball_knob"),
        material=chrome,
        name="ball_knob",
    )

    # ----------------------------------------------------------- articulation
    # Wing on the -X rivet: positive q swings the blade outward (-X) and up.
    model.articulation(
        "wing_pivot_0",
        ArticulationType.REVOLUTE,
        parent=body,
        child=wing_0,
        origin=Origin(xyz=(-PIVOT_X, 0.0, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=3.0, lower=0.0, upper=WING_RANGE),
    )
    # Mirrored wing on the +X rivet: positive q swings outward (+X) and up.
    model.articulation(
        "wing_pivot_1",
        ArticulationType.REVOLUTE,
        parent=body,
        child=wing_1,
        origin=Origin(xyz=(PIVOT_X, 0.0, PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=3.0, lower=0.0, upper=WING_RANGE),
    )
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
    wing_0 = object_model.get_part("wing_lever_0")
    wing_1 = object_model.get_part("wing_lever_1")
    sleeve = object_model.get_part("rack_spindle")
    handle = object_model.get_part("ball_knob_worm")
    j_w0 = object_model.get_articulation("wing_pivot_0")
    j_w1 = object_model.get_articulation("wing_pivot_1")
    j_travel = object_model.get_articulation("spindle_travel")
    j_spin = object_model.get_articulation("worm_spin")

    # --- joint plan conformance ------------------------------------------
    ctx.check(
        "both wings are revolute with ~100 deg range",
        j_w0.articulation_type == ArticulationType.REVOLUTE
        and j_w1.articulation_type == ArticulationType.REVOLUTE
        and j_w0.motion_limits.lower == 0.0
        and j_w1.motion_limits.lower == 0.0
        and abs(j_w0.motion_limits.upper - WING_RANGE) < 1e-9
        and abs(j_w1.motion_limits.upper - WING_RANGE) < 1e-9,
    )
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
        "overall height ~0.17 m at the ball knob top",
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

    # --- wings: pivots, rest pose, raised tip-to-tip span ------------------
    p0 = ctx.part_world_position(wing_0)
    p1 = ctx.part_world_position(wing_1)
    ctx.check(
        "wing pivots sit on the rivet bosses",
        p0 is not None
        and p1 is not None
        and abs(p0[0] + PIVOT_X) < 1e-6
        and abs(p1[0] - PIVOT_X) < 1e-6
        and abs(p0[2] - PIVOT_Z) < 1e-6
        and abs(p1[2] - PIVOT_Z) < 1e-6,
    )
    w0_rest = ctx.part_world_aabb(wing_0)
    w1_rest = ctx.part_world_aabb(wing_1)
    ctx.check(
        "wings hang down beside the frame at rest",
        w0_rest is not None
        and w1_rest is not None
        and 0.030 <= w0_rest[0][2] <= 0.055
        and 0.030 <= w1_rest[0][2] <= 0.055,
    )
    with ctx.pose({j_w0: WING_RANGE, j_w1: WING_RANGE}):
        w0_up = ctx.part_world_aabb(wing_0)
        w1_up = ctx.part_world_aabb(wing_1)
        span = w1_up[1][0] - w0_up[0][0]
        ctx.check(
            "raised wings span ~0.16 m tip-to-tip",
            0.150 <= span <= 0.200,
            details=f"span={span:.4f}",
        )
        ctx.check(
            "raising the levers swings both blades upward and outward",
            w0_up[0][2] > w0_rest[0][2] + 0.040
            and w1_up[0][2] > w1_rest[0][2] + 0.040
            and w0_up[0][0] < w0_rest[0][0] - 0.030
            and w1_up[1][0] > w1_rest[1][0] + 0.030,
        )

    # --- continuous spin: ball knob is spherical; worm helix proves rotation
    # Ball knob should read as roughly spherical (X and Y extents similar).
    ball_aabb = ctx.part_element_world_aabb(handle, elem="ball_knob")
    ctx.check(
        "ball knob is roughly spherical (X and Y extents similar)",
        ball_aabb is not None
        and abs((ball_aabb[1][0] - ball_aabb[0][0]) - (ball_aabb[1][1] - ball_aabb[0][1])) < 0.003,
        details=f"ball_knob aabb={ball_aabb}",
    )
    ctx.check(
        "ball knob diameter ~21 mm",
        ball_aabb is not None
        and 0.018 <= (ball_aabb[1][0] - ball_aabb[0][0]) <= 0.024,
        details=f"ball_knob x_extent={ball_aabb[1][0] - ball_aabb[0][0]:.4f}",
    )
    # Ball knob sits atop the shaft (above the shaft core top).
    shaft_aabb = ctx.part_element_world_aabb(handle, elem="shaft_core")
    ctx.check(
        "ball knob sits above the shaft core",
        ball_aabb is not None
        and shaft_aabb is not None
        and ball_aabb[0][2] > shaft_aabb[1][2] - 0.002,
        details=f"ball_bottom={ball_aabb[0][2]:.4f}, shaft_top={shaft_aabb[1][2]:.4f}",
    )

    # --- prismatic descent --------------------------------------------------
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
    # 1. Rivet pins captured in the wing hub bores (journal fit).
    ctx.allow_overlap(
        body,
        wing_0,
        elem_a="rivet_pin_0",
        elem_b="wing_blade",
        reason="Rivet pin is intentionally captured in the wing hub bore as the pivot journal.",
    )
    ctx.allow_overlap(
        body,
        wing_1,
        elem_a="rivet_pin_1",
        elem_b="wing_blade",
        reason="Rivet pin is intentionally captured in the wing hub bore as the pivot journal.",
    )
    ctx.expect_overlap(
        wing_0,
        body,
        axes="y",
        elem_b="rivet_pin_0",
        min_overlap=0.004,
        name="wing hub stays threaded on its rivet pin",
    )
    ctx.expect_overlap(
        wing_1,
        body,
        axes="y",
        elem_b="rivet_pin_1",
        min_overlap=0.004,
        name="mirrored wing hub stays threaded on its rivet pin",
    )
    # 2. Spindle guide collar press-fit in the slot between the body plates.
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
    # 3. Shaft core running fit inside the rack sleeve bore.
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
