from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Single-hole basin faucet variant: squat body, wide oval pedestal, swiveling
# spout, grip grooves, two rear screw caps, press-and-turn push cap.
# World frame: +Z up, deck at z = 0, spout projects toward +X (front).
# Body is vertical (no tilt).
# ---------------------------------------------------------------------------

# Oval pedestal (sits flat on the deck).
PED_XR = 0.035   # half-extent along X (front-back)
PED_YR = 0.025   # half-extent along Y (side-side)
PED_H = 0.012

# Main body barrel rising from the pedestal.
BODY_R = 0.022
BODY_Z0 = PED_H
BODY_Z1 = 0.075
BODY_H = BODY_Z1 - BODY_Z0

# Grip groove zone: thin recessed rings around mid-body.
GROOVE_Z_CENTER = (BODY_Z0 + BODY_Z1) / 2.0
GROOVE_R = BODY_R - 0.0015  # slightly inset
NUM_GROOVES = 5
GROOVE_SPACING = 0.005
GROOVE_H = 0.002

# Upper neck: slight step-in above the body.
NECK_R = 0.019
NECK_Z0 = BODY_Z1
NECK_Z1 = 0.095
NECK_H = NECK_Z1 - NECK_Z0

# Valve stem: prismatic press carrier, nested in neck bore.
# Stem part frame origin is at the cap_press articulation frame
# (body's z = NECK_Z1). In stem-local coords, stem extends below and above.
STEM_R = 0.010
STEM_LEN = 0.030  # total length; extends below the joint into the neck

# Push cap dimensions (cap-local frame: z=0 is disc bottom).
CAP_R = 0.026
CAP_DISC_H = 0.012
CAP_FLARE_H = 0.003
CAP_FLARE_R0 = 0.014

# Cap sits above the stem top. cap_turn origin is relative to stem frame.
# Stem top in stem-local: STEM_LEN/2 = 0.015 above stem origin.
# Cap disc bottom should sit at or just above the stem top.
CAP_LOCAL_Z = 0.015  # cap joint origin in stem-local z

# Spout exit on the body front at mid-height.
SPOUT_Z = 0.055

# Screw caps on the back of the body.
SCREW_R = 0.005
SCREW_H = 0.004
SCREW_Z = 0.050
SCREW_Y_OFFSET = 0.012

PRESS_TRAVEL = 0.008
TURN_LIMIT = math.radians(60.0)
SWIVEL_LIMIT = math.radians(90.0)


def _build_oval_pedestal() -> cq.Workplane:
    """Wide oval pedestal: an extruded ellipse sitting on z=0."""
    pedestal = (
        cq.Workplane("XY")
        .ellipse(PED_XR, PED_YR)
        .extrude(PED_H)
    )
    pedestal = pedestal.edges(">Z").chamfer(0.001)
    return pedestal


def _build_body_with_grooves() -> cq.Workplane:
    """Cylindrical body barrel with subtle horizontal grip grooves cut into
    the outer surface around mid-height."""
    body = (
        cq.Workplane("XY", origin=(0.0, 0.0, BODY_Z0))
        .circle(BODY_R)
        .extrude(BODY_H)
    )
    for i in range(NUM_GROOVES):
        z_g = GROOVE_Z_CENTER + (i - (NUM_GROOVES - 1) / 2.0) * GROOVE_SPACING
        groove_ring = (
            cq.Workplane("XY", origin=(0.0, 0.0, z_g - GROOVE_H / 2.0))
            .circle(BODY_R + 0.001)
            .circle(GROOVE_R)
            .extrude(GROOVE_H)
        )
        body = body.cut(groove_ring)
    body = body.edges(">Z").fillet(0.001)
    return body


def _build_spout_shape() -> cq.Workplane:
    """Hollow chrome spout: straight shank projecting forward (+X), smooth
    downward bend, flared open outlet rim. Spout-local origin at the swivel
    axis; the shank runs along local +X."""
    r_out = 0.012
    shank_x0 = 0.008
    shank_x1 = 0.040
    bend = 0.025
    end_x = shank_x1 + bend
    end_z = -bend

    path = (
        cq.Workplane("XZ")
        .moveTo(shank_x0, 0.0)
        .lineTo(shank_x1, 0.0)
        .tangentArcPoint((bend, -bend), relative=True)
    )
    tube = cq.Workplane("YZ", origin=(shank_x0, 0.0, 0.0)).circle(r_out).sweep(path)

    flare = (
        cq.Workplane("XY", origin=(end_x, 0.0, end_z + 0.005))
        .circle(0.0118)
        .workplane(offset=-0.008)
        .circle(0.015)
        .loft()
    )
    spout = tube.union(flare)

    bore = (
        cq.Workplane("XY", origin=(end_x, 0.0, end_z - 0.005))
        .circle(0.012)
        .workplane(offset=0.014)
        .circle(0.009)
        .loft()
    )
    return spout.cut(bore)


def _build_cap_shape() -> cq.Workplane:
    """Push cap shell: under-flare cone from the stem boss out to the flat
    disc, with a softened top rim. Cap-local z=0 is the disc bottom."""
    flare = (
        cq.Workplane("XY", origin=(0.0, 0.0, -CAP_FLARE_H))
        .circle(CAP_FLARE_R0)
        .workplane(offset=CAP_FLARE_H)
        .circle(CAP_R)
        .loft()
    )
    disc = cq.Workplane("XY").circle(CAP_R).extrude(CAP_DISC_H)
    cap = disc.union(flare)
    cap = cap.edges(">Z").fillet(0.0012)
    return cap


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hole_basin_faucet")

    model.material("chrome", rgba=(0.85, 0.87, 0.89, 1.0))
    model.material("chrome_dark", rgba=(0.42, 0.44, 0.47, 1.0))
    model.material("chrome_brushed", rgba=(0.70, 0.72, 0.74, 1.0))
    model.material("screw_cap", rgba=(0.55, 0.57, 0.60, 1.0))
    model.material("index_mark", rgba=(0.30, 0.32, 0.35, 1.0))

    # ======================== body (root) ==================================
    body = model.part("body")

    # Oval pedestal on the deck.
    body.visual(
        mesh_from_cadquery(_build_oval_pedestal(), "oval_pedestal", tolerance=0.0003),
        material="chrome",
        name="oval_pedestal",
    )
    # Cylindrical body barrel with grip grooves.
    body.visual(
        mesh_from_cadquery(_build_body_with_grooves(), "body_barrel", tolerance=0.0003),
        material="chrome",
        name="body_barrel",
    )
    # Upper neck above the body.
    body.visual(
        Cylinder(radius=NECK_R, length=NECK_H),
        origin=Origin(xyz=(0.0, 0.0, NECK_Z0 + NECK_H / 2.0)),
        material="chrome",
        name="body_neck",
    )

    # Two small screw caps on the back (-X side) of the body barrel.
    # Embed them so the inner face penetrates the body barrel surface.
    screw_x = -(BODY_R - 0.001)  # center pushed inside the barrel wall
    for i, y_off in enumerate((-SCREW_Y_OFFSET, SCREW_Y_OFFSET)):
        body.visual(
            Cylinder(radius=SCREW_R, length=SCREW_H),
            origin=Origin(
                xyz=(screw_x, y_off, SCREW_Z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material="screw_cap",
            name=f"screw_cap_{i}",
        )
        # Slot groove on each screw cap (thin dark stripe).
        body.visual(
            Box((0.0008, SCREW_R * 1.4, 0.0008)),
            origin=Origin(xyz=(screw_x - 0.002, y_off, SCREW_Z)),
            material="chrome_dark",
            name=f"screw_slot_{i}",
        )

    # ======================== spout (swiveling) ============================
    spout = model.part("spout")
    spout.visual(
        mesh_from_cadquery(_build_spout_shape(), "spout_tube", tolerance=0.0003),
        material="chrome",
        name="spout_tube",
    )
    # Revolute joint: spout swivels around the vertical body axis.
    model.articulation(
        "spout_swivel",
        ArticulationType.REVOLUTE,
        parent=body,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, SPOUT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=2.0, lower=-SWIVEL_LIMIT, upper=SWIVEL_LIMIT
        ),
    )

    # ======================== valve stem (prismatic press) =================
    stem = model.part("valve_stem")
    # Stem shaft in stem-local coords: centered at origin, extends
    # -STEM_LEN/2 to +STEM_LEN/2. The stem part frame is at the cap_press
    # articulation frame (body z = NECK_Z1). So the shaft goes from
    # world z = NECK_Z1 - STEM_LEN/2 to NECK_Z1 + STEM_LEN/2
    # = 0.095 - 0.015 to 0.095 + 0.015 = 0.080 to 0.110.
    stem.visual(
        Cylinder(radius=STEM_R, length=STEM_LEN),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="chrome",
        name="stem_shaft",
    )
    # cap_press: joint at neck top. Axis -Z means positive q presses DOWN.
    model.articulation(
        "cap_press",
        ArticulationType.PRISMATIC,
        parent=body,
        child=stem,
        origin=Origin(xyz=(0.0, 0.0, NECK_Z1)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=30.0, velocity=0.05, lower=0.0, upper=PRESS_TRAVEL
        ),
    )

    # ======================== push cap (revolute temperature) ==============
    cap = model.part("push_cap")
    cap.visual(
        mesh_from_cadquery(_build_cap_shape(), "push_cap", tolerance=0.0003),
        material="chrome",
        name="cap_shell",
    )
    cap.visual(
        Cylinder(radius=CAP_R - 0.002, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, CAP_DISC_H - 0.001)),
        material="chrome_brushed",
        name="cap_top_brushed",
    )
    # Temperature indicator dot on the front rim of the cap.
    cap.visual(
        Cylinder(radius=0.002, length=0.002),
        origin=Origin(
            xyz=(CAP_R - 0.001, 0.0, CAP_DISC_H / 2.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material="index_mark",
        name="temp_indicator_dot",
    )
    # cap_turn: revolute around Z, child of stem. Origin in stem frame
    # at stem top (local z = STEM_LEN/2).
    model.articulation(
        "cap_turn",
        ArticulationType.REVOLUTE,
        parent=stem,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, CAP_LOCAL_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=2.0, lower=-TURN_LIMIT, upper=TURN_LIMIT
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    spout = object_model.get_part("spout")
    stem = object_model.get_part("valve_stem")
    cap = object_model.get_part("push_cap")
    swivel = object_model.get_articulation("spout_swivel")
    press = object_model.get_articulation("cap_press")
    turn = object_model.get_articulation("cap_turn")

    # Intentional seated insertions.
    ctx.allow_overlap(
        spout,
        body,
        elem_a="spout_tube",
        elem_b="body_barrel",
        reason="Spout shank is intentionally seated inside the solid body casting.",
    )
    ctx.allow_overlap(
        stem,
        body,
        elem_a="stem_shaft",
        elem_b="body_neck",
        reason="Valve stem nests inside the solid neck bore proxy and slides deeper when pressed.",
    )
    ctx.allow_overlap(
        cap,
        stem,
        elem_a="cap_shell",
        elem_b="stem_shaft",
        reason="Stem top is press-fit into the cap under-flare boss.",
    )

    # ---- Oval pedestal: wider than tall, sits on deck --------------------
    pedestal_aabb = ctx.part_element_world_aabb(body, elem="oval_pedestal")
    ctx.check(
        "oval pedestal sits flat on the deck",
        pedestal_aabb is not None and abs(pedestal_aabb[0][2]) <= 0.001,
        details=f"pedestal aabb={pedestal_aabb}",
    )
    ctx.check(
        "pedestal is squat and wide (XY extents exceed Z)",
        pedestal_aabb is not None
        and (pedestal_aabb[1][0] - pedestal_aabb[0][0]) > 0.050
        and (pedestal_aabb[1][1] - pedestal_aabb[0][1]) > 0.035
        and (pedestal_aabb[1][2] - pedestal_aabb[0][2]) < 0.020,
        details=f"pedestal aabb={pedestal_aabb}",
    )
    ctx.check(
        "pedestal is oval (front-back wider than side-side)",
        pedestal_aabb is not None
        and (pedestal_aabb[1][0] - pedestal_aabb[0][0])
        > (pedestal_aabb[1][1] - pedestal_aabb[0][1]) + 0.005,
        details=f"pedestal aabb={pedestal_aabb}",
    )

    # ---- Body barrel above pedestal --------------------------------------
    barrel_aabb = ctx.part_element_world_aabb(body, elem="body_barrel")
    ctx.check(
        "body barrel is present above the pedestal",
        barrel_aabb is not None
        and barrel_aabb[0][2] > 0.008
        and barrel_aabb[1][2] > 0.050,
        details=f"barrel aabb={barrel_aabb}",
    )

    # ---- Screw caps on back of body --------------------------------------
    screw0_aabb = ctx.part_element_world_aabb(body, elem="screw_cap_0")
    screw1_aabb = ctx.part_element_world_aabb(body, elem="screw_cap_1")
    ctx.check(
        "two screw caps are present on the back of the body",
        screw0_aabb is not None
        and screw1_aabb is not None
        and screw0_aabb[1][0] < 0.0
        and screw1_aabb[1][0] < 0.0,
        details=f"screw0={screw0_aabb}, screw1={screw1_aabb}",
    )
    ctx.check(
        "screw caps are laterally separated",
        screw0_aabb is not None
        and screw1_aabb is not None
        and abs(
            (screw0_aabb[0][1] + screw0_aabb[1][1]) / 2.0
            - (screw1_aabb[0][1] + screw1_aabb[1][1]) / 2.0
        ) > 0.010,
        details=f"screw0={screw0_aabb}, screw1={screw1_aabb}",
    )

    # ---- Spout: projects forward and swivels -----------------------------
    ctx.expect_overlap(
        spout,
        body,
        axes="xz",
        min_overlap=0.003,
        name="spout shank stays seated in the body",
    )
    spout_rest_aabb = ctx.part_world_aabb(spout)
    ctx.check(
        "spout reaches forward from the body front",
        spout_rest_aabb is not None and spout_rest_aabb[1][0] > 0.050,
        details=f"spout aabb={spout_rest_aabb}",
    )

    # Spout swivel: use the spout tube element AABB to detect lateral motion.
    spout_rest_elem = ctx.part_element_world_aabb(spout, elem="spout_tube")
    with ctx.pose({swivel: SWIVEL_LIMIT}):
        spout_swiveled_elem = ctx.part_element_world_aabb(spout, elem="spout_tube")
    ctx.check(
        "spout swivels sideways when the swivel joint is actuated",
        spout_rest_elem is not None
        and spout_swiveled_elem is not None
        and abs(
            (spout_swiveled_elem[0][1] + spout_swiveled_elem[1][1]) / 2.0
            - (spout_rest_elem[0][1] + spout_rest_elem[1][1]) / 2.0
        ) > 0.020,
        details=f"rest_y_center={(spout_rest_elem[0][1] + spout_rest_elem[1][1]) / 2.0}, "
                f"swiveled_y_center={(spout_swiveled_elem[0][1] + spout_swiveled_elem[1][1]) / 2.0}",
    )

    # ---- Stem/cap stack: retained insertion and support ------------------
    ctx.expect_overlap(
        stem,
        body,
        axes="z",
        elem_a="stem_shaft",
        elem_b="body_neck",
        min_overlap=0.005,
        name="valve stem retained inside the neck bore",
    )
    ctx.expect_overlap(
        cap,
        stem,
        axes="z",
        elem_a="cap_shell",
        elem_b="stem_shaft",
        min_overlap=0.001,
        name="cap boss retains the stem top",
    )
    ctx.expect_within(
        stem,
        cap,
        axes="xy",
        inner_elem="stem_shaft",
        margin=0.002,
        name="stem stays centered under the push cap",
    )
    neck_aabb = ctx.part_element_world_aabb(body, elem="body_neck")
    cap_aabb = ctx.part_world_aabb(cap)
    ctx.check(
        "push cap sits above the neck",
        cap_aabb is not None
        and neck_aabb is not None
        and cap_aabb[0][2] > neck_aabb[0][2],
        details=f"cap aabb={cap_aabb}, neck aabb={neck_aabb}",
    )
    ctx.check(
        "push cap is wider than the neck (flared button)",
        cap_aabb is not None
        and neck_aabb is not None
        and (cap_aabb[1][1] - cap_aabb[0][1]) > (neck_aabb[1][1] - neck_aabb[0][1]) + 0.006,
        details=f"cap aabb={cap_aabb}, neck aabb={neck_aabb}",
    )
    ctx.check(
        "overall faucet height is about 0.12 m",
        cap_aabb is not None and 0.105 <= cap_aabb[1][2] <= 0.130,
        details=f"cap aabb={cap_aabb}",
    )

    # ---- Articulation limits ---------------------------------------------
    sl = swivel.motion_limits
    ctx.check(
        "spout swivel limits are -90 to +90 degrees",
        sl is not None
        and sl.lower is not None
        and sl.upper is not None
        and abs(sl.lower + SWIVEL_LIMIT) < 1e-6
        and abs(sl.upper - SWIVEL_LIMIT) < 1e-6,
        details=f"limits={sl}",
    )
    pl = press.motion_limits
    ctx.check(
        "press travel limits are 0 to 8 mm",
        pl is not None
        and pl.lower is not None
        and pl.upper is not None
        and abs(pl.lower) < 1e-9
        and abs(pl.upper - PRESS_TRAVEL) < 1e-9,
        details=f"limits={pl}",
    )
    tl = turn.motion_limits
    ctx.check(
        "temperature turn limits are -60 to +60 degrees",
        tl is not None
        and tl.lower is not None
        and tl.upper is not None
        and abs(tl.lower + TURN_LIMIT) < 1e-6
        and abs(tl.upper - TURN_LIMIT) < 1e-6,
        details=f"limits={tl}",
    )

    # ---- Decisive poses --------------------------------------------------
    rest_pos = ctx.part_world_position(cap)
    with ctx.pose({press: PRESS_TRAVEL}):
        pressed_pos = ctx.part_world_position(cap)
    ctx.check(
        "pressing the cap moves it down along the vertical body axis",
        rest_pos is not None
        and pressed_pos is not None
        and 0.006 <= (rest_pos[2] - pressed_pos[2]) <= 0.010,
        details=f"rest={rest_pos}, pressed={pressed_pos}",
    )

    dot_rest = ctx.part_element_world_aabb(cap, elem="temp_indicator_dot")
    with ctx.pose({turn: TURN_LIMIT}):
        dot_hot = ctx.part_element_world_aabb(cap, elem="temp_indicator_dot")
    ctx.check(
        "turning the cap +60 deg swings the index mark around the cap axis",
        dot_rest is not None
        and dot_hot is not None
        and abs((dot_rest[0][1] + dot_rest[1][1]) / 2.0) < 0.005
        and abs((dot_hot[0][1] + dot_hot[1][1]) / 2.0) > 0.010,
        details=f"rest={dot_rest}, turned={dot_hot}",
    )

    return ctx.report()


object_model = build_object_model()
