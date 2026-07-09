from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Single-hole basin faucet variant (v19), ~0.13 m tall, mirror chrome.
# World frame: +Z up, deck at z = 0, spout points toward +X (front).
# Body leans BACK a few degrees (long axis tilts toward -X).
# ---------------------------------------------------------------------------

TILT = math.radians(6.0)
SIN_T = math.sin(TILT)
COS_T = math.cos(TILT)

# Base flange (sits flat on the deck).
FLANGE_R = 0.030
FLANGE_H = 0.006

# Oval base gasket under the flange.
GASKET_A = 0.036          # semi-major axis (X, toward spout)
GASKET_B = 0.032          # semi-minor axis (Y)
GASKET_H = 0.003
GASKET_HOLE_R = 0.026     # circular pass-through for body

# Main body barrel.
BODY_R = 0.025
BODY_S0 = 0.006
BODY_S1 = 0.0725

# Thin recessed separation groove ring.
GROOVE_R = 0.0215
GROOVE_S0 = 0.0705
GROOVE_S1 = 0.0760

# Stepped-in upper neck above the groove.
NECK_R = 0.0228
NECK_S0 = 0.0740
NECK_S1 = 0.104

# Valve stem (prismatic press carrier) nested in the neck bore.
STEM_R = 0.011
STEM_S0 = 0.0925
STEM_S1 = 0.1155
STEM_LEN = STEM_S1 - STEM_S0

# Push cap (press-to-run button, also rotates for temperature).
CAP_JOINT_S = 0.117
CAP_R = 0.030
CAP_DISC_H = 0.013
CAP_FLARE_H = 0.004
CAP_FLARE_R0 = 0.016

# Spout exit station on the body axis.
SPOUT_S = 0.050

# Spout geometry constants (local frame at body axis, shank along +X).
SPOUT_R_OUT = 0.015
SHANK_X0 = 0.010          # seated ~10 mm inside body casting
SHANK_X1 = 0.035
BEND_R = 0.028            # bend radius
SPOUT_END_X = SHANK_X1 + BEND_R   # 0.063
SPOUT_END_Z = -BEND_R              # -0.028

# Collar seam ring at spout-body junction (detachable look).
COLLAR_X = 0.017          # just outside body surface
COLLAR_OUTER_R = 0.0195
COLLAR_LEN = 0.007

# Aerator disc at spout outlet mouth.
AERATOR_DISC_R = 0.013
AERATOR_DISC_H = 0.003
AERATOR_HINGE_OFFSET = 0.014  # hinge at back edge of outlet
# Outlet mouth bottom z in spout local frame.
OUTLET_MOUTH_Z = SPOUT_END_Z - 0.006  # -0.034

# Articulation limits.
PRESS_TRAVEL = 0.008
TURN_LIMIT = math.radians(60.0)
AERATOR_OPEN = math.radians(90.0)


def _axis_point(s: float) -> tuple[float, float, float]:
    """World position of the tilted body axis at axial station s."""
    return (-s * SIN_T, 0.0, s * COS_T)


def _tilted(s: float) -> Origin:
    """Origin on the body axis at station s, z-axis aligned with the axis."""
    return Origin(xyz=_axis_point(s), rpy=(0.0, -TILT, 0.0))


def _build_spout_shape() -> cq.Workplane:
    """Hollow chrome spout: straight shank, smooth downward bend, flared
    open outlet rim, collar seam ring at body junction. Built in spout-local
    frame whose origin sits on the body axis at SPOUT_S; shank along +X."""
    r_out = SPOUT_R_OUT

    path = (
        cq.Workplane("XZ")
        .moveTo(SHANK_X0, 0.0)
        .lineTo(SHANK_X1, 0.0)
        .tangentArcPoint((BEND_R, -BEND_R), relative=True)
    )
    tube = cq.Workplane("YZ", origin=(SHANK_X0, 0.0, 0.0)).circle(r_out).sweep(path)

    # Flared outlet skirt around the down-turned end.
    flare = (
        cq.Workplane("XY", origin=(SPOUT_END_X, 0.0, SPOUT_END_Z + 0.006))
        .circle(0.0148)
        .workplane(offset=-0.010)
        .circle(0.0185)
        .loft()
    )
    spout = tube.union(flare)

    # Tapered bore opening the outlet mouth (real hollow outlet rim).
    bore = (
        cq.Workplane("XY", origin=(SPOUT_END_X, 0.0, SPOUT_END_Z - 0.006))
        .circle(0.0155)
        .workplane(offset=0.018)
        .circle(0.011)
        .loft()
    )
    spout = spout.cut(bore)

    # Collar seam ring at spout-body junction (detachable look).
    collar = (
        cq.Workplane("YZ", origin=(COLLAR_X - 0.001, 0.0, 0.0))
        .circle(COLLAR_OUTER_R)
        .extrude(COLLAR_LEN)
    )
    collar_bore = (
        cq.Workplane("YZ", origin=(COLLAR_X - 0.002, 0.0, 0.0))
        .circle(r_out - 0.0005)
        .extrude(COLLAR_LEN + 0.002)
    )
    collar = collar.cut(collar_bore)
    spout = spout.union(collar)

    # Small hinge knuckle on the spout side (rear of outlet mouth).
    knuckle_x = SPOUT_END_X - AERATOR_HINGE_OFFSET
    knuckle_z = OUTLET_MOUTH_Z
    knuckle = (
        cq.Workplane("XZ", origin=(knuckle_x, -0.004, knuckle_z))
        .circle(0.002)
        .extrude(0.008)
    )
    spout = spout.union(knuckle)

    return spout


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
    cap = cap.edges(">Z").fillet(0.0015)
    return cap


def _build_gasket_shape() -> cq.Workplane:
    """Oval rubber gasket under the base flange with circular body hole."""
    gasket = (
        cq.Workplane("XY")
        .ellipse(GASKET_A, GASKET_B)
        .extrude(GASKET_H)
    )
    hole = (
        cq.Workplane("XY", origin=(0.0, 0.0, -0.001))
        .circle(GASKET_HOLE_R)
        .extrude(GASKET_H + 0.002)
    )
    return gasket.cut(hole)


def _build_aerator_shape() -> cq.Workplane:
    """Aerator disc with hinge tab. Aerator part frame sits at the hinge
    pin; the disc extends forward (+X) from the hinge to cover the outlet."""
    # Main disc: centered at +AERATOR_HINGE_OFFSET along +X, flat in XY.
    disc = (
        cq.Workplane("XY", origin=(AERATOR_HINGE_OFFSET, 0.0, 0.0))
        .circle(AERATOR_DISC_R)
        .extrude(AERATOR_DISC_H)
    )
    # Small hinge tab that wraps around the knuckle pin.
    tab = (
        cq.Workplane("YZ", origin=(-0.003, -0.003, -0.001))
        .box(0.006, 0.006, 0.004, centered=False)
    )
    aerator = disc.union(tab)
    # Perforated pattern (3 small holes to suggest mesh screen).
    for dx, dy in [(0.008, 0.0), (0.016, 0.004), (0.016, -0.004)]:
        hole = (
            cq.Workplane("XY", origin=(dx, dy, -0.001))
            .circle(0.002)
            .extrude(AERATOR_DISC_H + 0.002)
        )
        aerator = aerator.cut(hole)
    return aerator


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hole_basin_faucet_v19")

    model.material("chrome", rgba=(0.85, 0.87, 0.89, 1.0))
    model.material("chrome_dark", rgba=(0.42, 0.44, 0.47, 1.0))
    model.material("chrome_brushed", rgba=(0.70, 0.72, 0.74, 1.0))
    model.material("index_mark", rgba=(0.30, 0.32, 0.35, 1.0))
    model.material("rubber_gasket", rgba=(0.12, 0.12, 0.13, 1.0))
    model.material("aerator_screen", rgba=(0.55, 0.56, 0.58, 1.0))

    # ---------------- body (root): flange + barrel + groove + neck + gasket
    body = model.part("body")

    # Oval base gasket sits below the flange on the deck.
    body.visual(
        mesh_from_cadquery(_build_gasket_shape(), "base_gasket", tolerance=0.0003),
        origin=Origin(xyz=(0.0, 0.0, -GASKET_H)),
        material="rubber_gasket",
        name="base_gasket",
    )

    body.visual(
        Cylinder(radius=FLANGE_R, length=FLANGE_H),
        origin=Origin(xyz=(0.0, 0.0, FLANGE_H / 2.0)),
        material="chrome",
        name="base_flange",
    )
    body.visual(
        Cylinder(radius=BODY_R, length=BODY_S1 - BODY_S0),
        origin=_tilted((BODY_S0 + BODY_S1) / 2.0),
        material="chrome",
        name="body_barrel",
    )
    body.visual(
        Cylinder(radius=GROOVE_R, length=GROOVE_S1 - GROOVE_S0),
        origin=_tilted((GROOVE_S0 + GROOVE_S1) / 2.0),
        material="chrome_dark",
        name="groove_ring",
    )
    body.visual(
        Cylinder(radius=NECK_R, length=NECK_S1 - NECK_S0),
        origin=_tilted((NECK_S0 + NECK_S1) / 2.0),
        material="chrome",
        name="body_neck",
    )

    # ---------------- spout: swept hollow tube + collar + aerator hinge knuckle
    spout = model.part("spout")
    spout.visual(
        mesh_from_cadquery(_build_spout_shape(), "spout", tolerance=0.0003),
        material="chrome",
        name="spout_tube",
    )
    model.articulation(
        "spout_mount",
        ArticulationType.FIXED,
        parent=body,
        child=spout,
        origin=Origin(xyz=_axis_point(SPOUT_S)),
    )

    # ---------------- aerator: flip-open disc on tiny hinge at spout outlet
    aerator = model.part("aerator")
    aerator.visual(
        mesh_from_cadquery(_build_aerator_shape(), "aerator", tolerance=0.0003),
        material="aerator_screen",
        name="aerator_disc",
    )
    # Hinge origin: rear edge of outlet mouth in spout local frame.
    # Spout part frame = world frame translated to body axis at SPOUT_S.
    # Hinge point in spout frame: (SPOUT_END_X - AERATOR_HINGE_OFFSET, 0, OUTLET_MOUTH_Z)
    aerator_hinge_origin = Origin(
        xyz=(SPOUT_END_X - AERATOR_HINGE_OFFSET, 0.0, OUTLET_MOUTH_Z)
    )
    model.articulation(
        "aerator_flip",
        ArticulationType.REVOLUTE,
        parent=spout,
        child=aerator,
        origin=aerator_hinge_origin,
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0, velocity=2.0, lower=0.0, upper=AERATOR_OPEN
        ),
    )

    # ---------------- valve stem (prismatic press carrier)
    stem = model.part("valve_stem")
    stem.visual(
        Cylinder(radius=STEM_R, length=STEM_LEN),
        origin=Origin(),
        material="chrome",
        name="stem_shaft",
    )
    model.articulation(
        "cap_press",
        ArticulationType.PRISMATIC,
        parent=body,
        child=stem,
        origin=Origin(xyz=_axis_point(NECK_S1), rpy=(0.0, -TILT, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.05, lower=0.0, upper=PRESS_TRAVEL),
    )

    # ---------------- push cap (revolute temperature ring)
    cap = model.part("push_cap")
    cap.visual(
        mesh_from_cadquery(_build_cap_shape(), "push_cap", tolerance=0.0003),
        material="chrome",
        name="cap_shell",
    )
    cap.visual(
        Cylinder(radius=0.028, length=0.0025),
        origin=Origin(xyz=(0.0, 0.0, 0.01225)),
        material="chrome_brushed",
        name="cap_top_brushed",
    )
    cap.visual(
        Cylinder(radius=0.0025, length=0.0025),
        origin=Origin(xyz=(0.0295, 0.0, 0.0065), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="index_mark",
        name="temp_indicator_dot",
    )
    model.articulation(
        "cap_turn",
        ArticulationType.REVOLUTE,
        parent=stem,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, CAP_JOINT_S - NECK_S1)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=-TURN_LIMIT, upper=TURN_LIMIT),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    spout = object_model.get_part("spout")
    aerator = object_model.get_part("aerator")
    stem = object_model.get_part("valve_stem")
    cap = object_model.get_part("push_cap")
    press = object_model.get_articulation("cap_press")
    turn = object_model.get_articulation("cap_turn")
    aerator_flip = object_model.get_articulation("aerator_flip")

    # --- Intentional seated insertions (scoped per element) ---
    ctx.allow_overlap(
        spout,
        body,
        elem_a="spout_tube",
        elem_b="body_barrel",
        reason="Spout shank is intentionally seated ~10 mm into the solid body casting.",
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
        reason="Stem top is press-fit ~2.5 mm into the cap's under-flare boss.",
    )
    ctx.allow_overlap(
        aerator,
        spout,
        elem_a="aerator_disc",
        elem_b="spout_tube",
        reason="Aerator hinge tab wraps around the spout knuckle pin for hinge capture.",
    )

    # --- Oval base gasket ---
    gasket_aabb = ctx.part_element_world_aabb(body, elem="base_gasket")
    flange_aabb = ctx.part_element_world_aabb(body, elem="base_flange")
    ctx.check(
        "oval base gasket exists below the flange",
        gasket_aabb is not None
        and flange_aabb is not None
        and gasket_aabb[0][2] < flange_aabb[0][2],
        details=f"gasket={gasket_aabb}, flange={flange_aabb}",
    )
    ctx.check(
        "gasket is oval (wider in X than Y or vice versa)",
        gasket_aabb is not None
        and abs(
            (gasket_aabb[1][0] - gasket_aabb[0][0])
            - (gasket_aabb[1][1] - gasket_aabb[0][1])
        ) > 0.002,
        details=f"gasket aabb={gasket_aabb}",
    )

    # --- Flange seated on deck ---
    ctx.check(
        "base flange sits flat on the deck",
        flange_aabb is not None and abs(flange_aabb[0][2]) <= 0.001,
        details=f"flange aabb={flange_aabb}",
    )

    # --- Body leans back ---
    neck_aabb = ctx.part_element_world_aabb(body, elem="body_neck")
    ctx.check(
        "body leans back (neck offset toward -X behind the flange center)",
        neck_aabb is not None and (neck_aabb[0][0] + neck_aabb[1][0]) / 2.0 < -0.005,
        details=f"neck aabb={neck_aabb}",
    )

    # --- Spout collar seam ---
    spout_aabb = ctx.part_world_aabb(spout)
    ctx.check(
        "spout projects forward and includes collar at body junction",
        spout_aabb is not None
        and spout_aabb[1][0] > 0.060
        and spout_aabb[0][2] < 0.025
        and spout_aabb[0][2] > 0.008,
        details=f"spout aabb={spout_aabb}",
    )
    ctx.expect_overlap(
        spout,
        body,
        axes="xz",
        min_overlap=0.005,
        name="spout shank stays seated in the body",
    )

    # --- Aerator at spout outlet ---
    aerator_aabb = ctx.part_world_aabb(aerator)
    ctx.check(
        "aerator sits at the spout outlet mouth (low and forward)",
        aerator_aabb is not None
        and aerator_aabb[1][0] > 0.040
        and aerator_aabb[0][2] < 0.020,
        details=f"aerator aabb={aerator_aabb}",
    )

    # --- Aerator hinge articulation ---
    af_limits = aerator_flip.motion_limits
    ctx.check(
        "aerator flip limits are 0 to ~90 degrees",
        af_limits is not None
        and af_limits.lower is not None
        and af_limits.upper is not None
        and abs(af_limits.lower) < 1e-9
        and abs(af_limits.upper - AERATOR_OPEN) < 1e-6,
        details=f"limits={af_limits}",
    )

    # Decisive pose: aerator flips open downward (disc swings below spout).
    disc_rest_aabb = ctx.part_element_world_aabb(aerator, elem="aerator_disc")
    with ctx.pose({aerator_flip: AERATOR_OPEN}):
        disc_open_aabb = ctx.part_element_world_aabb(aerator, elem="aerator_disc")
    ctx.check(
        "aerator flips open (disc bottom drops when hinge opens to 90 deg)",
        disc_rest_aabb is not None
        and disc_open_aabb is not None
        and disc_open_aabb[0][2] < disc_rest_aabb[0][2] - 0.005,
        details=f"rest_aabb={disc_rest_aabb}, open_aabb={disc_open_aabb}",
    )

    # --- Stem/cap stack ---
    ctx.expect_overlap(
        stem,
        body,
        axes="z",
        elem_a="stem_shaft",
        elem_b="body_neck",
        min_overlap=0.008,
        name="valve stem retained inside the neck bore",
    )
    ctx.expect_overlap(
        cap,
        stem,
        axes="z",
        elem_a="cap_shell",
        elem_b="stem_shaft",
        min_overlap=0.002,
        name="cap boss retains the stem top",
    )
    ctx.expect_within(
        stem,
        cap,
        axes="xy",
        inner_elem="stem_shaft",
        margin=0.001,
        name="stem stays centered under the push cap",
    )
    ctx.expect_gap(
        cap,
        body,
        axis="z",
        min_gap=0.002,
        max_gap=0.012,
        name="push cap hovers just above the neck (carried by the stem)",
    )
    cap_aabb = ctx.part_world_aabb(cap)
    ctx.check(
        "push cap is wider than the stepped-in neck (flared button)",
        cap_aabb is not None
        and neck_aabb is not None
        and (cap_aabb[1][1] - cap_aabb[0][1]) > (neck_aabb[1][1] - neck_aabb[0][1]) + 0.010,
        details=f"cap aabb={cap_aabb}, neck aabb={neck_aabb}",
    )
    ctx.check(
        "overall tap height is about 0.13 m",
        cap_aabb is not None and 0.125 <= cap_aabb[1][2] <= 0.140,
        details=f"cap aabb={cap_aabb}",
    )

    # --- Press and turn limits ---
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

    # --- Decisive poses: press + turn ---
    rest_pos = ctx.part_world_position(cap)
    with ctx.pose({press: PRESS_TRAVEL}):
        pressed_pos = ctx.part_world_position(cap)
    ctx.check(
        "pressing the cap moves it down along the tilted body axis",
        rest_pos is not None
        and pressed_pos is not None
        and 0.006 <= (rest_pos[2] - pressed_pos[2]) <= 0.0095
        and (pressed_pos[0] - rest_pos[0]) > 0.0003,
        details=f"rest={rest_pos}, pressed={pressed_pos}",
    )

    dot_rest = ctx.part_element_world_aabb(cap, elem="temp_indicator_dot")
    with ctx.pose({turn: TURN_LIMIT}):
        dot_hot = ctx.part_element_world_aabb(cap, elem="temp_indicator_dot")
    ctx.check(
        "turning the cap +60 deg swings the index mark around the cap axis",
        dot_rest is not None
        and dot_hot is not None
        and abs((dot_rest[0][1] + dot_rest[1][1]) / 2.0) < 0.004
        and (dot_hot[0][1] + dot_hot[1][1]) / 2.0 > 0.018,
        details=f"rest={dot_rest}, turned={dot_hot}",
    )

    # --- At least one non-fixed joint exists ---
    non_fixed = [
        j for j in object_model.articulations
        if j.articulation_type != ArticulationType.FIXED
    ]
    ctx.check(
        "at least one non-fixed joint exists",
        len(non_fixed) >= 1,
        details=f"non-fixed joints: {[j.name for j in non_fixed]}",
    )

    return ctx.report()


object_model = build_object_model()
