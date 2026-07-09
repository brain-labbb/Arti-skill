from __future__ import annotations

# Rounded retro chrome pop-up toaster (domed barrel shell).
#
# Coordinate convention:
#   +Z is up, the body's long axis runs along X, and the brushed-silver control
#   panel is the +X end face. Looking at the panel (looking along -X), the
#   viewer's right is +Y, the viewer's left is -Y.
#
# Body silhouette change vs parent:
#   The squared modern box is replaced by a CadQuery-lofted dome barrel shell.
#   Three slot2D cross-sections (base → barrel → dome taper) are lofted into
#   a smooth curved body, then boolean-cut for cavity, bread slots, recess,
#   lever slot, button holes, and dial shaft hole. The shell uses polished
#   chrome material instead of matte gray plastic.
#
# Structure:
#   - body (root): lofted retro chrome dome shell, recessed rim plate,
#     brushed-silver control panel, four dark plastic feet, dial markings,
#     brand strip.
#   - carriage_lever: PRISMATIC, q in [0, 0.07] moving DOWN (axis (0,0,-1)).
#   - browning_dial: REVOLUTE about +X, ~270 degrees.
#   - cancel/frozen/bagel buttons: PRISMATIC 0.003 m press into the panel.

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---------------------------------------------------------------------------
# Absolute dimensions (meters)
# ---------------------------------------------------------------------------

BODY_L = 0.280          # along X
BODY_W = 0.160          # along Y
BODY_H = 0.190          # overall height including feet
FOOT_H = 0.014
FOOT_R = 0.011
SHELL_H = BODY_H - FOOT_H          # 0.176
SHELL_TOP = BODY_H

# Retro dome loft cross-sections (slot2D: length along X, diameter along Y)
# Bottom: nearly full body width (base)
# Middle: barrel (slightly wider than nominal)
# Top: dome taper (narrower in Y for the retro curved silhouette)
LOFT_BOTTOM_L = BODY_L * 0.99
LOFT_BOTTOM_W = BODY_W * 0.97
LOFT_MIDDLE_L = BODY_L * 1.01
LOFT_MIDDLE_W = BODY_W * 1.02
LOFT_TOP_L = BODY_L * 0.97
LOFT_TOP_W = BODY_W * 0.78

# Internal toasting cavity (shortened to stay below dome taper zone)
CAV_X0, CAV_X1 = -0.120, 0.126
CAV_Y = 0.052
CAV_Z0, CAV_Z1 = 0.060, 0.160

# Two bread slots cut through the top
SLOT_XC = -0.010
SLOT_L = 0.200
SLOT_W = 0.030
SLOT_YC = 0.034
SLOT_CUT_Z0 = 0.145

# Recessed lighter-gray rim plate around both slots
RECESS_L = 0.200
RECESS_W = 0.108
RECESS_Z0 = 0.175
RIM_T = 0.0048

# Front control panel plate (+X face)
PANEL_X0 = 0.1395
PANEL_T = 0.0045
PANEL_X1 = PANEL_X0 + PANEL_T
PANEL_W = 0.116
PANEL_Z0, PANEL_Z1 = 0.032, 0.152
PANEL_ZC = (PANEL_Z0 + PANEL_Z1) / 2.0

# Carriage lever
LEVER_TRAVEL = 0.070
LEVER_YC = 0.005
LEVER_REST_Z = 0.148
LEVER_SLOT_W = 0.012
LEVER_SLOT_Z0, LEVER_SLOT_Z1 = 0.070, 0.155

# Buttons (CANCEL / FROZEN / BAGEL), stacked on the viewer's left (-Y)
BTN_Y = -0.036
BTN_Z = (0.112, 0.086, 0.060)
BTN_CAP_R = 0.0065
BTN_CAP_L = 0.0065
BTN_STEM_R = 0.0045
BTN_HOLE_R = 0.0060
BTN_TRAVEL = 0.003

# Browning dial (1-4), bottom-right of panel
DIAL_Y = 0.030
DIAL_Z = 0.054
DIAL_D = 0.030
DIAL_H = 0.014
DIAL_SHAFT_R = 0.004
DIAL_HOLE_R = 0.0047
DIAL_RANGE = math.radians(270.0)


def _box(x0: float, x1: float, y0: float, y1: float, z0: float, z1: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(x1 - x0, y1 - y0, z1 - z0)
        .translate(((x0 + x1) / 2.0, (y0 + y1) / 2.0, (z0 + z1) / 2.0))
    )


def _xcyl(x0: float, x1: float, y: float, z: float, r: float) -> cq.Workplane:
    """Cylinder along +X from x0 to x1 at (y, z)."""
    return (
        cq.Workplane("YZ")
        .circle(r)
        .extrude(x1 - x0)
        .translate((x0, y, z))
    )


def _shell_shape() -> cq.Workplane:
    """Rounded retro chrome dome shell via CadQuery 3-section loft."""
    mid_offset = SHELL_H * 0.48
    top_offset = SHELL_H * 0.52

    # Loft through three slot2D cross-sections for barrel-dome silhouette
    s = (
        cq.Workplane("XY", origin=(0.0, 0.0, FOOT_H))
        .slot2D(LOFT_BOTTOM_L, LOFT_BOTTOM_W)
        .workplane(offset=mid_offset)
        .slot2D(LOFT_MIDDLE_L, LOFT_MIDDLE_W)
        .workplane(offset=top_offset)
        .slot2D(LOFT_TOP_L, LOFT_TOP_W)
        .loft()
    )

    # Smooth the top dome edge and base edge
    try:
        s = s.edges(">Z").fillet(0.016)
    except Exception:
        pass
    try:
        s = s.edges("<Z").fillet(0.005)
    except Exception:
        pass

    # Internal toasting cavity
    s = s.cut(_box(CAV_X0, CAV_X1, -CAV_Y, CAV_Y, CAV_Z0, CAV_Z1))

    # Two bread slot openings through the top
    for yc in (SLOT_YC, -SLOT_YC):
        s = s.cut(
            _box(
                SLOT_XC - SLOT_L / 2.0,
                SLOT_XC + SLOT_L / 2.0,
                yc - SLOT_W / 2.0,
                yc + SLOT_W / 2.0,
                SLOT_CUT_Z0,
                SHELL_TOP + 0.01,
            )
        )

    # Shallow recess for the lighter-gray rim plate
    s = s.cut(
        _box(
            SLOT_XC - RECESS_L / 2.0,
            SLOT_XC + RECESS_L / 2.0,
            -RECESS_W / 2.0,
            RECESS_W / 2.0,
            RECESS_Z0,
            SHELL_TOP + 0.01,
        )
    )

    # Vertical lever slot through the front wall (extended inward for dome taper)
    s = s.cut(
        _box(
            0.098,
            0.152,
            LEVER_YC - LEVER_SLOT_W / 2.0,
            LEVER_YC + LEVER_SLOT_W / 2.0,
            LEVER_SLOT_Z0,
            LEVER_SLOT_Z1,
        )
    )

    # Button stem holes through the front wall
    for z in BTN_Z:
        s = s.cut(_xcyl(0.118, 0.152, BTN_Y, z, BTN_HOLE_R))

    # Dial shaft hole
    s = s.cut(_xcyl(0.118, 0.152, DIAL_Y, DIAL_Z, DIAL_HOLE_R))

    return s


def _rim_plate_shape() -> cq.Workplane:
    p = (
        cq.Workplane("XY", origin=(SLOT_XC, 0.0, RECESS_Z0 - 0.0002))
        .box(RECESS_L - 0.001, RECESS_W - 0.002, RIM_T, centered=(True, True, False))
    )
    p = p.edges("|Z").fillet(0.006)
    for yc in (SLOT_YC, -SLOT_YC):
        p = p.cut(
            _box(
                SLOT_XC - SLOT_L / 2.0 - 0.001,
                SLOT_XC + SLOT_L / 2.0 + 0.001,
                yc - SLOT_W / 2.0 - 0.00075,
                yc + SLOT_W / 2.0 + 0.00075,
                RECESS_Z0 - 0.01,
                SHELL_TOP + 0.01,
            )
        )
    return p


def _panel_shape() -> cq.Workplane:
    p = (
        cq.Workplane("YZ", origin=(PANEL_X0, 0.0, PANEL_ZC))
        .rect(PANEL_W, PANEL_Z1 - PANEL_Z0)
        .extrude(PANEL_T)
    )
    p = p.edges("|X").fillet(0.010)
    # Vertical lever slot
    p = p.cut(
        _box(
            PANEL_X0 - 0.005,
            PANEL_X1 + 0.005,
            LEVER_YC - LEVER_SLOT_W / 2.0,
            LEVER_YC + LEVER_SLOT_W / 2.0,
            LEVER_SLOT_Z0,
            LEVER_SLOT_Z1,
        )
    )
    # Button holes
    for z in BTN_Z:
        p = p.cut(_xcyl(PANEL_X0 - 0.005, PANEL_X1 + 0.005, BTN_Y, z, BTN_HOLE_R))
    # Dial shaft hole
    p = p.cut(_xcyl(PANEL_X0 - 0.005, PANEL_X1 + 0.005, DIAL_Y, DIAL_Z, DIAL_HOLE_R))
    return p


def _lever_knob_shape() -> cq.Workplane:
    k = cq.Workplane("XY").box(0.014, 0.024, 0.016)
    k = k.edges().fillet(0.003)
    return k


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retro_chrome_toaster")

    # Retro chrome body material
    model.material("chrome_body", rgba=(0.78, 0.80, 0.85, 1.0))
    model.material("rim_gray", rgba=(0.63, 0.64, 0.65, 1.0))
    model.material("brushed_silver", rgba=(0.78, 0.79, 0.81, 1.0))
    model.material("dark_plastic", rgba=(0.11, 0.11, 0.12, 1.0))
    model.material("knob_gray", rgba=(0.48, 0.49, 0.50, 1.0))
    model.material("dial_dark_gray", rgba=(0.24, 0.25, 0.26, 1.0))
    model.material("button_silver", rgba=(0.72, 0.73, 0.75, 1.0))
    model.material("carriage_metal", rgba=(0.56, 0.57, 0.59, 1.0))
    model.material("marking_dark", rgba=(0.18, 0.18, 0.19, 1.0))

    # ---------------- body (root) ----------------
    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_shell_shape(), "shell"),
        material="chrome_body",
        name="shell",
    )
    body.visual(
        mesh_from_cadquery(_rim_plate_shape(), "slot_rim_plate"),
        material="rim_gray",
        name="slot_rim_plate",
    )
    body.visual(
        mesh_from_cadquery(_panel_shape(), "control_panel"),
        material="brushed_silver",
        name="control_panel",
    )
    # Four dark plastic feet (embedded 0.002 into the shell underside)
    for i, (fx, fy) in enumerate(
        [(-0.105, -0.052), (-0.105, 0.052), (0.105, -0.052), (0.105, 0.052)]
    ):
        body.visual(
            Cylinder(radius=FOOT_R, length=FOOT_H + 0.002),
            origin=Origin(xyz=(fx, fy, (FOOT_H + 0.002) / 2.0)),
            material="dark_plastic",
            name=f"foot_{i}",
        )
    # Browning level markings 1-4 around the dial (raised ticks on the panel)
    tick_r = 0.0205
    for i, ang_deg in enumerate((235.0, 180.0, 125.0, 70.0)):
        t = math.radians(ang_deg)
        body.visual(
            Box((0.0018, 0.0024, 0.0024)),
            origin=Origin(
                xyz=(
                    PANEL_X1 + 0.0006,
                    DIAL_Y + tick_r * math.cos(t),
                    DIAL_Z + tick_r * math.sin(t),
                )
            ),
            material="marking_dark",
            name=f"dial_mark_{i + 1}",
        )
    # Brand strip near the top of the panel
    body.visual(
        Box((0.0014, 0.044, 0.0065)),
        origin=Origin(xyz=(PANEL_X1 + 0.0004, 0.000, 0.148)),
        material="marking_dark",
        name="brand_strip",
    )

    # ---------------- carriage lever (prismatic, slides down) ----------------
    carriage = model.part("carriage_lever")
    carriage.visual(
        mesh_from_cadquery(_lever_knob_shape(), "lever_knob"),
        origin=Origin(xyz=(0.0188, 0.0, 0.0)),
        material="knob_gray",
        name="lever_knob",
    )
    carriage.visual(
        Box((0.030, 0.008, 0.010)),
        origin=Origin(xyz=(-0.001, 0.0, 0.0)),
        material="carriage_metal",
        name="lever_stem",
    )
    carriage.visual(
        Box((0.012, 0.092, 0.012)),
        origin=Origin(xyz=(-0.020, -LEVER_YC, 0.0)),
        material="carriage_metal",
        name="carriage_crossbar",
    )
    for tag, yc in (("right", SLOT_YC), ("left", -SLOT_YC)):
        carriage.visual(
            Box((0.230, 0.024, 0.005)),
            origin=Origin(xyz=(-0.127, yc - LEVER_YC, 0.0)),
            material="carriage_metal",
            name=f"bread_shelf_{tag}",
        )

    model.articulation(
        "body_to_carriage_lever",
        ArticulationType.PRISMATIC,
        parent=body,
        child=carriage,
        origin=Origin(xyz=(0.132, LEVER_YC, LEVER_REST_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=15.0, velocity=0.3, lower=0.0, upper=LEVER_TRAVEL),
    )

    # ---------------- browning dial (revolute about +X) ----------------
    dial_geo = KnobGeometry(
        DIAL_D,
        DIAL_H,
        body_style="cylindrical",
        edge_radius=0.0015,
        grip=KnobGrip(style="ribbed", count=14, depth=0.0006, width=0.0016),
        indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0, depth=0.0006),
        center=False,
    )
    dial = model.part("browning_dial")
    dial.visual(
        mesh_from_geometry(dial_geo, "browning_dial_cap"),
        origin=Origin(xyz=(-0.0003, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="dial_dark_gray",
        name="dial_cap",
    )
    dial.visual(
        Cylinder(radius=DIAL_SHAFT_R, length=0.022),
        origin=Origin(xyz=(-0.008, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="dark_plastic",
        name="dial_shaft",
    )
    dial.visual(
        Cylinder(radius=0.0022, length=0.004),
        origin=Origin(xyz=(DIAL_H - 0.0023, 0.0, -0.0095), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="button_silver",
        name="dial_pointer_nub",
    )
    model.articulation(
        "body_to_browning_dial",
        ArticulationType.REVOLUTE,
        parent=body,
        child=dial,
        origin=Origin(xyz=(PANEL_X1, DIAL_Y, DIAL_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=DIAL_RANGE),
    )

    # ---------------- CANCEL / FROZEN / BAGEL push buttons ----------------
    for name, z in zip(("cancel_button", "frozen_button", "bagel_button"), BTN_Z):
        btn = model.part(name)
        btn.visual(
            Cylinder(radius=BTN_CAP_R, length=BTN_CAP_L),
            origin=Origin(xyz=(BTN_CAP_L / 2.0 - 0.0003, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material="button_silver",
            name=f"{name}_cap",
        )
        btn.visual(
            Cylinder(radius=BTN_STEM_R, length=0.021),
            origin=Origin(xyz=(-0.0095, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material="dark_plastic",
            name=f"{name}_stem",
        )
        model.articulation(
            f"body_to_{name}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=btn,
            origin=Origin(xyz=(PANEL_X1, BTN_Y, z)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=0.1, lower=0.0, upper=BTN_TRAVEL),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    carriage = object_model.get_part("carriage_lever")
    dial = object_model.get_part("browning_dial")
    buttons = [
        object_model.get_part(n) for n in ("cancel_button", "frozen_button", "bagel_button")
    ]

    lever_joint = object_model.get_articulation("body_to_carriage_lever")
    dial_joint = object_model.get_articulation("body_to_browning_dial")

    # ---- intentional seated/flush embeddings ----
    ctx.allow_overlap(
        carriage,
        body,
        elem_a="lever_knob",
        elem_b="control_panel",
        reason="The carriage lever knob rides flush on the panel surface.",
    )
    ctx.allow_overlap(
        dial,
        body,
        elem_a="dial_cap",
        elem_b="control_panel",
        reason="The dial cap mounting face seats flush against the panel.",
    )
    for btn in buttons:
        ctx.allow_overlap(
            btn,
            body,
            elem_a=f"{btn.name}_cap",
            elem_b="control_panel",
            reason="The button cap rim seats flush against the panel face.",
        )

    # ---- overall identity: scale and grounding ----
    aabb = ctx.part_world_aabb(body)
    ctx.check(
        "body is grounded at z=0",
        aabb is not None and abs(aabb[0][2]) < 0.002,
        details=f"body aabb={aabb}",
    )
    if aabb is not None:
        dx = aabb[1][0] - aabb[0][0]
        dy = aabb[1][1] - aabb[0][1]
        dz = aabb[1][2] - aabb[0][2]
        ctx.check(
            "toaster has compact two-slice proportions (~0.28 x 0.16 x 0.19 m)",
            abs(dx - 0.280) < 0.018 and abs(dy - 0.160) < 0.018 and abs(dz - 0.190) < 0.012,
            details=f"dims=({dx:.4f},{dy:.4f},{dz:.4f})",
        )

    # ---- DOME SILHOUETTE: retro curved body taper ----
    rim = ctx.part_element_world_aabb(body, elem="slot_rim_plate")
    ctx.check(
        "rim plate recessed below dome shell top",
        rim is not None and rim[1][2] < SHELL_TOP - 0.002,
        details=f"rim aabb={rim}",
    )
    if rim is not None and aabb is not None:
        rim_dy = rim[1][1] - rim[0][1]
        body_dy = aabb[1][1] - aabb[0][1]
        ctx.check(
            "dome silhouette: rim plate at dome top is narrower than body barrel",
            rim_dy < body_dy * 0.82,
            details=f"rim_dy={rim_dy:.4f}, body_dy={body_dy:.4f}",
        )

    # ---- control panel on the +X front face ----
    panel = ctx.part_element_world_aabb(body, elem="control_panel")
    ctx.check(
        "brushed-silver control panel is on the +X front face",
        panel is not None and panel[1][0] > 0.143 and panel[0][0] > 0.135,
        details=f"panel aabb={panel}",
    )

    # ---- carriage lever: knob seats on panel, shelves under the slots ----
    ctx.expect_contact(
        carriage,
        body,
        elem_a="lever_knob",
        elem_b="control_panel",
        name="lever knob rides on the control panel",
    )
    for tag, yc in (("right", SLOT_YC), ("left", -SLOT_YC)):
        shelf = ctx.part_element_world_aabb(carriage, elem=f"bread_shelf_{tag}")
        ctx.check(
            f"bread shelf {tag} sits centered under its top slot at rest",
            shelf is not None
            and abs((shelf[0][1] + shelf[1][1]) / 2.0 - yc) < 0.003
            and shelf[0][2] > CAV_Z0
            and shelf[1][2] < CAV_Z1 + 0.025,
            details=f"shelf {tag} aabb={shelf}",
        )
    ctx.expect_within(
        carriage,
        body,
        axes="xy",
        inner_elem="bread_shelf_right",
        margin=0.0,
        name="carriage shelf stays inside the body footprint",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({lever_joint: LEVER_TRAVEL}):
        pressed_pos = ctx.part_world_position(carriage)
        shelf_dn = ctx.part_element_world_aabb(carriage, elem="bread_shelf_right")
        ctx.check(
            "pressed carriage stays above the cavity floor",
            shelf_dn is not None and shelf_dn[0][2] > CAV_Z0 + 0.005,
            details=f"pressed shelf aabb={shelf_dn}",
        )
    ctx.check(
        "carriage lever travels 0.07 m straight DOWN",
        rest_pos is not None
        and pressed_pos is not None
        and abs((rest_pos[2] - pressed_pos[2]) - LEVER_TRAVEL) < 1e-6
        and abs(rest_pos[0] - pressed_pos[0]) < 1e-9
        and abs(rest_pos[1] - pressed_pos[1]) < 1e-9,
        details=f"rest={rest_pos}, pressed={pressed_pos}",
    )
    ctx.check(
        "carriage lever joint is prismatic with ~0.07 m downward travel",
        lever_joint.articulation_type == ArticulationType.PRISMATIC
        and abs(lever_joint.motion_limits.upper - 0.070) < 1e-9
        and lever_joint.axis[2] < 0.0,
        details=f"axis={lever_joint.axis}, limits=({lever_joint.motion_limits.lower}, {lever_joint.motion_limits.upper})",
    )

    # ---- browning dial: revolute about panel normal, ~270 deg, off-axis nub ----
    ctx.check(
        "browning dial rotates ~270 deg about the horizontal +X panel normal",
        dial_joint.articulation_type == ArticulationType.REVOLUTE
        and abs(dial_joint.axis[0]) > 0.99
        and abs(dial_joint.motion_limits.upper - dial_joint.motion_limits.lower - DIAL_RANGE)
        < 1e-6,
        details=f"axis={dial_joint.axis}, limits=({dial_joint.motion_limits.lower}, {dial_joint.motion_limits.upper})",
    )
    nub0 = ctx.part_element_world_aabb(dial, elem="dial_pointer_nub")
    with ctx.pose({dial_joint: math.pi}):
        nub1 = ctx.part_element_world_aabb(dial, elem="dial_pointer_nub")
    ctx.check(
        "off-axis pointer nub sweeps with dial rotation (proves continuous rotation)",
        nub0 is not None
        and nub1 is not None
        and ((nub1[0][2] + nub1[1][2]) - (nub0[0][2] + nub0[1][2])) / 2.0 > 0.015,
        details=f"nub rest={nub0}, rotated={nub1}",
    )
    ctx.expect_contact(
        dial,
        body,
        elem_a="dial_cap",
        elem_b="control_panel",
        name="dial cap seats on the control panel",
    )

    # ---- three push buttons: placement and press travel ----
    ctx.expect_origin_gap(
        buttons[0], buttons[1], axis="z", min_gap=0.020, name="CANCEL sits above FROZEN"
    )
    ctx.expect_origin_gap(
        buttons[1], buttons[2], axis="z", min_gap=0.020, name="FROZEN sits above BAGEL"
    )
    for btn in buttons:
        joint = object_model.get_articulation(f"body_to_{btn.name}")
        ctx.check(
            f"{btn.name} is a 3 mm prismatic press into the panel",
            joint.articulation_type == ArticulationType.PRISMATIC
            and abs(joint.motion_limits.upper - BTN_TRAVEL) < 1e-9
            and joint.axis[0] < 0.0,
            details=f"axis={joint.axis}, upper={joint.motion_limits.upper}",
        )
        p0 = ctx.part_world_position(btn)
        with ctx.pose({joint: BTN_TRAVEL}):
            p1 = ctx.part_world_position(btn)
        ctx.check(
            f"{btn.name} presses inward along -X",
            p0 is not None and p1 is not None and (p0[0] - p1[0]) > BTN_TRAVEL - 1e-6,
            details=f"rest={p0}, pressed={p1}",
        )
        pos = ctx.part_world_position(btn)
        ctx.check(
            f"{btn.name} sits left of the lever slot",
            pos is not None and pos[1] < -0.02,
            details=f"pos={pos}",
        )

    # dial sits bottom-right of the panel, below and right of the buttons
    dial_pos = ctx.part_world_position(dial)
    ctx.check(
        "browning dial sits at the bottom-right of the panel",
        dial_pos is not None and dial_pos[1] > 0.015 and dial_pos[2] < 0.075,
        details=f"dial pos={dial_pos}",
    )

    return ctx.report()


object_model = build_object_model()
