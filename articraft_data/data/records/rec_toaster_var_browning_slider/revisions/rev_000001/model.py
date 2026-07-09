from __future__ import annotations

# Compact two-slice pop-up toaster (AmazonBasics style) — SLIDER variant.
#
# Coordinate convention:
#   +Z is up, the body's long axis runs along X, and the brushed-silver control
#   panel is the +X end face. Looking at the panel (looking along -X), the
#   viewer's right is +Y, the viewer's left is -Y.
#
# Structure:
#   - body (root): rounded hollow shell (CadQuery, slot cavity + two top slot
#     openings + recessed lighter-gray rim plate), four dark plastic feet,
#     brushed-silver front panel plate with lever slot / button holes / slider
#     track slot, slider level marks 1-4 and a brand strip.
#   - carriage_lever: gray knob riding the panel slot + stem through the front
#     wall + internal crossbar + two bread-carriage shelves visible through
#     the top slots. PRISMATIC, q in [0, 0.07] moving DOWN (axis (0,0,-1)).
#   - browning_slider: dark-gray tab on a vertical track on the panel,
#     PRISMATIC along +Z (upward = high browning), travel ~0.044 m, with a
#     stem through the track slot and an internal carriage plate.
#   - cancel/frozen/bagel buttons: small silver caps with stems through panel
#     holes, PRISMATIC pressing 0.003 m into the panel (axis (-1,0,0)).

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Absolute dimensions (meters)
# ---------------------------------------------------------------------------

BODY_L = 0.280          # along X
BODY_W = 0.160          # along Y
BODY_H = 0.190          # overall height including feet
FOOT_H = 0.014
FOOT_R = 0.011
SHELL_H = BODY_H - FOOT_H          # 0.176, shell spans z FOOT_H..BODY_H
SHELL_TOP = BODY_H

# Internal toasting cavity (hidden except through the slots)
CAV_X0, CAV_X1 = -0.120, 0.126
CAV_Y = 0.052                       # half width
CAV_Z0, CAV_Z1 = 0.060, 0.168

# Two bread slots cut through the top
SLOT_XC = -0.010
SLOT_L = 0.200                      # x extent of each slot opening
SLOT_W = 0.030                      # y width of each slot opening
SLOT_YC = 0.034                     # +/- y centers
SLOT_CUT_Z0 = 0.158                 # cut from here up through the top

# Recessed lighter-gray rim plate around both slots
RECESS_L = 0.220
RECESS_W = 0.114
RECESS_Z0 = 0.185                   # recess floor
RIM_T = 0.0048

# Front control panel plate (+X face)
SHELL_FACE_X = BODY_L / 2.0         # 0.140
PANEL_X0 = 0.1395                   # embeds 0.0005 into the shell face
PANEL_T = 0.0045
PANEL_X1 = PANEL_X0 + PANEL_T       # 0.1440 outer face
PANEL_W = 0.116
PANEL_Z0, PANEL_Z1 = 0.032, 0.172
PANEL_ZC = (PANEL_Z0 + PANEL_Z1) / 2.0

# Carriage lever
LEVER_TRAVEL = 0.070
LEVER_YC = 0.005                    # slot slightly right of panel center
LEVER_REST_Z = 0.150                # stem/knob center at rest (up)
LEVER_SLOT_W = 0.012                # panel/wall slot width (y)
LEVER_SLOT_Z0, LEVER_SLOT_Z1 = 0.070, 0.160

# Buttons (CANCEL / FROZEN / BAGEL), stacked on the viewer's left (-Y)
BTN_Y = -0.036
BTN_Z = (0.112, 0.086, 0.060)       # cancel, frozen, bagel (top to bottom)
BTN_CAP_R = 0.0065
BTN_CAP_L = 0.0065
BTN_STEM_R = 0.0045
BTN_HOLE_R = 0.0060
BTN_TRAVEL = 0.003

# Browning slider (1-4), bottom-right of panel, vertical track
SLIDER_Y = 0.030                    # y center of track on panel
SLIDER_REST_Z = 0.052               # tab center at rest (low position)
SLIDER_TRAVEL = 0.044               # total upward travel
SLIDER_TAB_W = 0.018                # tab width (Y)
SLIDER_TAB_H = 0.012                # tab height (Z)
SLIDER_TAB_D = 0.008                # tab depth/protrusion from panel (X)
TRACK_SLOT_W = 0.006                # track slot width (Y)
# Track slot Z extent: covers full tab travel plus half tab height each end
TRACK_SLOT_Z0 = SLIDER_REST_Z - SLIDER_TAB_H / 2.0 - 0.001
TRACK_SLOT_Z1 = SLIDER_REST_Z + SLIDER_TRAVEL + SLIDER_TAB_H / 2.0 + 0.001
SLIDER_STEM_W = 0.004               # stem width (Y) through the slot
SLIDER_STEM_D = 0.030               # stem depth (X) through wall


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
    s = (
        cq.Workplane("XY", origin=(0.0, 0.0, FOOT_H))
        .box(BODY_L, BODY_W, SHELL_H, centered=(True, True, False))
    )
    s = s.edges("|Z").fillet(0.030)
    s = s.edges(">Z").fillet(0.016)
    s = s.edges("<Z").fillet(0.005)

    # internal toasting cavity
    s = s.cut(_box(CAV_X0, CAV_X1, -CAV_Y, CAV_Y, CAV_Z0, CAV_Z1))
    # two bread slot openings through the top
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
    # shallow recess for the lighter-gray rim plate
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
    # vertical lever slot through the front wall
    s = s.cut(
        _box(
            0.118,
            0.152,
            LEVER_YC - LEVER_SLOT_W / 2.0,
            LEVER_YC + LEVER_SLOT_W / 2.0,
            LEVER_SLOT_Z0,
            LEVER_SLOT_Z1,
        )
    )
    # button stem holes through the front wall
    for z in BTN_Z:
        s = s.cut(_xcyl(0.118, 0.152, BTN_Y, z, BTN_HOLE_R))
    # vertical slider track slot through the front wall
    s = s.cut(
        _box(
            0.118,
            0.152,
            SLIDER_Y - TRACK_SLOT_W / 2.0,
            SLIDER_Y + TRACK_SLOT_W / 2.0,
            TRACK_SLOT_Z0,
            TRACK_SLOT_Z1,
        )
    )
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
    # vertical lever slot
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
    # button holes
    for z in BTN_Z:
        p = p.cut(_xcyl(PANEL_X0 - 0.005, PANEL_X1 + 0.005, BTN_Y, z, BTN_HOLE_R))
    # vertical slider track slot
    p = p.cut(
        _box(
            PANEL_X0 - 0.005,
            PANEL_X1 + 0.005,
            SLIDER_Y - TRACK_SLOT_W / 2.0,
            SLIDER_Y + TRACK_SLOT_W / 2.0,
            TRACK_SLOT_Z0,
            TRACK_SLOT_Z1,
        )
    )
    return p


def _lever_knob_shape() -> cq.Workplane:
    k = cq.Workplane("XY").box(0.014, 0.024, 0.016)
    k = k.edges().fillet(0.003)
    return k


def _slider_tab_shape() -> cq.Workplane:
    """Rounded rectangular slider tab protruding from the panel face."""
    tab = (
        cq.Workplane("XY")
        .box(SLIDER_TAB_D, SLIDER_TAB_W, SLIDER_TAB_H)
    )
    tab = tab.edges("|Z").fillet(0.002)
    tab = tab.edges(">Z").fillet(0.0015)
    tab = tab.edges("<Z").fillet(0.0015)
    return tab


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_slice_pop_up_toaster")

    model.material("body_gray", rgba=(0.40, 0.40, 0.41, 1.0))
    model.material("rim_gray", rgba=(0.63, 0.64, 0.65, 1.0))
    model.material("brushed_silver", rgba=(0.78, 0.79, 0.81, 1.0))
    model.material("dark_plastic", rgba=(0.11, 0.11, 0.12, 1.0))
    model.material("knob_gray", rgba=(0.48, 0.49, 0.50, 1.0))
    model.material("slider_dark", rgba=(0.22, 0.22, 0.24, 1.0))
    model.material("button_silver", rgba=(0.72, 0.73, 0.75, 1.0))
    model.material("carriage_metal", rgba=(0.56, 0.57, 0.59, 1.0))
    model.material("marking_dark", rgba=(0.18, 0.18, 0.19, 1.0))

    # ---------------- body (root) ----------------
    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_shell_shape(), "shell"),
        material="body_gray",
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
    # four dark plastic feet (embedded 0.002 into the shell underside)
    for i, (fx, fy) in enumerate(
        [(-0.105, -0.052), (-0.105, 0.052), (0.105, -0.052), (0.105, 0.052)]
    ):
        body.visual(
            Cylinder(radius=FOOT_R, length=FOOT_H + 0.002),
            origin=Origin(xyz=(fx, fy, (FOOT_H + 0.002) / 2.0)),
            material="dark_plastic",
            name=f"foot_{i}",
        )
    # browning level marks 1-4 as horizontal ticks beside the slider track
    mark_y = SLIDER_Y + 0.014       # ticks to the right of the track
    for i in range(4):
        mark_z = SLIDER_REST_Z + (i / 3.0) * SLIDER_TRAVEL
        body.visual(
            Box((0.0018, 0.008, 0.002)),
            origin=Origin(xyz=(PANEL_X1 + 0.0006, mark_y, mark_z)),
            material="marking_dark",
            name=f"slider_mark_{i + 1}",
        )
    # brand strip near the top of the panel
    body.visual(
        Box((0.0014, 0.044, 0.0065)),
        origin=Origin(xyz=(PANEL_X1 + 0.0004, 0.000, 0.1660)),
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

    # ---------------- browning slider (prismatic, slides up = high) ----------------
    slider = model.part("browning_slider")
    # Tab: protrudes from the panel face along +X
    slider.visual(
        mesh_from_cadquery(_slider_tab_shape(), "slider_tab"),
        origin=Origin(xyz=(SLIDER_TAB_D / 2.0 - 0.0003, 0.0, 0.0)),
        material="slider_dark",
        name="slider_tab",
    )
    # Stem: thin bar going through the track slot into the body
    slider.visual(
        Box((SLIDER_STEM_D, SLIDER_STEM_W, SLIDER_TAB_H - 0.002)),
        origin=Origin(xyz=(-SLIDER_STEM_D / 2.0 + 0.002, 0.0, 0.0)),
        material="dark_plastic",
        name="slider_stem",
    )
    # Internal carriage plate behind the front wall (connects to mechanism)
    slider.visual(
        Box((0.004, 0.014, 0.016)),
        origin=Origin(xyz=(-SLIDER_STEM_D + 0.002, 0.0, 0.0)),
        material="carriage_metal",
        name="slider_carriage_plate",
    )

    model.articulation(
        "body_to_browning_slider",
        ArticulationType.PRISMATIC,
        parent=body,
        child=slider,
        origin=Origin(xyz=(PANEL_X1, SLIDER_Y, SLIDER_REST_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.2, lower=0.0, upper=SLIDER_TRAVEL),
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
    slider = object_model.get_part("browning_slider")
    buttons = [
        object_model.get_part(n) for n in ("cancel_button", "frozen_button", "bagel_button")
    ]

    lever_joint = object_model.get_articulation("body_to_carriage_lever")
    slider_joint = object_model.get_articulation("body_to_browning_slider")

    # ---- intentional seated/flush embeddings (all tiny, <=0.0005) ----
    ctx.allow_overlap(
        carriage,
        body,
        elem_a="lever_knob",
        elem_b="control_panel",
        reason="The carriage lever knob intentionally rides flush on the panel surface (0.2 mm seat).",
    )
    ctx.allow_overlap(
        slider,
        body,
        elem_a="slider_tab",
        elem_b="control_panel",
        reason="The slider tab seats flush against the panel face (0.3 mm seat).",
    )
    for btn in buttons:
        ctx.allow_overlap(
            btn,
            body,
            elem_a=f"{btn.name}_cap",
            elem_b="control_panel",
            reason="The button cap rim seats flush against the panel face (0.3 mm seat).",
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
            abs(dx - 0.280) < 0.012 and abs(dy - 0.160) < 0.012 and abs(dz - 0.190) < 0.010,
            details=f"dims=({dx:.4f},{dy:.4f},{dz:.4f})",
        )

    # ---- bread slots and recessed rim ----
    rim = ctx.part_element_world_aabb(body, elem="slot_rim_plate")
    ctx.check(
        "lighter-gray rim plate sits recessed just below the shell top",
        rim is not None and rim[1][2] < SHELL_TOP - 0.0001 and rim[1][2] > SHELL_TOP - 0.0035,
        details=f"rim aabb={rim}",
    )

    # ---- control panel on the +X front face ----
    panel = ctx.part_element_world_aabb(body, elem="control_panel")
    ctx.check(
        "brushed-silver control panel is on the +X front face",
        panel is not None and panel[1][0] > 0.1435 and panel[0][0] > 0.135,
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
            and shelf[1][2] < CAV_Z1,
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

    # ---- browning slider: prismatic along +Z, tab moves upward for high ----
    ctx.check(
        "browning slider is prismatic along +Z (up = high)",
        slider_joint.articulation_type == ArticulationType.PRISMATIC
        and slider_joint.axis[2] > 0.99
        and abs(slider_joint.motion_limits.upper - SLIDER_TRAVEL) < 1e-9
        and slider_joint.motion_limits.lower == 0.0,
        details=f"axis={slider_joint.axis}, limits=({slider_joint.motion_limits.lower}, {slider_joint.motion_limits.upper})",
    )
    # Tab contacts the panel face at rest
    ctx.expect_contact(
        slider,
        body,
        elem_a="slider_tab",
        elem_b="control_panel",
        name="slider tab seats on the control panel",
    )
    # Slider tab moves upward when pushed to max
    slider_rest = ctx.part_world_position(slider)
    with ctx.pose({slider_joint: SLIDER_TRAVEL}):
        slider_high = ctx.part_world_position(slider)
    ctx.check(
        "browning slider travels upward by the full travel distance",
        slider_rest is not None
        and slider_high is not None
        and abs((slider_high[2] - slider_rest[2]) - SLIDER_TRAVEL) < 1e-6
        and abs(slider_high[0] - slider_rest[0]) < 1e-9
        and abs(slider_high[1] - slider_rest[1]) < 1e-9,
        details=f"rest={slider_rest}, high={slider_high}",
    )
    # Slider stays within the body footprint
    ctx.expect_within(
        slider,
        body,
        axes="y",
        inner_elem="slider_tab",
        margin=0.0,
        name="slider tab stays within body width",
    )
    # Slider sits at the bottom-right of the panel
    slider_pos = ctx.part_world_position(slider)
    ctx.check(
        "browning slider sits at the bottom-right of the panel",
        slider_pos is not None and slider_pos[1] > 0.015 and slider_pos[2] < 0.075,
        details=f"slider pos={slider_pos}",
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
        # buttons are on the viewer's left (-Y) half of the panel
        pos = ctx.part_world_position(btn)
        ctx.check(
            f"{btn.name} sits left of the lever slot",
            pos is not None and pos[1] < -0.02,
            details=f"pos={pos}",
        )

    return ctx.report()


object_model = build_object_model()
