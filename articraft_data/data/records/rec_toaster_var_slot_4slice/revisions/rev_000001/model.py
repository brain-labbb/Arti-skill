from __future__ import annotations

# Compact four-slice pop-up toaster (AmazonBasics style).
#
# Coordinate convention:
#   +Z is up, the body's long axis runs along X, and the brushed-silver control
#   panel is the +X end face. Looking at the panel (looking along -X), the
#   viewer's right is +Y, the viewer's left is -Y.
#
# Structure:
#   - body (root): rounded hollow shell (CadQuery, slot cavity + four top slot
#     openings + recessed lighter-gray rim plate), four dark plastic feet,
#     brushed-silver front panel plate with lever slot / button holes / dial
#     shaft hole, dial markings 1-4 and a brand strip.
#   - carriage_lever: gray knob riding the panel slot + stem through the front
#     wall + side rails + four bread-carriage shelves visible through the top
#     slots (loop-emitted). PRISMATIC, q in [0, 0.07] moving DOWN (axis (0,0,-1)).
#   - browning_dial: dark-gray knob (1-4) on the panel, REVOLUTE about +X
#     (horizontal, perpendicular to the panel), ~270 degrees of travel, with
#     an off-axis pointer nub.
#   - cancel/frozen/bagel buttons: small silver caps with stems through panel
#     holes, PRISMATIC pressing 0.003 m into the panel (axis (-1,0,0)).

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

NUM_SLOTS = 4                        # four-slice variant: 2 pairs × 2 slots

BODY_L = 0.396          # along X (extended from 0.280 for 2-slice)
BODY_W = 0.160          # along Y
BODY_H = 0.190          # overall height including feet
FOOT_H = 0.014
FOOT_R = 0.011
SHELL_H = BODY_H - FOOT_H          # 0.176, shell spans z FOOT_H..BODY_H
SHELL_TOP = BODY_H

# Internal toasting cavity (hidden except through the slots)
CAV_X0, CAV_X1 = -0.172, 0.178
CAV_Y = 0.052                       # half width
CAV_Z0, CAV_Z1 = 0.060, 0.168

# Four bread slots cut through the top (2 pairs end-to-end along X)
SLOT_L = 0.130                      # x extent of each slot opening
SLOT_W = 0.030                      # y width of each slot opening
SLOT_YC = 0.034                     # +/- y centers within each pair
SLOT_CUT_Z0 = 0.158                 # cut from here up through the top

# Slot pair X centers in world coords (rear pair far from panel, front pair near panel)
SLOT_PAIR_XC = (-0.060, 0.060)

# Build the 4 slot (xc, yc) positions via loop
SLOT_POSITIONS: list[tuple[float, float]] = []
for _px in SLOT_PAIR_XC:
    for _py in (SLOT_YC, -SLOT_YC):
        SLOT_POSITIONS.append((_px, _py))

# Recessed lighter-gray rim plate around all slots
RECESS_L = 0.330
RECESS_W = 0.114
RECESS_Z0 = 0.185                   # recess floor
RIM_T = 0.0048

# Front control panel plate (+X face)
SHELL_FACE_X = BODY_L / 2.0         # 0.198
PANEL_X0 = SHELL_FACE_X - 0.0005    # embeds 0.0005 into the shell face
PANEL_T = 0.0045
PANEL_X1 = PANEL_X0 + PANEL_T       # outer face
PANEL_W = 0.116
PANEL_Z0, PANEL_Z1 = 0.032, 0.172
PANEL_ZC = (PANEL_Z0 + PANEL_Z1) / 2.0

# Front wall cut range (for lever slot, button holes, dial hole)
WALL_CUT_X0 = SHELL_FACE_X - 0.022
WALL_CUT_X1 = SHELL_FACE_X + 0.012

# Carriage lever
LEVER_TRAVEL = 0.070
LEVER_YC = 0.005                    # slot slightly right of panel center
LEVER_REST_Z = 0.150                # stem/knob center at rest (up)
LEVER_SLOT_W = 0.012                # panel/wall slot width (y)
LEVER_SLOT_Z0, LEVER_SLOT_Z1 = 0.070, 0.160
CARRIAGE_ORIGIN_X = SHELL_FACE_X - 0.010   # carriage part origin X in world

# Buttons (CANCEL / FROZEN / BAGEL), stacked on the viewer's left (-Y)
BTN_Y = -0.036
BTN_Z = (0.112, 0.086, 0.060)       # cancel, frozen, bagel (top to bottom)
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
    s = (
        cq.Workplane("XY", origin=(0.0, 0.0, FOOT_H))
        .box(BODY_L, BODY_W, SHELL_H, centered=(True, True, False))
    )
    s = s.edges("|Z").fillet(0.030)
    s = s.edges(">Z").fillet(0.016)
    s = s.edges("<Z").fillet(0.005)

    # internal toasting cavity
    s = s.cut(_box(CAV_X0, CAV_X1, -CAV_Y, CAV_Y, CAV_Z0, CAV_Z1))
    # four bread slot openings through the top (loop-emitted)
    for i in range(NUM_SLOTS):
        xc, yc = SLOT_POSITIONS[i]
        s = s.cut(
            _box(
                xc - SLOT_L / 2.0,
                xc + SLOT_L / 2.0,
                yc - SLOT_W / 2.0,
                yc + SLOT_W / 2.0,
                SLOT_CUT_Z0,
                SHELL_TOP + 0.01,
            )
        )
    # shallow recess for the lighter-gray rim plate
    s = s.cut(
        _box(
            -RECESS_L / 2.0,
            RECESS_L / 2.0,
            -RECESS_W / 2.0,
            RECESS_W / 2.0,
            RECESS_Z0,
            SHELL_TOP + 0.01,
        )
    )
    # vertical lever slot through the front wall
    s = s.cut(
        _box(
            WALL_CUT_X0,
            WALL_CUT_X1,
            LEVER_YC - LEVER_SLOT_W / 2.0,
            LEVER_YC + LEVER_SLOT_W / 2.0,
            LEVER_SLOT_Z0,
            LEVER_SLOT_Z1,
        )
    )
    # button stem holes through the front wall
    for z in BTN_Z:
        s = s.cut(_xcyl(WALL_CUT_X0, WALL_CUT_X1, BTN_Y, z, BTN_HOLE_R))
    # dial shaft hole
    s = s.cut(_xcyl(WALL_CUT_X0, WALL_CUT_X1, DIAL_Y, DIAL_Z, DIAL_HOLE_R))
    return s


def _rim_plate_shape() -> cq.Workplane:
    p = (
        cq.Workplane("XY", origin=(0.0, 0.0, RECESS_Z0 - 0.0002))
        .box(RECESS_L - 0.001, RECESS_W - 0.002, RIM_T, centered=(True, True, False))
    )
    p = p.edges("|Z").fillet(0.006)
    # cut four slot openings through the rim plate (loop-emitted)
    for i in range(NUM_SLOTS):
        xc, yc = SLOT_POSITIONS[i]
        p = p.cut(
            _box(
                xc - SLOT_L / 2.0 - 0.001,
                xc + SLOT_L / 2.0 + 0.001,
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
    # dial shaft hole
    p = p.cut(_xcyl(PANEL_X0 - 0.005, PANEL_X1 + 0.005, DIAL_Y, DIAL_Z, DIAL_HOLE_R))
    return p


def _lever_knob_shape() -> cq.Workplane:
    k = cq.Workplane("XY").box(0.014, 0.024, 0.016)
    k = k.edges().fillet(0.003)
    return k


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="four_slice_pop_up_toaster")

    model.material("body_gray", rgba=(0.40, 0.40, 0.41, 1.0))
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
    foot_x = BODY_L / 2.0 - 0.028
    for i, (fx, fy) in enumerate(
        [(-foot_x, -0.052), (-foot_x, 0.052), (foot_x, -0.052), (foot_x, 0.052)]
    ):
        body.visual(
            Cylinder(radius=FOOT_R, length=FOOT_H + 0.002),
            origin=Origin(xyz=(fx, fy, (FOOT_H + 0.002) / 2.0)),
            material="dark_plastic",
            name=f"foot_{i}",
        )
    # browning level markings 1-4 around the dial (raised ticks on the panel)
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
    # Y-spanning crossbar at the lever end
    carriage.visual(
        Box((0.012, 0.092, 0.012)),
        origin=Origin(xyz=(-0.020, -LEVER_YC, 0.0)),
        material="carriage_metal",
        name="carriage_crossbar",
    )
    # Two X-extending side rails: run from behind the rear shelf pair all the
    # way to the crossbar, providing the structural spine of the carriage.
    rail_rear_world_x = SLOT_PAIR_XC[0] - SLOT_L / 2.0 + 0.010   # just inside rear shelf
    rail_front_world_x = CARRIAGE_ORIGIN_X - 0.020                # meets crossbar
    rail_x_span = rail_front_world_x - rail_rear_world_x
    rail_center_world_x = (rail_rear_world_x + rail_front_world_x) / 2.0
    for side_tag, dy in (("right", SLOT_YC), ("left", -SLOT_YC)):
        carriage.visual(
            Box((rail_x_span, 0.008, 0.008)),
            origin=Origin(xyz=(
                rail_center_world_x - CARRIAGE_ORIGIN_X,
                dy - LEVER_YC,
                0.0,
            )),
            material="carriage_metal",
            name=f"side_rail_{side_tag}",
        )
    # Four bread shelves (loop-emitted), one per slot
    for i in range(NUM_SLOTS):
        xc, yc = SLOT_POSITIONS[i]
        carriage.visual(
            Box((SLOT_L - 0.010, 0.024, 0.005)),
            origin=Origin(xyz=(xc - CARRIAGE_ORIGIN_X, yc - LEVER_YC, 0.0)),
            material="carriage_metal",
            name=f"bread_shelf_{i}",
        )

    model.articulation(
        "body_to_carriage_lever",
        ArticulationType.PRISMATIC,
        parent=body,
        child=carriage,
        origin=Origin(xyz=(CARRIAGE_ORIGIN_X, LEVER_YC, LEVER_REST_Z)),
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
        # local +Z (knob axis) -> world +X; mounting face embeds 0.0003 in panel
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
    # off-axis pointer nub on the dial face (points down toward "1" at rest)
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

    # ---- intentional seated/flush embeddings (all tiny, <=0.0005) ----
    ctx.allow_overlap(
        carriage,
        body,
        elem_a="lever_knob",
        elem_b="control_panel",
        reason="The carriage lever knob intentionally rides flush on the panel surface (0.2 mm seat).",
    )
    ctx.allow_overlap(
        dial,
        body,
        elem_a="dial_cap",
        elem_b="control_panel",
        reason="The dial cap mounting face seats flush against the panel (0.3 mm seat).",
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
            "toaster has four-slice proportions (~0.396 x 0.16 x 0.19 m)",
            abs(dx - BODY_L) < 0.012 and abs(dy - BODY_W) < 0.012 and abs(dz - BODY_H) < 0.010,
            details=f"dims=({dx:.4f},{dy:.4f},{dz:.4f})",
        )

    # ---- four-slice slot count: verify 4 bread shelves exist ----
    carriage_visuals = [v.name for v in carriage.visuals]
    shelf_names = [n for n in carriage_visuals if n.startswith("bread_shelf_")]
    ctx.check(
        "carriage carries exactly 4 bread shelves (four-slice)",
        len(shelf_names) == NUM_SLOTS,
        details=f"shelves={shelf_names}",
    )

    # ---- bread slots and recessed rim ----
    rim = ctx.part_element_world_aabb(body, elem="slot_rim_plate")
    ctx.check(
        "lighter-gray rim plate sits recessed just below the shell top",
        rim is not None and rim[1][2] < SHELL_TOP - 0.0001 and rim[1][2] > SHELL_TOP - 0.0035,
        details=f"rim aabb={rim}",
    )

    # ---- 4 bread shelves under the 4 slot positions ----
    for i in range(NUM_SLOTS):
        xc, yc = SLOT_POSITIONS[i]
        shelf = ctx.part_element_world_aabb(carriage, elem=f"bread_shelf_{i}")
        ctx.check(
            f"bread shelf {i} sits centered under slot position ({xc:.3f}, {yc:.3f})",
            shelf is not None
            and abs((shelf[0][1] + shelf[1][1]) / 2.0 - yc) < 0.003
            and shelf[0][2] > CAV_Z0
            and shelf[1][2] < CAV_Z1,
            details=f"shelf {i} aabb={shelf}",
        )

    # ---- control panel on the +X front face ----
    panel = ctx.part_element_world_aabb(body, elem="control_panel")
    ctx.check(
        "brushed-silver control panel is on the +X front face",
        panel is not None and panel[1][0] > SHELL_FACE_X - 0.001 and panel[0][0] > SHELL_FACE_X - 0.010,
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
    ctx.expect_within(
        carriage,
        body,
        axes="xy",
        inner_elem="bread_shelf_0",
        margin=0.0,
        name="carriage shelf 0 stays inside the body footprint",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({lever_joint: LEVER_TRAVEL}):
        pressed_pos = ctx.part_world_position(carriage)
        shelf_dn = ctx.part_element_world_aabb(carriage, elem="bread_shelf_0")
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
        # buttons are on the viewer's left (-Y) half of the panel
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

    # ---- four-slice body is longer than two-slice ----
    ctx.check(
        "four-slice body is longer than 0.35 m along X",
        aabb is not None and (aabb[1][0] - aabb[0][0]) > 0.35,
        details=f"body X extent={aabb[1][0] - aabb[0][0]:.4f}" if aabb else "no aabb",
    )

    return ctx.report()


object_model = build_object_model()
