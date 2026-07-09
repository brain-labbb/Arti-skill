from __future__ import annotations

# Industrial electrical control / disconnect panel.
#
# Reference: picture/Equipment/Control panel/003.png
# A grey sheet-metal enclosure mounted on a rear clamp carried by twin
# horizontal machine rails. The front door carries a recessed digital display
# window and a row of round push buttons. The defining mechanism is the
# SIDE-MOUNTED ROTARY
# DISCONNECT HANDLE on the left side of the enclosure: a flanged operator boss
# with a dark throw lever that rotates about a horizontal axis (pointing out of
# the left wall) to switch the disconnect between OFF (down) and ON (up).
#
# Articulation: the operating handle is the primary moving part -> REVOLUTE
# about the operator-shaft axis. Positive q lifts the lever from OFF toward ON.

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Real-world dimensions (meters). A compact rail-mounted disconnect enclosure.
# ---------------------------------------------------------------------------
ENC_W = 0.300  # enclosure width (X, across the front face)
ENC_H = 0.360  # enclosure height (Z)
ENC_D = 0.140  # enclosure depth (Y, into the wall)

DOOR_TH = 0.022  # front door thickness (Y)
WALL_TH = 0.004  # sheet-metal wall thickness

# Operator handle (side-mounted rotary disconnect) -------------------------
OP_X = -ENC_W / 2.0  # handle sits on the left wall (-X face)
OP_Z = 0.060  # height of the operator shaft above enclosure mid
SHAFT_R = 0.010  # operator shaft radius
FLANGE_R = 0.030  # operator base flange radius
LEVER_LEN = 0.090  # throw-lever length from shaft center to tip
LEVER_W = 0.026  # lever width
LEVER_TH = 0.016  # lever thickness

# ---------------------------------------------------------------------------
# Geometry builders (CadQuery, authored directly in meters).
# ---------------------------------------------------------------------------


def _build_enclosure_shape() -> cq.Workplane:
    """Hollow sheet-metal enclosure box, open at the front (+Y), with a back
    mounting flange and corner mounting ears."""
    outer = (
        cq.Workplane("XY")
        .box(ENC_W, ENC_D, ENC_H)
        .edges("|Y")
        .fillet(0.010)
    )
    # Hollow it out: remove the interior, leaving walls of WALL_TH.
    inner = (
        cq.Workplane("XY")
        .box(ENC_W - 2 * WALL_TH, ENC_D - 2 * WALL_TH, ENC_H - 2 * WALL_TH)
        .translate((0.0, 0.0, 0.0))
    )
    body = outer.cut(inner)
    # The enclosure stays a closed hollow box; the dead-front control panel is
    # bolted PROUD onto the front (+Y) face rather than recessed into it, which
    # matches the surface-mounted front plate in the reference.

    # Rear mounting flange tabs (top and bottom), proud of the back wall.
    for sz in (ENC_H / 2.0 - 0.012, -(ENC_H / 2.0 - 0.012)):
        ear = (
            cq.Workplane("XY")
            .box(ENC_W + 0.040, 0.006, 0.024)
            .translate((0.0, -ENC_D / 2.0 - 0.003, sz))
        )
        body = body.union(ear)
    return body


def _build_operator_bore_relief(shape: cq.Workplane) -> cq.Workplane:
    """Cut a clearance bore through the left wall for the operator shaft so the
    handle assembly reads as passing into the enclosure."""
    bore = (
        cq.Workplane("YZ")
        .circle(SHAFT_R + 0.0015)
        .extrude(0.040)
        .translate((OP_X - 0.001, 0.0, OP_Z))
    )
    return shape.cut(bore)


def _build_door_shape() -> cq.Workplane:
    """Front door panel: a slab with a recessed display window pocket and four
    round button counterbores, plus a small latch boss on the right edge."""
    door = (
        cq.Workplane("XY")
        .box(ENC_W - 2 * WALL_TH - 0.004, DOOR_TH, ENC_H - 2 * WALL_TH - 0.004)
        .edges("|Y")
        .fillet(0.004)
    )
    front_y = DOOR_TH / 2.0

    # Recessed display window pocket (upper area of the door).
    disp_w, disp_h, disp_depth = 0.150, 0.072, 0.008
    disp_z = 0.072
    pocket = (
        cq.Workplane("XY")
        .box(disp_w, disp_depth + 0.002, disp_h)
        .translate((0.0, front_y - disp_depth / 2.0 + 0.001, disp_z))
    )
    door = door.cut(pocket)

    # Four round button counterbores in a row below the display.
    btn_r, btn_depth = 0.016, 0.006
    btn_z = -0.030
    spacing = 0.060
    for i in range(4):
        bx = (i - 1.5) * spacing
        cb = (
            cq.Workplane("XZ")
            .circle(btn_r)
            .extrude(btn_depth + 0.002)
            .translate((bx, front_y + 0.001, btn_z))
            .rotate((0, 0, 0), (1, 0, 0), 90)
        )
        # Simpler: cut a shallow cylindrical recess facing +Y.
        cb = (
            cq.Workplane("XY")
            .circle(btn_r)
            .extrude(btn_depth + 0.002)
            .rotate((0, 0, 0), (1, 0, 0), -90)
            .translate((bx, front_y - btn_depth / 2.0 + 0.001, btn_z))
        )
        door = door.cut(cb)

    # Latch / quarter-turn boss on the right edge, slightly proud of the front
    # face (less proud than the push buttons so the controls stay dominant).
    latch = (
        cq.Workplane("XY")
        .box(0.016, 0.010, 0.044)
        .translate(((ENC_W - 2 * WALL_TH - 0.004) / 2.0 - 0.006, front_y + 0.001, -0.110))
    )
    door = door.union(latch)
    return door


def _build_display_glass_shape() -> cq.Workplane:
    """Dark glossy display lens that sits inside the door pocket."""
    return cq.Workplane("XY").box(0.146, 0.006, 0.068).edges("|Y").fillet(0.002)


def _build_button_shape() -> cq.Workplane:
    """One round push button cap, domed slightly, facing +Y."""
    cap = (
        cq.Workplane("XY")
        .circle(0.0145)
        .extrude(0.010)
        .rotate((0, 0, 0), (1, 0, 0), -90)
    )
    return cap


def _build_operator_base_shape() -> cq.Workplane:
    """Operator flange + shaft stub mounted on the enclosure left wall.

    Authored in the enclosure (base) part frame so it attaches to the wall.
    The flange face is on the -X wall; the shaft stub points out along -X.
    """
    flange = (
        cq.Workplane("YZ")
        .circle(FLANGE_R)
        .extrude(-0.014)  # proud of the wall toward -X
        .translate((OP_X, 0.0, OP_Z))
    )
    # Shaft stub passing out to the lever hub.
    shaft = (
        cq.Workplane("YZ")
        .circle(SHAFT_R)
        .extrude(-0.030)
        .translate((OP_X, 0.0, OP_Z))
    )
    return flange.union(shaft)


def _build_handle_shape() -> cq.Workplane:
    """Rotary disconnect throw lever and its hub.

    Authored about the operator axis at the LOCAL ORIGIN so the articulation
    frame (placed at the shaft) and the part frame coincide at q=0. The hub
    wraps the shaft; the lever arm extends along +Z (the OFF->ON throw direction
    when the closed lever points down is handled by the joint axis/limits).
    Geometry is built in the YZ-ish frame: long axis of the operator is X.
    """
    # Hub: a short cylinder around the X (operator) axis.
    hub = (
        cq.Workplane("YZ")
        .circle(0.016)
        .extrude(-0.020)  # extends outward (-X) from the shaft plane
    )
    # Bore for the shaft (so the hub reads as gripping the shaft).
    bore = (
        cq.Workplane("YZ")
        .circle(SHAFT_R + 0.0008)
        .extrude(-0.022)
    )
    hub = hub.cut(bore)

    # Lever arm: a rounded bar extending from the hub along local -Z (pointing
    # down at the OFF rest pose). Built at the outboard X plane of the hub.
    lever = (
        cq.Workplane("XY")
        .box(LEVER_TH, LEVER_W, LEVER_LEN)
        .edges("|X")
        .fillet(0.004)
        .translate((-0.014, 0.0, -LEVER_LEN / 2.0))
    )
    # Grip knob at the lever tip.
    grip = (
        cq.Workplane("YZ")
        .circle(0.014)
        .extrude(-0.016)
        .translate((-0.014, 0.0, -LEVER_LEN))
    )
    return hub.union(lever).union(grip)


RAIL_R = 0.012
RAIL_LEN = 0.560
RAIL_Y = -ENC_D / 2.0 - 0.075
RAIL_ZS = (-0.115, 0.115)
# The enclosure has shallow rear flange ears proud of the back wall.  The clamp
# front face bears on the back of those ears, avoiding broad hidden penetration.
CLAMP_FRONT_Y = -ENC_D / 2.0 - 0.006
CLAMP_TH = 0.018
CLAMP_CENTER_Y = CLAMP_FRONT_Y - CLAMP_TH / 2.0


def _build_horizontal_rail_shape(z: float) -> cq.Workplane:
    """One round horizontal support rail running left/right behind the box."""
    return (
        cq.Workplane("YZ")
        .circle(RAIL_R)
        .extrude(RAIL_LEN)
        .translate((-RAIL_LEN / 2.0, RAIL_Y, z))
    )


def _build_rear_clamp_shape() -> cq.Workplane:
    """Rear clamp mount that bolts to the enclosure back and wraps both rails.

    The front face of the plate is coplanar with the enclosure rear wall while
    the split saddle blocks bridge back to the two horizontal rails. The rails
    themselves are separate visuals in the same base part, but the clamp blocks
    overlap them as real split collars, making the base read as one rigid mount.
    """
    plate = (
        cq.Workplane("XY")
        .box(0.235, CLAMP_TH, 0.300)
        .edges("|Y")
        .fillet(0.004)
        .translate((0.0, CLAMP_CENTER_Y, 0.0))
    )

    clamp = plate
    for i, rz in enumerate(RAIL_ZS):
        saddle = (
            cq.Workplane("XY")
            .box(0.130, 0.048, 0.052)
            .edges("|Y")
            .fillet(0.004)
            .translate((0.0, RAIL_Y, rz))
        )
        # Cylindrical relief through the saddle leaves a believable split clamp
        # profile around the round rail rather than a square block placeholder.
        relief = (
            cq.Workplane("YZ")
            .circle(RAIL_R * 0.70)
            .extrude(0.136)
            .translate((-0.068, RAIL_Y, rz))
        )
        saddle = saddle.cut(relief)

        # Web from rear plate to saddle, visibly carrying the load to the rail.
        web_y = (CLAMP_CENTER_Y - CLAMP_TH / 2.0 + RAIL_Y + 0.024) / 2.0
        web_depth = abs((CLAMP_CENTER_Y - CLAMP_TH / 2.0) - (RAIL_Y + 0.024))
        web = (
            cq.Workplane("XY")
            .box(0.090, web_depth, 0.020)
            .translate((0.0, web_y, rz))
        )
        clamp = clamp.union(saddle).union(web)

        # Pair of small screw bosses on the clamp cap.
        for sx in (-0.045, 0.045):
            boss = (
                cq.Workplane("XY")
                .circle(0.006)
                .extrude(0.006)
                .translate((sx, RAIL_Y + 0.024, rz + (0.018 if i == 1 else -0.018)))
            )
            clamp = clamp.union(boss)

    # Four raised stand-off bolt pads bridge the last few millimeters from the
    # clamp plate to the enclosure back wall, giving an actual visible support
    # contact instead of an invisible fixed-joint anchor.
    for bx in (-0.090, 0.090):
        for bz in (-0.128, 0.128):
            pad = (
                cq.Workplane("XY")
                .box(0.020, abs((-ENC_D / 2.0) - CLAMP_FRONT_Y), 0.020)
                .edges("|Y")
                .fillet(0.002)
                .translate((bx, (CLAMP_FRONT_Y + (-ENC_D / 2.0)) / 2.0, bz))
            )
            clamp = clamp.union(pad)
    return clamp


# ---------------------------------------------------------------------------
# Model assembly.
# ---------------------------------------------------------------------------


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_disconnect_panel")

    model.material("enclosure_grey", rgba=(0.55, 0.57, 0.60, 1.0))
    model.material("door_grey", rgba=(0.50, 0.52, 0.55, 1.0))
    model.material("display_dark", rgba=(0.06, 0.07, 0.09, 1.0))
    model.material("button_grey", rgba=(0.30, 0.31, 0.34, 1.0))
    model.material("operator_steel", rgba=(0.66, 0.68, 0.71, 1.0))
    model.material("handle_dark", rgba=(0.12, 0.13, 0.15, 1.0))
    model.material("rail_dark", rgba=(0.20, 0.22, 0.24, 1.0))
    model.material("clamp_steel", rgba=(0.46, 0.47, 0.46, 1.0))

    # --- Base: twin horizontal rails + rear clamp carrying the enclosure ------
    base = model.part("base")
    for i, rz in enumerate(RAIL_ZS):
        base.visual(
            mesh_from_cadquery(_build_horizontal_rail_shape(rz), f"rail_{i}"),
            material="rail_dark",
            name=f"rail_{i}",
        )
    base.visual(
        mesh_from_cadquery(_build_rear_clamp_shape(), "rear_clamp"),
        material="clamp_steel",
        name="rear_clamp",
    )

    # --- Enclosure housing (mounted to the rear clamp) -----------------------
    enclosure_shape = _build_enclosure_shape()
    enclosure_shape = _build_operator_bore_relief(enclosure_shape)
    enclosure = model.part("enclosure")
    enclosure.visual(
        mesh_from_cadquery(enclosure_shape, "enclosure_housing"),
        material="enclosure_grey",
        name="enclosure_housing",
    )
    # Operator base flange + shaft are rigidly part of the enclosure wall.
    enclosure.visual(
        mesh_from_cadquery(_build_operator_base_shape(), "operator_base"),
        material="operator_steel",
        name="operator_base",
    )

    # --- Door (fixed to enclosure; carries display + buttons) ----------------
    # Dead-front control panel bolted proud on the enclosure front (+Y) face.
    door_y = ENC_D / 2.0 + DOOR_TH / 2.0
    door = model.part("door")
    door.visual(
        mesh_from_cadquery(_build_door_shape(), "door_panel"),
        material="door_grey",
        origin=Origin(xyz=(0.0, door_y, 0.0)),
        name="door_panel",
    )
    # Display lens seated in the door pocket.
    door.visual(
        mesh_from_cadquery(_build_display_glass_shape(), "display_glass"),
        material="display_dark",
        origin=Origin(xyz=(0.0, door_y + DOOR_TH / 2.0 - 0.005, 0.072)),
        name="display_glass",
    )
    # Four push buttons seated in their counterbores.
    btn_face_y = door_y + DOOR_TH / 2.0 - 0.006
    for i in range(4):
        bx = (i - 1.5) * 0.060
        door.visual(
            mesh_from_cadquery(_build_button_shape(), f"button_{i}"),
            material="button_grey",
            origin=Origin(xyz=(bx, btn_face_y, -0.030)),
            name=f"button_{i}",
        )

    # --- Rotary disconnect handle (the primary moving part) ------------------
    handle = model.part("handle")
    handle.visual(
        mesh_from_cadquery(_build_handle_shape(), "disconnect_handle"),
        material="handle_dark",
        name="disconnect_handle",
    )

    # === Articulations =======================================================
    # Enclosure is rigidly bolted to the rear clamp on the rail structure.
    model.articulation(
        "base_to_enclosure",
        ArticulationType.FIXED,
        parent=base,
        child=enclosure,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    # Door is bolted/latched onto the enclosure (treated as fixed; the hero
    # mechanism is the side rotary handle, not a swinging door here).
    model.articulation(
        "enclosure_to_door",
        ArticulationType.FIXED,
        parent=enclosure,
        child=door,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    # Rotary disconnect handle: REVOLUTE about the operator shaft (the local +X
    # operator axis, pointing out of the enclosure's left wall). The lever hangs
    # down along -Z at q=0 (OFF). With axis (+1,0,0), the right-hand rule sweeps
    # a -Z point up through +Y to +Z, so positive q raises the lever to the ON
    # (up) position. A ~160 deg throw makes ON read clearly upward.
    model.articulation(
        "operator_handle",
        ArticulationType.REVOLUTE,
        parent=enclosure,
        child=handle,
        origin=Origin(xyz=(OP_X - 0.018, 0.0, OP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0, velocity=2.0, lower=0.0, upper=math.radians(160.0)
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests: encode the prompt-specific visual + mechanical claims.
# ---------------------------------------------------------------------------


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    enclosure = object_model.get_part("enclosure")
    door = object_model.get_part("door")
    handle = object_model.get_part("handle")
    op_joint = object_model.get_articulation("operator_handle")

    # --- Intentional capture/seat overlaps -----------------------------------
    # The handle hub deliberately closes over the operator shaft stub so it
    # reads as gripping the shaft it rotates on.
    ctx.allow_overlap(
        handle,
        enclosure,
        elem_a="disconnect_handle",
        elem_b="operator_base",
        reason="The handle hub is captured on the operator shaft stub it rotates on.",
    )
    # The rear clamp's raised stand-off pads are modeled as a compressed bolted
    # seat into the enclosure back/ears; this local embed is the visible load
    # path from rails into the sheet-metal box.
    ctx.allow_overlap(
        base,
        enclosure,
        elem_a="rear_clamp",
        elem_b="enclosure_housing",
        reason="The rear clamp stand-off pads are intentionally seated into the enclosure back as a bolted mount.",
    )

    # --- Single root: the rail/clamp base carries everything. ----------------
    roots = object_model.root_parts()
    ctx.check(
        "single root is base",
        len(roots) == 1 and roots[0].name == "base",
        details=f"roots={[p.name for p in roots]}",
    )

    # --- Joint type / axis claims for the rotary disconnect handle. ----------
    ctx.check(
        "operator handle is revolute",
        op_joint.articulation_type == ArticulationType.REVOLUTE,
        details=str(op_joint.articulation_type),
    )
    ax = tuple(round(c, 6) for c in op_joint.axis)
    ctx.check(
        "operator handle axis is +/-X (operator shaft)",
        abs(ax[0]) > 0.99 and abs(ax[1]) < 1e-6 and abs(ax[2]) < 1e-6,
        details=f"axis={ax}",
    )
    lim = op_joint.motion_limits
    ctx.check(
        "operator handle has a large OFF->ON throw",
        lim is not None
        and lim.lower is not None
        and lim.upper is not None
        and (lim.upper - lim.lower) > math.radians(80.0),
        details=f"lower={lim.lower}, upper={lim.upper}",
    )

    # --- Hero parts present and placed. --------------------------------------
    rail0_aabb = ctx.part_element_world_aabb(base, elem="rail_0")
    rail1_aabb = ctx.part_element_world_aabb(base, elem="rail_1")
    clamp_aabb = ctx.part_element_world_aabb(base, elem="rear_clamp")
    ctx.check(
        "base uses two horizontal rails",
        rail0_aabb is not None
        and rail1_aabb is not None
        and (rail0_aabb[1][0] - rail0_aabb[0][0]) > 0.50
        and (rail1_aabb[1][0] - rail1_aabb[0][0]) > 0.50
        and (rail0_aabb[1][2] - rail0_aabb[0][2]) < 0.035
        and (rail1_aabb[1][2] - rail1_aabb[0][2]) < 0.035,
        details=f"rail0={rail0_aabb}, rail1={rail1_aabb}",
    )
    ctx.check(
        "rails are a separated twin pair behind the enclosure",
        rail0_aabb is not None
        and rail1_aabb is not None
        and abs(((rail1_aabb[0][2] + rail1_aabb[1][2]) / 2.0) - ((rail0_aabb[0][2] + rail0_aabb[1][2]) / 2.0)) > 0.20
        and rail0_aabb[1][1] < -ENC_D / 2.0
        and rail1_aabb[1][1] < -ENC_D / 2.0,
        details=f"rail0={rail0_aabb}, rail1={rail1_aabb}",
    )
    ctx.expect_contact(
        base,
        enclosure,
        elem_a="rear_clamp",
        elem_b="enclosure_housing",
        contact_tol=0.004,
        name="rear clamp bears on enclosure back",
    )
    ctx.check(
        "rear clamp is behind the enclosure front controls",
        clamp_aabb is not None and clamp_aabb[1][1] <= -ENC_D / 2.0 + 0.004,
        details=f"clamp={clamp_aabb}, enclosure_back={-ENC_D / 2.0}",
    )

    # Handle sits on the left (-X) side of the enclosure.
    handle_pos = ctx.part_world_position(handle)
    enc_pos = ctx.part_world_position(enclosure)
    ctx.check(
        "handle is on the left side of the enclosure",
        handle_pos is not None and enc_pos is not None and handle_pos[0] < enc_pos[0],
        details=f"handle={handle_pos}, enclosure={enc_pos}",
    )

    # Display glass sits in the upper half of the door front; buttons below it.
    disp_aabb = ctx.part_element_world_aabb(door, elem="display_glass")
    btn_aabb = ctx.part_element_world_aabb(door, elem="button_0")
    ctx.check(
        "display sits above the buttons",
        disp_aabb is not None
        and btn_aabb is not None
        and disp_aabb[0][2] > btn_aabb[1][2],
        details=f"display_zmin={disp_aabb and disp_aabb[0][2]}, button_zmax={btn_aabb and btn_aabb[1][2]}",
    )

    # Display + buttons face forward: they live on the front (+Y) side, clearly
    # ahead of the enclosure front plane (y = ENC_D/2).
    enc_front_y = ENC_D / 2.0
    ctx.check(
        "display lens is on the front (+Y) of the panel",
        disp_aabb is not None and disp_aabb[1][1] > enc_front_y,
        details=f"disp_ymax={disp_aabb and disp_aabb[1][1]}, enc_front={enc_front_y}",
    )
    btn0_aabb = ctx.part_element_world_aabb(door, elem="button_0")
    ctx.check(
        "buttons protrude from the front (+Y) of the panel",
        btn0_aabb is not None and btn0_aabb[1][1] > enc_front_y,
        details=f"btn0_ymax={btn0_aabb and btn0_aabb[1][1]}, enc_front={enc_front_y}",
    )

    # Door is seated within the enclosure footprint (front face) in X and Z.
    ctx.expect_within(
        door,
        enclosure,
        axes="xz",
        margin=0.004,
        name="door fits within enclosure opening",
    )

    # --- Mechanism actually moves: ON pose lifts the lever tip upward. -------
    # At q=0 (OFF) the lever points down; at the upper limit it should swing up.
    handle_tip_off = ctx.part_world_aabb(handle)
    with ctx.pose({op_joint: op_joint.motion_limits.upper}):
        handle_tip_on = ctx.part_world_aabb(handle)
    ctx.check(
        "throwing the handle to ON raises the lever",
        handle_tip_off is not None
        and handle_tip_on is not None
        and handle_tip_on[1][2] > handle_tip_off[1][2] + 0.02,
        details=f"off_zmax={handle_tip_off and handle_tip_off[1][2]}, on_zmax={handle_tip_on and handle_tip_on[1][2]}",
    )

    # Handle hub stays adjacent to the operator base on the enclosure (no float).
    ctx.expect_contact(
        handle,
        enclosure,
        contact_tol=0.006,
        name="handle hub meets the operator base",
    )

    return ctx.report()


object_model = build_object_model()
