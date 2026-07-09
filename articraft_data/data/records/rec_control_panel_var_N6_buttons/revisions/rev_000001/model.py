from __future__ import annotations

# Industrial electrical control / disconnect panel.
#
# Reference: picture/Equipment/Control panel/003.png
# A grey sheet-metal enclosure mounted on a wall in front of vertical conduit
# runs. The front door carries a recessed digital display window and a row of
# round push buttons. The defining mechanism is the SIDE-MOUNTED ROTARY
# DISCONNECT HANDLE on the left side of the enclosure: a flanged operator boss
# with a dark throw lever that rotates about a horizontal axis (pointing out of
# the left wall) to switch the disconnect between OFF (down) and ON (up). This
# fork changes only the front row: six identical spring-return push buttons
# replace the original four-button row.
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
# Real-world dimensions (meters). A compact wall-mounted disconnect enclosure.
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

# Front push-button row -----------------------------------------------------
BUTTON_COUNT = 6
BUTTON_PITCH = 0.044
BUTTON_R = 0.016
BUTTON_COUNTERBORE_R = 0.010
BUTTON_Z = -0.030
BUTTON_TRAVEL = 0.006

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
    """Front door panel: a slab with a recessed display window pocket and six
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

    # Raised rounded escutcheon around the push-button row, as in the reference.
    # It is part of the fixed door and gives the moving caps a real visible
    # parent face to bear against at q=0.
    row_w = (BUTTON_COUNT - 1) * BUTTON_PITCH + 0.060
    row_h = 0.054
    escutcheon = (
        cq.Workplane("XY")
        .box(row_w, BUTTON_TRAVEL, row_h)
        .edges("|Y")
        .fillet(0.010)
        .translate((0.0, front_y + BUTTON_TRAVEL / 2.0, BUTTON_Z))
    )
    door = door.union(escutcheon)

    # Six equal-pitch stem counterbores in a row below the display.
    btn_depth = BUTTON_TRAVEL + 0.014
    for i in range(BUTTON_COUNT):
        bx = (i - (BUTTON_COUNT - 1) / 2.0) * BUTTON_PITCH
        # Cut a shallow cylindrical stem bore facing +Y through the escutcheon
        # and into the door. The larger moving cap stays proud on the face.
        cb = (
            cq.Workplane("XY")
            .circle(BUTTON_COUNTERBORE_R)
            .extrude(btn_depth + 0.002)
            .rotate((0, 0, 0), (1, 0, 0), -90)
            .translate((bx, front_y - 0.012, BUTTON_Z))
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
    """One round push button cap, facing +Y."""
    cap = (
        cq.Workplane("XY")
        .circle(BUTTON_R)
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


CONDUIT_Y = -ENC_D / 2.0 - 0.013  # pipe centerline sits behind the back wall
# Mounting strap front face lands just behind the enclosure back wall plane
# (outer back wall is at y = -ENC_D/2). Small gap avoids a penetration flag;
# the base->enclosure FIXED joint provides the rigid connection.
STRAP_TH = 0.012
STRAP_Y = -ENC_D / 2.0 - 0.0005 - STRAP_TH / 2.0
PIPE_R = 0.012
PIPE_LEN = ENC_H + 0.520
PIPE_XS = (-0.055, -0.018, 0.018, 0.055)
PIPE_Z0 = -PIPE_LEN / 2.0 + ENC_H / 2.0 + 0.080  # pipes centered, run up & down


def _build_conduit_shape() -> cq.Workplane:
    """Vertical conduit / cable bundle running behind the enclosure, tied
    together by a horizontal mounting strap, plus a small junction box hanging
    below. This is the fixed wall infrastructure the enclosure mounts against,
    all one connected body so it reads as a single rigid run."""
    pipes = None
    for dx in PIPE_XS:
        pipe = (
            cq.Workplane("XY")
            .circle(PIPE_R)
            .extrude(PIPE_LEN)
            .translate((dx, CONDUIT_Y, PIPE_Z0))
        )
        pipes = pipe if pipes is None else pipes.union(pipe)

    # Horizontal mounting strap that bridges all four pipes into one body and
    # carries the enclosure (sits right behind the enclosure back wall).
    strap = (
        cq.Workplane("XY")
        .box(0.150, 0.012, 0.040)
        .translate((0.0, STRAP_Y, ENC_H / 2.0 - 0.030))
    )
    pipes = pipes.union(strap)
    strap2 = (
        cq.Workplane("XY")
        .box(0.150, 0.012, 0.040)
        .translate((0.0, CONDUIT_Y + PIPE_R, -ENC_H / 2.0 + 0.030))
    )
    pipes = pipes.union(strap2)

    # Drop conduit from the pipe run down into a small junction box that hangs
    # below the enclosure, kept connected to the run and entering the box.
    drop_top = PIPE_Z0  # bottom of the vertical pipes
    drop_bottom = -ENC_H / 2.0 - 0.140 + 0.020  # enters the junction box top
    drop_len = drop_top - drop_bottom + 0.010
    drop = (
        cq.Workplane("XY")
        .circle(0.008)
        .extrude(drop_len)
        .translate((-0.018, CONDUIT_Y, drop_bottom))
    )
    pipes = pipes.union(drop)
    jbox = (
        cq.Workplane("XY")
        .box(0.045, 0.030, 0.060)
        .edges("|Y")
        .fillet(0.004)
        .translate((-0.030, CONDUIT_Y + 0.003, -ENC_H / 2.0 - 0.140))
    )
    pipes = pipes.union(jbox)
    return pipes


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
    model.material("conduit_steel", rgba=(0.40, 0.42, 0.45, 1.0))

    # --- Base: fixed wall infrastructure (conduit) carrying the enclosure ----
    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_build_conduit_shape(), "conduit_runs"),
        material="conduit_steel",
    )

    # --- Enclosure housing (mounted to the conduit/wall) ---------------------
    enclosure_shape = _build_enclosure_shape()
    enclosure_shape = _build_operator_bore_relief(enclosure_shape)
    enclosure = model.part("enclosure")
    enclosure.visual(
        mesh_from_cadquery(enclosure_shape, "enclosure_housing"),
        material="enclosure_grey",
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
    # Six articulated push buttons seated on the front face.  Their child part
    # frames sit on the door surface at the back of each cap so positive
    # prismatic travel pushes the cap inward into the counterbore.
    buttons = []
    btn_contact_y = door_y + DOOR_TH / 2.0 + BUTTON_TRAVEL
    for i in range(BUTTON_COUNT):
        bx = (i - (BUTTON_COUNT - 1) / 2.0) * BUTTON_PITCH
        button = model.part(f"button_{i}")
        button.visual(
            mesh_from_cadquery(_build_button_shape(), f"button_{i}_cap"),
            material="button_grey",
            name="button_cap",
        )
        buttons.append((button, bx, btn_contact_y, BUTTON_Z))

    # --- Rotary disconnect handle (the primary moving part) ------------------
    handle = model.part("handle")
    handle.visual(
        mesh_from_cadquery(_build_handle_shape(), "disconnect_handle"),
        material="handle_dark",
        name="disconnect_handle",
    )

    # === Articulations =======================================================
    # Enclosure is rigidly bolted to the conduit/wall structure.
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
    # Front push buttons: identical spring-return plungers, each sliding inward
    # normal to the door face into its visible counterbore.
    for i, (button, bx, by, bz) in enumerate(buttons):
        model.articulation(
            f"button_slide_{i}",
            ArticulationType.PRISMATIC,
            parent=door,
            child=button,
            origin=Origin(xyz=(bx, by, bz)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0, velocity=0.5, lower=0.0, upper=BUTTON_TRAVEL
            ),
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
    buttons = [object_model.get_part(f"button_{i}") for i in range(BUTTON_COUNT)]
    op_joint = object_model.get_articulation("operator_handle")
    button_joints = [
        object_model.get_articulation(f"button_slide_{i}") for i in range(BUTTON_COUNT)
    ]

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
    # The conduit mounting strap seats against the enclosure rear; a hair of
    # embed reads as the bolted mount carrying the enclosure.
    ctx.allow_overlap(
        base,
        enclosure,
        reason="The conduit mounting strap is bolted flush against the enclosure rear wall/ears.",
    )

    # --- Single root: the conduit/wall base carries everything. --------------
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

    # --- Targeted fork change: the front row now has six identical sliders. ---
    actual_button_parts = [
        p.name for p in object_model.parts if p.name.startswith("button_")
    ]
    ctx.check(
        "front row has exactly six push buttons",
        len(actual_button_parts) == BUTTON_COUNT
        and all(f"button_{i}" in actual_button_parts for i in range(BUTTON_COUNT)),
        details=f"buttons={actual_button_parts}",
    )
    ctx.check(
        "all push buttons use identical prismatic joints",
        all(j.articulation_type == ArticulationType.PRISMATIC for j in button_joints)
        and all(tuple(round(c, 6) for c in j.axis) == (0.0, -1.0, 0.0) for j in button_joints)
        and all(
            j.motion_limits is not None
            and j.motion_limits.lower == 0.0
            and abs(j.motion_limits.upper - BUTTON_TRAVEL) < 1e-9
            for j in button_joints
        ),
        details=str([
            (j.name, j.articulation_type, j.axis, j.motion_limits)
            for j in button_joints
        ]),
    )
    button_positions = [ctx.part_world_position(button) for button in buttons]
    button_xs = [pos[0] for pos in button_positions if pos is not None]
    button_pitches = [
        round(button_xs[i + 1] - button_xs[i], 6)
        for i in range(len(button_xs) - 1)
    ]
    ctx.check(
        "six button centers use equal X pitch",
        len(button_xs) == BUTTON_COUNT
        and all(abs(p - BUTTON_PITCH) < 1e-6 for p in button_pitches),
        details=f"xs={button_xs}, pitches={button_pitches}",
    )
    button0_rest = ctx.part_world_position(buttons[0])
    with ctx.pose({button_joints[0]: BUTTON_TRAVEL}):
        button0_pressed = ctx.part_world_position(buttons[0])
        ctx.expect_within(
            buttons[0],
            door,
            axes="xz",
            margin=0.002,
            name="pressed button remains centered in its counterbore footprint",
        )
    ctx.check(
        "pressing a push button moves it inward",
        button0_rest is not None
        and button0_pressed is not None
        and button0_pressed[1] < button0_rest[1] - BUTTON_TRAVEL * 0.8,
        details=f"rest={button0_rest}, pressed={button0_pressed}",
    )

    # --- Hero parts present and placed. --------------------------------------
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
    btn_aabb = ctx.part_world_aabb(buttons[0])
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
    btn0_aabb = ctx.part_world_aabb(buttons[0])
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
