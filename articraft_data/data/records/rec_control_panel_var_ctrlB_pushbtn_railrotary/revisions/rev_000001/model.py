from __future__ import annotations

# Industrial electrical control / disconnect panel.
#
# Reference: picture/Equipment/Control panel/003.png
# A grey sheet-metal enclosure mounted on a wall in front of vertical conduit
# runs. The front door carries a recessed digital display window and a row of
# round controls. This fork replaces the former side rotary disconnect handle
# with a conduit-mounted side operator door. The side door carries only its own
# compact display and a row of round spring-return push buttons.
#
# Articulation: the side-door push buttons are the live moving controls ->
# PRISMATIC into the door face. Positive q depresses a button inward.

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

# Side-mounted replacement operator door ------------------------------------
# The visible service side face is the enclosure's -X wall.  A shallow operator
# door is bolted to that face and carries its own display plus push buttons.
SIDE_DOOR_TH = 0.024  # side-door thickness along X
SIDE_DOOR_W = 0.140  # side-door width along Y
SIDE_DOOR_H = 0.230  # side-door height along Z
SIDE_DOOR_Z = 0.010
SIDE_FACE_X = -ENC_W / 2.0 - SIDE_DOOR_TH  # visible -X face of the side door
SIDE_BUTTON_COUNT = 4
SIDE_BUTTON_SPACING = 0.026
SIDE_BUTTON_Z = SIDE_DOOR_Z - 0.056
SIDE_BUTTON_R = 0.0105
SIDE_BUTTON_TRAVEL = 0.006

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
    # Hollow it out with the cavity open at the front (+Y), leaving the rear
    # and side sheet-metal walls connected by a real front rim.
    inner = (
        cq.Workplane("XY")
        .box(ENC_W - 2 * WALL_TH, ENC_D, ENC_H - 2 * WALL_TH)
        .translate((0.0, WALL_TH, 0.0))
    )
    body = outer.cut(inner)
    # The dead-front control panel is bolted PROUD onto the front (+Y) rim,
    # which matches the surface-mounted front plate in the reference.

    # Rear mounting flange tabs (top and bottom), proud of the back wall.
    for sz in (ENC_H / 2.0 - 0.012, -(ENC_H / 2.0 - 0.012)):
        ear = (
            cq.Workplane("XY")
            .box(ENC_W + 0.040, 0.006, 0.024)
            .translate((0.0, -ENC_D / 2.0 + 0.001, sz))
        )
        body = body.union(ear)
    # A narrow rear spine bridges the two mounting ears into the back wall so
    # the sheet-metal mount is one continuous welded/formed assembly.
    rear_spine = (
        cq.Workplane("XY")
        .box(0.026, 0.006, ENC_H - 0.020)
        .translate((0.0, -ENC_D / 2.0 + 0.001, 0.0))
    )
    body = body.union(rear_spine)
    return body


def _build_side_door_shape() -> cq.Workplane:
    """Conduit-mounted side operator door on the -X face.

    The slab is built directly in the enclosure frame.  Its inboard face lands
    on the enclosure side wall (x = -ENC_W/2); the visible controls face -X.
    Display and button cavities are cut into the visible face so the live button
    parts can move inward without colliding with a solid proxy door.
    """
    center_x = -ENC_W / 2.0 - SIDE_DOOR_TH / 2.0
    side_door = (
        cq.Workplane("XY")
        .box(SIDE_DOOR_TH, SIDE_DOOR_W, SIDE_DOOR_H)
        .edges("|X")
        .fillet(0.0035)
        .translate((center_x, 0.0, SIDE_DOOR_Z))
    )

    # Raised edge gasket / door rim, still one connected side-door shell.
    rim_outer = (
        cq.Workplane("YZ")
        .rect(SIDE_DOOR_W - 0.012, SIDE_DOOR_H - 0.014)
        .extrude(-0.0035)
        .translate((SIDE_FACE_X, 0.0, SIDE_DOOR_Z))
    )
    rim_inner = (
        cq.Workplane("YZ")
        .rect(SIDE_DOOR_W - 0.032, SIDE_DOOR_H - 0.034)
        .extrude(-0.0045)
        .translate((SIDE_FACE_X - 0.0008, 0.0, SIDE_DOOR_Z))
    )
    side_door = side_door.union(rim_outer.cut(rim_inner))

    # Display recess (upper half): a shallow pocket facing -X.
    display_pocket = (
        cq.Workplane("YZ")
        .rect(0.070, 0.036)
        .extrude(0.006)
        .translate((SIDE_FACE_X - 0.001, 0.0, SIDE_DOOR_Z + 0.046))
    )
    side_door = side_door.cut(display_pocket)

    # Button row: large shallow cap counterbores plus smaller through-holes for
    # the stems.  Created in the same for-i-in-range(n) pattern as the live
    # child parts and joints.
    first_y = -0.5 * (SIDE_BUTTON_COUNT - 1) * SIDE_BUTTON_SPACING
    for i in range(SIDE_BUTTON_COUNT):
        by = first_y + i * SIDE_BUTTON_SPACING
        cap_recess = (
            cq.Workplane("YZ")
            .circle(SIDE_BUTTON_R)
            .extrude(SIDE_BUTTON_TRAVEL + 0.002)
            .translate((SIDE_FACE_X - 0.001, by, SIDE_BUTTON_Z))
        )
        stem_hole = (
            cq.Workplane("YZ")
            .circle(0.0065)
            .extrude(SIDE_DOOR_TH + 0.006)
            .translate((SIDE_FACE_X - 0.002, by, SIDE_BUTTON_Z))
        )
        side_door = side_door.cut(cap_recess).cut(stem_hole)

    # Two screw bosses on the cover, integrated with the slab rather than
    # separate decoration parts.
    for z in (SIDE_DOOR_Z + SIDE_DOOR_H / 2.0 - 0.026, SIDE_DOOR_Z - SIDE_DOOR_H / 2.0 + 0.026):
        boss = (
            cq.Workplane("YZ")
            .circle(0.007)
            .extrude(-0.004)
            .translate((SIDE_FACE_X, 0.0, z))
        )
        slot = (
            cq.Workplane("YZ")
            .rect(0.010, 0.0018)
            .extrude(-0.0048)
            .translate((SIDE_FACE_X - 0.0008, 0.0, z))
        )
        side_door = side_door.union(boss.cut(slot))

    return side_door


def _build_door_shape() -> cq.Workplane:
    """Front door panel: a slab with a recessed display window pocket and four
    round button counterbores, plus a small latch boss on the right edge."""
    door = (
        cq.Workplane("XY")
        .box(ENC_W - 0.004, DOOR_TH, ENC_H - 0.004)
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
        .translate(((ENC_W - 0.004) / 2.0 - 0.006, front_y + 0.001, -0.110))
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


def _build_side_display_glass_shape() -> cq.Workplane:
    """Dark lens for the replacement side operator door, facing -X."""
    return (
        cq.Workplane("XY")
        .box(0.004, 0.074, 0.040)
        .edges("|X")
        .fillet(0.0015)
    )


def _build_side_button_shape() -> cq.Workplane:
    """Round push button with a visible cap outside the side door and a smaller
    stem entering the bored opening. Authored about the joint origin on the door
    face; positive prismatic motion along +X depresses it inward."""
    cap = (
        cq.Workplane("YZ")
        .circle(SIDE_BUTTON_R + 0.0002)
        .extrude(-0.010)
        .edges("%CIRCLE")
        .fillet(0.0012)
    )
    stem = (
        cq.Workplane("YZ")
        .circle(0.0058)
        .extrude(0.014)
    )
    # A shallow raised lip on the button face, still connected to the cap.
    face_lip = (
        cq.Workplane("YZ")
        .circle(SIDE_BUTTON_R * 0.72)
        .extrude(-0.0012)
        .translate((-0.010, 0.0, 0.0))
    )
    # A microscopic positive-X seating shift removes visual/mesh clearance at
    # the collar while keeping the joint frame on the visible door face.
    return cap.union(stem).union(face_lip).translate((0.00008, 0.0, 0.0))


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
    model = ArticulatedObject(name="industrial_pushbutton_panel")

    model.material("enclosure_grey", rgba=(0.55, 0.57, 0.60, 1.0))
    model.material("door_grey", rgba=(0.50, 0.52, 0.55, 1.0))
    model.material("display_dark", rgba=(0.06, 0.07, 0.09, 1.0))
    model.material("button_grey", rgba=(0.30, 0.31, 0.34, 1.0))
    model.material("button_black", rgba=(0.09, 0.10, 0.11, 1.0))
    model.material("button_red", rgba=(0.55, 0.05, 0.04, 1.0))
    model.material("conduit_steel", rgba=(0.40, 0.42, 0.45, 1.0))

    # --- Base: fixed wall infrastructure (conduit) carrying the enclosure ----
    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_build_conduit_shape(), "conduit_runs"),
        material="conduit_steel",
    )

    # --- Enclosure housing (mounted to the conduit/wall) ---------------------
    enclosure_shape = _build_enclosure_shape()
    enclosure = model.part("enclosure")
    enclosure.visual(
        mesh_from_cadquery(enclosure_shape, "enclosure_housing"),
        material="enclosure_grey",
        name="enclosure_housing",
    )
    enclosure.visual(
        mesh_from_cadquery(_build_side_door_shape(), "side_door_panel"),
        material="door_grey",
        name="side_door_panel",
    )
    enclosure.visual(
        mesh_from_cadquery(_build_side_display_glass_shape(), "side_display_glass"),
        material="display_dark",
        origin=Origin(xyz=(SIDE_FACE_X - 0.0005, 0.0, SIDE_DOOR_Z + 0.046)),
        name="side_display_glass",
    )
    # --- Door (fixed to enclosure; carries display + buttons) ----------------
    # Dead-front control panel bolted proud on the enclosure front (+Y) face.
    door_y = ENC_D / 2.0 + DOOR_TH / 2.0 - 0.002
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

    # Round prismatic push buttons carried by the replacement side door.
    side_buttons = []
    first_y = -0.5 * (SIDE_BUTTON_COUNT - 1) * SIDE_BUTTON_SPACING
    for i in range(SIDE_BUTTON_COUNT):
        by = first_y + i * SIDE_BUTTON_SPACING
        button = model.part(f"side_button_{i}")
        button.visual(
            mesh_from_cadquery(_build_side_button_shape(), f"side_button_cap_{i}"),
            material="button_red" if i == 0 else "button_black",
            name=f"side_button_cap_{i}",
        )
        side_buttons.append((button, by))

    # === Articulations =======================================================
    # Enclosure is rigidly bolted to the conduit/wall structure.
    model.articulation(
        "base_to_enclosure",
        ArticulationType.FIXED,
        parent=base,
        child=enclosure,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    # Front door is bolted/latched onto the enclosure (treated as fixed; the
    # live controls in this fork are the side-door push buttons).
    model.articulation(
        "enclosure_to_door",
        ArticulationType.FIXED,
        parent=enclosure,
        child=door,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    # Uniform push-button policy: every round side-door button translates inward
    # along +X from the visible side-door face; at least one remains live for export.
    for i, (button, by) in enumerate(side_buttons):
        model.articulation(
            f"side_button_slide_{i}",
            ArticulationType.PRISMATIC,
            parent=enclosure,
            child=button,
            origin=Origin(xyz=(SIDE_FACE_X, by, SIDE_BUTTON_Z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=0.25,
                lower=0.0,
                upper=SIDE_BUTTON_TRAVEL,
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
    side_button_0 = object_model.get_part("side_button_0")
    button_joint_0 = object_model.get_articulation("side_button_slide_0")

    # --- Intentional capture/seat overlaps ----------------------------------
    # The conduit mounting strap seats against the enclosure rear; a hair of
    # embed reads as the bolted mount carrying the enclosure.
    ctx.allow_overlap(
        base,
        enclosure,
        reason="The conduit mounting strap is bolted flush against the enclosure rear wall/ears.",
    )
    ctx.expect_contact(
        base,
        enclosure,
        contact_tol=0.003,
        name="enclosure is carried by the conduit mounting strap",
    )
    for i in range(SIDE_BUTTON_COUNT):
        button = object_model.get_part(f"side_button_{i}")
        ctx.allow_overlap(
            enclosure,
            button,
            elem_a="side_door_panel",
            elem_b=f"side_button_cap_{i}",
            reason="The spring push-button cap is intentionally seated with a slight collar interference in the side door.",
        )
        ctx.expect_overlap(
            enclosure,
            button,
            axes="x",
            min_overlap=0.001,
            elem_a="side_door_panel",
            elem_b=f"side_button_cap_{i}",
            name=f"side button {i} remains seated in the side door collar",
        )
        ctx.expect_within(
            button,
            enclosure,
            axes="yz",
            margin=0.0,
            elem_b="side_door_panel",
            name=f"side button {i} stays centered in the side door face",
        )

    # --- Single root: the conduit/wall base carries everything. --------------
    roots = object_model.root_parts()
    ctx.check(
        "single root is base",
        len(roots) == 1 and roots[0].name == "base",
        details=f"roots={[p.name for p in roots]}",
    )

    # --- Changed mechanism: rotary disconnect removed, live prismatic button.
    ctx.check(
        "rotary disconnect handle part is removed",
        not any(part.name == "handle" for part in object_model.parts),
        details="part named handle should not exist in this fork",
    )
    ctx.check(
        "rotary operator joint is removed",
        not any(joint.name == "operator_handle" for joint in object_model.articulations),
        details="joint named operator_handle should not exist in this fork",
    )
    ctx.check(
        "side button is prismatic",
        button_joint_0.articulation_type == ArticulationType.PRISMATIC,
        details=str(button_joint_0.articulation_type),
    )
    ax = tuple(round(c, 6) for c in button_joint_0.axis)
    ctx.check(
        "side button depresses inward along +X",
        ax == (1.0, 0.0, 0.0),
        details=f"axis={ax}",
    )
    lim = button_joint_0.motion_limits
    ctx.check(
        "side button has short push-button travel",
        lim is not None
        and lim.lower == 0.0
        and lim.upper is not None
        and 0.003 <= lim.upper <= 0.010,
        details=f"lower={lim.lower}, upper={lim.upper}",
    )

    # --- Hero parts present and placed. --------------------------------------
    # Replacement side door is an enclosure-mounted side face, not a fixed
    # decoration part.
    side_panel_aabb = ctx.part_element_world_aabb(enclosure, elem="side_door_panel")
    enclosure_body_aabb = ctx.part_element_world_aabb(enclosure, elem="enclosure_housing")
    ctx.check(
        "side door is an inline enclosure visual",
        not any(part.name == "side_door" for part in object_model.parts)
        and side_panel_aabb is not None,
        details=f"side_panel_aabb={side_panel_aabb}",
    )
    ctx.check(
        "replacement side door is on the left side of the enclosure",
        side_panel_aabb is not None
        and enclosure_body_aabb is not None
        and side_panel_aabb[0][0] < enclosure_body_aabb[0][0],
        details=f"side_panel={side_panel_aabb}, enclosure_body={enclosure_body_aabb}",
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

    # The replacement side door has its own display above a regular row of
    # round prismatic push buttons, all on the visible -X face.
    side_disp_aabb = ctx.part_element_world_aabb(enclosure, elem="side_display_glass")
    side_btn0_aabb = ctx.part_world_aabb(side_button_0)
    ctx.check(
        "side display sits above the side button row",
        side_disp_aabb is not None
        and side_btn0_aabb is not None
        and side_disp_aabb[0][2] > side_btn0_aabb[1][2],
        details=f"side_display_zmin={side_disp_aabb and side_disp_aabb[0][2]}, side_button_zmax={side_btn0_aabb and side_btn0_aabb[1][2]}",
    )
    ctx.check(
        "side button protrudes from visible -X face",
        side_btn0_aabb is not None and side_btn0_aabb[0][0] < SIDE_FACE_X - 0.004,
        details=f"button_xmin={side_btn0_aabb and side_btn0_aabb[0][0]}, side_face={SIDE_FACE_X}",
    )
    for i in range(SIDE_BUTTON_COUNT):
        ctx.check(
            f"side push button {i} exists",
            object_model.get_part(f"side_button_{i}") is not None
            and object_model.get_articulation(f"side_button_slide_{i}") is not None,
            details=f"missing side_button_{i} or side_button_slide_{i}",
        )

    # Door is seated within the enclosure footprint (front face) in X and Z.
    ctx.expect_within(
        door,
        enclosure,
        axes="xz",
        margin=0.004,
        name="door fits within enclosure opening",
    )

    # --- Mechanism actually moves: pushing a button translates it inward. -----
    button_rest = ctx.part_world_position(side_button_0)
    with ctx.pose({button_joint_0: button_joint_0.motion_limits.upper}):
        button_pressed = ctx.part_world_position(side_button_0)
    ctx.check(
        "pressing the side button moves it inward",
        button_rest is not None
        and button_pressed is not None
        and button_pressed[0] > button_rest[0] + 0.003,
        details=f"rest={button_rest}, pressed={button_pressed}",
    )
    ctx.expect_contact(
        side_button_0,
        enclosure,
        elem_b="side_door_panel",
        contact_tol=0.002,
        name="side button cap sits on the side door face",
    )

    return ctx.report()


object_model = build_object_model()
