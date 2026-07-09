from __future__ import annotations

# Built-in compact microwave oven (Brastemp Gourmand style).
#
# Coordinate convention (Z-up, meters):
#   +X = out of the front face (toward the viewer)
#   +Y = width (left/right across the front)
#   +Z = height (object grounded at z = 0)
#
# X layout (back -> front):
#   cabinet shell body      x in [-0.350, 0.000]   (hollow box, open at front)
#   trim frame              x in [-0.002, 0.023]   (brushed stainless surround)
#   control strip           x in [-0.002, 0.012]   (recessed dark panel, top of opening)
#   door panel (closed)     x in [ 0.001, 0.021]   (recessed 2 mm behind trim front)
#   knobs                   protrude to x ~ 0.0335 (proud of the trim front)
#
# Articulations (per joint plan):
#   door_hinge   REVOLUTE   axis +Y along the door's bottom front edge;
#                           q=0 closed (vertical), q=+pi/2 dropped flat (horizontal)
#   knob_i_spin  CONTINUOUS axis +X (normal to the front face), 6 knobs in a row

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---------------------------------------------------------------------------
# Dimensions
# ---------------------------------------------------------------------------

# Trim frame (outer face plate, grounded at z=0).
TRIM_W = 0.60
TRIM_H = 0.45
TRIM_BACK_X = -0.002
TRIM_FRONT_X = 0.023
TRIM_T = TRIM_FRONT_X - TRIM_BACK_X  # 0.025

# Frame opening (centered on the trim).
OPEN_W = 0.50
OPEN_H = 0.37
OPEN_Z0 = (TRIM_H - OPEN_H) / 2.0  # 0.04
OPEN_Z1 = OPEN_Z0 + OPEN_H  # 0.41

# Cabinet shell behind the trim (hollow box, open front).
BODY_D = 0.35  # depth, x in [-0.350, 0]
BODY_W = 0.56
BODY_Z0 = 0.02
BODY_Z1 = 0.43
WALL_T = 0.02

# Fixed control strip across the top of the opening.
STRIP_W = 0.52  # extends past the opening so it laps the shell side walls
STRIP_Z0 = 0.34
STRIP_Z1 = OPEN_Z1  # 0.41
STRIP_BACK_X = -0.002
STRIP_FRONT_X = 0.012

# Drop-down door (fills the opening below the control strip, 2 mm clearances).
DOOR_W = 0.496
DOOR_Z0 = OPEN_Z0 + 0.002  # 0.042 (hinge height)
DOOR_Z1 = STRIP_Z0 - 0.002  # 0.338
DOOR_H = DOOR_Z1 - DOOR_Z0  # 0.296
DOOR_T = 0.020
DOOR_BACK_X = 0.001
DOOR_FRONT_X = DOOR_BACK_X + DOOR_T  # 0.021

# Frosted window (door-local frame: hinge at origin, panel x in [-DOOR_T, 0]).
WIN_W = 0.34
WIN_H = 0.13
WIN_ZC = 0.160  # door-local center height

# Vent slot band along the door's top edge.
SLOT_W = 0.012
SLOT_H = 0.022
SLOT_ZC = 0.270  # door-local
SLOT_COUNT = 14
SLOT_PITCH = 0.027

# Bar handle (low on the door, two stand-off posts).
HANDLE_ZC = 0.045  # door-local
HANDLE_BAR_R = 0.010
HANDLE_BAR_LEN = 0.36
HANDLE_BAR_XC = 0.030  # door-local; bar spans x in [0.020, 0.040]
HANDLE_POST_R = 0.008
HANDLE_POST_Y = 0.14

# Rotary knobs.
KNOB_D = 0.034
KNOB_H = 0.022
KNOB_Z = 0.358
KNOB_YS = (-0.125, -0.075, -0.025, 0.025, 0.075, 0.125)
KNOB_MOUNT_X = STRIP_FRONT_X - 0.0005  # 0.5 mm seated into the strip face


# ---------------------------------------------------------------------------
# CadQuery shapes
# ---------------------------------------------------------------------------


def _cabinet_shell_shape() -> cq.Workplane:
    """Hollow cabinet box, open at the front (+X)."""
    outer = (
        cq.Workplane("XY")
        .box(BODY_D, BODY_W, BODY_Z1 - BODY_Z0)
        .translate((-BODY_D / 2.0, 0.0, (BODY_Z0 + BODY_Z1) / 2.0))
    )
    # Cavity matches the trim opening and cuts through the front face.
    cav_d = BODY_D - WALL_T + 0.01
    cavity = (
        cq.Workplane("XY")
        .box(cav_d, OPEN_W, OPEN_H)
        .translate((-(BODY_D - WALL_T) / 2.0 + 0.005, 0.0, (OPEN_Z0 + OPEN_Z1) / 2.0))
    )
    return outer.cut(cavity)


def _trim_frame_shape() -> cq.Workplane:
    """Brushed stainless face frame with the rectangular opening."""
    plate = (
        cq.Workplane("XY")
        .box(TRIM_T, TRIM_W, TRIM_H)
        .translate(((TRIM_BACK_X + TRIM_FRONT_X) / 2.0, 0.0, TRIM_H / 2.0))
    )
    opening = (
        cq.Workplane("XY")
        .box(TRIM_T + 0.02, OPEN_W, OPEN_H)
        .translate(((TRIM_BACK_X + TRIM_FRONT_X) / 2.0, 0.0, (OPEN_Z0 + OPEN_Z1) / 2.0))
    )
    return plate.cut(opening)


def _door_panel_shape() -> cq.Workplane:
    """Door panel in door-local coords: hinge frame at the front-bottom edge.

    Panel spans x in [-DOOR_T, 0], y in [+-DOOR_W/2], z in [0, DOOR_H].
    The window opening and the vent slot band are cut through.
    """
    panel = (
        cq.Workplane("XY")
        .box(DOOR_T, DOOR_W, DOOR_H)
        .translate((-DOOR_T / 2.0, 0.0, DOOR_H / 2.0))
    )
    window = (
        cq.Workplane("XY")
        .box(DOOR_T + 0.02, WIN_W, WIN_H)
        .translate((-DOOR_T / 2.0, 0.0, WIN_ZC))
    )
    panel = panel.cut(window)
    y0 = -SLOT_PITCH * (SLOT_COUNT - 1) / 2.0
    for i in range(SLOT_COUNT):
        slot = (
            cq.Workplane("XY")
            .box(DOOR_T + 0.02, SLOT_W, SLOT_H)
            .translate((-DOOR_T / 2.0, y0 + i * SLOT_PITCH, SLOT_ZC))
        )
        panel = panel.cut(slot)
    return panel


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="built_in_compact_microwave_oven")

    model.material("stainless", rgba=(0.78, 0.79, 0.80, 1.0))
    model.material("cabinet_gray", rgba=(0.66, 0.67, 0.68, 1.0))
    model.material("door_taupe", rgba=(0.47, 0.45, 0.42, 1.0))
    model.material("strip_taupe", rgba=(0.36, 0.34, 0.32, 1.0))
    model.material("display_black", rgba=(0.10, 0.10, 0.11, 1.0))
    model.material("display_glow", rgba=(0.66, 0.74, 0.70, 1.0))
    model.material("button_metal", rgba=(0.74, 0.75, 0.76, 1.0))
    model.material("knob_metal", rgba=(0.81, 0.82, 0.84, 1.0))
    model.material("pointer_dark", rgba=(0.18, 0.18, 0.20, 1.0))
    model.material("frosted_glass", rgba=(0.91, 0.92, 0.93, 1.0))
    model.material("vent_dark", rgba=(0.12, 0.12, 0.13, 1.0))
    model.material("aluminum", rgba=(0.80, 0.81, 0.83, 1.0))

    # ----- root: cabinet body (shell + trim + fixed control strip) ---------
    body = model.part("cabinet_body")
    body.visual(
        mesh_from_cadquery(_cabinet_shell_shape(), "cabinet_shell"),
        material="cabinet_gray",
        name="cabinet_shell",
    )
    body.visual(
        mesh_from_cadquery(_trim_frame_shape(), "trim_frame"),
        material="stainless",
        name="trim_frame",
    )
    body.visual(
        Box((STRIP_FRONT_X - STRIP_BACK_X, STRIP_W, STRIP_Z1 - STRIP_Z0)),
        origin=Origin(
            xyz=((STRIP_BACK_X + STRIP_FRONT_X) / 2.0, 0.0, (STRIP_Z0 + STRIP_Z1) / 2.0)
        ),
        material="strip_taupe",
        name="control_strip",
    )
    # Digital display (dark bezel + pale segment screen), slightly proud.
    body.visual(
        Box((0.005, 0.116, 0.034)),
        origin=Origin(xyz=(0.0135, -0.010, 0.393)),
        material="display_black",
        name="display_bezel",
    )
    body.visual(
        Box((0.004, 0.100, 0.024)),
        origin=Origin(xyz=(0.016, -0.010, 0.393)),
        material="display_glow",
        name="display_screen",
    )
    # Small push buttons flanking the display.
    for i, by in enumerate((-0.105, -0.085, 0.066, 0.086)):
        body.visual(
            Box((0.005, 0.013, 0.013)),
            origin=Origin(xyz=(0.0135, by, 0.393)),
            material="button_metal",
            name=f"push_button_{i}",
        )

    # ----- drop-down door ---------------------------------------------------
    # Door-local frame: origin at the hinge line (front-bottom edge of the
    # closed door). Panel extends along -X (thickness) and +Z (height).
    door = model.part("door")
    door.visual(
        mesh_from_cadquery(_door_panel_shape(), "door_panel"),
        material="door_taupe",
        name="door_panel",
    )
    # Frosted window pane, recessed behind the panel front, lapping the rim.
    door.visual(
        Box((0.006, WIN_W + 0.02, WIN_H + 0.02)),
        origin=Origin(xyz=(-0.013, 0.0, WIN_ZC)),
        material="frosted_glass",
        name="window_glass",
    )
    # Dark liner behind the vent slot band so the slots read as openings.
    door.visual(
        Box((0.005, SLOT_PITCH * (SLOT_COUNT - 1) + 0.03, SLOT_H + 0.018)),
        origin=Origin(xyz=(-0.0215, 0.0, SLOT_ZC)),
        material="vent_dark",
        name="vent_liner",
    )
    # Hinge lugs at the bottom corners: they reach below the door edge and seat
    # 1 mm into the trim opening sill (z = OPEN_Z0), carrying the door like the
    # real hinge knuckles (scoped overlap allowance + contact proof in tests).
    for i, ly in enumerate((-0.20, 0.20)):
        door.visual(
            Box((0.018, 0.030, 0.014)),
            origin=Origin(xyz=(-0.011, ly, 0.004)),
            material="strip_taupe",
            name=f"hinge_lug_{i}",
        )
    # Bar handle on two stand-off posts, low on the door.
    for i, py in enumerate((-HANDLE_POST_Y, HANDLE_POST_Y)):
        door.visual(
            Cylinder(radius=HANDLE_POST_R, length=0.034),
            origin=Origin(xyz=(0.015, py, HANDLE_ZC), rpy=(0.0, math.pi / 2.0, 0.0)),
            material="aluminum",
            name=f"handle_post_{i}",
        )
    door.visual(
        Cylinder(radius=HANDLE_BAR_R, length=HANDLE_BAR_LEN),
        origin=Origin(xyz=(HANDLE_BAR_XC, 0.0, HANDLE_ZC), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="aluminum",
        name="handle_bar",
    )
    for i, cy in enumerate((-HANDLE_BAR_LEN / 2.0, HANDLE_BAR_LEN / 2.0)):
        door.visual(
            Sphere(radius=HANDLE_BAR_R),
            origin=Origin(xyz=(HANDLE_BAR_XC, cy, HANDLE_ZC)),
            material="aluminum",
            name=f"handle_cap_{i}",
        )

    # Hinge along the door's bottom front edge. Positive q about +Y swings the
    # top of the door outward (+X) and down: 0 = closed, pi/2 = horizontal.
    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(DOOR_FRONT_X, 0.0, DOOR_Z0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.5, lower=0.0, upper=math.pi / 2.0),
    )

    # ----- rotary knobs (x6, continuous, axis normal to the front face) -----
    knob_geo = KnobGeometry(
        KNOB_D,
        KNOB_H,
        body_style="cylindrical",
        grip=KnobGrip(style="fluted", count=20, depth=0.0010),
        center=False,
    )
    for i, ky in enumerate(KNOB_YS):
        knob = model.part(f"knob_{i}")
        # Knob mesh is built along local +Z; pitch by +90 deg so it points +X.
        knob.visual(
            mesh_from_geometry(knob_geo, f"knob_cap_{i}"),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material="knob_metal",
            name="knob_cap",
        )
        # Off-axis pointer bar on the knob face (proves continuous rotation).
        knob.visual(
            Box((0.005, 0.004, 0.009)),
            origin=Origin(xyz=(KNOB_H - 0.0005, 0.0, 0.0105)),
            material="pointer_dark",
            name="knob_pointer",
        )
        model.articulation(
            f"knob_{i}_spin",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=knob,
            origin=Origin(xyz=(KNOB_MOUNT_X, ky, KNOB_Z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=6.0),
        )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("cabinet_body")
    door = object_model.get_part("door")
    hinge = object_model.get_articulation("door_hinge")
    knobs = [object_model.get_part(f"knob_{i}") for i in range(6)]
    knob_joints = [object_model.get_articulation(f"knob_{i}_spin") for i in range(6)]

    # Knob caps intentionally seat 0.5 mm into the control strip face
    # (captured-shaft mount on the fixed control strip).
    for knob in knobs:
        ctx.allow_overlap(
            body,
            knob,
            elem_a="control_strip",
            elem_b="knob_cap",
            reason="Knob cap seats over its shaft boss, 0.5 mm into the control strip face.",
        )
    # Hinge knuckle lugs intentionally seat 1 mm into the trim opening sill,
    # standing in for the captured hinge pin of the drop-down door.
    for i in range(2):
        ctx.allow_overlap(
            door,
            body,
            elem_a=f"hinge_lug_{i}",
            elem_b="trim_frame",
            reason="Hinge knuckle lug seats 1 mm into the trim sill as the captured drop-down hinge mount.",
        )

    # --- overall scale and grounding ---------------------------------------
    body_aabb = ctx.part_world_aabb(body)
    trim_aabb = ctx.part_element_world_aabb(body, elem="trim_frame")
    ctx.check(
        "object grounded at z=0",
        body_aabb is not None and abs(body_aabb[0][2]) < 1e-4,
        details=f"body aabb={body_aabb}",
    )
    ctx.check(
        "trim frame is ~0.60 m wide x 0.45 m tall",
        trim_aabb is not None
        and abs((trim_aabb[1][1] - trim_aabb[0][1]) - TRIM_W) < 0.005
        and abs((trim_aabb[1][2] - trim_aabb[0][2]) - TRIM_H) < 0.005,
        details=f"trim aabb={trim_aabb}",
    )
    ctx.check(
        "body extends ~0.35 m deep behind the trim",
        body_aabb is not None and abs(body_aabb[0][0] + BODY_D) < 0.005,
        details=f"body aabb={body_aabb}",
    )

    # --- closed door seating -------------------------------------------------
    door_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    ctx.check(
        "closed door panel sits recessed behind the trim front",
        door_aabb is not None
        and trim_aabb is not None
        and door_aabb[1][0] < trim_aabb[1][0] - 0.0005,
        details=f"door max x={door_aabb}, trim max x={trim_aabb}",
    )
    ctx.expect_within(
        door,
        body,
        axes="yz",
        inner_elem="door_panel",
        outer_elem="trim_frame",
        margin=0.001,
        name="closed door panel stays inside the trim footprint",
    )
    ctx.expect_overlap(
        door,
        body,
        axes="yz",
        min_overlap=0.25,
        name="door covers most of the front opening",
    )
    for i in range(2):
        ctx.expect_contact(
            door,
            body,
            elem_a=f"hinge_lug_{i}",
            elem_b="trim_frame",
            name=f"hinge lug {i} seats on the trim opening sill",
        )

    # --- window and handle layout on the door --------------------------------
    glass_aabb = ctx.part_element_world_aabb(door, elem="window_glass")
    bar_aabb = ctx.part_element_world_aabb(door, elem="handle_bar")
    ctx.check(
        "frosted window pane is recessed behind the door front",
        glass_aabb is not None
        and door_aabb is not None
        and glass_aabb[1][0] < door_aabb[1][0] - 0.005,
        details=f"glass={glass_aabb}, door={door_aabb}",
    )
    ctx.check(
        "bar handle is mounted low, below the window",
        bar_aabb is not None
        and glass_aabb is not None
        and (bar_aabb[0][2] + bar_aabb[1][2]) / 2.0 < glass_aabb[0][2],
        details=f"bar={bar_aabb}, glass={glass_aabb}",
    )
    ctx.check(
        "bar handle protrudes in front of the door face",
        bar_aabb is not None and door_aabb is not None and bar_aabb[1][0] > door_aabb[1][0] + 0.01,
        details=f"bar={bar_aabb}, door={door_aabb}",
    )
    vent_aabb = ctx.part_element_world_aabb(door, elem="vent_liner")
    ctx.check(
        "vent slot band runs along the door's top edge",
        vent_aabb is not None and vent_aabb[0][2] > DOOR_Z0 + 0.20,
        details=f"vent liner={vent_aabb}",
    )

    # --- door hinge: drop-down about the bottom edge -------------------------
    limits = hinge.motion_limits
    ctx.check(
        "door hinge is revolute, 0..~90 deg",
        hinge.articulation_type == ArticulationType.REVOLUTE
        and limits is not None
        and limits.lower == 0.0
        and abs(limits.upper - math.pi / 2.0) < 1e-6,
        details=f"type={hinge.articulation_type}, limits={limits}",
    )
    ctx.check(
        "hinge axis is horizontal along the width",
        tuple(hinge.axis) == (0.0, 1.0, 0.0),
        details=f"axis={hinge.axis}",
    )
    with ctx.pose({hinge: math.pi / 2.0}):
        open_aabb = ctx.part_world_aabb(door)
        ctx.check(
            "open door lies horizontal, extending forward",
            open_aabb is not None and open_aabb[1][0] > 0.28 and open_aabb[1][2] < 0.12,
            details=f"open door aabb={open_aabb}",
        )
        ctx.check(
            "open door (and hanging handle) stays above the floor",
            open_aabb is not None and open_aabb[0][2] > 0.0,
            details=f"open door aabb={open_aabb}",
        )

    # --- knobs ----------------------------------------------------------------
    ctx.check(
        "exactly six rotary knobs on the control strip",
        len(knobs) == 6 and len(knob_joints) == 6,
        details=f"knobs={len(knobs)}, joints={len(knob_joints)}",
    )
    # Verify even horizontal spacing by checking Y-center distances are uniform.
    knob_y_positions = [
        ctx.part_world_position(k)[1] for k in knobs if ctx.part_world_position(k) is not None
    ]
    if len(knob_y_positions) == 6:
        spacings = [
            abs(knob_y_positions[i + 1] - knob_y_positions[i])
            for i in range(len(knob_y_positions) - 1)
        ]
        ctx.check(
            "knobs are evenly spaced horizontally",
            all(abs(s - spacings[0]) < 0.002 for s in spacings),
            details=f"spacings={spacings}",
        )
    for i, (knob, joint) in enumerate(zip(knobs, knob_joints)):
        ctx.check(
            f"knob_{i} joint is continuous about +X (face normal)",
            joint.articulation_type == ArticulationType.CONTINUOUS
            and tuple(joint.axis) == (1.0, 0.0, 0.0),
            details=f"type={joint.articulation_type}, axis={joint.axis}",
        )
        ctx.expect_contact(
            knob,
            body,
            elem_a="knob_cap",
            elem_b="control_strip",
            name=f"knob_{i} cap is seated on the control strip",
        )
    knob0_aabb = ctx.part_world_aabb(knobs[0])
    ctx.check(
        "knobs protrude in front of the trim frame",
        knob0_aabb is not None and trim_aabb is not None and knob0_aabb[1][0] > trim_aabb[1][0],
        details=f"knob={knob0_aabb}, trim={trim_aabb}",
    )
    ptr_rest = ctx.part_element_world_aabb(knobs[0], elem="knob_pointer")
    with ctx.pose({knob_joints[0]: math.pi}):
        ptr_flip = ctx.part_element_world_aabb(knobs[0], elem="knob_pointer")
    ctx.check(
        "off-axis pointer swings to the opposite side after a half turn",
        ptr_rest is not None
        and ptr_flip is not None
        and ((ptr_flip[0][2] + ptr_flip[1][2]) - (ptr_rest[0][2] + ptr_rest[1][2])) / 2.0 < -0.015,
        details=f"rest={ptr_rest}, half-turn={ptr_flip}",
    )

    return ctx.report()


object_model = build_object_model()
