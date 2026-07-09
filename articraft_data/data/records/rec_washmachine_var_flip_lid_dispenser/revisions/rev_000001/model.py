from __future__ import annotations

# White front-loading (drum) washing machine — Samsung-style (flip-lid dispenser variant).
#
# Coordinate convention (object-intrinsic frame):
#   +X = out of the FRONT face (front of the machine), -X toward the back wall.
#   +Z = up.
#   +Y = machine WIDTH; left/right span. "Left" side of the front face = +Y.
# Body: ~0.60 m wide (Y), ~0.60 m deep (X), ~0.85 m tall (Z).
#   Back face at x=0, front face at x=BODY_DEPTH; width centered on y=0;
#   floor at z=0, top at z=BODY_H.
#
# Articulated parts (children of the body unless noted):
#   - dispenser lid     : REVOLUTE, hinge on the TOP edge (Y axis) of the
#                         dispenser recess; flips forward/downward to open.
#   - control dial      : CONTINUOUS, spins about the front-facing axis (+X).
#   - door              : REVOLUTE, hinge on a VERTICAL (Z) edge on the LEFT (+Y)
#                         side, swings open ~100 deg toward front-left.
#   - drum              : CONTINUOUS, spins about the horizontal FRONT-BACK axis
#                         (+X). Recessed inside the body behind the door window.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    Box,
    Inertial,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---- Master dimensions (meters) ----
BODY_W = 0.60   # width  (Y)
BODY_D = 0.60   # depth  (X), back face at x=0, front face at x=BODY_D
BODY_H = 0.85   # height (Z)
FRONT_X = BODY_D  # front face plane

# Door / tub geometry
DOOR_CZ = 0.40          # door + drum center height
DOOR_CY = -0.02         # door center across width (slightly off toward -Y)
OPENING_R = 0.165       # radius of the circular opening cut into the front face
TUB_R = 0.205           # outer radius of the recessed tub cavity
TUB_BACK_X = 0.205      # tub cavity floor (back) plane
DRUM_R = 0.175          # drum outer radius
DRUM_FRONT_X = FRONT_X - 0.045   # drum mouth a little behind the front face
DRUM_BACK_X = TUB_BACK_X + 0.02

# Control panel band (top strip of the front face)
PANEL_Z0 = 0.715
PANEL_Z1 = BODY_H
DIAL_CY = 0.055
DIAL_CZ = 0.785

# Detergent dispenser (top-left of the front face, flip-lid tray)
DISPENSER_CY = 0.205
DISPENSER_W = 0.150        # along Y
DISPENSER_H = 0.055        # along Z
DISPENSER_CZ = 0.660
DISPENSER_DEPTH = 0.050    # shallow recess depth (along X)
DISPENSER_LID_T = 0.010    # lid panel thickness
DISPENSER_OPEN_ANGLE = math.radians(75)  # flip-lid max open angle

# Service hatch (bottom-right of the front face)
HATCH_CY = -0.18
HATCH_CZ = 0.115


def _body_solid() -> cq.Workplane:
    """White enamel cube body, with the circular door opening bored into the
    front face and a deep cylindrical tub cavity hollowed behind it."""
    # Outer box: x in [0, BODY_D], y centered, z in [0, BODY_H].
    body = (
        cq.Workplane("XY")
        .box(BODY_D, BODY_W, BODY_H, centered=(False, True, False))
    )

    # Tub cavity: a cylinder along X, open at the front, floor at TUB_BACK_X.
    tub = (
        cq.Workplane("ZY")
        .workplane(offset=-FRONT_X)  # plane normal along +X at the front face
        .center(DOOR_CZ, DOOR_CY)
        .circle(TUB_R)
        .extrude(FRONT_X - TUB_BACK_X)
    )
    body = body.cut(tub)

    # Wider, shallow door recess so the bezel seats into the front face.
    recess = (
        cq.Workplane("ZY")
        .workplane(offset=-FRONT_X)
        .center(DOOR_CZ, DOOR_CY)
        .circle(OPENING_R + 0.040)
        .extrude(0.035)
    )
    body = body.cut(recess)

    # Dispenser recess: shallow rectangular pocket cut into the front face, top-left.
    slot = (
        cq.Workplane("XY")
        .center(FRONT_X - DISPENSER_DEPTH / 2.0, DISPENSER_CY)
        .rect(DISPENSER_DEPTH + 0.002, DISPENSER_W + 0.004)
        .extrude(DISPENSER_H + 0.004, both=True)
        .translate((0.0, 0.0, DISPENSER_CZ))
    )
    body = body.cut(slot)

    return body


def _panel_inset() -> cq.Workplane:
    """Recessed control-panel face across the top strip of the front, with a
    sunken digital-display pocket to the right of the dial."""
    # A thin raised panel plate sitting just proud of the front face.
    plate = (
        cq.Workplane("XY")
        .center(FRONT_X + 0.004, DOOR_CY)
        .rect(0.012, BODY_W - 0.04)
        .extrude((PANEL_Z1 - PANEL_Z0) - 0.02, both=False)
        .translate((0.0, 0.0, PANEL_Z0 + 0.01))
    )
    # Sunken display pocket (carved back into the plate).
    disp = (
        cq.Workplane("XY")
        .center(FRONT_X + 0.0, -0.075)
        .rect(0.014, 0.150)
        .extrude(0.046, both=False)
        .translate((0.0, 0.0, DIAL_CZ - 0.023))
    )
    return plate.cut(disp)


def _display_solid() -> cq.Workplane:
    """Dark glossy digital display block sunk into the panel."""
    return (
        cq.Workplane("XY")
        .center(FRONT_X - 0.001, -0.075)
        .rect(0.010, 0.140)
        .extrude(0.040, both=False)
        .translate((0.0, 0.0, DIAL_CZ - 0.020))
    )


def _door_bezel_mesh():
    """Black door bezel: a thick ring/frame around the glass that overhangs the
    front opening. Opening lies in local XY; we orient depth along +X."""
    bez = BezelGeometry(
        (2 * OPENING_R * 0.86, 2 * OPENING_R * 0.86),
        (2 * (OPENING_R + 0.030), 2 * (OPENING_R + 0.030)),
        0.060,
        opening_shape="circle",
        outer_shape="circle",
        face=None,
    )
    # Bezel built with opening in XY and depth along +Z; rotate so depth -> +X.
    bez.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(bez, "door_bezel")


def _door_glass_mesh():
    """Tinted curved glass window: a shallow dome-ish disk bulging forward."""
    # Radius set slightly larger than the bezel opening (OPENING_R*0.86) so the
    # glass disk seats into / overlaps the bezel inner wall (no floating island).
    glass = (
        cq.Workplane("ZY")
        .center(0.0, 0.0)
        .circle(OPENING_R * 0.90)
        .extrude(0.010)
    )
    return mesh_from_cadquery(glass, "door_glass")


def _door_hinge_leaf_mesh():
    """Visible hinge leaf on the left side of the round door.

    This small vertical plate is centered on the door-local joint axis, so it
    reads as the physical pivot instead of a free-edge pull handle.
    """
    grip = (
        cq.Workplane("XY")
        .center(0.014, 0.0)
        .rect(0.028, 0.026)
        .extrude(0.120, both=True)
    )
    return mesh_from_cadquery(grip, "door_hinge_leaf")


def _drum_mesh():
    """Stainless drum: open-front cylindrical tub with a back wall and a ring of
    perforations around the wall. Axis along X."""
    outer = (
        cq.Workplane("ZY")
        .circle(DRUM_R)
        .extrude(DRUM_FRONT_X - DRUM_BACK_X)
    )
    inner = (
        cq.Workplane("ZY")
        .workplane(offset=-0.012)  # leave a back wall
        .circle(DRUM_R - 0.012)
        .extrude(DRUM_FRONT_X - DRUM_BACK_X + 0.02)
    )
    drum = outer.cut(inner)
    # A raised lifter rib inside helps it read as a real drum.
    for ang in (0.0, 2 * math.pi / 3.0, 4 * math.pi / 3.0):
        y = (DRUM_R - 0.020) * math.cos(ang)
        z = (DRUM_R - 0.020) * math.sin(ang)
        rib = (
            cq.Workplane("ZY")
            .center(z, y)
            .rect(0.018, 0.018)
            .extrude(DRUM_FRONT_X - DRUM_BACK_X - 0.03)
        )
        drum = drum.union(rib)
    return mesh_from_cadquery(drum, "drum_shell")


def _dispenser_tray_mesh():
    """Shallow compartment tray seated inside the dispenser recess.

    Built as a connected solid: a flat back plate with bottom, side, and
    divider walls rising from it.  No boolean cavity cut that could create
    disconnected islands.
    """
    # Base plate (back wall of the tray).
    base = (
        cq.Workplane("XY")
        .box(0.005, DISPENSER_W - 0.006, DISPENSER_H - 0.006,
             centered=(False, True, True))
    )
    # Collect all wall solids via union for connectivity.
    tray = base
    # Bottom wall.
    bottom = (
        cq.Workplane("XY")
        .box(DISPENSER_DEPTH, 0.005, DISPENSER_H - 0.006,
             centered=(False, True, True))
        .translate((0.0, -(DISPENSER_W - 0.006) / 2.0 + 0.0025, 0.0))
    )
    tray = tray.union(bottom)
    # Top wall.
    top_wall = (
        cq.Workplane("XY")
        .box(DISPENSER_DEPTH, 0.005, DISPENSER_H - 0.006,
             centered=(False, True, True))
        .translate((0.0, (DISPENSER_W - 0.006) / 2.0 - 0.0025, 0.0))
    )
    tray = tray.union(top_wall)
    # Floor plate.
    floor = (
        cq.Workplane("XY")
        .box(DISPENSER_DEPTH, DISPENSER_W - 0.006, 0.005,
             centered=(False, True, False))
        .translate((0.0, 0.0, -(DISPENSER_H - 0.006) / 2.0))
    )
    tray = tray.union(floor)
    # Compartment divider walls (connect base plate to floor, continuous).
    for yy in (-0.025, 0.025):
        wall = (
            cq.Workplane("XY")
            .box(DISPENSER_DEPTH - 0.006, 0.004, DISPENSER_H - 0.010,
                 centered=(False, True, True))
            .translate((0.003, yy, 0.0))
        )
        tray = tray.union(wall)
    return mesh_from_cadquery(tray, "dispenser_tray")


def _dispenser_lid_mesh():
    """Flat panel flip-lid. Part frame origin is at the hinge (top edge).

    In local coords at q=0 (closed), the lid panel hangs downward from the
    hinge: centered at (0, 0, -DISPENSER_H/2), thin in X.
    A small finger-grip tab protrudes at the bottom edge for pulling open.
    """
    panel = (
        cq.Workplane("XY")
        .box(DISPENSER_LID_T, DISPENSER_W + 0.002, DISPENSER_H + 0.002,
             centered=(True, True, True))
        .translate((0.0, 0.0, -(DISPENSER_H + 0.002) / 2.0))
    )
    # Small finger-grip tab at the bottom edge of the lid.
    grip = (
        cq.Workplane("XY")
        .box(0.014, 0.040, 0.008, centered=(True, True, True))
        .translate((0.0, 0.0, -(DISPENSER_H + 0.002) + 0.004))
    )
    lid = panel.union(grip)
    return mesh_from_cadquery(lid, "dispenser_lid")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="front_load_washer")

    white = model.material("enamel_white", rgba=(0.94, 0.94, 0.95, 1.0))
    black = model.material("bezel_black", rgba=(0.07, 0.07, 0.08, 1.0))
    glass = model.material("tinted_glass", rgba=(0.18, 0.20, 0.24, 0.45))
    steel = model.material("drum_steel", rgba=(0.72, 0.73, 0.76, 1.0))
    metal = model.material("dial_metal", rgba=(0.78, 0.79, 0.81, 1.0))
    dark = model.material("display_dark", rgba=(0.05, 0.07, 0.10, 1.0))
    panel_gray = model.material("panel_gray", rgba=(0.86, 0.86, 0.88, 1.0))

    # ================= BODY (root) =================
    body = model.part("body")
    body.visual(mesh_from_cadquery(_body_solid(), "body_shell"),
                material=white, name="body_shell")
    body.visual(mesh_from_cadquery(_panel_inset(), "panel_inset"),
                material=panel_gray, name="panel_inset")
    body.visual(mesh_from_cadquery(_display_solid(), "display"),
                material=dark, name="display")
    # Bottom-right square service hatch (a flush panel on the front face).
    body.visual(
        Box((0.010, 0.150, 0.150)),
        origin=Origin(xyz=(FRONT_X - 0.003, HATCH_CY, HATCH_CZ)),
        material=panel_gray,
        name="service_hatch",
    )
    # Four short feet at the corners.
    for fy in (-0.24, 0.24):
        for fx in (0.07, BODY_D - 0.07):
            body.visual(
                Box((0.05, 0.05, 0.02)),
                origin=Origin(xyz=(fx, fy, -0.01)),
                material=black,
                name=f"foot_{'f' if fx > 0.3 else 'r'}_{'p' if fy > 0 else 'n'}",
            )
    body.inertial = Inertial.from_geometry(
        Box((BODY_D, BODY_W, BODY_H)), mass=70.0,
        origin=Origin(xyz=(BODY_D / 2.0, 0.0, BODY_H / 2.0)),
    )

    # ================= DRUM (continuous, axis +X) =================
    drum = model.part("drum")
    drum.visual(_drum_mesh(), material=steel, name="drum_shell")
    drum.inertial = Inertial.from_geometry(
        Box((DRUM_FRONT_X - DRUM_BACK_X, 2 * DRUM_R, 2 * DRUM_R)), mass=8.0
    )
    model.articulation(
        "body_to_drum",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=drum,
        # Local drum mesh runs from x=-(front-back length) to x=0, so the
        # joint origin is the front lip on the spin axis, just behind the door.
        origin=Origin(xyz=(DRUM_FRONT_X, DOOR_CY, DOOR_CZ)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=15.0),
    )

    # ================= DOOR (revolute, vertical Z hinge on LEFT = +Y) =================
    # Door part frame origin sits ON the hinge line (left edge of the opening),
    # at the front face. The door geometry extends from the hinge across to -Y.
    HINGE_Y = DOOR_CY + (OPENING_R + 0.030)   # left edge of bezel outer rim
    door = model.part("door")
    # Bezel: ring centered at the opening center, expressed in door-local coords
    # (door frame is at the hinge, so opening center is at -(OPENING_R+0.030) in Y).
    door.visual(
        _door_bezel_mesh(),
        origin=Origin(xyz=(0.0, -(OPENING_R + 0.030), 0.0)),
        material=black,
        name="door_bezel",
    )
    door.visual(
        _door_glass_mesh(),
        origin=Origin(xyz=(0.012, -(OPENING_R + 0.030), 0.0)),
        material=glass,
        name="door_glass",
    )
    door.visual(
        _door_hinge_leaf_mesh(),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=black,
        name="door_hinge_leaf",
    )
    door.inertial = Inertial.from_geometry(
        Box((0.06, 2 * (OPENING_R + 0.030), 2 * (OPENING_R + 0.030))),
        mass=3.0,
        origin=Origin(xyz=(0.03, -(OPENING_R + 0.030), 0.0)),
    )
    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        # Hinge at the front face, on the left (+Y) edge of the opening.
        origin=Origin(xyz=(FRONT_X - 0.005, HINGE_Y, DOOR_CZ)),
        # +Z axis: positive q swings the free edge (at -Y) toward +X (front-left).
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=10.0, velocity=2.0, lower=0.0, upper=math.radians(100.0)
        ),
    )

    # ================= CONTROL DIAL (continuous, axis +X) =================
    dial = model.part("dial")
    knob = KnobGeometry(
        0.060,
        0.022,
        body_style="skirted",
        top_diameter=0.050,
        skirt=KnobSkirt(0.066, 0.005, flare=0.06, chamfer=0.0012),
        grip=KnobGrip(style="fluted", count=24, depth=0.0012),
        indicator=KnobIndicator(style="line", mode="raised", depth=0.0010),
    )
    # Knob built along local +Z; rotate so its axis points +X (out of front face).
    knob.rotate_y(math.pi / 2.0)
    dial.visual(mesh_from_geometry(knob, "dial_cap"), material=metal, name="dial_cap")
    dial.inertial = Inertial.from_geometry(
        Box((0.022, 0.066, 0.066)), mass=0.05
    )
    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        # Dial mounted on the control panel face, slightly proud of the front.
        origin=Origin(xyz=(FRONT_X - 0.001, DIAL_CY, DIAL_CZ)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=6.0),
    )

    # ---- Dispenser tray (inline on body, inside the recess) ----
    # The tray compartment is fixed to the body; shifted slightly back so the
    # base plate overlaps into the solid body shell behind the recess pocket.
    body.visual(
        _dispenser_tray_mesh(),
        origin=Origin(xyz=(FRONT_X - DISPENSER_DEPTH - 0.005, DISPENSER_CY, DISPENSER_CZ)),
        material=panel_gray,
        name="dispenser_tray",
    )

    # ================= DISPENSER LID (revolute, top-edge hinge, axis Y) =================
    # Lid part frame origin sits ON the hinge line at the top edge of the recess.
    # At q=0 the lid panel hangs downward (-Z local) to fill the recess flush.
    dispenser_lid = model.part("dispenser_lid")
    dispenser_lid.visual(
        _dispenser_lid_mesh(),
        material=panel_gray,
        name="dispenser_lid_panel",
    )
    dispenser_lid.inertial = Inertial.from_geometry(
        Box((DISPENSER_LID_T, DISPENSER_W, DISPENSER_H)), mass=0.15,
        origin=Origin(xyz=(0.0, 0.0, -DISPENSER_H / 2.0)),
    )
    model.articulation(
        "body_to_dispenser_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=dispenser_lid,
        # Hinge at the top edge of the dispenser recess on the front face.
        origin=Origin(
            xyz=(FRONT_X - 0.002, DISPENSER_CY, DISPENSER_CZ + DISPENSER_H / 2.0)
        ),
        # Axis (0, -1, 0): positive q swings the bottom edge of the lid forward (+X).
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0, velocity=2.0, lower=0.0, upper=DISPENSER_OPEN_ANGLE
        ),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def _center(aabb):
    mn, mx = aabb
    return (
        (mn[0] + mx[0]) / 2.0,
        (mn[1] + mx[1]) / 2.0,
        (mn[2] + mx[2]) / 2.0,
    )


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    drum = object_model.get_part("drum")
    door = object_model.get_part("door")
    dial = object_model.get_part("dial")
    dispenser_lid = object_model.get_part("dispenser_lid")

    drum_joint = object_model.get_articulation("body_to_drum")
    door_joint = object_model.get_articulation("body_to_door")
    dial_joint = object_model.get_articulation("body_to_dial")
    dispenser_lid_joint = object_model.get_articulation("body_to_dispenser_lid")

    # ---- Intentional overlaps (nested fits) ----
    ctx.allow_overlap(
        drum, body, elem_a="drum_shell", elem_b="body_shell",
        reason="Drum is nested inside the hollow tub cavity bored into the body.",
    )
    # Drum spins freely inside the tub without contacting the body shell.
    ctx.allow_isolated_part(
        drum,
        reason="Drum rotates freely inside the hollow tub cavity, supported by an implied rear axle bearing.",
    )
    ctx.allow_overlap(
        door, body, elem_a="door_bezel", elem_b="body_shell",
        reason="Door bezel rim seats over the front opening recess of the body.",
    )
    ctx.allow_overlap(
        door, body, elem_a="door_hinge_leaf", elem_b="body_shell",
        reason="Door hinge leaf is mounted against the front cabinet face.",
    )
    ctx.allow_overlap(
        dispenser_lid, body, elem_a="dispenser_lid_panel", elem_b="body_shell",
        reason="Dispenser lid sits flush inside the recess cut into the body.",
    )
    ctx.allow_overlap(
        dispenser_lid, body, elem_a="dispenser_lid_panel", elem_b="dispenser_tray",
        reason="Dispenser lid panel closes over the tray compartment seated in the recess.",
    )
    ctx.allow_overlap(
        dial, body, elem_a="dial_cap", elem_b="panel_inset",
        reason="Dial base seats against the control-panel inset.",
    )
    ctx.allow_overlap(
        dial, body, elem_a="dial_cap", elem_b="body_shell",
        reason="Dial shaft/back is recessed into the front face behind the panel.",
    )
    ctx.allow_overlap(
        door, door, elem_a="door_glass", elem_b="door_bezel",
        reason="Window glass disk seats into the bezel inner wall (intentional seat).",
    )

    # ---- Door bezel sits over the front opening (in front of the body) ----
    door_pos = ctx.part_world_position(door)
    ctx.check(
        "door mounted at the front face",
        door_pos is not None and door_pos[0] > FRONT_X - 0.05,
        details=f"door origin={door_pos}",
    )
    ctx.expect_overlap(
        door, body, axes="yz", min_overlap=0.05,
        elem_a="door_bezel", elem_b="body_shell",
        name="door bezel covers the front opening",
    )

    # ---- Drum sits behind the door, recessed in the body ----
    drum_aabb = ctx.part_world_aabb(drum)
    ctx.check(
        "drum recessed behind the front face",
        drum_aabb is not None and drum_aabb[1][0] <= FRONT_X - 0.015,
        details=f"drum max-x={None if drum_aabb is None else drum_aabb[1][0]}",
    )
    ctx.check(
        "drum stays fully inside the washer body",
        drum_aabb is not None
        and drum_aabb[0][0] >= TUB_BACK_X - 0.015
        and drum_aabb[1][0] <= FRONT_X - 0.015,
        details=f"drum x-span={None if drum_aabb is None else (drum_aabb[0][0], drum_aabb[1][0])}",
    )

    # ---- Drum spins about the front-back (X) axis: a lifter rib moves in YZ ----
    rib0 = _ext(ctx.part_world_aabb(drum))
    with ctx.pose({drum_joint: math.pi / 2.0}):
        rib90 = _ext(ctx.part_world_aabb(drum))
    # The X extent (axial) should be essentially unchanged when spinning about X.
    ctx.check(
        "drum spins about the front-back axis",
        abs(rib90[0] - rib0[0]) < 0.01,
        details=f"axial extent rest={rib0[0]:.4f} spun={rib90[0]:.4f}",
    )

    # ---- Dial spins (continuous about X); indicator line moves off-axis ----
    di0 = _ext(ctx.part_world_aabb(dial))
    with ctx.pose({dial_joint: math.pi / 2.0}):
        di90 = _ext(ctx.part_world_aabb(dial))
    ctx.check(
        "dial spins about the front-facing axis",
        abs(di90[0] - di0[0]) < 0.006,
        details=f"dial axial extent rest={di0[0]:.4f} spun={di90[0]:.4f}",
    )

    # ---- Door swings open on its vertical (Z) side hinge ----
    closed = ctx.part_world_aabb(door)
    with ctx.pose({door_joint: math.radians(100.0)}):
        opened = ctx.part_world_aabb(door)
    # Opening should push the door's free edge forward (greater max-x).
    ctx.check(
        "door swings forward when opened",
        opened[1][0] > closed[1][0] + 0.08,
        details=f"closed max-x={closed[1][0]:.4f}, open max-x={opened[1][0]:.4f}",
    )
    # Hinge is vertical: the door's Z extent stays about the same when swung.
    ctx.check(
        "door hinge axis is vertical (Z extent preserved)",
        abs(_ext(opened)[2] - _ext(closed)[2]) < 0.02,
        details=f"closed z-ext={_ext(closed)[2]:.4f}, open z-ext={_ext(opened)[2]:.4f}",
    )
    hinge_leaf = door.get_visual("door_hinge_leaf")
    hinge_closed = ctx.part_element_world_aabb(door, elem=hinge_leaf)
    with ctx.pose({door_joint: math.radians(100.0)}):
        hinge_open = ctx.part_element_world_aabb(door, elem=hinge_leaf)
    hinge_closed_center = _center(hinge_closed)
    hinge_open_center = _center(hinge_open)
    ctx.check(
        "visible left hinge leaf is the door pivot",
        hinge_closed_center[1] > DOOR_CY + OPENING_R
        and abs(hinge_open_center[0] - hinge_closed_center[0]) < 0.04
        and abs(hinge_open_center[1] - hinge_closed_center[1]) < 0.04,
        details=(
            f"closed hinge center={hinge_closed_center}, "
            f"open hinge center={hinge_open_center}"
        ),
    )

    # ---- Dispenser lid flips forward/downward to open ----
    lid_closed_aabb = ctx.part_world_aabb(dispenser_lid)
    with ctx.pose({dispenser_lid_joint: DISPENSER_OPEN_ANGLE}):
        lid_open_aabb = ctx.part_world_aabb(dispenser_lid)
    # The bottom edge of the lid swings forward when opened: max-X should increase.
    ctx.check(
        "dispenser lid opens forward",
        lid_open_aabb[1][0] > lid_closed_aabb[1][0] + 0.02,
        details=f"closed_max_x={lid_closed_aabb[1][0]:.4f}, open_max_x={lid_open_aabb[1][0]:.4f}",
    )
    # The hinge axis is horizontal (Y): the lid's Y extent stays about the same.
    ctx.check(
        "dispenser lid hinge axis is horizontal (Y extent preserved)",
        abs(_ext(lid_open_aabb)[1] - _ext(lid_closed_aabb)[1]) < 0.01,
        details=f"closed_y={_ext(lid_closed_aabb)[1]:.4f}, open_y={_ext(lid_open_aabb)[1]:.4f}",
    )
    # Lid seated in recess at rest (overlap with body along X).
    ctx.expect_overlap(
        dispenser_lid, body, axes="x", min_overlap=0.005,
        elem_a="dispenser_lid_panel", elem_b="body_shell",
        name="dispenser lid seated in recess",
    )

    return ctx.report()


object_model = build_object_model()
