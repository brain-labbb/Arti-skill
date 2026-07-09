from __future__ import annotations

# White front-loading (drum) washing machine — Samsung-style.
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
#   - detergent drawer : PRISMATIC, pulls straight out toward +X (top-left front).
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

# Detergent drawer (top-left of the front face)
DRAWER_CY = 0.205
DRAWER_W = 0.150        # along Y
DRAWER_H = 0.055        # along Z
DRAWER_CZ = 0.660
DRAWER_DEPTH = 0.235    # how deep into the body (along X)
DRAWER_TRAVEL = 0.150   # prismatic travel

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

    # Detergent-drawer slot: rectangular pocket cut into the front face, top-left.
    slot = (
        cq.Workplane("XY")
        .center(FRONT_X - DRAWER_DEPTH / 2.0, DRAWER_CY)
        .rect(DRAWER_DEPTH + 0.02, DRAWER_W + 0.006)
        .extrude(DRAWER_H + 0.006, both=True)
        .translate((0.0, 0.0, DRAWER_CZ))
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


def _door_chrome_ring_mesh():
    """Thick chrome outer ring for the porthole door.

    A heavy polished-metal annular ring that frames the convex glass.  Built
    in Z-up orientation then rotated so the ring axis lies along local +X.
    """
    ring_inner_r = OPENING_R * 0.88 + 0.004   # just outside the glass rim
    ring_outer_r = OPENING_R + 0.040          # extends well past the body opening
    ring_depth = 0.055                        # chunky axial thickness

    outer = cq.Workplane("XY").circle(ring_outer_r).extrude(ring_depth)
    inner = cq.Workplane("XY").circle(ring_inner_r).extrude(ring_depth)
    ring = outer.cut(inner)

    # Rounded outer lip (fillet the two outer circular edges).
    try:
        ring = ring.edges("|Z").edges(cq.selectors.RadiusNthSelector(-1)).fillet(0.006)
    except Exception:
        pass  # fillet is cosmetic; skip if topology rejects it

    # Center axially, then rotate axis to +X.
    ring = ring.translate((0, 0, -ring_depth / 2.0))
    ring = ring.rotate((0, 0, 0), (0, 1, 0), -90)
    return mesh_from_cadquery(ring, "door_chrome_ring")


def _door_porthole_glass_mesh():
    """Deep convex porthole glass dome bulging outward along +X.

    Built as a thick spherical cap shell, limited to the correct glass radius.
    """
    glass_r = OPENING_R * 0.88          # glass disk radius (matches ring inner)
    bulge = 0.050                       # how far the dome protrudes (50 mm)
    thickness = 0.006                   # glass shell thickness
    # Sphere radius that produces this bulge over this chord.
    R = (glass_r ** 2 + bulge ** 2) / (2.0 * bulge)

    # Build outer dome cap: sphere cut at z = R - bulge, then limited to glass_r.
    outer_sphere = cq.Workplane("XY").sphere(R)
    # Cut box removes everything below z = R - bulge.
    cut_outer = (
        cq.Workplane("XY")
        .rect(R * 3, R * 3)
        .extrude(R)
        .translate((0, 0, -(R)))
    )
    outer_cap = outer_sphere.cut(cut_outer)
    # Limit to glass_r diameter using a cylinder.
    limit_cyl = cq.Workplane("XY").circle(glass_r).extrude(bulge + 0.01)
    limit_cyl = limit_cyl.translate((0, 0, R - bulge - 0.005))
    outer_cap = outer_cap.intersect(limit_cyl)

    # Build inner dome cap (for hollowing).
    R_inner = R - thickness
    inner_sphere = cq.Workplane("XY").sphere(R_inner)
    cut_inner = (
        cq.Workplane("XY")
        .rect(R_inner * 3, R_inner * 3)
        .extrude(R_inner)
        .translate((0, 0, -(R_inner)))
    )
    inner_cap = inner_sphere.cut(cut_inner)
    # Limit inner cap to same radius.
    limit_cyl_inner = cq.Workplane("XY").circle(glass_r).extrude(bulge + 0.01)
    limit_cyl_inner = limit_cyl_inner.translate((0, 0, R_inner - bulge - 0.005))
    inner_cap = inner_cap.intersect(limit_cyl_inner)

    # Dome shell = outer cap minus inner cap.
    dome = outer_cap.cut(inner_cap)

    # Add a flat annular base ring to close the bottom of the shell.
    base_ring = (
        cq.Workplane("XY")
        .workplane(offset=R - bulge)
        .circle(glass_r)
        .circle(glass_r - thickness)
        .extrude(thickness)
    )
    dome = dome.union(base_ring)

    # Shift so the dome base sits at z = 0 (cap rises to z = bulge).
    dome = dome.translate((0, 0, -(R - bulge)))
    # Rotate dome axis from +Z to +X (positive rotation takes +Z to +X).
    dome = dome.rotate((0, 0, 0), (0, 1, 0), 90)
    return mesh_from_cadquery(dome, "door_porthole_glass")


def _door_latch_mesh():
    """Visible latch block on the free edge (right / −Y side) of the door.

    A small rectangular metal block with a recessed finger notch, positioned
    on the door's free edge opposite the hinge.
    """
    block = (
        cq.Workplane("XY")
        .box(0.030, 0.040, 0.060, centered=(True, True, True))
    )
    # Finger notch cut into the front face of the block.
    notch = (
        cq.Workplane("XY")
        .center(0.016, 0.0)
        .box(0.014, 0.028, 0.030, centered=(True, True, True))
    )
    block = block.cut(notch)
    return mesh_from_cadquery(block, "door_latch")


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


def _drawer_mesh():
    """Shallow open-top detergent tray: a box shelled out from the top."""
    tray = (
        cq.Workplane("XY")
        .box(DRAWER_DEPTH, DRAWER_W, DRAWER_H, centered=(False, True, True))
    )
    cavity = (
        cq.Workplane("XY")
        .center(0.012, 0.0)
        .box(DRAWER_DEPTH - 0.006, DRAWER_W - 0.016, DRAWER_H - 0.012,
             centered=(False, True, True))
        .translate((0.0, 0.0, 0.006))
    )
    tray = tray.cut(cavity)
    # Two compartment dividers.
    for yy in (-0.025, 0.025):
        wall = (
            cq.Workplane("XY")
            .center(DRAWER_DEPTH / 2.0, yy)
            .box(0.004, 0.004, DRAWER_H - 0.014, centered=(True, True, True))
        )
        tray = tray.union(wall)
    # Front face plate (the visible drawer front, a bit taller than the tray).
    front = (
        cq.Workplane("XY")
        .center(DRAWER_DEPTH + 0.006, 0.0)
        .box(0.012, DRAWER_W + 0.010, DRAWER_H + 0.014, centered=(True, True, True))
    )
    tray = tray.union(front)
    return mesh_from_cadquery(tray, "drawer_tray")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="front_load_washer")

    white = model.material("enamel_white", rgba=(0.94, 0.94, 0.95, 1.0))
    black = model.material("bezel_black", rgba=(0.07, 0.07, 0.08, 1.0))
    glass = model.material("tinted_glass", rgba=(0.18, 0.20, 0.24, 0.45))
    steel = model.material("drum_steel", rgba=(0.72, 0.73, 0.76, 1.0))
    metal = model.material("dial_metal", rgba=(0.78, 0.79, 0.81, 1.0))
    chrome = model.material("chrome_ring", rgba=(0.85, 0.86, 0.88, 1.0))
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
    HINGE_Y = DOOR_CY + (OPENING_R + 0.040)   # left edge of chrome ring outer rim
    door = model.part("door")
    # Opening center in door-local coords (door frame is at the hinge, so
    # opening center is at -(OPENING_R+0.040) in Y).
    _door_center_y = -(OPENING_R + 0.040)

    # Thick chrome outer ring centered at the opening.
    door.visual(
        _door_chrome_ring_mesh(),
        origin=Origin(xyz=(0.0, _door_center_y, 0.0)),
        material=chrome,
        name="door_chrome_ring",
    )
    # Deep convex porthole glass dome (bulges toward +X).
    door.visual(
        _door_porthole_glass_mesh(),
        origin=Origin(xyz=(0.005, _door_center_y, 0.0)),
        material=glass,
        name="door_porthole_glass",
    )
    # Hinge leaf on the left (+Y) side.
    door.visual(
        _door_hinge_leaf_mesh(),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=black,
        name="door_hinge_leaf",
    )
    # Visible latch block on the free edge (−Y side, opposite the hinge).
    # Positioned proud of the front face so it reads as mounted on the door edge.
    door.visual(
        _door_latch_mesh(),
        origin=Origin(xyz=(0.025, 2.0 * _door_center_y + 0.010, 0.0)),
        material=metal,
        name="door_latch",
    )
    door.inertial = Inertial.from_geometry(
        Box((0.06, 2 * (OPENING_R + 0.040), 2 * (OPENING_R + 0.040))),
        mass=3.5,
        origin=Origin(xyz=(0.03, _door_center_y, 0.0)),
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

    # ================= DETERGENT DRAWER (prismatic, +X) =================
    drawer = model.part("drawer")
    drawer.visual(_drawer_mesh(), material=panel_gray, name="drawer_tray")
    # Small recessed finger pull on the drawer front.
    drawer.visual(
        Box((0.014, 0.060, 0.012)),
        origin=Origin(xyz=(DRAWER_DEPTH + 0.014, 0.0, 0.0)),
        material=black,
        name="drawer_pull",
    )
    drawer.inertial = Inertial.from_geometry(
        Box((DRAWER_DEPTH, DRAWER_W, DRAWER_H)), mass=0.6,
        origin=Origin(xyz=(DRAWER_DEPTH / 2.0, 0.0, 0.0)),
    )
    model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        # Drawer seated in its slot; pulls straight out toward +X.
        origin=Origin(xyz=(FRONT_X - DRAWER_DEPTH - 0.004, DRAWER_CY, DRAWER_CZ)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=0.3, lower=0.0, upper=DRAWER_TRAVEL
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
    drawer = object_model.get_part("drawer")

    drum_joint = object_model.get_articulation("body_to_drum")
    door_joint = object_model.get_articulation("body_to_door")
    dial_joint = object_model.get_articulation("body_to_dial")
    drawer_joint = object_model.get_articulation("body_to_drawer")

    # ---- Intentional overlaps (nested fits) ----
    ctx.allow_overlap(
        drum, body, elem_a="drum_shell", elem_b="body_shell",
        reason="Drum is nested inside the hollow tub cavity bored into the body.",
    )
    # The drum is fully contained in the body tub cavity with a radial gap
    # to the tub walls.  It is connected via the continuous spin joint.
    ctx.allow_isolated_part(
        drum,
        reason="Drum is nested inside the hollow tub cavity with a radial clearance gap; "
               "it is retained by the continuous body_to_drum spin articulation.",
    )
    ctx.allow_overlap(
        door, body, elem_a="door_chrome_ring", elem_b="body_shell",
        reason="Door chrome ring seats over the front opening recess of the body.",
    )
    ctx.allow_overlap(
        door, body, elem_a="door_hinge_leaf", elem_b="body_shell",
        reason="Door hinge leaf is mounted against the front cabinet face.",
    )
    ctx.allow_overlap(
        drawer, body, elem_a="drawer_tray", elem_b="body_shell",
        reason="Detergent drawer tray slides inside its slot cut into the body.",
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
        door, door, elem_a="door_porthole_glass", elem_b="door_chrome_ring",
        reason="Porthole glass dome seats into the chrome ring inner wall (intentional seat).",
    )

    # ---- Door mounted at the front face ----
    door_pos = ctx.part_world_position(door)
    ctx.check(
        "door mounted at the front face",
        door_pos is not None and door_pos[0] > FRONT_X - 0.05,
        details=f"door origin={door_pos}",
    )
    ctx.expect_overlap(
        door, body, axes="yz", min_overlap=0.05,
        elem_a="door_chrome_ring", elem_b="body_shell",
        name="door chrome ring covers the front opening",
    )

    # ---- Deep convex porthole glass bulges forward ----
    glass_aabb = ctx.part_element_world_aabb(door, elem="door_porthole_glass")
    ring_aabb = ctx.part_element_world_aabb(door, elem="door_chrome_ring")
    ctx.check(
        "porthole glass bulges forward beyond chrome ring base",
        glass_aabb is not None and ring_aabb is not None
        and glass_aabb[1][0] > ring_aabb[0][0] + 0.030,
        details=f"glass max-x={None if glass_aabb is None else glass_aabb[1][0]}, "
                f"ring min-x={None if ring_aabb is None else ring_aabb[0][0]}",
    )

    # ---- Chrome ring is thick (substantial axial extent) ----
    ctx.check(
        "chrome ring has substantial axial thickness",
        ring_aabb is not None and (ring_aabb[1][0] - ring_aabb[0][0]) > 0.040,
        details=f"ring x-extent={None if ring_aabb is None else ring_aabb[1][0] - ring_aabb[0][0]}",
    )

    # ---- Visible latch block on the free edge (opposite hinge) ----
    latch_aabb = ctx.part_element_world_aabb(door, elem="door_latch")
    ctx.check(
        "latch block exists on door free edge",
        latch_aabb is not None,
        details="latch element not found or has no geometry",
    )
    # Latch should be on the −Y side (opposite hinge which is at +Y).
    ctx.check(
        "latch block positioned on free edge opposite hinge",
        latch_aabb is not None and latch_aabb[0][1] < DOOR_CY,
        details=f"latch min-y={None if latch_aabb is None else latch_aabb[0][1]}, DOOR_CY={DOOR_CY}",
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

    # ---- Drawer slides straight out toward the front (+X) ----
    dr_rest = ctx.part_world_position(drawer)
    with ctx.pose({drawer_joint: DRAWER_TRAVEL}):
        dr_out = ctx.part_world_position(drawer)
    ctx.check(
        "detergent drawer slides out forward",
        dr_out[0] > dr_rest[0] + DRAWER_TRAVEL - 0.005,
        details=f"rest_x={dr_rest[0]:.4f}, out_x={dr_out[0]:.4f}",
    )
    # Drawer retained in its slot at rest (overlap with body along X).
    ctx.expect_overlap(
        drawer, body, axes="x", min_overlap=0.05,
        elem_a="drawer_tray", elem_b="body_shell",
        name="drawer retained in its slot",
    )

    return ctx.report()


object_model = build_object_model()
