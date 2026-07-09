from __future__ import annotations

# Front-swing-door public street trash can (Trashcan2 family fork).
#
# Real object: a cylindrical black plastic public trash can with a solid top
# cap and a tall rectangular front hatch. The hatch is closed by a curved door
# panel hinged along its TOP edge; a user lifts the door outward and upward to
# drop trash through the front opening. The body wall behind the door is a real
# open void (the front wall is cut away), not a sealed panel.
#
# Coordinate convention:
#   +Z is up; the body floor sits at z = 0.
#   +X is "front" (the door side).
#   The hinge axis is horizontal along Y (left-right across the front face).
#
# Root structure: the body is the root. The solid top cap, hinge boss, and
# cap accent ring are inline visuals on the body. Body-side hinge knuckles and
# door frame rivets are inline visuals emitted via for-i-in-range loops.
# The door is a separate child part with integrated knuckles and handle,
# connected by a single REVOLUTE articulation.
#
# Articulation:
#   body_to_door : REVOLUTE, axis along -Y so positive q swings the door
#                  outward (+X) and upward. Limits 0 .. ~1.75 rad (~100 deg).

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---- key dimensions (meters) -------------------------------------------------
R_BOTTOM = 0.120            # body radius at the floor
R_TOP = 0.140              # body radius at the mouth (gentle outward taper)
BODY_H = 0.500             # body height (top rim z)
WALL_T = 0.004             # wall thickness
FLOOR_T = 0.008            # floor disc thickness
CAP_T = 0.010              # solid top cap thickness

# Door opening (rectangular hatch in the upper front wall)
DOOR_W = 0.140             # hatch width (Y extent)
DOOR_H = 0.250             # hatch height (Z extent)
DOOR_T = 0.006             # door panel thickness
DOOR_TOP_Z = BODY_H - CAP_T - 0.015   # top of opening, just below the cap rim
DOOR_BOTTOM_Z = DOOR_TOP_Z - DOOR_H   # bottom of opening

# Hinge line
HINGE_Z = DOOR_TOP_Z       # hinge axis at the top edge of the opening

# Hinge parameters
N_KNUCKLES = 5
KNUCKLE_R = 0.005
KNUCKLE_PITCH = DOOR_W / N_KNUCKLES
KNUCKLE_LEN = KNUCKLE_PITCH * 0.75

# Hinge boss (thickened wall section above cutout for knuckle mounting)
BOSS_H = 0.012             # boss height above hinge line
BOSS_EXTRA_R = 0.004       # boss protrusion beyond outer wall


def _radius_at_z(z: float) -> float:
    """Outer wall radius of the tapered body at height z."""
    return R_BOTTOM + (R_TOP - R_BOTTOM) * (z / BODY_H)


HINGE_R = _radius_at_z(HINGE_Z)


# ---- shared geometry helpers ------------------------------------------------
def _knuckle_cq() -> cq.Workplane:
    """Small cylindrical hinge knuckle, axis along local Y, centered at origin."""
    cyl = (
        cq.Workplane("XY")
        .circle(KNUCKLE_R)
        .extrude(KNUCKLE_LEN)
    )
    cyl = cyl.translate((0.0, 0.0, -KNUCKLE_LEN / 2.0))
    cyl = cyl.rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
    return cyl


def _rivet_cq() -> cq.Workplane:
    """Small cylindrical rivet head, axis along local Z, base at z=0."""
    r = 0.003
    h = 0.004
    return (
        cq.Workplane("XY")
        .circle(r)
        .extrude(h)
    )


# ---- CadQuery: body shell with front cutout + solid top cap + hinge boss ----
def _body_cq() -> cq.Workplane:
    """Hollow tapered cylinder with solid top cap, rectangular front cutout,
    and a hinge boss above the cutout for knuckle mounting.
    """
    # Outer tapered solid via loft
    outer = (
        cq.Workplane("XY")
        .circle(R_BOTTOM)
        .workplane(offset=BODY_H)
        .circle(R_TOP)
        .loft()
    )

    # Inner cavity (hollow interior, leave floor)
    ir_bot = R_BOTTOM - WALL_T
    ir_top = R_TOP - WALL_T
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=FLOOR_T)
        .circle(ir_bot)
        .workplane(offset=BODY_H - FLOOR_T)
        .circle(ir_top)
        .loft()
    )
    body = outer.cut(cavity)

    # Solid top cap disc
    cap = (
        cq.Workplane("XY")
        .workplane(offset=BODY_H - CAP_T)
        .circle(R_TOP)
        .extrude(CAP_T)
    )
    body = body.union(cap)

    # Hinge boss: thickened wall section above the cutout for knuckle mounting
    boss_outer = HINGE_R + BOSS_EXTRA_R
    boss_inner = HINGE_R - WALL_T
    boss = (
        cq.Workplane("XY")
        .workplane(offset=HINGE_Z)
        .center(boss_outer / 2.0 + boss_inner / 2.0, 0.0)
        .rect(BOSS_EXTRA_R + WALL_T + 0.002, DOOR_W + 0.010)
        .extrude(BOSS_H)
    )
    body = body.union(boss)

    # Rectangular front-wall cutout (the hatch opening - a genuine void)
    mid_z = (DOOR_BOTTOM_Z + DOOR_TOP_Z) / 2.0
    mid_r = _radius_at_z(mid_z)
    cut_x_depth = WALL_T + BOSS_EXTRA_R + 0.020

    cutter = (
        cq.Workplane("XY")
        .workplane(offset=DOOR_BOTTOM_Z)
        .center(mid_r, 0.0)
        .rect(cut_x_depth, DOOR_W)
        .extrude(DOOR_H)
    )
    body = body.cut(cutter)

    return body


# ---- CadQuery: curved door panel with handle + knuckles --------------------
def _door_cq() -> cq.Workplane:
    """Curved door panel with integrated handle and door-side hinge knuckles,
    authored in door-local frame (origin at hinge, panel extends -Z).
    """
    outer_r = HINGE_R
    inner_r = HINGE_R - DOOR_T

    # Cylindrical shell section
    outer_cyl = cq.Workplane("XY").circle(outer_r).extrude(DOOR_H)
    inner_cyl = cq.Workplane("XY").circle(inner_r).extrude(DOOR_H)
    tube = outer_cyl.cut(inner_cyl)

    # Trim to keep only the +X arc within ±DOOR_W/2 in Y
    big = HINGE_R * 4.0

    back_cut = (
        cq.Workplane("XY")
        .center(-big / 2.0, 0.0)
        .rect(big, big * 2)
        .extrude(DOOR_H)
    )
    tube = tube.cut(back_cut)

    side_top = (
        cq.Workplane("XY")
        .center(0.0, DOOR_W / 2.0 + big / 2.0)
        .rect(big * 2, big)
        .extrude(DOOR_H)
    )
    tube = tube.cut(side_top)

    side_bot = (
        cq.Workplane("XY")
        .center(0.0, -(DOOR_W / 2.0 + big / 2.0))
        .rect(big * 2, big)
        .extrude(DOOR_H)
    )
    tube = tube.cut(side_bot)

    # Shift into door-local frame
    tube = tube.translate((-HINGE_R, 0.0, -DOOR_H))

    # --- Door-side hinge knuckles (odd indices: 1, 3) ---
    for i in range(N_KNUCKLES):
        if i % 2 == 1:
            y_pos = -DOOR_W / 2.0 + KNUCKLE_PITCH * (i + 0.5)
            kq = _knuckle_cq()
            kq = kq.translate((-KNUCKLE_R * 0.4, y_pos, 0.0))
            tube = tube.union(kq)

    # --- Handle bar on the door exterior ---
    # A simple horizontal bar that embeds into the door panel surface for
    # mesh connectivity. The bar center is offset outward from the panel outer
    # face, but the bar radius overlaps into the panel thickness.
    bar_r = 0.005
    bar_len = 0.070
    handle_z = -DOOR_H * 0.70
    # Place bar center just outside the outer face; bar radius overlaps inward
    handle_x = DOOR_T * 0.3

    bar = (
        cq.Workplane("XY")
        .circle(bar_r)
        .extrude(bar_len)
    )
    bar = bar.translate((0.0, 0.0, -bar_len / 2.0))
    bar = bar.rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
    bar = bar.translate((handle_x, 0.0, handle_z))

    tube = tube.union(bar)
    return tube


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="front_swing_door_trash_can")

    # Materials
    black = model.material("can_black", rgba=(0.09, 0.09, 0.10, 1.0))
    dark_grey = model.material("door_grey", rgba=(0.16, 0.16, 0.18, 1.0))
    silver = model.material("cap_silver", rgba=(0.72, 0.74, 0.77, 1.0))
    hinge_metal = model.material("hinge_steel", rgba=(0.52, 0.53, 0.55, 1.0))

    # ---- body (root) --------------------------------------------------------
    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_cq(), "body_shell"),
        material=black,
        name="body_shell",
    )
    body.inertial = Inertial.from_geometry(
        Cylinder(radius=R_TOP, length=BODY_H),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, BODY_H * 0.45)),
    )

    # Silver top cap accent ring
    cap_ring = (
        cq.Workplane("XY")
        .workplane(offset=BODY_H - 0.002)
        .circle(R_TOP + 0.004)
        .circle(R_TOP - 0.001)
        .extrude(0.005)
    )
    body.visual(
        mesh_from_cadquery(cap_ring, "cap_ring"),
        material=silver,
        name="cap_ring",
    )

    # --- Body-side hinge knuckles (even indices: 0, 2, 4) ---
    # Emitted via for-i-in-range loop with shared geometry helper
    for i in range(N_KNUCKLES):
        if i % 2 == 0:
            y_pos = -DOOR_W / 2.0 + KNUCKLE_PITCH * (i + 0.5)
            kq = _knuckle_cq()
            # Place on the hinge boss, embedded slightly into the boss surface
            kq = kq.translate((HINGE_R + BOSS_EXTRA_R - KNUCKLE_R * 0.4, y_pos,
                               HINGE_Z + BOSS_H * 0.4))
            body.visual(
                mesh_from_cadquery(kq, f"hinge_knuckle_{i}"),
                material=hinge_metal,
                name=f"hinge_knuckle_{i}",
            )

    # --- Door frame rivets around the cutout perimeter ---
    # Emitted via for-i-in-range loop with shared geometry helper.
    # Each rivet is positioned on the cylinder surface and oriented radially.
    rivet_specs = []  # list of (x, y, z, angle_from_x_axis)
    # Top edge (on the hinge boss face, above opening)
    for j in range(4):
        y_off = -DOOR_W / 2.0 + DOOR_W * (j + 0.5) / 4.0
        rz = HINGE_Z + BOSS_H * 0.8
        r_wall = _radius_at_z(rz) + BOSS_EXTRA_R
        angle = math.atan2(y_off, r_wall)
        rx = r_wall * math.cos(angle) + 0.001
        ry = y_off
        rivet_specs.append((rx, ry, rz, math.degrees(angle)))
    # Bottom edge (on the wall below the opening, center rivets only)
    for j in range(2):
        y_off = -DOOR_W / 4.0 + DOOR_W * j / 2.0
        rz = DOOR_BOTTOM_Z - 0.015
        r_wall = _radius_at_z(rz)
        angle = math.atan2(y_off, r_wall)
        rx = r_wall * math.cos(angle) + 0.001
        ry = y_off
        rivet_specs.append((rx, ry, rz, math.degrees(angle)))

    for i, (rx, ry, rz, angle_deg) in enumerate(rivet_specs):
        rv = _rivet_cq()
        # Orient rivet to face radially outward from cylinder axis
        # First rotate to point along +X, then rotate to the correct radial angle
        rv = rv.rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -90.0)
        rv = rv.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg)
        rv = rv.translate((rx, ry, rz))
        body.visual(
            mesh_from_cadquery(rv, f"rivet_{i}"),
            material=hinge_metal,
            name=f"rivet_{i}",
        )

    # ---- door (child, articulated) ------------------------------------------
    door = model.part("door")
    door.visual(
        mesh_from_cadquery(_door_cq(), "door_panel"),
        material=dark_grey,
        name="door_panel",
    )
    door.inertial = Inertial.from_geometry(
        Box((DOOR_T, DOOR_W, DOOR_H)),
        mass=0.25,
        origin=Origin(xyz=(0.0, 0.0, -DOOR_H / 2.0)),
    )

    # ---- articulation: body_to_door (REVOLUTE) ------------------------------
    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(HINGE_R, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0, velocity=2.0, lower=0.0, upper=1.75,
        ),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    door = object_model.get_part("door")
    hinge = object_model.get_articulation("body_to_door")

    # --- body sits at z=0, height & diameter match street-can proportions ----
    baabb = ctx.part_world_aabb(body)
    bext = _ext(baabb)
    ctx.check(
        "body base sits at z=0",
        baabb is not None and abs(baabb[0][2]) < 0.005,
        details=f"body_min_z={None if baabb is None else baabb[0][2]}",
    )
    ctx.check(
        "body height is ~0.50 m (street can scale)",
        abs(bext[2] - BODY_H) < 0.025,
        details=f"body_ext_z={bext[2]}",
    )
    ctx.check(
        "body diameter is ~0.24-0.30 m",
        0.22 < bext[0] < 0.34 and 0.22 < bext[1] < 0.34,
        details=f"body_ext=({bext[0]:.3f}, {bext[1]:.3f})",
    )

    # --- top cap is solid (no opening) ----------------------------------------
    ctx.check(
        "top cap rises to body height",
        baabb[1][2] >= BODY_H - 0.005,
        details=f"body_max_z={baabb[1][2]}",
    )

    # --- door panel is on the front (+X) face --------------------------------
    daabb = ctx.part_world_aabb(door)
    dext = _ext(daabb)
    ctx.check(
        "door panel height matches ~DOOR_H",
        abs(dext[2] - DOOR_H) < 0.03,
        details=f"door_ext_z={dext[2]:.3f}",
    )
    ctx.check(
        "door panel width matches ~DOOR_W",
        abs(dext[1] - DOOR_W) < 0.04,
        details=f"door_ext_y={dext[1]:.3f}",
    )
    ctx.check(
        "door is on the front (+X) face of the body",
        daabb[0][0] > 0.0,
        details=f"door_min_x={daabb[0][0]:.3f}",
    )

    # --- door is seated in the cutout at rest --------------------------------
    ctx.expect_contact(
        door, body,
        name="door panel contacts body at closed rest",
    )
    ctx.allow_overlap(
        door, body,
        reason="The door panel seats inside the body wall cutout and hinge knuckles interleave at the pivot line.",
    )

    # --- articulation: REVOLUTE with correct axis and travel -----------------
    ctx.check(
        "door joint is REVOLUTE",
        str(hinge.articulation_type).endswith("REVOLUTE"),
        details=f"type={hinge.articulation_type}",
    )
    ctx.check(
        "hinge axis is horizontal (|Y| dominant)",
        abs(hinge.axis[1]) > 0.95
        and abs(hinge.axis[0]) < 0.05
        and abs(hinge.axis[2]) < 0.05,
        details=f"axis={hinge.axis}",
    )
    lim = hinge.motion_limits
    ctx.check(
        "door swing travel well past 80 degrees (upper >= 1.4 rad)",
        lim is not None and lim.upper is not None and lim.upper >= 1.4,
        details=f"upper={lim.upper}",
    )
    ctx.check(
        "door closed at lower=0",
        lim is not None and lim.lower is not None and abs(lim.lower) < 0.05,
        details=f"lower={lim.lower}",
    )

    # --- positive q opens door outward (+X) and upward -----------------------
    with ctx.pose({hinge: 1.4}):
        open_aabb = ctx.part_world_aabb(door)

    ctx.check(
        "opening door moves bottom edge outward past body front",
        open_aabb[1][0] > baabb[1][0] + 0.02,
        details=f"body_max_x={baabb[1][0]:.3f}, open_max_x={open_aabb[1][0]:.3f}",
    )
    ctx.check(
        "opening door lifts bottom edge upward",
        open_aabb[0][2] > daabb[0][2] + 0.02,
        details=f"rest_min_z={daabb[0][2]:.3f}, open_min_z={open_aabb[0][2]:.3f}",
    )

    # --- door covers the hatch opening ----------------------------------------
    ctx.expect_overlap(
        door, body,
        axes="z",
        min_overlap=DOOR_H * 0.7,
        name="door covers the hatch opening in Z",
    )

    return ctx.report()


object_model = build_object_model()
