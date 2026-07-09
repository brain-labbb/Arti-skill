from __future__ import annotations

# Portable toilet cabin (porta-potty).
#
# Coordinate convention:
#   - up is +Z; the cabin base sits on the ground at z = 0.
#   - the DOOR is on the front face (+X). The vent stack runs up a rear corner.
#   - plan is roughly square, centered on x = y = 0.
#
# Root structure: the cabin (floor pan + three ribbed walls + sloped translucent
# white roof + corner vent stack) is the root. The single moving part is the
# full-height front door, hinged on a VERTICAL axis (+Z) at one front corner.
#
# Visual cues matched from the reference:
#   - magenta-red molded plastic body with vertical wall ribs.
#   - a gently sloped translucent white roof (higher at the back).
#   - a black vent stack pipe up a rear corner with a cap.
#   - a full-height front door with a top vent louver, an occupancy latch
#     indicator, and a dark handle.

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
W = 1.16               # front-back footprint (x)
D = 1.16               # side-side footprint (y)
WALL_T = 0.035
FLOOR_H = 0.12         # raised floor pan / skid base
WALL_BASE = FLOOR_H
WALL_TOP = 2.18        # flat top of the side/back walls
ROOF_T = 0.06          # translucent roof cap thickness
ROOF_RISE = 0.14       # roof rises from front (+X) to back (-X)
ROOF_PITCH = math.atan2(ROOF_RISE, W)
ROOF_MID_Z = WALL_TOP + 0.02
TOTAL_H = WALL_TOP + ROOF_RISE * 0.5 + ROOF_T

RIB_COUNT = 7          # vertical ribs per side
DOOR_GAP = 0.006
OPEN_ANGLE = math.radians(100.0)


def _curved_roof(part, *, cy, span_y, mat, name_prefix):
    """Molded convex (barrel-vault) roof cap, replacing a flat tilted slab. The
    cap is a smooth circular arc curving front (+X) to back (-X), with the crown
    biased toward the back so it reads higher at the rear and rounds gently down
    to the front eave, matching the rounded plastic roofs in the reference."""
    n = 12
    a = W * 0.5
    sagitta = 0.18                       # crown rise of the arc over the chord
    radius = (a * a + sagitta * sagitta) / (2.0 * sagitta)
    xc = -0.12                           # arc crown biased toward the back (-X)
    front_dx = abs(W * 0.5 - xc)
    front_drop = radius - math.sqrt(radius * radius - front_dx * front_dx)
    z_top = WALL_TOP + front_drop + 0.04  # lift so the front eave sits at the wall top
    step = W / n
    crown_z = z_top
    for s in range(n):
        x = -W * 0.5 + (s + 0.5) * step
        dx = x - xc
        root = math.sqrt(radius * radius - dx * dx)
        z = z_top - (radius - root)
        beta = math.atan2(dx, root)      # tangent tilt about +Y, 0 at the crown
        part.visual(
            Box((step / math.cos(beta) * 1.18, span_y, ROOF_T)),
            origin=Origin(xyz=(x, cy, z + ROOF_T * 0.5 * math.cos(beta)),
                          rpy=(0.0, beta, 0.0)),
            material=mat,
            name=f"{name_prefix}_seg_{s}",
        )
        if abs(dx) < step:
            crown_z = z
    return crown_z


def _ribbed_wall(part, *, face, span, z0, z1, mat, rib_mat, name_prefix):
    """Add one ribbed wall panel on the given outer face (+/-x or +/-y)."""
    h = z1 - z0
    zc = 0.5 * (z0 + z1)
    if face in ("+x", "-x"):
        sign = 1.0 if face == "+x" else -1.0
        x = sign * (W * 0.5 - WALL_T * 0.5)
        part.visual(
            Box((WALL_T, span, h)),
            origin=Origin(xyz=(x, 0.0, zc)),
            material=mat,
            name=f"{name_prefix}_panel",
        )
        rib_x = sign * (W * 0.5 + 0.004)
        for r in range(RIB_COUNT):
            y = -span * 0.5 + span * (r + 0.5) / RIB_COUNT
            part.visual(
                Box((0.016, 0.024, h * 0.94)),
                origin=Origin(xyz=(rib_x, y, zc)),
                material=rib_mat,
                name=f"{name_prefix}_rib_{r}",
            )
    else:
        sign = 1.0 if face == "+y" else -1.0
        y = sign * (D * 0.5 - WALL_T * 0.5)
        part.visual(
            Box((span, WALL_T, h)),
            origin=Origin(xyz=(0.0, y, zc)),
            material=mat,
            name=f"{name_prefix}_panel",
        )
        rib_y = sign * (D * 0.5 + 0.004)
        for r in range(RIB_COUNT):
            x = -span * 0.5 + span * (r + 0.5) / RIB_COUNT
            part.visual(
                Box((0.024, 0.016, h * 0.94)),
                origin=Origin(xyz=(x, rib_y, zc)),
                material=rib_mat,
                name=f"{name_prefix}_rib_{r}",
            )


def _build_cabin_body(part, mats):
    """Add the static cabin shell (floor, 3 ribbed walls, front frame, roof,
    vent) to `part`, centered at x=y=0. The door is added separately."""
    magenta, magenta_dk, roof_white, dark, vent_white = mats

    # --- floor pan / skid base ---
    part.visual(
        Box((W, D, FLOOR_H)),
        origin=Origin(xyz=(0.0, 0.0, FLOOR_H * 0.5)),
        material=magenta_dk,
        name="floor_pan",
    )
    for sy in (1.0, -1.0):
        part.visual(
            Box((W + 0.02, 0.12, 0.04)),
            origin=Origin(xyz=(0.0, sy * (D * 0.5 - 0.12), 0.02)),
            material=dark,
            name=f"skid_{'p' if sy > 0 else 'm'}",
        )

    # --- three ribbed walls (+Y, -Y sides and -X back); front (+X) is the door ---
    _ribbed_wall(part, face="+y", span=D - 2 * WALL_T, z0=WALL_BASE, z1=WALL_TOP,
                 mat=magenta, rib_mat=magenta_dk, name_prefix="wall_left")
    _ribbed_wall(part, face="-y", span=D - 2 * WALL_T, z0=WALL_BASE, z1=WALL_TOP,
                 mat=magenta, rib_mat=magenta_dk, name_prefix="wall_right")
    _ribbed_wall(part, face="-x", span=D - 2 * WALL_T, z0=WALL_BASE, z1=WALL_TOP,
                 mat=magenta, rib_mat=magenta_dk, name_prefix="wall_back")

    # --- front header above the door opening (ties the two front corners) ---
    header_h = 0.12
    part.visual(
        Box((WALL_T, D - 2 * WALL_T, header_h)),
        origin=Origin(xyz=(W * 0.5 - WALL_T * 0.5, 0.0, WALL_TOP - header_h * 0.5)),
        material=magenta,
        name="front_header",
    )
    # short front corner jambs so the door has a frame to seat against
    jamb_h = WALL_TOP - WALL_BASE
    for sy in (1.0, -1.0):
        part.visual(
            Box((WALL_T, 0.05, jamb_h)),
            origin=Origin(xyz=(W * 0.5 - WALL_T * 0.5, sy * (D * 0.5 - WALL_T - 0.025), WALL_BASE + jamb_h * 0.5)),
            material=magenta,
            name=f"front_jamb_{'p' if sy > 0 else 'm'}",
        )

    # --- molded convex translucent white roof cap (curves up, higher at back) ---
    # a thin eave lip overhanging the walls, then the curved barrel-vault cap
    part.visual(
        Box((W + 0.08, D + 0.08, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, WALL_TOP - 0.005), rpy=(0.0, ROOF_PITCH, 0.0)),
        material=roof_white,
        name="roof_eave",
    )
    crown_z = _curved_roof(part, cy=0.0, span_y=D + 0.06, mat=roof_white, name_prefix="roof_cap")
    # small raised roof vent near the back, sitting on the curved cap
    part.visual(
        Box((0.20, 0.20, 0.06)),
        origin=Origin(xyz=(-W * 0.22, 0.0, crown_z + 0.05)),
        material=vent_white,
        name="roof_vent",
    )

    # --- black vent stack pipe up the rear-left corner ---
    stack_x = -W * 0.5 + 0.12
    stack_y = D * 0.5 - 0.12
    stack_top = WALL_TOP + 0.30
    part.visual(
        Cylinder(radius=0.05, length=stack_top),
        origin=Origin(xyz=(stack_x, stack_y, stack_top * 0.5)),
        material=dark,
        name="vent_stack",
    )
    part.visual(
        Cylinder(radius=0.066, length=0.04),
        origin=Origin(xyz=(stack_x, stack_y, stack_top + 0.01)),
        material=dark,
        name="vent_stack_cap",
    )


def _urinal_bowl_mesh():
    """Build a wall-mounted urinal bowl using CadQuery: a rounded rectangular
    body with a concave basin scooped from the front face. The back half of the
    body is left solid so the fixture reads as a single wall-mounted piece that
    connects cleanly to the cabin wall."""
    body_w, body_h, body_d = 0.34, 0.44, 0.18
    # Main body – rounded box, slightly deeper so the back sits firmly against
    # the wall and the front protrudes into the cabin.
    body = (
        cq.Workplane("XY")
        .box(body_w, body_h, body_d)
        .edges("|Z")
        .fillet(0.04)
        .edges(">Z or <Z")
        .fillet(0.02)
    )
    # Scoop out the concave basin from the front (+X) face.
    # The sphere is positioned so it carves only the front portion, leaving the
    # back half solid for wall-mounting connectivity.
    scoop_r = 0.18
    scoop = (
        cq.Workplane("XY")
        .transformed(offset=(body_d * 0.5 + scoop_r * 0.42, 0.0, 0.02))
        .sphere(scoop_r)
    )
    body = body.cut(scoop)
    return body


SCREEN_COUNT = 2  # two privacy screens, one on each side of the urinal


def _privacy_screen_mesh():
    """Build one privacy screen panel: a flat rectangular panel with a rounded
    top edge and a small mounting bracket foot at the bottom."""
    panel_w, panel_h, panel_t = 0.025, 0.52, 0.30
    panel = (
        cq.Workplane("XY")
        .box(panel_w, panel_h, panel_t)
        .edges("|Y")
        .fillet(0.010)
    )
    # Mounting bracket – a small L-shaped foot at the bottom
    bracket = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, -panel_h * 0.5 + 0.015, -panel_t * 0.5 + 0.015))
        .box(panel_w, 0.06, 0.06)
    )
    panel = panel.union(bracket)
    return panel


def _build_urinal_fixture(part, mats):
    """Add a wall-mounted urinal with two privacy screens to the cabin back
    wall (-X). The urinal is centered in Y and mounted at typical standing
    height. Privacy screens flank the urinal on each side."""
    _, _, _, dark, vent_white = mats
    ceramic = vent_white  # white/off-white for the ceramic fixture

    # --- Urinal bowl (CadQuery mesh) ---
    urinal_cq = _urinal_bowl_mesh()
    urinal_mesh = mesh_from_cadquery(urinal_cq, "urinal_bowl")
    # Mount on the back wall: the back wall inner face is at x = -W/2 + WALL_T.
    # Push the urinal back plate slightly into the wall so the fixture reads as
    # wall-mounted (small intentional embed for physical connectivity).
    wall_inner_x = -W * 0.5 + WALL_T
    urinal_body_half_depth = 0.09  # half of the 0.18 m body depth
    urinal_x = wall_inner_x + urinal_body_half_depth - 0.012  # 12 mm embed
    urinal_z = 0.95
    part.visual(
        urinal_mesh,
        origin=Origin(xyz=(urinal_x, 0.0, urinal_z)),
        material=ceramic,
        name="urinal_bowl",
    )

    # Flush pipe – a short cylinder emerging from the wall above the urinal and
    # angling down to meet the urinal top. Positioned so its upper end is inside
    # the wall (connectivity) and its lower end overlaps the urinal rim.
    flush_pipe_x = wall_inner_x + 0.02  # starts inside the wall
    part.visual(
        Cylinder(radius=0.018, length=0.22),
        origin=Origin(xyz=(flush_pipe_x, 0.0, urinal_z + 0.32),
                      rpy=(0.0, math.radians(12), 0.0)),
        material=dark,
        name="urinal_flush_pipe",
    )

    # --- Privacy screens (loop over SCREEN_COUNT) ---
    screen_cq = _privacy_screen_mesh()
    screen_mesh = mesh_from_cadquery(screen_cq, "privacy_screen")
    screen_y_offset = 0.26  # distance from center to each screen
    for i in range(SCREEN_COUNT):
        side = 1.0 if i == 0 else -1.0
        part.visual(
            screen_mesh,
            origin=Origin(xyz=(urinal_x + 0.06, side * screen_y_offset, urinal_z + 0.04)),
            material=ceramic,
            name=f"privacy_screen_{i}",
        )


def _build_door_leaf(part, mats):
    """Add the door leaf, hinge stile, ribs, and vent louver to `part`, authored
    with the hinge at local y=0 (the +Y front corner) and the leaf along -Y."""
    magenta, magenta_dk, roof_white, dark, vent_white = mats

    door_w = (D - 2 * WALL_T) - 2 * DOOR_GAP
    door_z0 = WALL_BASE + DOOR_GAP
    door_z1 = WALL_TOP - 0.12 - DOOR_GAP
    # Authored in a LOCAL frame whose origin is the vertical hinge at the +Y front
    # corner: the leaf face is at local x = 0, leaf along -Y. The joint origin
    # places this hinge at the real front-face corner so the leaf swings about its
    # own edge (it stays carried by the cabin) instead of about the cabin center.
    door_face_x = 0.0
    leaf_cy = -door_w * 0.5
    door_h = door_z1 - door_z0
    door_zc = 0.5 * (door_z0 + door_z1)

    part.visual(
        Box((0.045, door_w, door_h)),
        origin=Origin(xyz=(door_face_x, leaf_cy, door_zc)),
        material=magenta,
        name="door_leaf",
    )
    # hinge stile reaching to the corner jamb so the door is physically carried
    part.visual(
        Cylinder(radius=0.026, length=door_h),
        origin=Origin(xyz=(door_face_x, 0.010, door_zc)),
        material=dark,
        name="hinge_stile",
    )
    # vertical ribs across the door face to match the body
    for r in range(4):
        y = leaf_cy - door_w * 0.32 + door_w * 0.64 * r / 3.0
        part.visual(
            Box((0.014, 0.022, door_h * 0.9)),
            origin=Origin(xyz=(door_face_x + 0.026, y, door_zc)),
            material=magenta_dk,
            name=f"door_rib_{r}",
        )
    # top vent louver (dark slatted strip near the top of the door)
    for s in range(4):
        part.visual(
            Box((0.018, door_w * 0.62, 0.014)),
            origin=Origin(xyz=(door_face_x + 0.028, leaf_cy, door_z1 - 0.06 - s * 0.026)),
            material=dark,
            name=f"door_vent_slat_{s}",
        )
    return door_w, door_z0, door_z1, door_face_x, leaf_cy, door_zc


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_toilet")

    magenta = model.material("cabin_magenta", rgba=(0.74, 0.10, 0.30, 1.0))
    magenta_dk = model.material("cabin_magenta_dark", rgba=(0.57, 0.07, 0.22, 1.0))
    roof_white = model.material("roof_white", rgba=(0.90, 0.90, 0.88, 0.92))
    dark = model.material("dark_plastic", rgba=(0.11, 0.11, 0.13, 1.0))
    vent_white = model.material("vent_white", rgba=(0.85, 0.85, 0.84, 1.0))
    indicator_mat = model.material("occupancy_indicator", rgba=(0.85, 0.18, 0.16, 1.0))
    mats = (magenta, magenta_dk, roof_white, dark, vent_white)

    # ===================== CABIN (root) ==================================
    cabin = model.part("cabin")
    _build_cabin_body(cabin, mats)
    _build_urinal_fixture(cabin, mats)
    cabin.inertial = Inertial.from_geometry(
        Box((W, D, TOTAL_H)),
        mass=85.0,
        origin=Origin(xyz=(0.0, 0.0, TOTAL_H * 0.45)),
    )

    # ===================== DOOR (child) ==================================
    door = model.part("door")
    door_w, door_z0, door_z1, door_face_x, leaf_cy, door_zc = _build_door_leaf(door, mats)
    # occupancy latch indicator on the latch (free, -Y) edge above mid height
    latch_y = -door_w + 0.05
    door.visual(
        Box((0.03, 0.05, 0.09)),
        origin=Origin(xyz=(door_face_x + 0.030, latch_y, door_zc + 0.18)),
        material=indicator_mat,
        name="occupancy_indicator",
    )
    # dark handle below the indicator
    door.visual(
        Box((0.05, 0.028, 0.13)),
        origin=Origin(xyz=(door_face_x + 0.038, latch_y, door_zc)),
        material=dark,
        name="door_handle",
    )
    door.inertial = Inertial.from_geometry(
        Box((0.05, door_w, door_z1 - door_z0)),
        mass=13.0,
        origin=Origin(xyz=(door_face_x, leaf_cy, door_zc)),
    )

    # ===================== ARTICULATION ==================================
    # Door swings open about a vertical hinge at the +Y front corner.
    hinge_y = D * 0.5 - WALL_T - DOOR_GAP
    hinge_x = W * 0.5 - WALL_T - 0.004
    model.articulation(
        "cabin_to_door",
        ArticulationType.REVOLUTE,
        parent=cabin,
        child=door,
        origin=Origin(xyz=(hinge_x, hinge_y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.5, lower=0.0, upper=OPEN_ANGLE),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabin = object_model.get_part("cabin")
    door = object_model.get_part("door")
    hinge = object_model.get_articulation("cabin_to_door")

    ctx.check(
        "door joint is revolute",
        str(hinge.articulation_type).endswith("REVOLUTE"),
        details=f"type={hinge.articulation_type}",
    )
    ctx.check(
        "door hinge axis is vertical (Z)",
        abs(hinge.axis[0]) < 1e-6 and abs(hinge.axis[1]) < 1e-6 and abs(abs(hinge.axis[2]) - 1.0) < 1e-6,
        details=f"axis={hinge.axis}",
    )
    lim = hinge.motion_limits
    ctx.check(
        "door closed at q=0 and opens ~100 deg",
        lim is not None and abs(lim.lower) < 1e-6 and math.radians(90) <= lim.upper <= math.radians(110),
        details=f"lower={None if lim is None else lim.lower}, upper={None if lim is None else lim.upper}",
    )

    caabb = ctx.part_world_aabb(cabin)
    ctx.check(
        "cabin base rests at z=0",
        caabb is not None and abs(caabb[0][2]) < 1e-3,
        details=f"cabin_min_z={None if caabb is None else caabb[0][2]}",
    )
    ctx.check(
        "cabin is roughly 2.2-2.6 m tall",
        caabb is not None and 2.1 < caabb[1][2] < 2.7,
        details=f"cabin_top_z={None if caabb is None else caabb[1][2]}",
    )

    roof_aabb = ctx.part_element_world_aabb(cabin, elem="roof_cap_seg_6")
    if roof_aabb is not None:
        ctx.check(
            "curved roof cap is present and elevated",
            roof_aabb[1][2] > 2.15,
            details=f"roof_top_z={roof_aabb[1][2]}",
        )

    ctx.expect_contact(
        cabin,
        door,
        elem_a="front_jamb_p",
        elem_b="hinge_stile",
        contact_tol=0.02,
        name="door hinge stile meets the corner jamb",
    )

    daabb = ctx.part_world_aabb(door)
    if daabb is not None:
        ctx.check(
            "closed door is on the front (+X) side",
            daabb[1][0] > W * 0.5 - 0.12,
            details=f"door_max_x={daabb[1][0]}",
        )
        ctx.check(
            "closed door is full height",
            (daabb[1][2] - daabb[0][2]) > 1.7,
            details=f"door_height={daabb[1][2] - daabb[0][2]}",
        )

    stack_aabb = ctx.part_element_world_aabb(cabin, elem="vent_stack")
    if stack_aabb is not None:
        ctx.check(
            "vent stack rises to roof height",
            stack_aabb[1][2] > 2.2,
            details=f"stack_top_z={stack_aabb[1][2]}",
        )

    rest = ctx.part_world_aabb(door)
    rest_max_x = rest[1][0] if rest else None
    with ctx.pose({hinge: OPEN_ANGLE}):
        oa = ctx.part_world_aabb(door)
        open_max_x = oa[1][0] if oa else None
    ctx.check(
        "opening swings the door leaf outward (+X)",
        rest_max_x is not None and open_max_x is not None and open_max_x > rest_max_x + 0.18,
        details=f"rest_max_x={rest_max_x}, open_max_x={open_max_x}",
    )

    # --- Urinal fixture checks ---
    urinal_aabb = ctx.part_element_world_aabb(cabin, elem="urinal_bowl")
    if urinal_aabb is not None:
        ctx.check(
            "urinal bowl is mounted on the back wall (-X)",
            urinal_aabb[0][0] < -W * 0.5 + 0.30,
            details=f"urinal_min_x={urinal_aabb[0][0]}",
        )
        ctx.check(
            "urinal bowl is at standing height (above floor pan)",
            urinal_aabb[0][2] > FLOOR_H + 0.40,
            details=f"urinal_min_z={urinal_aabb[0][2]}",
        )

    screen_aabb = ctx.part_element_world_aabb(cabin, elem="privacy_screen_0")
    if screen_aabb is not None:
        ctx.check(
            "privacy screen panel is present beside the urinal",
            (screen_aabb[1][2] - screen_aabb[0][2]) > 0.30,
            details=f"screen_height={screen_aabb[1][2] - screen_aabb[0][2]}",
        )

    ctx.allow_overlap(
        cabin,
        door,
        reason="The closed door seats into the front opening between the corner jambs, a small intentional seating embed at the jamb.",
    )

    return ctx.report()


object_model = build_object_model()
