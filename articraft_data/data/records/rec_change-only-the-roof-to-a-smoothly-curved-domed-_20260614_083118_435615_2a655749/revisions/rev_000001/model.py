from __future__ import annotations

# Portable toilet cabin (porta-potty).
#
# Coordinate convention:
#   - up is +Z; the cabin base sits on the ground at z = 0.
#   - the DOOR is on the front face (+X). The vent stack runs up a rear corner.
#   - plan is roughly square, centered on x = y = 0.
#
# Root structure: the cabin (floor pan + three ribbed walls + smooth barrel-vault
# translucent white roof + corner vent stack) is the root. The single moving part
# is the full-height front door, hinged on a VERTICAL axis (+Z) at one front corner.
#
# Visual cues matched from the reference:
#   - magenta-red molded plastic body with vertical wall ribs.
#   - a smoothly curved barrel-vault translucent white roof.
#   - a black vent stack pipe up a rear corner with a cap.
#   - a full-height front door with a top vent louver, an occupancy latch
#     indicator, and a dark handle.

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

# ---- key dimensions (meters) -------------------------------------------------
W = 1.16               # front-back footprint (x)
D = 1.16               # side-side footprint (y)
WALL_T = 0.035
FLOOR_H = 0.12         # raised floor pan / skid base
WALL_BASE = FLOOR_H
WALL_TOP = 2.18        # flat top of the side/back walls
ROOF_T = 0.04          # translucent roof shell thickness
VAULT_RISE = 0.18      # barrel-vault crown rise above the eave line
TOTAL_H = WALL_TOP + VAULT_RISE + ROOF_T

RIB_COUNT = 7          # vertical ribs per side
DOOR_GAP = 0.006
OPEN_ANGLE = math.radians(100.0)


def _barrel_vault_mesh(span_x, span_y, rise, thickness, *, arc_segments=20):
    """Build a smooth barrel-vault shell mesh. The vault is a circular arc
    curving along X (front to back), extruded along Y (side to side). The
    eaves sit at z=0 and the crown rises to z=rise. Returns a MeshGeometry
    with the vault centered at x=0, y=0, with eaves at z=0."""
    # Compute arc radius from chord (span_x) and sagitta (rise)
    half_chord = span_x * 0.5
    radius = (half_chord * half_chord + rise * rise) / (2.0 * rise)
    # Arc center is below the eave line
    center_z = -(radius - rise)
    
    # Sample the arc from -half_chord to +half_chord
    # We'll parametrize by x position and compute z from the circle equation
    arc_points = []
    for i in range(arc_segments + 1):
        t = i / arc_segments
        x = -half_chord + t * span_x
        # Circle equation: (x - 0)^2 + (z - center_z)^2 = radius^2
        # Solve for z (take the upper solution)
        z = center_z + math.sqrt(radius * radius - x * x)
        arc_points.append((x, z))
    
    # Build the outer surface (top of the vault shell)
    geom = MeshGeometry()
    y_half = span_y * 0.5
    
    # Outer surface vertices: arc_points along X, two rows at y = +/- y_half
    outer_verts = []
    for x, z in arc_points:
        v0 = geom.add_vertex(x, -y_half, z + thickness)
        v1 = geom.add_vertex(x, y_half, z + thickness)
        outer_verts.append((v0, v1))
    
    # Outer surface triangles
    for i in range(len(outer_verts) - 1):
        a0, a1 = outer_verts[i]
        b0, b1 = outer_verts[i + 1]
        geom.add_face(a0, b0, b1)
        geom.add_face(a0, b1, a1)
    
    # Inner surface vertices (underside of the shell)
    inner_verts = []
    for x, z in arc_points:
        v0 = geom.add_vertex(x, -y_half, z)
        v1 = geom.add_vertex(x, y_half, z)
        inner_verts.append((v0, v1))
    
    # Inner surface triangles (reversed winding)
    for i in range(len(inner_verts) - 1):
        a0, a1 = inner_verts[i]
        b0, b1 = inner_verts[i + 1]
        geom.add_face(a0, b1, b0)
        geom.add_face(a0, a1, b1)
    
    # End caps (front and back edges)
    # Front cap (at x = -half_chord)
    geom.add_face(outer_verts[0][0], inner_verts[0][1], inner_verts[0][0])
    geom.add_face(outer_verts[0][0], outer_verts[0][1], inner_verts[0][1])
    # Back cap (at x = +half_chord)
    geom.add_face(outer_verts[-1][0], inner_verts[-1][0], inner_verts[-1][1])
    geom.add_face(outer_verts[-1][0], inner_verts[-1][1], outer_verts[-1][1])
    
    # Side caps (along the eaves at y = +/- y_half)
    for i in range(len(outer_verts) - 1):
        # y = -y_half side
        a_out, _ = outer_verts[i]
        b_out, _ = outer_verts[i + 1]
        a_in, _ = inner_verts[i]
        b_in, _ = inner_verts[i + 1]
        geom.add_face(a_out, a_in, b_in)
        geom.add_face(a_out, b_in, b_out)
        # y = +y_half side
        _, a_out = outer_verts[i]
        _, b_out = outer_verts[i + 1]
        _, a_in = inner_verts[i]
        _, b_in = inner_verts[i + 1]
        geom.add_face(a_out, b_in, a_in)
        geom.add_face(a_out, b_out, b_in)
    
    return geom, arc_points


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

    # --- smooth barrel-vault translucent white roof cap ---
    # The vault spans the cabin footprint with a small overhang
    vault_span_x = W + 0.06  # slight front/back overhang
    vault_span_y = D + 0.06  # slight side overhang
    vault_mesh, arc_pts = _barrel_vault_mesh(
        vault_span_x, vault_span_y, VAULT_RISE, ROOF_T, arc_segments=20
    )
    # The mesh has eaves at z=0; lift so eaves sit at wall top
    vault_mesh.translate(0.0, 0.0, WALL_TOP)
    part.visual(
        mesh_from_geometry(vault_mesh, "roof_vault"),
        material=roof_white,
        name="roof_vault",
    )
    # Compute vault surface height at the vent position (x = -W*0.22) for correct seating.
    # The vault arc: radius R, center at z_center below eaves.
    _half_chord = vault_span_x * 0.5
    _R = (_half_chord * _half_chord + VAULT_RISE * VAULT_RISE) / (2.0 * VAULT_RISE)
    _zc = -(_R - VAULT_RISE)
    _vent_x = -W * 0.22
    _vent_surface_z = WALL_TOP + _zc + math.sqrt(_R * _R - _vent_x * _vent_x) + ROOF_T
    # Seat the vent box so its bottom embeds slightly into the vault shell
    vent_h = 0.06
    vent_z_center = _vent_surface_z + vent_h * 0.5 - 0.01  # 10mm embed into shell
    # small raised roof vent near the back, sitting on the curved cap
    part.visual(
        Box((0.20, 0.20, vent_h)),
        origin=Origin(xyz=(_vent_x, 0.0, vent_z_center)),
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

    roof_aabb = ctx.part_element_world_aabb(cabin, elem="roof_vault")
    if roof_aabb is not None:
        ctx.check(
            "barrel-vault roof cap is present and elevated",
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

    ctx.allow_overlap(
        cabin,
        door,
        reason="The closed door seats into the front opening between the corner jambs, a small intentional seating embed at the jamb.",
    )

    return ctx.report()


object_model = build_object_model()
