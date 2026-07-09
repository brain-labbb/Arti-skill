from __future__ import annotations

"""Corner cabinet variant with angled front doors.

Variant 28: corner cabinet with pentagonal footprint, two angled front doors
(one mirrored), interior shelf boards visible through open gap, and visible
door gap seams around all moving fronts.

Overall envelope ~0.9 m wide at back, ~0.75 m deep (to front peak), ~1.8 m tall.
Brushed/tarnished raw steel. Pentagonal carcass on four splayed legs with
riveted top cap. The front face has two angled panels meeting at a central peak,
each carrying one hinged door. The left door hinges on its left edge, the right
(mirrored) door hinges on its right edge; both swing outward away from center.
"""

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Pentagon footprint (top view, metres). Back wall along X, peak at +Y.
# ---------------------------------------------------------------------------
V1 = (-0.45, -0.30)  # back left
V2 = (0.45, -0.30)   # back right
V3 = (0.45, 0.25)    # front right
V4 = (0.00, 0.45)    # front peak
V5 = (-0.45, 0.25)   # front left

# Heights
CAB_TOP = 1.80
LEG_H = 0.15
WALL_T = 0.02
CARCASS_H = CAB_TOP - LEG_H  # 1.65
CARCASS_ZC = LEG_H + CARCASS_H / 2.0

# Angled front face geometry
FACE_DX = abs(V5[0] - V4[0])  # 0.45
FACE_DY = V4[1] - V5[1]       # 0.20
FACE_LEN = math.sqrt(FACE_DX ** 2 + FACE_DY ** 2)  # ~0.492
FACE_ANGLE = math.atan2(FACE_DY, FACE_DX)  # ~0.418 rad ~24°

# Face midpoints
LF_CX = (V5[0] + V4[0]) / 2.0  # -0.225
LF_CY = (V5[1] + V4[1]) / 2.0  # 0.35
RF_CX = (V4[0] + V3[0]) / 2.0  # 0.225
RF_CY = (V4[1] + V3[1]) / 2.0  # 0.35

# Frame elements
STILE_W = 0.03
BOTTOM_RAIL_H = 0.06
TOP_RAIL_H = 0.06

# Door dimensions
DOOR_GAP = 0.005
DOOR_W = FACE_LEN - STILE_W / 2.0 - DOOR_GAP * 2.0  # ~0.467
DOOR_T = WALL_T
DOOR_Z0 = LEG_H + BOTTOM_RAIL_H + DOOR_GAP  # 0.215
DOOR_Z1 = CAB_TOP - TOP_RAIL_H - DOOR_GAP    # 1.735
DOOR_H = DOOR_Z1 - DOOR_Z0                    # 1.52
DOOR_ZC = (DOOR_Z0 + DOOR_Z1) / 2.0

DOOR_OPEN_ANGLE = math.radians(100.0)
KNOB_TURN = math.radians(90.0)

CAP_T = 0.022

# Side walls shortened to avoid door-leaf overlap at hinge junction
WALL_SHORTEN = 0.028
SIDE_DEPTH = abs(V5[1] - V1[1]) - WALL_SHORTEN  # 0.55 - 0.028 = 0.522
SIDE_YC = (V1[1] + V5[1] - WALL_SHORTEN) / 2.0  # center of shortened wall

# Shelf dimensions (rectangular shelves contacting walls)
SHELF_T = 0.015
SHELF_W = abs(V2[0] - V1[0]) - 2.0 * WALL_T  # 0.86
SHELF_D = 0.48  # from back wall inner face to near front

# Shelf heights
SHELF_HEIGHTS = [0.55, 0.95, 1.30]

# Hinge barrel
BARREL_R = 0.007
KNUCKLE_R = 0.009
BARREL_LEN = DOOR_H - 0.04


# ---------------------------------------------------------------------------
# CadQuery mesh helpers — all door meshes are pre-rotated into door part frame
# ---------------------------------------------------------------------------
def _pentagon_panel(thickness: float, mesh_name: str):
    panel = (
        cq.Workplane("XY")
        .moveTo(V1[0], V1[1])
        .lineTo(V2[0], V2[1])
        .lineTo(V3[0], V3[1])
        .lineTo(V4[0], V4[1])
        .lineTo(V5[0], V5[1])
        .close()
        .extrude(thickness)
    )
    return mesh_from_cadquery(panel, mesh_name)


def _door_leaf_mesh(sign: float, theta: float, mesh_name: str):
    """Door leaf pre-rotated by theta around Z. In the unrotated frame the
    panel extends along +X from hinge edge; sign controls inner-face side."""
    yc = -sign * DOOR_T / 2.0
    panel = (
        cq.Workplane("XY")
        .box(DOOR_W, DOOR_T, DOOR_H)
        .translate((DOOR_W / 2.0, yc, 0.0))
    )
    slot_len = min(0.22, DOOR_W * 0.55)
    cutter = (
        cq.Workplane("XZ")
        .slot2D(slot_len, 0.022, 90)
        .extrude(0.06, both=True)
        .translate((DOOR_W / 2.0, 0.0, -DOOR_H * 0.35))
    )
    panel = panel.cut(cutter)
    panel = panel.rotate((0, 0, 0), (0, 0, 1), math.degrees(theta))
    return mesh_from_cadquery(panel, mesh_name)


def _vent_backing_mesh(sign: float, theta: float, mesh_name: str):
    back_y = -sign * (DOOR_T + 0.001)
    slot_len = min(0.22, DOOR_W * 0.55)
    backing = (
        cq.Workplane("XY")
        .box(0.035, 0.004, slot_len + 0.03)
        .translate((DOOR_W / 2.0, back_y, -DOOR_H * 0.35))
    )
    backing = backing.rotate((0, 0, 0), (0, 0, 1), math.degrees(theta))
    return mesh_from_cadquery(backing, mesh_name)


def _vent_line_mesh(sign: float, theta: float, dz: float, mesh_name: str):
    line = (
        cq.Workplane("XY")
        .box(0.14, 0.003, 0.005)
        .translate((DOOR_W / 2.0, -sign * 0.001, dz))
    )
    line = line.rotate((0, 0, 0), (0, 0, 1), math.degrees(theta))
    return mesh_from_cadquery(line, mesh_name)


def _hinge_barrel_mesh(mesh_name: str):
    barrel = cq.Workplane("XY").circle(BARREL_R).extrude(BARREL_LEN / 2.0, both=True)
    ring_h = 0.05
    for zc in (-0.55, -0.25, 0.0, 0.25, 0.55):
        ring = (
            cq.Workplane("XY")
            .circle(KNUCKLE_R)
            .extrude(ring_h / 2.0, both=True)
            .translate((0.0, 0.0, zc))
        )
        barrel = barrel.union(ring)
    return mesh_from_cadquery(barrel, mesh_name)


def _leg_solid(mesh_name: str):
    leg = (
        cq.Workplane("XY")
        .center(0.025, 0.025)
        .rect(0.032, 0.032)
        .workplane(offset=LEG_H + 0.01)
        .center(-0.025, -0.025)
        .rect(0.055, 0.055)
        .loft()
    )
    return mesh_from_cadquery(leg, mesh_name)


# ---------------------------------------------------------------------------
# Build
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="corner_cabinet")

    steel_body = model.material("steel_body", rgba=(0.58, 0.59, 0.61, 1.0))
    steel_door = model.material("steel_door", rgba=(0.53, 0.54, 0.56, 1.0))
    steel_door_b = model.material("steel_door_b", rgba=(0.48, 0.49, 0.52, 1.0))
    steel_trim = model.material("steel_trim", rgba=(0.44, 0.45, 0.47, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.06, 0.06, 0.07, 1.0))
    steel_leg = model.material("steel_leg", rgba=(0.36, 0.37, 0.39, 1.0))
    steel_knob = model.material("steel_knob", rgba=(0.16, 0.16, 0.18, 1.0))
    steel_rivet = model.material("steel_rivet", rgba=(0.50, 0.51, 0.53, 1.0))
    steel_shelf = model.material("steel_shelf", rgba=(0.55, 0.56, 0.58, 1.0))
    steel_seam = model.material("steel_seam", rgba=(0.10, 0.10, 0.11, 1.0))

    # ------------------------------------------------------------------
    # Cabinet body
    # ------------------------------------------------------------------
    body = model.part("cabinet_body")

    # Back wall
    body.visual(
        Box((abs(V2[0] - V1[0]), WALL_T, CARCASS_H)),
        origin=Origin(xyz=(0.0, V1[1] + WALL_T / 2.0, CARCASS_ZC)),
        material=steel_body,
        name="back_wall",
    )
    # Left side wall (shortened to clear door leaf at hinge)
    body.visual(
        Box((WALL_T, SIDE_DEPTH, CARCASS_H)),
        origin=Origin(xyz=(V1[0] + WALL_T / 2.0, SIDE_YC, CARCASS_ZC)),
        material=steel_body,
        name="left_wall",
    )
    # Right side wall (shortened)
    body.visual(
        Box((WALL_T, SIDE_DEPTH, CARCASS_H)),
        origin=Origin(xyz=(V2[0] - WALL_T / 2.0, SIDE_YC, CARCASS_ZC)),
        material=steel_body,
        name="right_wall",
    )

    # Bottom panel (pentagonal)
    body.visual(
        _pentagon_panel(WALL_T, "bottom_panel"),
        origin=Origin(xyz=(0.0, 0.0, LEG_H)),
        material=steel_body,
        name="bottom_panel",
    )
    # Top panel
    body.visual(
        _pentagon_panel(WALL_T, "top_panel"),
        origin=Origin(xyz=(0.0, 0.0, CAB_TOP - WALL_T)),
        material=steel_body,
        name="top_panel",
    )

    # Interior shelves (rectangular, contacting back and side walls)
    shelf_y_c = V1[1] + WALL_T + SHELF_D / 2.0  # centered, touching back wall
    for i, sz in enumerate(SHELF_HEIGHTS):
        body.visual(
            Box((SHELF_W, SHELF_D, SHELF_T)),
            origin=Origin(xyz=(0.0, shelf_y_c, sz)),
            material=steel_shelf,
            name=f"shelf_{i}",
        )

    # Front frame: bottom/top rails on angled faces
    stile_h = DOOR_H + 2 * DOOR_GAP + 0.01

    # Left face rails
    body.visual(
        Box((FACE_LEN, WALL_T, BOTTOM_RAIL_H)),
        origin=Origin(
            xyz=(LF_CX, LF_CY, LEG_H + BOTTOM_RAIL_H / 2.0),
            rpy=(0.0, 0.0, FACE_ANGLE),
        ),
        material=steel_body,
        name="left_bottom_rail",
    )
    body.visual(
        Box((FACE_LEN, WALL_T, TOP_RAIL_H)),
        origin=Origin(
            xyz=(LF_CX, LF_CY, CAB_TOP - TOP_RAIL_H / 2.0),
            rpy=(0.0, 0.0, FACE_ANGLE),
        ),
        material=steel_body,
        name="left_top_rail",
    )
    # Right face rails
    body.visual(
        Box((FACE_LEN, WALL_T, BOTTOM_RAIL_H)),
        origin=Origin(
            xyz=(RF_CX, RF_CY, LEG_H + BOTTOM_RAIL_H / 2.0),
            rpy=(0.0, 0.0, -FACE_ANGLE),
        ),
        material=steel_body,
        name="right_bottom_rail",
    )
    body.visual(
        Box((FACE_LEN, WALL_T, TOP_RAIL_H)),
        origin=Origin(
            xyz=(RF_CX, RF_CY, CAB_TOP - TOP_RAIL_H / 2.0),
            rpy=(0.0, 0.0, -FACE_ANGLE),
        ),
        material=steel_body,
        name="right_top_rail",
    )

    # Center stile at V4
    body.visual(
        Box((STILE_W, WALL_T, stile_h)),
        origin=Origin(xyz=(V4[0], V4[1], DOOR_ZC)),
        material=steel_trim,
        name="center_stile",
    )

    # Corner posts at V5 and V3 (bridge gap between wall and front face)
    post_depth = WALL_SHORTEN + 0.005
    post_yc_left = V5[1] - post_depth / 2.0
    post_yc_right = V3[1] - post_depth / 2.0
    body.visual(
        Box((WALL_T + 0.004, post_depth, CARCASS_H)),
        origin=Origin(xyz=(V5[0] + WALL_T / 2.0, post_yc_left, CARCASS_ZC)),
        material=steel_body,
        name="left_corner_post",
    )
    body.visual(
        Box((WALL_T + 0.004, post_depth, CARCASS_H)),
        origin=Origin(xyz=(V3[0] - WALL_T / 2.0, post_yc_right, CARCASS_ZC)),
        material=steel_body,
        name="right_corner_post",
    )

    # Door gap seam strips (thin dark reveal lines, embedded into body rails)
    seam_w = 0.004
    seam_d = 0.008  # deep enough to embed into the body rail structure
    for z_seam, sname in [
        (LEG_H + BOTTOM_RAIL_H - seam_w / 2.0, "left_seam_bottom"),
        (CAB_TOP - TOP_RAIL_H + seam_w / 2.0, "left_seam_top"),
    ]:
        body.visual(
            Box((FACE_LEN - STILE_W - 0.02, seam_d, seam_w)),
            origin=Origin(
                xyz=(LF_CX, LF_CY + seam_d / 2.0 - 0.002, z_seam),
                rpy=(0.0, 0.0, FACE_ANGLE),
            ),
            material=steel_seam,
            name=sname,
        )
    for z_seam, sname in [
        (LEG_H + BOTTOM_RAIL_H - seam_w / 2.0, "right_seam_bottom"),
        (CAB_TOP - TOP_RAIL_H + seam_w / 2.0, "right_seam_top"),
    ]:
        body.visual(
            Box((FACE_LEN - STILE_W - 0.02, seam_d, seam_w)),
            origin=Origin(
                xyz=(RF_CX, RF_CY + seam_d / 2.0 - 0.002, z_seam),
                rpy=(0.0, 0.0, -FACE_ANGLE),
            ),
            material=steel_seam,
            name=sname,
        )
    # Vertical seam strips at center stile edges
    body.visual(
        Box((seam_w, seam_d, DOOR_H - 0.02)),
        origin=Origin(xyz=(V4[0] - STILE_W / 2.0 - seam_w / 2.0, V4[1] + 0.002, DOOR_ZC)),
        material=steel_seam,
        name="center_seam_left",
    )
    body.visual(
        Box((seam_w, seam_d, DOOR_H - 0.02)),
        origin=Origin(xyz=(V4[0] + STILE_W / 2.0 + seam_w / 2.0, V4[1] + 0.002, DOOR_ZC)),
        material=steel_seam,
        name="center_seam_right",
    )

    # Top cap (pentagonal with slight overhang)
    ovh = 0.018
    cap = (
        cq.Workplane("XY")
        .moveTo(V1[0] - ovh, V1[1] - ovh)
        .lineTo(V2[0] + ovh, V2[1] - ovh)
        .lineTo(V3[0] + ovh, V3[1] + ovh * 0.5)
        .lineTo(V4[0], V4[1] + ovh)
        .lineTo(V5[0] - ovh, V5[1] + ovh * 0.5)
        .close()
        .extrude(CAP_T)
    )
    body.visual(
        mesh_from_cadquery(cap, "top_cap"),
        origin=Origin(xyz=(0.0, 0.0, CAB_TOP - 0.001)),
        material=steel_trim,
        name="top_cap",
    )

    # Rivets along front top rails (on cap surface)
    cap_top_z = CAB_TOP - 0.001 + CAP_T
    n_riv = 6
    for face_cx, face_cy, face_ang, prefix in [
        (LF_CX, LF_CY, FACE_ANGLE, "rivet_l"),
        (RF_CX, RF_CY, -FACE_ANGLE, "rivet_r"),
    ]:
        for j in range(n_riv):
            t = (j + 0.5) / n_riv
            rx = face_cx + (FACE_LEN * 0.40) * (2.0 * t - 1.0) * math.cos(face_ang)
            ry = face_cy + (FACE_LEN * 0.40) * (2.0 * t - 1.0) * math.sin(face_ang)
            body.visual(
                Sphere(radius=0.005),
                origin=Origin(xyz=(rx, ry, cap_top_z)),
                material=steel_rivet,
                name=f"{prefix}_{j}",
            )

    # Legs at four non-peak corners
    leg_mesh = _leg_solid("splayed_leg")
    leg_specs = [
        (V1[0], V1[1], math.pi),
        (V2[0], V2[1], 3.0 * math.pi / 2.0),
        (V3[0], V3[1], 0.0),
        (V5[0], V5[1], math.pi / 2.0),
    ]
    for i, (lx, ly, yaw) in enumerate(leg_specs):
        body.visual(
            leg_mesh,
            origin=Origin(xyz=(lx, ly, 0.0), rpy=(0.0, 0.0, yaw)),
            material=steel_leg,
            name=f"leg_{i}",
        )

    # ------------------------------------------------------------------
    # Doors — all meshes pre-rotated into the door part frame
    # ------------------------------------------------------------------
    hinge_barrel_mesh = _hinge_barrel_mesh("hinge_barrel")

    door_specs = [
        # (name, hinge_xyz, theta_rad, axis_z, sign, material)
        ("door_left", (V5[0], V5[1], DOOR_ZC), FACE_ANGLE, +1.0, +1.0, steel_door),
        ("door_right", (V3[0], V3[1], DOOR_ZC), math.pi - FACE_ANGLE, -1.0, -1.0, steel_door_b),
    ]

    doors = []
    hinges = []
    for name, hinge_xyz, theta, axis_z, sign, mat in door_specs:
        door = model.part(name)
        doors.append(door)

        # Leaf (pre-rotated mesh, no visual rpy)
        door.visual(
            _door_leaf_mesh(sign, theta, f"{name}_leaf"),
            material=mat,
            name="leaf",
        )
        # Vent backing (pre-rotated)
        door.visual(
            _vent_backing_mesh(sign, theta, f"{name}_backing"),
            material=steel_dark,
            name="vent_backing",
        )
        # Vent lines (pre-rotated)
        for j, dz in enumerate((0.55, 0.57, 0.59)):
            door.visual(
                _vent_line_mesh(sign, theta, dz, f"{name}_vline_{j}"),
                material=steel_dark,
                name=f"vent_line_{j}",
            )
        # Hinge barrel at origin (symmetric around Z)
        door.visual(
            hinge_barrel_mesh,
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=steel_trim,
            name="hinge_barrel",
        )

        # Articulation
        hinge = model.articulation(
            f"{name}_hinge",
            ArticulationType.REVOLUTE,
            parent=body,
            child=door,
            origin=Origin(xyz=hinge_xyz),
            axis=(0.0, 0.0, axis_z),
            motion_limits=MotionLimits(
                effort=35.0, velocity=2.0, lower=0.0, upper=DOOR_OPEN_ANGLE
            ),
        )
        hinges.append(hinge)

        # Latch knob — on the door outer face (pre-rotated mesh)
        knob_dist = DOOR_W - 0.06  # distance from hinge along face
        knob_x_rot = knob_dist * math.cos(theta)
        knob_y_rot = knob_dist * math.sin(theta)

        knob = model.part(f"latch_{name}")
        # In the latch frame (rotated by theta), +Y is outward for sign=+1,
        # -Y is outward for sign=-1. The leaf outer surface is at y≈0.
        cyl_roll = sign * math.pi / 2.0

        # Backplate: flat disk seated on the door outer face
        knob.visual(
            Cylinder(radius=0.016, length=0.006),
            origin=Origin(xyz=(0.0, sign * 0.001, 0.0), rpy=(cyl_roll, 0.0, 0.0)),
            material=steel_knob,
            name="backplate",
        )
        # Boss: protruding cylinder from backplate
        knob.visual(
            Cylinder(radius=0.006, length=0.014),
            origin=Origin(xyz=(0.0, sign * 0.010, 0.0), rpy=(cyl_roll, 0.0, 0.0)),
            material=steel_knob,
            name="boss",
        )
        # Handle bar: thin vertical bar on the boss
        knob.visual(
            Box((0.009, 0.007, 0.030)),
            origin=Origin(xyz=(0.0, sign * 0.020, 0.0)),
            material=steel_knob,
            name="handle_bar",
        )
        # Handle tip: sphere at bottom of bar
        knob.visual(
            Sphere(radius=0.005),
            origin=Origin(xyz=(0.0, sign * 0.020, -0.017)),
            material=steel_knob,
            name="handle_tip",
        )

        model.articulation(
            f"latch_{name}_joint",
            ArticulationType.REVOLUTE,
            parent=door,
            child=knob,
            origin=Origin(
                xyz=(knob_x_rot, knob_y_rot, 0.0),
                rpy=(0.0, 0.0, theta),
            ),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=4.0, velocity=4.0, lower=0.0, upper=KNOB_TURN
            ),
        )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("cabinet_body")
    door_left = object_model.get_part("door_left")
    door_right = object_model.get_part("door_right")
    hinge_left = object_model.get_articulation("door_left_hinge")
    hinge_right = object_model.get_articulation("door_right_hinge")

    # Corner-post / door-leaf overlap: at the hinge pocket the door leaf wraps
    # around the carcass front corner post (intentional hinge-pocket embedding).
    ctx.allow_overlap(
        body, door_left,
        elem_a="left_corner_post", elem_b="leaf",
        reason="Door leaf wraps around the carcass front corner post at the hinge pocket.",
    )
    ctx.allow_overlap(
        body, door_left,
        elem_a="left_corner_post", elem_b="hinge_barrel",
        reason="Hinge barrel is captured inside the corner post pocket at the pivot.",
    )
    ctx.expect_contact(
        door_left, body,
        elem_a="hinge_barrel", elem_b="left_corner_post",
        contact_tol=0.005,
        name="left door hinge barrel contacts corner post (supported)",
    )
    ctx.allow_overlap(
        body, door_right,
        elem_a="right_corner_post", elem_b="leaf",
        reason="Door leaf wraps around the carcass front corner post at the hinge pocket.",
    )
    ctx.allow_overlap(
        body, door_right,
        elem_a="right_corner_post", elem_b="hinge_barrel",
        reason="Hinge barrel is captured inside the corner post pocket at the pivot.",
    )
    ctx.expect_contact(
        door_right, body,
        elem_a="hinge_barrel", elem_b="right_corner_post",
        contact_tol=0.005,
        name="right door hinge barrel contacts corner post (supported)",
    )

    # Latch backplate embeds into the door leaf surface (seated trim).
    ctx.allow_overlap(
        door_left, object_model.get_part("latch_door_left"),
        elem_a="leaf", elem_b="backplate",
        reason="Latch backplate is seated into the door leaf outer face (seated trim).",
    )
    ctx.expect_contact(
        object_model.get_part("latch_door_left"), door_left,
        elem_a="boss", elem_b="leaf",
        contact_tol=0.005,
        name="left latch boss near the leaf face",
    )
    ctx.allow_overlap(
        door_right, object_model.get_part("latch_door_right"),
        elem_a="leaf", elem_b="backplate",
        reason="Latch backplate is seated into the door leaf outer face (seated trim).",
    )
    ctx.expect_contact(
        object_model.get_part("latch_door_right"), door_right,
        elem_a="boss", elem_b="leaf",
        contact_tol=0.005,
        name="right latch boss protrudes from the leaf face",
    )

    # --- Overall envelope ---
    aabb = ctx.part_world_aabb(body)
    ctx.check("body has bounds", aabb is not None, details=str(aabb))
    if aabb is not None:
        (x0, y0, z0), (x1, y1, z1) = aabb
        ctx.check(
            "cabinet width ~0.9 m",
            0.85 <= (x1 - x0) <= 1.05,
            details=f"width={x1 - x0:.3f}",
        )
        ctx.check(
            "cabinet depth ~0.75 m",
            0.65 <= (y1 - y0) <= 0.85,
            details=f"depth={y1 - y0:.3f}",
        )
        ctx.check(
            "cabinet height ~1.8 m",
            1.78 <= z1 <= 1.88,
            details=f"top={z1:.3f}",
        )
        ctx.check("legs on floor", abs(z0) < 1e-5, details=f"zmin={z0:.5f}")

    # --- Door hinges: revolute, vertical axis, correct range ---
    for hname, hinge in [("left", hinge_left), ("right", hinge_right)]:
        ctx.check(
            f"door_{hname} hinge is revolute",
            hinge.articulation_type == ArticulationType.REVOLUTE,
        )
        ax = hinge.axis
        ctx.check(
            f"door_{hname} hinge axis is vertical",
            abs(ax[0]) < 1e-9 and abs(ax[1]) < 1e-9 and abs(abs(ax[2]) - 1.0) < 1e-9,
            details=str(ax),
        )
        lim = hinge.motion_limits
        ctx.check(
            f"door_{hname} opens 0..~100 deg",
            lim is not None
            and lim.lower == 0.0
            and abs(lim.upper - math.radians(100.0)) < 1e-6,
        )

    # --- Mirrored hinge positions ---
    ctx.check(
        "left hinge at left edge, right hinge at right edge (mirrored)",
        hinge_left.origin.xyz[0] < -0.40
        and hinge_right.origin.xyz[0] > 0.40,
        details=f"left_x={hinge_left.origin.xyz[0]:.3f}, right_x={hinge_right.origin.xyz[0]:.3f}",
    )

    # --- Opening pose: doors swing outward ---
    closed_left = ctx.part_world_aabb(door_left)
    closed_right = ctx.part_world_aabb(door_right)
    with ctx.pose({hinge_left: DOOR_OPEN_ANGLE, hinge_right: DOOR_OPEN_ANGLE}):
        open_left = ctx.part_world_aabb(door_left)
        open_right = ctx.part_world_aabb(door_right)

    ctx.check(
        "open doors swing outward past front peak",
        open_left is not None
        and open_right is not None
        and open_left[1][1] > 0.50
        and open_right[1][1] > 0.50,
        details=f"open_left_y={open_left[1][1] if open_left else None}, open_right_y={open_right[1][1] if open_right else None}",
    )
    ctx.check(
        "mirrored opening: left free edge goes left, right goes right",
        open_left is not None
        and open_right is not None
        and closed_left is not None
        and closed_right is not None
        and open_left[0][0] < closed_left[0][0] - 0.05
        and open_right[1][0] > closed_right[1][0] + 0.05,
        details=f"cL={closed_left}, oL={open_left}, cR={closed_right}, oR={open_right}",
    )

    # --- Shelves visible inside ---
    for i in range(3):
        shelf_aabb = ctx.part_element_world_aabb(body, elem=f"shelf_{i}")
        ctx.check(
            f"shelf_{i} exists inside cabinet",
            shelf_aabb is not None
            and shelf_aabb[0][2] > LEG_H
            and shelf_aabb[1][2] < CAB_TOP,
            details=str(shelf_aabb),
        )

    # --- Shelves contact walls (structural support) ---
    ctx.expect_contact(
        body, body,
        elem_a="shelf_1", elem_b="left_wall",
        contact_tol=0.003,
        name="shelf_1 contacts left wall",
    )
    ctx.expect_contact(
        body, body,
        elem_a="shelf_1", elem_b="back_wall",
        contact_tol=0.003,
        name="shelf_1 contacts back wall",
    )

    # --- Door gap seams visible ---
    seam_aabb = ctx.part_element_world_aabb(body, elem="left_seam_bottom")
    ctx.check(
        "left door bottom seam present",
        seam_aabb is not None and seam_aabb[0][2] > LEG_H and seam_aabb[1][2] < DOOR_Z1,
        details=str(seam_aabb),
    )
    seam_r = ctx.part_element_world_aabb(body, elem="right_seam_top")
    ctx.check(
        "right door top seam present",
        seam_r is not None and seam_r[0][2] > DOOR_Z0 and seam_r[1][2] < CAB_TOP,
        details=str(seam_r),
    )

    # --- Shelves visible through open door gap ---
    with ctx.pose({hinge_left: DOOR_OPEN_ANGLE}):
        shelf_mid = ctx.part_element_world_aabb(body, elem="shelf_1")
        door_open_aabb = ctx.part_world_aabb(door_left)
    ctx.check(
        "shelf visible through open left door gap",
        shelf_mid is not None
        and door_open_aabb is not None
        and shelf_mid[1][1] > 0.0
        and door_open_aabb[0][0] < -0.50,
        details=f"shelf={shelf_mid}, door={door_open_aabb}",
    )

    # --- Angled front faces ---
    left_leaf = ctx.part_element_world_aabb(door_left, elem="leaf")
    right_leaf = ctx.part_element_world_aabb(door_right, elem="leaf")
    ctx.check(
        "left door leaf angled (X span < full face)",
        left_leaf is not None
        and (left_leaf[1][0] - left_leaf[0][0]) < FACE_LEN + 0.02,
        details=str(left_leaf),
    )
    ctx.check(
        "right door leaf angled (X span < full face)",
        right_leaf is not None
        and (right_leaf[1][0] - right_leaf[0][0]) < FACE_LEN + 0.02,
        details=str(right_leaf),
    )

    # --- Latch knobs exist and are on the doors ---
    for name in ["door_left", "door_right"]:
        latch = object_model.get_part(f"latch_{name}")
        latch_joint = object_model.get_articulation(f"latch_{name}_joint")
        ctx.check(
            f"latch_{name} is revolute",
            latch_joint.articulation_type == ArticulationType.REVOLUTE,
        )

    return ctx.report()


object_model = build_object_model()
