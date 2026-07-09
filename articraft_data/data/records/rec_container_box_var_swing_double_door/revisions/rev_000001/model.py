from __future__ import annotations

# Corrugated cardboard box with two front-hinged swing doors.
# Fork of the four-flap shipping box: same rectangular shell footprint,
# but the top flaps are replaced by a fixed ceiling and the front wall is
# split into two cabinet-style swing doors on vertical hinges.
#
# Frame:
#   - X is the long box axis (length ~0.28 m), X in [-0.14, 0.14]
#   - Y is the short box axis (depth ~0.24 m), Y in [-0.12, 0.12]
#   - Z is up (height ~0.22 m), floor at z=0, top rim at z=0.22
# Construction:
#   - body (root): open-front rectangular shell = floor + ceiling + back wall
#     + two side walls, visible corrugated wall thickness.
#   - two swing doors, each REVOLUTE on a vertical hinge at its side edge:
#       * door_0 (left): hinge at x=-IN_HX, y=-HY, swings outward (-Y)
#       * door_1 (right): hinge at x=+IN_HX, y=-HY, swings outward (-Y)
#     Each door panel has an integrated cylindrical pull handle on the outer
#     face near the center seam, plus a printed label.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---- master dimensions (meters) ----
LEN_X = 0.28            # outer length (long axis)
DEP_Y = 0.24           # outer depth (short axis)
HGT_Z = 0.22           # outer height
WALL = 0.006           # corrugated wall / floor / ceiling thickness

HX = LEN_X / 2.0       # 0.140
HY = DEP_Y / 2.0       # 0.120
TOP_Z = HGT_Z          # top rim plane

# inner cavity extents
IN_HX = HX - WALL      # inner half length
IN_HY = HY - WALL      # inner half depth

# door panel
DOOR_T = WALL          # door thickness matches wall
SEAM = 0.002           # center seam gap between doors (half per side)
DOOR_W = IN_HX - SEAM  # each door spans from hinge to just short of center
DOOR_H = HGT_Z - 2.0 * WALL  # door height = inner cavity height

OPEN_ANGLE = math.radians(90.0)

NUM_DOORS = 2


def _shell_solid() -> cq.Workplane:
    """Open-front box shell: floor + ceiling + back wall + two side walls.

    Built as an outer solid block with the interior cavity cut away.
    The cavity extends through the front face (-Y), leaving no front wall.
    """
    outer = cq.Workplane("XY").box(LEN_X, DEP_Y, HGT_Z, centered=(True, True, False))

    # Inner cavity: open at front (-Y), between floor and ceiling
    cavity_w = LEN_X - 2.0 * WALL       # inner width
    cavity_d = DEP_Y - WALL             # from front face to inner back wall
    cavity_h = HGT_Z - 2.0 * WALL      # between floor and ceiling
    cavity_cy = -WALL / 2.0             # Y center: (-HY + IN_HY) / 2
    cavity_cz = HGT_Z / 2.0            # Z center: (WALL + HGT_Z-WALL) / 2

    cavity = (
        cq.Workplane("XY")
        .box(cavity_w, cavity_d, cavity_h, centered=(True, True, True))
        .translate((0.0, cavity_cy, cavity_cz))
    )
    return outer.cut(cavity)


def _door_panel(side: int) -> cq.Workplane:
    """Flat door panel with integrated pull handle.

    side=-1: left door, extends +X from hinge at local x=0.
    side=+1: right door, extends -X from hinge at local x=0.
    Local origin at the hinge edge, outer face at y=0, z centered.
    """
    # Door panel: x from 0..DOOR_W (left) or mirrored; y from 0..DOOR_T; z centered
    panel = (
        cq.Workplane("XY")
        .box(DOOR_W, DOOR_T, DOOR_H, centered=(False, False, True))
    )

    # Integrated cylindrical pull handle near the free (center) edge.
    # The handle protrudes outward from the outer face (y=0 toward -Y).
    handle_r = 0.005
    handle_len = 0.012
    handle_x = DOOR_W - 0.025  # near free edge

    handle = (
        cq.Workplane("XZ")
        .workplane(offset=0.002)        # start slightly inside panel for connectivity
        .center(handle_x, 0.0)
        .circle(handle_r)
        .extrude(-(handle_len + 0.002)) # protrude outward through the face
    )
    panel = panel.union(handle)

    if side > 0:
        panel = panel.mirror("YZ")

    return panel


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cardboard_box_doors")

    kraft = model.material("kraft", rgba=(0.78, 0.62, 0.40, 1.0))
    kraft_in = model.material("kraft_inner", rgba=(0.72, 0.56, 0.36, 1.0))
    print_ink = model.material("print_ink", rgba=(0.20, 0.20, 0.22, 1.0))

    # ---- body (root): open-front shell ----
    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_shell_solid(), "shell"),
        material=kraft,
        name="shell",
    )

    body.inertial = Inertial.from_geometry(
        Box((LEN_X, DEP_Y, HGT_Z)),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.0, HGT_Z / 2.0)),
    )

    # ---- two swing doors (for-loop with shared geometry helper) ----
    # side=-1: left door, hinge at x=-IN_HX; side=+1: right door, hinge at x=+IN_HX
    sides = [-1, 1]
    for i in range(NUM_DOORS):
        side = sides[i]
        door_name = f"door_{i}"
        door = model.part(door_name)

        # Door panel with integrated handle (single connected mesh)
        door.visual(
            mesh_from_cadquery(_door_panel(side), "panel"),
            material=kraft_in,
            name="panel",
        )

        # Printed label on outer face, centered on the panel
        label_w = 0.055
        label_h = 0.032
        label_x = (DOOR_W / 2.0) if side < 0 else -(DOOR_W / 2.0)
        label_y = -0.0005  # just proud of outer face
        door.visual(
            Box((label_w, 0.001, label_h)),
            origin=Origin(xyz=(label_x, label_y, 0.0)),
            material=print_ink,
            name="label",
        )

        # Small ink icons on the label (two per door)
        icon_names = ("icon_0", "icon_1")
        icon_offsets = (-0.015, 0.015)
        for j in range(2):
            icon_x = label_x + icon_offsets[j]
            door.visual(
                Box((0.018, 0.0008, 0.022)),
                origin=Origin(xyz=(icon_x, label_y - 0.0008, 0.0)),
                material=print_ink,
                name=icon_names[j],
            )

        door.inertial = Inertial.from_geometry(
            Box((DOOR_W, DOOR_T, DOOR_H)),
            mass=0.04,
            origin=Origin(xyz=(side * (DOOR_W / 2.0), DOOR_T / 2.0, 0.0)),
        )

        # Vertical hinge at the side edge, on the front face
        hinge_x = -IN_HX if side < 0 else IN_HX
        hinge_y = -HY
        hinge_z = HGT_Z / 2.0

        # Axis chosen so positive angle swings the free edge outward (-Y):
        # Left door (extends +X from hinge): rotate around -Z to take +X toward -Y.
        # Right door (extends -X from hinge): rotate around +Z to take -X toward -Y.
        axis = (0.0, 0.0, -1.0) if side < 0 else (0.0, 0.0, 1.0)

        model.articulation(
            f"door_{i}_hinge",
            ArticulationType.REVOLUTE,
            parent=body,
            child=door,
            origin=Origin(xyz=(hinge_x, hinge_y, hinge_z)),
            axis=axis,
            motion_limits=MotionLimits(
                effort=3.0, velocity=2.0, lower=0.0, upper=OPEN_ANGLE
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    door_0 = object_model.get_part("door_0")
    door_1 = object_model.get_part("door_1")
    hinge_0 = object_model.get_articulation("door_0_hinge")
    hinge_1 = object_model.get_articulation("door_1_hinge")

    doors = [
        ("door_0", door_0, hinge_0),
        ("door_1", door_1, hinge_1),
    ]

    # --- Shell has box footprint and full height ---
    sh_aabb = ctx.part_element_world_aabb(body, elem="shell")
    sh_dx = sh_aabb[1][0] - sh_aabb[0][0]
    sh_dy = sh_aabb[1][1] - sh_aabb[0][1]
    sh_dz = sh_aabb[1][2] - sh_aabb[0][2]
    ctx.check(
        "shell has box footprint and full height",
        abs(sh_dx - LEN_X) < 0.01 and abs(sh_dy - DEP_Y) < 0.01 and abs(sh_dz - HGT_Z) < 0.01,
        details=f"shell dims=({sh_dx:.4f}, {sh_dy:.4f}, {sh_dz:.4f})",
    )

    # --- Doors cover the front opening when closed ---
    # Each door should span the inner height (from floor to ceiling)
    for name, door, _h in doors:
        door_aabb = ctx.part_world_aabb(door)
        door_dz = door_aabb[1][2] - door_aabb[0][2]
        ctx.check(
            f"{name} spans the front opening height when closed",
            abs(door_dz - DOOR_H) < 0.01,
            details=f"{name} height={door_dz:.4f}, expected={DOOR_H:.4f}",
        )

    # --- Two doors meet at center with a seam gap when closed ---
    d0_aabb = ctx.part_world_aabb(door_0)
    d1_aabb = ctx.part_world_aabb(door_1)
    # door_0 extends toward +X from left hinge; its free edge is max_x
    # door_1 extends toward -X from right hinge; its free edge is min_x
    seam_gap = d1_aabb[0][0] - d0_aabb[1][0]
    ctx.check(
        "doors have a center seam gap when closed",
        -0.005 < seam_gap < 0.02,
        details=f"seam_gap={seam_gap:.4f}",
    )

    # --- Doors swing outward when opened ---
    for name, door, hinge in doors:
        rest_aabb = ctx.part_world_aabb(door)
        rest_min_y = rest_aabb[0][1]
        with ctx.pose({hinge: OPEN_ANGLE}):
            open_aabb = ctx.part_world_aabb(door)
            open_min_y = open_aabb[0][1]
        ctx.check(
            f"{name} swings outward when opened",
            open_min_y < rest_min_y - 0.03,
            details=f"{name} rest_min_y={rest_min_y:.4f}, open_min_y={open_min_y:.4f}",
        )

    # --- Doors remain in the front plane when closed (y near -HY) ---
    for name, door, _h in doors:
        door_aabb = ctx.part_world_aabb(door)
        door_min_y = door_aabb[0][1]
        ctx.check(
            f"{name} sits at the front face when closed",
            abs(door_min_y - (-HY)) < 0.015,
            details=f"{name} min_y={door_min_y:.4f}, expected={-HY:.4f}",
        )

    # --- Doors contact the body at the hinge ---
    for name, door, _h in doors:
        ctx.allow_overlap(
            door, body, elem_a="panel", elem_b="shell",
            reason=f"{name} hinge edge sits flush against the shell side wall at the front opening.",
        )
        ctx.expect_contact(door, body, name=f"{name} hinged on the shell edge")

    # --- Labels sit on the door panels ---
    for name, door, _h in doors:
        ctx.expect_contact(
            door, door, elem_a="icon_0", elem_b="label",
            name=f"{name} icon printed on label",
        )

    # --- Doors are on opposite sides of center ---
    d0_center_x = (d0_aabb[0][0] + d0_aabb[1][0]) / 2.0
    d1_center_x = (d1_aabb[0][0] + d1_aabb[1][0]) / 2.0
    ctx.check(
        "doors are on opposite sides of center",
        d0_center_x < -0.01 and d1_center_x > 0.01,
        details=f"door_0 center_x={d0_center_x:.4f}, door_1 center_x={d1_center_x:.4f}",
    )

    return ctx.report()


object_model = build_object_model()
