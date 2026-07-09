from __future__ import annotations

# Front-loading corrugated cardboard bin with a single drop-down front door.
# Forked from the open corrugated four-flap shipping box: the four top fold-out
# flaps are replaced by a fixed top lid and one tall front-wall panel hinged
# along its bottom edge that folds down and forward to open.
#
# Frame:
#   - X is the long box axis (length ~0.28 m), X in [-0.14, 0.14]
#   - Y is the short box axis (depth ~0.24 m), Y in [-0.12, 0.12]
#   - Z is up (height ~0.22 m), floor at z=0, top rim at z=0.22
# Construction:
#   - body (root): closed rectangular shell = floor + back wall + two side
#     walls + fixed top lid. Front face is the door opening.
#   - front_door: tall panel filling the front opening, REVOLUTE hinged along
#     its bottom edge at the floor-front junction; positive q folds it down
#     and forward (front-loading bin style).

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
DEP_Y = 0.24            # outer depth (short axis)
HGT_Z = 0.22            # outer height
WALL = 0.006            # corrugated wall / floor / lid thickness

HX = LEN_X / 2.0        # 0.140
HY = DEP_Y / 2.0        # 0.120

# inner cavity extents (between wall inner faces)
IN_HX = HX - WALL       # 0.134
IN_HY = HY - WALL       # 0.114

# door panel: fits between side walls and between floor and ceiling
DOOR_CLEAR = 0.001      # 1 mm edge clearance on each side
DOOR_W = LEN_X - 2.0 * WALL - 2.0 * DOOR_CLEAR
DOOR_H = HGT_Z - 2.0 * WALL
DOOR_T = WALL            # same thickness as other walls

OPEN_ANGLE = math.radians(85.0)  # door opens to near-horizontal


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _shell_solid() -> cq.Workplane:
    """Closed rectangular shell with the front face removed (door opening).

    Built as the full outer box minus an inner cavity that punches through
    the front wall while leaving the floor, ceiling, back wall, and side
    walls intact.
    """
    outer = cq.Workplane("XY").box(
        LEN_X, DEP_Y, HGT_Z, centered=(True, True, False),
    )

    # Inner cavity volume: leaves WALL-thick floor, ceiling, back, and sides.
    cav_w = LEN_X - 2.0 * WALL           # between inner side-wall faces
    cav_h = HGT_Z - 2.0 * WALL           # between floor top and ceiling bottom
    # Cavity extends past the front face so it cuts through the front wall.
    cav_d = DEP_Y - WALL + 0.02          # back edge at inner back-wall face
    cav_cy = (HY - WALL) - cav_d / 2.0   # centre Y so back edge = HY - WALL

    cavity = (
        cq.Workplane("XY")
        .workplane(offset=WALL)
        .box(cav_w, cav_d, cav_h, centered=(True, True, False))
        .translate((0.0, cav_cy, 0.0))
    )
    return outer.cut(cavity)


def _door_panel(width: float, thickness: float, height: float) -> cq.Workplane:
    """Flat cardboard door panel.

    Local frame: hinge edge at origin along X; panel extends along +Z
    (upward when closed) and +Y (thickness, inward when closed).
    """
    return cq.Workplane("XY").box(
        width, thickness, height, centered=(True, True, False),
    )


# ---------------------------------------------------------------------------
# Object model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cardboard_bin")

    kraft = model.material("kraft", rgba=(0.78, 0.62, 0.40, 1.0))
    kraft_in = model.material("kraft_inner", rgba=(0.72, 0.56, 0.36, 1.0))
    print_ink = model.material("print_ink", rgba=(0.20, 0.20, 0.22, 1.0))
    tape = model.material("packing_tape", rgba=(0.82, 0.75, 0.55, 0.7))

    # ---- body (root): shell with open front + packing-tape top detail ----
    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_shell_solid(), "shell"),
        material=kraft,
        name="shell",
    )

    # Packing tape strip across the fixed top lid (sealing detail).
    body.visual(
        Box((LEN_X + 0.01, 0.05, 0.0005)),
        origin=Origin(xyz=(0.0, 0.0, HGT_Z)),
        material=tape,
        name="top_tape",
    )

    body.inertial = Inertial.from_geometry(
        Box((LEN_X, DEP_Y, HGT_Z)),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.0, HGT_Z / 2.0)),
    )

    # ---- front door: hinged at bottom front edge ----
    door = model.part("front_door")
    door.visual(
        mesh_from_cadquery(_door_panel(DOOR_W, DOOR_T, DOOR_H), "panel"),
        material=kraft,
        name="panel",
    )

    # Printed label panel on the door outer face.
    # Panel local Y range: [-DOOR_T/2, DOOR_T/2] = [-0.003, 0.003].
    # Outer face at local y = -DOOR_T/2 = -0.003.
    label_w = 0.16
    label_h = 0.085
    label_cz = DOOR_H * 0.50          # centred vertically on the door
    label_t = 0.001
    label_y = -DOOR_T / 2.0            # label centred on panel outer face
    door.visual(
        Box((label_w, label_t, label_h)),
        origin=Origin(xyz=(0.0, label_y, label_cz)),
        material=kraft_in,
        name="front_label",
    )

    # Printed ink icons (recycle / handling symbols) on the label.
    icon_t = 0.0008
    icon_y = label_y - label_t / 2.0 - icon_t / 2.0 + 0.0002
    for i, ix in enumerate((-0.057, -0.019, 0.019, 0.057)):
        door.visual(
            Box((0.022, icon_t, 0.030)),
            origin=Origin(xyz=(ix, icon_y, label_cz)),
            material=print_ink,
            name=f"icon_{i}",
        )

    door.inertial = Inertial.from_geometry(
        Box((DOOR_W, DOOR_T, DOOR_H)),
        mass=0.04,
        origin=Origin(xyz=(0.0, DOOR_T / 2.0, DOOR_H / 2.0)),
    )

    # Hinge at the outer bottom-front edge of the box (floor-front junction).
    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(0.0, -HY, WALL)),
        # axis=(+1,0,0): right-hand rule rotates local +Z toward -Y,
        # so positive q folds the door top forward and down.
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0, velocity=2.0, lower=0.0, upper=OPEN_ANGLE,
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    door = object_model.get_part("front_door")
    hinge = object_model.get_articulation("door_hinge")

    # --- Shell has full box footprint and height (with closed top). ---
    sa = ctx.part_element_world_aabb(body, elem="shell")
    sh = (sa[1][0] - sa[0][0], sa[1][1] - sa[0][1], sa[1][2] - sa[0][2])
    ctx.check(
        "shell has box footprint and full height",
        abs(sh[0] - LEN_X) < 0.01
        and abs(sh[1] - DEP_Y) < 0.01
        and abs(sh[2] - HGT_Z) < 0.01,
        details=f"shell extents={sh}",
    )

    # --- Door is a tall thin panel covering the front opening. ---
    da = ctx.part_world_aabb(door)
    door_dy = da[1][1] - da[0][1]
    door_dz = da[1][2] - da[0][2]
    ctx.check(
        "door is tall panel covering front opening",
        door_dz > HGT_Z * 0.8 and door_dy < 0.02,
        details=f"door dz={door_dz:.4f}, dy={door_dy:.4f}",
    )

    # --- Door sits at the front face when closed (q=0). ---
    door_cy = (da[0][1] + da[1][1]) / 2.0
    expected_cy = -HY + WALL / 2.0
    ctx.check(
        "door sits at front face when closed",
        abs(door_cy - expected_cy) < 0.015,
        details=f"door center_y={door_cy:.4f}, expected~{expected_cy:.4f}",
    )

    # --- Positive q swings door forward (center moves toward -Y). ---
    rest_y = door_cy
    with ctx.pose({hinge: OPEN_ANGLE}):
        oa = ctx.part_world_aabb(door)
        open_y = (oa[0][1] + oa[1][1]) / 2.0
    ctx.check(
        "door swings forward when opened",
        open_y < rest_y - 0.05,
        details=f"rest_y={rest_y:.4f}, open_y={open_y:.4f}",
    )

    # --- Door top drops when opened (z extent decreases). ---
    rest_top = da[1][2]
    with ctx.pose({hinge: OPEN_ANGLE}):
        open_top = ctx.part_world_aabb(door)[1][2]
    ctx.check(
        "door top drops when opened",
        open_top < rest_top - 0.05,
        details=f"rest_top_z={rest_top:.4f}, open_top_z={open_top:.4f}",
    )

    # --- Door contacts body at the hinge edge (bottom-front junction). ---
    ctx.expect_contact(door, body, name="door hinged on body at bottom edge")

    # --- Printed icons sit on the door front label. ---
    ctx.expect_contact(
        door, door, elem_a="icon_0", elem_b="front_label",
        name="icon printed on door front label",
    )

    return ctx.report()


object_model = build_object_model()
