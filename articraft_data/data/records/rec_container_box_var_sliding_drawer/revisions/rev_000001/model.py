from __future__ import annotations

# Corrugated cardboard box with slide-out inner drawer (matchbox / sliding-tray style).
# Forked from the open-corrugated four-flap shipping box: the fold-out flaps are
# replaced by a fixed top panel and a single PRISMATIC inner drawer that pulls
# straight out the front (-Y) face.
#
# Frame:
#   - X is the long box axis (length ~0.28 m), X in [-0.14, 0.14]
#   - Y is the short box axis (depth ~0.24 m), Y in [-0.12, 0.12]
#   - Z is up (height ~0.22 m), floor at z=0, top rim at z=0.22
#
# Construction:
#   - body (root): closed rectangular shell = 4 walls + floor + fixed top panel,
#     with a rectangular opening cut in the front (-Y) wall for the drawer.
#   - drawer: inner tray (floor + 2 side walls + back wall + front face plate),
#     open top, slides out through the front opening on a PRISMATIC joint.

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
WALL = 0.006            # corrugated wall / floor / top thickness

HX = LEN_X / 2.0        # 0.140
HY = DEP_Y / 2.0        # 0.120

# Top panel thickness (same as wall for uniform corrugated construction)
TOP_WALL = WALL

# Drawer-to-shell clearance on each side
CLEAR = 0.002

# Drawer outer extents (fits inside cavity with CLEAR gap all around)
DRAW_W = LEN_X - 2.0 * WALL - 2.0 * CLEAR   # 0.264
DRAW_H = HGT_Z - WALL - TOP_WALL - 2.0 * CLEAR  # 0.204
DRAW_D = DEP_Y - 2.0 * WALL - 2.0 * CLEAR   # 0.224

# Drawer tray wall / plate thicknesses
DRAW_WALL = 0.004       # side and back wall thickness
DRAW_FRONT_T = WALL     # front face plate thickness (matches shell wall)
DRAW_FLOOR_T = WALL     # floor thickness (matches shell floor)

# Prismatic travel: retain ~40 mm of the tray inside the shell at max extension
DRAW_UPPER = DRAW_D - 0.040


def _shell_solid() -> cq.Workplane:
    """Closed box shell: 4 walls + floor + fixed top, with front drawer opening."""
    outer = cq.Workplane("XY").box(LEN_X, DEP_Y, HGT_Z, centered=(True, True, False))

    # Inner cavity: closed top (leave TOP_WALL at top, WALL at bottom).
    cavity_h = HGT_Z - WALL - TOP_WALL
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=WALL)
        .box(
            LEN_X - 2.0 * WALL,
            DEP_Y - 2.0 * WALL,
            cavity_h,
            centered=(True, True, False),
        )
    )
    shell = outer.cut(cavity)

    # Rectangular opening in the front (-Y) wall for the drawer to pass through.
    # The opening matches the drawer front face plate extents exactly.
    open_cy = -HY + WALL / 2.0
    open_cz = WALL + CLEAR + DRAW_H / 2.0
    front_cut = (
        cq.Workplane("XY")
        .box(DRAW_W, WALL + 0.010, DRAW_H, centered=(True, True, True))
        .translate((0.0, open_cy, open_cz))
    )
    shell = shell.cut(front_cut)

    return shell


def _drawer_tray() -> cq.Workplane:
    """Inner drawer tray: front face plate + floor + 2 side walls + back wall, open top.

    Local frame:
      - origin at center of front face plate outer surface
      - y=0 is the outer front face, tray extends along +Y to y=DRAW_D
      - z=0 is the bottom, z=DRAW_H is the top of the front face
      - x centered
    """
    # Outer solid envelope
    outer = cq.Workplane("XY").box(
        DRAW_W, DRAW_D, DRAW_H, centered=(True, False, False),
    )

    # Inner cavity (open top): punch through from above the floor to past the top.
    cavity_w = DRAW_W - 2.0 * DRAW_WALL
    cavity_d = DRAW_D - DRAW_FRONT_T - DRAW_WALL
    cavity_h = DRAW_H - DRAW_FLOOR_T + 0.005

    cavity = (
        cq.Workplane("XY")
        .workplane(offset=DRAW_FLOOR_T)
        .box(cavity_w, cavity_d, cavity_h, centered=(True, False, False))
        .translate((0.0, DRAW_FRONT_T, 0.0))
    )

    return outer.cut(cavity)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cardboard_box_drawer")

    kraft = model.material("kraft", rgba=(0.78, 0.62, 0.40, 1.0))
    kraft_in = model.material("kraft_inner", rgba=(0.72, 0.56, 0.36, 1.0))
    print_ink = model.material("print_ink", rgba=(0.20, 0.20, 0.22, 1.0))
    drawer_kraft = model.material("drawer_kraft", rgba=(0.75, 0.58, 0.38, 1.0))

    # ---- body (root): closed shell with front drawer opening ----
    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_shell_solid(), "shell"),
        material=kraft,
        name="shell",
    )
    body.inertial = Inertial.from_geometry(
        Box((LEN_X, DEP_Y, HGT_Z)),
        mass=0.50,
        origin=Origin(xyz=(0.0, 0.0, HGT_Z / 2.0)),
    )

    # ---- drawer: sliding inner tray ----
    drawer = model.part("drawer")
    drawer.visual(
        mesh_from_cadquery(_drawer_tray(), "tray"),
        material=drawer_kraft,
        name="tray",
    )

    # Front panel decoration on the drawer face plate (carried from parent).
    panel_w = 0.16
    panel_h = 0.085
    panel_cz = DRAW_H * 0.42
    panel_y = -0.0005  # just proud of the front face outer surface
    drawer.visual(
        Box((panel_w, 0.001, panel_h)),
        origin=Origin(xyz=(0.0, panel_y, panel_cz)),
        material=kraft_in,
        name="front_panel",
    )

    # Printed icons on the front panel (loop-built repeated sub-parts).
    icon_y = panel_y - 0.0008
    for i, ix in enumerate((-0.057, -0.019, 0.019, 0.057)):
        drawer.visual(
            Box((0.022, 0.0008, 0.030)),
            origin=Origin(xyz=(ix, icon_y, panel_cz)),
            material=print_ink,
            name=f"icon_{i}",
        )

    # Small finger-pull grip bar near the top of the drawer front.
    # Grip back face must contact the tray front face (local y=0).
    grip_w = 0.050
    grip_d = 0.004
    grip_h = 0.010
    drawer.visual(
        Box((grip_w, grip_d, grip_h)),
        origin=Origin(xyz=(0.0, -grip_d / 2.0, DRAW_H - 0.020)),
        material=kraft_in,
        name="pull_grip",
    )

    drawer.inertial = Inertial.from_geometry(
        Box((DRAW_W, DRAW_D, DRAW_H)),
        mass=0.15,
        origin=Origin(xyz=(0.0, DRAW_D / 2.0, DRAW_H / 2.0)),
    )

    # ---- prismatic joint: drawer slides out the front (-Y) ----
    # At q=0 the drawer origin (front face outer center) coincides with the
    # joint origin, placing the front face flush with the box front wall and
    # the tray body fully inserted inside the shell cavity.
    joint_z = WALL + CLEAR  # drawer bottom aligns with the opening bottom edge
    model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(0.0, -HY, joint_z)),
        axis=(0.0, -1.0, 0.0),  # positive q pulls drawer toward -Y (out the front)
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.5,
            lower=0.0,
            upper=DRAW_UPPER,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    drawer = object_model.get_part("drawer")
    slide = object_model.get_articulation("body_to_drawer")

    # --- Shell has box footprint and full height ---
    sh_aabb = ctx.part_element_world_aabb(body, elem="shell")
    sh_ext = (
        sh_aabb[1][0] - sh_aabb[0][0],
        sh_aabb[1][1] - sh_aabb[0][1],
        sh_aabb[1][2] - sh_aabb[0][2],
    )
    ctx.check(
        "shell has box footprint and full height",
        abs(sh_ext[0] - LEN_X) < 0.01
        and abs(sh_ext[1] - DEP_Y) < 0.01
        and abs(sh_ext[2] - HGT_Z) < 0.01,
        details=f"shell extents={sh_ext}",
    )

    # The drawer tray intentionally sits inside the shell cavity: the shell is
    # a hollow box and the tray occupies the cavity void. Scope the allowance
    # to the exact pair and justify with proof checks.
    ctx.allow_overlap(
        body,
        drawer,
        elem_a="shell",
        elem_b="tray",
        reason=(
            "The inner drawer tray occupies the hollow shell cavity void. "
            "The shell walls enclose the tray on all sides except the front opening."
        ),
    )

    # --- Drawer tray fits within shell cavity in XY at rest ---
    ctx.expect_within(
        drawer,
        body,
        axes="xy",
        inner_elem="tray",
        outer_elem="shell",
        margin=0.005,
        name="drawer tray stays within shell cavity in XY at rest",
    )

    # --- Drawer tray fits within shell height at rest ---
    ctx.expect_within(
        drawer,
        body,
        axes="z",
        inner_elem="tray",
        outer_elem="shell",
        margin=0.005,
        name="drawer tray stays within shell height at rest",
    )

    # --- Drawer front face sits near the box front wall at rest ---
    # The tray front face (local y=0) should align with the shell front wall
    # (world y = -HY). Use the tray min_y vs the shell min_y on the -Y side.
    shell_aabb = ctx.part_element_world_aabb(body, elem="shell")
    tray_aabb = ctx.part_element_world_aabb(drawer, elem="tray")
    front_diff = abs(tray_aabb[0][1] - shell_aabb[0][1])
    ctx.check(
        "drawer front face flush with box front wall at rest",
        front_diff < 0.005,
        details=f"shell_front_y={shell_aabb[0][1]}, tray_front_y={tray_aabb[0][1]}, diff={front_diff}",
    )

    # --- Drawer pulls out toward -Y when slide joint increases ---
    rest_pos = ctx.part_world_position(drawer)
    with ctx.pose({slide: DRAW_UPPER}):
        ext_pos = ctx.part_world_position(drawer)
    ctx.check(
        "drawer pulls out toward -Y",
        rest_pos is not None
        and ext_pos is not None
        and ext_pos[1] < rest_pos[1] - 0.10,
        details=f"rest_y={rest_pos[1] if rest_pos else None}, "
        f"extended_y={ext_pos[1] if ext_pos else None}",
    )

    # --- Drawer retains insertion in shell at max extension ---
    with ctx.pose({slide: DRAW_UPPER}):
        ctx.expect_overlap(
            drawer,
            body,
            axes="y",
            elem_a="tray",
            elem_b="shell",
            min_overlap=0.030,
            name="drawer retains insertion in shell at max extension",
        )

    # --- Drawer stays within shell vertically at max extension ---
    with ctx.pose({slide: DRAW_UPPER}):
        ctx.expect_within(
            drawer,
            body,
            axes="z",
            inner_elem="tray",
            outer_elem="shell",
            margin=0.005,
            name="drawer stays within shell height at max extension",
        )

    # --- Printed icons on drawer front panel ---
    ctx.expect_contact(
        drawer,
        drawer,
        elem_a="icon_0",
        elem_b="front_panel",
        name="recycle icon printed on drawer front panel",
    )

    # --- Pull grip contacts drawer front face ---
    ctx.expect_contact(
        drawer,
        drawer,
        elem_a="pull_grip",
        elem_b="tray",
        name="finger pull grip mounted on drawer front face",
    )

    return ctx.report()


object_model = build_object_model()
