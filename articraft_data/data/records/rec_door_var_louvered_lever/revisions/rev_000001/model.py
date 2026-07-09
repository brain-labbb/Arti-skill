from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    VentGrilleGeometry,
    VentGrilleSlats,
    VentGrilleSleeve,
    mesh_from_geometry,
)

# ---------------------------------------------------------------------------
# Single louvered utility door in gray painted steel (Z-up: stands upright).
#
#   - FIXED metal casing (root): head, sill, two side jambs.
#   - Operable steel leaf with a raised perimeter frame and two real
#     horizontal-louver vented panels (upper + lower) using VentGrilleGeometry.
#   - Brushed-steel lever handle on a round rose at lever height near the
#     latch (left) edge; the lever pivots on its own spindle joint.
#   - Leaf swings open on a vertical hinge along the right edge.
#
# Coordinate convention (world UP = +Z):
#   - height runs along +Z, sill/base at z=0, top ~2.04 m.
#   - width runs along X.
#   - leaf thickness + swing-normal run along Y.
#   - hinge axis is vertical, (0, 0, 1).
# ---------------------------------------------------------------------------

LEAF_W = 0.86          # door leaf width (along X)
LEAF_H = 2.04          # door leaf height (along Z)
LEAF_T = 0.045         # door leaf thickness (along Y, the door normal)

RAIL = 0.075           # raised perimeter frame face width on the leaf
STILE = 0.075
MID_RAIL = 0.090       # central rail separating the two louver panels (lock rail)

CASING_W = 0.060       # casing member face width
CASING_D = 0.090       # casing depth (along Y)
REVEAL = 0.005         # gap between leaf and casing

# Casing opening sized around the leaf.
OPEN_W = LEAF_W + 2.0 * REVEAL
OPEN_H = LEAF_H + 2.0 * REVEAL
CASING_SPAN_W = OPEN_W + 2.0 * CASING_W
CASING_SPAN_H = OPEN_H + 2.0 * CASING_W

# Vertical placement: the casing/leaf sit on the ground plane (base at z=0).
# The leaf center is at z = OPEN_H / 2 + CASING_W (sill casing below it).
GROUND_OFFSET = CASING_W  # sill casing occupies z in [0, CASING_W]
LEAF_CZ = GROUND_OFFSET + OPEN_H / 2.0  # world z of the leaf vertical center

# Leaf-local geometry: origin on the hinge line (right edge); leaf extends -X.
# Local x in [-LEAF_W, 0], local z centered on the leaf, local y centered.
LEAF_CX_LOCAL = -LEAF_W / 2.0

# Louver panel layout (within the leaf, between rails/stiles).
PANEL_W = LEAF_W - 2.0 * STILE
INNER_H = LEAF_H - 2.0 * RAIL - MID_RAIL
UPPER_PANEL_H = INNER_H * 0.55
LOWER_PANEL_H = INNER_H * 0.45

# Vertical centers (leaf-local Z, leaf centered at 0).
TOP_INNER = LEAF_H / 2.0 - RAIL
BOT_INNER = -LEAF_H / 2.0 + RAIL
UPPER_CZ = TOP_INNER - UPPER_PANEL_H / 2.0
LOWER_CZ = BOT_INNER + LOWER_PANEL_H / 2.0
MID_RAIL_CZ = (UPPER_CZ - UPPER_PANEL_H / 2.0 + LOWER_CZ + LOWER_PANEL_H / 2.0) / 2.0

# Rotate a grille (built in local XY extruding along +Z) so its face lies in the
# XZ plane: local X -> world X (horizontal slats), local Y -> world Z (vertical
# stack), local +Z extrusion -> world +Y (the door thickness / normal).
GRILLE_RPY = (math.pi / 2.0, 0.0, 0.0)





def _louver_panel(width: float, height: float, name: str):
    grille = VentGrilleGeometry(
        (width, height),
        frame=0.010,
        face_thickness=0.004,
        duct_depth=LEAF_T * 0.7,
        duct_wall=0.003,
        slat_pitch=0.026,
        slat_width=0.020,
        slat_angle_deg=35.0,
        corner_radius=0.004,
        slats=VentGrilleSlats(profile="flat", direction="down"),
        sleeve=VentGrilleSleeve(style="short"),
        center=True,
    )
    return mesh_from_geometry(grille, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="louvered_utility_door")

    steel = model.material("gray_steel", color=(0.55, 0.56, 0.57))
    steel_dark = model.material("gray_steel_dark", color=(0.42, 0.43, 0.45))
    metal = model.material("brushed_metal", color=(0.72, 0.72, 0.74))

    # -------------------------------------------------------------------
    # FIXED casing (root): head, sill, two jambs. Base sits on z=0.
    # -------------------------------------------------------------------
    casing = model.part("casing")
    casing.visual(
        Box((CASING_SPAN_W, CASING_D, CASING_W)),
        origin=Origin(xyz=(0.0, 0.0, GROUND_OFFSET + OPEN_H + CASING_W / 2.0)),
        material=steel_dark,
        name="head_casing",
    )
    casing.visual(
        Box((CASING_SPAN_W, CASING_D, CASING_W)),
        origin=Origin(xyz=(0.0, 0.0, CASING_W / 2.0)),
        material=steel_dark,
        name="sill_casing",
    )
    for sgn, tag in ((1, "right"), (-1, "left")):
        casing.visual(
            Box((CASING_W, CASING_D, OPEN_H)),
            origin=Origin(xyz=(sgn * (OPEN_W / 2.0 + CASING_W / 2.0), 0.0, LEAF_CZ)),
            material=steel_dark,
            name=f"{tag}_jamb",
        )

    # -------------------------------------------------------------------
    # Operable leaf. Local frame at hinge line (right edge); leaf in -X.
    # -------------------------------------------------------------------
    leaf = model.part("leaf")
    cx = LEAF_CX_LOCAL

    # Raised perimeter frame: top/bottom rails + hinge/latch stiles + lock rail.
    leaf.visual(
        Box((LEAF_W, LEAF_T, RAIL)),
        origin=Origin(xyz=(cx, 0.0, LEAF_H / 2.0 - RAIL / 2.0)),
        material=steel,
        name="top_rail",
    )
    leaf.visual(
        Box((LEAF_W, LEAF_T, RAIL)),
        origin=Origin(xyz=(cx, 0.0, -LEAF_H / 2.0 + RAIL / 2.0)),
        material=steel,
        name="bottom_rail",
    )
    for sgn, tag in ((1, "hinge"), (-1, "latch")):
        leaf.visual(
            Box((STILE, LEAF_T, LEAF_H)),
            origin=Origin(xyz=(cx + sgn * (LEAF_W / 2.0 - STILE / 2.0), 0.0, 0.0)),
            material=steel,
            name=f"{tag}_stile",
        )
    leaf.visual(
        Box((PANEL_W + 0.006, LEAF_T, MID_RAIL)),
        origin=Origin(xyz=(cx, 0.0, MID_RAIL_CZ)),
        material=steel,
        name="lock_rail",
    )

    # Two louver panels seated into the leaf opening. The grille face lies in XZ
    # and extends along Y; place at the leaf y-center. The panel frame is sized a
    # touch larger than the opening so it seats under the surrounding rails/stiles.
    leaf.visual(
        _louver_panel(PANEL_W + 0.010, UPPER_PANEL_H + 0.010, "upper_louver"),
        origin=Origin(xyz=(cx, 0.0, UPPER_CZ), rpy=GRILLE_RPY),
        material=steel,
        name="upper_louver_panel",
    )
    leaf.visual(
        _louver_panel(PANEL_W + 0.010, LOWER_PANEL_H + 0.010, "lower_louver"),
        origin=Origin(xyz=(cx, 0.0, LOWER_CZ), rpy=GRILLE_RPY),
        material=steel,
        name="lower_louver_panel",
    )

    # Brushed-steel lever handle on a round rose, latch (-X) edge, room side
    # (+Y), at lock-rail (lever) height.  The rose is a static visual on the
    # leaf; the lever arm is a separate part with its own spindle joint.
    lever_x = cx - LEAF_W / 2.0 + STILE * 0.55
    lever_z = MID_RAIL_CZ + 0.02
    rose_face_y = LEAF_T / 2.0  # room-side face of the leaf

    # Round rose (escutcheon plate) seated on the leaf latch stile.
    leaf.visual(
        Cylinder(radius=0.025, length=0.006),
        origin=Origin(
            xyz=(lever_x, rose_face_y + 0.003, lever_z),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=metal,
        name="lever_rose",
    )

    # Lever handle part: primitives in its own local frame (origin at the
    # spindle pivot, Y = spindle axis outward, +X = lever direction at rest,
    # Z = up).  The part frame is placed at the spindle via the articulation.
    lever = model.part("lever")

    # Lever arm: a rounded box extending in +X from the spindle, proud of the
    # door face at y=arm_cy.
    arm_len = 0.115
    arm_dy = 0.015
    arm_dz = 0.019
    arm_cy = 0.012  # center Y above the door face
    lever.visual(
        Box((arm_len, arm_dy, arm_dz)),
        origin=Origin(xyz=(arm_len / 2.0, arm_cy, 0.0)),
        material=metal,
        name="lever_arm",
    )
    # Hub: a short cylinder along Y connecting the arm to the rose/leaf face.
    # Extends from y=arm_cy - arm_dy/2 (arm bottom) down to just below the
    # leaf face so the spindle appears embedded through the door.
    hub_dy_min = -0.002  # 2 mm below the leaf face for visible support
    hub_dy_max = arm_cy - arm_dy / 2.0  # meets the arm bottom
    hub_len = hub_dy_max - hub_dy_min
    hub_cy = (hub_dy_min + hub_dy_max) / 2.0
    lever.visual(
        Cylinder(radius=0.012, length=hub_len),
        origin=Origin(xyz=(0.0, hub_cy, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="lever_hub",
    )

    # Spindle joint: lever pivots about Y (door normal) on the leaf.
    # The lever arm extends in +X from the spindle. Positive q about (0, +1, 0)
    # rotates +X toward -Z, pressing the lever downward.
    model.articulation(
        "leaf_to_lever",
        ArticulationType.REVOLUTE,
        parent=leaf,
        child=lever,
        origin=Origin(xyz=(lever_x, rose_face_y, lever_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=4.0, lower=0.0, upper=0.55),
    )

    # Hinge on the right edge. Leaf extends in local -X from x=0; positive q
    # about +Z swings the free (latch / -X) edge toward -Y (out of the casing).
    # Place the hinge line at the right jamb inner face so the closed leaf's
    # hinge stile touches the casing (grounded support, no floating gap).
    hinge_x = OPEN_W / 2.0
    model.articulation(
        "casing_to_leaf",
        ArticulationType.REVOLUTE,
        parent=casing,
        child=leaf,
        origin=Origin(xyz=(hinge_x, 0.0, LEAF_CZ)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=2.0, lower=0.0, upper=1.6),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    casing = object_model.get_part("casing")
    leaf = object_model.get_part("leaf")
    lever = object_model.get_part("lever")
    hinge = object_model.get_articulation("casing_to_leaf")
    spindle = object_model.get_articulation("leaf_to_lever")

    upper = leaf.get_visual("upper_louver_panel")
    lower = leaf.get_visual("lower_louver_panel")
    rose = leaf.get_visual("lever_rose")
    lever_arm = lever.get_visual("lever_arm")
    lever_hub = lever.get_visual("lever_hub")

    # Orientation: the door stands upright, height along +Z, base on z=0.
    casing_ab = ctx.part_world_aabb(casing)
    leaf_ab = ctx.part_world_aabb(leaf)
    if casing_ab is not None and leaf_ab is not None:
        (cx0, cy0, cz0), (cx1, cy1, cz1) = casing_ab
        (lx0, ly0, lz0), (lx1, ly1, lz1) = leaf_ab
        ox0, oy0, oz0 = min(cx0, lx0), min(cy0, ly0), min(cz0, lz0)
        ox1, oy1, oz1 = max(cx1, lx1), max(cy1, ly1), max(cz1, lz1)
        dx, dy, dz = ox1 - ox0, oy1 - oy0, oz1 - oz0
        ctx.check("rests_on_ground", abs(oz0) < 0.01, details=f"zmin={oz0}")
        ctx.check("height_along_z", abs(dz - CASING_SPAN_H) < 0.05, details=f"dz={dz}")
        ctx.check("upright_tallest_axis_z", dz > dx and dz > dy, details=f"dx={dx},dy={dy},dz={dz}")

    # Hinge axis is vertical.
    ctx.check(
        "hinge_axis_vertical",
        abs(hinge.axis[2]) > 0.99 and abs(hinge.axis[0]) < 0.01 and abs(hinge.axis[1]) < 0.01,
        details=f"axis={hinge.axis}",
    )

    # Hero feature: two louver panels present and sized as large vented panels.
    ctx.check("upper_louver_present", upper is not None)
    ctx.check("lower_louver_present", lower is not None)

    up_ab = ctx.part_element_world_aabb(leaf, elem=upper)
    if up_ab is not None:
        (x0, _, z0), (x1, _, z1) = up_ab
        ctx.check("upper_louver_width", (x1 - x0) > 0.5, details=f"w={x1 - x0}")
        ctx.check("upper_louver_height", (z1 - z0) > 0.4, details=f"h={z1 - z0}")
    lo_ab = ctx.part_element_world_aabb(leaf, elem=lower)
    if lo_ab is not None:
        (x0, _, z0), (x1, _, z1) = lo_ab
        ctx.check("lower_louver_width", (x1 - x0) > 0.5, details=f"w={x1 - x0}")
        ctx.check("lower_louver_height", (z1 - z0) > 0.3, details=f"h={z1 - z0}")

    # Hero feature: lever handle on round rose present on the latch edge.
    ctx.check("lever_rose_present", rose is not None)
    ctx.check("lever_arm_present", lever_arm is not None)
    ctx.check("lever_hub_present", lever_hub is not None)

    # Rose is mounted on the leaf latch stile (contact or near-contact).
    latch_stile = leaf.get_visual("latch_stile")
    ctx.expect_contact(
        leaf,
        leaf,
        elem_a=rose,
        elem_b=latch_stile,
        contact_tol=0.02,
        name="rose_mounted_on_leaf",
    )

    # The lever hub penetrates the leaf latch stile by ~2 mm to represent the
    # spindle passing through the door — a local intentional embedding.
    ctx.allow_overlap(
        leaf,
        lever,
        elem_a="latch_stile",
        elem_b="lever_hub",
        reason="The spindle hub passes through the latch stile to represent the lever spindle mounting.",
    )
    ctx.expect_contact(
        leaf,
        lever,
        elem_a=latch_stile,
        elem_b=lever_hub,
        contact_tol=0.005,
        name="hub_seated_on_latch_stile",
    )

    # Lever handle is on the latch edge (toward -X relative to the leaf center).
    full_leaf_ab = ctx.part_world_aabb(leaf)
    lever_ab = ctx.part_world_aabb(lever)
    if full_leaf_ab is not None and lever_ab is not None:
        (flx0, _, _), (flx1, _, _) = full_leaf_ab
        (lvx0, _, _), (lvx1, _, _) = lever_ab
        lcx = (lvx0 + lvx1) / 2.0
        ctx.check(
            "lever_on_latch_edge",
            lcx < (flx0 + flx1) / 2.0,
            details=f"lever_x={lcx}, leaf_center={(flx0 + flx1) / 2.0}",
        )

    # Spindle joint axis is along world Y (door normal / perpendicular to face).
    ctx.check(
        "spindle_axis_along_y",
        abs(spindle.axis[1]) > 0.99 and abs(spindle.axis[0]) < 0.01 and abs(spindle.axis[2]) < 0.01,
        details=f"axis={spindle.axis}",
    )

    # Hinge edge meets the right jamb at rest; leaf connected, not floating.
    right_jamb = casing.get_visual("right_jamb")
    hinge_stile = leaf.get_visual("hinge_stile")

    def _lever_tip_z() -> float | None:
        ab = ctx.part_world_aabb(lever)
        if ab is None:
            return None
        (_, _, z0), (_, _, z1) = ab
        return z0  # lowest z of the lever AABB (the tip when pressed down)

    with ctx.pose({hinge: 0.0}):
        ctx.expect_contact(
            leaf,
            casing,
            elem_a=hinge_stile,
            elem_b=right_jamb,
            contact_tol=0.04,
            name="hinge_edge_meets_jamb_closed",
        )

    with ctx.pose({hinge: 1.4}):
        ctx.expect_contact(
            leaf,
            casing,
            elem_a=hinge_stile,
            elem_b=right_jamb,
            contact_tol=0.10,
            name="hinge_edge_stays_connected_open",
        )

    # Lever spindle: pressing the lever should move the free end downward.
    rest_tip_z = _lever_tip_z()
    with ctx.pose({spindle: 0.45}):
        pressed_tip_z = _lever_tip_z()

    ctx.check(
        "lever_presses_downward",
        rest_tip_z is not None and pressed_tip_z is not None and pressed_tip_z < rest_tip_z - 0.01,
        details=f"rest_tip_z={rest_tip_z}, pressed_tip_z={pressed_tip_z}",
    )

    return ctx.report()


object_model = build_object_model()
