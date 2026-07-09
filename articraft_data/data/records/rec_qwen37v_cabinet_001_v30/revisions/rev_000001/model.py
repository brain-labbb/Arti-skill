from __future__ import annotations

"""Narrow bathroom wall cabinet with mirrored door and sliding tambour front.

A compact wall-mounted bathroom cabinet, approximately 0.45 m wide, 0.60 m tall,
0.15 m deep, in a light satin-white painted steel finish. The carcass is a thin-
walled hollow box divided into two sections:

- Left section (~55% width): enclosed compartment with a hinged mirrored door.
  The door swings open on a vertical hinge at its left edge (0..~110 deg).
- Right section (~45% width): open shelving with two interior shelf boards.
  A tambour sliding panel covers the open front and slides sideways along the X
  axis on a prismatic joint to reveal the shelves through the open gap.

Wall-mount brackets protrude from the back panel.
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
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Global dimensions (meters). Cabinet centred on X, front face at +Y.
# ---------------------------------------------------------------------------
CAB_W = 0.45   # overall width (X)
CAB_H = 0.60   # overall height (Z)
CAB_D = 0.15   # overall depth (Y)
WALL_T = 0.012  # wall thickness

FRONT_Y = CAB_D / 2.0   # +0.075
BACK_Y = -CAB_D / 2.0   # -0.075

# Inner extents (between wall inner faces)
INNER_LEFT = -CAB_W / 2.0 + WALL_T    # -0.213
INNER_RIGHT = CAB_W / 2.0 - WALL_T    # +0.213
INNER_BOTTOM = WALL_T                  # 0.012
INNER_TOP = CAB_H - WALL_T             # 0.588
INNER_DEPTH = CAB_D - 2.0 * WALL_T    # 0.126

# Divider separating door section (left) from shelf section (right)
DIVIDER_X = 0.02   # divider slightly right of centre
DIVIDER_W = WALL_T  # 0.012
DIVIDER_LEFT_FACE = DIVIDER_X - DIVIDER_W / 2.0   # 0.014
DIVIDER_RIGHT_FACE = DIVIDER_X + DIVIDER_W / 2.0  # 0.026

# Door section inner extents
DOOR_SECTION_W = DIVIDER_LEFT_FACE - INNER_LEFT  # 0.227
DOOR_W = DOOR_SECTION_W - 0.004                  # 0.223 (clearance)
DOOR_T = 0.010
DOOR_H = INNER_TOP - INNER_BOTTOM - 0.004        # 0.572
DOOR_ZC = CAB_H / 2.0                            # 0.300

# Hinge at left inner wall edge
HINGE_X = INNER_LEFT + 0.002  # -0.211

# Shelf section inner extents
SHELF_SECTION_W = INNER_RIGHT - DIVIDER_RIGHT_FACE  # 0.187
SHELF_SECTION_XC = (DIVIDER_RIGHT_FACE + INNER_RIGHT) / 2.0  # 0.1195

# Tambour panel fits within shelf section opening
TAMBOUR_W = SHELF_SECTION_W - 0.006  # 0.181
TAMBOUR_H = DOOR_H                    # 0.572
TAMBOUR_T = 0.008
TAMBOUR_TRAVEL = SHELF_SECTION_W      # 0.187

# Shelves
SHELF_T = 0.008
SHELF_D = INNER_DEPTH - 0.004         # 0.122

DOOR_OPEN = math.radians(110.0)


def _door_leaf(mesh_name: str):
    """Door panel: flat slab with a small pull-notch near the free edge."""
    panel = cq.Workplane("XY").box(DOOR_W, DOOR_T, DOOR_H)
    # Small recessed pull notch near the right edge at mid-height
    notch = (
        cq.Workplane("XY")
        .box(0.035, 0.006, 0.022)
        .translate((DOOR_W / 2.0 - 0.025, -DOOR_T / 2.0 - 0.002, 0.0))
    )
    leaf = panel.cut(notch)
    return mesh_from_cadquery(leaf, mesh_name)


def _tambour_panel(mesh_name: str):
    """Tambour sliding panel with horizontal slat grooves."""
    base = cq.Workplane("XY").box(TAMBOUR_W, TAMBOUR_T, TAMBOUR_H)
    # Cut thin horizontal grooves to suggest tambour slats
    for i in range(12):
        zc = -TAMBOUR_H / 2.0 + 0.025 + i * (TAMBOUR_H - 0.05) / 11.0
        groove = (
            cq.Workplane("XY")
            .box(TAMBOUR_W - 0.01, 0.003, 0.002)
            .translate((0.0, TAMBOUR_T / 2.0 + 0.001, zc))
        )
        base = base.cut(groove)
    return mesh_from_cadquery(base, mesh_name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bathroom_wall_cabinet")

    # Materials
    body_mat = model.material("body_white", rgba=(0.90, 0.91, 0.92, 1.0))
    body_inner = model.material("body_inner", rgba=(0.85, 0.86, 0.87, 1.0))
    mirror_mat = model.material("mirror", rgba=(0.82, 0.85, 0.88, 1.0))
    door_mat = model.material("door_white", rgba=(0.88, 0.89, 0.90, 1.0))
    tambour_mat = model.material("tambour_satin", rgba=(0.80, 0.82, 0.84, 1.0))
    shelf_mat = model.material("shelf_white", rgba=(0.87, 0.88, 0.89, 1.0))
    handle_mat = model.material("handle_chrome", rgba=(0.70, 0.72, 0.74, 1.0))
    track_mat = model.material("track_alum", rgba=(0.65, 0.67, 0.69, 1.0))
    bracket_mat = model.material("bracket_steel", rgba=(0.50, 0.52, 0.54, 1.0))

    # ------------------------------------------------------------------
    # Cabinet body: hollow carcass, divider, shelves, tracks, wall brackets
    # ------------------------------------------------------------------
    body = model.part("cabinet_body")

    # Side walls (full depth, full height)
    body.visual(
        Box((WALL_T, CAB_D, CAB_H)),
        origin=Origin(xyz=(-(CAB_W / 2.0 - WALL_T / 2.0), 0.0, CAB_H / 2.0)),
        material=body_mat,
        name="side_wall_left",
    )
    body.visual(
        Box((WALL_T, CAB_D, CAB_H)),
        origin=Origin(xyz=(CAB_W / 2.0 - WALL_T / 2.0, 0.0, CAB_H / 2.0)),
        material=body_mat,
        name="side_wall_right",
    )
    # Back wall
    body.visual(
        Box((CAB_W, WALL_T, CAB_H)),
        origin=Origin(xyz=(0.0, BACK_Y + WALL_T / 2.0, CAB_H / 2.0)),
        material=body_mat,
        name="back_wall",
    )
    # Top and bottom panels
    body.visual(
        Box((CAB_W, CAB_D, WALL_T)),
        origin=Origin(xyz=(0.0, 0.0, WALL_T / 2.0)),
        material=body_mat,
        name="bottom_panel",
    )
    body.visual(
        Box((CAB_W, CAB_D, WALL_T)),
        origin=Origin(xyz=(0.0, 0.0, CAB_H - WALL_T / 2.0)),
        material=body_mat,
        name="top_panel",
    )

    # Internal divider wall between door section and shelf section
    body.visual(
        Box((DIVIDER_W, INNER_DEPTH, INNER_TOP - INNER_BOTTOM)),
        origin=Origin(xyz=(DIVIDER_X, 0.0, CAB_H / 2.0)),
        material=body_inner,
        name="divider_wall",
    )

    # Tambour track rails (top and bottom grooves at front of shelf section)
    track_w = SHELF_SECTION_W + 0.008
    for tz, tname in (
        (INNER_BOTTOM + 0.003, "track_rail_bottom"),
        (INNER_TOP - 0.003, "track_rail_top"),
    ):
        body.visual(
            Box((track_w, 0.012, 0.005)),
            origin=Origin(xyz=(SHELF_SECTION_XC, FRONT_Y - 0.006, tz)),
            material=track_mat,
            name=tname,
        )

    # Interior shelf boards in the right (shelf) section
    # Shelves span the full section width with a small overlap into the walls
    # (like real shelves seated in dado grooves) so they read as supported.
    shelf_full_w = SHELF_SECTION_W + 0.004
    shelf_z_positions = [
        INNER_BOTTOM + (INNER_TOP - INNER_BOTTOM) * 0.33,
        INNER_BOTTOM + (INNER_TOP - INNER_BOTTOM) * 0.66,
    ]
    for i, sz in enumerate(shelf_z_positions):
        body.visual(
            Box((shelf_full_w, SHELF_D, SHELF_T)),
            origin=Origin(xyz=(SHELF_SECTION_XC, 0.0, sz)),
            material=shelf_mat,
            name=f"shelf_board_{i}",
        )

    # Wall-mount brackets on the back (two L-shaped protrusions)
    for bz, bname in ((0.15, "mount_bracket_upper"), (CAB_H - 0.15, "mount_bracket_lower")):
        body.visual(
            Box((0.06, 0.004, 0.04)),
            origin=Origin(xyz=(0.0, BACK_Y - 0.002, bz)),
            material=bracket_mat,
            name=f"{bname}_plate",
        )
        body.visual(
            Box((0.06, 0.030, 0.004)),
            origin=Origin(xyz=(0.0, BACK_Y + 0.013, bz - 0.022)),
            material=bracket_mat,
            name=f"{bname}_tab",
        )

    # ------------------------------------------------------------------
    # Mirror door (hinged on left edge, extends along +X from hinge line)
    # ------------------------------------------------------------------
    door = model.part("mirror_door")

    # Door leaf positioned so it extends along +X from the door part origin (hinge)
    door.visual(
        _door_leaf("door_leaf"),
        origin=Origin(xyz=(DOOR_W / 2.0, 0.0, 0.0)),
        material=door_mat,
        name="leaf",
    )
    # Mirror surface on front face (slightly inset, reflective material)
    mirror_w = DOOR_W - 0.03
    mirror_h = DOOR_H - 0.06
    door.visual(
        Box((mirror_w, 0.003, mirror_h)),
        origin=Origin(xyz=(DOOR_W / 2.0, DOOR_T / 2.0 + 0.001, 0.0)),
        material=mirror_mat,
        name="mirror_glass",
    )
    # Small chrome handle near the right (free) edge
    door.visual(
        Cylinder(radius=0.005, length=0.04),
        origin=Origin(
            xyz=(DOOR_W - 0.020, DOOR_T / 2.0 + 0.008, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=handle_mat,
        name="door_handle",
    )
    # Hinge barrel (small cylinder at hinge edge, aligned with Z axis)
    door.visual(
        Cylinder(radius=0.004, length=DOOR_H - 0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=handle_mat,
        name="hinge_barrel",
    )

    # Door hinge articulation: vertical axis at the left edge of the door section
    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(HINGE_X, FRONT_Y, DOOR_ZC)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0, velocity=2.0, lower=0.0, upper=DOOR_OPEN
        ),
    )

    # ------------------------------------------------------------------
    # Tambour sliding panel (prismatic, slides along -X to reveal shelves)
    # ------------------------------------------------------------------
    tambour = model.part("tambour_panel")
    tambour.visual(
        _tambour_panel("tambour_shell"),
        material=tambour_mat,
        name="shell",
    )
    # Small pull tab on the right edge
    tambour.visual(
        Box((0.008, 0.006, 0.03)),
        origin=Origin(xyz=(TAMBOUR_W / 2.0 + 0.004, TAMBOUR_T / 2.0 + 0.003, 0.0)),
        material=handle_mat,
        name="pull_tab",
    )

    # Prismatic articulation: slides along -X to reveal shelves
    # At q=0, tambour covers the shelf opening.
    # At q=TAMBOUR_TRAVEL, it slides fully left to uncover the shelves.
    model.articulation(
        "tambour_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tambour,
        origin=Origin(xyz=(SHELF_SECTION_XC, FRONT_Y - TAMBOUR_T / 2.0, CAB_H / 2.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0, velocity=0.15, lower=0.0, upper=TAMBOUR_TRAVEL
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("cabinet_body")
    door = object_model.get_part("mirror_door")
    tambour = object_model.get_part("tambour_panel")
    hinge = object_model.get_articulation("door_hinge")
    slide = object_model.get_articulation("tambour_slide")

    # --- Overall envelope: narrow bathroom wall cabinet scale ---------------
    aabb = ctx.part_world_aabb(body)
    ok = aabb is not None
    ctx.check("cabinet body has bounds", ok, details=str(aabb))
    if aabb is not None:
        (x0, y0, z0), (x1, y1, z1) = aabb
        ctx.check(
            "cabinet width ~0.45 m",
            0.42 <= (x1 - x0) <= 0.50,
            details=f"width={x1 - x0:.3f}",
        )
        ctx.check(
            "cabinet height ~0.60 m",
            0.57 <= (z1 - z0) <= 0.65,
            details=f"height={z1 - z0:.3f}",
        )
        ctx.check(
            "cabinet depth ~0.15 m (narrow wall cabinet)",
            0.13 <= (y1 - y0) <= 0.22,
            details=f"depth={y1 - y0:.3f}",
        )

    # --- Mirror door: revolute hinge, opens outward -------------------------
    ctx.check(
        "door hinge is revolute",
        hinge.articulation_type == ArticulationType.REVOLUTE,
    )
    ax = hinge.axis
    ctx.check(
        "door hinge axis is vertical",
        abs(ax[0]) < 1e-9 and abs(ax[1]) < 1e-9 and abs(abs(ax[2]) - 1.0) < 1e-9,
        details=str(ax),
    )
    lim = hinge.motion_limits
    ctx.check(
        "door opens 0..~110 deg",
        lim is not None
        and lim.lower == 0.0
        and abs(lim.upper - math.radians(110.0)) < 1e-6,
    )

    # Mirror glass present on the door
    mirror_aabb = ctx.part_element_world_aabb(door, elem="mirror_glass")
    ctx.check(
        "mirror glass is present on the door",
        mirror_aabb is not None,
        details=str(mirror_aabb),
    )
    if mirror_aabb is not None:
        ctx.check(
            "mirror sits near front face when closed",
            abs(mirror_aabb[1][1] - FRONT_Y) < 0.015,
            details=f"mirror_front_y={mirror_aabb[1][1]:.4f}",
        )

    # Open pose: door swings outward
    with ctx.pose({hinge: DOOR_OPEN}):
        open_door_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "open door swings outward past the front face",
        open_door_aabb is not None
        and open_door_aabb[1][1] > FRONT_Y + 0.05,
        details=f"open_door_max_y={open_door_aabb[1][1] if open_door_aabb else None}",
    )

    # --- Tambour: prismatic slide, reveals shelves --------------------------
    ctx.check(
        "tambour slide is prismatic",
        slide.articulation_type == ArticulationType.PRISMATIC,
    )
    slide_ax = slide.axis
    ctx.check(
        "tambour slides along horizontal X axis",
        abs(abs(slide_ax[0]) - 1.0) < 1e-9
        and abs(slide_ax[1]) < 1e-9
        and abs(slide_ax[2]) < 1e-9,
        details=str(slide_ax),
    )
    slide_lim = slide.motion_limits
    ctx.check(
        "tambour has positive travel range",
        slide_lim is not None
        and slide_lim.lower == 0.0
        and slide_lim.upper > 0.05,
        details=f"upper={slide_lim.upper:.3f}" if slide_lim else "no limits",
    )

    # At rest (q=0), tambour covers the shelf section on X
    ctx.expect_overlap(
        tambour,
        body,
        axes="x",
        elem_a="shell",
        elem_b="shelf_board_0",
        min_overlap=0.02,
        name="tambour at rest covers the shelf opening on X",
    )

    # At max slide, tambour moves left, revealing shelves
    tambour_rest = ctx.part_world_aabb(tambour)
    with ctx.pose({slide: TAMBOUR_TRAVEL}):
        tambour_open = ctx.part_world_aabb(tambour)
    ctx.check(
        "tambour slides leftward to reveal shelves",
        tambour_open is not None
        and tambour_rest is not None
        and tambour_open[1][0] < tambour_rest[0][0] + 0.02,
        details=(
            f"rest_min_x={tambour_rest[0][0]:.4f}, open_max_x={tambour_open[1][0]:.4f}"
            if tambour_open and tambour_rest else "missing aabb"
        ),
    )

    # Shelf boards are present at different heights
    shelf0 = ctx.part_element_world_aabb(body, elem="shelf_board_0")
    shelf1 = ctx.part_element_world_aabb(body, elem="shelf_board_1")
    ctx.check("shelf board 0 is present", shelf0 is not None)
    ctx.check("shelf board 1 is present", shelf1 is not None)
    if shelf0 is not None and shelf1 is not None:
        ctx.check(
            "shelves are at different heights",
            abs(shelf0[0][2] - shelf1[0][2]) > 0.05,
            details=f"shelf0_z={shelf0[0][2]:.3f}, shelf1_z={shelf1[0][2]:.3f}",
        )

    # Hinge barrel contacts the door leaf
    ctx.expect_contact(
        door,
        door,
        elem_a="hinge_barrel",
        elem_b="leaf",
        contact_tol=0.005,
        name="hinge barrel is attached to door leaf",
    )

    # Intentional overlaps: hinge barrel near the side wall pivot, and
    # tambour panel nested in the track rail grooves
    ctx.allow_overlap(
        door,
        body,
        elem_a="hinge_barrel",
        elem_b="side_wall_left",
        reason="Hinge barrel intentionally overlaps the side wall edge at the pivot point.",
    )
    ctx.allow_overlap(
        tambour,
        body,
        elem_a="shell",
        elem_b="track_rail_bottom",
        reason="Tambour panel slides within the track rail groove.",
    )
    ctx.allow_overlap(
        tambour,
        body,
        elem_a="shell",
        elem_b="track_rail_top",
        reason="Tambour panel slides within the track rail groove.",
    )

    return ctx.report()


object_model = build_object_model()
