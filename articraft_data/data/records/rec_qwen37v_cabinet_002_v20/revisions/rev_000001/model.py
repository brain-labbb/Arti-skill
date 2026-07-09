from __future__ import annotations

# Narrow bathroom wall cabinet with mirrored door (~0.40 m W × 0.65 m H × 0.15 m D).
#
# Layout: back against the wall at x=0, cabinet extends in +X (depth),
# width along Y (centered), height along +Z from z=0.
# White painted carcass; one mirrored door on left-side hinges swings outward
# (REVOLUTE around -Z at left edge); two interior shelves; recessed panel
# border frame around the mirror on the door face.

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)

# ---------- key dimensions (meters) ----------
W = 0.40          # cabinet width (Y)
H = 0.65          # cabinet height (Z)
D = 0.15          # cabinet depth (X, back to front)
WALL = 0.012      # carcass panel thickness

# Inner opening (between top/bottom and side panels)
INNER_W = W - 2 * WALL       # 0.376
INNER_H = H - 2 * WALL       # 0.626
INNER_D = D - WALL            # 0.138

# Door (slightly smaller than opening for reveal gaps)
DOOR_REVEAL = 0.003
DOOR_W = INNER_W - DOOR_REVEAL     # ~0.373
DOOR_H = INNER_H - DOOR_REVEAL     # ~0.623
DOOR_THK = 0.018

# Recessed panel border (raised frame around mirror)
BORDER_W = 0.030
BORDER_PROUD = 0.004          # border strips proud of door face
MIRROR_THK = 0.003
MIRROR_W = DOOR_W - 2 * BORDER_W   # ~0.313
MIRROR_H = DOOR_H - 2 * BORDER_W   # ~0.563

# Shelves
SHELF_THK = 0.010
SHELF_COUNT = 2

# Hinge position: left inner edge of opening, at bottom of opening, front face
HINGE_X = D
HINGE_Y = -(W / 2.0 - WALL)       # left inner edge
HINGE_Z = WALL                      # bottom inner edge

# Knob
KNOB_R = 0.010
KNOB_STEM_R = 0.004
KNOB_STEM_L = 0.012

# Embed for connectivity (prevents disconnected geometry islands)
EMBED = 0.003


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bathroom_wall_cabinet")

    # Materials
    white = model.material("white_paint", rgba=(0.92, 0.92, 0.93, 1.0))
    white_inner = model.material("white_inner", rgba=(0.87, 0.87, 0.89, 1.0))
    mirror_mat = model.material("mirror_glass", rgba=(0.82, 0.87, 0.92, 1.0))
    chrome = model.material("chrome", rgba=(0.78, 0.80, 0.83, 1.0))
    shelf_mat = model.material("shelf_white", rgba=(0.90, 0.90, 0.91, 1.0))

    # ================================================================
    # ROOT: cabinet body (open-front box + interior shelves)
    # ================================================================
    cabinet = model.part("cabinet")

    # Back panel (structural backbone, full width × full height)
    cabinet.visual(
        Box((WALL, W, H)),
        origin=Origin(xyz=(WALL / 2.0, 0.0, H / 2.0)),
        material=white_inner,
        name="back_panel",
    )

    # Side panels (full depth × full height, embed into back panel)
    for tag, s in (("left", -1), ("right", 1)):
        cabinet.visual(
            Box((D + EMBED, WALL, H)),
            origin=Origin(xyz=((D + EMBED) / 2.0 - EMBED,
                               s * (W / 2.0 - WALL / 2.0), H / 2.0)),
            material=white,
            name=f"side_{tag}",
        )

    # Top panel
    cabinet.visual(
        Box((D + EMBED, INNER_W + EMBED * 2, WALL)),
        origin=Origin(xyz=((D + EMBED) / 2.0 - EMBED, 0.0,
                           H - WALL / 2.0)),
        material=white,
        name="top_panel",
    )

    # Bottom panel
    cabinet.visual(
        Box((D + EMBED, INNER_W + EMBED * 2, WALL)),
        origin=Origin(xyz=((D + EMBED) / 2.0 - EMBED, 0.0, WALL / 2.0)),
        material=white,
        name="bottom_panel",
    )

    # Interior shelves (embed into back panel for connectivity)
    for i in range(SHELF_COUNT):
        shelf_z = WALL + (i + 1) * INNER_H / (SHELF_COUNT + 1)
        shelf_dx = INNER_D + EMBED   # slightly into back panel
        cabinet.visual(
            Box((shelf_dx, INNER_W - 0.004, SHELF_THK)),
            origin=Origin(xyz=(WALL - EMBED + shelf_dx / 2.0, 0.0, shelf_z)),
            material=shelf_mat,
            name=f"shelf_{i}",
        )

    # Hinge barrel hardware (small chrome cylinders on left stile)
    hinge_barrel_r = 0.005
    hinge_barrel_h = 0.022
    for tag, hz in (("upper", H - WALL - 0.06), ("lower", WALL + 0.06)):
        cabinet.visual(
            Cylinder(radius=hinge_barrel_r, length=hinge_barrel_h),
            origin=Origin(xyz=(D, HINGE_Y, hz)),
            material=chrome,
            name=f"hinge_barrel_{tag}",
        )

    cabinet.inertial = Inertial.from_geometry(
        Box((D, W, H)), mass=6.0)

    # ================================================================
    # DOOR: mirrored door with recessed panel border
    # ================================================================
    door = model.part("door")

    # Door backing slab (full door panel)
    door.visual(
        Box((DOOR_THK, DOOR_W, DOOR_H)),
        origin=Origin(xyz=(DOOR_THK / 2.0, DOOR_W / 2.0, DOOR_H / 2.0)),
        material=white,
        name="door_panel",
    )

    # Recessed panel border strips (proud of door face, forming frame)
    # These overlap the door front face by EMBED for connectivity.
    bx = DOOR_THK - EMBED + BORDER_PROUD / 2.0  # center x of border strips

    # Top border strip
    door.visual(
        Box((BORDER_PROUD + EMBED, DOOR_W, BORDER_W)),
        origin=Origin(xyz=(bx, DOOR_W / 2.0, DOOR_H - BORDER_W / 2.0)),
        material=white,
        name="border_top",
    )
    # Bottom border strip
    door.visual(
        Box((BORDER_PROUD + EMBED, DOOR_W, BORDER_W)),
        origin=Origin(xyz=(bx, DOOR_W / 2.0, BORDER_W / 2.0)),
        material=white,
        name="border_bottom",
    )
    # Left border strip
    door.visual(
        Box((BORDER_PROUD + EMBED, BORDER_W, DOOR_H - 2 * BORDER_W)),
        origin=Origin(xyz=(bx, BORDER_W / 2.0, DOOR_H / 2.0)),
        material=white,
        name="border_left",
    )
    # Right border strip
    door.visual(
        Box((BORDER_PROUD + EMBED, BORDER_W, DOOR_H - 2 * BORDER_W)),
        origin=Origin(xyz=(bx, DOOR_W - BORDER_W / 2.0, DOOR_H / 2.0)),
        material=white,
        name="border_right",
    )

    # Mirror glass (recessed inside the border frame)
    # Mirror front surface is at door face + MIRROR_THK; border front is at
    # door face + BORDER_PROUD. Since BORDER_PROUD > MIRROR_THK, mirror is
    # visually recessed behind the border frame surface.
    mirror_cx = DOOR_THK - EMBED + MIRROR_THK / 2.0
    door.visual(
        Box((MIRROR_THK + EMBED, MIRROR_W, MIRROR_H)),
        origin=Origin(xyz=(mirror_cx, DOOR_W / 2.0, DOOR_H / 2.0)),
        material=mirror_mat,
        name="mirror",
    )

    # Door knob (right side, opposite hinge, chrome ball on stem)
    knob_y = DOOR_W - 0.025
    knob_z = DOOR_H / 2.0
    door.visual(
        Cylinder(radius=KNOB_STEM_R, length=KNOB_STEM_L + 0.005),
        origin=Origin(xyz=(DOOR_THK + (KNOB_STEM_L + 0.005) / 2.0 - 0.005,
                           knob_y, knob_z),
                      rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="knob_stem",
    )
    door.visual(
        Sphere(radius=KNOB_R),
        origin=Origin(xyz=(DOOR_THK + KNOB_STEM_L + KNOB_R - 0.005,
                           knob_y, knob_z)),
        material=chrome,
        name="knob_ball",
    )

    door.inertial = Inertial.from_geometry(
        Box((DOOR_THK, DOOR_W, DOOR_H)), mass=2.5)

    # ================================================================
    # ARTICULATION: door hinge (revolute, left side, opens outward)
    # ================================================================
    # axis=(0,0,-1): right-hand rule around -Z is clockwise from above,
    # so +Y direction (door width) rotates toward +X (outward from cabinet).
    # Positive q opens the door outward.
    model.articulation(
        "cabinet_to_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(HINGE_X, HINGE_Y, HINGE_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=1.5,
                                   lower=0.0, upper=1.50),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    door = object_model.get_part("door")
    hinge = object_model.get_articulation("cabinet_to_door")

    # --- Hinge barrel embedding (intentional overlap) ---
    # The hinge barrels sit at the hinge pivot, partially embedded in the door
    # panel for structural representation.
    ctx.allow_overlap(
        cabinet, door,
        elem_a="hinge_barrel_upper", elem_b="door_panel",
        reason="Hinge barrel is partially embedded in the door panel at the pivot point.",
    )
    ctx.allow_overlap(
        cabinet, door,
        elem_a="hinge_barrel_lower", elem_b="door_panel",
        reason="Hinge barrel is partially embedded in the door panel at the pivot point.",
    )
    # Proof: hinge barrels are in contact with the door panel
    ctx.expect_contact(
        cabinet, door,
        elem_a="hinge_barrel_upper", elem_b="door_panel",
        contact_tol=0.005,
        name="hinge_barrel_upper_contacts_door",
    )
    ctx.expect_contact(
        cabinet, door,
        elem_a="hinge_barrel_lower", elem_b="door_panel",
        contact_tol=0.005,
        name="hinge_barrel_lower_contacts_door",
    )

    # --- Overall cabinet dimensions (~0.40 × 0.65 × 0.15 m) ---
    ca = ctx.part_world_aabb(cabinet)
    assert ca is not None
    cab_w = ca[1][1] - ca[0][1]
    cab_h = ca[1][2] - ca[0][2]
    cab_d = ca[1][0] - ca[0][0]
    ctx.check("width_narrow", abs(cab_w - W) < 0.015,
              details=f"w={cab_w:.3f}")
    ctx.check("height_065", abs(cab_h - H) < 0.015,
              details=f"h={cab_h:.3f}")
    ctx.check("depth_shallow", abs(cab_d - D) < 0.02,
              details=f"d={cab_d:.3f}")

    # --- Door joint: revolute with sensible range ---
    ctx.check("door_is_revolute",
              hinge.articulation_type == ArticulationType.REVOLUTE)
    lim = hinge.motion_limits
    ctx.check("door_hinge_limits",
              lim is not None and abs(lim.lower) < 1e-9 and lim.upper > 1.0,
              details=f"range=({lim.lower if lim else None},{lim.upper if lim else None})")

    # --- Closed pose: door covers front opening ---
    door_aabb = ctx.part_world_aabb(door)
    assert door_aabb is not None
    # Door front face should be at or past cabinet front (x = D)
    door_front_x = door_aabb[1][0]
    ctx.check("door_at_front",
              door_front_x > D - 0.005,
              details=f"door front x={door_front_x:.4f}")
    # Door width should approximately cover the opening
    door_span_y = door_aabb[1][1] - door_aabb[0][1]
    ctx.check("door_covers_opening_width",
              door_span_y > INNER_W * 0.90,
              details=f"door y span={door_span_y:.3f} vs inner={INNER_W:.3f}")

    # --- Mirror on door face ---
    mirror_aabb = ctx.part_element_world_aabb(door, elem="mirror")
    door_panel_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    assert mirror_aabb is not None and door_panel_aabb is not None
    ctx.check("mirror_on_door_front",
              mirror_aabb[1][0] > door_panel_aabb[1][0] - 0.010,
              details=f"mirror front x={mirror_aabb[1][0]:.4f}")

    # --- Recessed panel border: frame proud of mirror ---
    border_top_aabb = ctx.part_element_world_aabb(door, elem="border_top")
    assert border_top_aabb is not None
    ctx.check("border_proud_of_mirror",
              border_top_aabb[1][0] >= mirror_aabb[1][0] - 0.001,
              details=f"border front={border_top_aabb[1][0]:.4f}, "
                      f"mirror front={mirror_aabb[1][0]:.4f}")

    # All four border strips exist
    for bname in ("border_top", "border_bottom", "border_left", "border_right"):
        b = ctx.part_element_world_aabb(door, elem=bname)
        ctx.check(f"{bname}_exists", b is not None)

    # --- Shelves inside cabinet cavity ---
    for i in range(SHELF_COUNT):
        shelf_aabb = ctx.part_element_world_aabb(cabinet, elem=f"shelf_{i}")
        assert shelf_aabb is not None
        # Shelf should be within cabinet depth (between back panel and front)
        ctx.check(f"shelf_{i}_inside_depth",
                  shelf_aabb[0][0] > 0.005 and shelf_aabb[1][0] < D + 0.005,
                  details=f"shelf x=({shelf_aabb[0][0]:.3f},{shelf_aabb[1][0]:.3f})")
        # Shelf should be within cabinet width
        ctx.check(f"shelf_{i}_inside_width",
                  shelf_aabb[0][1] > ca[0][1] + 0.005
                  and shelf_aabb[1][1] < ca[1][1] - 0.005,
                  details=f"shelf y=({shelf_aabb[0][1]:.3f},{shelf_aabb[1][1]:.3f})")

    # Shelves should be at different heights
    s0 = ctx.part_element_world_aabb(cabinet, elem="shelf_0")
    s1 = ctx.part_element_world_aabb(cabinet, elem="shelf_1")
    assert s0 is not None and s1 is not None
    ctx.check("shelves_at_different_heights",
              abs(s0[0][2] - s1[0][2]) > 0.10,
              details=f"shelf0 z={s0[0][2]:.3f}, shelf1 z={s1[0][2]:.3f}")

    # --- Open pose: door swings outward ---
    # Track the knob ball position (right edge of door) to detect outward swing.
    # The part origin is at the hinge pivot, so it doesn't move; we use the knob.
    rest_knob = ctx.part_element_world_aabb(door, elem="knob_ball")
    rest_center_y = (rest_knob[0][1] + rest_knob[1][1]) / 2.0 if rest_knob else 0.0
    assert rest_knob is not None

    with ctx.pose({hinge: hinge.motion_limits.upper}):
        open_knob = ctx.part_element_world_aabb(door, elem="knob_ball")
        open_door_aabb = ctx.part_world_aabb(door)
    assert open_knob is not None and open_door_aabb is not None

    open_knob_y = (open_knob[0][1] + open_knob[1][1]) / 2.0
    # When door opens, the right-edge (knob) should swing toward -Y (leftward)
    # or move significantly in Y compared to rest position.
    # Also the door AABB should extend past D in X (outward).
    ctx.check("door_swings_outward",
              open_door_aabb[1][0] > D + 0.05,
              details=f"open door max x={open_door_aabb[1][0]:.4f}")

    # --- Shelves visible when door is open ---
    with ctx.pose({hinge: hinge.motion_limits.upper}):
        door_open_aabb = ctx.part_world_aabb(door)
    assert door_open_aabb is not None
    # When door is open, its X extent should be mostly outside the cabinet
    ctx.check("door_clears_opening_when_open",
              door_open_aabb[1][0] > D + 0.05,
              details=f"open door front x={door_open_aabb[1][0]:.3f}")

    # --- Knob exists on door ---
    knob = ctx.part_element_world_aabb(door, elem="knob_ball")
    ctx.check("knob_on_door", knob is not None)

    return ctx.report()


object_model = build_object_model()
