from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

# ---------------------------------------------------------------------------
# Wall-mount bracket dimensions
# ---------------------------------------------------------------------------
WALL_PLATE_W = 0.200
WALL_PLATE_H = 0.150
WALL_PLATE_D = 0.008

BRACKET_BACKPLATE_W = 0.140
BRACKET_BACKPLATE_D = 0.006
BRACKET_BACKPLATE_H = 0.080

BRACKET_ARM_W = 0.030
BRACKET_ARM_H = 0.025
BRACKET_ARM_DEPTH = 0.050

ARM_SPACING = 0.050  # ±50 mm from centre (VESA-style rail spacing)

PIVOT_RADIUS = 0.016
PIVOT_LENGTH = 0.024

# ---------------------------------------------------------------------------
# Screen dimensions — identical to parent baseline
# ---------------------------------------------------------------------------
SCREEN_OUTER_W = 0.585
SCREEN_OUTER_H = 0.375
PANEL_W = 0.512
PANEL_H = 0.320
SCREEN_CENTER_Z = 0.105

# ---------------------------------------------------------------------------
# Derived bracket-local frame positions
# ---------------------------------------------------------------------------
# +Y is toward the wall; -Y is toward the viewer.
# mount_bracket origin sits BRACKET_STANDOFF away from the wall_plate front face.
BRACKET_STANDOFF = 0.0  # bracket backplate is flush against wall plate front face
WALL_PLATE_FRONT_Y = -WALL_PLATE_D / 2.0  # wall plate front face in wall_plate frame
BRACKET_ORIGIN_Y = WALL_PLATE_FRONT_Y - BRACKET_STANDOFF

# In bracket-local frame:
BRACKET_BACKPLATE_Y = -BRACKET_BACKPLATE_D / 2.0  # centre of backplate slab
# Backplate back face sits at bracket y = 0 (bracket origin).

# Tilt axis location (fixed, independent of arm depth)
HINGE_LOCAL_Y = -(BRACKET_BACKPLATE_D + BRACKET_ARM_DEPTH + 0.002)

# Arm extends from backplate front face to just before the barrel front face.
BARREL_RADIUS = 0.014  # matches hinge_barrel on the screen part
ARM_FRONT_FACE_Y = -BRACKET_BACKPLATE_D
ARM_BACK_FACE_Y = HINGE_LOCAL_Y + BARREL_RADIUS + 0.002  # 2 mm clearance
BRACKET_ARM_DEPTH_ACTUAL = ARM_FRONT_FACE_Y - ARM_BACK_FACE_Y
BRACKET_ARM_CENTER_Y = (ARM_FRONT_FACE_Y + ARM_BACK_FACE_Y) / 2.0


# ---------------------------------------------------------------------------
# Object model
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="widescreen_lcd_monitor_wall_mount")

    matte_black = model.material("matte_black", rgba=(0.006, 0.007, 0.007, 1.0))
    soft_black = model.material("soft_black", rgba=(0.018, 0.020, 0.020, 1.0))
    dark_graphite = model.material("dark_graphite", rgba=(0.060, 0.062, 0.060, 1.0))
    lcd_white = model.material("lcd_white", rgba=(0.88, 0.92, 0.95, 1.0))
    dark_silver = model.material("dark_silver", rgba=(0.38, 0.38, 0.35, 1.0))

    # ===================================================================
    # Wall plate (root, mounts flush against the wall — no ground contact)
    # ===================================================================
    wall_plate = model.part("wall_plate")
    wall_plate.visual(
        Box((WALL_PLATE_W, WALL_PLATE_D, WALL_PLATE_H)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=matte_black,
        name="plate_body",
    )

    # ===================================================================
    # Mount bracket (fixed to wall plate, spaces the screen off the wall)
    # ===================================================================
    mount_bracket = model.part("mount_bracket")

    # Thin backplate that bolts against the wall plate front face
    mount_bracket.visual(
        Box((BRACKET_BACKPLATE_W, BRACKET_BACKPLATE_D, BRACKET_BACKPLATE_H)),
        origin=Origin(xyz=(0.0, BRACKET_BACKPLATE_Y, 0.0)),
        material=matte_black,
        name="bracket_backplate",
    )

    # Two horizontal arms + pivot cylinders (loop for repeated sub-parts)
    for i, dx in enumerate((-ARM_SPACING, ARM_SPACING)):
        mount_bracket.visual(
            Box((BRACKET_ARM_W, BRACKET_ARM_DEPTH_ACTUAL, BRACKET_ARM_H)),
            origin=Origin(xyz=(dx, BRACKET_ARM_CENTER_Y, 0.0)),
            material=matte_black,
            name=f"bracket_arm_{i}",
        )
        mount_bracket.visual(
            Cylinder(radius=PIVOT_RADIUS, length=PIVOT_LENGTH),
            origin=Origin(
                xyz=(dx, HINGE_LOCAL_Y, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=dark_silver,
            name=f"bracket_pivot_{i}",
        )

    # ===================================================================
    # Screen (unchanged from parent baseline)
    # ===================================================================
    screen = model.part("screen")
    screen.visual(
        Box((0.568, 0.034, 0.360)),
        origin=Origin(xyz=(0.0, -0.047, SCREEN_CENTER_Z)),
        material=soft_black,
        name="rear_shell",
    )
    screen.visual(
        mesh_from_geometry(
            BezelGeometry(
                (PANEL_W, PANEL_H),
                (SCREEN_OUTER_W, SCREEN_OUTER_H),
                0.020,
                opening_shape="rounded_rect",
                outer_shape="rounded_rect",
                opening_corner_radius=0.004,
                outer_corner_radius=0.018,
            ),
            "screen_bezel",
        ),
        origin=Origin(xyz=(0.0, -0.067, SCREEN_CENTER_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="bezel",
    )
    screen.visual(
        Box((PANEL_W, 0.003, PANEL_H)),
        origin=Origin(xyz=(0.0, -0.076, SCREEN_CENTER_Z)),
        material=lcd_white,
        name="display_panel",
    )
    screen.visual(
        Box((0.074, 0.042, 0.100)),
        origin=Origin(xyz=(0.0, -0.030, 0.0)),
        material=dark_graphite,
        name="rear_mount",
    )
    screen.visual(
        Cylinder(radius=0.014, length=0.086),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="hinge_barrel",
    )

    # ===================================================================
    # Articulations
    # ===================================================================

    # FIXED — wall plate carries the bracket on its front face with standoff
    model.articulation(
        "wall_to_bracket",
        ArticulationType.FIXED,
        parent=wall_plate,
        child=mount_bracket,
        origin=Origin(xyz=(0.0, BRACKET_ORIGIN_Y, 0.0)),
    )

    # REVOLUTE tilt — bracket_to_screen replaces stand_to_screen
    # Axis X so positive q tilts the top of the screen toward the viewer.
    model.articulation(
        "bracket_to_screen",
        ArticulationType.REVOLUTE,
        parent=mount_bracket,
        child=screen,
        origin=Origin(xyz=(0.0, HINGE_LOCAL_Y, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=9.0, velocity=1.5, lower=-0.22, upper=0.20
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wall_plate = object_model.get_part("wall_plate")
    mount_bracket = object_model.get_part("mount_bracket")
    screen = object_model.get_part("screen")
    tilt = object_model.get_articulation("bracket_to_screen")

    # --- Panel aspect ratio (unchanged) ---
    ctx.check(
        "panel is 16:10 widescreen",
        abs((PANEL_W / PANEL_H) - 1.6) < 0.02,
        details=f"panel ratio={PANEL_W / PANEL_H:.3f}",
    )

    # --- Wall plate front face contacts bracket backplate (flush mount) ---
    ctx.expect_contact(
        wall_plate,
        mount_bracket,
        elem_a="plate_body",
        elem_b="bracket_backplate",
        name="wall_plate plate_body contacts bracket_backplate (flush VESA mount)",
    )

    # --- Bracket spaces the screen well away from the wall plate ---
    ctx.expect_gap(
        wall_plate,
        screen,
        axis="y",
        min_gap=0.02,
        positive_elem="plate_body",
        negative_elem="rear_shell",
        name="wall_plate is behind the screen (mount_bracket spaces them apart)",
    )

    # --- Bracket arm contacts its own backplate (structural connectivity) ---
    ctx.expect_contact(
        mount_bracket,
        mount_bracket,
        elem_a="bracket_arm_0",
        elem_b="bracket_backplate",
        name="bracket_arm_0 contacts bracket_backplate",
    )
    ctx.expect_contact(
        mount_bracket,
        mount_bracket,
        elem_a="bracket_arm_1",
        elem_b="bracket_backplate",
        name="bracket_arm_1 contacts bracket_backplate",
    )

    # --- Hinge barrel is centered on the bracket arms in Z ---
    ctx.expect_within(
        screen,
        mount_bracket,
        axes="z",
        margin=0.004,
        inner_elem="hinge_barrel",
        outer_elem="bracket_arm_0",
        name="screen hinge_barrel Z-within bracket_arm_0 (pivot centered)",
    )

    # --- Tilt joint moves display forward/back ---
    with ctx.pose({tilt: -0.20}):
        back_aabb = ctx.part_element_world_aabb(screen, elem="display_panel")
    with ctx.pose({tilt: 0.18}):
        forward_aabb = ctx.part_element_world_aabb(screen, elem="display_panel")

    def _center_y(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return (lo[1] + hi[1]) / 2.0

    back_y = _center_y(back_aabb)
    forward_y = _center_y(forward_aabb)
    ctx.check(
        "bracket_to_screen tilt moves display forward and back",
        back_y is not None and forward_y is not None and forward_y < back_y - 0.025,
        details=f"back_y={back_y}, forward_y={forward_y}",
    )

    # --- Intentional overlaps: pivot cylinders capture hinge-barrel ends ---
    ctx.allow_overlap(
        mount_bracket,
        screen,
        elem_a="bracket_pivot_0",
        elem_b="hinge_barrel",
        reason="Bracket pivot cylinder captures the hinge barrel end (seated pivot).",
    )
    ctx.allow_overlap(
        mount_bracket,
        screen,
        elem_a="bracket_pivot_1",
        elem_b="hinge_barrel",
        reason="Bracket pivot cylinder captures the hinge barrel end (seated pivot).",
    )

    # Proof: pivots are at the barrel ends
    ctx.expect_contact(
        mount_bracket,
        screen,
        elem_a="bracket_pivot_0",
        elem_b="hinge_barrel",
        contact_tol=0.012,
        name="bracket_pivot_0 is at hinge_barrel end",
    )
    ctx.expect_contact(
        mount_bracket,
        screen,
        elem_a="bracket_pivot_1",
        elem_b="hinge_barrel",
        contact_tol=0.012,
        name="bracket_pivot_1 is at hinge_barrel end",
    )

    # --- Bracket arm is clear of the hinge barrel (no bore overlap) ---
    ctx.expect_gap(
        mount_bracket,
        screen,
        axis="y",
        min_gap=0.0,
        max_gap=0.010,
        positive_elem="bracket_arm_0",
        negative_elem="hinge_barrel",
        name="bracket_arm_0 is clear of hinge_barrel (arm stops before barrel)",
    )

    # --- Flush mating surface: wall plate front ↔ bracket backplate back ---
    ctx.allow_coplanar_surfaces(
        wall_plate,
        mount_bracket,
        reason="Bracket backplate bolts flush against the wall plate front face (VESA mount interface).",
        elem_a="plate_body",
        elem_b="bracket_backplate",
    )

    return ctx.report()


object_model = build_object_model()
