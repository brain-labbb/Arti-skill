from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


WHITE = Material("warm_white_plastic", color=(0.92, 0.93, 0.91, 1.0))
LIGHT_GRAY = Material("light_gray_plastic", color=(0.64, 0.66, 0.65, 1.0))
DARK_GRAY = Material("charcoal_gray_plastic", color=(0.16, 0.17, 0.17, 1.0))
BLACK = Material("gloss_black", color=(0.01, 0.012, 0.014, 1.0))
SCREEN = Material("lit_color_screen", color=(0.05, 0.12, 0.22, 1.0))
CYAN = Material("cyan_touch_icon", color=(0.0, 0.65, 0.95, 1.0))
GREEN = Material("green_touch_icon", color=(0.08, 0.70, 0.25, 1.0))
PAPER = Material("warm_copy_paper", color=(0.96, 0.94, 0.88, 1.0))
LOGO_GRAY = Material("hp_logo_gray", color=(0.45, 0.47, 0.47, 1.0))


def add_box(part, name: str, size, xyz, material):
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def add_front_plate(part, name: str, size_xz, xyz, material, thickness: float = 0.004):
    """Thin feature mounted to the printer's front face (front is -Y)."""
    sx, sz = size_xz
    part.visual(
        Box((sx, thickness, sz)),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hp_officejet_pro_all_in_one")

    # Printer frame dimensions: roughly a desktop OfficeJet footprint.
    width = 0.50
    depth = 0.42
    front_y = -depth / 2.0
    rear_y = depth / 2.0

    chassis = model.part("chassis")

    # Boxy white upper printer/scanner body, with the lower base split around the
    # pull-out paper drawer so the drawer has a real open bay instead of sitting
    # inside a solid block.
    add_box(chassis, "upper_white_shell", (width, depth, 0.170), (0.0, 0.0, 0.155), WHITE)
    add_box(chassis, "scanner_bed_frame", (width, depth, 0.035), (0.0, 0.0, 0.2575), WHITE)
    add_box(chassis, "lower_side_0", (0.130, depth, 0.070), (-0.185, 0.0, 0.035), DARK_GRAY)
    add_box(chassis, "lower_side_1", (0.130, depth, 0.070), (0.185, 0.0, 0.035), DARK_GRAY)
    add_box(chassis, "front_white_panel", (width, 0.010, 0.138), (0.0, front_y - 0.002, 0.159), WHITE)

    # Flatbed scanner details visible when the lid opens: a dark glass inset in a
    # shallow gray surround.
    add_box(chassis, "scanner_glass", (0.355, 0.285, 0.004), (0.025, -0.020, 0.2770), BLACK)
    add_box(chassis, "scanner_glass_highlight", (0.320, 0.245, 0.002), (0.025, -0.020, 0.2790), Material("dark_blue_glass", color=(0.02, 0.04, 0.07, 1.0)))
    add_box(chassis, "rear_lid_seat", (width, 0.018, 0.012), (0.0, rear_y - 0.009, 0.278), LIGHT_GRAY)

    # Front-left touchscreen/control area: black raised bezel, lit display, and
    # colored app tiles on the screen.
    add_front_plate(chassis, "control_panel_bezel", (0.096, 0.086), (-0.178, front_y - 0.007, 0.181), BLACK, thickness=0.010)
    add_front_plate(chassis, "touchscreen_glass", (0.068, 0.052), (-0.178, front_y - 0.012, 0.181), SCREEN, thickness=0.006)
    add_front_plate(chassis, "screen_icon_0", (0.014, 0.018), (-0.198, front_y - 0.015, 0.181), CYAN, thickness=0.004)
    add_front_plate(chassis, "screen_icon_1", (0.014, 0.018), (-0.178, front_y - 0.015, 0.181), GREEN, thickness=0.004)
    add_front_plate(chassis, "screen_icon_2", (0.014, 0.018), (-0.158, front_y - 0.015, 0.181), Material("magenta_touch_icon", color=(0.70, 0.20, 0.75, 1.0)), thickness=0.004)
    add_front_plate(chassis, "screen_status_bar", (0.055, 0.006), (-0.178, front_y - 0.015, 0.204), WHITE, thickness=0.003)

    # Central output slot and the short output tray shelf, including a sheet of
    # paper protruding from the slot.
    add_front_plate(chassis, "output_slot_shadow", (0.325, 0.040), (0.040, front_y - 0.006, 0.128), BLACK, thickness=0.008)
    add_box(chassis, "output_tray_shelf", (0.310, 0.085, 0.014), (0.030, front_y - 0.040, 0.102), LIGHT_GRAY)
    add_box(chassis, "output_tray_lip", (0.255, 0.018, 0.034), (0.030, front_y - 0.082, 0.112), DARK_GRAY)
    add_box(chassis, "printed_sheet", (0.280, 0.120, 0.004), (0.030, front_y - 0.052, 0.123), PAPER)

    # HP badge on the front, approximated with a round gray emblem and white
    # strokes; mounted proud of the front panel.
    chassis.visual(
        Cylinder(radius=0.020, length=0.003),
        origin=Origin(xyz=(0.005, front_y - 0.008, 0.172), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=LOGO_GRAY,
        name="hp_roundel",
    )
    add_front_plate(chassis, "hp_stroke_0", (0.004, 0.025), (-0.004, front_y - 0.010, 0.172), WHITE, thickness=0.003)
    add_front_plate(chassis, "hp_stroke_1", (0.004, 0.021), (0.006, front_y - 0.010, 0.170), WHITE, thickness=0.003)
    add_front_plate(chassis, "hp_crossbar", (0.019, 0.003), (0.002, front_y - 0.010, 0.177), WHITE, thickness=0.003)

    # Small seams and cartridge/access door cues on the front lower panels.
    add_front_plate(chassis, "left_lower_seam", (0.002, 0.065), (-0.120, front_y - 0.002, 0.035), BLACK, thickness=0.006)
    add_front_plate(chassis, "right_lower_label", (0.060, 0.024), (0.195, front_y - 0.003, 0.033), Material("dark_label", color=(0.08, 0.09, 0.09, 1.0)), thickness=0.006)
    add_front_plate(chassis, "label_mark", (0.030, 0.004), (0.195, front_y - 0.006, 0.037), WHITE, thickness=0.004)

    # Hinged flatbed/ADF lid.  The child frame is the rear hinge line; the closed
    # slab extends along local -Y, so the -X hinge axis makes positive q lift the
    # front edge upward.
    scanner_lid = model.part("scanner_lid")
    add_box(scanner_lid, "flatbed_lid_slab", (width, depth, 0.025), (0.0, -depth / 2.0, 0.0165), LIGHT_GRAY)
    add_box(scanner_lid, "adf_upper_cover", (0.440, 0.260, 0.042), (0.000, -0.205, 0.050), DARK_GRAY)
    add_box(scanner_lid, "adf_white_feed_face", (0.410, 0.032, 0.020), (0.000, -0.330, 0.040), WHITE)
    add_box(scanner_lid, "adf_paper_path_shadow", (0.350, 0.020, 0.010), (0.000, -0.335, 0.056), BLACK)
    add_box(scanner_lid, "adf_top_tray", (0.360, 0.135, 0.012), (0.000, -0.155, 0.077), LIGHT_GRAY)
    add_box(scanner_lid, "adf_rear_hinge_strip", (0.460, 0.018, 0.018), (0.000, -0.006, 0.020), DARK_GRAY)
    # Low front bevel detail on the ADF lid.
    add_box(scanner_lid, "adf_front_bevel", (0.420, 0.030, 0.014), (0.000, -0.337, 0.024), DARK_GRAY)

    model.articulation(
        "chassis_to_scanner_lid",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=scanner_lid,
        origin=Origin(xyz=(0.0, rear_y - 0.002, 0.280)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=0.0, upper=1.20),
    )

    # Pull-out bottom paper input drawer.  It is narrow enough to run between the
    # two dark lower side blocks and long enough to remain retained when extended.
    input_tray = model.part("input_tray")
    add_box(input_tray, "tray_floor", (0.218, 0.410, 0.012), (0.0, 0.205, 0.008), DARK_GRAY)
    add_box(input_tray, "tray_side_0", (0.012, 0.410, 0.040), (-0.114, 0.205, 0.026), LIGHT_GRAY)
    add_box(input_tray, "tray_side_1", (0.012, 0.410, 0.040), (0.114, 0.205, 0.026), LIGHT_GRAY)
    add_box(input_tray, "tray_front_panel", (0.240, 0.022, 0.062), (0.0, -0.011, 0.036), DARK_GRAY)
    add_box(input_tray, "tray_grip_recess", (0.105, 0.006, 0.016), (0.0, -0.025, 0.036), BLACK)
    add_box(input_tray, "tray_paper_stack", (0.190, 0.285, 0.014), (0.0, 0.205, 0.021), PAPER)

    model.articulation(
        "chassis_to_input_tray",
        ArticulationType.PRISMATIC,
        parent=chassis,
        child=input_tray,
        origin=Origin(xyz=(0.0, front_y - 0.006, 0.004)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.25, lower=0.0, upper=0.160),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    chassis = object_model.get_part("chassis")
    scanner_lid = object_model.get_part("scanner_lid")
    input_tray = object_model.get_part("input_tray")
    lid_joint = object_model.get_articulation("chassis_to_scanner_lid")
    tray_joint = object_model.get_articulation("chassis_to_input_tray")

    with ctx.pose({lid_joint: 0.0, tray_joint: 0.0}):
        ctx.expect_gap(
            scanner_lid,
            chassis,
            axis="z",
            min_gap=0.0,
            max_gap=0.010,
            name="closed scanner lid sits just above scanner bed",
        )
        ctx.expect_overlap(
            scanner_lid,
            chassis,
            axes="xy",
            min_overlap=0.34,
            name="scanner lid covers the flatbed footprint",
        )
        ctx.expect_within(
            input_tray,
            chassis,
            axes="x",
            margin=0.002,
            name="input drawer is centered between lower side blocks",
        )

    closed_lid_aabb = ctx.part_world_aabb(scanner_lid)
    with ctx.pose({lid_joint: 0.90}):
        open_lid_aabb = ctx.part_world_aabb(scanner_lid)
    ctx.check(
        "scanner lid opens upward",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.10,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    closed_tray_pos = ctx.part_world_position(input_tray)
    with ctx.pose({tray_joint: 0.160}):
        extended_tray_pos = ctx.part_world_position(input_tray)
        ctx.expect_overlap(
            input_tray,
            chassis,
            axes="y",
            min_overlap=0.035,
            name="extended input tray remains retained in printer bay",
        )
    ctx.check(
        "input tray pulls out from the front",
        closed_tray_pos is not None
        and extended_tray_pos is not None
        and extended_tray_pos[1] < closed_tray_pos[1] - 0.12,
        details=f"closed={closed_tray_pos}, extended={extended_tray_pos}",
    )

    return ctx.report()


object_model = build_object_model()
