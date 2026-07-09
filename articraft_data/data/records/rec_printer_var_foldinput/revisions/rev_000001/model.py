from __future__ import annotations

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


BODY_W = 0.430
BODY_D = 0.310
BODY_H = 0.130


def _printer_body_shell():
    """Rounded HP DeskJet-like housing with a real front output bay cutout."""
    body = cq.Workplane("XY").box(BODY_W, BODY_D, BODY_H)
    body = body.edges("|Z").fillet(0.030)

    # The front output bay is cut through the rounded front, leaving side cheeks,
    # a top bridge, and a lower sill like a compact inkjet printer.
    bay_cut = (
        cq.Workplane("XY")
        .box(0.305, 0.105, 0.062)
        .translate((0.0, -BODY_D * 0.5 + 0.020, -0.004))
    )
    body = body.cut(bay_cut)

    # Shallow front shadow slot under the scanner lid seam.
    seam_cut = (
        cq.Workplane("XY")
        .box(0.330, 0.010, 0.010)
        .translate((0.0, -BODY_D * 0.5 - 0.002, 0.032))
    )
    body = body.cut(seam_cut)
    return body


def _rounded_panel(width: float, depth: float, height: float, radius: float):
    panel = cq.Workplane("XY").box(width, depth, height)
    return panel.edges("|Z").fillet(radius)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="white_hp_deskjet_all_in_one")

    white = model.material("warm_white_plastic", rgba=(0.94, 0.94, 0.92, 1.0))
    seam_grey = model.material("soft_grey_seams", rgba=(0.70, 0.72, 0.72, 1.0))
    dark = model.material("shadow_black", rgba=(0.05, 0.055, 0.055, 1.0))
    paper_mat = model.material("paper_white", rgba=(0.985, 0.985, 0.965, 1.0))
    glass = model.material("scanner_glass_blue", rgba=(0.55, 0.68, 0.76, 0.40))
    accent = model.material("yellow_green_extension", rgba=(0.78, 0.95, 0.02, 1.0))
    ink_blue = model.material("hp_blue_mark", rgba=(0.18, 0.44, 0.76, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_printer_body_shell(), "deskjet_body_shell", tolerance=0.001),
        origin=Origin(xyz=(0.0, 0.0, BODY_H * 0.5)),
        material=white,
        name="rounded_body",
    )
    body.visual(
        Box((0.315, 0.006, 0.052)),
        origin=Origin(xyz=(0.0, -0.107, 0.060)),
        material=dark,
        name="output_shadow",
    )
    body.visual(
        Box((0.060, 0.014, 0.004)),
        origin=Origin(xyz=(-0.110, -0.105, 0.033)),
        material=dark,
        name="feed_roller_0",
    )
    body.visual(
        Box((0.060, 0.014, 0.004)),
        origin=Origin(xyz=(0.000, -0.105, 0.033)),
        material=dark,
        name="feed_roller_1",
    )
    body.visual(
        Box((0.060, 0.014, 0.004)),
        origin=Origin(xyz=(0.110, -0.105, 0.033)),
        material=dark,
        name="feed_roller_2",
    )
    body.visual(
        Box((0.050, 0.060, 0.004)),
        origin=Origin(xyz=(-0.194, -0.095, BODY_H + 0.002)),
        material=seam_grey,
        name="control_deck",
    )
    body.visual(
        Box((0.010, 0.010, 0.0015)),
        origin=Origin(xyz=(-0.207, -0.115, BODY_H + 0.00475)),
        material=ink_blue,
        name="hp_mark",
    )
    body.visual(
        Box((0.037, 0.003, 0.0015)),
        origin=Origin(xyz=(-0.193, -0.125, BODY_H + 0.00475)),
        material=seam_grey,
        name="deskjet_label",
    )
    body.visual(
        Box((0.070, 0.018, 0.004)),
        origin=Origin(xyz=(-0.105, 0.129, BODY_H + 0.000)),
        material=white,
        name="hinge_saddle_0",
    )
    body.visual(
        Box((0.070, 0.018, 0.004)),
        origin=Origin(xyz=(0.105, 0.129, BODY_H + 0.000)),
        material=white,
        name="hinge_saddle_1",
    )

    scanner_lid = model.part("scanner_lid")
    scanner_lid.visual(
        mesh_from_cadquery(
            _rounded_panel(0.350, 0.250, 0.018, 0.022),
            "scanner_lid_panel",
            tolerance=0.001,
        ),
        # Child frame is on the rear hinge line; the closed lid extends forward
        # in local -Y, so positive motion about -X lifts the front edge.
        origin=Origin(xyz=(0.0, -0.125, 0.009)),
        material=white,
        name="lid_panel",
    )
    scanner_lid.visual(
        Box((0.318, 0.212, 0.002)),
        origin=Origin(xyz=(0.0, -0.123, 0.001)),
        material=glass,
        name="glass_underlay",
    )
    scanner_lid.visual(
        Cylinder(radius=0.006, length=0.290),
        origin=Origin(xyz=(0.0, 0.000, 0.006), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=white,
        name="hinge_barrel",
    )
    model.articulation(
        "body_to_scanner_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=scanner_lid,
        origin=Origin(xyz=(0.0, 0.129, BODY_H + 0.002)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.6, lower=0.0, upper=1.25),
    )

    rear_tray = model.part("rear_tray")
    # At q=0 the tray lies flat on the body top (extending forward in -Y from
    # the rear hinge).  Visuals use rpy=(π/2,0,0) so their tall Z dimension
    # becomes the Y extent when flat; when the part rotates up about -X the
    # assembly stands vertical for paper feeding.
    rear_tray.visual(
        Box((0.300, 0.026, 0.010)),
        origin=Origin(xyz=(0.0, -0.013, 0.005)),
        material=white,
        name="tray_base",
    )
    rear_tray.visual(
        Box((0.280, 0.012, 0.205)),
        origin=Origin(xyz=(0.0, -0.128, 0.016), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=white,
        name="upright_support",
    )
    rear_tray.visual(
        Box((0.230, 0.006, 0.026)),
        origin=Origin(xyz=(0.0, -0.003, 0.013), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=seam_grey,
        name="paper_slot_lip",
    )
    model.articulation(
        "body_to_rear_tray",
        ArticulationType.REVOLUTE,
        parent=body,
        child=rear_tray,
        origin=Origin(xyz=(0.0, BODY_D * 0.5, BODY_H)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=1.45),
    )

    paper_stack = model.part("paper_stack")
    # Paper rides with the fold: at q=0 it lies flat on top of the tray
    # assembly; at q=upper it stands upright behind the printer.
    paper_stack.visual(
        Box((0.232, 0.003, 0.180)),
        origin=Origin(xyz=(0.0, -0.116, 0.024), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=paper_mat,
        name="standing_paper",
    )
    paper_stack.visual(
        Box((0.232, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, -0.030, 0.024), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=paper_mat,
        name="paper_feet",
    )
    model.articulation(
        "rear_tray_to_paper_stack",
        ArticulationType.FIXED,
        parent=rear_tray,
        child=paper_stack,
        origin=Origin(),
    )

    output_tray = model.part("output_tray")
    output_tray.visual(
        mesh_from_cadquery(
            _rounded_panel(0.330, 0.135, 0.012, 0.012),
            "output_tray_plate",
            tolerance=0.001,
        ),
        origin=Origin(xyz=(0.0, -0.0675, -0.006)),
        material=white,
        name="tray_plate",
    )
    output_tray.visual(
        Box((0.275, 0.018, 0.004)),
        origin=Origin(xyz=(0.0, -0.124, 0.002)),
        material=seam_grey,
        name="extension_recess",
    )
    model.articulation(
        "body_to_output_tray",
        ArticulationType.FIXED,
        parent=body,
        child=output_tray,
        origin=Origin(xyz=(0.0, -BODY_D * 0.5 + 0.004, 0.030)),
    )

    extension_arm = model.part("extension_arm")
    extension_arm.visual(
        Box((0.255, 0.030, 0.006)),
        origin=Origin(xyz=(0.0, -0.014, 0.003)),
        material=accent,
        name="yellow_green_arm",
    )
    extension_arm.visual(
        Box((0.022, 0.062, 0.004)),
        origin=Origin(xyz=(0.0, 0.020, 0.002)),
        material=accent,
        name="sliding_tongue",
    )
    model.articulation(
        "output_tray_to_extension_arm",
        ArticulationType.PRISMATIC,
        parent=output_tray,
        child=extension_arm,
        origin=Origin(xyz=(0.0, -0.103, 0.000)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=0.15, lower=0.0, upper=0.070),
    )

    for name, x, y, radius, color in (
        ("power_button", -0.202, -0.087, 0.0060, ink_blue),
        ("cancel_button", -0.187, -0.103, 0.0045, seam_grey),
        ("wireless_button", -0.202, -0.119, 0.0045, seam_grey),
    ):
        button = model.part(name)
        button.visual(
            Cylinder(radius=radius, length=0.004),
            origin=Origin(xyz=(0.0, 0.0, 0.002)),
            material=color,
            name="button_cap",
        )
        model.articulation(
            f"body_to_{name}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x, y, BODY_H + 0.004)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=1.0, velocity=0.05, lower=0.0, upper=0.002),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("scanner_lid")
    output_tray = object_model.get_part("output_tray")
    extension = object_model.get_part("extension_arm")
    rear_tray = object_model.get_part("rear_tray")
    paper = object_model.get_part("paper_stack")
    lid_joint = object_model.get_articulation("body_to_scanner_lid")
    extension_joint = object_model.get_articulation("output_tray_to_extension_arm")

    # The push buttons sit in a simplified solid control deck. At depressed
    # poses their caps intentionally pass through the unmodeled apertures.
    for button_name in ("power_button", "cancel_button", "wireless_button"):
        ctx.allow_overlap(
            body,
            button_name,
            elem_a="control_deck",
            elem_b="button_cap",
            reason="Button travel is represented by caps depressing through simplified control-panel apertures.",
        )
        ctx.expect_contact(
            body,
            button_name,
            elem_a="control_deck",
            elem_b="button_cap",
            contact_tol=0.001,
            name=f"{button_name}_seated_on_control_deck",
        )

    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        min_overlap=0.20,
        elem_a="lid_panel",
        elem_b="rounded_body",
        name="scanner_lid_covers_flatbed_area",
    )
    ctx.expect_gap(
        lid,
        body,
        axis="z",
        min_gap=0.0,
        max_gap=0.004,
        positive_elem="lid_panel",
        negative_elem="rounded_body",
        name="closed_scanner_lid_sits_above_body",
    )
    ctx.expect_contact(
        lid,
        body,
        elem_a="hinge_barrel",
        elem_b="hinge_saddle_0",
        contact_tol=0.002,
        name="scanner_hinge_supported",
    )

    closed_lid_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_joint: 0.85}):
        open_lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "scanner_lid_opens_upward",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and float(open_lid_aabb[1][2] - closed_lid_aabb[1][2]) > 0.045,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    ctx.expect_overlap(
        extension,
        output_tray,
        axes="x",
        min_overlap=0.22,
        elem_a="yellow_green_arm",
        elem_b="tray_plate",
        name="accent_extension_spans_output_tray",
    )
    rest_extension_aabb = ctx.part_world_aabb(extension)
    with ctx.pose({extension_joint: 0.060}):
        extended_extension_aabb = ctx.part_world_aabb(extension)
        ctx.expect_overlap(
            extension,
            output_tray,
            axes="y",
            min_overlap=0.010,
            elem_a="sliding_tongue",
            elem_b="tray_plate",
            name="extension_retains_sliding_tongue",
        )
    ctx.check(
        "extension_pulls_forward",
        rest_extension_aabb is not None
        and extended_extension_aabb is not None
        and float(rest_extension_aabb[0][1] - extended_extension_aabb[0][1]) > 0.045,
        details=f"rest={rest_extension_aabb}, extended={extended_extension_aabb}",
    )

    rear_tray_joint = object_model.get_articulation("body_to_rear_tray")

    # At rest (q=0) the rear tray lies flat on the body top and the closed
    # scanner lid sits over it.  Allow the intentional stacking overlap.
    ctx.allow_overlap(
        lid,
        rear_tray,
        reason="The rear paper support folds flat against the body top beneath the closed scanner lid.",
    )
    ctx.allow_overlap(
        lid,
        paper,
        reason="Paper in the folded-flat rear tray lies beneath the closed scanner lid.",
    )
    ctx.allow_overlap(
        body,
        rear_tray,
        reason="The tray base and upright support pass near hinge saddles when folded flat against the body top.",
    )

    # Verify the body_to_rear_tray REVOLUTE fold: flat at q=0, upright at q=upper.
    flat_tray_aabb = ctx.part_world_aabb(rear_tray)
    flat_paper_aabb = ctx.part_world_aabb(paper)
    body_aabb = ctx.part_world_aabb(body)
    with ctx.pose({rear_tray_joint: 1.45}):
        upright_tray_aabb = ctx.part_world_aabb(rear_tray)
        upright_paper_aabb = ctx.part_world_aabb(paper)

    ctx.check(
        "rear_tray_folds_up_from_flat",
        flat_tray_aabb is not None
        and upright_tray_aabb is not None
        and body_aabb is not None
        and float(upright_tray_aabb[1][2] - flat_tray_aabb[1][2]) > 0.10,
        details=f"flat={flat_tray_aabb}, upright={upright_tray_aabb}",
    )
    ctx.check(
        "paper_rides_with_rear_tray_fold",
        flat_paper_aabb is not None
        and upright_paper_aabb is not None
        and float(upright_paper_aabb[1][2] - flat_paper_aabb[1][2]) > 0.08,
        details=f"flat={flat_paper_aabb}, upright={upright_paper_aabb}",
    )
    ctx.check(
        "rear_tray_flat_at_rest_near_body_top",
        flat_tray_aabb is not None
        and body_aabb is not None
        and flat_tray_aabb[1][2] < body_aabb[1][2] + 0.04,
        details=f"body_top={body_aabb[1][2]}, tray_max_z={flat_tray_aabb[1][2] if flat_tray_aabb else None}",
    )

    return ctx.report()


object_model = build_object_model()
