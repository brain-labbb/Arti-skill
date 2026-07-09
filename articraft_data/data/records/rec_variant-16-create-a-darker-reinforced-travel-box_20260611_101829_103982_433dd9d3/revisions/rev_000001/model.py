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
    TorusGeometry,
    mesh_from_geometry,
)

# --- Dimensions (meters) -----------------------------------------------------
W = 0.50          # width (X)
D = 0.34          # depth (Y)
HB = 0.24         # body height (Z)
T = 0.018         # plank wall thickness
LID_T = 0.040     # flat lid thickness
SEAM_GAP = 0.0    # lid rests directly on the body rim


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="reinforced_travel_box")

    wood = model.material("dark_oiled_plank", rgba=(0.22, 0.13, 0.07, 1.0))
    wood_dark = model.material("endgrain_dark", rgba=(0.13, 0.08, 0.045, 1.0))
    iron = model.material("blackened_iron", rgba=(0.06, 0.065, 0.07, 1.0))
    worn_iron = model.material("worn_edge_iron", rgba=(0.20, 0.20, 0.19, 1.0))

    # --- Body (root) ---------------------------------------------------------
    body = model.part("box_body")
    body.visual(Box((W, D, T)), origin=Origin(xyz=(0.0, 0.0, T / 2.0)),
                material=wood_dark, name="floor_panel")
    cz = HB / 2.0
    body.visual(Box((W, T, HB)), origin=Origin(xyz=(0.0, -(D / 2.0 - T / 2.0), cz)),
                material=wood, name="wall_front")
    body.visual(Box((W, T, HB)), origin=Origin(xyz=(0.0, (D / 2.0 - T / 2.0), cz)),
                material=wood, name="wall_back")
    body.visual(Box((T, D, HB)), origin=Origin(xyz=(-(W / 2.0 - T / 2.0), 0.0, cz)),
                material=wood, name="wall_left")
    body.visual(Box((T, D, HB)), origin=Origin(xyz=((W / 2.0 - T / 2.0), 0.0, cz)),
                material=wood, name="wall_right")

    # blackened iron corner brackets at the four vertical edges
    bk = 0.030
    idx = 0
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            body.visual(
                Box((bk, bk, HB - 0.02)),
                origin=Origin(xyz=(sx * (W / 2.0 - bk / 2.0 + 0.003),
                                   sy * (D / 2.0 - bk / 2.0 + 0.003), HB / 2.0)),
                material=iron, name=f"corner_bracket_{idx}",
            )
            idx += 1

    # Non-decorative travel-box reinforcement: metal strap bands wrap around
    # the body and tie the plank sides together.
    for z, tag in ((0.055, "lower"), (HB - 0.055, "upper")):
        if tag == "upper":
            # The upper front band is broken around the center hasp so the
            # latch can sit over wood instead of colliding with the strap.
            for sx, suffix in ((-1.0, "0"), (1.0, "1")):
                body.visual(Box((0.205, 0.018, 0.032)),
                            origin=Origin(xyz=(sx * 0.1625, -(D / 2.0 + 0.011), z)),
                            material=iron, name=f"{tag}_front_band_{suffix}")
        else:
            body.visual(Box((W + 0.030, 0.018, 0.032)),
                        origin=Origin(xyz=(0.0, -(D / 2.0 + 0.011), z)),
                        material=iron, name=f"{tag}_front_band")
        body.visual(Box((W + 0.030, 0.018, 0.032)),
                    origin=Origin(xyz=(0.0, (D / 2.0 + 0.011), z)),
                    material=iron, name=f"{tag}_rear_band")
        body.visual(Box((0.018, D + 0.030, 0.032)),
                    origin=Origin(xyz=(-(W / 2.0 + 0.011), 0.0, z)),
                    material=iron, name=f"{tag}_side_band_0")
        body.visual(Box((0.018, D + 0.030, 0.032)),
                    origin=Origin(xyz=((W / 2.0 + 0.011), 0.0, z)),
                    material=iron, name=f"{tag}_side_band_1")

    # vertical iron straps on the lid-facing travel-box sides
    for sx in (-0.14, 0.14):
        body.visual(Box((0.030, 0.018, HB + 0.006)),
                    origin=Origin(xyz=(sx, -(D / 2.0 + 0.012), HB / 2.0)),
                    material=iron, name=f"front_vertical_strap_{0 if sx < 0 else 1}")
        body.visual(Box((0.030, 0.018, HB + 0.006)),
                    origin=Origin(xyz=(sx, (D / 2.0 + 0.012), HB / 2.0)),
                    material=iron, name=f"rear_vertical_strap_{0 if sx < 0 else 1}")

    # side handle mounting blocks and hinge pins for rotating iron rings
    for sx, plate_name, clevis_0_name, clevis_1_name, pin_name in (
        (-1.0, "side_0_handle_plate", "side_0_clevis_0", "side_0_clevis_1", "side_0_handle_pin"),
        (1.0, "side_1_handle_plate", "side_1_clevis_0", "side_1_clevis_1", "side_1_handle_pin"),
    ):
        xface = sx * (W / 2.0)
        # backplate and two cheeks make the ring visibly mounted, not floating.
        body.visual(Box((0.014, 0.120, 0.056)),
                    origin=Origin(xyz=(xface + sx * 0.009, 0.0, HB * 0.66)),
                    material=iron, name=plate_name)
        body.visual(Box((0.032, 0.016, 0.058)),
                    origin=Origin(xyz=(xface + sx * 0.026, -0.036, HB * 0.66)),
                    material=iron, name=clevis_0_name)
        body.visual(Box((0.032, 0.016, 0.058)),
                    origin=Origin(xyz=(xface + sx * 0.026, 0.036, HB * 0.66)),
                    material=iron, name=clevis_1_name)
        body.visual(
            Cylinder(radius=0.007, length=0.105),
            origin=Origin(xyz=(xface + sx * 0.034, 0.0, HB * 0.66),
                          rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=worn_iron, name=pin_name,
        )

    # front staple that receives the hasp
    body.visual(Box((0.04, 0.012, 0.03)),
                origin=Origin(xyz=(0.0, -(D / 2.0) - 0.006, HB * 0.45)),
                material=iron, name="hasp_keeper")

    # --- Lid (flat, hinged at the back top edge) -----------------------------
    lid = model.part("box_lid")
    lid.visual(Box((W + 0.02, D + 0.02, LID_T)),
               origin=Origin(xyz=(0.0, -D / 2.0, SEAM_GAP + LID_T / 2.0)),
               material=wood, name="lid_panel")
    # iron straps across the lid, aligned with body travel bands
    for sx in (-0.13, 0.13):
        lid.visual(Box((0.028, D + 0.02, 0.006)),
                   origin=Origin(xyz=(sx, -D / 2.0, SEAM_GAP + LID_T + 0.003)),
                   material=iron, name=f"lid_strap_{'l' if sx < 0 else 'r'}")
    # hinge mount tab that carries the front hasp (reaches out to contact it)
    lid.visual(Box((0.05, 0.020, 0.016)),
               origin=Origin(xyz=(0.0, -(D + 0.003), SEAM_GAP + LID_T * 0.5)),
               material=iron, name="hasp_mount")

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, D / 2.0, HB)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=2.0, lower=0.0, upper=2.1),
    )

    # --- Rotating side ring handles -----------------------------------------
    ring_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.064, tube=0.0075, radial_segments=16, tubular_segments=64),
        "travel_box_ring_handle",
    )
    for sx, name in ((-1.0, "ring_handle_0"), (1.0, "ring_handle_1")):
        ring = model.part(name)
        # Torus lies in the side-face plane (YZ); part origin is the hinge pin.
        ring.visual(
            ring_mesh,
            origin=Origin(xyz=(0.0, 0.0, -0.064), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=worn_iron,
            name="ring",
        )
        ring.visual(
            Box((0.018, 0.030, 0.010)),
            origin=Origin(xyz=(0.0, 0.0, -0.003)),
            material=worn_iron,
            name="top_saddle",
        )
        model.articulation(
            f"{name}_pivot",
            ArticulationType.REVOLUTE,
            parent=body,
            child=ring,
            origin=Origin(xyz=(sx * (W / 2.0 + 0.034), 0.0, HB * 0.66)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=4.0, lower=-1.35, upper=1.35),
        )

    # --- Front hasp (hinged on the lid front edge, swings to latch) ----------
    hasp = model.part("hasp")
    hasp.visual(Box((0.05, 0.010, 0.10)),
                origin=Origin(xyz=(0.0, 0.0, -0.05)),
                material=iron, name="hasp_arm")
    hasp.visual(Box((0.024, 0.014, 0.018)),
                origin=Origin(xyz=(0.0, -0.004, -0.092)),
                material=iron, name="hasp_eye")
    # hinge at the lid front edge, proud of the body front face; q=0 is closed
    # (arm hangs down over the front), positive q lifts it to release.
    model.articulation(
        "hasp_hinge",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=hasp,
        origin=Origin(xyz=(0.0, -(D + 0.018), SEAM_GAP + LID_T * 0.5)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=3.0, lower=0.0, upper=1.4),
    )

    return model


object_model = build_object_model()


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("box_body")
    lid = object_model.get_part("box_lid")
    hasp = object_model.get_part("hasp")
    ring_0 = object_model.get_part("ring_handle_0")
    ring_1 = object_model.get_part("ring_handle_1")
    lid_hinge = object_model.get_articulation("lid_hinge")
    hasp_hinge = object_model.get_articulation("hasp_hinge")
    ring_pivot_0 = object_model.get_articulation("ring_handle_0_pivot")

    # Closed lid seats on the body rim without penetrating it.
    with ctx.pose({lid_hinge: 0.0, hasp_hinge: 0.0}):
        ctx.expect_gap(lid, body, axis="z", min_gap=0.0, max_gap=0.006,
                       positive_elem="lid_panel", negative_elem="wall_front",
                       name="lid seats on the body rim when closed")
        ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.25,
                           name="closed lid covers the body opening")
        closed_lid = ctx.part_world_aabb(lid)

    with ctx.pose({lid_hinge: 1.9}):
        open_lid = ctx.part_world_aabb(lid)
    ctx.check(
        "lid opens upward",
        closed_lid is not None and open_lid is not None
        and open_lid[1][2] > closed_lid[1][2] + 0.10,
        details=f"closed={closed_lid}, open={open_lid}",
    )

    # Hasp releases when lifted.
    with ctx.pose({lid_hinge: 0.0, hasp_hinge: 0.0}):
        hasp_closed = ctx.part_world_aabb(hasp)
    with ctx.pose({lid_hinge: 0.0, hasp_hinge: 1.3}):
        hasp_open = ctx.part_world_aabb(hasp)
    ctx.check(
        "front hasp lifts to release",
        hasp_closed is not None and hasp_open is not None
        and hasp_open[0][2] > hasp_closed[0][2] + 0.02,
        details=f"closed={hasp_closed}, open={hasp_open}",
    )

    # Dark reinforced travel-box structure: body bands wrap all four sides.
    ctx.check(
        "metal strap bands wrap the body",
        all(
            body.get_visual(name) is not None
            for name in (
                "upper_front_band_0",
                "upper_front_band_1",
                "upper_rear_band",
                "upper_side_band_0",
                "upper_side_band_1",
                "lower_front_band",
                "lower_rear_band",
                "lower_side_band_0",
                "lower_side_band_1",
            )
        ),
        details="expected upper and lower strap bands on front, rear, and both sides",
    )

    # The variant has two side iron ring handles, and at least one ring rotates.
    ctx.check(
        "two side ring handles present",
        ring_0.get_visual("ring") is not None and ring_1.get_visual("ring") is not None,
        details="expected ring visual on both side handle parts",
    )
    with ctx.pose({ring_pivot_0: 0.0}):
        ring_rest = ctx.part_world_aabb(ring_0)
    with ctx.pose({ring_pivot_0: 1.0}):
        ring_raised = ctx.part_world_aabb(ring_0)
    ctx.check(
        "side ring handle rotates upward",
        ring_rest is not None and ring_raised is not None
        and ring_raised[0][2] > ring_rest[0][2] + 0.025,
        details=f"rest={ring_rest}, raised={ring_raised}",
    )
    with ctx.pose({ring_pivot_0: 0.0}):
        ctx.expect_overlap(ring_0, body, axes="yz", min_overlap=0.030,
                           name="side ring aligns with side handle mount")

    ctx.allow_overlap(
        body,
        ring_0,
        elem_a="side_0_handle_pin",
        elem_b="ring",
        reason="The ring is intentionally captured on the hinge pin through its top eye.",
    )
    ctx.allow_overlap(
        body,
        ring_0,
        elem_a="side_0_handle_pin",
        elem_b="top_saddle",
        reason="The saddle block is intentionally bored by the simplified hinge pin.",
    )
    ctx.allow_overlap(
        body,
        ring_0,
        elem_a="side_0_clevis_0",
        elem_b="ring",
        reason="The rotating ring passes through the lower clevis cheek at the handle pivot.",
    )
    ctx.allow_overlap(
        body,
        ring_0,
        elem_a="side_0_clevis_1",
        elem_b="ring",
        reason="The rotating ring passes through the upper clevis cheek at the handle pivot.",
    )
    ctx.allow_overlap(
        body,
        ring_1,
        elem_a="side_1_handle_pin",
        elem_b="ring",
        reason="The ring is intentionally captured on the hinge pin through its top eye.",
    )
    ctx.allow_overlap(
        body,
        ring_1,
        elem_a="side_1_handle_pin",
        elem_b="top_saddle",
        reason="The saddle block is intentionally bored by the simplified hinge pin.",
    )
    ctx.allow_overlap(
        body,
        ring_1,
        elem_a="side_1_clevis_0",
        elem_b="ring",
        reason="The rotating ring passes through the lower clevis cheek at the handle pivot.",
    )
    ctx.allow_overlap(
        body,
        ring_1,
        elem_a="side_1_clevis_1",
        elem_b="ring",
        reason="The rotating ring passes through the upper clevis cheek at the handle pivot.",
    )

    return ctx.report()
