from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

# --- Dimensions (meters) -----------------------------------------------------
W = 0.50          # width (X)
D = 0.34          # depth (Y)
HB = 0.24         # body height (Z)
T = 0.018         # plank wall thickness
LID_T = 0.040     # flat lid thickness
SEAM_GAP = 0.0    # lid rests directly on the body rim


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ribbed_plank_storage_box")

    wood = model.material("wood_plank", rgba=(0.62, 0.45, 0.26, 1.0))
    wood_dark = model.material("wood_dark", rgba=(0.45, 0.32, 0.18, 1.0))
    iron = model.material("iron_fitting", rgba=(0.22, 0.22, 0.24, 1.0))
    rope = model.material("rope", rgba=(0.55, 0.45, 0.28, 1.0))

    # --- Body (root): a ribbed-plank chest with reinforced rails -------------
    body = model.part("box_body")
    body.visual(Box((W, D, T)), origin=Origin(xyz=(0.0, 0.0, T / 2.0)),
                material=wood_dark, name="floor_panel")
    cz = HB / 2.0

    # Thin shadow backing panels keep the box enclosed while the exterior reads
    # as individual vertical planks separated by dark seams.
    liner_t = 0.006
    body.visual(Box((W - 2 * T, liner_t, HB - 0.012)),
                origin=Origin(xyz=(0.0, -(D / 2.0 - liner_t / 2.0), cz)),
                material=wood_dark, name="front_shadow_liner")
    body.visual(Box((W - 2 * T, liner_t, HB - 0.012)),
                origin=Origin(xyz=(0.0, (D / 2.0 - liner_t / 2.0), cz)),
                material=wood_dark, name="rear_shadow_liner")
    body.visual(Box((liner_t, D - 2 * T, HB - 0.012)),
                origin=Origin(xyz=(-(W / 2.0 - liner_t / 2.0), 0.0, cz)),
                material=wood_dark, name="side_shadow_liner_0")
    body.visual(Box((liner_t, D - 2 * T, HB - 0.012)),
                origin=Origin(xyz=((W / 2.0 - liner_t / 2.0), 0.0, cz)),
                material=wood_dark, name="side_shadow_liner_1")

    # Vertical exterior slats/ribs.  Each slat reaches into the top and bottom
    # rails so the wall reads as one supported wooden assembly, not loose trim.
    front_slat_w = 0.052
    for i, x in enumerate((-0.208, -0.156, -0.104, -0.052, 0.0, 0.052, 0.104, 0.156, 0.208)):
        mat = wood if i % 2 == 0 else wood_dark
        body.visual(Box((front_slat_w - 0.006, T, HB)),
                    origin=Origin(xyz=(x, -(D / 2.0 - T / 2.0), cz)),
                    material=mat, name=f"front_slat_{i}")
        body.visual(Box((front_slat_w - 0.006, T, HB)),
                    origin=Origin(xyz=(x, (D / 2.0 - T / 2.0), cz)),
                    material=mat, name=f"rear_slat_{i}")
    body.visual(Box((0.014, T + 0.010, HB)),
                origin=Origin(xyz=(0.0, -(D / 2.0 - (T + 0.010) / 2.0 - 0.004), cz)),
                material=wood, name="front_center_rib")

    side_slat_w = 0.052
    for i, y in enumerate((-0.104, -0.052, 0.0, 0.052, 0.104)):
        mat = wood if i % 2 == 0 else wood_dark
        body.visual(Box((T, side_slat_w - 0.006, HB)),
                    origin=Origin(xyz=(-(W / 2.0 - T / 2.0), y, cz)),
                    material=mat, name=f"side_slat_0_{i}")
        body.visual(Box((T, side_slat_w - 0.006, HB)),
                    origin=Origin(xyz=((W / 2.0 - T / 2.0), y, cz)),
                    material=mat, name=f"side_slat_1_{i}")

    # Non-decorative reinforced top and bottom rails bind the vertical slats.
    rail_h = 0.036
    rail_out = T + 0.010
    z = rail_h / 2.0
    body.visual(Box((W + 0.014, rail_out, rail_h)),
                origin=Origin(xyz=(0.0, -(D / 2.0 - rail_out / 2.0 - 0.002), z)),
                material=wood_dark, name="front_bottom_rail")
    body.visual(Box((W + 0.014, rail_out, rail_h)),
                origin=Origin(xyz=(0.0, (D / 2.0 - rail_out / 2.0 - 0.002), z)),
                material=wood_dark, name="rear_bottom_rail")
    body.visual(Box((rail_out, D + 0.014, rail_h)),
                origin=Origin(xyz=(-(W / 2.0 - rail_out / 2.0 - 0.002), 0.0, z)),
                material=wood_dark, name="side_bottom_rail_0")
    body.visual(Box((rail_out, D + 0.014, rail_h)),
                origin=Origin(xyz=((W / 2.0 - rail_out / 2.0 - 0.002), 0.0, z)),
                material=wood_dark, name="side_bottom_rail_1")
    z = HB - rail_h / 2.0
    body.visual(Box((W + 0.014, rail_out, rail_h)),
                origin=Origin(xyz=(0.0, -(D / 2.0 - rail_out / 2.0 - 0.002), z)),
                material=wood_dark, name="front_top_rail")
    body.visual(Box((W + 0.014, rail_out, rail_h)),
                origin=Origin(xyz=(0.0, (D / 2.0 - rail_out / 2.0 - 0.002), z)),
                material=wood_dark, name="rear_top_rail")
    body.visual(Box((rail_out, D + 0.014, rail_h)),
                origin=Origin(xyz=(-(W / 2.0 - rail_out / 2.0 - 0.002), 0.0, z)),
                material=wood_dark, name="side_top_rail_0")
    body.visual(Box((rail_out, D + 0.014, rail_h)),
                origin=Origin(xyz=((W / 2.0 - rail_out / 2.0 - 0.002), 0.0, z)),
                material=wood_dark, name="side_top_rail_1")

    # iron corner brackets at the four vertical edges
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

    # rope side handles: a hanging rope held by two iron staples on each end face
    for sx, tag in ((-1.0, "left"), (1.0, "right")):
        xface = sx * (W / 2.0)
        for sy in (-0.08, 0.08):
            body.visual(Box((0.032, 0.024, 0.03)),
                        origin=Origin(xyz=(xface + sx * 0.013, sy, HB * 0.62)),
                        material=iron, name=f"staple_{tag}_{'f' if sy < 0 else 'b'}")
        # rope grab handle spanning between the two staples (ends seat in staples)
        body.visual(
            Cylinder(radius=0.010, length=0.20),
            origin=Origin(xyz=(xface + sx * 0.022, 0.0, HB * 0.62),
                          rpy=(1.5707963, 0.0, 0.0)),
            material=rope, name=f"rope_{tag}",
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
    # iron straps across the lid
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
    lid_hinge = object_model.get_articulation("lid_hinge")
    hasp_hinge = object_model.get_articulation("hasp_hinge")

    # Closed lid seats on the body rim without penetrating it.
    with ctx.pose({lid_hinge: 0.0, hasp_hinge: 0.0}):
        ctx.expect_gap(lid, body, axis="z", min_gap=0.0, max_gap=0.006,
                       positive_elem="lid_panel", negative_elem="front_top_rail",
                       name="lid seats on the reinforced top rail when closed")
        ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.25,
                           name="closed lid covers the body opening")
        ctx.expect_overlap(body, body, axes="z", min_overlap=0.025,
                           elem_a="front_center_rib", elem_b="front_top_rail",
                           name="center front slat is captured by the top rail")
        ctx.expect_overlap(body, body, axes="z", min_overlap=0.025,
                           elem_a="front_center_rib", elem_b="front_bottom_rail",
                           name="center front slat is captured by the bottom rail")
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

    # Rope side handles exist on both ends.
    ctx.check(
        "rope side handles present on both ends",
        body.get_visual("rope_left") is not None and body.get_visual("rope_right") is not None,
        details="expected rope_left and rope_right",
    )

    ctx.check(
        "ribbed plank walls and rails are present",
        all(body.get_visual(name) is not None for name in (
            "front_slat_0", "front_slat_4", "front_slat_8",
            "rear_slat_4", "side_slat_0_2", "side_slat_1_2",
            "front_top_rail", "front_bottom_rail", "rear_top_rail",
            "side_top_rail_0", "side_bottom_rail_1",
        )),
        details="expected vertical slats on all sides plus reinforced top/bottom rails",
    )

    return ctx.report()
