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
# Small workbench jewelry-box / keepsake-chest proportions.
W = 0.30          # width (X)
D = 0.20          # depth (Y)
HB = 0.115        # body height (Z)
T = 0.010         # wall thickness
LID_T = 0.024     # flat lid thickness
SEAM_GAP = 0.0    # lid rests directly on the body rim
FOOT_H = 0.018
FOOT_W = 0.038
TRAY_LIP_H = 0.014
TRAY_LIP_T = 0.009


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="small_jewelry_storage_box")

    wood = model.material("wood_plank", rgba=(0.62, 0.45, 0.26, 1.0))
    wood_dark = model.material("wood_dark", rgba=(0.45, 0.32, 0.18, 1.0))
    iron = model.material("iron_fitting", rgba=(0.22, 0.22, 0.24, 1.0))
    rope = model.material("rope", rgba=(0.55, 0.45, 0.28, 1.0))

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

    # slightly raised block feet make this a small jewelry-box variant while
    # still keeping a rectangular chest body.
    foot_z = -FOOT_H / 2.0
    foot_idx = 0
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            body.visual(
                Box((FOOT_W, FOOT_W, FOOT_H)),
                origin=Origin(xyz=(sx * (W / 2.0 - FOOT_W * 0.75),
                                   sy * (D / 2.0 - FOOT_W * 0.75), foot_z)),
                material=wood_dark,
                name=f"raised_foot_{foot_idx}",
            )
            foot_idx += 1

    # A shallow interior tray lip / ledge just below the lid; it is visible
    # inside the open box and gives the small chest a jewelry-box construction.
    lip_z = HB - TRAY_LIP_H / 2.0 - 0.010
    body.visual(Box((W - 2 * T, TRAY_LIP_T, TRAY_LIP_H)),
                origin=Origin(xyz=(0.0, -(D / 2.0 - T - TRAY_LIP_T / 2.0), lip_z)),
                material=wood_dark, name="front_tray_lip")
    body.visual(Box((W - 2 * T, TRAY_LIP_T, TRAY_LIP_H)),
                origin=Origin(xyz=(0.0, (D / 2.0 - T - TRAY_LIP_T / 2.0), lip_z)),
                material=wood_dark, name="back_tray_lip")
    body.visual(Box((TRAY_LIP_T, D - 2 * T, TRAY_LIP_H)),
                origin=Origin(xyz=(-(W / 2.0 - T - TRAY_LIP_T / 2.0), 0.0, lip_z)),
                material=wood_dark, name="side_tray_lip_0")
    body.visual(Box((TRAY_LIP_T, D - 2 * T, TRAY_LIP_H)),
                origin=Origin(xyz=((W / 2.0 - T - TRAY_LIP_T / 2.0), 0.0, lip_z)),
                material=wood_dark, name="side_tray_lip_1")

    # small iron corner brackets at the four vertical edges
    bk = 0.020
    idx = 0
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            body.visual(
                Box((bk, bk, HB - 0.014)),
                origin=Origin(xyz=(sx * (W / 2.0 - bk / 2.0 + 0.003),
                                   sy * (D / 2.0 - bk / 2.0 + 0.003), HB / 2.0)),
                material=iron, name=f"corner_bracket_{idx}",
            )
            idx += 1

    # vertical iron battens on the long faces
    for sx in (-0.075, 0.075):
        body.visual(Box((0.018, D + 0.004, 0.005)),
                    origin=Origin(xyz=(sx, 0.0, HB * 0.5)),
                    material=iron, name=f"batten_{'l' if sx < 0 else 'r'}")

    # compact rope side handles: small pull loops held by two iron staples on each end face
    for sx, tag in ((-1.0, "left"), (1.0, "right")):
        xface = sx * (W / 2.0)
        for sy in (-0.045, 0.045):
            body.visual(Box((0.020, 0.016, 0.018)),
                        origin=Origin(xyz=(xface + sx * 0.009, sy, HB * 0.58)),
                        material=iron, name=f"staple_{tag}_{'f' if sy < 0 else 'b'}")
        # rope grab handle spanning between the two staples (ends seat in staples)
        body.visual(
            Cylinder(radius=0.006, length=0.12),
            origin=Origin(xyz=(xface + sx * 0.016, 0.0, HB * 0.58),
                          rpy=(1.5707963, 0.0, 0.0)),
            material=rope, name=f"rope_{tag}",
        )

    # front staple that receives the hasp
    body.visual(Box((0.028, 0.009, 0.020)),
                origin=Origin(xyz=(0.0, -(D / 2.0) - 0.0045, HB * 0.45)),
                material=iron, name="hasp_keeper")

    # --- Lid (flat, hinged at the back top edge) -----------------------------
    lid = model.part("box_lid")
    lid.visual(Box((W + 0.014, D + 0.014, LID_T)),
               origin=Origin(xyz=(0.0, -D / 2.0, SEAM_GAP + LID_T / 2.0)),
               material=wood, name="lid_panel")
    # iron straps across the lid
    for sx in (-0.075, 0.075):
        lid.visual(Box((0.018, D + 0.014, 0.005)),
                   origin=Origin(xyz=(sx, -D / 2.0, SEAM_GAP + LID_T + 0.0025)),
                   material=iron, name=f"lid_strap_{'l' if sx < 0 else 'r'}")
    # hinge mount tab that carries the front hasp (reaches out to contact it)
    lid.visual(Box((0.034, 0.014, 0.012)),
               origin=Origin(xyz=(0.0, -(D + 0.002), SEAM_GAP + LID_T * 0.5)),
               material=iron, name="hasp_mount")

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, D / 2.0, HB)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=2.0),
    )

    # --- Front hasp (hinged on the lid front edge, swings to latch) ----------
    hasp = model.part("hasp")
    hasp.visual(Box((0.034, 0.008, 0.060)),
                origin=Origin(xyz=(0.0, 0.0, -0.030)),
                material=iron, name="hasp_arm")
    hasp.visual(Box((0.018, 0.011, 0.014)),
                origin=Origin(xyz=(0.0, -0.003, -0.054)),
                material=iron, name="hasp_eye")
    # hinge at the lid front edge, proud of the body front face; q=0 is closed
    # (arm hangs down over the front), positive q lifts it to release.
    model.articulation(
        "hasp_hinge",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=hasp,
        origin=Origin(xyz=(0.0, -(D + 0.009), SEAM_GAP + LID_T * 0.5)),
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
                       positive_elem="lid_panel", negative_elem="wall_front",
                       name="lid seats on the body rim when closed")
        ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.15,
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

    # Variant-specific geometry: raised feet and the shallow tray lip/ledge are
    # structural small-box features, not just color changes.
    ctx.check(
        "small jewelry box scale",
        W <= 0.32 and D <= 0.22 and HB <= 0.13,
        details=f"W={W}, D={D}, HB={HB}",
    )
    ctx.expect_gap(body, body, axis="z", min_gap=0.0, max_gap=0.001, positive_elem="floor_panel",
                   negative_elem="raised_foot_0",
                   name="raised feet contact underside of floor")
    ctx.check(
        "feet project below box body",
        FOOT_H >= 0.015,
        details=f"FOOT_H={FOOT_H}",
    )
    ctx.expect_overlap(body, body, axes="x", min_overlap=0.14,
                       elem_a="front_tray_lip", elem_b="back_tray_lip",
                       name="opposing tray lips span the interior width")
    ctx.expect_gap(lid, body, axis="z", min_gap=0.006, max_gap=0.03,
                   positive_elem="lid_panel", negative_elem="front_tray_lip",
                   name="tray lip sits visibly below closed lid")

    # Rope side handles exist on both ends.
    ctx.check(
        "rope side handles present on both ends",
        body.get_visual("rope_left") is not None and body.get_visual("rope_right") is not None,
        details="expected rope_left and rope_right",
    )

    return ctx.report()
