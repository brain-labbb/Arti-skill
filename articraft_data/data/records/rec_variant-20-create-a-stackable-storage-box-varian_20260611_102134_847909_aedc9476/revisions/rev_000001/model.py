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
FOOT_W = 0.055     # small stackable nesting foot size
FOOT_H = 0.032
RECESS_T = 0.007   # underside outline thickness showing the bottom recess


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rustic_wooden_box")

    wood = model.material("wood_plank", rgba=(0.62, 0.45, 0.26, 1.0))
    wood_dark = model.material("wood_dark", rgba=(0.45, 0.32, 0.18, 1.0))
    iron = model.material("iron_fitting", rgba=(0.22, 0.22, 0.24, 1.0))
    rope = model.material("rope", rgba=(0.55, 0.45, 0.28, 1.0))

    # --- Body (root) ---------------------------------------------------------
    body = model.part("box_body")
    body.visual(Box((W, D, T)), origin=Origin(xyz=(0.0, 0.0, T / 2.0)),
                material=wood_dark, name="floor_panel")
    # Stackable variant: four proud feet project below the box, while a smaller
    # rectangular underside outline marks the recessed bottom that accepts feet
    # from another box in a stack.
    foot_idx = 0
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            body.visual(
                Box((FOOT_W, FOOT_W * 0.88, FOOT_H)),
                origin=Origin(
                    xyz=(
                        sx * (W / 2.0 - 0.070),
                        sy * (D / 2.0 - 0.055),
                        -FOOT_H / 2.0,
                    )
                ),
                material=wood_dark,
                name=f"nesting_foot_{foot_idx}",
            )
            foot_idx += 1
    body.visual(Box((W - 0.26, 0.012, RECESS_T)),
                origin=Origin(xyz=(0.0, -(D / 2.0 - 0.090), -RECESS_T / 2.0)),
                material=iron, name="bottom_recess_front")
    body.visual(Box((W - 0.26, 0.012, RECESS_T)),
                origin=Origin(xyz=(0.0, (D / 2.0 - 0.090), -RECESS_T / 2.0)),
                material=iron, name="bottom_recess_back")
    body.visual(Box((0.012, D - 0.20, RECESS_T)),
                origin=Origin(xyz=(-(W / 2.0 - 0.130), 0.0, -RECESS_T / 2.0)),
                material=iron, name="bottom_recess_left")
    body.visual(Box((0.012, D - 0.20, RECESS_T)),
                origin=Origin(xyz=((W / 2.0 - 0.130), 0.0, -RECESS_T / 2.0)),
                material=iron, name="bottom_recess_right")
    cz = HB / 2.0
    body.visual(Box((W, T, HB)), origin=Origin(xyz=(0.0, -(D / 2.0 - T / 2.0), cz)),
                material=wood, name="wall_front")
    body.visual(Box((W, T, HB)), origin=Origin(xyz=(0.0, (D / 2.0 - T / 2.0), cz)),
                material=wood, name="wall_back")
    body.visual(Box((T, D, HB)), origin=Origin(xyz=(-(W / 2.0 - T / 2.0), 0.0, cz)),
                material=wood, name="wall_left")
    body.visual(Box((T, D, HB)), origin=Origin(xyz=((W / 2.0 - T / 2.0), 0.0, cz)),
                material=wood, name="wall_right")

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

    # vertical iron battens on the long faces
    for sx in (-0.13, 0.13):
        body.visual(Box((0.028, D + 0.004, 0.008)),
                    origin=Origin(xyz=(sx, 0.0, HB * 0.5)),
                    material=iron, name=f"batten_{'l' if sx < 0 else 'r'}")

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

    # Rope side handles exist on both ends.
    ctx.check(
        "rope side handles present on both ends",
        body.get_visual("rope_left") is not None and body.get_visual("rope_right") is not None,
        details="expected rope_left and rope_right",
    )

    # Stackable variant features: four small feet and a visible recessed bottom outline.
    foot_names = [f"nesting_foot_{i}" for i in range(4)]
    recess_names = [
        "bottom_recess_front",
        "bottom_recess_back",
        "bottom_recess_left",
        "bottom_recess_right",
    ]
    ctx.check(
        "four nesting feet are present",
        all(body.get_visual(name) is not None for name in foot_names),
        details=f"expected {foot_names}",
    )
    ctx.check(
        "recessed bottom outline is present",
        all(body.get_visual(name) is not None for name in recess_names),
        details=f"expected {recess_names}",
    )

    floor_aabb = ctx.part_element_world_aabb(body, elem="floor_panel")
    foot_aabbs = [ctx.part_element_world_aabb(body, elem=name) for name in foot_names]
    ctx.check(
        "nesting feet project below the floor",
        floor_aabb is not None
        and all(aabb is not None and aabb[0][2] < floor_aabb[0][2] - 0.020 for aabb in foot_aabbs),
        details=f"floor={floor_aabb}, feet={foot_aabbs}",
    )

    recess_front = ctx.part_element_world_aabb(body, elem="bottom_recess_front")
    recess_back = ctx.part_element_world_aabb(body, elem="bottom_recess_back")
    recess_left = ctx.part_element_world_aabb(body, elem="bottom_recess_left")
    recess_right = ctx.part_element_world_aabb(body, elem="bottom_recess_right")
    ctx.check(
        "bottom recess outline is inset from the outer walls",
        floor_aabb is not None
        and all(aabb is not None for aabb in (recess_front, recess_back, recess_left, recess_right))
        and recess_left[0][0] > floor_aabb[0][0] + 0.10
        and recess_right[1][0] < floor_aabb[1][0] - 0.10
        and recess_front[0][1] > floor_aabb[0][1] + 0.07
        and recess_back[1][1] < floor_aabb[1][1] - 0.07,
        details=(
            f"floor={floor_aabb}, front={recess_front}, back={recess_back}, "
            f"left={recess_left}, right={recess_right}"
        ),
    )

    return ctx.report()
