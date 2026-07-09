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
HB = 0.42         # deeper body height (Z)
T = 0.018         # plank wall thickness
LID_T = 0.040     # flat lid thickness
SEAM_GAP = 0.0    # lid rests directly on the body rim
TRAY_W = 0.39
TRAY_D = 0.24
TRAY_T = 0.012
TRAY_H = 0.070
TRAY_REST_Z = 0.255
TRAY_LIFT = 0.140


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="double_depth_rustic_wooden_box")

    wood = model.material("wood_plank", rgba=(0.62, 0.45, 0.26, 1.0))
    wood_dark = model.material("wood_dark", rgba=(0.45, 0.32, 0.18, 1.0))
    tray_wood = model.material("tray_wood", rgba=(0.70, 0.52, 0.30, 1.0))
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

    # Internal cleats make the box visibly double-depth and guide/support the
    # removable upper tray without changing the workbench storage-box category.
    rail_z = TRAY_REST_Z - 0.009
    for sx, tag in ((-1.0, "left"), (1.0, "right")):
        body.visual(
            Box((0.026, D - 2.0 * T, 0.018)),
            origin=Origin(xyz=(sx * (W / 2.0 - T - 0.013), 0.0, rail_z)),
            material=wood_dark,
            name=f"inner_cleat_{tag}",
        )
    for sy, tag in ((-1.0, "front"), (1.0, "back")):
        body.visual(
            Box((W - 2.0 * T, 0.022, 0.018)),
            origin=Origin(xyz=(0.0, sy * (D / 2.0 - T - 0.011), rail_z)),
            material=wood_dark,
            name=f"inner_cleat_{tag}",
        )

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

    # --- Lift-out tray (slides vertically from the deep box) -----------------
    tray = model.part("lift_tray")
    tray.visual(Box((TRAY_W, TRAY_D, TRAY_T)),
                origin=Origin(xyz=(0.0, 0.0, TRAY_T / 2.0)),
                material=tray_wood, name="tray_floor")
    tray.visual(Box((TRAY_W, 0.014, TRAY_H)),
                origin=Origin(xyz=(0.0, -(TRAY_D / 2.0 - 0.007), TRAY_T + TRAY_H / 2.0)),
                material=tray_wood, name="tray_front")
    tray.visual(Box((TRAY_W, 0.014, TRAY_H)),
                origin=Origin(xyz=(0.0, (TRAY_D / 2.0 - 0.007), TRAY_T + TRAY_H / 2.0)),
                material=tray_wood, name="tray_back")
    tray.visual(Box((0.014, TRAY_D, TRAY_H)),
                origin=Origin(xyz=(-(TRAY_W / 2.0 - 0.007), 0.0, TRAY_T + TRAY_H / 2.0)),
                material=tray_wood, name="tray_left")
    tray.visual(Box((0.014, TRAY_D, TRAY_H)),
                origin=Origin(xyz=((TRAY_W / 2.0 - 0.007), 0.0, TRAY_T + TRAY_H / 2.0)),
                material=tray_wood, name="tray_right")
    # proud rim strips keep the tray recognizable as a shallow lift-out insert.
    tray.visual(Box((TRAY_W + 0.018, 0.018, 0.012)),
                origin=Origin(xyz=(0.0, -TRAY_D / 2.0, TRAY_T + TRAY_H + 0.006)),
                material=tray_wood, name="tray_front_rim")
    tray.visual(Box((TRAY_W + 0.018, 0.018, 0.012)),
                origin=Origin(xyz=(0.0, TRAY_D / 2.0, TRAY_T + TRAY_H + 0.006)),
                material=tray_wood, name="tray_back_rim")
    tray.visual(Box((0.018, TRAY_D + 0.018, 0.012)),
                origin=Origin(xyz=(-TRAY_W / 2.0, 0.0, TRAY_T + TRAY_H + 0.006)),
                material=tray_wood, name="tray_left_rim")
    tray.visual(Box((0.018, TRAY_D + 0.018, 0.012)),
                origin=Origin(xyz=(TRAY_W / 2.0, 0.0, TRAY_T + TRAY_H + 0.006)),
                material=tray_wood, name="tray_right_rim")
    # Bottom runners sit directly on the fixed inner cleats at q=0, providing
    # a visible support path for the lift-out tray.
    for sx, tag in ((-1.0, "left"), (1.0, "right")):
        tray.visual(Box((0.020, TRAY_D, TRAY_T)),
                    origin=Origin(xyz=(sx * (TRAY_W / 2.0 + 0.010), 0.0, TRAY_T / 2.0)),
                    material=tray_wood, name=f"tray_runner_{tag}")

    model.articulation(
        "tray_lift",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tray,
        origin=Origin(xyz=(0.0, 0.0, TRAY_REST_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.6, lower=0.0, upper=TRAY_LIFT),
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
    tray = object_model.get_part("lift_tray")
    hasp = object_model.get_part("hasp")
    lid_hinge = object_model.get_articulation("lid_hinge")
    tray_lift = object_model.get_articulation("tray_lift")
    hasp_hinge = object_model.get_articulation("hasp_hinge")

    # Closed lid seats on the body rim without penetrating it.
    with ctx.pose({lid_hinge: 0.0, hasp_hinge: 0.0}):
        ctx.expect_gap(lid, body, axis="z", min_gap=0.0, max_gap=0.006,
                       positive_elem="lid_panel", negative_elem="wall_front",
                       name="lid seats on the body rim when closed")
        ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.25,
                           name="closed lid covers the body opening")
        ctx.expect_gap(lid, tray, axis="z", min_gap=0.04,
                       positive_elem="lid_panel", negative_elem="tray_front_rim",
                       name="closed lid clears the nested lift tray")
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

    # The new insert is a real vertically moving lift-out tray, not just color.
    with ctx.pose({tray_lift: 0.0}):
        ctx.expect_within(tray, body, axes="xy", margin=0.002,
                          name="tray nests inside the deep box footprint")
        tray_lower = ctx.part_world_aabb(tray)
    with ctx.pose({lid_hinge: 1.9, tray_lift: TRAY_LIFT}):
        ctx.expect_within(tray, body, axes="xy", margin=0.002,
                          name="raised tray stays guided by the box opening")
        tray_raised = ctx.part_world_aabb(tray)
        body_open = ctx.part_world_aabb(body)
    ctx.check(
        "prismatic tray lifts vertically above the rim",
        tray_lower is not None and tray_raised is not None and body_open is not None
        and tray_raised[0][2] > tray_lower[0][2] + 0.12
        and tray_raised[1][2] > body_open[1][2] + 0.02,
        details=f"lower={tray_lower}, raised={tray_raised}, body={body_open}",
    )

    ctx.check(
        "deep box has internal support cleats",
        body.get_visual("inner_cleat_left") is not None
        and body.get_visual("inner_cleat_right") is not None
        and body.get_visual("inner_cleat_front") is not None
        and body.get_visual("inner_cleat_back") is not None,
        details="expected four inner cleats supporting the lift-out tray",
    )

    # Rope side handles exist on both ends.
    ctx.check(
        "rope side handles present on both ends",
        body.get_visual("rope_left") is not None and body.get_visual("rope_right") is not None,
        details="expected rope_left and rope_right",
    )

    return ctx.report()
