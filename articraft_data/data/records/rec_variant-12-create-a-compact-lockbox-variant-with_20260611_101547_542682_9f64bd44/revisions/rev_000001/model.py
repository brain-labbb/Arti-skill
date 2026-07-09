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
W = 0.32          # compact lockbox width (X)
D = 0.22          # compact lockbox depth (Y)
HB = 0.14         # body height (Z)
T = 0.012         # sheet-metal wall thickness
LID_T = 0.025     # flat reinforced lid thickness
SEAM_GAP = 0.0    # lid rests directly on the body rim


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_lockbox")

    body_paint = model.material("painted_steel", rgba=(0.13, 0.20, 0.18, 1.0))
    inner_dark = model.material("dark_interior", rgba=(0.05, 0.06, 0.06, 1.0))
    iron = model.material("brushed_steel", rgba=(0.35, 0.35, 0.36, 1.0))
    rubber = model.material("rubber", rgba=(0.025, 0.025, 0.025, 1.0))

    # --- Body (root) ---------------------------------------------------------
    body = model.part("box_body")
    body.visual(Box((W, D, T)), origin=Origin(xyz=(0.0, 0.0, T / 2.0)),
                material=inner_dark, name="floor_panel")
    cz = HB / 2.0
    body.visual(Box((W, T, HB)), origin=Origin(xyz=(0.0, -(D / 2.0 - T / 2.0), cz)),
                material=body_paint, name="wall_front")
    body.visual(Box((W, T, HB)), origin=Origin(xyz=(0.0, (D / 2.0 - T / 2.0), cz)),
                material=body_paint, name="wall_back")
    body.visual(Box((T, D, HB)), origin=Origin(xyz=(-(W / 2.0 - T / 2.0), 0.0, cz)),
                material=body_paint, name="wall_left")
    body.visual(Box((T, D, HB)), origin=Origin(xyz=((W / 2.0 - T / 2.0), 0.0, cz)),
                material=body_paint, name="wall_right")

    # compact lockbox corner caps at the four vertical edges
    bk = 0.022
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

    # continuous reinforced top rim and low rubber feet: structural lockbox changes
    body.visual(Box((W, 0.014, 0.018)),
                origin=Origin(xyz=(0.0, -(D / 2.0 - 0.007), HB - 0.009)),
                material=iron, name="front_rim")
    body.visual(Box((W, 0.014, 0.018)),
                origin=Origin(xyz=(0.0, (D / 2.0 - 0.007), HB - 0.009)),
                material=iron, name="rear_rim")
    body.visual(Box((0.014, D, 0.018)),
                origin=Origin(xyz=(-(W / 2.0 - 0.007), 0.0, HB - 0.009)),
                material=iron, name="side_rim_0")
    body.visual(Box((0.014, D, 0.018)),
                origin=Origin(xyz=((W / 2.0 - 0.007), 0.0, HB - 0.009)),
                material=iron, name="side_rim_1")
    for sx in (-0.11, 0.11):
        for sy in (-0.07, 0.07):
            body.visual(Box((0.045, 0.030, 0.010)),
                        origin=Origin(xyz=(sx, sy, -0.005)),
                        material=rubber, name=f"bench_foot_{0 if sx < 0 else 1}_{0 if sy < 0 else 1}")

    # front keeper loop that the rotating hasp plate covers
    body.visual(Box((0.090, 0.010, 0.038)),
                origin=Origin(xyz=(0.0, -(D / 2.0) - 0.005, 0.090)),
                material=iron, name="keeper_base")
    body.visual(Box((0.010, 0.026, 0.036)),
                origin=Origin(xyz=(-0.038, -(D / 2.0) - 0.023, 0.090)),
                material=iron, name="keeper_lug_0")
    body.visual(Box((0.010, 0.026, 0.036)),
                origin=Origin(xyz=(0.038, -(D / 2.0) - 0.023, 0.090)),
                material=iron, name="keeper_lug_1")
    body.visual(Box((0.086, 0.010, 0.010)),
                origin=Origin(xyz=(0.0, -(D / 2.0) - 0.034, 0.086)),
                material=iron, name="keeper_bar")

    # --- Lid (flat, hinged at the back top edge) -----------------------------
    lid = model.part("box_lid")
    lid.visual(Box((W + 0.02, D + 0.02, LID_T)),
               origin=Origin(xyz=(0.0, -D / 2.0, SEAM_GAP + LID_T / 2.0)),
               material=body_paint, name="lid_panel")
    lid.visual(Box((W - 0.03, D - 0.03, 0.008)),
               origin=Origin(xyz=(0.0, -D / 2.0, SEAM_GAP + LID_T + 0.004)),
               material=iron, name="lid_reinforcement")
    # front pivot boss for the rotating hasp plate
    lid.visual(Box((0.070, 0.018, 0.026)),
               origin=Origin(xyz=(0.0, -(D + 0.004), SEAM_GAP + LID_T * 0.5)),
               material=iron, name="hasp_pivot_mount")

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, D / 2.0, HB)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=2.0, lower=0.0, upper=2.1),
    )

    # --- Front hasp plate (rotates in the front face plane to latch) ----------
    hasp = model.part("hasp")
    hasp.visual(Box((0.060, 0.010, 0.082)),
                origin=Origin(xyz=(0.0, -0.002, -0.041)),
                material=iron, name="hasp_plate")
    hasp.visual(Cylinder(radius=0.016, length=0.014),
                origin=Origin(xyz=(0.0, -0.003, 0.0), rpy=(1.5707963, 0.0, 0.0)),
                material=iron, name="hasp_pivot_disk")
    hasp.visual(Box((0.034, 0.014, 0.018)),
                origin=Origin(xyz=(0.0, -0.007, -0.067)),
                material=iron, name="hasp_catch")
    # pivot is proud of the body front face; q=0 hangs over keeper, positive q rotates aside.
    model.articulation(
        "hasp_hinge",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=hasp,
        origin=Origin(xyz=(0.0, -(D + 0.014), SEAM_GAP + LID_T * 0.5)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=3.0, lower=0.0, upper=1.55),
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

    def has_visual(part, visual_name: str) -> bool:
        try:
            part.get_visual(visual_name)
            return True
        except Exception:
            return False

    # Closed lid seats on the body rim without penetrating it.
    with ctx.pose({lid_hinge: 0.0, hasp_hinge: 0.0}):
        ctx.expect_gap(lid, body, axis="z", min_gap=0.0, max_gap=0.006,
                       positive_elem="lid_panel", negative_elem="wall_front",
                       name="lid seats on the body rim when closed")
        ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.18,
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

    # Rotating front hasp plate covers the keeper, then swings sideways to release.
    with ctx.pose({lid_hinge: 0.0, hasp_hinge: 0.0}):
        ctx.expect_overlap(hasp, body, axes="xz", min_overlap=0.009,
                           elem_a="hasp_catch", elem_b="keeper_bar",
                           name="closed rotating hasp covers the keeper bar")
        hasp_closed = ctx.part_world_aabb(hasp)
    with ctx.pose({lid_hinge: 0.0, hasp_hinge: 1.45}):
        hasp_open = ctx.part_world_aabb(hasp)
    ctx.check(
        "front hasp plate rotates aside to release",
        hasp_closed is not None and hasp_open is not None
        and hasp_open[0][0] < hasp_closed[0][0] - 0.04,
        details=f"closed={hasp_closed}, open={hasp_open}",
    )

    # Compact workbench lockbox geometry: rectangular body, no side carry ropes, low feet.
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "compact rectangular lockbox body",
        body_aabb is not None
        and (body_aabb[1][0] - body_aabb[0][0]) < 0.37
        and (body_aabb[1][1] - body_aabb[0][1]) < 0.27
        and (body_aabb[1][2] - body_aabb[0][2]) < 0.18,
        details=f"body_aabb={body_aabb}",
    )
    ctx.check(
        "workbench feet replace side ropes",
        has_visual(body, "bench_foot_0_0")
        and has_visual(body, "bench_foot_1_1")
        and not has_visual(body, "rope_left")
        and not has_visual(body, "rope_right"),
        details="expected low feet and no rope side handles on compact lockbox",
    )

    return ctx.report()
