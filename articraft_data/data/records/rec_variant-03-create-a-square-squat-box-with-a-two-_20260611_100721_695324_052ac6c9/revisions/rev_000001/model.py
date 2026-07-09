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
W = 0.42          # square footprint width (X)
D = 0.42          # square footprint depth (Y)
HB = 0.20         # squat body height (Z)
T = 0.018         # plank wall thickness
LID_T = 0.040     # flat lid thickness
SEAM_GAP = 0.0    # lid rests directly on the body rim
CENTER_GAP = 0.006  # visible split between the two top leaves


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

    # --- Two-piece split top lid --------------------------------------------
    # Each leaf is hinged on an outside rim and reaches to the center seam,
    # making a square, squat workbench storage box with butterfly-style leaves.
    leaf_depth = D / 2.0 - CENTER_GAP / 2.0

    front_leaf = model.part("front_lid_leaf")
    front_leaf.visual(
        Box((W + 0.018, leaf_depth, LID_T)),
        origin=Origin(xyz=(0.0, leaf_depth / 2.0, SEAM_GAP + LID_T / 2.0)),
        material=wood,
        name="leaf_panel",
    )
    front_leaf.visual(
        Box((W + 0.018, 0.014, 0.010)),
        origin=Origin(xyz=(0.0, leaf_depth - 0.007, SEAM_GAP + LID_T + 0.005)),
        material=iron,
        name="center_seam_band",
    )
    for sx in (-0.12, 0.12):
        front_leaf.visual(
            Box((0.026, leaf_depth - 0.018, 0.006)),
            origin=Origin(xyz=(sx, leaf_depth / 2.0, SEAM_GAP + LID_T + 0.003)),
            material=iron,
            name=f"lid_strap_{'l' if sx < 0 else 'r'}",
        )
    front_leaf.visual(
        Cylinder(radius=0.010, length=W + 0.03),
        origin=Origin(xyz=(0.0, 0.0, SEAM_GAP + LID_T * 0.55),
                      rpy=(0.0, 1.5707963, 0.0)),
        material=iron,
        name="hinge_barrel",
    )

    rear_leaf = model.part("rear_lid_leaf")
    rear_leaf.visual(
        Box((W + 0.018, leaf_depth, LID_T)),
        origin=Origin(xyz=(0.0, -leaf_depth / 2.0, SEAM_GAP + LID_T / 2.0)),
        material=wood,
        name="leaf_panel",
    )
    rear_leaf.visual(
        Box((W + 0.018, 0.014, 0.010)),
        origin=Origin(xyz=(0.0, -(leaf_depth - 0.007), SEAM_GAP + LID_T + 0.005)),
        material=iron,
        name="center_seam_band",
    )
    for sx in (-0.12, 0.12):
        rear_leaf.visual(
            Box((0.026, leaf_depth - 0.018, 0.006)),
            origin=Origin(xyz=(sx, -leaf_depth / 2.0, SEAM_GAP + LID_T + 0.003)),
            material=iron,
            name=f"lid_strap_{'l' if sx < 0 else 'r'}",
        )
    rear_leaf.visual(
        Cylinder(radius=0.010, length=W + 0.03),
        origin=Origin(xyz=(0.0, 0.0, SEAM_GAP + LID_T * 0.55),
                      rpy=(0.0, 1.5707963, 0.0)),
        material=iron,
        name="hinge_barrel",
    )

    model.articulation(
        "front_lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=front_leaf,
        origin=Origin(xyz=(0.0, -D / 2.0, HB)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=0.0, upper=1.9),
    )
    model.articulation(
        "rear_lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=rear_leaf,
        origin=Origin(xyz=(0.0, D / 2.0, HB)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=0.0, upper=1.9),
    )

    return model


object_model = build_object_model()


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("box_body")
    front_leaf = object_model.get_part("front_lid_leaf")
    rear_leaf = object_model.get_part("rear_lid_leaf")
    front_hinge = object_model.get_articulation("front_lid_hinge")
    rear_hinge = object_model.get_articulation("rear_lid_hinge")

    # Closed split lid leaves seat on opposite rims and meet at a narrow seam.
    with ctx.pose({front_hinge: 0.0, rear_hinge: 0.0}):
        ctx.expect_gap(
            front_leaf, body, axis="z", min_gap=0.0, max_gap=0.006,
            positive_elem="leaf_panel", negative_elem="wall_front",
            name="front leaf seats on the front rim",
        )
        ctx.expect_gap(
            rear_leaf, body, axis="z", min_gap=0.0, max_gap=0.006,
            positive_elem="leaf_panel", negative_elem="wall_back",
            name="rear leaf seats on the rear rim",
        )
        ctx.expect_overlap(
            front_leaf, body, axes="xy", min_overlap=0.18,
            elem_a="leaf_panel", name="front leaf covers half of the box opening",
        )
        ctx.expect_overlap(
            rear_leaf, body, axes="xy", min_overlap=0.18,
            elem_a="leaf_panel", name="rear leaf covers half of the box opening",
        )
        ctx.expect_gap(
            rear_leaf, front_leaf, axis="y", min_gap=0.004, max_gap=0.008,
            positive_elem="leaf_panel", negative_elem="leaf_panel",
            name="split lid has a narrow center seam",
        )
        closed_front = ctx.part_world_aabb(front_leaf)
        closed_rear = ctx.part_world_aabb(rear_leaf)

    with ctx.pose({front_hinge: 1.5, rear_hinge: 1.5}):
        open_front = ctx.part_world_aabb(front_leaf)
        open_rear = ctx.part_world_aabb(rear_leaf)
    ctx.check(
        "both split lid leaves open upward",
        closed_front is not None and open_front is not None
        and closed_rear is not None and open_rear is not None
        and open_front[1][2] > closed_front[1][2] + 0.10
        and open_rear[1][2] > closed_rear[1][2] + 0.10,
        details=f"front_closed={closed_front}, front_open={open_front}, "
                f"rear_closed={closed_rear}, rear_open={open_rear}",
    )

    # The body is intentionally a square, squat wooden box rather than a tall chest.
    floor_aabb = ctx.part_element_world_aabb(body, elem="floor_panel")
    wall_aabb = ctx.part_element_world_aabb(body, elem="wall_front")
    ctx.check(
        "body is square and squat",
        floor_aabb is not None and wall_aabb is not None
        and abs((floor_aabb[1][0] - floor_aabb[0][0]) - (floor_aabb[1][1] - floor_aabb[0][1])) < 0.01
        and (wall_aabb[1][2] - wall_aabb[0][2]) < 0.60 * (floor_aabb[1][0] - floor_aabb[0][0]),
        details=f"floor_aabb={floor_aabb}, wall_aabb={wall_aabb}",
    )

    # Rope side handles exist on both ends.
    ctx.check(
        "rope side handles present on both ends",
        body.get_visual("rope_left") is not None and body.get_visual("rope_right") is not None,
        details="expected rope_left and rope_right",
    )

    return ctx.report()
