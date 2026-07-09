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
D = 0.36          # depth (Y)
HB = 0.32         # deeper body height (Z)
T = 0.018         # plank wall thickness
LID_T = 0.040     # flat lid thickness
SEAM_GAP = 0.0    # lid rests directly on the body rim


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

    # Deep-box structural rim: continuous inner cleats carry the lid load and
    # make the walls read as a real chest rather than a shallow crate.
    rim_z = HB - 0.030
    body.visual(Box((W - 2 * T, 0.018, 0.022)),
                origin=Origin(xyz=(0.0, -(D / 2.0 - T - 0.009), rim_z)),
                material=wood_dark, name="front_inner_rim")
    body.visual(Box((W - 2 * T, 0.018, 0.022)),
                origin=Origin(xyz=(0.0, (D / 2.0 - T - 0.009), rim_z)),
                material=wood_dark, name="rear_inner_rim")
    body.visual(Box((0.018, D - 2 * T, 0.022)),
                origin=Origin(xyz=(-(W / 2.0 - T - 0.009), 0.0, rim_z)),
                material=wood_dark, name="side_inner_rim_0")
    body.visual(Box((0.018, D - 2 * T, 0.022)),
                origin=Origin(xyz=((W / 2.0 - T - 0.009), 0.0, rim_z)),
                material=wood_dark, name="side_inner_rim_1")

    # two front keeper plates that receive the rotating clasp latches
    for i, x in enumerate((-0.13, 0.13)):
        body.visual(Box((0.055, 0.012, 0.038)),
                    origin=Origin(xyz=(x, -(D / 2.0) - 0.006, HB - 0.068)),
                    material=iron, name=f"clasp_keeper_{i}")

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
    # hinge tabs that carry the two front clasp latches
    for i, x in enumerate((-0.13, 0.13)):
        lid.visual(Box((0.060, 0.020, 0.016)),
                   origin=Origin(xyz=(x, -(D + 0.003), SEAM_GAP + LID_T * 0.5)),
                   material=iron, name=f"clasp_mount_{i}")

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, D / 2.0, HB)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=2.0, lower=0.0, upper=2.1),
    )

    # --- Two front clasp latches (small rotating catches on the lid edge) ----
    for i, x in enumerate((-0.13, 0.13)):
        clasp = model.part(f"clasp_{i}")
        clasp.visual(Box((0.046, 0.010, 0.086)),
                     origin=Origin(xyz=(0.0, 0.0, -0.043)),
                     material=iron, name="clasp_arm")
        clasp.visual(Box((0.030, 0.014, 0.032)),
                     origin=Origin(xyz=(0.0, -0.004, -0.079)),
                     material=iron, name="clasp_hook")
        # q=0 hangs the clasp over the front keeper; positive q rotates it up
        # and away from the keeper so the lid can be lifted.
        model.articulation(
            f"clasp_hinge_{i}",
            ArticulationType.REVOLUTE,
            parent=lid,
            child=clasp,
            origin=Origin(xyz=(x, -(D + 0.018), SEAM_GAP + LID_T * 0.5)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=0.0, upper=1.35),
        )

    return model


object_model = build_object_model()


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("box_body")
    lid = object_model.get_part("box_lid")
    clasp_0 = object_model.get_part("clasp_0")
    clasp_1 = object_model.get_part("clasp_1")
    lid_hinge = object_model.get_articulation("lid_hinge")
    clasp_hinge_0 = object_model.get_articulation("clasp_hinge_0")
    clasp_hinge_1 = object_model.get_articulation("clasp_hinge_1")

    # Closed lid seats on the body rim without penetrating it.
    with ctx.pose({lid_hinge: 0.0, clasp_hinge_0: 0.0, clasp_hinge_1: 0.0}):
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

    # Both small clasp latches release when rotated upward.
    with ctx.pose({lid_hinge: 0.0, clasp_hinge_0: 0.0, clasp_hinge_1: 0.0}):
        clasp0_closed = ctx.part_world_aabb(clasp_0)
        clasp1_closed = ctx.part_world_aabb(clasp_1)
        ctx.expect_overlap(clasp_0, body, axes="xz", min_overlap=0.025,
                           elem_a="clasp_hook", elem_b="clasp_keeper_0",
                           name="first clasp hook covers its keeper")
        ctx.expect_overlap(clasp_1, body, axes="xz", min_overlap=0.025,
                           elem_a="clasp_hook", elem_b="clasp_keeper_1",
                           name="second clasp hook covers its keeper")
    with ctx.pose({lid_hinge: 0.0, clasp_hinge_0: 1.25, clasp_hinge_1: 1.25}):
        clasp0_open = ctx.part_world_aabb(clasp_0)
        clasp1_open = ctx.part_world_aabb(clasp_1)
    ctx.check(
        "both front clasps rotate upward to release",
        clasp0_closed is not None and clasp0_open is not None
        and clasp1_closed is not None and clasp1_open is not None
        and clasp0_open[0][2] > clasp0_closed[0][2] + 0.02
        and clasp1_open[0][2] > clasp1_closed[0][2] + 0.02,
        details=f"c0_closed={clasp0_closed}, c0_open={clasp0_open}, c1_closed={clasp1_closed}, c1_open={clasp1_open}",
    )

    # Variant geometry: it is a deep rectangular storage box with an inner rim.
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "body is a deep rectangular box",
        body_aabb is not None
        and (body_aabb[1][2] - body_aabb[0][2]) > 0.30
        and (body_aabb[1][0] - body_aabb[0][0]) > (body_aabb[1][1] - body_aabb[0][1]) > 0.30,
        details=f"body_aabb={body_aabb}",
    )
    ctx.check(
        "inner support rim present",
        body.get_visual("front_inner_rim") is not None
        and body.get_visual("rear_inner_rim") is not None,
        details="expected front and rear inner rim cleats",
    )

    # Rope side handles exist on both ends.
    ctx.check(
        "rope side handles present on both ends",
        body.get_visual("rope_left") is not None and body.get_visual("rope_right") is not None,
        details="expected rope_left and rope_right",
    )

    return ctx.report()
