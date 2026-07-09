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
# Long, narrow workbench chest proportions. X is the long axis; the rear hinge
# remains on +Y and the hasp remains on the front -Y face.
W = 0.82          # length (X)
D = 0.24          # narrow depth (Y)
HB = 0.20         # body height (Z)
T = 0.018         # plank wall thickness
LID_T = 0.034     # flat lid thickness
SEAM_GAP = 0.0    # lid rests directly on the body rim


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="long_narrow_wooden_chest")

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

    # Rounded metal end caps replace the simple corner brackets.  Each cap has a
    # thin end plate plus round top/bottom rims and upright edge rolls.
    for sx, tag in ((-1.0, "end_0"), (1.0, "end_1")):
        xface = sx * (W / 2.0)
        body.visual(
            Box((0.018, D + 0.045, HB)),
            origin=Origin(xyz=(xface + sx * 0.006, 0.0, HB / 2.0)),
            material=iron,
            name=f"{tag}_cap_plate",
        )
        for z, ztag in ((0.006, "lower"), (HB - 0.022, "upper")):
            body.visual(
                Cylinder(radius=0.018, length=D + 0.052),
                origin=Origin(
                    xyz=(xface + sx * 0.006, 0.0, z),
                    rpy=(1.5707963, 0.0, 0.0),
                ),
                material=iron,
                name=f"{tag}_{ztag}_round",
            )
        for sy, ytag in ((-1.0, "front"), (1.0, "rear")):
            body.visual(
                Cylinder(radius=0.012, length=HB + 0.020),
                origin=Origin(
                    xyz=(xface + sx * 0.009, sy * (D / 2.0 + 0.020), HB / 2.0),
                ),
                material=iron,
                name=f"{tag}_{ytag}_upright",
            )

    # Long side plank seams and two steel bands reinforce the stretched chest.
    for sy, face in ((-1.0, "front"), (1.0, "rear")):
        for z, idx in ((HB * 0.36, 0), (HB * 0.68, 1)):
            body.visual(Box((W - 0.10, 0.006, 0.006)),
                        origin=Origin(xyz=(0.0, sy * (D / 2.0 + 0.001), z)),
                        material=wood_dark, name=f"{face}_plank_seam_{idx}")
    for sx in (-0.23, 0.23):
        body.visual(Box((0.026, D + 0.026, 0.010)),
                    origin=Origin(xyz=(sx, 0.0, HB * 0.55)),
                    material=iron, name=f"steel_band_{'a' if sx < 0 else 'b'}")

    # rope side handles: a hanging rope held by two iron staples on each metal end cap
    for sx, tag in ((-1.0, "end_0"), (1.0, "end_1")):
        xface = sx * (W / 2.0)
        for sy in (-0.060, 0.060):
            body.visual(Box((0.032, 0.024, 0.03)),
                        origin=Origin(xyz=(xface + sx * 0.024, sy, HB * 0.58)),
                        material=iron, name=f"staple_{tag}_{'f' if sy < 0 else 'b'}")
        # rope grab handle spanning between the two staples (ends seat in staples)
        body.visual(
            Cylinder(radius=0.010, length=0.20),
            origin=Origin(xyz=(xface + sx * 0.036, 0.0, HB * 0.58),
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
    # iron straps across the lid, aligned with the body steel bands
    for sx in (-0.23, 0.23):
        lid.visual(Box((0.028, D + 0.02, 0.006)),
                   origin=Origin(xyz=(sx, -D / 2.0, SEAM_GAP + LID_T + 0.003)),
                   material=iron, name=f"lid_strap_{'a' if sx < 0 else 'b'}")
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
        ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.20,
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

    body_box = ctx.part_world_aabb(body)
    ctx.check(
        "long narrow chest proportions",
        body_box is not None and (body_box[1][0] - body_box[0][0]) > 2.7 * (body_box[1][1] - body_box[0][1]),
        details=f"body_aabb={body_box}",
    )
    ctx.check(
        "rounded metal end caps present",
        body.get_visual("end_0_cap_plate") is not None
        and body.get_visual("end_1_cap_plate") is not None
        and body.get_visual("end_0_upper_round") is not None
        and body.get_visual("end_1_upper_round") is not None,
        details="expected cap plates and rounded upper rims on both narrow ends",
    )
    ctx.check(
        "rear hinge retained on long box back",
        lid_hinge.origin.xyz[1] > 0.0 and abs(lid_hinge.axis[0]) > 0.9,
        details=f"origin={lid_hinge.origin}, axis={lid_hinge.axis}",
    )

    # Rope side handles exist on both capped ends.
    ctx.check(
        "rope side handles present on both capped ends",
        body.get_visual("rope_end_0") is not None and body.get_visual("rope_end_1") is not None,
        details="expected rope_end_0 and rope_end_1",
    )

    return ctx.report()
