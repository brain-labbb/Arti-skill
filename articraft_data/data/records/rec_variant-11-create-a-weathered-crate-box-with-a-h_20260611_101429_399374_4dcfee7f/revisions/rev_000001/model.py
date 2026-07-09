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
T = 0.018         # inner wall thickness
PLANK_T = 0.014   # raised weathered plank thickness
LID_T = 0.040     # flat lid thickness
SEAM_GAP = 0.0    # lid rests directly on the body rim


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="weathered_crate_box")

    wood = model.material("sun_bleached_plank", rgba=(0.50, 0.39, 0.25, 1.0))
    wood_light = model.material("worn_edge_wood", rgba=(0.66, 0.55, 0.38, 1.0))
    wood_dark = model.material("dark_gap_wood", rgba=(0.27, 0.20, 0.13, 1.0))
    iron = model.material("dull_black_iron", rgba=(0.12, 0.12, 0.13, 1.0))
    rope = model.material("frayed_rope", rgba=(0.56, 0.48, 0.32, 1.0))

    # --- Body (root) ---------------------------------------------------------
    body = model.part("box_body")
    body.visual(Box((W, D, T)), origin=Origin(xyz=(0.0, 0.0, T / 2.0)),
                material=wood_dark, name="floor_panel")
    cz = HB / 2.0
    # Dark recessed backing panels make the raised boards read as individual crate planks.
    body.visual(Box((W, T, HB)), origin=Origin(xyz=(0.0, -(D / 2.0 - T / 2.0), cz)),
                material=wood_dark, name="front_backing")
    body.visual(Box((W, T, HB)), origin=Origin(xyz=(0.0, (D / 2.0 - T / 2.0), cz)),
                material=wood_dark, name="back_backing")
    body.visual(Box((T, D, HB)), origin=Origin(xyz=(-(W / 2.0 - T / 2.0), 0.0, cz)),
                material=wood_dark, name="side_backing_0")
    body.visual(Box((T, D, HB)), origin=Origin(xyz=((W / 2.0 - T / 2.0), 0.0, cz)),
                material=wood_dark, name="side_backing_1")

    # Raised plank courses, with uneven heights/colors and dark grooves between them.
    plank_rows = (
        (0.046, 0.046, wood),
        (0.112, 0.052, wood_light),
        (0.183, 0.058, wood),
    )
    for i, (z, h, mat) in enumerate(plank_rows):
        body.visual(Box((W + 0.018, PLANK_T, h)),
                    origin=Origin(xyz=(0.0, -D / 2.0 - PLANK_T / 2.0, z)),
                    material=mat, name=f"front_plank_{i}")
        body.visual(Box((W + 0.018, PLANK_T, h)),
                    origin=Origin(xyz=(0.0, D / 2.0 + PLANK_T / 2.0, z)),
                    material=mat, name=f"back_plank_{i}")
        body.visual(Box((PLANK_T, D + 0.018, h)),
                    origin=Origin(xyz=(-W / 2.0 - PLANK_T / 2.0, 0.0, z)),
                    material=mat, name=f"side_plank_0_{i}")
        body.visual(Box((PLANK_T, D + 0.018, h)),
                    origin=Origin(xyz=(W / 2.0 + PLANK_T / 2.0, 0.0, z)),
                    material=mat, name=f"side_plank_1_{i}")

    # Raised wooden corner blocks replace the parent metal corner brackets and
    # act as real cleats tying the plank courses together.
    bk = 0.058
    idx = 0
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            body.visual(
                Box((bk, bk, HB)),
                origin=Origin(xyz=(sx * (W / 2.0 - bk / 2.0 + 0.018),
                                   sy * (D / 2.0 - bk / 2.0 + 0.018), HB / 2.0)),
                material=wood_light, name=f"corner_block_{idx}",
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

    # fixed front pull plate only; the variant keeps the lid as the sole moving part.
    body.visual(Box((0.08, 0.010, 0.035)),
                origin=Origin(xyz=(0.0, -(D / 2.0) - 0.018, HB * 0.56)),
                material=iron, name="front_pull_plate")

    # --- Lid (flat, hinged at the back top edge) -----------------------------
    lid = model.part("box_lid")
    lid.visual(Box((W + 0.02, D + 0.02, LID_T)),
               origin=Origin(xyz=(0.0, -D / 2.0, SEAM_GAP + LID_T / 2.0)),
               material=wood, name="lid_panel")
    lid.visual(Box((W + 0.04, 0.050, 0.026)),
               origin=Origin(xyz=(0.0, -D + 0.045, SEAM_GAP + LID_T + 0.013)),
               material=wood_light, name="front_lid_cleat")
    lid.visual(Box((W + 0.04, 0.050, 0.026)),
               origin=Origin(xyz=(0.0, -0.045, SEAM_GAP + LID_T + 0.013)),
               material=wood_light, name="rear_lid_cleat")
    # iron straps across the lid
    for sx in (-0.13, 0.13):
        lid.visual(Box((0.028, D + 0.02, 0.006)),
                   origin=Origin(xyz=(sx, -D / 2.0, SEAM_GAP + LID_T + 0.003)),
                   material=iron, name=f"lid_strap_{'l' if sx < 0 else 'r'}")
    lid.visual(Box((0.085, 0.012, 0.030)),
               origin=Origin(xyz=(0.0, -(D + 0.016), SEAM_GAP + LID_T * 0.55)),
               material=iron, name="fixed_latch_plate")

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, D / 2.0, HB)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=2.0, lower=0.0, upper=2.1),
    )

    return model


object_model = build_object_model()


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("box_body")
    lid = object_model.get_part("box_lid")
    lid_hinge = object_model.get_articulation("lid_hinge")

    ctx.check(
        "single lid articulation only",
        len(object_model.articulations) == 1 and object_model.articulations[0].name == "lid_hinge",
        details=f"articulations={[a.name for a in object_model.articulations]}",
    )

    # Closed lid seats on the body rim without penetrating it.
    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(lid, body, axis="z", min_gap=0.0, max_gap=0.006,
                       positive_elem="lid_panel", negative_elem="front_backing",
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

    # Rope side handles exist on both ends.
    ctx.check(
        "rope side handles present on both ends",
        body.get_visual("rope_left") is not None and body.get_visual("rope_right") is not None,
        details="expected rope_left and rope_right",
    )
    ctx.check(
        "raised wooden corner blocks present",
        all(body.get_visual(f"corner_block_{i}") is not None for i in range(4)),
        details="expected four raised corner_block visuals",
    )
    ctx.check(
        "crate plank courses present",
        all(body.get_visual(f"front_plank_{i}") is not None for i in range(3))
        and all(body.get_visual(f"side_plank_0_{i}") is not None for i in range(3)),
        details="expected raised plank courses on front and end faces",
    )

    return ctx.report()
