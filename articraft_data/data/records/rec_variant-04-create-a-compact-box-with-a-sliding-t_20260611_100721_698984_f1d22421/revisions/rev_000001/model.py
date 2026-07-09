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
# Compact workbench storage box with a sliding top board captured in side grooves.
W = 0.42          # width (X)
D = 0.28          # depth (Y)
HB = 0.18         # body height (Z)
T = 0.016         # plank wall thickness
LID_T = 0.018     # sliding panel thickness
RUNNER_H = 0.008  # lower groove runner height
LID_Z = HB + RUNNER_H + LID_T / 2.0
SLIDE_TRAVEL = 0.135


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_sliding_plank_box")

    wood = model.material("wood_plank", rgba=(0.62, 0.45, 0.26, 1.0))
    wood_dark = model.material("wood_dark", rgba=(0.45, 0.32, 0.18, 1.0))
    groove_shadow = model.material("groove_shadow", rgba=(0.24, 0.16, 0.09, 1.0))
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

    # Subtle darker seams make the sides read as individual planks, not a
    # featureless block. They sit proud of the wall faces and remain fixed to
    # the body.
    for i, x in enumerate((-0.105, 0.0, 0.105)):
        body.visual(Box((0.004, 0.006, HB - 0.026)),
                    origin=Origin(xyz=(x, -(D / 2.0 + 0.003), HB / 2.0)),
                    material=groove_shadow, name=f"front_plank_seam_{i}")
        body.visual(Box((0.004, 0.006, HB - 0.026)),
                    origin=Origin(xyz=(x, (D / 2.0 + 0.003), HB / 2.0)),
                    material=groove_shadow, name=f"rear_plank_seam_{i}")
    for i, y in enumerate((-0.070, 0.0, 0.070)):
        body.visual(Box((0.006, 0.004, HB - 0.026)),
                    origin=Origin(xyz=(-(W / 2.0 + 0.003), y, HB / 2.0)),
                    material=groove_shadow, name=f"side_plank_seam_{i}")
        body.visual(Box((0.006, 0.004, HB - 0.026)),
                    origin=Origin(xyz=((W / 2.0 + 0.003), y, HB / 2.0)),
                    material=groove_shadow, name=f"side_plank_seam_r_{i}")

    # Shallow side grooves: lower runners carry the sliding lid and upper lips
    # retain the lid edges. These are the structural change from a hinged chest.
    runner_w = 0.034
    rail_x = W / 2.0 - T - runner_w / 2.0
    rail_len = D + 0.060
    body.visual(Box((runner_w, rail_len, RUNNER_H)),
                origin=Origin(xyz=(-rail_x, 0.010, HB + RUNNER_H / 2.0)),
                material=wood_dark, name="lower_runner_left")
    body.visual(Box((runner_w, rail_len, RUNNER_H)),
                origin=Origin(xyz=(rail_x, 0.010, HB + RUNNER_H / 2.0)),
                material=wood_dark, name="lower_runner_right")
    body.visual(Box((runner_w, rail_len, 0.006)),
                origin=Origin(xyz=(-rail_x, 0.010, HB + RUNNER_H + LID_T + 0.004)),
                material=wood_dark, name="upper_lip_left")
    body.visual(Box((runner_w, rail_len, 0.006)),
                origin=Origin(xyz=(rail_x, 0.010, HB + RUNNER_H + LID_T + 0.004)),
                material=wood_dark, name="upper_lip_right")
    web_h = RUNNER_H + LID_T + 0.009
    body.visual(Box((0.016, rail_len, web_h)),
                origin=Origin(xyz=(-(W / 2.0 - 0.008), 0.010, HB + web_h / 2.0)),
                material=wood_dark, name="groove_web_left")
    body.visual(Box((0.016, rail_len, web_h)),
                origin=Origin(xyz=((W / 2.0 - 0.008), 0.010, HB + web_h / 2.0)),
                material=wood_dark, name="groove_web_right")

    # A front stop keeps the sliding lid from drifting out of the front slot.
    body.visual(Box((W, 0.018, 0.007)),
                origin=Origin(xyz=(0.0, -(D / 2.0 + 0.009), HB + 0.0035)),
                material=wood_dark, name="front_lid_stop")

    # rope side handles: a hanging rope held by two iron staples on each end face
    for sx, tag in ((-1.0, "left"), (1.0, "right")):
        xface = sx * (W / 2.0)
        for sy in (-0.08, 0.08):
            body.visual(Box((0.032, 0.024, 0.03)),
                        origin=Origin(xyz=(xface + sx * 0.013, sy, HB * 0.62)),
                        material=wood_dark, name=f"staple_{tag}_{'f' if sy < 0 else 'b'}")
        # rope grab handle spanning between the two staples (ends seat in staples)
        body.visual(
            Cylinder(radius=0.010, length=0.20),
            origin=Origin(xyz=(xface + sx * 0.022, 0.0, HB * 0.62),
                          rpy=(1.5707963, 0.0, 0.0)),
            material=rope, name=f"rope_{tag}",
        )

    # --- Sliding top lid panel -----------------------------------------------
    lid = model.part("sliding_lid")
    lid.visual(Box((W - 2.0 * T - 0.010, D + 0.020, LID_T)),
               origin=Origin(xyz=(0.0, 0.0, 0.0)),
               material=wood, name="lid_panel")
    # End grip cleat and plank seams are part of the sliding board.
    lid.visual(Box((W - 2.0 * T - 0.070, 0.018, 0.014)),
               origin=Origin(xyz=(0.0, -(D / 2.0 + 0.018), 0.012)),
               material=wood_dark, name="pull_cleat")
    for i, x in enumerate((-0.090, 0.0, 0.090)):
        lid.visual(Box((0.004, D + 0.010, 0.004)),
                   origin=Origin(xyz=(x, 0.0, LID_T / 2.0 + 0.002)),
                   material=groove_shadow, name=f"lid_plank_seam_{i}")

    model.articulation(
        "lid_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, LID_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.6, lower=0.0, upper=SLIDE_TRAVEL),
    )

    return model


object_model = build_object_model()


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("box_body")
    lid = object_model.get_part("sliding_lid")
    lid_slide = object_model.get_articulation("lid_slide")

    # Closed sliding board is supported by the shallow runners and covers the
    # storage opening.
    with ctx.pose({lid_slide: 0.0}):
        ctx.expect_gap(lid, body, axis="z", min_gap=0.0, max_gap=0.001,
                       positive_elem="lid_panel", negative_elem="lower_runner_left",
                       name="sliding lid rests on left groove runner")
        ctx.expect_gap(lid, body, axis="z", min_gap=0.0, max_gap=0.001,
                       positive_elem="lid_panel", negative_elem="lower_runner_right",
                       name="sliding lid rests on right groove runner")
        ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.18,
                           elem_a="lid_panel", elem_b="floor_panel",
                           name="closed sliding lid covers the box footprint")
        closed_lid = ctx.part_world_aabb(lid)

    with ctx.pose({lid_slide: SLIDE_TRAVEL}):
        open_lid = ctx.part_world_aabb(lid)
        ctx.expect_overlap(lid, body, axes="y", min_overlap=0.15,
                           elem_a="lid_panel", elem_b="lower_runner_left",
                           name="extended lid remains captured in grooves")
    ctx.check(
        "lid slides rearward to uncover the front",
        closed_lid is not None and open_lid is not None
        and open_lid[0][1] > closed_lid[0][1] + 0.10,
        details=f"closed={closed_lid}, open={open_lid}",
    )

    # Rope side handles exist on both ends.
    ctx.check(
        "rope side handles present on both ends",
        body.get_visual("rope_left") is not None and body.get_visual("rope_right") is not None,
        details="expected rope_left and rope_right",
    )

    ctx.check(
        "box has visible side grooves",
        body.get_visual("lower_runner_left") is not None
        and body.get_visual("upper_lip_left") is not None
        and body.get_visual("lower_runner_right") is not None
        and body.get_visual("upper_lip_right") is not None,
        details="expected lower runners and upper retaining lips on both sides",
    )

    return ctx.report()
