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
W = 0.42          # width (X)
D = 0.28          # depth (Y)
HB = 0.58         # upright body height (Z)
T = 0.018         # plank wall thickness
RAIL = 0.055      # fixed front frame / rim rail width
PANEL_GAP = 0.004


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="upright_keepsake_box")

    wood = model.material("wood_plank", rgba=(0.62, 0.45, 0.26, 1.0))
    wood_dark = model.material("wood_dark", rgba=(0.45, 0.32, 0.18, 1.0))
    iron = model.material("iron_fitting", rgba=(0.22, 0.22, 0.24, 1.0))
    rope = model.material("rope", rgba=(0.55, 0.45, 0.28, 1.0))

    # --- Upright carcass (root) ---------------------------------------------
    body = model.part("box_body")
    body.visual(Box((W, D, T)), origin=Origin(xyz=(0.0, 0.0, T / 2.0)),
                material=wood_dark, name="floor_panel")
    cz = HB / 2.0
    body.visual(Box((W, T, HB)), origin=Origin(xyz=(0.0, (D / 2.0 - T / 2.0), cz)),
                material=wood, name="wall_back")
    body.visual(Box((T, D, HB)), origin=Origin(xyz=(-(W / 2.0 - T / 2.0), 0.0, cz)),
                material=wood, name="wall_left")
    body.visual(Box((T, D, HB)), origin=Origin(xyz=((W / 2.0 - T / 2.0), 0.0, cz)),
                material=wood, name="wall_right")
    body.visual(Box((W, D, T)), origin=Origin(xyz=(0.0, 0.0, HB - T / 2.0)),
                material=wood_dark, name="top_rim_slab")
    body.visual(Box((W, T, RAIL)), origin=Origin(xyz=(0.0, -(D / 2.0 - T / 2.0), HB - RAIL / 2.0)),
                material=wood_dark, name="front_top_rail")
    body.visual(Box((W, T, RAIL)), origin=Origin(xyz=(0.0, -(D / 2.0 - T / 2.0), RAIL / 2.0)),
                material=wood_dark, name="front_bottom_rail")
    body.visual(Box((RAIL, T, HB)), origin=Origin(xyz=(-(W / 2.0 - RAIL / 2.0),
                                                      -(D / 2.0 - T / 2.0), cz)),
                material=wood_dark, name="front_stile_left")
    body.visual(Box((RAIL, T, HB)), origin=Origin(xyz=((W / 2.0 - RAIL / 2.0),
                                                      -(D / 2.0 - T / 2.0), cz)),
                material=wood_dark, name="front_stile_right")

    # Exposed plank courses on the taller case.
    for i, z in enumerate((0.13, 0.24, 0.35, 0.46)):
        body.visual(Box((W + 0.006, 0.010, 0.014)),
                    origin=Origin(xyz=(0.0, D / 2.0 + 0.003, z)),
                    material=wood_dark, name=f"back_plank_seam_{i}")
        body.visual(Box((0.010, D + 0.004, 0.014)),
                    origin=Origin(xyz=(-(W / 2.0 + 0.003), 0.0, z)),
                    material=wood_dark, name=f"side_seam_left_{i}")
        body.visual(Box((0.010, D + 0.004, 0.014)),
                    origin=Origin(xyz=((W / 2.0 + 0.003), 0.0, z)),
                    material=wood_dark, name=f"side_seam_right_{i}")

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

    # vertical iron battens on the rear face
    for sx in (-0.11, 0.11):
        body.visual(Box((0.024, 0.010, HB - 0.08)),
                    origin=Origin(xyz=(sx, D / 2.0 + 0.005, HB * 0.5)),
                    material=iron, name=f"rear_batten_{'l' if sx < 0 else 'r'}")

    # rope side handles: a hanging rope held by two iron staples on each end face
    for sx, tag in ((-1.0, "left"), (1.0, "right")):
        xface = sx * (W / 2.0)
        for sy in (-0.08, 0.08):
            body.visual(Box((0.032, 0.024, 0.03)),
                        origin=Origin(xyz=(xface + sx * 0.013, sy, HB * 0.55)),
                        material=iron, name=f"staple_{tag}_{'f' if sy < 0 else 'b'}")
        # rope grab handle spanning between the two staples (ends seat in staples)
        body.visual(
            Cylinder(radius=0.010, length=0.20),
            origin=Origin(xyz=(xface + sx * 0.022, 0.0, HB * 0.55),
                          rpy=(1.5707963, 0.0, 0.0)),
            material=rope, name=f"rope_{tag}",
        )

    # front staple that receives the drop-panel latch plate
    body.visual(Box((0.04, 0.022, 0.03)),
                origin=Origin(xyz=(0.0, -(D / 2.0) - 0.011, HB - RAIL / 2.0)),
                material=iron, name="hasp_keeper")

    # --- Front drop panel, hinged along the lower front edge -----------------
    panel_h = HB - 2.0 * RAIL - 2.0 * PANEL_GAP
    panel_w = W - 2.0 * RAIL - 2.0 * PANEL_GAP
    panel = model.part("drop_panel")
    panel.visual(Box((panel_w, T, panel_h)),
                 origin=Origin(xyz=(0.0, 0.0, panel_h / 2.0)),
                 material=wood, name="drop_panel")
    for i, z in enumerate((0.12, 0.24, 0.36)):
        panel.visual(Box((panel_w + 0.006, 0.008, 0.012)),
                     origin=Origin(xyz=(0.0, -0.008, z)),
                     material=wood_dark, name=f"panel_plank_seam_{i}")
    for sx in (-0.075, 0.075):
        panel.visual(Box((0.022, 0.010, panel_h - 0.06)),
                     origin=Origin(xyz=(sx, -0.010, panel_h / 2.0)),
                     material=iron, name=f"panel_strap_{'l' if sx < 0 else 'r'}")
    panel.visual(Box((0.052, 0.012, 0.070)),
                 origin=Origin(xyz=(0.0, -0.012, panel_h - 0.040)),
                 material=iron, name="latch_plate")
    panel.visual(Box((panel_w, 0.008, 0.022)),
                 origin=Origin(xyz=(0.0, 0.008, 0.008)),
                 material=iron, name="bottom_hinge_leaf")

    model.articulation(
        "panel_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=panel,
        origin=Origin(xyz=(0.0, -(D / 2.0 + T / 2.0), RAIL)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=0.0, upper=1.55),
    )

    return model


object_model = build_object_model()


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("box_body")
    panel = object_model.get_part("drop_panel")
    panel_hinge = object_model.get_articulation("panel_hinge")
    ctx.allow_overlap(
        body,
        panel,
        elem_a="front_bottom_rail",
        elem_b="bottom_hinge_leaf",
        reason="The drop panel hinge leaf is intentionally seated into the fixed lower rail.",
    )

    # Closed drop panel fills the framed front opening and sits just proud of it.
    with ctx.pose({panel_hinge: 0.0}):
        ctx.expect_within(panel, body, axes="xz", margin=0.003,
                          inner_elem="drop_panel",
                          name="drop panel height is within the front frame")
        ctx.expect_gap(body, panel, axis="y", min_gap=0.0, max_gap=0.014,
                       positive_elem="front_top_rail", negative_elem="drop_panel",
                       name="closed panel sits against the fixed front rim")
        ctx.expect_overlap(panel, body, axes="x", min_overlap=0.28,
                           elem_a="drop_panel", elem_b="front_bottom_rail",
                           name="drop panel spans the lower front opening")
        ctx.expect_overlap(panel, body, axes="xz", min_overlap=0.002,
                           elem_a="bottom_hinge_leaf", elem_b="front_bottom_rail",
                           name="bottom hinge leaf is retained by the lower rail")
        closed_panel = ctx.part_world_aabb(panel)

    with ctx.pose({panel_hinge: 1.35}):
        open_panel = ctx.part_world_aabb(panel)
    ctx.check(
        "front drop panel hinges downward",
        closed_panel is not None and open_panel is not None
        and open_panel[0][1] < closed_panel[0][1] - 0.18
        and open_panel[1][2] < closed_panel[1][2] - 0.10,
        details=f"closed={closed_panel}, open={open_panel}",
    )

    ctx.check("fixed top rim present",
              body.get_visual("top_rim_slab") is not None and body.get_visual("front_top_rail") is not None,
              details="expected fixed top rim slab and front top rail")

    # Rope side handles exist on both ends.
    ctx.check(
        "rope side handles present on both ends",
        body.get_visual("rope_left") is not None and body.get_visual("rope_right") is not None,
        details="expected rope_left and rope_right",
    )

    return ctx.report()
