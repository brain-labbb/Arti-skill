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
W = 0.52          # width (X)
D = 0.34          # depth (Y)
HB = 0.18         # low body height (Z)
T = 0.018         # plank wall thickness
LID_T = 0.040     # flat lid thickness
SEAM_GAP = 0.0    # lid rests directly on the body rim
DRAWER_W = 0.32   # sliding front compartment size
DRAWER_D = 0.22
DRAWER_H = 0.080


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
    # Front face is rebuilt as a real drawer opening: a lower sill, two side
    # stiles, and an upper rail instead of one unbroken plank wall.
    front_y = -(D / 2.0 - T / 2.0)
    opening_w = DRAWER_W + 0.020
    opening_bottom = 0.032
    opening_top = 0.118
    sill_h = opening_bottom
    rail_h = HB - opening_top
    stile_w = (W - opening_w) / 2.0
    body.visual(Box((W, T, sill_h)), origin=Origin(xyz=(0.0, front_y, sill_h / 2.0)),
                material=wood_dark, name="front_sill")
    body.visual(Box((W, T, rail_h)), origin=Origin(xyz=(0.0, front_y, opening_top + rail_h / 2.0)),
                material=wood, name="front_upper_rail")
    for sx, tag in ((-1.0, "left"), (1.0, "right")):
        body.visual(Box((stile_w, T, opening_top - opening_bottom)),
                    origin=Origin(xyz=(sx * (opening_w / 2.0 + stile_w / 2.0), front_y,
                                       (opening_bottom + opening_top) / 2.0)),
                    material=wood, name=f"front_stile_{tag}")
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
                    origin=Origin(xyz=(sx, 0.0, HB * 0.86)),
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
    body.visual(Box((0.04, 0.012, 0.026)),
                origin=Origin(xyz=(0.0, -(D / 2.0) - 0.006, 0.136)),
                material=iron, name="hasp_keeper")

    # Drawer runners fixed to the body inside the opening; they make the
    # prismatic drawer read as supported by rails rather than floating.
    body.visual(Box((0.018, DRAWER_D + 0.020, 0.018)),
                origin=Origin(xyz=(-(DRAWER_W / 2.0 + 0.018),
                                   -D / 2.0 + DRAWER_D / 2.0,
                                   0.040)),
                material=wood_dark, name="drawer_runner_left")
    body.visual(Box((0.018, DRAWER_D + 0.020, 0.018)),
                origin=Origin(xyz=((DRAWER_W / 2.0 + 0.018),
                                   -D / 2.0 + DRAWER_D / 2.0,
                                   0.040)),
                material=wood_dark, name="drawer_runner_right")

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
    hasp.visual(Box((0.05, 0.010, 0.070)),
                origin=Origin(xyz=(0.0, 0.0, -0.035)),
                material=iron, name="hasp_arm")
    hasp.visual(Box((0.024, 0.014, 0.018)),
                origin=Origin(xyz=(0.0, -0.004, -0.068)),
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

    # --- Small front drawer-like compartment ---------------------------------
    drawer = model.part("front_drawer")
    drawer_front_y = -D / 2.0 - 0.012
    drawer_inner_y = -D / 2.0 + DRAWER_D / 2.0 - 0.002
    drawer_z = 0.072
    drawer.visual(Box((DRAWER_W, 0.024, DRAWER_H)),
                  origin=Origin(xyz=(0.0, drawer_front_y, drawer_z)),
                  material=wood_dark, name="drawer_face")
    drawer.visual(Box((DRAWER_W * 0.90, DRAWER_D, 0.014)),
                  origin=Origin(xyz=(0.0, drawer_inner_y, opening_bottom + 0.007)),
                  material=wood, name="drawer_floor")
    for sx, tag in ((-1.0, "left"), (1.0, "right")):
        drawer.visual(Box((0.014, DRAWER_D, DRAWER_H * 0.78)),
                      origin=Origin(xyz=(sx * (DRAWER_W * 0.45 - 0.007), drawer_inner_y,
                                         opening_bottom + DRAWER_H * 0.39)),
                      material=wood, name=f"drawer_side_{tag}")
    drawer.visual(Box((DRAWER_W * 0.90, 0.014, DRAWER_H * 0.78)),
                  origin=Origin(xyz=(0.0, drawer_inner_y + DRAWER_D / 2.0 - 0.007,
                                     opening_bottom + DRAWER_H * 0.39)),
                  material=wood, name="drawer_back")
    drawer.visual(Cylinder(radius=0.014, length=0.032),
                  origin=Origin(xyz=(0.0, drawer_front_y - 0.016, drawer_z),
                                rpy=(1.5707963, 0.0, 0.0)),
                  material=iron, name="drawer_pull")

    model.articulation(
        "drawer_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.5, lower=0.0, upper=0.145),
    )

    return model


object_model = build_object_model()


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("box_body")
    lid = object_model.get_part("box_lid")
    hasp = object_model.get_part("hasp")
    drawer = object_model.get_part("front_drawer")
    lid_hinge = object_model.get_articulation("lid_hinge")
    hasp_hinge = object_model.get_articulation("hasp_hinge")
    drawer_slide = object_model.get_articulation("drawer_slide")

    # Closed lid seats on the body rim without penetrating it.
    with ctx.pose({lid_hinge: 0.0, hasp_hinge: 0.0}):
        ctx.expect_gap(lid, body, axis="z", min_gap=0.0, max_gap=0.006,
                       positive_elem="lid_panel", negative_elem="front_upper_rail",
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

    # New variant geometry: a low chest body with a real front drawer opening.
    ctx.check(
        "front drawer opening is framed by rails and stiles",
        body.get_visual("front_sill") is not None
        and body.get_visual("front_upper_rail") is not None
        and body.get_visual("front_stile_left") is not None
        and body.get_visual("front_stile_right") is not None,
        details="expected structural front opening members",
    )
    ctx.expect_within(drawer, body, axes="x", inner_elem="drawer_face",
                      outer_elem="front_upper_rail", margin=0.001,
                      name="drawer face fits within box width")
    ctx.expect_gap(body, drawer, axis="y", min_gap=0.0, max_gap=0.004,
                   positive_elem="front_upper_rail", negative_elem="drawer_face",
                   name="closed drawer face seats against front rail")

    with ctx.pose({drawer_slide: 0.0}):
        closed_drawer = ctx.part_world_aabb(drawer)
    with ctx.pose({drawer_slide: 0.12}):
        extended_drawer = ctx.part_world_aabb(drawer)
        ctx.expect_overlap(drawer, body, axes="y", elem_a="drawer_floor",
                           elem_b="drawer_runner_left", min_overlap=0.07,
                           name="extended drawer remains retained on runners")
    ctx.check(
        "front drawer slides outward from the box",
        closed_drawer is not None and extended_drawer is not None
        and extended_drawer[0][1] < closed_drawer[0][1] - 0.09,
        details=f"closed={closed_drawer}, extended={extended_drawer}",
    )

    return ctx.report()
