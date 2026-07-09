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
DIV_T = 0.018     # adjustable divider thickness (X)
DIV_D = D - 2.0 * T - 0.040
DIV_H = 0.174
DIV_Z = T + 0.016 + DIV_H / 2.0


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

    # Interior guide rails: a structural change for the movable divider panel.
    rail_len = W - 2.0 * T
    rail_y = D / 2.0 - T - 0.007
    for sy, tag in ((-1.0, "front"), (1.0, "back")):
        body.visual(
            Box((rail_len, 0.014, 0.014)),
            origin=Origin(xyz=(0.0, sy * rail_y, T + 0.008)),
            material=wood_dark,
            name=f"divider_lower_rail_{tag}",
        )
        body.visual(
            Box((rail_len, 0.014, 0.012)),
            origin=Origin(xyz=(0.0, sy * rail_y, HB - 0.040)),
            material=wood_dark,
            name=f"divider_upper_rail_{tag}",
        )

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

    # --- Adjustable interior divider (slides left-right in the guide rails) ---
    divider = model.part("divider_panel")
    divider.visual(
        Box((DIV_T, DIV_D, DIV_H)),
        origin=Origin(xyz=(0.0, 0.0, DIV_Z)),
        material=wood_dark,
        name="divider_board",
    )
    # Slightly thicker top and bottom tongues read as captured sliding edges.
    divider.visual(
        Box((DIV_T + 0.010, DIV_D + 0.012, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, T + 0.017)),
        material=wood,
        name="lower_tongue",
    )
    divider.visual(
        Box((DIV_T + 0.010, DIV_D + 0.012, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, HB - 0.040)),
        material=wood,
        name="upper_tongue",
    )
    divider.visual(
        Cylinder(radius=0.010, length=0.070),
        origin=Origin(xyz=(0.0, -DIV_D / 2.0 - 0.006, DIV_Z),
                      rpy=(0.0, 1.5707963, 0.0)),
        material=iron,
        name="divider_finger_pull",
    )

    model.articulation(
        "divider_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=divider,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.4, lower=-0.155, upper=0.155),
    )

    return model


object_model = build_object_model()


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("box_body")
    lid = object_model.get_part("box_lid")
    hasp = object_model.get_part("hasp")
    divider = object_model.get_part("divider_panel")
    lid_hinge = object_model.get_articulation("lid_hinge")
    hasp_hinge = object_model.get_articulation("hasp_hinge")
    divider_slide = object_model.get_articulation("divider_slide")

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

    # Variant-specific box geometry: hollow body with interior guide rails.
    ctx.check(
        "hollow box has guide rails for an adjustable divider",
        body.get_visual("floor_panel") is not None
        and body.get_visual("wall_front") is not None
        and body.get_visual("wall_back") is not None
        and body.get_visual("divider_lower_rail_front") is not None
        and body.get_visual("divider_upper_rail_back") is not None,
        details="expected floor, side walls, and front/back divider guide rails",
    )
    ctx.expect_within(
        divider,
        body,
        axes="y",
        inner_elem="divider_board",
        outer_elem="floor_panel",
        margin=0.0,
        name="divider panel fits between the box front and back walls",
    )
    ctx.expect_gap(
        divider,
        body,
        axis="z",
        min_gap=0.0,
        max_gap=0.030,
        positive_elem="lower_tongue",
        negative_elem="floor_panel",
        name="divider lower tongue rides just above the box floor",
    )

    # Divider slides left-right while staying captured inside the storage box.
    with ctx.pose({divider_slide: 0.0}):
        divider_center = ctx.part_world_aabb(divider)
    with ctx.pose({divider_slide: 0.140}):
        divider_right = ctx.part_world_aabb(divider)
        ctx.expect_within(
            divider,
            body,
            axes="xy",
            inner_elem="divider_board",
            outer_elem="floor_panel",
            margin=0.002,
            name="divider remains inside the box at right adjustment",
        )
    with ctx.pose({divider_slide: -0.140}):
        divider_left = ctx.part_world_aabb(divider)
        ctx.expect_within(
            divider,
            body,
            axes="xy",
            inner_elem="divider_board",
            outer_elem="floor_panel",
            margin=0.002,
            name="divider remains inside the box at left adjustment",
        )
    ctx.check(
        "divider prismatic joint moves left-right",
        divider_center is not None and divider_right is not None and divider_left is not None
        and divider_right[0][0] > divider_center[0][0] + 0.10
        and divider_left[1][0] < divider_center[1][0] - 0.10,
        details=f"center={divider_center}, right={divider_right}, left={divider_left}",
    )

    return ctx.report()
