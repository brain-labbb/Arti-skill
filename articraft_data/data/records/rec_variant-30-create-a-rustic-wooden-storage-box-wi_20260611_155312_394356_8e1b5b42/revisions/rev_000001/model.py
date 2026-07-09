from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# --- Dimensions (meters) -----------------------------------------------------
W = 0.48          # width (X)
D = 0.40          # depth (Y)
HB = 0.25         # body height (Z)
T = 0.018         # plank wall thickness
LID_T = 0.040     # flat lid thickness
SEAM_GAP = 0.0    # lid rests directly on the body rim
HATCH_W = 0.22
HATCH_D = 0.20
HATCH_T = 0.026
HATCH_RISE = 0.010


def side_wall_with_grip() -> object:
    wall = cq.Workplane("XY").box(T, D, HB)
    grip = (
        cq.Workplane("YZ")
        .center(0.0, HB * 0.64)
        .rect(0.16, 0.055)
        .vertices()
        .circle(0.014)
        .extrude(T * 3.0, both=True)
    )
    return wall.cut(grip)


def lid_frame_shape() -> object:
    outer = cq.Workplane("XY").box(W + 0.02, D + 0.02, LID_T)
    hatch_cut = (
        cq.Workplane("XY")
        .box(HATCH_W + 0.010, HATCH_D + 0.010, LID_T + 0.004)
        .translate((0.0, 0.0, 0.002))
    )
    lip = (
        cq.Workplane("XY")
        .box(HATCH_W + 0.055, HATCH_D + 0.055, 0.010)
        .cut(cq.Workplane("XY").box(HATCH_W + 0.014, HATCH_D + 0.014, 0.012))
        .translate((0.0, 0.0, LID_T / 2.0 + 0.005))
    )
    return outer.cut(hatch_cut).union(lip)


def hatch_shape() -> object:
    base = cq.Workplane("XY").box(HATCH_W, HATCH_D, HATCH_T)
    raised = (
        cq.Workplane("XY")
        .box(HATCH_W - 0.050, HATCH_D - 0.050, HATCH_RISE)
        .translate((0.0, 0.0, HATCH_T / 2.0 + HATCH_RISE / 2.0))
    )
    return base.union(raised)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rustic_wooden_box")

    wood = model.material("wood_plank", rgba=(0.62, 0.45, 0.26, 1.0))
    wood_dark = model.material("wood_dark", rgba=(0.45, 0.32, 0.18, 1.0))
    iron = model.material("iron_fitting", rgba=(0.22, 0.22, 0.24, 1.0))
    # --- Body (root) ---------------------------------------------------------
    body = model.part("box_body")
    body.visual(Box((W, D, T)), origin=Origin(xyz=(0.0, 0.0, T / 2.0)),
                material=wood_dark, name="floor_panel")
    cz = HB / 2.0
    body.visual(Box((W, T, HB)), origin=Origin(xyz=(0.0, -(D / 2.0 - T / 2.0), cz)),
                material=wood, name="wall_front")
    body.visual(Box((W, T, HB)), origin=Origin(xyz=(0.0, (D / 2.0 - T / 2.0), cz)),
                material=wood, name="wall_back")
    grip_wall = mesh_from_cadquery(side_wall_with_grip(), "side_wall_grip")
    body.visual(grip_wall, origin=Origin(xyz=(-(W / 2.0 - T / 2.0), 0.0, cz)),
                material=wood, name="wall_left")
    body.visual(grip_wall, origin=Origin(xyz=((W / 2.0 - T / 2.0), 0.0, cz)),
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

    # vertical iron battens on the broad faces
    for sx in (-0.13, 0.13):
        body.visual(Box((0.028, D + 0.004, 0.008)),
                    origin=Origin(xyz=(sx, 0.0, HB * 0.5)),
                    material=iron, name=f"batten_{'l' if sx < 0 else 'r'}")

    # front staple that receives the hasp
    body.visual(Box((0.04, 0.012, 0.03)),
                origin=Origin(xyz=(0.0, -(D / 2.0) - 0.006, HB * 0.45)),
                material=iron, name="hasp_keeper")

    # --- Lid (flat, hinged at the back top edge) -----------------------------
    lid = model.part("box_lid")
    lid.visual(
        mesh_from_cadquery(lid_frame_shape(), "lid_frame"),
        origin=Origin(xyz=(0.0, -D / 2.0, SEAM_GAP + LID_T / 2.0)),
        material=wood,
        name="lid_frame",
    )
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

    hatch = model.part("access_hatch")
    hatch.visual(
        mesh_from_cadquery(hatch_shape(), "access_hatch"),
        origin=Origin(xyz=(0.0, -HATCH_D / 2.0, 0.020)),
        material=wood_dark,
        name="hatch_panel",
    )
    hatch.visual(
        Box((0.08, 0.016, 0.010)),
        origin=Origin(xyz=(0.0, -0.160, 0.038)),
        material=iron,
        name="hatch_pull",
    )
    hatch.visual(
        Box((0.14, 0.018, 0.008)),
        origin=Origin(xyz=(0.0, -0.006, 0.007)),
        material=iron,
        name="hatch_hinge_leaf",
    )
    model.articulation(
        "hatch_hinge",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=hatch,
        origin=Origin(xyz=(0.0, -D / 2.0 + (D - HATCH_D) / 2.0 + 0.005, SEAM_GAP + LID_T + 0.004)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.5, lower=0.0, upper=1.7),
    )

    # --- Front hasp (hinged on the lid front edge, swings to latch) ----------
    hasp = model.part("hasp")
    hasp.visual(Box((0.05, 0.010, 0.10)),
                origin=Origin(xyz=(0.0, 0.0, -0.05)),
                material=iron, name="hasp_arm")
    hasp.visual(Cylinder(radius=0.007, length=0.014),
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
    hatch = object_model.get_part("access_hatch")
    hasp = object_model.get_part("hasp")
    lid_hinge = object_model.get_articulation("lid_hinge")
    hatch_hinge = object_model.get_articulation("hatch_hinge")
    hasp_hinge = object_model.get_articulation("hasp_hinge")

    # Closed lid seats on the body rim without penetrating it.
    with ctx.pose({lid_hinge: 0.0, hatch_hinge: 0.0, hasp_hinge: 0.0}):
        ctx.expect_gap(lid, body, axis="z", min_gap=0.0, max_gap=0.006,
                       positive_elem="lid_frame", negative_elem="wall_front",
                       name="lid seats on the body rim when closed")
        ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.25,
                           name="closed lid covers the body opening")
        closed_lid = ctx.part_world_aabb(lid)
        closed_hatch = ctx.part_world_aabb(hatch)

    with ctx.pose({lid_hinge: 1.9}):
        open_lid = ctx.part_world_aabb(lid)
    ctx.check(
        "lid opens upward",
        closed_lid is not None and open_lid is not None
        and open_lid[1][2] > closed_lid[1][2] + 0.10,
        details=f"closed={closed_lid}, open={open_lid}",
    )

    with ctx.pose({lid_hinge: 0.0, hatch_hinge: 1.4}):
        open_hatch = ctx.part_world_aabb(hatch)
    ctx.check(
        "inset hatch opens independently",
        closed_hatch is not None and open_hatch is not None
        and open_hatch[1][2] > closed_hatch[1][2] + 0.04,
        details=f"closed={closed_hatch}, open={open_hatch}",
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

    # Side cutout grip body panels exist instead of rope handles.
    ctx.check(
        "chunky cutout grip side panels present",
        body.get_visual("wall_left") is not None
        and body.get_visual("wall_right") is not None,
        details="expected grip-cutout side walls on both ends",
    )

    return ctx.report()
