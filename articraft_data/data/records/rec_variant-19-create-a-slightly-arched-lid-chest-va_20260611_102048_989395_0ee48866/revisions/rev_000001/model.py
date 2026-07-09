from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

# --- Dimensions (meters) -----------------------------------------------------
W = 0.50          # width (X)
D = 0.34          # depth (Y)
HB = 0.24         # body height (Z)
T = 0.018         # plank wall thickness
LID_EDGE_H = 0.036  # arched lid height at front/rear edges
LID_RISE = 0.040    # modest crown rise over the box center
SEAM_GAP = 0.0    # lid rests directly on the body rim


def _add_quad(geom: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _arched_lid_mesh(width: float, depth: float, edge_h: float, rise: float, *, segments: int = 18) -> MeshGeometry:
    """Closed shallow barrel-top prism, local rear hinge at y=0 and underside z=0."""
    half_w = width / 2.0
    y_front = -depth
    y_rear = 0.0
    top: list[tuple[float, float]] = []
    for i in range(segments + 1):
        t = i / segments
        y = y_front + (y_rear - y_front) * t
        z = edge_h + rise * math.sin(math.pi * t)
        top.append((y, z))

    # cross-section loop in YZ: bottom front, bottom rear, top rear, arched top to front.
    section = [(y_front, 0.0), (y_rear, 0.0), top[-1], *reversed(top[:-1])]
    geom = MeshGeometry()
    left = [geom.add_vertex(-half_w, y, z) for y, z in section]
    right = [geom.add_vertex(half_w, y, z) for y, z in section]
    n = len(section)
    for i in range(n):
        j = (i + 1) % n
        _add_quad(geom, left[i], left[j], right[j], right[i])
    # end caps
    for i in range(1, n - 1):
        geom.add_face(left[0], left[i], left[i + 1])
        geom.add_face(right[0], right[i + 1], right[i])
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="arched_lid_storage_chest")

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

    # shallow grooves divide the sides into real-looking horizontal planks.
    for z, tag in ((HB * 0.36, "lower"), (HB * 0.68, "upper")):
        body.visual(Box((W + 0.006, 0.004, 0.006)),
                    origin=Origin(xyz=(0.0, -D / 2.0 - 0.003, z)),
                    material=wood_dark, name=f"front_plank_seam_{tag}")
        body.visual(Box((W + 0.006, 0.004, 0.006)),
                    origin=Origin(xyz=(0.0, D / 2.0 + 0.003, z)),
                    material=wood_dark, name=f"rear_plank_seam_{tag}")
        body.visual(Box((0.004, D + 0.006, 0.006)),
                    origin=Origin(xyz=(-W / 2.0 - 0.003, 0.0, z)),
                    material=wood_dark, name=f"side_plank_seam_{tag}_0")
        body.visual(Box((0.004, D + 0.006, 0.006)),
                    origin=Origin(xyz=(W / 2.0 + 0.003, 0.0, z)),
                    material=wood_dark, name=f"side_plank_seam_{tag}_1")

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

    # rear hinge plates bolted to the body carry the curved chest lid.
    for sx in (-0.13, 0.13):
        body.visual(Box((0.050, 0.014, 0.045)),
                    origin=Origin(xyz=(sx, D / 2.0 + 0.007, HB - 0.018)),
                    material=iron, name=f"rear_hinge_plate_{'l' if sx < 0 else 'r'}")
        body.visual(Box((0.045, 0.020, 0.012)),
                    origin=Origin(xyz=(sx, D / 2.0 + 0.023, HB + 0.002)),
                    material=iron, name=f"hinge_web_{'l' if sx < 0 else 'r'}")
        body.visual(Cylinder(radius=0.010, length=0.060),
                    origin=Origin(xyz=(sx, D / 2.0 + 0.032, HB + 0.006),
                                  rpy=(0.0, 1.5707963, 0.0)),
                    material=iron, name=f"hinge_barrel_{'l' if sx < 0 else 'r'}")

    # --- Lid (shallow barrel top, hinged at the back top edge) ---------------
    lid = model.part("box_lid")
    lid.visual(mesh_from_geometry(
        _arched_lid_mesh(W + 0.022, D + 0.024, LID_EDGE_H, LID_RISE),
        "arched_chest_lid_panel",
    ), origin=Origin(xyz=(0.0, 0.012, SEAM_GAP)),
       material=wood, name="arched_lid_panel")
    # end cap bands and iron straps reinforce the curved top.
    lid.visual(Box((0.010, D + 0.024, LID_EDGE_H + LID_RISE + 0.004)),
               origin=Origin(xyz=(-(W / 2.0 + 0.016), -D / 2.0, (LID_EDGE_H + LID_RISE) / 2.0)),
               material=iron, name="lid_end_band_0")
    lid.visual(Box((0.010, D + 0.024, LID_EDGE_H + LID_RISE + 0.004)),
               origin=Origin(xyz=((W / 2.0 + 0.016), -D / 2.0, (LID_EDGE_H + LID_RISE) / 2.0)),
               material=iron, name="lid_end_band_1")
    # iron straps across the lid: narrow arched strips following the crown.
    for sx in (-0.13, 0.13):
        lid.visual(mesh_from_geometry(
            _arched_lid_mesh(0.028, D + 0.026, LID_EDGE_H + 0.004, LID_RISE, segments=18),
            f"arched_lid_strap_{'l' if sx < 0 else 'r'}",
        ), origin=Origin(xyz=(sx, 0.013, SEAM_GAP)),
           material=iron, name=f"lid_strap_{'l' if sx < 0 else 'r'}")
    # hinge mount tab that carries the front hasp (reaches out to contact it)
    lid.visual(Box((0.05, 0.020, 0.016)),
               origin=Origin(xyz=(0.0, -(D + 0.003), SEAM_GAP + LID_EDGE_H * 0.5)),
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
        origin=Origin(xyz=(0.0, -(D + 0.018), SEAM_GAP + LID_EDGE_H * 0.5)),
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
                       positive_elem="arched_lid_panel", negative_elem="wall_front",
                       name="arched lid seats on the body rim when closed")
        ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.25,
                           name="closed lid covers the body opening")
        lid_panel_aabb = ctx.part_element_world_aabb(lid, elem="arched_lid_panel")
        ctx.check(
            "curved lid crowns above the straight box rim",
            lid_panel_aabb is not None and lid_panel_aabb[1][2] > HB + 0.070,
            details=f"arched_lid_panel_aabb={lid_panel_aabb}",
        )
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

    ctx.check(
        "plank body has visible horizontal seams",
        body.get_visual("front_plank_seam_lower") is not None
        and body.get_visual("front_plank_seam_upper") is not None
        and body.get_visual("rear_plank_seam_lower") is not None,
        details="expected horizontal plank seam visuals on body faces",
    )

    # Rope side handles exist on both ends.
    ctx.check(
        "rope side handles present on both ends",
        body.get_visual("rope_left") is not None and body.get_visual("rope_right") is not None,
        details="expected rope_left and rope_right",
    )

    return ctx.report()
