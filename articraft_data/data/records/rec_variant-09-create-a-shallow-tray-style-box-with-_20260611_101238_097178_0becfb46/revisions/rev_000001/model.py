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
HB = 0.085        # shallow tray body height (Z)
T = 0.014         # low plank wall thickness
LID_T = 0.012     # thin clear display lid thickness
SEAM_GAP = 0.0    # lid rests directly on the body rim


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="shallow_display_tray_box")

    wood = model.material("wood_plank", rgba=(0.62, 0.45, 0.26, 1.0))
    wood_dark = model.material("wood_dark", rgba=(0.45, 0.32, 0.18, 1.0))
    iron = model.material("iron_fitting", rgba=(0.22, 0.22, 0.24, 1.0))
    rope = model.material("rope", rgba=(0.55, 0.45, 0.28, 1.0))
    clear = model.material("clear_acrylic", rgba=(0.70, 0.88, 1.0, 0.38))

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

    # low iron corner brackets at the four vertical edges
    bk = 0.030
    idx = 0
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            body.visual(
                Box((bk, bk, HB - 0.012)),
                origin=Origin(xyz=(sx * (W / 2.0 - bk / 2.0 + 0.003),
                                   sy * (D / 2.0 - bk / 2.0 + 0.003), HB / 2.0)),
                material=iron, name=f"corner_bracket_{idx}",
            )
            idx += 1

    # flat iron bands around the shallow tray walls
    for sx in (-0.13, 0.13):
        body.visual(Box((0.026, D + 0.004, 0.006)),
                    origin=Origin(xyz=(sx, 0.0, HB * 0.58)),
                    material=iron, name=f"batten_{'l' if sx < 0 else 'r'}")

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

    # front staple that receives the hasp
    body.visual(Box((0.04, 0.012, 0.03)),
                origin=Origin(xyz=(0.0, -(D / 2.0) - 0.006, HB * 0.45)),
                material=iron, name="hasp_keeper")

    # --- Clear display lid (hinged at the back top edge) ---------------------
    lid = model.part("box_lid")
    lid.visual(Box((W + 0.018, D + 0.018, LID_T)),
               origin=Origin(xyz=(0.0, -D / 2.0, SEAM_GAP + LID_T / 2.0)),
               material=clear, name="clear_lid_panel")
    # slim wood rim captures the transparent panel so it reads as a real display lid
    rail_t = 0.020
    rail_z = SEAM_GAP + LID_T + 0.006
    lid.visual(Box((W + 0.032, rail_t, 0.012)),
               origin=Origin(xyz=(0.0, 0.0, rail_z)),
               material=wood, name="rear_lid_rail")
    lid.visual(Box((W + 0.032, rail_t, 0.012)),
               origin=Origin(xyz=(0.0, -D, rail_z)),
               material=wood, name="front_lid_rail")
    lid.visual(Box((rail_t, D + 0.032, 0.012)),
               origin=Origin(xyz=(-(W / 2.0 + 0.006), -D / 2.0, rail_z)),
               material=wood, name="side_lid_rail_0")
    lid.visual(Box((rail_t, D + 0.032, 0.012)),
               origin=Origin(xyz=((W / 2.0 + 0.006), -D / 2.0, rail_z)),
               material=wood, name="side_lid_rail_1")
    # hinge mount tab that carries the front hasp (reaches out to contact it)
    lid.visual(Box((0.05, 0.018, 0.012)),
               origin=Origin(xyz=(0.0, -(D + 0.006), SEAM_GAP + LID_T * 0.5)),
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
    hasp.visual(Box((0.05, 0.010, 0.055)),
                origin=Origin(xyz=(0.0, 0.0, -0.0275)),
                material=iron, name="hasp_arm")
    hasp.visual(Box((0.024, 0.014, 0.018)),
                origin=Origin(xyz=(0.0, -0.004, -0.053)),
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
                       positive_elem="clear_lid_panel", negative_elem="wall_front",
                       name="clear lid seats on the low tray rim when closed")
        ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.25,
                           elem_a="clear_lid_panel", elem_b="floor_panel",
                           name="clear display lid covers the tray opening")
        closed_lid = ctx.part_world_aabb(lid)

    with ctx.pose({lid_hinge: 1.9}):
        open_lid = ctx.part_world_aabb(lid)
    ctx.check(
        "clear display lid opens upward",
        closed_lid is not None and open_lid is not None
        and open_lid[1][2] > closed_lid[1][2] + 0.10,
        details=f"closed={closed_lid}, open={open_lid}",
    )

    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "box is a shallow low-walled tray",
        body_aabb is not None and (body_aabb[1][2] - body_aabb[0][2]) < 0.12,
        details=f"body_aabb={body_aabb}",
    )
    ctx.check(
        "transparent display panel is present",
        lid.get_visual("clear_lid_panel") is not None
        and lid.get_visual("front_lid_rail") is not None
        and lid.get_visual("rear_lid_rail") is not None,
        details="expected clear_lid_panel captured by wood lid rails",
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

    return ctx.report()
