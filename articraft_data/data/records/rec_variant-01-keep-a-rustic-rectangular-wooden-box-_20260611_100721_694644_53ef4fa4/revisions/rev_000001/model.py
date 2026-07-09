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
W = 0.72          # width (X), widened for the low workbench chest variant
D = 0.30          # depth (Y)
HB = 0.16         # lower body height (Z)
T = 0.022         # thicker plank wall thickness
LID_T = 0.032     # flat lid thickness
SEAM_GAP = 0.0    # lid rests directly on the body rim
HINGE_Y = 0.024   # rear hinge barrel offset behind the box
HINGE_Z = 0.006   # rear hinge barrel offset above the rim


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

    # raised plank bands thicken the side construction and break the low chest
    # into visible horizontal boards rather than a single smooth box wall.
    band_h = 0.030
    band_t = 0.014
    for zc, ztag in ((HB * 0.32, "lower"), (HB * 0.72, "upper")):
        body.visual(Box((W + 0.020, band_t, band_h)),
                    origin=Origin(xyz=(0.0, -(D / 2.0 + band_t / 2.0 - 0.003), zc)),
                    material=wood_dark, name=f"front_plank_band_{ztag}")
        body.visual(Box((W + 0.020, band_t, band_h)),
                    origin=Origin(xyz=(0.0, (D / 2.0 + band_t / 2.0 - 0.003), zc)),
                    material=wood_dark, name=f"rear_plank_band_{ztag}")
        body.visual(Box((band_t, D + 0.010, band_h)),
                    origin=Origin(xyz=(-(W / 2.0 + band_t / 2.0 - 0.003), 0.0, zc)),
                    material=wood_dark, name=f"end_plank_band_{ztag}_0")
        body.visual(Box((band_t, D + 0.010, band_h)),
                    origin=Origin(xyz=((W / 2.0 + band_t / 2.0 - 0.003), 0.0, zc)),
                    material=wood_dark, name=f"end_plank_band_{ztag}_1")

    # chunky iron corner caps at the four vertical edges, with top and bottom
    # blocks instead of thin decorative straps.
    bk = 0.052
    cap_h = 0.052
    idx = 0
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            for zc, ztag in ((cap_h / 2.0, "bottom"), (HB - cap_h / 2.0, "top")):
                body.visual(
                    Box((bk, bk, cap_h)),
                    origin=Origin(xyz=(sx * (W / 2.0 - bk / 2.0 + 0.006),
                                       sy * (D / 2.0 - bk / 2.0 + 0.006), zc)),
                    material=iron, name=f"corner_cap_{ztag}_{idx}",
                )
            idx += 1

    # vertical iron battens on the wide front and rear plank faces
    for sx in (-0.22, 0.22):
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

    # visible rear hinge hardware: two fixed outer knuckles on the body, with
    # leaves screwed to the rear plank band.
    body.visual(Box((0.14, 0.026, 0.020)),
                origin=Origin(xyz=(-0.24, D / 2.0 + 0.013, HB - 0.010)),
                material=iron, name="rear_hinge_leaf_0")
    body.visual(Cylinder(radius=0.009, length=0.14),
                origin=Origin(xyz=(-0.24, D / 2.0 + HINGE_Y, HB + HINGE_Z),
                              rpy=(0.0, 1.5707963, 0.0)),
                material=iron, name="rear_hinge_barrel_0")
    body.visual(Box((0.14, 0.026, 0.020)),
                origin=Origin(xyz=(0.24, D / 2.0 + 0.013, HB - 0.010)),
                material=iron, name="rear_hinge_leaf_1")
    body.visual(Cylinder(radius=0.009, length=0.14),
                origin=Origin(xyz=(0.24, D / 2.0 + HINGE_Y, HB + HINGE_Z),
                              rpy=(0.0, 1.5707963, 0.0)),
                material=iron, name="rear_hinge_barrel_1")

    # --- Lid (flat, hinged at the back top edge) -----------------------------
    lid = model.part("box_lid")
    lid.visual(Box((W + 0.02, D + 0.02, LID_T)),
               origin=Origin(xyz=(0.0, -D / 2.0 - HINGE_Y,
                                  SEAM_GAP + LID_T / 2.0 - HINGE_Z)),
               material=wood, name="lid_panel")
    # iron straps across the lid
    for sx in (-0.22, 0.22):
        lid.visual(Box((0.028, D + 0.02, 0.006)),
                   origin=Origin(xyz=(sx, -D / 2.0 - HINGE_Y,
                                      SEAM_GAP + LID_T + 0.003 - HINGE_Z)),
                   material=iron, name=f"lid_strap_{'l' if sx < 0 else 'r'}")
    # central hinge knuckle moves with the lid between the two fixed body knuckles
    lid.visual(Box((0.24, 0.030, 0.012)),
               origin=Origin(xyz=(0.0, -0.012, 0.0)),
               material=iron, name="lid_hinge_leaf")
    lid.visual(Cylinder(radius=0.009, length=0.24),
               origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 1.5707963, 0.0)),
               material=iron, name="lid_hinge_barrel")
    # hinge mount tab that carries the front hasp (reaches out to contact it)
    lid.visual(Box((0.05, 0.020, 0.016)),
               origin=Origin(xyz=(0.0, -(D + HINGE_Y + 0.018),
                                  SEAM_GAP + LID_T * 0.5 - HINGE_Z)),
               material=iron, name="hasp_mount")

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, D / 2.0 + HINGE_Y, HB + HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=2.0, lower=0.0, upper=2.1),
    )

    # --- Front hasp (hinged on the lid front edge, swings to latch) ----------
    hasp = model.part("hasp")
    hasp.visual(Box((0.05, 0.010, 0.096)),
                origin=Origin(xyz=(0.0, 0.0, -0.052)),
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
        origin=Origin(xyz=(0.0, -(D + HINGE_Y + 0.018),
                           SEAM_GAP + LID_T * 0.5 - HINGE_Z)),
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
        ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.25,
                           name="closed lid covers the body opening")
        ctx.expect_overlap(lid, body, axes="yz", min_overlap=0.01,
                           elem_a="lid_hinge_barrel", elem_b="rear_hinge_barrel_0",
                           name="rear hinge knuckles share the same hinge line")
        closed_lid = ctx.part_world_aabb(lid)

    with ctx.pose({lid_hinge: 1.9}):
        open_lid = ctx.part_world_aabb(lid)
    ctx.check(
        "lid opens upward",
        closed_lid is not None and open_lid is not None
        and open_lid[1][2] > closed_lid[1][2] + 0.10,
        details=f"closed={closed_lid}, open={open_lid}",
    )

    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "box body is lower and wider than the parent chest",
        body_aabb is not None
        and (body_aabb[1][0] - body_aabb[0][0]) > 0.70
        and (body_aabb[1][2] - body_aabb[0][2]) < 0.22,
        details=f"body_aabb={body_aabb}",
    )
    ctx.check(
        "thick plank bands and chunky corner caps are present",
        body.get_visual("front_plank_band_upper") is not None
        and body.get_visual("rear_plank_band_lower") is not None
        and body.get_visual("corner_cap_top_0") is not None
        and body.get_visual("corner_cap_bottom_3") is not None,
        details="expected raised plank bands plus top/bottom metal corner caps",
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
