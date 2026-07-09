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
W = 0.32          # width (X), small cube-like workbench box
D = 0.32          # depth (Y)
HB = 0.32         # body height (Z)
T = 0.018         # plank wall thickness
DOOR_T = 0.022    # front door thickness
FRONT_Y = -D / 2.0


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cube_front_door_plank_box")

    wood = model.material("wood_plank", rgba=(0.62, 0.45, 0.26, 1.0))
    wood_dark = model.material("wood_dark", rgba=(0.45, 0.32, 0.18, 1.0))
    iron = model.material("iron_fitting", rgba=(0.22, 0.22, 0.24, 1.0))
    rope = model.material("rope", rgba=(0.55, 0.45, 0.28, 1.0))

    # --- Body (root) ---------------------------------------------------------
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
                material=wood_dark, name="top_panel")

    # narrow fixed front frame around the door opening
    body.visual(Box((W, T, 0.030)), origin=Origin(xyz=(0.0, FRONT_Y - T / 2.0, 0.015)),
                material=wood_dark, name="front_sill")
    body.visual(Box((W, T, 0.030)), origin=Origin(xyz=(0.0, FRONT_Y - T / 2.0, HB - 0.015)),
                material=wood_dark, name="front_header")
    body.visual(Box((0.030, T, HB)), origin=Origin(xyz=(-(W / 2.0 - 0.015), FRONT_Y - T / 2.0, cz)),
                material=wood_dark, name="front_jamb_hinge")
    body.visual(Box((0.030, T, HB)), origin=Origin(xyz=((W / 2.0 - 0.015), FRONT_Y - T / 2.0, cz)),
                material=wood_dark, name="front_jamb_latch")

    # proud plank seams: visible boards on the sides, back, and top
    seam = model.material("dark_plank_gaps", rgba=(0.12, 0.08, 0.04, 1.0))
    for z in (0.11, 0.21):
        body.visual(Box((W + 0.004, 0.004, 0.006)),
                    origin=Origin(xyz=(0.0, D / 2.0 + 0.002, z)),
                    material=seam, name=f"back_plank_gap_{z:.2f}")
        for sx, tag in ((-1.0, "hinge"), (1.0, "latch")):
            body.visual(Box((0.004, D - 0.04, 0.006)),
                        origin=Origin(xyz=(sx * (W / 2.0 + 0.002), 0.0, z)),
                        material=seam, name=f"side_plank_gap_{tag}_{z:.2f}")
    body.visual(Box((0.006, D + 0.004, 0.004)),
                origin=Origin(xyz=(0.0, 0.0, HB + 0.002)),
                material=seam, name="top_plank_gap")

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

    # vertical iron battens on the side faces
    for sx, tag in ((-1.0, "hinge"), (1.0, "latch")):
        body.visual(Box((0.008, 0.028, HB - 0.04)),
                    origin=Origin(xyz=(sx * (W / 2.0 + 0.004), 0.0, HB * 0.5)),
                    material=iron, name=f"side_batten_{tag}")

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

    # right-side keeper that receives the hasp when the door is shut
    body.visual(Box((0.070, 0.040, 0.040)),
                origin=Origin(xyz=(W / 2.0 + 0.018, FRONT_Y - 0.024, HB * 0.52)),
                material=iron, name="hasp_keeper")

    # hinge knuckles fixed to the body frame, alternating with door knuckles
    hinge_x = -W / 2.0
    hinge_y = FRONT_Y - 0.026
    for i, (z, length) in enumerate(((0.035, 0.050), (0.170, 0.050), (0.295, 0.045))):
        body.visual(Box((0.034, 0.008, length)),
                    origin=Origin(xyz=(hinge_x + 0.010, hinge_y + 0.010, z)),
                    material=iron, name=f"body_hinge_leaf_{i}")
        body.visual(Cylinder(radius=0.010, length=length),
                    origin=Origin(xyz=(hinge_x, hinge_y, z)),
                    material=iron, name=f"body_hinge_knuckle_{i}")

    # --- Front door (hinged at the left/front vertical edge) ------------------
    door = model.part("front_door")
    door.visual(Box((W - 0.040, DOOR_T, HB - 0.048)),
                origin=Origin(xyz=(W / 2.0, -0.006, HB / 2.0)),
                material=wood, name="door_panel")
    # separate visible planks and iron straps on the door face
    for z in (0.115, 0.205):
        door.visual(Box((W - 0.050, 0.004, 0.006)),
                    origin=Origin(xyz=(W / 2.0, -0.019, z)),
                    material=seam, name=f"door_plank_gap_{z:.2f}")
    for x, tag in ((W * 0.33, "lower"), (W * 0.67, "upper")):
        door.visual(Box((0.020, 0.006, HB - 0.070)),
                    origin=Origin(xyz=(x, -0.022, HB / 2.0)),
                    material=iron, name=f"door_strap_{tag}")
    for i, (z, length) in enumerate(((0.095, 0.060), (0.235, 0.060))):
        door.visual(Box((0.040, 0.008, length)),
                    origin=Origin(xyz=(0.020, 0.004, z)),
                    material=iron, name=f"door_hinge_leaf_{i}")
        door.visual(Cylinder(radius=0.010, length=length),
                    origin=Origin(xyz=(0.0, 0.0, z)),
                    material=iron, name=f"door_hinge_knuckle_{i}")

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(hinge_x, hinge_y, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.0, lower=0.0, upper=1.65),
    )

    # --- Front hasp (hinged on the door, swings away to unlatch) -------------
    hasp = model.part("hasp")
    hasp.visual(Box((0.030, 0.006, 0.040)),
                origin=Origin(xyz=(0.0, 0.006, 0.0)),
                material=iron, name="hasp_hinge_plate")
    hasp.visual(Box((0.110, 0.010, 0.026)),
                origin=Origin(xyz=(0.055, 0.0, 0.0)),
                material=iron, name="hasp_arm")
    hasp.visual(Box((0.020, 0.014, 0.030)),
                origin=Origin(xyz=(0.112, -0.001, 0.0)),
                material=iron, name="hasp_eye")
    # q=0 bridges across the right-side door seam; positive q swings it clear.
    model.articulation(
        "hasp_hinge",
        ArticulationType.REVOLUTE,
        parent=door,
        child=hasp,
        origin=Origin(xyz=(W - 0.075, -0.025, HB * 0.52)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=3.0, lower=0.0, upper=1.2),
    )

    return model


object_model = build_object_model()


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("box_body")
    door = object_model.get_part("front_door")
    hasp = object_model.get_part("hasp")
    door_hinge = object_model.get_articulation("door_hinge")
    hasp_hinge = object_model.get_articulation("hasp_hinge")

    # The new variant is a cube-like chest with a front hinged door, not a top lid.
    ctx.check(
        "cube-like storage box proportions",
        abs(W - D) < 0.01 and abs(HB - W) < 0.01,
        details=f"W={W}, D={D}, HB={HB}",
    )
    ctx.check(
        "front door and fixed top panel are present",
        door.get_visual("door_panel") is not None and body.get_visual("top_panel") is not None,
        details="expected a hinged front door plus a fixed top panel",
    )

    # Closed door sits just proud of the fixed front frame while covering the opening.
    with ctx.pose({door_hinge: 0.0, hasp_hinge: 0.0}):
        ctx.expect_gap(body, door, axis="y", min_gap=0.0, max_gap=0.006,
                       positive_elem="front_jamb_latch", negative_elem="door_panel",
                       name="closed front door sits proud of the frame")
        ctx.expect_overlap(door, body, axes="xz", min_overlap=0.20,
                           elem_a="door_panel",
                           name="closed door covers the front opening height")
        ctx.expect_overlap(hasp, body, axes="xz", min_overlap=0.010,
                           elem_a="hasp_eye", elem_b="hasp_keeper",
                           name="hasp eye aligns with the side keeper when shut")
        closed_door = ctx.part_world_aabb(door)

    with ctx.pose({door_hinge: 1.35, hasp_hinge: 0.0}):
        open_door = ctx.part_world_aabb(door)
    ctx.check(
        "front door swings outward",
        closed_door is not None and open_door is not None
        and open_door[0][1] < closed_door[0][1] - 0.10,
        details=f"closed={closed_door}, open={open_door}",
    )

    # Hasp releases by rotating away from the keeper.
    with ctx.pose({door_hinge: 0.0, hasp_hinge: 0.0}):
        hasp_closed = ctx.part_world_aabb(hasp)
    with ctx.pose({door_hinge: 0.0, hasp_hinge: 1.1}):
        hasp_open = ctx.part_world_aabb(hasp)
    ctx.check(
        "front hasp swings clear to release",
        hasp_closed is not None and hasp_open is not None
        and hasp_open[0][1] < hasp_closed[0][1] - 0.04,
        details=f"closed={hasp_closed}, open={hasp_open}",
    )

    # Rope side handles exist on both ends.
    ctx.check(
        "rope side handles present on both ends",
        body.get_visual("rope_left") is not None and body.get_visual("rope_right") is not None,
        details="expected rope_left and rope_right",
    )

    return ctx.report()
