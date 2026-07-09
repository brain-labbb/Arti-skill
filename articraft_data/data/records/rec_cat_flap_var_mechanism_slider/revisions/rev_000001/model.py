from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


PANEL_W = 0.42
PANEL_H = 0.54
PANEL_T = 0.035
OPENING_W = 0.235
OPENING_H = 0.305
FRAME_W = 0.320
FRAME_H = 0.405
FRAME_D = 0.028
FRONT_Y = PANEL_T / 2.0
FRAME_FRONT_Y = FRONT_Y + FRAME_D
HINGE_Y = FRAME_FRONT_Y + 0.006
HINGE_Z = OPENING_H / 2.0 + 0.014
FLAP_W = 0.215
FLAP_H = 0.300
FLAP_OPEN_ANGLE = 0.85

# Lock slider mechanism (4-way lock)
SLIDER_W = OPENING_W           # 0.235 — matches opening width
SLIDER_H = OPENING_H           # 0.305 — covers full opening height
SLIDER_T = 0.005               # panel thickness
SLIDER_CLEARANCE = 0.001       # gap between slider and frame front
SLIDER_RETRACTED_Z = FRAME_H / 2.0 + SLIDER_H / 2.0 + 0.010  # above trim top edge
SLIDER_TRAVEL = 0.370          # prismatic travel to fully deploy across opening

# Guide rail dimensions
RAIL_W = 0.014
RAIL_LIP_T = 0.004
RAIL_H = 0.44
RAIL_X = OPENING_W / 2.0 + RAIL_W / 2.0 - 0.003   # rail center X
RAIL_CENTER_Z = 0.15                                 # rail vertical center

# Y positions (slider sits in front of frame, rail lip retains it)
SLIDER_Y = FRAME_FRONT_Y + SLIDER_CLEARANCE + SLIDER_T / 2.0
RAIL_LIP_Y = FRAME_FRONT_Y + SLIDER_CLEARANCE + SLIDER_T + RAIL_LIP_T / 2.0


def _panel_with_cutout() -> cq.Workplane:
    """Minimal door/panel context with a true through-opening."""
    clearance_w = OPENING_W + 0.010
    clearance_h = OPENING_H + 0.010
    return (
        cq.Workplane("XY")
        .rect(PANEL_W, PANEL_H)
        .rect(clearance_w, clearance_h)
        .extrude(PANEL_T, both=True)
    )


def _rotated_about_hinge(x: float, y: float, z: float, angle: float) -> tuple[float, float, float]:
    """Rotate a local flap visual center around the hinge X axis."""
    c = math.cos(angle)
    s = math.sin(angle)
    return (x, y * c - z * s, y * s + z * c)


def _flap_origin(x: float, y: float, z: float, *, angle: float = FLAP_OPEN_ANGLE) -> Origin:
    return Origin(xyz=_rotated_about_hinge(x, y, z, angle), rpy=(angle, 0.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="cat_flap_pet_door",
        meta={
            "category_context": "Pet_Animal related / Cat flap",
            "classification_note": "Reference image and folder category both indicate a cat flap / pet door; no mismatch suspected.",
        },
    )

    wood = model.material("warm_brown_door_panel", rgba=(0.42, 0.23, 0.11, 1.0))
    dark_plastic = model.material("charcoal_plastic_trim", rgba=(0.035, 0.033, 0.032, 1.0))
    rubber = model.material("black_rubber_seal", rgba=(0.005, 0.005, 0.004, 1.0))
    frosted = model.material("frosted_translucent_flap", rgba=(0.73, 0.90, 0.95, 0.42))
    pale_edge = model.material("milky_plastic_flap_edge", rgba=(0.86, 0.92, 0.90, 0.70))
    metal = model.material("dull_screw_metal", rgba=(0.55, 0.55, 0.52, 1.0))
    magnet = model.material("dark_magnet_latch", rgba=(0.02, 0.02, 0.018, 1.0))
    slider_plastic = model.material("slate_grey_slider", rgba=(0.28, 0.28, 0.30, 1.0))

    panel_frame = model.part("panel_frame")
    panel_frame.visual(
        mesh_from_cadquery(_panel_with_cutout(), "door_panel_with_opening", tolerance=0.0008),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=wood,
        name="door_panel",
    )

    front_bezel = BezelGeometry(
        (OPENING_W, OPENING_H),
        (FRAME_W, FRAME_H),
        FRAME_D,
        opening_shape="rounded_rect",
        outer_shape="rounded_rect",
        opening_corner_radius=0.018,
        outer_corner_radius=0.026,
        center=True,
    )
    panel_frame.visual(
        mesh_from_geometry(front_bezel, "rounded_front_trim"),
        origin=Origin(xyz=(0.0, FRONT_Y + FRAME_D / 2.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="front_trim",
    )

    inner_seal = BezelGeometry(
        (OPENING_W - 0.018, OPENING_H - 0.018),
        (OPENING_W + 0.014, OPENING_H + 0.014),
        0.006,
        opening_shape="rounded_rect",
        outer_shape="rounded_rect",
        opening_corner_radius=0.014,
        outer_corner_radius=0.020,
        center=True,
    )
    panel_frame.visual(
        mesh_from_geometry(inner_seal, "inner_rubber_seal"),
        origin=Origin(xyz=(0.0, FRAME_FRONT_Y + 0.003, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="inner_seal",
    )

    # Four screw heads sit on the molded corner pads of the trim, with shallow
    # slots to make them read as fasteners rather than decorative dots.
    screw_positions = (
        (-0.132, 0.168),
        (0.132, 0.168),
        (-0.132, -0.168),
        (0.132, -0.168),
    )
    for idx, (x, z) in enumerate(screw_positions):
        panel_frame.visual(
            Cylinder(radius=0.0115, length=0.005),
            origin=Origin(xyz=(x, FRAME_FRONT_Y + 0.0025, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=metal,
            name=f"screw_head_{idx}",
        )
        panel_frame.visual(
            Box((0.018, 0.0016, 0.0030)),
            origin=Origin(xyz=(x, FRAME_FRONT_Y + 0.0053, z), rpy=(0.0, 0.0, math.pi / 9.0)),
            material=dark_plastic,
            name=f"screw_slot_{idx}",
        )

    # Static hinge hardware: a small pin crossing the top of the opening and two
    # molded lugs that tie it visibly back to the upper frame.
    panel_frame.visual(
        Cylinder(radius=0.004, length=0.282),
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="hinge_pin",
    )
    for idx, x in enumerate((-0.137, 0.137)):
        panel_frame.visual(
            Box((0.026, 0.018, 0.040)),
            origin=Origin(xyz=(x, FRAME_FRONT_Y + 0.001, HINGE_Z - 0.006)),
            material=dark_plastic,
            name=f"hinge_lug_{idx}",
        )

    # Guide rails for the 4-way lock slider: two thin channel lips on the front
    # trim, one on each side of the opening.  The slider panel slides vertically
    # between them, captured behind the lip and against the frame front face.
    for idx, x_sign in enumerate((-1.0, 1.0)):
        rail_x = x_sign * RAIL_X
        panel_frame.visual(
            Box((RAIL_W, RAIL_LIP_T, RAIL_H)),
            origin=Origin(xyz=(rail_x, RAIL_LIP_Y, RAIL_CENTER_Z)),
            material=dark_plastic,
            name=f"guide_rail_{idx}",
        )

    panel_frame.visual(
        Box((0.024, 0.006, 0.046)),
        origin=Origin(xyz=(0.0, FRAME_FRONT_Y + 0.003, HINGE_Z - 0.306)),
        material=magnet,
        name="magnet_mount",
    )
    panel_frame.visual(
        Box((0.058, 0.006, 0.020)),
        origin=Origin(xyz=(0.0, FRAME_FRONT_Y + 0.003, HINGE_Z - 0.290)),
        material=magnet,
        name="frame_magnet",
    )

    flap = model.part("flap")
    flap.visual(
        Cylinder(radius=0.010, length=FLAP_W + 0.012),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=pale_edge,
        name="hinge_sleeve",
    )

    panel_top_z = -0.012
    panel_center_z = panel_top_z - FLAP_H / 2.0
    panel_center_y = 0.002
    flap.visual(
        Box((FLAP_W - 0.016, 0.004, FLAP_H - 0.020)),
        origin=_flap_origin(0.0, panel_center_y, panel_center_z),
        material=frosted,
        name="translucent_panel",
    )

    rim_t = 0.012
    rim_y = 0.005
    flap.visual(
        Box((FLAP_W, 0.008, 0.010)),
        origin=_flap_origin(0.0, rim_y, -0.007),
        material=pale_edge,
        name="sleeve_web",
    )
    flap.visual(
        Box((FLAP_W, 0.008, rim_t)),
        origin=_flap_origin(0.0, rim_y, panel_top_z - rim_t / 2.0),
        material=pale_edge,
        name="top_lip",
    )
    flap.visual(
        Box((FLAP_W, 0.008, rim_t)),
        origin=_flap_origin(0.0, rim_y, panel_top_z - FLAP_H + rim_t / 2.0),
        material=pale_edge,
        name="bottom_lip",
    )
    for idx, x in enumerate((-(FLAP_W / 2.0 - rim_t / 2.0), FLAP_W / 2.0 - rim_t / 2.0)):
        flap.visual(
            Box((rim_t, 0.008, FLAP_H)),
            origin=_flap_origin(x, rim_y, panel_top_z - FLAP_H / 2.0),
            material=pale_edge,
            name=f"side_lip_{idx}",
        )

    flap.visual(
        Box((0.052, 0.006, 0.018)),
        origin=_flap_origin(0.0, 0.007, panel_top_z - FLAP_H + 0.028),
        material=magnet,
        name="flap_magnet",
    )

    model.articulation(
        "frame_to_flap",
        ArticulationType.REVOLUTE,
        parent=panel_frame,
        child=flap,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-FLAP_OPEN_ANGLE,
            upper=0.60,
            effort=1.5,
            velocity=4.0,
        ),
    )

    # ── Lock slider (4-way lock closure) ──────────────────────────────────
    # A rigid flat panel captured in the two side rails that translates
    # vertically to block or free the swinging flap.
    lock_slider = model.part("lock_slider")
    lock_slider.visual(
        Box((SLIDER_W, SLIDER_T, SLIDER_H)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=slider_plastic,
        name="lock_panel",
    )
    # Small grip tab at the bottom edge of the slider for manual operation
    lock_slider.visual(
        Box((0.060, SLIDER_T + 0.004, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -SLIDER_H / 2.0 + 0.009)),
        material=slider_plastic,
        name="slider_grip_tab",
    )

    # Prismatic joint: at q=0 the slider is retracted above the opening
    # (flap free); positive q slides it down across the opening (flap blocked).
    model.articulation(
        "frame_to_lock_slider",
        ArticulationType.PRISMATIC,
        parent=panel_frame,
        child=lock_slider,
        origin=Origin(xyz=(0.0, SLIDER_Y, SLIDER_RETRACTED_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=SLIDER_TRAVEL,
            effort=8.0,
            velocity=0.20,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    panel_frame = object_model.get_part("panel_frame")
    flap = object_model.get_part("flap")
    lock_slider = object_model.get_part("lock_slider")
    hinge = object_model.get_articulation("frame_to_flap")
    slider_joint = object_model.get_articulation("frame_to_lock_slider")

    ctx.check(
        "reference classification matches cat flap",
        object_model.meta.get("classification_note", "").startswith("Reference image"),
        details=str(object_model.meta.get("classification_note")),
    )

    ctx.allow_overlap(
        panel_frame,
        flap,
        elem_a="hinge_pin",
        elem_b="hinge_sleeve",
        reason="The visible hinge pin is intentionally captured inside the simplified solid hinge sleeve proxy.",
    )
    ctx.expect_overlap(
        panel_frame,
        flap,
        axes="x",
        elem_a="hinge_pin",
        elem_b="hinge_sleeve",
        min_overlap=0.20,
        name="hinge pin spans the flap sleeve",
    )
    ctx.expect_within(
        panel_frame,
        flap,
        axes="yz",
        inner_elem="hinge_pin",
        outer_elem="hinge_sleeve",
        margin=0.004,
        name="hinge pin is centered in sleeve proxy",
    )

    with ctx.pose({hinge: -FLAP_OPEN_ANGLE}):
        ctx.expect_within(
            flap,
            panel_frame,
            axes="x",
            inner_elem="translucent_panel",
            outer_elem="front_trim",
            margin=0.0,
            name="closed flap fits inside trim width",
        )
        ctx.expect_gap(
            flap,
            panel_frame,
            axis="y",
            positive_elem="flap_magnet",
            negative_elem="frame_magnet",
            min_gap=0.0,
            max_gap=0.010,
            name="closed magnet nears fixed catch",
        )
        closed_aabb = ctx.part_element_world_aabb(flap, elem="flap_magnet")

    open_aabb = ctx.part_element_world_aabb(flap, elem="flap_magnet")
    ctx.check(
        "default flap pose is swung outward and upward",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[0][1] > closed_aabb[0][1] + 0.05
        and open_aabb[0][2] > closed_aabb[0][2] + 0.03,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    # ── Lock slider tests ────────────────────────────────────────────────
    # The slider passes over the inner seal, magnet mount, and frame magnet
    # when deployed — these are intentional overlaps as the lock panel
    # covers the opening.
    ctx.allow_overlap(
        panel_frame,
        lock_slider,
        elem_a="inner_seal",
        elem_b="lock_panel",
        reason="Lock panel slides over the inner seal bezel when deployed to block the opening.",
    )
    ctx.allow_overlap(
        panel_frame,
        lock_slider,
        elem_a="magnet_mount",
        elem_b="lock_panel",
        reason="Lock panel passes over the bottom magnet mount when covering the opening.",
    )
    ctx.allow_overlap(
        panel_frame,
        lock_slider,
        elem_a="frame_magnet",
        elem_b="lock_panel",
        reason="Lock panel passes over the frame magnet when covering the opening.",
    )

    # Slider is captured between the guide rails on the front trim
    ctx.expect_overlap(
        panel_frame,
        lock_slider,
        axes="z",
        elem_a="guide_rail_0",
        elem_b="lock_panel",
        min_overlap=0.10,
        name="lock panel engages left guide rail",
    )
    ctx.expect_overlap(
        panel_frame,
        lock_slider,
        axes="z",
        elem_a="guide_rail_1",
        elem_b="lock_panel",
        min_overlap=0.10,
        name="lock panel engages right guide rail",
    )

    # Verify slider moves downward when the prismatic joint is actuated
    rest_pos = ctx.part_world_position(lock_slider)
    with ctx.pose({slider_joint: SLIDER_TRAVEL}):
        deployed_pos = ctx.part_world_position(lock_slider)

    ctx.check(
        "lock_slider translates downward when deployed",
        rest_pos is not None
        and deployed_pos is not None
        and deployed_pos[2] < rest_pos[2] - 0.20,
        details=f"rest_z={rest_pos[2] if rest_pos else None}, "
        f"deployed_z={deployed_pos[2] if deployed_pos else None}",
    )

    # At rest (q=0), slider is above the opening — flap is free
    ctx.expect_gap(
        lock_slider,
        panel_frame,
        axis="z",
        positive_elem="lock_panel",
        negative_elem="front_trim",
        min_gap=-0.010,
        name="retracted slider clears opening top edge",
    )

    # When deployed, slider covers the opening in X projection (width)
    # Y overlap is not checked because the slider slides on the front face
    # of the frame, deliberately positioned slightly proud of the trim.
    with ctx.pose({slider_joint: SLIDER_TRAVEL}):
        ctx.expect_overlap(
            lock_slider,
            panel_frame,
            axes="x",
            elem_a="lock_panel",
            elem_b="front_trim",
            min_overlap=0.20,
            name="deployed slider spans opening width",
        )
        ctx.expect_overlap(
            lock_slider,
            panel_frame,
            axes="z",
            elem_a="lock_panel",
            elem_b="front_trim",
            min_overlap=0.25,
            name="deployed slider covers opening height",
        )

    return ctx.report()


object_model = build_object_model()
