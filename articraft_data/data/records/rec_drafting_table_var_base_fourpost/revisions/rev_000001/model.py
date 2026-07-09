from __future__ import annotations

import math

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


def _beam_orientation(p0: tuple[float, float, float], p1: tuple[float, float, float]) -> tuple[Origin, float]:
    """Return an origin that aligns a local +Z beam from p0 to p1."""
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        raise ValueError("beam endpoints must be distinct")
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.sqrt(dx * dx + dy * dy), dz)
    origin = Origin(
        xyz=((p0[0] + p1[0]) * 0.5, (p0[1] + p1[1]) * 0.5, (p0[2] + p1[2]) * 0.5),
        rpy=(0.0, pitch, yaw),
    )
    return origin, length


def _rect_beam(part, p0, p1, size: float, material, name: str, overshoot: float = 0.018) -> None:
    origin, length = _beam_orientation(p0, p1)
    part.visual(
        Box((size, size, length + overshoot)),
        origin=origin,
        material=material,
        name=name,
    )


def _round_beam(part, p0, p1, radius: float, material, name: str, overshoot: float = 0.012) -> None:
    origin, length = _beam_orientation(p0, p1)
    part.visual(
        Cylinder(radius=radius, length=length + overshoot),
        origin=origin,
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="tilting_drafting_table",
        meta={"reference_category": "Workspace / Drafting table"},
    )

    black_metal = model.material("powder_coated_black_metal", rgba=(0.015, 0.017, 0.016, 1.0))
    worn_edges = model.material("slightly_worn_dark_edges", rgba=(0.05, 0.055, 0.052, 1.0))
    wood = model.material("warm_reddish_drafting_wood", rgba=(0.46, 0.20, 0.075, 1.0))
    wood_dark = model.material("darker_wood_grain", rgba=(0.20, 0.085, 0.035, 1.0))
    wood_mid = model.material("burnished_wood_grain", rgba=(0.34, 0.14, 0.050, 1.0))
    wood_highlight = model.material("subtle_amber_grain", rgba=(0.58, 0.25, 0.090, 1.0))
    grey = model.material("gunmetal_hardware", rgba=(0.19, 0.20, 0.19, 1.0))
    plastic = model.material("black_plastic_knobs", rgba=(0.01, 0.011, 0.012, 1.0))
    pale_panel = model.material("pale_drawer_faces", rgba=(0.78, 0.76, 0.68, 1.0))

    frame = model.part("frame")

    # Real-world scale: institutional drafting desk, about 1.25 m wide and 0.8 m deep.
    # Four straight vertical corner posts.
    leg_size = 0.040
    leg_bottom = 0.02
    leg_top = 0.72
    corners = [
        (-0.55, -0.38),   # 0: front-left
        ( 0.55, -0.38),   # 1: front-right
        (-0.55,  0.38),   # 2: rear-left
        ( 0.55,  0.38),   # 3: rear-right
    ]
    for i, (cx, cy) in enumerate(corners):
        _rect_beam(frame, (cx, cy, leg_bottom), (cx, cy, leg_top), leg_size, black_metal, f"leg_{i}")

    # Rectangular apron/stretcher frame just under the board line.
    tube = 0.035
    apron_z = 0.69
    _rect_beam(frame, (-0.55, -0.38, apron_z), (0.55, -0.38, apron_z), tube, black_metal, "front_apron")
    _rect_beam(frame, (-0.55,  0.38, apron_z), (0.55,  0.38, apron_z), tube, black_metal, "rear_apron")
    _rect_beam(frame, (-0.55, -0.38, apron_z), (-0.55, 0.38, apron_z), tube, black_metal, "left_apron")
    _rect_beam(frame, ( 0.55, -0.38, apron_z), ( 0.55, 0.38, apron_z), tube, black_metal, "right_apron")

    # Mid-height side stretchers carry the ratchet/lock-knob hardware below the apron.
    side_stretcher_z = 0.44
    for idx, sx in enumerate((-1.0, 1.0)):
        x = sx * 0.55
        _rect_beam(frame, (x, -0.38, side_stretcher_z), (x, 0.38, side_stretcher_z), 0.028, black_metal, f"side_stretcher_{idx}")

    for idx, sx in enumerate((-1.0, 1.0)):
        # Ratchet/angle plates on both sides: a toothed strip under the tilting top.
        _rect_beam(frame, (sx * 0.525, -0.20, 0.635), (sx * 0.525, 0.24, 0.845), 0.020, grey, f"ratchet_backbone_{idx}")
        for tooth in range(6):
            y = -0.15 + tooth * 0.065
            z = 0.66 + tooth * 0.032
            frame.visual(
                Box((0.026, 0.035, 0.020)),
                origin=Origin(xyz=(sx * 0.525, y, z), rpy=(0.0, 0.0, 0.0)),
                material=grey,
                name=f"ratchet_tooth_{idx}_{tooth}",
            )

        # Side bushings that receive the lock-knob shafts.
        frame.visual(
            Cylinder(radius=0.030, length=0.034),
            origin=Origin(xyz=(sx * 0.552, -0.175, 0.455), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=grey,
            name=f"knob_socket_{idx}",
        )

    # Front hinge rail mounted atop the front apron uprights.
    _rect_beam(frame, (-0.58, -0.365, 0.722), (0.58, -0.365, 0.722), tube, black_metal, "front_hinge_rail")
    # Rear top rail between rear legs caps the rear apron line.
    _rect_beam(frame, (-0.55, 0.38, 0.72), (0.55, 0.38, 0.72), tube, black_metal, "rear_top_rail")
    # Low front crossbar ties the two front legs at mid-height for lateral rigidity.
    _rect_beam(frame, (-0.55, -0.38, 0.30), (0.55, -0.38, 0.30), 0.028, black_metal, "low_crossbar")
    # Tray support rails hung from the front apron.
    _rect_beam(frame, (-0.55, -0.385, 0.655), (0.55, -0.385, 0.655), 0.030, black_metal, "tray_front_rail")
    _rect_beam(frame, (-0.55, -0.205, 0.655), (0.55, -0.205, 0.655), 0.030, black_metal, "tray_rear_rail")

    # Front supply tray/cubby shelf mounted to the frame, visible below the hinged board.
    frame.visual(
        Box((1.08, 0.18, 0.020)),
        origin=Origin(xyz=(0.0, -0.295, 0.645)),
        material=black_metal,
        name="front_supply_tray",
    )
    for x in (-0.30, 0.0, 0.30):
        frame.visual(
            Box((0.018, 0.18, 0.065)),
            origin=Origin(xyz=(x, -0.295, 0.685)),
            material=black_metal,
            name=f"tray_divider_{int((x + 0.30) / 0.30)}",
        )
    for x in (-0.42, -0.14, 0.14, 0.42):
        frame.visual(
            Box((0.22, 0.012, 0.040)),
            origin=Origin(xyz=(x, -0.392, 0.682)),
            material=pale_panel,
            name=f"drawer_face_{int((x + 0.42) / 0.28)}",
        )

    # Hinge pin represented as a captured steel rod through the table barrel.
    frame.visual(
        Cylinder(radius=0.012, length=1.18),
        origin=Origin(xyz=(0.0, -0.365, 0.755), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grey,
        name="hinge_pin",
    )
    for idx, sx in enumerate((-1.0, 1.0)):
        frame.visual(
            Box((0.030, 0.050, 0.055)),
            origin=Origin(xyz=(sx * 0.585, -0.365, 0.745)),
            material=grey,
            name=f"hinge_end_bracket_{idx}",
        )
        # Vertical bracket from side apron down toward the knob socket height.
        frame.visual(
            Box((0.020, 0.040, 0.200)),
            origin=Origin(xyz=(sx * 0.552, -0.175, 0.590)),
            material=grey,
            name=f"knob_mount_tab_{idx}",
        )

    tabletop = model.part("tabletop")
    tabletop.visual(
        Box((1.20, 0.76, 0.040)),
        origin=Origin(xyz=(0.0, 0.380, 0.035)),
        material=wood,
        name="wood_drawing_board",
    )
    # Dark side and rear rails form the black raised perimeter in the reference.
    tabletop.visual(
        Box((1.26, 0.045, 0.060)),
        origin=Origin(xyz=(0.0, 0.015, 0.070)),
        material=worn_edges,
        name="front_lip",
    )
    tabletop.visual(
        Box((1.24, 0.038, 0.050)),
        origin=Origin(xyz=(0.0, 0.765, 0.065)),
        material=worn_edges,
        name="rear_lip",
    )
    for idx, sx in enumerate((-1.0, 1.0)):
        tabletop.visual(
            Box((0.045, 0.78, 0.055)),
            origin=Origin(xyz=(sx * 0.625, 0.390, 0.065)),
            material=worn_edges,
            name=f"side_lip_{idx}",
        )
        tabletop.visual(
            Box((0.030, 0.52, 0.020)),
            origin=Origin(xyz=(sx * 0.555, 0.395, 0.065)),
            material=black_metal,
            name=f"paper_channel_{idx}",
        )

    # A front paper stop/parallel rule bar mounted on the board face.
    tabletop.visual(
        Box((1.04, 0.022, 0.020)),
        origin=Origin(xyz=(0.0, 0.105, 0.065)),
        material=black_metal,
        name="paper_stop",
    )
    tabletop.visual(
        Cylinder(radius=0.021, length=1.12),
        origin=Origin(xyz=(0.0, -0.006, 0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grey,
        name="tilt_barrel",
    )

    # Fine in-board wood grain follows the photographed reddish desktop without crossing the lips.
    for i, (x, y, length, width_y, material) in enumerate(
        (
            (-0.12, 0.155, 0.78, 0.006, wood_mid),
            (-0.06, 0.195, 0.84, 0.004, wood_highlight),
            (-0.18, 0.240, 0.72, 0.005, wood_dark),
            (0.02, 0.285, 0.86, 0.004, wood_mid),
            (-0.08, 0.330, 0.80, 0.005, wood_highlight),
            (0.10, 0.375, 0.70, 0.004, wood_dark),
            (-0.04, 0.420, 0.82, 0.005, wood_mid),
            (0.05, 0.465, 0.76, 0.004, wood_highlight),
            (-0.16, 0.510, 0.68, 0.005, wood_dark),
            (0.00, 0.555, 0.84, 0.004, wood_mid),
            (-0.09, 0.605, 0.74, 0.005, wood_highlight),
            (0.12, 0.650, 0.62, 0.004, wood_dark),
        )
    ):
        tabletop.visual(
            Box((length, width_y, 0.0018)),
            origin=Origin(xyz=(x, y, 0.056), rpy=(0.0, 0.0, 0.025 * (-1 if i % 2 else 1))),
            material=material,
            name=f"wood_grain_{i}",
        )

    tilt_origin = Origin(xyz=(0.0, -0.365, 0.755), rpy=(math.radians(20.0), 0.0, 0.0))
    model.articulation(
        "frame_to_tabletop",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=tabletop,
        origin=tilt_origin,
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.45, lower=-0.30, upper=0.55),
    )

    # Two user-adjustable lock knobs on the sides of the frame.  The child part
    # frame is the knob spindle center, so spinning the joint rotates the visible disk.
    for idx, sx in enumerate((-1.0, 1.0)):
        knob = model.part(f"lock_knob_{idx}")
        knob.visual(
            Cylinder(radius=0.055, length=0.026),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=plastic,
            name="knob_disk",
        )
        knob.visual(
            Cylinder(radius=0.014, length=0.090),
            origin=Origin(xyz=(-sx * 0.045, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=grey,
            name="knob_shaft",
        )
        for k, angle in enumerate((0.0, math.pi / 3.0, 2.0 * math.pi / 3.0)):
            knob.visual(
                Box((0.012, 0.080, 0.010)),
                origin=Origin(xyz=(sx * 0.026, 0.0, 0.004), rpy=(0.0, math.pi / 2.0, angle)),
                material=grey,
                name=f"grip_rib_{k}",
            )
        model.articulation(
            f"frame_to_lock_knob_{idx}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=knob,
            origin=Origin(xyz=(sx * 0.600, -0.175, 0.455)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=-math.pi, upper=math.pi),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    tabletop = object_model.get_part("tabletop")
    tilt = object_model.get_articulation("frame_to_tabletop")

    # The simplified hinge is a real captured-pin relationship: the moving
    # barrel intentionally surrounds the fixed pin, and this is the only broad
    # frame/tabletop interpenetration expected at the current tilted pose.
    ctx.allow_overlap(
        frame,
        tabletop,
        elem_a="hinge_pin",
        elem_b="tilt_barrel",
        reason="The tabletop hinge barrel is intentionally represented around the frame hinge pin.",
    )
    ctx.expect_overlap(
        frame,
        tabletop,
        axes="x",
        elem_a="hinge_pin",
        elem_b="tilt_barrel",
        min_overlap=1.05,
        name="hinge barrel spans the fixed pin",
    )
    ctx.expect_overlap(
        frame,
        tabletop,
        axes="y",
        elem_a="hinge_pin",
        elem_b="tilt_barrel",
        min_overlap=0.020,
        name="hinge barrel is coaxial with pin",
    )

    for idx in (0, 1):
        knob = object_model.get_part(f"lock_knob_{idx}")
        joint = object_model.get_articulation(f"frame_to_lock_knob_{idx}")
        ctx.allow_overlap(
            frame,
            knob,
            elem_a=f"knob_socket_{idx}",
            elem_b="knob_shaft",
            reason="The lock-knob shaft intentionally seats in the side bushing.",
        )
        ctx.allow_overlap(
            frame,
            knob,
            elem_a=f"side_stretcher_{idx}",
            elem_b="knob_shaft",
            reason="The knob shaft passes through the side stretcher zone near the socket; the stretcher provides lateral support for the lock mechanism.",
        )
        ctx.expect_overlap(
            frame,
            knob,
            axes="x",
            elem_a=f"knob_socket_{idx}",
            elem_b="knob_shaft",
            min_overlap=0.012,
            name=f"lock knob {idx} shaft enters side socket",
        )
        ctx.expect_contact(
            frame,
            knob,
            elem_a=f"side_stretcher_{idx}",
            elem_b="knob_shaft",
            contact_tol=0.020,
            name=f"lock knob {idx} shaft passes near side stretcher",
        )
        before = ctx.part_world_position(knob)
        with ctx.pose({joint: 1.2}):
            after = ctx.part_world_position(knob)
        ctx.check(
            f"lock knob {idx} spins in place",
            before is not None and after is not None and max(abs(before[i] - after[i]) for i in range(3)) < 1e-6,
            details=f"before={before}, after={after}",
        )

    rest_aabb = ctx.part_world_aabb(tabletop)
    with ctx.pose({tilt: 0.45}):
        raised_aabb = ctx.part_world_aabb(tabletop)
    ctx.check(
        "positive tabletop tilt raises rear edge",
        rest_aabb is not None and raised_aabb is not None and raised_aabb[1][2] > rest_aabb[1][2] + 0.12,
        details=f"rest={rest_aabb}, raised={raised_aabb}",
    )
    ctx.expect_overlap(
        frame,
        tabletop,
        axes="x",
        elem_b="wood_drawing_board",
        min_overlap=1.0,
        name="wide table top is carried by the frame width",
    )

    # --- Variant-specific: four vertical corner posts + rectangular apron ---
    def _has_visual(part_obj, vname: str) -> bool:
        try:
            part_obj.get_visual(vname)
            return True
        except Exception:
            return False

    # Confirm the four straight vertical legs exist on the frame.
    for i in range(4):
        ctx.check(
            f"vertical leg_{i} present",
            _has_visual(frame, f"leg_{i}"),
            details=f"leg_{i} visual missing from four-post frame",
        )
    # Confirm the rectangular perimeter apron beams exist.
    for apron_name in ("front_apron", "rear_apron", "left_apron", "right_apron"):
        ctx.check(
            f"apron beam {apron_name} present",
            _has_visual(frame, apron_name),
            details=f"{apron_name} beam missing from box-apron frame",
        )
    # Confirm the old splayed-leg topology is gone.
    ctx.check(
        "no splayed front legs remain",
        not _has_visual(frame, "front_leg_0"),
        details="splayed front_leg_0 should be replaced by vertical corner posts",
    )
    ctx.check(
        "no diagonal side braces remain",
        not _has_visual(frame, "side_brace_0"),
        details="diagonal side_brace_0 should be dropped in the four-post variant",
    )

    return ctx.report()


object_model = build_object_model()
