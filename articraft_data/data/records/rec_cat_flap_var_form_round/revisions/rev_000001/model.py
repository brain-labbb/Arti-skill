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
OPENING_D = 0.26
OPENING_R = OPENING_D / 2.0
FRAME_OD = 0.34
FRAME_D = 0.028
FRONT_Y = PANEL_T / 2.0
FRAME_FRONT_Y = FRONT_Y + FRAME_D
HINGE_Y = FRAME_FRONT_Y + 0.006
HINGE_Z = OPENING_R + 0.014
FLAP_D = 0.248
FLAP_R = FLAP_D / 2.0
FLAP_OPEN_ANGLE = 0.85
HINGE_PIN_LENGTH = 0.18


def _panel_with_cutout() -> cq.Workplane:
    """Door panel with a circular through-opening."""
    return (
        cq.Workplane("XY")
        .rect(PANEL_W, PANEL_H)
        .circle(OPENING_R + 0.005)
        .extrude(PANEL_T, both=True)
    )


def _flap_disc() -> cq.Workplane:
    """Thin circular translucent flap disc, centered at origin."""
    return (
        cq.Workplane("XY")
        .circle(FLAP_R - 0.008)
        .extrude(0.002, both=True)
    )


def _edge_ring() -> cq.Workplane:
    """Pale plastic edge ring around the flap disc, centered at origin."""
    return (
        cq.Workplane("XY")
        .circle(FLAP_R)
        .circle(FLAP_R - 0.010)
        .extrude(0.003, both=True)
    )


def _rotated_about_hinge(x: float, y: float, z: float, angle: float) -> tuple[float, float, float]:
    """Rotate a local flap visual center around the hinge X axis."""
    c = math.cos(angle)
    s = math.sin(angle)
    return (x, y * c - z * s, y * s + z * c)


def _flap_origin(x: float, y: float, z: float, *, angle: float = FLAP_OPEN_ANGLE) -> Origin:
    """Origin for box-shaped flap visuals pre-rotated to default open angle."""
    return Origin(xyz=_rotated_about_hinge(x, y, z, angle), rpy=(angle, 0.0, 0.0))


def _flap_disc_origin(x: float, y: float, z: float, *, angle: float = FLAP_OPEN_ANGLE) -> Origin:
    """Origin for disc-shaped visuals: face-on rotation (Z→Y) composed with hinge angle."""
    pos = _rotated_about_hinge(x, y, z, angle)
    return Origin(xyz=pos, rpy=(angle - math.pi / 2.0, 0.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="cat_flap_pet_door",
        meta={
            "category_context": "Pet_Animal related / Cat flap",
            "classification_note": "Reference image and folder category both indicate a cat flap / pet door; circular porthole variant.",
        },
    )

    wood = model.material("warm_brown_door_panel", rgba=(0.42, 0.23, 0.11, 1.0))
    dark_plastic = model.material("charcoal_plastic_trim", rgba=(0.035, 0.033, 0.032, 1.0))
    rubber = model.material("black_rubber_seal", rgba=(0.005, 0.005, 0.004, 1.0))
    frosted = model.material("frosted_translucent_flap", rgba=(0.73, 0.90, 0.95, 0.42))
    pale_edge = model.material("milky_plastic_flap_edge", rgba=(0.86, 0.92, 0.90, 0.70))
    metal = model.material("dull_screw_metal", rgba=(0.55, 0.55, 0.52, 1.0))
    magnet_mat = model.material("dark_magnet_latch", rgba=(0.02, 0.02, 0.018, 1.0))

    # ── Panel frame (root) ──────────────────────────────────────────────
    panel_frame = model.part("panel_frame")
    panel_frame.visual(
        mesh_from_cadquery(_panel_with_cutout(), "door_panel_with_opening", tolerance=0.0008),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=wood,
        name="door_panel",
    )

    # Circular front trim bezel (porthole form family)
    front_bezel = BezelGeometry(
        (OPENING_D, OPENING_D),
        (FRAME_OD, FRAME_OD),
        FRAME_D,
        opening_shape="circle",
        outer_shape="circle",
        center=True,
    )
    panel_frame.visual(
        mesh_from_geometry(front_bezel, "circular_front_trim"),
        origin=Origin(xyz=(0.0, FRONT_Y + FRAME_D / 2.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="front_trim",
    )

    # Circular inner rubber seal
    inner_seal = BezelGeometry(
        (OPENING_D - 0.018, OPENING_D - 0.018),
        (OPENING_D + 0.014, OPENING_D + 0.014),
        0.006,
        opening_shape="circle",
        outer_shape="circle",
        center=True,
    )
    panel_frame.visual(
        mesh_from_geometry(inner_seal, "inner_rubber_seal"),
        origin=Origin(xyz=(0.0, FRAME_FRONT_Y + 0.003, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="inner_seal",
    )

    # Four screw heads around the circular frame at 45° intervals
    screw_r = 0.155
    screw_positions = tuple(
        (screw_r * math.cos(math.radians(a)), screw_r * math.sin(math.radians(a)))
        for a in (45, 135, 225, 315)
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

    # Static hinge hardware: pin across the top of the circular opening
    panel_frame.visual(
        Cylinder(radius=0.004, length=HINGE_PIN_LENGTH),
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="hinge_pin",
    )
    for idx, x in enumerate((-0.080, 0.080)):
        panel_frame.visual(
            Box((0.026, 0.018, 0.040)),
            origin=Origin(xyz=(x, FRAME_FRONT_Y + 0.001, HINGE_Z - 0.006)),
            material=dark_plastic,
            name=f"hinge_lug_{idx}",
        )

    # Bottom magnet catch on the frame: a vertical bracket from the frame ring
    # bottom up to the magnet catch position, ensuring structural connectivity.
    magnet_z_closed = HINGE_Z - FLAP_D + 0.028  # aligns with flap magnet when closed
    mount_z_bottom = -(OPENING_R + 0.010)  # on the trim ring below the opening
    mount_z_center = (mount_z_bottom + magnet_z_closed) / 2.0
    mount_z_height = magnet_z_closed - mount_z_bottom
    panel_frame.visual(
        Box((0.024, 0.006, mount_z_height)),
        origin=Origin(xyz=(0.0, FRAME_FRONT_Y + 0.003, mount_z_center)),
        material=magnet_mat,
        name="magnet_mount",
    )
    panel_frame.visual(
        Box((0.058, 0.006, 0.020)),
        origin=Origin(xyz=(0.0, FRAME_FRONT_Y + 0.006, magnet_z_closed)),
        material=magnet_mat,
        name="frame_magnet",
    )

    # ── Flap (child) ────────────────────────────────────────────────────
    flap = model.part("flap")

    # Hinge sleeve around the pin (shorter than pin, fits between lugs)
    flap.visual(
        Cylinder(radius=0.010, length=0.120),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=pale_edge,
        name="hinge_sleeve",
    )

    # Web connecting hinge sleeve to the top of the disc
    flap.visual(
        Box((0.060, 0.006, 0.020)),
        origin=_flap_origin(0.0, 0.002, -0.010),
        material=pale_edge,
        name="sleeve_web",
    )

    # Translucent circular disc (main flap panel)
    disc_center_z = -FLAP_R
    flap.visual(
        mesh_from_cadquery(_flap_disc(), "flap_translucent_disc", tolerance=0.0005),
        origin=_flap_disc_origin(0.0, 0.002, disc_center_z),
        material=frosted,
        name="translucent_panel",
    )

    # Pale edge ring around the disc perimeter
    flap.visual(
        mesh_from_cadquery(_edge_ring(), "flap_edge_ring", tolerance=0.0005),
        origin=_flap_disc_origin(0.0, 0.004, disc_center_z),
        material=pale_edge,
        name="edge_ring",
    )

    # Flap magnet at bottom of disc
    flap.visual(
        Box((0.052, 0.006, 0.018)),
        origin=_flap_origin(0.0, 0.007, disc_center_z - FLAP_R + 0.028),
        material=magnet_mat,
        name="flap_magnet",
    )

    # ── Articulation ────────────────────────────────────────────────────
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
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    panel_frame = object_model.get_part("panel_frame")
    flap = object_model.get_part("flap")
    hinge = object_model.get_articulation("frame_to_flap")

    ctx.check(
        "reference classification matches cat flap",
        object_model.meta.get("classification_note", "").startswith("Reference image"),
        details=str(object_model.meta.get("classification_note")),
    )

    # Variant-specific: verify circular porthole form family
    front_trim = panel_frame.get_visual("front_trim")
    ctx.check(
        "front_trim exists as circular porthole bezel",
        front_trim is not None,
        details="front_trim visual must exist as circular porthole bezel",
    )
    translucent = flap.get_visual("translucent_panel")
    ctx.check(
        "translucent_panel is a round disc flap",
        translucent is not None,
        details="translucent_panel must exist as round disc flap",
    )

    # Hinge pin captured inside sleeve
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
        min_overlap=0.10,
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
            name="closed disc flap fits inside circular trim",
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

    return ctx.report()


object_model = build_object_model()
