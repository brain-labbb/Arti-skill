from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)

# ── Bag dimensions ──
BAG_W = 0.320        # X width
BAG_D = 0.180        # Y depth (front-to-back)
BAG_H = 0.260        # Z height (tall soft pouch)
WALL_T = 0.014       # fabric wall thickness
RIM_T = 0.020        # padded top rim
FLAP_DRAPE = 0.130   # how far the fold-over flap drapes down the front

# Flap display pose: open = folded back behind the bag
FLAP_OPEN_ANGLE = math.radians(150.0)

# Hinge at the top-rear seam, slightly behind the back panel outer face
HINGE_Y = BAG_D / 2 + WALL_T          # 0.104
FLAP_TOP_LEN = HINGE_Y + BAG_D / 2    # 0.194  (top-cover length in local frame)

# D-ring height on the side panels
DRING_Z = BAG_H * 0.82                # ~0.213


def _rounded_plate(
    width: float, depth: float, thickness: float, radius: float, name: str
):
    return mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(width, depth, radius, corner_segments=10),
            thickness,
            center=True,
        ),
        name,
    )


def _rounded_ring(
    outer_x: float,
    outer_y: float,
    inner_x: float,
    inner_y: float,
    thickness: float,
    outer_r: float,
    inner_r: float,
    name: str,
):
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(outer_x, outer_y, outer_r, corner_segments=10),
            [list(reversed(rounded_rect_profile(inner_x, inner_y, inner_r, corner_segments=10)))],
            thickness,
            center=True,
        ),
        name,
    )


def _flap_xyz(local_xyz: tuple[float, float, float]) -> tuple[float, float, float]:
    """Rotate a closed-flap local point into the displayed open-flap pose (X-axis)."""
    x, y, z = local_xyz
    c = math.cos(FLAP_OPEN_ANGLE)
    s = math.sin(FLAP_OPEN_ANGLE)
    return (x, y * c - z * s, y * s + z * c)


def _flap_origin(
    local_xyz: tuple[float, float, float],
    extra_rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> Origin:
    return Origin(
        xyz=_flap_xyz(local_xyz),
        rpy=(FLAP_OPEN_ANGLE + extra_rpy[0], extra_rpy[1], extra_rpy[2]),
    )


# ────────────────────────────────────────────────────────────────────────────
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="soft_aed_shoulder_bag",
        meta={
            "run_notes": (
                "Soft shoulder-bag AED pouch with fold-over top flap closure and "
                "carry strap anchored to side D-rings.  Black/red EMS colorway. "
                "Variant fork of the defibrillator carry-case family: primary form "
                "changed from rigid clamshell to soft sling bag."
            )
        },
    )

    # ── Materials ──
    case_red = model.material("case_red_fabric", rgba=(0.82, 0.035, 0.045, 1.0))
    black_fabric = model.material("black_fabric", rgba=(0.015, 0.014, 0.016, 1.0))
    dark_foam = model.material("dark_compartment_foam", rgba=(0.035, 0.038, 0.042, 1.0))
    gray_plastic = model.material("gray_plastic", rgba=(0.50, 0.53, 0.53, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.075, 0.085, 0.090, 1.0))
    screen_black = model.material("screen_black", rgba=(0.005, 0.007, 0.009, 1.0))
    screen_white = model.material("screen_white", rgba=(0.92, 0.95, 0.94, 1.0))
    print_red = model.material("simple_red_print", rgba=(0.90, 0.04, 0.035, 1.0))
    orange_button = model.material("orange_button", rgba=(1.0, 0.45, 0.10, 1.0))
    green_light = model.material("green_indicator", rgba=(0.08, 0.85, 0.34, 1.0))
    blue_connector = model.material("blue_connector", rgba=(0.02, 0.40, 0.75, 1.0))
    cable_white = model.material("white_cable", rgba=(0.94, 0.96, 0.96, 1.0))
    metal = model.material("dull_metal", rgba=(0.55, 0.57, 0.56, 1.0))
    strap_nylon = model.material("strap_nylon", rgba=(0.04, 0.04, 0.05, 1.0))

    # ════════════════════════════════════════════════════════════════════════
    # case_base – tall soft pouch body
    # ════════════════════════════════════════════════════════════════════════
    base = model.part("case_base")

    # ── bottom panel ──
    base.visual(
        _rounded_plate(BAG_W - 0.010, BAG_D - 0.010, WALL_T, 0.024, "red_bottom_skin"),
        origin=Origin(xyz=(0.0, 0.0, WALL_T / 2)),
        material=case_red,
        name="red_bottom_skin",
    )

    # ── outer walls ──
    base.visual(
        Box((BAG_W, WALL_T, BAG_H)),
        origin=Origin(xyz=(0.0, -BAG_D / 2, BAG_H / 2)),
        material=black_fabric,
        name="bag_front_panel",
    )
    base.visual(
        Box((BAG_W, WALL_T, BAG_H)),
        origin=Origin(xyz=(0.0, BAG_D / 2, BAG_H / 2)),
        material=black_fabric,
        name="bag_back_panel",
    )
    base.visual(
        Box((WALL_T, BAG_D - 2 * WALL_T, BAG_H)),
        origin=Origin(xyz=(-BAG_W / 2, 0.0, BAG_H / 2)),
        material=case_red,
        name="bag_left_panel",
    )
    base.visual(
        Box((WALL_T, BAG_D - 2 * WALL_T, BAG_H)),
        origin=Origin(xyz=(BAG_W / 2, 0.0, BAG_H / 2)),
        material=case_red,
        name="bag_right_panel",
    )

    # ── inner floor (foam pad) ──
    base.visual(
        _rounded_plate(
            BAG_W - 2 * WALL_T, BAG_D - 2 * WALL_T, 0.010, 0.016, "inner_floor"
        ),
        origin=Origin(xyz=(0.0, 0.0, WALL_T + 0.005)),
        material=dark_foam,
        name="inner_floor",
    )

    # ── inner liners ──
    inner_h = BAG_H - WALL_T - RIM_T
    liner_off = WALL_T
    for i, (sx, sy, w, d) in enumerate(
        [
            (0.0, -(BAG_D / 2 - liner_off), BAG_W - 4 * WALL_T, 0.008),
            (0.0, (BAG_D / 2 - liner_off), BAG_W - 4 * WALL_T, 0.008),
            (-(BAG_W / 2 - liner_off), 0.0, 0.008, BAG_D - 4 * WALL_T),
            ((BAG_W / 2 - liner_off), 0.0, 0.008, BAG_D - 4 * WALL_T),
        ]
    ):
        base.visual(
            Box((w, d, inner_h)),
            origin=Origin(xyz=(sx, sy, WALL_T + inner_h / 2)),
            material=black_fabric,
            name=f"black_inner_liner_{i}",
        )

    # ── padded top rim ──
    rim_z = BAG_H - RIM_T / 2
    for i, (sx, sy, w, d) in enumerate(
        [
            (0.0, -BAG_D / 2, BAG_W + WALL_T, RIM_T),
            (0.0, BAG_D / 2, BAG_W + WALL_T, RIM_T),
            (-BAG_W / 2, 0.0, RIM_T, BAG_D + WALL_T),
            (BAG_W / 2, 0.0, RIM_T, BAG_D + WALL_T),
        ]
    ):
        base.visual(
            Box((w, d, RIM_T)),
            origin=Origin(xyz=(sx, sy, rim_z)),
            material=black_fabric,
            name=f"padded_top_rim_{i}",
        )

    # ── red accent stripe on front ──
    base.visual(
        Box((BAG_W * 0.70, 0.004, 0.035)),
        origin=Origin(xyz=(0.0, -BAG_D / 2 - 0.002, BAG_H * 0.72)),
        material=case_red,
        name="red_front_stripe",
    )

    # ── front buckle receiver patch ──
    base.visual(
        Box((0.060, 0.008, 0.040)),
        origin=Origin(xyz=(0.0, -BAG_D / 2 - 0.004, BAG_H * 0.45)),
        material=black_fabric,
        name="front_latch_patch",
    )

    # ── side D-rings (strap anchor points) ──
    base.visual(
        Cylinder(radius=0.008, length=0.028),
        origin=Origin(
            xyz=(-BAG_W / 2 - 0.008, 0.0, DRING_Z),
            rpy=(0.0, math.pi / 2, 0.0),
        ),
        material=metal,
        name="side_d_ring_0",
    )
    base.visual(
        Cylinder(radius=0.008, length=0.028),
        origin=Origin(
            xyz=(BAG_W / 2 + 0.008, 0.0, DRING_Z),
            rpy=(0.0, math.pi / 2, 0.0),
        ),
        material=metal,
        name="side_d_ring_1",
    )

    # ── internal foam divider (above AED, spans inner depth for connectivity) ──
    base.visual(
        Box((BAG_W * 0.35, BAG_D - 2 * WALL_T, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.140)),
        material=dark_foam,
        name="foam_divider",
    )

    # ── top carry handle (padded grip anchored to side rims) ──
    base.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (-BAG_W / 2 + 0.005, 0.0, BAG_H - RIM_T / 2),
                    (-BAG_W / 4, 0.0, BAG_H + 0.020),
                    (0.0, 0.0, BAG_H + 0.035),
                    (BAG_W / 4, 0.0, BAG_H + 0.020),
                    (BAG_W / 2 - 0.005, 0.0, BAG_H - RIM_T / 2),
                ],
                radius=0.009,
                samples_per_segment=14,
                radial_segments=10,
                cap_ends=True,
            ),
            "top_handle",
        ),
        material=black_fabric,
        name="top_handle",
    )

    # ── shoulder strap (tube spline from D-ring to D-ring) ──
    base.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (-BAG_W / 2 - 0.005, 0.0, DRING_Z),
                    (-BAG_W / 3, 0.0, BAG_H + 0.100),
                    (-0.040, 0.0, BAG_H + 0.160),
                    (0.040, 0.0, BAG_H + 0.160),
                    (BAG_W / 3, 0.0, BAG_H + 0.100),
                    (BAG_W / 2 + 0.005, 0.0, DRING_Z),
                ],
                radius=0.011,
                samples_per_segment=16,
                radial_segments=10,
                cap_ends=True,
            ),
            "shoulder_strap",
        ),
        material=strap_nylon,
        name="shoulder_strap",
    )

    # ── shoulder pad on strap ──
    base.visual(
        Box((0.120, 0.038, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, BAG_H + 0.166)),
        material=black_fabric,
        name="strap_shoulder_pad",
    )

    # ── EMS cross patch on front (simple red print) ──
    base.visual(
        Box((0.005, 0.004, 0.050)),
        origin=Origin(xyz=(0.065, -BAG_D / 2 - 0.002, BAG_H * 0.72)),
        material=print_red,
        name="ems_cross_v",
    )
    base.visual(
        Box((0.050, 0.004, 0.005)),
        origin=Origin(xyz=(0.065, -BAG_D / 2 - 0.002, BAG_H * 0.72)),
        material=print_red,
        name="ems_cross_h",
    )

    # ── hinge barrel at top-rear seam (connects to lid hinge pin) ──
    base.visual(
        Cylinder(radius=0.010, length=0.060),
        origin=Origin(
            xyz=(0.0, HINGE_Y, BAG_H),
            rpy=(0.0, math.pi / 2, 0.0),
        ),
        material=dark_plastic,
        name="hinge_barrel",
    )

    # ════════════════════════════════════════════════════════════════════════
    # lid – large fold-over top flap
    # ════════════════════════════════════════════════════════════════════════
    lid = model.part("lid")

    # ── top cover section (red outer face) ──
    lid.visual(
        Box((BAG_W * 0.94, FLAP_TOP_LEN, 0.012)),
        origin=_flap_origin((0.0, -FLAP_TOP_LEN / 2, -0.006)),
        material=case_red,
        name="lid_flap_top",
    )

    # ── front drape section (red outer face) ──
    lid.visual(
        Box((BAG_W * 0.92, 0.012, FLAP_DRAPE)),
        origin=_flap_origin((0.0, -FLAP_TOP_LEN - 0.006, -FLAP_DRAPE / 2)),
        material=case_red,
        name="lid_flap_drape",
    )

    # ── inner lining – top ──
    lid.visual(
        Box((BAG_W * 0.88, FLAP_TOP_LEN - 0.020, 0.006)),
        origin=_flap_origin((0.0, -FLAP_TOP_LEN / 2, -0.016)),
        material=black_fabric,
        name="lid_inner_panel",
    )

    # ── inner lining – drape ──
    lid.visual(
        Box((BAG_W * 0.86, 0.006, FLAP_DRAPE - 0.020)),
        origin=_flap_origin((0.0, -FLAP_TOP_LEN + 0.003, -FLAP_DRAPE / 2)),
        material=black_fabric,
        name="lid_drape_lining",
    )

    # ── edge binding at bottom of drape ──
    lid.visual(
        Box((BAG_W * 0.94, 0.016, 0.014)),
        origin=_flap_origin((0.0, -FLAP_TOP_LEN - 0.008, -FLAP_DRAPE - 0.007)),
        material=dark_plastic,
        name="lid_edge_binding",
    )

    # ── buckle clip at free end of flap ──
    lid.visual(
        Box((0.044, 0.020, 0.030)),
        origin=_flap_origin((0.0, -FLAP_TOP_LEN - 0.014, -FLAP_DRAPE - 0.022)),
        material=dark_plastic,
        name="buckle_clip_tab",
    )

    # ── small pocket on inside of flap ──
    lid.visual(
        Box((0.100, 0.006, 0.060)),
        origin=_flap_origin((0.0, -FLAP_TOP_LEN / 2 + 0.010, -0.020)),
        material=dark_plastic,
        name="flap_inner_pocket",
    )

    # ── red accent stripe on flap outer ──
    lid.visual(
        Box((BAG_W * 0.60, 0.004, 0.025)),
        origin=_flap_origin((0.0, -FLAP_TOP_LEN / 2, 0.002)),
        material=print_red,
        name="flap_red_stripe",
    )

    # ── hinge pin at rotation center (always at hinge origin) ──
    lid.visual(
        Cylinder(radius=0.008, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        material=dark_plastic,
        name="hinge_pin",
    )

    # ════════════════════════════════════════════════════════════════════════
    # aed_device – compact AED unit (unchanged from parent)
    # ════════════════════════════════════════════════════════════════════════
    aed = model.part("aed_device")
    aed.visual(
        _rounded_plate(0.180, 0.135, 0.044, 0.018, "aed_body"),
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material=gray_plastic,
        name="aed_body",
    )
    aed.visual(
        _rounded_ring(
            0.174, 0.129, 0.150, 0.105, 0.010, 0.016, 0.012, "aed_black_top_bezel"
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.047)),
        material=dark_plastic,
        name="aed_black_top_bezel",
    )
    aed.visual(
        Box((0.088, 0.052, 0.006)),
        origin=Origin(xyz=(-0.030, -0.012, 0.054)),
        material=screen_black,
        name="display_bezel",
    )
    aed.visual(
        Box((0.067, 0.035, 0.007)),
        origin=Origin(xyz=(-0.030, -0.012, 0.058)),
        material=screen_white,
        name="display_screen",
    )
    aed.visual(
        Sphere(radius=0.007),
        origin=Origin(xyz=(0.042, 0.004, 0.058)),
        material=green_light,
        name="status_light",
    )
    aed.visual(
        Cylinder(radius=0.014, length=0.010),
        origin=Origin(xyz=(-0.057, 0.038, 0.054)),
        material=orange_button,
        name="shock_button",
    )
    aed.visual(
        Cylinder(radius=0.013, length=0.018),
        origin=Origin(xyz=(0.074, -0.006, 0.061), rpy=(0.0, math.pi / 2, 0.0)),
        material=blue_connector,
        name="electrode_socket",
    )
    aed.visual(
        Cylinder(radius=0.007, length=0.025),
        origin=Origin(xyz=(0.069, -0.006, 0.074)),
        material=blue_connector,
        name="plug_cap",
    )
    aed.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (0.074, -0.006, 0.063),
                    (0.062, -0.025, 0.080),
                    (0.025, -0.048, 0.088),
                    (-0.025, -0.038, 0.078),
                ],
                radius=0.0032,
                samples_per_segment=18,
                radial_segments=14,
                cap_ends=True,
            ),
            "white_electrode_cable",
        ),
        material=cable_white,
        name="white_electrode_cable",
    )

    # ════════════════════════════════════════════════════════════════════════
    # front_flap – small buckle tab on front face
    # ════════════════════════════════════════════════════════════════════════
    flap = model.part("front_flap")
    flap.visual(
        Box((0.044, 0.008, 0.055)),
        origin=Origin(xyz=(0.0, -0.008, -0.028)),
        material=black_fabric,
        name="soft_flap_body",
    )
    flap.visual(
        Cylinder(radius=0.005, length=0.004),
        origin=Origin(xyz=(0.0, -0.014, -0.045)),
        material=metal,
        name="velcro_face",
    )
    flap.visual(
        Box((0.054, 0.006, 0.014)),
        origin=Origin(xyz=(0.0, -0.010, 0.002)),
        material=black_fabric,
        name="sewn_hinge_band",
    )

    # ════════════════════════════════════════════════════════════════════════
    # Articulations
    # ════════════════════════════════════════════════════════════════════════

    # base_to_lid – fold-over flap hinge at top-rear seam
    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, BAG_H)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=-FLAP_OPEN_ANGLE,
            upper=math.radians(10.0),
        ),
    )

    # base_to_aed – fixed mount inside bag (AED sits on foam floor)
    model.articulation(
        "base_to_aed",
        ArticulationType.FIXED,
        parent=base,
        child=aed,
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
    )

    # base_to_front_flap – buckle tab on front face
    model.articulation(
        "base_to_front_flap",
        ArticulationType.REVOLUTE,
        parent=base,
        child=flap,
        origin=Origin(xyz=(0.0, -BAG_D / 2, BAG_H * 0.45)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=1.6,
            lower=-math.radians(25.0),
            upper=math.radians(5.0),
        ),
    )

    return model


# ────────────────────────────────────────────────────────────────────────────
def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("case_base")
    lid = object_model.get_part("lid")
    aed = object_model.get_part("aed_device")
    flap = object_model.get_part("front_flap")
    hinge = object_model.get_articulation("base_to_lid")

    # ── intentional hinge seam: hinge pin nested inside barrel ──
    ctx.allow_overlap(
        lid,
        base,
        elem_a="hinge_pin",
        elem_b="hinge_barrel",
        reason=(
            "The fold-over flap hinge pin is intentionally nested inside the "
            "hinge barrel at the top-rear seam; this is the physical flap pivot."
        ),
    )
    ctx.expect_contact(
        lid,
        base,
        elem_a="hinge_pin",
        elem_b="hinge_barrel",
        name="hinge pin contacts barrel at the flap seam",
    )

    # ── hinge barrel seats in the flap seam ──
    ctx.allow_overlap(
        base,
        lid,
        elem_a="hinge_barrel",
        elem_b="lid_flap_top",
        reason=(
            "The hinge barrel is embedded in the flap top-cover seam where the "
            "fold-over flap pivots at the bag rear top edge."
        ),
    )
    ctx.expect_overlap(
        base,
        lid,
        axes="x",
        elem_a="hinge_barrel",
        elem_b="lid_flap_top",
        min_overlap=0.030,
        name="hinge barrel overlaps flap top cover along hinge axis",
    )

    # ── buckle tab seats against the bag front panel ──
    ctx.allow_overlap(
        flap,
        base,
        elem_a="soft_flap_body",
        elem_b="bag_front_panel",
        reason=(
            "The buckle tab body sits flush against the bag front panel as a "
            "surface-mounted trim piece with minimal local compression."
        ),
    )
    ctx.expect_contact(
        flap,
        base,
        elem_a="soft_flap_body",
        elem_b="bag_front_panel",
        name="buckle tab body contacts the bag front panel",
    )

    # ── AED fits inside the pouch ──
    ctx.expect_within(
        aed,
        base,
        axes="xy",
        margin=0.005,
        inner_elem="aed_body",
        outer_elem="inner_floor",
        name="AED device footprint is inside the bag floor",
    )
    ctx.expect_gap(
        aed,
        base,
        axis="z",
        positive_elem="aed_body",
        negative_elem="inner_floor",
        min_gap=-0.002,
        max_gap=0.012,
        name="AED body rests on the padded floor",
    )

    # ── case_base is a tall soft pouch, not a flat box ──
    base_aabb = ctx.part_world_aabb(base)
    base_height = (base_aabb[1][2] - base_aabb[0][2]) if base_aabb else 0.0
    ctx.check(
        "case_base is a tall soft pouch (height > 0.18 m)",
        base_height > 0.18,
        details=f"base_height={base_height:.3f} m",
    )

    # ── default pose: flap is open (folded back behind bag) ──
    open_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid fold-over flap default pose is visibly open (folded back)",
        open_aabb is not None and open_aabb[0][1] > 0.0,
        details=f"open_flap_min_y={open_aabb[0][1] if open_aabb else None}",
    )

    # ── closed pose: flap covers the bag top ──
    with ctx.pose({hinge: -FLAP_OPEN_ANGLE}):
        closed_aabb = ctx.part_world_aabb(lid)
        ctx.expect_overlap(
            lid,
            base,
            axes="xy",
            min_overlap=0.10,
            name="closed flap covers the bag top footprint",
        )

    # ── base_to_lid hinge actually moves the flap ──
    ctx.check(
        "base_to_lid fold-over flap moves between open and closed poses",
        open_aabb is not None
        and closed_aabb is not None
        and abs(open_aabb[0][1] - closed_aabb[0][1]) > 0.05,
        details=(
            f"open_min_y={open_aabb[0][1] if open_aabb else None}, "
            f"closed_min_y={closed_aabb[0][1] if closed_aabb else None}"
        ),
    )

    # ── shoulder strap is present ──
    strap_aabb = ctx.part_element_world_aabb(base, elem="shoulder_strap")
    ctx.check(
        "shoulder strap is present on case_base",
        strap_aabb is not None
        and (strap_aabb[1][2] - strap_aabb[0][2]) > 0.10,
        details=f"strap_aabb={strap_aabb}",
    )

    # ── front_flap buckle tab part is present ──
    ctx.check(
        "front_flap buckle tab part is present",
        flap.name == "front_flap",
        details="front_flap should remain a separate hinged buckle tab",
    )

    return ctx.report()


object_model = build_object_model()
