from __future__ import annotations

import math

import cadquery as cq

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
    mesh_from_cadquery,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


OPEN_LID_ANGLE = math.radians(103.0)

# Hard-shell case dimensions
CASE_W = 0.360
CASE_D = 0.280
BASE_H = 0.082
LID_H = 0.045
WALL_T = 0.005
CORNER_R = 0.018


def _rounded_plate(width: float, depth: float, thickness: float, radius: float, name: str):
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


def _lid_xyz(local_xyz: tuple[float, float, float]) -> tuple[float, float, float]:
    """Position a closed-lid local point in the displayed open-lid pose."""
    x, y, z = local_xyz
    c = math.cos(OPEN_LID_ANGLE)
    s = math.sin(OPEN_LID_ANGLE)
    return (x * c + z * s, y, -x * s + z * c)


def _lid_origin(
    local_xyz: tuple[float, float, float],
    extra_rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> Origin:
    return Origin(
        xyz=_lid_xyz(local_xyz),
        rpy=(extra_rpy[0], OPEN_LID_ANGLE + extra_rpy[1], extra_rpy[2]),
    )


def _build_lower_shell() -> object:
    """CadQuery rigid tray lower shell with rounded corners."""
    w, d, h = CASE_W, CASE_D, BASE_H
    r = CORNER_R
    t = WALL_T

    outer = (
        cq.Workplane("XY")
        .box(w, d, h, centered=(True, True, False))
        .edges("|Z")
        .fillet(r)
    )
    # Interior cavity (open top tray)
    iw = w - 2 * t
    id_ = d - 2 * t
    ih = h - t
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=t)
        .box(iw, id_, ih, centered=(True, True, False))
        .edges("|Z")
        .fillet(r - t)
    )
    return outer.cut(cavity)


def _build_lid_shell() -> object:
    """CadQuery rigid domed lid shell (open at bottom)."""
    w = CASE_W - 0.008
    d = CASE_D - 0.008
    h = LID_H
    r = CORNER_R - 0.004
    t = WALL_T

    outer = (
        cq.Workplane("XY")
        .box(w, d, h, centered=(True, True, False))
        .edges("|Z")
        .fillet(r)
    )
    # Cavity open at bottom, leaving top dome intact
    iw = w - 2 * t
    id_ = d - 2 * t
    ih = h - t + 0.001
    cavity = (
        cq.Workplane("XY")
        .workplane(offset=-0.001)
        .box(iw, id_, ih, centered=(True, True, False))
        .edges("|Z")
        .fillet(max(0.002, r - t))
    )
    return outer.cut(cavity)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="rigid_hard_shell_defibrillator_case",
        meta={
            "run_notes": (
                "Rigid hard-shell clamshell AED transport case (Pelican-style) "
                "with charcoal molded lower shell, ribbed dome lid, and two "
                "front over-center snap latches in safety orange. "
                "Fork variant of the soft fabric carry case - same AED inside."
            )
        },
    )

    # ── materials ──────────────────────────────────────────────────────────
    charcoal_shell = model.material("charcoal_shell", rgba=(0.18, 0.19, 0.20, 1.0))
    dark_liner = model.material("dark_liner", rgba=(0.06, 0.065, 0.07, 1.0))
    dark_foam = model.material("dark_compartment_foam", rgba=(0.035, 0.038, 0.042, 1.0))
    gray_plastic = model.material("gray_plastic", rgba=(0.50, 0.53, 0.53, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.075, 0.085, 0.090, 1.0))
    screen_black = model.material("screen_black", rgba=(0.005, 0.007, 0.009, 1.0))
    screen_white = model.material("screen_white", rgba=(0.92, 0.95, 0.94, 1.0))
    white_pack = model.material("white_accessory_pack", rgba=(0.92, 0.93, 0.91, 1.0))
    print_red = model.material("simple_red_print", rgba=(0.90, 0.04, 0.035, 1.0))
    orange_latch = model.material("safety_orange_latch", rgba=(1.0, 0.42, 0.0, 1.0))
    orange_accent = model.material("orange_accent", rgba=(1.0, 0.35, 0.0, 1.0))
    metal = model.material("dull_metal", rgba=(0.55, 0.57, 0.56, 1.0))
    green_light = model.material("green_indicator", rgba=(0.08, 0.85, 0.34, 1.0))
    blue_connector = model.material("blue_connector", rgba=(0.02, 0.40, 0.75, 1.0))
    cable_white = model.material("white_cable", rgba=(0.94, 0.96, 0.96, 1.0))
    orange_button = model.material("orange_button", rgba=(1.0, 0.45, 0.10, 1.0))
    pink_key = model.material("pink_safety_key", rgba=(0.96, 0.42, 0.63, 1.0))

    # =====================================================================
    # CASE BASE — rigid lower shell
    # =====================================================================
    base = model.part("case_base")

    # Main molded shell body (CadQuery)
    base.visual(
        mesh_from_cadquery(_build_lower_shell(), "base_shell_body"),
        material=charcoal_shell,
        name="base_shell_body",
    )

    # Inner floor foam pad (keep name from parent)
    base.visual(
        _rounded_plate(0.300, 0.220, 0.010, 0.014, "inner_floor"),
        origin=Origin(xyz=(0.0, 0.0, WALL_T + 0.005)),
        material=dark_foam,
        name="inner_floor",
    )

    # External reinforcement ribs — sides (Y faces)
    rib_zs = [0.018, 0.038, 0.058]
    for i, rz in enumerate(rib_zs):
        base.visual(
            Box((CASE_W - 0.020, 0.003, 0.005)),
            origin=Origin(xyz=(0.0, -(CASE_D / 2 + 0.0015), rz)),
            material=charcoal_shell,
            name=f"rib_side_left_{i}",
        )
        base.visual(
            Box((CASE_W - 0.020, 0.003, 0.005)),
            origin=Origin(xyz=(0.0, +(CASE_D / 2 + 0.0015), rz)),
            material=charcoal_shell,
            name=f"rib_side_right_{i}",
        )

    # External reinforcement ribs — front face (-X)
    for i, rz in enumerate(rib_zs):
        base.visual(
            Box((0.003, CASE_D - 0.020, 0.005)),
            origin=Origin(xyz=(-(CASE_W / 2 + 0.0015), 0.0, rz)),
            material=charcoal_shell,
            name=f"rib_front_{i}",
        )

    # Rear hinge barrel on base
    base.visual(
        Cylinder(radius=0.006, length=0.230),
        origin=Origin(
            xyz=(CASE_W / 2 - 0.004, 0.0, BASE_H),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=metal,
        name="rear_hinge_barrel_base",
    )

    # Front latch mounting bosses (charcoal pads on front face)
    for i, ly in enumerate((-0.060, 0.060)):
        base.visual(
            Box((0.010, 0.038, 0.032)),
            origin=Origin(xyz=(-(CASE_W / 2 + 0.002), ly, 0.066)),
            material=charcoal_shell,
            name=f"latch_boss_{i}",
        )

    # Orange accent stripe along the parting-line rim (proud of outer face)
    base.visual(
        Box((CASE_W - 0.060, 0.004, 0.004)),
        origin=Origin(xyz=(0.0, -(CASE_D / 2 + 0.001), BASE_H - 0.004)),
        material=orange_accent,
        name="accent_stripe_left",
    )
    base.visual(
        Box((CASE_W - 0.060, 0.004, 0.004)),
        origin=Origin(xyz=(0.0, +(CASE_D / 2 + 0.001), BASE_H - 0.004)),
        material=orange_accent,
        name="accent_stripe_right",
    )

    # Foam divider (keep functional relationship from parent)
    base.visual(
        Box((0.190, 0.010, 0.032)),
        origin=Origin(xyz=(-0.030, -0.052, WALL_T + 0.026)),
        material=dark_liner,
        name="foam_divider",
    )

    # Electrode pack leaning against inner wall
    base.visual(
        Box((0.090, 0.007, 0.080)),
        origin=Origin(xyz=(0.030, -0.105, 0.058), rpy=(0.0, -0.35, 0.0)),
        material=white_pack,
        name="electrode_pack",
    )
    base.visual(
        Box((0.074, 0.008, 0.055)),
        origin=Origin(xyz=(0.026, -0.110, 0.060), rpy=(0.0, -0.35, 0.0)),
        material=screen_white,
        name="electrode_pack_label",
    )
    base.visual(
        Box((0.055, 0.006, 0.012)),
        origin=Origin(xyz=(0.018, -0.115, 0.078), rpy=(0.0, -0.35, 0.0)),
        material=print_red,
        name="simple_pack_mark",
    )

    # =====================================================================
    # LID — rigid domed ribbed shell
    # =====================================================================
    lid = model.part("lid")
    lid_w = CASE_W - 0.008
    lid_d = CASE_D - 0.008

    # Main dome shell (CadQuery)
    lid.visual(
        mesh_from_cadquery(_build_lid_shell(), "lid_shell_body"),
        origin=_lid_origin((-lid_w / 2, 0.0, 0.0)),
        material=charcoal_shell,
        name="lid_shell_body",
    )

    # Inner liner panel
    lid.visual(
        _rounded_plate(lid_w - 0.012, lid_d - 0.012, 0.003, 0.010, "lid_inner_liner"),
        origin=_lid_origin((-lid_w / 2, 0.0, WALL_T + 0.0015)),
        material=dark_liner,
        name="lid_inner_liner",
    )

    # External ribs on lid top (running along X)
    lid_rib_ys = [-0.080, -0.040, 0.0, 0.040, 0.080]
    for i, ry in enumerate(lid_rib_ys):
        lid.visual(
            Box((lid_w - 0.040, 0.005, 0.003)),
            origin=_lid_origin((-lid_w / 2, ry, LID_H + 0.0015)),
            material=charcoal_shell,
            name=f"lid_rib_{i}",
        )

    # Hinge barrel on lid side
    lid.visual(
        Cylinder(radius=0.005, length=0.210),
        origin=_lid_origin((0.0, 0.0, 0.0), extra_rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="lid_hinge_barrel",
    )

    # Orange accent stripe on lid edges (proud of outer face)
    lid.visual(
        Box((lid_w - 0.040, 0.004, 0.003)),
        origin=_lid_origin((-lid_w / 2, -(lid_d / 2 + 0.001), 0.004)),
        material=orange_accent,
        name="lid_accent_front",
    )
    lid.visual(
        Box((lid_w - 0.040, 0.004, 0.003)),
        origin=_lid_origin((-lid_w / 2, +(lid_d / 2 + 0.001), 0.004)),
        material=orange_accent,
        name="lid_accent_rear",
    )

    # Instruction strip (keep from parent)
    lid.visual(
        Box((0.128, 0.052, 0.004)),
        origin=_lid_origin((-0.214, -0.052, 0.005)),
        material=screen_white,
        name="instruction_strip",
    )

    # Red icon panels (keep from parent)
    for index, lx in enumerate((-0.252, -0.214, -0.176)):
        lid.visual(
            Box((0.030, 0.042, 0.005)),
            origin=_lid_origin((lx, -0.052, 0.0025)),
            material=print_red,
            name=f"red_icon_panel_{index}",
        )

    # Small inner pouch + pink pull key (keep from parent)
    lid.visual(
        Box((0.076, 0.056, 0.005)),
        origin=_lid_origin((-0.205, 0.080, 0.005)),
        material=dark_plastic,
        name="small_inner_pouch",
    )
    lid.visual(
        Box((0.041, 0.026, 0.004)),
        origin=_lid_origin((-0.208, 0.073, 0.001)),
        material=screen_black,
        name="pouch_label_patch",
    )
    lid.visual(
        Cylinder(radius=0.007, length=0.030),
        origin=_lid_origin(
            (-0.194, 0.118, 0.003), extra_rpy=(math.pi / 2.0, 0.0, 0.0)
        ),
        material=pink_key,
        name="pink_pull_key",
    )
    lid.visual(
        Sphere(radius=0.010),
        origin=_lid_origin((-0.194, 0.101, 0.003)),
        material=pink_key,
        name="pink_key_tab",
    )

    # =====================================================================
    # SNAP LATCHES (over-center front latches)
    # =====================================================================
    latch_ys = (-0.060, 0.060)
    for i, ly in enumerate(latch_ys):
        latch = model.part(f"snap_latch_{i}")
        # Lever body (orange) — extends upward from pivot in locked pose
        latch.visual(
            Box((0.008, 0.028, 0.026)),
            origin=Origin(xyz=(-0.004, 0.0, 0.013)),
            material=orange_latch,
            name=f"latch_lever_{i}",
        )
        # Metal hook tab at top of lever
        latch.visual(
            Box((0.012, 0.022, 0.005)),
            origin=Origin(xyz=(-0.006, 0.0, 0.028)),
            material=metal,
            name=f"latch_hook_{i}",
        )
        # Pivot pin
        latch.visual(
            Cylinder(radius=0.003, length=0.032),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=metal,
            name=f"latch_pin_{i}",
        )

    # =====================================================================
    # AED DEVICE (preserved from parent)
    # =====================================================================
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
        origin=Origin(
            xyz=(0.074, -0.006, 0.061), rpy=(0.0, math.pi / 2.0, 0.0)
        ),
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
                    (0.070, -0.030, 0.087),
                    (0.012, -0.090, 0.100),
                    (-0.086, -0.112, 0.070),
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

    # =====================================================================
    # ARTICULATIONS
    # =====================================================================
    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(CASE_W / 2 - 0.002, 0.0, BASE_H)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=2.0,
            lower=-OPEN_LID_ANGLE,
            upper=math.radians(12.0),
        ),
    )
    model.articulation(
        "base_to_aed",
        ArticulationType.FIXED,
        parent=base,
        child=aed,
        origin=Origin(xyz=(-0.032, 0.035, WALL_T + 0.010)),
    )

    # Latch joints — revolute, flip outward from front face
    for i, ly in enumerate(latch_ys):
        model.articulation(
            f"base_to_latch_{i}",
            ArticulationType.REVOLUTE,
            parent=base,
            child=model.get_part(f"snap_latch_{i}"),
            origin=Origin(xyz=(-(CASE_W / 2 + 0.004), ly, 0.066)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=4.0,
                lower=0.0,
                upper=math.radians(130.0),
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("case_base")
    lid = object_model.get_part("lid")
    aed = object_model.get_part("aed_device")
    hinge = object_model.get_articulation("base_to_lid")
    latch_0 = object_model.get_part("snap_latch_0")
    latch_1 = object_model.get_part("snap_latch_1")
    latch_joint_0 = object_model.get_articulation("base_to_latch_0")

    # ── Intentional overlaps (hinge barrels + captured latch pins) ─────
    ctx.allow_overlap(
        base,
        lid,
        elem_a="rear_hinge_barrel_base",
        elem_b="lid_hinge_barrel",
        reason="Hinge barrels interleave at the clamshell pivot as in a real case hinge.",
    )
    ctx.allow_overlap(
        base,
        lid,
        elem_a="base_shell_body",
        elem_b="lid_hinge_barrel",
        reason="Lid hinge barrel passes through the rear shell wall at the pivot point.",
    )
    ctx.allow_overlap(
        base,
        latch_0,
        elem_a="latch_boss_0",
        elem_b="latch_pin_0",
        reason="Latch pivot pin is captured inside the mounting boss on the front face.",
    )
    ctx.allow_overlap(
        base,
        latch_1,
        elem_a="latch_boss_1",
        elem_b="latch_pin_1",
        reason="Latch pivot pin is captured inside the mounting boss on the front face.",
    )

    # Proof: hinge barrels stay aligned along the hinge axis
    ctx.expect_contact(
        base,
        lid,
        elem_a="rear_hinge_barrel_base",
        elem_b="lid_hinge_barrel",
        contact_tol=0.010,
        name="hinge barrels remain co-located at the pivot",
    )

    # ── AED containment (preserved from parent) ───────────────────────
    ctx.expect_within(
        aed,
        base,
        axes="xy",
        margin=0.004,
        inner_elem="aed_body",
        outer_elem="base_shell_body",
        name="AED device footprint is inside the rigid shell",
    )
    ctx.expect_gap(
        aed,
        base,
        axis="z",
        positive_elem="aed_body",
        negative_elem="inner_floor",
        min_gap=-0.001,
        max_gap=0.008,
        name="AED body rests on the inner floor pad",
    )

    # ── Rigid shell visual claim (new for this axis) ──────────────────
    ctx.check(
        "case_base has rigid shell_body visual",
        any(v.name == "base_shell_body" for v in base.visuals),
        details="rigid hard-shell lower shell must be present as a named visual",
    )
    ctx.check(
        "lid has rigid shell_body visual",
        any(v.name == "lid_shell_body" for v in lid.visuals),
        details="rigid ribbed dome lid must be present as a named visual",
    )

    # ── Snap latch presence and articulation ──────────────────────────
    ctx.check(
        "snap_latch_0 is a separate articulated part",
        latch_0 is not None and latch_joint_0 is not None,
        details="over-center snap latch must be a distinct revolute-jointed part",
    )
    ctx.check(
        "snap_latch_1 exists",
        latch_1 is not None,
        details="second snap latch must exist",
    )

    # ── Lid open / closed pose checks ────────────────────────────────
    open_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid default pose is visibly open",
        open_aabb is not None and open_aabb[1][2] > 0.20,
        details=f"open_lid_aabb={open_aabb}",
    )

    with ctx.pose({hinge: -OPEN_LID_ANGLE}):
        closed_aabb = ctx.part_world_aabb(lid)
        ctx.expect_overlap(
            lid,
            base,
            axes="xy",
            min_overlap=0.15,
            name="closed lid covers the base footprint",
        )

    ctx.check(
        "hinged lid closes downward from the displayed open pose",
        open_aabb is not None
        and closed_aabb is not None
        and closed_aabb[1][2] < open_aabb[1][2] - 0.08,
        details=f"open={open_aabb}, closed={closed_aabb}",
    )

    # ── Latch flip check ─────────────────────────────────────────────
    latch_rest = ctx.part_world_aabb(latch_0)
    with ctx.pose({latch_joint_0: math.radians(100.0)}):
        latch_flipped = ctx.part_world_aabb(latch_0)
    ctx.check(
        "snap latch flips outward when released",
        latch_rest is not None
        and latch_flipped is not None
        and latch_flipped[0][0] < latch_rest[0][0] - 0.005,
        details=f"rest={latch_rest}, flipped={latch_flipped}",
    )

    return ctx.report()


object_model = build_object_model()
