from __future__ import annotations

"""Decorative Japanese katana display set.

A black lacquered two-tier sword stand on a rectangular base box with a
gold-framed kanji plaque. Two sheathed katanas rest horizontally in the
crescent cradles of the two pillars and a third katana rests across the base
box top. Each katana's blade assembly (blade + habaki + tsuba + handle) slides
out of its fixed white scabbard (saya) along the saya's long axis via an
independent prismatic joint, travel 0 .. 0.70 m.

World frame: +X is the sword long axis (handles toward +X), +Z up, the kanji
plaque faces -Y (front). Ground plane at z = 0.
"""

from math import pi, sin, cos

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

# ---------------------------------------------------------------- constants
SAYA_LEN = 0.73  # scabbard length along X (mouth at local x=0, tail at -0.73)
SAYA_R = 0.019  # scabbard outer radius
BORE_R = 0.0145  # scabbard inner bore radius
BORE_DEPTH = SAYA_LEN - 0.005  # closed tail end keeps a 5 mm cap
TRAVEL = 0.70  # prismatic draw travel

BLADE_ROOT_X = -0.005  # blade loft root (near the mouth) in blade-local frame
BLADE_TIP_X = -0.72  # blade tip deep inside the saya at q = 0

GRIP_X0, GRIP_X1 = 0.015, 0.2555  # smooth samegawa grip span
KASHIRA_X0, KASHIRA_X1 = 0.2545, 0.2665  # dark pommel cap span

PILLAR_T = 0.022  # pillar plank thickness along X
PILLAR_X = 0.09  # pillar center offset from stand center
NOTCH_R = 0.021  # cradle crescent radius
CRADLE_Y = -0.0375  # cradle seat center, forward of the pillar column
SEAT_DROP = 0.0025  # saya center sits this far below the notch center

BOX_TOP_Z = 0.12  # base box overall height

# saya mouth (= prismatic joint frame) position in the stand frame
SWORD_MOUNTS = {
    "top": (0.28, CRADLE_Y, 0.46 - SEAT_DROP),
    "middle": (0.28, CRADLE_Y, 0.32 - SEAT_DROP),
    "bottom": (0.17, -0.06, BOX_TOP_Z + SAYA_R - 0.0005),
}


# ---------------------------------------------------------------- cq shapes
def _saya_tube_shape() -> cq.Workplane:
    """Hollow scabbard tube: open mouth at x = 0, closed tail cap at -SAYA_LEN."""
    outer = cq.Workplane("YZ").circle(SAYA_R).extrude(-SAYA_LEN)
    bore = cq.Workplane("YZ").circle(BORE_R).extrude(-BORE_DEPTH)
    return outer.cut(bore)


def _ring_shape(r_out: float, r_in: float, length: float) -> cq.Workplane:
    """Annular collar spanning local x in [0, length], axis along +X."""
    outer = cq.Workplane("YZ").circle(r_out).extrude(length)
    inner = cq.Workplane("YZ").circle(r_in).extrude(length)
    return outer.cut(inner)


def _blade_shape() -> cq.Workplane:
    """Tapered katana blade lofted along -X (root near mouth, tip deep inside)."""
    return (
        cq.Workplane("YZ", origin=(BLADE_ROOT_X, 0.0, 0.0))
        .rect(0.0060, 0.0260)
        .workplane(offset=-0.600)
        .rect(0.0050, 0.0220)
        .workplane(offset=-0.115)
        .rect(0.0018, 0.0070)
        .loft(ruled=True)
    )


def _pillar_shape() -> cq.Workplane:
    """Stand pillar plank with two forward cradle arms and upward crescents.

    Profile lives in the stand YZ frame (z absolute, plank extruded 0..PILLAR_T
    along +X).
    """
    shape = cq.Workplane("YZ").center(0.015, 0.289).rect(0.050, 0.342).extrude(PILLAR_T)
    for z_center in (0.29, 0.43):
        arm = (
            cq.Workplane("YZ").center(-0.0125, z_center).rect(0.105, 0.060).extrude(PILLAR_T)
        )
        shape = shape.union(arm)
    for z_top in (0.32, 0.46):
        notch = (
            cq.Workplane("YZ").center(CRADLE_Y, z_top).circle(NOTCH_R).extrude(PILLAR_T)
        )
        shape = shape.cut(notch)
    return shape


# ---------------------------------------------------------------- model
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="katana_display_set")

    model.material("black_lacquer", rgba=(0.06, 0.05, 0.06, 1.0))
    model.material("gold", rgba=(0.80, 0.64, 0.25, 1.0))
    model.material("plaque_white", rgba=(0.93, 0.92, 0.88, 1.0))
    model.material("ink", rgba=(0.10, 0.09, 0.10, 1.0))
    model.material("saya_white", rgba=(0.94, 0.93, 0.94, 1.0))
    model.material("blossom_pink", rgba=(0.95, 0.65, 0.76, 1.0))
    model.material("deep_pink", rgba=(0.89, 0.45, 0.62, 1.0))
    model.material("samegawa", rgba=(0.91, 0.88, 0.80, 1.0))  # smooth lacquered ray skin
    model.material("tsuba_red", rgba=(0.40, 0.10, 0.12, 1.0))
    model.material("dark_fitting", rgba=(0.13, 0.11, 0.13, 1.0))
    model.material("steel", rgba=(0.83, 0.85, 0.88, 1.0))

    # ------------------------------------------------------------- stand
    stand = model.part("display_stand")
    stand.visual(
        Box((0.31, 0.19, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material="black_lacquer",
        name="base_plinth",
    )
    stand.visual(
        Box((0.30, 0.18, 0.100)),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material="black_lacquer",
        name="base_body",
    )
    stand.visual(
        Box((0.31, 0.19, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.113)),
        material="black_lacquer",
        name="base_cap",
    )

    pillar_mesh = mesh_from_cadquery(_pillar_shape(), "stand_pillar")
    for index, px in enumerate((-PILLAR_X, PILLAR_X)):
        stand.visual(
            pillar_mesh,
            origin=Origin(xyz=(px - PILLAR_T / 2.0, 0.0, 0.0)),
            material="black_lacquer",
            name=f"pillar_{index}",
        )

    # gold-framed white kanji plaque on the front (-Y) face of the base box
    stand.visual(
        Box((0.088, 0.005, 0.058)),
        origin=Origin(xyz=(0.0, -0.0905, 0.065)),
        material="plaque_white",
        name="plaque_panel",
    )
    frame_bars = (
        ("frame_top", Box((0.105, 0.007, 0.009)), (0.0, -0.0925, 0.098)),
        ("frame_bottom", Box((0.105, 0.007, 0.009)), (0.0, -0.0925, 0.032)),
        ("frame_left", Box((0.009, 0.007, 0.075)), (-0.048, -0.0925, 0.065)),
        ("frame_right", Box((0.009, 0.007, 0.075)), (0.048, -0.0925, 0.065)),
    )
    for bar_name, bar_geom, bar_xyz in frame_bars:
        stand.visual(bar_geom, origin=Origin(xyz=bar_xyz), material="gold", name=bar_name)
    # stylized dark kanji strokes on the white panel
    kanji_strokes = (
        (Box((0.034, 0.002, 0.0045)), (0.0, -0.0935, 0.082), (0.0, 0.0, 0.0)),
        (Box((0.046, 0.002, 0.0045)), (0.0, -0.0935, 0.068), (0.0, 0.0, 0.0)),
        (Box((0.030, 0.002, 0.0045)), (-0.012, -0.0935, 0.050), (0.0, 0.9, 0.0)),
        (Box((0.030, 0.002, 0.0045)), (0.012, -0.0935, 0.050), (0.0, -0.9, 0.0)),
    )
    for index, (glyph_geom, glyph_xyz, glyph_rpy) in enumerate(kanji_strokes):
        stand.visual(
            glyph_geom,
            origin=Origin(xyz=glyph_xyz, rpy=glyph_rpy),
            material="ink",
            name=f"kanji_stroke_{index}",
        )

    # ------------------------------------------------------------- shared sword meshes
    saya_mesh = mesh_from_cadquery(_saya_tube_shape(), "saya_tube")
    blade_mesh = mesh_from_cadquery(_blade_shape(), "katana_blade")
    band_len = 0.022
    band_mesh = mesh_from_cadquery(_ring_shape(0.0198, 0.0146, band_len), "saya_band")
    mouth_len = 0.012
    mouth_mesh = mesh_from_cadquery(_ring_shape(0.0205, 0.0146, mouth_len), "saya_mouth_ring")
    rim_len = 0.009
    rim_mesh = mesh_from_cadquery(_ring_shape(0.0435, 0.0375, rim_len), "tsuba_rim")

    # blossom decoration spots in saya-local coordinates: (x, azimuth, big?)
    blossom_spots = ((-0.55, 0.30, True), (-0.36, -0.15, True), (-0.47, 0.65, True))
    dot_spots = ((-0.52, -0.35), (-0.40, 0.45), (-0.61, 0.10))

    def _build_saya(part_name: str) -> object:
        saya = model.part(part_name)
        saya.visual(saya_mesh, material="saya_white", name="saya_tube")
        saya.visual(
            mouth_mesh,
            origin=Origin(xyz=(-mouth_len, 0.0, 0.0)),
            material="deep_pink",
            name="mouth_ring",
        )
        for index, band_x in enumerate((-0.665, -0.30)):
            saya.visual(
                band_mesh,
                origin=Origin(xyz=(band_x - band_len / 2.0, 0.0, 0.0)),
                material="deep_pink",
                name=f"band_{index}",
            )
        for index, (bx, az, _big) in enumerate(blossom_spots):
            radial = 0.019
            saya.visual(
                Cylinder(radius=0.0075, length=0.003),
                origin=Origin(xyz=(bx, -radial * sin(az), radial * cos(az)), rpy=(az, 0.0, 0.0)),
                material="blossom_pink",
                name=f"blossom_{index}",
            )
            saya.visual(
                Cylinder(radius=0.0027, length=0.0036),
                origin=Origin(
                    xyz=(bx, -0.0195 * sin(az), 0.0195 * cos(az)), rpy=(az, 0.0, 0.0)
                ),
                material="gold",
                name=f"blossom_center_{index}",
            )
        for index, (dx, az) in enumerate(dot_spots):
            saya.visual(
                Cylinder(radius=0.004, length=0.003),
                origin=Origin(xyz=(dx, -0.019 * sin(az), 0.019 * cos(az)), rpy=(az, 0.0, 0.0)),
                material="blossom_pink",
                name=f"petal_dot_{index}",
            )
        return saya

    def _build_blade_assembly(part_name: str) -> object:
        blade = model.part(part_name)
        blade.visual(blade_mesh, material="steel", name="blade_body")
        # habaki collar: bridges the blade root through the mouth into the tsuba
        blade.visual(
            Box((0.0305, 0.010, 0.024)),
            origin=Origin(xyz=(-0.01475, 0.0, 0.0)),
            material="gold",
            name="habaki_collar",
        )
        blade.visual(
            Cylinder(radius=0.041, length=0.007),
            origin=Origin(xyz=(0.0035, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material="tsuba_red",
            name="tsuba_disc",
        )
        blade.visual(
            rim_mesh,
            origin=Origin(xyz=(-0.001, 0.0, 0.0)),
            material="gold",
            name="tsuba_rim",
        )
        blade.visual(
            Cylinder(radius=0.0160, length=0.0105),
            origin=Origin(xyz=(0.01175, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material="dark_fitting",
            name="fuchi_collar",
        )
        grip_len = GRIP_X1 - GRIP_X0
        blade.visual(
            Cylinder(radius=0.0150, length=grip_len),
            origin=Origin(xyz=((GRIP_X0 + GRIP_X1) / 2.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material="samegawa",
            name="tsuka_grip",
        )
        blade.visual(
            Cylinder(radius=0.0158, length=KASHIRA_X1 - KASHIRA_X0),
            origin=Origin(
                xyz=((KASHIRA_X0 + KASHIRA_X1) / 2.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)
            ),
            material="dark_fitting",
            name="kashira_pommel",
        )
        return blade

    # ------------------------------------------------------------- swords
    for prefix, mount_xyz in SWORD_MOUNTS.items():
        saya = _build_saya(f"{prefix}_saya")
        blade = _build_blade_assembly(f"{prefix}_blade_assembly")
        model.articulation(
            f"{prefix}_saya_mount",
            ArticulationType.FIXED,
            parent=stand,
            child=saya,
            origin=Origin(xyz=mount_xyz),
        )
        model.articulation(
            f"{prefix}_blade_draw",
            ArticulationType.PRISMATIC,
            parent=saya,
            child=blade,
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=25.0, velocity=0.6, lower=0.0, upper=TRAVEL),
        )

    return model


# ---------------------------------------------------------------- tests
def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("display_stand")

    # ---- stand geometry: real-world base box scale and grounding
    stand_aabb = ctx.part_world_aabb(stand)
    assert stand_aabb is not None
    assert -1e-6 <= stand_aabb[0][2] <= 0.002, "stand must rest on the ground plane"
    plinth = ctx.part_element_world_aabb(stand, elem="base_plinth")
    assert plinth is not None
    assert 0.28 <= plinth[1][0] - plinth[0][0] <= 0.33, "base box ~0.30 m wide"
    assert 0.16 <= plinth[1][1] - plinth[0][1] <= 0.21, "base box ~0.18 m deep"
    cap = ctx.part_element_world_aabb(stand, elem="base_cap")
    assert cap is not None
    assert 0.11 <= cap[1][2] <= 0.13, "base box ~0.12 m tall"
    pillar = ctx.part_element_world_aabb(stand, elem="pillar_0")
    assert pillar is not None
    assert pillar[1][2] > 0.40, "pillars rise to the upper cradle tier"
    plaque = ctx.part_element_world_aabb(stand, elem="plaque_panel")
    assert plaque is not None
    assert plaque[0][1] < -0.090, "kanji plaque is proud of the front face"

    swords = {}
    for prefix in ("top", "middle", "bottom"):
        saya = object_model.get_part(f"{prefix}_saya")
        blade = object_model.get_part(f"{prefix}_blade_assembly")
        draw = object_model.get_articulation(f"{prefix}_blade_draw")
        swords[prefix] = (saya, blade, draw)

        # joint plan: independent prismatic draw along the saya long axis
        assert draw.articulation_type == ArticulationType.PRISMATIC
        assert draw.axis == (1.0, 0.0, 0.0)
        limits = draw.motion_limits
        assert limits is not None and limits.lower == 0.0
        assert limits.upper is not None and abs(limits.upper - TRAVEL) < 1e-9

        # saya rests in its cradle / on the box top: scoped seating allowances
        if prefix == "bottom":
            ctx.allow_overlap(
                saya,
                stand,
                elem_a="saya_tube",
                elem_b="base_cap",
                reason="bottom scabbard rests on the base box top with a 0.5 mm seat",
            )
        else:
            for pillar_elem in ("pillar_0", "pillar_1"):
                ctx.allow_overlap(
                    saya,
                    stand,
                    elem_a="saya_tube",
                    elem_b=pillar_elem,
                    reason="scabbard seats 0.5 mm into the crescent cradle notch",
                )
        ctx.expect_contact(saya, stand, name=f"{prefix}_saya seated on the stand")

        # sheathed insertion: the blade rides inside the hollow saya bore by design
        ctx.allow_overlap(
            blade,
            saya,
            elem_a="blade_body",
            elem_b="saya_tube",
            reason="sheathed blade nests inside the hollow saya bore (seated insertion)",
        )

        # sheathed pose: blade hidden inside the hollow saya, tsuba at the mouth
        ctx.expect_origin_distance(
            blade, saya, axes="x", min_dist=0.0, max_dist=1e-6,
            name=f"{prefix} blade fully sheathed at q=0",
        )
        ctx.expect_within(
            blade, saya, axes="yz", inner_elem="blade_body", outer_elem="saya_tube",
            name=f"{prefix} blade nests inside the saya bore",
        )
        ctx.expect_within(
            blade, saya, axes="x", inner_elem="blade_body", outer_elem="saya_tube",
            margin=0.001, name=f"{prefix} blade hidden along the saya length",
        )

        # draw pose: blade translates along +X and keeps retained insertion
        with ctx.pose({draw: TRAVEL}):
            ctx.expect_origin_gap(
                blade, saya, axis="x", min_gap=TRAVEL - 0.001, max_gap=TRAVEL + 0.001,
                name=f"{prefix} draw moves the blade {TRAVEL} m along +X",
            )
            ctx.expect_overlap(
                blade, saya, axes="x", elem_a="blade_body", elem_b="saya_tube",
                min_overlap=0.010, name=f"{prefix} blade stays engaged at full draw",
            )
        with ctx.pose({draw: 0.35}):
            ctx.expect_origin_gap(
                blade, saya, axis="x", min_gap=0.349, max_gap=0.351,
                name=f"{prefix} mid draw translates 0.35 m",
            )

        # hero sword features: ~1.0 m sheathed katana, smooth samegawa grip, tsuba, pommel
        saya_aabb = ctx.part_world_aabb(saya)
        blade_aabb = ctx.part_world_aabb(blade)
        assert saya_aabb is not None and blade_aabb is not None
        total_len = max(saya_aabb[1][0], blade_aabb[1][0]) - min(
            saya_aabb[0][0], blade_aabb[0][0]
        )
        assert 0.95 <= total_len <= 1.05, f"{prefix} katana ~1.0 m long, got {total_len:.3f}"
        grip = ctx.part_element_world_aabb(blade, elem="tsuka_grip")
        assert grip is not None
        assert 0.22 <= grip[1][0] - grip[0][0] <= 0.27, "tsuka ~0.25 m long"
        tsuba = ctx.part_element_world_aabb(blade, elem="tsuba_disc")
        assert tsuba is not None
        tsuba_dia = tsuba[1][2] - tsuba[0][2]
        assert 0.07 <= tsuba_dia <= 0.09, "ornate tsuba disc ~0.082 m diameter"
        assert tsuba_dia > 2.2 * SAYA_R, "tsuba guard is wider than the saya (off-axis disc)"
        pommel = ctx.part_element_world_aabb(blade, elem="kashira_pommel")
        assert pommel is not None
        assert pommel[0][0] >= grip[1][0] - 0.002, "dark pommel caps the handle end"
        # smooth lacquered samegawa grip: no diamond wrap accents
        blade_visual_names = {v.name for v in blade.visuals}
        assert "wrap_diamond_top_0" not in blade_visual_names, (
            f"{prefix} tsuka must be smooth samegawa with no diamond wrap"
        )
        assert "wrap_diamond_front_0" not in blade_visual_names, (
            f"{prefix} tsuka must be smooth samegawa with no diamond wrap"
        )
        # grip material is samegawa (not pink)
        grip_visual = blade.get_visual("tsuka_grip")
        assert grip_visual is not None
        assert grip_visual.material == "samegawa", (
            f"{prefix} tsuka grip must use samegawa material, got {grip_visual.material}"
        )
        # decoration present on every saya
        assert saya.get_visual("blossom_0") is not None
        assert saya.get_visual("band_0") is not None

    # ---- rack arrangement: two cradled tiers plus one on the box top
    top_saya, _, _ = swords["top"]
    middle_saya, _, _ = swords["middle"]
    bottom_saya, _, _ = swords["bottom"]
    ctx.expect_origin_gap(
        top_saya, middle_saya, axis="z", min_gap=0.10, max_gap=0.18,
        name="top katana rides one tier above the middle katana",
    )
    ctx.expect_origin_gap(
        middle_saya, bottom_saya, axis="z", min_gap=0.14, max_gap=0.22,
        name="middle katana rides above the box-top katana",
    )
    bottom_aabb = ctx.part_world_aabb(bottom_saya)
    assert bottom_aabb is not None
    assert bottom_aabb[0][2] > BOX_TOP_Z - 0.005, "bottom katana rests on the box top"

    return ctx.report()


object_model = build_object_model()
