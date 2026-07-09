from __future__ import annotations

"""Decorative Japanese katana display set — upright post variant.

A black lacquered three-post upright stand on a rectangular base box with a
gold-framed kanji plaque.  Each post is a U-channel column that holds a
sheathed katana at a near-vertical angle (2° lean).  Each katana's blade
assembly (blade + habaki + tsuba + handle) slides out of its fixed white
scabbard (saya) along the saya's long axis via an independent prismatic
joint, travel 0 .. 0.70 m.

World frame: +Z up, the kanji plaque faces -Y (front).  Ground at z = 0.
"""

from math import pi, sin, cos, tan

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

# ============================================================= constants
SAYA_LEN = 0.73          # scabbard length (mouth at x=0, tail at -0.73)
SAYA_R = 0.019           # scabbard outer radius
BORE_R = 0.0145          # scabbard bore radius
BORE_DEPTH = SAYA_LEN - 0.005
TRAVEL = 0.70            # prismatic draw travel

BLADE_ROOT_X = -0.005
BLADE_TIP_X = -0.72

GRIP_X0, GRIP_X1 = 0.015, 0.2555
KASHIRA_X0, KASHIRA_X1 = 0.2545, 0.2665

BOX_TOP_Z = 0.12         # base-box overall height

# ---------- post geometry
POST_W = 0.085           # post width (X)
POST_D = 0.025           # post depth (Y)
POST_H = 0.650           # post body height (Z)
WALL_T = 0.008           # side-wall thickness
BACK_T = 0.008           # back-panel thickness
FOOT_T = 0.008           # foot thickness
CAP_T  = 0.008           # top-cap thickness

# U-channel inner dimensions
CHAN_W = POST_W - 2 * WALL_T   # 0.069 m
CHAN_D = POST_D - BACK_T       # 0.017 m

# ---------- lean geometry (2° from vertical toward +X)
LEAN = 0.0349            # radians ≈ 2°
PITCH = -(pi / 2.0 - LEAN)

# ---------- mount geometry
MOUTH_Z = 0.87           # saya-mouth height in stand frame
# back-panel front face Y (in post-local coords = stand Y since post at y=0)
_BACK_FRONT_Y = POST_D / 2.0 - BACK_T  # 0.0045
MOUTH_Y = _BACK_FRONT_Y - SAYA_R + 0.0005  # 0.5 mm seat into back panel

N_SWORDS = 3
POST_XS = (-0.10, 0.0, 0.10)

# pre-computed mouth-X offset so the saya is centred in the channel at
# mid-post height
_MID_POST_Z = BOX_TOP_Z + POST_H / 2.0
_MOUTH_X_OFF = (MOUTH_Z - _MID_POST_Z) * tan(LEAN)


# ============================================================= CQ shapes
def _saya_tube_shape() -> cq.Workplane:
    """Hollow scabbard tube: open mouth at x = 0, closed tail at -SAYA_LEN."""
    outer = cq.Workplane("YZ").circle(SAYA_R).extrude(-SAYA_LEN)
    bore  = cq.Workplane("YZ").circle(BORE_R).extrude(-BORE_DEPTH)
    return outer.cut(bore)


def _ring_shape(r_out: float, r_in: float, length: float) -> cq.Workplane:
    outer = cq.Workplane("YZ").circle(r_out).extrude(length)
    inner = cq.Workplane("YZ").circle(r_in).extrude(length)
    return outer.cut(inner)


def _blade_shape() -> cq.Workplane:
    return (
        cq.Workplane("YZ", origin=(BLADE_ROOT_X, 0.0, 0.0))
        .rect(0.0060, 0.0260)
        .workplane(offset=-0.600)
        .rect(0.0050, 0.0220)
        .workplane(offset=-0.115)
        .rect(0.0018, 0.0070)
        .loft(ruled=True)
    )


def _post_shape() -> cq.Workplane:
    """Upright U-channel post: back panel + two side walls + foot + cap."""
    back_cy = POST_D / 2.0 - BACK_T / 2.0
    shape = (
        cq.Workplane("XY")
        .center(0.0, back_cy)
        .rect(POST_W, BACK_T)
        .extrude(POST_H)
    )
    for sign in (-1, 1):
        wall_cx = sign * (POST_W / 2.0 - WALL_T / 2.0)
        wall = (
            cq.Workplane("XY")
            .center(wall_cx, 0.0)
            .rect(WALL_T, POST_D)
            .extrude(POST_H)
        )
        shape = shape.union(wall)
    # solid foot (wider than post, ties walls together at base)
    foot = (
        cq.Workplane("XY")
        .rect(POST_W + 0.012, POST_D + 0.012)
        .extrude(FOOT_T)
    )
    shape = shape.union(foot)
    # open top — channel is open so the sheathed katana can pass through;
    # the back panel and side walls are already connected through the full
    # height corner joints plus the solid foot at the base.
    return shape


# ============================================================= model
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="katana_display_set")

    # materials -------------------------------------------------------
    model.material("black_lacquer", rgba=(0.06, 0.05, 0.06, 1.0))
    model.material("gold",          rgba=(0.80, 0.64, 0.25, 1.0))
    model.material("plaque_white",  rgba=(0.93, 0.92, 0.88, 1.0))
    model.material("ink",           rgba=(0.10, 0.09, 0.10, 1.0))
    model.material("saya_white",    rgba=(0.94, 0.93, 0.94, 1.0))
    model.material("blossom_pink",  rgba=(0.95, 0.65, 0.76, 1.0))
    model.material("deep_pink",     rgba=(0.89, 0.45, 0.62, 1.0))
    model.material("grip_pink",     rgba=(0.92, 0.58, 0.70, 1.0))
    model.material("tsuba_red",     rgba=(0.40, 0.10, 0.12, 1.0))
    model.material("dark_fitting",  rgba=(0.13, 0.11, 0.13, 1.0))
    model.material("steel",         rgba=(0.83, 0.85, 0.88, 1.0))

    # stand: base box -------------------------------------------------
    stand = model.part("display_stand")
    stand.visual(
        Box((0.31, 0.19, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material="black_lacquer", name="base_plinth",
    )
    stand.visual(
        Box((0.30, 0.18, 0.100)),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material="black_lacquer", name="base_body",
    )
    stand.visual(
        Box((0.31, 0.19, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.113)),
        material="black_lacquer", name="base_cap",
    )

    # upright slotted posts -------------------------------------------
    post_mesh = mesh_from_cadquery(_post_shape(), "stand_post")
    for i in range(N_SWORDS):
        stand.visual(
            post_mesh,
            origin=Origin(xyz=(POST_XS[i], 0.0, BOX_TOP_Z)),
            material="black_lacquer",
            name=f"post_{i}",
        )

    # gold-framed kanji plaque (front -Y face of base box) ------------
    stand.visual(
        Box((0.088, 0.005, 0.058)),
        origin=Origin(xyz=(0.0, -0.0905, 0.065)),
        material="plaque_white", name="plaque_panel",
    )
    _frame_bars = (
        ("frame_top",    Box((0.105, 0.007, 0.009)), (0.0,   -0.0925, 0.098)),
        ("frame_bottom", Box((0.105, 0.007, 0.009)), (0.0,   -0.0925, 0.032)),
        ("frame_left",   Box((0.009, 0.007, 0.075)), (-0.048, -0.0925, 0.065)),
        ("frame_right",  Box((0.009, 0.007, 0.075)), (0.048, -0.0925, 0.065)),
    )
    for bar_name, bar_geom, bar_xyz in _frame_bars:
        stand.visual(
            bar_geom, origin=Origin(xyz=bar_xyz),
            material="gold", name=bar_name,
        )
    _kanji = (
        (Box((0.034, 0.002, 0.0045)), (0.0,   -0.0935, 0.082), (0.0, 0.0, 0.0)),
        (Box((0.046, 0.002, 0.0045)), (0.0,   -0.0935, 0.068), (0.0, 0.0, 0.0)),
        (Box((0.030, 0.002, 0.0045)), (-0.012, -0.0935, 0.050), (0.0, 0.9, 0.0)),
        (Box((0.030, 0.002, 0.0045)), (0.012, -0.0935, 0.050), (0.0, -0.9, 0.0)),
    )
    for idx, (geom, xyz, rpy) in enumerate(_kanji):
        stand.visual(
            geom, origin=Origin(xyz=xyz, rpy=rpy),
            material="ink", name=f"kanji_stroke_{idx}",
        )

    # shared sword meshes ---------------------------------------------
    saya_mesh  = mesh_from_cadquery(_saya_tube_shape(), "saya_tube")
    blade_mesh = mesh_from_cadquery(_blade_shape(), "katana_blade")
    band_len   = 0.022
    band_mesh  = mesh_from_cadquery(_ring_shape(0.0198, 0.0146, band_len), "saya_band")
    mouth_len  = 0.012
    mouth_mesh = mesh_from_cadquery(_ring_shape(0.0205, 0.0146, mouth_len), "saya_mouth_ring")
    rim_len    = 0.009
    rim_mesh   = mesh_from_cadquery(_ring_shape(0.0435, 0.0375, rim_len), "tsuba_rim")

    _blossom_spots = ((-0.55, 0.30, True), (-0.36, -0.15, True), (-0.47, 0.65, True))
    _dot_spots     = ((-0.52, -0.35), (-0.40, 0.45), (-0.61, 0.10))

    # ---- sword builders
    def _build_saya(part_name: str):
        saya = model.part(part_name)
        saya.visual(saya_mesh, material="saya_white", name="saya_tube")
        saya.visual(
            mouth_mesh,
            origin=Origin(xyz=(-mouth_len, 0.0, 0.0)),
            material="deep_pink", name="mouth_ring",
        )
        for j, bx in enumerate((-0.665, -0.30)):
            saya.visual(
                band_mesh,
                origin=Origin(xyz=(bx - band_len / 2.0, 0.0, 0.0)),
                material="deep_pink", name=f"band_{j}",
            )
        for j, (bx, az, _big) in enumerate(_blossom_spots):
            r = 0.019
            saya.visual(
                Cylinder(radius=0.0075, length=0.003),
                origin=Origin(xyz=(bx, -r * sin(az), r * cos(az)), rpy=(az, 0.0, 0.0)),
                material="blossom_pink", name=f"blossom_{j}",
            )
            saya.visual(
                Cylinder(radius=0.0027, length=0.0036),
                origin=Origin(
                    xyz=(bx, -0.0195 * sin(az), 0.0195 * cos(az)),
                    rpy=(az, 0.0, 0.0),
                ),
                material="gold", name=f"blossom_center_{j}",
            )
        for j, (dx, az) in enumerate(_dot_spots):
            saya.visual(
                Cylinder(radius=0.004, length=0.003),
                origin=Origin(
                    xyz=(dx, -0.019 * sin(az), 0.019 * cos(az)),
                    rpy=(az, 0.0, 0.0),
                ),
                material="blossom_pink", name=f"petal_dot_{j}",
            )
        return saya

    def _build_blade(part_name: str):
        bl = model.part(part_name)
        bl.visual(blade_mesh, material="steel", name="blade_body")
        # habaki collar
        bl.visual(
            Box((0.0305, 0.010, 0.024)),
            origin=Origin(xyz=(-0.01475, 0.0, 0.0)),
            material="gold", name="habaki_collar",
        )
        # tsuba disc
        bl.visual(
            Cylinder(radius=0.041, length=0.007),
            origin=Origin(xyz=(0.0035, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material="tsuba_red", name="tsuba_disc",
        )
        bl.visual(
            rim_mesh,
            origin=Origin(xyz=(-0.001, 0.0, 0.0)),
            material="gold", name="tsuba_rim",
        )
        # fuchi collar
        bl.visual(
            Cylinder(radius=0.0160, length=0.0105),
            origin=Origin(xyz=(0.01175, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material="dark_fitting", name="fuchi_collar",
        )
        # tsuka grip
        grip_len = GRIP_X1 - GRIP_X0
        bl.visual(
            Cylinder(radius=0.0150, length=grip_len),
            origin=Origin(
                xyz=((GRIP_X0 + GRIP_X1) / 2.0, 0.0, 0.0),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material="grip_pink", name="tsuka_grip",
        )
        # kashira pommel
        bl.visual(
            Cylinder(radius=0.0158, length=KASHIRA_X1 - KASHIRA_X0),
            origin=Origin(
                xyz=((KASHIRA_X0 + KASHIRA_X1) / 2.0, 0.0, 0.0),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material="dark_fitting", name="kashira_pommel",
        )
        # diamond wrap accents
        for j, dx in enumerate((0.060, 0.135, 0.210)):
            bl.visual(
                Box((0.016, 0.016, 0.0024)),
                origin=Origin(xyz=(dx, 0.0, 0.0150), rpy=(0.0, 0.0, pi / 4.0)),
                material="dark_fitting", name=f"wrap_diamond_top_{j}",
            )
        for j, dx in enumerate((0.0975, 0.1725)):
            bl.visual(
                Box((0.016, 0.0024, 0.016)),
                origin=Origin(xyz=(dx, -0.0150, 0.0), rpy=(0.0, pi / 4.0, 0.0)),
                material="dark_fitting", name=f"wrap_diamond_front_{j}",
            )
        return bl

    # ---- build each sword + post mount
    for i in range(N_SWORDS):
        px = POST_XS[i]
        mouth_x = px + _MOUTH_X_OFF
        mount_xyz = (mouth_x, MOUTH_Y, MOUTH_Z)

        saya  = _build_saya(f"saya_{i}")
        blade = _build_blade(f"blade_{i}")

        # fixed mount: saya seated in the post U-channel
        model.articulation(
            f"saya_mount_{i}",
            ArticulationType.FIXED,
            parent=stand,
            child=saya,
            origin=Origin(xyz=mount_xyz, rpy=(0.0, PITCH, 0.0)),
        )
        # prismatic blade draw along the saya long axis
        model.articulation(
            f"blade_draw_{i}",
            ArticulationType.PRISMATIC,
            parent=saya,
            child=blade,
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=25.0, velocity=0.6, lower=0.0, upper=TRAVEL,
            ),
        )

    return model


# ============================================================= tests
def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("display_stand")

    # ---- base box: real-world scale and grounding
    stand_aabb = ctx.part_world_aabb(stand)
    assert stand_aabb is not None
    assert -1e-6 <= stand_aabb[0][2] <= 0.002, "stand rests on ground"
    plinth = ctx.part_element_world_aabb(stand, elem="base_plinth")
    assert plinth is not None
    assert 0.28 <= plinth[1][0] - plinth[0][0] <= 0.33, "base ~0.30 m wide"
    assert 0.16 <= plinth[1][1] - plinth[0][1] <= 0.21, "base ~0.18 m deep"
    cap = ctx.part_element_world_aabb(stand, elem="base_cap")
    assert cap is not None
    assert 0.11 <= cap[1][2] <= 0.13, "base ~0.12 m tall"

    # ---- three upright posts
    for i in range(N_SWORDS):
        post_aabb = ctx.part_element_world_aabb(stand, elem=f"post_{i}")
        assert post_aabb is not None
        post_h = post_aabb[1][2] - post_aabb[0][2]
        assert post_h > 0.60, f"post_{i} is tall (>0.60 m), got {post_h:.3f}"
        post_w = post_aabb[1][0] - post_aabb[0][0]
        assert 0.075 <= post_w <= 0.10, f"post_{i} width ~0.085 m, got {post_w:.3f}"
        # post base on the box top
        assert abs(post_aabb[0][2] - BOX_TOP_Z) < 0.005, f"post_{i} sits on box top"

    # ---- kanji plaque
    plaque = ctx.part_element_world_aabb(stand, elem="plaque_panel")
    assert plaque is not None
    assert plaque[0][1] < -0.090, "plaque proud of front face"

    # ---- each sword: near-vertical, prismatic draw, decoration
    for i in range(N_SWORDS):
        saya  = object_model.get_part(f"saya_{i}")
        blade = object_model.get_part(f"blade_{i}")
        draw  = object_model.get_articulation(f"blade_draw_{i}")

        # joint contract
        assert draw.articulation_type == ArticulationType.PRISMATIC
        assert draw.axis == (1.0, 0.0, 0.0)
        lim = draw.motion_limits
        assert lim is not None and lim.lower == 0.0
        assert lim.upper is not None and abs(lim.upper - TRAVEL) < 1e-9

        # near-vertical: Z extent must dominate X extent
        saya_aabb = ctx.part_world_aabb(saya)
        assert saya_aabb is not None
        z_ext = saya_aabb[1][2] - saya_aabb[0][2]
        x_ext = saya_aabb[1][0] - saya_aabb[0][0]
        assert z_ext > 3.0 * x_ext, (
            f"saya_{i} near-vertical: z={z_ext:.3f} >> x={x_ext:.3f}"
        )
        assert z_ext > 0.65, f"saya_{i} Z extent > 0.65 m, got {z_ext:.3f}"

        # saya seated in the post channel: the tube and decorative bands
        # contact/embed slightly into the channel back panel by design
        ctx.allow_overlap(
            saya, stand,
            elem_a="saya_tube", elem_b=f"post_{i}",
            reason=f"saya_{i} seats 0.5 mm into the post_{i} channel back panel",
        )
        for band_name in ("band_0", "band_1"):
            ctx.allow_overlap(
                saya, stand,
                elem_a=band_name, elem_b=f"post_{i}",
                reason=(
                    f"saya_{i} {band_name} ring protrudes 0.8 mm past the saya "
                    f"surface into the post_{i} channel back"
                ),
            )
        ctx.expect_contact(
            saya, stand,
            name=f"saya_{i} contacts the stand post",
        )

        # sheathed blade inside the hollow saya
        ctx.allow_overlap(
            blade, saya,
            elem_a="blade_body", elem_b="saya_tube",
            reason="sheathed blade nests inside the hollow saya bore",
        )
        ctx.expect_within(
            blade, saya, axes="yz",
            inner_elem="blade_body", outer_elem="saya_tube",
            name=f"blade_{i} nests inside saya bore",
        )
        ctx.expect_origin_distance(
            blade, saya, axes="x", min_dist=0.0, max_dist=1e-6,
            name=f"blade_{i} fully sheathed at q=0",
        )

        # draw pose: blade translates along +X (mostly world +Z)
        with ctx.pose({draw: TRAVEL}):
            ctx.expect_overlap(
                blade, saya, axes="x",
                elem_a="blade_body", elem_b="saya_tube",
                min_overlap=0.010,
                name=f"blade_{i} stays engaged at full draw",
            )

        # katana total height (near-vertical, so Z extent ≈ total length)
        blade_aabb = ctx.part_world_aabb(blade)
        assert blade_aabb is not None
        total_z = max(saya_aabb[1][2], blade_aabb[1][2]) - min(
            saya_aabb[0][2], blade_aabb[0][2]
        )
        assert 0.90 <= total_z <= 1.10, (
            f"katana_{i} total height ~1.0 m, got {total_z:.3f}"
        )

        # tsuba wider than saya
        tsuba = ctx.part_element_world_aabb(blade, elem="tsuba_disc")
        assert tsuba is not None
        tsuba_max = max(
            tsuba[1][0] - tsuba[0][0],
            tsuba[1][1] - tsuba[0][1],
            tsuba[1][2] - tsuba[0][2],
        )
        assert tsuba_max > 2.0 * SAYA_R, "tsuba wider than saya"

        # decoration present
        assert saya.get_visual("blossom_0") is not None
        assert saya.get_visual("band_0") is not None

    # ---- arrangement: three swords side by side along X
    for i in range(1, N_SWORDS):
        prev = object_model.get_part(f"saya_{i - 1}")
        curr = object_model.get_part(f"saya_{i}")
        ctx.expect_origin_gap(
            curr, prev, axis="x",
            min_gap=0.05, max_gap=0.18,
            name=f"saya_{i} is to the right of saya_{i-1}",
        )

    return ctx.report()


object_model = build_object_model()
