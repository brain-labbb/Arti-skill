from __future__ import annotations

"""Decorative Japanese katana wall-mount display set.

A flat black-lacquered wall panel carrying six horizontal L-bracket hooks
arranged in three tiers, each tier cradling one sheathed katana. A gold-
framed white kanji plaque adorns the panel front below the lowest tier.
Each katana's blade assembly (blade + habaki + tsuba + handle) slides out
of its fixed white scabbard along the saya long axis via an independent
prismatic joint, travel 0 .. 0.70 m.

World frame: +X sword long axis (handles toward +X), +Z up, panel front
faces -Y. Panel front face at y = 0, panel body at y in [0, PANEL_T].
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
SAYA_LEN = 0.73       # scabbard length along X (mouth at x=0, tail at -0.73)
SAYA_R = 0.019        # scabbard outer radius
BORE_R = 0.0145       # scabbard inner bore radius
BORE_DEPTH = SAYA_LEN - 0.005  # closed tail cap
TRAVEL = 0.70         # prismatic draw travel

BLADE_ROOT_X = -0.005
BLADE_TIP_X = -0.72

GRIP_X0, GRIP_X1 = 0.015, 0.2555
KASHIRA_X0, KASHIRA_X1 = 0.2545, 0.2665

SEAT_DROP = 0.0025    # saya center drops this far below the cradle notch center
NOTCH_R = 0.021       # cradle notch radius (slightly > SAYA_R)

# ---- wall panel
PANEL_W = 0.30        # panel width (X)
PANEL_H = 0.58        # panel height (Z)
PANEL_T = 0.018       # panel thickness (Y)
PANEL_BOTTOM_Z = 0.08 # panel bottom above ground (wall-mounted)

# ---- L-bracket hook
HOOK_T = 0.020        # hook thickness along X (extrusion)
ARM_REACH = 0.048     # arm length from cradle center toward panel (+Y local)
ARM_OUTBOARD = 0.028  # arm extension past cradle center toward -Y
ARM_H = 0.028         # arm height (Z extent below cradle center)
BACK_T = 0.006        # mounting back-plate thickness along Y
BACK_H = 0.058        # back-plate height along Z
RIB_LEN = 0.034       # stiffener rib length along Y (from back plate inward)
RIB_DROP = 0.016      # rib extends below arm bottom

# ---- tier and hook layout
TIER_Z = (0.52, 0.38, 0.24)   # top, middle, bottom tier cradle heights
HOOK_X = (-0.09, 0.09)        # left and right hook X positions
CRADLE_Y = -ARM_REACH          # cradle center world Y (saya centerline)
SWORD_MOUTH_X = 0.28          # saya mouth X (handles extend toward +X)


# ---------------------------------------------------------------- cq shapes
def _saya_tube_shape() -> cq.Workplane:
    """Hollow scabbard tube: open mouth at x=0, closed tail at -SAYA_LEN."""
    outer = cq.Workplane("YZ").circle(SAYA_R).extrude(-SAYA_LEN)
    bore = cq.Workplane("YZ").circle(BORE_R).extrude(-BORE_DEPTH)
    return outer.cut(bore)


def _ring_shape(r_out: float, r_in: float, length: float) -> cq.Workplane:
    """Annular collar spanning local x in [0, length], axis along +X."""
    outer = cq.Workplane("YZ").circle(r_out).extrude(length)
    inner = cq.Workplane("YZ").circle(r_in).extrude(length)
    return outer.cut(inner)


def _blade_shape() -> cq.Workplane:
    """Tapered katana blade lofted along -X."""
    return (
        cq.Workplane("YZ", origin=(BLADE_ROOT_X, 0.0, 0.0))
        .rect(0.0060, 0.0260)
        .workplane(offset=-0.600)
        .rect(0.0050, 0.0220)
        .workplane(offset=-0.115)
        .rect(0.0018, 0.0070)
        .loft(ruled=True)
    )


def _hook_shape() -> cq.Workplane:
    """L-bracket hook with cradle notch and stiffener rib.

    Local origin at the cradle center (where the saya centerline passes).
    +Y toward panel, -Y outward. +Z up. Extruded along +X by HOOK_T.
    """
    # horizontal arm
    arm_y_lo = -ARM_OUTBOARD
    arm_y_hi = ARM_REACH
    arm_z_lo = -ARM_H
    arm_z_hi = 0.0
    arm = (
        cq.Workplane("YZ")
        .center((arm_y_lo + arm_y_hi) / 2.0, (arm_z_lo + arm_z_hi) / 2.0)
        .rect(arm_y_hi - arm_y_lo, arm_z_hi - arm_z_lo)
        .extrude(HOOK_T)
    )

    # mounting back plate (flush against panel front at y_local = ARM_REACH)
    back_y_lo = ARM_REACH - BACK_T
    back_y_hi = ARM_REACH
    back_z_lo = arm_z_lo - 0.005
    back_z_hi = back_z_lo + BACK_H
    back = (
        cq.Workplane("YZ")
        .center((back_y_lo + back_y_hi) / 2.0, (back_z_lo + back_z_hi) / 2.0)
        .rect(back_y_hi - back_y_lo, back_z_hi - back_z_lo)
        .extrude(HOOK_T)
    )
    shape = arm.union(back)

    # stiffener rib below the arm, running from back plate inward
    rib_y_hi = ARM_REACH - BACK_T
    rib_y_lo = rib_y_hi - RIB_LEN
    rib_z_lo = arm_z_lo - RIB_DROP
    rib_z_hi = arm_z_lo
    rib = (
        cq.Workplane("YZ")
        .center((rib_y_lo + rib_y_hi) / 2.0, (rib_z_lo + rib_z_hi) / 2.0)
        .rect(rib_y_hi - rib_y_lo, rib_z_hi - rib_z_lo)
        .extrude(HOOK_T)
    )
    shape = shape.union(rib)

    # cradle notch: semicircular groove cut from arm top at the cradle center
    notch = (
        cq.Workplane("YZ")
        .center(0.0, 0.0)
        .circle(NOTCH_R)
        .extrude(HOOK_T)
    )
    shape = shape.cut(notch)
    return shape


# ---------------------------------------------------------------- model
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="katana_wall_display")

    model.material("black_lacquer", rgba=(0.06, 0.05, 0.06, 1.0))
    model.material("gold", rgba=(0.80, 0.64, 0.25, 1.0))
    model.material("plaque_white", rgba=(0.93, 0.92, 0.88, 1.0))
    model.material("ink", rgba=(0.10, 0.09, 0.10, 1.0))
    model.material("saya_white", rgba=(0.94, 0.93, 0.94, 1.0))
    model.material("blossom_pink", rgba=(0.95, 0.65, 0.76, 1.0))
    model.material("deep_pink", rgba=(0.89, 0.45, 0.62, 1.0))
    model.material("grip_pink", rgba=(0.92, 0.58, 0.70, 1.0))
    model.material("tsuba_red", rgba=(0.40, 0.10, 0.12, 1.0))
    model.material("dark_fitting", rgba=(0.13, 0.11, 0.13, 1.0))
    model.material("steel", rgba=(0.83, 0.85, 0.88, 1.0))

    # --------------------------------------------------------- wall panel (root)
    panel = model.part("wall_panel")
    panel_cz = PANEL_BOTTOM_Z + PANEL_H / 2.0
    panel.visual(
        Box((PANEL_W, PANEL_T, PANEL_H)),
        origin=Origin(xyz=(0.0, PANEL_T / 2.0, panel_cz)),
        material="black_lacquer",
        name="panel_body",
    )

    # gold frame border around panel edge (front face, proud by ~1 mm)
    frame_specs = (
        ("frame_top",    Box((PANEL_W + 0.006, 0.004, 0.010)),
         (0.0, -0.001, PANEL_BOTTOM_Z + PANEL_H - 0.005)),
        ("frame_bottom", Box((PANEL_W + 0.006, 0.004, 0.010)),
         (0.0, -0.001, PANEL_BOTTOM_Z + 0.005)),
        ("frame_left",   Box((0.010, 0.004, PANEL_H - 0.010)),
         (-PANEL_W / 2.0 + 0.005, -0.001, panel_cz)),
        ("frame_right",  Box((0.010, 0.004, PANEL_H - 0.010)),
         (PANEL_W / 2.0 - 0.005, -0.001, panel_cz)),
    )
    for f_name, f_geom, f_xyz in frame_specs:
        panel.visual(f_geom, origin=Origin(xyz=f_xyz), material="gold", name=f_name)

    # gold-framed kanji plaque below the bottom tier
    plaque_z = PANEL_BOTTOM_Z + 0.055
    panel.visual(
        Box((0.088, 0.005, 0.058)),
        origin=Origin(xyz=(0.0, -0.001, plaque_z)),
        material="plaque_white",
        name="plaque_panel",
    )
    plaque_frame = (
        ("plaque_frame_top",    Box((0.105, 0.007, 0.009)),
         (0.0, -0.003, plaque_z + 0.033)),
        ("plaque_frame_bottom", Box((0.105, 0.007, 0.009)),
         (0.0, -0.003, plaque_z - 0.033)),
        ("plaque_frame_left",   Box((0.009, 0.007, 0.075)),
         (-0.048, -0.003, plaque_z)),
        ("plaque_frame_right",  Box((0.009, 0.007, 0.075)),
         (0.048, -0.003, plaque_z)),
    )
    for pf_name, pf_geom, pf_xyz in plaque_frame:
        panel.visual(pf_geom, origin=Origin(xyz=pf_xyz), material="gold", name=pf_name)

    # stylized kanji strokes
    kanji_strokes = (
        (Box((0.034, 0.002, 0.0045)), (0.0, -0.004, plaque_z + 0.017)),
        (Box((0.046, 0.002, 0.0045)), (0.0, -0.004, plaque_z + 0.003)),
        (Box((0.030, 0.002, 0.0045)), (-0.012, -0.004, plaque_z - 0.015)),
        (Box((0.030, 0.002, 0.0045)), (0.012, -0.004, plaque_z - 0.015)),
    )
    for i, (k_geom, k_xyz) in enumerate(kanji_strokes):
        panel.visual(
            k_geom, origin=Origin(xyz=k_xyz), material="ink", name=f"kanji_stroke_{i}"
        )

    # bracket hooks: 6 total (2 per tier), shared mesh, placed via loop
    hook_mesh = mesh_from_cadquery(_hook_shape(), "bracket_hook")
    hook_idx = 0
    for tier_z in TIER_Z:
        for hook_x in HOOK_X:
            panel.visual(
                hook_mesh,
                origin=Origin(xyz=(hook_x - HOOK_T / 2.0, CRADLE_Y, tier_z)),
                material="black_lacquer",
                name=f"hook_{hook_idx}",
            )
            hook_idx += 1

    # --------------------------------------------------------- shared sword meshes
    saya_mesh = mesh_from_cadquery(_saya_tube_shape(), "saya_tube")
    blade_mesh = mesh_from_cadquery(_blade_shape(), "katana_blade")
    band_len = 0.022
    band_mesh = mesh_from_cadquery(_ring_shape(0.0198, 0.0146, band_len), "saya_band")
    mouth_len = 0.012
    mouth_mesh = mesh_from_cadquery(
        _ring_shape(0.0205, 0.0146, mouth_len), "saya_mouth_ring"
    )
    rim_len = 0.009
    rim_mesh = mesh_from_cadquery(_ring_shape(0.0435, 0.0375, rim_len), "tsuba_rim")

    blossom_spots = ((-0.55, 0.30, True), (-0.36, -0.15, True), (-0.47, 0.65, True))
    dot_spots = ((-0.52, -0.35), (-0.40, 0.45), (-0.61, 0.10))

    def _build_saya(part_name: str):
        saya = model.part(part_name)
        saya.visual(saya_mesh, material="saya_white", name="saya_tube")
        saya.visual(
            mouth_mesh,
            origin=Origin(xyz=(-mouth_len, 0.0, 0.0)),
            material="deep_pink",
            name="mouth_ring",
        )
        for j, band_x in enumerate((-0.665, -0.30)):
            saya.visual(
                band_mesh,
                origin=Origin(xyz=(band_x - band_len / 2.0, 0.0, 0.0)),
                material="deep_pink",
                name=f"band_{j}",
            )
        for j, (bx, az, _big) in enumerate(blossom_spots):
            radial = 0.019
            saya.visual(
                Cylinder(radius=0.0075, length=0.003),
                origin=Origin(
                    xyz=(bx, -radial * sin(az), radial * cos(az)), rpy=(az, 0.0, 0.0)
                ),
                material="blossom_pink",
                name=f"blossom_{j}",
            )
            saya.visual(
                Cylinder(radius=0.0027, length=0.0036),
                origin=Origin(
                    xyz=(bx, -0.0195 * sin(az), 0.0195 * cos(az)), rpy=(az, 0.0, 0.0)
                ),
                material="gold",
                name=f"blossom_center_{j}",
            )
        for j, (dx, az) in enumerate(dot_spots):
            saya.visual(
                Cylinder(radius=0.004, length=0.003),
                origin=Origin(
                    xyz=(dx, -0.019 * sin(az), 0.019 * cos(az)), rpy=(az, 0.0, 0.0)
                ),
                material="blossom_pink",
                name=f"petal_dot_{j}",
            )
        return saya

    def _build_blade_assembly(part_name: str):
        blade = model.part(part_name)
        blade.visual(blade_mesh, material="steel", name="blade_body")
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
            origin=Origin(
                xyz=((GRIP_X0 + GRIP_X1) / 2.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)
            ),
            material="grip_pink",
            name="tsuka_grip",
        )
        blade.visual(
            Cylinder(radius=0.0158, length=KASHIRA_X1 - KASHIRA_X0),
            origin=Origin(
                xyz=((KASHIRA_X0 + KASHIRA_X1) / 2.0, 0.0, 0.0),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material="dark_fitting",
            name="kashira_pommel",
        )
        for j, dx in enumerate((0.060, 0.135, 0.210)):
            blade.visual(
                Box((0.016, 0.016, 0.0024)),
                origin=Origin(xyz=(dx, 0.0, 0.0150), rpy=(0.0, 0.0, pi / 4.0)),
                material="dark_fitting",
                name=f"wrap_diamond_top_{j}",
            )
        for j, dx in enumerate((0.0975, 0.1725)):
            blade.visual(
                Box((0.016, 0.0024, 0.016)),
                origin=Origin(xyz=(dx, -0.0150, 0.0), rpy=(0.0, pi / 4.0, 0.0)),
                material="dark_fitting",
                name=f"wrap_diamond_front_{j}",
            )
        return blade

    # --------------------------------------------------------- swords
    for i in range(3):
        tier_z = TIER_Z[i]
        saya = _build_saya(f"saya_{i}")
        blade = _build_blade_assembly(f"blade_{i}")

        mount_xyz = (SWORD_MOUTH_X, CRADLE_Y, tier_z - SEAT_DROP)
        model.articulation(
            f"saya_mount_{i}",
            ArticulationType.FIXED,
            parent=panel,
            child=saya,
            origin=Origin(xyz=mount_xyz),
        )
        model.articulation(
            f"blade_draw_{i}",
            ArticulationType.PRISMATIC,
            parent=saya,
            child=blade,
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=25.0, velocity=0.6, lower=0.0, upper=TRAVEL
            ),
        )

    return model


# ---------------------------------------------------------------- tests
def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    panel = object_model.get_part("wall_panel")

    # ---- panel: flat vertical wall-mount board
    body = ctx.part_element_world_aabb(panel, elem="panel_body")
    assert body is not None
    body_dx = body[1][0] - body[0][0]
    body_dy = body[1][1] - body[0][1]
    body_dz = body[1][2] - body[0][2]
    assert 0.28 <= body_dx <= 0.32, "panel ~0.30 m wide"
    assert 0.55 <= body_dz <= 0.62, "panel ~0.58 m tall"
    assert 0.014 <= body_dy <= 0.024, "panel ~0.018 m thick"
    assert body_dz > 10.0 * body_dy, "panel is a tall vertical board"
    assert body_dx > 5.0 * body_dy, "panel is a wide vertical board"

    panel_aabb = ctx.part_world_aabb(panel)
    assert panel_aabb is not None
    assert panel_aabb[0][2] >= 0.06, "panel bottom above ground (wall-mounted)"

    # ---- hooks: 6 brackets protruding from panel front
    for i in range(6):
        hook_aabb = ctx.part_element_world_aabb(panel, elem=f"hook_{i}")
        assert hook_aabb is not None, f"hook_{i} exists"
        assert hook_aabb[0][1] < -0.025, f"hook_{i} protrudes at least 25 mm from panel"
        assert hook_aabb[1][1] > -0.005, f"hook_{i} reaches near the panel front face"

    # ---- plaque on panel front, below the bottom tier
    plaque = ctx.part_element_world_aabb(panel, elem="plaque_panel")
    assert plaque is not None
    assert plaque[0][1] < 0.0, "kanji plaque proud of the panel front face"
    assert plaque[1][2] < TIER_Z[2] - 0.01, "plaque sits below the bottom tier"

    # gold frame present
    assert panel.get_visual("frame_top") is not None
    assert panel.get_visual("plaque_frame_top") is not None

    # ---- swords: three tiers, each with prismatic blade draw
    swords = {}
    for i in range(3):
        saya = object_model.get_part(f"saya_{i}")
        blade = object_model.get_part(f"blade_{i}")
        draw = object_model.get_articulation(f"blade_draw_{i}")
        swords[i] = (saya, blade, draw)

        # joint policy: prismatic draw along +X, travel = TRAVEL
        assert draw.articulation_type == ArticulationType.PRISMATIC
        assert draw.axis == (1.0, 0.0, 0.0)
        limits = draw.motion_limits
        assert limits is not None and limits.lower == 0.0
        assert limits.upper is not None and abs(limits.upper - TRAVEL) < 1e-9

        # saya seated in hook cradles (scoped overlap per hook pair)
        for j in range(2):
            ctx.allow_overlap(
                saya, panel,
                elem_a="saya_tube",
                elem_b=f"hook_{2 * i + j}",
                reason=f"saya_{i} seats into the hook_{2*i+j} cradle notch",
            )
        ctx.expect_contact(saya, panel, name=f"saya_{i} seated on bracket hooks")

        # sheathed blade inside hollow saya bore
        ctx.allow_overlap(
            blade, saya,
            elem_a="blade_body",
            elem_b="saya_tube",
            reason="sheathed blade nests inside the hollow saya bore",
        )

        # sheathed pose: blade at q=0, hidden inside saya
        ctx.expect_origin_distance(
            blade, saya, axes="x", min_dist=0.0, max_dist=1e-6,
            name=f"blade_{i} fully sheathed at q=0",
        )
        ctx.expect_within(
            blade, saya, axes="yz",
            inner_elem="blade_body", outer_elem="saya_tube",
            name=f"blade_{i} nests inside saya bore (YZ)",
        )
        ctx.expect_within(
            blade, saya, axes="x",
            inner_elem="blade_body", outer_elem="saya_tube",
            margin=0.001,
            name=f"blade_{i} hidden along saya length",
        )

        # draw pose: blade translates along +X, retains insertion
        with ctx.pose({draw: TRAVEL}):
            ctx.expect_origin_gap(
                blade, saya, axis="x",
                min_gap=TRAVEL - 0.001, max_gap=TRAVEL + 0.001,
                name=f"blade_{i} draw translates {TRAVEL} m along +X",
            )
            ctx.expect_overlap(
                blade, saya, axes="x",
                elem_a="blade_body", elem_b="saya_tube",
                min_overlap=0.010,
                name=f"blade_{i} stays engaged in saya at full draw",
            )
        with ctx.pose({draw: 0.35}):
            ctx.expect_origin_gap(
                blade, saya, axis="x",
                min_gap=0.349, max_gap=0.351,
                name=f"blade_{i} mid-draw translates 0.35 m",
            )

        # hero sword features: ~1.0 m total, pink grip, tsuba, pommel
        saya_aabb = ctx.part_world_aabb(saya)
        blade_aabb = ctx.part_world_aabb(blade)
        assert saya_aabb is not None and blade_aabb is not None
        total_len = max(saya_aabb[1][0], blade_aabb[1][0]) - min(
            saya_aabb[0][0], blade_aabb[0][0]
        )
        assert 0.95 <= total_len <= 1.05, f"sword {i} ~1.0 m long, got {total_len:.3f}"
        grip = ctx.part_element_world_aabb(blade, elem="tsuka_grip")
        assert grip is not None
        assert 0.22 <= grip[1][0] - grip[0][0] <= 0.27, "tsuka ~0.25 m long"
        tsuba = ctx.part_element_world_aabb(blade, elem="tsuba_disc")
        assert tsuba is not None
        tsuba_dia = tsuba[1][2] - tsuba[0][2]
        assert 0.07 <= tsuba_dia <= 0.09, "ornate tsuba disc ~0.082 m diameter"
        assert tsuba_dia > 2.2 * SAYA_R, "tsuba wider than the saya"
        pommel = ctx.part_element_world_aabb(blade, elem="kashira_pommel")
        assert pommel is not None
        assert pommel[0][0] >= grip[1][0] - 0.002, "dark pommel caps the handle end"
        # decoration present on every saya
        assert saya.get_visual("blossom_0") is not None
        assert saya.get_visual("band_0") is not None

    # ---- tier arrangement: three stacked tiers with clear vertical separation
    for i in range(2):
        ctx.expect_origin_gap(
            swords[i][0], swords[i + 1][0], axis="z",
            min_gap=0.08, max_gap=0.20,
            name=f"tier {i} katana rides above tier {i + 1}",
        )

    # wall-mount proof: all swords clear of ground (no base box)
    for i in range(3):
        saya_aabb = ctx.part_world_aabb(swords[i][0])
        assert saya_aabb is not None
        assert saya_aabb[0][2] > 0.15, f"saya_{i} well above ground (wall-mounted)"

    return ctx.report()


object_model = build_object_model()
