from __future__ import annotations

"""Kitchen knife block set — universal bristle-insert variant.

A smoked-acrylic housing (~0.14 × 0.10 × 0.22 m) filled with a dense grid of
flexible bristle rods that knives push between.  One chef knife slides
vertically on a prismatic joint; five companion knives sit fixed in the
bristles as inline block visuals.  Kitchen shears ride in a front pocket
(prismatic slide + revolute pivot).
"""

import math

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

# ── housing ───────────────────────────────────────────────────────────────
HW = 0.140          # width  (Y)
HD = 0.100          # depth  (X)
HH = 0.220          # height (Z)
WT = 0.004          # wall thickness
FT = 0.006          # floor thickness
LIFT = 0.008        # rubber-foot lift

CW = HW - 2 * WT   # cavity width  0.132
CD = HD - 2 * WT   # cavity depth  0.092

# ── bristle grid ──────────────────────────────────────────────────────────
BR_R  = 0.0015      # rod radius (3 mm dia.)
BR_SP = 0.013       # centre-to-centre spacing
BR_H  = HH - FT - 0.010           # rod height (stop below rim)

N_BX = max(2, int(CD / BR_SP) - 1)   # rows   in X
N_BY = max(2, int(CW / BR_SP) - 1)   # cols   in Y
N_BRISTLES = N_BX * N_BY

# ── front pocket (shears bay) ────────────────────────────────────────────
PK_W = 0.110
PK_D = 0.025
PK_H = 0.140
PK_X = HD / 2 + PK_D / 2            # pocket centre X

# ── knife insertion reference ────────────────────────────────────────────
# Knife local frame: bolster centre at z = 0.023.
# Fully-seated → bolster at the housing rim  (z = LIFT + HH).
INSERT_Z = LIFT + HH                 # part / visual origin z for seated knife
VIS_ZOFF = -0.023                    # visual offset so bolster lands on rim

# ── fixed-knife table: (name, x, y, blade_len, blade_w, grip_len) ────────
FIXED_KNIVES = [
    ("bread_knife",    -0.016, -0.048, 0.155, 0.028, 0.105),
    ("santoku_knife",  -0.016,  0.048, 0.145, 0.040, 0.105),
    ("utility_knife",   0.016, -0.024, 0.125, 0.024, 0.090),
    ("paring_knife_0",  0.016,  0.024, 0.092, 0.020, 0.085),
    ("paring_knife_1",  0.016,  0.048, 0.092, 0.020, 0.085),
]

# ── chef knife (the single prismatic slide-in) ───────────────────────────
CHEF_BLADE   = 0.175
CHEF_BLADE_W = 0.040
CHEF_GRIP    = 0.110
CHEF_TRAVEL  = 0.20

# ── shears ───────────────────────────────────────────────────────────────
SHEARS_TRAVEL = 0.10
SHEARS_OPEN   = 0.70          # rad ≈ 40°
PIVOT_Z       = 0.006


# ═════════════════════════════════════════════════════════════════════════
#  geometry helpers
# ═════════════════════════════════════════════════════════════════════════

def _build_housing() -> cq.Workplane:
    """Hollow smoked-acrylic shell, open at the top."""
    outer = (
        cq.Workplane("XY")
        .box(HD, HW, HH)
        .translate((0, 0, HH / 2))
    )
    cav_h = HH - FT
    inner = (
        cq.Workplane("XY")
        .box(CD, CW, cav_h)
        .translate((0, 0, FT + cav_h / 2))
    )
    return outer.cut(inner)


def _build_front_pocket() -> cq.Workplane:
    """Open-top pocket for shears fused to the housing front face."""
    outer = (
        cq.Workplane("XY")
        .box(PK_D, PK_W, PK_H)
        .translate((PK_X, 0, PK_H / 2))
    )
    inner_h = PK_H - WT
    inner_cut = (
        cq.Workplane("XY")
        .box(PK_D - 2 * WT, PK_W - 2 * WT, inner_h)
        .translate((PK_X, 0, WT + inner_h / 2))
    )
    return outer.cut(inner_cut)


def _knife_steel(blade_len: float, blade_w: float) -> cq.Workplane:
    """Tapered blade + bolster, blade extending down −Z."""
    half = blade_w / 2.0
    pts = [
        (-half, 0.015), (half, 0.015),
        (half, -0.45 * blade_len), (-half + 0.003, -blade_len),
    ]
    blade = cq.Workplane("XZ").polyline(pts).close().extrude(0.00125, both=True)
    bolster = cq.Workplane("XY").box(0.013, 0.011, 0.020).translate((0, 0, 0.023))
    return blade.union(bolster)


def _grip_loft(sections):
    """Loft elliptical cross-sections stacked along +Z: (z, x, y, rx, ry)."""
    z0, x0, y0, a0, b0 = sections[0]
    wp = cq.Workplane("XY").workplane(offset=z0).center(x0, y0).ellipse(a0, b0)
    pz, px, py = z0, x0, y0
    for z, x, y, a, b in sections[1:]:
        wp = wp.workplane(offset=z - pz).center(x - px, y - py).ellipse(a, b)
        pz, px, py = z, x, y
    return wp.loft(combine=True)


def _knife_grip(grip_len: float) -> cq.Workplane:
    """Curved walnut handle sweeping back as it rises."""
    z0 = 0.031
    return _grip_loft([
        (z0, 0.000, 0.0, 0.0110, 0.0085),
        (z0 + 0.35 * grip_len, -0.004, 0.0, 0.0130, 0.0100),
        (z0 + 0.75 * grip_len, -0.010, 0.0, 0.0125, 0.0095),
        (z0 + grip_len, -0.016, 0.0, 0.0090, 0.0070),
    ])


def _knife_rivet_x(grip_len: float, frac: float) -> float:
    stations = [(0.0, 0.0), (0.35, -0.004), (0.75, -0.010), (1.0, -0.016)]
    for (f0, x0), (f1, x1) in zip(stations, stations[1:]):
        if frac <= f1:
            t = (frac - f0) / (f1 - f0)
            return x0 + t * (x1 - x0)
    return stations[-1][1]


def _shears_steel(inner: bool) -> cq.Workplane:
    if inner:
        pts = [(-0.008, 0.016), (0.008, 0.016), (0.008, -0.025),
               (0.001, -0.094), (-0.004, -0.094), (-0.008, -0.025)]
        xoff = -0.00225
    else:
        pts = [(-0.008, 0.010), (0.008, 0.010), (0.008, -0.031),
               (0.001, -0.100), (-0.004, -0.100), (-0.008, -0.031)]
        xoff = 0.00225
    half = (
        cq.Workplane("YZ").polyline(pts).close()
        .extrude(0.00175, both=True)
        .translate((xoff, 0, 0))
    )
    if not inner:
        hole = cq.Workplane("YZ").circle(0.0045).extrude(0.02, both=True)
        half = half.cut(hole)
    return half


def _shears_grip(inner: bool) -> cq.Workplane:
    if inner:
        secs = [
            (0.012, -0.00425, 0.0050, 0.00375, 0.0060),
            (0.040, -0.00425, 0.0145, 0.00375, 0.0070),
            (0.070, -0.00425, 0.0235, 0.00375, 0.0070),
            (0.092, -0.00425, 0.0190, 0.00350, 0.0055),
        ]
    else:
        secs = [
            (0.006, 0.00425, -0.0050, 0.00375, 0.0060),
            (0.034, 0.00425, -0.0145, 0.00375, 0.0070),
            (0.064, 0.00425, -0.0235, 0.00375, 0.0070),
            (0.086, 0.00425, -0.0190, 0.00350, 0.0055),
        ]
    return _grip_loft(secs)


# ═════════════════════════════════════════════════════════════════════════
#  build
# ═════════════════════════════════════════════════════════════════════════

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="kitchen_knife_block_set")

    # ── materials ────────────────────────────────────────────────────────
    acrylic  = model.material("smoked_acrylic", rgba=(0.30, 0.33, 0.38, 0.75))
    bristle  = model.material("bristle_rod",    rgba=(0.12, 0.12, 0.13, 1.0))
    walnut   = model.material("walnut",          rgba=(0.32, 0.19, 0.11, 1.0))
    steel    = model.material("stainless_steel", rgba=(0.78, 0.79, 0.82, 1.0))
    rubber   = model.material("rubber",          rgba=(0.08, 0.08, 0.08, 1.0))
    logo_mat = model.material("logo_dark",       rgba=(0.18, 0.20, 0.22, 1.0))

    # ══════════════════════════════════════════════════════════════════════
    #  block  (root part — housing + bristles + feet + logo + fixed knives)
    # ══════════════════════════════════════════════════════════════════════
    block = model.part("knife_block")

    # housing shell (union of main body + front pocket)
    housing_cq = _build_housing().union(_build_front_pocket())
    block.visual(
        mesh_from_cadquery(housing_cq, "housing_shell"),
        origin=Origin(xyz=(0, 0, LIFT)),
        material=acrylic,
        name="housing_shell",
    )

    # four rubber feet (1 mm embedded into the housing floor)
    foot_xy = [(0.040, -0.056), (0.040, 0.056), (-0.040, -0.056), (-0.040, 0.056)]
    for i in range(4):
        fx, fy = foot_xy[i]
        block.visual(
            Cylinder(radius=0.009, length=0.009),
            origin=Origin(xyz=(fx, fy, 0.0045)),
            material=rubber,
            name=f"foot_{i}",
        )

    # logo panel on front face (above pocket)
    logo_x = HD / 2 + 0.001
    logo_z = LIFT + HH * 0.62
    block.visual(
        Box((0.002, 0.060, 0.040)),
        origin=Origin(xyz=(logo_x, 0, logo_z)),
        material=logo_mat,
        name="logo_panel",
    )

    # ── bristle grid (inline visuals, emitted via for-i-in-range loop) ───
    x_start = -CD / 2 + BR_SP
    y_start = -CW / 2 + BR_SP
    bristle_base_z = LIFT + FT - 0.001          # 1 mm root embed into floor
    for i in range(N_BRISTLES):
        ix = i // N_BY
        iy = i % N_BY
        bx = x_start + ix * BR_SP
        by = y_start + iy * BR_SP
        block.visual(
            Cylinder(radius=BR_R, length=BR_H + 0.001),   # +1 mm for embed
            origin=Origin(xyz=(bx, by, bristle_base_z + (BR_H + 0.001) / 2)),
            material=bristle,
            name=f"bristle_{i}",
        )

    # ── fixed knives (inline visuals on block, no separate parts) ────────
    for kname, kx, ky, bl, bw, gl in FIXED_KNIVES:
        block.visual(
            mesh_from_cadquery(_knife_steel(bl, bw), f"{kname}_steel"),
            origin=Origin(xyz=(kx, ky, INSERT_Z + VIS_ZOFF)),
            material=steel,
            name=f"{kname}_steel",
        )
        block.visual(
            mesh_from_cadquery(_knife_grip(gl), f"{kname}_grip"),
            origin=Origin(xyz=(kx, ky, INSERT_Z + VIS_ZOFF)),
            material=walnut,
            name=f"{kname}_grip",
        )
        for j, frac in enumerate((0.30, 0.70)):
            block.visual(
                Cylinder(radius=0.0032, length=0.022),
                origin=Origin(
                    xyz=(kx + _knife_rivet_x(gl, frac), ky,
                         INSERT_Z + VIS_ZOFF + 0.031 + frac * gl),
                    rpy=(math.pi / 2, 0, 0),
                ),
                material=steel,
                name=f"{kname}_rivet_{j}",
            )

    # ══════════════════════════════════════════════════════════════════════
    #  chef knife  (the removable prismatic slide-in)
    # ══════════════════════════════════════════════════════════════════════
    chef = model.part("chef_knife")
    chef.visual(
        mesh_from_cadquery(_knife_steel(CHEF_BLADE, CHEF_BLADE_W), "chef_steel"),
        origin=Origin(xyz=(0, 0, VIS_ZOFF)),
        material=steel,
        name="chef_steel",
    )
    chef.visual(
        mesh_from_cadquery(_knife_grip(CHEF_GRIP), "chef_grip"),
        origin=Origin(xyz=(0, 0, VIS_ZOFF)),
        material=walnut,
        name="chef_grip",
    )
    for j, frac in enumerate((0.30, 0.70)):
        chef.visual(
            Cylinder(radius=0.0032, length=0.022),
            origin=Origin(
                xyz=(_knife_rivet_x(CHEF_GRIP, frac), 0,
                     VIS_ZOFF + 0.031 + frac * CHEF_GRIP),
                rpy=(math.pi / 2, 0, 0),
            ),
            material=steel,
            name=f"chef_rivet_{j}",
        )

    # prismatic slide: origin at housing rim (real contact surface)
    model.articulation(
        "chef_slide",
        ArticulationType.PRISMATIC,
        parent=block,
        child=chef,
        origin=Origin(xyz=(0, 0, INSERT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0, velocity=0.5,
            lower=0.0, upper=CHEF_TRAVEL,
        ),
    )

    # ══════════════════════════════════════════════════════════════════════
    #  shears  (prismatic slide + revolute pivot)
    # ══════════════════════════════════════════════════════════════════════
    shears_origin_z = LIFT + PK_H          # pocket rim (contact surface)

    inner = model.part("shears_inner_half")
    inner.visual(
        mesh_from_cadquery(_shears_steel(inner=True), "shears_inner_steel"),
        material=steel, name="shears_inner_steel",
    )
    inner.visual(
        mesh_from_cadquery(_shears_grip(inner=True), "shears_inner_grip"),
        material=walnut, name="shears_inner_grip",
    )
    inner.visual(
        Cylinder(radius=0.0035, length=0.013),
        origin=Origin(xyz=(0, 0, PIVOT_Z), rpy=(0, math.pi / 2, 0)),
        material=steel, name="shears_pivot_rivet",
    )
    for j, (rz, ry) in enumerate(((0.030, 0.011), (0.065, 0.022))):
        inner.visual(
            Cylinder(radius=0.0022, length=0.0085),
            origin=Origin(xyz=(-0.00425, ry, rz), rpy=(0, math.pi / 2, 0)),
            material=steel, name=f"shears_inner_rivet_{j}",
        )

    outer = model.part("shears_outer_half")
    outer.visual(
        mesh_from_cadquery(_shears_steel(inner=False), "shears_outer_steel"),
        material=steel, name="shears_outer_steel",
    )
    outer.visual(
        mesh_from_cadquery(_shears_grip(inner=False), "shears_outer_grip"),
        material=walnut, name="shears_outer_grip",
    )
    for j, (rz, ry) in enumerate(((0.024, -0.011), (0.059, -0.022))):
        outer.visual(
            Cylinder(radius=0.0022, length=0.0085),
            origin=Origin(xyz=(0.00425, ry, rz), rpy=(0, math.pi / 2, 0)),
            material=steel, name=f"shears_outer_rivet_{j}",
        )

    model.articulation(
        "shears_slide",
        ArticulationType.PRISMATIC,
        parent=block,
        child=inner,
        origin=Origin(xyz=(PK_X, 0, shears_origin_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=15.0, velocity=0.5,
            lower=0.0, upper=SHEARS_TRAVEL,
        ),
    )
    model.articulation(
        "shears_pivot",
        ArticulationType.REVOLUTE,
        parent=inner,
        child=outer,
        origin=Origin(xyz=(0, 0, PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=2.0,
            lower=0.0, upper=SHEARS_OPEN,
        ),
    )

    return model


# ═════════════════════════════════════════════════════════════════════════
#  tests
# ═════════════════════════════════════════════════════════════════════════

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    block  = object_model.get_part("knife_block")
    chef   = object_model.get_part("chef_knife")
    inner  = object_model.get_part("shears_inner_half")
    outer  = object_model.get_part("shears_outer_half")

    chef_slide   = object_model.get_articulation("chef_slide")
    shears_slide = object_model.get_articulation("shears_slide")
    shears_pivot = object_model.get_articulation("shears_pivot")

    # ── intentional-overlap allowances ───────────────────────────────────
    # Chef blade is inserted into the bristle cavity inside the housing.
    ctx.allow_overlap(
        block, chef,
        elem_a="housing_shell", elem_b="chef_steel",
        reason="Chef blade is intentionally inserted into the bristle cavity.",
    )
    # Shears blades sit inside the front pocket cavity.
    ctx.allow_overlap(
        block, inner,
        elem_a="housing_shell", elem_b="shears_inner_steel",
        reason="Shears inner blade sits inside the front pocket cavity.",
    )
    ctx.allow_overlap(
        block, outer,
        elem_a="housing_shell", elem_b="shears_outer_steel",
        reason="Shears outer blade sits inside the front pocket cavity.",
    )

    # ── isolated-part allowances ─────────────────────────────────────────
    ctx.allow_isolated_part(
        chef,
        reason="Chef knife is gravity-held in bristles; bristles flex to grip.",
    )
    ctx.allow_isolated_part(
        inner,
        reason="Shears inner half rests by gravity in the front pocket.",
    )
    ctx.allow_isolated_part(
        outer,
        reason="Shears outer half rides the pivot rivet in the pocket.",
    )

    # ── housing: grounded, correct scale ─────────────────────────────────
    bb = ctx.part_world_aabb(block)
    ctx.check(
        "housing stands on feet near z=0",
        bb is not None and abs(bb[0][2]) < 0.002,
        details=f"aabb={bb}",
    )
    ctx.check(
        "housing is ~0.22 m tall (plus feet + handles)",
        bb is not None and 0.22 < bb[1][2] < 0.40,
        details=f"zmax={bb[1][2] if bb else None}",
    )
    ctx.check(
        "housing is ~0.14 m wide",
        bb is not None and 0.13 < (bb[1][1] - bb[0][1]) < 0.18,
        details=f"width={(bb[1][1] - bb[0][1]) if bb else None}",
    )

    # ── bristle grid: rods present inside the housing ────────────────────
    bristle_0 = ctx.part_element_world_aabb(block, elem="bristle_0")
    ctx.check(
        "bristle_0 exists inside the housing",
        bristle_0 is not None
        and bristle_0[0][2] > 0.0
        and bristle_0[1][2] < LIFT + HH + 0.005,
        details=f"bristle_0 aabb={bristle_0}",
    )
    ctx.expect_within(
        block, block,
        axes="xy",
        inner_elem="bristle_0", outer_elem="housing_shell",
        margin=0.001,
        name="bristle rods sit inside the housing cavity",
    )

    # ── chef knife: prismatic joint ──────────────────────────────────────
    lim = chef_slide.motion_limits
    ctx.check(
        "chef_slide is prismatic with 0.20 m travel",
        chef_slide.articulation_type == ArticulationType.PRISMATIC
        and lim is not None
        and abs(lim.lower) < 1e-9
        and abs(lim.upper - CHEF_TRAVEL) < 1e-6,
        details=f"limits=({lim.lower}, {lim.upper})" if lim else "none",
    )

    # stowed: blade inside housing footprint
    ctx.expect_within(
        chef, block,
        axes="xy",
        inner_elem="chef_steel", outer_elem="housing_shell",
        margin=0.002,
        name="chef blade stays inside housing footprint when stowed",
    )
    ctx.expect_overlap(
        chef, block,
        axes="z",
        elem_a="chef_steel", elem_b="housing_shell",
        min_overlap=0.7 * CHEF_BLADE,
        name="chef blade deeply inserted into bristles when stowed",
    )

    # handle protrudes above housing rim
    grip_aabb = ctx.part_element_world_aabb(chef, elem="chef_grip")
    housing_top_z = LIFT + HH
    ctx.check(
        "chef handle protrudes above housing rim when stowed",
        grip_aabb is not None and grip_aabb[1][2] > housing_top_z + 0.01,
        details=f"grip zmax={grip_aabb[1][2] if grip_aabb else None}",
    )

    # drawn: blade clears housing + moves upward
    rest_steel = ctx.part_element_world_aabb(chef, elem="chef_steel")
    with ctx.pose({chef_slide: CHEF_TRAVEL}):
        drawn_steel = ctx.part_element_world_aabb(chef, elem="chef_steel")
        ctx.check(
            "chef blade clears housing rim at full draw",
            rest_steel is not None and drawn_steel is not None
            and drawn_steel[0][2] > housing_top_z - 0.005,
            details=f"blade zmin={drawn_steel[0][2] if drawn_steel else None}",
        )
        ctx.check(
            "chef knife translates upward when drawn",
            rest_steel is not None and drawn_steel is not None
            and (drawn_steel[1][2] - rest_steel[1][2]) > 0.9 * CHEF_TRAVEL,
            details=f"rest zmax={rest_steel[1][2]}, drawn zmax={drawn_steel[1][2]}",
        )

    # ── fixed knives: inline block visuals, not separate parts ───────────
    for kname, *_ in FIXED_KNIVES:
        steel_aabb = ctx.part_element_world_aabb(block, elem=f"{kname}_steel")
        ctx.check(
            f"{kname} steel is a block visual (inline decoration)",
            steel_aabb is not None,
            details=f"aabb={steel_aabb}",
        )
        grip_aabb = ctx.part_element_world_aabb(block, elem=f"{kname}_grip")
        ctx.check(
            f"{kname} handle protrudes above housing",
            grip_aabb is not None and grip_aabb[1][2] > housing_top_z + 0.005,
            details=f"grip zmax={grip_aabb[1][2] if grip_aabb else None}",
        )

    # ── shears: slide + pivot ────────────────────────────────────────────
    slim = shears_slide.motion_limits
    ctx.check(
        "shears_slide is prismatic with 0.10 m travel",
        shears_slide.articulation_type == ArticulationType.PRISMATIC
        and slim is not None and abs(slim.upper - SHEARS_TRAVEL) < 1e-6,
        details=f"limits=({slim.lower}, {slim.upper})" if slim else "none",
    )
    plim = shears_pivot.motion_limits
    ctx.check(
        "shears_pivot is revolute 0..~40°",
        shears_pivot.articulation_type == ArticulationType.REVOLUTE
        and plim is not None
        and abs(plim.lower) < 1e-9
        and 0.6 < plim.upper < 0.8,
        details=f"limits=({plim.lower}, {plim.upper})" if plim else "none",
    )

    # stowed: shears blade in pocket
    ctx.expect_overlap(
        inner, block,
        axes="z",
        elem_a="shears_inner_steel", elem_b="housing_shell",
        min_overlap=0.04,
        name="stowed shears blades overlap pocket in Z",
    )

    # slide out
    rest_pos = ctx.part_world_position(inner)
    rest_handle = ctx.part_element_world_aabb(outer, elem="shears_outer_grip")
    with ctx.pose({shears_slide: SHEARS_TRAVEL}):
        out_pos = ctx.part_world_position(inner)
        ctx.check(
            "shears slide upward out of pocket",
            rest_pos is not None and out_pos is not None
            and (out_pos[2] - rest_pos[2]) > 0.9 * SHEARS_TRAVEL,
            details=f"rest={rest_pos}, out={out_pos}",
        )
        with ctx.pose({shears_slide: SHEARS_TRAVEL, shears_pivot: 0.6}):
            open_handle = ctx.part_element_world_aabb(outer, elem="shears_outer_grip")
            ctx.check(
                "pivot swings outer handle open",
                rest_handle is not None and open_handle is not None
                and open_handle[0][1] < rest_handle[0][1] - 0.012,
                details=(
                    f"rest ymin={rest_handle[0][1]}, "
                    f"open ymin={open_handle[0][1]}"
                ),
            )

    return ctx.report()


object_model = build_object_model()
