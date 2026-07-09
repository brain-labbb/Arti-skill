from __future__ import annotations

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

# ---------------------------------------------------------------------------
# Single-hole basin faucet variant, ~0.13 m tall, polished chrome.
# World frame: +Z up, deck at z = 0, spout points toward +X (front).
# Body leans back a few degrees (tilted axis toward -X).
# Side lever on the +Y side; drain rod behind the body (-X).
# ---------------------------------------------------------------------------

TILT = math.radians(5.0)
SIN_T = math.sin(TILT)
COS_T = math.cos(TILT)

# Base flange.
FLANGE_R = 0.028
FLANGE_H = 0.005

# Main body barrel.
BODY_R = 0.022
BODY_S0 = 0.005
BODY_S1 = 0.085

# Upper neck (slightly narrower above a decorative groove).
NECK_R = 0.019
NECK_S0 = 0.082
NECK_S1 = 0.110

# Decorative groove ring.
GROOVE_R = 0.0175
GROOVE_S0 = 0.080
GROOVE_S1 = 0.084

# Spout exit station.
SPOUT_S = 0.055

# Lever housing: cylindrical boss on +Y side of body at upper portion.
HOUSING_S = 0.075  # axial station of housing center
HOUSING_R = 0.013
HOUSING_LEN = 0.022  # protrusion from body surface

# Cartridge cap seam: thin dark ring just below the housing.
SEAM_S = 0.068
SEAM_R = BODY_R + 0.001
SEAM_H = 0.0015

# Lever handle dimensions.
LEVER_LEN = 0.065  # length of handle from pivot
LEVER_W = 0.012  # width
LEVER_H = 0.009  # height (thin flat handle)
LEVER_TIP_R = 0.006  # rounded tip

# Drain rod.
DRAIN_R = 0.003
DRAIN_LEN = 0.075
DRAIN_KNOB_R = 0.006
DRAIN_KNOB_H = 0.008
DRAIN_S = 0.040  # height at which the rod exits the body rear

# Lever pivot limits.
LEVER_LOWER = math.radians(-25.0)  # down position (closed)
LEVER_UPPER = math.radians(65.0)  # up position (full flow)

# Drain rod travel.
DRAIN_TRAVEL = 0.035


def _axis_point(s: float) -> tuple[float, float, float]:
    """World position of the tilted body axis at axial station s."""
    return (-s * SIN_T, 0.0, s * COS_T)


def _tilted(s: float) -> Origin:
    """Origin on the body axis at station s, z-axis aligned with the axis."""
    return Origin(xyz=_axis_point(s), rpy=(0.0, -TILT, 0.0))


def _build_spout_shape() -> cq.Workplane:
    """Hollow chrome spout: straight shank + smooth downward bend + flared
    outlet rim. Local origin on the body axis at SPOUT_S; shank runs +X."""
    r_out = 0.012
    shank_x0 = 0.008
    shank_x1 = 0.035
    bend = 0.025
    end_x = shank_x1 + bend
    end_z = -bend

    path = (
        cq.Workplane("XZ")
        .moveTo(shank_x0, 0.0)
        .lineTo(shank_x1, 0.0)
        .tangentArcPoint((bend, -bend), relative=True)
    )
    tube = cq.Workplane("YZ", origin=(shank_x0, 0.0, 0.0)).circle(r_out).sweep(path)

    # Flared outlet.
    flare = (
        cq.Workplane("XY", origin=(end_x, 0.0, end_z + 0.005))
        .circle(0.0118)
        .workplane(offset=-0.008)
        .circle(0.015)
        .loft()
    )
    spout = tube.union(flare)

    # Bore opening at the outlet mouth.
    bore = (
        cq.Workplane("XY", origin=(end_x, 0.0, end_z - 0.005))
        .circle(0.0125)
        .workplane(offset=0.015)
        .circle(0.009)
        .loft()
    )
    return spout.cut(bore)


def _build_lever_shape() -> cq.Workplane:
    """Lever handle with subtle grip grooves on the top surface.
    Lever-local: pivot at origin, handle extends along +X, width along Y.
    The grip grooves are shallow channels cut across the top face."""
    # Main handle body - slightly tapered.
    handle = (
        cq.Workplane("XY")
        .center(LEVER_LEN / 2.0, 0.0)
        .rect(LEVER_LEN, LEVER_W)
        .extrude(LEVER_H)
    )

    # Rounded tip.
    tip = (
        cq.Workplane("XY", origin=(LEVER_LEN, 0.0, LEVER_H / 2.0))
        .sphere(LEVER_TIP_R)
    )
    handle = handle.union(tip)

    # Pivot boss (cylindrical socket area at the base).
    boss = (
        cq.Workplane("XY", origin=(0.0, 0.0, LEVER_H / 2.0))
        .circle(0.008)
        .extrude(LEVER_H / 2.0)
    )
    handle = handle.union(boss)

    # Grip grooves: 5 shallow channels across the top surface.
    groove_depth = 0.001
    groove_width = 0.002
    for i in range(5):
        x_pos = 0.020 + i * 0.010
        cutter = (
            cq.Workplane("XY", origin=(x_pos, 0.0, LEVER_H))
            .rect(groove_width, LEVER_W * 0.8)
            .extrude(groove_depth + 0.001)
        )
        handle = handle.cut(cutter)

    # Fillet edges for a manufactured look.
    try:
        handle = handle.edges("|Z").fillet(0.001)
    except Exception:
        pass

    return handle


def _build_housing_shape() -> cq.Workplane:
    """Cylindrical lever housing boss with a cartridge cap seam ring at the base.
    Housing local: z=0 at the body surface, extends along +Z (outward)."""
    # Main cylindrical boss.
    boss = cq.Workplane("XY").circle(HOUSING_R).extrude(HOUSING_LEN)

    # Flared base ring for visual transition to the body.
    base_ring = (
        cq.Workplane("XY")
        .circle(HOUSING_R + 0.003)
        .workplane(offset=0.003)
        .circle(HOUSING_R)
        .loft()
    )
    housing = boss.union(base_ring)

    # Socket bore (visible dark hole where lever inserts).
    bore = (
        cq.Workplane("XY", origin=(0.0, 0.0, HOUSING_LEN - 0.002))
        .circle(0.007)
        .extrude(0.005)
    )
    housing = housing.cut(bore)

    return housing


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hole_basin_faucet_side_lever")

    model.material("chrome", rgba=(0.85, 0.87, 0.89, 1.0))
    model.material("chrome_dark", rgba=(0.35, 0.37, 0.40, 1.0))
    model.material("chrome_brushed", rgba=(0.70, 0.72, 0.74, 1.0))
    model.material("seam_dark", rgba=(0.20, 0.22, 0.24, 1.0))

    # ---------------- body (root): flange + barrel + groove + neck ---------
    body = model.part("body")
    body.visual(
        Cylinder(radius=FLANGE_R, length=FLANGE_H),
        origin=Origin(xyz=(0.0, 0.0, FLANGE_H / 2.0)),
        material="chrome",
        name="base_flange",
    )
    body.visual(
        Cylinder(radius=BODY_R, length=BODY_S1 - BODY_S0),
        origin=_tilted((BODY_S0 + BODY_S1) / 2.0),
        material="chrome",
        name="body_barrel",
    )
    body.visual(
        Cylinder(radius=GROOVE_R, length=GROOVE_S1 - GROOVE_S0),
        origin=_tilted((GROOVE_S0 + GROOVE_S1) / 2.0),
        material="chrome_dark",
        name="groove_ring",
    )
    body.visual(
        Cylinder(radius=NECK_R, length=NECK_S1 - NECK_S0),
        origin=_tilted((NECK_S0 + NECK_S1) / 2.0),
        material="chrome",
        name="body_neck",
    )
    # Cartridge cap seam: thin dark ring on the body just below lever housing.
    body.visual(
        Cylinder(radius=SEAM_R, length=SEAM_H),
        origin=_tilted(SEAM_S),
        material="seam_dark",
        name="cartridge_seam",
    )
    # Drain rod guide boss: vertical sleeve on the rear of the body.
    _drain_pt = _axis_point(DRAIN_S)
    _drain_x = _drain_pt[0] - BODY_R - 0.003
    body.visual(
        Cylinder(radius=0.005, length=0.015),
        origin=Origin(
            xyz=(_drain_x, 0.0, _drain_pt[2] + 0.0075),
        ),
        material="chrome",
        name="drain_guide_boss",
    )
    # Small bracket connecting the guide boss to the body rear surface.
    body.visual(
        Box((0.006, 0.008, 0.012)),
        origin=Origin(
            xyz=(_drain_x + 0.003, 0.0, _drain_pt[2] + 0.0075),
        ),
        material="chrome",
        name="drain_bracket",
    )
    # Neck top cap (dome-like closure).
    body.visual(
        Cylinder(radius=NECK_R - 0.002, length=0.004),
        origin=_tilted(NECK_S1 + 0.002),
        material="chrome_brushed",
        name="neck_cap",
    )

    # ---------------- spout (fixed): swept tube + flared outlet ------------
    spout = model.part("spout")
    spout.visual(
        mesh_from_cadquery(_build_spout_shape(), "spout", tolerance=0.0003),
        material="chrome",
        name="spout_tube",
    )
    model.articulation(
        "spout_mount",
        ArticulationType.FIXED,
        parent=body,
        child=spout,
        origin=Origin(xyz=_axis_point(SPOUT_S)),
    )

    # ---------------- lever housing (fixed to body, offset +Y) -------------
    housing = model.part("lever_housing")
    housing.visual(
        mesh_from_cadquery(_build_housing_shape(), "lever_housing", tolerance=0.0003),
        material="chrome",
        name="housing_boss",
    )
    # Mount the housing on the +Y side of the body at HOUSING_S.
    # rpy (-pi/2, 0, 0): housing local +Z → world +Y (outward from body).
    housing_pt = _axis_point(HOUSING_S)
    model.articulation(
        "housing_mount",
        ArticulationType.FIXED,
        parent=body,
        child=housing,
        origin=Origin(
            xyz=(housing_pt[0], BODY_R + 0.001, housing_pt[2]),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
    )

    # ---------------- lever (revolute pivot in the housing) ----------------
    lever = model.part("lever")
    lever.visual(
        mesh_from_cadquery(_build_lever_shape(), "lever", tolerance=0.0003),
        material="chrome",
        name="lever_handle",
    )
    # The lever handle extends along +X (forward) from the housing tip.
    # Joint frame has housing orientation: X_j=X_w, Y_j=-Z_w, Z_j=Y_w.
    # axis (0,0,-1) = -Z_j = -Y_w: right-hand rule rotates +X toward +Z_w.
    # Positive q lifts the handle upward (flow on).
    model.articulation(
        "lever_pivot",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=lever,
        origin=Origin(
            xyz=(0.0, 0.0, HOUSING_LEN - 0.002),
            rpy=(0.0, 0.0, 0.0),
        ),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=3.0, lower=LEVER_LOWER, upper=LEVER_UPPER
        ),
    )

    # ---------------- drain rod (prismatic, slides vertically) -------------
    drain = model.part("drain_rod")
    drain.visual(
        Cylinder(radius=DRAIN_R, length=DRAIN_LEN),
        origin=Origin(xyz=(0.0, 0.0, DRAIN_LEN / 2.0)),
        material="chrome",
        name="drain_shaft",
    )
    drain.visual(
        Cylinder(radius=DRAIN_KNOB_R, length=DRAIN_KNOB_H),
        origin=Origin(xyz=(0.0, 0.0, DRAIN_LEN + DRAIN_KNOB_H / 2.0)),
        material="chrome_brushed",
        name="drain_knob",
    )
    # Drain rod frame at the guide boss, extends upward.
    # The rod slides along world +Z (pull up to open drain).
    model.articulation(
        "drain_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drain,
        origin=Origin(
            xyz=(_drain_x, 0.0, _drain_pt[2]),
            rpy=(0.0, 0.0, 0.0),
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=10.0, velocity=0.1, lower=0.0, upper=DRAIN_TRAVEL
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    spout = object_model.get_part("spout")
    housing = object_model.get_part("lever_housing")
    lever = object_model.get_part("lever")
    drain = object_model.get_part("drain_rod")
    pivot = object_model.get_articulation("lever_pivot")
    slide = object_model.get_articulation("drain_slide")

    # Intentional seated insertions (scoped).
    ctx.allow_overlap(
        spout,
        body,
        elem_a="spout_tube",
        elem_b="body_barrel",
        reason="Spout shank is intentionally seated inside the solid body barrel.",
    )
    ctx.allow_overlap(
        housing,
        body,
        elem_a="housing_boss",
        elem_b="body_barrel",
        reason="Lever housing boss base blends into the body surface for a flush mount.",
    )
    ctx.allow_overlap(
        drain,
        body,
        elem_a="drain_shaft",
        elem_b="drain_guide_boss",
        reason="Drain rod slides through the guide boss on the body rear.",
    )

    # ---- Base flange sits on deck ----
    flange_aabb = ctx.part_element_world_aabb(body, elem="base_flange")
    ctx.check(
        "base flange sits flat on the deck",
        flange_aabb is not None and abs(flange_aabb[0][2]) <= 0.001,
        details=f"flange aabb={flange_aabb}",
    )

    # ---- Single deck penetration: only the flange touches z=0 ----
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "body extends from deck upward (single-hole installation)",
        body_aabb is not None
        and body_aabb[0][2] >= -0.001
        and body_aabb[1][2] > 0.10,
        details=f"body aabb={body_aabb}",
    )

    # ---- Cartridge seam exists below lever housing ----
    seam_aabb = ctx.part_element_world_aabb(body, elem="cartridge_seam")
    housing_aabb = ctx.part_world_aabb(housing)
    seam_center_z = (seam_aabb[0][2] + seam_aabb[1][2]) / 2.0 if seam_aabb else 0
    housing_center_z = (housing_aabb[0][2] + housing_aabb[1][2]) / 2.0 if housing_aabb else 0
    ctx.check(
        "cartridge cap seam ring center is below lever housing center",
        seam_aabb is not None and housing_aabb is not None
        and seam_center_z < housing_center_z,
        details=f"seam_center_z={seam_center_z:.4f}, housing_center_z={housing_center_z:.4f}",
    )

    # ---- Lever housing is offset to the side (+Y) ----
    ctx.check(
        "lever housing is offset to the +Y side of the body",
        housing_aabb is not None
        and housing_aabb[0][1] > 0.010,
        details=f"housing aabb={housing_aabb}",
    )

    # ---- Spout projects forward and curves down ----
    ctx.expect_overlap(
        spout,
        body,
        axes="xz",
        min_overlap=0.003,
        name="spout shank stays seated in the body",
    )
    spout_aabb = ctx.part_world_aabb(spout)
    ctx.check(
        "spout reaches forward and curves to a low outlet above the deck",
        spout_aabb is not None
        and spout_aabb[1][0] > 0.050
        and spout_aabb[0][2] < 0.030
        and spout_aabb[0][2] > 0.005,
        details=f"spout aabb={spout_aabb}",
    )

    # ---- Lever pivot: limits match expected range ----
    pl = pivot.motion_limits
    ctx.check(
        "lever pivot limits are -25 to +65 degrees",
        pl is not None
        and pl.lower is not None
        and pl.upper is not None
        and abs(pl.lower - LEVER_LOWER) < 1e-6
        and abs(pl.upper - LEVER_UPPER) < 1e-6,
        details=f"limits={pl}",
    )

    # ---- Lever handle has grip grooves (visible width < full lever width) ----
    lever_aabb = ctx.part_world_aabb(lever)
    ctx.check(
        "lever handle has a reasonable size for a faucet lever",
        lever_aabb is not None
        and (lever_aabb[1][0] - lever_aabb[0][0]) > 0.04
        and (lever_aabb[1][1] - lever_aabb[0][1]) > 0.005,
        details=f"lever aabb={lever_aabb}",
    )

    # ---- Decisive pose: lever lifts up ----
    lever_rest_aabb = ctx.part_world_aabb(lever)
    with ctx.pose({pivot: LEVER_UPPER}):
        lever_up_aabb = ctx.part_world_aabb(lever)
    ctx.check(
        "lever lifts upward at positive pivot angle (flow on)",
        lever_rest_aabb is not None
        and lever_up_aabb is not None
        and lever_up_aabb[1][2] > lever_rest_aabb[1][2] + 0.005,
        details=f"rest_max_z={lever_rest_aabb[1][2] if lever_rest_aabb else None}, "
                f"up_max_z={lever_up_aabb[1][2] if lever_up_aabb else None}",
    )

    # ---- Drain rod: prismatic limits ----
    sl = slide.motion_limits
    ctx.check(
        "drain rod travel is 0 to 35 mm",
        sl is not None
        and sl.lower is not None
        and sl.upper is not None
        and abs(sl.lower) < 1e-9
        and abs(sl.upper - DRAIN_TRAVEL) < 1e-9,
        details=f"limits={sl}",
    )

    # ---- Drain rod is behind the body (-X side) ----
    drain_aabb = ctx.part_world_aabb(drain)
    ctx.check(
        "drain rod is positioned behind the body (negative X side)",
        drain_aabb is not None
        and drain_aabb[0][0] < -0.010,
        details=f"drain aabb={drain_aabb}",
    )

    # ---- Decisive pose: drain rod slides up ----
    drain_rest = ctx.part_world_position(drain)
    with ctx.pose({slide: DRAIN_TRAVEL}):
        drain_up = ctx.part_world_position(drain)
    ctx.check(
        "drain rod slides upward when pulled",
        drain_rest is not None
        and drain_up is not None
        and drain_up[2] > drain_rest[2] + 0.025,
        details=f"rest={drain_rest}, pulled={drain_up}",
    )

    # ---- Overall height check ----
    ctx.check(
        "overall faucet height is about 0.13 m",
        body_aabb is not None and 0.11 <= body_aabb[1][2] <= 0.14,
        details=f"body aabb top={body_aabb[1][2] if body_aabb else None}",
    )

    return ctx.report()


object_model = build_object_model()
