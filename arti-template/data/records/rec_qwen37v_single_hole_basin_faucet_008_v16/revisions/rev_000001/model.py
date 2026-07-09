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
# Single-hole basin faucet variant, ~0.13 m tall, mirror chrome.
# World frame: +Z up, deck at z = 0, spout points toward +X (front).
# Body tilts back a few degrees (long axis toward -X).
# Variant features:
#   - Offset side lever housing (+Y) with revolute lever
#   - Flip-open aerator on tiny hinge at spout outlet
#   - Subtle grip grooves on the barrel
#   - Two small screw caps on the body back (-X)
# ---------------------------------------------------------------------------

TILT = math.radians(6.0)
SIN_T = math.sin(TILT)
COS_T = math.cos(TILT)

# Base flange.
FLANGE_R = 0.030
FLANGE_H = 0.006

# Main body barrel.
BODY_R = 0.025
BODY_S0 = 0.006
BODY_S1 = 0.0725

# Separation groove ring.
GROOVE_R = 0.0215
GROOVE_S0 = 0.0705
GROOVE_S1 = 0.0760

# Upper neck.
NECK_R = 0.0228
NECK_S0 = 0.0740
NECK_S1 = 0.104

# Neck cap (fixed dome cap on top of the neck, replacing the push-cap).
NECK_CAP_R = 0.023
NECK_CAP_H = 0.006

# Grip grooves (3 thin rings on the lower barrel, well below the spout exit).
GRIP_STATIONS = [0.015, 0.022, 0.029]
GRIP_R = 0.0235
GRIP_LEN = 0.002

# Screw caps on the back of the body.
SCREW_S = 0.045
SCREW_R = 0.004
SCREW_H = 0.002
SCREW_Y_OFFSETS = [0.009, -0.009]

# Spout.
SPOUT_S = 0.050

# Side lever housing (offset to +Y).
LEVER_S = 0.052
HOUSING_R = 0.013
HOUSING_LEN = 0.020

# Lever handle.
LEVER_HANDLE_R = 0.006
LEVER_HANDLE_LEN = 0.075
LEVER_BOSS_R = 0.009
LEVER_BOSS_LEN = 0.006

# Lever pivot limits.
LEVER_LOWER = -0.10
LEVER_UPPER = 0.65

# Aerator hinge.
AERATOR_R = 0.013
AERATOR_H = 0.003
HINGE_BARREL_R = 0.0025
HINGE_BARREL_LEN = 0.014
AERATOR_LOWER = 0.0
AERATOR_UPPER = math.radians(85.0)


def _axis_point(s: float) -> tuple[float, float, float]:
    """World position of the tilted body axis at axial station s."""
    return (-s * SIN_T, 0.0, s * COS_T)


def _tilted(s: float) -> Origin:
    """Origin on the body axis at station s, z-axis aligned with the axis."""
    return Origin(xyz=_axis_point(s), rpy=(0.0, -TILT, 0.0))


def _build_spout_shape() -> cq.Workplane:
    """Hollow chrome spout: straight shank, smooth downward bend, flared
    open outlet rim."""
    r_out = 0.015
    shank_x0 = 0.010
    shank_x1 = 0.035
    bend = 0.028
    end_x = shank_x1 + bend
    end_z = -bend

    path = (
        cq.Workplane("XZ")
        .moveTo(shank_x0, 0.0)
        .lineTo(shank_x1, 0.0)
        .tangentArcPoint((bend, -bend), relative=True)
    )
    tube = cq.Workplane("YZ", origin=(shank_x0, 0.0, 0.0)).circle(r_out).sweep(path)

    flare = (
        cq.Workplane("XY", origin=(end_x, 0.0, end_z + 0.006))
        .circle(0.0148)
        .workplane(offset=-0.010)
        .circle(0.0185)
        .loft()
    )
    spout = tube.union(flare)

    bore = (
        cq.Workplane("XY", origin=(end_x, 0.0, end_z - 0.006))
        .circle(0.0155)
        .workplane(offset=0.018)
        .circle(0.011)
        .loft()
    )
    return spout.cut(bore)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hole_basin_faucet_lever")

    model.material("chrome", rgba=(0.85, 0.87, 0.89, 1.0))
    model.material("chrome_dark", rgba=(0.42, 0.44, 0.47, 1.0))
    model.material("chrome_brushed", rgba=(0.70, 0.72, 0.74, 1.0))
    model.material("screw_cap", rgba=(0.35, 0.36, 0.38, 1.0))
    model.material("aerator_mesh", rgba=(0.55, 0.57, 0.60, 1.0))

    # ---------------- body (root): flange + barrel + grooves + neck --------
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
    # Fixed cap on top of the neck.
    body.visual(
        Cylinder(radius=NECK_CAP_R, length=NECK_CAP_H),
        origin=_tilted(NECK_S1 + NECK_CAP_H / 2.0),
        material="chrome_brushed",
        name="neck_cap",
    )

    # Grip grooves: subtle dark rings on the lower barrel.
    for i, gs in enumerate(GRIP_STATIONS):
        body.visual(
            Cylinder(radius=GRIP_R, length=GRIP_LEN),
            origin=_tilted(gs),
            material="chrome_dark",
            name=f"grip_groove_{i}",
        )

    # Screw caps on the back of the body (-X side).
    # Embed ~1 mm into the barrel surface for geometric connectivity.
    screw_pt = _axis_point(SCREW_S)
    for i, dy in enumerate(SCREW_Y_OFFSETS):
        body.visual(
            Cylinder(radius=SCREW_R, length=SCREW_H),
            origin=Origin(
                xyz=(screw_pt[0] - BODY_R + 0.001, screw_pt[1] + dy, screw_pt[2]),
                rpy=(0.0, -math.pi / 2.0, 0.0),
            ),
            material="screw_cap",
            name=f"screw_cap_{i}",
        )

    # ---------------- spout (fixed): swept tube + flared outlet ------------
    spout = model.part("spout")
    spout.visual(
        mesh_from_cadquery(_build_spout_shape(), "spout", tolerance=0.0003),
        material="chrome",
        name="spout_tube",
    )
    # Tiny hinge barrel on the spout for the aerator flip.
    # Located at the back edge of the spout outlet.
    spout_end_x = 0.063
    spout_end_z = -0.028
    hinge_x = spout_end_x - AERATOR_R
    spout.visual(
        Cylinder(radius=HINGE_BARREL_R, length=HINGE_BARREL_LEN),
        origin=Origin(
            xyz=(hinge_x, 0.0, spout_end_z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material="chrome_dark",
        name="hinge_barrel",
    )
    model.articulation(
        "spout_mount",
        ArticulationType.FIXED,
        parent=body,
        child=spout,
        origin=Origin(xyz=_axis_point(SPOUT_S)),
    )

    # ---------------- aerator (revolute hinge at spout outlet) -------------
    aerator = model.part("aerator")
    # Aerator disc: flat coin that covers the outlet when closed.
    # Part origin is at the hinge pin; disc extends +X from the pin.
    aerator.visual(
        Cylinder(radius=AERATOR_R, length=AERATOR_H),
        origin=Origin(xyz=(AERATOR_R, 0.0, 0.0)),
        material="aerator_mesh",
        name="aerator_disc",
    )
    # Tiny knuckle around the hinge pin.
    aerator.visual(
        Cylinder(radius=HINGE_BARREL_R - 0.0003, length=HINGE_BARREL_LEN - 0.004),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="chrome",
        name="aerator_knuckle",
    )
    model.articulation(
        "aerator_hinge",
        ArticulationType.REVOLUTE,
        parent=spout,
        child=aerator,
        origin=Origin(xyz=(hinge_x, 0.0, spout_end_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0, velocity=2.0, lower=AERATOR_LOWER, upper=AERATOR_UPPER
        ),
    )

    # ---------------- lever housing (fixed, offset to +Y side) -------------
    lever_housing = model.part("lever_housing")
    housing_pt = _axis_point(LEVER_S)
    # Housing part frame sits at the housing center, offset +Y from body axis.
    housing_y_offset = BODY_R + HOUSING_LEN / 2.0
    model.articulation(
        "housing_mount",
        ArticulationType.FIXED,
        parent=body,
        child=lever_housing,
        origin=Origin(xyz=(housing_pt[0], housing_pt[1] + housing_y_offset, housing_pt[2])),
    )
    # Housing boss: cylinder along local +Y, centered at the part frame origin.
    lever_housing.visual(
        Cylinder(radius=HOUSING_R, length=HOUSING_LEN),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="chrome",
        name="housing_boss",
    )

    # ---------------- lever (revolute pivot for flow control) --------------
    lever = model.part("lever")
    # Pivot boss at the outer face of the housing.
    lever.visual(
        Cylinder(radius=LEVER_BOSS_R, length=LEVER_BOSS_LEN),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material="chrome",
        name="lever_boss",
    )
    # Elongated handle extending +Y from the pivot.
    lever.visual(
        Cylinder(radius=LEVER_HANDLE_R, length=LEVER_HANDLE_LEN),
        origin=Origin(
            xyz=(0.0, LEVER_BOSS_LEN / 2.0 + LEVER_HANDLE_LEN / 2.0, 0.0),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material="chrome",
        name="lever_handle",
    )
    # Small grip ridge on the lever tip.
    lever.visual(
        Cylinder(radius=LEVER_HANDLE_R + 0.002, length=0.008),
        origin=Origin(
            xyz=(0.0, LEVER_BOSS_LEN / 2.0 + LEVER_HANDLE_LEN - 0.004, 0.0),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material="chrome_brushed",
        name="lever_grip_ridge",
    )
    # Lever pivot at the outer face of the housing; axis along X so
    # positive q raises the lever tip (+Y side) upward (+Z).
    pivot_y_in_housing = HOUSING_LEN / 2.0
    model.articulation(
        "lever_pivot",
        ArticulationType.REVOLUTE,
        parent=lever_housing,
        child=lever,
        origin=Origin(xyz=(0.0, pivot_y_in_housing, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=2.0, lower=LEVER_LOWER, upper=LEVER_UPPER
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    spout = object_model.get_part("spout")
    aerator = object_model.get_part("aerator")
    lever_housing = object_model.get_part("lever_housing")
    lever = object_model.get_part("lever")
    aerator_hinge = object_model.get_articulation("aerator_hinge")
    lever_pivot = object_model.get_articulation("lever_pivot")

    # --- Intentional overlap: spout shank seated in body -------------------
    ctx.allow_overlap(
        spout,
        body,
        elem_a="spout_tube",
        elem_b="body_barrel",
        reason="Spout shank is intentionally seated ~15 mm into the solid body casting.",
    )

    # --- Hero geometry: flange on deck, body leans back --------------------
    flange_aabb = ctx.part_element_world_aabb(body, elem="base_flange")
    ctx.check(
        "base flange sits flat on the deck",
        flange_aabb is not None and abs(flange_aabb[0][2]) <= 0.0005,
        details=f"flange aabb={flange_aabb}",
    )
    neck_aabb = ctx.part_element_world_aabb(body, elem="body_neck")
    ctx.check(
        "body leans back (neck offset toward -X behind flange center)",
        neck_aabb is not None and (neck_aabb[0][0] + neck_aabb[1][0]) / 2.0 < -0.005,
        details=f"neck aabb={neck_aabb}",
    )

    # --- Grip grooves present on the body barrel ---------------------------
    for i in range(len(GRIP_STATIONS)):
        gv = body.get_visual(f"grip_groove_{i}")
        ctx.check(
            f"grip groove {i} exists on the body",
            gv is not None,
            details="missing grip groove visual",
        )

    # --- Screw caps on the back of the body --------------------------------
    for i in range(len(SCREW_Y_OFFSETS)):
        sv = body.get_visual(f"screw_cap_{i}")
        ctx.check(
            f"screw cap {i} exists on body back",
            sv is not None,
            details="missing screw cap visual",
        )
    # Screw caps should be on the -X side (behind the body axis).
    screw0_aabb = ctx.part_element_world_aabb(body, elem="screw_cap_0")
    barrel_aabb = ctx.part_element_world_aabb(body, elem="body_barrel")
    ctx.check(
        "screw caps are on the back (-X side) of the body",
        screw0_aabb is not None
        and barrel_aabb is not None
        and (screw0_aabb[0][0] + screw0_aabb[1][0]) / 2.0
        < (barrel_aabb[0][0] + barrel_aabb[1][0]) / 2.0 - 0.010,
        details=f"screw0 aabb={screw0_aabb}, barrel aabb={barrel_aabb}",
    )

    # --- Spout: projects forward and curves down ---------------------------
    ctx.expect_overlap(
        spout,
        body,
        axes="xz",
        min_overlap=0.005,
        name="spout shank stays seated in the body",
    )
    spout_aabb = ctx.part_world_aabb(spout)
    ctx.check(
        "spout reaches forward and droops to a low outlet above the deck",
        spout_aabb is not None
        and spout_aabb[1][0] > 0.060
        and spout_aabb[0][2] < 0.025
        and spout_aabb[0][2] > 0.008,
        details=f"spout aabb={spout_aabb}",
    )

    # --- Lever housing is offset to the side (+Y) -------------------------
    housing_aabb = ctx.part_world_aabb(lever_housing)
    ctx.check(
        "lever housing is offset to the +Y side of the body",
        housing_aabb is not None and housing_aabb[0][1] > 0.015,
        details=f"housing aabb={housing_aabb}",
    )
    # Housing should not penetrate the deck (single deck penetration only).
    ctx.check(
        "lever housing stays above the deck",
        housing_aabb is not None and housing_aabb[0][2] > 0.010,
        details=f"housing aabb={housing_aabb}",
    )

    # --- Lever pivot: limits and decisive pose -----------------------------
    lp = lever_pivot.motion_limits
    ctx.check(
        "lever pivot has non-trivial revolute limits",
        lp is not None
        and lp.lower is not None
        and lp.upper is not None
        and lp.upper > lp.lower + 0.2,
        details=f"limits={lp}",
    )
    lever_handle_rest = ctx.part_element_world_aabb(lever, elem="lever_handle")
    with ctx.pose({lever_pivot: LEVER_UPPER}):
        lever_handle_raised = ctx.part_element_world_aabb(lever, elem="lever_handle")
    ctx.check(
        "lever pivot positive pose raises the lever tip",
        lever_handle_rest is not None
        and lever_handle_raised is not None
        and (lever_handle_raised[0][2] + lever_handle_raised[1][2]) / 2.0
        > (lever_handle_rest[0][2] + lever_handle_rest[1][2]) / 2.0 + 0.005,
        details=f"rest={lever_handle_rest}, raised={lever_handle_raised}",
    )

    # --- Aerator hinge: limits and decisive pose --------------------------
    ah = aerator_hinge.motion_limits
    ctx.check(
        "aerator hinge has non-trivial revolute limits",
        ah is not None
        and ah.lower is not None
        and ah.upper is not None
        and ah.upper > ah.lower + 0.3,
        details=f"limits={ah}",
    )
    # At rest (q=0) the aerator disc should overlap the spout outlet in XY.
    ctx.expect_overlap(
        aerator,
        spout,
        axes="xy",
        min_overlap=0.005,
        name="aerator disc covers the spout outlet when closed",
    )
    aerator_rest_center = ctx.part_element_world_aabb(aerator, elem="aerator_disc")
    with ctx.pose({aerator_hinge: AERATOR_UPPER}):
        aerator_open_center = ctx.part_element_world_aabb(aerator, elem="aerator_disc")
    ctx.check(
        "aerator hinge opens the disc away from the spout outlet",
        aerator_rest_center is not None
        and aerator_open_center is not None
        and abs(
            (aerator_open_center[0][2] + aerator_open_center[1][2]) / 2.0
            - (aerator_rest_center[0][2] + aerator_rest_center[1][2]) / 2.0
        )
        > 0.008,
        details=f"rest={aerator_rest_center}, open={aerator_open_center}",
    )

    # --- Overall height check ----------------------------------------------
    neck_cap_aabb = ctx.part_element_world_aabb(body, elem="neck_cap")
    ctx.check(
        "overall faucet height is about 0.11 to 0.14 m",
        neck_cap_aabb is not None and 0.105 <= neck_cap_aabb[1][2] <= 0.145,
        details=f"neck_cap aabb={neck_cap_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
