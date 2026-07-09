from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Single-hole basin faucet, ~0.13 m tall, polished chrome.
# World frame: +Z up, deck at z = 0, spout points toward +X (front).
# The body is a vertical cylindrical column on a round base flange.
# A lever on top rotates side-to-side (temperature) and tilts up (flow).
# ---------------------------------------------------------------------------

# Base flange (sits flat on the deck).
FLANGE_R = 0.030
FLANGE_H = 0.006

# Main body column (vertical).
COLUMN_R = 0.022
COLUMN_H = 0.090
COLUMN_Z0 = FLANGE_H  # column starts on top of the flange
COLUMN_Z1 = COLUMN_Z0 + COLUMN_H  # top of column

# Decorative ring near the top of the column.
RING_R = 0.025
RING_H = 0.004
RING_Z0 = COLUMN_Z1 - 0.012

# Spout exit: center of the spout bore on the column surface.
SPOUT_Z = COLUMN_Z0 + 0.050  # mid-height of column
SPOUT_R = 0.012  # spout tube outer radius

# Lever pivot sits on top of the column.
LEVER_PIVOT_Z = COLUMN_Z1
LEVER_LENGTH = 0.075  # handle length from pivot center
LEVER_HANDLE_R = 0.007  # handle cross-section radius
LEVER_BASE_R = 0.014  # pivot dome radius
LEVER_BASE_H = 0.012  # pivot dome height

# Articulation limits.
SWIVEL_LIMIT = math.radians(45.0)  # side-to-side temperature rotation
TILT_LIMIT = math.radians(40.0)  # upward tilt for flow on/off


def _build_spout_shape() -> cq.Workplane:
    """Hollow chrome spout: straight shank from the column, gentle downward
    curve, flared open outlet rim with a real hollow bore.
    Built in spout-local frame: origin at the column surface where the spout
    exits, shank runs along local +X, droops toward -Z."""
    r_out = SPOUT_R
    shank_x0 = -0.008  # seated inside the column wall
    shank_x1 = 0.040  # end of straight section
    bend_r = 0.030  # bend radius for the downward curve
    # After the bend the tube heads downward.
    end_x = shank_x1 + bend_r
    end_z = -bend_r

    # Sweep path: straight then arc downward.
    path = (
        cq.Workplane("XZ")
        .moveTo(shank_x0, 0.0)
        .lineTo(shank_x1, 0.0)
        .tangentArcPoint((bend_r, -bend_r), relative=True)
    )
    tube = cq.Workplane("YZ", origin=(shank_x0, 0.0, 0.0)).circle(r_out).sweep(path)

    # Flared outlet skirt around the down-turned end.
    flare = (
        cq.Workplane("XY", origin=(end_x, 0.0, end_z + 0.005))
        .circle(r_out - 0.001)
        .workplane(offset=-0.009)
        .circle(r_out + 0.005)
        .loft()
    )
    spout = tube.union(flare)

    # Tapered bore opening the outlet mouth (real hollow outlet).
    bore = (
        cq.Workplane("XY", origin=(end_x, 0.0, end_z - 0.005))
        .circle(r_out + 0.002)
        .workplane(offset=0.016)
        .circle(r_out - 0.004)
        .loft()
    )
    return spout.cut(bore)


def _build_lever_handle() -> cq.Workplane:
    """Lever handle shape in handle-local frame: origin at the tilt pivot,
    handle extends along local +X."""
    # Spherical pivot dome at the base.
    dome = cq.Workplane("XY").circle(LEVER_BASE_R).extrude(LEVER_BASE_H)
    dome = dome.edges(">Z").fillet(0.003)

    # Cylindrical handle bar extending outward.
    handle = (
        cq.Workplane("YZ", origin=(0.005, 0.0, LEVER_BASE_H * 0.6))
        .circle(LEVER_HANDLE_R)
        .extrude(LEVER_LENGTH - 0.005)
    )
    # Rounded end cap.
    end_cap = (
        cq.Workplane("XY", origin=(LEVER_LENGTH, 0.0, LEVER_BASE_H * 0.6))
        .sphere(LEVER_HANDLE_R)
    )
    lever = dome.union(handle).union(end_cap)
    return lever


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_hole_basin_faucet")

    model.material("chrome", rgba=(0.85, 0.87, 0.89, 1.0))
    model.material("chrome_dark", rgba=(0.42, 0.44, 0.47, 1.0))
    model.material("chrome_brushed", rgba=(0.70, 0.72, 0.74, 1.0))

    # ---------------- body (root): flange + column + ring -----------------
    body = model.part("body")
    body.visual(
        Cylinder(radius=FLANGE_R, length=FLANGE_H),
        origin=Origin(xyz=(0.0, 0.0, FLANGE_H / 2.0)),
        material="chrome",
        name="base_flange",
    )
    body.visual(
        Cylinder(radius=COLUMN_R, length=COLUMN_H),
        origin=Origin(xyz=(0.0, 0.0, COLUMN_Z0 + COLUMN_H / 2.0)),
        material="chrome",
        name="body_column",
    )
    body.visual(
        Cylinder(radius=RING_R, length=RING_H),
        origin=Origin(xyz=(0.0, 0.0, RING_Z0 + RING_H / 2.0)),
        material="chrome_dark",
        name="decorative_ring",
    )

    # ---------------- spout (fixed): swept hollow tube + flared outlet ----
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
        origin=Origin(xyz=(COLUMN_R, 0.0, SPOUT_Z)),
    )

    # ---------------- lever swivel base (revolute: temperature) -----------
    # The swivel base sits on top of the column and rotates around Z.
    lever_base = model.part("lever_base")
    lever_base.visual(
        Cylinder(radius=LEVER_BASE_R + 0.003, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material="chrome_brushed",
        name="swivel_plate",
    )
    model.articulation(
        "lever_swivel",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lever_base,
        origin=Origin(xyz=(0.0, 0.0, LEVER_PIVOT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=3.0, velocity=2.0, lower=-SWIVEL_LIMIT, upper=SWIVEL_LIMIT
        ),
    )

    # ---------------- lever handle (revolute: flow tilt) ------------------
    lever = model.part("lever_handle")
    lever.visual(
        mesh_from_cadquery(_build_lever_handle(), "lever_handle", tolerance=0.0003),
        material="chrome",
        name="lever_bar",
    )
    model.articulation(
        "lever_tilt",
        ArticulationType.REVOLUTE,
        parent=lever_base,
        child=lever,
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0, velocity=2.0, lower=0.0, upper=TILT_LIMIT
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    spout = object_model.get_part("spout")
    lever_base = object_model.get_part("lever_base")
    lever = object_model.get_part("lever_handle")
    swivel = object_model.get_articulation("lever_swivel")
    tilt = object_model.get_articulation("lever_tilt")

    # Intentional seated insertion: spout shank into the solid body column.
    ctx.allow_overlap(
        spout,
        body,
        elem_a="spout_tube",
        elem_b="body_column",
        reason="Spout shank is intentionally seated inside the solid body column wall.",
    )

    # ---- hero geometry: flange on deck, vertical column ------------------
    flange_aabb = ctx.part_element_world_aabb(body, elem="base_flange")
    ctx.check(
        "base flange sits flat on the deck",
        flange_aabb is not None and abs(flange_aabb[0][2]) <= 0.0005,
        details=f"flange aabb={flange_aabb}",
    )

    # ---- spout: projects forward and curves gently downward --------------
    spout_aabb = ctx.part_world_aabb(spout)
    ctx.check(
        "spout reaches forward and curves gently downward",
        spout_aabb is not None
        and spout_aabb[1][0] > 0.050  # extends well forward
        and spout_aabb[0][2] < SPOUT_Z - 0.010  # droops below exit height
        and spout_aabb[0][2] > 0.010,  # stays above the deck
        details=f"spout aabb={spout_aabb}",
    )
    ctx.expect_overlap(
        spout,
        body,
        axes="xz",
        min_overlap=0.003,
        name="spout shank stays seated in the body",
    )

    # ---- lever handle sits on top of the column --------------------------
    lever_aabb = ctx.part_world_aabb(lever)
    column_aabb = ctx.part_element_world_aabb(body, elem="body_column")
    ctx.check(
        "lever handle is mounted above the column top",
        lever_aabb is not None
        and column_aabb is not None
        and lever_aabb[0][2] >= column_aabb[1][2] - 0.008,
        details=f"lever aabb={lever_aabb}, column aabb={column_aabb}",
    )

    # ---- overall faucet height is about 0.13 m --------------------------
    ctx.check(
        "overall faucet height is about 0.11 to 0.14 m",
        lever_aabb is not None and 0.105 <= lever_aabb[1][2] <= 0.145,
        details=f"lever aabb={lever_aabb}",
    )

    # ---- articulation limits match the prompt ---------------------------
    sl = swivel.motion_limits
    ctx.check(
        "lever swivel limits are -45 to +45 degrees",
        sl is not None
        and sl.lower is not None
        and sl.upper is not None
        and abs(sl.lower + SWIVEL_LIMIT) < 1e-6
        and abs(sl.upper - SWIVEL_LIMIT) < 1e-6,
        details=f"limits={sl}",
    )
    tl = tilt.motion_limits
    ctx.check(
        "lever tilt limits are 0 to 40 degrees",
        tl is not None
        and tl.lower is not None
        and tl.upper is not None
        and abs(tl.lower) < 1e-9
        and abs(tl.upper - TILT_LIMIT) < 1e-6,
        details=f"limits={tl}",
    )

    # ---- decisive poses: lever tilt raises the handle end upward ---------
    bar_rest = ctx.part_element_world_aabb(lever, elem="lever_bar")
    with ctx.pose({tilt: TILT_LIMIT}):
        bar_tilted = ctx.part_element_world_aabb(lever, elem="lever_bar")
    ctx.check(
        "tilting the lever raises the handle end upward (flow on)",
        bar_rest is not None
        and bar_tilted is not None
        and (bar_tilted[1][2] + bar_tilted[0][2]) / 2.0
        > (bar_rest[1][2] + bar_rest[0][2]) / 2.0 + 0.003,
        details=f"rest={bar_rest}, tilted={bar_tilted}",
    )

    # ---- decisive poses: swivel rotates the handle side-to-side ----------
    bar_center = ctx.part_element_world_aabb(lever, elem="lever_bar")
    with ctx.pose({swivel: SWIVEL_LIMIT}):
        bar_swung = ctx.part_element_world_aabb(lever, elem="lever_bar")
    ctx.check(
        "swiveling the lever moves the handle sideways (temperature)",
        bar_center is not None
        and bar_swung is not None
        and abs(
            (bar_swung[1][1] + bar_swung[0][1]) / 2.0
            - (bar_center[1][1] + bar_center[0][1]) / 2.0
        )
        > 0.010,
        details=f"center={bar_center}, swung={bar_swung}",
    )

    # ---- hollow outlet: spout outlet is lower than spout exit -----------
    ctx.check(
        "spout outlet mouth is below the spout exit (gentle downward curve)",
        spout_aabb is not None and spout_aabb[0][2] < SPOUT_Z - 0.015,
        details=f"spout aabb={spout_aabb}, SPOUT_Z={SPOUT_Z}",
    )

    return ctx.report()


object_model = build_object_model()
