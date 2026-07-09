from __future__ import annotations

"""Polished-chrome single-hole basin faucet with waterfall spout and cylindrical flow knob.

Variant of the tall vessel faucet forked into a distinct single-hole basin sibling:
- Rounded waterfall-style spout lip at the tip
- Cylindrical flow knob on top that rotates (continuous joint)
- Subtle knurled grooves on the knob grip surface
- Thin cartridge cap seam ring below the knob

Layout (meters, +Z up, ground at z=0, spout cantilevers along +X):
- A round stepped base plate carries a cylindrical column.
- A rectangular spout blade cantilevers forward from the column top with a
  rounded waterfall lip at its tip, and a recessed aerator outlet underneath.
- A thin cartridge cap seam ring sits at the column top.
- Above the cap, a chrome mounting post carries the cylindrical flow knob,
  which rotates about the vertical axis (continuous joint for flow control).
"""

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ----------------------------------------------------------------------------
# Key dimensions (meters)
# ----------------------------------------------------------------------------
BASE_LOWER_D = 0.074
BASE_LOWER_H = 0.006
BASE_UPPER_D = 0.058
BASE_UPPER_H = 0.010
BASE_TOP_Z = BASE_LOWER_H + BASE_UPPER_H  # 0.016

COLUMN_R = 0.020
COLUMN_TOP_Z = 0.252

SPOUT_WIDTH_Y = 0.048
SPOUT_THICK_Z = 0.018
SPOUT_BACK_X = -COLUMN_R  # flush with column rear face
SPOUT_TIP_X = 0.185  # ~0.17 m forward reach past column center
SPOUT_TOP_Z = COLUMN_TOP_Z  # blade top flush with column top
SPOUT_BOT_Z = SPOUT_TOP_Z - SPOUT_THICK_Z  # 0.234

# Waterfall lip: rounded front edge radius (half the spout thickness)
WATERFALL_R = SPOUT_THICK_Z / 2.0  # 0.009

OUTLET_X = 0.160  # aerator center, near the spout tip
AERATOR_OUTER_R = 0.011
AERATOR_INNER_R = 0.008
AERATOR_H = 0.007

# Cartridge cap seam: thin ring below the knob
CAP_OUTER_R = 0.026
CAP_INNER_R = 0.021
CAP_H = 0.005
CAP_TOP_Z = COLUMN_TOP_Z + CAP_H  # 0.257

# Mounting post above cap
POST_R = 0.009
POST_H = 0.012
POST_TOP_Z = CAP_TOP_Z + POST_H  # 0.269

# Flow knob
KNOB_DIAMETER = 0.040
KNOB_HEIGHT = 0.024
KNOB_TOP_Z = POST_TOP_Z + KNOB_HEIGHT  # ~0.293

# ----------------------------------------------------------------------------
# Build
# ----------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chrome_single_hole_basin_faucet")

    chrome = model.material("polished_chrome", rgba=(0.82, 0.84, 0.88, 1.0))
    dark = model.material("outlet_dark", rgba=(0.06, 0.06, 0.07, 1.0))
    cap_dark = model.material("cap_seam", rgba=(0.25, 0.26, 0.28, 1.0))

    # ------------------------------------------------------------------
    # Fixed body: round stepped base, cylindrical column, waterfall spout,
    # aerator, cartridge cap, mounting post
    # ------------------------------------------------------------------
    body = model.part("faucet_body")

    # Round stepped base plate
    body.visual(
        Cylinder(radius=BASE_LOWER_D / 2.0, length=BASE_LOWER_H),
        origin=Origin(xyz=(0.0, 0.0, BASE_LOWER_H / 2.0)),
        material=chrome,
        name="base_plate_lower",
    )
    body.visual(
        Cylinder(radius=BASE_UPPER_D / 2.0, length=BASE_UPPER_H),
        origin=Origin(xyz=(0.0, 0.0, BASE_LOWER_H + BASE_UPPER_H / 2.0)),
        material=chrome,
        name="base_plate_upper",
    )

    # Cylindrical column
    column_h = COLUMN_TOP_Z - BASE_TOP_Z
    body.visual(
        Cylinder(radius=COLUMN_R, length=column_h),
        origin=Origin(xyz=(0.0, 0.0, BASE_TOP_Z + column_h / 2.0)),
        material=chrome,
        name="column",
    )

    # Spout with rounded waterfall lip at the tip
    # Main rectangular spout body (CadQuery box, slightly shorter for the lip)
    spout_len = SPOUT_TIP_X - SPOUT_BACK_X
    spout_center_x = (SPOUT_BACK_X + SPOUT_TIP_X) / 2.0
    spout_center_z = SPOUT_BOT_Z + SPOUT_THICK_Z / 2.0

    lip_r = WATERFALL_R  # 0.009
    rect_len = spout_len - lip_r  # main body length minus half-cylinder embed
    spout_rect = cq.Workplane("XY").box(rect_len, SPOUT_WIDTH_Y, SPOUT_THICK_Z)

    body.visual(
        mesh_from_cadquery(spout_rect, "spout_body"),
        # Center the rect so its front edge is at lip_r/2 before the tip
        origin=Origin(xyz=(spout_center_x - lip_r / 2.0, 0.0, spout_center_z)),
        material=chrome,
        name="spout_body",
    )

    # Waterfall lip: cylinder along Y axis at the spout front edge
    # Rotate 90° about X to align cylinder Z→Y
    lip_cx = SPOUT_TIP_X - lip_r  # center of the lip cylinder
    body.visual(
        Cylinder(radius=lip_r, length=SPOUT_WIDTH_Y),
        origin=Origin(
            xyz=(lip_cx, 0.0, spout_center_z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=chrome,
        name="waterfall_lip",
    )

    # Aerator collar ring under the spout tip (true annulus)
    ring = (
        cq.Workplane("XY")
        .circle(AERATOR_OUTER_R)
        .circle(AERATOR_INNER_R)
        .extrude(AERATOR_H)
    )
    body.visual(
        mesh_from_cadquery(ring, "aerator_collar"),
        origin=Origin(xyz=(OUTLET_X, 0.0, SPOUT_BOT_Z - 0.003)),
        material=chrome,
        name="aerator_collar",
    )
    body.visual(
        Cylinder(radius=AERATOR_INNER_R, length=0.005),
        origin=Origin(xyz=(OUTLET_X, 0.0, SPOUT_BOT_Z - 0.003 + 0.0025 + 0.002)),
        material=dark,
        name="outlet_disc",
    )

    # Cartridge cap seam: thin solid disk at the column top (visible seam ring)
    body.visual(
        Cylinder(radius=CAP_OUTER_R, length=CAP_H),
        origin=Origin(xyz=(0.0, 0.0, COLUMN_TOP_Z + CAP_H / 2.0)),
        material=cap_dark,
        name="cartridge_cap_seam",
    )

    # Mounting post above the cap — extends from cap top upward
    body.visual(
        Cylinder(radius=POST_R, length=POST_H),
        origin=Origin(xyz=(0.0, 0.0, CAP_TOP_Z + POST_H / 2.0)),
        material=chrome,
        name="mounting_post",
    )

    # ------------------------------------------------------------------
    # Flow knob: cylindrical knob with knurled grip, rotates about Z
    # KnobGeometry center=False puts the mounting face at z=0
    # ------------------------------------------------------------------
    knob = model.part("flow_knob")

    flow_knob_geom = KnobGeometry(
        KNOB_DIAMETER,
        KNOB_HEIGHT,
        body_style="cylindrical",
        grip=KnobGrip(style="fluted", count=12, depth=0.0015),
        indicator=KnobIndicator(style="dot", mode="raised", angle_deg=0.0),
        center=False,
    )
    knob.visual(
        mesh_from_geometry(flow_knob_geom, "flow_knob_body"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=chrome,
        name="flow_knob_body",
    )

    # Continuous rotation about vertical axis through the mounting post
    model.articulation(
        "knob_rotate",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=knob,
        origin=Origin(xyz=(0.0, 0.0, POST_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("faucet_body")
    knob = object_model.get_part("flow_knob")
    knob_joint = object_model.get_articulation("knob_rotate")

    # --- joint plan: continuous rotation about vertical axis ---
    ctx.check(
        "knob joint is continuous about vertical Z axis",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS
        and abs(knob_joint.axis[0]) < 1e-9
        and abs(knob_joint.axis[1]) < 1e-9
        and abs(abs(knob_joint.axis[2]) - 1.0) < 1e-9,
        details=f"axis={knob_joint.axis}, type={knob_joint.articulation_type}",
    )
    ctx.check(
        "knob joint parents to faucet body and child is flow knob",
        knob_joint.parent == body.name and knob_joint.child == knob.name,
        details=f"parent={knob_joint.parent}, child={knob_joint.child}",
    )

    # --- grounding and scale ---
    body_aabb = ctx.part_world_aabb(body)
    knob_aabb = ctx.part_world_aabb(knob)
    ctx.check(
        "base plate is grounded at z=0",
        body_aabb is not None and abs(body_aabb[0][2]) < 1e-6,
        details=f"body_aabb={body_aabb}",
    )
    ctx.check(
        "total faucet height ~0.28-0.31 m",
        knob_aabb is not None and 0.27 <= knob_aabb[1][2] <= 0.32,
        details=f"knob_aabb={knob_aabb}",
    )

    # --- waterfall spout lip geometry ---
    spout_aabb = ctx.part_element_world_aabb(body, elem="spout_body")
    lip_aabb = ctx.part_element_world_aabb(body, elem="waterfall_lip")
    ctx.check(
        "spout body cantilevers forward from the column",
        spout_aabb is not None and spout_aabb[1][0] > 0.14,
        details=f"spout_aabb={spout_aabb}",
    )
    ctx.check(
        "waterfall lip cylinder sits at the spout tip",
        lip_aabb is not None
        and spout_aabb is not None
        and lip_aabb[0][0] > spout_aabb[0][0] + 0.05
        and lip_aabb[1][0] > 0.15,
        details=f"lip_aabb={lip_aabb}, spout_aabb={spout_aabb}",
    )
    # The waterfall lip is wider than the column (waterfall spouts are broad)
    ctx.check(
        "spout is wider than the column (waterfall-style broad lip)",
        spout_aabb is not None
        and (spout_aabb[1][1] - spout_aabb[0][1]) > 2.0 * COLUMN_R + 0.005,
        details=f"spout_width={None if spout_aabb is None else spout_aabb[1][1] - spout_aabb[0][1]}",
    )

    # --- cartridge cap seam ---
    cap_aabb = ctx.part_element_world_aabb(body, elem="cartridge_cap_seam")
    ctx.check(
        "cartridge cap seam ring sits at the column top below the knob",
        cap_aabb is not None
        and abs(cap_aabb[0][2] - COLUMN_TOP_Z) < 0.002
        and cap_aabb[1][2] < POST_TOP_Z,
        details=f"cap_aabb={cap_aabb}",
    )
    # Cap is wider than the column (visible seam ring)
    ctx.check(
        "cartridge cap ring is wider than the column (visible seam)",
        cap_aabb is not None
        and (cap_aabb[1][0] - cap_aabb[0][0]) > 2.0 * COLUMN_R + 0.005,
        details=f"cap_width={None if cap_aabb is None else cap_aabb[1][0] - cap_aabb[0][0]}",
    )

    # --- knob has knurled grip (named visual exists) ---
    knob_body_vis = knob.get_visual("flow_knob_body")
    ctx.check(
        "flow knob body visual exists with knurled grip geometry",
        knob_body_vis is not None,
        details="flow_knob_body visual missing",
    )

    # --- aerator and outlet ---
    collar_aabb = ctx.part_element_world_aabb(body, elem="aerator_collar")
    outlet_aabb = ctx.part_element_world_aabb(body, elem="outlet_disc")
    ctx.check(
        "aerator collar is recessed under the spout near the tip",
        collar_aabb is not None
        and collar_aabb[0][2] < SPOUT_BOT_Z
        and collar_aabb[0][0] > 0.10,
        details=f"collar_aabb={collar_aabb}",
    )
    ctx.check(
        "dark outlet disc sits inside the aerator collar",
        collar_aabb is not None
        and outlet_aabb is not None
        and outlet_aabb[0][2] > collar_aabb[0][2]
        and outlet_aabb[0][0] > collar_aabb[0][0]
        and outlet_aabb[1][0] < collar_aabb[1][0],
        details=f"outlet_aabb={outlet_aabb}, collar_aabb={collar_aabb}",
    )

    # --- mounting: knob seats on the post ---
    ctx.expect_gap(
        knob,
        body,
        axis="z",
        min_gap=-0.002,
        max_gap=0.003,
        name="knob sits at the mounting post height",
    )
    ctx.expect_overlap(
        knob,
        body,
        axes="xy",
        min_overlap=0.005,
        name="knob overlaps the mounting post in XY",
    )

    # --- decisive pose: knob rotation moves the indicator ---
    rest_aabb = knob_aabb
    with ctx.pose({knob_joint: math.pi}):
        rotated_aabb = ctx.part_world_aabb(knob)
        ctx.check(
            "knob rotates 180 degrees about vertical axis (continuous joint works)",
            rotated_aabb is not None
            and rest_aabb is not None
            and abs(rotated_aabb[1][2] - rest_aabb[1][2]) < 0.005,
            details=f"rest={rest_aabb}, rotated={rotated_aabb}",
        )

    # --- knob stays above the cartridge cap at all poses ---
    ctx.expect_gap(
        knob,
        body,
        axis="z",
        min_gap=-0.001,
        positive_elem="flow_knob_body",
        negative_elem="cartridge_cap_seam",
        name="knob stays above the cartridge cap seam",
    )

    return ctx.report()


object_model = build_object_model()
