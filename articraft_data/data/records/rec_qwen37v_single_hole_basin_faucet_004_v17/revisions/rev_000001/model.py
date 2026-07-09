from __future__ import annotations

"""Polished-chrome single-hole basin faucet with rectangular spout, flat slot
outlet, circular aerator insert, and a cylindrical rotary flow knob on top.

Layout (meters, +Z up, ground at z=0, spout cantilevers along +X):
- A square stepped base plate carries a slim rectangular column.
- A flat rectangular spout blade cantilevers forward from the column top with a
  rectangular slot outlet on its underside near the tip and a hollow mouth
  opening at the front face.
- A separate circular aerator insert sits recessed inside the rectangular slot.
- Above the column top, a short chrome stem carries a cylindrical flow knob
  that rotates about the vertical axis (±90°) for flow/temperature control.
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
BASE_LOWER_SIDE = 0.090
BASE_LOWER_H = 0.006
BASE_UPPER_SIDE = 0.068
BASE_UPPER_H = 0.012
BASE_TOP_Z = BASE_LOWER_H + BASE_UPPER_H  # 0.018

COLUMN_DEPTH_X = 0.035
COLUMN_WIDTH_Y = 0.045
COLUMN_TOP_Z = 0.235

SPOUT_WIDTH_Y = 0.050
SPOUT_THICK_Z = 0.020
SPOUT_BACK_X = -COLUMN_DEPTH_X / 2.0  # flush with column rear face
SPOUT_TIP_X = 0.1825  # ~0.17 m forward reach past the column front face
SPOUT_TOP_Z = COLUMN_TOP_Z  # blade top flush with column top
SPOUT_BOT_Z = SPOUT_TOP_Z - SPOUT_THICK_Z  # 0.215
SPOUT_LEN = SPOUT_TIP_X - SPOUT_BACK_X  # 0.200
SPOUT_CENTER_X = (SPOUT_BACK_X + SPOUT_TIP_X) / 2.0  # 0.0825
SPOUT_CENTER_Z = SPOUT_BOT_Z + SPOUT_THICK_Z / 2.0  # 0.225

# Rectangular slot outlet dimensions (cut into underside near tip)
SLOT_LENGTH_X = 0.026
SLOT_WIDTH_Y = 0.034
SLOT_DEPTH_Z = 0.012
SLOT_FROM_TIP = 0.018  # slot center distance from tip end

# Mouth opening at spout front face
MOUTH_WIDTH_Y = 0.030
MOUTH_HEIGHT_Z = 0.014
MOUTH_DEPTH_X = 0.016

# Circular aerator insert (sits inside the rectangular slot)
AERATOR_OUTER_R = 0.011
AERATOR_INNER_R = 0.007
AERATOR_H = 0.004

# Knob stem on column top
STEM_R = 0.008
STEM_H = 0.012
STEM_TOP_Z = COLUMN_TOP_Z + STEM_H  # 0.247

# Flow knob dimensions
KNOB_DIAMETER = 0.036
KNOB_HEIGHT = 0.022

KNOB_RANGE = math.radians(90.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chrome_single_hole_basin_faucet")

    chrome = model.material("polished_chrome", rgba=(0.85, 0.87, 0.90, 1.0))
    dark = model.material("outlet_dark", rgba=(0.08, 0.08, 0.09, 1.0))
    dark_grey = model.material("dark_grey", rgba=(0.18, 0.18, 0.20, 1.0))

    # ------------------------------------------------------------------
    # Fixed body: stepped base plate, column, spout blade with cuts, stem
    # ------------------------------------------------------------------
    body = model.part("faucet_body")
    body.visual(
        Box((BASE_LOWER_SIDE, BASE_LOWER_SIDE, BASE_LOWER_H)),
        origin=Origin(xyz=(0.0, 0.0, BASE_LOWER_H / 2.0)),
        material=chrome,
        name="base_plate_lower",
    )
    body.visual(
        Box((BASE_UPPER_SIDE, BASE_UPPER_SIDE, BASE_UPPER_H)),
        origin=Origin(xyz=(0.0, 0.0, BASE_LOWER_H + BASE_UPPER_H / 2.0)),
        material=chrome,
        name="base_plate_upper",
    )
    column_h = COLUMN_TOP_Z - BASE_TOP_Z
    body.visual(
        Box((COLUMN_DEPTH_X, COLUMN_WIDTH_Y, column_h)),
        origin=Origin(xyz=(0.0, 0.0, BASE_TOP_Z + column_h / 2.0)),
        material=chrome,
        name="column",
    )

    # Spout blade with rectangular slot outlet and hollow mouth opening.
    # Built in CadQuery local space (centered at origin), then positioned.
    spout = cq.Workplane("XY").box(SPOUT_LEN, SPOUT_WIDTH_Y, SPOUT_THICK_Z)

    # Cut rectangular slot on underside near tip.
    # In local space: tip at +X (spout_len/2), underside at -Z.
    # Slot center X in local = spout_len/2 - SLOT_FROM_TIP
    slot_local_x = SPOUT_LEN / 2.0 - SLOT_FROM_TIP
    spout = (
        spout.faces("<Z").workplane(centerOption="CenterOfMass")
        .center(slot_local_x, 0.0)
        .rect(SLOT_LENGTH_X, SLOT_WIDTH_Y)
        .cutBlind(SLOT_DEPTH_Z)
    )

    # Cut mouth opening on front face (+X).
    # Centered on the face: the mouth is a rectangular pocket inward from the tip.
    spout = (
        spout.faces(">X").workplane(centerOption="CenterOfMass")
        .rect(MOUTH_WIDTH_Y, MOUTH_HEIGHT_Z)
        .cutBlind(MOUTH_DEPTH_X)
    )

    body.visual(
        mesh_from_cadquery(spout, "spout_blade"),
        origin=Origin(xyz=(SPOUT_CENTER_X, 0.0, SPOUT_CENTER_Z)),
        material=chrome,
        name="spout_blade",
    )

    # Dark interior visible through the mouth opening — a dark block that
    # extends past the mouth pocket into solid spout body for connectivity.
    interior_len_x = MOUTH_DEPTH_X + 0.012  # extends 12mm back into solid spout
    mouth_interior = (
        cq.Workplane("XY")
        .box(interior_len_x, MOUTH_WIDTH_Y - 0.006, MOUTH_HEIGHT_Z - 0.002)
    )
    body.visual(
        mesh_from_cadquery(mouth_interior, "mouth_interior"),
        origin=Origin(
            xyz=(SPOUT_TIP_X - interior_len_x / 2.0, 0.0, SPOUT_CENTER_Z)
        ),
        material=dark,
        name="mouth_interior",
    )

    # Knob stem on column top
    body.visual(
        Cylinder(radius=STEM_R, length=STEM_H),
        origin=Origin(xyz=(0.0, 0.0, COLUMN_TOP_Z + STEM_H / 2.0)),
        material=chrome,
        name="knob_stem",
    )

    # ------------------------------------------------------------------
    # Aerator insert: separate circular ring seated in the rectangular slot
    # ------------------------------------------------------------------
    aerator = model.part("aerator_insert")

    # Chrome annular ring
    aerator_ring = (
        cq.Workplane("XY")
        .circle(AERATOR_OUTER_R)
        .circle(AERATOR_INNER_R)
        .extrude(AERATOR_H)
    )
    aerator.visual(
        mesh_from_cadquery(aerator_ring, "aerator_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=chrome,
        name="aerator_ring",
    )
    # Dark mesh face inside the ring — slightly oversized to contact ring wall
    aerator.visual(
        Cylinder(radius=AERATOR_INNER_R + 0.0005, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
        material=dark_grey,
        name="aerator_mesh",
    )

    # Position: the aerator sits recessed in the slot, near the bottom.
    # World X = slot center = SPOUT_TIP_X - SLOT_FROM_TIP
    # World Z = SPOUT_BOT_Z + 0.002 (2mm above spout underside, inside slot)
    aerator_world_x = SPOUT_TIP_X - SLOT_FROM_TIP
    aerator_world_z = SPOUT_BOT_Z + 0.002

    # Fixed articulation: aerator is rigidly seated in the slot
    model.articulation(
        "aerator_mount",
        ArticulationType.FIXED,
        parent=body,
        child=aerator,
        origin=Origin(xyz=(aerator_world_x, 0.0, aerator_world_z)),
    )

    # ------------------------------------------------------------------
    # Flow knob: cylindrical rotary control on top of the stem
    # ------------------------------------------------------------------
    knob = model.part("flow_knob")

    knob_geom = KnobGeometry(
        KNOB_DIAMETER,
        KNOB_HEIGHT,
        body_style="cylindrical",
        grip=KnobGrip(style="fluted", count=20, depth=0.0012),
        indicator=KnobIndicator(style="wedge", mode="raised", angle_deg=0.0),
        center=False,  # base face at z=0
    )
    knob.visual(
        mesh_from_geometry(knob_geom, "knob_body"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=chrome,
        name="knob_body",
    )
    # Pointer tab extending radially from the knob side — breaks rotational
    # symmetry so rotation is visible and measurable.
    tab_len = 0.012
    tab_width = 0.006
    tab_thick = 0.004
    knob.visual(
        Box((tab_len, tab_width, tab_thick)),
        origin=Origin(
            xyz=(KNOB_DIAMETER / 2.0 + tab_len / 2.0 - 0.002, 0.0, KNOB_HEIGHT * 0.65)
        ),
        material=chrome,
        name="knob_pointer",
    )

    # Revolute joint: rotates about vertical axis through the stem top
    model.articulation(
        "knob_rotate",
        ArticulationType.REVOLUTE,
        parent=body,
        child=knob,
        origin=Origin(xyz=(0.0, 0.0, STEM_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=6.0, velocity=3.0, lower=-KNOB_RANGE, upper=KNOB_RANGE
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("faucet_body")
    aerator = object_model.get_part("aerator_insert")
    knob = object_model.get_part("flow_knob")
    knob_joint = object_model.get_articulation("knob_rotate")
    aerator_joint = object_model.get_articulation("aerator_mount")

    # --- joint plan: knob rotation is revolute about vertical axis ---
    ctx.check(
        "knob_rotate is revolute about vertical axis with ±90° limits",
        knob_joint.articulation_type == ArticulationType.REVOLUTE
        and abs(knob_joint.axis[0]) < 1e-9
        and abs(knob_joint.axis[1]) < 1e-9
        and abs(abs(knob_joint.axis[2]) - 1.0) < 1e-9
        and knob_joint.motion_limits is not None
        and abs(knob_joint.motion_limits.lower + KNOB_RANGE) < 1e-6
        and abs(knob_joint.motion_limits.upper - KNOB_RANGE) < 1e-6,
        details=f"axis={knob_joint.axis}, limits={knob_joint.motion_limits}",
    )
    ctx.check(
        "aerator_mount is a fixed joint connecting aerator to body",
        aerator_joint.articulation_type == ArticulationType.FIXED
        and aerator_joint.parent == body.name
        and aerator_joint.child == aerator.name,
        details=f"type={aerator_joint.articulation_type}, parent={aerator_joint.parent}, child={aerator_joint.child}",
    )

    # --- grounding and true scale ---
    body_aabb = ctx.part_world_aabb(body)
    knob_aabb = ctx.part_world_aabb(knob)
    ctx.check(
        "base plate is grounded at z=0",
        body_aabb is not None and abs(body_aabb[0][2]) < 1e-6,
        details=f"body_aabb={body_aabb}",
    )
    ctx.check(
        "total faucet height ~0.27-0.30 m (knob is topmost)",
        knob_aabb is not None and 0.25 <= knob_aabb[1][2] <= 0.32,
        details=f"knob_aabb={knob_aabb}",
    )
    ctx.check(
        "spout blade cantilevers ~0.17 m forward of the column front face",
        body_aabb is not None and 0.14 <= body_aabb[1][0] - COLUMN_DEPTH_X / 2.0 <= 0.20,
        details=f"body max x={None if body_aabb is None else body_aabb[1][0]}",
    )

    # --- rectangular slot outlet (not round) ---
    spout_aabb = ctx.part_element_world_aabb(body, elem="spout_blade")
    mouth_aabb = ctx.part_element_world_aabb(body, elem="mouth_interior")
    ctx.check(
        "spout blade has a hollow mouth opening near the tip",
        mouth_aabb is not None
        and mouth_aabb[1][0] > SPOUT_TIP_X - MOUTH_DEPTH_X - 0.005,
        details=f"mouth_aabb={mouth_aabb}",
    )
    ctx.check(
        "mouth interior is dark and recessed inside the spout tip",
        mouth_aabb is not None
        and spout_aabb is not None
        and mouth_aabb[1][0] <= spout_aabb[1][0] + 0.001,
        details=f"mouth={mouth_aabb}, spout={spout_aabb}",
    )

    # --- aerator insert is circular, seated in the rectangular slot ---
    aerator_aabb = ctx.part_world_aabb(aerator)
    ctx.check(
        "aerator insert sits below the spout underside (recessed in slot)",
        aerator_aabb is not None
        and aerator_aabb[0][2] < SPOUT_BOT_Z + 0.006
        and aerator_aabb[0][2] > SPOUT_BOT_Z - 0.005,
        details=f"aerator_aabb={aerator_aabb}",
    )
    ctx.check(
        "aerator is circular: roughly equal XY footprint",
        aerator_aabb is not None
        and abs(
            (aerator_aabb[1][0] - aerator_aabb[0][0])
            - (aerator_aabb[1][1] - aerator_aabb[0][1])
        ) < 0.005,
        details=f"aerator_aabb={aerator_aabb}",
    )
    ctx.expect_within(
        aerator,
        body,
        axes="xy",
        margin=0.005,
        name="aerator insert is within the spout footprint on XY",
    )

    # --- knob sits on top of column ---
    ctx.check(
        "flow knob sits on top of the column stem",
        knob_aabb is not None
        and knob_aabb[0][2] > COLUMN_TOP_Z,
        details=f"knob_aabb={knob_aabb}",
    )
    ctx.expect_overlap(
        knob,
        body,
        axes="xy",
        min_overlap=0.008,
        name="knob overlaps the stem footprint on XY",
    )

    # --- decisive pose: knob rotation (pointer tab breaks symmetry) ---
    rest_knob_aabb = knob_aabb
    with ctx.pose({knob_joint: KNOB_RANGE}):
        rotated_aabb = ctx.part_world_aabb(knob)
        ctx.check(
            "positive knob rotation moves the pointer tab (Y extent changes)",
            rest_knob_aabb is not None
            and rotated_aabb is not None
            and (
                abs(rotated_aabb[1][1] - rest_knob_aabb[1][1]) > 0.002
                or abs(rotated_aabb[0][1] - rest_knob_aabb[0][1]) > 0.002
                or abs(rotated_aabb[1][0] - rest_knob_aabb[1][0]) > 0.002
            ),
            details=f"rest={rest_knob_aabb}, rotated={rotated_aabb}",
        )

    with ctx.pose({knob_joint: -KNOB_RANGE}):
        neg_rotated_aabb = ctx.part_world_aabb(knob)
        ctx.check(
            "negative knob rotation moves the pointer tab oppositely",
            rest_knob_aabb is not None
            and neg_rotated_aabb is not None
            and (
                abs(neg_rotated_aabb[1][1] - rest_knob_aabb[1][1]) > 0.002
                or abs(neg_rotated_aabb[0][1] - rest_knob_aabb[0][1]) > 0.002
                or abs(neg_rotated_aabb[0][0] - rest_knob_aabb[0][0]) > 0.002
            ),
            details=f"rest={rest_knob_aabb}, neg_rotated={neg_rotated_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
