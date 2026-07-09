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
# High-arc gooseneck faucet variant (~0.38 m tall, single deck hole).
#
# Layout (world frame, deck plane at z = 0):
# - +X is the front of the faucet (direction the gooseneck reaches over the
#   sink), +Z is up.
# - A chrome base disc sits on the deck (single main deck hole).
# - A gloss-black cylindrical column (0.04 m dia) rises on the Z axis.
# - A side-mounted mixer body (lever handle) protrudes from the column on the
#   right side (+Y). The mixer pivots up/down on a horizontal axis to control
#   water flow (revolute, -45..+15 deg).
# - A thin chrome collar ring separates the column from the swivel spout.
# - The gooseneck spout swivels left-right on a vertical CONTINUOUS joint at
#   the collar, allowing full 360-degree rotation.
# - The spout arcs up and over to an apex at ~0.38 m and ends in a spray head
#   with shallow circumferential ribbing (3 ribs), a chrome tip sleeve, and a
#   downward outlet.
# - Gloss-black finish with faint veining approximated by a dark charcoal tone.
# ---------------------------------------------------------------------------

# Base + column
BASE_DISC_R = 0.042
BASE_DISC_H = 0.008
COLUMN_R = 0.020  # 0.04 m diameter
COLUMN_TOP = 0.132  # shaft reaches into the collar for connectivity

# Side mixer body (mounted on the column at mid-height)
MIXER_Z = 0.075  # height of mixer mount on column
MIXER_MOUNT_R = 0.014  # mounting boss radius
MIXER_MOUNT_LEN = 0.012  # mounting boss length (embeds into column)
MIXER_BODY_W = 0.028  # mixer body width (Y extent)
MIXER_BODY_H = 0.022  # mixer body height (Z extent)
MIXER_BODY_D = 0.055  # mixer body depth/length (X extent, protruding outward)
MIXER_HANDLE_R = 0.008  # handle grip radius
MIXER_HANDLE_LEN = 0.045  # handle grip length

# Swivel collar + gooseneck
COLLAR_R = 0.0215
COLLAR_LEN = 0.010
SWIVEL_Z = 0.140

TUBE_R = 0.015
ARC_R = 0.072
RISER_TOP = 0.153  # centerline apex = 0.153 + 0.072 = 0.225; +TUBE_R -> 0.240
REACH_X = 2.0 * ARC_R  # 0.144 m horizontal reach
DROP_END = 0.124  # spout-local z of the open tube tip (world 0.264)

# Spray head with ribbing
SLEEVE_R = 0.0175  # slightly wider for spray head body
SLEEVE_LEN = 0.035  # chrome tip sleeve / spray head body
RIB_R = 0.0185  # rib outer radius (1 mm proud of sleeve)
RIB_THICK = 0.002  # each rib is 2 mm tall
RIB_COUNT = 3  # three shallow ribs
RIB_SPACING = 0.008  # spacing between rib centers

AERATOR_R = 0.0128
AERATOR_LEN = 0.003  # dark outlet ring

APEX_WORLD = SWIVEL_Z + RISER_TOP + ARC_R + TUBE_R  # 0.380 m


def _gooseneck_shape() -> cq.Workplane:
    """Swan-neck tube: straight riser, high semicircular arc, short drop leg."""
    path = (
        cq.Workplane("XZ")
        .moveTo(0.0, 0.0)
        .lineTo(0.0, RISER_TOP)
        .threePointArc((ARC_R, RISER_TOP + ARC_R), (REACH_X, RISER_TOP))
        .lineTo(REACH_X, DROP_END)
    )
    return cq.Workplane("XY").circle(TUBE_R).sweep(path)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="high_arc_gooseneck_faucet")

    gloss_black = model.material("veined_gloss_black", rgba=(0.075, 0.075, 0.085, 1.0))
    matte_black = model.material("matte_black", rgba=(0.035, 0.035, 0.038, 1.0))
    chrome = model.material("polished_chrome", rgba=(0.85, 0.87, 0.89, 1.0))
    outlet_dark = model.material("outlet_dark", rgba=(0.10, 0.10, 0.10, 1.0))
    rib_material = model.material("rib_grey", rgba=(0.25, 0.25, 0.28, 1.0))

    # ------------------------------------------------------------------ column
    column = model.part("body_column")
    column.visual(
        Cylinder(radius=BASE_DISC_R, length=BASE_DISC_H),
        origin=Origin(xyz=(0.0, 0.0, BASE_DISC_H / 2.0)),
        material=chrome,
        name="base_disc",
    )
    column.visual(
        Cylinder(radius=COLUMN_R, length=COLUMN_TOP - 0.004),
        origin=Origin(xyz=(0.0, 0.0, (COLUMN_TOP + 0.004) / 2.0)),
        material=gloss_black,
        name="column_shaft",
    )
    # Thin chrome collar ring separating the column from the swivel spout.
    column.visual(
        Cylinder(radius=COLLAR_R, length=COLLAR_LEN),
        origin=Origin(xyz=(0.0, 0.0, SWIVEL_Z - COLLAR_LEN / 2.0)),
        material=chrome,
        name="swivel_collar",
    )
    # Mixer mounting boss (protrudes from column side toward +Y, embeds slightly)
    column.visual(
        Cylinder(radius=MIXER_MOUNT_R, length=MIXER_MOUNT_LEN),
        origin=Origin(xyz=(0.0, COLUMN_R + MIXER_MOUNT_LEN / 2.0 - 0.004, MIXER_Z),
                       rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gloss_black,
        name="mixer_mount_boss",
    )

    # --------------------------------------------------------------- gooseneck
    spout = model.part("gooseneck_spout")
    spout.visual(
        mesh_from_cadquery(_gooseneck_shape(), "gooseneck_tube"),
        material=gloss_black,
        name="gooseneck_tube",
    )
    # Spray head body (chrome sleeve at spout tip)
    spout.visual(
        Cylinder(radius=SLEEVE_R, length=SLEEVE_LEN),
        origin=Origin(xyz=(REACH_X, 0.0, DROP_END + SLEEVE_LEN / 2.0)),
        material=chrome,
        name="spray_head_sleeve",
    )
    # Shallow ribbing on the spray head (3 circumferential ribs)
    for rib_idx in range(RIB_COUNT):
        rib_z = DROP_END + 0.006 + rib_idx * RIB_SPACING
        spout.visual(
            Cylinder(radius=RIB_R, length=RIB_THICK),
            origin=Origin(xyz=(REACH_X, 0.0, rib_z)),
            material=rib_material,
            name=f"spray_rib_{rib_idx}",
        )
    # Downward outlet aerator
    spout.visual(
        Cylinder(radius=AERATOR_R, length=AERATOR_LEN),
        origin=Origin(xyz=(REACH_X, 0.0, DROP_END - 0.001)),
        material=outlet_dark,
        name="outlet_aerator",
    )
    # Spout swivel: CONTINUOUS joint about vertical axis at collar
    model.articulation(
        "spout_swivel",
        ArticulationType.CONTINUOUS,
        parent=column,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, SWIVEL_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=15.0, velocity=2.0),
    )

    # --------------------------------------------------------- side mixer body
    mixer = model.part("mixer_handle")
    # Mixer body: a block protruding outward from the mount point (+Y direction).
    # Offset back 4mm so the near face overlaps the mount boss for connectivity.
    mixer.visual(
        Box((MIXER_BODY_W, MIXER_BODY_D, MIXER_BODY_H)),
        origin=Origin(xyz=(0.0, MIXER_BODY_D / 2.0 - 0.004, 0.0)),
        material=gloss_black,
        name="mixer_body",
    )
    # Mixer handle grip: cylindrical rod at the far end of the mixer body
    mixer.visual(
        Cylinder(radius=MIXER_HANDLE_R, length=MIXER_HANDLE_LEN),
        origin=Origin(xyz=(0.0, MIXER_BODY_D - 0.014, MIXER_HANDLE_LEN / 2.0)),
        material=chrome,
        name="mixer_grip",
    )
    # Mixer pivot: revolute joint about horizontal X axis at the mount point
    # Positive q tilts the handle upward (away from column), negative tilts down
    model.articulation(
        "mixer_pivot",
        ArticulationType.REVOLUTE,
        parent=column,
        child=mixer,
        origin=Origin(xyz=(0.0, COLUMN_R + MIXER_MOUNT_LEN - 0.004, MIXER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=-math.radians(45.0),
            upper=math.radians(15.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    column = object_model.get_part("body_column")
    spout = object_model.get_part("gooseneck_spout")
    mixer = object_model.get_part("mixer_handle")

    swivel = object_model.get_articulation("spout_swivel")
    mixer_pivot = object_model.get_articulation("mixer_pivot")

    # Intentional seated insertion: mixer mount boss embeds into mixer body
    # for structural connectivity.
    ctx.allow_overlap(
        column,
        mixer,
        elem_a="mixer_mount_boss",
        elem_b="mixer_body",
        reason="Mixer mount boss intentionally seats into the mixer body for structural mounting.",
    )

    # ----- grounding, scale, proportions
    col_aabb = ctx.part_world_aabb(column)
    ctx.check(
        "faucet grounded on the deck plane",
        col_aabb is not None and abs(col_aabb[0][2]) <= 0.002,
        details=f"column aabb={col_aabb}",
    )
    disc = ctx.part_element_world_aabb(column, elem="base_disc")
    ctx.check(
        "single chrome base disc sits on the deck (single deck hole)",
        disc is not None
        and 0.080 <= (disc[1][0] - disc[0][0]) <= 0.090
        and (disc[1][2] - disc[0][2]) <= 0.010,
        details=f"base disc aabb={disc}",
    )
    shaft = ctx.part_element_world_aabb(column, elem="column_shaft")
    ctx.check(
        "vertical column is ~0.04 m diameter",
        shaft is not None and 0.038 <= (shaft[1][0] - shaft[0][0]) <= 0.042,
        details=f"column shaft aabb={shaft}",
    )
    spout_aabb = ctx.part_world_aabb(spout)
    ctx.check(
        "gooseneck apex near 0.38 m (high-arc silhouette)",
        spout_aabb is not None and 0.372 <= spout_aabb[1][2] <= 0.388,
        details=f"spout aabb={spout_aabb}",
    )
    ctx.check(
        "gooseneck arcs forward over the sink (+X reach)",
        spout_aabb is not None and spout_aabb[1][0] >= 0.130,
        details=f"spout aabb={spout_aabb}",
    )

    # ----- chrome collar ring between column and spout
    collar = ctx.part_element_world_aabb(column, elem="swivel_collar")
    ctx.check(
        "thin chrome collar sits at the spout base",
        collar is not None
        and collar[0][2] >= 0.130
        and collar[1][2] <= 0.145,
        details=f"collar={collar}",
    )
    ctx.expect_contact(
        spout,
        column,
        elem_a="gooseneck_tube",
        elem_b="swivel_collar",
        contact_tol=0.001,
        name="gooseneck riser seats on the chrome collar",
    )

    # ----- spray head with shallow ribbing
    sleeve = ctx.part_element_world_aabb(spout, elem="spray_head_sleeve")
    aerator = ctx.part_element_world_aabb(spout, elem="outlet_aerator")
    ctx.check(
        "spray head sleeve at spout tip with downward outlet",
        sleeve is not None
        and aerator is not None
        and aerator[0][2] < sleeve[0][2],
        details=f"sleeve={sleeve}, aerator={aerator}",
    )
    # Verify ribs exist and are positioned on the spray head
    rib_0 = ctx.part_element_world_aabb(spout, elem="spray_rib_0")
    rib_2 = ctx.part_element_world_aabb(spout, elem="spray_rib_2")
    ctx.check(
        "shallow ribbing on spray head (at least 3 ribs)",
        rib_0 is not None
        and rib_2 is not None
        and sleeve is not None
        and rib_0[0][2] >= sleeve[0][2] - 0.001
        and rib_2[1][2] <= sleeve[1][2] + 0.001,
        details=f"rib_0={rib_0}, rib_2={rib_2}, sleeve={sleeve}",
    )
    # Ribs should be slightly wider than the sleeve (protruding ridges)
    ctx.check(
        "ribs protrude slightly beyond sleeve radius",
        rib_0 is not None
        and sleeve is not None
        and (rib_0[1][0] - rib_0[0][0]) > (sleeve[1][0] - sleeve[0][0]) - 0.0005,
        details=f"rib_width={rib_0[1][0] - rib_0[0][0] if rib_0 else None}, sleeve_width={sleeve[1][0] - sleeve[0][0] if sleeve else None}",
    )

    # ----- side-mounted mixer body
    mixer_aabb = ctx.part_world_aabb(mixer)
    mixer_body = ctx.part_element_world_aabb(mixer, elem="mixer_body")
    mount_boss = ctx.part_element_world_aabb(column, elem="mixer_mount_boss")
    ctx.check(
        "side-mounted mixer body protrudes from the column",
        mixer_aabb is not None
        and mixer_body is not None
        and mixer_aabb[1][1] > COLUMN_R + 0.01,  # extends beyond column on +Y side
        details=f"mixer aabb={mixer_aabb}",
    )
    ctx.check(
        "mixer mount boss present on column side",
        mount_boss is not None
        and mount_boss[1][1] > COLUMN_R - 0.002,
        details=f"mount_boss={mount_boss}",
    )
    ctx.expect_overlap(
        column,
        mixer,
        axes="y",
        elem_a="mixer_mount_boss",
        elem_b="mixer_body",
        min_overlap=0.002,
        name="mixer mount boss seats into the mixer body",
    )

    # ----- joint plan: spout swivel is CONTINUOUS about vertical axis
    ctx.check(
        "spout swivel is a CONTINUOUS joint about the vertical axis (full rotation)",
        swivel.articulation_type == ArticulationType.CONTINUOUS
        and tuple(swivel.axis) == (0.0, 0.0, 1.0)
        and swivel.motion_limits is not None
        and swivel.motion_limits.lower is None
        and swivel.motion_limits.upper is None,
    )

    # ----- joint plan: mixer pivot is REVOLUTE with proper limits
    ctx.check(
        "mixer pivot is REVOLUTE with -45..+15 deg range on horizontal axis",
        mixer_pivot.articulation_type == ArticulationType.REVOLUTE
        and tuple(mixer_pivot.axis) == (1.0, 0.0, 0.0)
        and mixer_pivot.motion_limits is not None
        and abs(mixer_pivot.motion_limits.lower + math.radians(45.0)) < 1e-6
        and abs(mixer_pivot.motion_limits.upper - math.radians(15.0)) < 1e-6,
    )

    # ----- swivel pose: spout sweeps sideways about the vertical axis
    rest_sleeve = ctx.part_element_world_aabb(spout, elem="spray_head_sleeve")
    with ctx.pose({swivel: math.pi / 2.0}):
        sw_sleeve = ctx.part_element_world_aabb(spout, elem="spray_head_sleeve")
    ctx.check(
        "spout swivel rotates the spray head about the vertical axis",
        rest_sleeve is not None
        and sw_sleeve is not None
        and abs(0.5 * (rest_sleeve[0][1] + rest_sleeve[1][1])) < 0.002
        and abs(0.5 * (sw_sleeve[0][1] + sw_sleeve[1][1])) > 0.08,
        details=f"rest={rest_sleeve}, swiveled={sw_sleeve}",
    )

    # ----- mixer pose: handle tilts upward at positive q
    rest_mixer = ctx.part_world_aabb(mixer)
    with ctx.pose({mixer_pivot: math.radians(15.0)}):
        tilted_mixer = ctx.part_world_aabb(mixer)
    ctx.check(
        "mixer handle tilts upward at positive pivot angle",
        rest_mixer is not None
        and tilted_mixer is not None
        and tilted_mixer[1][2] > rest_mixer[1][2] + 0.005,
        details=f"rest={rest_mixer}, tilted={tilted_mixer}",
    )

    # ----- at least one non-fixed joint exists
    all_joints = [
        object_model.get_articulation(name)
        for name in ("spout_swivel", "mixer_pivot")
    ]
    ctx.check(
        "at least one non-fixed joint exists",
        any(j.articulation_type != ArticulationType.FIXED for j in all_joints),
    )

    return ctx.report()


object_model = build_object_model()
