from __future__ import annotations

# Red painted sheet-metal fire bucket with hinged lid (LIDDED variant).
#
# Coordinate convention:
#   - up is +Z. The flat bottom rests on the ground at z=0.
#   - the bucket is a tapered cylinder, wider at the open top, narrower at the
#     flat bottom, modeled as a hollow thin-wall revolved shell.
#   - the rolled top rim is a torus around the top edge.
#   - a flat circular lid covers the mouth, hinged at the +X rim edge.
#   - the lid is a REVOLUTE joint about the Y-tangent axis at the rim,
#     swinging from closed (flat over the mouth) upward and open.
#   - two hinge ears on the +X rim wall hold the hinge pin; a hinge strap
#     on the lid bridges the lid disk to the pin.

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)

# --- key dimensions (meters) ---
TOP_R = 0.140  # outer radius at the open top
BOT_R = 0.105  # outer radius at the flat bottom (tapered narrower)
BODY_H = 0.260  # bucket height (bottom z=0 -> rim z=BODY_H)
WALL = 0.0016  # sheet-metal wall thickness
BOTTOM_T = 0.004  # thickness of the flat bottom plate

RIM_TUBE = 0.006  # rolled-rim tube (minor) radius
RIM_CENTER_R = TOP_R - RIM_TUBE * 0.4  # torus center radius at the top edge
RIM_Z = BODY_H  # rolled rim sits at the top edge

# --- lid dimensions ---
LID_R = TOP_R  # lid radius matches bucket top outer radius
LID_T = 0.002  # 2mm sheet-metal lid

# --- hinge dimensions ---
HINGE_X = TOP_R  # hinge at the +X rim edge
HINGE_Z = RIM_Z  # hinge pin at rim center height
HINGE_BARREL_R = 0.004  # hinge pin/barrel radius
HINGE_BARREL_LEN = 0.036  # barrel length along Y
HINGE_EAR_T = 0.002  # ear plate thickness (along X)
HINGE_EAR_W = 0.012  # ear width (along Y)
HINGE_EAR_H = 0.024  # ear height (along Z)
HINGE_EAR_GAP = HINGE_BARREL_LEN - 2 * HINGE_EAR_W  # gap between ears
HINGE_STRAP_W = HINGE_EAR_GAP - 0.002  # strap fits between ears with clearance
HINGE_STRAP_LEN = 0.016  # strap length along X
HINGE_STRAP_H = RIM_TUBE + LID_T + 0.002  # strap height bridges barrel to lid top

# Lid seat: lid bottom sits slightly above rim top to avoid penetration.
LID_SEAT_Z = RIM_TUBE + 0.001  # 1mm above the rim top tube surface


def _revolved_shell_mesh(
    top_r: float,
    bot_r: float,
    height: float,
    wall: float,
    bottom_t: float,
    name: str,
):
    """Hollow tapered bucket wall + flat bottom as one revolved thin shell."""
    outer = [
        (bot_r, 0.0),
        (bot_r + (top_r - bot_r) * 0.5, height * 0.5),
        (top_r, height),
    ]
    inner_bot_r = bot_r - wall
    inner_top_r = top_r - wall
    inner = [
        (0.0, bottom_t),
        (inner_bot_r, bottom_t),
        (inner_bot_r + (inner_top_r - inner_bot_r) * 0.5, height * 0.5),
        (inner_top_r, height),
    ]
    geom = LatheGeometry.from_shell_profiles(outer, inner, segments=64)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fire_bucket_lidded")

    red_metal = model.material("red_metal", rgba=(0.62, 0.09, 0.08, 1.0))
    dark_red = model.material("dark_red", rgba=(0.48, 0.06, 0.05, 1.0))
    steel = model.material("steel", rgba=(0.72, 0.74, 0.77, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.45, 0.47, 0.50, 1.0))

    # --- bucket body (root): hollow tapered shell + rolled rim + hinge ears ---
    bucket = model.part("bucket")

    shell_mesh = _revolved_shell_mesh(TOP_R, BOT_R, BODY_H, WALL, BOTTOM_T, "bucket_shell")
    bucket.visual(shell_mesh, material=red_metal, name="bucket_shell")

    rim_geom = TorusGeometry(
        radius=RIM_CENTER_R, tube=RIM_TUBE,
        radial_segments=24, tubular_segments=64,
    )
    rim_mesh = mesh_from_geometry(rim_geom, "rim")
    bucket.visual(
        rim_mesh,
        origin=Origin(xyz=(0.0, 0.0, RIM_Z)),
        material=red_metal,
        name="rolled_rim",
    )

    # Two hinge ears on the +X rim side, spaced apart along Y.
    # Each ear is a flat plate riveted to the outer wall, extending from
    # below the rim upward past the hinge pin.
    ear_bottom_z = RIM_Z - 0.018
    ear_center_z = ear_bottom_z + HINGE_EAR_H / 2.0
    ear_center_x = HINGE_X + HINGE_EAR_T / 2.0
    for i in range(2):
        y_sign = 1.0 if i == 0 else -1.0
        y_pos = y_sign * (HINGE_EAR_W / 2.0 + HINGE_EAR_GAP / 2.0)
        bucket.visual(
            Box((HINGE_EAR_T, HINGE_EAR_W, HINGE_EAR_H)),
            origin=Origin(xyz=(ear_center_x, y_pos, ear_center_z)),
            material=steel,
            name=f"hinge_ear_{i}",
        )

    # Hinge barrel (pin): cylinder along Y through the ears at the hinge point.
    bucket.visual(
        Cylinder(radius=HINGE_BARREL_R, length=HINGE_BARREL_LEN),
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hinge_barrel",
    )

    bucket.inertial = Inertial.from_geometry(
        Cylinder(radius=TOP_R, length=BODY_H),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, BODY_H / 2.0)),
    )

    # --- lid: flat circular disk + hinge strap + rim ring + knob ---
    # Lid part frame origin is at the hinge pin (articulation frame at q=0).
    # In lid-local coordinates, the mouth center is at (-TOP_R, 0, 0).
    lid = model.part("lid")

    # Lid disk: flat cylinder covering the mouth when closed.
    lid.visual(
        Cylinder(radius=LID_R, length=LID_T),
        origin=Origin(xyz=(-TOP_R, 0.0, LID_SEAT_Z + LID_T / 2.0)),
        material=red_metal,
        name="lid_disk",
    )

    # Lid rim: thin torus near the lid edge for a rolled-lip detail.
    lid_rim_r = LID_R - 0.004
    lid_rim_geom = TorusGeometry(
        radius=lid_rim_r, tube=0.002,
        radial_segments=12, tubular_segments=48,
    )
    lid_rim_mesh = mesh_from_geometry(lid_rim_geom, "lid_rim")
    lid.visual(
        lid_rim_mesh,
        origin=Origin(xyz=(-TOP_R, 0.0, LID_SEAT_Z + LID_T / 2.0)),
        material=dark_red,
        name="lid_rim",
    )

    # Hinge strap: flat plate connecting lid edge to the hinge barrel.
    # Bridges vertically from the hinge pin level up to the lid surface.
    # Positioned at the hinge edge (x ≈ 0 in lid-local frame).
    strap_center_x = -0.002  # slightly into the lid from the hinge center
    strap_center_z = HINGE_STRAP_H / 2.0
    lid.visual(
        Box((HINGE_STRAP_LEN, HINGE_STRAP_W, HINGE_STRAP_H)),
        origin=Origin(xyz=(strap_center_x, 0.0, strap_center_z)),
        material=steel,
        name="hinge_strap",
    )

    # Lid knob: small cylindrical grip on top of the lid near center.
    knob_r = 0.010
    knob_h = 0.008
    lid.visual(
        Cylinder(radius=knob_r, length=knob_h),
        origin=Origin(xyz=(-TOP_R, 0.0, LID_SEAT_Z + LID_T + knob_h / 2.0)),
        material=dark_steel,
        name="lid_knob",
    )

    lid.inertial = Inertial.from_geometry(
        Cylinder(radius=LID_R, length=LID_T),
        mass=0.15,
        origin=Origin(xyz=(-TOP_R, 0.0, LID_SEAT_Z + LID_T / 2.0)),
    )

    # --- articulation: REVOLUTE hinge at +X rim edge ---
    # Axis along Y (tangent to the rim circle at +X). Positive q opens the
    # lid upward (the far edge at local -X rotates toward +Z via RH rule about +Y).
    model.articulation(
        "bucket_to_lid",
        ArticulationType.REVOLUTE,
        parent=bucket,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(110.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bucket = object_model.get_part("bucket")
    lid = object_model.get_part("lid")
    joint = object_model.get_articulation("bucket_to_lid")

    # --- intentional overlaps ---
    # Hinge ears are riveted onto the rim/wall.
    for i in range(2):
        ctx.allow_overlap(
            bucket, bucket,
            elem_a=f"hinge_ear_{i}", elem_b="rolled_rim",
            reason=f"Hinge ear {i} is riveted onto the bucket rim/wall.",
        )
        ctx.allow_overlap(
            bucket, bucket,
            elem_a="hinge_barrel", elem_b=f"hinge_ear_{i}",
            reason=f"Hinge pin passes through ear {i}.",
        )
    # Hinge strap wraps around the barrel pin.
    ctx.allow_overlap(
        lid, bucket,
        elem_a="hinge_strap", elem_b="hinge_barrel",
        reason="Hinge strap wraps around the barrel pin.",
    )
    # Hinge strap passes through the rim area to connect to the barrel.
    ctx.allow_overlap(
        lid, bucket,
        elem_a="hinge_strap", elem_b="rolled_rim",
        reason="Hinge strap bridges through the rim area to reach the hinge barrel.",
    )
    # Lid disk edge contacts the barrel at the hinge.
    ctx.allow_overlap(
        lid, bucket,
        elem_a="lid_disk", elem_b="hinge_barrel",
        reason="Lid disk edge meets the hinge barrel at the pivot.",
    )

    # Proof checks for intentional overlaps:
    ctx.expect_contact(
        lid, bucket,
        elem_a="hinge_strap", elem_b="hinge_barrel",
        contact_tol=0.005,
        name="hinge strap contacts barrel",
    )

    # ================================================================
    # 1. No bail-handle lugs or handle part
    # ================================================================
    visual_names = [v.name for v in bucket.visuals]
    ctx.check(
        "no bail-handle lugs on the bucket",
        "lug_pos" not in visual_names and "lug_neg" not in visual_names
        and not any(n.startswith("lug_") for n in visual_names),
        details=f"bucket visuals: {visual_names}",
    )
    part_names = [p.name for p in object_model.parts]
    ctx.check(
        "no separate handle part",
        "handle" not in part_names,
        details=f"parts: {part_names}",
    )

    # ================================================================
    # 2. Circular lid covers the mouth at the rim plane
    # ================================================================
    lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_disk")
    lid_cx = (lid_aabb[0][0] + lid_aabb[1][0]) / 2.0
    lid_cy = (lid_aabb[0][1] + lid_aabb[1][1]) / 2.0
    ctx.check(
        "lid disk centered over the mouth",
        abs(lid_cx) < 0.015 and abs(lid_cy) < 0.015,
        details=f"lid_center=({lid_cx:.4f}, {lid_cy:.4f})",
    )
    lid_span_x = lid_aabb[1][0] - lid_aabb[0][0]
    lid_span_y = lid_aabb[1][1] - lid_aabb[0][1]
    mouth_d = 2.0 * TOP_R
    ctx.check(
        "lid disk diameter covers the mouth",
        lid_span_x >= mouth_d * 0.95 and lid_span_y >= mouth_d * 0.95,
        details=f"lid_span=({lid_span_x:.4f}, {lid_span_y:.4f}), mouth_d={mouth_d:.4f}",
    )
    lid_bottom_z = lid_aabb[0][2]
    ctx.check(
        "lid sits at the rim plane",
        abs(lid_bottom_z - (RIM_Z + RIM_TUBE)) < 0.008,
        details=f"lid_bottom_z={lid_bottom_z:.4f}, rim_top_z={RIM_Z + RIM_TUBE:.4f}",
    )

    # ================================================================
    # 3. Lid hinge is REVOLUTE on a horizontal rim-tangent axis
    # ================================================================
    ctx.check(
        "lid joint is revolute",
        str(joint.articulation_type).upper().endswith("REVOLUTE"),
        details=f"type={joint.articulation_type}",
    )
    ax = joint.axis
    ctx.check(
        "hinge axis is horizontal and tangent to rim (Y direction)",
        abs(ax[1]) > 0.99 and abs(ax[0]) < 0.02 and abs(ax[2]) < 0.02,
        details=f"axis={ax}",
    )
    jo = joint.origin.xyz
    ctx.check(
        "hinge origin at the rim edge (+X side)",
        abs(jo[0] - HINGE_X) < 0.01 and abs(jo[1]) < 0.01 and abs(jo[2] - HINGE_Z) < 0.015,
        details=f"origin={jo}, expected=({HINGE_X}, 0, {HINGE_Z})",
    )
    # Hinge axis is horizontal (no Z component in world).
    ctx.check(
        "hinge axis is horizontal (no vertical component)",
        abs(ax[2]) < 0.02,
        details=f"axis_z={ax[2]}",
    )

    # ================================================================
    # 4. Lid swings clear of the mouth when opened
    # ================================================================
    with ctx.pose({joint: math.radians(75.0)}):
        lid_open_aabb = ctx.part_element_world_aabb(lid, elem="lid_disk")
    # At 75°, the lid center should be well above the rim (the near edge
    # stays near the hinge by design, so we check the center height).
    lid_open_cz = (lid_open_aabb[0][2] + lid_open_aabb[1][2]) / 2.0
    lid_rest_cz = (lid_aabb[0][2] + lid_aabb[1][2]) / 2.0
    ctx.check(
        "lid swings clear - center rises well above rim at 75 deg",
        lid_open_cz > RIM_Z + 0.05,
        details=f"open_lid_center_z={lid_open_cz:.4f}, rim_z={RIM_Z:.4f}",
    )
    ctx.check(
        "lid center rises when opened",
        lid_open_cz > lid_rest_cz + 0.03,
        details=f"rest_cz={lid_rest_cz:.4f}, open_cz={lid_open_cz:.4f}",
    )

    # ================================================================
    # Bucket identity checks (preserved from parent)
    # ================================================================
    shell_aabb = ctx.part_element_world_aabb(bucket, elem="bucket_shell")
    ctx.check(
        "flat bottom rests on the ground at z~0",
        abs(shell_aabb[0][2]) < 0.002,
        details=f"shell_minZ={shell_aabb[0][2]:.4f}",
    )
    ctx.check(
        "bucket height matches design (~0.26 m)",
        abs((shell_aabb[1][2] - shell_aabb[0][2]) - BODY_H) < 0.01,
        details=f"shell_h={shell_aabb[1][2] - shell_aabb[0][2]:.4f}",
    )
    rim_aabb = ctx.part_element_world_aabb(bucket, elem="rolled_rim")
    top_w = rim_aabb[1][0] - rim_aabb[0][0]
    bot_d = 2.0 * BOT_R
    ctx.check(
        "tapered: open top wider than flat bottom",
        top_w > bot_d + 0.03,
        details=f"top_w={top_w:.4f}, bottom_d={bot_d:.4f}",
    )
    ctx.check(
        "body modeled hollow (thin wall, open interior)",
        WALL < TOP_R * 0.05 and BOTTOM_T < BODY_H * 0.1,
        details=f"wall={WALL}, bottom_t={BOTTOM_T}, top_r={TOP_R}",
    )

    # Colors
    shell_rgba = bucket.get_visual("bucket_shell").material.rgba
    ctx.check(
        "bucket body is red",
        shell_rgba[0] > 0.5 and shell_rgba[1] < 0.25 and shell_rgba[2] < 0.25,
        details=f"shell_rgba={shell_rgba}",
    )
    lid_rgba = lid.get_visual("lid_disk").material.rgba
    ctx.check(
        "lid is red",
        lid_rgba[0] > 0.5 and lid_rgba[1] < 0.25 and lid_rgba[2] < 0.25,
        details=f"lid_rgba={lid_rgba}",
    )

    return ctx.report()


object_model = build_object_model()
