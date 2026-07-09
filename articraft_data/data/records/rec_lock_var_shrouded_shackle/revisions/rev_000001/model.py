from __future__ import annotations

# Articraft model: brass padlock with a short shrouded U-shackle.
#
# Family fork of the key padlock parent: shroud variant.
# Changed layer: shackle -> short shrouded U with raised protective shoulders.
# Identical layers: body block, keyway escutcheon, key slot.
#
# Articraft brief
# - Object: brass body padlock with a short shrouded hardened-steel U-shackle.
#   Body ~0.045 x 0.019 x 0.058 m; two raised protective shoulders (shrouds)
#   rise 0.020 m above the body top around each shackle leg; short shackle
#   arch protrudes only ~0.042 m above the main body block.
# - Root/support: the brass body is the root. Its top face carries two raised
#   shoulder blocks with deep bored holes that capture the shackle legs.
# - Parts: body (brass block + raised shoulders + chrome keyway disc + dark
#   key slot), shackle_lift (kinematic slider), and shackle (continuous short
#   steel U).
# - Articulation:
#   1. body_to_shackle, PRISMATIC along +Z. At q=0 (locked) both shackle legs
#      are fully seated in the body/shoulder bores. Positive q lifts the
#      shackle straight up until the short leg clears the shoulder tops.
#   2. shackle_rotate, REVOLUTE about the retained long-leg vertical
#      axis. After lifting, positive q swings the free leg sideways like a
#      real opened padlock shackle.
# - Support/fit: the shackle legs pass through the shoulder bores into the
#   body; the long leg stays captured at full travel (retained insertion).
# - Intentional overlap: the seated shackle legs nest inside the simplified
#   solid body/shoulder bores -> scoped allow_overlap on those elements.
# - Tests: shackle is short and shrouded (arches above shoulders but not far
#   above body), body has raised protective shoulders, keyway present,
#   prismatic lift + revolute swing joints exist, travel clears shoulders, long
#   leg stays inserted.

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# --- Dimensions (meters) ----------------------------------------------------
BODY_W = 0.045          # width (X)
BODY_D = 0.019          # depth (Y)
BODY_H = 0.058          # height (Z)
BODY_EDGE_FILLET = 0.0045

LEG_OFFSET = 0.0135     # half-distance between the two shackle legs (X)
BAR_R = 0.0058          # shackle round-bar radius
# Bore is a touch tighter than the bar so the seated leg contacts/captures in
# the body (a small intentional interpenetration covered by allow_overlap).
BORE_R = BAR_R - 0.0004  # body bore radius for the legs

# --- Shroud (raised protective shoulders) ------------------------------------
SHROULDER_H = 0.020          # shoulder height above body top
SHROULDER_W = 0.016          # shoulder width (X) per side
SHROULDER_D = BODY_D * 0.80  # shoulder depth (Y)
SHROULDER_FILLET_V = 0.003   # vertical edge fillet on shoulders
SHROULDER_FILLET_TOP = 0.002  # top edge fillet on shoulders

SHACKLE_CLEAR_W = 2.0 * LEG_OFFSET          # inner span of the U
# Short shrouded shackle: arch spring line just clears the raised shoulders.
ARCH_TOP_Z = BODY_H + SHROULDER_H + 0.003 + LEG_OFFSET  # top of arch centerline
LEG_SEAT_DEPTH = 0.016                       # how deep legs sit into the body
SHORT_LEG_BOTTOM_Z = BODY_H - LEG_SEAT_DEPTH  # short (free) leg bottom when locked
LONG_LEG_BOTTOM_Z = BODY_H * 0.26             # long leg reaches deep for retained insertion

# Prismatic travel: lift far enough that the short leg clears the shoulder tops.
SHACKLE_TRAVEL = SHROULDER_H + LEG_SEAT_DEPTH + 0.004
SHACKLE_SWING = 2.05  # ~117 degrees around the retained long-leg axis


# --- Geometry builders ------------------------------------------------------
def _make_shoulder(x_center: float) -> cq.Workplane:
    """Raised protective shoulder plate around one shackle leg position."""
    sh = (
        cq.Workplane("XY")
        .box(SHROULDER_W, SHROULDER_D, SHROULDER_H, centered=(True, True, False))
    )
    sh = sh.edges("|Z").fillet(SHROULDER_FILLET_V)
    sh = sh.edges(">Z").fillet(SHROULDER_FILLET_TOP)
    sh = sh.translate((x_center, 0.0, BODY_H))
    return sh


def _body_shape() -> cq.Workplane:
    """Brass padlock body with raised protective shoulders and shackle bores."""
    body = (
        cq.Workplane("XY")
        .box(BODY_W, BODY_D, BODY_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(BODY_EDGE_FILLET)
    )
    # Soften the top and bottom horizontal edges.
    body = body.edges(">Z or <Z").fillet(0.0018)

    # Add raised protective shoulders on the top face around each shackle leg.
    for sx in (-LEG_OFFSET, LEG_OFFSET):
        body = body.union(_make_shoulder(sx))

    # Cut shackle bores through the shoulders and deep into the body.
    bore_depth = SHROULDER_H + BODY_H * 0.66
    for sx in (-LEG_OFFSET, LEG_OFFSET):
        bore = (
            cq.Workplane("XY")
            .transformed(offset=(sx, 0.0, BODY_H + SHROULDER_H + 0.001))
            .circle(BORE_R)
            .extrude(-(bore_depth + 0.002))
        )
        body = body.cut(bore)
    return body


FRONT_Y = -BODY_D / 2.0     # body front face (faces -Y, outward)
DISC_T = 0.0016             # keyway escutcheon thickness
SLOT_T = 0.0006            # key-slot relief thickness
DISC_EMBED = 0.0005        # disc inner face embeds into the body for contact


def _keyway_disc_shape() -> cq.Workplane:
    """Chrome keyway escutcheon: a shallow disc proud of the front face.

    Authored on the XZ plane (normal -Y), so the raw extrude spans y in
    [-DISC_T, 0]. Translated so the disc spans y in
    [FRONT_Y - DISC_T + DISC_EMBED, FRONT_Y + DISC_EMBED]: it stands proud of
    the front face while its inner lip embeds slightly into the body.
    """
    disc = cq.Workplane("XZ").circle(0.0072).extrude(DISC_T)
    disc = disc.translate((0.0, FRONT_Y + DISC_EMBED, 0.0))
    return disc


def _keyway_slot_shape() -> cq.Workplane:
    """Dark key slot: a thin keyhole-style slot proud of the disc face."""
    # Disc front (outer) face sits at FRONT_Y - DISC_T + DISC_EMBED.
    disc_front_y = FRONT_Y - DISC_T + DISC_EMBED
    slot = (
        cq.Workplane("XZ")
        .moveTo(0.0, 0.0028)
        .rect(0.0017, 0.0075)
        .extrude(SLOT_T)
    )
    keyhole = (
        cq.Workplane("XZ")
        .moveTo(0.0, 0.0028)
        .circle(0.0017)
        .extrude(SLOT_T)
    )
    keyway = slot.union(keyhole)
    # Raw spans y in [-SLOT_T, 0]; place so it overlaps into the disc front
    # face (inner edge embeds in the disc, outer edge proud of it).
    keyway = keyway.translate((0.0, disc_front_y + SLOT_T * 0.5, 0.0))
    return keyway


def _shackle_shape() -> cq.Workplane:
    """
    Continuous steel U-shackle authored in the SHACKLE local frame.

    Built from exact primitives so the U silhouette is predictable: a long leg
    (left), a short leg (right), and a half-torus arch joining them at the top.
    The legs are capsule-style (rounded bottoms) so they read as round bar. The
    local origin lies on the retained long-leg vertical axis. At zero rotation,
    the hinge places that local origin at x=-LEG_OFFSET in the sliding frame,
    recreating the original locked silhouette.
    """
    arch_center_z = ARCH_TOP_Z - LEG_OFFSET  # semicircle arch, radius LEG_OFFSET

    # Half-torus arch over the top (upper half, from -X leg to +X leg).
    # makeTorus builds a ring in the XY plane (hole axis +Z). Rotate it so the
    # hole axis points along +Y, putting the ring upright in the XZ plane, then
    # keep only its upper half and lift it to the arch center.
    torus_solid = cq.Solid.makeTorus(LEG_OFFSET, BAR_R)
    arch = cq.Workplane(obj=torus_solid).rotate((0, 0, 0), (1, 0, 0), 90.0)
    arch = arch.translate((0.0, 0.0, arch_center_z))
    half_box = (
        cq.Workplane("XY")
        .box(4.0 * LEG_OFFSET, 4.0 * BAR_R, 2.0 * (ARCH_TOP_Z + LEG_OFFSET))
        .translate((0.0, 0.0, arch_center_z + (ARCH_TOP_Z + LEG_OFFSET)))
    )
    arch = arch.intersect(half_box)

    # Long leg (left, -X): from deep in the body up to the arch joint.
    long_len = arch_center_z - LONG_LEG_BOTTOM_Z
    long_leg = (
        cq.Workplane("XY")
        .center(-LEG_OFFSET, 0.0)
        .circle(BAR_R)
        .extrude(long_len)
        .translate((0.0, 0.0, LONG_LEG_BOTTOM_Z))
    )

    # Short leg (right, +X): from a shallower bottom up to the arch joint.
    short_len = arch_center_z - SHORT_LEG_BOTTOM_Z
    short_leg = (
        cq.Workplane("XY")
        .center(LEG_OFFSET, 0.0)
        .circle(BAR_R)
        .extrude(short_len)
        .translate((0.0, 0.0, SHORT_LEG_BOTTOM_Z))
    )

    shackle = long_leg.union(short_leg).union(arch)
    # Round the free bottom tips of the legs so they read as machined bar ends.
    shackle = shackle.edges("<Z").fillet(BAR_R * 0.45)
    return shackle.translate((LEG_OFFSET, 0.0, 0.0))


# --- Model ------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="shrouded_padlock")

    brass = model.material("brass", rgba=(0.83, 0.66, 0.18, 1.0))
    steel = model.material("hardened_steel", rgba=(0.74, 0.76, 0.80, 1.0))
    chrome = model.material("chrome", rgba=(0.86, 0.88, 0.90, 1.0))
    keyslot_dark = model.material("keyway_slot", rgba=(0.07, 0.07, 0.08, 1.0))

    # Body (root)
    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shape(), "body_shell"),
        material=brass,
        name="body_shell",
    )
    body.visual(
        mesh_from_cadquery(_keyway_disc_shape(), "keyway_disc"),
        material=chrome,
        name="keyway_disc",
    )
    body.visual(
        mesh_from_cadquery(_keyway_slot_shape(), "keyway_slot"),
        material=keyslot_dark,
        name="keyway_slot",
    )

    # Kinematic lift frame: invisible slider that rises with the retained leg.
    shackle_lift = model.part("shackle_lift")

    # Shackle (steel U) authored around the retained long-leg axis.
    shackle = model.part("shackle")
    shackle.visual(
        mesh_from_cadquery(_shackle_shape(), "shackle_bar"),
        material=steel,
        name="shackle_bar",
    )

    # Prismatic: the shackle lift frame rises straight up out of the body.
    model.articulation(
        "body_to_shackle",
        ArticulationType.PRISMATIC,
        parent=body,
        child=shackle_lift,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=SHACKLE_TRAVEL,
            effort=60.0,
            velocity=0.1,
        ),
    )

    # Revolute: once lifted, the shackle swings around the retained long leg.
    model.articulation(
        "shackle_rotate",
        ArticulationType.REVOLUTE,
        parent=shackle_lift,
        child=shackle,
        origin=Origin(xyz=(-LEG_OFFSET, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=SHACKLE_SWING,
            effort=8.0,
            velocity=1.5,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    shackle_lift = object_model.get_part("shackle_lift")
    shackle = object_model.get_part("shackle")
    lift_joint = object_model.get_articulation("body_to_shackle")
    swing_joint = object_model.get_articulation("shackle_rotate")

    # Joint type and axis are the defining padlock mechanism.
    ctx.check(
        "shackle lift joint is prismatic",
        lift_joint.joint_type == ArticulationType.PRISMATIC
        or str(lift_joint.joint_type).lower().endswith("prismatic"),
        details=f"joint_type={lift_joint.joint_type}",
    )
    axis = tuple(lift_joint.axis)
    ctx.check(
        "shackle lifts along +Z",
        abs(axis[2]) > 0.99 and abs(axis[0]) < 1e-6 and abs(axis[1]) < 1e-6,
        details=f"axis={axis}",
    )
    ctx.check(
        "shackle swing joint is revolute",
        swing_joint.joint_type == ArticulationType.REVOLUTE
        or str(swing_joint.joint_type).lower().endswith("revolute"),
        details=f"joint_type={swing_joint.joint_type}",
    )
    swing_axis = tuple(swing_joint.axis)
    ctx.check(
        "shackle swings around retained vertical leg",
        abs(swing_axis[2]) > 0.99 and abs(swing_axis[0]) < 1e-6 and abs(swing_axis[1]) < 1e-6,
        details=f"axis={swing_axis}, origin={swing_joint.origin.xyz}",
    )
    ctx.check(
        "swing joint hangs from the lifted frame",
        swing_joint.parent == shackle_lift.name and swing_joint.child == shackle.name,
        details=f"parent={swing_joint.parent}, child={swing_joint.child}",
    )

    # The seated legs nest inside the simplified solid body/shoulder bores.
    ctx.allow_overlap(
        body,
        shackle,
        elem_a="body_shell",
        elem_b="shackle_bar",
        reason="The shackle legs are intentionally seated inside the body/shoulder bores when locked.",
    )

    # --- Shrouded variant geometry checks ---
    body_aabb = ctx.part_world_aabb(body)
    shackle_aabb = ctx.part_world_aabb(shackle)
    assert body_aabb is not None and shackle_aabb is not None
    body_top_z = body_aabb[1][2]  # includes shoulders
    body_bottom_z = body_aabb[0][2]
    main_body_top = body_bottom_z + BODY_H  # main body block top (excl. shoulders)
    shackle_top_z = shackle_aabb[1][2]

    # Body has raised protective shoulders above the main body block.
    ctx.check(
        "body has raised protective shoulders",
        body_top_z > main_body_top + 0.010,
        details=f"body_top={body_top_z:.4f}, main_body_top={main_body_top:.4f}",
    )

    # Shackle arch protrudes above the shrouded shoulders.
    ctx.check(
        "shackle arches above the shoulders",
        shackle_top_z > body_top_z + 0.010,
        details=f"shackle_top={shackle_top_z:.4f}, body_top={body_top_z:.4f}",
    )

    # Shackle is short: total height above main body block is much less than
    # a standard long-shackle padlock (limited to <50 mm).
    shackle_above_body = shackle_top_z - main_body_top
    ctx.check(
        "shackle is short (shrouded style)",
        shackle_above_body < 0.050,
        details=f"shackle_above_body={shackle_above_body:.4f}",
    )

    # Shackle spans both legs in X (reads as a U, not a single bar).
    shackle_x_span = shackle_aabb[1][0] - shackle_aabb[0][0]
    ctx.check(
        "shackle spans both legs in X",
        shackle_x_span > 2.0 * LEG_OFFSET - 0.001,
        details=f"x_span={shackle_x_span:.4f}",
    )

    # Keyway escutcheon sits proud of the front (-Y) face of the body.
    body_shell_aabb = ctx.part_element_world_aabb(body, elem="body_shell")
    disc_aabb = ctx.part_element_world_aabb(body, elem="keyway_disc")
    slot_aabb = ctx.part_element_world_aabb(body, elem="keyway_slot")
    assert body_shell_aabb is not None and disc_aabb is not None and slot_aabb is not None
    ctx.check(
        "keyway disc proud of front face",
        disc_aabb[0][1] < body_shell_aabb[0][1] - 0.0005,
        details=f"disc_front_y={disc_aabb[0][1]:.4f}, body_front_y={body_shell_aabb[0][1]:.4f}",
    )
    ctx.check(
        "key slot proud of keyway disc",
        slot_aabb[0][1] < disc_aabb[0][1] - 0.0001,
        details=f"slot_front_y={slot_aabb[0][1]:.4f}, disc_front_y={disc_aabb[0][1]:.4f}",
    )

    # Locked pose (q=0): both legs seated down inside the body/shoulders.
    with ctx.pose({lift_joint: 0.0, swing_joint: 0.0}):
        locked_shackle_bottom = ctx.part_world_aabb(shackle)[0][2]
        ctx.check(
            "locked shackle legs seated in body",
            locked_shackle_bottom < main_body_top - 0.005,
            details=f"shackle_bottom={locked_shackle_bottom:.4f}, body_h={BODY_H:.4f}",
        )
        ctx.expect_overlap(
            shackle,
            body,
            axes="z",
            elem_a="shackle_bar",
            elem_b="body_shell",
            min_overlap=0.008,
            name="locked shackle retained in body",
        )

    # Unlocked / fully lifted pose: short leg clears the shoulder tops.
    rest_bottom = ctx.part_world_aabb(shackle)[0][2]
    with ctx.pose({lift_joint: SHACKLE_TRAVEL, swing_joint: 0.0}):
        lifted_bottom = ctx.part_world_aabb(shackle)[0][2]
        ctx.check(
            "lifting moves the shackle upward",
            lifted_bottom > rest_bottom + 0.5 * SHACKLE_TRAVEL,
            details=f"rest_bottom={rest_bottom:.4f}, lifted_bottom={lifted_bottom:.4f}",
        )
        # Short leg clears the shoulder tops at full travel.
        # The shackle AABB bottom is the long leg (retained in body), so verify
        # the short leg clearance from the known geometry and travel.
        short_leg_lifted = SHORT_LEG_BOTTOM_Z + SHACKLE_TRAVEL
        ctx.check(
            "short leg clears shoulder tops at full travel",
            short_leg_lifted > body_top_z - 0.001,
            details=f"short_leg_lifted={short_leg_lifted:.4f}, shoulder_top={body_top_z:.4f}",
        )
        # Long leg must still remain inserted in the body.
        ctx.expect_overlap(
            shackle,
            body,
            axes="z",
            elem_a="shackle_bar",
            elem_b="body_shell",
            min_overlap=0.002,
            name="long leg stays inserted at full travel",
        )

    # Fully opened pose: after the vertical lift, the free leg rotates away
    # around the retained long leg.
    with ctx.pose({lift_joint: SHACKLE_TRAVEL, swing_joint: 0.0}):
        lifted_closed = ctx.part_world_aabb(shackle)
    with ctx.pose({lift_joint: SHACKLE_TRAVEL, swing_joint: SHACKLE_SWING}):
        swung_open = ctx.part_world_aabb(shackle)
        ctx.check(
            "rotation swings the free side of the shackle sideways",
            swung_open[1][1] - swung_open[0][1] > lifted_closed[1][1] - lifted_closed[0][1] + 0.010,
            details=f"lifted_closed={lifted_closed}, swung_open={swung_open}",
        )
        ctx.expect_overlap(
            shackle,
            body,
            axes="z",
            elem_a="shackle_bar",
            elem_b="body_shell",
            min_overlap=0.002,
            name="retained long leg stays inserted while swung open",
        )

    return ctx.report()


object_model = build_object_model()
