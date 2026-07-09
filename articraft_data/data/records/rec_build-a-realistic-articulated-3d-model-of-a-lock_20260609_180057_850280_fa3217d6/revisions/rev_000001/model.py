from __future__ import annotations

# Articraft model: a brass padlock with a tall hardened-steel U-shackle.
#
# Articraft brief
# - Object: classic brass body padlock with a tall round-bar steel shackle.
#   Body ~0.045 x 0.019 x 0.058 m; shackle round bar radius ~0.0058 m, arch
#   rising well above the body (long-shackle style as shown in the reference).
# - Root/support: the brass body is the root. Its top face carries two deep
#   bored holes that capture the two shackle legs.
# - Parts: body (brass block with bored top, chamfered edges, chrome keyway
#   disc + dark key slot on the front face) and shackle (continuous steel U).
# - Articulation: body_to_shackle, PRISMATIC along +Z. At q=0 (locked) both
#   shackle legs are fully seated in the body bores. Positive q lifts the
#   shackle straight up out of the body until the short leg clears, popping the
#   lock open. This is the defining padlock motion.
# - Support/fit: the shackle legs sit inside the two body bores; the long leg
#   stays captured at full travel (retained insertion).
# - Intentional overlap: the seated shackle legs nest inside the simplified
#   solid body bores -> scoped allow_overlap on those elements.
# - Tests: shackle present as a U above the body, body keyway present, joint is
#   prismatic +Z, raising it lifts the shackle clear of the body, long leg
#   stays inserted at full travel, locked pose seats the legs in the body.

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

SHACKLE_CLEAR_W = 2.0 * LEG_OFFSET          # inner span of the U
ARCH_TOP_Z = BODY_H + 0.072                 # world Z of the top of the arch
LEG_SEAT_DEPTH = 0.016                       # how deep legs sit into the body
SHORT_LEG_BOTTOM_Z = BODY_H - LEG_SEAT_DEPTH  # short (free) leg bottom when locked
LONG_LEG_BOTTOM_Z = BODY_H - (BODY_H * 0.62)  # long leg reaches deep into body

# Prismatic travel: lift far enough that the short leg fully clears the body.
SHACKLE_TRAVEL = LEG_SEAT_DEPTH + 0.006


# --- Geometry builders ------------------------------------------------------
def _body_shape() -> cq.Workplane:
    """Brass padlock body: rounded block with two top bores and a beveled top."""
    body = (
        cq.Workplane("XY")
        .box(BODY_W, BODY_D, BODY_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(BODY_EDGE_FILLET)
    )
    # Soften the top and bottom horizontal edges.
    body = body.edges(">Z or <Z").fillet(0.0018)

    # Two deep bores in the top face for the shackle legs.
    for sx in (-LEG_OFFSET, LEG_OFFSET):
        body = (
            body.faces(">Z")
            .workplane(centerOption="CenterOfBoundBox")
            .pushPoints([(sx, 0.0)])
            .hole(2.0 * BORE_R, depth=BODY_H * 0.66)
        )
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
    Continuous steel U-shackle authored in the BODY frame.

    Built from exact primitives so the U silhouette is predictable: a long leg
    (left), a short leg (right), and a half-torus arch joining them at the top.
    The legs are capsule-style (rounded bottoms) so they read as round bar.
    The part is authored so that the joint frame is the body frame and a pure
    +Z translation lifts the whole shackle.
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
    return shackle


# --- Model ------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="brass_padlock")

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

    # Shackle (steel U) authored in the body frame.
    shackle = model.part("shackle")
    shackle.visual(
        mesh_from_cadquery(_shackle_shape(), "shackle_bar"),
        material=steel,
        name="shackle_bar",
    )

    # Prismatic: the shackle lifts straight up out of the body when unlocked.
    model.articulation(
        "body_to_shackle",
        ArticulationType.PRISMATIC,
        parent=body,
        child=shackle,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=SHACKLE_TRAVEL,
            effort=60.0,
            velocity=0.1,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    shackle = object_model.get_part("shackle")
    joint = object_model.get_articulation("body_to_shackle")

    # Joint type and axis are the defining padlock mechanism.
    ctx.check(
        "joint is prismatic",
        joint.joint_type == ArticulationType.PRISMATIC
        or str(joint.joint_type).lower().endswith("prismatic"),
        details=f"joint_type={joint.joint_type}",
    )
    axis = tuple(joint.axis)
    ctx.check(
        "shackle lifts along +Z",
        abs(axis[2]) > 0.99 and abs(axis[0]) < 1e-6 and abs(axis[1]) < 1e-6,
        details=f"axis={axis}",
    )

    # The seated legs nest inside the simplified solid body bores.
    ctx.allow_overlap(
        body,
        shackle,
        elem_a="body_shell",
        elem_b="shackle_bar",
        reason="The shackle legs are intentionally seated inside the body bores when locked.",
    )

    # Hero geometry: shackle arches well above the body.
    body_aabb = ctx.part_world_aabb(body)
    shackle_aabb = ctx.part_world_aabb(shackle)
    assert body_aabb is not None and shackle_aabb is not None
    body_top_z = body_aabb[1][2]
    shackle_top_z = shackle_aabb[1][2]
    ctx.check(
        "shackle arches above the body",
        shackle_top_z > body_top_z + 0.05,
        details=f"shackle_top={shackle_top_z:.4f}, body_top={body_top_z:.4f}",
    )

    # Shackle spans the two leg positions in X (reads as a U, not a single bar).
    shackle_x_span = shackle_aabb[1][0] - shackle_aabb[0][0]
    ctx.check(
        "shackle spans both legs in X",
        shackle_x_span > 2.0 * LEG_OFFSET - 0.001,
        details=f"x_span={shackle_x_span:.4f}",
    )

    # Keyway escutcheon sits proud of the front (-Y) face of the body: the disc
    # outer face must extend beyond (more -Y than) the body front face.
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

    # Locked pose (q=0): both legs seated down inside the body.
    with ctx.pose({joint: 0.0}):
        locked_shackle_bottom = ctx.part_world_aabb(shackle)[0][2]
        ctx.check(
            "locked shackle legs seated in body",
            locked_shackle_bottom < BODY_H - 0.005,
            details=f"shackle_bottom={locked_shackle_bottom:.4f}, body_h={BODY_H:.4f}",
        )
        # The shackle leg overlaps the body in Z (retained insertion when locked).
        ctx.expect_overlap(
            shackle,
            body,
            axes="z",
            elem_a="shackle_bar",
            elem_b="body_shell",
            min_overlap=0.008,
            name="locked shackle retained in body",
        )

    # Unlocked / fully lifted pose: shackle has moved up and the short (free)
    # leg has cleared the body top, popping the lock open.
    rest_bottom = ctx.part_world_aabb(shackle)[0][2]
    with ctx.pose({joint: SHACKLE_TRAVEL}):
        lifted_bottom = ctx.part_world_aabb(shackle)[0][2]
        ctx.check(
            "lifting moves the shackle upward",
            lifted_bottom > rest_bottom + 0.5 * SHACKLE_TRAVEL,
            details=f"rest_bottom={rest_bottom:.4f}, lifted_bottom={lifted_bottom:.4f}",
        )
        # Long leg must still remain inserted in the body (retained insertion).
        ctx.expect_overlap(
            shackle,
            body,
            axes="z",
            elem_a="shackle_bar",
            elem_b="body_shell",
            min_overlap=0.002,
            name="long leg stays inserted at full travel",
        )

    return ctx.report()


object_model = build_object_model()
