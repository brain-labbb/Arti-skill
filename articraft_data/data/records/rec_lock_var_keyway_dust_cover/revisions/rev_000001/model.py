from __future__ import annotations

# Articraft model: brass padlock with tall steel shackle + hinged dust cover.
#
# Family fork variant of the brass padlock: adds a hinged swinging dust cover
# over the keyway. Body and shackle are identical to the parent.
#
# Articraft brief
# - Object: classic brass body padlock with tall hardened-steel U-shackle and
#   a hinged brass dust cover flap shielding the keyway from debris.
# - Root/support: the brass body is the root. Its top face carries two deep
#   bored holes for the shackle legs. A chrome hinge bracket is mounted on the
#   front face above the keyway disc.
# - Parts: body (brass block, chrome keyway disc + slot, chrome hinge bracket),
#   shackle_lift (kinematic slider), shackle (continuous steel U), dust_cover
#   (brass flap with grip lip).
# - Articulations:
#   1. body_to_shackle: PRISMATIC +Z. The shackle first lifts upward to clear
#      the free leg from the body.
#   2. shackle_rotate: REVOLUTE about the retained long-leg vertical axis.
#      After lifting, positive q swings the free leg sideways like an opened
#      padlock.
#   3. body_to_dust_cover: REVOLUTE around -X at the hinge point above the
#      keyway. Positive q swings the cover outward and upward, exposing the
#      keyway. Limits 0 to ~120°.
# - Support/fit: shackle legs in body bores (retained insertion); dust cover
#   hangs from the hinge bracket on the body front face.
# - Intentional overlap: shackle legs in body bores (scoped allow_overlap);
#   hinge bracket embeds slightly into the body front face.
# - Tests: all parent shackle/keyway tests preserved; shackle lift + rotate
#   joints are present; dust cover joint is revolute with horizontal axis,
#   closed cover shields keyway, open cover swings outward exposing the keyway.

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
SHACKLE_SWING = 2.05  # ~117 degrees around the retained long-leg axis


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

# --- Dust cover (hinged keyway shield) --------------------------------------
DISC_FRONT_Y = FRONT_Y - DISC_T + DISC_EMBED   # keyway disc outer face Y
HINGE_Y = DISC_FRONT_Y - 0.0003                # hinge sits just in front of disc
HINGE_Z = 0.0028 + 0.0072 + 0.0012             # above disc top edge

COVER_W = 0.017          # dust cover width (X)
COVER_H = 0.017          # dust cover height (Z, hinge to bottom)
COVER_T = 0.0012         # dust cover thickness (Y)
COVER_FILLET = 0.003     # corner rounding

BRACKET_W = 0.014        # hinge bracket mounting plate width
BRACKET_H = 0.005        # bracket height
BARREL_R = 0.0012        # hinge pin radius
BARREL_LEN = 0.013       # hinge pin length (slightly shorter than cover width)

COVER_OPEN_ANGLE = 2.1   # ~120° open


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
    local origin lies on the retained long-leg vertical axis; the revolute joint
    places that origin at x=-LEG_OFFSET in the sliding frame.
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


def _hinge_bracket_shape() -> cq.Workplane:
    """Hinge bracket: mounting plate + pin barrel, authored in body frame.

    The bracket mounts on the body front face above the keyway disc. A small
    plate embeds into the body and protrudes outward to carry the hinge barrel.
    Note: XZ workplane extrude goes in -Y (workplane normal is -Y).
    """
    # Plate embeds 2mm into the body for solid mesh connectivity.
    plate_inner_y = FRONT_Y + 0.002  # well inside the body
    mount_depth = plate_inner_y - HINGE_Y  # total plate depth

    # Mounting plate: rect on XZ plane, extrude goes in -Y.
    # Raw extrude Y: [-mount_depth, 0].
    # After translate: outer face at HINGE_Y, inner face at HINGE_Y + mount_depth.
    plate = (
        cq.Workplane("XZ")
        .rect(BRACKET_W, BRACKET_H)
        .extrude(mount_depth)
    )
    plate = plate.edges("|Y").fillet(0.0008)
    # Shift so outer face at Y=HINGE_Y: translate Y by HINGE_Y + mount_depth
    plate = plate.translate((0.0, HINGE_Y + mount_depth, HINGE_Z))

    # Hinge pin barrel: horizontal cylinder along X at the hinge point.
    # YZ extrude goes in +X; circle is in world YZ plane.
    barrel = (
        cq.Workplane("YZ")
        .circle(BARREL_R)
        .extrude(BARREL_LEN)
        .translate((-BARREL_LEN / 2.0, 0.0, 0.0))
    )
    barrel = barrel.translate((0.0, HINGE_Y, HINGE_Z))

    return plate.union(barrel)


def _dust_cover_shape() -> cq.Workplane:
    """Dust cover flap, authored in the part's local frame.

    Origin at the hinge point (top center of the cover).
    Cover extends downward (-Z) and outward (-Y) from the hinge.
    Inner face (toward the body) at local Y = 0.

    Note: XZ workplane extrude goes in -Y (workplane normal is -Y), so
    the raw extrude Y range is [-COVER_T, 0]: inner face at Y=0 (correct),
    outer face at Y=-COVER_T (outward from body).
    """
    # Rounded rectangle flap
    flap = (
        cq.Workplane("XZ")
        .rect(COVER_W, COVER_H)
        .extrude(COVER_T)
    )
    flap = flap.edges("|Y").fillet(COVER_FILLET)
    # Extrude Y: [-COVER_T, 0] — already correct (inner face at Y=0).
    # rect Z: [-COVER_H/2, COVER_H/2] → shift by -COVER_H/2 for top at Z=0.
    flap = flap.translate((0.0, 0.0, -COVER_H / 2.0))

    # Small grip lip at the bottom edge (thin ridge for finger grip)
    lip = (
        cq.Workplane("XZ")
        .rect(COVER_W * 0.5, 0.002)
        .extrude(COVER_T + 0.001)
    )
    lip = lip.edges("|Y").fillet(0.0005)
    # Lip Y: [-(COVER_T+0.001), 0] — protrudes 1mm further outward than flap.
    # Position at bottom of cover: Z centered at -COVER_H - 0.001.
    lip = lip.translate((0.0, 0.0, -COVER_H - 0.001))

    return flap.union(lip)


# --- Model ------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="brass_padlock_dust_cover")

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

    # Hinge bracket on body (stationary hardware above the keyway)
    body.visual(
        mesh_from_cadquery(_hinge_bracket_shape(), "hinge_bracket"),
        material=chrome,
        name="hinge_bracket",
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

    # Dust cover: hinged flap that shields the keyway from dust and debris.
    dust_cover = model.part("dust_cover")
    dust_cover.visual(
        mesh_from_cadquery(_dust_cover_shape(), "cover_flap"),
        material=brass,
        name="cover_flap",
    )

    # Revolute hinge at the top of the keyway area.
    # axis = (-1, 0, 0): right-hand rule around -X makes the bottom of the
    # cover swing outward (-Y) when q increases, opening the dust shield.
    model.articulation(
        "body_to_dust_cover",
        ArticulationType.REVOLUTE,
        parent=body,
        child=dust_cover,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=COVER_OPEN_ANGLE,
            effort=2.0,
            velocity=2.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    shackle_lift = object_model.get_part("shackle_lift")
    shackle = object_model.get_part("shackle")
    dust_cover = object_model.get_part("dust_cover")
    shackle_joint = object_model.get_articulation("body_to_shackle")
    shackle_rotate = object_model.get_articulation("shackle_rotate")
    cover_joint = object_model.get_articulation("body_to_dust_cover")

    # --- Shackle joint (identical to parent) --------------------------------
    ctx.check(
        "shackle joint is prismatic",
        shackle_joint.joint_type == ArticulationType.PRISMATIC
        or str(shackle_joint.joint_type).lower().endswith("prismatic"),
        details=f"joint_type={shackle_joint.joint_type}",
    )
    axis = tuple(shackle_joint.axis)
    ctx.check(
        "shackle lifts along +Z",
        abs(axis[2]) > 0.99 and abs(axis[0]) < 1e-6 and abs(axis[1]) < 1e-6,
        details=f"axis={axis}",
    )
    ctx.check(
        "shackle rotate joint is revolute",
        shackle_rotate.joint_type == ArticulationType.REVOLUTE
        or str(shackle_rotate.joint_type).lower().endswith("revolute"),
        details=f"joint_type={shackle_rotate.joint_type}",
    )
    rotate_axis = tuple(shackle_rotate.axis)
    ctx.check(
        "shackle rotates around retained vertical leg",
        abs(rotate_axis[2]) > 0.99 and abs(rotate_axis[0]) < 1e-6 and abs(rotate_axis[1]) < 1e-6,
        details=f"axis={rotate_axis}, origin={shackle_rotate.origin.xyz}",
    )
    ctx.check(
        "shackle rotate joint hangs from the lifted frame",
        shackle_rotate.parent == shackle_lift.name and shackle_rotate.child == shackle.name,
        details=f"parent={shackle_rotate.parent}, child={shackle_rotate.child}",
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

    shackle_x_span = shackle_aabb[1][0] - shackle_aabb[0][0]
    ctx.check(
        "shackle spans both legs in X",
        shackle_x_span > 2.0 * LEG_OFFSET - 0.001,
        details=f"x_span={shackle_x_span:.4f}",
    )

    # Keyway escutcheon sits proud of the front face.
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

    # Locked pose: shackle legs seated in body.
    with ctx.pose({shackle_joint: 0.0, shackle_rotate: 0.0}):
        locked_shackle_bottom = ctx.part_world_aabb(shackle)[0][2]
        ctx.check(
            "locked shackle legs seated in body",
            locked_shackle_bottom < BODY_H - 0.005,
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

    # Unlocked pose: shackle lifted, long leg still inserted.
    rest_bottom = ctx.part_world_aabb(shackle)[0][2]
    with ctx.pose({shackle_joint: SHACKLE_TRAVEL, shackle_rotate: 0.0}):
        lifted_bottom = ctx.part_world_aabb(shackle)[0][2]
        ctx.check(
            "lifting moves the shackle upward",
            lifted_bottom > rest_bottom + 0.5 * SHACKLE_TRAVEL,
            details=f"rest_bottom={rest_bottom:.4f}, lifted_bottom={lifted_bottom:.4f}",
        )
        ctx.expect_overlap(
            shackle,
            body,
            axes="z",
            elem_a="shackle_bar",
            elem_b="body_shell",
            min_overlap=0.002,
            name="long leg stays inserted at full travel",
        )

    # Open shackle pose: after the vertical lift, the free side rotates away
    # around the retained long leg.
    with ctx.pose({shackle_joint: SHACKLE_TRAVEL, shackle_rotate: 0.0}):
        lifted_closed = ctx.part_world_aabb(shackle)
    with ctx.pose({shackle_joint: SHACKLE_TRAVEL, shackle_rotate: SHACKLE_SWING}):
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

    # --- Dust cover mechanism -----------------------------------------------

    # Joint type and axis for the hinged dust cover.
    ctx.check(
        "dust cover joint is revolute",
        cover_joint.joint_type == ArticulationType.REVOLUTE
        or str(cover_joint.joint_type).lower().endswith("revolute"),
        details=f"joint_type={cover_joint.joint_type}",
    )
    cover_axis = tuple(cover_joint.axis)
    ctx.check(
        "dust cover hinge axis is horizontal (X)",
        abs(cover_axis[0]) > 0.99 and abs(cover_axis[1]) < 1e-6 and abs(cover_axis[2]) < 1e-6,
        details=f"axis={cover_axis}",
    )

    # Hinge bracket visible on the body above the keyway.
    bracket_aabb = ctx.part_element_world_aabb(body, elem="hinge_bracket")
    assert bracket_aabb is not None
    ctx.check(
        "hinge bracket above keyway disc",
        bracket_aabb[1][2] > disc_aabb[1][2] - 0.001,
        details=f"bracket_top_z={bracket_aabb[1][2]:.4f}, disc_top_z={disc_aabb[1][2]:.4f}",
    )

    # Closed pose (q=0): dust cover hangs over the keyway disc area.
    with ctx.pose({cover_joint: 0.0}):
        cover_aabb_closed = ctx.part_world_aabb(dust_cover)
        assert cover_aabb_closed is not None

        # Cover overlaps the keyway disc in XZ projection (covers the keyway).
        ctx.expect_overlap(
            dust_cover,
            body,
            axes="xz",
            elem_a="cover_flap",
            elem_b="keyway_disc",
            min_overlap=0.005,
            name="closed dust cover shields the keyway",
        )

        # Cover is in front of (more negative Y than) the keyway disc.
        ctx.check(
            "closed cover sits in front of keyway disc",
            cover_aabb_closed[1][1] < disc_aabb[0][1] + 0.001,
            details=f"cover_back_y={cover_aabb_closed[1][1]:.4f}, disc_front_y={disc_aabb[0][1]:.4f}",
        )

        closed_bottom_z = cover_aabb_closed[0][2]

    # Open pose: dust cover swings outward, exposing the keyway.
    with ctx.pose({cover_joint: COVER_OPEN_ANGLE}):
        cover_aabb_open = ctx.part_world_aabb(dust_cover)
        assert cover_aabb_open is not None

        # Bottom of the cover should be higher when open (swung up and out).
        open_bottom_z = cover_aabb_open[0][2]
        ctx.check(
            "open cover bottom lifts above closed position",
            open_bottom_z > closed_bottom_z + 0.005,
            details=f"closed_bottom={closed_bottom_z:.4f}, open_bottom={open_bottom_z:.4f}",
        )

        # Cover outer face should be further outward when open (swung away).
        open_front_y = cover_aabb_open[0][1]
        closed_front_y = cover_aabb_closed[0][1]
        ctx.check(
            "open cover swings outward from the body",
            open_front_y < closed_front_y - 0.003
            or cover_aabb_open[1][2] > cover_aabb_closed[1][2] + 0.003,
            details=f"closed_front_y={closed_front_y:.4f}, open_front_y={open_front_y:.4f}, "
                    f"closed_top_z={cover_aabb_closed[1][2]:.4f}, open_top_z={cover_aabb_open[1][2]:.4f}",
        )

    return ctx.report()


object_model = build_object_model()
