from __future__ import annotations

# Articraft model: a laminated steel padlock with a hardened-steel U-shackle.
#
# Family fork of the brass padlock — body changed to a laminated steel plate
# stack held together with rivets. Shackle and keyway are identical to parent.
#
# Articraft brief
# - Object: laminated-steel body padlock with a tall round-bar steel shackle.
#   Body ~0.045 x 0.019 x 0.058 m made of 5 stacked horizontal steel plates
#   riveted together. Shackle round bar ~0.0058 m radius, long-shackle style.
# - Root/support: the laminated body is the root. Its plate stack carries two
#   bore channels that capture the shackle legs.
# - Parts: body (laminated plate stack with 4 through-rivets, chrome keyway
#   disc, dark key slot) and shackle (continuous steel U).
# - Articulation: body_to_shackle, PRISMATIC along +Z. At q=0 both legs are
#   fully seated. Positive q lifts the shackle straight up until the short leg
#   clears the plate stack, popping the lock open.
# - Support/fit: shackle legs nest inside the plate stack; the long leg stays
#   captured at full travel (retained insertion).
# - Intentional overlap: seated shackle legs inside the plate stack and rivet
#   shafts intersecting the leg paths — scoped allow_overlap on those elements.
# - Tests: body is laminated (multiple plate visuals stacked along depth),
#   rivets present and proud of both faces, shackle mechanism unchanged.

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

# --- Lamination -------------------------------------------------------------
NUM_PLATES = 5
PLATE_GAP = 0.00008     # hairline seam between plates (~80 µm)
PLATE_T = (BODY_D - (NUM_PLATES - 1) * PLATE_GAP) / NUM_PLATES
PLATE_FILLET = 0.0015   # vertical-edge fillet (< PLATE_T / 2)
PLATE_EDGE_CHAMFER = 0.00025  # top/bottom edge break

# --- Shackle (unchanged from parent) ----------------------------------------
LEG_OFFSET = 0.0135     # half-distance between the two shackle legs (X)
BAR_R = 0.0058          # shackle round-bar radius
BORE_R = BAR_R - 0.0004  # bore radius (tighter than bar for capture)

SHACKLE_CLEAR_W = 2.0 * LEG_OFFSET
ARCH_TOP_Z = BODY_H + 0.072
LEG_SEAT_DEPTH = 0.016
SHORT_LEG_BOTTOM_Z = BODY_H - LEG_SEAT_DEPTH
LONG_LEG_BOTTOM_Z = BODY_H - (BODY_H * 0.62)

SHACKLE_TRAVEL = LEG_SEAT_DEPTH + 0.006

# --- Rivets -----------------------------------------------------------------
NUM_RIVETS = 4
RIVET_HEAD_R = 0.0025
RIVET_HEAD_H = 0.0006
RIVET_SHAFT_R = 0.0018
RIVET_EMBED = 0.0003    # head embed depth for face connectivity
# (x, z) positions — 2 × 2 grid, wide in X, lower/upper in Z.
RIVET_XZ = [
    (-0.016, 0.012),
    ( 0.016, 0.012),
    (-0.016, 0.046),
    ( 0.016, 0.046),
]

# --- Keyway (unchanged from parent) -----------------------------------------
FRONT_Y = -BODY_D / 2.0
DISC_T = 0.0016
SLOT_T = 0.0006
DISC_EMBED = 0.0005


# --- Geometry builders ------------------------------------------------------

def _plate_shape(index: int) -> cq.Workplane:
    """Single laminated plate positioned at its Y station in the stack."""
    y_center = -BODY_D / 2.0 + PLATE_T / 2.0 + index * (PLATE_T + PLATE_GAP)
    plate = (
        cq.Workplane("XY")
        .box(BODY_W, PLATE_T, BODY_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(PLATE_FILLET)
    )
    # Break the top and bottom horizontal edges so each plate reads as a
    # distinct stamped/machined layer.
    plate = plate.edges(">Z or <Z").chamfer(PLATE_EDGE_CHAMFER)
    plate = plate.translate((0.0, y_center, 0.0))
    return plate


def _rivet_shape(index: int) -> cq.Workplane:
    """Through-rivet: shaft spanning the body depth with domed heads on both
    outer plate faces. Authored so XZ workplane extrudes in -Y."""
    x, z = RIVET_XZ[index]

    # Shaft: cylinder from Y = -BODY_D/2 to Y = +BODY_D/2.
    # XZ extrude goes -Y; raw Y in [-BODY_D, 0]; translate to center.
    shaft = (
        cq.Workplane("XZ")
        .center(x, z)
        .circle(RIVET_SHAFT_R)
        .extrude(BODY_D)
        .translate((0.0, BODY_D / 2.0, 0.0))
    )

    # Front head: proud of -Y face, embedded slightly for connectivity.
    # Raw Y in [-(H + embed), 0]; place so outer face is at -BODY_D/2 - H.
    front_head = (
        cq.Workplane("XZ")
        .center(x, z)
        .circle(RIVET_HEAD_R)
        .extrude(RIVET_HEAD_H + RIVET_EMBED)
        .translate((0.0, -BODY_D / 2.0 + RIVET_EMBED, 0.0))
    )

    # Back head: proud of +Y face, embedded slightly.
    # Raw Y in [-(H + embed), 0]; translate so inner edge is at BODY_D/2 - embed.
    back_head = (
        cq.Workplane("XZ")
        .center(x, z)
        .circle(RIVET_HEAD_R)
        .extrude(RIVET_HEAD_H + RIVET_EMBED)
        .translate((0.0, BODY_D / 2.0 + RIVET_HEAD_H, 0.0))
    )

    return front_head.union(shaft).union(back_head)


def _keyway_disc_shape() -> cq.Workplane:
    """Chrome keyway escutcheon: shallow disc proud of the front face.

    XZ extrude goes -Y, so raw spans y in [-DISC_T, 0]. After translate the
    disc stands proud of FRONT_Y while its inner lip embeds into the body.
    """
    disc = cq.Workplane("XZ").circle(0.0072).extrude(DISC_T)
    disc = disc.translate((0.0, FRONT_Y + DISC_EMBED, 0.0))
    return disc


def _keyway_slot_shape() -> cq.Workplane:
    """Dark key slot: a thin keyhole-style slot proud of the disc face."""
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
    keyway = keyway.translate((0.0, disc_front_y + SLOT_T * 0.5, 0.0))
    return keyway


def _shackle_shape() -> cq.Workplane:
    """Continuous steel U-shackle authored in the body frame.

    Long leg (left, -X), short leg (right, +X), half-torus arch joining them.
    Legs are round bar with machined-end fillets.
    """
    arch_center_z = ARCH_TOP_Z - LEG_OFFSET

    # Half-torus arch: build a full torus, rotate upright, intersect with
    # upper-half box to keep only the arch.
    torus_solid = cq.Solid.makeTorus(LEG_OFFSET, BAR_R)
    arch = cq.Workplane(obj=torus_solid).rotate((0, 0, 0), (1, 0, 0), 90.0)
    arch = arch.translate((0.0, 0.0, arch_center_z))
    half_box = (
        cq.Workplane("XY")
        .box(4.0 * LEG_OFFSET, 4.0 * BAR_R, 2.0 * (ARCH_TOP_Z + LEG_OFFSET))
        .translate((0.0, 0.0, arch_center_z + (ARCH_TOP_Z + LEG_OFFSET)))
    )
    arch = arch.intersect(half_box)

    # Long leg (left, -X): deep into the body.
    long_len = arch_center_z - LONG_LEG_BOTTOM_Z
    long_leg = (
        cq.Workplane("XY")
        .center(-LEG_OFFSET, 0.0)
        .circle(BAR_R)
        .extrude(long_len)
        .translate((0.0, 0.0, LONG_LEG_BOTTOM_Z))
    )

    # Short leg (right, +X): shallower seat.
    short_len = arch_center_z - SHORT_LEG_BOTTOM_Z
    short_leg = (
        cq.Workplane("XY")
        .center(LEG_OFFSET, 0.0)
        .circle(BAR_R)
        .extrude(short_len)
        .translate((0.0, 0.0, SHORT_LEG_BOTTOM_Z))
    )

    shackle = long_leg.union(short_leg).union(arch)
    shackle = shackle.edges("<Z").fillet(BAR_R * 0.45)
    return shackle


# --- Model ------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="laminated_padlock")

    # Two alternating plate tones for visible lamination.
    plate_dark = model.material("plate_steel_dark", rgba=(0.36, 0.38, 0.42, 1.0))
    plate_mid = model.material("plate_steel_mid", rgba=(0.46, 0.48, 0.52, 1.0))
    rivet_steel = model.material("rivet_steel", rgba=(0.58, 0.60, 0.63, 1.0))
    steel = model.material("hardened_steel", rgba=(0.74, 0.76, 0.80, 1.0))
    chrome = model.material("chrome", rgba=(0.86, 0.88, 0.90, 1.0))
    keyslot_dark = model.material("keyway_slot", rgba=(0.07, 0.07, 0.08, 1.0))

    plate_materials = [plate_dark, plate_mid]

    # Body (root) — laminated plate stack.
    body = model.part("body")
    for i in range(NUM_PLATES):
        body.visual(
            mesh_from_cadquery(_plate_shape(i), f"plate_{i}"),
            material=plate_materials[i % 2],
            name=f"plate_{i}",
        )
    for i in range(NUM_RIVETS):
        body.visual(
            mesh_from_cadquery(_rivet_shape(i), f"rivet_{i}"),
            material=rivet_steel,
            name=f"rivet_{i}",
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

    # --- Joint type and axis (unchanged from parent) -----------------------
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

    # --- Lamination: body is built from stacked plates ---------------------
    plate_names = [f"plate_{i}" for i in range(NUM_PLATES)]
    plate_aabbs = []
    for name in plate_names:
        aabb = ctx.part_element_world_aabb(body, elem=name)
        assert aabb is not None, f"plate visual {name} missing"
        plate_aabbs.append(aabb)

    ctx.check(
        "body has multiple laminated plates",
        len(plate_aabbs) >= 4,
        details=f"plate_count={len(plate_aabbs)}",
    )

    # Plates are stacked along Y (depth axis), not all at the same Y.
    y_centers = [(a[0][1] + a[1][1]) / 2.0 for a in plate_aabbs]
    y_sorted = sorted(y_centers)
    y_spread = y_sorted[-1] - y_sorted[0]
    ctx.check(
        "plates spread across body depth",
        y_spread > BODY_D * 0.4,
        details=f"y_spread={y_spread:.4f}, body_d={BODY_D:.4f}",
    )

    # Each plate is thin relative to the full body depth.
    plate_thicknesses = [a[1][1] - a[0][1] for a in plate_aabbs]
    max_plate_t = max(plate_thicknesses)
    ctx.check(
        "individual plates are thin",
        max_plate_t < BODY_D * 0.5,
        details=f"max_plate_t={max_plate_t:.4f}",
    )

    # --- Rivets present and proud of body faces ----------------------------
    rivet_names = [f"rivet_{i}" for i in range(NUM_RIVETS)]
    rivet_aabbs = []
    for name in rivet_names:
        aabb = ctx.part_element_world_aabb(body, elem=name)
        ctx.check(
            f"rivet {name} exists",
            aabb is not None,
            details=f"aabb={aabb}",
        )
        if aabb is not None:
            rivet_aabbs.append((name, aabb))

    # Frontmost and backmost plate faces define the body depth extent.
    plate_front_aabb = plate_aabbs[0]   # plate_0 at -Y side
    plate_back_aabb = plate_aabbs[-1]   # plate_4 at +Y side

    if rivet_aabbs:
        rivet_0_aabb = rivet_aabbs[0][1]
        ctx.check(
            "rivet head proud of front plate face",
            rivet_0_aabb[0][1] < plate_front_aabb[0][1] - 0.0001,
            details=(
                f"rivet_front_y={rivet_0_aabb[0][1]:.5f}, "
                f"plate_front_y={plate_front_aabb[0][1]:.5f}"
            ),
        )
        ctx.check(
            "rivet head proud of back plate face",
            rivet_0_aabb[1][1] > plate_back_aabb[1][1] + 0.0001,
            details=(
                f"rivet_back_y={rivet_0_aabb[1][1]:.5f}, "
                f"plate_back_y={plate_back_aabb[1][1]:.5f}"
            ),
        )

    # --- Overlap allowances ------------------------------------------------
    # Shackle legs seated inside the laminated plate stack.
    for i in range(NUM_PLATES):
        ctx.allow_overlap(
            body, shackle,
            elem_a=f"plate_{i}", elem_b="shackle_bar",
            reason="The shackle legs are intentionally seated inside the laminated plate stack when locked.",
        )
    # Rivet shafts pass through the body and may intersect the shackle legs.
    for i in range(NUM_RIVETS):
        ctx.allow_overlap(
            body, shackle,
            elem_a=f"rivet_{i}", elem_b="shackle_bar",
            reason="Rivet shafts pass through the full plate stack and may intersect the seated shackle legs.",
        )

    # --- Shackle geometry (unchanged from parent) -------------------------
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

    # --- Keyway (unchanged from parent) ------------------------------------
    disc_aabb = ctx.part_element_world_aabb(body, elem="keyway_disc")
    slot_aabb = ctx.part_element_world_aabb(body, elem="keyway_slot")
    assert disc_aabb is not None and slot_aabb is not None
    ctx.check(
        "keyway disc proud of front plate face",
        disc_aabb[0][1] < plate_front_aabb[0][1] - 0.0005,
        details=(
            f"disc_front_y={disc_aabb[0][1]:.4f}, "
            f"plate_front_y={plate_front_aabb[0][1]:.4f}"
        ),
    )
    ctx.check(
        "key slot proud of keyway disc",
        slot_aabb[0][1] < disc_aabb[0][1] - 0.0001,
        details=f"slot_front_y={slot_aabb[0][1]:.4f}, disc_front_y={disc_aabb[0][1]:.4f}",
    )

    # --- Locked pose (q=0): legs seated in body ---------------------------
    with ctx.pose({joint: 0.0}):
        locked_shackle_bottom = ctx.part_world_aabb(shackle)[0][2]
        ctx.check(
            "locked shackle legs seated in body",
            locked_shackle_bottom < BODY_H - 0.005,
            details=f"shackle_bottom={locked_shackle_bottom:.4f}, body_h={BODY_H:.4f}",
        )
        ctx.expect_overlap(
            shackle, body,
            axes="z",
            elem_a="shackle_bar",
            elem_b="plate_2",
            min_overlap=0.008,
            name="locked shackle retained in plate stack",
        )

    # --- Unlocked / fully lifted pose -------------------------------------
    rest_bottom = ctx.part_world_aabb(shackle)[0][2]
    with ctx.pose({joint: SHACKLE_TRAVEL}):
        lifted_bottom = ctx.part_world_aabb(shackle)[0][2]
        ctx.check(
            "lifting moves the shackle upward",
            lifted_bottom > rest_bottom + 0.5 * SHACKLE_TRAVEL,
            details=f"rest_bottom={rest_bottom:.4f}, lifted_bottom={lifted_bottom:.4f}",
        )
        ctx.expect_overlap(
            shackle, body,
            axes="z",
            elem_a="shackle_bar",
            elem_b="plate_2",
            min_overlap=0.002,
            name="long leg stays inserted at full travel",
        )

    return ctx.report()


object_model = build_object_model()
