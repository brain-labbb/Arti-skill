from __future__ import annotations

# Articraft model: a discus-style round lock with a partially recessed shackle.
#
# Articraft brief
# - Object: discus (round/circular) lock with a hardened-steel U-shackle
#   partially recessed into the cylindrical case. Body ~64 mm diameter, ~26 mm
#   thick; shackle bar ~9.6 mm diameter, arch rising ~35 mm above the body.
# - Root/support: the round body is the root. Its curved top surface has two
#   bored holes that capture the shackle legs.
# - Parts: body (round cylinder with chamfered edges, chrome keyway disc + dark
#   key slot on the front circular face) and shackle (continuous steel U).
# - Articulation: body_to_shackle, PRISMATIC along +Z. At q=0 (locked) both
#   legs are seated in the body bores. Positive q lifts the shackle up until
#   the short leg clears the body top.
# - Support/fit: shackle legs nest inside the body bores.
# - Intentional overlap: seated legs inside simplified solid body -> scoped
#   allow_overlap on those elements.
# - Tests: round body (X & Z spans ≈ diameter, Y thinner than diameter),
#   shackle U above body, keyway on front face, prismatic +Z joint,
#   locked/unlocked poses, retained insertion.

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

# --- Discus body dimensions (meters) ----------------------------------------
DISC_RADIUS = 0.032            # body radius (64 mm diameter)
DISC_THICKNESS = 0.026         # body depth along Y (26 mm)
BODY_TOP_Z = 2.0 * DISC_RADIUS  # 0.064 — top of the round body
BODY_CENTER_Z = DISC_RADIUS     # 0.032 — vertical center of the disc

# --- Shackle dimensions -----------------------------------------------------
LEG_OFFSET = 0.012             # half-distance between legs (X)
BAR_R = 0.0048                 # shackle bar radius (~9.6 mm bar)
BORE_R = BAR_R - 0.0004        # body bore radius (capture fit)

ARCH_TOP_Z = BODY_TOP_Z + 0.035     # arch peak at 0.099
LEG_SEAT_DEPTH = 0.018               # short-leg seat depth from body top
SHORT_LEG_BOTTOM_Z = BODY_TOP_Z - LEG_SEAT_DEPTH         # 0.046
LONG_LEG_BOTTOM_Z = BODY_TOP_Z - DISC_RADIUS * 1.10      # ~0.029

BORE_DEPTH = BODY_TOP_Z - LONG_LEG_BOTTOM_Z + 0.005      # bore cutter depth

# Prismatic travel: enough for the short leg to fully clear the body top.
SHACKLE_TRAVEL = LEG_SEAT_DEPTH + 0.008                   # 0.026

# --- Keyway placement -------------------------------------------------------
FRONT_Y = -DISC_THICKNESS / 2.0
DISC_T = 0.0016              # keyway escutcheon thickness
SLOT_T = 0.0006              # key-slot relief thickness
DISC_EMBED = 0.0005          # disc inner face embeds into body for contact


# --- Geometry builders ------------------------------------------------------

def _body_shape() -> cq.Workplane:
    """Discus lock body: round cylinder with chamfered rim and shackle bores."""
    # Build cylinder along Z, then rotate to Y axis so the flat faces are at ±Y.
    cyl = (
        cq.Workplane("XY")
        .circle(DISC_RADIUS)
        .extrude(DISC_THICKNESS)
        .rotate((0, 0, 0), (1, 0, 0), 90)
        .translate((0, DISC_THICKNESS / 2.0, DISC_RADIUS))
    )
    # Chamfer the two circular rim edges for a machined finish.
    cyl = cyl.edges().chamfer(0.0018)

    # Bore two holes through the curved top for the shackle legs.
    for sx in (-LEG_OFFSET, LEG_OFFSET):
        bore = (
            cq.Workplane("XY")
            .center(sx, 0.0)
            .circle(BORE_R)
            .extrude(BORE_DEPTH + 0.010)
            .translate((0.0, 0.0, BODY_TOP_Z - BORE_DEPTH))
        )
        cyl = cyl.cut(bore)
    return cyl


def _keyway_disc_shape() -> cq.Workplane:
    """Chrome keyway escutcheon on the front circular face."""
    disc = cq.Workplane("XZ").circle(0.0072).extrude(DISC_T)
    # XZ extrude goes along −Y; translate to front face, centered at body Z.
    disc = disc.translate((0.0, FRONT_Y + DISC_EMBED, BODY_CENTER_Z))
    return disc


def _keyway_slot_shape() -> cq.Workplane:
    """Dark keyhole slot proud of the escutcheon face."""
    disc_front_y = FRONT_Y - DISC_T + DISC_EMBED
    # Pin-tumbler keyway: vertical slot with round top.
    slot = (
        cq.Workplane("XZ")
        .rect(0.0012, 0.008)
        .extrude(SLOT_T)
    )
    keyhole = (
        cq.Workplane("XZ")
        .moveTo(0.0, 0.003)
        .circle(0.002)
        .extrude(SLOT_T)
    )
    keyway = slot.union(keyhole)
    keyway = keyway.translate(
        (0.0, disc_front_y + SLOT_T * 0.5, BODY_CENTER_Z),
    )
    return keyway


def _shackle_shape() -> cq.Workplane:
    """Continuous steel U-shackle authored in the body frame."""
    arch_center_z = ARCH_TOP_Z - LEG_OFFSET

    # Half-torus arch over the top.
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
    # Round the free bottom tips so they read as machined bar ends.
    shackle = shackle.edges("<Z").fillet(BAR_R * 0.45)
    return shackle


# --- Model ------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="discus_lock")

    brushed_steel = model.material(
        "brushed_steel", rgba=(0.72, 0.73, 0.76, 1.0),
    )
    hardened_steel = model.material(
        "hardened_steel", rgba=(0.74, 0.76, 0.80, 1.0),
    )
    chrome = model.material("chrome", rgba=(0.86, 0.88, 0.90, 1.0))
    keyslot_dark = model.material(
        "keyway_slot", rgba=(0.07, 0.07, 0.08, 1.0),
    )

    # Body (root) — round discus shell with keyway on front face.
    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shape(), "body_shell"),
        material=brushed_steel,
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

    # Shackle — hardened-steel U, partially recessed into the round case.
    shackle = model.part("shackle")
    shackle.visual(
        mesh_from_cadquery(_shackle_shape(), "shackle_bar"),
        material=hardened_steel,
        name="shackle_bar",
    )

    # Prismatic joint: the shackle lifts straight up out of the body.
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

    # --- Joint type and axis ---
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

    # --- Seated legs overlap allowance ---
    ctx.allow_overlap(
        body,
        shackle,
        elem_a="body_shell",
        elem_b="shackle_bar",
        reason="The shackle legs are intentionally seated inside the body bores when locked.",
    )

    # --- Round body verification ---
    body_aabb = ctx.part_world_aabb(body)
    shackle_aabb = ctx.part_world_aabb(shackle)
    assert body_aabb is not None and shackle_aabb is not None

    body_x_span = body_aabb[1][0] - body_aabb[0][0]
    body_z_span = body_aabb[1][2] - body_aabb[0][2]
    body_y_span = body_aabb[1][1] - body_aabb[0][1]

    ctx.check(
        "body is round (X span matches diameter)",
        abs(body_x_span - 2.0 * DISC_RADIUS) < 0.005,
        details=f"x_span={body_x_span:.4f}, expected={2*DISC_RADIUS:.4f}",
    )
    ctx.check(
        "body is round (Z span matches diameter)",
        abs(body_z_span - 2.0 * DISC_RADIUS) < 0.005,
        details=f"z_span={body_z_span:.4f}, expected={2*DISC_RADIUS:.4f}",
    )
    ctx.check(
        "body is disc-shaped (thinner than diameter)",
        body_y_span < body_x_span * 0.9,
        details=f"y_span={body_y_span:.4f}, x_span={body_x_span:.4f}",
    )

    # --- Shackle arches above the round body ---
    body_top_z = body_aabb[1][2]
    shackle_top_z = shackle_aabb[1][2]
    ctx.check(
        "shackle arches above the body",
        shackle_top_z > body_top_z + 0.025,
        details=f"shackle_top={shackle_top_z:.4f}, body_top={body_top_z:.4f}",
    )

    # --- Shackle U spans both legs ---
    shackle_x_span = shackle_aabb[1][0] - shackle_aabb[0][0]
    ctx.check(
        "shackle spans both legs in X",
        shackle_x_span > 2.0 * LEG_OFFSET - 0.001,
        details=f"x_span={shackle_x_span:.4f}",
    )

    # --- Keyway on front circular face ---
    body_shell_aabb = ctx.part_element_world_aabb(body, elem="body_shell")
    disc_aabb = ctx.part_element_world_aabb(body, elem="keyway_disc")
    slot_aabb = ctx.part_element_world_aabb(body, elem="keyway_slot")
    assert (
        body_shell_aabb is not None
        and disc_aabb is not None
        and slot_aabb is not None
    )
    ctx.check(
        "keyway disc proud of front face",
        disc_aabb[0][1] < body_shell_aabb[0][1] - 0.0005,
        details=(
            f"disc_front_y={disc_aabb[0][1]:.4f}, "
            f"body_front_y={body_shell_aabb[0][1]:.4f}"
        ),
    )
    ctx.check(
        "key slot proud of keyway disc",
        slot_aabb[0][1] < disc_aabb[0][1] - 0.0001,
        details=(
            f"slot_front_y={slot_aabb[0][1]:.4f}, "
            f"disc_front_y={disc_aabb[0][1]:.4f}"
        ),
    )

    # --- Locked pose (q=0): legs seated in body ---
    with ctx.pose({joint: 0.0}):
        locked_bottom = ctx.part_world_aabb(shackle)[0][2]
        ctx.check(
            "locked shackle legs seated in body",
            locked_bottom < BODY_TOP_Z - 0.005,
            details=(
                f"shackle_bottom={locked_bottom:.4f}, "
                f"body_top={BODY_TOP_Z:.4f}"
            ),
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

    # --- Unlocked pose: shackle lifts, long leg stays inserted ---
    rest_bottom = ctx.part_world_aabb(shackle)[0][2]
    with ctx.pose({joint: SHACKLE_TRAVEL}):
        lifted_bottom = ctx.part_world_aabb(shackle)[0][2]
        ctx.check(
            "lifting moves the shackle upward",
            lifted_bottom > rest_bottom + 0.5 * SHACKLE_TRAVEL,
            details=f"rest={rest_bottom:.4f}, lifted={lifted_bottom:.4f}",
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

    return ctx.report()


object_model = build_object_model()
