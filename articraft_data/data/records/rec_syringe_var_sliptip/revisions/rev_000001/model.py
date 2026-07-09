from __future__ import annotations

# Realistic articulated 3 mL disposable medical syringe with an integrated
# slip-tip nozzle (molded plastic taper) and needle. The hero mechanism is
# the plunger sliding (PRISMATIC) inside the graduated barrel.
#
# Orientation: the geometry is AUTHORED along a local +Z axis (needle toward
# local -Z, plunger entering from local +Z), but the assembled syringe LIES
# HORIZONTALLY on the ground plane like a syringe resting on a table: the whole
# assembly is rotated -90 deg about X at the root (barrel axis -> world +Y,
# needle pointing toward -Y) and lifted so the lowest surface (the thumb-rest
# disc rim) sits exactly at z = 0. The plunger prismatic slides along the now
# horizontal barrel axis.

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Real-world dimensions (meters)
# ---------------------------------------------------------------------------
BARREL_OUTER_R = 0.0058          # barrel outer radius (~11.6 mm dia)
BARREL_WALL = 0.0009             # barrel wall thickness
BARREL_INNER_R = BARREL_OUTER_R - BARREL_WALL
BARREL_LEN = 0.060               # graduated barrel length
BARREL_Z0 = 0.0                  # barrel bottom (where it necks into the hub)
BARREL_Z1 = BARREL_Z0 + BARREL_LEN

LIQUID_LEVEL = 0.044             # teal liquid height inside barrel (~3 mL fill)

FLANGE_W = 0.026                 # full span of the two finger flanges
FLANGE_DEPTH = 0.0085            # flange depth (along Y)
FLANGE_THICK = 0.0016            # flange plate thickness

# Integrated slip-tip nozzle (molded plastic taper off the barrel neck)
NOZZLE_TOP_R = BARREL_OUTER_R    # smooth transition from barrel neck
NOZZLE_BOT_R = 0.0012            # narrow tip opening, just wider than needle
NOZZLE_LEN = 0.0120              # smooth tapered length
NOZZLE_Z1 = BARREL_Z0            # nozzle top flush with barrel bottom
NOZZLE_Z0 = NOZZLE_Z1 - NOZZLE_LEN

# Needle
NEEDLE_R = 0.00045               # 18G-ish needle radius
NEEDLE_LEN = 0.040
NEEDLE_Z1 = NOZZLE_Z0            # needle base at nozzle tip
NEEDLE_Z0 = NEEDLE_Z1 - NEEDLE_LEN

# Plunger
GASKET_R = BARREL_INNER_R + 0.00006     # rubber stopper, slight seal interference in the bore
GASKET_LEN = 0.0060
ROD_RIB_W = 0.0078               # cross-rib full width (the "+" plunger spine)
ROD_RIB_T = 0.0011               # rib thickness
THUMB_R = 0.0072                 # thumb-rest disc radius
THUMB_THICK = 0.0017

# Travel: plunger pushed fully in moves the gasket from the rest position down
# toward the nozzle neck. Rest pose seats the gasket just under the liquid surface.
GASKET_REST_Z = LIQUID_LEVEL - GASKET_LEN          # gasket top at liquid line
ROD_LEN = (BARREL_Z1 + 0.012) - (GASKET_REST_Z + GASKET_LEN)  # rod up past barrel rim
PLUNGER_TRAVEL = GASKET_REST_Z - (BARREL_Z0 + 0.004)          # down to near the neck

# Rest pose: a syringe cannot stand on its needle tip, so it lies flat on the
# ground. One root-level transform is applied to every barrel visual AND the
# plunger joint origin: roll -90 deg about X (authored local +Z -> world +Y,
# needle toward world -Y) plus a +Z lift equal to the largest radial extent
# (the thumb-rest disc) so the overall z-min is exactly 0.
REST_RPY = (-math.pi / 2.0, 0.0, 0.0)
REST_LIFT = THUMB_R
REST_XYZ = (0.0, 0.0, REST_LIFT)

TEAL = Material(name="teal_liquid", rgba=(0.18, 0.78, 0.74, 0.85))
GLASS = Material(name="barrel_clear", rgba=(0.86, 0.92, 0.93, 0.34))
STEEL = Material(name="stainless", rgba=(0.74, 0.76, 0.78, 1.0))
RUBBER = Material(name="black_rubber", rgba=(0.10, 0.10, 0.11, 1.0))
WHITE_PLASTIC = Material(name="white_plastic", rgba=(0.92, 0.93, 0.94, 1.0))
INK = Material(name="scale_ink", rgba=(0.10, 0.12, 0.14, 1.0))


def _barrel_shell() -> cq.Workplane:
    """Hollow open-top transparent barrel tube (revolved wall profile)."""
    # Profile in (r, z) revolved about the Z axis: a thin annular wall plus a
    # small rolled rim at the top.
    outer = BARREL_OUTER_R
    inner = BARREL_INNER_R
    rim = BARREL_OUTER_R + 0.0007
    profile = (
        cq.Workplane("XZ")
        .moveTo(inner, BARREL_Z0)
        .lineTo(outer, BARREL_Z0)
        .lineTo(outer, BARREL_Z1 - 0.0016)
        .lineTo(rim, BARREL_Z1 - 0.0012)
        .lineTo(rim, BARREL_Z1)
        .lineTo(inner, BARREL_Z1)
        .close()
    )
    return profile.revolve(360.0, (0.0, 0.0, 0.0), (0.0, 1.0, 0.0))


def _graduation_marks() -> cq.Workplane:
    """Thin embossed scale tick marks on the barrel surface (3 mL of ticks)."""
    marks = None
    r = BARREL_OUTER_R
    z_lo = 0.012
    z_hi = LIQUID_LEVEL + 0.001
    n = 16  # minor ticks
    for i in range(n + 1):
        z = z_lo + (z_hi - z_lo) * i / n
        # major tick every 5 (the 1/2/3 mL lines) is longer
        major = i % 5 == 0
        half_len = 0.0026 if major else 0.0014
        tick = (
            cq.Workplane("XY")
            .workplane(offset=z)
            .box(0.0008, half_len, 0.0006, centered=(True, True, True))
            .translate((r - 0.0001, 0.0, 0.0))
        )
        marks = tick if marks is None else marks.union(tick)
    return marks


def _finger_flanges() -> cq.Workplane:
    """Two flat thumb-and-forefinger wings at the top of the barrel."""
    z = BARREL_Z1 - 0.0020
    plate = (
        cq.Workplane("XY")
        .workplane(offset=z)
        .box(FLANGE_W, FLANGE_DEPTH, FLANGE_THICK, centered=(True, True, True))
        .edges("|Z")
        .fillet(0.0020)
    )
    # carve the central bore so the flange wraps the barrel rim cleanly
    bore = (
        cq.Workplane("XY")
        .workplane(offset=z)
        .circle(BARREL_OUTER_R + 0.0006)
        .extrude(FLANGE_THICK * 3, both=True)
    )
    return plate.cut(bore)


def _slip_tip_nozzle() -> cq.Workplane:
    """Integrated slip-tip nozzle: smooth tapered cone molded off the barrel neck."""
    # Smooth revolved taper from barrel neck radius down to the tip opening.
    # No steps, no slots — just a clean conical profile.
    profile = (
        cq.Workplane("XZ")
        .moveTo(0.0, NOZZLE_Z1)
        .lineTo(NOZZLE_TOP_R, NOZZLE_Z1)
        .lineTo(NOZZLE_BOT_R, NOZZLE_Z0)
        .lineTo(0.0, NOZZLE_Z0)
        .close()
        .revolve(360.0, (0.0, 0.0, 0.0), (0.0, 1.0, 0.0))
    )
    # Hollow bore through the nozzle so the needle lumen is visible at the tip.
    # Bore radius matches needle radius so the inserted needle shaft contacts
    # the bore wall (connectivity).
    bore = (
        cq.Workplane("XY")
        .workplane(offset=NOZZLE_Z1 + 0.0002)
        .circle(NEEDLE_R)
        .extrude(-(NOZZLE_LEN + 0.0004))
    )
    return profile.cut(bore)


def _needle() -> cq.Workplane:
    """Thin stainless needle with a beveled tip and a hollow lumen face."""
    # The needle extends slightly into the nozzle bore so it shares contact
    # with the nozzle interior (realistic press-fit seating).
    needle_insert = 0.0010
    shaft = (
        cq.Workplane("XY")
        .workplane(offset=NEEDLE_Z1 + needle_insert)
        .circle(NEEDLE_R)
        .extrude(-(NEEDLE_LEN + needle_insert - 0.0030))
    )
    # Beveled cutting tip: slice the very end at an angle.
    tip = (
        cq.Workplane("XY")
        .workplane(offset=NEEDLE_Z0 + 0.0030)
        .circle(NEEDLE_R)
        .extrude(-0.0030)
    )
    cutter = (
        cq.Workplane("XZ")
        .workplane(offset=-NEEDLE_R * 2)
        .moveTo(-NEEDLE_R * 2, NEEDLE_Z0)
        .lineTo(NEEDLE_R * 2, NEEDLE_Z0)
        .lineTo(NEEDLE_R * 2, NEEDLE_Z0 + 0.0042)
        .close()
        .extrude(NEEDLE_R * 4)
    )
    tip = tip.cut(cutter)
    return shaft.union(tip)


def _liquid() -> cq.Workplane:
    """Teal medication column inside the barrel, capped by the gasket above it.

    The column starts at the barrel floor and meets the inner wall so it reads
    as one connected body with the shell, and its surface sits just under the
    gasket so the two never interpenetrate.
    """
    liq_bot = BARREL_Z0 + 0.0002
    liq_top = GASKET_REST_Z - 0.0002  # surface just below the gasket base
    return (
        cq.Workplane("XY")
        .workplane(offset=liq_bot)
        .circle(BARREL_INNER_R)  # meets the inner wall for connectivity
        .extrude(liq_top - liq_bot)
    )


def _plunger_rod() -> cq.Workplane:
    """Cross-rib ('+') plunger spine running up out of the barrel."""
    z0 = GASKET_REST_Z + GASKET_LEN  # top of gasket
    rib_a = (
        cq.Workplane("XY")
        .workplane(offset=z0)
        .box(ROD_RIB_W, ROD_RIB_T, ROD_LEN, centered=(True, True, False))
    )
    rib_b = (
        cq.Workplane("XY")
        .workplane(offset=z0)
        .box(ROD_RIB_T, ROD_RIB_W, ROD_LEN, centered=(True, True, False))
    )
    return rib_a.union(rib_b)


def _gasket() -> cq.Workplane:
    """Black rubber stopper at the bottom of the plunger."""
    return (
        cq.Workplane("XY")
        .workplane(offset=GASKET_REST_Z)
        .circle(GASKET_R)
        .extrude(GASKET_LEN)
        .edges("%CIRCLE")
        .fillet(0.0008)
    )


def _thumb_rest() -> cq.Workplane:
    """Round thumb-press disc at the very top of the plunger rod."""
    z = GASKET_REST_Z + GASKET_LEN + ROD_LEN
    return (
        cq.Workplane("XY")
        .workplane(offset=z - THUMB_THICK)
        .circle(THUMB_R)
        .extrude(THUMB_THICK)
        .edges(">Z or <Z")
        .fillet(0.0006)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="medical_syringe")
    for mat in (TEAL, GLASS, STEEL, RUBBER, WHITE_PLASTIC, INK):
        model.material(mat.name, rgba=mat.rgba)

    # ---- Root: barrel assembly (barrel + flanges + liquid + hub + needle) ----
    # Every barrel visual carries the same root rest transform (lay flat + lift).
    rest = Origin(xyz=REST_XYZ, rpy=REST_RPY)
    barrel = model.part("barrel_assembly")
    barrel.visual(mesh_from_cadquery(_barrel_shell(), "barrel_shell"), origin=rest, material=GLASS, name="barrel_shell")
    barrel.visual(mesh_from_cadquery(_liquid(), "liquid"), origin=rest, material=TEAL, name="liquid")
    barrel.visual(mesh_from_cadquery(_graduation_marks(), "scale_marks"), origin=rest, material=INK, name="scale_marks")
    barrel.visual(mesh_from_cadquery(_finger_flanges(), "finger_flanges"), origin=rest, material=WHITE_PLASTIC, name="finger_flanges")
    barrel.visual(mesh_from_cadquery(_slip_tip_nozzle(), "slip_tip_nozzle"), origin=rest, material=WHITE_PLASTIC, name="slip_tip_nozzle")
    barrel.visual(mesh_from_cadquery(_needle(), "needle"), origin=rest, material=STEEL, name="needle")

    # ---- Child: plunger (rod + gasket + thumb rest) ----
    # Plunger geometry stays authored in the local vertical frame; the joint
    # origin below carries the same rest transform, so the plunger lies down
    # with the barrel and stays coaxial with the bore.
    plunger = model.part("plunger")
    plunger.visual(mesh_from_cadquery(_gasket(), "gasket"), material=RUBBER, name="gasket")
    plunger.visual(mesh_from_cadquery(_plunger_rod(), "plunger_rod"), material=WHITE_PLASTIC, name="plunger_rod")
    plunger.visual(mesh_from_cadquery(_thumb_rest(), "thumb_rest"), material=WHITE_PLASTIC, name="thumb_rest")

    # ---- Prismatic joint: plunger slides DOWN the barrel bore to dispense ----
    # Joint frame at the barrel bottom interior, rotated/lifted by the same
    # rest transform as the barrel visuals. The axis is -Z in the joint's local
    # frame (the barrel bore axis), which is world -Y in the lying rest pose,
    # so positive q still pushes the plunger toward the hub/needle end.
    model.articulation(
        "barrel_to_plunger",
        ArticulationType.PRISMATIC,
        parent=barrel,
        child=plunger,
        origin=Origin(xyz=REST_XYZ, rpy=REST_RPY),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=0.1,
            lower=0.0,
            upper=PLUNGER_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    barrel = object_model.get_part("barrel_assembly")
    plunger = object_model.get_part("plunger")
    joint = object_model.get_articulation("barrel_to_plunger")

    # --- Joint is the prismatic dispensing mechanism along the barrel axis ---
    ctx.check(
        "plunger joint is prismatic",
        joint.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={joint.articulation_type}",
    )
    ax = joint.axis
    ctx.check(
        "plunger slides along the barrel bore axis (joint-local -Z)",
        abs(ax[0]) < 1e-6 and abs(ax[1]) < 1e-6 and abs(ax[2]) > 0.99,
        details=f"axis={ax}",
    )

    # The syringe lies flat: the barrel axis runs along world Y (needle toward
    # -Y, plunger toward +Y), and the lowest surface rests on the ground.
    # --- Hero parts present and in the right places ---
    barrel_box = ctx.part_element_world_aabb(barrel, elem="barrel_shell")
    # Needle extends beyond the slip-tip nozzle toward the -Y (needle) end.
    needle_box = ctx.part_element_world_aabb(barrel, elem="needle")
    nozzle_box = ctx.part_element_world_aabb(barrel, elem="slip_tip_nozzle")
    ctx.check(
        "needle extends beyond the nozzle along the barrel axis",
        needle_box is not None and nozzle_box is not None and needle_box[0][1] < nozzle_box[0][1] - 0.02,
        details=f"needle_min_y={None if needle_box is None else needle_box[0][1]}",
    )
    # Slip-tip nozzle is a smooth taper (no stepped metal collar).
    ctx.check(
        "slip-tip nozzle is present and tapered",
        nozzle_box is not None
        and (nozzle_box[1][2] - nozzle_box[0][2]) > 0.0005
        and (nozzle_box[1][0] - nozzle_box[0][0]) > 0.0005,
        details=f"nozzle_box={nozzle_box}",
    )
    # Nozzle sits below the barrel along Y (toward the needle end).
    ctx.check(
        "nozzle is below the barrel along the barrel axis",
        nozzle_box is not None and barrel_box is not None and nozzle_box[0][1] < barrel_box[0][1] - 0.002,
        details=f"nozzle_min_y={None if nozzle_box is None else nozzle_box[0][1]}",
    )
    # Liquid sits inside the barrel, between the neck and the rim along Y.
    liquid_box = ctx.part_element_world_aabb(barrel, elem="liquid")
    ctx.check(
        "teal liquid is inside the barrel between neck and rim",
        liquid_box is not None
        and barrel_box is not None
        and liquid_box[1][1] < barrel_box[1][1]
        and liquid_box[0][1] > barrel_box[0][1],
        details=f"liquid_y={None if liquid_box is None else (liquid_box[0][1], liquid_box[1][1])}",
    )
    # Finger flanges span wider than the barrel near the rim end.
    flange_box = ctx.part_element_world_aabb(barrel, elem="finger_flanges")
    ctx.check(
        "finger flanges are wider than the barrel",
        flange_box is not None and (flange_box[1][0] - flange_box[0][0]) > BARREL_OUTER_R * 3,
        details=f"flange_x_span={None if flange_box is None else flange_box[1][0]-flange_box[0][0]}",
    )
    # Thumb rest sits past the barrel rim at the +Y (plunger) end.
    thumb_box = ctx.part_element_world_aabb(plunger, elem="thumb_rest")
    ctx.check(
        "thumb rest is beyond the barrel rim",
        thumb_box is not None and barrel_box is not None and thumb_box[0][1] > barrel_box[1][1] - 0.001,
        details=f"thumb_min_y={None if thumb_box is None else thumb_box[0][1]}",
    )

    # --- Gasket is retained inside the barrel bore (snug fit) at rest ---
    # Lying along Y, the bore cross-section is the world XZ plane.
    ctx.expect_within(
        plunger,
        barrel,
        axes="xz",
        inner_elem="gasket",
        outer_elem="barrel_shell",
        margin=0.0005,
        name="gasket stays within the barrel bore",
    )
    # Gasket sits inside the barrel along the bore (Y) at rest (retained insertion).
    ctx.expect_overlap(
        plunger,
        barrel,
        axes="y",
        elem_a="gasket",
        elem_b="barrel_shell",
        min_overlap=0.003,
        name="gasket is inserted in the barrel at rest",
    )

    # --- Decisive pose check: pushing the plunger moves the gasket toward the needle (-Y) ---
    rest_pos = ctx.part_world_position(plunger)
    with ctx.pose({joint: PLUNGER_TRAVEL}):
        pushed_pos = ctx.part_world_position(plunger)
        # Gasket must still be retained inside the barrel at full stroke.
        ctx.expect_within(
            plunger,
            barrel,
            axes="xz",
            inner_elem="gasket",
            outer_elem="barrel_shell",
            margin=0.0005,
            name="gasket centered in bore at full stroke",
        )
        ctx.expect_overlap(
            plunger,
            barrel,
            axes="y",
            elem_a="gasket",
            elem_b="barrel_shell",
            min_overlap=0.003,
            name="gasket retained in barrel at full stroke",
        )
    ctx.check(
        "pushing plunger dispenses (moves gasket toward the needle end, -Y)",
        rest_pos is not None and pushed_pos is not None and pushed_pos[1] < rest_pos[1] - 0.01,
        details=f"rest_y={None if rest_pos is None else rest_pos[1]}, pushed_y={None if pushed_pos is None else pushed_pos[1]}",
    )

    # The gasket nesting in the bore is an intentional snug seal fit.
    ctx.allow_overlap(
        plunger,
        barrel,
        elem_a="gasket",
        elem_b="barrel_shell",
        reason="The rubber gasket is intentionally seated snugly inside the barrel bore to seal the plunger.",
    )

    return ctx.report()


object_model = build_object_model()
