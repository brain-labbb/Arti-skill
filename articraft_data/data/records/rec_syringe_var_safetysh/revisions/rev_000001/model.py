from __future__ import annotations

# Realistic articulated 3 mL disposable medical syringe with a stainless Luer
# hub and needle. The hero mechanism is the plunger sliding (PRISMATIC) inside
# the transparent graduated barrel.
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

# Stainless Luer hub (metal connector below the barrel)
HUB_TOP_R = 0.0050               # where hub meets barrel neck
HUB_BOT_R = 0.0034               # narrow end toward the needle
HUB_LEN = 0.0150
HUB_Z1 = BARREL_Z0               # hub top flush with barrel bottom
HUB_Z0 = HUB_Z1 - HUB_LEN

# Needle
NEEDLE_R = 0.00045               # 18G-ish needle radius
NEEDLE_LEN = 0.040
NEEDLE_Z1 = HUB_Z0               # needle base at hub bottom
NEEDLE_Z0 = NEEDLE_Z1 - NEEDLE_LEN

# Safety shield (translucent plastic guard sleeve around the needle)
SHIELD_OUTER_R = 0.0040          # outer radius of the guard tube
SHIELD_WALL = 0.0008             # wall thickness
SHIELD_INNER_R = SHIELD_OUTER_R - SHIELD_WALL
SHIELD_LEN = 0.033               # guard length (needle protrudes ~7 mm)
SHIELD_Z1 = HUB_Z0               # shield top meets hub bottom
SHIELD_Z0 = SHIELD_Z1 - SHIELD_LEN

# Plunger
GASKET_R = BARREL_INNER_R + 0.00006     # rubber stopper, slight seal interference in the bore
GASKET_LEN = 0.0060
ROD_RIB_W = 0.0078               # cross-rib full width (the "+" plunger spine)
ROD_RIB_T = 0.0011               # rib thickness
THUMB_R = 0.0072                 # thumb-rest disc radius
THUMB_THICK = 0.0017

# Travel: plunger pushed fully in moves the gasket from the rest position down
# toward the hub neck. Rest pose seats the gasket just under the liquid surface.
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
SHIELD_MAT = Material(name="shield_plastic", rgba=(0.82, 0.90, 0.95, 0.38))


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


def _hub() -> cq.Workplane:
    """Stainless Luer hub: a stepped tapered collar with an open throat."""
    # Outer stepped body (revolved).
    body = (
        cq.Workplane("XZ")
        .moveTo(0.0, HUB_Z1)
        .lineTo(HUB_TOP_R, HUB_Z1)
        .lineTo(HUB_TOP_R, HUB_Z1 - 0.0030)
        .lineTo(HUB_TOP_R - 0.0008, HUB_Z1 - 0.0042)
        .lineTo(HUB_BOT_R, HUB_Z0 + 0.0030)
        .lineTo(HUB_BOT_R, HUB_Z0)
        .lineTo(0.0, HUB_Z0)
        .close()
        .revolve(360.0, (0.0, 0.0, 0.0), (0.0, 1.0, 0.0))
    )
    # Hollow out the upper throat (the Luer slip socket) so it reads hollow.
    throat = (
        cq.Workplane("XY")
        .workplane(offset=HUB_Z1 + 0.0005)
        .circle(HUB_TOP_R - 0.0014)
        .extrude(-0.0080)
    )
    body = body.cut(throat)
    # Two side reliefs (the visible internal slots seen on the metal connector).
    for ang in (0.0, math.pi / 2):
        slot = (
            cq.Workplane("XY")
            .workplane(offset=HUB_Z1 - 0.0050)
            .box(0.0016, HUB_TOP_R * 2.4, 0.0070, centered=(True, True, True))
            .rotate((0, 0, 0), (0, 0, 1), math.degrees(ang))
        )
        body = body.cut(slot)
    return body


def _needle() -> cq.Workplane:
    """Thin stainless needle with a beveled tip and a hollow lumen face."""
    shaft = (
        cq.Workplane("XY")
        .workplane(offset=NEEDLE_Z1)
        .circle(NEEDLE_R)
        .extrude(-(NEEDLE_LEN - 0.0030))
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


def _safety_shield() -> cq.Workplane:
    """Translucent plastic guard sleeve surrounding the needle.

    A hollow open-ended tube with a small mounting collar at the top that
    seats against the hub bottom. The needle protrudes from the open distal
    end.
    """
    # Main tube body (hollow annulus extruded downward from SHIELD_Z1).
    outer = (
        cq.Workplane("XY")
        .workplane(offset=SHIELD_Z1)
        .circle(SHIELD_OUTER_R)
        .extrude(-SHIELD_LEN)
    )
    bore = (
        cq.Workplane("XY")
        .workplane(offset=SHIELD_Z1 + 0.001)
        .circle(SHIELD_INNER_R)
        .extrude(-(SHIELD_LEN + 0.002))
    )
    tube = outer.cut(bore)
    # Small mounting collar at the top (wider ring that seats on the hub).
    collar = (
        cq.Workplane("XY")
        .workplane(offset=SHIELD_Z1)
        .circle(HUB_BOT_R + 0.0004)
        .extrude(-0.0030)
    )
    collar_bore = (
        cq.Workplane("XY")
        .workplane(offset=SHIELD_Z1 + 0.001)
        .circle(SHIELD_INNER_R)
        .extrude(-0.005)
    )
    collar = collar.cut(collar_bore)
    return tube.union(collar)


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
    for mat in (TEAL, GLASS, STEEL, RUBBER, WHITE_PLASTIC, INK, SHIELD_MAT):
        model.material(mat.name, rgba=mat.rgba)

    # ---- Root: barrel assembly (barrel + flanges + liquid + hub + needle) ----
    # Every barrel visual carries the same root rest transform (lay flat + lift).
    rest = Origin(xyz=REST_XYZ, rpy=REST_RPY)
    barrel = model.part("barrel_assembly")
    barrel.visual(mesh_from_cadquery(_barrel_shell(), "barrel_shell"), origin=rest, material=GLASS, name="barrel_shell")
    barrel.visual(mesh_from_cadquery(_liquid(), "liquid"), origin=rest, material=TEAL, name="liquid")
    barrel.visual(mesh_from_cadquery(_graduation_marks(), "scale_marks"), origin=rest, material=INK, name="scale_marks")
    barrel.visual(mesh_from_cadquery(_finger_flanges(), "finger_flanges"), origin=rest, material=WHITE_PLASTIC, name="finger_flanges")
    barrel.visual(mesh_from_cadquery(_hub(), "luer_hub"), origin=rest, material=STEEL, name="luer_hub")
    barrel.visual(mesh_from_cadquery(_needle(), "needle"), origin=rest, material=STEEL, name="needle")
    barrel.visual(mesh_from_cadquery(_safety_shield(), "safety_shield"), origin=rest, material=SHIELD_MAT, name="safety_shield")

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
    # Needle extends beyond the hub toward the -Y (needle) end.
    needle_box = ctx.part_element_world_aabb(barrel, elem="needle")
    hub_box = ctx.part_element_world_aabb(barrel, elem="luer_hub")
    ctx.check(
        "needle extends beyond the hub along the barrel axis",
        needle_box is not None and hub_box is not None and needle_box[0][1] < hub_box[0][1] - 0.02,
        details=f"needle_min_y={None if needle_box is None else needle_box[0][1]}",
    )
    # Liquid sits inside the barrel, between the neck and the rim along Y.
    liquid_box = ctx.part_element_world_aabb(barrel, elem="liquid")
    barrel_box = ctx.part_element_world_aabb(barrel, elem="barrel_shell")
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

    # --- Safety shield surrounds the needle and the needle protrudes ---
    shield_box = ctx.part_element_world_aabb(barrel, elem="safety_shield")
    ctx.check(
        "safety shield is present on the barrel assembly",
        shield_box is not None,
        details="safety_shield visual missing from barrel_assembly",
    )
    # Shield surrounds the needle radially (in the lying pose, XZ is the
    # cross-section plane of the barrel axis along Y).
    if shield_box is not None and needle_box is not None:
        ctx.expect_within(
            barrel,
            barrel,
            axes="xz",
            inner_elem="needle",
            outer_elem="safety_shield",
            margin=0.0005,
            name="needle is enclosed within the shield cross-section",
        )
    # Needle protrudes beyond the shield's open distal end (toward -Y).
    if shield_box is not None and needle_box is not None:
        ctx.check(
            "needle protrudes beyond the shield distal end",
            needle_box[0][1] < shield_box[0][1] - 0.003,
            details=f"needle_min_y={needle_box[0][1]}, shield_min_y={shield_box[0][1]}",
        )
    # Shield sits between the hub and the needle tip along the barrel axis.
    if shield_box is not None and hub_box is not None:
        ctx.check(
            "shield is mounted below the hub toward the needle end",
            shield_box[1][1] <= hub_box[0][1] + 0.002,
            details=f"shield_max_y={shield_box[1][1]}, hub_min_y={hub_box[0][1]}",
        )

    return ctx.report()


object_model = build_object_model()
