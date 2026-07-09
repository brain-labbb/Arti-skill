from __future__ import annotations

# Realistic articulated large-volume (≈60 mL) wide-barrel medical syringe with
# a stainless Luer hub and needle.  The barrel has a stepped wider mid section
# (broader outer diameter in the central volume chamber) and a broader bore
# compared to a standard 3 mL syringe, while keeping the same graduated
# transparent tube family.
#
# Hero mechanism: plunger sliding (PRISMATIC) inside the barrel bore.
#
# Orientation: geometry is AUTHORED along local +Z (needle toward local -Z,
# plunger entering from local +Z), but the assembled syringe LIES HORIZONTALLY
# on the ground plane: the whole assembly is rotated -90 deg about X at the
# root (barrel axis -> world +Y, needle pointing toward -Y) and lifted so the
# lowest surface (thumb-rest disc rim) sits exactly at z = 0.

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
# Real-world dimensions (meters) — large-volume wide-barrel syringe (~60 mL)
# ---------------------------------------------------------------------------

# Barrel bore — broader than a standard 3 mL syringe
BARREL_BORE_R = 0.0100           # bore radius (20 mm diameter)
BARREL_WALL = 0.0012             # wall thickness
BARREL_LEN = 0.095               # graduated barrel length (longer for volume)

# Stepped outer profile radii
BARREL_LOWER_OUTER_R = BARREL_BORE_R + BARREL_WALL   # 0.0112 — lower neck
BARREL_MID_OUTER_R = 0.0165                          # 0.0165 — wide mid chamber (33 mm dia)
BARREL_UPPER_OUTER_R = BARREL_BORE_R + BARREL_WALL   # 0.0112 — upper neck

# Step transition heights along the barrel
BARREL_Z0 = 0.0
BARREL_STEP1_Z = 0.020           # lower-to-mid step
BARREL_STEP2_Z = 0.072           # mid-to-upper step
BARREL_Z1 = BARREL_Z0 + BARREL_LEN  # 0.095

LIQUID_LEVEL = 0.068             # teal liquid height inside barrel (~50 mL fill)

FLANGE_W = 0.038                 # wider finger flanges for the large barrel
FLANGE_DEPTH = 0.026             # flange depth must exceed the barrel bore diameter
FLANGE_THICK = 0.0020            # flange plate thickness

# Stainless Luer hub (metal connector below the barrel) — scaled to wider barrel
HUB_TOP_R = BARREL_BORE_R        # slip-fits into the barrel bore at the neck (surface contact)
HUB_BOT_R = 0.0034               # narrow end toward the needle
HUB_LEN = 0.0180
HUB_Z1 = BARREL_Z0               # hub top flush with barrel bottom
HUB_Z0 = HUB_Z1 - HUB_LEN

# Needle
NEEDLE_R = 0.00045               # 18G-ish needle radius
NEEDLE_LEN = 0.040
NEEDLE_Z1 = HUB_Z0               # needle base at hub bottom
NEEDLE_Z0 = NEEDLE_Z1 - NEEDLE_LEN

# Plunger — gasket matches the broader bore
GASKET_R = BARREL_BORE_R + 0.00008    # rubber stopper, slight seal interference
GASKET_LEN = 0.0075
ROD_RIB_W = 0.0100               # cross-rib full width (wider for the larger plunger)
ROD_RIB_T = 0.0014               # rib thickness
THUMB_R = 0.0095                 # thumb-rest disc radius (larger)
THUMB_THICK = 0.0020

# Travel: plunger pushed fully in moves the gasket from the rest position down
# toward the hub neck. Rest pose seats the gasket just under the liquid surface.
GASKET_REST_Z = LIQUID_LEVEL - GASKET_LEN
ROD_LEN = (BARREL_Z1 + 0.014) - (GASKET_REST_Z + GASKET_LEN)
PLUNGER_TRAVEL = GASKET_REST_Z - (BARREL_Z0 + 0.005)

# Rest pose: syringe lies flat on the ground. Root transform rolls -90 deg
# about X (local +Z -> world +Y, needle toward world -Y) and lifts so the
# lowest surface (thumb-rest disc rim) sits at z = 0.
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
    """Hollow open-top transparent barrel tube with a stepped wider mid section.

    The bore is uniform at BARREL_BORE_R for the main travel length so the
    plunger gasket seals throughout. The outer profile steps out to a wider
    mid section between BARREL_STEP1_Z and BARREL_STEP2_Z. A short neck at
    the bottom tapers from the barrel bore down to the hub fitting diameter
    for connectivity with the Luer hub.
    """
    bore = BARREL_BORE_R
    lower_r = BARREL_LOWER_OUTER_R
    mid_r = BARREL_MID_OUTER_R
    upper_r = BARREL_UPPER_OUTER_R
    rim_r = upper_r + 0.0009       # rolled rim at the top
    step_fillet = 0.0008           # small shoulder chamfer at each step

    # Neck transition below the barrel body
    neck_z0 = -0.004               # neck bottom (overlaps hub top for contact)
    neck_inner_bot = HUB_TOP_R     # matches hub top for a slip-fit look
    neck_outer_bot = HUB_TOP_R + 0.0008

    # Revolved wall profile in (r, z):
    profile = (
        cq.Workplane("XZ")
        # Start at the neck bottom inner edge
        .moveTo(neck_inner_bot, neck_z0)
        .lineTo(neck_outer_bot, neck_z0)
        # Neck taper outer wall: from hub diameter up to barrel lower diameter
        .lineTo(lower_r, BARREL_Z0)
        # Lower section outer wall up to first step
        .lineTo(lower_r, BARREL_STEP1_Z - step_fillet)
        .lineTo(lower_r + step_fillet, BARREL_STEP1_Z)
        # Step out to mid section
        .lineTo(mid_r - step_fillet, BARREL_STEP1_Z)
        .lineTo(mid_r, BARREL_STEP1_Z + step_fillet)
        # Mid section outer wall
        .lineTo(mid_r, BARREL_STEP2_Z - step_fillet)
        .lineTo(mid_r - step_fillet, BARREL_STEP2_Z)
        # Step in to upper section
        .lineTo(upper_r + step_fillet, BARREL_STEP2_Z)
        .lineTo(upper_r, BARREL_STEP2_Z + step_fillet)
        # Upper section outer wall
        .lineTo(upper_r, BARREL_Z1 - 0.0020)
        # Rolled rim at top
        .lineTo(rim_r, BARREL_Z1 - 0.0015)
        .lineTo(rim_r, BARREL_Z1)
        # Inner wall (uniform bore for the main travel, then neck inner taper)
        .lineTo(bore, BARREL_Z1)
        .lineTo(bore, BARREL_Z0)
        .lineTo(neck_inner_bot, neck_z0)
        .close()
    )
    return profile.revolve(360.0, (0.0, 0.0, 0.0), (0.0, 1.0, 0.0))


def _graduation_marks() -> cq.Workplane:
    """Thin embossed scale tick marks on the barrel surface.

    Marks are placed on the mid-section outer surface where the barrel is
    widest, giving maximum readable area for the graduation scale.
    """
    marks = None
    r = BARREL_MID_OUTER_R        # marks sit on the wide mid section
    z_lo = BARREL_STEP1_Z + 0.005
    z_hi = BARREL_STEP2_Z - 0.005
    n = 20                        # minor ticks across the volume range
    for i in range(n + 1):
        z = z_lo + (z_hi - z_lo) * i / n
        major = i % 5 == 0
        half_len = 0.0036 if major else 0.0018
        tick = (
            cq.Workplane("XY")
            .workplane(offset=z)
            .box(0.0009, half_len, 0.0007, centered=(True, True, True))
            .translate((r - 0.0001, 0.0, 0.0))
        )
        marks = tick if marks is None else marks.union(tick)
    return marks


def _finger_flanges() -> cq.Workplane:
    """Two flat thumb-and-forefinger wings at the top of the barrel."""
    z = BARREL_Z1 - 0.0025
    plate = (
        cq.Workplane("XY")
        .workplane(offset=z)
        .box(FLANGE_W, FLANGE_DEPTH, FLANGE_THICK, centered=(True, True, True))
        .edges("|Z")
        .fillet(0.0028)
    )
    bore = (
        cq.Workplane("XY")
        .workplane(offset=z)
        .circle(BARREL_UPPER_OUTER_R + 0.0008)
        .extrude(FLANGE_THICK * 3, both=True)
    )
    return plate.cut(bore)


def _hub() -> cq.Workplane:
    """Stainless Luer hub: a stepped tapered collar with an open throat."""
    body = (
        cq.Workplane("XZ")
        .moveTo(0.0, HUB_Z1)
        .lineTo(HUB_TOP_R, HUB_Z1)
        .lineTo(HUB_TOP_R, HUB_Z1 - 0.0035)
        .lineTo(HUB_TOP_R - 0.0012, HUB_Z1 - 0.0050)
        .lineTo(HUB_BOT_R, HUB_Z0 + 0.0035)
        .lineTo(HUB_BOT_R, HUB_Z0)
        .lineTo(0.0, HUB_Z0)
        .close()
        .revolve(360.0, (0.0, 0.0, 0.0), (0.0, 1.0, 0.0))
    )
    # Hollow throat: shallow socket in the upper hub only. The throat radius
    # and depth stay well within the hub wall at every height so the body
    # remains one connected solid.
    throat_r = HUB_TOP_R - 0.0030   # 0.005 — leaves wall at all heights
    throat_depth = 0.0060           # ends at z ≈ -0.0055, above the taper
    throat = (
        cq.Workplane("XY")
        .workplane(offset=HUB_Z1 + 0.0005)
        .circle(throat_r)
        .extrude(-throat_depth)
    )
    body = body.cut(throat)
    # Annular groove on the upper hub exterior for Luer-lock visual detail.
    # Shallow cut that stays within the hub wall without reaching the bore.
    groove_outer = HUB_TOP_R + 0.0001
    groove_inner = HUB_TOP_R - 0.0004
    groove = (
        cq.Workplane("XY")
        .workplane(offset=HUB_Z1 - 0.0018)
        .circle(groove_outer)
        .circle(groove_inner)
        .extrude(0.0010)
    )
    body = body.cut(groove)
    return body


def _needle() -> cq.Workplane:
    """Thin stainless needle with a beveled tip and a hollow lumen face.

    A small base collar sits flush against the hub bottom face so the needle
    mesh contacts the hub mesh for visual connectivity.
    """
    # Needle shaft from the hub bottom face downward.
    shaft = (
        cq.Workplane("XY")
        .workplane(offset=NEEDLE_Z1)
        .circle(NEEDLE_R)
        .extrude(-(NEEDLE_LEN - 0.0030))
    )
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
    # Base collar: a small disc that straddles the hub bottom face, extending
    # slightly into the hub body for guaranteed mesh surface intersection.
    collar_r = 0.0020
    collar_bot = NEEDLE_Z1 - 0.0008
    collar_top = NEEDLE_Z1 + 0.0008   # embeds into the hub bottom face
    collar = (
        cq.Workplane("XY")
        .workplane(offset=collar_bot)
        .circle(collar_r)
        .extrude(collar_top - collar_bot)
    )
    return shaft.union(tip).union(collar)


def _liquid() -> cq.Workplane:
    """Teal medication column inside the barrel bore, capped by the gasket."""
    liq_bot = BARREL_Z0 + 0.0002
    liq_top = GASKET_REST_Z - 0.0002
    return (
        cq.Workplane("XY")
        .workplane(offset=liq_bot)
        .circle(BARREL_BORE_R)
        .extrude(liq_top - liq_bot)
    )


def _plunger_rod() -> cq.Workplane:
    """Cross-rib ('+') plunger spine running up out of the barrel."""
    z0 = GASKET_REST_Z + GASKET_LEN
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
        .fillet(0.0010)
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
        .fillet(0.0008)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wide_barrel_syringe")
    for mat in (TEAL, GLASS, STEEL, RUBBER, WHITE_PLASTIC, INK):
        model.material(mat.name, rgba=mat.rgba)

    # ---- Root: barrel assembly (barrel + flanges + liquid + hub + needle) ----
    rest = Origin(xyz=REST_XYZ, rpy=REST_RPY)
    barrel = model.part("barrel_assembly")
    barrel.visual(mesh_from_cadquery(_barrel_shell(), "barrel_shell"), origin=rest, material=GLASS, name="barrel_shell")
    barrel.visual(mesh_from_cadquery(_liquid(), "liquid"), origin=rest, material=TEAL, name="liquid")
    barrel.visual(mesh_from_cadquery(_graduation_marks(), "scale_marks"), origin=rest, material=INK, name="scale_marks")
    barrel.visual(mesh_from_cadquery(_finger_flanges(), "finger_flanges"), origin=rest, material=WHITE_PLASTIC, name="finger_flanges")
    barrel.visual(mesh_from_cadquery(_hub(), "luer_hub"), origin=rest, material=STEEL, name="luer_hub")
    barrel.visual(mesh_from_cadquery(_needle(), "needle"), origin=rest, material=STEEL, name="needle")

    # ---- Child: plunger (rod + gasket + thumb rest) ----
    plunger = model.part("plunger")
    plunger.visual(mesh_from_cadquery(_gasket(), "gasket"), material=RUBBER, name="gasket")
    plunger.visual(mesh_from_cadquery(_plunger_rod(), "plunger_rod"), material=WHITE_PLASTIC, name="plunger_rod")
    plunger.visual(mesh_from_cadquery(_thumb_rest(), "thumb_rest"), material=WHITE_PLASTIC, name="thumb_rest")

    # ---- Prismatic joint: plunger slides DOWN the barrel bore to dispense ----
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

    # --- Barrel has a stepped wider mid section ---
    barrel_box = ctx.part_element_world_aabb(barrel, elem="barrel_shell")
    # The barrel shell should be significantly wider (X or Z in world) than a
    # standard 3 mL barrel (~11.6 mm dia). The wide mid section should push
    # the cross-section well above 25 mm.
    if barrel_box is not None:
        barrel_x_span = barrel_box[1][0] - barrel_box[0][0]
        barrel_z_span = barrel_box[1][2] - barrel_box[0][2]
        max_cross = max(barrel_x_span, barrel_z_span)
        ctx.check(
            "barrel has a wide mid section (cross-section > 25 mm)",
            max_cross > 0.025,
            details=f"x_span={barrel_x_span:.4f}, z_span={barrel_z_span:.4f}",
        )
    else:
        ctx.fail("barrel shell AABB missing", "Could not measure barrel shell")

    # --- Barrel bore is broader than a standard syringe (>16 mm dia) ---
    # The liquid column fills the bore, so its cross-section reflects the bore.
    liquid_box = ctx.part_element_world_aabb(barrel, elem="liquid")
    if liquid_box is not None:
        liq_x_span = liquid_box[1][0] - liquid_box[0][0]
        liq_z_span = liquid_box[1][2] - liquid_box[0][2]
        liq_cross = max(liq_x_span, liq_z_span)
        ctx.check(
            "barrel bore is broad (liquid cross-section > 16 mm)",
            liq_cross > 0.016,
            details=f"liquid_x={liq_x_span:.4f}, liquid_z={liq_z_span:.4f}",
        )

    # --- Needle extends beyond the hub toward the -Y (needle) end ---
    needle_box = ctx.part_element_world_aabb(barrel, elem="needle")
    hub_box = ctx.part_element_world_aabb(barrel, elem="luer_hub")
    ctx.check(
        "needle extends beyond the hub along the barrel axis",
        needle_box is not None and hub_box is not None and needle_box[0][1] < hub_box[0][1] - 0.02,
        details=f"needle_min_y={None if needle_box is None else needle_box[0][1]}",
    )

    # --- Liquid sits inside the barrel along the bore (Y) ---
    ctx.check(
        "teal liquid is inside the barrel between neck and rim",
        liquid_box is not None
        and barrel_box is not None
        and liquid_box[1][1] < barrel_box[1][1]
        and liquid_box[0][1] > barrel_box[0][1],
        details=f"liquid_y={None if liquid_box is None else (liquid_box[0][1], liquid_box[1][1])}",
    )

    # --- Finger flanges span wider than the barrel mid section ---
    flange_box = ctx.part_element_world_aabb(barrel, elem="finger_flanges")
    ctx.check(
        "finger flanges are wider than the barrel lower section",
        flange_box is not None and (flange_box[1][0] - flange_box[0][0]) > BARREL_LOWER_OUTER_R * 3,
        details=f"flange_x_span={None if flange_box is None else flange_box[1][0]-flange_box[0][0]}",
    )

    # --- Thumb rest sits past the barrel rim at the +Y (plunger) end ---
    thumb_box = ctx.part_element_world_aabb(plunger, elem="thumb_rest")
    ctx.check(
        "thumb rest is beyond the barrel rim",
        thumb_box is not None and barrel_box is not None and thumb_box[0][1] > barrel_box[1][1] - 0.001,
        details=f"thumb_min_y={None if thumb_box is None else thumb_box[0][1]}",
    )

    # --- Gasket is retained inside the barrel bore at rest ---
    ctx.expect_within(
        plunger,
        barrel,
        axes="xz",
        inner_elem="gasket",
        outer_elem="barrel_shell",
        margin=0.001,
        name="gasket stays within the barrel bore",
    )
    ctx.expect_overlap(
        plunger,
        barrel,
        axes="y",
        elem_a="gasket",
        elem_b="barrel_shell",
        min_overlap=0.003,
        name="gasket is inserted in the barrel at rest",
    )

    # --- Decisive pose: pushing the plunger moves gasket toward the needle ---
    rest_pos = ctx.part_world_position(plunger)
    with ctx.pose({joint: PLUNGER_TRAVEL}):
        pushed_pos = ctx.part_world_position(plunger)
        ctx.expect_within(
            plunger,
            barrel,
            axes="xz",
            inner_elem="gasket",
            outer_elem="barrel_shell",
            margin=0.001,
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
        rest_pos is not None and pushed_pos is not None and pushed_pos[1] < rest_pos[1] - 0.02,
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
