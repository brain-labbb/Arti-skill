from __future__ import annotations

# Manual barrel hand pump (cast-metal piston pump).
#
# Real object: a vertical machined-aluminium pump barrel mounted on a cast base
# fitting by a yoke clamp. A piston rod telescopes up out of a knurled top cap
# and is capped with a broad black mushroom palm push-disc; pushing the disc down drives the
# piston into the barrel (PRIMARY = PRISMATIC stroke). A black foot/wing lever
# with a white end cap is pinned to the yoke and rocks about that pin
# (SECONDARY = REVOLUTE). A black rubber hose exits the base outlet, loops
# loosely around the pump, and finishes as a free end resting on the ground.
from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)

TOL = 0.0006
ATOL = 0.2

# ----------------------------------------------------------------------------
# Key dimensions (meters)
# ----------------------------------------------------------------------------
BARREL_R = 0.042            # outer radius of the main pump barrel
BARREL_BORE_R = 0.0345     # inner bore radius (hollow barrel)
BARREL_BOTTOM_Z = 0.072    # barrel lower rim sits above the yoke/lever
BARREL_TOP_Z = 0.262       # barrel upper rim (under the cap)
CAP_TOP_Z = 0.279          # top of knurled cap

ROD_R = 0.0085             # piston rod radius
DISC_R = 0.044             # wide palm push-disc radius
DISC_BOTTOM_Z = 0.360      # underside of the palm disc at rest
DISC_TOP_Z = 0.390         # domed top of the palm disc at rest

CUP_LEN = 0.018            # piston cup length
CUP_CENTER_Z = 0.230       # piston cup center height at rest (inside the bore)
STROKE = 0.065             # unchanged plunger downstroke travel; disc clears the cap at full stroke

YOKE_BASE_Z = 0.022        # top of the cast base fitting / yoke seat
YOKE_PIN_Z = 0.044         # lever pivot pin height (below the barrel rim)
CLEVIS_X = -0.052          # clevis offset out past the base casting (-X)


# ----------------------------------------------------------------------------
# CadQuery shapes (returned in the part-local frame, Z up)
# ----------------------------------------------------------------------------
def _barrel_shape() -> cq.Workplane:
    """Hollow machined barrel: open-top tube with a rolled lower rim."""
    barrel = (
        cq.Workplane("XY")
        .circle(BARREL_R)
        .workplane(offset=BARREL_TOP_Z - BARREL_BOTTOM_Z)
        .circle(BARREL_R)
        .loft(combine=True)
        .translate((0.0, 0.0, BARREL_BOTTOM_Z))
    )
    # Bore it fully through so it reads as a real open cylinder the piston
    # slides inside. The bore opens just above the lower rim.
    bore = (
        cq.Workplane("XY")
        .circle(BARREL_BORE_R)
        .extrude(BARREL_TOP_Z - BARREL_BOTTOM_Z + 0.020)
        .translate((0.0, 0.0, BARREL_BOTTOM_Z + 0.010))
    )
    barrel = barrel.cut(bore)
    # Rolled lower rim (slightly proud collar) for a finished casting look.
    rim = (
        cq.Workplane("XY")
        .circle(BARREL_R + 0.0035)
        .extrude(0.012)
        .translate((0.0, 0.0, BARREL_BOTTOM_Z))
    )
    barrel = barrel.union(rim)
    return barrel


def _cap_shape() -> cq.Workplane:
    """Knurled top cap ring screwed onto the barrel, with a rod gland."""
    cap = (
        cq.Workplane("XY")
        .circle(BARREL_R + 0.004)
        .extrude(CAP_TOP_Z - BARREL_TOP_Z)
        .translate((0.0, 0.0, BARREL_TOP_Z))
    )
    cap = cap.edges(">Z").chamfer(0.0015)
    # Vertical knurl flutes around the cap.
    n_flutes = 40
    flute_h = CAP_TOP_Z - BARREL_TOP_Z
    for i in range(n_flutes):
        a = 2.0 * pi * i / n_flutes
        groove = (
            cq.Workplane("XY")
            .circle(0.0012)
            .extrude(flute_h + 0.002)
            .translate((BARREL_R + 0.0045, 0.0, BARREL_TOP_Z - 0.001))
            .rotate((0, 0, 0), (0, 0, 1), a * 180.0 / pi)
        )
        cap = cap.cut(groove)
    # Central gland boss with bore for the rod to pass through.
    boss = (
        cq.Workplane("XY")
        .circle(0.018)
        .extrude(0.010)
        .translate((0.0, 0.0, CAP_TOP_Z))
    )
    gland_bore = (
        cq.Workplane("XY")
        .circle(ROD_R - 0.0006)
        .extrude(0.040)
        .translate((0.0, 0.0, BARREL_TOP_Z - 0.001))
    )
    cap = cap.union(boss).cut(gland_bore)
    return cap


def _guide_collar_shape() -> cq.Workplane:
    """Lower split guide ring clamped around the barrel, just above the yoke."""
    collar = (
        cq.Workplane("XY")
        .circle(BARREL_R + 0.006)
        .circle(BARREL_R + 0.0005)
        .extrude(0.014)
        .translate((0.0, 0.0, BARREL_BOTTOM_Z - 0.004))
    )
    # Two clamp ears on the +X / -X sides.
    for sx in (1.0, -1.0):
        ear = (
            cq.Workplane("XY")
            .box(0.010, 0.016, 0.014, centered=(True, True, False))
            .translate((sx * (BARREL_R + 0.008), 0.0, BARREL_BOTTOM_Z - 0.004))
        )
        collar = collar.union(ear)
    return collar


def _base_fitting_shape() -> cq.Workplane:
    """Cast base outlet fitting under the barrel: hex-ish boss + outlet stub."""
    # Tapered cast body.
    body = (
        cq.Workplane("XY")
        .circle(0.030)
        .workplane(offset=YOKE_BASE_Z)
        .circle(0.034)
        .loft(combine=True)
    )
    # Lower foot flange.
    foot = (
        cq.Workplane("XY")
        .polygon(6, 0.082)
        .extrude(0.012)
    )
    foot = foot.edges("|Z").fillet(0.003)
    body = body.union(foot)
    # Neck that the barrel/yoke seats onto; it rises into the barrel base so
    # the barrel is positively seated on the casting.
    neck = (
        cq.Workplane("XY")
        .circle(0.026)
        .extrude(BARREL_BOTTOM_Z + 0.012 - YOKE_BASE_Z)
        .translate((0.0, 0.0, YOKE_BASE_Z))
    )
    body = body.union(neck)
    # Side outlet stub for the hose: a short cylinder pointing -Y.
    outlet = (
        cq.Workplane("XY")
        .circle(0.0085)
        .extrude(0.030, both=True)
        .rotate((0, 0, 0), (1, 0, 0), 90.0)   # lay along Y
        .translate((0.0, -0.038, 0.030))
    )
    body = body.union(outlet)
    return body


def _yoke_shape() -> cq.Workplane:
    """Clamp collar around the base neck plus a clevis that carries the lever
    pivot pin (pin axis = Y), sitting low beside the barrel base."""
    # Central collar gripping the base neck and supporting the barrel above.
    collar = (
        cq.Workplane("XY")
        .circle(0.036)
        .circle(0.0265)
        .extrude(0.040)
        .translate((0.0, 0.0, YOKE_BASE_Z))
    )
    yoke = collar
    # Clevis is offset out along -X (past the base casting) so the lever arm
    # never crosses the central body. Two cheeks spaced along Y carry the pin.
    cheek_bot = YOKE_PIN_Z - 0.018
    for sy in (1.0, -1.0):
        cheek = (
            cq.Workplane("XY")
            .box(0.024, 0.011, 0.030, centered=(True, True, False))
            .translate((CLEVIS_X, sy * 0.024, cheek_bot))
        )
        cheek = cheek.edges("|X").fillet(0.005)
        yoke = yoke.union(cheek)
    # Low web spine tying the clevis back to the collar, kept below the lever
    # arm so it never collides with the swinging grip.
    web = (
        cq.Workplane("XY")
        .box(abs(CLEVIS_X) + 0.030, 0.044, 0.014, centered=(True, True, False))
        .translate((CLEVIS_X / 2.0, 0.0, YOKE_PIN_Z - 0.028))
    )
    yoke = yoke.union(web)
    # Pivot pin spanning the cheeks (lever pivots on this, axis along Y).
    pin = (
        cq.Workplane("XY")
        .circle(0.005)
        .extrude(0.040, both=True)            # centered cylinder along Z
        .rotate((0, 0, 0), (1, 0, 0), 90.0)   # lay the pin along Y
        .translate((CLEVIS_X, 0.0, YOKE_PIN_Z))
    )
    yoke = yoke.union(pin)
    return yoke


def _lever_shape() -> cq.Workplane:
    """Black foot/wing lever: a thick grip bar with a hub that wraps the pin.

    Authored in the lever-local frame with the pivot at the origin and the
    grip extending along -X (toward the foot side seen in the image). The
    pivot pin runs along Y, so the hub is a Y-axis cylinder.
    """
    # Pivot hub (wraps the yoke pin; cylinder axis = Y).
    hub = (
        cq.Workplane("XY")
        .circle(0.012)
        .extrude(0.013, both=True)
        .rotate((0, 0, 0), (1, 0, 0), 90.0)
    )
    # Grip arm extending out from the hub along -X (toward the foot side).
    # Extrude +Z then rotate -90 about Y so the bar runs into -X.
    arm = (
        cq.Workplane("XY")
        .circle(0.012)
        .extrude(0.130)
        .rotate((0, 0, 0), (0, 1, 0), -90.0)
    )
    lever = hub.union(arm)
    # Bore for the pivot pin, cut through the whole lever. The bore is slightly
    # smaller than the pin so the hub positively captures (seats on) the pin;
    # that intentional capture overlap is allowed in run_tests().
    pin_bore = (
        cq.Workplane("XY")
        .circle(0.0044)
        .extrude(0.040, both=True)
        .rotate((0, 0, 0), (1, 0, 0), 90.0)
    )
    lever = lever.cut(pin_bore)
    return lever


def _lever_endcap_shape() -> cq.Workplane:
    """White plastic end cap on the free end of the lever grip."""
    cap = (
        cq.Workplane("XY")
        .circle(0.0135)
        .extrude(0.014)
        .edges(">Z")
        .fillet(0.003)
    )
    cap = cap.rotate((0, 0, 0), (0, 1, 0), -90.0).translate((-0.122, 0.0, 0.0))
    return cap


def _piston_head_shape() -> cq.Workplane:
    """Piston cup at the bottom of the rod, hidden inside the barrel bore."""
    head = (
        cq.Workplane("XY")
        .circle(BARREL_BORE_R - 0.0015)
        .extrude(0.018)
    )
    head = head.edges(">Z").fillet(0.002)
    return head


def _push_disc_geometry() -> LatheGeometry:
    """Lathe-turned mushroom / palm push-disc for the top of the plunger rod."""
    # A single radius/Z profile gives the replacement handle a broad flattened
    # palm pad, rounded rim, shallow top dish, and a small underside boss that
    # visibly seats onto the steel rod.  The profile is authored in the
    # piston-local world Z location so the prismatic stroke remains exactly the
    # same as the parent.
    mid_z = (DISC_BOTTOM_Z + DISC_TOP_Z) / 2.0
    profile = [
        (0.0, DISC_BOTTOM_Z),
        (0.0135, DISC_BOTTOM_Z),
        (0.0180, DISC_BOTTOM_Z + 0.004),
        (0.0390, DISC_BOTTOM_Z + 0.007),
        (DISC_R, mid_z - 0.001),
        (0.0415, DISC_TOP_Z - 0.006),
        (0.0300, DISC_TOP_Z),
        (0.0240, DISC_TOP_Z - 0.0012),
        (0.0060, DISC_TOP_Z - 0.0028),
        (0.0, DISC_TOP_Z - 0.0024),
    ]
    return LatheGeometry(profile, segments=72, closed=True)


# ----------------------------------------------------------------------------
# Model
# ----------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="manual_barrel_hand_pump")

    model.material("barrel_alu", rgba=(0.46, 0.49, 0.54, 1.0))
    model.material("cap_steel", rgba=(0.34, 0.36, 0.40, 1.0))
    model.material("cast_metal", rgba=(0.58, 0.59, 0.60, 1.0))
    model.material("matte_black", rgba=(0.08, 0.08, 0.09, 1.0))
    model.material("rod_steel", rgba=(0.40, 0.42, 0.46, 1.0))
    model.material("white_cap", rgba=(0.90, 0.91, 0.92, 1.0))
    model.material("rubber_hose", rgba=(0.06, 0.06, 0.07, 1.0))

    # --- ROOT: base fitting + yoke + barrel + cap + guide collar + hose ---
    body = model.part("pump_body")
    body.visual(
        mesh_from_cadquery(_base_fitting_shape(), "base_fitting", tolerance=TOL, angular_tolerance=ATOL),
        material="cast_metal",
        name="base_fitting",
    )
    body.visual(
        mesh_from_cadquery(_yoke_shape(), "yoke", tolerance=TOL, angular_tolerance=ATOL),
        material="cast_metal",
        name="yoke",
    )
    body.visual(
        mesh_from_cadquery(_barrel_shape(), "barrel", tolerance=TOL, angular_tolerance=ATOL),
        material="barrel_alu",
        name="barrel",
    )
    body.visual(
        mesh_from_cadquery(_guide_collar_shape(), "guide_collar", tolerance=TOL, angular_tolerance=ATOL),
        material="cap_steel",
        name="guide_collar",
    )
    body.visual(
        mesh_from_cadquery(_cap_shape(), "cap", tolerance=TOL, angular_tolerance=ATOL),
        material="cap_steel",
        name="cap",
    )

    # Black rubber hose: from the base outlet stub (-Y), looping out and around
    # with a loose capped end dropped to the ground instead of reconnecting to
    # the yoke collar.
    hose_pts = [
        (0.0, -0.060, 0.030),     # plugged onto the outlet stub
        (0.02, -0.110, 0.014),
        (0.075, -0.115, 0.012),
        (0.105, -0.050, 0.014),
        (0.105, 0.030, 0.016),
        (0.070, 0.072, 0.013),
        (0.020, 0.088, 0.0085),
        (-0.032, 0.070, 0.0075),
        (-0.072, 0.040, 0.0075),  # free hose end rests on the floor
    ]
    hose_geom = tube_from_spline_points(
        hose_pts,
        radius=0.0075,
        samples_per_segment=16,
        radial_segments=18,
        cap_ends=True,
    )
    body.visual(
        mesh_from_geometry(hose_geom, "hose"),
        material="rubber_hose",
        name="hose",
    )

    # --- PISTON: rod + palm push-disc + hidden piston cup (PRISMATIC plunger) ---
    piston = model.part("piston")
    # Hidden piston cup high inside the barrel bore; it stays inserted across
    # the full downstroke. Authored in piston-local frame (rest = identity).
    cup_top = CUP_CENTER_Z + CUP_LEN / 2.0
    piston.visual(
        mesh_from_cadquery(_piston_head_shape(), "piston_head", tolerance=TOL, angular_tolerance=ATOL),
        origin=Origin(xyz=(0.0, 0.0, CUP_CENTER_Z)),
        material="matte_black",
        name="piston_head",
    )
    # Rod runs from the cup up through the cap gland into the underside boss of
    # the palm push-disc.
    rod_bottom = cup_top - 0.003
    rod_top = DISC_BOTTOM_Z + 0.006
    rod_len = rod_top - rod_bottom
    piston.visual(
        Cylinder(radius=ROD_R, length=rod_len),
        origin=Origin(xyz=(0.0, 0.0, rod_bottom + rod_len / 2.0)),
        material="rod_steel",
        name="rod",
    )
    # Broad black mushroom / palm push-disc handle, replacing the parent ball knob.
    piston.visual(
        mesh_from_geometry(_push_disc_geometry(), "push_disc"),
        material="matte_black",
        name="push_disc",
    )

    # --- LEVER: black foot lever + white end cap (REVOLUTE rocker) ---
    lever = model.part("lever")
    lever.visual(
        mesh_from_cadquery(_lever_shape(), "lever", tolerance=TOL, angular_tolerance=ATOL),
        material="matte_black",
        name="lever_arm",
    )
    lever.visual(
        mesh_from_cadquery(_lever_endcap_shape(), "lever_endcap", tolerance=TOL, angular_tolerance=ATOL),
        material="white_cap",
        name="lever_endcap",
    )

    # --- ARTICULATIONS ---
    # PRIMARY: plunger stroke. Push the palm disc DOWN into the barrel.
    # axis -Z so positive q lowers the piston (the pumping stroke).
    model.articulation(
        "barrel_to_piston",
        ArticulationType.PRISMATIC,
        parent=body,
        child=piston,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.40, lower=0.0, upper=STROKE),
    )

    # SECONDARY: foot lever rocks about the yoke pivot pin (axis along Y).
    # Positive q rotates the -X grip arm upward (right-hand rule about +Y).
    model.articulation(
        "yoke_to_lever",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lever,
        origin=Origin(xyz=(CLEVIS_X, 0.0, YOKE_PIN_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=2.0, lower=-0.5, upper=0.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("pump_body")
    piston = object_model.get_part("piston")
    lever = object_model.get_part("lever")

    plunger = object_model.get_articulation("barrel_to_piston")
    rocker = object_model.get_articulation("yoke_to_lever")

    barrel = body.get_visual("barrel")
    hose = body.get_visual("hose")
    rod = piston.get_visual("rod")

    # --- Joint type / axis claims ---
    ctx.check(
        "plunger is prismatic",
        plunger.joint_type == ArticulationType.PRISMATIC,
        details=f"got {plunger.joint_type}",
    )
    ctx.check(
        "plunger axis is vertical",
        abs(plunger.axis[2]) > 0.99 and abs(plunger.axis[0]) < 1e-6 and abs(plunger.axis[1]) < 1e-6,
        details=f"axis={plunger.axis}",
    )
    ctx.check(
        "plunger stroke is unchanged",
        abs(plunger.motion_limits.upper - STROKE) < 1e-9 and abs(plunger.motion_limits.lower) < 1e-9,
        details=f"limits={plunger.motion_limits}",
    )
    ctx.check(
        "lever is revolute",
        rocker.joint_type == ArticulationType.REVOLUTE,
        details=f"got {rocker.joint_type}",
    )
    ctx.check(
        "lever pivot axis is along Y",
        abs(rocker.axis[1]) > 0.99 and abs(rocker.axis[0]) < 1e-6 and abs(rocker.axis[2]) < 1e-6,
        details=f"axis={rocker.axis}",
    )

    # --- Hero geometry placement (rest pose) ---
    # Palm push-disc sits well above the barrel/cap, centered on the axis.
    ctx.expect_gap(
        piston, body, axis="z",
        positive_elem="push_disc", negative_elem="cap",
        min_gap=0.005,
        name="push disc clears the cap above the barrel",
    )
    push_disc_aabb = ctx.part_element_world_aabb(piston, elem="push_disc")
    ctx.check(
        "push disc is a wide flat palm pad",
        push_disc_aabb is not None
        and (push_disc_aabb[1][0] - push_disc_aabb[0][0]) > 0.080
        and (push_disc_aabb[1][2] - push_disc_aabb[0][2]) < 0.035,
        details=f"push_disc aabb={push_disc_aabb}",
    )
    ctx.expect_within(
        piston, body, axes="xy",
        inner_elem=rod, outer_elem=barrel, margin=0.001,
        name="rod is centered within the barrel footprint",
    )
    # The rod actually passes through / overlaps the cap gland in XY.
    ctx.expect_overlap(
        piston, body, axes="xy",
        elem_a="rod", elem_b="cap", min_overlap=0.010,
        name="rod runs through the cap gland",
    )
    # White end cap is on the lever, well off to the side (lever exists/placed).
    lever_aabb = ctx.part_world_aabb(lever)
    ctx.check(
        "lever grip extends to the side",
        lever_aabb is not None and lever_aabb[0][0] < -0.10,
        details=f"lever aabb={lever_aabb}",
    )
    hose_aabb = ctx.part_element_world_aabb(body, elem="hose")
    ctx.check(
        "hose has a loose end resting on the ground",
        hose is not None
        and hose_aabb is not None
        and hose_aabb[0][2] <= 0.0015
        and hose_aabb[0][0] < -0.070,
        details=f"hose aabb={hose_aabb}",
    )

    # --- Retained insertion: piston cup stays in the barrel bore ---
    # The piston cup is nested inside the hollow barrel; allow that overlap.
    ctx.allow_overlap(
        piston, body,
        elem_a="piston_head", elem_b="barrel",
        reason="The piston cup is intentionally represented as sliding inside the hollow barrel bore.",
    )
    ctx.allow_overlap(
        piston, body,
        elem_a="rod", elem_b="cap",
        reason="The piston rod passes through the cap gland bore.",
    )
    ctx.expect_overlap(
        piston, body, axes="z",
        elem_a="piston_head", elem_b="barrel", min_overlap=0.010,
        name="piston cup is inserted in the barrel at rest",
    )

    # --- Lever pivot: the hub captures (seats on) the yoke pin ---
    ctx.allow_overlap(
        lever, body,
        elem_a="lever_arm", elem_b="yoke",
        reason="The lever hub bore is sized to capture the yoke pivot pin so the lever stays seated on the pin it rotates about.",
    )
    ctx.expect_contact(
        lever, body,
        elem_a="lever_arm", elem_b="yoke",
        contact_tol=0.001,
        name="lever hub is seated on the yoke pivot pin",
    )

    rest_disc = ctx.part_element_world_aabb(piston, elem="push_disc")

    # --- PRIMARY motion: pumping lowers the push-disc and keeps the cup engaged ---
    with ctx.pose({plunger: STROKE}):
        pushed_disc = ctx.part_element_world_aabb(piston, elem="push_disc")
        ctx.check(
            "pumping the plunger lowers the push disc",
            rest_disc is not None
            and pushed_disc is not None
            and pushed_disc[1][2] < rest_disc[1][2] - 0.055,
            details=f"rest_top={rest_disc}, pushed_top={pushed_disc}",
        )
        ctx.expect_gap(
            piston, body, axis="z",
            positive_elem="push_disc", negative_elem="cap",
            min_gap=0.005,
            name="push disc clears the cap at full stroke",
        )
        ctx.expect_overlap(
            piston, body, axes="z",
            elem_a="piston_head", elem_b="barrel", min_overlap=0.010,
            name="piston cup stays inserted at full stroke",
        )

    # --- SECONDARY motion: lever rocks about the pin ---
    rest_endcap = ctx.part_element_world_aabb(lever, elem="lever_endcap")
    with ctx.pose({rocker: 0.45}):
        up_endcap = ctx.part_element_world_aabb(lever, elem="lever_endcap")
        ctx.check(
            "rocking the lever moves its end cap vertically",
            rest_endcap is not None
            and up_endcap is not None
            and abs(up_endcap[0][2] - rest_endcap[0][2]) > 0.03,
            details=f"rest={rest_endcap}, posed={up_endcap}",
        )

    return ctx.report()


object_model = build_object_model()
