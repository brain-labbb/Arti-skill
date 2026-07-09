from __future__ import annotations

# Manual barrel hand pump (cast-metal piston pump).
#
# Real object: a vertical machined-aluminium pump barrel mounted on a cast base
# fitting by a yoke clamp. A piston rod telescopes up out of a knurled top cap
# and is capped with a large black ball knob; pushing the knob down drives the
# piston into the barrel (PRIMARY = PRISMATIC stroke). A black foot/wing lever
# with a white end cap is pinned to the yoke and rocks about that pin
# (SECONDARY = REVOLUTE). A rigid cast gooseneck spout exits the base outlet,
# arches away from the pump body, and ends in a downturned hollow nozzle.
from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
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
KNOB_R = 0.030             # ball knob radius
KNOB_CENTER_Z = 0.390      # ball knob center height at rest

CUP_LEN = 0.018            # piston cup length
CUP_CENTER_Z = 0.230       # piston cup center height at rest (inside the bore)
STROKE = 0.065             # plunger downstroke travel; ball clears the cap at full stroke

YOKE_BASE_Z = 0.022        # top of the cast base fitting / yoke seat
YOKE_PIN_Z = 0.044         # lever pivot pin height (below the barrel rim)
CLEVIS_X = -0.052          # clevis offset out past the base casting (-X)

SPOUT_R = 0.0078           # rigid cast gooseneck tube radius
NOZZLE_X = 0.096           # downturned nozzle center, offset clear of lever
NOZZLE_Y = 0.016
NOZZLE_BOTTOM_Z = 0.022
NOZZLE_TOP_Z = 0.066


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


def _spout_socket_shape() -> cq.Workplane:
    """Short cast socket slipped over the existing base outlet stub."""
    socket = (
        cq.Workplane("XY")
        .circle(0.0115)
        .circle(0.0062)
        .extrude(0.028, both=True)
        .rotate((0, 0, 0), (1, 0, 0), 90.0)   # socket bore runs along Y
        .translate((0.0, -0.061, 0.030))
    )
    return socket


def _downturned_nozzle_shape() -> cq.Workplane:
    """Open hollow vertical nozzle and small cast lips at the spout outlet."""
    h = NOZZLE_TOP_Z - NOZZLE_BOTTOM_Z
    body = (
        cq.Workplane("XY")
        .circle(0.0090)
        .extrude(h)
        .translate((NOZZLE_X, NOZZLE_Y, NOZZLE_BOTTOM_Z))
    )
    lower_lip = (
        cq.Workplane("XY")
        .circle(0.0115)
        .extrude(0.006)
        .translate((NOZZLE_X, NOZZLE_Y, NOZZLE_BOTTOM_Z))
    )
    upper_collar = (
        cq.Workplane("XY")
        .circle(0.0105)
        .extrude(0.008)
        .translate((NOZZLE_X, NOZZLE_Y, NOZZLE_TOP_Z - 0.008))
    )
    bore = (
        cq.Workplane("XY")
        .circle(0.0052)
        .extrude(h + 0.012)
        .translate((NOZZLE_X, NOZZLE_Y, NOZZLE_BOTTOM_Z - 0.006))
    )
    return body.union(lower_lip).union(upper_collar).cut(bore)


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
        .box(abs(CLEVIS_X) + 0.030, 0.044, 0.010, centered=(True, True, False))
        .translate((CLEVIS_X / 2.0, 0.0, YOKE_PIN_Z - 0.028))
    )
    yoke = yoke.union(web)
    # Pivot pin spanning the cheeks (lever pivots on this, axis along Y).
    pin = (
        cq.Workplane("XY")
        .circle(0.005)
        .extrude(0.034, both=True)            # centered cylinder along Z
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
    model.material("spout_cast", rgba=(0.36, 0.37, 0.39, 1.0))

    # --- ROOT: base fitting + yoke + barrel + cap + guide collar + spout ---
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

    # Rigid cast gooseneck spout: the parent hose is replaced by a metal tube
    # slipped over the unchanged base outlet, arched high enough to read as
    # fixed plumbing, then terminating in a vertical downturned open nozzle.
    spout_pts = [
        (0.0, -0.060, 0.030),            # seated on existing -Y outlet stub
        (0.030, -0.087, 0.052),
        (0.070, -0.076, 0.083),
        (0.105, -0.038, 0.091),         # high cast gooseneck crown
        (0.112, 0.004, 0.079),
        (NOZZLE_X, NOZZLE_Y, NOZZLE_TOP_Z - 0.002),
    ]
    spout_geom = tube_from_spline_points(
        spout_pts,
        radius=SPOUT_R,
        samples_per_segment=18,
        radial_segments=24,
        cap_ends=True,
    )
    body.visual(
        mesh_from_cadquery(_spout_socket_shape(), "spout_socket", tolerance=TOL, angular_tolerance=ATOL),
        material="spout_cast",
        name="spout_socket",
    )
    body.visual(
        mesh_from_geometry(spout_geom, "gooseneck_spout"),
        material="spout_cast",
        name="gooseneck_spout",
    )
    body.visual(
        mesh_from_cadquery(_downturned_nozzle_shape(), "downturned_nozzle", tolerance=TOL, angular_tolerance=ATOL),
        material="spout_cast",
        name="downturned_nozzle",
    )

    # --- PISTON: rod + ball knob + hidden piston cup (PRISMATIC plunger) ---
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
    # Rod runs from the cup up through the cap gland to just under the knob.
    rod_bottom = cup_top - 0.003
    rod_top = KNOB_CENTER_Z - KNOB_R + 0.006
    rod_len = rod_top - rod_bottom
    piston.visual(
        Cylinder(radius=ROD_R, length=rod_len),
        origin=Origin(xyz=(0.0, 0.0, rod_bottom + rod_len / 2.0)),
        material="rod_steel",
        name="rod",
    )
    # Large black ball knob handle.
    piston.visual(
        Sphere(radius=KNOB_R),
        origin=Origin(xyz=(0.0, 0.0, KNOB_CENTER_Z)),
        material="matte_black",
        name="ball_knob",
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
    # PRIMARY: plunger stroke. Push the ball knob DOWN into the barrel.
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
    spout = body.get_visual("gooseneck_spout")
    nozzle = body.get_visual("downturned_nozzle")
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
        "lever is revolute",
        rocker.joint_type == ArticulationType.REVOLUTE,
        details=f"got {rocker.joint_type}",
    )
    ctx.check(
        "lever pivot axis is along Y",
        abs(rocker.axis[1]) > 0.99 and abs(rocker.axis[0]) < 1e-6 and abs(rocker.axis[2]) < 1e-6,
        details=f"axis={rocker.axis}",
    )
    ctx.check(
        "plunger stroke limits unchanged",
        plunger.motion_limits is not None
        and plunger.motion_limits.lower == 0.0
        and plunger.motion_limits.upper == STROKE,
        details=f"limits={plunger.motion_limits}",
    )
    ctx.check(
        "lever rocker limits unchanged",
        rocker.motion_limits is not None
        and rocker.motion_limits.lower == -0.5
        and rocker.motion_limits.upper == 0.5
        and tuple(rocker.origin.xyz) == (CLEVIS_X, 0.0, YOKE_PIN_Z),
        details=f"limits={rocker.motion_limits}, origin={rocker.origin}",
    )

    # --- Hero geometry placement (rest pose) ---
    # Ball knob sits well above the barrel/cap, centered on the axis.
    ctx.expect_gap(
        piston, body, axis="z",
        positive_elem="ball_knob", negative_elem="cap",
        min_gap=0.005,
        name="ball knob clears the cap above the barrel",
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
    visual_names = {v.name for v in body.visuals}
    ctx.check(
        "loose rubber hose was replaced",
        "hose" not in visual_names and spout is not None and nozzle is not None,
        details=f"body visuals={sorted(visual_names)}",
    )
    spout_aabb = ctx.part_element_world_aabb(body, elem="gooseneck_spout")
    ctx.check(
        "cast gooseneck spout arches above the outlet",
        spout_aabb is not None
        and spout_aabb[1][2] > 0.088
        and spout_aabb[0][2] > 0.018
        and spout_aabb[1][0] > 0.105,
        details=f"spout aabb={spout_aabb}",
    )
    nozzle_aabb = ctx.part_element_world_aabb(body, elem="downturned_nozzle")
    ctx.check(
        "spout ends in a downturned vertical nozzle",
        nozzle_aabb is not None
        and spout_aabb is not None
        and nozzle_aabb[1][2] - nozzle_aabb[0][2] > 0.040
        and nozzle_aabb[1][2] < spout_aabb[1][2]
        and nozzle_aabb[0][2] > 0.015
        and (nozzle_aabb[1][0] - nozzle_aabb[0][0]) < 0.026
        and (nozzle_aabb[1][1] - nozzle_aabb[0][1]) < 0.026,
        details=f"nozzle aabb={nozzle_aabb}, spout aabb={spout_aabb}",
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

    rest_knob = ctx.part_element_world_aabb(piston, elem="ball_knob")

    # --- PRIMARY motion: pumping lowers the knob and keeps the cup engaged ---
    with ctx.pose({plunger: STROKE}):
        pushed_knob = ctx.part_element_world_aabb(piston, elem="ball_knob")
        ctx.check(
            "pumping the plunger lowers the ball knob",
            rest_knob is not None
            and pushed_knob is not None
            and pushed_knob[1][2] < rest_knob[1][2] - 0.055,
            details=f"rest_top={rest_knob}, pushed_top={pushed_knob}",
        )
        ctx.expect_gap(
            piston, body, axis="z",
            positive_elem="ball_knob", negative_elem="cap",
            min_gap=0.005,
            name="ball knob clears the cap at full stroke",
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
