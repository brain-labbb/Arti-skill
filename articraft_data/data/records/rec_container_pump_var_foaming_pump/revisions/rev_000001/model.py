from __future__ import annotations

# Foaming soap-dispenser bottle: fork of the clear press-pump dispenser.
# Same bottle body, shoulder, neck, and collar interface. The pump head is
# replaced with a tall wide-chamber foaming pump: a fat cylindrical mixing
# chamber rises well above the collar, topped by a flat push-down actuator
# that travels straight down (PRISMATIC) and springs back. A short stubby
# foamer spout exits near the chamber top (no long gooseneck). A shorter
# dip tube hangs below the stem.
#
# Frame: bottle main axis along +Z (vertical). Bottle bottom at z=0.
# Joint chain: bottle -> collar (FIXED)
#   collar --[REVOLUTE swivel +Z]--> head_carrier
#   head_carrier --[PRISMATIC press +Z]--> foamer_head

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CylinderGeometry,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---- bottle geometry (unchanged from parent) ----
BOTTLE_BOTTOM = 0.0
BOTTLE_BODY_TOP = 0.130
SHOULDER_TOP = 0.150
NECK_TOP = 0.168
BODY_R = 0.030
NECK_R = 0.0150

COLLAR_R = 0.0185
COLLAR_BOTTOM = 0.150
COLLAR_TOP = 0.176

PRESS_TRAVEL = 0.015

# ---- foamer head dimensions ----
FOAMER_R = 0.020           # chamber outer radius (40 mm diameter — wide)
FOAMER_BOTTOM = 0.186      # base of the mixing chamber
FOAMER_TOP = 0.258         # top of the chamber wall (72 mm tall chamber)
ACTUATOR_R = 0.023         # flat actuator disk radius (wider than chamber)
ACTUATOR_H = 0.007         # actuator disk thickness
ACTUATOR_TOP = FOAMER_TOP + ACTUATOR_H

# Short stubby spout (no long gooseneck)
SPOUT_EXIT_Z = 0.244       # where spout exits the chamber
SPOUT_R = 0.007            # wider than parent (foam outlet)
SPOUT_REACH = 0.015        # short reach past chamber wall


# ==== bottle (unchanged) ====

def _bottle_mesh():
    wall = 0.0022
    outer = [
        (0.0, BOTTLE_BOTTOM),
        (BODY_R, BOTTLE_BOTTOM + 0.004),
        (BODY_R, BOTTLE_BODY_TOP),
        (NECK_R + 0.004, SHOULDER_TOP),
        (NECK_R, SHOULDER_TOP + 0.004),
        (NECK_R, NECK_TOP),
    ]
    inner = [
        (0.0, BOTTLE_BOTTOM + 0.006),
        (BODY_R - wall, BOTTLE_BOTTOM + 0.010),
        (BODY_R - wall, BOTTLE_BODY_TOP),
        (NECK_R + 0.004 - wall, SHOULDER_TOP),
        (NECK_R - wall, SHOULDER_TOP + 0.004),
        (NECK_R - wall, NECK_TOP + 0.002),
    ]
    geo = LatheGeometry.from_shell_profiles(outer, inner, segments=48)
    return mesh_from_geometry(geo, "bottle_shell")


def _neck_threads_mesh():
    geo = None
    for i, zz in enumerate((0.154, 0.160, 0.166)):
        ring = TorusGeometry(NECK_R + 0.0005, 0.0012, radial_segments=8, tubular_segments=36)
        ring.translate(0.0, 0.0, zz)
        geo = ring if geo is None else geo.merge(ring)
    return mesh_from_geometry(geo, "neck_threads")


# ==== collar (unchanged) ====

def _collar_mesh():
    outer = [
        (0.0, COLLAR_BOTTOM),
        (COLLAR_R, COLLAR_BOTTOM),
        (COLLAR_R, COLLAR_TOP),
        (NECK_R + 0.0015, COLLAR_TOP),
    ]
    inner = [
        (0.0, COLLAR_BOTTOM + 0.0025),
        (NECK_R + 0.0010, COLLAR_BOTTOM + 0.0025),
        (NECK_R + 0.0010, COLLAR_TOP + 0.002),
    ]
    geo = LatheGeometry.from_shell_profiles(outer, inner, segments=48)
    return mesh_from_geometry(geo, "collar_shell")


def _collar_knurl_mesh():
    geo = None
    h = COLLAR_TOP - COLLAR_BOTTOM
    for i in range(22):
        ang = 2 * math.pi * i / 22
        rib = CylinderGeometry(0.0012, h, radial_segments=6)
        rib.translate(COLLAR_R - 0.0002, 0.0, COLLAR_BOTTOM + h / 2.0)
        rib.rotate_z(ang)
        geo = rib if geo is None else geo.merge(rib)
    return mesh_from_geometry(geo, "collar_knurl")


# ==== foamer head (new) ====

def _foamer_chamber_mesh():
    """Tall wide cylindrical mixing chamber — hollow thin-walled shell."""
    outer = [
        (0.0, FOAMER_BOTTOM),
        (FOAMER_R - 0.003, FOAMER_BOTTOM),
        (FOAMER_R, FOAMER_BOTTOM + 0.004),
        (FOAMER_R, FOAMER_TOP - 0.004),
        (FOAMER_R - 0.003, FOAMER_TOP),
        (0.0, FOAMER_TOP),
    ]
    wall = 0.002
    inner = [
        (0.0, FOAMER_BOTTOM + 0.003),
        (FOAMER_R - wall - 0.003, FOAMER_BOTTOM + 0.003),
        (FOAMER_R - wall, FOAMER_BOTTOM + 0.006),
        (FOAMER_R - wall, FOAMER_TOP - 0.004),
        (FOAMER_R - wall - 0.003, FOAMER_TOP - 0.001),
        (0.0, FOAMER_TOP - 0.001),
    ]
    geo = LatheGeometry.from_shell_profiles(outer, inner, segments=48)
    return mesh_from_geometry(geo, "foamer_chamber")


def _chamber_grip_mesh():
    """Vertical grip ribs around the foamer chamber exterior."""
    geo = None
    n_ribs = 16
    grip_h = FOAMER_TOP - FOAMER_BOTTOM - 0.012
    grip_center_z = (FOAMER_BOTTOM + FOAMER_TOP) / 2.0
    for i in range(n_ribs):
        ang = 2 * math.pi * i / n_ribs
        rib = CylinderGeometry(0.0009, grip_h, radial_segments=4)
        rib.translate(FOAMER_R + 0.0003, 0.0, grip_center_z)
        rib.rotate_z(ang)
        geo = rib if geo is None else geo.merge(rib)
    return mesh_from_geometry(geo, "chamber_grip")


def _actuator_mesh():
    """Flat push-down actuator disk sitting on top of the chamber."""
    profile = [
        (0.0, FOAMER_TOP),
        (ACTUATOR_R - 0.002, FOAMER_TOP),
        (ACTUATOR_R, FOAMER_TOP + 0.002),
        (ACTUATOR_R, FOAMER_TOP + ACTUATOR_H - 0.002),
        (ACTUATOR_R - 0.002, FOAMER_TOP + ACTUATOR_H),
        (0.0, FOAMER_TOP + ACTUATOR_H),
    ]
    geo = LatheGeometry(profile, segments=48)
    return mesh_from_geometry(geo, "actuator_cap")


def _foamer_spout_mesh():
    """Short stubby foamer spout — wide tube, minimal droop, no gooseneck."""
    spout_pts = [
        (FOAMER_R - 0.002, 0.0, SPOUT_EXIT_Z),
        (FOAMER_R + 0.005, 0.0, SPOUT_EXIT_Z - 0.001),
        (FOAMER_R + SPOUT_REACH, 0.0, SPOUT_EXIT_Z - 0.004),
    ]
    geo = tube_from_spline_points(
        spout_pts, radius=SPOUT_R, samples_per_segment=10, radial_segments=14
    )
    # Wide foam nozzle tip at the spout end
    tip = CylinderGeometry(SPOUT_R + 0.001, 0.005, radial_segments=16)
    tip.translate(FOAMER_R + SPOUT_REACH, 0.0, SPOUT_EXIT_Z - 0.004)
    geo = geo.merge(tip)
    return mesh_from_geometry(geo, "foamer_spout")


def _foamer_stem_mesh():
    """Vertical stem from chamber bottom down through the carrier/collar."""
    stem_h = 0.040
    stem_center = FOAMER_BOTTOM - stem_h / 2.0  # top at FOAMER_BOTTOM
    stem = CylinderGeometry(0.009, stem_h, radial_segments=20)
    stem.translate(0.0, 0.0, stem_center)
    return mesh_from_geometry(stem, "foamer_stem")


def _short_dip_tube_mesh():
    """Shorter dip tube — foaming pumps draw from higher in the bottle."""
    tube_h = 0.100
    # Top connects to the stem bottom (FOAMER_BOTTOM - 0.040 = 0.146)
    tube_top = FOAMER_BOTTOM - 0.040
    tube_center = tube_top - tube_h / 2.0
    tube = CylinderGeometry(0.003, tube_h, radial_segments=12)
    tube.translate(0.0, 0.0, tube_center)
    return mesh_from_geometry(tube, "dip_tube")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="foaming_soap_dispenser")

    clear = model.material("clear_plastic", rgba=(0.74, 0.80, 0.82, 0.25))
    label_mat = model.material("label", rgba=(0.96, 0.96, 0.94, 1.0))
    white = model.material("pump_white", rgba=(0.93, 0.93, 0.94, 1.0))
    foam_white = model.material("foamer_white", rgba=(0.95, 0.95, 0.96, 1.0))
    accent_grey = model.material("grip_grey", rgba=(0.82, 0.83, 0.84, 1.0))
    tube_mat = model.material("tube_white", rgba=(0.88, 0.90, 0.90, 0.85))

    # ---- bottle (root): clear shell + label band + neck threads ----
    bottle = model.part("bottle")
    bottle.visual(_bottle_mesh(), material=clear, name="bottle_shell")
    label = LatheGeometry.from_shell_profiles(
        [
            (BODY_R + 0.0008, 0.040),
            (BODY_R + 0.0008, 0.110),
        ],
        [
            (BODY_R - 0.0002, 0.040),
            (BODY_R - 0.0002, 0.110),
        ],
        segments=48,
    )
    bottle.visual(mesh_from_geometry(label, "label_band"), material=label_mat, name="label_band")
    bottle.visual(_neck_threads_mesh(), material=clear, name="neck_threads")
    bottle.inertial = Inertial.from_geometry(
        Cylinder(BODY_R, BOTTLE_BODY_TOP),
        mass=0.20,
        origin=Origin(xyz=(0.0, 0.0, BOTTLE_BODY_TOP / 2.0)),
    )

    # ---- collar: white threaded cap, fixed onto the bottle neck ----
    collar = model.part("collar")
    collar.visual(_collar_mesh(), material=white, name="collar_shell")
    collar.visual(_collar_knurl_mesh(), material=white, name="collar_knurl")
    collar.inertial = Inertial.from_geometry(
        Cylinder(COLLAR_R, COLLAR_TOP - COLLAR_BOTTOM),
        mass=0.02,
        origin=Origin(xyz=(0.0, 0.0, (COLLAR_BOTTOM + COLLAR_TOP) / 2.0)),
    )
    model.articulation(
        "bottle_to_collar",
        ArticulationType.FIXED,
        parent=bottle,
        child=collar,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # ---- head_carrier: massless swivel link seated on the collar ----
    carrier = model.part("head_carrier")
    hub = CylinderGeometry(COLLAR_R, 0.004, radial_segments=24)
    hub.translate(0.0, 0.0, COLLAR_TOP + 0.002)
    carrier.visual(mesh_from_geometry(hub, "carrier_hub"), material=white, name="carrier_hub")
    carrier.inertial = Inertial.from_geometry(
        Cylinder(COLLAR_R, 0.004), mass=0.001, origin=Origin(xyz=(0.0, 0.0, COLLAR_TOP + 0.002))
    )
    # REVOLUTE swivel about +Z: head/spout rotates to aim/lock
    model.articulation(
        "pump_swivel",
        ArticulationType.REVOLUTE,
        parent=collar,
        child=carrier,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.0, lower=-math.pi, upper=math.pi),
    )

    # ---- foamer_head: tall chamber + actuator + stubby spout + stem + dip tube ----
    head = model.part("foamer_head")
    head.visual(_foamer_chamber_mesh(), material=foam_white, name="foamer_chamber")
    head.visual(_chamber_grip_mesh(), material=accent_grey, name="chamber_grip")
    head.visual(_actuator_mesh(), material=foam_white, name="actuator_cap")
    head.visual(_foamer_spout_mesh(), material=white, name="foamer_spout")
    head.visual(_foamer_stem_mesh(), material=white, name="foamer_stem")
    head.visual(_short_dip_tube_mesh(), material=tube_mat, name="dip_tube")
    head.inertial = Inertial.from_geometry(
        Cylinder(FOAMER_R, ACTUATOR_TOP - FOAMER_BOTTOM),
        mass=0.04,
        origin=Origin(xyz=(0.0, 0.0, (FOAMER_BOTTOM + ACTUATOR_TOP) / 2.0)),
    )
    # PRISMATIC press along +Z: pressing down then springing back
    model.articulation(
        "pump_press",
        ArticulationType.PRISMATIC,
        parent=carrier,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.1, lower=-PRESS_TRAVEL, upper=0.0),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    from sdk import TestContext

    ctx = TestContext(object_model)

    bottle = object_model.get_part("bottle")
    collar = object_model.get_part("collar")
    carrier = object_model.get_part("head_carrier")
    head = object_model.get_part("foamer_head")
    swivel = object_model.get_articulation("pump_swivel")
    press = object_model.get_articulation("pump_press")

    # --- bottle is clear (alpha < 1) ---
    shell_vis = bottle.get_visual("bottle_shell")
    rgba = shell_vis.material.rgba
    ctx.check(
        "bottle is transparent",
        rgba is not None and rgba[3] < 1.0,
        details=f"bottle_shell rgba={rgba}",
    )

    # --- collar is seated on the bottle neck (intentional thread overlap) ---
    ctx.allow_overlap(
        collar, bottle, elem_a="collar_shell", elem_b="neck_threads",
        reason="The white collar is threaded over the bottle neck; the threads engage inside the collar.",
    )
    ctx.allow_overlap(
        collar, bottle, elem_a="collar_shell", elem_b="bottle_shell",
        reason="The collar skirt wraps over the bottle neck wall.",
    )
    ctx.expect_overlap(
        collar, bottle, axes="z", min_overlap=0.005, name="collar seated over the neck"
    )

    # --- stem/head ride inside the collar bore (intentional nested fit) ---
    ctx.allow_overlap(
        head, collar, elem_a="foamer_stem", elem_b="collar_shell",
        reason="The foamer stem passes down through the collar bore.",
    )
    ctx.allow_overlap(
        carrier, collar, elem_a="carrier_hub", elem_b="collar_shell",
        reason="The swivel hub sits in the collar throat.",
    )
    ctx.allow_overlap(
        head, carrier, elem_a="foamer_stem", elem_b="carrier_hub",
        reason="The stem passes through the carrier hub it is mounted on.",
    )
    # --- dip tube hangs inside the clear bottle (intentional) ---
    ctx.allow_overlap(
        head, bottle, elem_a="dip_tube", elem_b="bottle_shell",
        reason="The dip tube hangs down inside the bottle.",
    )
    ctx.allow_overlap(
        head, bottle, elem_a="foamer_stem", elem_b="bottle_shell",
        reason="The foamer stem reaches down into the bottle neck.",
    )

    # --- foamer chamber is TALL: rises well above the collar ---
    chamber_aabb = ctx.part_element_world_aabb(head, elem="foamer_chamber")
    ctx.check(
        "foamer chamber rises well above the collar",
        chamber_aabb is not None and chamber_aabb[1][2] > COLLAR_TOP + 0.060,
        details=f"chamber top z={chamber_aabb[1][2] if chamber_aabb else None}, collar top={COLLAR_TOP}",
    )

    # --- foamer chamber is WIDE: wider than the collar ---
    ctx.check(
        "foamer chamber is wider than the collar",
        chamber_aabb is not None and (chamber_aabb[1][0] - chamber_aabb[0][0]) > 2 * COLLAR_R,
        details=f"chamber dx={chamber_aabb[1][0] - chamber_aabb[0][0] if chamber_aabb else None}, collar diameter={2*COLLAR_R}",
    )

    # --- flat actuator cap on top ---
    act_aabb = ctx.part_element_world_aabb(head, elem="actuator_cap")
    ctx.check(
        "actuator cap sits at the top of the foamer head",
        act_aabb is not None and act_aabb[1][2] > FOAMER_TOP,
        details=f"actuator top z={act_aabb[1][2] if act_aabb else None}",
    )

    # --- short stubby spout: minimal droop (no long gooseneck hanging down) ---
    spout_aabb = ctx.part_element_world_aabb(head, elem="foamer_spout")
    spout_tip_x = spout_aabb[1][0] if spout_aabb else 0.0
    spout_droop = (spout_aabb[1][2] - spout_aabb[0][2]) if spout_aabb else 999.0
    ctx.check(
        "foamer spout has minimal droop (no gooseneck)",
        spout_aabb is not None and spout_droop < 0.020,
        details=f"spout z range={spout_droop:.4f}m (parent gooseneck droops ~0.025m)",
    )
    # Spout should still extend off-axis for swivel visibility
    ctx.check(
        "spout extends off-axis (+X) for swivel visibility",
        spout_aabb is not None and spout_tip_x > 0.030,
        details=f"spout tip x={spout_tip_x}",
    )

    # --- dip tube is SHORTER than parent: does not reach near the bottle bottom ---
    tube_aabb = ctx.part_element_world_aabb(head, elem="dip_tube")
    ctx.check(
        "dip tube is shorter — does not reach near bottle bottom",
        tube_aabb is not None and tube_aabb[0][2] > 0.030,
        details=f"dip_tube min z={tube_aabb[0][2] if tube_aabb else None}",
    )

    # --- PRISMATIC press: head moves straight down when pressed ---
    rest = ctx.part_world_position(head)
    with ctx.pose({press: -PRESS_TRAVEL}):
        pressed = ctx.part_world_position(head)
    ctx.check(
        "foamer head presses straight down",
        rest is not None and pressed is not None and pressed[2] < rest[2] - 0.010,
        details=f"rest_z={rest[2] if rest else None}, pressed_z={pressed[2] if pressed else None}",
    )

    # --- REVOLUTE swivel: spout heading changes about +Z ---
    ext0 = _ext(ctx.part_world_aabb(head))
    with ctx.pose({swivel: math.pi / 2.0}):
        ext90 = _ext(ctx.part_world_aabb(head))
    ctx.check(
        "spout points along +X at rest",
        ext0[0] > ext0[1] + 0.008,
        details=f"rest extents={ext0}",
    )
    ctx.check(
        "swivel rotates the spout heading about the vertical axis",
        ext90[1] > ext90[0] + 0.008,
        details=f"quarter-turn extents={ext90}",
    )

    # --- carrier/head are mounted on the collar, not floating ---
    ctx.expect_contact(carrier, collar, name="swivel carrier seated on collar")

    return ctx.report()


object_model = build_object_model()
