from __future__ import annotations

"""Cordless electric miter/chop saw with a teal motor housing on a pivoting
drop-cut arm over a stationary flat base.

Canonical frame: +X points forward (toward the front of the base / operator
side), +Z is up, +Y is the left side (motor-cap side). Real-world scale:
about 0.38 × 0.30 m base, 140 mm blade.

Structural layers:
- base (root): stamped-steel base plate, cast turntable, two-piece fence
  with a blade kerf gap, hinge pillars, hinge pin, and rubber feet.
- arm: cast-aluminum pivoting arm carrying the entire cutting head.
- cutting head (visuals on arm): teal housing, motor barrel, upper guard,
  gearbox nose, top handle, battery pack, trigger ears, etc.
- blade, lower_guard, trigger, safety_lock: jointed children of arm.

Articulated mechanisms:
- arm_hinge: REVOLUTE — the arm swings the cutting head down onto the
  workpiece (positive q = head descends).
- blade_spin: CONTINUOUS — toothed circular blade spins about the arbor.
- lower_guard_retract: REVOLUTE — spring-loaded lower guard swings back.
- trigger_squeeze: REVOLUTE — squeeze trigger in the top handle.
- safety_lock_press: PRISMATIC — side safety lock-off button.
"""

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)

HALF_PI = math.pi / 2.0

# ── Blade geometry constants (shared by build + tests) ──────────────────
BLADE_RADIUS = 0.076
BLADE_CX = 0.095   # arbor X in the parent body frame
BLADE_CZ = 0.070   # arbor Z in the parent body frame
BLADE_Y = -0.078   # blade plane on the right side of the housing
GUARD_Y = -0.058   # fixed guard sits inboard of the blade

# ── Miter-saw arm hinge location (in base / root frame) ─────────────────
HINGE_X = -0.12
HINGE_Z = 0.28

# ── Arm-local offsets for every cutting-head visual ──────────────────────
# Parent body-local (ox, oy, oz) → arm-local (ox + ARM_DX, oy, oz + ARM_DZ).
# MOUNT_RISE lifts the cutting head so the blade clears the base at rest.
MOUNT_RISE = 0.08
ARM_DX = -HINGE_X               #  0.12
ARM_DZ = -HINGE_Z + MOUNT_RISE  # -0.20


# ── Geometry helpers ─────────────────────────────────────────────────────

def _side_extrusion(profile, width: float, name: str):
    """Extrude an XZ side-silhouette profile along the Y (width) axis."""
    geom = ExtrudeGeometry(profile, width, cap=True, center=True)
    geom.rotate_x(HALF_PI)
    return mesh_from_geometry(geom, name)


def _side_loop(outer, hole, width: float, name: str):
    """Extrude an XZ loop profile with a through opening along Y."""
    geom = ExtrudeWithHolesGeometry(outer, [hole], width, cap=True, center=True)
    geom.rotate_x(HALF_PI)
    return mesh_from_geometry(geom, name)


def _blade_teeth_profile(inner_r: float, outer_r: float, n: int):
    """Closed polygon profile of a toothed disc in the XZ plane (local)."""
    pts = []
    for i in range(n):
        a0 = (i / n) * 2.0 * math.pi
        a1 = ((i + 0.5) / n) * 2.0 * math.pi
        pts.append((outer_r * math.cos(a0), outer_r * math.sin(a0)))
        pts.append((inner_r * math.cos(a1), inner_r * math.sin(a1)))
    return pts


# ── Model ────────────────────────────────────────────────────────────────

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cordless_miter_saw")

    # ── Materials ────────────────────────────────────────────────────────
    teal = model.material("teal_housing", rgba=(0.10, 0.52, 0.55, 1.0))
    teal_dark = model.material("teal_dark", rgba=(0.07, 0.40, 0.43, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.09, 0.09, 0.10, 1.0))
    charcoal = model.material("charcoal_plastic", rgba=(0.17, 0.18, 0.19, 1.0))
    chrome = model.material("chrome_steel", rgba=(0.82, 0.83, 0.85, 1.0))
    guard_steel = model.material("guard_steel", rgba=(0.74, 0.75, 0.77, 1.0))
    blade_steel = model.material("blade_steel", rgba=(0.66, 0.67, 0.70, 1.0))
    red_accent = model.material("red_accent", rgba=(0.72, 0.14, 0.12, 1.0))
    cast_alu = model.material("cast_aluminum", rgba=(0.62, 0.63, 0.65, 1.0))
    rubber = model.material("rubber", rgba=(0.10, 0.10, 0.11, 1.0))

    # =====================================================================
    # BASE (root) — stationary flat base with fence and hinge pillars
    # =====================================================================
    base = model.part("base")

    # Stamped-steel base plate.
    base.visual(
        Box((0.38, 0.28, 0.008)),
        origin=Origin(xyz=(0.04, 0.0, 0.004)),
        material=charcoal,
        name="base_plate",
    )

    # Cast-aluminum turntable disk on top of the base plate.
    base.visual(
        Cylinder(radius=0.11, length=0.005),
        origin=Origin(xyz=(0.04, 0.0, 0.0105)),
        material=cast_alu,
        name="turntable_disk",
    )

    # Blade kerf slot (thin dark groove in the turntable where the blade
    # passes through on a down-cut).
    base.visual(
        Box((0.16, 0.004, 0.002)),
        origin=Origin(xyz=(0.04, BLADE_Y, 0.014)),
        material=black_plastic,
        name="blade_slot",
    )

    # Two-piece fence with a gap for the blade kerf.
    # Fence left (operator side of the blade).
    base.visual(
        Box((0.018, 0.17, 0.055)),
        origin=Origin(xyz=(0.02, 0.045, 0.040)),
        material=chrome,
        name="fence_left",
    )
    # Fence right (motor side of the blade).
    base.visual(
        Box((0.018, 0.05, 0.055)),
        origin=Origin(xyz=(0.02, -0.115, 0.040)),
        material=chrome,
        name="fence_right",
    )

    # Hinge pillars — two vertical posts rising from the rear of the base
    # to carry the arm pivot pin.
    base.visual(
        Box((0.035, 0.030, 0.28)),
        origin=Origin(xyz=(HINGE_X, 0.065, 0.14)),
        material=charcoal,
        name="hinge_pillar_left",
    )
    base.visual(
        Box((0.035, 0.030, 0.28)),
        origin=Origin(xyz=(HINGE_X, -0.065, 0.14)),
        material=charcoal,
        name="hinge_pillar_right",
    )

    # Hinge pin passing through both pillars and the arm barrel.
    base.visual(
        Cylinder(radius=0.008, length=0.14),
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z), rpy=(HALF_PI, 0.0, 0.0)),
        material=chrome,
        name="hinge_pin",
    )

    # Rubber feet at the four corners of the base plate.
    foot_positions = [
        (0.04 + 0.15, 0.11, -0.003),
        (0.04 + 0.15, -0.11, -0.003),
        (0.04 - 0.15, 0.11, -0.003),
        (0.04 - 0.15, -0.11, -0.003),
    ]
    for i, (fx, fy, fz) in enumerate(foot_positions):
        base.visual(
            Cylinder(radius=0.012, length=0.006),
            origin=Origin(xyz=(fx, fy, fz)),
            material=rubber,
            name=f"rubber_foot_{i}",
        )

    # =====================================================================
    # ARM — pivoting drop-cut arm carrying the entire cutting head
    # =====================================================================
    arm = model.part("arm")

    # Hinge barrel wrapping around the base hinge pin.
    arm.visual(
        Cylinder(radius=0.018, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(HALF_PI, 0.0, 0.0)),
        material=charcoal,
        name="arm_hinge_barrel",
    )

    # Rear riser bridging the hinge barrel up to the elevated beam
    # (shifted forward to clear the hinge pin).
    arm.visual(
        Box((0.040, 0.048, 0.040)),
        origin=Origin(xyz=(0.03, 0.0, 0.017)),
        material=cast_alu,
        name="arm_rear_riser",
    )

    # Main horizontal arm beam (elevated above the handle area).
    arm.visual(
        Box((0.26, 0.070, 0.038)),
        origin=Origin(xyz=(0.14, 0.0, 0.035)),
        material=cast_alu,
        name="arm_beam",
    )

    # Two side mounting plates connecting the beam down to the cutting head,
    # leaving the centre clear for the handle, trigger, and safety lock.
    arm.visual(
        Box((0.045, 0.010, 0.080)),
        origin=Origin(xyz=(0.17, 0.030, -0.002)),
        material=cast_alu,
        name="arm_side_plate_left",
    )
    arm.visual(
        Box((0.045, 0.010, 0.080)),
        origin=Origin(xyz=(0.17, -0.030, -0.002)),
        material=cast_alu,
        name="arm_side_plate_right",
    )

    # ── Cutting-head visuals (parent body geometry, shifted to arm frame) ─

    # Main motor housing block (rounded side silhouette, extruded across Y).
    arm.visual(
        _side_extrusion(rounded_rect_profile(0.17, 0.11, 0.035), 0.075, "housing_block"),
        origin=Origin(xyz=(0.03 + ARM_DX, 0.0, 0.105 + ARM_DZ)),
        material=teal,
        name="housing_shell",
    )

    # Cross-mounted motor barrel on the left side.
    arm.visual(
        Cylinder(radius=0.046, length=0.070),
        origin=Origin(xyz=(0.045 + ARM_DX, 0.055, 0.115 + ARM_DZ),
                      rpy=(HALF_PI, 0.0, 0.0)),
        material=teal,
        name="motor_barrel",
    )
    arm.visual(
        Cylinder(radius=0.036, length=0.012),
        origin=Origin(xyz=(0.045 + ARM_DX, 0.093, 0.115 + ARM_DZ),
                      rpy=(HALF_PI, 0.0, 0.0)),
        material=black_plastic,
        name="motor_end_cap",
    )

    # Upper fixed blade guard: inboard half-disc shroud over the blade top.
    ug_ang = [math.pi * (0.03 + 0.94 * i / 36) for i in range(37)]
    arm.visual(
        _side_extrusion(
            [((BLADE_RADIUS + 0.016) * math.cos(a) + BLADE_CX,
              (BLADE_RADIUS + 0.016) * math.sin(a) + BLADE_CZ)
             for a in ug_ang]
            + [(BLADE_RADIUS * math.cos(a) + BLADE_CX,
                BLADE_RADIUS * math.sin(a) + BLADE_CZ)
               for a in reversed(ug_ang)],
            0.020,
            "upper_guard_ring",
        ),
        origin=Origin(xyz=(0.0 + ARM_DX, GUARD_Y, 0.0 + ARM_DZ)),
        material=teal_dark,
        name="upper_guard",
    )
    # Solid top cap kept inboard so the lower/front blade teeth remain visible.
    arm.visual(
        _side_extrusion(
            [
                ((BLADE_RADIUS + 0.014) * math.cos(a) + BLADE_CX,
                 (BLADE_RADIUS + 0.014) * math.sin(a) + BLADE_CZ)
                for a in [math.pi * (0.05 + 0.9 * i / 24) for i in range(25)]
            ]
            + [(BLADE_CX - (BLADE_RADIUS + 0.014), BLADE_CZ),
               (BLADE_CX + (BLADE_RADIUS + 0.014), BLADE_CZ)],
            0.030,
            "upper_guard_web",
        ),
        origin=Origin(xyz=(0.0 + ARM_DX, GUARD_Y + 0.003, 0.0 + ARM_DZ)),
        material=teal_dark,
        name="upper_guard_web",
    )

    # Arbor boss sticking out to carry the blade.
    arm.visual(
        Cylinder(radius=0.014, length=0.026),
        origin=Origin(xyz=(BLADE_CX + ARM_DX, BLADE_Y + 0.020, BLADE_CZ + ARM_DZ),
                      rpy=(HALF_PI, 0.0, 0.0)),
        material=charcoal,
        name="arbor_boss",
    )

    # Gearbox nose bridging the housing out to the arbor and guard cluster.
    arm.visual(
        Box((0.060, 0.035, 0.080)),
        origin=Origin(xyz=(0.128 + ARM_DX, -0.030, 0.075 + ARM_DZ)),
        material=teal,
        name="gearbox_nose",
    )

    # Rear-mounted battery pack.
    arm.visual(
        Box((0.075, 0.090, 0.070)),
        origin=Origin(xyz=(-0.060 + ARM_DX, 0.0, 0.075 + ARM_DZ)),
        material=black_plastic,
        name="battery_pack",
    )
    arm.visual(
        Box((0.060, 0.070, 0.014)),
        origin=Origin(xyz=(-0.030 + ARM_DX, 0.0, 0.075 + ARM_DZ)),
        material=charcoal,
        name="battery_terminal_shroud",
    )

    # Top handle: black rubber-overmolded arch with a through opening.
    arm.visual(
        _side_loop(
            [(x - 0.005, z + 0.199) for x, z in rounded_rect_profile(0.150, 0.090, 0.028)],
            [(x - 0.005, z + 0.194) for x, z in rounded_rect_profile(0.090, 0.045, 0.020)],
            0.032,
            "top_handle",
        ),
        origin=Origin(xyz=(ARM_DX, 0.0, ARM_DZ)),
        material=black_plastic,
        name="top_handle",
    )

    # Front grip knob on the housing top-front.
    arm.visual(
        Cylinder(radius=0.024, length=0.055),
        origin=Origin(xyz=(0.105 + ARM_DX, 0.0, 0.150 + ARM_DZ),
                      rpy=(HALF_PI, 0.0, 0.0)),
        material=black_plastic,
        name="front_grip",
    )

    # Trigger pivot ears hanging under the handle front strut.
    arm.visual(
        Box((0.014, 0.010, 0.020)),
        origin=Origin(xyz=(0.075 + ARM_DX, 0.020, 0.150 + ARM_DZ)),
        material=charcoal,
        name="trigger_ear_left",
    )
    arm.visual(
        Box((0.014, 0.010, 0.020)),
        origin=Origin(xyz=(0.075 + ARM_DX, -0.020, 0.150 + ARM_DZ)),
        material=charcoal,
        name="trigger_ear_right",
    )

    # Lower guard pivot boss (coaxial with arbor, front side).
    arm.visual(
        Cylinder(radius=0.010, length=0.038),
        origin=Origin(xyz=(BLADE_CX + ARM_DX, -0.076, BLADE_CZ + ARM_DZ),
                      rpy=(HALF_PI, 0.0, 0.0)),
        material=charcoal,
        name="lower_guard_pivot_boss",
    )

    # Power/label plate accent on the housing side.
    arm.visual(
        Box((0.070, 0.004, 0.030)),
        origin=Origin(xyz=(0.03 + ARM_DX, 0.075, 0.130 + ARM_DZ)),
        material=red_accent,
        name="brand_plate",
    )

    # Short rubber bumper cord stub at the rear (routed below the hinge
    # pillars to avoid interference).
    cord_geom = tube_from_spline_points(
        [
            (-0.095 + ARM_DX, 0.025, 0.075 + ARM_DZ),
            (-0.110 + ARM_DX, 0.035, 0.070 + ARM_DZ),
            (-0.120 + ARM_DX, 0.040, 0.060 + ARM_DZ),
        ],
        radius=0.006,
        samples_per_segment=8,
    )
    arm.visual(
        mesh_from_geometry(cord_geom, "vent_stub"),
        material=charcoal,
        name="vent_stub",
    )

    # =====================================================================
    # BLADE — toothed circular saw blade (child of arm)
    # =====================================================================
    blade = model.part("blade")
    blade.visual(
        _side_extrusion(
            _blade_teeth_profile(BLADE_RADIUS - 0.007, BLADE_RADIUS, 48),
            0.0026,
            "blade_disc",
        ),
        material=blade_steel,
        name="blade_disc",
    )
    blade.visual(
        Cylinder(radius=BLADE_RADIUS - 0.005, length=0.0016),
        origin=Origin(rpy=(HALF_PI, 0.0, 0.0)),
        material=chrome,
        name="blade_plate",
    )
    blade.visual(
        Cylinder(radius=BLADE_RADIUS * 0.36, length=0.0032),
        origin=Origin(rpy=(HALF_PI, 0.0, 0.0)),
        material=blade_steel,
        name="blade_inner_ring",
    )
    blade.visual(
        Cylinder(radius=0.014, length=0.010),
        origin=Origin(rpy=(HALF_PI, 0.0, 0.0)),
        material=charcoal,
        name="arbor_bolt",
    )

    model.articulation(
        "blade_spin",
        ArticulationType.CONTINUOUS,
        parent=arm,
        child=blade,
        origin=Origin(xyz=(BLADE_CX + ARM_DX, BLADE_Y, BLADE_CZ + ARM_DZ)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=350.0),
    )

    # =====================================================================
    # LOWER GUARD — spring-loaded retractable shroud (child of arm)
    # =====================================================================
    lower_guard = model.part("lower_guard")
    lg_ang = [math.pi * (1.02 + 0.62 * i / 24) for i in range(25)]
    lower_guard.visual(
        _side_extrusion(
            [((BLADE_RADIUS + 0.010) * math.cos(a), (BLADE_RADIUS + 0.010) * math.sin(a))
             for a in lg_ang]
            + [((BLADE_RADIUS - 0.004) * math.cos(a), (BLADE_RADIUS - 0.004) * math.sin(a))
               for a in reversed(lg_ang)],
            0.024,
            "lower_guard_shell",
        ),
        material=guard_steel,
        name="lower_guard_shell",
    )
    lower_guard.visual(
        Box((0.030, 0.010, 0.014)),
        origin=Origin(xyz=(-(BLADE_RADIUS + 0.004), 0.0, 0.0)),
        material=black_plastic,
        name="lower_guard_tab",
    )

    model.articulation(
        "lower_guard_retract",
        ArticulationType.REVOLUTE,
        parent=arm,
        child=lower_guard,
        origin=Origin(xyz=(BLADE_CX + ARM_DX, BLADE_Y - 0.011, BLADE_CZ + ARM_DZ)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=6.0, lower=0.0, upper=1.8),
    )

    # =====================================================================
    # TRIGGER — squeeze trigger in the top handle (child of arm)
    # =====================================================================
    trigger = model.part("trigger")
    trigger.visual(
        Box((0.014, 0.030, 0.014)),
        origin=Origin(xyz=(0.002, 0.0, -0.004)),
        material=black_plastic,
        name="trigger_boss",
    )
    trigger.visual(
        Box((0.014, 0.034, 0.050)),
        origin=Origin(xyz=(-0.010, 0.0, -0.030)),
        material=black_plastic,
        name="trigger_blade",
    )

    model.articulation(
        "trigger_squeeze",
        ArticulationType.REVOLUTE,
        parent=arm,
        child=trigger,
        origin=Origin(xyz=(0.075 + ARM_DX, 0.0, 0.150 + ARM_DZ)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=6.0, lower=0.0, upper=0.40),
    )

    # =====================================================================
    # SAFETY LOCK — lock-off button on the handle (child of arm)
    # =====================================================================
    safety_lock = model.part("safety_lock")
    safety_lock.visual(
        Cylinder(radius=0.009, length=0.014),
        origin=Origin(rpy=(HALF_PI, 0.0, 0.0)),
        material=red_accent,
        name="safety_lock_cap",
    )

    model.articulation(
        "safety_lock_press",
        ArticulationType.PRISMATIC,
        parent=arm,
        child=safety_lock,
        origin=Origin(xyz=(0.055 + ARM_DX, 0.014, 0.190 + ARM_DZ)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=0.1, lower=0.0, upper=0.005),
    )

    # =====================================================================
    # ARM HINGE — the primary structural joint: base → arm
    # =====================================================================
    model.articulation(
        "arm_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=arm,
        # Hinge pin location at the top of the pillars, rear of the base.
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        # Positive q swings the front of the arm downward (cutting stroke).
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.0, lower=0.0, upper=0.55),
    )

    return model


# ── Tests ────────────────────────────────────────────────────────────────

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    arm = object_model.get_part("arm")
    blade = object_model.get_part("blade")
    lower_guard = object_model.get_part("lower_guard")
    trigger = object_model.get_part("trigger")
    safety_lock = object_model.get_part("safety_lock")

    arm_hinge = object_model.get_articulation("arm_hinge")
    blade_joint = object_model.get_articulation("blade_spin")
    guard_joint = object_model.get_articulation("lower_guard_retract")
    trigger_joint = object_model.get_articulation("trigger_squeeze")
    lock_joint = object_model.get_articulation("safety_lock_press")

    # ── Intentional overlap allowances ───────────────────────────────────

    # Arm hinge barrel captures the base hinge pin.
    ctx.allow_overlap(
        base, arm,
        elem_a="hinge_pin", elem_b="arm_hinge_barrel",
        reason="Arm hinge barrel wraps around the base hinge pin (captured pin hinge).",
    )
    ctx.expect_contact(
        base, arm,
        elem_a="hinge_pin", elem_b="arm_hinge_barrel",
        contact_tol=0.012,
        name="hinge pin seats inside the arm barrel",
    )

    # Blade arbor stack overlaps (same as parent, arm replaces body).
    ctx.allow_overlap(
        arm, blade,
        elem_a="arbor_boss", elem_b="arbor_bolt",
        reason="Blade arbor bolt seats onto the arm arbor boss.",
    )
    ctx.allow_overlap(
        blade, arm,
        elem_a="arbor_bolt", elem_b="lower_guard_pivot_boss",
        reason="Blade arbor bolt and lower-guard pivot boss share the same arbor stack.",
    )
    ctx.allow_overlap(
        arm, blade,
        elem_a="upper_guard", elem_b="blade_disc",
        reason="Upper fixed guard shrouds the top rim of the blade.",
    )
    ctx.allow_overlap(
        arm, blade,
        elem_a="upper_guard_web", elem_b="blade_plate",
        reason="Upper guard web overlaps the blade body it covers.",
    )
    ctx.allow_overlap(
        arm, lower_guard,
        elem_a="lower_guard_pivot_boss", elem_b="lower_guard_shell",
        reason="Lower guard shell pivots on the arm pivot boss.",
    )
    ctx.allow_overlap(
        arm, lower_guard,
        elem_a="upper_guard", elem_b="lower_guard_shell",
        reason="Lower guard nests concentrically inside the fixed upper guard band.",
    )
    ctx.allow_overlap(
        arm, lower_guard,
        elem_a="upper_guard_web", elem_b="lower_guard_shell",
        reason="Lower guard tucks under the fixed upper guard web when closed.",
    )
    ctx.allow_overlap(
        arm, lower_guard,
        elem_a="upper_guard", elem_b="lower_guard_tab",
        reason="Retract tab passes under the fixed upper guard band.",
    )
    ctx.allow_overlap(
        arm, lower_guard,
        elem_a="upper_guard_web", elem_b="lower_guard_tab",
        reason="Retract tab tucks under the fixed upper guard web when closed.",
    )
    ctx.allow_overlap(
        blade, lower_guard,
        elem_a="blade_disc", elem_b="lower_guard_shell",
        reason="Lower guard rides just outside the blade rim, thin clearance.",
    )
    ctx.allow_overlap(
        arm, trigger,
        elem_a="trigger_ear_left", elem_b="trigger_boss",
        reason="Trigger pivot boss is captured between the handle ears.",
    )
    ctx.allow_overlap(
        arm, trigger,
        elem_a="trigger_ear_right", elem_b="trigger_boss",
        reason="Trigger pivot boss is captured between the handle ears.",
    )
    ctx.allow_overlap(
        arm, trigger,
        elem_a="top_handle", elem_b="trigger_blade",
        reason="Trigger blade hangs into the handle opening from the pivot.",
    )
    ctx.allow_overlap(
        blade, arm,
        elem_a="arbor_bolt", elem_b="upper_guard",
        reason="Blade hub sits inside the fixed upper guard shroud.",
    )
    ctx.allow_overlap(
        blade, arm,
        elem_a="arbor_bolt", elem_b="upper_guard_web",
        reason="Blade hub is covered by the upper guard web.",
    )
    ctx.allow_overlap(
        blade, arm,
        elem_a="blade_disc", elem_b="upper_guard_web",
        reason="Upper half of the blade is shrouded by the fixed guard web.",
    )
    ctx.allow_overlap(
        blade, arm,
        elem_a="blade_plate", elem_b="upper_guard_web",
        reason="Blade body plate upper half sits under the guard web.",
    )
    ctx.allow_overlap(
        arm, safety_lock,
        elem_a="top_handle", elem_b="safety_lock_cap",
        reason="Safety lock-off button stem embeds into the handle front strut.",
    )

    # ── Prompt-level shape: arm_hinge swings the head down ──────────────

    # At rest the blade is well above the base plate.
    rest_aabb = ctx.part_world_aabb(blade)
    ctx.check(
        "blade clears the base plate at arm rest",
        rest_aabb is not None and rest_aabb[0][2] > 0.04,
        details=f"blade_min_z={rest_aabb[0][2] if rest_aabb else None}",
    )

    # With the arm swung down, the blade reaches the base plate level.
    with ctx.pose({arm_hinge: 0.50}):
        down_aabb = ctx.part_world_aabb(blade)
    ctx.check(
        "arm_hinge swings blade down toward the base plate",
        rest_aabb is not None and down_aabb is not None
        and down_aabb[0][2] < rest_aabb[0][2] - 0.05,
        details=f"rest_min_z={rest_aabb[0][2] if rest_aabb else None}, "
                f"down_min_z={down_aabb[0][2] if down_aabb else None}",
    )

    # Base has fence geometry rising above the plate.
    base_aabb = ctx.part_world_aabb(base)
    ctx.check(
        "base has fence rising above the plate",
        base_aabb is not None and base_aabb[1][2] > 0.05,
        details=f"base_max_z={base_aabb[1][2] if base_aabb else None}",
    )

    # Blade is a large disc.
    ctx.check(
        "blade is a large disc",
        rest_aabb is not None and (rest_aabb[1][0] - rest_aabb[0][0]) > 0.12,
        details=f"blade_dx={rest_aabb[1][0] - rest_aabb[0][0] if rest_aabb else None}",
    )

    # Blade seats on the arbor boss.
    ctx.expect_contact(
        arm, blade,
        elem_a="arbor_boss", elem_b="arbor_bolt",
        contact_tol=4e-3,
        name="blade seats on the arbor boss",
    )

    # Top handle rises well above the housing.
    arm_aabb = ctx.part_world_aabb(arm)
    ctx.check(
        "top handle rises well above the arm beam",
        arm_aabb is not None and arm_aabb[1][2] > 0.30,
        details=f"arm_max_z={arm_aabb[1][2] if arm_aabb else None}",
    )

    # ── Decisive poses ───────────────────────────────────────────────────

    # Blade spins.
    p_rest = ctx.part_world_aabb(blade)
    with ctx.pose({blade_joint: 0.8}):
        p_spun = ctx.part_world_aabb(blade)
    ctx.check(
        "blade rotates about the arbor",
        p_rest is not None and p_spun is not None,
        details=f"rest={p_rest}, spun={p_spun}",
    )

    # Lower guard retracts.
    g_rest = ctx.part_world_aabb(lower_guard)
    with ctx.pose({guard_joint: 1.6}):
        g_open = ctx.part_world_aabb(lower_guard)
    ctx.check(
        "lower guard retracts to expose the blade",
        g_rest is not None and g_open is not None
        and abs(g_open[0][0] - g_rest[0][0]) + abs(g_open[1][2] - g_rest[1][2]) > 0.02,
        details=f"rest={g_rest}, open={g_open}",
    )

    # Trigger squeezes.
    t_rest = ctx.part_world_aabb(trigger)
    with ctx.pose({trigger_joint: 0.40}):
        t_squeezed = ctx.part_world_aabb(trigger)
    ctx.check(
        "squeezing swings the trigger rearward",
        t_rest is not None and t_squeezed is not None
        and t_squeezed[0][0] < t_rest[0][0] - 0.008,
        details=f"rest={t_rest}, squeezed={t_squeezed}",
    )

    # Safety lock presses inward.
    b_rest = ctx.part_world_position(safety_lock)
    with ctx.pose({lock_joint: 0.005}):
        b_pressed = ctx.part_world_position(safety_lock)
    ctx.check(
        "safety lock-off button presses inward",
        b_rest is not None and b_pressed is not None
        and b_pressed[1] < b_rest[1] - 0.003,
        details=f"rest={b_rest}, pressed={b_pressed}",
    )

    return ctx.report()


object_model = build_object_model()
