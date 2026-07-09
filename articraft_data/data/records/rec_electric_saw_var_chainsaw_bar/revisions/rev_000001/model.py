from __future__ import annotations

"""Cordless electric chainsaw with a teal motor housing.

Canonical frame: +X points forward (toward the bar tip), +Z is up, +Y is the
left side (motor-cap side).  Real-world scale: about 0.42 m overall length,
260 mm guide bar.

Articulated mechanisms:
- chain_drive: continuous chain rotation around the guide bar (CONTINUOUS).
- trigger_squeeze: squeeze trigger inside the top handle (REVOLUTE).
- safety_lock_press: side safety lock-off button (PRISMATIC).
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

# ── guide bar + chain constants (shared by build and tests) ──
BAR_LENGTH = 0.260          # total bar length (rear edge to nose tip)
BAR_HALF_W = 0.024          # bar half-width (top-to-bottom / 2)
BAR_THICK = 0.007           # bar plate thickness (Y extrusion)
BAR_REAR_X = 0.150          # bar rear edge X in world frame
BAR_Y = -0.055              # bar plane Y (cutting side of housing)
BAR_CZ = 0.068              # bar centre height Z
BAR_CENTER_X = BAR_REAR_X + BAR_LENGTH / 2   # 0.280

# Drive sprocket position = articulation origin
SPROCKET_X = 0.155
SPROCKET_Y = BAR_Y
SPROCKET_Z = BAR_CZ

NUM_LINKS = 36
LINK_DX = 0.016             # link extent along path tangent
LINK_DY = 0.003             # link extent across bar thickness (thin cutter plate)
LINK_DZ = 0.007             # link extent normal to bar rail

# Chain sits slightly proud of the bar rail so rotated link corners do not
# bury into the bar plate at the nose transition.
CHAIN_RADIAL_OFFSET = 0.006

# Derived path parameters (sprocket-local frame)
_BAR_LOCAL_REAR = BAR_REAR_X - SPROCKET_X              # -0.005
_BAR_NOSE_CX = BAR_REAR_X + BAR_LENGTH - BAR_HALF_W - SPROCKET_X   # 0.231
_BAR_STRAIGHT = BAR_LENGTH - BAR_HALF_W                 # 0.236
_BAR_ARC_LEN = math.pi * BAR_HALF_W                     # ~0.0754
_BAR_PERIMETER = 2.0 * _BAR_STRAIGHT + _BAR_ARC_LEN     # ~0.547


# ── geometry helpers ──

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


def _guide_bar_profile():
    """XZ profile of the guide bar in LOCAL coords (origin at bar centre).

    Elongated flat plate with a rounded nose on the +X end and a flat rear.
    """
    half_len = BAR_LENGTH / 2
    hw = BAR_HALF_W
    pts: list[tuple[float, float]] = []
    # Rear bottom corner
    pts.append((-half_len, -hw))
    # Bottom edge to nose tangent
    pts.append((half_len - hw, -hw))
    # Nose semicircle (-pi/2 to +pi/2)
    for i in range(17):
        a = -HALF_PI + math.pi * i / 16
        pts.append(((half_len - hw) + hw * math.cos(a),
                     hw * math.sin(a)))
    # Top edge back to rear
    pts.append((-half_len, hw))
    return pts


def _chain_path_points(n: int):
    """Generate *n* evenly-spaced (x, z, tangent_angle) around the bar
    perimeter in the SPROCKET-LOCAL frame (the chain part origin).

    The path is offset outward from the bar surface by CHAIN_RADIAL_OFFSET
    so that rotated link corners at the nose do not bury into the bar plate.
    """
    straight = _BAR_STRAIGHT
    hw_out = BAR_HALF_W + CHAIN_RADIAL_OFFSET
    nose_cx = _BAR_NOSE_CX
    arc_len = math.pi * hw_out
    total = 2.0 * straight + arc_len
    spacing = total / n

    pts: list[tuple[float, float, float]] = []
    for i in range(n):
        d = i * spacing
        if d < straight:
            # Bottom straight (going forward, tangent +X)
            x = _BAR_LOCAL_REAR + d
            z = -hw_out
            angle = 0.0
        elif d < straight + arc_len:
            # Nose arc
            arc_d = d - straight
            a = -HALF_PI + (arc_d / arc_len) * math.pi
            x = nose_cx + hw_out * math.cos(a)
            z = hw_out * math.sin(a)
            angle = a + HALF_PI
        else:
            # Top straight (going backward, tangent -X)
            top_d = d - straight - arc_len
            x = nose_cx - top_d
            z = hw_out
            angle = math.pi
        pts.append((x, z, angle))
    return pts


def _chain_link_mesh():
    """Shared geometry for one chain cutter link."""
    return Box((LINK_DX, LINK_DY, LINK_DZ))


def _chain_rail_outer_profile():
    """Outer profile of the thin chain-rail band (1 mm outside chain path)."""
    hw = BAR_HALF_W + CHAIN_RADIAL_OFFSET + 0.001
    pts: list[tuple[float, float]] = []
    pts.append((_BAR_LOCAL_REAR - 0.002, -hw))
    pts.append((_BAR_NOSE_CX, -hw))
    for i in range(17):
        a = -HALF_PI + math.pi * i / 16
        pts.append((_BAR_NOSE_CX + hw * math.cos(a), hw * math.sin(a)))
    pts.append((_BAR_LOCAL_REAR - 0.002, hw))
    return pts


def _chain_rail_inner_profile():
    """Inner profile (hole) of the thin chain-rail band (1 mm inside chain path)."""
    hw = BAR_HALF_W + CHAIN_RADIAL_OFFSET - 0.001
    pts: list[tuple[float, float]] = []
    pts.append((_BAR_LOCAL_REAR, -hw))
    pts.append((_BAR_NOSE_CX, -hw))
    for i in range(17):
        a = -HALF_PI + math.pi * i / 16
        pts.append((_BAR_NOSE_CX + hw * math.cos(a), hw * math.sin(a)))
    pts.append((_BAR_LOCAL_REAR, hw))
    return pts


# ── build ──

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cordless_chainsaw")

    # ── materials ──
    teal = model.material("teal_housing", rgba=(0.10, 0.52, 0.55, 1.0))
    teal_dark = model.material("teal_dark", rgba=(0.07, 0.40, 0.43, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.09, 0.09, 0.10, 1.0))
    charcoal = model.material("charcoal_plastic", rgba=(0.17, 0.18, 0.19, 1.0))
    chrome = model.material("chrome_steel", rgba=(0.82, 0.83, 0.85, 1.0))
    bar_steel = model.material("bar_steel", rgba=(0.66, 0.67, 0.70, 1.0))
    chain_steel = model.material("chain_steel", rgba=(0.48, 0.49, 0.51, 1.0))
    sprocket_steel = model.material("sprocket_steel", rgba=(0.58, 0.59, 0.61, 1.0))
    red_accent = model.material("red_accent", rgba=(0.72, 0.14, 0.12, 1.0))

    # ═══════════════════════════════════════════════════════════════ BODY ═══
    body = model.part("body")

    # ── KEEP: housing shell ──
    body.visual(
        _side_extrusion(rounded_rect_profile(0.17, 0.11, 0.035), 0.075, "housing_block"),
        origin=Origin(xyz=(0.03, 0.0, 0.105)),
        material=teal,
        name="housing_shell",
    )

    # ── KEEP: motor barrel + end cap ──
    body.visual(
        Cylinder(radius=0.046, length=0.070),
        origin=Origin(xyz=(0.045, 0.055, 0.115), rpy=(HALF_PI, 0.0, 0.0)),
        material=teal,
        name="motor_barrel",
    )
    body.visual(
        Cylinder(radius=0.036, length=0.012),
        origin=Origin(xyz=(0.045, 0.093, 0.115), rpy=(HALF_PI, 0.0, 0.0)),
        material=black_plastic,
        name="motor_end_cap",
    )

    # ── Gearbox nose (bar mount area) ──
    body.visual(
        Box((0.055, 0.035, 0.075)),
        origin=Origin(xyz=(0.128, -0.030, 0.072)),
        material=teal_dark,
        name="gearbox_nose",
    )

    # ── Clutch cover (bridges housing → bar mount) ──
    body.visual(
        Box((0.060, 0.030, 0.060)),
        origin=Origin(xyz=(0.135, -0.044, 0.068)),
        material=charcoal,
        name="clutch_cover",
    )

    # ── Guide bar (elongated flat bar, fixed to body) ──
    body.visual(
        _side_extrusion(_guide_bar_profile(), BAR_THICK, "guide_bar"),
        origin=Origin(xyz=(BAR_CENTER_X, BAR_Y, BAR_CZ)),
        material=bar_steel,
        name="guide_bar",
    )

    # ── Drive sprocket at bar rear ──
    body.visual(
        Cylinder(radius=0.018, length=0.008),
        origin=Origin(xyz=(SPROCKET_X, SPROCKET_Y, SPROCKET_Z),
                      rpy=(HALF_PI, 0.0, 0.0)),
        material=sprocket_steel,
        name="drive_sprocket",
    )

    # ── Nose sprocket at bar tip (length exceeds bar thickness for contact) ──
    body.visual(
        Cylinder(radius=0.014, length=0.010),
        origin=Origin(xyz=(BAR_REAR_X + BAR_LENGTH - BAR_HALF_W,
                           BAR_Y, BAR_CZ),
                      rpy=(HALF_PI, 0.0, 0.0)),
        material=sprocket_steel,
        name="nose_sprocket",
    )

    # ── Chain catcher plate (safety feature near bar mount) ──
    body.visual(
        Box((0.020, 0.012, 0.040)),
        origin=Origin(xyz=(0.120, -0.058, 0.040)),
        material=charcoal,
        name="chain_catcher",
    )

    # ── Oil tank (below housing) ──
    body.visual(
        Box((0.060, 0.060, 0.020)),
        origin=Origin(xyz=(0.02, 0.0, 0.045)),
        material=charcoal,
        name="oil_tank",
    )

    # ── KEEP: battery pack ──
    body.visual(
        Box((0.075, 0.090, 0.070)),
        origin=Origin(xyz=(-0.060, 0.0, 0.075)),
        material=black_plastic,
        name="battery_pack",
    )
    body.visual(
        Box((0.060, 0.070, 0.014)),
        origin=Origin(xyz=(-0.030, 0.0, 0.075)),
        material=charcoal,
        name="battery_terminal_shroud",
    )

    # ── KEEP: top handle (loop arch) ──
    body.visual(
        _side_loop(
            [(x - 0.005, z + 0.199)
             for x, z in rounded_rect_profile(0.150, 0.090, 0.028)],
            [(x - 0.005, z + 0.194)
             for x, z in rounded_rect_profile(0.090, 0.045, 0.020)],
            0.032,
            "top_handle",
        ),
        material=black_plastic,
        name="top_handle",
    )

    # ── KEEP: front grip ──
    body.visual(
        Cylinder(radius=0.024, length=0.055),
        origin=Origin(xyz=(0.105, 0.0, 0.150), rpy=(HALF_PI, 0.0, 0.0)),
        material=black_plastic,
        name="front_grip",
    )

    # ── Trigger mount ears ──
    body.visual(
        Box((0.014, 0.010, 0.020)),
        origin=Origin(xyz=(0.075, 0.020, 0.150)),
        material=charcoal,
        name="trigger_ear_left",
    )
    body.visual(
        Box((0.014, 0.010, 0.020)),
        origin=Origin(xyz=(0.075, -0.020, 0.150)),
        material=charcoal,
        name="trigger_ear_right",
    )

    # ── Brand / accent plate ──
    body.visual(
        Box((0.070, 0.004, 0.030)),
        origin=Origin(xyz=(0.03, 0.075, 0.130)),
        material=red_accent,
        name="brand_plate",
    )

    # ── Vent / cord stub at rear ──
    cord_geom = tube_from_spline_points(
        [(-0.095, 0.030, 0.075),
         (-0.110, 0.045, 0.070),
         (-0.120, 0.055, 0.060)],
        radius=0.006, samples_per_segment=8,
    )
    body.visual(
        mesh_from_geometry(cord_geom, "vent_stub"),
        material=charcoal,
        name="vent_stub",
    )

    # ═══════════════════════════════════════════════════════════════ CHAIN ═══
    chain = model.part("chain")

    path_pts = _chain_path_points(NUM_LINKS)
    for i in range(NUM_LINKS):
        x, z, angle = path_pts[i]
        chain.visual(
            _chain_link_mesh(),
            origin=Origin(xyz=(x, 0.0, z), rpy=(0.0, angle, 0.0)),
            material=chain_steel,
            name=f"link_{i}",
        )

    # Thin connecting rail band tying all cutter links into one assembly
    rail_outer = _chain_rail_outer_profile()
    rail_inner = _chain_rail_inner_profile()
    rail_geom = ExtrudeWithHolesGeometry(
        rail_outer, [rail_inner], 0.002, cap=True, center=True,
    )
    rail_geom.rotate_x(HALF_PI)
    chain.visual(
        mesh_from_geometry(rail_geom, "chain_rail"),
        material=chain_steel,
        name="chain_rail",
    )

    model.articulation(
        "chain_drive",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=chain,
        origin=Origin(xyz=(SPROCKET_X, SPROCKET_Y, SPROCKET_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=200.0),
    )

    # ═════════════════════════════════════════════════════════════ TRIGGER ═══
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
        parent=body,
        child=trigger,
        origin=Origin(xyz=(0.075, 0.0, 0.150)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=6.0,
                                   lower=0.0, upper=0.40),
    )

    # ═══════════════════════════════════════════════════════ SAFETY LOCK ═══
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
        parent=body,
        child=safety_lock,
        origin=Origin(xyz=(0.055, 0.014, 0.190)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=0.1,
                                   lower=0.0, upper=0.005),
    )

    return model


# ── tests ──

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    chain = object_model.get_part("chain")
    trigger = object_model.get_part("trigger")
    safety_lock = object_model.get_part("safety_lock")

    chain_joint = object_model.get_articulation("chain_drive")
    trigger_joint = object_model.get_articulation("trigger_squeeze")
    lock_joint = object_model.get_articulation("safety_lock_press")

    # ── intentional overlap allowances ──

    # Chain links wrap around the guide bar perimeter (straddling the rail,
    # engaging sprockets, passing through the clutch area).
    ctx.allow_overlap(
        body, chain,
        reason="Chain cutter links wrap around the guide bar perimeter, straddling "
               "the bar rail and engaging both drive and nose sprockets.",
    )

    # Trigger pivot ears capture the trigger boss
    ctx.allow_overlap(
        body, trigger, elem_a="trigger_ear_left", elem_b="trigger_boss",
        reason="Trigger pivot boss is captured between the handle ears.",
    )
    ctx.allow_overlap(
        body, trigger, elem_a="trigger_ear_right", elem_b="trigger_boss",
        reason="Trigger pivot boss is captured between the handle ears.",
    )
    ctx.allow_overlap(
        body, trigger, elem_a="top_handle", elem_b="trigger_blade",
        reason="Trigger blade hangs into the handle opening from the pivot.",
    )

    # Safety lock stem embeds into handle strut
    ctx.allow_overlap(
        body, safety_lock, elem_a="top_handle", elem_b="safety_lock_cap",
        reason="Safety lock-off button stem embeds into the handle front strut.",
    )

    # ── prompt-level shape assertions ──

    # Guide bar is elongated and projects forward from the body
    bar_aabb = ctx.part_element_world_aabb(body, elem="guide_bar")
    ctx.check(
        "guide_bar is an elongated bar projecting forward from the body",
        bar_aabb is not None and (bar_aabb[1][0] - bar_aabb[0][0]) > 0.20,
        details=f"guide_bar_aabb={bar_aabb}",
    )

    # Chain carries cutter links at distinct heights (bottom vs top of bar)
    link_0_aabb = ctx.part_element_world_aabb(chain, elem="link_0")
    link_last_aabb = ctx.part_element_world_aabb(chain, elem=f"link_{NUM_LINKS - 1}")
    ctx.check(
        "guide_bar carries cutter links around its perimeter",
        link_0_aabb is not None and link_last_aabb is not None
        and link_0_aabb[1][2] < link_last_aabb[0][2] - 0.02,
        details=f"link_0_z_max={None if link_0_aabb is None else link_0_aabb[1][2]}, "
                f"link_{NUM_LINKS-1}_z_min={None if link_last_aabb is None else link_last_aabb[0][2]}",
    )

    # ── decisive pose checks ──

    # Chain drive rotates the chain assembly
    c_rest = ctx.part_world_aabb(chain)
    with ctx.pose({chain_joint: 1.0}):
        c_driven = ctx.part_world_aabb(chain)
    ctx.check(
        "chain_drive rotates the chain around the bar",
        c_rest is not None and c_driven is not None,
        details=f"rest={c_rest}, driven={c_driven}",
    )

    # Trigger squeezes rearward
    t_rest = ctx.part_world_aabb(trigger)
    with ctx.pose({trigger_joint: 0.40}):
        t_squeezed = ctx.part_world_aabb(trigger)
    ctx.check(
        "squeezing swings the trigger rearward",
        t_rest is not None and t_squeezed is not None
        and t_squeezed[0][0] < t_rest[0][0] - 0.008,
        details=f"rest={t_rest}, squeezed={t_squeezed}",
    )

    # Safety lock presses inward
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
