from __future__ import annotations

# Orange handheld firefighting hose nozzle (pistol-grip branch nozzle).
#
# Layout (barrel axis along +X, "front" = +X = outlet, "rear" = -X = inlet):
#   - rear: a flared bell inlet coupling (hose connection)
#   - mid: a cast barrel body (valve housing) with a raised valve band
#   - front: a stepped fog/spray outlet diffuser with a rotating pattern collar
#   - underneath: a vertical pistol grip with finger grooves, butt at z=0
#   - on top: a curved bail shut-off lever pivoting about a horizontal cross-pin
#
# Primary articulation: the bail shut-off lever (revolute about the cross-pin,
# horizontal Y axis) that swings from closed (forward/down) to open (raised).
# Secondary: the spray-pattern selector collar rotates about the barrel axis.

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="firehose_nozzle")

    nozzle_orange = model.material("nozzle_orange", rgba=(0.90, 0.36, 0.22, 1.0))
    steel = model.material("steel", rgba=(0.62, 0.63, 0.66, 1.0))
    dark_grip = model.material("dark_grip", rgba=(0.78, 0.30, 0.18, 1.0))

    # ---------- key dimensions (meters) ----------
    barrel_r = 0.034
    barrel_axis_z = 0.150  # height of the barrel centerline above the floor
    rear_x = -0.120        # rear (inlet) end of barrel body
    grip_bottom_z = 0.0    # pistol-grip butt rests on the floor
    barrel_len = 0.205

    # ===================================================================
    # ROOT: nozzle body (barrel + inlet bell + outlet neck + pistol grip + lugs)
    # ===================================================================
    body = model.part("body")

    # --- Barrel body: revolved valve housing about local Z, laid along world X.
    barrel_profile = [
        (0.0, 0.0),
        (barrel_r, 0.0),
        (barrel_r, 0.055),
        (barrel_r + 0.006, 0.075),   # raised mid valve band
        (barrel_r + 0.006, 0.095),
        (barrel_r, 0.115),
        (barrel_r - 0.002, 0.175),
        (barrel_r - 0.006, 0.205),   # taper toward outlet neck
        (0.0, 0.205),
    ]
    barrel_geom = LatheGeometry(barrel_profile, segments=40)
    body.visual(
        mesh_from_geometry(barrel_geom, "barrel"),
        origin=Origin(xyz=(rear_x, 0.0, barrel_axis_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=nozzle_orange,
        name="barrel",
    )

    # --- Rear flared bell inlet coupling (hose connection) ---
    # Throat (local z=0) butts against the barrel rear face at rear_x and the
    # bell flares outward toward -X (local +Z -> world -X via rpy pitch=-90).
    bell_profile = [
        (0.0, 0.0),
        (0.034, 0.0),     # throat meets the barrel rear (radius == barrel_r)
        (0.040, 0.010),
        (0.052, 0.028),   # flare out
        (0.066, 0.048),
        (0.066, 0.054),   # rim lip
        (0.058, 0.054),
        (0.044, 0.030),   # inner wall back toward the bore
        (0.030, 0.010),
        (0.030, 0.0),
        (0.0, 0.0),
    ]
    bell_geom = LatheGeometry(bell_profile, segments=40)
    body.visual(
        mesh_from_geometry(bell_geom, "inlet_bell"),
        origin=Origin(xyz=(rear_x, 0.0, barrel_axis_z), rpy=(0.0, -math.pi / 2.0, 0.0)),
        material=nozzle_orange,
        name="inlet_bell",
    )
    # steel coupling ring at the inlet throat
    inlet_ring = TorusGeometry(0.034, 0.006, radial_segments=18, tubular_segments=30)
    body.visual(
        mesh_from_geometry(inlet_ring, "inlet_ring"),
        origin=Origin(xyz=(rear_x, 0.0, barrel_axis_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="inlet_ring",
    )

    # --- Front outlet neck stub (fixed; the collar mounts/rotates on it) ---
    neck_x = rear_x + barrel_len  # = 0.085
    neck = LatheGeometry(
        [
            (0.0, 0.0),
            (0.028, 0.0),
            (0.028, 0.030),
            (0.026, 0.030),
            (0.0, 0.030),
        ],
        segments=32,
    )
    body.visual(
        mesh_from_geometry(neck, "outlet_neck"),
        origin=Origin(xyz=(neck_x, 0.0, barrel_axis_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="outlet_neck",
    )

    # --- Pistol grip (vertical, butt on the floor) with finger grooves ---
    grip_top_z = barrel_axis_z - barrel_r - 0.004  # tucks up under the barrel
    grip_x = -0.010  # grip sits slightly behind the barrel midpoint
    grip_len = grip_top_z - grip_bottom_z
    grip_profile = rounded_rect_profile(0.030, 0.044, 0.012)
    grip_path = [
        (grip_x + 0.004, 0.0, grip_top_z + 0.010),
        (grip_x, 0.0, grip_top_z - grip_len * 0.4),
        (grip_x - 0.004, 0.0, grip_top_z - grip_len * 0.75),
        (grip_x - 0.006, 0.0, grip_bottom_z + 0.010),
    ]
    grip_geom = sweep_profile_along_spline(
        grip_path, profile=grip_profile, samples_per_segment=10, cap_profile=True
    )
    body.visual(
        mesh_from_geometry(grip_geom, "pistol_grip"),
        material=dark_grip,
        name="pistol_grip",
    )
    # finger-groove ridges on the front of the grip
    for i in range(4):
        gz = grip_top_z - grip_len * (0.28 + 0.17 * i)
        ridge = TorusGeometry(0.020, 0.004, radial_segments=8, tubular_segments=16)
        body.visual(
            mesh_from_geometry(ridge, f"grip_ridge_{i}"),
            origin=Origin(xyz=(grip_x + 0.014, 0.0, gz), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_grip,
            name=f"grip_ridge_{i}",
        )
    # butt cap so the grip reads as resting on the floor
    butt = LatheGeometry(
        [(0.0, 0.0), (0.024, 0.0), (0.026, 0.006), (0.020, 0.012), (0.0, 0.012)],
        segments=24,
    )
    body.visual(
        mesh_from_geometry(butt, "grip_butt"),
        origin=Origin(xyz=(grip_x - 0.006, 0.0, grip_bottom_z)),
        material=dark_grip,
        name="grip_butt",
    )

    # --- two bail pivot lugs on top of the barrel (carry the cross-pin) ---
    pin_x = 0.030          # cross-pin X position (forward of barrel mid)
    barrel_top_z = barrel_axis_z + barrel_r  # ~0.184
    pin_z = barrel_top_z + 0.022
    pin_dy = 0.026         # lug half-spacing along Y
    # Lugs root down into the rounded barrel shoulder at their y-offset so they
    # stay connected to the barrel (the shoulder is lower off the centerline).
    lug_base_z = barrel_axis_z + 0.006
    lug_len = pin_z - lug_base_z
    for s, tag in ((1, "left"), (-1, "right")):
        lug = Cylinder(radius=0.009, length=lug_len)
        body.visual(
            lug,
            origin=Origin(xyz=(pin_x, s * pin_dy, lug_base_z + lug_len / 2.0)),
            material=steel,
            name=f"bail_lug_{tag}",
        )
    # cross-pin spanning and embedding into both lug tops
    pin = Cylinder(radius=0.005, length=2 * pin_dy + 0.022)
    body.visual(
        pin,
        origin=Origin(xyz=(pin_x, 0.0, pin_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="bail_pin",
    )

    body.inertial = Inertial.from_geometry(
        Cylinder(radius=barrel_r, length=barrel_len + 0.10), mass=2.5
    )

    # ===================================================================
    # BAIL SHUT-OFF LEVER (revolute about the cross-pin, horizontal Y)
    # ===================================================================
    bail = model.part("bail_lever")
    # Authored in the joint-local frame (pin at origin). At q=0 the lever lies
    # forward/low (closed); positive q raises it (open). Axis = local Y.
    bail_pts = [
        (-0.004, pin_dy, 0.0),          # at the left lug
        (0.022, pin_dy, 0.026),
        (0.062, pin_dy, 0.044),         # forward-top left corner
        (0.062, -pin_dy, 0.044),        # cross to the right side (top bar)
        (0.022, -pin_dy, 0.026),
        (-0.004, -pin_dy, 0.0),         # at the right lug
    ]
    bail_geom = tube_from_spline_points(
        bail_pts, radius=0.006, samples_per_segment=10, radial_segments=12,
        spline="catmull_rom",
    )
    bail.visual(
        mesh_from_geometry(bail_geom, "bail_lever"),
        material=steel,
        name="bail_lever",
    )
    bail.inertial = Inertial.from_geometry(
        Cylinder(radius=0.006, length=0.16), mass=0.2
    )
    model.articulation(
        "body_to_bail",
        ArticulationType.REVOLUTE,
        parent=body,
        child=bail,
        origin=Origin(xyz=(pin_x, 0.0, pin_z)),
        # The arch extends forward/up at q=0 (closed-along-barrel). Using -Y so
        # positive q swings the forward tip up and back (valve open).
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=2.0, lower=0.0, upper=1.2),
    )

    # ===================================================================
    # SPRAY-PATTERN SELECTOR COLLAR + FOG DIFFUSER (rotates about barrel axis)
    # ===================================================================
    collar = model.part("selector_collar")
    # Stepped fog/spray cone: concentric rings (diffuser face) + a knurled grip
    # collar. Authored with barrel axis = local +Z, mounted with rpy pitch=+90
    # so the joint axis (local Z) maps to world +X.
    diffuser = LatheGeometry(
        [
            (0.026, 0.0),
            (0.030, 0.006),    # knurled collar ring
            (0.030, 0.026),
            (0.026, 0.030),
            (0.046, 0.040),    # first cone step
            (0.038, 0.048),
            (0.060, 0.060),    # outer spreader ring
            (0.052, 0.066),
            (0.020, 0.066),    # bore mouth
            (0.020, 0.040),
            (0.022, 0.006),
            (0.026, 0.0),
        ],
        segments=40,
    )
    collar.visual(
        mesh_from_geometry(diffuser, "fog_diffuser"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=nozzle_orange,
        name="fog_diffuser",
    )
    # steel knurled grip ring at the base of the collar
    knurl = TorusGeometry(0.031, 0.006, radial_segments=18, tubular_segments=32)
    collar.visual(
        mesh_from_geometry(knurl, "selector_knurl"),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=steel,
        name="selector_knurl",
    )
    collar.inertial = Inertial.from_geometry(
        Cylinder(radius=0.060, length=0.066), mass=0.4
    )
    # Joint at the outlet neck mouth, joint axis (local Z) = world +X.
    model.articulation(
        "body_to_selector",
        ArticulationType.REVOLUTE,
        parent=body,
        child=collar,
        origin=Origin(xyz=(neck_x + 0.022, 0.0, barrel_axis_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-2.0, upper=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    bail = object_model.get_part("bail_lever")
    collar = object_model.get_part("selector_collar")
    bail_joint = object_model.get_articulation("body_to_bail")
    sel_joint = object_model.get_articulation("body_to_selector")

    # --- Pistol grip butt rests on the floor (z=0), object not floating ---
    bb = ctx.part_world_aabb(body)
    assert bb is not None
    ctx.check("grip_on_floor", abs(bb[0][2]) < 0.006, details=f"body min z={bb[0][2]:.4f}")

    # --- Barrel runs horizontally (wider in X than tall) ---
    ctx.check(
        "barrel_horizontal",
        (bb[1][0] - bb[0][0]) > (bb[1][2] - bb[0][2]) * 0.6,
        details=f"x-extent={bb[1][0]-bb[0][0]:.3f}, z-extent={bb[1][2]-bb[0][2]:.3f}",
    )

    # --- Inlet bell at the rear (-X), outlet/collar at the front (+X) ---
    cpos = ctx.part_world_position(collar)
    assert cpos is not None
    bell_aabb = ctx.part_element_world_aabb(body, elem="inlet_bell")
    assert bell_aabb is not None
    bell_x = (bell_aabb[0][0] + bell_aabb[1][0]) / 2
    ctx.check(
        "inlet_rear_outlet_front",
        bell_x < 0.0 < cpos[0],
        details=f"bell_x={bell_x:.3f}, collar_x={cpos[0]:.3f}",
    )

    # --- Bail lever pivots about the horizontal Y axis and swings up/back ---
    assert abs(bail_joint.axis[1]) > 0.99, "bail axis must be horizontal Y"
    rest_tip = ctx.part_element_world_aabb(bail, elem="bail_lever")
    assert rest_tip is not None
    rest_fwd_x = rest_tip[1][0]
    with ctx.pose({bail_joint: bail_joint.motion_limits.upper}):
        open_tip = ctx.part_element_world_aabb(bail, elem="bail_lever")
        assert open_tip is not None
        open_top_z = open_tip[1][2]
        open_fwd_x = open_tip[1][0]
    ctx.check(
        "bail_swings_open",
        open_top_z > rest_tip[1][2] - 0.001 and open_fwd_x < rest_fwd_x - 0.01,
        details=f"rest tip x={rest_fwd_x:.3f}, open tip x={open_fwd_x:.3f}, open top z={open_top_z:.3f}",
    )

    # --- Bail is carried by the lugs/pin (contacts the body) ---
    ctx.expect_contact(bail, body, contact_tol=0.010, name="bail_supported_by_pin")
    # The bail lever wraps around the cross-pin (intended pivot capture).
    ctx.allow_overlap(
        bail, body, elem_a="bail_lever", elem_b="bail_pin",
        reason="The bail lever pivots on and wraps around the cross-pin (captured hinge).",
    )

    # --- Selector collar spins about the barrel (world X) axis ---
    before = ctx.part_world_aabb(collar)
    with ctx.pose({sel_joint: 1.2}):
        after = ctx.part_world_aabb(collar)
    assert before is not None and after is not None
    spun = any(abs(before[k][i] - after[k][i]) > 1e-4 for k in (0, 1) for i in (1, 2))
    ctx.check("selector_rotates", spun, details="collar AABB should change in Y/Z when spun")

    # --- Selector collar seats at the outlet neck (contacts the body) ---
    ctx.expect_contact(collar, body, contact_tol=0.012, name="collar_seated_on_neck")

    # The selector collar wraps over the outlet neck stub (seated, rotating fit).
    ctx.allow_overlap(
        collar, body, elem_a="fog_diffuser", elem_b="outlet_neck",
        reason="The selector collar is seated over the outlet neck and rotates on it.",
    )

    # --- Bail lugs root into the barrel shoulder (mounted, connected) ---
    ctx.expect_contact(body, body, elem_a="bail_lug_left", elem_b="barrel",
                       contact_tol=0.001, name="left_lug_rooted")
    ctx.expect_contact(body, body, elem_a="bail_lug_right", elem_b="barrel",
                       contact_tol=0.001, name="right_lug_rooted")

    return ctx.report()


object_model = build_object_model()
