from __future__ import annotations

# Realistic articulated caulking gun (skeleton / half-barrel cartridge gun).
#
# Geometry conventions (all meters, real-world scale):
#   +X : barrel axis pointing forward toward the nozzle.
#   +Z : up.
#   +Y : left/right across the gun.
# The fixed frame is the red half-barrel cradle plus the front cap ring, the
# rear back-plate, the D-ring loop handle, and the pivot lugs. A blue/gray
# silicone cartridge is seated in the cradle with a white tapered nozzle at
# the front. The two real user mechanisms are:
#   - the trigger lever (REVOLUTE about Y at the upper-front of the D-ring), and
#   - the plunger rod + push plate that advances the cartridge contents
#     (PRISMATIC along the barrel axis +X).

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
    mesh_from_geometry,
    tube_from_spline_points,
)

# ---------------------------------------------------------------------------
# Key dimensions (meters)
# ---------------------------------------------------------------------------
BARREL_R = 0.027          # cartridge cradle inner radius
BARREL_LEN = 0.260        # length of the cradle / cartridge body
CART_R = 0.0255           # silicone cartridge radius (sits inside cradle)
CART_LEN = 0.250          # cartridge body length
FRONT_X = BARREL_LEN / 2.0      # +X face of the barrel/cartridge
BACK_X = -BARREL_LEN / 2.0      # -X face (rear) of the barrel

# Cradle is a half-pipe open at the top (so the cartridge reads as seated, not
# enclosed). It wraps roughly the lower 300 deg of the tube.
CRADLE_WALL = 0.0035
CRADLE_OUTER_R = BARREL_R + CRADLE_WALL

# D-ring loop handle
# The D-ring is a closed loop of round bar that rises from the barrel underside.
# The hand passes through the opening; the trigger sits inside the loop.
DRING_BAR_R = 0.005       # round bar radius (10 mm diameter stock)
DRING_TOP_Z = -0.032      # top of the loop at the barrel underside
# D-ring centerline path (XZ plane, Y=0): a D-shape with the flat top along the
# barrel and the curved bottom hanging below. The front drops well forward of
# the trigger blade so the blade swings freely inside the loop opening.
DRING_PATH = [
    (-0.110, 0.0, DRING_TOP_Z),       # top-rear attachment
    (-0.076, 0.0, DRING_TOP_Z + 0.004),  # top-center (slight rise)
    (-0.048, 0.0, DRING_TOP_Z),       # top near pivot (behind hub)
    (-0.014, 0.0, -0.052),            # front-upper (well forward of trigger)
    (-0.008, 0.0, -0.092),            # front-lower
    (-0.035, 0.0, -0.130),            # bottom-front
    (-0.068, 0.0, -0.140),            # bottom-center (lowest)
    (-0.108, 0.0, -0.130),            # bottom-rear
    (-0.128, 0.0, -0.065),            # rear rise
]

# Trigger pivot: just FORWARD of the D-ring front and BELOW the cartridge,
# so the finger loop sits inside the D-ring opening.
PIVOT_X = -0.042
PIVOT_Z = -0.040
PIVOT_Y_HALF = 0.012      # half-spacing of the pivot lugs


def _red():
    return cq.Color(0.86, 0.16, 0.16, 1.0)


# ---------------------------------------------------------------------------
# Frame geometry (cradle + front cap ring + rear plate + D-ring bosses + pivot lugs)
# ---------------------------------------------------------------------------
def _build_cradle() -> cq.Workplane:
    # Half-pipe: full tube wall, then cut away the top opening so the cartridge
    # shows. Built along Z then rotated to lie along X.
    tube = (
        cq.Workplane("XY")
        .circle(CRADLE_OUTER_R)
        .circle(BARREL_R)
        .extrude(BARREL_LEN)
        .translate((0, 0, -BARREL_LEN / 2.0))
    )
    # Cut the top ~60 deg gap so it is an open cradle, not a closed pipe.
    gap = (
        cq.Workplane("XY")
        .box(2 * CRADLE_OUTER_R, 2 * CRADLE_OUTER_R, BARREL_LEN + 0.01)
        .translate((0, CRADLE_OUTER_R * 0.92, 0))
    )
    cradle = tube.cut(gap)
    # Orient: cylinder axis Z -> barrel axis X.
    cradle = cradle.rotate((0, 0, 0), (0, 1, 0), 90.0)
    return cradle


def _build_front_ring() -> cq.Workplane:
    # Solid front cap ring with a center bore the nozzle passes through.
    ring = (
        cq.Workplane("YZ")
        .workplane(offset=FRONT_X)
        .circle(CRADLE_OUTER_R + 0.004)
        .circle(0.012)
        .extrude(0.014)
    )
    return ring


def _build_rear_plate() -> cq.Workplane:
    # Rear end plate that closes the back of the cradle; rod passes through it.
    plate = (
        cq.Workplane("YZ")
        .workplane(offset=BACK_X - 0.010)
        .circle(CRADLE_OUTER_R + 0.004)
        .circle(0.006)
        .extrude(0.012)
    )
    return plate


def _build_dring_loop():
    """Closed D-ring loop handle: round bar swept along a D-shaped spline path."""
    loop = tube_from_spline_points(
        DRING_PATH,
        radius=DRING_BAR_R,
        samples_per_segment=18,
        radial_segments=20,
        closed_spline=True,
        cap_ends=False,
        up_hint=(0.0, 1.0, 0.0),
    )
    return mesh_from_geometry(loop, "dring_loop")


def _build_dring_bosses() -> cq.Workplane:
    """Two small cylindrical mounting bosses where the D-ring bar welds into
    the barrel underside. Each boss is a short puck protruding downward from
    the cradle surface at the D-ring attachment points."""
    boss_r = DRING_BAR_R + 0.003
    boss_h = 0.008

    def boss_at(x: float, z: float):
        return (
            cq.Workplane("XZ")
            .workplane(offset=0.008)
            .center(x, z)
            .circle(boss_r)
            .extrude(-0.016)
        )

    # Rear attachment boss
    rear = boss_at(DRING_PATH[0][0], DRING_PATH[0][2])
    # Front attachment boss
    front = boss_at(DRING_PATH[3 - 1][0], DRING_PATH[3 - 1][2])
    return rear.union(front)


def _build_pivot_lugs() -> cq.Workplane:
    # A yoke of two side plates drops from the barrel underside down to the
    # pivot lugs, straddling the trigger hub (a center gap is left open for the
    # hub). This mounts the trigger to the frame with real structure and no
    # solid block intruding into the hub's swing.
    strut_top_z = -CRADLE_OUTER_R + 0.002
    plate_h = abs(strut_top_z - PIVOT_Z) + 0.012
    plate_cz = (PIVOT_Z + strut_top_z) / 2.0

    def side_plate(y_inner: float, thickness: float):
        return (
            cq.Workplane("XZ")
            .workplane(offset=y_inner)
            .center(PIVOT_X, plate_cz)
            .rect(0.018, plate_h)
            .extrude(thickness)
            .edges("|Y")
            .fillet(0.004)
        )

    # Left and right side plates with lug discs at the bottom carrying the pin.
    left_plate = side_plate(PIVOT_Y_HALF, 0.006)
    right_plate = side_plate(-PIVOT_Y_HALF - 0.006, 0.006)

    left_lug = (
        cq.Workplane("XZ")
        .workplane(offset=PIVOT_Y_HALF)
        .center(PIVOT_X, PIVOT_Z)
        .circle(0.0095)
        .extrude(0.006)
    )
    right_lug = (
        cq.Workplane("XZ")
        .workplane(offset=-PIVOT_Y_HALF - 0.006)
        .center(PIVOT_X, PIVOT_Z)
        .circle(0.0095)
        .extrude(0.006)
    )
    return left_plate.union(right_plate).union(left_lug).union(right_lug)


def _build_frame_mesh():
    frame = _build_cradle()
    frame = frame.union(_build_front_ring())
    frame = frame.union(_build_rear_plate())
    frame = frame.union(_build_dring_bosses())
    frame = frame.union(_build_pivot_lugs())
    return mesh_from_cadquery(frame, "frame")


def _build_pivot_pin_mesh():
    pin = (
        cq.Workplane("XZ")
        .workplane(offset=PIVOT_Y_HALF + 0.005)
        .center(PIVOT_X, PIVOT_Z)
        .circle(0.0035)
        .extrude(-(2 * PIVOT_Y_HALF + 0.010))
    )
    return mesh_from_cadquery(pin, "pivot_pin")


# ---------------------------------------------------------------------------
# Cartridge + nozzle (seated consumable)
# ---------------------------------------------------------------------------
def _build_cartridge_mesh():
    # Blue/gray cylindrical silicone cartridge body. Slightly shorter than the
    # cradle so it reads as dropped in. Built along Z then rotated to X.
    body = (
        cq.Workplane("XY")
        .circle(CART_R)
        .extrude(CART_LEN)
        .translate((0, 0, -CART_LEN / 2.0))
        .edges(">Z or <Z")
        .fillet(0.004)
    )
    body = body.rotate((0, 0, 0), (0, 1, 0), 90.0)
    return mesh_from_cadquery(body, "cartridge")


def _build_front_collar_mesh():
    # Red front shoulder/collar of the cartridge that tapers down to the nozzle.
    collar = (
        cq.Workplane("YZ")
        .workplane(offset=CART_LEN / 2.0 - 0.006)
        .circle(CART_R)
        .workplane(offset=0.022)
        .circle(0.011)
        .loft()
    )
    return mesh_from_cadquery(collar, "front_collar")


def _build_nozzle_mesh():
    # White tapered conical nozzle tip protruding from the front.
    nz_base_x = CART_LEN / 2.0 + 0.016
    nozzle = (
        cq.Workplane("YZ")
        .workplane(offset=nz_base_x)
        .circle(0.010)
        .workplane(offset=0.055)
        .circle(0.0045)
        .loft()
    )
    # small bore at the very tip
    bore = (
        cq.Workplane("YZ")
        .workplane(offset=nz_base_x + 0.030)
        .circle(0.0028)
        .extrude(0.030)
    )
    nozzle = nozzle.cut(bore)
    return mesh_from_cadquery(nozzle, "nozzle")


# ---------------------------------------------------------------------------
# Trigger lever
# ---------------------------------------------------------------------------
def _build_trigger_mesh():
    # The trigger pivots at the top (pivot pin) and the finger blade curves
    # down and forward. Authored in the part-local frame whose origin is the
    # pivot, so the joint frame and mesh frame coincide.
    # Local frame: X forward, Z up, pivot at local origin. The finger blade
    # hangs below the pivot and curves slightly forward; its lower tip is the
    # part the user pulls. All points stay at local z <= +0.009 so the trigger
    # clears the cartridge above it.
    pts = [
        (0.009, 0.008),
        (0.016, -0.006),
        (0.020, -0.040),
        (0.014, -0.078),
        (0.000, -0.080),
        (-0.002, -0.045),
        (-0.012, -0.012),
        (-0.011, 0.006),
    ]
    blade = (
        cq.Workplane("XZ")
        .polyline(pts)
        .close()
        .extrude(0.014)
        .translate((0, 0.007, 0))
        .edges("|Y")
        .fillet(0.0035)
    )
    # Pivot eye (hub) around local origin so it wraps the pin.
    hub = (
        cq.Workplane("XZ")
        .workplane(offset=0.011)
        .circle(0.0095)
        .extrude(-0.022)
    )
    trigger = blade.union(hub)
    return mesh_from_cadquery(trigger, "trigger")


# ---------------------------------------------------------------------------
# Plunger rod + push plate
# ---------------------------------------------------------------------------
ROD_LEN = 0.230
PLATE_X = BACK_X + 0.006   # push plate seats against the rear of the cartridge


def _build_plunger_mesh():
    # Push plate (disc that bears on the cartridge plunger) + long rod that
    # extends out the back, ending in a thumb hook / handle.
    plate = (
        cq.Workplane("YZ")
        .workplane(offset=PLATE_X)
        .circle(CART_R - 0.002)
        .extrude(-0.008)
    )
    rod = (
        cq.Workplane("YZ")
        .workplane(offset=PLATE_X - 0.008)
        .circle(0.0045)
        .extrude(-ROD_LEN)
    )
    plunger = plate.union(rod)
    # Rear thumb plate / pull handle at the very back of the rod.
    rear_x = PLATE_X - 0.008 - ROD_LEN
    handle = (
        cq.Workplane("XZ")
        .workplane(offset=0.006)
        .center(rear_x - 0.010, 0)
        .rect(0.020, 0.055)
        .extrude(-0.012)
        .edges("|Y")
        .fillet(0.004)
    )
    plunger = plunger.union(handle)
    return mesh_from_cadquery(plunger, "plunger")


# ---------------------------------------------------------------------------
# Model assembly
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="caulking_gun")

    model.material("frame_red", rgba=(0.84, 0.15, 0.15, 1.0))
    model.material("steel", rgba=(0.55, 0.56, 0.58, 1.0))
    model.material("cartridge_blue", rgba=(0.32, 0.42, 0.52, 1.0))
    model.material("collar_red", rgba=(0.78, 0.18, 0.18, 1.0))
    model.material("nozzle_white", rgba=(0.92, 0.92, 0.90, 1.0))

    # --- Fixed frame (root) ---
    frame = model.part("frame")
    frame.visual(_build_frame_mesh(), material="frame_red", name="frame_shell")
    frame.visual(_build_pivot_pin_mesh(), material="steel", name="pivot_pin")
    frame.visual(_build_dring_loop(), material="steel", name="dring_loop")

    # --- Cartridge (seated consumable, fixed to frame) ---
    cartridge = model.part("cartridge")
    cartridge.visual(_build_cartridge_mesh(), material="cartridge_blue", name="cartridge_body")
    cartridge.visual(_build_front_collar_mesh(), material="collar_red", name="front_collar")
    cartridge.visual(_build_nozzle_mesh(), material="nozzle_white", name="nozzle")

    model.articulation(
        "frame_to_cartridge",
        ArticulationType.FIXED,
        parent=frame,
        child=cartridge,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # --- Trigger (revolute about Y at the pivot pin) ---
    trigger = model.part("trigger")
    trigger.visual(_build_trigger_mesh(), material="frame_red", name="trigger_blade")

    # Trigger mesh is authored in a pivot-local frame (origin at the pivot).
    # Closed blade hangs down/forward; squeezing pulls the lower blade back
    # toward the grip. With the blade reaching toward -X/-Z, a +Y axis with a
    # positive q rotates the lower tip toward the grip (+X).
    model.articulation(
        "frame_to_trigger",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=trigger,
        origin=Origin(xyz=(PIVOT_X, 0.0, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=3.0, lower=0.0, upper=0.5),
    )

    # --- Plunger rod (prismatic along +X, advances toward the nozzle) ---
    plunger = model.part("plunger")
    plunger.visual(_build_plunger_mesh(), material="steel", name="plunger_rod")

    model.articulation(
        "frame_to_plunger",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=plunger,
        # Joint frame at the rear plate seating plane; +X dispenses.
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=300.0, velocity=0.05, lower=0.0, upper=0.180),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    cartridge = object_model.get_part("cartridge")
    trigger = object_model.get_part("trigger")
    plunger = object_model.get_part("plunger")

    trig_joint = object_model.get_articulation("frame_to_trigger")
    plunger_joint = object_model.get_articulation("frame_to_plunger")
    cart_joint = object_model.get_articulation("frame_to_cartridge")

    # --- Joint type / axis claims ---
    ctx.check(
        "trigger is revolute about Y",
        str(trig_joint.articulation_type).upper().endswith("REVOLUTE")
        and tuple(round(v, 3) for v in trig_joint.axis) == (0.0, 1.0, 0.0),
        details=f"type={trig_joint.articulation_type} axis={trig_joint.axis}",
    )
    ctx.check(
        "plunger is prismatic along X",
        str(plunger_joint.articulation_type).upper().endswith("PRISMATIC")
        and tuple(round(v, 3) for v in plunger_joint.axis) == (1.0, 0.0, 0.0),
        details=f"type={plunger_joint.articulation_type} axis={plunger_joint.axis}",
    )
    ctx.check(
        "cartridge fixed to frame",
        str(cart_joint.articulation_type).upper().endswith("FIXED"),
        details=f"type={cart_joint.articulation_type}",
    )

    # --- Hero parts present and placed ---
    # Nozzle protrudes forward (+X) of the front cap ring.
    nozzle_aabb = ctx.part_element_world_aabb(cartridge, elem="nozzle")
    frame_aabb = ctx.part_world_aabb(frame)
    ctx.check(
        "nozzle protrudes forward past the front ring",
        nozzle_aabb is not None and nozzle_aabb[1][0] >= frame_aabb[1][0] - 0.001,
        details=f"nozzle_max_x={None if nozzle_aabb is None else nozzle_aabb[1][0]}, frame_max_x={frame_aabb[1][0]}",
    )

    # D-ring loop hangs well below the barrel (the hand passes through it).
    ctx.check(
        "D-ring loop hangs well below the barrel",
        frame_aabb[0][2] < -0.10,
        details=f"frame_min_z={frame_aabb[0][2]}",
    )

    # D-ring loop visual exists and is named correctly.
    dring_aabb = ctx.part_element_world_aabb(frame, elem="dring_loop")
    ctx.check(
        "D-ring loop handle is present on the frame",
        dring_aabb is not None,
        details="dring_loop visual missing",
    )

    # Trigger sits inside the D-ring opening: the trigger XY footprint is
    # within the D-ring footprint and the trigger Z range overlaps the D-ring.
    if dring_aabb is not None:
        trig_aabb = ctx.part_world_aabb(trigger)
        ctx.check(
            "trigger sits inside the D-ring loop opening",
            trig_aabb is not None
            and trig_aabb[0][0] >= dring_aabb[0][0] - 0.005
            and trig_aabb[1][0] <= dring_aabb[1][0] + 0.005
            and trig_aabb[0][2] >= dring_aabb[0][2] - 0.005
            and trig_aabb[1][2] <= dring_aabb[1][2] + 0.005,
            details=f"trigger_aabb={trig_aabb}, dring_aabb={dring_aabb}",
        )

    # Cartridge seated inside the cradle: stays within the cradle radius in YZ
    # and overlaps the cradle along the barrel axis.
    ctx.expect_within(
        cartridge,
        frame,
        axes="yz",
        inner_elem="cartridge_body",
        outer_elem="frame_shell",
        margin=0.003,
        name="cartridge seated inside the cradle (yz)",
    )
    ctx.expect_overlap(
        cartridge,
        frame,
        axes="x",
        elem_a="cartridge_body",
        elem_b="frame_shell",
        min_overlap=0.15,
        name="cartridge runs the length of the cradle",
    )

    # Cartridge is dropped into the open cradle; its lower body nests inside the
    # half-pipe cradle wall (seated consumable fit).
    ctx.allow_overlap(
        cartridge,
        frame,
        elem_a="cartridge_body",
        elem_b="frame_shell",
        reason="Silicone cartridge is seated/nested inside the open half-pipe cradle of the frame.",
    )
    # The cartridge front shoulder/collar seats into the front cap ring (the
    # nose pokes through the ring bore, exactly as the cartridge meets the red
    # ring in the reference image).
    ctx.allow_overlap(
        cartridge,
        frame,
        elem_a="front_collar",
        elem_b="frame_shell",
        reason="Cartridge front shoulder seats into and passes through the front cap ring bore.",
    )

    # Trigger hub wraps the pivot pin -> intentional capture overlap.
    ctx.allow_overlap(
        trigger,
        frame,
        elem_a="trigger_blade",
        elem_b="pivot_pin",
        reason="Trigger pivot hub intentionally captures the steel pivot pin.",
    )
    # Trigger hub also sits between the pivot lugs (side plates + discs that
    # are part of frame_shell); the hub cylinder overlaps the lug discs at the
    # pivot center.
    ctx.allow_overlap(
        trigger,
        frame,
        elem_a="trigger_blade",
        elem_b="frame_shell",
        reason="Trigger pivot hub intentionally nests between the pivot lug discs on the frame.",
    )
    # D-ring loop bar passes near the trigger pivot hub; the bar and the hub
    # converge at the pivot attachment point (in reality the bar is welded to
    # the frame at the pivot and the hub rotates around the pin).
    ctx.allow_overlap(
        trigger,
        frame,
        elem_a="trigger_blade",
        elem_b="dring_loop",
        reason="D-ring loop bar converges with the trigger pivot hub at the frame pivot attachment.",
    )
    ctx.expect_contact(
        trigger,
        frame,
        elem_a="trigger_blade",
        elem_b="pivot_pin",
        contact_tol=0.002,
        name="trigger hub stays in contact with the pivot pin (captured joint)",
    )

    # Plunger plate + rod slide inside the barrel; the rod passes through the
    # rear plate bore and the plate runs inside the cradle (solid-proxy slide).
    ctx.allow_overlap(
        plunger,
        frame,
        elem_a="plunger_rod",
        elem_b="frame_shell",
        reason="Plunger rod/plate intentionally slides inside the barrel cradle and through the rear plate bore.",
    )

    # Plunger push plate bears on the rear of the cartridge contents.
    ctx.allow_overlap(
        plunger,
        cartridge,
        elem_a="plunger_rod",
        elem_b="cartridge_body",
        reason="Push plate bears on the rear of the cartridge contents (simplified solid cartridge proxy).",
    )

    # --- Mechanism motion checks ---
    # Squeezing the trigger swings its lower blade rearward toward the D-ring
    # handle (at -X relative to the pivot), pivoting about the top pin.
    trig_rest = ctx.part_world_aabb(trigger)
    with ctx.pose({trig_joint: 0.45}):
        trig_squeezed = ctx.part_world_aabb(trigger)
    ctx.check(
        "squeezing the trigger swings its lower blade rearward toward the grip",
        trig_rest is not None
        and trig_squeezed is not None
        and trig_squeezed[0][0] < trig_rest[0][0] - 0.020,
        details=f"rest_min_x={trig_rest[0][0]}, squeezed_min_x={trig_squeezed[0][0]}",
    )

    # Advancing the plunger moves the push plate forward toward the nozzle.
    plate_rest = ctx.part_world_position(plunger)
    with ctx.pose({plunger_joint: 0.150}):
        plate_adv = ctx.part_world_position(plunger)
        # plate must still be inside the cartridge footprint in YZ when advanced
        ctx.expect_within(
            plunger,
            cartridge,
            axes="yz",
            inner_elem="plunger_rod",
            outer_elem="cartridge_body",
            margin=0.006,
            name="advanced plunger plate stays inside the cartridge bore (yz)",
        )
    ctx.check(
        "advancing the plunger drives the rod forward toward the nozzle (+X)",
        plate_rest is not None
        and plate_adv is not None
        and plate_adv[0] > plate_rest[0] + 0.10,
        details=f"rest_x={plate_rest[0]}, advanced_x={plate_adv[0]}",
    )

    return ctx.report()


object_model = build_object_model()
