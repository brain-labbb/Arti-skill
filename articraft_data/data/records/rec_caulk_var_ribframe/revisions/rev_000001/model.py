from __future__ import annotations

# Realistic articulated caulking gun — open skeleton rod-cage frame variant.
#
# Geometry conventions (all meters, real-world scale):
#   +X : barrel axis pointing forward toward the nozzle.
#   +Z : up.
#   +Y : left/right across the gun.
#
# The fixed frame is an OPEN SKELETON ROD-CAGE built from N_RODS = 4 thin
# longitudinal metal rib rods arranged around the lower circumference of the
# cartridge, connecting the front ring to the rear plate. A full-length bottom
# spine rail carries the pistol grip and pivot lugs. Two user mechanisms:
#   - trigger lever (REVOLUTE about Y at the pivot pin)
#   - plunger rod + push plate (PRISMATIC along +X)

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
)

# ---------------------------------------------------------------------------
# Key dimensions (meters)
# ---------------------------------------------------------------------------
BARREL_R = 0.027          # rod-cage placement radius (inner cradle radius)
BARREL_LEN = 0.260        # length of the cage / cartridge body
CART_R = 0.0255           # silicone cartridge outer radius
CART_LEN = 0.250          # cartridge body length
FRONT_X = BARREL_LEN / 2.0      # +X face of the barrel
BACK_X = -BARREL_LEN / 2.0      # -X face (rear) of the barrel

CRADLE_WALL = 0.0035
CRADLE_OUTER_R = BARREL_R + CRADLE_WALL  # ≈ 0.0305

# Rod cage parameters
N_RODS = 4
ROD_R = 0.003             # thin metal rod radius
ROD_ARC_START_DEG = 210   # first rod angle (deg from +Y toward +Z)
ROD_ARC_END_DEG = 330     # last rod angle
# With 4 rods: 210°, 250°, 290°, 330° (evenly spaced across lower 120° arc)

# Grip / handle
GRIP_TOP_Z = -0.014       # where the grip meets the barrel underside
GRIP_X = BACK_X + 0.040   # grip just forward of the rear plate
GRIP_LEN = 0.115          # how far the grip hangs down
GRIP_FRONT_X = GRIP_X + 0.030

# Trigger pivot
PIVOT_X = GRIP_FRONT_X + 0.018
PIVOT_Z = -0.040
PIVOT_Y_HALF = 0.012      # half-spacing of the pivot lugs

# Plunger rod
ROD_LEN = 0.230
PLATE_X = BACK_X + 0.006  # push plate seats against cartridge rear


# ---------------------------------------------------------------------------
# Shared geometry helper: one thin longitudinal rib rod
# ---------------------------------------------------------------------------
def _rib_rod_geometry(angle_rad: float) -> cq.Workplane:
    """Thin cylinder along the barrel axis at the given angular position."""
    y = BARREL_R * math.cos(angle_rad)
    z = BARREL_R * math.sin(angle_rad)
    # Rod extends slightly into the front ring and rear plate for connectivity.
    rod = (
        cq.Workplane("YZ")
        .workplane(offset=BACK_X - 0.003)
        .center(y, z)
        .circle(ROD_R)
        .extrude(BARREL_LEN + 0.006)
    )
    return rod


# ---------------------------------------------------------------------------
# Frame components (front ring, rear plate, spine rail, grip, pivot lugs)
# ---------------------------------------------------------------------------
def _build_front_ring() -> cq.Workplane:
    """Front cap ring with center bore the nozzle passes through."""
    ring = (
        cq.Workplane("YZ")
        .workplane(offset=FRONT_X)
        .circle(CRADLE_OUTER_R + 0.004)
        .circle(0.012)
        .extrude(0.014)
    )
    return ring


def _build_rear_plate() -> cq.Workplane:
    """Rear end plate closing the back; plunger rod passes through the bore."""
    plate = (
        cq.Workplane("YZ")
        .workplane(offset=BACK_X - 0.010)
        .circle(CRADLE_OUTER_R + 0.004)
        .circle(0.006)
        .extrude(0.012)
    )
    return plate


def _build_spine_rail() -> cq.Workplane:
    """Full-length bottom rail connecting front ring to rear plate.

    This flat bar runs along the underside of the rod cage, providing the
    structural backbone that carries the grip and pivot lugs.
    """
    length = BARREL_LEN + 0.020  # extends past both ring and plate
    cz = -0.032                  # just below the rod cage bottom
    spine = (
        cq.Workplane("XY")
        .box(length, 0.030, 0.006)
        .translate((0.0, 0.0, cz))
    )
    return spine


def _build_grip() -> cq.Workplane:
    """Pistol grip: curved tongue hanging from the rear underside."""
    pts = [
        (GRIP_X - 0.020, GRIP_TOP_Z),
        (GRIP_X + 0.030, GRIP_TOP_Z),
        (GRIP_X + 0.018, GRIP_TOP_Z - GRIP_LEN * 0.55),
        (GRIP_X + 0.002, GRIP_TOP_Z - GRIP_LEN),
        (GRIP_X - 0.030, GRIP_TOP_Z - GRIP_LEN * 0.95),
        (GRIP_X - 0.034, GRIP_TOP_Z - GRIP_LEN * 0.4),
        (GRIP_X - 0.022, GRIP_TOP_Z - 0.006),
    ]
    grip = (
        cq.Workplane("XZ")
        .polyline(pts)
        .close()
        .extrude(0.020)
        .translate((0, 0.010, 0))
        .edges("|Y")
        .fillet(0.006)
    )
    return grip


def _build_pivot_lugs() -> cq.Workplane:
    """Yoke of two side plates carrying the trigger pivot pin.

    The strut tops reach down to the spine rail (replacing the old cradle
    attachment) so the pivot lugs are structurally grounded.
    """
    strut_top_z = -0.030  # reaches the spine rail top surface
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


def _build_frame_body_mesh() -> cq.Workplane:
    """United frame body: front ring + rear plate + spine rail + grip + lugs."""
    body = _build_front_ring()
    body = body.union(_build_rear_plate())
    body = body.union(_build_spine_rail())
    body = body.union(_build_grip())
    body = body.union(_build_pivot_lugs())
    return mesh_from_cadquery(body, "frame_body")


def _build_pivot_pin_mesh() -> cq.Workplane:
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
def _build_cartridge_mesh() -> cq.Workplane:
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


def _build_front_collar_mesh() -> cq.Workplane:
    collar = (
        cq.Workplane("YZ")
        .workplane(offset=CART_LEN / 2.0 - 0.006)
        .circle(CART_R)
        .workplane(offset=0.022)
        .circle(0.011)
        .loft()
    )
    return mesh_from_cadquery(collar, "front_collar")


def _build_nozzle_mesh() -> cq.Workplane:
    nz_base_x = CART_LEN / 2.0 + 0.016
    nozzle = (
        cq.Workplane("YZ")
        .workplane(offset=nz_base_x)
        .circle(0.010)
        .workplane(offset=0.055)
        .circle(0.0045)
        .loft()
    )
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
def _build_trigger_mesh() -> cq.Workplane:
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
def _build_plunger_mesh() -> cq.Workplane:
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
    # United body: front ring + rear plate + spine rail + grip + pivot lugs
    frame.visual(_build_frame_body_mesh(), material="frame_red", name="frame_body")
    frame.visual(_build_pivot_pin_mesh(), material="steel", name="pivot_pin")

    # Rib rods: N identical thin longitudinal rods forming the open cage.
    # Evenly spaced by angle across the lower circumference of the cartridge,
    # connecting the front ring to the rear plate.
    for i in range(N_RODS):
        angle_deg = (
            ROD_ARC_START_DEG
            + i * (ROD_ARC_END_DEG - ROD_ARC_START_DEG) / max(N_RODS - 1, 1)
        )
        rod_mesh = mesh_from_cadquery(
            _rib_rod_geometry(math.radians(angle_deg)), f"rib_rod_{i}"
        )
        frame.visual(rod_mesh, material="steel", name=f"rib_rod_{i}")

    # --- Cartridge (seated consumable, fixed to frame) ---
    cartridge = model.part("cartridge")
    cartridge.visual(
        _build_cartridge_mesh(), material="cartridge_blue", name="cartridge_body"
    )
    cartridge.visual(
        _build_front_collar_mesh(), material="collar_red", name="front_collar"
    )
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

    # --- Skeleton rod cage: all N rods present as named visuals ---
    frame_visual_names = {v.name for v in frame.visuals}
    for i in range(N_RODS):
        ctx.check(
            f"rod cage has rib_rod_{i}",
            f"rib_rod_{i}" in frame_visual_names,
            details=f"frame visuals={sorted(frame_visual_names)}",
        )

    # Each rod is a thin longitudinal cylinder (small cross-section, long along X).
    for i in range(N_RODS):
        rod_aabb = ctx.part_element_world_aabb(frame, elem=f"rib_rod_{i}")
        if rod_aabb is not None:
            dx = rod_aabb[1][0] - rod_aabb[0][0]
            dy = rod_aabb[1][1] - rod_aabb[0][1]
            dz = rod_aabb[1][2] - rod_aabb[0][2]
            ctx.check(
                f"rib_rod_{i} is a thin longitudinal rod",
                dx > 0.20 and dy < 0.012 and dz < 0.012,
                details=f"dx={dx:.4f} dy={dy:.4f} dz={dz:.4f}",
            )

    # All rods sit below the barrel axis (lower circumference placement).
    for i in range(N_RODS):
        rod_aabb = ctx.part_element_world_aabb(frame, elem=f"rib_rod_{i}")
        if rod_aabb is not None:
            rod_center_z = (rod_aabb[0][2] + rod_aabb[1][2]) / 2.0
            ctx.check(
                f"rib_rod_{i} is below barrel axis",
                rod_center_z < -0.005,
                details=f"center_z={rod_center_z:.4f}",
            )

    # --- Hero parts present and placed ---
    nozzle_aabb = ctx.part_element_world_aabb(cartridge, elem="nozzle")
    frame_aabb = ctx.part_world_aabb(frame)
    ctx.check(
        "nozzle protrudes forward past the front ring",
        nozzle_aabb is not None and nozzle_aabb[1][0] >= frame_aabb[1][0] - 0.001,
        details=f"nozzle_max_x={nozzle_aabb[1][0] if nozzle_aabb else None}, "
        f"frame_max_x={frame_aabb[1][0]}",
    )

    ctx.check(
        "grip hangs well below the barrel",
        frame_aabb[0][2] < -0.10,
        details=f"frame_min_z={frame_aabb[0][2]}",
    )

    # Cartridge runs the length of the rod cage along the barrel axis.
    ctx.expect_overlap(
        cartridge,
        frame,
        axes="x",
        elem_a="cartridge_body",
        elem_b="rib_rod_0",
        min_overlap=0.15,
        name="cartridge runs the length of the rod cage",
    )

    # Cartridge sits within the frame cross-section (the front ring and rear
    # plate discs are larger than the cartridge, confirming it seats inside).
    ctx.expect_within(
        cartridge,
        frame,
        axes="yz",
        inner_elem="cartridge_body",
        outer_elem="frame_body",
        margin=0.008,
        name="cartridge sits within the frame cross-section (yz)",
    )

    # --- Overlap allowances ---
    # Cartridge body rests against the rod cage rods (seated consumable fit).
    for i in range(N_RODS):
        ctx.allow_overlap(
            cartridge,
            frame,
            elem_a="cartridge_body",
            elem_b=f"rib_rod_{i}",
            reason=f"Cartridge is seated in the open rod cage, resting on rib_rod_{i}.",
        )

    # Cartridge body also seats into the front ring and rear plate (the two
    # end-caps of the united frame_body), so there is local overlap where the
    # cartridge end faces contact the ring/plate bores.
    ctx.allow_overlap(
        cartridge,
        frame,
        elem_a="cartridge_body",
        elem_b="frame_body",
        reason="Cartridge body seats into the front ring bore and rear plate bore of the united frame body.",
    )

    # Cartridge front collar seats into the front cap ring bore.
    ctx.allow_overlap(
        cartridge,
        frame,
        elem_a="front_collar",
        elem_b="frame_body",
        reason="Cartridge front shoulder seats into and passes through the front cap ring bore.",
    )

    # Trigger hub captures the steel pivot pin.
    ctx.allow_overlap(
        trigger,
        frame,
        elem_a="trigger_blade",
        elem_b="pivot_pin",
        reason="Trigger pivot hub intentionally captures the steel pivot pin.",
    )

    # Trigger blade hub also contacts the pivot lug side plates of the frame
    # body (the hub rotates between the two lugs).
    ctx.allow_overlap(
        trigger,
        frame,
        elem_a="trigger_blade",
        elem_b="frame_body",
        reason="Trigger pivot hub rotates between the pivot lug side plates of the frame body.",
    )

    # Plunger rod/plate slides inside the rod cage and through the rear plate bore.
    ctx.allow_overlap(
        plunger,
        frame,
        elem_a="plunger_rod",
        elem_b="frame_body",
        reason="Plunger rod/plate slides inside the rod cage and through the rear plate bore.",
    )

    # Push plate bears on the rear of the cartridge contents.
    ctx.allow_overlap(
        plunger,
        cartridge,
        elem_a="plunger_rod",
        elem_b="cartridge_body",
        reason="Push plate bears on the rear of the cartridge contents (simplified solid proxy).",
    )

    # --- Mechanism motion checks ---
    # Squeezing the trigger swings its lower blade rearward toward the grip.
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

    # Advancing the plunger drives the rod forward toward the nozzle (+X).
    plate_rest = ctx.part_world_position(plunger)
    with ctx.pose({plunger_joint: 0.150}):
        plate_adv = ctx.part_world_position(plunger)
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
