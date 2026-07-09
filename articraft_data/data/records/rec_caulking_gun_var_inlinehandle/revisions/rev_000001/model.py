from __future__ import annotations

# Realistic articulated caulking gun — inline D-handle variant.
#
# Geometry conventions (all meters, real-world scale):
#   +X : barrel axis pointing forward toward the nozzle.
#   +Z : up.
#   +Y : left/right across the gun.
#
# The fixed frame is the red half-barrel cradle, the front cap ring, the rear
# back-plate, the inline rear D-shaped handle, and the trigger pivot lugs.
# A blue/gray silicone cartridge is seated in the cradle with a white tapered
# nozzle at the front. The two real user mechanisms are:
#   - the thumb-squeeze trigger lever (REVOLUTE about Y at the top-rear of the
#     barrel/handle junction), and
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

# Cradle is a half-pipe open at one side so the cartridge reads as seated.
CRADLE_WALL = 0.0035
CRADLE_OUTER_R = BARREL_R + CRADLE_WALL

# Inline rear D-handle (aligned with the barrel axis)
HANDLE_LEN = 0.110                          # how far the handle extends rearward
HANDLE_REAR_X = BACK_X - HANDLE_LEN         # rearmost X of the handle
HANDLE_FRONT_X = BACK_X + 0.005             # front of handle (inside cradle/rear plate)
HANDLE_TOP_Z = CRADLE_OUTER_R + 0.008       # top rail slightly above barrel
HANDLE_BOT_Z = -(CRADLE_OUTER_R + 0.008)    # bottom rail slightly below barrel
HANDLE_Y_HALF = 0.017                       # half-width of handle in Y

# Thumb-squeeze trigger pivot: at the top-rear of the barrel, where the
# D-handle meets the cradle. The lever extends upward from here.
TRIG_PIVOT_X = BACK_X - 0.002              # just behind the barrel rear face
TRIG_PIVOT_Z = CRADLE_OUTER_R + 0.020      # well above the handle top rail
PIVOT_Y_HALF = 0.013                        # half-spacing of the pivot lugs

# Plunger rod
ROD_LEN = 0.230
PLATE_X = BACK_X + 0.006   # push plate seats against the rear of the cartridge


# ---------------------------------------------------------------------------
# Color helpers
# ---------------------------------------------------------------------------
def _red():
    return cq.Color(0.86, 0.16, 0.16, 1.0)


# ---------------------------------------------------------------------------
# Frame geometry (cradle + front ring + rear plate + D-handle + pivot lugs)
# ---------------------------------------------------------------------------
def _build_cradle() -> cq.Workplane:
    """Half-pipe cradle along the barrel axis."""
    tube = (
        cq.Workplane("XY")
        .circle(CRADLE_OUTER_R)
        .circle(BARREL_R)
        .extrude(BARREL_LEN)
        .translate((0, 0, -BARREL_LEN / 2.0))
    )
    # Cut the top gap so it is an open cradle, not a closed pipe.
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
    """Solid front cap ring with a center bore the nozzle passes through.

    Starts 3mm inside the cradle front face for mesh connectivity.
    """
    ring = (
        cq.Workplane("YZ")
        .workplane(offset=FRONT_X - 0.003)
        .circle(CRADLE_OUTER_R + 0.004)
        .circle(0.012)
        .extrude(0.017)
    )
    return ring


def _build_rear_plate() -> cq.Workplane:
    """Rear end plate closing the back of the cradle; rod passes through it."""
    plate = (
        cq.Workplane("YZ")
        .workplane(offset=BACK_X - 0.010)
        .circle(CRADLE_OUTER_R + 0.004)
        .circle(0.006)
        .extrude(0.012)
    )
    return plate


def _build_d_handle() -> cq.Workplane:
    """Inline rear D-shaped handle aligned with the barrel axis.

    Built as a rounded rectangular frame with an interior cutout (the D-hole).
    The front of the frame overlaps with the cradle and rear plate for
    structural connectivity. The rear grip section is the part the user holds.
    """
    wall_front = 0.018
    wall_rear = 0.014
    wall_top = 0.010
    wall_bot = 0.010
    y_wall = 0.004

    hx0 = HANDLE_FRONT_X   # front (+X side)
    hx1 = HANDLE_REAR_X    # rear (-X side)
    hz0 = HANDLE_BOT_Z     # bottom
    hz1 = HANDLE_TOP_Z     # top
    hy0 = -HANDLE_Y_HALF   # -Y side
    hy1 = HANDLE_Y_HALF    # +Y side

    # Outer solid block
    outer_cx = (hx0 + hx1) / 2.0
    outer_cz = (hz0 + hz1) / 2.0
    outer_wx = hx0 - hx1   # X extent (positive)
    outer_wz = hz1 - hz0   # Z extent (positive)

    outer = (
        cq.Workplane("XZ")
        .workplane(offset=hy0)
        .center(outer_cx, outer_cz)
        .rect(outer_wx, outer_wz)
        .extrude(hy1 - hy0)
        .edges("|Y")
        .fillet(0.005)
    )

    # Interior cutout (the D-hole for the hand)
    cx0 = hx0 - wall_front    # cutout front X
    cx1 = hx1 + wall_rear     # cutout rear X
    cz0 = hz0 + wall_bot      # cutout bottom Z
    cz1 = hz1 - wall_top      # cutout top Z
    cy0 = hy0 + y_wall        # cutout -Y
    cy1 = hy1 - y_wall        # cutout +Y

    cut_cx = (cx0 + cx1) / 2.0
    cut_cz = (cz0 + cz1) / 2.0
    cut_wx = cx0 - cx1        # positive
    cut_wz = cz1 - cz0        # positive

    inner = (
        cq.Workplane("XZ")
        .workplane(offset=cy0)
        .center(cut_cx, cut_cz)
        .rect(cut_wx, cut_wz)
        .extrude(cy1 - cy0)
    )

    handle = outer.cut(inner)
    return handle


def _build_pivot_lugs() -> cq.Workplane:
    """Two lugs rising from the D-handle top rail, carrying the trigger pivot.

    The lugs extend from deep inside the handle top rail up above it to the
    pivot pin height, straddling the trigger hub. A cross-bar connects the two
    lugs below the trigger hub for single-piece mesh connectivity.
    """
    lug_base_z = HANDLE_TOP_Z - 0.008   # deep into the top rail for connectivity
    lug_top_z = TRIG_PIVOT_Z + 0.005    # extend slightly above the pivot
    lug_height = lug_top_z - lug_base_z
    lug_cz = (lug_base_z + lug_top_z) / 2.0
    lug_x_len = 0.018
    lug_thickness = 0.005

    def side_lug(y_inner: float):
        return (
            cq.Workplane("XZ")
            .workplane(offset=y_inner)
            .center(TRIG_PIVOT_X, lug_cz)
            .rect(lug_x_len, lug_height)
            .extrude(lug_thickness)
            .edges("|Y")
            .fillet(0.002)
        )

    left_lug = side_lug(PIVOT_Y_HALF)
    right_lug = side_lug(-(PIVOT_Y_HALF + lug_thickness))

    # Cross-bar connecting both lugs below the trigger hub.
    # Sits at the lug base level (deep inside the handle top rail).
    bar_z = lug_base_z + 0.004
    bar_h = 0.006
    bar_span = 2 * PIVOT_Y_HALF + 2 * lug_thickness  # full width across both lugs
    bridge = (
        cq.Workplane("XZ")
        .workplane(offset=-(PIVOT_Y_HALF + lug_thickness))
        .center(TRIG_PIVOT_X, bar_z + bar_h / 2.0)
        .rect(lug_x_len, bar_h)
        .extrude(bar_span)
    )

    return left_lug.union(right_lug).union(bridge)


def _build_frame_mesh():
    """Assemble the complete fixed frame from sub-components."""
    frame = _build_cradle()
    frame = frame.union(_build_front_ring())
    frame = frame.union(_build_rear_plate())
    frame = frame.union(_build_d_handle())
    frame = frame.union(_build_pivot_lugs())
    return mesh_from_cadquery(frame, "frame")


def _build_pivot_pin_mesh():
    """Steel pivot pin spanning between the two trigger lugs."""
    pin = (
        cq.Workplane("XZ")
        .workplane(offset=PIVOT_Y_HALF + 0.004)
        .center(TRIG_PIVOT_X, TRIG_PIVOT_Z)
        .circle(0.003)
        .extrude(-(2 * PIVOT_Y_HALF + 0.008))
    )
    return mesh_from_cadquery(pin, "pivot_pin")


# ---------------------------------------------------------------------------
# Shared geometry helper for repeated handle rivets
# ---------------------------------------------------------------------------
def _build_rivet_mesh(x: float, z: float, name: str):
    """Small rivet/bolt head on the outer +Y face of the handle side plate."""
    rivet = (
        cq.Workplane("XZ")
        .workplane(offset=HANDLE_Y_HALF)
        .center(x, z)
        .circle(0.0025)
        .extrude(0.002)
    )
    return mesh_from_cadquery(rivet, name)


# ---------------------------------------------------------------------------
# Cartridge + nozzle (seated consumable)
# ---------------------------------------------------------------------------
def _build_cartridge_mesh():
    """Blue/gray cylindrical silicone cartridge body."""
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
    """Red front shoulder/collar tapering down to the nozzle."""
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
    """White tapered conical nozzle tip protruding from the front."""
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
# Trigger lever (thumb-squeeze, inline variant)
# ---------------------------------------------------------------------------
def _build_trigger_mesh():
    """Thumb-squeeze trigger lever. Authored with pivot at local origin.

    Local frame: pivot at origin, lever extends upward (local +Z). The hub
    wraps the pivot pin. When squeezed (positive q about +Y axis), the lever
    top tips forward (+X toward the nozzle).
    """
    lever_w = 0.014  # lever width in Y

    # Lever profile in XZ plane: wider near the top for the thumb pad.
    # Build centered in Y using a workplane offset.
    pts = [
        (0.005, -0.004),    # front base (overlaps hub for union)
        (0.006, 0.028),     # front mid
        (0.009, 0.038),     # front thumb pad (wider)
        (0.004, 0.046),     # front top
        (-0.004, 0.046),    # rear top
        (-0.009, 0.038),    # rear thumb pad
        (-0.006, 0.028),    # rear mid
        (-0.005, -0.004),   # rear base
    ]
    lever = (
        cq.Workplane("XZ")
        .workplane(offset=-lever_w / 2.0)
        .polyline(pts)
        .close()
        .extrude(lever_w)
        .edges("|Y")
        .fillet(0.003)
    )
    # Hub around pivot pin (centered cylinder at local origin)
    hub_r = 0.007
    hub_w = 0.020
    hub = (
        cq.Workplane("XZ")
        .workplane(offset=-hub_w / 2.0)
        .circle(hub_r)
        .extrude(hub_w)
    )
    trigger = lever.union(hub)
    return mesh_from_cadquery(trigger, "trigger")


# ---------------------------------------------------------------------------
# Plunger rod + push plate
# ---------------------------------------------------------------------------
def _build_plunger_mesh():
    """Push plate + long rod extending out the back, ending in a thumb hook."""
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
    model.material("handle_grey", rgba=(0.42, 0.43, 0.45, 1.0))

    # --- Fixed frame (root) ---
    frame = model.part("frame")
    frame.visual(_build_frame_mesh(), material="frame_red", name="frame_shell")
    frame.visual(_build_pivot_pin_mesh(), material="steel", name="pivot_pin")

    # Handle rivets along the top rail (repeated decoration via loop)
    n_rivets = 4
    for i in range(n_rivets):
        rx = BACK_X - 0.020 - i * 0.025
        rz = HANDLE_TOP_Z - 0.003
        frame.visual(
            _build_rivet_mesh(rx, rz, f"rivet_{i}"),
            material="steel",
            name=f"rivet_{i}",
        )

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

    # --- Trigger (revolute about Y at the pivot pin, thumb-squeeze lever) ---
    trigger = model.part("trigger")
    trigger.visual(_build_trigger_mesh(), material="handle_grey", name="trigger_blade")

    # Trigger mesh is authored with pivot at local origin. The lever extends
    # upward (local +Z). With axis=(0,1,0) and positive q, the lever top tips
    # toward +X (forward, toward the nozzle) — a natural thumb-push motion.
    model.articulation(
        "frame_to_trigger",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=trigger,
        origin=Origin(xyz=(TRIG_PIVOT_X, 0.0, TRIG_PIVOT_Z)),
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

    # D-handle extends rearward from the barrel (frame min X is well behind barrel rear).
    ctx.check(
        "D-handle extends well behind the barrel rear",
        frame_aabb[0][0] < BACK_X - 0.080,
        details=f"frame_min_x={frame_aabb[0][0]}, BACK_X={BACK_X}",
    )

    # D-handle is inline with the barrel: the frame extends roughly
    # symmetrically above and below Z=0 (not hanging down like a pistol grip).
    ctx.check(
        "D-handle is inline (handle top and bottom roughly symmetric about barrel axis)",
        abs(abs(frame_aabb[1][2]) - abs(frame_aabb[0][2])) < 0.020,
        details=f"frame_max_z={frame_aabb[1][2]}, frame_min_z={frame_aabb[0][2]}",
    )

    # Trigger lever sits above the barrel/handle junction.
    trig_aabb = ctx.part_world_aabb(trigger)
    ctx.check(
        "trigger lever sits above the barrel top",
        trig_aabb is not None and trig_aabb[0][2] > 0.0,
        details=f"trigger_min_z={trig_aabb[0][2]}",
    )

    # Cartridge seated inside the cradle.
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

    # Cartridge nested inside the open cradle (seated consumable fit).
    ctx.allow_overlap(
        cartridge,
        frame,
        elem_a="cartridge_body",
        elem_b="frame_shell",
        reason="Silicone cartridge is seated/nested inside the open half-pipe cradle of the frame.",
    )
    # Cartridge front shoulder seats into the front cap ring.
    ctx.allow_overlap(
        cartridge,
        frame,
        elem_a="front_collar",
        elem_b="frame_shell",
        reason="Cartridge front shoulder seats into and passes through the front cap ring bore.",
    )

    # Trigger hub wraps the pivot pin and sits between the pivot lugs ->
    # intentional capture overlap (hub is the bearing surface on the pin).
    ctx.allow_overlap(
        trigger,
        frame,
        elem_a="trigger_blade",
        elem_b="pivot_pin",
        reason="Trigger pivot hub intentionally captures the steel pivot pin.",
    )
    ctx.allow_overlap(
        trigger,
        frame,
        elem_a="trigger_blade",
        elem_b="frame_shell",
        reason="Trigger hub sits between the pivot lugs in the frame; small local capture for the revolute bearing.",
    )

    # Proof: trigger hub stays between the pivot lugs (within the lug Y span).
    ctx.expect_within(
        trigger,
        frame,
        axes="y",
        margin=0.006,
        name="trigger hub stays within the pivot lug Y span",
    )

    # Plunger rod slides inside the barrel cradle and through the rear plate bore.
    ctx.allow_overlap(
        plunger,
        frame,
        elem_a="plunger_rod",
        elem_b="frame_shell",
        reason="Plunger rod/plate intentionally slides inside the barrel cradle and through the rear plate bore.",
    )
    # Push plate bears on the rear of the cartridge contents.
    ctx.allow_overlap(
        plunger,
        cartridge,
        elem_a="plunger_rod",
        elem_b="cartridge_body",
        reason="Push plate bears on the rear of the cartridge contents (simplified solid cartridge proxy).",
    )

    # --- Mechanism motion checks ---
    # Squeezing the thumb trigger tips its lever top forward toward the nozzle
    # (+X direction), pivoting about the top pin.
    trig_rest = ctx.part_world_aabb(trigger)
    with ctx.pose({trig_joint: 0.45}):
        trig_squeezed = ctx.part_world_aabb(trigger)
    ctx.check(
        "squeezing the trigger tips its lever top forward toward the nozzle (+X)",
        trig_rest is not None
        and trig_squeezed is not None
        and trig_squeezed[1][0] > trig_rest[1][0] + 0.010,
        details=f"rest_max_x={trig_rest[1][0]}, squeezed_max_x={trig_squeezed[1][0]}",
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
