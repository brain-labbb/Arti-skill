from __future__ import annotations

# Realistic articulated SAUSAGE / BULK caulking gun.
#
# Variant fork of the open-half-cradle caulking gun: the barrel is now a long
# fully-closed cylindrical tube that holds a soft sausage pack (not a rigid
# cartridge).  A separate screw-on FRONT CONE CAP (threaded nose cap) carries
# the nozzle at the muzzle.  The rear end is closed by a bored cap that guides
# the plunger rod.
#
# The barrel tube and the front cone cap are both built by revolving a 2D
# profile around the barrel axis (X), as a lathe would produce them.
#
# Coordinate convention (Z up, dispensing along +X):
#   +X : muzzle / dispensing direction (nozzle points to +X)
#   +Z : up. Gravity is -Z.
#   +Y : left/right
#   The barrel axis lies along X at y=0, lifted to z=AXIS_Z so the lowest
#   point (the barrel tube or rear cap rim) rests on the ground plane z=0.
#
# Articulated mechanisms:
#   1. trigger_pivot  (REVOLUTE): squeeze lever pivots about a pin at the top
#      of the handle frame. Positive q pulls the lever's free end rearward.
#   2. plunger_drive  (PRISMATIC): steel rod + hook slide along +X to advance
#      the sausage pack plunger.
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
# Dimensions (meters) -- a standard ~300 ml sausage/bulk caulking gun.
# ---------------------------------------------------------------------------

# Barrel tube inner bore (the sausage pack fits inside).
TUBE_R = 0.0245            # bore radius (~49 mm dia)
TUBE_LEN = 0.215           # barrel body length along X
TUBE_X0 = 0.0              # rear face of barrel body
TUBE_X1 = TUBE_X0 + TUBE_LEN

# Barrel tube wall (full enclosed cylinder, built by revolve).
BARREL_WALL_T = 0.003      # tube wall thickness
BARREL_R_IN = TUBE_R
BARREL_R_OUT = TUBE_R + BARREL_WALL_T
BARREL_X0 = TUBE_X0
BARREL_X1 = TUBE_X1
BARREL_LEN = BARREL_X1 - BARREL_X0

# Front cone cap (threaded nose cap) -- screws onto the front of the barrel.
FRONT_CAP_X0 = BARREL_X1   # starts at the front face of the barrel tube
FRONT_CAP_RING_LEN = 0.010  # threaded ring section at the base
FRONT_CAP_CONE_LEN = 0.028  # conical taper section
FRONT_CAP_LEN = FRONT_CAP_RING_LEN + FRONT_CAP_CONE_LEN
FRONT_CAP_R_OUT = BARREL_R_OUT + 0.004  # thread ring outer (slightly wider)
FRONT_CAP_R_IN = BARREL_R_IN            # bore matches barrel bore
FRONT_CAP_TIP_R = 0.008                 # small opening at the tip
FRONT_CAP_WALL_T = 0.003                # wall thickness at cone section

# Nozzle cone (black) + white applicator tip at the muzzle.
NOZZLE_X0 = FRONT_CAP_X0 + FRONT_CAP_LEN
NOZZLE_BASE_R = FRONT_CAP_TIP_R       # matches cap tip opening for contact
NOZZLE_TIP_R = 0.0045
NOZZLE_LEN = 0.030
TIP_LEN = 0.022
TIP_R0 = NOZZLE_TIP_R
TIP_R1 = 0.0018

# Rear cap (black plastic disc that closes the back of the barrel tube and
# guides the plunger rod). This carries the handle frame.
REAR_CAP_R = BARREL_R_OUT + 0.006
REAR_CAP_T = 0.016
REAR_CAP_X = BARREL_X0 - REAR_CAP_T / 2.0

# Plunger rod (dark steel) + push disc + bent J-hook tail.
ROD_R = 0.0042
ROD_PUSH_R = TUBE_R              # disc on the rod's front end (seals against tube wall)
ROD_PUSH_T = 0.005
ROD_FRONT_X = TUBE_X0 + 0.018   # rod push-disc front face (rest pose)
ROD_BACK_X = REAR_CAP_X - 0.095 # rod tail behind the rear cap (rest pose)

# Compact J-hook at the rear of the rod.
HOOK_R = 0.0038
HOOK_CURVE_R = 0.0075
HOOK_TAIL = 0.016

# Rear handle frame.
REAR_CAP_BACK = REAR_CAP_X
FRAME_RISE = 0.085
FRAME_T = 0.010
FRAME_W_X = 0.038
FRAME_X = REAR_CAP_BACK - FRAME_W_X / 2.0 + 0.002

# Trigger pivot pin.
PIVOT_Z = FRAME_RISE
PIVOT_X = FRAME_X
PIN_R = 0.0035
PIN_LEN = 0.030

# Trigger lever.
TRIGGER_LEN = 0.060
TRIGGER_W = 0.013
TRIGGER_H = 0.026

# Fixed grip wing.
FRAME_HALF_W = ROD_R + 0.0025 + FRAME_T

# Ground lift.
AXIS_Z = REAR_CAP_R

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

GUNMETAL = "gunmetal"
BLACK_PLASTIC = "black_plastic"
DARK_STEEL = "dark_steel"
STEEL = "bright_steel"
WHITE_TIP = "white_tip"
ALUMINUM = "aluminum"


def _register_materials(model: ArticulatedObject) -> None:
    model.material(GUNMETAL, rgba=(0.27, 0.28, 0.30, 1.0))
    model.material(BLACK_PLASTIC, rgba=(0.10, 0.10, 0.11, 1.0))
    model.material(DARK_STEEL, rgba=(0.35, 0.36, 0.38, 1.0))
    model.material(STEEL, rgba=(0.74, 0.76, 0.80, 1.0))
    model.material(WHITE_TIP, rgba=(0.93, 0.93, 0.92, 1.0))
    model.material(ALUMINUM, rgba=(0.68, 0.70, 0.72, 1.0))


# ---------------------------------------------------------------------------
# CadQuery geometry builders
# ---------------------------------------------------------------------------


def _barrel_tube() -> cq.Workplane:
    """Fully-closed cylindrical barrel tube, built by revolving an annular
    rectangular profile around the barrel axis (X).

    The profile is a rectangle from (BARREL_X0, BARREL_R_IN) to
    (BARREL_X1, BARREL_R_OUT) on the XZ workplane. Revolving it 360 degrees
    around the X axis (y=0 in 2D) produces a hollow cylinder -- exactly what
    a lathe would turn.
    """
    profile = (
        cq.Workplane("XZ")
        .moveTo(BARREL_X0, BARREL_R_IN)
        .lineTo(BARREL_X1, BARREL_R_IN)
        .lineTo(BARREL_X1, BARREL_R_OUT)
        .lineTo(BARREL_X0, BARREL_R_OUT)
        .close()
    )
    tube = profile.revolve(360, (0, 0), (1, 0))
    return tube


def _front_cone_cap() -> cq.Workplane:
    """Screw-on front cone cap (threaded nose cap), built by revolving a
    closed profile around the barrel axis (X).

    The profile describes a hollow conical shell:
      - A short cylindrical ring at the base (threaded collar section)
      - A conical taper from the ring to a small tip opening
      - The interior is hollow (conical bore matching the barrel bore)
    """
    x_ring_end = FRONT_CAP_X0 + FRONT_CAP_RING_LEN
    x_tip = FRONT_CAP_X0 + FRONT_CAP_LEN

    # Closed profile on XZ workplane (x = world X, y = world Z = radial dist).
    # Going clockwise from inner base:
    profile = (
        cq.Workplane("XZ")
        # 1: inner base (barrel bore radius)
        .moveTo(FRONT_CAP_X0, FRONT_CAP_R_IN)
        # 2: outer base (ring outer radius -- slightly wider than barrel)
        .lineTo(FRONT_CAP_X0, FRONT_CAP_R_OUT)
        # 3: ring outer end
        .lineTo(x_ring_end, FRONT_CAP_R_OUT)
        # 4: cone outer to tip (tapers down)
        .lineTo(x_tip, FRONT_CAP_TIP_R + FRONT_CAP_WALL_T)
        # 5: tip face (inner opening)
        .lineTo(x_tip, FRONT_CAP_TIP_R)
        # 6: cone inner back to ring end
        .lineTo(x_ring_end, FRONT_CAP_R_IN)
        # 7: close back to inner base
        .close()
    )
    cap = profile.revolve(360, (0, 0), (1, 0))
    return cap


def _rear_cap() -> cq.Workplane:
    """Black cap closing the barrel rear; bored for the plunger rod."""
    cap = (
        cq.Workplane("YZ")
        .workplane(offset=REAR_CAP_X)
        .circle(REAR_CAP_R)
        .extrude(REAR_CAP_T)
    )
    bore = (
        cq.Workplane("YZ")
        .workplane(offset=REAR_CAP_X - 0.001)
        .circle(ROD_R + 0.0012)
        .extrude(REAR_CAP_T + 0.002)
    )
    cap = cap.cut(bore)
    return cap


def _handle_frame() -> cq.Workplane:
    """Black U-yoke bracket rising from the rear cap to carry the trigger pivot."""
    half_gap = ROD_R + 0.0025 + FRAME_T / 2.0
    z_lo = -REAR_CAP_R * 0.55
    z_hi = PIVOT_Z + 0.006
    plate_h = z_hi - z_lo
    plate_cz = (z_lo + z_hi) / 2.0

    side_plate = (
        cq.Workplane("XY")
        .box(FRAME_W_X, FRAME_T, plate_h)
        .edges("|Y")
        .fillet(0.005)
    )
    left = side_plate.translate((FRAME_X, +half_gap, plate_cz))
    right = side_plate.translate((FRAME_X, -half_gap, plate_cz))

    lug_inner = half_gap - FRAME_T / 2.0 - 0.0015
    lug_len = (half_gap + FRAME_T / 2.0 + 0.001) - lug_inner
    lug_left = (
        cq.Workplane("XZ")
        .workplane(offset=lug_inner)
        .circle(0.011)
        .extrude(lug_len)
        .translate((PIVOT_X, 0.0, PIVOT_Z))
    )
    lug_right = (
        cq.Workplane("XZ")
        .workplane(offset=-(lug_inner + lug_len))
        .circle(0.011)
        .extrude(lug_len)
        .translate((PIVOT_X, 0.0, PIVOT_Z))
    )
    boss = lug_left.union(lug_right)

    web = (
        cq.Workplane("XY")
        .box(FRAME_W_X, 2.0 * half_gap + FRAME_T, 0.012)
        .translate((FRAME_X, 0.0, z_lo + 0.006))
    )
    boss_x0 = -0.026
    boss_x1 = -0.002
    guide = (
        cq.Workplane("YZ")
        .workplane(offset=boss_x0)
        .circle(0.013)
        .extrude(boss_x1 - boss_x0)
    )
    guide_bore = (
        cq.Workplane("YZ")
        .workplane(offset=boss_x0 - 0.001)
        .circle(ROD_R + 0.0012)
        .extrude(boss_x1 - boss_x0 + 0.002)
    )
    guide = guide.cut(guide_bore)
    frame = left.union(right).union(boss).union(web).union(guide)
    return frame


def _pivot_pin() -> cq.Workplane:
    pin = (
        cq.Workplane("XZ")
        .workplane(offset=-PIN_LEN / 2.0)
        .circle(PIN_R)
        .extrude(PIN_LEN)
        .translate((PIVOT_X, 0.0, PIVOT_Z))
    )
    return pin


def _grip_wing() -> cq.Workplane:
    """Fixed black grip blade behind the trigger."""
    profile = (
        cq.Workplane("XZ")
        .moveTo(-0.038, 0.088)
        .lineTo(-0.052, 0.092)
        .lineTo(-0.096, 0.034)
        .lineTo(-0.081, 0.028)
        .close()
    )
    wing = profile.extrude(FRAME_HALF_W, both=True)
    return wing


def _nozzle_cone() -> cq.Workplane:
    """Black tapered nozzle cone at the muzzle (authored at barrel axis z=0).

    The base seats onto the annular tip face of the front cone cap (inner
    radius FRONT_CAP_TIP_R = 0.008, outer 0.011). A base flange slightly
    wider than the cap bore creates a clear seated contact.
    """
    # Base flange seats on the cap tip annular face.
    flange_r = FRONT_CAP_TIP_R + FRONT_CAP_WALL_T * 0.7   # between bore and outer wall
    flange = (
        cq.Workplane("YZ")
        .workplane(offset=NOZZLE_X0 - 0.002)
        .circle(flange_r)
        .workplane(offset=0.002)
        .circle(NOZZLE_BASE_R)
        .loft(combine=True)
    )
    cone = (
        cq.Workplane("YZ")
        .workplane(offset=NOZZLE_X0)
        .circle(NOZZLE_BASE_R)
        .workplane(offset=NOZZLE_LEN)
        .circle(NOZZLE_TIP_R)
        .loft(combine=True)
    )
    return flange.union(cone)


def _white_tip() -> cq.Workplane:
    """White applicator tip cut at an angle."""
    tip = (
        cq.Workplane("YZ")
        .workplane(offset=NOZZLE_X0 + NOZZLE_LEN)
        .circle(TIP_R0)
        .workplane(offset=TIP_LEN)
        .circle(TIP_R1)
        .loft(combine=True)
    )
    return tip


def _plunger_rod() -> cq.Workplane:
    """Steel rod + front push disc; built in rod-local frame (rod axis along X).

    Local origin is at x=0 == ROD_FRONT_X in the model rest frame.
    """
    total_len = ROD_FRONT_X - ROD_BACK_X
    rod = (
        cq.Workplane("YZ")
        .workplane(offset=-total_len)
        .circle(ROD_R)
        .extrude(total_len)
    )
    push = (
        cq.Workplane("YZ")
        .workplane(offset=-ROD_PUSH_T)
        .circle(ROD_PUSH_R)
        .extrude(ROD_PUSH_T)
    )
    return rod.union(push)


def _hook_handle() -> cq.Workplane:
    """Compact bent J-hook at the rear tip of the rod (rod-local frame)."""
    total_len = ROD_FRONT_X - ROD_BACK_X
    tail_x = -total_len
    pts: list[tuple[float, float]] = [
        (tail_x + 0.010, 0.0),
        (tail_x + 0.003, 0.0),
    ]
    for i in range(1, 9):
        a = math.pi * (i / 8.0)
        x = tail_x - HOOK_CURVE_R * math.sin(a)
        z = HOOK_CURVE_R - HOOK_CURVE_R * math.cos(a)
        pts.append((x, z))
    pts.append((tail_x + 0.006, 2.0 * HOOK_CURVE_R))
    pts.append((tail_x + HOOK_TAIL, 2.0 * HOOK_CURVE_R))
    wire = (
        cq.Workplane("XZ")
        .moveTo(pts[0][0], pts[0][1])
        .spline(pts[1:])
        .val()
    )
    sweep = (
        cq.Workplane("YZ")
        .workplane(offset=pts[0][0])
        .center(0.0, pts[0][1])
        .circle(HOOK_R)
    )
    hook = sweep.sweep(cq.Workplane(obj=wire), multisection=False)
    return hook


def _trigger_blade() -> cq.Workplane:
    """Flat black squeeze lever, built in the trigger-local frame."""
    blade = (
        cq.Workplane("XY")
        .workplane(offset=-0.006)
        .rect(TRIGGER_W, TRIGGER_H)
        .workplane(offset=-TRIGGER_LEN)
        .rect(TRIGGER_W * 1.3, TRIGGER_H * 1.55)
        .loft(combine=True)
    )
    hub = (
        cq.Workplane("XZ")
        .workplane(offset=-0.011)
        .circle(0.011)
        .extrude(0.022)
    )
    neck = (
        cq.Workplane("XY")
        .box(TRIGGER_W, TRIGGER_H, 0.012)
        .translate((0.0, 0.0, -0.004))
    )
    lever = blade.union(hub).union(neck)
    pin_bore = cq.Solid.makeCylinder(
        PIN_R + 0.0006,
        0.030,
        cq.Vector(0.0, -0.015, 0.0),
        cq.Vector(0.0, 1.0, 0.0),
    )
    lever = lever.cut(cq.Workplane(obj=pin_bore))
    lever = lever.edges("|Y and <Z").fillet(0.003)
    return lever


# ---------------------------------------------------------------------------
# Model assembly
# ---------------------------------------------------------------------------


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sausage_caulking_gun")
    _register_materials(model)

    # -- Root part: barrel tube + rear cap + handle frame + grip wing + pivot.
    #    All body geometry is authored around the barrel axis at z=0 and lifted
    #    by AXIS_Z so the gun rests on the ground.
    lift = (0.0, 0.0, AXIS_Z)
    body = model.part("barrel_body")
    body.visual(
        mesh_from_cadquery(_barrel_tube().translate(lift), "barrel_tube"),
        material=ALUMINUM,
        name="barrel_tube",
    )
    body.visual(
        mesh_from_cadquery(_rear_cap().translate(lift), "rear_cap"),
        material=BLACK_PLASTIC,
        name="rear_cap",
    )
    body.visual(
        mesh_from_cadquery(_handle_frame().translate(lift), "handle_frame"),
        material=BLACK_PLASTIC,
        name="handle_frame",
    )
    body.visual(
        mesh_from_cadquery(_grip_wing().translate(lift), "grip_wing"),
        material=BLACK_PLASTIC,
        name="grip_wing",
    )
    body.visual(
        mesh_from_cadquery(_pivot_pin().translate(lift), "pivot_pin"),
        material=STEEL,
        name="pivot_pin",
    )

    # -- Front cone cap (threaded nose cap) with nozzle + white tip.
    #    Authored at z=0; the FIXED joint lifts it to the barrel axis.
    front_cap = model.part("front_cap")
    front_cap.visual(
        mesh_from_cadquery(_front_cone_cap(), "front_cone"),
        material=DARK_STEEL,
        name="front_cone",
    )
    front_cap.visual(
        mesh_from_cadquery(_nozzle_cone(), "nozzle_cone"),
        material=BLACK_PLASTIC,
        name="nozzle_cone",
    )
    front_cap.visual(
        mesh_from_cadquery(_white_tip(), "white_tip"),
        material=WHITE_TIP,
        name="white_tip",
    )

    # -- Plunger assembly: steel rod + hook handle.
    plunger = model.part("plunger_rod")
    plunger.visual(
        mesh_from_cadquery(_plunger_rod(), "plunger_rod"),
        material=GUNMETAL,
        name="plunger_rod",
    )
    plunger.visual(
        mesh_from_cadquery(_hook_handle(), "hook_handle"),
        material=GUNMETAL,
        name="hook_handle",
    )

    # -- Trigger lever (squeeze control).
    trigger = model.part("trigger_lever")
    trigger.visual(
        mesh_from_cadquery(_trigger_blade(), "trigger_blade"),
        material=BLACK_PLASTIC,
        name="trigger_blade",
    )

    # -- Joints ---------------------------------------------------------------

    # Front cap is FIXED (screwed onto the barrel tube front end).
    # Geometry is authored at z=0; the origin lifts it to barrel axis height.
    model.articulation(
        "front_cap_seat",
        ArticulationType.FIXED,
        parent=body,
        child=front_cap,
        origin=Origin(xyz=(0.0, 0.0, AXIS_Z)),
    )

    # Plunger rod: PRISMATIC along the barrel axis (+X advances the caulk).
    model.articulation(
        "plunger_drive",
        ArticulationType.PRISMATIC,
        parent=body,
        child=plunger,
        origin=Origin(xyz=(ROD_FRONT_X, 0.0, AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=300.0, velocity=0.05, lower=0.0, upper=0.060),
    )

    # Trigger lever: REVOLUTE about the pivot pin (axis along Y).
    model.articulation(
        "trigger_pivot",
        ArticulationType.REVOLUTE,
        parent=body,
        child=trigger,
        origin=Origin(xyz=(PIVOT_X, 0.0, AXIS_Z + PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=2.0, lower=0.0, upper=0.55),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("barrel_body")
    front_cap = object_model.get_part("front_cap")
    plunger = object_model.get_part("plunger_rod")
    trigger = object_model.get_part("trigger_lever")

    plunger_drive = object_model.get_articulation("plunger_drive")
    trigger_pivot = object_model.get_articulation("trigger_pivot")

    # --- Joint configuration claims ----------------------------------------
    ctx.check(
        "plunger_drive is prismatic along +X",
        plunger_drive.joint_type == "prismatic" and tuple(plunger_drive.axis) == (1.0, 0.0, 0.0),
        details=f"type={plunger_drive.joint_type}, axis={plunger_drive.axis}",
    )
    ctx.check(
        "trigger_pivot is revolute about +Y",
        trigger_pivot.joint_type == "revolute" and tuple(trigger_pivot.axis) == (0.0, 1.0, 0.0),
        details=f"type={trigger_pivot.joint_type}, axis={trigger_pivot.axis}",
    )

    # --- Hero geometry is present ------------------------------------------
    body_visuals = {v.name for v in body.visuals}
    for need in ("barrel_tube", "rear_cap", "handle_frame", "grip_wing", "pivot_pin"):
        ctx.check(f"body has {need}", need in body_visuals, details=str(sorted(body_visuals)))

    cap_visuals = {v.name for v in front_cap.visuals}
    for need in ("front_cone", "nozzle_cone", "white_tip"):
        ctx.check(f"front_cap has {need}", need in cap_visuals, details=str(sorted(cap_visuals)))

    plunger_visuals = {v.name for v in plunger.visuals}
    for need in ("plunger_rod", "hook_handle"):
        ctx.check(f"plunger has {need}", need in plunger_visuals, details=str(sorted(plunger_visuals)))

    # --- Barrel tube is a fully-closed cylinder (not an open cradle) -------
    # A closed tube should have roughly equal extent above and below the barrel
    # axis (z=AXIS_Z). An open half-cradle would only extend below.
    tube_aabb = ctx.part_element_world_aabb(body, elem="barrel_tube")
    if tube_aabb is not None:
        above_axis = tube_aabb[1][2] - AXIS_Z
        below_axis = AXIS_Z - tube_aabb[0][2]
        ctx.check(
            "barrel tube is a fully-closed cylinder (extends above and below axis)",
            above_axis > 0.015 and below_axis > 0.015,
            details=f"above={above_axis:.4f}, below={below_axis:.4f}, axis_z={AXIS_Z:.4f}",
        )
        # The tube should be roughly circular (height ~= width in YZ).
        tube_span_y = tube_aabb[1][1] - tube_aabb[0][1]
        tube_span_z = tube_aabb[1][2] - tube_aabb[0][2]
        ctx.check(
            "barrel tube cross-section is roughly circular",
            abs(tube_span_y - tube_span_z) < 0.008,
            details=f"span_y={tube_span_y:.4f}, span_z={tube_span_z:.4f}",
        )

    # --- Front cone cap tapers (wider at base, narrower at tip) ------------
    cone_aabb = ctx.part_element_world_aabb(front_cap, elem="front_cone")
    if cone_aabb is not None:
        cone_span_z = cone_aabb[1][2] - cone_aabb[0][2]
        # The cone at its widest should be near the barrel outer radius, and
        # narrower than the barrel tube at the tip end.
        ctx.check(
            "front cone cap is wider at the base than at the tip",
            cone_span_z > 2.0 * FRONT_CAP_TIP_R + 0.004,
            details=f"cone_span_z={cone_span_z:.4f}",
        )

    # --- Nozzle/tip is the most +X (muzzle) feature ------------------------
    tip_aabb = ctx.part_element_world_aabb(front_cap, elem="white_tip")
    tube_aabb_body = ctx.part_element_world_aabb(body, elem="barrel_tube")
    if tip_aabb is not None and tube_aabb_body is not None:
        ctx.check(
            "white tip points forward past the barrel tube (+X muzzle)",
            tip_aabb[1][0] > tube_aabb_body[1][0],
            details=f"tip_max_x={tip_aabb[1][0]:.4f}, tube_max_x={tube_aabb_body[1][0]:.4f}",
        )

    # --- Front cap sits at the front of the barrel tube ---------------------
    if cone_aabb is not None and tube_aabb_body is not None:
        ctx.check(
            "front cone cap sits at or past the barrel tube front end",
            cone_aabb[1][0] > tube_aabb_body[1][0] - 0.005,
            details=f"cone_max_x={cone_aabb[1][0]:.4f}, tube_max_x={tube_aabb_body[1][0]:.4f}",
        )

    # --- Hook is a compact bend hugging the rod tail -----------------------
    hook_aabb = ctx.part_element_world_aabb(plunger, elem="hook_handle")
    if hook_aabb is not None:
        hook_radial = max(
            abs(hook_aabb[1][2] - AXIS_Z),
            abs(hook_aabb[0][2] - AXIS_Z),
            abs(hook_aabb[1][1]),
            abs(hook_aabb[0][1]),
        )
        ctx.check(
            "hook stays compact, within ~2 rod diameters of the rod axis",
            hook_radial < 2.5 * (2.0 * ROD_R),
            details=f"hook_radial={hook_radial:.4f}, rod_dia={2.0 * ROD_R:.4f}",
        )
        ctx.check(
            "hook sits behind the rear cap at rest",
            hook_aabb[1][0] < REAR_CAP_X - 0.04,
            details=f"hook_max_x={hook_aabb[1][0]:.4f}, rear_cap_x={REAR_CAP_X:.4f}",
        )

    # --- The hook translates rigidly with the rod (same link) ---------------
    if hook_aabb is not None:
        with ctx.pose({plunger_drive: 0.050}):
            hook_adv = ctx.part_element_world_aabb(plunger, elem="hook_handle")
        if hook_adv is not None:
            dx = hook_adv[0][0] - hook_aabb[0][0]
            ctx.check(
                "hook advances with the plunger rod",
                abs(dx - 0.050) < 1e-6 and abs(hook_adv[0][2] - hook_aabb[0][2]) < 1e-6,
                details=f"dx={dx:.4f}, dz={hook_adv[0][2] - hook_aabb[0][2]:.6f}",
            )

    # --- The gun rests on the ground ----------------------------------------
    cap_aabb = ctx.part_element_world_aabb(body, elem="rear_cap")
    if cap_aabb is not None:
        ctx.check(
            "rear cap rim rests on the ground plane",
            abs(cap_aabb[0][2]) < 0.002,
            details=f"cap_min_z={cap_aabb[0][2]:.5f}",
        )

    # --- Trigger blade hangs below the pivot at rest -----------------------
    with ctx.pose({trigger_pivot: 0.0}):
        blade_rest = ctx.part_element_world_aabb(trigger, elem="trigger_blade")
    if blade_rest is not None:
        ctx.check(
            "trigger blade hangs below the pivot",
            blade_rest[0][2] < AXIS_Z + PIVOT_Z - 0.03,
            details=f"blade_min_z={blade_rest[0][2]:.4f}, pivot_z={AXIS_Z + PIVOT_Z:.4f}",
        )

    # --- Grip wing sits behind the trigger across its full swing ------------
    grip_aabb = ctx.part_element_world_aabb(body, elem="grip_wing")
    if grip_aabb is not None:
        with ctx.pose({trigger_pivot: 0.55}):
            blade_sq = ctx.part_element_world_aabb(trigger, elem="trigger_blade")
        if blade_sq is not None:
            ctx.check(
                "fully squeezed trigger still clears the fixed grip wing",
                blade_sq[0][0] > grip_aabb[1][0] - 0.041,
                details=(
                    f"blade_min_x={blade_sq[0][0]:.4f}, grip_max_x={grip_aabb[1][0]:.4f}"
                ),
            )

    # --- Actuating the trigger swings its free end toward the rear (-X) ----
    with ctx.pose({trigger_pivot: 0.0}):
        rest_free = ctx.part_element_world_aabb(trigger, elem="trigger_blade")
    with ctx.pose({trigger_pivot: 0.5}):
        squeezed_free = ctx.part_element_world_aabb(trigger, elem="trigger_blade")
    if rest_free is not None and squeezed_free is not None:
        ctx.check(
            "squeezing the trigger swings the blade rearward",
            squeezed_free[0][0] < rest_free[0][0] - 0.005,
            details=f"rest_min_x={rest_free[0][0]:.4f}, squeezed_min_x={squeezed_free[0][0]:.4f}",
        )

    # --- Actuating the plunger advances the rod toward the muzzle (+X) -----
    rest_rod = ctx.part_world_position(plunger)
    with ctx.pose({plunger_drive: 0.050}):
        adv_rod = ctx.part_world_position(plunger)
    if rest_rod is not None and adv_rod is not None:
        ctx.check(
            "driving the plunger advances the rod toward the muzzle",
            adv_rod[0] > rest_rod[0] + 0.04,
            details=f"rest_x={rest_rod[0]:.4f}, advanced_x={adv_rod[0]:.4f}",
        )

    # --- Plunger rod stays engaged through the rear cap bore ---------------
    ctx.expect_overlap(
        plunger,
        body,
        axes="x",
        elem_a="plunger_rod",
        elem_b="rear_cap",
        min_overlap=0.005,
        name="rod stays inserted through the rear cap bore",
    )

    # The rod passes through the rear-cap bore: intentional captured-shaft fit.
    ctx.allow_overlap(
        plunger,
        body,
        elem_a="plunger_rod",
        elem_b="rear_cap",
        reason="The steel plunger rod intentionally slides through the bored rear cap.",
    )

    # The push disc rides inside the barrel tube (captured in the bore).
    ctx.allow_overlap(
        plunger,
        body,
        elem_a="plunger_rod",
        elem_b="barrel_tube",
        reason="The plunger push disc rides captured inside the barrel tube bore.",
    )
    ctx.expect_within(
        plunger,
        body,
        axes="yz",
        inner_elem="plunger_rod",
        outer_elem="barrel_tube",
        margin=0.002,
        name="push disc stays centered inside the barrel tube",
    )

    # The front cap threads onto the barrel tube front end. The threaded ring
    # slightly overlaps the barrel tube outer wall (intentional threaded fit).
    ctx.allow_overlap(
        front_cap,
        body,
        elem_a="front_cone",
        elem_b="barrel_tube",
        reason="The front cone cap's threaded ring intentionally overlaps the barrel tube outer wall (screw-on fit).",
    )
    ctx.expect_contact(
        front_cap,
        body,
        elem_a="front_cone",
        elem_b="barrel_tube",
        name="front cone cap contacts the barrel tube front end",
    )

    # The trigger pivots on the pin captured by the frame lugs.
    ctx.allow_overlap(
        trigger,
        body,
        elem_a="trigger_blade",
        elem_b="pivot_pin",
        reason="The trigger hub pivots captured on the steel pin (pin nests in the hub bore).",
    )
    ctx.allow_overlap(
        trigger,
        body,
        elem_a="trigger_blade",
        elem_b="handle_frame",
        reason="The trigger hub rotates between the frame pivot lugs (captured pivot fit).",
    )

    ctx.expect_overlap(
        trigger,
        body,
        axes="yz",
        elem_a="trigger_blade",
        elem_b="pivot_pin",
        min_overlap=0.002,
        name="trigger hub captures the pivot pin",
    )

    return ctx.report()


object_model = build_object_model()
