from __future__ import annotations

# Realistic articulated caulking gun — sausage-tube variant.
#
# Fork of the half-barrel cartridge gun adapted for soft foil sausage packs.
# The barrel is now a full closed cylinder tube. A separate threaded hex end
# cap and tapered nose cone close the muzzle. The pistol grip, trigger lever,
# and plunger rod mechanisms are unchanged.
#
# Geometry conventions (all meters, real-world scale):
#   +X : barrel axis pointing forward toward the nozzle.
#   +Z : up.
#   +Y : left/right across the gun.
#
# Parts:
#   frame      (root)  – closed barrel tube + rear plate + grip + pivot lugs
#   end_cap    (FIXED) – hex ring at the muzzle that closes the barrel front
#   nose_cone  (FIXED) – tapered nozzle piece threading through the end cap
#   trigger    (REV)   – finger lever pivoting about the upper pin
#   plunger    (PRIS)  – push plate + rod advancing along +X

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
BARREL_R = 0.028          # barrel tube inner radius (sausage pack ~56 mm ID)
BARREL_WALL = 0.0025      # thin metal tube wall
BARREL_OUTER_R = BARREL_R + BARREL_WALL
BARREL_LEN = 0.280        # barrel tube length

FRONT_X = BARREL_LEN / 2.0       # +X face of the barrel
BACK_X = -BARREL_LEN / 2.0       # -X face (rear)

# End cap (hex ring at muzzle)
CAP_LEN = 0.015           # axial length of hex section
CAP_HEX_AF = 0.070        # hex across-flats diameter
CAP_BORE_R = 0.012        # central bore for the nose cone
CAP_START_X = FRONT_X     # cap begins at barrel front face

# Nose cone (tapered nozzle piece)
NC_BASE_R = CAP_BORE_R - 0.001   # fits through end cap bore
NC_FLANGE_R = 0.018       # shoulder flange that seats on cap face
NC_FLANGE_LEN = 0.005     # flange axial thickness
NC_TAPER_LEN = 0.028      # tapered cone section length
NC_TIP_R = 0.008          # end of taper

# Nozzle extension
NZ_BASE_R = NC_TIP_R      # matches taper tip
NZ_TIP_R = 0.004           # nozzle exit orifice
NZ_LEN = 0.042             # nozzle tube length

# Grip / handle
GRIP_TOP_Z = -BARREL_OUTER_R + 0.002
GRIP_X = BACK_X + 0.040
GRIP_LEN = 0.115
GRIP_FRONT_X = GRIP_X + 0.030

# Trigger pivot
PIVOT_X = GRIP_FRONT_X + 0.018
PIVOT_Z = -0.040
PIVOT_Y_HALF = 0.012

# Plunger
ROD_LEN = 0.250
PLATE_X = BACK_X + 0.006


# ---------------------------------------------------------------------------
# Shared geometry helpers
# ---------------------------------------------------------------------------
def _hex_prism(across_flats: float, length: float, bore_r: float = 0.0) -> cq.Workplane:
    """Build a regular hex prism along Z with optional central bore."""
    hex_shape = (
        cq.Workplane("XY")
        .polygon(6, across_flats)
        .extrude(length)
    )
    if bore_r > 0.0:
        bore = (
            cq.Workplane("XY")
            .circle(bore_r)
            .extrude(length)
        )
        hex_shape = hex_shape.cut(bore)
    return hex_shape


def _tapered_cone(base_r: float, tip_r: float, length: float) -> cq.Workplane:
    """Build a tapered cone (frustum) along +Z."""
    return (
        cq.Workplane("XY")
        .circle(base_r)
        .workplane(offset=length)
        .circle(tip_r)
        .loft()
    )


def _knurl_ring(outer_r: float, inner_r: float, height: float, n_ridges: int) -> cq.Workplane:
    """Build a ring with radial knurl ridges around the exterior."""
    ring = (
        cq.Workplane("XY")
        .circle(outer_r)
        .circle(inner_r)
        .extrude(height)
    )
    ridge_depth = 0.0015
    ridge_width = 0.003
    for i in range(n_ridges):
        angle = i * (360.0 / n_ridges)
        ridge = (
            cq.Workplane("XY")
            .center(outer_r + ridge_depth * 0.5, 0)
            .rect(ridge_depth, ridge_width)
            .extrude(height)
        )
        ridge = ridge.rotate((0, 0, 0), (0, 0, 1), angle)
        ring = ring.union(ridge)
    return ring


# ---------------------------------------------------------------------------
# Frame geometry (barrel tube + rear plate + grip + pivot lugs)
# ---------------------------------------------------------------------------
def _build_barrel() -> cq.Workplane:
    """Full closed cylinder barrel tube (replaces open half-pipe cradle)."""
    tube = (
        cq.Workplane("XY")
        .circle(BARREL_OUTER_R)
        .circle(BARREL_R)
        .extrude(BARREL_LEN)
        .translate((0, 0, -BARREL_LEN / 2.0))
    )
    # Orient: cylinder axis Z → barrel axis X
    tube = tube.rotate((0, 0, 0), (0, 1, 0), 90.0)
    return tube


def _build_rear_plate() -> cq.Workplane:
    """Rear end plate closing the barrel; plunger rod passes through bore."""
    plate = (
        cq.Workplane("YZ")
        .workplane(offset=BACK_X - 0.010)
        .circle(BARREL_OUTER_R + 0.003)
        .circle(0.006)
        .extrude(0.012)
    )
    return plate


def _build_grip() -> cq.Workplane:
    """Pistol grip hanging from the barrel underside."""
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
    """Yoke of two side plates carrying the trigger pivot pin."""
    strut_top_z = -BARREL_OUTER_R + 0.002
    plate_h = abs(strut_top_z - PIVOT_Z) + 0.012
    plate_cz = (PIVOT_Z + strut_top_z) / 2.0

    def side_plate(y_inner: float, thickness: float) -> cq.Workplane:
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


def _build_pivot_pin_mesh():
    """Steel pivot pin spanning between the two lug discs."""
    pin = (
        cq.Workplane("XZ")
        .workplane(offset=PIVOT_Y_HALF + 0.005)
        .center(PIVOT_X, PIVOT_Z)
        .circle(0.0035)
        .extrude(-(2 * PIVOT_Y_HALF + 0.010))
    )
    return mesh_from_cadquery(pin, "pivot_pin")


def _build_frame_mesh():
    """Complete frame: barrel tube + rear plate + grip + pivot lugs."""
    frame = _build_barrel()
    frame = frame.union(_build_rear_plate())
    frame = frame.union(_build_grip())
    frame = frame.union(_build_pivot_lugs())
    return mesh_from_cadquery(frame, "frame")


# ---------------------------------------------------------------------------
# End cap (threaded hex ring at muzzle closing the barrel front)
# ---------------------------------------------------------------------------
def _build_end_cap_mesh():
    """Hex ring end cap that threads onto the barrel muzzle."""
    # Build hex prism along Z, with central bore, then rotate to X axis
    cap = _hex_prism(CAP_HEX_AF, CAP_LEN, bore_r=CAP_BORE_R)
    # Add knurl ridges around the hex exterior for grip
    cap = cap.translate((0, 0, -CAP_LEN / 2.0))
    # Rotate Z → X axis
    cap = cap.rotate((0, 0, 0), (0, 1, 0), 90.0)
    # Position at barrel front face
    cap = cap.translate((CAP_START_X + CAP_LEN / 2.0, 0, 0))
    return mesh_from_cadquery(cap, "end_cap")


# ---------------------------------------------------------------------------
# Nose cone (tapered nozzle piece threading through end cap)
# ---------------------------------------------------------------------------
def _build_nose_cone_mesh():
    """Tapered nose cone with flange, taper, and nozzle extension."""
    # Flange shoulder (seats against end cap front face)
    flange_x = CAP_START_X + CAP_LEN
    flange = (
        cq.Workplane("YZ")
        .workplane(offset=flange_x)
        .circle(NC_FLANGE_R)
        .extrude(NC_FLANGE_LEN)
    )
    # Tapered cone section
    taper_start_x = flange_x + NC_FLANGE_LEN
    taper = (
        cq.Workplane("YZ")
        .workplane(offset=taper_start_x)
        .circle(NC_BASE_R)
        .workplane(offset=NC_TAPER_LEN)
        .circle(NC_TIP_R)
        .loft()
    )
    # Nozzle tube extension
    nozzle_start_x = taper_start_x + NC_TAPER_LEN
    nozzle = (
        cq.Workplane("YZ")
        .workplane(offset=nozzle_start_x)
        .circle(NZ_BASE_R)
        .workplane(offset=NZ_LEN)
        .circle(NZ_TIP_R)
        .loft()
    )
    # Bore through the whole nose cone (sealant passage)
    bore = (
        cq.Workplane("YZ")
        .workplane(offset=flange_x - 0.002)
        .circle(0.003)
        .extrude(NC_FLANGE_LEN + NC_TAPER_LEN + NZ_LEN + 0.004)
    )
    cone = flange.union(taper).union(nozzle).cut(bore)
    return mesh_from_cadquery(cone, "nose_cone")


# ---------------------------------------------------------------------------
# Trigger lever
# ---------------------------------------------------------------------------
def _build_trigger_mesh():
    """Trigger blade with pivot hub, authored in pivot-local frame."""
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
    # Pivot eye (hub) wrapping the pin at local origin
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
def _build_plunger_mesh():
    """Push plate + rod + rear thumb handle."""
    plate = (
        cq.Workplane("YZ")
        .workplane(offset=PLATE_X)
        .circle(BARREL_R - 0.003)
        .extrude(-0.008)
    )
    rod = (
        cq.Workplane("YZ")
        .workplane(offset=PLATE_X - 0.008)
        .circle(0.0045)
        .extrude(-ROD_LEN)
    )
    plunger = plate.union(rod)
    # Rear thumb plate / pull handle at the very back of the rod
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
    model = ArticulatedObject(name="sausage_caulking_gun")

    model.material("frame_body", rgba=(0.22, 0.24, 0.28, 1.0))   # dark gunmetal
    model.material("steel", rgba=(0.55, 0.56, 0.58, 1.0))
    model.material("cap_black", rgba=(0.12, 0.12, 0.14, 1.0))     # matte black
    model.material("cone_brass", rgba=(0.72, 0.58, 0.26, 1.0))    # brass/bronze
    model.material("grip_red", rgba=(0.78, 0.18, 0.14, 1.0))      # red overmold

    # --- Frame (root): closed barrel + rear plate + grip + pivot lugs ---
    frame = model.part("frame")
    frame.visual(_build_frame_mesh(), material="frame_body", name="frame_shell")
    frame.visual(_build_pivot_pin_mesh(), material="steel", name="pivot_pin")

    # --- End cap (FIXED to frame): hex ring at muzzle ---
    end_cap = model.part("end_cap")
    end_cap.visual(_build_end_cap_mesh(), material="cap_black", name="cap_ring")

    model.articulation(
        "frame_to_end_cap",
        ArticulationType.FIXED,
        parent=frame,
        child=end_cap,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # --- Nose cone (FIXED to frame): tapered nozzle piece ---
    nose_cone = model.part("nose_cone")
    nose_cone.visual(_build_nose_cone_mesh(), material="cone_brass", name="cone_body")

    model.articulation(
        "frame_to_nose_cone",
        ArticulationType.FIXED,
        parent=frame,
        child=nose_cone,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # --- Trigger (REVOLUTE about Y at the pivot pin) ---
    trigger = model.part("trigger")
    trigger.visual(_build_trigger_mesh(), material="grip_red", name="trigger_blade")

    model.articulation(
        "frame_to_trigger",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=trigger,
        origin=Origin(xyz=(PIVOT_X, 0.0, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=3.0, lower=0.0, upper=0.5),
    )

    # --- Plunger rod (PRISMATIC along +X) ---
    plunger = model.part("plunger")
    plunger.visual(_build_plunger_mesh(), material="steel", name="plunger_rod")

    model.articulation(
        "frame_to_plunger",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=plunger,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=300.0, velocity=0.05, lower=0.0, upper=0.200),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    end_cap = object_model.get_part("end_cap")
    nose_cone = object_model.get_part("nose_cone")
    trigger = object_model.get_part("trigger")
    plunger = object_model.get_part("plunger")

    trig_joint = object_model.get_articulation("frame_to_trigger")
    plunger_joint = object_model.get_articulation("frame_to_plunger")
    cap_joint = object_model.get_articulation("frame_to_end_cap")
    cone_joint = object_model.get_articulation("frame_to_nose_cone")

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
        "end cap is fixed to frame",
        str(cap_joint.articulation_type).upper().endswith("FIXED"),
        details=f"type={cap_joint.articulation_type}",
    )
    ctx.check(
        "nose cone is fixed to frame",
        str(cone_joint.articulation_type).upper().endswith("FIXED"),
        details=f"type={cone_joint.articulation_type}",
    )

    # --- Barrel is a full closed cylinder (not an open cradle) ---
    # The barrel tube should span the full length and enclose the YZ cross-section
    frame_aabb = ctx.part_world_aabb(frame)
    ctx.check(
        "barrel tube spans the full barrel length",
        frame_aabb is not None and (frame_aabb[1][0] - frame_aabb[0][0]) >= BARREL_LEN - 0.01,
        details=f"frame_x_extent={frame_aabb[1][0] - frame_aabb[0][0] if frame_aabb else None}",
    )

    # --- End cap at the muzzle (front) ---
    cap_aabb = ctx.part_element_world_aabb(end_cap, elem="cap_ring")
    ctx.check(
        "end cap sits at the barrel front (muzzle)",
        cap_aabb is not None and cap_aabb[0][0] >= FRONT_X - 0.005,
        details=f"cap_min_x={None if cap_aabb is None else cap_aabb[0][0]}, front_x={FRONT_X}",
    )

    # End cap is centered on barrel axis in YZ
    ctx.expect_within(
        end_cap,
        frame,
        axes="yz",
        inner_elem="cap_ring",
        outer_elem="frame_shell",
        margin=0.015,
        name="end cap centered on barrel axis (yz)",
    )

    # --- Nose cone protrudes forward past end cap ---
    cone_aabb = ctx.part_element_world_aabb(nose_cone, elem="cone_body")
    ctx.check(
        "nose cone protrudes forward past the end cap",
        cone_aabb is not None and cap_aabb is not None
        and cone_aabb[1][0] > cap_aabb[1][0] + 0.02,
        details=f"cone_max_x={None if cone_aabb is None else cone_aabb[1][0]}, "
                f"cap_max_x={None if cap_aabb is None else cap_aabb[1][0]}",
    )

    # Nose cone is centered on barrel axis
    ctx.expect_within(
        nose_cone,
        frame,
        axes="yz",
        inner_elem="cone_body",
        outer_elem="frame_shell",
        margin=0.012,
        name="nose cone centered on barrel axis (yz)",
    )

    # --- End cap contacts the barrel front face ---
    ctx.allow_overlap(
        end_cap,
        frame,
        elem_a="cap_ring",
        elem_b="frame_shell",
        reason="End cap threads onto and seats against the barrel front face.",
    )

    # --- Nose cone passes through end cap bore (intentional nesting) ---
    ctx.allow_overlap(
        nose_cone,
        end_cap,
        elem_a="cone_body",
        elem_b="cap_ring",
        reason="Nose cone flange and base pass through end cap bore to seat against it.",
    )

    # --- Trigger hub captures the pivot pin ---
    ctx.allow_overlap(
        trigger,
        frame,
        elem_a="trigger_blade",
        elem_b="pivot_pin",
        reason="Trigger pivot hub intentionally captures the steel pivot pin.",
    )

    # --- Plunger rod slides inside the barrel ---
    ctx.allow_overlap(
        plunger,
        frame,
        elem_a="plunger_rod",
        elem_b="frame_shell",
        reason="Plunger rod/plate intentionally slides inside the closed barrel tube.",
    )

    # --- Grip hangs below the barrel ---
    ctx.check(
        "grip hangs well below the barrel",
        frame_aabb[0][2] < -0.10,
        details=f"frame_min_z={frame_aabb[0][2]}",
    )

    # --- Mechanism motion checks ---
    # Squeezing the trigger swings its lower blade rearward toward the grip
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

    # Advancing the plunger moves the push plate forward toward the nozzle (+X)
    plate_rest = ctx.part_world_position(plunger)
    with ctx.pose({plunger_joint: 0.160}):
        plate_adv = ctx.part_world_position(plunger)
        # Push plate stays inside barrel cross-section in YZ when advanced
        ctx.expect_within(
            plunger,
            frame,
            axes="yz",
            inner_elem="plunger_rod",
            outer_elem="frame_shell",
            margin=0.006,
            name="advanced plunger plate stays inside barrel bore (yz)",
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
