from __future__ import annotations

# Aerosol SPRAY PAINT CAN with a trigger-lever spray handle (Montana-style pro cap).
# Frame: can axis along +Z. Base rim sits at z=0, can body rises in +Z, the
# crimped dome and trigger mechanism are at the top. The cylinder is ~0.20 m
# tall and ~0.065 m in diameter.
# Articulations:
#   - dust cap: PRISMATIC lift straight UP (+Z) by ~0.09 m (identical to parent)
#   - trigger lever: REVOLUTE on horizontal Y-axis, finger squeeze presses valve
# The trigger lever and its fixed housing are children of the can body; the
# dust cap sleeves over both when seated.

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---- key dimensions (meters) ----
CAN_R = 0.0325            # body radius (~0.065 m diameter)
BODY_BOTTOM = 0.0         # base plane
BODY_TOP = 0.150          # where the straight wall ends and the dome begins
DOME_TOP = 0.176          # top of the crimped dome (mounting cup rim)
VALVE_TOP = 0.182         # top of the central valve stem
CAP_BOTTOM = 0.150        # dust cap lower rim
CAP_TOP = 0.210           # dust cap top

# Trigger mechanism dimensions
HOUSING_W = 0.020         # housing width (X)
HOUSING_D = 0.024         # housing depth (Y)
HOUSING_H = 0.024         # housing height (Z, from DOME_TOP)
TRIGGER_PIVOT_X = 0.002   # pivot X offset (slightly forward of center)
TRIGGER_PIVOT_Y = 0.0     # pivot on centerline
TRIGGER_PIVOT_Z = DOME_TOP + HOUSING_H  # pivot height = 0.200


def _can_body_solid() -> cq.Workplane:
    """Tall cylindrical can body with crimped dome and valve stem boss.
    Identical to the parent asset."""
    # Bottom rim (slightly larger ring at the base for the rolled seam).
    body = (
        cq.Workplane("XY")
        .circle(CAN_R + 0.0012)
        .extrude(0.006)
    )
    # Main straight wall.
    body = body.union(
        cq.Workplane("XY")
        .workplane(offset=0.005)
        .circle(CAN_R)
        .extrude(BODY_TOP - 0.005)
    )
    # Crimped top dome: loft from the wall up and inward to the mounting cup rim.
    dome = (
        cq.Workplane("XY")
        .workplane(offset=BODY_TOP - 0.001)
        .circle(CAN_R)
        .workplane(offset=0.006)
        .circle(CAN_R - 0.001)
        .workplane(offset=0.004)
        .circle(CAN_R - 0.006)
        .workplane(offset=0.017)
        .circle(0.014)
        .loft(ruled=False)
    )
    body = body.union(dome)
    # Central valve stem boss the trigger presses on.
    body = body.union(
        cq.Workplane("XY")
        .workplane(offset=DOME_TOP - 0.001)
        .circle(0.006)
        .extrude(VALVE_TOP - DOME_TOP)
    )
    return body


def _trigger_housing_solid() -> cq.Workplane:
    """Fixed cap housing sitting on the dome top. Origin at housing base (z=0
    local = DOME_TOP world). A small rounded block with a pivot ear boss at
    the top-front where the trigger pin goes through."""
    body = (
        cq.Workplane("XY")
        .box(HOUSING_W, HOUSING_D, HOUSING_H, centered=(True, True, False))
        .edges("|Z").fillet(0.004)
        .edges(">Z").fillet(0.002)
    )
    # Pivot ear boss at the top-front (+X side) for the trigger pivot pin.
    # Centered at local x=+0.008, z = HOUSING_H - 0.004 (near top).
    ear = (
        cq.Workplane("XY")
        .workplane(offset=HOUSING_H - 0.008)
        .center(0.008, 0)
        .box(0.008, HOUSING_D + 0.002, 0.010, centered=(True, True, False))
        .edges(">Z").fillet(0.002)
    )
    return body.union(ear)


def _trigger_lever_solid() -> cq.Workplane:
    """Trigger lever in local frame (origin at pivot point).
    The lever extends forward (+X) and downward (-Z) from the pivot as a
    single extruded profile, forming a finger grip that swings inward when
    squeezed. The profile is carefully shaped to stay clear of the can body
    dome and valve stem at rest."""
    # Single extruded profile in XZ plane: an angled lever with a wider
    # finger pad at the bottom and a pivot bushing area at the top.
    # Profile points (local XZ, origin at pivot):
    #   A(0,0) → B(+X,0): top edge near pivot
    #   B → C: angled arm going forward and down
    #   C → D → E → F: wider finger pad at the bottom
    #   F → G → H: rear return to near pivot
    profile = (
        cq.Workplane("XZ")
        .moveTo(0.000, 0.000)       # A: pivot
        .lineTo(0.006, 0.000)       # B: top-front
        .lineTo(0.018, -0.014)      # C: mid arm (angled forward)
        .lineTo(0.020, -0.018)      # D: pad front-top
        .lineTo(0.020, -0.020)      # E: pad front-bottom
        .lineTo(0.010, -0.020)      # F: pad rear-bottom
        .lineTo(0.008, -0.018)      # G: pad rear-top
        .lineTo(0.004, -0.004)      # H: rear return
        .close()                    # back to A
    )
    # Note: XZ workplane extrude direction is -Y, so positive translate
    # centers the geometry around Y=0.
    lever = profile.extrude(0.018).translate((0, 0.009, 0))
    # Pivot bushing: small cylinder along Y at the pivot for the pin bore.
    bushing = (
        cq.Workplane("XZ")
        .center(0.0, 0.0)
        .circle(0.005)
        .extrude(HOUSING_D - 0.002)
        .translate((0, (HOUSING_D - 0.002) / 2.0, 0))
    )
    lever = lever.union(bushing)
    # Grip ridges on the finger pad front face (repeated detail).
    # Ridges are set back from the profile edge to ensure mesh connectivity
    # with the main lever body.
    for i in range(3):
        z_off = -0.017 - i * 0.001
        ridge = (
            cq.Workplane("XZ")
            .center(0.019, z_off)
            .rect(0.004, 0.001)
            .extrude(0.016)
            .translate((0, 0.008, 0))
        )
        lever = lever.union(ridge)
    return lever


def _dust_cap_solid() -> cq.Workplane:
    """Dark grey dust cap: hollow cylindrical shell with closed domed top.
    Identical to the parent asset."""
    h = CAP_TOP - CAP_BOTTOM
    outer = (
        cq.Workplane("XY")
        .circle(CAN_R + 0.0035)
        .extrude(h - 0.010)
    )
    # Rounded closed top.
    top = (
        cq.Workplane("XY")
        .workplane(offset=h - 0.011)
        .circle(CAN_R + 0.0035)
        .workplane(offset=0.007)
        .circle(CAN_R - 0.004)
        .workplane(offset=0.004)
        .circle(0.010)
        .loft(ruled=False)
    )
    cap = outer.union(top)
    # Hollow the inside so it slips over the can top.
    bore = (
        cq.Workplane("XY")
        .workplane(offset=-0.001)
        .circle(CAN_R - 0.0006)
        .extrude(h - 0.014)
    )
    cap = cap.cut(bore)
    return cap


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="spray_paint_can_trigger")

    label = model.material("label_splatter", rgba=(0.90, 0.90, 0.92, 1.0))
    metal = model.material("metal", rgba=(0.78, 0.80, 0.83, 1.0))
    dark_cap = model.material("dark_grey_cap", rgba=(0.20, 0.21, 0.20, 1.0))
    housing_mat = model.material("housing_dark", rgba=(0.12, 0.12, 0.13, 1.0))
    trigger_mat = model.material("trigger_red", rgba=(0.78, 0.14, 0.10, 1.0))

    # ---- can body (root) ----
    body = model.part("can_body")
    body.visual(
        mesh_from_cadquery(_can_body_solid(), "can_body"),
        material=metal,
        name="can_body",
    )
    # Splatter-graphic label band wrapped on the straight wall.
    label_band = CylinderGeometry(CAN_R + 0.0004, 0.105, radial_segments=64)
    label_band.translate(0.0, 0.0, 0.012 + 0.105 / 2.0)
    body.visual(
        mesh_from_geometry(label_band, "label_band"),
        material=label,
        name="label_band",
    )
    # Trigger housing: fixed cap body on the dome (inline visual, not a separate part).
    body.visual(
        mesh_from_cadquery(_trigger_housing_solid(), "trigger_housing"),
        material=housing_mat,
        origin=Origin(xyz=(0.0, 0.0, DOME_TOP)),
        name="trigger_housing",
    )
    body.inertial = Inertial.from_geometry(
        Cylinder(CAN_R, 0.176),
        mass=0.30,
        origin=Origin(xyz=(0.0, 0.0, 0.088)),
    )

    # ---- trigger lever: child of can body, REVOLUTE squeeze ----
    trigger = model.part("trigger_lever")
    trigger.visual(
        mesh_from_cadquery(_trigger_lever_solid(), "trigger_lever"),
        material=trigger_mat,
        name="trigger_lever",
    )
    trigger.inertial = Inertial.from_geometry(
        Box((0.018, 0.018, 0.032)),
        mass=0.006,
        origin=Origin(xyz=(0.008, 0.0, -0.016)),
    )
    model.articulation(
        "trigger_squeeze",
        ArticulationType.REVOLUTE,
        parent=body,
        child=trigger,
        origin=Origin(xyz=(TRIGGER_PIVOT_X, TRIGGER_PIVOT_Y, TRIGGER_PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),  # horizontal Y-axis
        motion_limits=MotionLimits(effort=5.0, velocity=2.0, lower=0.0, upper=0.45),
    )

    # ---- dust cap: child of can body, lifts straight up (IDENTICAL to parent) ----
    cap = model.part("dust_cap")
    cap.visual(
        mesh_from_cadquery(_dust_cap_solid(), "dust_cap"),
        material=dark_cap,
        name="dust_cap",
    )
    cap.inertial = Inertial.from_geometry(
        Cylinder(CAN_R + 0.0035, CAP_TOP - CAP_BOTTOM),
        mass=0.015,
        origin=Origin(xyz=(0.0, 0.0, (CAP_TOP - CAP_BOTTOM) / 2.0)),
    )
    model.articulation(
        "cap_lift",
        ArticulationType.PRISMATIC,
        parent=body,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, CAP_BOTTOM)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=0.2, lower=0.0, upper=0.090),
    )

    return model


object_model = build_object_model()


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("can_body")
    trigger = object_model.get_part("trigger_lever")
    cap = object_model.get_part("dust_cap")
    cap_lift = object_model.get_articulation("cap_lift")
    trigger_squeeze = object_model.get_articulation("trigger_squeeze")

    # --- can is a tall cylinder (identical check to parent) ---
    bmn, bmx = ctx.part_world_aabb(body)
    height = bmx[2] - bmn[2]
    dia_x = bmx[0] - bmn[0]
    dia_y = bmx[1] - bmn[1]
    ctx.check(
        "can body is a tall cylinder (~0.18 m, ~0.065 m dia)",
        height > 0.16 and 0.05 < dia_x < 0.08 and 0.05 < dia_y < 0.08 and height > dia_x * 2.0,
        details=f"height={height:.3f}, dia_x={dia_x:.3f}, dia_y={dia_y:.3f}",
    )

    # --- trigger_squeeze is REVOLUTE (replaces the prismatic nozzle) ---
    ctx.check(
        "trigger_squeeze is a REVOLUTE joint",
        trigger_squeeze.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={trigger_squeeze.articulation_type}",
    )

    # --- trigger lever is mounted on the can top near the dome ---
    tpos = ctx.part_world_position(trigger)
    ctx.check(
        "trigger lever is mounted on the can top",
        tpos is not None and tpos[2] > 0.16 and abs(tpos[1]) < 0.02,
        details=f"trigger position={tpos}",
    )

    # --- trigger housing is an inline visual on can_body ---
    housing_vis = body.get_visual("trigger_housing")
    ctx.check(
        "trigger housing is an inline visual on can_body",
        housing_vis is not None,
        details="trigger_housing visual not found on can_body",
    )

    # --- trigger lever pivot area contacts the housing (pivot support) ---
    ctx.allow_overlap(
        trigger,
        body,
        elem_a="trigger_lever",
        elem_b="trigger_housing",
        reason="Trigger lever pivot bushing is intentionally nested in the housing ear boss.",
    )
    ctx.expect_contact(
        trigger, body,
        elem_a="trigger_lever", elem_b="trigger_housing",
        name="trigger lever pivots from the housing ear",
    )

    # --- trigger squeeze motion: lever swings inward and presses down ---
    rest_mn, rest_mx = ctx.part_world_aabb(trigger)
    with ctx.pose({trigger_squeeze: 0.40}):
        sq_mn, sq_mx = ctx.part_world_aabb(trigger)
    ctx.check(
        "trigger squeeze swings the lever toward the can (min_x decreases)",
        sq_mn[0] < rest_mn[0] - 0.003,
        details=f"rest_min_x={rest_mn[0]:.4f}, squeezed_min_x={sq_mn[0]:.4f}",
    )
    ctx.check(
        "trigger squeeze pushes lever downward (presses valve, min_z decreases)",
        sq_mn[2] < rest_mn[2] - 0.003,
        details=f"rest_min_z={rest_mn[2]:.4f}, squeezed_min_z={sq_mn[2]:.4f}",
    )

    # --- dust cap sleeves over the can top and trigger (unchanged mechanism) ---
    ctx.allow_overlap(
        cap,
        body,
        elem_a="dust_cap",
        elem_b="can_body",
        reason="Dust cap intentionally sleeves down over the can top shoulder when seated.",
    )
    ctx.allow_overlap(
        cap,
        trigger,
        elem_a="dust_cap",
        elem_b="trigger_lever",
        reason="Dust cap intentionally encloses the trigger lever when seated (cap covers mechanism).",
    )
    ctx.allow_overlap(
        cap,
        body,
        elem_a="dust_cap",
        elem_b="trigger_housing",
        reason="Dust cap sleeves over the trigger housing when seated.",
    )
    cap_rest_mn, cap_rest_mx = ctx.part_world_aabb(cap)
    ctx.check(
        "dust cap is seated over the can top at rest",
        cap_rest_mn[2] < 0.16 and cap_rest_mx[2] > 0.20,
        details=f"cap z range at rest=({cap_rest_mn[2]:.3f}, {cap_rest_mx[2]:.3f})",
    )

    # --- dust cap lifts straight UP, revealing the trigger mechanism ---
    tmn, tmx = ctx.part_world_aabb(trigger)
    trigger_top = tmx[2]
    cap_rest_pos_z = ctx.part_world_position(cap)[2]
    with ctx.pose({cap_lift: 0.090}):
        cap_up_mn, cap_up_mx = ctx.part_world_aabb(cap)
        cap_up_pos = ctx.part_world_position(cap)
    ctx.check(
        "dust cap lifts straight up by a large amount (~0.09 m)",
        cap_up_pos[2] - cap_rest_pos_z > 0.085,
        details=f"cap rose from z={cap_rest_pos_z:.3f} to z={cap_up_pos[2]:.3f}",
    )
    ctx.check(
        "lifted cap bottom clears the trigger top (trigger revealed)",
        cap_up_mn[2] > trigger_top + 0.002,
        details=f"lifted cap bottom={cap_up_mn[2]:.3f}, trigger top={trigger_top:.3f}",
    )
    ctx.check(
        "cap lift is purely vertical (no lateral drift)",
        abs(cap_up_pos[0]) < 0.005 and abs(cap_up_pos[1]) < 0.005,
        details=f"lifted cap xy=({cap_up_pos[0]:.4f}, {cap_up_pos[1]:.4f})",
    )

    return ctx.report()
