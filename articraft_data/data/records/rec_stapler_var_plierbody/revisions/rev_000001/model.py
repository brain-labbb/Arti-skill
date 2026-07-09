from __future__ import annotations

# Realistic hand-squeeze plier-style stapler, forked from the desktop variant.
#
# Real object: a charcoal/dark-gray stapler with a curved ergonomic grip handle
# (plier-style) instead of a flat desk tray. The lower body is a curved grip
# arm meant to be squeezed by hand. The big rounded "hump" on top is the
# magazine-cover / handle arm. It is hinged at the rear and swings up about a
# transverse pivot pin so staples can be loaded and so the head presses down
# to drive a staple. Underneath the cover is a metal staple carrier rail.
# The lower grip curves downward for hand-squeeze action; near the front sits
# a bright metal anvil plate carrying the two staple-clinching grooves.
#
# Axes (model frame):
#   +X = length (front of jaw at +X, rear hinge at -X)
#   +Y = width
#   +Z = up
#
# Primary articulation: base_to_arm, REVOLUTE about the rear hinge (axis +Y).
# Positive q opens the jaws (releases the squeeze); squeezing closes them.

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
# Dimensions (meters)
# ---------------------------------------------------------------------------
TOTAL_LEN = 0.158  # nose-to-tail length of the closed body
WIDTH = 0.040  # body width
WALL = 0.0015  # plastic shell wall thickness

# Lower body (base tray). The deck under the cover is kept low so the cover
# clears it; only the rear heel rises to carry the hinge.
BASE_LEN = 0.158
DECK_FRONT_Z = 0.010  # thin deck at the front jaw
DECK_MID_Z = 0.012  # deck under the cover
BASE_REAR_H = 0.019  # rear heel that carries the hinge mount
BASE_FOOT_Z = 0.0  # base sits on z=0

# Top arm (magazine cover / handle) - the big curved hump
ARM_LEN = 0.150
ARM_MAX_H = 0.036  # crown height of the hump above its underside

# Rear hinge. Pivot sits above the rear heel so the cover underside clears the
# base deck in the closed pose.
HINGE_X = -TOTAL_LEN / 2.0 + 0.009  # pivot near the rear
HINGE_Z = 0.0205  # pivot height above z=0
HINGE_PIN_R = 0.0024
HINGE_PIN_LEN = WIDTH + 0.004
HINGE_KNUCKLE_R = 0.0042
HINGE_KNUCKLE_LEN = 0.010

# Metal anvil plate (front, on the base top deck) carrying clinch grooves
ANVIL_LEN = 0.030
ANVIL_W = 0.026
ANVIL_TH = 0.0016
ANVIL_X = TOTAL_LEN / 2.0 - 0.030
ANVIL_SEAT_Z = -0.008 + 0.006 + 0.0005  # plate rests on top of the anvil platform

# Staple carrier rail under the arm (the metal track)
RAIL_LEN = 0.110
RAIL_W = 0.012
RAIL_H = 0.010

# Colors
COL_BODY = (0.20, 0.21, 0.22, 1.0)  # charcoal plastic
COL_ARM = (0.235, 0.245, 0.255, 1.0)  # slightly lighter charcoal hump
COL_METAL = (0.78, 0.79, 0.80, 1.0)  # bright steel anvil / rail
COL_DARK_METAL = (0.42, 0.43, 0.45, 1.0)  # darker pin / carrier


# ---------------------------------------------------------------------------
# Geometry builders (CadQuery, authored directly in meters)
# ---------------------------------------------------------------------------
def _base_grip() -> cq.Workplane:
    """Plier-style curved grip handle: ergonomic lower body meant to be squeezed
    by hand. The grip curves downward and back from the hinge area, with the
    anvil/staple exit at the front. Built as a swept elliptical cross-section
    along a curved spline path."""
    
    # Define the grip centerline path in XZ plane
    # Stay below the arm shell (which starts at HINGE_Z=0.0205)
    # The elliptical cross-section has vertical radius 0.009, so center must be
    # at most Z=0.011 to keep the top below Z=0.020
    path_pts = [
        (HINGE_X + 0.002, 0.011),              # start well below arm shell
        (HINGE_X + 0.015, HINGE_Z - 0.012),   # transition down
        (HINGE_X + 0.025, HINGE_Z - 0.030),   # curve down
        (-0.020, -0.032),                       # lowest point
        (0.025, -0.026),                        # rise slightly
        (0.055, -0.016),                        # approach anvil
        (ANVIL_X, -0.006),                      # anvil zone
    ]
    
    # Create the path as a spline in XZ plane
    path = cq.Workplane("XZ").spline(path_pts)
    
    # Create the cross-section profile at the start of the path
    # Elliptical grip profile, oriented perpendicular to the path
    grip_profile = (
        cq.Workplane("YZ", origin=(path_pts[0][0], 0.0, path_pts[0][1]))
        .ellipse(WIDTH * 0.45, 0.009)
    )
    
    # Sweep the profile along the path
    grip = grip_profile.sweep(path)
    
    # Add a flat platform at the front for the anvil plate
    anvil_platform = (
        cq.Workplane("XY")
        .box(ANVIL_LEN + 0.008, WIDTH * 0.85, 0.006, centered=(True, True, False))
        .translate((ANVIL_X, 0.0, -0.008))
    )
    grip = grip.union(anvil_platform)
    
    # The hinge knuckles (separate visual) provide the hinge mount structure
    # and bridge the connection from grip to pivot point
    
    return grip


def _grip_texture_ridges() -> list[cq.Workplane]:
    """Ergonomic finger-grip ridges along the curved handle. Generated as a loop
    of small raised bumps for tactile grip texture, positioned along the grip
    centerline path."""
    ridges = []
    # Path points matching the grip centerline
    path_pts = [
        (HINGE_X + 0.002, 0.011),
        (HINGE_X + 0.015, HINGE_Z - 0.012),
        (HINGE_X + 0.025, HINGE_Z - 0.030),
        (-0.020, -0.032),
        (0.025, -0.026),
        (0.055, -0.016),
        (ANVIL_X, -0.006),
    ]
    
    # Place 5 ridges along the grip length for finger placement
    n_ridges = 5
    for i in range(n_ridges):
        # Interpolate position along the path (skip the first segment near hinge)
        t = 0.25 + (i * 0.13)  # 0.25 to 0.77
        # Find the segment
        idx = min(int(t * (len(path_pts) - 1)), len(path_pts) - 2)
        local_t = (t * (len(path_pts) - 1)) - idx
        x0, z0 = path_pts[idx]
        x1, z1 = path_pts[idx + 1]
        x_pos = x0 + local_t * (x1 - x0)
        z_pos = z0 + local_t * (z1 - z0)
        
        # Create a small elliptical ridge protruding from the grip surface
        # Positioned at the top of the grip cross-section
        ridge = (
            cq.Workplane("XY")
            .ellipse(0.007, 0.004)
            .extrude(0.002, both=True)
            .rotate((0, 0, 0), (1, 0, 0), 90)
            .translate((x_pos, 0.0, z_pos + 0.008))  # sit on top of grip
        )
        ridges.append(ridge)
    
    return ridges


def _anvil_plate() -> cq.Workplane:
    """Bright metal anvil plate seated on the front deck with two clinch
    grooves (the slots that bend staple legs)."""
    plate = (
        cq.Workplane("XY")
        .box(ANVIL_LEN, ANVIL_W, ANVIL_TH, centered=(True, True, False))
        .edges("|Z").fillet(0.002)
    )
    # Two clinch grooves running across the plate width.
    groove = cq.Workplane("XY").box(0.0026, 0.012, 0.0014, centered=(True, True, False))
    plate = plate.cut(groove.translate((-0.004, 0.0, ANVIL_TH - 0.0013)))
    plate = plate.cut(groove.translate((0.004, 0.0, ANVIL_TH - 0.0013)))
    return plate.translate((ANVIL_X, 0.0, ANVIL_SEAT_Z))


def _arm_shell() -> cq.Workplane:
    """The top magazine cover / handle: the big rounded hump. Authored in the
    ARM (child) local frame where the hinge pivot sits at local origin and the
    cover extends along +X. Built from a curved side silhouette extruded across
    the width, hollowed underneath, with two rear cheeks that reach down to the
    hinge pin so the whole arm is one connected solid."""
    L = ARM_LEN
    crown = ARM_MAX_H
    # Side silhouette of the cover in XZ. The cover bottom is a few mm above the
    # pivot (arm-z=0) so it clears the base deck; the rear cheeks bridge down.
    bottom = 0.006
    pts = [
        (0.004, bottom),
        (0.004, crown * 0.62),
        (0.016, crown * 0.92),
        (0.044, crown),
        (L - 0.045, crown),
        (L - 0.014, crown * 0.86),
        (L, crown * 0.42),  # rounded front nose
        (L, bottom),
    ]
    prof = cq.Workplane("XZ").polyline(pts).close()
    shell = prof.extrude(WIDTH / 2.0, both=True)
    # Round the long crown edges and the nose to get the soft pebble shape.
    shell = shell.edges("|Y and >Z").fillet(0.010)
    shell = shell.edges("|Y and >X").fillet(0.006)
    shell = shell.edges("|X and >Z").fillet(0.008)
    # Hollow the underside so the cover is a real shell, not a solid block.
    cavity = (
        cq.Workplane("XZ")
        .polyline(
            [
                (0.014, bottom + 0.0015),
                (0.014, crown * 0.70),
                (0.044, crown * 0.78),
                (L - 0.050, crown * 0.78),
                (L - 0.024, crown * 0.55),
                (L - 0.018, bottom + 0.0015),
            ]
        )
        .close()
        .extrude((WIDTH / 2.0) - WALL, both=True)
    )
    shell = shell.cut(cavity)
    # Two rear cheeks descend from the cover bottom to the hinge pin, forming the
    # arm-side hinge ears that the pin runs through. They sit inboard of the base
    # knuckles (which are near the outer edges) so they interleave on the shared
    # pin instead of colliding with the knuckle rings.
    cheek = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.005, 0.0),
                (0.012, 0.0),
                (0.012, bottom + 0.002),
                (-0.005, bottom + 0.002),
            ]
        )
        .close()
        .extrude(0.005)
    )
    left_cheek = cheek.translate((0.0, 0.006, 0.0))
    right_cheek = cheek.translate((0.0, -0.006 - 0.005, 0.0))
    shell = shell.union(left_cheek).union(right_cheek)
    return shell


def _hinge_knuckles() -> cq.Workplane:
    """Two visible hinge knuckles on the rear heel of the base that capture the
    pin, authored in the BASE frame around the pivot at (HINGE_X, *, HINGE_Z).
    Each knuckle is bored for the pin and skirted down to the heel so it reads
    as mounted, not floating."""
    # The knuckle ring axis runs along +Y (the pivot axis); XZ workplane normal
    # is +Y, so extruding it builds a Y-axis cylinder.
    kn = cq.Workplane("XZ").circle(HINGE_KNUCKLE_R).extrude(HINGE_KNUCKLE_LEN / 2.0, both=True)
    # Skirt connecting the knuckle ring down into the rear heel deck.
    skirt = cq.Workplane("XY").box(
        2 * HINGE_KNUCKLE_R, HINGE_KNUCKLE_LEN, HINGE_Z, centered=(True, True, False)
    ).translate((0.0, 0.0, -HINGE_Z / 2.0))
    knuckle = kn.union(skirt)
    # Bore the pin hole through the ring along +Y.
    bore = cq.Workplane("XZ").circle(HINGE_PIN_R + 0.0006).extrude(0.05, both=True)
    knuckle = knuckle.cut(bore)
    left = knuckle.translate((HINGE_X, WIDTH / 2.0 - 0.003, HINGE_Z))
    right = knuckle.translate((HINGE_X, -(WIDTH / 2.0 - 0.003), HINGE_Z))
    return left.union(right)


def _hinge_pin() -> cq.Workplane:
    """Transverse hinge pin spanning the rear along +Y, authored in the ARM
    frame at the arm local origin (the pivot). XZ workplane normal is +Y, so
    extruding it builds a Y-axis cylinder (the pivot axis)."""
    pin = (
        cq.Workplane("XZ")
        .circle(HINGE_PIN_R)
        .extrude(HINGE_PIN_LEN / 2.0, both=True)
    )
    return pin


def _carrier_rail() -> cq.Workplane:
    """Metal staple carrier rail under the arm; rides with the arm. Authored in
    the ARM frame, hanging below the cover crown toward the deck."""
    rail = (
        cq.Workplane("XY")
        .box(RAIL_LEN, RAIL_W, RAIL_H, centered=(True, True, False))
        .edges("|X").fillet(0.0015)
    )
    # An end pusher block (spring follower face) at the rear.
    pusher = cq.Workplane("XY").box(0.006, RAIL_W - 0.002, RAIL_H + 0.004, centered=(True, True, False))
    rail = rail.union(pusher.translate((-RAIL_LEN / 2.0 + 0.003, 0.0, 0.0)))
    # Place along the arm underside: front of rail near the nose, just below crown.
    rail = rail.translate((0.030 + RAIL_LEN / 2.0, 0.0, 0.003))
    return rail


def _driver_blade() -> cq.Workplane:
    """The driver/anvil head at the front of the arm: a thin metal blade tongue
    that pushes the staple down. Authored in the ARM frame, at the nose."""
    blade = (
        cq.Workplane("XY")
        .box(0.006, 0.020, 0.016, centered=(True, True, False))
        .edges("|Z").fillet(0.001)
    )
    return blade.translate((ARM_LEN - 0.018, 0.0, 0.001))


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="plier_style_stapler")

    body_mat = model.material("body_plastic", rgba=COL_BODY)
    arm_mat = model.material("arm_plastic", rgba=COL_ARM)
    metal_mat = model.material("bright_steel", rgba=COL_METAL)
    dark_metal_mat = model.material("dark_steel", rgba=COL_DARK_METAL)

    # ---- Base (root) - curved plier-style grip handle --------------------
    base = model.part("base")
    base.visual(mesh_from_cadquery(_base_grip(), "base_grip"), material=body_mat, name="base_grip")
    
    # Add grip texture ridges using a loop with name_i style naming
    grip_ridges = _grip_texture_ridges()
    for i, ridge in enumerate(grip_ridges):
        ridge_name = f"grip_ridge_{i}"
        base.visual(
            mesh_from_cadquery(ridge, ridge_name),
            material=dark_metal_mat,
            name=ridge_name,
        )
    
    base.visual(mesh_from_cadquery(_anvil_plate(), "anvil_plate"), material=metal_mat, name="anvil_plate")
    base.visual(mesh_from_cadquery(_hinge_knuckles(), "hinge_knuckles"), material=dark_metal_mat, name="hinge_knuckles")

    # ---- Top arm (magazine cover / handle) -------------------------------
    arm = model.part("top_arm")
    arm.visual(mesh_from_cadquery(_arm_shell(), "arm_shell"), material=arm_mat, name="arm_shell")
    arm.visual(mesh_from_cadquery(_carrier_rail(), "carrier_rail"), material=metal_mat, name="carrier_rail")
    arm.visual(mesh_from_cadquery(_driver_blade(), "driver_blade"), material=metal_mat, name="driver_blade")
    arm.visual(mesh_from_cadquery(_hinge_pin(), "hinge_pin"), material=dark_metal_mat, name="hinge_pin")

    # ---- Rear hinge ------------------------------------------------------
    # The arm is authored with its pivot at the arm local origin and the cover
    # extending along +X. At q=0 the arm origin coincides with the hinge frame
    # placed on the base at (HINGE_X, 0, HINGE_Z). The closed cover extends
    # along +X above the deck. The cover extends +X from the pivot, so axis=-Y
    # makes positive q lift the front nose upward (right-hand rule).
    model.articulation(
        "base_to_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=arm,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=3.0, lower=0.0, upper=0.52),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    arm = object_model.get_part("top_arm")
    hinge = object_model.get_articulation("base_to_arm")

    # --- Joint contract ---------------------------------------------------
    ctx.check(
        "hinge is revolute",
        hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={hinge.articulation_type}",
    )
    ctx.check(
        "hinge axis is transverse (along -Y so positive q opens upward)",
        tuple(round(a, 6) for a in hinge.axis) == (0.0, -1.0, 0.0),
        details=f"axis={hinge.axis}",
    )
    lim = hinge.motion_limits
    ctx.check(
        "hinge opens upward from closed (lower=0, upper>0)",
        lim is not None and lim.lower == 0.0 and lim.upper is not None and lim.upper > 0.3,
        details=f"limits=({lim.lower if lim else None},{lim.upper if lim else None})",
    )

    # --- Hero parts exist -------------------------------------------------
    base_grip = base.get_visual("base_grip")
    anvil = base.get_visual("anvil_plate")
    knuckles = base.get_visual("hinge_knuckles")
    arm_shell = arm.get_visual("arm_shell")
    rail = arm.get_visual("carrier_rail")
    blade = arm.get_visual("driver_blade")
    pin = arm.get_visual("hinge_pin")
    
    # Verify grip texture ridges (loop-generated with name_i naming)
    grip_ridges = [base.get_visual(f"grip_ridge_{i}") for i in range(5)]
    
    for label, vis in [
        ("base_grip", base_grip),
        ("anvil_plate", anvil),
        ("hinge_knuckles", knuckles),
        ("arm_shell", arm_shell),
        ("carrier_rail", rail),
        ("driver_blade", blade),
        ("hinge_pin", pin),
    ]:
        ctx.check(f"{label} present", vis is not None, details=f"{label} missing")
    
    # Verify all grip ridges are present
    for i, ridge in enumerate(grip_ridges):
        ctx.check(
            f"grip_ridge_{i} present",
            ridge is not None,
            details=f"grip_ridge_{i} missing",
        )

    # --- Closed pose: the top cover sits above and overlaps the grip handle ----
    with ctx.pose({hinge: 0.0}):
        # The cover crown is clearly above the grip body (true clamshell stack):
        # the cover underside (arm_shell min-z) clears the grip.
        shell_aabb = ctx.part_element_world_aabb(arm, elem="arm_shell")
        grip_aabb = ctx.part_element_world_aabb(base, elem="base_grip")
        ctx.check(
            "cover crown rises well above the grip handle",
            shell_aabb is not None
            and grip_aabb is not None
            and shell_aabb[1][2] > grip_aabb[1][2] + 0.025,
            details=f"shell_top={shell_aabb[1][2]:.4f} grip_top={grip_aabb[1][2]:.4f}",
        )

        # Footprint of the cover overlaps the grip in plan view (it covers the
        # grip). Width is only ~40 mm, so require a modest y-overlap.
        ctx.expect_overlap(
            arm, base, axes="xy", min_overlap=0.03,
            name="closed cover covers the grip handle footprint",
        )

        # The metal anvil plate rests on the front platform of the grip,
        # not floating high above it nor sunk below it.
        anvil_aabb = ctx.part_element_world_aabb(base, elem="anvil_plate")
        ctx.check(
            "anvil plate seated on the grip front platform",
            anvil_aabb is not None and -0.012 < anvil_aabb[0][2] < 0.005,
            details=f"anvil_min_z={anvil_aabb[0][2] if anvil_aabb else None}",
        )

        # The driver blade hangs above the anvil plate (jaw closed but clearing),
        # proving the head registers over the clinch plate.
        blade_aabb = ctx.part_element_world_aabb(arm, elem="driver_blade")
        ctx.check(
            "driver blade registers above the anvil plate",
            blade_aabb is not None
            and anvil_aabb is not None
            and blade_aabb[0][2] >= anvil_aabb[1][2] - 0.001
            and abs(blade_aabb[0][0] - anvil_aabb[0][0]) < 0.04,
            details=f"blade_min_z={blade_aabb[0][2]:.4f} anvil_max_z={anvil_aabb[1][2]:.4f}",
        )

        # Hinge pin (arm) is captured by the base knuckles at the rear pivot.
        ctx.expect_contact(
            arm, base, elem_a=pin, elem_b=knuckles, contact_tol=0.0015,
            name="hinge pin captured by base knuckles",
        )
        closed_nose = ctx.part_element_world_aabb(arm, elem="driver_blade")
        
        # Verify the grip curves downward (key structural change from flat tray):
        # the grip bottom should extend well below the hinge level for hand-squeeze action
        grip_aabb = ctx.part_element_world_aabb(base, elem="base_grip")
        ctx.check(
            "grip handle curves downward below hinge level for hand-squeeze action",
            grip_aabb is not None and grip_aabb[0][2] < HINGE_Z - 0.020,
            details=f"grip_min_z={grip_aabb[0][2]:.4f}, hinge_z={HINGE_Z:.4f}",
        )

    # --- Squeeze action: opening the hinge raises the driver blade ----
    # In a hand-squeeze stapler, releasing the grip opens the jaws; squeezing
    # (closing the hinge) drives the staple. We verify the open pose lifts the blade.
    with ctx.pose({hinge: hinge.motion_limits.upper}):
        open_nose = ctx.part_element_world_aabb(arm, elem="driver_blade")

    ctx.check(
        "releasing grip opens the jaws (driver blade rises)",
        closed_nose is not None
        and open_nose is not None
        and open_nose[0][2] > closed_nose[0][2] + 0.03,
        details=f"closed_blade_min_z={closed_nose[0][2]:.4f}, open_blade_min_z={open_nose[0][2]:.4f}",
    )

    # The hinge pin is intentionally captured inside the base knuckle bores, and
    # the arm-side cheeks interleave with the base knuckles on the shared pin.
    # Allow those local hinge overlaps so the pivot reads as a real knuckle joint.
    ctx.allow_overlap(
        arm,
        base,
        elem_a=pin,
        elem_b=knuckles,
        reason="The transverse hinge pin is intentionally captured inside the base hinge-knuckle bores.",
    )
    ctx.allow_overlap(
        arm,
        base,
        elem_a=arm_shell,
        elem_b=knuckles,
        reason="The arm-side hinge cheeks interleave with the base knuckles on the shared pivot pin.",
    )

    return ctx.report()


object_model = build_object_model()
