from __future__ import annotations

# Sports bottle with a flip-top straw cap.
# Frame: bottle axis along +Z, base at z=0, flip cap at top (+Z).
# The body is a translucent thin-wall shell: rounded base + cylindrical
# barrel + shoulder taper + short neck with visible hollow mouth opening.
# A flip-top stopper pivots on side hinge arms (REVOLUTE joint about -Y)
# to open/close the mouth. Positive q opens the lid upward/backward.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---- key dimensions (meters) ----
BODY_R = 0.035          # outer barrel radius (~0.07 m dia)
WALL = 0.0018           # thin wall thickness
BASE_Z = 0.0            # bottom of the bottle
BARREL_TOP_Z = 0.155    # where the shoulder taper begins
SHOULDER_TOP_Z = 0.185  # top of shoulder, base of the neck
NECK_R = 0.016          # neck outer radius
NECK_TOP_Z = 0.210      # top rim of the neck (mouth opening)
MOUTH_R = 0.012         # inner mouth opening radius

# Flip cap dimensions
CAP_R = 0.019           # cap disc radius (slightly larger than neck)
CAP_THICK = 0.010       # cap disc thickness
SPOUT_R = 0.004         # drinking spout radius
SPOUT_H = 0.014         # spout height above cap

# Collar ring on the neck
COLLAR_R = 0.021        # collar outer radius
COLLAR_H = 0.012        # collar height

# Hinge geometry
HINGE_PIVOT_X = -(NECK_R + 0.005)  # hinge at the back (-X) of the neck collar
HINGE_PIVOT_Z = NECK_TOP_Z + COLLAR_H - 0.002  # near top of collar
HINGE_ARM_W = 0.004     # hinge arm width (Y direction)
HINGE_ARM_H = 0.015     # hinge arm height (from pivot down to collar)
HINGE_PIN_R = 0.0025    # hinge pin radius


def _bottle_shell():
    """Translucent thin-wall sports bottle as one solid revolve, shelled open
    at the top so the neck reads as a hollow mouth opening."""
    wp = (
        cq.Workplane("XZ")
        .moveTo(0.0, BASE_Z)
        # rounded base corner
        .lineTo(BODY_R - 0.008, BASE_Z)
        .threePointArc((BODY_R, BASE_Z + 0.008), (BODY_R, BASE_Z + 0.014))
        # straight cylindrical barrel
        .lineTo(BODY_R, BARREL_TOP_Z)
        # shoulder taper up to the neck
        .threePointArc(
            ((BODY_R + NECK_R) / 2.0, (BARREL_TOP_Z + SHOULDER_TOP_Z) / 2.0 + 0.006),
            (NECK_R, SHOULDER_TOP_Z),
        )
        # straight neck
        .lineTo(NECK_R, NECK_TOP_Z)
        # close along axis at top (this will be shelled away)
        .lineTo(0.0, NECK_TOP_Z)
        .close()
    )
    outer = wp.revolve(360.0, (0, 0, 0), (0, 1, 0))
    # Shell: remove top face and hollow inward to create the mouth opening
    return outer.faces(">Z").shell(-WALL)


def _collar_with_hinge():
    """A collar ring that sits on the neck with two hinge bracket arms
    extending upward at the back (-X side). One connected solid.
    The collar overlaps the neck for physical contact with the bottle."""
    # Collar ring slips over the neck: starts slightly below neck top
    collar_z_start = NECK_TOP_Z - 0.006  # overlap with neck for contact
    collar_h = COLLAR_H + 0.006
    collar = (
        cq.Workplane("XY")
        .transformed(offset=(0, 0, collar_z_start))
        .circle(COLLAR_R)
        .circle(NECK_R + 0.001)  # hollow to slip over neck
        .extrude(collar_h)
    )

    # Hinge bracket: two upright plates at the back of the collar
    # These overlap with the collar body for connectivity
    bracket_y_spacing = 0.010
    plate_thickness = 0.004
    plate_height = collar_h + 0.010  # extends above collar top, rooted inside collar
    plate_width = 0.010

    for y_off in [-bracket_y_spacing / 2.0, bracket_y_spacing / 2.0]:
        plate_z_base = collar_z_start + 0.002
        plate = (
            cq.Workplane("XY")
            .transformed(
                offset=(HINGE_PIVOT_X + plate_width / 2.0, y_off, plate_z_base + plate_height / 2.0)
            )
            .box(plate_width, plate_thickness, plate_height)
        )
        collar = collar.union(plate)

    # Hinge pin: a box spanning between the two bracket plates along Y
    pin_y_len = bracket_y_spacing + plate_thickness + 0.002
    pin_box = (
        cq.Workplane("XY")
        .transformed(offset=(HINGE_PIVOT_X, 0, HINGE_PIVOT_Z))
        .box(HINGE_PIN_R * 2, pin_y_len, HINGE_PIN_R * 2)
    )
    collar = collar.union(pin_box)

    return collar


def _flip_cap():
    """Flip-top cap: a disc with a raised spout and two hinge lugs that
    wrap around the pivot pin.

    Local frame: origin at the hinge pivot point (HINGE_PIVOT_X, 0, HINGE_PIVOT_Z).
    At q=0 the cap extends along +X from the hinge to cover the mouth.
    axis=(0,-1,0) makes positive q lift the free edge upward.
    """
    # Disc center in local coords: extends along +X from pivot
    # When closed, disc should be centered over the mouth at x=0
    # pivot is at HINGE_PIVOT_X (negative), so disc center x = -HINGE_PIVOT_X (positive)
    disc_cx = -HINGE_PIVOT_X  # positive, toward bottle center
    disc_cz = 0.0  # at the same z as the pivot (cap sits at pivot height)

    # Main disc (cylinder along Z)
    cap = (
        cq.Workplane("XY")
        .transformed(offset=(disc_cx, 0, disc_cz))
        .circle(CAP_R)
        .extrude(CAP_THICK)
    )
    cap = cap.edges(">Z").fillet(0.002)

    # Drinking spout on top of the disc (offset toward front)
    spout_x = disc_cx + 0.003  # slightly forward of center
    spout = (
        cq.Workplane("XY")
        .transformed(offset=(spout_x, 0, disc_cz + CAP_THICK))
        .circle(SPOUT_R)
        .extrude(SPOUT_H)
    )
    cap = cap.union(spout)

    # Hinge lugs: two small boxes at the back edge of the cap that
    # wrap around the hinge pin (local origin = pivot)
    lug_size = HINGE_PIN_R * 2 + 0.002
    lug_width = 0.005
    for y_off in [-0.003, 0.003]:
        lug = (
            cq.Workplane("XY")
            .transformed(offset=(0, y_off, 0))
            .box(lug_size, lug_width, lug_size)
        )
        cap = cap.union(lug)

    # Bridge connecting hinge lugs to the disc
    bridge = (
        cq.Workplane("XY")
        .transformed(offset=(disc_cx * 0.4, 0, disc_cz + CAP_THICK / 2.0))
        .box(abs(disc_cx) * 0.8, CAP_R * 0.6, CAP_THICK)
    )
    cap = cap.union(bridge)

    return cap


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sports_bottle")

    # Translucent blue sports bottle body and dark grey flip cap
    translucent = model.material("bottle_blue", rgba=(0.30, 0.55, 0.75, 0.40))
    dark_grey = model.material("cap_grey", rgba=(0.18, 0.18, 0.20, 1.0))

    # ---- bottle body (root) ----
    bottle = model.part("bottle")
    shell = _bottle_shell()
    bottle.visual(
        mesh_from_cadquery(shell, "bottle_shell"),
        material=translucent,
        name="bottle_shell",
    )
    bottle.inertial = Inertial.from_geometry(
        Cylinder(BODY_R, NECK_TOP_Z),
        mass=0.045,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z / 2.0)),
    )

    # ---- collar with hinge brackets (rigidly mounted on neck) ----
    collar = model.part("collar")
    collar_geo = _collar_with_hinge()
    collar.visual(
        mesh_from_cadquery(collar_geo, "collar_shell"),
        material=dark_grey,
        name="collar_shell",
    )
    collar.inertial = Inertial.from_geometry(
        Cylinder(COLLAR_R, COLLAR_H + 0.01),
        mass=0.005,
        origin=Origin(xyz=(0.0, 0.0, NECK_TOP_Z + (COLLAR_H + 0.01) / 2.0)),
    )

    # ---- flip-top cap ----
    flip_lid = model.part("flip_lid")
    cap_geo = _flip_cap()
    flip_lid.visual(
        mesh_from_cadquery(cap_geo, "cap_disc"),
        material=dark_grey,
        name="cap_disc",
    )
    flip_lid.inertial = Inertial.from_geometry(
        Cylinder(CAP_R, CAP_THICK + SPOUT_H),
        mass=0.008,
        origin=Origin(xyz=(-HINGE_PIVOT_X, 0.0, (CAP_THICK + SPOUT_H) / 2.0)),
    )

    # ---- collar fixed to bottle ----
    model.articulation(
        "bottle_to_collar",
        ArticulationType.FIXED,
        parent=bottle,
        child=collar,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # ---- flip hinge joint ----
    # REVOLUTE about -Y axis at the back-side hinge pivot.
    # At q=0 the cap is closed (covering the mouth).
    # Positive q rotates the cap upward/backward (opening).
    # Cap extends along +X from hinge; axis=(0,-1,0) lifts the free edge.
    model.articulation(
        "flip_hinge",
        ArticulationType.REVOLUTE,
        parent=collar,
        child=flip_lid,
        origin=Origin(xyz=(HINGE_PIVOT_X, 0.0, HINGE_PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=2.4,
            effort=2.0,
            velocity=4.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bottle = object_model.get_part("bottle")
    collar = object_model.get_part("collar")
    flip_lid = object_model.get_part("flip_lid")
    hinge = object_model.get_articulation("flip_hinge")

    bottle_shell = bottle.get_visual("bottle_shell")
    cap_disc = flip_lid.get_visual("cap_disc")

    # --- bottle is translucent (alpha < 1), cap is opaque dark ---
    ctx.check(
        "bottle material is translucent (alpha < 1)",
        bottle_shell.material.rgba is not None and bottle_shell.material.rgba[3] < 1.0,
        details=f"bottle rgba={bottle_shell.material.rgba}",
    )
    ctx.check(
        "cap material is opaque dark grey",
        cap_disc.material.rgba is not None
        and cap_disc.material.rgba[3] >= 0.99
        and max(cap_disc.material.rgba[:3]) < 0.35,
        details=f"cap rgba={cap_disc.material.rgba}",
    )

    # --- collar is mounted on the neck ---
    collar_aabb = ctx.part_world_aabb(collar)
    if collar_aabb is not None:
        collar_zmin = collar_aabb[0][2]
        collar_zmax = collar_aabb[1][2]
        ctx.check(
            "collar mounted on the neck (above shoulder, near neck top)",
            collar_zmin > SHOULDER_TOP_Z - 0.005 and collar_zmax > NECK_TOP_Z,
            details=f"collar z-range=({collar_zmin:.4f}, {collar_zmax:.4f})",
        )

    # --- flip lid sits at the top ---
    lid_pos = ctx.part_world_position(flip_lid)
    ctx.check(
        "flip lid mounted above barrel top",
        lid_pos is not None and lid_pos[2] > BARREL_TOP_Z,
        details=f"lid origin={lid_pos}",
    )

    # --- collar slips over the neck (intentional seated overlap) ---
    ctx.allow_overlap(
        collar,
        bottle,
        elem_a="collar_shell",
        elem_b="bottle_shell",
        reason="Collar ring is intentionally seated over the bottle neck for a secure mount.",
    )
    ctx.expect_contact(
        collar,
        bottle,
        elem_a="collar_shell",
        elem_b="bottle_shell",
        name="collar contacts bottle neck",
    )

    # --- hinge arms/pin overlap with collar bracket (intentional pivot) ---
    ctx.allow_overlap(
        flip_lid,
        collar,
        elem_a="cap_disc",
        elem_b="collar_shell",
        reason="Cap hinge lugs wrap around the collar hinge pin for pivot articulation.",
    )

    # --- hinge is revolute with valid limits ---
    ctx.check(
        "flip_hinge is a revolute joint with positive range",
        hinge.articulation_type == ArticulationType.REVOLUTE
        and hinge.motion_limits.upper > hinge.motion_limits.lower,
        details=f"type={hinge.articulation_type}, limits=({hinge.motion_limits.lower}, {hinge.motion_limits.upper})",
    )

    # --- at rest (q=0), the lid covers the mouth ---
    rest_aabb = ctx.part_world_aabb(flip_lid)
    rest_zmax = rest_aabb[1][2] if rest_aabb else 0.0

    # --- at open pose (q=1.2, ~70°), the lid is clearly raised ---
    with ctx.pose({hinge: 1.2}):
        mid_aabb = ctx.part_world_aabb(flip_lid)
    mid_zmax = mid_aabb[1][2] if mid_aabb else 0.0

    ctx.check(
        "flip hinge opens the lid upward (z-max increases at mid-open)",
        mid_zmax > rest_zmax + 0.005,
        details=f"rest_zmax={rest_zmax:.4f}, mid_zmax={mid_zmax:.4f}",
    )

    # --- at full open (q=2.4), the lid is flipped back ---
    with ctx.pose({hinge: 2.4}):
        full_aabb = ctx.part_world_aabb(flip_lid)
    full_xmin = full_aabb[0][0] if full_aabb else 0.0
    rest_xmin = rest_aabb[0][0] if rest_aabb else 0.0

    ctx.check(
        "flip hinge swings lid backward (x-min shifts at full open)",
        abs(full_xmin - rest_xmin) > 0.005,
        details=f"rest_xmin={rest_xmin:.4f}, full_xmin={full_xmin:.4f}",
    )

    # --- bottle body has sports-bottle proportions ---
    bottle_aabb = ctx.part_world_aabb(bottle)
    if bottle_aabb is not None:
        mn, mx = bottle_aabb
        height = mx[2] - mn[2]
        ctx.check(
            "bottle has sports-bottle proportions (height > 0.15m)",
            height > 0.15,
            details=f"bottle height={height:.4f}m",
        )

    return ctx.report()


object_model = build_object_model()
