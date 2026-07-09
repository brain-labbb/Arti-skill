from __future__ import annotations

# Round porthole window with a glossy black ring frame and a center-pivot
# circular sash. Reference image 008: a circular window with a thick glossy
# black outer ring frame, an inner concentric black sash ring holding a round
# dark/reflective glass disc (visible concentric rings), mounted on a wall.
#
# Coordinate convention (Z up):
#   - The window is a vertical disc. Its circular face lies in the X-Z plane;
#     the frame depth / glazing thickness / pivot-normal runs along Y.
#   - The assembly is built so its bottom sits near z=0 (porthole standing on a
#     wall, not centered below the floor).
#   - Center-pivot articulation: the inner sash pivots about a HORIZONTAL
#     diameter axis (1,0,0) through the circle center, rotating within the outer
#     ring. q=0 = closed (sash coplanar with the frame); the open pose ~0.8 rad
#     tips the top of the sash inward / bottom outward like a hopper-pivot.
#
# Structure:
#   - frame (static root): a thick annular black ring (disc with a central bore)
#     plus a shallow back collar, built in CadQuery. This is the fixed outer ring.
#   - sash (moving): a thinner black ring + a round glass disc filling the bore,
#     authored centered on the circle center so the pivot sits at the center.

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ---------------------------------------------------------------------------
# Absolute dimensions (meters)
# ---------------------------------------------------------------------------

OUTER_R = 0.300        # outer radius of the black ring frame (0.60 m diameter)
RING_W = 0.055         # radial width of the outer ring (thick glossy bezel)
INNER_BORE_R = OUTER_R - RING_W   # inner clear radius of the outer ring = 0.245
FRAME_DEPTH = 0.075    # outer ring depth along Y

# Back collar (a shallower tube behind the bezel that sets the window into a wall).
COLLAR_R = INNER_BORE_R + 0.018
COLLAR_DEPTH = 0.050

# Inner sash: a thinner ring that seats inside the outer bore, plus glass.
SASH_OUTER_R = INNER_BORE_R - 0.004   # small running clearance inside the bore
SASH_RING_W = 0.030                    # radial width of the inner sash ring
SASH_GLASS_R = SASH_OUTER_R - SASH_RING_W + 0.006  # glass tucks under the ring lip
SASH_DEPTH = 0.045     # sash ring depth along Y
GLASS_T = 0.010        # glass disc thickness along Y

CENTER_Z = OUTER_R + 0.02   # circle center height so the bottom sits ~z=0.02

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

BLACK_RGBA = (0.045, 0.045, 0.050, 1.0)   # glossy near-black frame/sash
GLASS_RGBA = (0.05, 0.06, 0.09, 0.45)     # dark reflective glass, semi-transparent
PIN_RGBA = (0.30, 0.31, 0.33, 1.0)        # dark satin pivot-pin metal

# Pivot pins: two short studs on the horizontal diameter that capture the sash
# to the outer ring and carry the center-pivot axis. The pin length spans the
# running clearance plus a few mm of engagement into each member.
PIN_R = 0.010
PIN_LEN = 0.030


# ---------------------------------------------------------------------------
# Geometry helpers (CadQuery). Circular face in X-Z plane; axis (tube) along Y.
# We build on the "XZ" workplane so extrusions run along the global Y axis.
# ---------------------------------------------------------------------------

def _ring_shape(outer_r: float, inner_r: float, depth: float) -> cq.Workplane:
    """An annular ring (tube): outer circle minus inner bore, extruded along Y.

    Built on the XZ workplane and centered at the origin, then translated to the
    circle center height by the caller via the part visual origin.
    """
    return (
        cq.Workplane("XZ")
        .circle(outer_r)
        .circle(inner_r)
        .extrude(depth, both=True)  # symmetric about Y=0
    )


def _disc_shape(radius: float, depth: float) -> cq.Workplane:
    """A solid disc extruded along Y (for the back collar / glass)."""
    return (
        cq.Workplane("XZ")
        .circle(radius)
        .extrude(depth, both=True)
    )


def _build_outer_frame_shape() -> cq.Workplane:
    """Static glossy-black ring frame: thick front bezel ring + back collar tube.

    Authored centered on the origin in X/Y/Z; the part visual origin lifts it to
    CENTER_Z so the porthole stands with its bottom near z=0.
    """
    bezel = _ring_shape(OUTER_R, INNER_BORE_R, FRAME_DEPTH / 2.0)
    # Back collar sits behind the bezel (+Y is the back / wall side), a shallower
    # tube that gives the frame real depth into the wall.
    collar = (
        cq.Workplane("XZ")
        .workplane(offset=FRAME_DEPTH / 2.0)  # start at the back face of the bezel
        .circle(COLLAR_R + RING_W * 0.5)
        .circle(COLLAR_R)
        .extrude(COLLAR_DEPTH)
    )
    return bezel.union(collar)


def _build_sash_ring_shape() -> cq.Workplane:
    """Inner circular sash ring (thinner black ring), centered on the origin."""
    return _ring_shape(SASH_OUTER_R, SASH_OUTER_R - SASH_RING_W, SASH_DEPTH / 2.0)


def _build_sash_glass_shape() -> cq.Workplane:
    """Round glass disc filling the sash, tucked under the sash ring lip."""
    return _disc_shape(SASH_GLASS_R, GLASS_T / 2.0)


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="round_porthole_window")

    model.material("black", rgba=BLACK_RGBA)
    model.material("glass", rgba=GLASS_RGBA)
    model.material("pin", rgba=PIN_RGBA)

    # --- Static outer ring frame (root) ---
    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_outer_frame_shape(), "frame"),
        origin=Origin(xyz=(0.0, 0.0, CENTER_Z)),
        material="black",
        name="frame_ring",
    )

    # --- Inner center-pivot sash (ring + round glass), authored centered on the
    #     circle center so the pivot axis passes through the center. The part
    #     frame is the pivot point; the joint origin places it at the world
    #     center, so the visuals sit at local origin (no offset here). ---
    sash = model.part("sash")
    sash.visual(
        mesh_from_cadquery(_build_sash_ring_shape(), "sash_ring"),
        material="black",
        name="sash_ring",
    )
    sash.visual(
        mesh_from_cadquery(_build_sash_glass_shape(), "sash_glass"),
        material="glass",
        name="sash_glass",
    )

    # Two pivot pins on the horizontal diameter (the +/-X ends of the sash ring).
    # Each pin runs along X, straddling the running clearance: it engages into the
    # sash ring on the inner side and reaches into the outer-ring bore wall on the
    # outer side, capturing the sash to the frame and defining the pivot axis.
    # Cylinder long axis is local Z, so rpy=(0, pi/2, 0) aligns it to world X.
    pin_center_x = (SASH_OUTER_R + INNER_BORE_R) / 2.0
    for sign, tag in ((1.0, "right"), (-1.0, "left")):
        sash.visual(
            Cylinder(radius=PIN_R, length=PIN_LEN),
            origin=Origin(xyz=(sign * pin_center_x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material="pin",
            name=f"pivot_pin_{tag}",
        )

    # ----- Articulation -----
    # Center-pivot porthole: the sash pivots about the horizontal diameter axis
    # (1,0,0) through the circle center. Joint origin at the world circle center
    # (0,0,CENTER_Z). At q=0 the sash is coplanar with the frame (closed). A
    # positive rotation about +X tips the sash's top edge toward +Y (back/in) and
    # bottom toward -Y (front/out), opening it like the photo.
    model.articulation(
        "frame_to_sash",
        ArticulationType.REVOLUTE,
        parent="frame",
        child="sash",
        origin=Origin(xyz=(0.0, 0.0, CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=-0.8, upper=0.8),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

OPEN_POSE = 0.8  # radians, the opening pose


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    sash = object_model.get_part("sash")
    pivot = object_model.get_articulation("frame_to_sash")

    # --- Intentional overlaps ---
    # Glass disc is rebated under the sash ring lip (captured glass).
    ctx.allow_overlap(
        sash, sash,
        elem_a="sash_glass",
        elem_b="sash_ring",
        reason="Round glass disc is rebated under the inner sash ring lip so it reads as captured, not floating.",
    )
    # The two pivot pins capture the sash to the outer ring: each pin engages
    # into the sash ring on its inner half and into the outer-ring bore wall on
    # its outer half. That is the real center-pivot support path, so allow the
    # pin-into-frame and pin-into-sash embeddings.
    ctx.allow_overlap(
        frame, sash,
        reason="The two horizontal-diameter pivot pins reach from the sash into the outer-ring bore wall, capturing the sash to the frame on the center-pivot axis.",
    )
    for tag in ("left", "right"):
        ctx.allow_overlap(
            sash, sash,
            elem_a=f"pivot_pin_{tag}",
            elem_b="sash_ring",
            reason="Pivot pin is seated into the sash ring to carry the center-pivot axis.",
        )

    # --- Frame is the static root, round, and larger than the sash ---
    frame_aabb = ctx.part_world_aabb(frame)
    fw = frame_aabb[1][0] - frame_aabb[0][0]   # X span (diameter)
    fh = frame_aabb[1][2] - frame_aabb[0][2]   # Z span (diameter)
    ctx.check(
        "outer frame is round (square X-Z bounding box)",
        abs(fw - fh) < 0.02,
        details=f"frame X span={fw:.3f}, Z span={fh:.3f}",
    )
    ctx.check(
        "frame diameter is porthole-scale (~0.5-0.7 m)",
        0.45 < fw < 0.75,
        details=f"frame diameter={fw:.3f}",
    )
    # Porthole stands on the wall: bottom near z=0.
    ctx.check(
        "porthole bottom sits near the floor (z>=0)",
        frame_aabb[0][2] >= -1e-4 and frame_aabb[0][2] < 0.06,
        details=f"frame zmin={frame_aabb[0][2]:.4f}",
    )

    with ctx.pose({pivot: 0.0}):
        sash_aabb = ctx.part_world_aabb(sash)
        sw = sash_aabb[1][0] - sash_aabb[0][0]
        ctx.check(
            "sash sits inside the outer ring bore",
            sw < fw - 0.04,
            details=f"sash X span={sw:.3f}, frame X span={fw:.3f}",
        )
        # Closed sash is concentric with the frame (centers align in X and Z).
        sash_cx = (sash_aabb[0][0] + sash_aabb[1][0]) / 2.0
        sash_cz = (sash_aabb[0][2] + sash_aabb[1][2]) / 2.0
        frame_cx = (frame_aabb[0][0] + frame_aabb[1][0]) / 2.0
        frame_cz = (frame_aabb[0][2] + frame_aabb[1][2]) / 2.0
        ctx.check(
            "closed sash is concentric with the ring",
            abs(sash_cx - frame_cx) < 0.01 and abs(sash_cz - frame_cz) < 0.01,
            details=f"sash center=({sash_cx:.3f},{sash_cz:.3f}), frame center=({frame_cx:.3f},{frame_cz:.3f})",
        )
        # Closed pose is shallow in Y (seated in the ring plane).
        closed_dy = sash_aabb[1][1] - sash_aabb[0][1]
        ctx.check(
            "closed sash is shallow in Y (seated in ring plane)",
            closed_dy < 0.10,
            details=f"closed sash Y depth={closed_dy:.3f}",
        )
        # The sash sits within the frame footprint in the closed plane.
        ctx.expect_within(
            sash, frame,
            axes=("x", "z"),
            margin=0.005,
            name="closed sash stays within the outer ring footprint",
        )
        # Pivot pins make real contact with the outer ring (support path is real,
        # not floating), and lie on the horizontal diameter (Z ~ center).
        ctx.expect_contact(
            sash, frame,
            elem_a="pivot_pin_right",
            name="right pivot pin contacts the outer ring",
        )
        ctx.expect_contact(
            sash, frame,
            elem_a="pivot_pin_left",
            name="left pivot pin contacts the outer ring",
        )
        for tag in ("left", "right"):
            pin_aabb = ctx.part_element_world_aabb(sash, elem=f"pivot_pin_{tag}")
            pin_cz = (pin_aabb[0][2] + pin_aabb[1][2]) / 2.0
            ctx.check(
                f"{tag} pivot pin lies on the horizontal diameter",
                abs(pin_cz - CENTER_Z) < 0.01,
                details=f"{tag} pin Z center={pin_cz:.3f}, center Z={CENTER_Z:.3f}",
            )
        closed_top = ctx.part_element_world_aabb(sash, elem="sash_ring")
        closed_top_y = closed_top[1][1]  # max Y of the sash ring when shut

    # --- HERO: pivoting about the center diameter tilts the sash out of plane,
    #     while its center stays put at the circle center (it pivots about its
    #     own center, not a rim hinge) ---
    rest_cx = (sash_aabb[0][0] + sash_aabb[1][0]) / 2.0
    rest_cz = (sash_aabb[0][2] + sash_aabb[1][2]) / 2.0
    with ctx.pose({pivot: OPEN_POSE}):
        open_aabb = ctx.part_world_aabb(sash)
        open_dy = open_aabb[1][1] - open_aabb[0][1]
        # The sash now occupies much more Y depth (it has tilted out of plane).
        ctx.check(
            "open sash tilts out of the ring plane (grows in Y)",
            open_dy > closed_dy + 0.15,
            details=f"closed Y depth={closed_dy:.3f}, open Y depth={open_dy:.3f}",
        )
        # Center-pivot proof: the sash center stays at the circle center in X/Z
        # (it rotates about its own center, not swinging from a rim).
        open_cx = (open_aabb[0][0] + open_aabb[1][0]) / 2.0
        open_cz = (open_aabb[0][2] + open_aabb[1][2]) / 2.0
        ctx.check(
            "sash pivots about its own center (X stays put)",
            abs(open_cx - rest_cx) < 0.01,
            details=f"rest cx={rest_cx:.3f}, open cx={open_cx:.3f}",
        )
        ctx.check(
            "sash pivots about its own center (Z center stays put)",
            abs(open_cz - rest_cz) < 0.01,
            details=f"rest cz={rest_cz:.3f}, open cz={open_cz:.3f}",
        )
        # Top edge of the sash swings to +Y (back), bottom to -Y (front): the two
        # halves separate across the pivot diameter.
        ctx.check(
            "open sash spans both sides of the closed plane",
            open_aabb[0][1] < closed_top_y - 0.05 and open_aabb[1][1] > closed_top_y + 0.02,
            details=f"open Y range=({open_aabb[0][1]:.3f},{open_aabb[1][1]:.3f}), closed back Y={closed_top_y:.3f}",
        )

    return ctx.report()


object_model = build_object_model()
