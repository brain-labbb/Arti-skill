from __future__ import annotations

# Clear plastic bottle with tapered shoulder, molded volume bands,
# a removable measuring-cup cap, and a flip-up straw spout.
# Frame: vertical axis along +Z, bottle standing on z=0, centerline on x=y=0.
#   - rounded base -> straight cylindrical body (with raised volume bands)
#     -> long tapered shoulder -> short threaded neck
#   - measuring cup cap (open-bottom cup that lifts off the neck)
#   - straw spout pivots from flat (stored on cap top) to upright (drinking)
# Articulations:
#   - cap_lift:    PRISMATIC lift of the cup cap along +Z (removable).
#   - spout_pivot: REVOLUTE flip of the straw about -Y (flat -> upright).

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---- key heights (m) along +Z ----
BASE_Z = 0.0
BODY_TOP_Z = 0.110       # end of straight cylindrical body
SHOULDER_TOP_Z = 0.156   # end of tapered shoulder, base of neck
NECK_TOP_Z = 0.176       # top of threaded neck

BODY_R = 0.0275          # body radius (~55 mm dia)
NECK_R = 0.0125          # outer thread/neck radius
NECK_BORE_R = 0.0098     # neck inner bore

# ---- measuring-cup cap ----
CUP_R = 0.0210           # outer radius of measuring cup
CUP_HEIGHT = 0.030       # height of measuring cup
CUP_WALL = 0.0015        # wall / top-disc thickness
# Inner skirt that grips the neck (friction fit)
SKIRT_OUTER_R = NECK_R + 0.001   # slightly larger than neck
SKIRT_INNER_R = NECK_R - 0.001   # slightly smaller for grip
SKIRT_HEIGHT = 0.014             # skirt extends into the neck region

# ---- straw spout ----
STRAW_R = 0.0035         # straw outer radius
STRAW_LENGTH = 0.052     # straw tube length
STRAW_WALL = 0.0008      # straw wall thickness

# ---- volume bands ----
BAND_HEIGHTS = [0.030, 0.050, 0.070, 0.090]
BAND_MAJOR_R = BODY_R            # centerline at body surface
BAND_MINOR_R = 0.0010            # half inside, half proud (visible ridge)

# Cap mount: the cup frame sits so the inner skirt engages the upper neck.
# At rest (q=0) the cup bottom is below the neck top, wrapping over the neck.
CUP_MOUNT_Z = NECK_TOP_Z - SKIRT_HEIGHT  # = 0.162

# Spout hinge: on top of the cap, offset toward -X so the straw lies across
# the cap top and extends past the +X edge.
SPOUT_HINGE_X = -0.010
SPOUT_HINGE_Z = CUP_HEIGHT  # at the cap top surface


# ---------------------------------------------------------------------------
# Bottle body geometry
# ---------------------------------------------------------------------------

def _profile_sections():
    """(z, radius) of the outer wall from base to neck rim."""
    return [
        (0.000, 0.0150),
        (0.006, 0.0250),
        (0.014, 0.0273),
        (BODY_TOP_Z, BODY_R),
        (0.124, 0.0268),
        (0.138, 0.0228),
        (SHOULDER_TOP_Z, 0.0148),
        (0.160, NECK_R),
        (NECK_TOP_Z, NECK_R),
    ]


def _bottle_solid() -> cq.Workplane:
    pts = _profile_sections()
    wp = cq.Workplane("XZ").moveTo(0.0, pts[0][0])
    for r, z in [(r, z) for (z, r) in pts]:
        wp = wp.lineTo(r, z)
    wp = wp.lineTo(0.0, pts[-1][0]).close()
    outer = wp.revolve(360.0, (0, 0, 0), (0, 1, 0))

    wall = 0.0014
    inner_pts = [
        (0.010, 0.006),
        (0.0235, 0.012),
        (BODY_R - wall, 0.014),
        (BODY_R - wall, BODY_TOP_Z),
        (0.0254, 0.124),
        (0.0214, 0.138),
        (0.0134, SHOULDER_TOP_Z),
        (NECK_BORE_R, 0.160),
        (NECK_BORE_R, NECK_TOP_Z + 0.005),
    ]
    iwp = cq.Workplane("XZ").moveTo(0.0, inner_pts[0][1])
    for r, z in inner_pts:
        iwp = iwp.lineTo(r, z)
    iwp = iwp.lineTo(0.0, inner_pts[-1][1]).close()
    cavity = iwp.revolve(360.0, (0, 0, 0), (0, 1, 0))
    return outer.cut(cavity)


def _bottle_mesh():
    return mesh_from_cadquery(_bottle_solid(), "bottle_shell")


def _volume_bands():
    """Raised rings around the body as molded volume indicators."""
    g = None
    for z in BAND_HEIGHTS:
        band = TorusGeometry(
            BAND_MAJOR_R, BAND_MINOR_R,
            radial_segments=8, tubular_segments=48,
        )
        band.translate(0.0, 0.0, z)
        if g is None:
            g = band
        else:
            g.merge(band)
    return mesh_from_geometry(g, "volume_bands")


def _neck_threads():
    """Helical-ish thread rings on the neck."""
    g = None
    for zt in (0.163, 0.169):
        ring = TorusGeometry(
            NECK_R - 0.0006, 0.0012,
            radial_segments=10, tubular_segments=40,
        )
        ring.translate(0.0, 0.0, zt)
        if g is None:
            g = ring
        else:
            g.merge(ring)
    return mesh_from_geometry(g, "neck_threads")


# ---------------------------------------------------------------------------
# Measuring-cup cap
# ---------------------------------------------------------------------------

def _cup_cap_solid() -> cq.Workplane:
    """Measuring cup: open-bottom cylinder with a solid top disc and an inner
    friction skirt that grips the bottle neck.

    The outer cylinder is extruded to CUP_HEIGHT. A smaller cylinder is cut
    from the bottom, leaving side walls + a solid top disc (CUP_WALL thick).
    An inner ring (the skirt) extends from the cup bottom upward to engage
    the threaded neck, providing a realistic friction-fit contact.
    """
    cup = (
        cq.Workplane("XY")
        .circle(CUP_R)
        .extrude(CUP_HEIGHT)
    )
    bore = (
        cq.Workplane("XY")
        .circle(CUP_R - CUP_WALL)
        .extrude(CUP_HEIGHT - CUP_WALL)
    )
    cup = cup.cut(bore)
    # Inner skirt: thin ring that wraps snugly over the neck.
    skirt = (
        cq.Workplane("XY")
        .circle(SKIRT_OUTER_R)
        .circle(SKIRT_INNER_R)
        .extrude(SKIRT_HEIGHT)
    )
    cup = cup.union(skirt)
    # Connecting ledge: thin annular shelf at the skirt top that ties
    # the skirt to the cup inner wall (realistic thread-stop ring).
    ledge = (
        cq.Workplane("XY")
        .workplane(offset=SKIRT_HEIGHT - 0.002)
        .circle(CUP_R - CUP_WALL + 0.001)   # slightly into the cup wall
        .circle(SKIRT_INNER_R)
        .extrude(0.002)
    )
    cup = cup.union(ledge)
    return cup


def _cup_cap_mesh():
    return mesh_from_cadquery(_cup_cap_solid(), "cup_cap_shell")


def _cup_graduations():
    """Thin raised lines on the cup exterior as graduation marks.
    Each mark is partially embedded in the cup wall for geometric connectivity."""
    result = None
    for frac in (0.30, 0.55, 0.80):
        z = frac * (CUP_HEIGHT - CUP_WALL)
        # Center the mark at the cup outer surface so it overlaps with the wall.
        mark = (
            cq.Workplane("XY")
            .workplane(offset=z - 0.00025)
            .center(CUP_R - 0.0002, 0.0)
            .box(0.0008, 0.009, 0.0005, centered=(True, True, True))
        )
        if result is None:
            result = mark
        else:
            result = result.union(mark)
    return mesh_from_cadquery(result, "cup_graduations")


# ---------------------------------------------------------------------------
# Straw spout
# ---------------------------------------------------------------------------

def _straw_spout_solid() -> cq.Workplane:
    """Thin hollow tube extending along +X from the hinge origin."""
    outer = (
        cq.Workplane("YZ")
        .circle(STRAW_R)
        .extrude(STRAW_LENGTH)
    )
    bore = (
        cq.Workplane("YZ")
        .circle(STRAW_R - STRAW_WALL)
        .extrude(STRAW_LENGTH - 0.002)
    )
    return outer.cut(bore)


def _straw_spout_mesh():
    return mesh_from_cadquery(_straw_spout_solid(), "straw_tube")


# ---------------------------------------------------------------------------
# Build
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="measuring_cup_bottle")

    # Materials
    clear = model.material("clear_pet", rgba=(0.78, 0.85, 0.88, 0.25))
    band_mat = model.material("band_ridge", rgba=(0.72, 0.80, 0.84, 0.35))
    neck_mat = model.material("clear_neck", rgba=(0.72, 0.80, 0.84, 0.30))
    cup_mat = model.material("cup_blue", rgba=(0.18, 0.42, 0.72, 1.0))
    grad_mat = model.material("cup_marks", rgba=(0.92, 0.92, 0.94, 1.0))
    straw_mat = model.material("straw_gray", rgba=(0.82, 0.82, 0.80, 1.0))

    # ---- bottle body (root): transparent hollow PET shell ----
    body = model.part("bottle_body")
    body.visual(_bottle_mesh(), material=clear, name="bottle_shell")
    body.visual(_neck_threads(), material=neck_mat, name="neck_threads")
    body.visual(_volume_bands(), material=band_mat, name="volume_bands")
    body.inertial = Inertial.from_geometry(
        Cylinder(BODY_R, 0.176),
        mass=0.022,
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
    )

    # ---- measuring-cup cap ----
    cup_cap = model.part("cup_cap")
    cup_cap.visual(_cup_cap_mesh(), material=cup_mat, name="cup_cap_shell")
    cup_cap.visual(_cup_graduations(), material=grad_mat, name="cup_graduations")
    cup_cap.inertial = Inertial.from_geometry(
        Cylinder(CUP_R, CUP_HEIGHT),
        mass=0.008,
        origin=Origin(xyz=(0.0, 0.0, CUP_HEIGHT / 2.0)),
    )

    # ---- straw spout ----
    spout = model.part("straw_spout")
    spout.visual(_straw_spout_mesh(), material=straw_mat, name="straw_tube")
    # Hinge knuckle: short cylinder along Y (the pivot axis) at the straw base.
    # Rotated from +Z primitive to align along Y via roll = pi/2.
    spout.visual(
        Cylinder(STRAW_R + 0.001, 0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=straw_mat,
        name="straw_hinge",
    )
    spout.inertial = Inertial.from_geometry(
        Cylinder(STRAW_R, STRAW_LENGTH),
        mass=0.003,
        origin=Origin(xyz=(STRAW_LENGTH / 2.0, 0.0, 0.0)),
    )

    # ---- articulations ----

    # cap_lift: PRISMATIC along +Z.  q=0 -> cap seated on neck;
    # q=upper -> cap fully removed from the bottle.
    model.articulation(
        "cap_lift",
        ArticulationType.PRISMATIC,
        parent=body,
        child=cup_cap,
        origin=Origin(xyz=(0.0, 0.0, CUP_MOUNT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.045, effort=2.0, velocity=0.5),
    )

    # spout_pivot: REVOLUTE about -Y.
    # q=0 -> straw lies flat on the cap top (stored);
    # q=pi/2 -> straw stands upright for drinking.
    # axis=(0,-1,0) makes positive q rotate +X toward +Z (upward).
    model.articulation(
        "spout_pivot",
        ArticulationType.REVOLUTE,
        parent=cup_cap,
        child=spout,
        origin=Origin(xyz=(SPOUT_HINGE_X, 0.0, SPOUT_HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0, upper=math.pi / 2.0,
            effort=1.0, velocity=2.0,
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("bottle_body")
    cup_cap = object_model.get_part("cup_cap")
    spout = object_model.get_part("straw_spout")
    cap_lift = object_model.get_articulation("cap_lift")
    spout_pivot = object_model.get_articulation("spout_pivot")

    # --- bottle shell is clear (alpha < 1) ---
    clear_mat = next(m for m in object_model.materials if m.name == "clear_pet")
    alpha = clear_mat.rgba[3] if clear_mat.rgba is not None else 1.0
    ctx.check(
        "bottle shell is transparent",
        alpha < 1.0,
        details=f"clear_pet alpha={alpha}",
    )

    # --- molded volume bands are present ---
    bands = body.get_visual("volume_bands")
    ctx.check(
        "molded volume bands present on body",
        bands is not None,
        details="volume_bands visual not found on bottle_body",
    )

    # --- cup cap is present and has graduation marks ---
    grads = cup_cap.get_visual("cup_graduations")
    ctx.check(
        "cup cap has graduation marks",
        grads is not None,
        details="cup_graduations visual not found on cup_cap",
    )

    # --- cup cap sits at the top of the bottle at rest ---
    cap_pos = ctx.part_world_position(cup_cap)
    ctx.check(
        "cup cap mounted at top of bottle",
        cap_pos is not None and cap_pos[2] > 0.15,
        details=f"cup_cap origin z={cap_pos}",
    )

    # --- cup cap bore intentionally fits over the neck ---
    ctx.allow_overlap(
        cup_cap, body,
        elem_a="cup_cap_shell", elem_b="bottle_shell",
        reason="The cup cap bore intentionally fits over the threaded neck.",
    )
    ctx.allow_overlap(
        cup_cap, body,
        elem_a="cup_cap_shell", elem_b="neck_threads",
        reason="The cup cap covers the neck threads when seated.",
    )

    # --- cap lifts off the bottle (prismatic) ---
    cap_bottom_rest = ctx.part_world_aabb(cup_cap)[0][2]
    with ctx.pose({cap_lift: 0.045}):
        cap_bottom_lifted = ctx.part_world_aabb(cup_cap)[0][2]
    ctx.check(
        "cup cap lifts clear of the neck",
        cap_bottom_lifted > NECK_TOP_Z + 0.010,
        details=f"rest bottom={cap_bottom_rest:.4f}, lifted bottom={cap_bottom_lifted:.4f}, neck_top={NECK_TOP_Z:.4f}",
    )

    # --- straw spout lies flat at rest ---
    rest_aabb = ctx.part_world_aabb(spout)
    rest_dx = rest_aabb[1][0] - rest_aabb[0][0]
    rest_dz = rest_aabb[1][2] - rest_aabb[0][2]
    ctx.check(
        "straw lies flat at rest (wider than tall)",
        rest_dx > rest_dz * 1.5,
        details=f"rest dx={rest_dx:.4f}, dz={rest_dz:.4f}",
    )

    # --- straw spout stands upright when pivoted ---
    with ctx.pose({spout_pivot: math.pi / 2.0}):
        up_aabb = ctx.part_world_aabb(spout)
    up_dx = up_aabb[1][0] - up_aabb[0][0]
    up_dz = up_aabb[1][2] - up_aabb[0][2]
    ctx.check(
        "straw stands upright when pivoted (taller than wide)",
        up_dz > up_dx * 1.5,
        details=f"upright dx={up_dx:.4f}, dz={up_dz:.4f}",
    )

    # --- spout tip actually moves upward ---
    rest_tip_z = rest_aabb[1][2]
    up_tip_z = up_aabb[1][2]
    ctx.check(
        "spout tip rises when pivoted",
        up_tip_z > rest_tip_z + 0.020,
        details=f"rest tip z={rest_tip_z:.4f}, upright tip z={up_tip_z:.4f}",
    )

    # --- joint types ---
    ctx.check(
        "cap lift is prismatic",
        cap_lift.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={cap_lift.articulation_type}",
    )
    ctx.check(
        "spout pivot is revolute",
        spout_pivot.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={spout_pivot.articulation_type}",
    )

    # --- bottle proportions ---
    body_aabb = ctx.part_world_aabb(body)
    body_dx = body_aabb[1][0] - body_aabb[0][0]
    body_dz = body_aabb[1][2] - body_aabb[0][2]
    ctx.check(
        "bottle is tall (taller than wide)",
        body_dz > 2.5 * body_dx,
        details=f"body dx={body_dx:.4f}, dz={body_dz:.4f}",
    )
    ctx.check(
        "tapered shoulder narrows toward the top",
        NECK_R < BODY_R * 0.6,
        details=f"neck_r={NECK_R}, body_r={BODY_R}",
    )

    # --- straw hinge knuckle is intentionally embedded at the cap surface ---
    ctx.allow_overlap(
        spout, cup_cap,
        elem_a="straw_hinge", elem_b="cup_cap_shell",
        reason="The straw hinge knuckle is seated at the cap top surface as a pivot mount.",
    )
    ctx.allow_overlap(
        spout, cup_cap,
        elem_a="straw_tube", elem_b="cup_cap_shell",
        reason="The straw tube rests on the cap top surface in the stored (flat) position.",
    )
    ctx.expect_contact(
        spout, cup_cap,
        elem_a="straw_hinge", elem_b="cup_cap_shell",
        contact_tol=0.005,
        name="straw hinge contacts cap at pivot point",
    )

    return ctx.report()


object_model = build_object_model()
