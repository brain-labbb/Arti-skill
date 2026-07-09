from __future__ import annotations

# Dark glass beer bottle with a threaded screw cap.
# Frame: bottle axis along +Z (base at z=0, mouth/lip at the top), centered on
#   the z-axis. Scale ~0.24 m tall, ~0.065 m body diameter.
# Articulations:
#   - screw cap: CONTINUOUS, rotates about +Z (bottle axis) at the lip top
#     contact surface. Cap spins to seat/unseat on the threaded bottle mouth.

import math

import cadquery as cq

GLASS_RGBA = (0.18, 0.12, 0.07, 0.4)
CAP_RGBA = (0.62, 0.60, 0.58, 1.0)

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

# ---- Bottle profile heights (z, in metres) ----------------------------------
BASE_Z = 0.0
BODY_TOP_Z = 0.120  # body cylinder ends, shoulder begins
SHOULDER_TOP_Z = 0.150  # shoulder blends into the neck
NECK_TOP_Z = 0.222  # top of the slim neck, where the lip starts
LIP_TOP_Z = 0.230  # top face of the mouth / lip

BODY_R = 0.0325  # body radius (~0.065 m diameter)
NECK_R = 0.0115  # slim long-neck radius
LIP_R = 0.0135  # rolled lip is slightly fatter than the neck

WALL = 0.0028  # glass wall thickness (hollow shell)
BORE_R = 0.0085  # inner bore radius at the mouth

# ---- Screw cap dimensions ---------------------------------------------------
CAP_OUTER_R = 0.0170  # outer radius of the knurled shell
CAP_BORE_R = 0.0150  # inner bore radius (clearance over lip + neck threads)
CAP_SKIRT_H = 0.020  # skirt extends below the lip top over the thread zone
CAP_TOP_WALL = 0.003  # solid top wall thickness
CAP_TOTAL_H = CAP_SKIRT_H + CAP_TOP_WALL  # total cap height

# Knurling
N_KNURLS = 36  # number of knurl ridges around circumference
KNURL_DEPTH = 0.0008  # radial protrusion of each knurl ridge

# Threading — interleaved cap and neck thread rings create face-to-face contact
N_THREADS = 3  # number of thread rings (cap and neck)
THREAD_HEIGHT = 0.0010  # axial thickness of each thread ring
THREAD_SPACING = 0.0045  # center-to-center axial spacing between rings
THREAD_DEPTH = 0.0012  # radial protrusion of neck thread rings

# Cap thread inner radius protrudes past neck thread outer radius for
# radial overlap at the contact face (cap thread top meets neck thread bottom).
# Neck thread outer = NECK_R + THREAD_DEPTH = 0.0127.
CAP_THREAD_INNER_R = 0.0120  # cap thread inner edge (< neck thread outer)


def _profile_loft(sections, ruled: bool = True) -> cq.Workplane:
    # sections: list of (z, radius). Lofts circular sections stacked along +Z.
    wp = cq.Workplane("XY")
    prev = 0.0
    for i, (z, r) in enumerate(sections):
        off = z if i == 0 else z - prev
        if i > 0 or abs(off) > 1e-12:
            wp = wp.workplane(offset=off)
        wp = wp.circle(r)
        prev = z
    return wp.loft(ruled=ruled)


def _bottle_solid() -> cq.Workplane:
    # Outer silhouette: rounded base -> straight body -> sloped shoulder ->
    # long slim neck -> slightly flared rolled lip.
    outer = _profile_loft(
        [
            (BASE_Z, 0.0235),
            (0.003, 0.0250),
            (0.008, 0.0305),
            (0.013, 0.0322),
            (0.018, 0.0325),
            (BODY_TOP_Z, 0.0325),
            (0.128, 0.0316),
            (0.137, 0.0288),
            (SHOULDER_TOP_Z, 0.0205),
            (0.160, 0.0150),
            (0.170, 0.0122),
            (NECK_TOP_Z, NECK_R),
            (0.224, 0.0128),
            (0.227, LIP_R),
            (LIP_TOP_Z, LIP_R),
        ]
    )

    # Hollow it: subtract an inner cavity that opens at the mouth.
    inner = _profile_loft(
        [
            (0.010, 0.0200),
            (0.018, 0.0270),
            (0.024, 0.0297),
            (BODY_TOP_Z - 0.004, 0.0297),
            (0.128, 0.0288),
            (0.137, 0.0260),
            (SHOULDER_TOP_Z, 0.0177),
            (0.160, 0.0122),
            (0.170, 0.0094),
            (NECK_TOP_Z, BORE_R),
            (LIP_TOP_Z + 0.004, BORE_R),
        ]
    )
    return outer.cut(inner)


def _thread_ring(z: float, inner_r: float, outer_r: float) -> cq.Workplane:
    """Shared helper: a single annular thread ring at height z."""
    return (
        cq.Workplane("XY")
        .workplane(offset=z)
        .circle(outer_r)
        .circle(inner_r)
        .extrude(THREAD_HEIGHT)
    )


def _neck_thread_z_starts():
    """World-z start heights for neck thread rings.

    Neck thread i sits directly above cap thread i: the neck thread bottom
    face (at z_start) contacts the cap thread top face.
    """
    # Cap thread i in local coords: z_start = -CAP_SKIRT_H + i * THREAD_SPACING
    # Cap thread i top face (z_end) in world = LIP_TOP_Z + (-CAP_SKIRT_H + (i+1)*THREAD_SPACING - THREAD_SPACING + THREAD_HEIGHT)
    # Simpler: cap thread i world z_end = LIP_TOP_Z - CAP_SKIRT_H + i * THREAD_SPACING + THREAD_HEIGHT
    # Neck thread i world z_start = cap thread i world z_end (contact face)
    result = []
    for i in range(N_THREADS):
        cap_z_end_world = LIP_TOP_Z - CAP_SKIRT_H + i * THREAD_SPACING + THREAD_HEIGHT
        result.append(cap_z_end_world)
    return result


def _neck_threads_mesh():
    """External thread rings on the bottle neck for the screw cap to engage."""
    thread_inner_r = NECK_R - 0.0005  # overlap with neck wall for connectivity
    thread_outer_r = NECK_R + THREAD_DEPTH

    z_starts = _neck_thread_z_starts()
    result = None
    for i in range(N_THREADS):
        ring = _thread_ring(z_starts[i], thread_inner_r, thread_outer_r)
        result = ring if result is None else result.union(ring)

    return mesh_from_cadquery(result, "neck_threads")


def _screw_cap_mesh():
    """Knurled cylindrical screw cap with internal thread rings."""
    # --- Knurled outer shell: star-profile extrusion ---
    r_root = CAP_OUTER_R - KNURL_DEPTH
    r_crest = CAP_OUTER_R
    pts = []
    for i in range(N_KNURLS * 2):
        ang = math.pi * i / N_KNURLS
        r = r_crest if i % 2 == 0 else r_root
        pts.append((r * math.cos(ang), r * math.sin(ang)))

    # Build star profile at the skirt bottom and extrude upward.
    wp = cq.Workplane("XY").workplane(offset=-CAP_SKIRT_H)
    wp = wp.moveTo(pts[0][0], pts[0][1])
    for pt in pts[1:]:
        wp = wp.lineTo(pt[0], pt[1])
    outer_shell = wp.close().extrude(CAP_TOTAL_H)

    # --- Inner bore: hollow cylinder open at the bottom, closed by the top wall ---
    bore_depth = CAP_TOTAL_H - CAP_TOP_WALL
    bore = (
        cq.Workplane("XY")
        .workplane(offset=-CAP_SKIRT_H - 0.001)
        .circle(CAP_BORE_R)
        .extrude(bore_depth + 0.001)
    )
    cap = outer_shell.cut(bore)

    # --- Internal thread rings: protrude inward from the bore wall ---
    # Cap threads sit BELOW neck threads so their top faces contact the
    # neck thread bottom faces (interleaved thread engagement).
    thread_outer_r = CAP_BORE_R + 0.0005  # overlap with cap wall for connectivity

    for i in range(N_THREADS):
        z = -CAP_SKIRT_H + i * THREAD_SPACING
        ring = _thread_ring(z, CAP_THREAD_INNER_R, thread_outer_r)
        cap = cap.union(ring)

    return mesh_from_cadquery(cap, "screw_cap")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="beer_bottle")

    # Dark/brown glass: translucent (alpha < 1) so it reads as glass.
    glass = model.material("dark_glass", rgba=GLASS_RGBA)
    cap_metal = model.material("cap_metal", rgba=CAP_RGBA)

    # ---- bottle (root): hollow dark-glass shell ----
    bottle = model.part("bottle")
    bottle.visual(
        mesh_from_cadquery(_bottle_solid(), "bottle_glass"),
        material=glass,
        name="bottle_glass",
    )
    # External thread rings on neck (inline decoration on bottle part).
    bottle.visual(
        _neck_threads_mesh(),
        material=glass,
        name="neck_threads",
    )
    # Inertial from an equivalent solid cylinder spanning the body.
    bottle.inertial = Inertial.from_geometry(
        Cylinder(radius=BODY_R, length=LIP_TOP_Z),
        mass=0.30,
        origin=Origin(xyz=(0.0, 0.0, LIP_TOP_Z / 2.0)),
    )

    # ---- screw cap: knurled cylindrical metal, rotates on bottle axis ----
    cap = model.part("screw_cap")
    cap.visual(
        _screw_cap_mesh(),
        material=cap_metal,
        name="screw_cap",
    )
    cap.inertial = Inertial.from_geometry(
        Cylinder(radius=CAP_OUTER_R, length=CAP_TOTAL_H),
        mass=0.008,
        origin=Origin(xyz=(0.0, 0.0, -CAP_SKIRT_H + CAP_TOTAL_H / 2.0)),
    )

    # Joint origin at the lip top contact surface (where the cap seats).
    # Cap part frame coincides with articulation frame at rest.
    # Cap geometry extends both below (skirt) and above (top wall) the joint origin.
    # CONTINUOUS about +Z: positive rotation spins the cap to seat/unseat.
    model.articulation(
        "bottle_to_cap",
        ArticulationType.CONTINUOUS,
        parent=bottle,
        child=cap,
        origin=Origin(xyz=(0.0, 0.0, LIP_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=6.0),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bottle = object_model.get_part("bottle")
    cap = object_model.get_part("screw_cap")
    cap_joint = object_model.get_articulation("bottle_to_cap")

    # --- Dark glass: translucent body material (alpha < 1). ---
    ctx.check(
        "bottle glass is translucent (alpha < 1)",
        GLASS_RGBA[3] < 1.0,
        details=f"glass rgba={GLASS_RGBA}",
    )

    # --- Long neck: the bottle is tall, and the upper neck region is narrow. ---
    body_ext = _ext(ctx.part_world_aabb(bottle))
    ctx.check(
        "bottle is tall (long-neck silhouette)",
        body_ext[2] > 0.20 and body_ext[2] > 2.5 * max(body_ext[0], body_ext[1]),
        details=f"bottle extents={body_ext}",
    )

    # --- Cap joint is CONTINUOUS rotation about bottle axis (+Z). ---
    ctx.check(
        "cap joint is continuous rotation",
        cap_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joint type={cap_joint.articulation_type}",
    )
    ctx.check(
        "cap joint axis is bottle axis (+Z)",
        tuple(cap_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={cap_joint.axis}",
    )

    # --- Cap mounted at the bottle mouth (top). ---
    cap_pos = ctx.part_world_position(cap)
    ctx.check(
        "cap mounted at the bottle mouth (top)",
        cap_pos is not None and cap_pos[2] > 0.20,
        details=f"cap origin={cap_pos}",
    )

    # --- Thread engagement: cap threads contact neck threads (intentional). ---
    # The interleaved thread rings create face-to-face contact at the thread
    # interface, which is the mechanical connection for the screw cap.
    ctx.allow_overlap(
        bottle,
        cap,
        elem_a="bottle_glass",
        elem_b="screw_cap",
        reason="Cap internal thread rings engage with bottle neck external thread rings; "
               "the interleaved thread faces are in intentional contact at the engagement interface.",
    )

    # --- Cap overlaps the lip region in Z when seated. ---
    ctx.expect_overlap(
        cap, bottle, axes="z", min_overlap=0.002,
        name="cap seated over the bottle lip region",
    )

    # --- Cap XY footprint is centered on the bottle mouth. ---
    ctx.expect_within(
        cap, bottle, axes="xy", margin=0.005,
        name="cap centered over bottle mouth",
    )

    # --- Cap rotation does not translate (spins in place on bottle axis). ---
    rest_pos = ctx.part_world_position(cap)
    with ctx.pose({cap_joint: math.pi}):
        rotated_pos = ctx.part_world_position(cap)
        ctx.check(
            "cap spins in place (Z unchanged after half turn)",
            rest_pos is not None
            and rotated_pos is not None
            and abs(rotated_pos[2] - rest_pos[2]) < 0.001,
            details=f"rest_z={rest_pos}, rotated_z={rotated_pos}",
        )
        ctx.check(
            "cap XY unchanged after rotation",
            rest_pos is not None
            and rotated_pos is not None
            and abs(rotated_pos[0] - rest_pos[0]) < 0.001
            and abs(rotated_pos[1] - rest_pos[1]) < 0.001,
            details=f"rest={rest_pos}, rotated={rotated_pos}",
        )

    # --- Screw cap has thread engagement visual. ---
    cap_visuals = cap.visuals
    ctx.check(
        "screw cap has at least one visual",
        len(cap_visuals) >= 1,
        details=f"cap visuals={len(cap_visuals)}",
    )

    # --- Bottle neck has thread rings visual. ---
    bottle_visuals = bottle.visuals
    thread_vis = None
    for v in bottle_visuals:
        if v.name == "neck_threads":
            thread_vis = v
            break
    ctx.check(
        "bottle has neck_threads visual",
        thread_vis is not None,
        details=f"bottle visuals={[v.name for v in bottle_visuals]}",
    )

    # --- Thread engagement: cap and neck threads contact at engagement faces. ---
    ctx.expect_contact(
        cap, bottle,
        elem_a="screw_cap",
        elem_b="neck_threads",
        contact_tol=0.0005,
        name="cap threads contact neck threads (thread engagement)",
    )

    return ctx.report()


object_model = build_object_model()
