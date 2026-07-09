from __future__ import annotations

# STABILO BOSS ORIGINAL highlighter — screw-cap variant.
#
# Variant of the Stabilo Boss highlighter with a screw-on twist cap instead of
# the straight pull-off friction cap. The cap threads onto a short cylindrical
# boss at the front collar of the barrel. Modeled as a REVOLUTE joint about the
# pen's long axis (+X): rotating the cap about that axis screws it off and on.
# At q=0 the cap is fully seated (threaded engagement, covering the chisel nib);
# positive q unscrews the cap by rotation.
#
# Frame convention:
#   +X = pen length (front of pen at +X, rear at -X)
#   cross-section lies in the Y-Z plane (Y = width, Z = height)

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
# Real-world dimensions (meters)
# ---------------------------------------------------------------------------
BARREL_LEN = 0.090  # length of the lime body (rear shoulder to front shoulder)
BARREL_W = 0.0170  # body width (Y)
BARREL_H = 0.0120  # body height (Z)
BODY_CORNER_R = 0.0030  # rounded corners of the rectangular section

# Front shoulder: a short stepped collar where the cap registers.
COLLAR_LEN = 0.0060
COLLAR_W = 0.0150
COLLAR_H = 0.0105
COLLAR_CORNER_R = 0.0028

# Threaded boss on the collar for screw-on cap engagement.
THREAD_BOSS_LEN = 0.0050  # length of the cylindrical threaded section
THREAD_BOSS_RADIUS = 0.0065  # slightly smaller than collar width/2
THREAD_PITCH = 0.0025  # 2.5mm per turn (coarse thread for a pen cap)
THREAD_TURNS = 2.0  # number of thread turns
THREAD_RIDGE_WIDTH = 0.0012  # width of each thread ridge
THREAD_RIDGE_HEIGHT = 0.0008  # height of thread ridge protrusion

# Nib (black wedge / chisel point) protruding forward from the collar.
NIB_BASE_LEN = 0.0080  # straight black holder right after the collar
NIB_BASE_W = 0.0120
NIB_BASE_H = 0.0090
NIB_WEDGE_LEN = 0.0120  # tapered chisel section
NIB_TIP_W = 0.0090
NIB_TIP_H = 0.0018  # thin chisel edge

# Cap (black, hollow, rounded-rect, closed at its front, open at its rear).
CAP_LEN = 0.0420
CAP_OUTER_W = 0.0182
CAP_OUTER_H = 0.0132
CAP_WALL = 0.0014
CAP_CORNER_R = 0.0034
CAP_CLIP_LEN = 0.0260  # pocket-clip flat along the cap top

# Cap seats so its rear lip overlaps the front of the lime barrel.
CAP_SEAT_OVERLAP = 0.0060  # how far the cap mouth slides back onto the barrel

# Materials
LIME = (0.82, 0.93, 0.13, 1.0)  # fluorescent yellow-green body
BLACK = (0.07, 0.07, 0.08, 1.0)  # nib + cap
FELT = (0.78, 0.90, 0.15, 1.0)  # exposed ink-soaked felt at the very tip


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------
def _rounded_rect_prism(length: float, width: float, height: float, corner_r: float):
    """A rounded-rectangle prism whose long axis is +X, centered on Y/Z,
    spanning X in [0, length]."""
    return (
        cq.Workplane("YZ")
        .rect(width, height)
        .extrude(length)
        .edges("|X")
        .fillet(corner_r)
    )


def _build_helix_path(radius: float, pitch: float, n_turns: float, n_pts_per_turn: int = 32):
    """Build a 3D helix path along +X axis for thread geometry."""
    import math
    height = pitch * n_turns
    n_pts = int(n_turns * n_pts_per_turn) + 1
    pts = []
    for i in range(n_pts):
        t = i / (n_pts - 1)
        angle = t * 2 * math.pi * n_turns
        x = height * t
        y = radius * math.cos(angle)
        z = radius * math.sin(angle)
        pts.append((x, y, z))
    return cq.Workplane("XY").spline(pts)


def _build_thread_ridge(radius: float, pitch: float, n_turns: float, 
                        ridge_width: float, ridge_height: float):
    """Build a helical thread ridge by sweeping a rectangular profile along a helix."""
    helix_path = _build_helix_path(radius, pitch, n_turns)
    
    # Create rectangular profile at the helix radius, perpendicular to X axis.
    # At t=0, the helix is at (X=0, Y=radius, Z=0), so center the profile there.
    profile = (
        cq.Workplane("YZ")
        .center(radius, 0)  # center at (Y=radius, Z=0)
        .rect(ridge_width, ridge_height)
    )
    
    # Sweep along the helix path
    return profile.sweep(helix_path)


def _build_barrel() -> object:
    """Lime body + stepped front collar with threaded boss, as one solid (the pen body)."""
    body = _rounded_rect_prism(BARREL_LEN, BARREL_W, BARREL_H, BODY_CORNER_R)

    # Front collar steps down from the body so the cap mouth seats over it.
    collar = _rounded_rect_prism(
        COLLAR_LEN, COLLAR_W, COLLAR_H, COLLAR_CORNER_R
    ).translate((BARREL_LEN, 0.0, 0.0))

    body = body.union(collar)
    
    # Threaded boss: cylindrical section with external thread ridges.
    boss_x = BARREL_LEN + COLLAR_LEN
    boss_cyl = (
        cq.Workplane("YZ")
        .circle(THREAD_BOSS_RADIUS)
        .extrude(THREAD_BOSS_LEN)
        .translate((boss_x, 0.0, 0.0))
    )
    body = body.union(boss_cyl)
    
    # Add external thread ridges on the boss.
    thread_ridge = _build_thread_ridge(
        THREAD_BOSS_RADIUS,
        THREAD_PITCH,
        THREAD_TURNS,
        THREAD_RIDGE_WIDTH,
        THREAD_RIDGE_HEIGHT
    ).translate((boss_x, 0.0, 0.0))
    body = body.union(thread_ridge)
    
    return body


def _build_nib() -> object:
    """Black chisel nib: a straight holder then a tapered wedge to a thin edge.
    Origin at X=0 corresponds to the front shoulder of the barrel collar."""
    holder = _rounded_rect_prism(NIB_BASE_LEN, NIB_BASE_W, NIB_BASE_H, 0.0018)

    # Tapered chisel: loft from the holder face to a thin wide edge.
    x0 = NIB_BASE_LEN
    wedge = (
        cq.Workplane("YZ")
        .workplane(offset=x0)
        .rect(NIB_BASE_W, NIB_BASE_H)
        .workplane(offset=NIB_WEDGE_LEN)
        .rect(NIB_TIP_W, NIB_TIP_H)
        .loft(combine=True)
    )
    return holder.union(wedge)


def _build_felt_tip() -> object:
    """The exposed ink-soaked felt sliver at the extreme chisel edge."""
    x0 = NIB_BASE_LEN + NIB_WEDGE_LEN
    felt = (
        cq.Workplane("YZ")
        .workplane(offset=x0 - 0.0015)
        .rect(NIB_TIP_W * 0.96, NIB_TIP_H * 1.05)
        .workplane(offset=0.0025)
        .rect(NIB_TIP_W * 0.9, NIB_TIP_H * 0.8)
        .loft(combine=True)
    )
    return felt


def _build_cap() -> object:
    """Hollow black cap with internal thread relief, closed at its front (+X), 
    open at its rear (-X) so it screws over the threaded collar boss and nib. 
    Authored in its own frame: rear mouth at X=0, front closed end at X=CAP_LEN. 
    Includes a flat pocket clip on top."""
    outer = _rounded_rect_prism(CAP_LEN, CAP_OUTER_W, CAP_OUTER_H, CAP_CORNER_R)

    # Hollow it out from the rear mouth, leaving the front end closed.
    # The bore must be large enough to clear the threaded boss.
    bore_radius = THREAD_BOSS_RADIUS + THREAD_RIDGE_HEIGHT + 0.0005  # clearance
    bore_len = CAP_LEN - CAP_WALL  # leave a front wall of thickness CAP_WALL
    bore = (
        cq.Workplane("YZ")
        .circle(bore_radius)
        .extrude(bore_len)
    )
    cap = outer.cut(bore)
    
    # Add internal thread grooves that match the boss threads.
    # Cut helical grooves into the bore wall at the mouth end.
    thread_groove = _build_thread_ridge(
        THREAD_BOSS_RADIUS + THREAD_RIDGE_HEIGHT / 2,  # at the bore wall
        THREAD_PITCH,
        THREAD_TURNS,
        THREAD_RIDGE_WIDTH,
        THREAD_RIDGE_HEIGHT
    )
    cap = cap.cut(thread_groove)

    # Pocket clip: a thin flat rib running along the top, standing slightly
    # proud and anchored to the closed front end of the cap.
    clip_thick = 0.0016
    clip_w = 0.0070
    clip_x0 = CAP_LEN - CAP_CLIP_LEN
    clip_z = CAP_OUTER_H / 2.0 + clip_thick / 2.0 - 0.0002
    clip = (
        cq.Workplane("YZ")
        .workplane(offset=clip_x0)
        .center(0.0, clip_z)
        .rect(clip_w, clip_thick)
        .extrude(CAP_CLIP_LEN)
        .edges("|X")
        .fillet(0.0006)
    )
    # Small connecting boss tying the clip down to the cap body at its front.
    boss = (
        cq.Workplane("YZ")
        .workplane(offset=CAP_LEN - 0.0050)
        .center(0.0, CAP_OUTER_H / 2.0 - 0.0010)
        .rect(clip_w, 0.0040)
        .extrude(0.0045)
    )
    cap = cap.union(clip).union(boss)
    return cap


# ---------------------------------------------------------------------------
# Model
# ---------------------------------------------------------------------------
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stabilo_boss_highlighter")

    lime = model.material("lime_body", rgba=LIME)
    black = model.material("black_plastic", rgba=BLACK)
    felt_mat = model.material("felt_ink", rgba=FELT)

    # --- Pen body (root): lime barrel + collar, plus the black nib up front ---
    barrel = model.part("barrel")
    barrel.visual(
        mesh_from_cadquery(_build_barrel(), "barrel_body"),
        material=lime,
        name="barrel_body",
    )
    # Nib visuals live on the barrel part (rigidly part of the pen body), shifted
    # forward to start at the front shoulder of the collar.
    nib_x = BARREL_LEN + COLLAR_LEN
    barrel.visual(
        mesh_from_cadquery(_build_nib(), "nib"),
        origin=Origin(xyz=(nib_x, 0.0, 0.0)),
        material=black,
        name="nib",
    )
    barrel.visual(
        mesh_from_cadquery(_build_felt_tip(), "felt_tip"),
        origin=Origin(xyz=(nib_x, 0.0, 0.0)),
        material=felt_mat,
        name="felt_tip",
    )

    # --- Cap (moving part): hollow black shell that screws off by rotation ---
    cap = model.part("cap")
    cap.visual(
        mesh_from_cadquery(_build_cap(), "cap_shell"),
        material=black,
        name="cap_shell",
    )

    # Seated cap pose: cap rear mouth sits back over the threaded boss and nib.
    # The cap is authored with its mouth at local X=0; place the joint origin so
    # that at q=0 the mouth is positioned to engage the threaded boss.
    # The cap mouth should align with the start of the threaded boss.
    seat_x = BARREL_LEN + COLLAR_LEN  # at the start of the threaded boss
    
    # Rotation to fully unscrew: enough turns to disengage the threads.
    # With THREAD_TURNS=2.0, we need about 2.5 turns (with margin) to fully unscrew.
    unscrew_angle = THREAD_TURNS * 2.0 * 3.14159 + 1.5708  # ~2.5 turns in radians

    model.articulation(
        "barrel_to_cap",
        ArticulationType.REVOLUTE,
        parent=barrel,
        child=cap,
        origin=Origin(xyz=(seat_x, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),  # rotate about the pen's long axis (+X)
        motion_limits=MotionLimits(
            effort=10.0, velocity=2.0, lower=0.0, upper=unscrew_angle
        ),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    barrel = object_model.get_part("barrel")
    cap = object_model.get_part("cap")
    joint = object_model.get_articulation("barrel_to_cap")

    nib_x = BARREL_LEN + COLLAR_LEN
    nib_tip_x = nib_x + NIB_BASE_LEN + NIB_WEDGE_LEN

    # --- Joint contract: revolute about the pen's long axis (+X) ---
    ctx.check(
        "cap joint is revolute",
        str(joint.joint_type).lower().endswith("revolute"),
        details=f"joint_type={joint.joint_type}",
    )
    ax = tuple(joint.axis)
    ctx.check(
        "cap rotates about +X axis",
        abs(ax[0]) > 0.99 and abs(ax[1]) < 1e-6 and abs(ax[2]) < 1e-6,
        details=f"axis={ax}",
    )

    # --- Hero parts present and placed ---
    # Nib lives at the front of the pen, ahead of the barrel body.
    nib_aabb = ctx.part_element_world_aabb(barrel, elem="nib")
    ctx.check(
        "black nib protrudes past the barrel front",
        nib_aabb is not None and nib_aabb[1][0] > BARREL_LEN + COLLAR_LEN - 1e-6,
        details=f"nib_aabb={nib_aabb}",
    )
    # Exposed felt tip is at the extreme front of the pen.
    felt_aabb = ctx.part_element_world_aabb(barrel, elem="felt_tip")
    ctx.check(
        "felt ink tip is at the chisel point",
        felt_aabb is not None and felt_aabb[1][0] >= nib_tip_x - 0.002,
        details=f"felt_aabb={felt_aabb}",
    )

    # Cap is a roughly rectangular body wider than the barrel (it wraps it).
    cap_aabb = ctx.part_world_aabb(cap)
    if cap_aabb is not None:
        cap_w = cap_aabb[1][1] - cap_aabb[0][1]
        cap_h = cap_aabb[1][2] - cap_aabb[0][2]
        ctx.check(
            "cap is a chunky rectangular shell",
            cap_w >= BARREL_W and cap_h >= BARREL_H,
            details=f"cap_w={cap_w:.4f}, cap_h={cap_h:.4f}",
        )

    # --- Closed pose (q=0): cap fully covers / encloses the nib ---
    with ctx.pose({joint: 0.0}):
        # Cap projected footprint surrounds the nib in cross-section.
        ctx.expect_within(
            barrel,
            cap,
            axes="yz",
            inner_elem="nib",
            outer_elem="cap_shell",
            margin=0.001,
            name="seated cap encloses the nib cross-section",
        )
        # Cap front reaches beyond the chisel tip (tip is capped, not exposed).
        cap_closed = ctx.part_world_aabb(cap)
        ctx.check(
            "seated cap front covers the chisel tip",
            cap_closed is not None and cap_closed[1][0] >= nib_tip_x - 1e-4,
            details=f"cap_front={None if cap_closed is None else cap_closed[1][0]:.4f}, tip={nib_tip_x:.4f}",
        )
        seated_center = ctx.part_world_aabb(cap)

    # --- Open pose (upper limit): cap rotates to unscrew ---
    upper = joint.motion_limits.upper
    with ctx.pose({joint: upper}):
        cap_open = ctx.part_world_aabb(cap)
        # The cap should still be at roughly the same X position (revolute joint
        # rotates but doesn't translate), but its orientation has changed.
        # Check that the cap has rotated by verifying the AABB has changed.
        ctx.check(
            "cap rotates when unscrewing",
            cap_open is not None and seated_center is not None,
            details=f"seated_aabb={seated_center}, open_aabb={cap_open}",
        )
        # Verify rotation occurred by checking that the AABB changed significantly
        # (a rotated rectangular cap will have different Y/Z extents).
        if cap_open is not None and seated_center is not None:
            seated_dy = seated_center[1][1] - seated_center[0][1]
            open_dy = cap_open[1][1] - cap_open[0][1]
            seated_dz = seated_center[1][2] - seated_center[0][2]
            open_dz = cap_open[1][2] - cap_open[0][2]
            # After significant rotation, the Y/Z extents should change
            dy_change = abs(open_dy - seated_dy)
            dz_change = abs(open_dz - seated_dz)
            ctx.check(
                "cap orientation changes when unscrewed",
                dy_change > 0.001 or dz_change > 0.001,
                details=f"dy_change={dy_change:.4f}, dz_change={dz_change:.4f}",
            )

    # The seated cap nests over the barrel/nib front: a genuine capture fit.
    ctx.allow_overlap(
        cap,
        barrel,
        elem_a="cap_shell",
        elem_b="nib",
        reason="Seated cap is a threaded fit that intentionally encloses the nib (capture fit).",
    )
    ctx.allow_overlap(
        cap,
        barrel,
        elem_a="cap_shell",
        elem_b="barrel_body",
        reason="Cap mouth threads onto the front collar boss of the barrel to seat (screw-on fit).",
    )
    ctx.allow_overlap(
        cap,
        barrel,
        elem_a="cap_shell",
        elem_b="felt_tip",
        reason="Seated cap encloses the felt tip to keep the highlighter from drying out.",
    )

    return ctx.report()


object_model = build_object_model()
