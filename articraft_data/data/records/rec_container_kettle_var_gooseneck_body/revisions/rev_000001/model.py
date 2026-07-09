from __future__ import annotations

# Pour kettle body variant — a low, wide pear-shaped vessel with a compact,
# natural pour spout. This deliberately moves away from the parent bell kettle
# and away from the previous tall body with an odd high S-shaped tube.
# Frame: Z is up, base at z=0.
# Articulations (three INDEPENDENT joints):
#   - lid (with knob): PRISMATIC, lifts straight up off the kettle top (+Z).
#   - whistle cap: REVOLUTE, flap at the pour-spout tip flips up about its hinge.
#   - bail handle: REVOLUTE, arching handle swings down to the side.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    SphereGeometry,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)

# --- key dimensions (low pear pour kettle) ---
BODY_BASE_R = 0.083       # wide shoulder / belly radius
BODY_TOP_R = 0.038        # smaller top neck where the rim sits
RIM_Z = 0.148             # z of the kettle top rim (lid seat)
NECK_Z = 0.132            # z where the body tapers to the neck

LID_SEAT_Z = RIM_Z
LID_LIFT = 0.040          # prismatic lift travel

# Compact pour spout: exits the upper shoulder, rises gently, then settles into
# a small downturned mouth. It stays close to the body instead of making a tall
# looping pipe.
SPOUT_POINTS = [
    (0.080, 0.0, 0.088),   # root: exits the broad upper shoulder
    (0.107, 0.0, 0.108),   # short smooth rise
    (0.137, 0.0, 0.123),   # main forward reach
    (0.166, 0.0, 0.124),   # nearly level pour section
    (0.184, 0.0, 0.114),   # modest downturned mouth
]
SPOUT_RADIUS = 0.011      # outer radius of the pour tube
SPOUT_BORE_R = 0.007      # bore radius through the body wall

# Handle mounts: two lugs on the upper body, offset in +/-Y.
MOUNT_X = 0.0
MOUNT_Y = 0.074
MOUNT_Z = 0.112


def _loft_z(sections) -> cq.Workplane:
    """Loft circles stacked along +Z (XY planes)."""
    wp = cq.Workplane("XY")
    prev = 0.0
    for i, (z, r) in enumerate(sections):
        off = z if i == 0 else z - prev
        if i > 0 or abs(off) > 1e-12:
            wp = wp.workplane(offset=off)
        wp = wp.circle(r)
        prev = z
    return wp.loft(ruled=False)


def _body_solid() -> cq.Workplane:
    """Low, wide hollow shell with pear-shaped belly and narrow neck."""
    outer = _loft_z(
        [
            (0.000, 0.054),   # flat foot
            (0.010, 0.074),   # rounded lower swell
            (0.030, 0.086),   # wide pot-belly
            (0.064, 0.084),   # broad shoulder
            (0.100, 0.066),   # strong taper inward
            (0.124, 0.046),   # short neck shoulder
            (NECK_Z, BODY_TOP_R),
            (RIM_Z, BODY_TOP_R + 0.004),  # slight rim flare
        ]
    )
    inner = _loft_z(
        [
            (0.008, 0.047),
            (0.028, 0.077),
            (0.064, 0.075),
            (0.100, 0.058),
            (0.124, 0.039),
            (NECK_Z, BODY_TOP_R - 0.006),
            (RIM_Z + 0.018, BODY_TOP_R - 0.003),
        ]
    )
    # Cut the spout bore through the body wall so the interior connects to
    # the pour-spout passage.
    return outer.cut(inner).cut(_spout_bore_cutter())


def _spout_initial_direction():
    """Unit direction of the spout at the root (XZ plane)."""
    p0, p1 = SPOUT_POINTS[0], SPOUT_POINTS[1]
    dx = p1[0] - p0[0]
    dz = p1[2] - p0[2]
    length = math.hypot(dx, dz)
    return (dx / length, 0.0, dz / length)


def _spout_bore_cutter() -> cq.Workplane:
    """Short cylinder that punches through the body wall at the spout root."""
    ax = _spout_initial_direction()
    p0 = SPOUT_POINTS[0]
    # Start inside the body cavity, extend outward past the shell.
    inset = 0.012
    start = (
        p0[0] - ax[0] * inset,
        0.0,
        p0[2] - ax[2] * inset,
    )
    plane = cq.Plane(origin=start, normal=ax)
    return (
        cq.Workplane(plane)
        .circle(SPOUT_BORE_R)
        .extrude(0.035)
    )


def _pour_spout_mesh():
    """Short smooth pour spout tube via spline-through-points."""
    tube = tube_from_spline_points(
        SPOUT_POINTS,
        radius=SPOUT_RADIUS,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
        up_hint=(0.0, 0.0, 1.0),
    )
    return mesh_from_geometry(tube, "pour_spout")


def _spout_root_fairing():
    """Short collar/fairing where the pour spout meets the body wall."""
    ax = _spout_initial_direction()
    p0 = SPOUT_POINTS[0]
    # Build a short tapered collar along the initial spout direction.
    inset = 0.012  # starts inside the body wall
    outset = 0.014  # extends just past the shell
    start = (
        p0[0] - ax[0] * inset,
        0.0,
        p0[2] - ax[2] * inset,
    )
    plane = cq.Plane(origin=start, normal=ax)
    fairing = (
        cq.Workplane(plane)
        .circle(SPOUT_RADIUS * 2.5)   # broad saddle at body wall
        .workplane(offset=inset + outset)
        .circle(SPOUT_RADIUS * 1.35)  # tapers onto the tube
        .loft(ruled=False)
    )
    return mesh_from_cadquery(fairing, "spout_root_fairing")


def _lid_mesh():
    """Round domed lid that seats on the rim."""
    lid = _loft_z(
        [
            (0.0, BODY_TOP_R + 0.006),
            (0.006, BODY_TOP_R + 0.002),
            (0.016, BODY_TOP_R - 0.012),
            (0.024, BODY_TOP_R - 0.030),
        ]
    )
    return mesh_from_cadquery(lid, "lid_dome")


def _bail_mesh():
    """Arching bail handle from one mount lug over the top to the other."""
    pts = [
        (MOUNT_X, MOUNT_Y, 0.0),
        (MOUNT_X + 0.006, MOUNT_Y * 0.92, 0.034),
        (MOUNT_X + 0.008, MOUNT_Y * 0.55, 0.082),
        (MOUNT_X + 0.008, 0.0, 0.112),
        (MOUNT_X + 0.008, -MOUNT_Y * 0.55, 0.082),
        (MOUNT_X + 0.006, -MOUNT_Y * 0.92, 0.034),
        (MOUNT_X, -MOUNT_Y, 0.0),
    ]
    bail = tube_from_spline_points(
        pts, radius=0.006, samples_per_segment=16, radial_segments=14
    )
    return mesh_from_geometry(bail, "bail_arch")


def _tip_direction():
    """Unit direction at the pour-spout tip (from last two spline points)."""
    p_prev = SPOUT_POINTS[-2]
    p_tip = SPOUT_POINTS[-1]
    dx = p_tip[0] - p_prev[0]
    dz = p_tip[2] - p_prev[2]
    length = math.hypot(dx, dz)
    return (dx / length, 0.0, dz / length)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_wide_pour_kettle")

    steel = model.material("brushed_steel", rgba=(0.78, 0.79, 0.81, 1.0))
    steel_dark = model.material("steel_shadow", rgba=(0.62, 0.63, 0.66, 1.0))
    copper = model.material("copper_accent", rgba=(0.72, 0.45, 0.20, 1.0))
    black = model.material("black_grip", rgba=(0.12, 0.12, 0.13, 1.0))

    # ---- body (root): low pear shell + compact pour spout + mount lugs ----
    body = model.part("body")

    body_shell = _body_solid()
    body.visual(
        mesh_from_cadquery(body_shell, "body_shell"), material=steel, name="body_shell"
    )

    # Rim ring around the open top so the lid has a visible seat.
    rim = TorusGeometry(BODY_TOP_R + 0.002, 0.005, radial_segments=12, tubular_segments=40)
    rim.translate(0.0, 0.0, RIM_Z)
    body.visual(mesh_from_geometry(rim, "top_rim"), material=steel_dark, name="top_rim")

    # Compact pour spout: short, smooth, and visually attached to the shoulder.
    body.visual(
        _pour_spout_mesh(), material=steel, name="pour_spout"
    )
    # Root fairing collar where spout meets body.
    body.visual(
        _spout_root_fairing(), material=steel_dark, name="spout_root_fairing"
    )

    # Two handle mount lugs on the upper body that carry the bail.
    for i, sgn in enumerate((1.0, -1.0)):
        nm = f"mount_lug_{i}"
        lug = CylinderGeometry(0.008, 0.042, radial_segments=18).rotate_x(math.pi / 2.0)
        lug.translate(MOUNT_X, sgn * 0.064, MOUNT_Z)
        body.visual(mesh_from_geometry(lug, nm), material=steel_dark, name=nm)

    body.inertial = Inertial.from_geometry(
        Cylinder(radius=BODY_BASE_R, length=RIM_Z),
        mass=0.75,
        origin=Origin(xyz=(0.0, 0.0, RIM_Z / 2.0)),
    )

    # ---- lid: domed round lid with a black knob; lifts straight up ----
    lid = model.part("lid")
    lid.visual(_lid_mesh(), material=steel, origin=Origin(xyz=(0.0, 0.0, 0.0)), name="lid_dome")
    # Knob stem + ball on top center.
    stem = CylinderGeometry(0.005, 0.012, radial_segments=16)
    stem.translate(0.0, 0.0, 0.028)
    lid.visual(mesh_from_geometry(stem, "knob_stem"), material=black, name="knob_stem")
    knob = SphereGeometry(0.010, width_segments=20, height_segments=14)
    knob.translate(0.0, 0.0, 0.038)
    lid.visual(mesh_from_geometry(knob, "knob_ball"), material=black, name="knob_ball")
    lid.inertial = Inertial.from_geometry(
        Cylinder(radius=BODY_TOP_R, length=0.024),
        mass=0.10,
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
    )
    model.articulation(
        "body_to_lid",
        ArticulationType.PRISMATIC,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, LID_SEAT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.2, lower=0.0, upper=LID_LIFT),
    )

    # ---- whistle cap: flap at the pour-spout tip, flips up about hinge ----
    cap = model.part("whistle_cap")
    spout_dir = _tip_direction()
    SPOUT_TIP = SPOUT_POINTS[-1]
    mouth_r = SPOUT_RADIUS + 0.002
    # Perpendicular to the tip direction in XZ that points upward (+Z dominant).
    # For direction (dx, 0, dz), the two perpendiculars are (-dz, 0, dx) and
    # (dz, 0, -dx). Pick the one with positive Z component.
    px, pz = -spout_dir[2], spout_dir[0]  # candidate 1
    if pz < 0:
        px, pz = -px, -pz  # flip to candidate 2
    hinge = (
        SPOUT_TIP[0] + mouth_r * px,
        0.0,
        SPOUT_TIP[2] + mouth_r * pz,
    )
    # Flap centre seats on the mouth centre; expressed relative to hinge.
    off = (SPOUT_TIP[0] - hinge[0], 0.0, SPOUT_TIP[2] - hinge[2])
    # Orient disk normal along the spout tip direction.
    spout_ang = math.atan2(spout_dir[2], spout_dir[0])
    flap = CylinderGeometry(mouth_r, 0.004, radial_segments=22)
    flap.rotate_y(math.pi / 2.0 - spout_ang)
    flap.translate(off[0], 0.0, off[2])
    cap.visual(mesh_from_geometry(flap, "cap_flap"), material=copper, name="cap_flap")
    # Small lift tab on the outer face of the flap.
    tab = BoxGeometry((0.005, 0.010, 0.008))
    tab.rotate_y(math.pi / 2.0 - spout_ang)
    tab.translate(off[0] + spout_dir[0] * 0.005, 0.0, off[2] + spout_dir[2] * 0.005)
    cap.visual(mesh_from_geometry(tab, "cap_tab"), material=black, name="cap_tab")
    cap.inertial = Inertial.from_geometry(
        Cylinder(radius=mouth_r, length=0.006), mass=0.008
    )
    model.articulation(
        "spout_to_cap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cap,
        origin=Origin(xyz=hinge),
        # Positive travel flips the cap outward, away from the kettle body.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=0.0, upper=1.4),
    )

    # ---- bail handle: arching tube with black grip sleeve; swings to side ----
    handle = model.part("bail_handle")
    handle.visual(_bail_mesh(), material=steel, name="bail_arch")
    # Black grip sleeve over the top span of the arch.
    grip_pts = [
        (MOUNT_X + 0.008, 0.034, 0.101),
        (MOUNT_X + 0.008, 0.0, 0.112),
        (MOUNT_X + 0.008, -0.034, 0.101),
    ]
    grip = tube_from_spline_points(
        grip_pts, radius=0.010, samples_per_segment=14, radial_segments=16
    )
    handle.visual(mesh_from_geometry(grip, "grip_sleeve"), material=black, name="grip_sleeve")
    handle.inertial = Inertial.from_geometry(
        Box((0.02, 2 * MOUNT_Y, 0.12)),
        mass=0.06,
        origin=Origin(xyz=(MOUNT_X, 0.0, 0.060)),
    )
    model.articulation(
        "body_to_handle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=(MOUNT_X, 0.0, MOUNT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=3.0, lower=0.0, upper=1.6),
    )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    cap = object_model.get_part("whistle_cap")
    handle = object_model.get_part("bail_handle")

    lid_joint = object_model.get_articulation("body_to_lid")
    cap_joint = object_model.get_articulation("spout_to_cap")
    handle_joint = object_model.get_articulation("body_to_handle")

    # --- Body is low and wide with a strong pear-shaped belly. ---
    shell_aabb = ctx.part_element_world_aabb(body, elem="body_shell")
    shell_ext = _ext(shell_aabb) if shell_aabb else (0, 0, 0)
    ctx.check(
        "body shell is low and broad (wide pear kettle profile)",
        shell_aabb is not None
        and 0.13 < shell_ext[2] < 0.17
        and shell_ext[0] > 0.16
        and shell_ext[1] > 0.16,
        details=f"body_shell extents={shell_ext}",
    )
    ctx.check(
        "body shell is wider than tall, making the body-form change obvious",
        shell_aabb is not None
        and shell_ext[0] > shell_ext[2] * 1.05
        and shell_ext[1] > shell_ext[2] * 1.05,
        details=f"body_shell xy extents=({shell_ext[0]:.3f}, {shell_ext[1]:.3f})",
    )

    # --- Pour spout is compact and natural, not a tall looping pipe. ---
    spout_aabb = ctx.part_element_world_aabb(body, elem="pour_spout")
    ctx.check(
        "pour spout stays below the rim line instead of looping high",
        spout_aabb is not None
        and spout_aabb[1][2] < RIM_Z + 0.005
        and spout_aabb[0][2] > 0.065,
        details=f"spout aabb={spout_aabb}",
    )
    ctx.check(
        "pour spout projects forward but remains short and controlled",
        spout_aabb is not None
        and spout_aabb[1][0] > 0.17
        and (spout_aabb[1][0] - spout_aabb[0][0]) < 0.14
        and (spout_aabb[1][2] - spout_aabb[0][2]) < 0.075,
        details=f"spout max_x={spout_aabb[1] if spout_aabb else None}",
    )
    ctx.check(
        "pour spout has a modest downturned mouth",
        spout_aabb is not None
        and SPOUT_POINTS[-1][2] < max(p[2] for p in SPOUT_POINTS),
        details=f"tip_z={SPOUT_POINTS[-1][2]}, peak_z={max(p[2] for p in SPOUT_POINTS)}",
    )

    # Root fairing wraps the spout-body junction.
    fairing_aabb = ctx.part_element_world_aabb(body, elem="spout_root_fairing")
    ctx.check(
        "spout root fairing wraps the body-spout junction",
        fairing_aabb is not None
        and fairing_aabb[0][0] < 0.070
        and fairing_aabb[1][0] > 0.095
        and fairing_aabb[0][2] < 0.095
        and fairing_aabb[1][2] > 0.095,
        details=f"fairing aabb={fairing_aabb}",
    )

    # Allow the fairing to embed slightly in the body shell (welded collar).
    ctx.allow_overlap(
        body, body,
        elem_a="spout_root_fairing", elem_b="body_shell",
        reason="Spout root fairing intentionally penetrates the body wall as a welded collar.",
    )
    ctx.expect_contact(
        body, body,
        elem_a="spout_root_fairing", elem_b="body_shell",
        name="spout root fairing seated into body",
    )
    # Gooseneck tube passes through the fairing collar.
    ctx.allow_overlap(
        body, body,
        elem_a="pour_spout", elem_b="spout_root_fairing",
        reason="Pour tube passes through its root fairing collar.",
    )
    ctx.expect_contact(
        body, body,
        elem_a="pour_spout", elem_b="spout_root_fairing",
        name="pour tube blends into root fairing",
    )

    # --- Lid seats on the rim and lifts straight up. ---
    ctx.allow_overlap(
        lid, body,
        elem_a="lid_dome", elem_b="top_rim",
        reason="Lid skirt intentionally seats over the kettle rim.",
    )
    ctx.allow_overlap(
        lid, body,
        elem_a="lid_dome", elem_b="body_shell",
        reason="Lid dome skirt overlaps the body shell top edge when seated on the rim.",
    )
    rest_lid = ctx.part_world_position(lid)
    ctx.check(
        "lid sits on top of the kettle",
        rest_lid is not None and rest_lid[2] > 0.14,
        details=f"lid pos={rest_lid}",
    )
    with ctx.pose({lid_joint: LID_LIFT}):
        lifted_lid = ctx.part_world_position(lid)
    ctx.check(
        "lid lifts straight up off the kettle top",
        lifted_lid is not None
        and lifted_lid[2] > rest_lid[2] + 0.03
        and abs(lifted_lid[0] - rest_lid[0]) < 1e-4
        and abs(lifted_lid[1] - rest_lid[1]) < 1e-4,
        details=f"rest={rest_lid}, lifted={lifted_lid}",
    )

    # --- Whistle cap at the pour-spout tip flips open. ---
    cap_rest = ctx.part_world_aabb(cap)
    ctx.check(
        "whistle cap sits at the compact pour-spout mouth",
        cap_rest is not None
        and cap_rest[1][0] > 0.17
        and 0.09 < cap_rest[0][2] < 0.13,
        details=f"cap aabb={cap_rest}",
    )
    rest_center_x = (cap_rest[0][0] + cap_rest[1][0]) / 2.0
    rest_min_z = cap_rest[0][2]
    with ctx.pose({cap_joint: 1.4}):
        cap_open = ctx.part_world_aabb(cap)
    open_center_x = (cap_open[0][0] + cap_open[1][0]) / 2.0
    open_min_z = cap_open[0][2]
    # Opening the cap swings the flap outward from the downturned compact mouth.
    ctx.check(
        "whistle cap opens outward away from the kettle body",
        cap_open is not None
        and open_center_x > rest_center_x + 0.010
        and open_min_z > rest_min_z,
        details=f"rest_center_x={rest_center_x:.4f}, open_center_x={open_center_x:.4f}, "
        f"rest_min_z={rest_min_z:.4f}, open_min_z={open_min_z:.4f}",
    )
    # Cap nests on the spout tip.
    ctx.allow_overlap(
        cap, body,
        elem_a="cap_flap", elem_b="pour_spout",
        reason="Whistle flap caps the pour-spout mouth when closed.",
    )
    ctx.expect_contact(cap, body, name="whistle cap seated on pour-spout tip")

    # --- Bail handle arches over the top and swings to the side. ---
    handle_rest = ctx.part_world_aabb(handle)
    ctx.check(
        "bail handle arches above the kettle at rest",
        handle_rest is not None and handle_rest[1][2] > 0.22,
        details=f"handle aabb={handle_rest}",
    )
    rest_top = handle_rest[1][2]
    rest_xspan = handle_rest[1][0] - handle_rest[0][0]
    with ctx.pose({handle_joint: 1.5}):
        handle_down = ctx.part_world_aabb(handle)
    down_top = handle_down[1][2]
    down_xspan = handle_down[1][0] - handle_down[0][0]
    ctx.check(
        "bail handle falls to the side",
        down_top < rest_top - 0.03 and down_xspan > rest_xspan + 0.03,
        details=f"rest_top={rest_top}, down_top={down_top}, "
        f"rest_xspan={rest_xspan}, down_xspan={down_xspan}",
    )
    # Handle mounts connect bail to body.
    ctx.allow_overlap(
        handle, body,
        elem_a="bail_arch", elem_b="mount_lug_0",
        reason="Bail leg ends seat into the mount lugs that carry the pivot.",
    )
    ctx.allow_overlap(
        handle, body,
        elem_a="bail_arch", elem_b="mount_lug_1",
        reason="Bail leg ends seat into the mount lugs that carry the pivot.",
    )
    ctx.expect_contact(handle, body, name="bail handle connected to body lugs")

    return ctx.report()


object_model = build_object_model()
