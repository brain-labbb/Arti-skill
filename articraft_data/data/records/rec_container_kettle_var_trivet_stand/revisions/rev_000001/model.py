from __future__ import annotations

# Stovetop whistling kettle on a trivet stand.
# Frame: Z is up. A 4-legged cast iron trivet ring sits on the stove at z=0.
#   The bell body sits on the trivet ring (top surface at z≈0.027) and lifts
#   straight up off the stand via a prismatic joint (like an electric kettle
#   lifting off its power base). Body is widest near the base and tapers up
#   to a narrower neck (bell silhouette).
# Spout points out along +X (angled upward off the lower shoulder).
# Handle bail arches over the top, pivoting about two mounts on the shoulder.
# Articulations (four joints):
#   - body off stand: PRISMATIC, lifts straight up off the trivet (+Z, ~0.06m).
#   - lid (with knob): PRISMATIC, lifts straight up off the kettle top (+Z, ~0.04m).
#   - whistle cap: REVOLUTE, the flap at the spout tip flips up about its hinge.
#   - bail handle: REVOLUTE, the arching handle swings down to the side about
#     the two top mounts.

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

# --- key dimensions ---
# Trivet stand: 4-leg cast iron ring the kettle sits on.
TRIVET_RING_R_OUTER = 0.085   # outer radius of the ring top
TRIVET_RING_R_INNER = 0.055   # inner radius (open center for heat)
TRIVET_RING_THICK = 0.008     # ring thickness
TRIVET_LEG_R = 0.012          # leg stub radius
TRIVET_LEG_H = 0.020          # leg stub height (ground contact to ring bottom)
TRIVET_TOP_Z = TRIVET_LEG_H + TRIVET_RING_THICK  # top surface where kettle seats

BODY_BASE_R = 0.090       # base radius of the bell body (at widest, relative to body origin)
BODY_TOP_R = 0.058        # radius at the top neck where the rim sits
RIM_Z = 0.150             # z of the kettle top rim (lid seat), relative to body origin
NECK_Z = 0.140            # z where the shoulder reaches the neck

LID_SEAT_Z = RIM_Z        # lid sits on the rim
LID_LIFT = 0.040          # lid prismatic lift travel

BODY_LIFT = 0.060         # body prismatic lift off the trivet (same as electric-kettle base)

# Spout: a hollow tapered tube whose bore is cut straight through the body
# shoulder wall so the spout mouth genuinely opens into the kettle interior.
SPOUT_ROOT = (0.086, 0.0, 0.055)   # visible root center on the outer shoulder wall
SPOUT_TIP = (0.150, 0.0, 0.105)    # angled-up tip / pour mouth
SPOUT_EMBED_DEPTH = 0.034           # how far the tube continues into the kettle wall
SPOUT_R_ROOT = 0.024               # outer radius at the root
SPOUT_R_TIP = 0.014                # outer radius at the tip
SPOUT_BORE_ROOT = 0.016            # inner bore radius at the root
SPOUT_BORE_TIP = 0.008             # inner bore radius at the tip
SPOUT_FAIRING_INSET = 0.024         # root fairing starts inside the curved shell
SPOUT_FAIRING_OUTSET = 0.016        # root fairing ends just outside the shell
SPOUT_FAIRING_R_BODY = 0.035        # broad saddle radius where it meets the body
SPOUT_FAIRING_R_OUTER = 0.027       # tapered radius where it blends into the spout

# Handle mounts: two lugs on the shoulder, offset in +/-Y, slightly +X biased.
# Mounts sit just outside the body neck so the bail legs clear the shell.
MOUNT_X = 0.0
MOUNT_Y = 0.080
MOUNT_Z = 0.134


def _loft_z(sections) -> cq.Workplane:
    # sections: list of (z, r) circles stacked along +Z (XY planes).
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
    # Bell-shaped body: wide rounded base flaring to a narrower neck, hollow
    # shell so the top reads as an open kettle mouth.
    outer = _loft_z(
        [
            (0.0, 0.072),
            (0.012, 0.090),
            (0.050, 0.089),
            (0.095, 0.080),
            (NECK_Z, BODY_TOP_R),
            (RIM_Z, BODY_TOP_R + 0.004),
        ]
    )
    inner = _loft_z(
        [
            (0.006, 0.066),
            (0.050, 0.083),
            (0.095, 0.074),
            (NECK_Z, BODY_TOP_R - 0.006),
            (RIM_Z + 0.02, BODY_TOP_R - 0.004),
        ]
    )
    # Cut the spout bore straight through the shoulder wall so the kettle
    # interior and the spout share one continuous channel (the mouth pours).
    return outer.cut(inner).cut(_spout_bore())


def _spout_axis():
    # Unit direction (in the XZ plane) and length of the root->tip spout axis.
    dx = SPOUT_TIP[0] - SPOUT_ROOT[0]
    dz = SPOUT_TIP[2] - SPOUT_ROOT[2]
    length = math.hypot(dx, dz)
    return (dx / length, 0.0, dz / length), length


def _spout_point(distance_from_root: float):
    ax, _ = _spout_axis()
    return (
        SPOUT_ROOT[0] + ax[0] * distance_from_root,
        0.0,
        SPOUT_ROOT[2] + ax[2] * distance_from_root,
    )


def _spout_bore() -> cq.Workplane:
    # Tapered through-bore. It starts behind the root (inside the hollow body
    # cavity) and runs past the tip, so cutting it from both the body shell and
    # the spout solid opens one continuous interior->mouth channel.
    ax, length = _spout_axis()
    bore_start = -SPOUT_EMBED_DEPTH - 0.006
    bore_end = length + 0.012
    root_in = _spout_point(bore_start)
    plane = cq.Plane(origin=root_in, normal=ax)
    return (
        cq.Workplane(plane)
        .circle(SPOUT_BORE_ROOT)
        .workplane(offset=bore_end - bore_start)
        .circle(SPOUT_BORE_TIP)
        .loft(ruled=True)
    )


def _spout_solid() -> cq.Workplane:
    # Hollow tapered spout: the outer tube starts inside the shoulder and passes
    # through the body wall, so the visible root cannot float off the shell.
    ax, length = _spout_axis()
    outer_start = _spout_point(-SPOUT_EMBED_DEPTH)
    plane = cq.Plane(origin=outer_start, normal=ax)
    outer = (
        cq.Workplane(plane)
        .circle(SPOUT_R_ROOT)
        .workplane(offset=length + SPOUT_EMBED_DEPTH)
        .circle(SPOUT_R_TIP)
        .loft(ruled=True)
    )
    return outer.cut(_spout_bore())


def _spout_root_fairing() -> cq.Workplane:
    # Short broad saddle/collar around the spout root. It begins inside the
    # curved kettle wall and tapers onto the tube, visually welding the two
    # pieces together instead of leaving an exposed tangent gap.
    ax, _ = _spout_axis()
    start = _spout_point(-SPOUT_FAIRING_INSET)
    plane = cq.Plane(origin=start, normal=ax)
    fairing = (
        cq.Workplane(plane)
        .circle(SPOUT_FAIRING_R_BODY)
        .workplane(offset=SPOUT_FAIRING_INSET + SPOUT_FAIRING_OUTSET)
        .circle(SPOUT_FAIRING_R_OUTER)
        .loft(ruled=False)
    )
    return fairing.cut(_spout_bore())


def _trivet_ring_mesh():
    # Flat annular ring (the trivet top platform the kettle seats on).
    ring = (
        cq.Workplane("XY")
        .circle(TRIVET_RING_R_OUTER)
        .circle(TRIVET_RING_R_INNER)
        .extrude(TRIVET_RING_THICK)
        .translate((0.0, 0.0, TRIVET_LEG_H))
    )
    return mesh_from_cadquery(ring, "trivet_ring")


def _trivet_leg_geometry():
    # Single leg stub cylinder (ground contact to ring underside).
    leg = CylinderGeometry(TRIVET_LEG_R, TRIVET_LEG_H, radial_segments=18)
    leg.translate(0.0, 0.0, TRIVET_LEG_H / 2.0)
    return leg


def _lid_mesh():
    # Round domed lid that seats on the rim.
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
    # Arching bail handle: a swept tube rising from one mount, over the top,
    # down to the other mount. Authored in the handle's local frame so the
    # pivot line (between the two mounts, along X at the rest top) is the
    # joint axis. We place the two ends at (+/-Y) and arch up in +Z.
    # In the child local frame the pivot is at the origin and runs along Y... we
    # instead author the bail in the parent frame and let the joint origin set
    # the pivot. Here we build it centered so the two leg tops are at the mounts.
    pts = [
        (MOUNT_X, MOUNT_Y, 0.0),
        (MOUNT_X + 0.006, MOUNT_Y * 0.92, 0.040),
        (MOUNT_X + 0.008, MOUNT_Y * 0.55, 0.082),
        (MOUNT_X + 0.008, 0.0, 0.100),
        (MOUNT_X + 0.008, -MOUNT_Y * 0.55, 0.082),
        (MOUNT_X + 0.006, -MOUNT_Y * 0.92, 0.040),
        (MOUNT_X, -MOUNT_Y, 0.0),
    ]
    bail = tube_from_spline_points(
        pts, radius=0.006, samples_per_segment=16, radial_segments=14
    )
    return mesh_from_geometry(bail, "bail_arch")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="whistling_kettle_on_trivet")

    # Materials
    cast_iron = model.material("cast_iron", rgba=(0.22, 0.22, 0.24, 1.0))
    steel = model.material("brushed_steel", rgba=(0.78, 0.79, 0.81, 1.0))
    steel_dark = model.material("steel_shadow", rgba=(0.66, 0.67, 0.69, 1.0))
    black = model.material("black_grip", rgba=(0.12, 0.12, 0.13, 1.0))

    # ---- trivet stand (root): 4-legged cast iron ring on the stove ----
    trivet = model.part("trivet_stand")

    # Annular ring top platform
    trivet.visual(_trivet_ring_mesh(), material=cast_iron, name="trivet_ring")

    # Four leg stubs evenly spaced under the ring (shared helper, loop pattern)
    for i in range(4):
        angle = i * math.pi / 2.0  # 0°, 90°, 180°, 270°
        r = (TRIVET_RING_R_OUTER + TRIVET_RING_R_INNER) / 2.0  # mid-ring radius
        leg = _trivet_leg_geometry()
        leg.translate(r * math.cos(angle), r * math.sin(angle), 0.0)
        trivet.visual(
            mesh_from_geometry(leg, f"trivet_leg_{i}"),
            material=cast_iron,
            name=f"trivet_leg_{i}",
        )

    trivet.inertial = Inertial.from_geometry(
        Cylinder(radius=TRIVET_RING_R_OUTER, length=TRIVET_TOP_Z),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, TRIVET_TOP_Z / 2.0)),
    )

    # ---- body: bell shell + spout + two handle mount lugs (sits on trivet) ----
    body = model.part("body")

    body_shell = _body_solid()
    body.visual(
        mesh_from_cadquery(body_shell, "body_shell"), material=steel, name="body_shell"
    )

    # Rim ring around the open top so the lid has a visible seat.
    rim = TorusGeometry(BODY_TOP_R + 0.002, 0.006, radial_segments=12, tubular_segments=40)
    rim.translate(0.0, 0.0, RIM_Z)
    body.visual(mesh_from_geometry(rim, "top_rim"), material=steel_dark, name="top_rim")

    # Spout off the +X shoulder: hollow tube whose bore opens into the body.
    body.visual(mesh_from_cadquery(_spout_solid(), "spout_shell"), material=steel, name="spout")
    body.visual(
        mesh_from_cadquery(_spout_root_fairing(), "spout_root_fairing"),
        material=steel_dark,
        name="spout_root_fairing",
    )

    # Two handle mount lugs on the shoulder (small steel bosses) that carry the bail.
    for sgn, nm in ((1.0, "mount_lug_0"), (-1.0, "mount_lug_1")):
        # Horizontal boss (axis along Y) bridging the body shell out to the
        # bail leg end so the swinging handle is carried by the body.
        lug = CylinderGeometry(0.009, 0.046, radial_segments=18).rotate_x(math.pi / 2.0)
        lug.translate(MOUNT_X, sgn * 0.078, MOUNT_Z)
        body.visual(mesh_from_geometry(lug, nm), material=steel_dark, name=nm)

    body.inertial = Inertial.from_geometry(
        Cylinder(radius=BODY_BASE_R, length=RIM_Z),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, RIM_Z / 2.0)),
    )

    # Prismatic joint: body lifts straight up off the trivet stand.
    # At q=0, body origin coincides with the trivet top surface.
    model.articulation(
        "trivet_to_body",
        ArticulationType.PRISMATIC,
        parent=trivet,
        child=body,
        origin=Origin(xyz=(0.0, 0.0, TRIVET_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=0.3, lower=0.0, upper=BODY_LIFT),
    )

    # ---- lid: domed round lid with a black knob; lifts straight up ----
    lid = model.part("lid")
    lid.visual(_lid_mesh(), material=steel, origin=Origin(xyz=(0.0, 0.0, 0.0)), name="lid_dome")
    # Knob stem + ball on top center.
    stem = CylinderGeometry(0.006, 0.012, radial_segments=16)
    stem.translate(0.0, 0.0, 0.030)
    lid.visual(mesh_from_geometry(stem, "knob_stem"), material=black, name="knob_stem")
    knob = SphereGeometry(0.011, width_segments=20, height_segments=14)
    knob.translate(0.0, 0.0, 0.040)
    lid.visual(mesh_from_geometry(knob, "knob_ball"), material=black, name="knob_ball")
    lid.inertial = Inertial.from_geometry(
        Cylinder(radius=BODY_TOP_R, length=0.024),
        mass=0.12,
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

    # ---- whistle cap: flap seated flush on the spout mouth, flips up ----
    # Hinge sits on the upper rim of the spout mouth; the flap disk lies in the
    # mouth plane (perpendicular to the spout axis) so at rest it caps the
    # opening flush instead of floating off the tip.
    cap = model.part("whistle_cap")
    spout_dir, _ = _spout_axis()
    spout_ang = math.atan2(spout_dir[2], spout_dir[0])
    mouth_r = SPOUT_R_TIP + 0.002  # disk just covers the tip opening
    # "Up" perpendicular to the spout axis, in the XZ plane.
    up = (-math.sin(spout_ang), math.cos(spout_ang))
    hinge = (
        SPOUT_TIP[0] + mouth_r * up[0],
        0.0,
        SPOUT_TIP[2] + mouth_r * up[1],
    )
    # Flap centre seats on the mouth centre; expressed in the hinge-local frame.
    off = (SPOUT_TIP[0] - hinge[0], 0.0, SPOUT_TIP[2] - hinge[2])
    flap = CylinderGeometry(mouth_r, 0.005, radial_segments=22)
    flap.rotate_y(math.pi / 2.0 - spout_ang)  # disk normal -> spout axis
    flap.translate(off[0], 0.0, off[2])
    cap.visual(mesh_from_geometry(flap, "cap_flap"), material=steel_dark, name="cap_flap")
    # Small black lift tab on the outer face of the flap.
    tab = BoxGeometry((0.006, 0.012, 0.010))
    tab.rotate_y(math.pi / 2.0 - spout_ang)
    tab.translate(off[0] + spout_dir[0] * 0.006, 0.0, off[2] + spout_dir[2] * 0.006)
    cap.visual(mesh_from_geometry(tab, "cap_tab"), material=black, name="cap_tab")
    cap.inertial = Inertial.from_geometry(
        Cylinder(radius=mouth_r, length=0.008), mass=0.01
    )
    model.articulation(
        "spout_to_cap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cap,
        # Hinge on the upper rim of the mouth; axis along Y so the flap flips up.
        origin=Origin(xyz=hinge),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=0.0, upper=1.4),
    )

    # ---- bail handle: arching tube with a black grip sleeve; swings to side ----
    handle = model.part("bail_handle")
    handle.visual(_bail_mesh(), material=steel, name="bail_arch")
    # Black grip sleeve over the top span of the arch.
    grip_pts = [
        (MOUNT_X + 0.008, 0.032, 0.092),
        (MOUNT_X + 0.008, 0.0, 0.100),
        (MOUNT_X + 0.008, -0.032, 0.092),
    ]
    grip = tube_from_spline_points(
        grip_pts, radius=0.011, samples_per_segment=14, radial_segments=16
    )
    handle.visual(mesh_from_geometry(grip, "grip_sleeve"), material=black, name="grip_sleeve")
    handle.inertial = Inertial.from_geometry(
        Box((0.02, 2 * MOUNT_Y, 0.11)),
        mass=0.06,
        origin=Origin(xyz=(MOUNT_X, 0.0, 0.055)),
    )
    model.articulation(
        "body_to_handle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        # Pivot at the mount line height; axis along Y (line between the two
        # mounts) so the arch swings over toward +/-X (falls to the side).
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

    trivet = object_model.get_part("trivet_stand")
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    cap = object_model.get_part("whistle_cap")
    handle = object_model.get_part("bail_handle")

    trivet_joint = object_model.get_articulation("trivet_to_body")
    lid_joint = object_model.get_articulation("body_to_lid")
    cap_joint = object_model.get_articulation("spout_to_cap")
    handle_joint = object_model.get_articulation("body_to_handle")

    # --- Trivet stand is a 4-legged ring at the base. ---
    trivet_aabb = ctx.part_world_aabb(trivet)
    trivet_ext = _ext(trivet_aabb)
    ctx.check(
        "trivet stand is low and ring-sized",
        trivet_aabb is not None
        and trivet_aabb[0][2] < 0.001  # sits on ground
        and trivet_ext[2] < 0.04  # low height
        and 0.14 < trivet_ext[0] < 0.22,  # reasonable ring diameter
        details=f"trivet aabb={trivet_aabb}, ext={trivet_ext}",
    )

    # --- Body sits on the trivet and lifts straight up off it. ---
    rest_body = ctx.part_world_position(body)
    ctx.check(
        "body is seated on the trivet at rest",
        rest_body is not None and abs(rest_body[2] - TRIVET_TOP_Z) < 0.002,
        details=f"body rest pos={rest_body}",
    )
    with ctx.pose({trivet_joint: BODY_LIFT}):
        lifted_body = ctx.part_world_position(body)
    ctx.check(
        "body lifts straight up off the trivet stand",
        lifted_body is not None
        and lifted_body[2] > rest_body[2] + 0.05
        and abs(lifted_body[0] - rest_body[0]) < 1e-4
        and abs(lifted_body[1] - rest_body[1]) < 1e-4,
        details=f"rest={rest_body}, lifted={lifted_body}",
    )

    # --- Kettle body is bell-shaped: wider at the base than at the top. ---
    body_aabb = ctx.part_world_aabb(body)
    ext = _ext(body_aabb)
    ctx.check(
        "kettle body is tall and bell-sized",
        ext[2] > 0.14 and 0.16 < ext[0] < 0.40,
        details=f"body extents={ext}",
    )

    # --- Lid seats on the rim and lifts straight up off the top. ---
    ctx.allow_overlap(
        lid,
        body,
        elem_a="lid_dome",
        elem_b="top_rim",
        reason="The lid skirt intentionally seats over the kettle rim.",
    )
    rest_lid = ctx.part_world_position(lid)
    ctx.check(
        "lid sits on top of the kettle",
        rest_lid is not None and rest_lid[2] > 0.13,
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

    # --- Spout is angled off the +X side of the body. ---
    spout_aabb = ctx.part_element_world_aabb(body, elem="spout")
    ctx.check(
        "spout projects out to the +X side and rises",
        spout_aabb is not None
        and spout_aabb[1][0] > 0.13
        and spout_aabb[1][2] > 0.09,
        details=f"spout aabb={spout_aabb}",
    )
    fairing_aabb = ctx.part_element_world_aabb(body, elem="spout_root_fairing")
    ctx.check(
        "spout root fairing wraps the kettle shoulder",
        fairing_aabb is not None
        and fairing_aabb[0][0] < 0.080
        and fairing_aabb[1][0] > 0.105
        and fairing_aabb[0][2] < 0.045
        and fairing_aabb[1][2] > 0.070,
        details=f"fairing aabb={fairing_aabb}",
    )
    ctx.allow_overlap(
        body,
        body,
        elem_a="spout_root_fairing",
        elem_b="body_shell",
        reason="The spout root fairing intentionally penetrates the kettle shoulder as a welded saddle.",
    )
    ctx.expect_contact(
        body,
        body,
        elem_a="spout_root_fairing",
        elem_b="body_shell",
        name="spout root fairing is seated into the kettle body",
    )
    ctx.allow_overlap(
        body,
        body,
        elem_a="spout",
        elem_b="spout_root_fairing",
        reason="The hollow spout tube intentionally passes through its welded root fairing.",
    )
    ctx.expect_contact(
        body,
        body,
        elem_a="spout",
        elem_b="spout_root_fairing",
        name="spout tube blends into the root fairing",
    )

    # --- Whistle cap is at the spout tip and flips up about its hinge. ---
    cap_rest = ctx.part_world_aabb(cap)
    ctx.check(
        "whistle cap sits at the spout tip",
        cap_rest is not None and cap_rest[1][0] > 0.13,
        details=f"cap aabb={cap_rest}",
    )
    top_rest = cap_rest[1][2]
    with ctx.pose({cap_joint: 1.4}):
        cap_open = ctx.part_world_aabb(cap)
    ctx.check(
        "whistle cap flips up when opened",
        cap_open is not None and cap_open[1][2] > top_rest + 0.005,
        details=f"rest_top={top_rest}, open_top={cap_open[1][2] if cap_open else None}",
    )

    # --- Bail handle arches over the top and swings down to the side. ---
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
        "bail handle falls to the side (drops and swings out in X)",
        down_top < rest_top - 0.03 and down_xspan > rest_xspan + 0.03,
        details=f"rest_top={rest_top}, down_top={down_top}, "
        f"rest_xspan={rest_xspan}, down_xspan={down_xspan}",
    )

    # --- Handle is mounted to the body lugs (not floating). ---
    ctx.allow_overlap(
        handle,
        body,
        elem_a="bail_arch",
        elem_b="mount_lug_0",
        reason="Bail leg ends seat into the mount lugs that carry the pivot.",
    )
    ctx.allow_overlap(
        handle,
        body,
        elem_a="bail_arch",
        elem_b="mount_lug_1",
        reason="Bail leg ends seat into the mount lugs that carry the pivot.",
    )
    ctx.expect_contact(handle, body, name="bail handle connected to body lugs")

    # --- Body seats on the trivet ring (contact at rest). ---
    ctx.expect_contact(
        body,
        trivet,
        elem_a="body_shell",
        elem_b="trivet_ring",
        name="body shell seats on the trivet ring at rest",
    )

    # --- Whistle cap nests on the spout tip (intentional seating). ---
    ctx.allow_overlap(
        cap,
        body,
        elem_a="cap_flap",
        elem_b="spout",
        reason="The whistle flap caps the spout mouth when closed.",
    )
    ctx.expect_contact(cap, body, name="whistle cap seated on spout")

    return ctx.report()


object_model = build_object_model()
