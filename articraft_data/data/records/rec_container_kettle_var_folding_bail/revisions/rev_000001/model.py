from __future__ import annotations

# Stovetop whistling kettle with a two-segment folding bail handle.
# Frame: Z is up. The kettle bell body sits on the stove with its base at z=0
#   and its open top rim near z=0.155. The body is widest at the base and
#   tapers up to a narrower neck (bell silhouette).
# Spout points out along +X (angled upward off the lower shoulder).
# Folding bail: lower legs pivot at shoulder mounts, upper grip span hinges
#   at the knuckle so the grip folds flat down against the body when stowed.
# Articulations (four INDEPENDENT joints):
#   - lid (with knob): PRISMATIC, lifts straight up off the kettle top (+Z, ~0.04m).
#   - whistle cap: REVOLUTE, the flap at the spout tip flips up about its hinge.
#   - bail legs: REVOLUTE, the lower leg pair swings to the side at the mounts.
#   - bail grip: REVOLUTE, the upper grip span folds at the knuckle hinge.

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
BODY_BASE_R = 0.090       # base radius of the bell body
BODY_TOP_R = 0.058        # radius at the top neck where the rim sits
RIM_Z = 0.150             # z of the kettle top rim (lid seat)
NECK_Z = 0.140            # z where the shoulder reaches the neck

LID_SEAT_Z = RIM_Z        # lid sits on the rim
LID_LIFT = 0.040          # prismatic lift travel

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


# Knuckle hinge location in the legs-local frame (measured from mount line).
KNUCKLE_X = 0.008
KNUCKLE_Y_HALF = MOUNT_Y * 0.55   # half-span between the two knuckle pivots
KNUCKLE_Z = 0.082                 # height above mount line


def _leg_mesh(sgn: float):
    """One lower leg tube: from a mount lug up to its knuckle point.

    Authored in the legs-local frame where the mount line is at z=0 and the
    knuckle line is at z=KNUCKLE_Z.  ``sgn`` selects the +Y or -Y leg.
    """
    pts = [
        (0.0, sgn * MOUNT_Y, 0.0),
        (0.004, sgn * MOUNT_Y * 0.92, 0.030),
        (0.007, sgn * MOUNT_Y * 0.70, 0.058),
        (KNUCKLE_X, sgn * KNUCKLE_Y_HALF, KNUCKLE_Z),
    ]
    leg = tube_from_spline_points(
        pts, radius=0.005, samples_per_segment=12, radial_segments=12
    )
    return mesh_from_geometry(leg, f"leg_{'0' if sgn > 0 else '1'}")


def _grip_arch_mesh():
    """Upper grip arch: spans between the two knuckle pivots and crowns above.

    Authored in the grip-local frame where the knuckle joint origin is at
    (0, 0, 0) and the arch rises in local +Z.
    """
    pts = [
        (0.0, KNUCKLE_Y_HALF, 0.0),
        (0.0, KNUCKLE_Y_HALF * 0.65, 0.020),
        (0.0, 0.0, 0.038),
        (0.0, -KNUCKLE_Y_HALF * 0.65, 0.020),
        (0.0, -KNUCKLE_Y_HALF, 0.0),
    ]
    arch = tube_from_spline_points(
        pts, radius=0.005, samples_per_segment=14, radial_segments=12
    )
    return mesh_from_geometry(arch, "grip_arch")


def _grip_sleeve_mesh():
    """Black grip sleeve over the centre of the arch."""
    pts = [
        (0.0, 0.026, 0.026),
        (0.0, 0.0, 0.038),
        (0.0, -0.026, 0.026),
    ]
    sleeve = tube_from_spline_points(
        pts, radius=0.010, samples_per_segment=12, radial_segments=14
    )
    return mesh_from_geometry(sleeve, "grip_sleeve")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="whistling_kettle")

    steel = model.material("brushed_steel", rgba=(0.78, 0.79, 0.81, 1.0))
    steel_dark = model.material("steel_shadow", rgba=(0.66, 0.67, 0.69, 1.0))
    black = model.material("black_grip", rgba=(0.12, 0.12, 0.13, 1.0))

    # ---- body (root): bell shell + spout + two handle mount lugs ----
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

    # ---- bail legs: lower leg pair pivoting at the shoulder mounts ----
    # Two steel tubes run from each mount lug up to the mid-height knuckle
    # line.  The whole pair swings as one rigid member about the mount axis.
    legs = model.part("bail_legs")
    for i, sgn in enumerate((1.0, -1.0)):
        legs.visual(
            _leg_mesh(sgn), material=steel, name=f"leg_{i}"
        )
    # Knuckle pin: a short cross-bar at the knuckle line so the grip hinge
    # has a visible barrel to pivot on.
    knuckle_bar = CylinderGeometry(0.006, 2 * KNUCKLE_Y_HALF + 0.008, radial_segments=16)
    knuckle_bar.rotate_x(math.pi / 2.0)
    knuckle_bar.translate(KNUCKLE_X, 0.0, KNUCKLE_Z)
    legs.visual(
        mesh_from_geometry(knuckle_bar, "knuckle_bar"),
        material=steel_dark,
        name="knuckle_bar",
    )
    legs.inertial = Inertial.from_geometry(
        Box((0.02, 2 * MOUNT_Y, KNUCKLE_Z + 0.02)),
        mass=0.03,
        origin=Origin(xyz=(KNUCKLE_X / 2.0, 0.0, KNUCKLE_Z / 2.0)),
    )
    model.articulation(
        "body_to_legs",
        ArticulationType.REVOLUTE,
        parent=body,
        child=legs,
        # Pivot at the mount line height; axis along Y so the legs swing
        # over toward +/-X (falls to the side).
        origin=Origin(xyz=(MOUNT_X, 0.0, MOUNT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=3.0, lower=0.0, upper=1.6),
    )

    # ---- bail grip: upper arch span hinged at the knuckle ----
    # The grip arch connects the two leg tops and folds flat against the body
    # when stowed.  A black sleeve wraps the centre for a comfortable hold.
    grip = model.part("bail_grip")
    grip.visual(_grip_arch_mesh(), material=steel, name="grip_arch")
    grip.visual(_grip_sleeve_mesh(), material=black, name="grip_sleeve")
    grip.inertial = Inertial.from_geometry(
        Box((0.02, 2 * KNUCKLE_Y_HALF, 0.04)),
        mass=0.03,
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
    )
    model.articulation(
        "legs_to_grip",
        ArticulationType.REVOLUTE,
        parent=legs,
        child=grip,
        # Knuckle hinge at the top of the legs (in legs-local frame).
        # Axis along Y so positive q folds the grip inward (toward the body).
        origin=Origin(xyz=(KNUCKLE_X, 0.0, KNUCKLE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=0.0, upper=1.6),
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

    lid_joint = object_model.get_articulation("body_to_lid")
    cap_joint = object_model.get_articulation("spout_to_cap")

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

    # --- Folding bail: two-segment mechanism (legs + grip) ---
    bail_legs = object_model.get_part("bail_legs")
    bail_grip = object_model.get_part("bail_grip")
    legs_joint = object_model.get_articulation("body_to_legs")
    grip_joint = object_model.get_articulation("legs_to_grip")

    # At rest both segments form a full arch above the kettle top.
    legs_rest = ctx.part_world_aabb(bail_legs)
    grip_rest = ctx.part_world_aabb(bail_grip)
    ctx.check(
        "bail legs reach above the kettle body",
        legs_rest is not None and legs_rest[1][2] > 0.20,
        details=f"legs aabb={legs_rest}",
    )
    ctx.check(
        "bail grip crowns above the legs at rest",
        grip_rest is not None and grip_rest[1][2] > 0.24,
        details=f"grip aabb={grip_rest}",
    )

    # Legs swing to the side at the mount pivot.
    legs_rest_top = legs_rest[1][2]
    legs_rest_xspan = legs_rest[1][0] - legs_rest[0][0]
    with ctx.pose({legs_joint: 1.5}):
        legs_down = ctx.part_world_aabb(bail_legs)
    legs_down_top = legs_down[1][2]
    legs_down_xspan = legs_down[1][0] - legs_down[0][0]
    ctx.check(
        "bail legs swing to the side at the mounts",
        legs_down_top < legs_rest_top - 0.03 and legs_down_xspan > legs_rest_xspan + 0.03,
        details=f"rest_top={legs_rest_top}, down_top={legs_down_top}, "
        f"rest_xspan={legs_rest_xspan}, down_xspan={legs_down_xspan}",
    )

    # Grip folds flat at the knuckle when both joints are driven.
    grip_rest_top = grip_rest[1][2]
    with ctx.pose({legs_joint: 1.5, grip_joint: 1.5}):
        grip_folded = ctx.part_world_aabb(bail_grip)
    grip_folded_top = grip_folded[1][2]
    ctx.check(
        "bail grip folds flat when both joints are driven",
        grip_folded_top < grip_rest_top - 0.04,
        details=f"grip_rest_top={grip_rest_top}, grip_folded_top={grip_folded_top}",
    )

    # Grip knuckle folds the grip inward even with legs held at rest.
    with ctx.pose({grip_joint: 1.4}):
        grip_knuckle = ctx.part_world_aabb(bail_grip)
    grip_knuckle_top = grip_knuckle[1][2]
    ctx.check(
        "grip folds inward at the knuckle when legs are still up",
        grip_knuckle_top < grip_rest_top - 0.02,
        details=f"grip_rest_top={grip_rest_top}, knuckle_top={grip_knuckle_top}",
    )

    # Legs are mounted to the body lugs (not floating).
    for i in range(2):
        ctx.allow_overlap(
            bail_legs,
            body,
            elem_a=f"leg_{i}",
            elem_b=f"mount_lug_{i}",
            reason="Bail leg lower end seats into the mount lug that carries the shoulder pivot.",
        )
    ctx.expect_contact(bail_legs, body, name="bail legs connected to body lugs")

    # Grip is mounted on the legs at the knuckle (not floating).
    ctx.allow_overlap(
        bail_grip,
        bail_legs,
        elem_a="grip_arch",
        elem_b="knuckle_bar",
        reason="Grip arch endpoints seat on the knuckle pin that carries the folding hinge.",
    )
    ctx.expect_contact(bail_grip, bail_legs, name="bail grip connected to legs at knuckle")

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
