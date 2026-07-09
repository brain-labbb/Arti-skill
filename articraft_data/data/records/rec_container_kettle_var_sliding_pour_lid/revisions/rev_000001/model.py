from __future__ import annotations

# Stovetop whistling kettle with a sliding pour-shutter lid.
# Frame: Z is up. The kettle bell body sits on the stove with its base at z=0
#   and its open top rim near z=0.155. The body is widest at the base and
#   tapers up to a narrower neck (bell silhouette).
# Spout points out along +X (angled upward off the lower shoulder).
# Handle bail arches over the top, pivoting about two mounts on the shoulder.
# Lid: a domed lid sits permanently on the rim with a pour opening cut through
#   the top. A crescent shutter plate slides horizontally (PRISMATIC along +Y)
#   across the dome surface to open or close the pour opening.
# Articulations (three INDEPENDENT joints):
#   - shutter: PRISMATIC, slides horizontally in +Y to uncover the pour opening.
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
BODY_BASE_R = 0.090       # base radius of the bell body
BODY_TOP_R = 0.058        # radius at the top neck where the rim sits
RIM_Z = 0.150             # z of the kettle top rim (lid seat)
NECK_Z = 0.140            # z where the shoulder reaches the neck

# Lid dome (fixed on the rim) with a pour opening for the sliding shutter.
DOME_TOP_Z = 0.014        # dome platform height above rim in lid-local frame
POUR_OPEN_Y = -0.015      # pour opening centre offset in lid-local Y
POUR_OPEN_R = 0.014       # pour opening radius

# Shutter: crescent plate that slides along +Y across the dome top.
SHUTTER_R = 0.022          # shutter plate outer radius
SHUTTER_THICK = 0.003      # plate thickness
SHUTTER_SLIDE = 0.035      # prismatic travel (closed -> open)

# Track rails flanking the shutter slide path.
RAIL_W = 0.003             # rail width (X)
RAIL_H = 0.005             # rail height (Z) above dome top
RAIL_HALF_SPAN = 0.025     # rail X offset from centre (half-gap)
RAIL_Y_START = -0.040      # rail start Y (lid-local)
RAIL_Y_END = 0.045         # rail end Y (lid-local)

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


def _lid_dome_solid() -> cq.Workplane:
    # Low-profile dome lid that seats on the rim. A circular pour opening is
    # cut through the dome platform so the sliding shutter can cover or reveal
    # it. The dome is flatter than a classic kettle dome to give the shutter a
    # near-planar track surface.
    dome = _loft_z(
        [
            (0.0, BODY_TOP_R + 0.006),       # skirt lip over rim
            (0.004, BODY_TOP_R + 0.003),     # lower slope
            (0.008, BODY_TOP_R),             # mid slope
            (0.011, BODY_TOP_R - 0.002),     # upper slope
            (DOME_TOP_Z, BODY_TOP_R - 0.004),  # flat platform
        ]
    )
    # Cut the pour opening: vertical cylinder through the dome.
    opening = (
        cq.Workplane("XY")
        .center(0.0, POUR_OPEN_Y)
        .circle(POUR_OPEN_R)
        .extrude(0.025)
    )
    return dome.cut(opening)


def _shutter_plate_solid() -> cq.Workplane:
    # Crescent-shaped shutter plate. A circular disk with a concave notch cut
    # from one side gives it the crescent silhouette. The notch is shallow
    # enough that the plate still covers the pour opening when closed.
    # A small grip tab protrudes from the +Y edge for the user to slide.
    plate = (
        cq.Workplane("XY")
        .circle(SHUTTER_R)
        .extrude(SHUTTER_THICK)
    )
    # Crescent notch: a circle offset in +X removes material from the +X side.
    notch = (
        cq.Workplane("XY")
        .center(0.026, 0.0)
        .circle(0.013)
        .extrude(SHUTTER_THICK)
    )
    crescent = plate.cut(notch)
    # Grip tab on the +Y edge, taller than the plate for finger purchase.
    tab = (
        cq.Workplane("XY")
        .center(0.0, SHUTTER_R + 0.001)
        .rect(0.012, 0.006)
        .extrude(0.008)
    )
    return crescent.union(tab)


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
    model = ArticulatedObject(name="whistling_kettle")

    steel = model.material("brushed_steel", rgba=(0.78, 0.79, 0.81, 1.0))
    steel_dark = model.material("steel_shadow", rgba=(0.66, 0.67, 0.69, 1.0))
    black = model.material("black_grip", rgba=(0.12, 0.12, 0.13, 1.0))
    copper = model.material("copper_accent", rgba=(0.72, 0.45, 0.20, 1.0))

    # ---- body (root): bell shell + spout + handle mounts + fixed lid dome ----
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

    # ---- Fixed lid dome: seated permanently on the rim with a pour opening. ----
    body.visual(
        mesh_from_cadquery(_lid_dome_solid(), "lid_dome"),
        material=steel,
        origin=Origin(xyz=(0.0, 0.0, RIM_Z)),
        name="lid_dome",
    )
    # Raised pour lip ring around the opening on the dome platform.
    pour_lip = TorusGeometry(
        POUR_OPEN_R + 0.002, 0.002, radial_segments=10, tubular_segments=32
    )
    pour_lip.translate(0.0, POUR_OPEN_Y, RIM_Z + DOME_TOP_Z)
    body.visual(mesh_from_geometry(pour_lip, "pour_lip"), material=copper, name="pour_lip")
    # Fixed knob finial on the dome: the stem extends from the dome platform
    # up past the shutter travel so the knob is physically rooted in the dome.
    # The thin stem passes through the shutter's slide path (allowed overlap).
    knob_base_z = RIM_Z + DOME_TOP_Z
    knob_top_z = knob_base_z + SHUTTER_THICK + 0.014
    stem_height = knob_top_z - knob_base_z
    knob_stem = CylinderGeometry(0.004, stem_height, radial_segments=14)
    knob_stem.translate(0.0, 0.0, knob_base_z + stem_height / 2.0)
    body.visual(mesh_from_geometry(knob_stem, "knob_stem"), material=black, name="knob_stem")
    knob_ball = SphereGeometry(0.007, width_segments=16, height_segments=12)
    knob_ball.translate(0.0, 0.0, knob_top_z)
    body.visual(mesh_from_geometry(knob_ball, "knob_ball"), material=black, name="knob_ball")
    # Two track rails on the dome platform, flanking the shutter slide path.
    rail_len = RAIL_Y_END - RAIL_Y_START
    rail_y_c = (RAIL_Y_START + RAIL_Y_END) / 2.0
    rail_z_c = RIM_Z + DOME_TOP_Z + RAIL_H / 2.0
    for i in range(2):
        sgn = 1.0 if i == 0 else -1.0
        nm = f"track_rail_{i}"
        rail = BoxGeometry((RAIL_W, rail_len, RAIL_H))
        rail.translate(sgn * RAIL_HALF_SPAN, rail_y_c, rail_z_c)
        body.visual(mesh_from_geometry(rail, nm), material=steel_dark, name=nm)

    body.inertial = Inertial.from_geometry(
        Cylinder(radius=BODY_BASE_R, length=RIM_Z),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, RIM_Z / 2.0)),
    )

    # ---- shutter: crescent plate that slides along +Y across the dome ----
    shutter = model.part("shutter")
    shutter.visual(
        mesh_from_cadquery(_shutter_plate_solid(), "shutter_plate"),
        material=steel_dark,
        name="shutter_plate",
    )
    shutter.inertial = Inertial.from_geometry(
        Cylinder(radius=SHUTTER_R, length=SHUTTER_THICK),
        mass=0.02,
        origin=Origin(xyz=(0.0, 0.0, SHUTTER_THICK / 2.0)),
    )
    model.articulation(
        "body_to_shutter",
        ArticulationType.PRISMATIC,
        parent=body,
        child=shutter,
        # Joint origin at the pour opening on the dome platform surface.
        origin=Origin(xyz=(0.0, POUR_OPEN_Y, RIM_Z + DOME_TOP_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0, velocity=0.3, lower=0.0, upper=SHUTTER_SLIDE
        ),
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

    body = object_model.get_part("body")
    shutter = object_model.get_part("shutter")
    cap = object_model.get_part("whistle_cap")
    handle = object_model.get_part("bail_handle")

    shutter_joint = object_model.get_articulation("body_to_shutter")
    cap_joint = object_model.get_articulation("spout_to_cap")
    handle_joint = object_model.get_articulation("body_to_handle")

    # --- Kettle body is bell-shaped: wider at the base than at the top. ---
    body_aabb = ctx.part_world_aabb(body)
    ext = _ext(body_aabb)
    ctx.check(
        "kettle body is tall and bell-sized",
        ext[2] > 0.14 and 0.16 < ext[0] < 0.40,
        details=f"body extents={ext}",
    )

    # --- Lid dome is fixed on the body with a visible pour opening. ---
    dome_aabb = ctx.part_element_world_aabb(body, elem="lid_dome")
    ctx.check(
        "lid dome sits on top of the kettle rim",
        dome_aabb is not None
        and dome_aabb[0][2] > RIM_Z - 0.005
        and dome_aabb[1][2] > RIM_Z + 0.008,
        details=f"dome aabb={dome_aabb}",
    )
    pour_lip_aabb = ctx.part_element_world_aabb(body, elem="pour_lip")
    ctx.check(
        "pour lip ring is on the dome platform",
        pour_lip_aabb is not None
        and pour_lip_aabb[0][2] > RIM_Z + DOME_TOP_Z - 0.004
        and pour_lip_aabb[1][2] < RIM_Z + DOME_TOP_Z + 0.006,
        details=f"pour_lip aabb={pour_lip_aabb}",
    )

    # --- Shutter sits on the dome and slides horizontally to open/close. ---
    # The shutter plate seats on the dome surface (intentional contact).
    ctx.allow_overlap(
        shutter,
        body,
        elem_a="shutter_plate",
        elem_b="lid_dome",
        reason="The shutter plate slides on the dome platform surface with intentional surface contact.",
    )
    # The knob stem passes through the shutter slide path (the shutter slides
    # past the thin stem post that roots the knob in the dome).
    ctx.allow_overlap(
        shutter,
        body,
        elem_a="shutter_plate",
        elem_b="knob_stem",
        reason="The knob stem post passes through the shutter slide path; the crescent shutter slides past the thin fixed post.",
    )
    rest_shutter = ctx.part_world_position(shutter)
    ctx.check(
        "shutter sits on the dome at the pour opening height",
        rest_shutter is not None and rest_shutter[2] > RIM_Z + DOME_TOP_Z - 0.005,
        details=f"shutter pos={rest_shutter}",
    )
    # At rest (q=0), shutter covers the pour opening.
    ctx.expect_overlap(
        shutter,
        body,
        axes="xy",
        elem_a="shutter_plate",
        elem_b="pour_lip",
        min_overlap=0.008,
        name="closed shutter overlaps the pour lip footprint",
    )
    # Shutter slides in +Y: open pose moves it away from the opening.
    with ctx.pose({shutter_joint: SHUTTER_SLIDE}):
        open_shutter = ctx.part_world_position(shutter)
    ctx.check(
        "shutter slides horizontally in +Y to open the pour opening",
        open_shutter is not None
        and rest_shutter is not None
        and open_shutter[1] > rest_shutter[1] + SHUTTER_SLIDE - 0.002
        and abs(open_shutter[0] - rest_shutter[0]) < 1e-4
        and abs(open_shutter[2] - rest_shutter[2]) < 1e-4,
        details=f"rest={rest_shutter}, open={open_shutter}",
    )
    # At max slide, the shutter has moved past the pour opening in Y.
    with ctx.pose({shutter_joint: SHUTTER_SLIDE}):
        open_shutter_aabb = ctx.part_world_aabb(shutter)
    pour_open_y = POUR_OPEN_Y + RIM_Z * 0.0  # pour opening world Y = local Y
    ctx.check(
        "open shutter is clear of the pour opening in +Y",
        open_shutter_aabb is not None
        and open_shutter_aabb[0][1] > POUR_OPEN_Y + POUR_OPEN_R - 0.002,
        details=f"open shutter aabb={open_shutter_aabb}, pour_open_y={POUR_OPEN_Y}",
    )

    # --- Track rails flank the shutter path on the dome. ---
    for i in range(2):
        nm = f"track_rail_{i}"
        rail_aabb = ctx.part_element_world_aabb(body, elem=nm)
        ctx.check(
            f"{nm} sits on the dome platform",
            rail_aabb is not None
            and rail_aabb[0][2] > RIM_Z + DOME_TOP_Z - 0.004
            and rail_aabb[1][2] < RIM_Z + DOME_TOP_Z + RAIL_H + 0.004,
            details=f"{nm} aabb={rail_aabb}",
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
