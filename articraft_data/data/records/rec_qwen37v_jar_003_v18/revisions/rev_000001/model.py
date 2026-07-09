from __future__ import annotations

# Honey jar with a dipper holder on the screw-on lid.
# Frame: vertical axis +Z, jar centered on the world Z axis, base on z=0.
#   - body: a clear, wide square-section (rounded-corner) hollow glass jar.
#           Main body has thin 4mm walls; the mouth rim (top 15mm) has
#           thickened 7mm glass walls with a circular opening — clearly
#           showing the glass wall thickness at the mouth.
#   - lid : a round wooden lid with a short skirt that screws into the
#           jar mouth (CONTINUOUS joint about +Z). A cylindrical dipper
#           holder post sits on top of the lid, with a honey dipper stick
#           inserted into it.
#
# Articulation:
#   - body_to_lid: CONTINUOUS about +Z. The lid rotates freely (screw thread).

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Inertial,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

# ----- jar dimensions (meters) -----
JAR_SECT = 0.080          # outer square section width
CORNER_R = 0.008          # rounded corner radius
GLASS_WALL = 0.004        # regular glass wall thickness
MOUTH_RIM_H = 0.015       # height of thickened mouth rim section
JAR_BODY_H = 0.085        # main body height (below mouth rim)
JAR_TOTAL_H = JAR_BODY_H + MOUTH_RIM_H  # 0.100 m total
MOUTH_OPENING_R = 0.030   # mouth circular opening radius (60mm dia)
GLASS_FLOOR = 0.004       # glass floor thickness

# ----- lid dimensions -----
LID_D = 0.074             # lid disk outer diameter
LID_H = 0.012             # lid disk height
LID_SKIRT_D = 0.056       # skirt diameter (fits inside mouth opening)
LID_SKIRT_H = 0.008       # skirt depth into mouth

# ----- dipper holder dimensions -----
HOLDER_OD = 0.016         # holder outer diameter
HOLDER_WALL = 0.003       # holder wall thickness
HOLDER_H = 0.028          # holder height above lid top

# ----- dipper stick dimensions -----
DIPPER_HANDLE_D = 0.006   # dipper handle rod diameter
DIPPER_HANDLE_L = 0.055   # dipper handle length above holder
DIPPER_BALL_D = 0.016     # dipper ball (grooved end) diameter


def _jar_body_solid() -> cq.Workplane:
    """Hollow rounded-square glass jar with thickened mouth rim.

    The main body has thin walls; the mouth section has a circular opening
    through thicker glass, showing visible wall thickness at the mouth.
    """
    # Outer shell: rounded square, full height
    outer = (
        cq.Workplane("XY")
        .rect(JAR_SECT, JAR_SECT)
        .extrude(JAR_TOTAL_H)
        .edges("|Z")
        .fillet(CORNER_R)
    )

    # Lower cavity: thin-walled hollow from glass floor up to mouth start
    inner_w = JAR_SECT - 2.0 * GLASS_WALL
    lower_cavity = (
        cq.Workplane("XY")
        .workplane(offset=GLASS_FLOOR)
        .rect(inner_w, inner_w)
        .extrude(JAR_BODY_H)  # up to mouth start
        .edges("|Z")
        .fillet(max(CORNER_R - GLASS_WALL, 0.002))
    )

    # Upper mouth cavity: circular opening through the mouth rim section
    # This creates visibly thicker glass at the mouth corners
    mouth_cavity = (
        cq.Workplane("XY")
        .workplane(offset=JAR_BODY_H)
        .circle(MOUTH_OPENING_R)
        .extrude(MOUTH_RIM_H + 0.002)  # overcut to open through top
    )

    return outer.cut(lower_cavity).cut(mouth_cavity)


def _lid_solid() -> cq.Workplane:
    """Round wooden lid with a skirt that fits into the jar mouth.

    Lid local frame: open underside (skirt bottom) at z=0, disk top at z=LID_H.
    The skirt extends below z=0 into negative Z (into the mouth).
    Actually, let's build it with the skirt top at z=0 and disk above.
    
    Revised: skirt bottom at z=-LID_SKIRT_H, disk top at z=LID_H.
    Part origin at the jar top surface (z=0 in lid frame = z=JAR_TOTAL_H in world).
    """
    # Main disk
    disk = (
        cq.Workplane("XY")
        .circle(LID_D / 2.0)
        .extrude(LID_H)
    )

    # Skirt (threaded portion that goes into the mouth)
    skirt = (
        cq.Workplane("XY")
        .workplane(offset=-LID_SKIRT_H)
        .circle(LID_SKIRT_D / 2.0)
        .extrude(LID_SKIRT_H)
    )

    # Add a small rim/lip at the bottom of the disk for visual detail
    rim = (
        cq.Workplane("XY")
        .circle(LID_D / 2.0)
        .extrude(0.002)
    )

    return disk.union(skirt).union(rim)


def _dipper_holder_solid() -> cq.Workplane:
    """Cylindrical dipper holder tube that sits on top of the lid.

    Built in lid-local frame: base at z=LID_H (on top of the lid disk).
    """
    # Outer cylinder
    outer = (
        cq.Workplane("XY")
        .workplane(offset=LID_H)
        .circle(HOLDER_OD / 2.0)
        .extrude(HOLDER_H)
    )
    # Inner bore (hollow tube)
    inner_r = HOLDER_OD / 2.0 - HOLDER_WALL
    inner = (
        cq.Workplane("XY")
        .workplane(offset=LID_H)
        .circle(inner_r)
        .extrude(HOLDER_H - HOLDER_WALL)  # closed bottom, open top
    )
    return outer.cut(inner)


def _dipper_stick_solid() -> cq.Workplane:
    """Honey dipper stick: rod with a ball at the bottom end.

    Built in lid-local frame: ball center sits near the holder bottom,
    handle extends upward above the holder.
    """
    # Ball at the bottom (inside the holder)
    ball_z_center = LID_H + 0.002  # just above holder floor
    ball = (
        cq.Workplane("XY")
        .workplane(offset=ball_z_center)
        .sphere(DIPPER_BALL_D / 2.0)
    )

    # Handle rod extending upward from ball through holder and above
    handle_start_z = ball_z_center + DIPPER_BALL_D / 2.0 - 0.002
    handle = (
        cq.Workplane("XY")
        .workplane(offset=handle_start_z)
        .circle(DIPPER_HANDLE_D / 2.0)
        .extrude(DIPPER_HANDLE_L)
    )

    # Small knob at top of handle
    knob_z = handle_start_z + DIPPER_HANDLE_L
    knob = (
        cq.Workplane("XY")
        .workplane(offset=knob_z)
        .sphere(DIPPER_HANDLE_D * 0.8)
    )

    return ball.union(handle).union(knob)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="honey_jar")

    glass = model.material("clear_glass", rgba=(0.85, 0.88, 0.82, 0.30))
    wood = model.material("natural_wood", rgba=(0.72, 0.52, 0.30, 1.0))
    light_wood = model.material("light_wood", rgba=(0.82, 0.65, 0.38, 1.0))

    # ---- body (root): clear glass honey jar ----
    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_jar_body_solid(), "glass_jar"),
        material=glass,
        name="glass_jar",
    )
    body.inertial = Inertial.from_geometry(
        Box((JAR_SECT, JAR_SECT, JAR_TOTAL_H)),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, JAR_TOTAL_H / 2.0)),
    )

    # ---- lid: wooden screw-on lid with dipper holder ----
    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_solid(), "lid_disk"),
        material=wood,
        name="lid_disk",
    )
    lid.visual(
        mesh_from_cadquery(_dipper_holder_solid(), "dipper_holder"),
        material=wood,
        name="dipper_holder",
    )
    lid.visual(
        mesh_from_cadquery(_dipper_stick_solid(), "dipper_stick"),
        material=light_wood,
        name="dipper_stick",
    )
    lid.inertial = Inertial.from_geometry(
        Cylinder(radius=LID_D / 2.0, length=LID_H + HOLDER_H + LID_SKIRT_H),
        mass=0.06,
        origin=Origin(xyz=(0.0, 0.0, (LID_H + HOLDER_H) / 2.0)),
    )

    # Lid screws onto jar mouth. CONTINUOUS joint about +Z allows unlimited
    # rotation representing the screw thread. At q=0 the lid is seated.
    model.articulation(
        "body_to_lid",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, JAR_TOTAL_H)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    screw = object_model.get_articulation("body_to_lid")

    # The lid skirt intentionally inserts into the jar mouth opening.
    ctx.allow_overlap(
        lid,
        body,
        elem_a="lid_disk",
        elem_b="glass_jar",
        reason="Lid skirt is intentionally inserted into the jar mouth (screw-thread fit).",
    )

    # ---- jar is wider than tall (honey jar proportions) ----
    body_aabb = ctx.part_world_aabb(body)
    mn, mx = body_aabb
    body_dx = mx[0] - mn[0]
    body_dy = mx[1] - mn[1]
    body_dz = mx[2] - mn[2]
    ctx.check(
        "jar body is wider than or comparable to its height (jar proportions)",
        body_dx >= body_dz * 0.6,
        details=f"body extents: dx={body_dx:.4f}, dy={body_dy:.4f}, dz={body_dz:.4f}",
    )

    # ---- mouth has thickened glass walls (circular opening through square section) ----
    # The mouth cavity is circular (r=30mm) while the outer section is square (80mm),
    # so the glass at the mouth corners is much thicker than the body walls.
    # We verify the mouth opening is smaller than the jar section.
    ctx.check(
        "mouth opening is smaller than jar section (thick walls at mouth)",
        2.0 * MOUTH_OPENING_R < JAR_SECT - 2.0 * GLASS_WALL - 0.005,
        details=f"mouth_dia={2*MOUTH_OPENING_R:.4f}, inner_body_w={JAR_SECT-2*GLASS_WALL:.4f}",
    )

    # ---- lid sits on top of the jar at rest ----
    lid_pos = ctx.part_world_position(lid)
    ctx.check(
        "lid sits at the top of the jar",
        lid_pos is not None and lid_pos[2] >= JAR_TOTAL_H - 0.001,
        details=f"lid origin z={lid_pos[2] if lid_pos else None}, jar top={JAR_TOTAL_H}",
    )

    # ---- lid has skirt overlap into the mouth ----
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        min_overlap=0.020,
        name="lid footprint overlaps jar mouth",
    )

    # ---- dipper holder exists on the lid ----
    holder_vis = lid.get_visual("dipper_holder")
    ctx.check(
        "dipper holder visual exists on the lid",
        holder_vis is not None,
        details="dipper_holder visual not found on lid part",
    )

    # ---- dipper holder is above the lid disk ----
    holder_aabb = ctx.part_element_world_aabb(lid, elem="dipper_holder")
    lid_disk_aabb = ctx.part_element_world_aabb(lid, elem="lid_disk")
    if holder_aabb and lid_disk_aabb:
        ctx.check(
            "dipper holder sits above the lid disk",
            holder_aabb[0][2] >= lid_disk_aabb[1][2] - 0.002,
            details=f"holder_min_z={holder_aabb[0][2]:.4f}, lid_disk_max_z={lid_disk_aabb[1][2]:.4f}",
        )

    # ---- dipper stick exists ----
    stick_vis = lid.get_visual("dipper_stick")
    ctx.check(
        "dipper stick visual exists on the lid",
        stick_vis is not None,
        details="dipper_stick visual not found on lid part",
    )

    # ---- articulation is CONTINUOUS (screw joint) ----
    ctx.check(
        "lid articulation is CONTINUOUS (unlimited screw rotation)",
        screw.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joint type={screw.articulation_type}",
    )

    # ---- screw joint rotates lid about Z axis ----
    ctx.check(
        "screw joint axis is Z (vertical rotation)",
        screw.axis is not None and abs(screw.axis[2]) > 0.99,
        details=f"joint axis={screw.axis}",
    )

    # ---- lid rotates without translating (verify at a rotated pose) ----
    rest_z = ctx.part_world_position(lid)[2]
    with ctx.pose({screw: 3.14159}):
        rotated_pos = ctx.part_world_position(lid)
    ctx.check(
        "lid stays at same height when rotated (screw represented as pure rotation)",
        rotated_pos is not None and abs(rotated_pos[2] - rest_z) < 0.001,
        details=f"rest_z={rest_z:.4f}, rotated_z={rotated_pos[2] if rotated_pos else None}",
    )

    # ---- materials are distinct ----
    lid_mat = lid.get_visual("lid_disk").material
    body_mat = body.get_visual("glass_jar").material
    ctx.check(
        "lid is wood and body is glass (distinct materials)",
        lid_mat is not None
        and body_mat is not None
        and getattr(lid_mat, "name", None) == "natural_wood"
        and getattr(body_mat, "name", None) == "clear_glass",
        details=f"lid_mat={getattr(lid_mat, 'name', None)}, body_mat={getattr(body_mat, 'name', None)}",
    )

    return ctx.report()


object_model = build_object_model()
