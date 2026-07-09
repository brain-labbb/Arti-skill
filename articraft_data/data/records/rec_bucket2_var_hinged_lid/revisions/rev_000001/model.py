from __future__ import annotations

# Small wooden keg / barrel with a HINGED flip-up lid.
#
# Real object (from the reference image):
#   - A short, wide wooden keg: a slightly bulged barrel body built from
#     vertical staves, warm wood-grain coloring.
#   - Two dark metal hoop bands wrap the body (an upper band and a lower band).
#   - The mouth has a recessed inner rim/ledge that the lid seats into.
#   - A circular wooden lid sits on top, with a dark/blue arched grip handle.
#
# Coordinate convention:
#   - up is +Z. The barrel base footprint sits at z = 0.
#   - the body is a surface of revolution about the world Z axis, so it has no
#     meaningful left/right/front frame (link names avoid side labels).
#   - "rear" for the hinge is defined as the -X side (arbitrary for a
#     rotationally symmetric barrel).
#
# Root structure: the staved barrel body (barrel_body) is the root. The two
# metal hoops and the recessed inner rim are FIXED to the body. The wooden lid
# is the one moving part; the grip handle is fixed to the lid.
#
# PRIMARY ARTICULATION:
#   - body_to_lid : REVOLUTE, axis (0, -1, 0) — a horizontal Y-axis hinge
#     anchored at the rear (-X) edge of the mouth rim, at the contact line
#     where the lid edge meets the rim top. At q=0 the lid is seated flush on
#     the rim (closed). Positive q swings the lid upward and back; the upper
#     limit (~90°) leaves the lid standing nearly vertical, clear of the mouth.
#   - lid_to_handle : FIXED. The arched grip handle is fastened on top of the
#     lid and rides with it as the lid swings.

import cadquery as cq
import numpy as np

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

# ---- key dimensions (meters) -------------------------------------------------
BODY_H = 0.360            # barrel body height (base at z=0, mouth at z=BODY_H)
R_END = 0.150            # outer radius at the top/bottom rims
R_MID = 0.186            # outer radius at the mid-bulge (slightly bulged)
WALL = 0.016            # stave wall thickness (body is hollow)
FLOOR_T = 0.020         # solid wooden base thickness (closes the bottom)

N_STAVES = 16            # number of suggested vertical staves
GROOVE_W = 0.0045        # vertical seam-groove width between staves
GROOVE_DEPTH = 0.006     # groove depth carved into the outer surface

UPPER_HOOP_Z = 0.262     # center height of the upper metal hoop
LOWER_HOOP_Z = 0.072     # center height of the lower metal hoop
HOOP_HALF_H = 0.022      # half-height of a hoop band
HOOP_PROUD = 0.007       # how far the hoop stands proud of the wood surface

RIM_LEDGE_DZ = 0.020     # how far the inner seating ledge sits below the mouth
RIM_LEDGE_W = 0.014      # radial width of the inner seating ledge

LID_FLANGE_T = 0.022     # thickness of the lid's wooden disk (flange)
LID_PLUG_DROP = 0.018    # how far the lid plug drops down into the mouth
LID_OPEN_ANGLE = 1.50    # max hinge angle in radians (~86°, nearly 90°)

HANDLE_R = 0.0095        # tube radius of the arched grip handle
HANDLE_SPAN = 0.150      # span of the handle across the lid
HANDLE_RISE = 0.052      # how high the handle arch rises above the lid top

# Derived: lid flange radius and hinge geometry
R_MOUTH_OUTER = R_END    # _outer_radius(BODY_H) == R_END for the parabolic profile
R_FLANGE = R_MOUTH_OUTER - 0.003   # lid flange radius (slightly smaller than mouth)
HINGE_X = -R_FLANGE      # hinge origin X: rear (-X) edge of the mouth rim


def _outer_radius(z: float) -> float:
    """Outer barrel radius at height z: symmetric parabolic mid-bulge."""
    t = z / BODY_H
    return R_END + (R_MID - R_END) * (1.0 - (2.0 * t - 1.0) ** 2)


def _build_body_mesh():
    """Bulged barrel shell with a SOLID bottom and vertical stave grooves.

    The revolved profile runs up the outer wall, back down the inner cavity
    wall only as far as the floor level (z = FLOOR_T), then in across the cavity
    floor to the central axis and down to z = 0. Closing the loop along the
    bottom yields a solid wooden base of thickness FLOOR_T, with the cavity open
    at the mouth above it (so the barrel is hollow but no longer bottomless)."""
    zs_outer = np.linspace(0.0, BODY_H, 13)
    outer = [(_outer_radius(z), z) for z in zs_outer]
    # Inner cavity wall: from the mouth down only to the floor level.
    zs_inner = np.linspace(BODY_H, FLOOR_T, 12)
    inner = [(_outer_radius(z) - WALL, z) for z in zs_inner]
    # Close in across the cavity floor to the axis, then down the centerline.
    # The implicit close() segment back to outer[0] forms the outer bottom face.
    pts = outer + inner + [(0.0, FLOOR_T), (0.0, 0.0)]

    wp = cq.Workplane("XZ").moveTo(*pts[0])
    for p in pts[1:]:
        wp = wp.lineTo(*p)
    # Revolve the (radius, height) loop 360 deg about the world Z axis.
    shell = wp.close().revolve(360, (0, 0, 0), (0, 1, 0))

    # Carve thin vertical seam grooves to suggest individual staves.
    cutters = None
    for i in range(N_STAVES):
        ang = 360.0 / N_STAVES * i
        slot = (
            cq.Workplane("XY")
            .box(GROOVE_DEPTH * 2.0, GROOVE_W, BODY_H + 0.02)
            .translate((R_MID, 0.0, BODY_H / 2.0))
            .rotate((0, 0, 0), (0, 0, 1), ang)
        )
        cutters = slot if cutters is None else cutters.union(slot)
    body = shell.cut(cutters)
    return mesh_from_cadquery(body, "barrel_body", tolerance=0.0015)


def _build_rim_mesh():
    """Recessed inner seating ledge just below the mouth that the lid rests on."""
    r_mouth_inner = _outer_radius(BODY_H) - WALL          # inner wall radius at mouth
    z_top = BODY_H
    z_bot = BODY_H - RIM_LEDGE_DZ
    r_out = r_mouth_inner                                  # flush with inner wall
    r_in = r_mouth_inner - RIM_LEDGE_W                     # ledge inner edge
    # A short inward ring forming a horizontal shelf at the top of the mouth.
    ring = (
        cq.Workplane("XZ")
        .moveTo(r_in, z_bot)
        .lineTo(r_out, z_bot)
        .lineTo(r_out, z_top)
        .lineTo(r_in, z_top)
        .close()
        .revolve(360, (0, 0, 0), (0, 1, 0))
    )
    return mesh_from_cadquery(ring, "rim_ledge", tolerance=0.0015)


def _build_hoop_mesh(name: str, z_center: float):
    """A dark metal hoop band wrapping the body, standing slightly proud."""
    r_body = _outer_radius(z_center)
    r_in = r_body - 0.001                                  # bite into the wood
    r_out = r_body + HOOP_PROUD
    z_bot = z_center - HOOP_HALF_H
    z_top = z_center + HOOP_HALF_H
    band = (
        cq.Workplane("XZ")
        .moveTo(r_in, z_bot)
        .lineTo(r_out, z_bot)
        .lineTo(r_out, z_top)
        .lineTo(r_in, z_top)
        .close()
        .revolve(360, (0, 0, 0), (0, 1, 0))
    )
    return mesh_from_cadquery(band, name, tolerance=0.0015)


def _build_lid_mesh():
    """Wooden lid: a flat disk flange that rests on the mouth, plus a downward
    plug that drops into the recessed rim.

    Authored in the lid-local (hinge) frame:
      - Origin at the hinge point (rear edge of the lid).
      - Seating plane (flange bottom) at local z=0.
      - Lid disk center offset to (R_FLANGE, 0, 0) so the hinge edge is at the
        origin and the disk extends in the +X direction toward the barrel center.
    """
    r_plug = _outer_radius(BODY_H) - WALL - 0.004          # drops into the mouth

    # Flange disk: bottom at z=0, top at z=LID_FLANGE_T, centered at (R_FLANGE, 0).
    flange = (
        cq.Workplane("XY")
        .center(R_FLANGE, 0.0)
        .circle(R_FLANGE)
        .extrude(LID_FLANGE_T)
    )
    # Downward plug ring that nests into the mouth recess (engages the rim).
    plug = (
        cq.Workplane("XY")
        .center(R_FLANGE, 0.0)
        .circle(r_plug)
        .extrude(-LID_PLUG_DROP)
    )
    lid = flange.union(plug)
    # Round the top outer edge a touch so the lid reads like a turned wood lid.
    lid = lid.edges(">Z").fillet(0.006)
    return mesh_from_cadquery(lid, "barrel_lid", tolerance=0.0015)


def _build_handle_mesh():
    """Squared-arch grip handle: two short feet rising from the lid top to a
    horizontal crossbar.

    Authored in lid-local (hinge) coordinates: the lid center is at
    (R_FLANGE, 0, LID_FLANGE_T), so the handle feet and crossbar are offset
    to straddle that center. The feet seat into the lid top surface so the
    handle reads as fastened, not floating."""
    z0 = LID_FLANGE_T                                      # lid top surface
    half = HANDLE_SPAN / 2.0
    foot_z = z0 - 0.004                                    # feet embed into wood
    top_z = z0 + HANDLE_RISE                               # crossbar center height
    leg_h = top_z - foot_z
    cx = R_FLANGE                                          # lid center X in local frame

    leg_left = (
        cq.Workplane("XY").circle(HANDLE_R).extrude(leg_h).translate((cx - half, 0.0, foot_z))
    )
    leg_right = (
        cq.Workplane("XY").circle(HANDLE_R).extrude(leg_h).translate((cx + half, 0.0, foot_z))
    )
    crossbar = (
        cq.Workplane("YZ").circle(HANDLE_R).extrude(HANDLE_SPAN).translate((cx - half, 0.0, top_z))
    )
    handle = leg_left.union(leg_right).union(crossbar)
    return mesh_from_cadquery(handle, "lid_handle", tolerance=0.0015)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wooden_keg_barrel")

    model.material("oak_wood", rgba=(0.74, 0.52, 0.32, 1.0))
    model.material("lid_wood", rgba=(0.80, 0.60, 0.39, 1.0))
    model.material("dark_metal", rgba=(0.16, 0.17, 0.19, 1.0))
    model.material("handle_blue", rgba=(0.20, 0.30, 0.45, 1.0))

    # --- staved barrel body (root) -------------------------------------------
    body = model.part("barrel_body")
    body.visual(_build_body_mesh(), material="oak_wood")
    body.visual(_build_rim_mesh(), material="oak_wood")
    body.inertial = Inertial.from_geometry(
        Cylinder(radius=R_MID, length=BODY_H),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.0, BODY_H / 2.0)),
    )

    # --- two metal hoop bands (fixed to the body) ----------------------------
    upper_hoop = model.part("upper_hoop")
    upper_hoop.visual(_build_hoop_mesh("upper_hoop", UPPER_HOOP_Z), material="dark_metal")
    upper_hoop.inertial = Inertial.from_geometry(
        Cylinder(radius=_outer_radius(UPPER_HOOP_Z) + HOOP_PROUD, length=2 * HOOP_HALF_H),
        mass=0.4,
        origin=Origin(xyz=(0.0, 0.0, UPPER_HOOP_Z)),
    )

    lower_hoop = model.part("lower_hoop")
    lower_hoop.visual(_build_hoop_mesh("lower_hoop", LOWER_HOOP_Z), material="dark_metal")
    lower_hoop.inertial = Inertial.from_geometry(
        Cylinder(radius=_outer_radius(LOWER_HOOP_Z) + HOOP_PROUD, length=2 * HOOP_HALF_H),
        mass=0.4,
        origin=Origin(xyz=(0.0, 0.0, LOWER_HOOP_Z)),
    )

    # --- wooden lid (the moving part) ----------------------------------------
    # The lid mesh is authored in the hinge frame: origin at the rear (-X) edge,
    # seating plane at local z=0, lid disk extending in +X toward barrel center.
    # The joint frame is placed at the hinge contact line on the rim top.
    lid = model.part("barrel_lid")
    lid.visual(_build_lid_mesh(), material="lid_wood")
    lid.inertial = Inertial.from_geometry(
        Cylinder(radius=R_FLANGE, length=LID_FLANGE_T),
        mass=0.7,
        origin=Origin(xyz=(R_FLANGE, 0.0, LID_FLANGE_T / 2.0)),
    )

    # --- arched grip handle (fixed on top of the lid) ------------------------
    handle = model.part("lid_handle")
    handle.visual(_build_handle_mesh(), material="handle_blue")
    handle.inertial = Inertial.from_geometry(
        Cylinder(radius=HANDLE_R, length=HANDLE_SPAN),
        mass=0.08,
        origin=Origin(xyz=(R_FLANGE, 0.0, LID_FLANGE_T + HANDLE_RISE / 2.0)),
    )

    # --- articulations -------------------------------------------------------
    model.articulation(
        "body_to_upper_hoop",
        ArticulationType.FIXED,
        parent=body,
        child=upper_hoop,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    model.articulation(
        "body_to_lower_hoop",
        ArticulationType.FIXED,
        parent=body,
        child=lower_hoop,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # PRIMARY: hinged lid — REVOLUTE about a horizontal Y-axis at the rear
    # (-X) edge of the mouth rim. At q=0 the lid sits flush on the rim (closed).
    # axis = (0, -1, 0): positive q rotates the lid's +X-extending disk upward
    # toward +Z, swinging the far edge above the mouth.
    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, BODY_H)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=LID_OPEN_ANGLE, effort=5.0, velocity=1.5),
    )

    # Handle is fastened on the lid; the lid frame at q=0 coincides with the
    # joint frame at the hinge, so the handle local frame mounts there too.
    model.articulation(
        "lid_to_handle",
        ArticulationType.FIXED,
        parent=lid,
        child=handle,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("barrel_body")
    upper_hoop = object_model.get_part("upper_hoop")
    lower_hoop = object_model.get_part("lower_hoop")
    lid = object_model.get_part("barrel_lid")
    handle = object_model.get_part("lid_handle")
    lid_joint = object_model.get_articulation("body_to_lid")

    # --- PRIMARY articulation: revolute, horizontal Y-axis hinge ------------
    ctx.check(
        "lid joint is revolute",
        lid_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={lid_joint.articulation_type}",
    )
    ax = tuple(lid_joint.axis)
    ctx.check(
        "lid joint axis is horizontal (Y-dominant)",
        abs(ax[0]) < 1e-9 and abs(ax[2]) < 1e-9 and abs(ax[1]) > 0.5,
        details=f"axis={ax}",
    )
    lim = lid_joint.motion_limits
    ctx.check(
        "lid hinge range starts at 0 and opens to ~90°",
        lim is not None and lim.lower == 0.0 and lim.upper > 1.0,
        details=f"lower={None if lim is None else lim.lower}, upper={None if lim is None else lim.upper}",
    )

    # --- hinge origin at rear edge of mouth rim ------------------------------
    jnt_origin = lid_joint.origin.xyz
    ctx.check(
        "hinge origin is at mouth rim height (z=BODY_H)",
        abs(jnt_origin[2] - BODY_H) < 1e-6,
        details=f"joint origin z={jnt_origin[2]}",
    )
    ctx.check(
        "hinge origin is at the rear (-X) edge of the rim",
        abs(jnt_origin[0] - HINGE_X) < 0.005 and abs(jnt_origin[1]) < 0.005,
        details=f"joint origin=({jnt_origin[0]:.4f}, {jnt_origin[1]:.4f})",
    )

    # --- base sits at z = 0 --------------------------------------------------
    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "barrel base sits at z=0",
        body_aabb is not None and abs(body_aabb[0][2]) < 1e-3,
        details=f"body z-min={None if body_aabb is None else body_aabb[0][2]}",
    )

    # --- body is hollow (thin shell, open mouth) ----------------------------
    inner_r = _outer_radius(BODY_H) - WALL
    ctx.check(
        "body wall is a thin hollow shell",
        WALL < 0.5 * _outer_radius(BODY_H) and inner_r > 0.10,
        details=f"wall={WALL}, inner_r={inner_r:.3f}",
    )

    # --- materials present (wood + dark metal + blue handle) -----------------
    mat_names = {m.name for m in object_model.materials}
    ctx.check(
        "wood, dark-metal, and blue-handle materials present",
        {"oak_wood", "lid_wood", "dark_metal", "handle_blue"}.issubset(mat_names),
        details=f"materials={sorted(mat_names)}",
    )

    # --- both hoops present and wrapping the body ---------------------------
    for hoop, zc, nm in (
        (upper_hoop, UPPER_HOOP_Z, "upper"),
        (lower_hoop, LOWER_HOOP_Z, "lower"),
    ):
        h_aabb = ctx.part_world_aabb(hoop)
        ctx.check(
            f"{nm} hoop present at expected height",
            h_aabb is not None and abs((h_aabb[0][2] + h_aabb[1][2]) / 2.0 - zc) < 0.01,
            details=f"{nm} hoop aabb={h_aabb}",
        )
        # the hoop must overlap the body in z (it wraps around it)
        ctx.expect_overlap(
            hoop,
            body,
            axes="z",
            min_overlap=0.01,
            name=f"{nm} hoop wraps the body (z overlap)",
        )

    # --- lid seats flush on the rim when closed (no floating gap) -----------
    # At rest pose (q=0) the lid flange bottom touches the body mouth top.
    # The plug intentionally drops into the mouth recess (negative gap allowed).
    ctx.expect_gap(
        lid,
        body,
        axis="z",
        max_gap=0.001,
        max_penetration=LID_PLUG_DROP + 0.001,
        name="closed lid seats flush on the rim (no gap)",
    )
    # The lid plug intentionally nests into the recessed mouth rim to seat.
    ctx.allow_overlap(
        lid,
        body,
        reason="The lid plug intentionally nests into the recessed mouth rim to seat the lid.",
    )
    # Each metal hoop bites slightly into the wood (real coopered hoop fit).
    ctx.allow_overlap(
        upper_hoop,
        body,
        reason="The upper metal hoop intentionally embeds slightly into the wood staves where it wraps the body.",
    )
    ctx.allow_overlap(
        lower_hoop,
        body,
        reason="The lower metal hoop intentionally embeds slightly into the wood staves where it wraps the body.",
    )
    # The handle feet intentionally seat into the lid top wood.
    ctx.allow_overlap(
        handle,
        lid,
        reason="The grip-handle feet intentionally embed into the lid top so the handle reads as fastened, not floating.",
    )

    # --- handle sits on top of the lid (not floating) -----------------------
    ctx.expect_contact(
        handle,
        lid,
        contact_tol=0.006,
        name="grip handle is fastened to the lid top",
    )

    # --- lid swings open: far edge lifts above the mouth --------------------
    rest_pos = ctx.part_world_position(lid)
    with ctx.pose({lid_joint: LID_OPEN_ANGLE}):
        open_pos = ctx.part_world_position(lid)
        open_lid_aabb = ctx.part_world_aabb(lid)

    # At the open pose the lid has swung upward; its z-max should be well
    # above the mouth (BODY_H) since the far edge has lifted high.
    ctx.check(
        "open lid far edge lifts above the mouth",
        open_lid_aabb is not None and open_lid_aabb[1][2] > BODY_H + 0.10,
        details=f"open lid z-max={None if open_lid_aabb is None else open_lid_aabb[1][2]}, mouth z={BODY_H}",
    )

    # The lid part origin stays at the hinge (it shouldn't translate).
    ctx.check(
        "lid hinge does not translate (origin fixed at rim)",
        rest_pos is not None
        and open_pos is not None
        and abs(open_pos[0] - rest_pos[0]) < 1e-6
        and abs(open_pos[1] - rest_pos[1]) < 1e-6
        and abs(open_pos[2] - rest_pos[2]) < 1e-6,
        details=f"rest={rest_pos}, open={open_pos}",
    )

    # At the open pose, the lid z-min should still be near BODY_H (the hinge
    # edge stays at the rim), confirming a swing rather than a lift.
    ctx.check(
        "open lid hinge edge stays at rim height (swing, not lift)",
        open_lid_aabb is not None and abs(open_lid_aabb[0][2] - BODY_H) < 0.01,
        details=f"open lid z-min={None if open_lid_aabb is None else open_lid_aabb[0][2]}, expected ~{BODY_H}",
    )

    return ctx.report()


object_model = build_object_model()
