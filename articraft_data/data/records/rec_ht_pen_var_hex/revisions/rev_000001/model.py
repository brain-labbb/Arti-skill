from __future__ import annotations

# Realistic articulated retractable (click-action) ballpoint pen — hex barrel variant.
#
# Articraft brief:
# - Object: a silver metallic click-action ballpoint pen with hexagonal barrel,
#   ~0.146 m long, hex across-flats ~0.0112 m. Modeled upright along +Z: writing
#   tip at the bottom (-Z), push button at the top (+Z).
# - Root/support: the barrel (hex body + round conical nose + tip) is the fixed
#   root that carries everything. The pocket clip is rigidly mounted to the
#   upper collar. Six grip strips are inline visuals on the barrel faces.
# - Parts:
#     * barrel  - root: hexagonal prism body, round conical nose cone, writing
#                  ball-tip, and the fixed upper hex collar.
#     * clip    - spring-steel pocket clip with oval cut-out window, fixed to
#                  the upper collar (rigid mount, reads as one piece).
#     * plunger - push-button knob + plunger shaft that slides into the barrel
#                  top. This is the click mechanism.
# - Articulations:
#     * barrel_to_plunger, PRISMATIC, axis -Z (button travels DOWN into the
#       barrel when pressed). Positive q presses the button down. Travel ~7 mm.
#     * barrel_to_clip, FIXED. The clip is structural trim.
# - Visible geometry: hexagonal prism body with six flat faces, round conical
#   nose, tiny ball tip, knurled push-button cap, flat spring clip with oval
#   window, six dark rubber grip strips on barrel faces.
# - Support/fit: clip mounts on the collar; plunger shaft is seated inside the
#   barrel bore (intentional nested fit -> scoped allow_overlap).
# - Intentional overlaps: plunger shaft inside the barrel-top bore proxy.
# - Tests: barrel/clip/plunger present; hex barrel XY extent proves six flat
#   faces; clip window present; plunger is PRISMATIC along Z; pressing button
#   moves it down; plunger stays captured at both stroke ends; nose tip is
#   the lowest geometry; grip strips exist and sit on the barrel.

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# --- Key dimensions (meters) ---------------------------------------------------
BARREL_TOP_Z = 0.118          # top of the fixed barrel body (collar top)
COLLAR_TOP_Z = 0.118
COLLAR_BOT_Z = 0.100          # collar / grip band lower edge
BODY_TOP_Z = 0.100            # main tapered body upper edge
NOSE_TOP_Z = 0.030            # where the body meets the nose cone
NOSE_TIP_Z = 0.006            # base of the metal writing ball seat
TIP_END_Z = 0.000             # ball tip lowest point

R_COLLAR = 0.0062             # across-flats / 2 of the upper hex collar
R_BODY_TOP = 0.0058           # across-flats / 2 of the upper body hex
R_BODY_MID = 0.0056           # across-flats / 2 of the mid body hex
R_NOSE_TOP = 0.0050           # cone radius where nose meets hex body
R_TIP_SEAT = 0.0011           # tip seat radius
R_BALL = 0.0006               # writing ball radius

BORE_R = 0.0040               # inner bore the plunger slides in
PLUNGER_TRAVEL = 0.007         # click stroke length

CHROME = (0.83, 0.84, 0.86, 1.0)
ALUM = (0.74, 0.75, 0.77, 1.0)
TIP_METAL = (0.62, 0.63, 0.66, 1.0)
GRIP_RUBBER = (0.15, 0.15, 0.17, 1.0)  # dark rubber grip accents

BORE_BOTTOM_Z = COLLAR_TOP_Z - 0.055

# --- Shared geometry helpers ---------------------------------------------------

def _hex_outer_radius(across_flats: float) -> float:
    """Circumscribed radius of a regular hexagon from its across-flats distance."""
    return across_flats / (2.0 * math.cos(math.pi / 6.0))


def _hex_prism(across_flats: float, height: float, z_base: float) -> cq.Workplane:
    """Build a regular hexagonal prism centered on the Z axis."""
    r = _hex_outer_radius(across_flats)
    pts = []
    for i in range(6):
        angle = math.pi / 6.0 + i * math.pi / 3.0
        pts.append((r * math.cos(angle), r * math.sin(angle)))
    return (
        cq.Workplane("XY")
        .workplane(offset=z_base)
        .polyline(pts)
        .close()
        .extrude(height)
    )


def _build_face_strip(
    across_flats: float,
    z_lo: float,
    z_hi: float,
    width: float = 0.0025,
    proud: float = 0.0005,
) -> cq.Workplane:
    """Thin grip strip sitting on one hex face (canonical +X face).

    The strip is partially embedded in the barrel face for visual connectivity:
    about 30% is inside, 70% is proud of the face surface.
    """
    d = across_flats / 2.0
    h = z_hi - z_lo
    # Place so ~30% of the thickness is embedded in the barrel face
    strip_center_x = d + proud * 0.3
    return (
        cq.Workplane("XY")
        .workplane(offset=z_lo)
        .center(strip_center_x, 0.0)
        .rect(proud, width)
        .extrude(h)
    )


# --- Barrel geometry (hex prism body + round conical nose) ---------------------

def _build_barrel_geometry() -> cq.Workplane:
    """Hexagonal prism barrel body + round conical nose + bore hole.

    The body is a hex prism (six flat faces) from NOSE_TOP_Z to COLLAR_TOP_Z
    with a small step at the collar. The nose below NOSE_TOP_Z is a round
    conical solid of revolution. A cylindrical bore at the top accepts the
    plunger shaft.
    """
    # Main body hex from nose junction to collar seam
    body = _hex_prism(2.0 * R_BODY_MID, COLLAR_BOT_Z - NOSE_TOP_Z, NOSE_TOP_Z)

    # Collar hex (slightly wider, the grip band area)
    collar = _hex_prism(2.0 * R_COLLAR, COLLAR_TOP_Z - COLLAR_BOT_Z, COLLAR_BOT_Z)
    body = body.union(collar)

    # Round conical nose — extends into the hex body for a clean boolean union.
    # Below NOSE_TOP_Z only the cone is visible; above NOSE_TOP_Z the hex body
    # is wider and dominates.
    nose = (
        cq.Workplane("XZ")
        .moveTo(0.0, TIP_END_Z)
        .lineTo(R_BALL, NOSE_TIP_Z * 0.45)
        .lineTo(R_TIP_SEAT, NOSE_TIP_Z)
        .lineTo(R_NOSE_TOP, NOSE_TOP_Z)
        .lineTo(R_NOSE_TOP, BODY_TOP_Z)
        .lineTo(0.0, BODY_TOP_Z)
        .close()
        .revolve(360, (0, 0), (0, 1))
    )
    body = body.union(nose)

    # Cut bore hole from collar top down into the body
    bore = (
        cq.Workplane("XY")
        .workplane(offset=BORE_BOTTOM_Z)
        .circle(BORE_R)
        .extrude(COLLAR_TOP_Z - BORE_BOTTOM_Z)
    )
    body = body.cut(bore)

    return body


# --- Clip geometry (identical to parent) ---------------------------------------

def _build_clip_shape() -> cq.Workplane:
    """Spring-steel pocket clip: a tab on the collar curving down the barrel,
    with an oval cut-out window, modeled as one connected piece."""
    clip_len = 0.052
    clip_w = 0.0072
    clip_t = 0.0009
    top_z = COLLAR_TOP_Z - 0.002
    blade = (
        cq.Workplane("YZ")
        .workplane(offset=R_COLLAR + clip_t / 2.0 + 0.0006)
        .center(0.0, top_z - clip_len / 2.0)
        .rect(clip_w, clip_len)
        .extrude(clip_t)
    )
    win_l = 0.020
    win_w = 0.0034
    window = (
        cq.Workplane("YZ")
        .workplane(offset=R_COLLAR + clip_t / 2.0 + 0.0006)
        .center(0.0, top_z - 0.020)
        .ellipse(win_w / 2.0, win_l / 2.0)
        .extrude(clip_t + 0.002)
    )
    blade = blade.cut(window)
    bridge = (
        cq.Workplane("XZ")
        .workplane(offset=clip_w / 2.0)
        .moveTo(R_COLLAR - 0.0008, top_z + 0.0008)
        .lineTo(R_COLLAR + clip_t / 2.0 + 0.0016, top_z + 0.0008)
        .lineTo(R_COLLAR + clip_t / 2.0 + 0.0016, top_z - 0.002)
        .lineTo(R_COLLAR - 0.0008, top_z - 0.002)
        .close()
        .extrude(-clip_w)
    )
    foot = (
        cq.Workplane("YZ")
        .workplane(offset=R_COLLAR + 0.0006)
        .center(0.0, top_z - clip_len + 0.004)
        .circle(0.0028)
        .extrude(0.0016)
    )
    clip = blade.union(bridge).union(foot)
    return clip


# --- Plunger geometry (identical to parent) ------------------------------------

PLUNGER_SHAFT_LEN = 0.034


def _build_plunger_geometry() -> LatheGeometry:
    """Push-button knob + plunger shaft as ONE connected revolved solid.

    Built in its own local frame with the button crown apex at z=0 (the joint
    origin) and the shaft extending DOWN in -Z so it stays captured in the
    barrel bore.
    """
    knurl_r = 0.0064
    cap_r = 0.0056
    shaft_r = BORE_R

    z_crown = 0.0
    z_cap = -0.0018
    z_collar_top = -0.0040
    z_collar_bot = -0.0090
    z_step = -0.0110
    z_shaft_top = -0.0120
    z_shaft_tip = z_shaft_top - PLUNGER_SHAFT_LEN

    profile = [
        (0.0, z_crown),
        (cap_r * 0.55, z_crown - 0.0006),
        (cap_r, z_cap),
        (knurl_r, z_collar_top),
        (knurl_r, z_collar_bot),
        (cap_r * 0.78, z_step),
        (shaft_r, z_shaft_top),
        (shaft_r, z_shaft_tip),
        (0.0, z_shaft_tip),
    ]
    return LatheGeometry(profile, segments=48, closed=True)


# --- Object model --------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hex_barrel_retractable_pen")
    model.material("pen_alum", rgba=ALUM)
    model.material("pen_chrome", rgba=CHROME)
    model.material("pen_tip", rgba=TIP_METAL)
    model.material("grip_rubber", rgba=GRIP_RUBBER)

    # --- Barrel (root) ---
    barrel = model.part("barrel")
    barrel.visual(
        mesh_from_cadquery(_build_barrel_geometry(), "barrel_body"),
        material="pen_alum",
        name="barrel_body",
    )

    # Six grip strips, one per hex face, emitted via a loop with shared helper
    strip_mesh = mesh_from_cadquery(
        _build_face_strip(
            2.0 * R_BODY_MID,
            NOSE_TOP_Z + 0.008,
            BODY_TOP_Z - 0.008,
        ),
        "face_strip",
    )
    for i in range(6):
        # Face i normal direction is at angle (i+1) * pi/3 around Z
        angle = (i + 1) * math.pi / 3.0
        barrel.visual(
            strip_mesh,
            origin=Origin(rpy=(0.0, 0.0, angle)),
            material="grip_rubber",
            name=f"face_strip_{i}",
        )

    # --- Pocket clip (fixed structural trim on the collar) ---
    clip = model.part("clip")
    clip.visual(
        mesh_from_cadquery(_build_clip_shape(), "clip_blade"),
        material="pen_chrome",
        name="clip_blade",
    )

    # --- Plunger / push button (the click mechanism) ---
    plunger = model.part("plunger")
    plunger.visual(
        mesh_from_geometry(_build_plunger_geometry(), "plunger_button"),
        material="pen_chrome",
        name="plunger_button",
    )

    # Clip is rigidly mounted; it is built already in the barrel frame.
    model.articulation(
        "barrel_to_clip",
        ArticulationType.FIXED,
        parent=barrel,
        child=clip,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    # Plunger slides down into the barrel when pressed.
    # Axis is -Z so positive q presses the button DOWN into the barrel.
    button_rest_z = COLLAR_TOP_Z + 0.010
    model.articulation(
        "barrel_to_plunger",
        ArticulationType.PRISMATIC,
        parent=barrel,
        child=plunger,
        origin=Origin(xyz=(0.0, 0.0, button_rest_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=0.1,
            lower=0.0,
            upper=PLUNGER_TRAVEL,
        ),
    )

    return model


# --- Tests ---------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    barrel = object_model.get_part("barrel")
    clip = object_model.get_part("clip")
    plunger = object_model.get_part("plunger")
    plunge_joint = object_model.get_articulation("barrel_to_plunger")
    clip_joint = object_model.get_articulation("barrel_to_clip")

    # The plunger shaft is intentionally nested inside the barrel bore proxy.
    ctx.allow_overlap(
        barrel,
        plunger,
        reason="The plunger shaft is captured inside the barrel bore; this is the click mechanism's sliding fit.",
    )

    # --- Joint type / axis claims ---
    ctx.check(
        "plunger is prismatic",
        plunge_joint.articulation_type == ArticulationType.PRISMATIC,
        details=f"got {plunge_joint.articulation_type}",
    )
    ax = tuple(round(a, 6) for a in plunge_joint.axis)
    ctx.check(
        "plunger axis is along Z",
        abs(ax[0]) < 1e-6 and abs(ax[1]) < 1e-6 and abs(ax[2]) > 0.5,
        details=f"axis={ax}",
    )
    ctx.check(
        "clip joint is fixed",
        clip_joint.articulation_type == ArticulationType.FIXED,
        details=f"got {clip_joint.articulation_type}",
    )

    # --- Hex barrel geometry: XY extent proves flat faces ---
    barrel_aabb = ctx.part_world_aabb(barrel)
    if barrel_aabb is not None:
        dx = barrel_aabb[1][0] - barrel_aabb[0][0]
        dy = barrel_aabb[1][1] - barrel_aabb[0][1]
        # Hex orientation: flat faces perpendicular to X (dx = across_flats),
        # corners along Y (dy = across_flats * 2/sqrt(3) ≈ 1.155 * across_flats).
        # A circular barrel would give dx ≈ dy. The hex proves by dy > dx.
        across_flats_collar = 2.0 * R_COLLAR
        across_corners_collar = across_flats_collar / math.cos(math.pi / 6.0)
        ctx.check(
            "barrel XY is non-circular (proves hex faces)",
            dy > dx * 1.05,
            details=f"dx={dx:.6f}, dy={dy:.6f}, ratio={dy/max(dx,1e-9):.3f}",
        )
        ctx.check(
            "barrel XY extent consistent with hex across-corners",
            max(dx, dy) < across_corners_collar * 1.15,
            details=f"dx={dx:.6f}, dy={dy:.6f}, across_corners={across_corners_collar:.6f}",
        )

    # --- Clip present and mounted ---
    clip_aabb = ctx.part_world_aabb(clip)
    if clip_aabb is not None and barrel_aabb is not None:
        clip_len = clip_aabb[1][2] - clip_aabb[0][2]
        ctx.check(
            "clip is a long blade",
            clip_len > 0.035,
            details=f"clip z-length={clip_len}",
        )
        ctx.check(
            "clip stands off on +X side",
            clip_aabb[1][0] > barrel_aabb[1][0] - 0.001,
            details=f"clip max x={clip_aabb[1][0]}, barrel max x={barrel_aabb[1][0]}",
        )

    ctx.expect_contact(
        clip,
        barrel,
        contact_tol=0.0015,
        name="clip is anchored to the barrel collar",
    )

    # --- Writing tip is the lowest geometry ---
    if barrel_aabb is not None:
        ctx.check(
            "writing tip reaches the bottom",
            barrel_aabb[0][2] <= 0.001,
            details=f"barrel min z={barrel_aabb[0][2]}",
        )

    # --- Grip strips present on barrel faces ---
    for i in range(6):
        strip_name = f"face_strip_{i}"
        strip_vis = barrel.get_visual(strip_name)
        ctx.check(
            f"grip strip {strip_name} exists",
            strip_vis is not None,
            details=f"visual {strip_name} not found on barrel",
        )

    # --- Click mechanism actuates correctly ---
    rest_pos = ctx.part_world_position(plunger)
    rest_aabb = ctx.part_world_aabb(plunger)
    with ctx.pose({plunge_joint: PLUNGER_TRAVEL}):
        pressed_pos = ctx.part_world_position(plunger)
        ctx.expect_overlap(
            plunger,
            barrel,
            axes="z",
            min_overlap=0.010,
            name="plunger stays captured in barrel when pressed",
        )
    ctx.check(
        "pressing the button moves it down",
        rest_pos is not None
        and pressed_pos is not None
        and pressed_pos[2] < rest_pos[2] - 0.005,
        details=f"rest={rest_pos}, pressed={pressed_pos}",
    )

    # Plunger button stays above the barrel collar at rest (button is exposed).
    if rest_aabb is not None and barrel_aabb is not None:
        ctx.check(
            "button protrudes above barrel at rest",
            rest_aabb[1][2] > barrel_aabb[1][2],
            details=f"button top={rest_aabb[1][2]}, barrel top={barrel_aabb[1][2]}",
        )

    # Plunger stays captured at rest too (retained insertion).
    ctx.expect_overlap(
        plunger,
        barrel,
        axes="z",
        min_overlap=0.010,
        name="plunger captured in barrel at rest",
    )

    return ctx.report()


object_model = build_object_model()
