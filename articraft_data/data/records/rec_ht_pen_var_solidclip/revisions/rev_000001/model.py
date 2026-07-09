from __future__ import annotations

# Realistic articulated retractable (click-action) ballpoint pen.
#
# Articraft brief:
# - Object: a silver metallic click-action ballpoint pen, ~0.146 m long, barrel
#   diameter ~0.011 m. Modeled upright along +Z: writing tip at the bottom
#   (-Z), push button at the top (+Z).
# - Root/support: the barrel (body + nose cone + tip) is the fixed root that
#   carries everything. The pocket clip is rigidly mounted to the upper barrel.
# - Parts:
#     * barrel  - root: tapered body shell, conical nose cone, writing ball-tip,
#                  and the fixed upper collar that the clip clamps onto.
#     * clip    - solid tapered pocket clip (no cutout window) with a pronounced
#                  curled-over ball-foot at its lower tip, fixed to the upper
#                  collar (rigid mount, reads as one piece).
#     * plunger - the push-button knob + plunger shaft that slides into the top
#                  of the barrel. This is the click mechanism.
# - Articulations:
#     * barrel_to_plunger, PRISMATIC, axis -Z (button travels DOWN into the
#       barrel when pressed). Positive q presses the button down. Travel ~7 mm.
#       The plunger shaft is long enough to stay captured inside the barrel
#       throughout the click stroke.
#     * barrel_to_clip, FIXED. The clip does not actuate; it is structural trim.
# - Visible geometry: lathed tapered metal barrel, conical nose, tiny round
#   ball tip, knurled push-button cap, solid tapered clip with curled-over
#   ball-foot at its lower tip, restrained brushed-aluminium / chrome materials.
# - Support/fit: clip mounts on the collar; plunger shaft is seated inside the
#   barrel bore (intentional nested fit -> scoped allow_overlap).
# - Intentional overlaps: plunger shaft inside the barrel-top bore proxy.
# - Tests: barrel/clip/plunger present; clip is solid and tapered with a
#   pronounced curled-over ball-foot at its lower tip; plunger is PRISMATIC
#   along Z; pressing the button moves it DOWN; the plunger stays captured
#   in the barrel at both stroke ends; nose tip is the lowest geometry.

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
BODY_TOP_Z = 0.100           # main tapered body upper edge
NOSE_TOP_Z = 0.030           # where the body meets the nose cone
NOSE_TIP_Z = 0.006           # base of the metal writing ball seat
TIP_END_Z = 0.000            # ball tip lowest point

R_COLLAR = 0.0062            # radius of the upper collar that holds the clip
R_BODY_TOP = 0.0058          # upper body radius
R_BODY_MID = 0.0056
R_NOSE_TOP = 0.0050          # body radius where nose begins
R_TIP_SEAT = 0.0011         # tip seat radius
R_BALL = 0.0006             # writing ball radius

BORE_R = 0.0040             # inner bore the plunger slides in
PLUNGER_TRAVEL = 0.007       # click stroke length

CHROME = (0.83, 0.84, 0.86, 1.0)
ALUM = (0.74, 0.75, 0.77, 1.0)
TIP_METAL = (0.62, 0.63, 0.66, 1.0)


def _build_barrel_geometry() -> LatheGeometry:
    """Lathed tapered barrel body + conical nose + writing tip, open/hollow top.

    The profile traces the outer silhouette (tip -> collar) then dives down the
    inner bore wall and back to the axis, producing a one-piece revolve that is
    hollow at the top where the plunger slides in. Profile points are (radius, z)
    and the lathe axis is +Z.
    """
    bore_bottom_z = COLLAR_TOP_Z - 0.055
    profile = [
        (0.0, TIP_END_Z),                 # tip apex on axis (lowest point)
        (R_BALL, NOSE_TIP_Z * 0.45),      # rounded ball
        (R_TIP_SEAT, NOSE_TIP_Z),         # tip seat shoulder
        (R_NOSE_TOP, NOSE_TOP_Z),         # top of the conical nose cone
        (R_BODY_MID, 0.060),              # mid body
        (R_BODY_TOP, BODY_TOP_Z),         # upper body
        (R_BODY_TOP, COLLAR_BOT_Z),       # body/collar seam
        (R_COLLAR, COLLAR_BOT_Z + 0.001), # collar step out (grip band)
        (R_COLLAR, COLLAR_TOP_Z),         # collar top (outer lip)
        (BORE_R, COLLAR_TOP_Z),           # inner lip of the open bore
        (BORE_R, bore_bottom_z),          # down the bore wall
        (0.0, bore_bottom_z),             # close across to axis (bore floor)
    ]
    return LatheGeometry(profile, segments=64, closed=True)


def _build_clip_shape() -> cq.Workplane:
    """Solid tapered pocket clip with a pronounced curled-over ball-foot.

    No window cutout. The blade is a solid tapered strip that is wider at the
    collar end and narrows toward its lower tip. The lower tip sweeps outward
    (away from the barrel) in a curl arc and terminates in a pronounced ball.
    The clip is anchored to the collar via a bridge at the top.
    """
    import math

    clip_len = 0.050          # straight blade length (not counting curl)
    top_w = 0.0078            # blade width at collar end (wider)
    bot_w = 0.0042            # blade width at bottom end (narrower, tapered)
    clip_t = 0.0010           # blade thickness (radial direction)
    top_z = COLLAR_TOP_Z - 0.002

    x_base = R_COLLAR + 0.0006           # inner face of clip (near barrel surface)
    x_outer = x_base + clip_t            # outer face
    x_mid = (x_base + x_outer) / 2.0    # blade center in X

    blade_bottom_z = top_z - clip_len

    # --- Solid tapered blade (no window cutout) ---
    blade = (
        cq.Workplane("YZ")
        .workplane(offset=x_base)
        .moveTo(-top_w / 2.0, top_z)
        .lineTo(top_w / 2.0, top_z)
        .lineTo(bot_w / 2.0, blade_bottom_z)
        .lineTo(-bot_w / 2.0, blade_bottom_z)
        .close()
        .extrude(clip_t)
    )

    # --- Bridge anchor at top (connects blade to the collar wall) ---
    bridge = (
        cq.Workplane("XZ")
        .workplane(offset=top_w / 2.0)
        .moveTo(R_COLLAR - 0.0008, top_z + 0.0008)
        .lineTo(x_outer + 0.0002, top_z + 0.0008)
        .lineTo(x_outer + 0.0002, top_z - 0.002)
        .lineTo(R_COLLAR - 0.0008, top_z - 0.002)
        .close()
        .extrude(-top_w)
    )

    # --- Curl-over section ---
    # The lower tip of the blade sweeps outward (+X) in an arc and terminates
    # in a pronounced ball-foot. The arc lies in the XZ plane (Y=0).
    curl_r = 0.0055           # radius of the curl arc path
    tube_r = bot_w / 2.0      # cross-section radius of the curl tube
    curl_angle_deg = 140.0    # how far the curl wraps

    # Arc center sits below the blade bottom, on the blade centerline in X.
    arc_cx = x_mid
    arc_cz = blade_bottom_z - curl_r

    # Start angle: 90° (top of arc = blade bottom), tangent goes +X (outward).
    start_angle = math.radians(90.0)
    end_angle = math.radians(90.0 - curl_angle_deg)  # sweeps clockwise

    start_x = arc_cx + curl_r * math.cos(start_angle)
    start_z = arc_cz + curl_r * math.sin(start_angle)

    end_x = arc_cx + curl_r * math.cos(end_angle)
    end_z = arc_cz + curl_r * math.sin(end_angle)

    # Midpoint for threePointArc (ensures correct arc direction).
    mid_angle = (start_angle + end_angle) / 2.0
    mid_x = arc_cx + curl_r * math.cos(mid_angle)
    mid_z = arc_cz + curl_r * math.sin(mid_angle)

    # Arc path in XZ plane (Y=0).
    path = (
        cq.Workplane("XZ")
        .moveTo(start_x, start_z)
        .threePointArc((mid_x, mid_z), (end_x, end_z))
    )

    # Cross-section perpendicular to path start. The tangent at the start
    # points in +X, so the profile lies in the YZ plane at x = start_x.
    curl = (
        cq.Workplane("YZ")
        .workplane(offset=start_x)
        .center(0.0, start_z)
        .circle(tube_r)
        .sweep(path, isFrenet=True)
    )

    # --- Pronounced ball-foot at the curl tip ---
    ball_r = 0.0030
    ball = (
        cq.Workplane("XY")
        .workplane(offset=end_z)
        .center(end_x, 0.0)
        .sphere(ball_r)
    )

    clip = blade.union(bridge).union(curl).union(ball)
    return clip


PLUNGER_SHAFT_LEN = 0.034


def _build_plunger_geometry() -> LatheGeometry:
    """Push-button knob + plunger shaft as ONE connected revolved solid.

    Built in its own local frame with the button crown apex at z=0 (the joint
    origin) and the shaft extending DOWN in -Z so it stays captured in the
    barrel bore. The shaft radius matches the bore so it contacts the barrel
    (mechanical support) instead of floating. Profile points are (radius, z),
    lathe axis +Z, traced from the crown apex down the outside to the shaft tip.
    """
    knurl_r = 0.0064          # outer knurled collar radius of the button
    cap_r = 0.0056            # button cap radius
    shaft_r = BORE_R          # snug sliding fit against the bore wall

    z_crown = 0.0            # rounded crown apex (top)
    z_cap = -0.0018          # below the small dome
    z_collar_top = -0.0040   # top of the knurled collar
    z_collar_bot = -0.0090   # bottom of the knurled collar
    z_step = -0.0110         # step down to the shaft
    z_shaft_top = -0.0120
    z_shaft_tip = z_shaft_top - PLUNGER_SHAFT_LEN

    profile = [
        (0.0, z_crown),                  # crown apex on axis
        (cap_r * 0.55, z_crown - 0.0006),
        (cap_r, z_cap),                  # cap shoulder
        (knurl_r, z_collar_top),         # flare out to the knurled collar
        (knurl_r, z_collar_bot),         # straight knurled collar wall
        (cap_r * 0.78, z_step),          # taper in toward the shaft
        (shaft_r, z_shaft_top),          # shaft begins
        (shaft_r, z_shaft_tip),          # down the shaft
        (0.0, z_shaft_tip),              # close across to axis (shaft tip)
    ]
    return LatheGeometry(profile, segments=48, closed=True)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retractable_ballpoint_pen")
    model.material("pen_alum", rgba=ALUM)
    model.material("pen_chrome", rgba=CHROME)
    model.material("pen_tip", rgba=TIP_METAL)

    # --- Barrel (root) ---
    barrel = model.part("barrel")
    barrel.visual(
        mesh_from_geometry(_build_barrel_geometry(), "barrel_body"),
        material="pen_alum",
        name="barrel_body",
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
    # The plunger button-top sits just above the collar at rest (q=0).
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

    # --- Hero parts present and placed ---
    clip_aabb = ctx.part_world_aabb(clip)
    barrel_aabb = ctx.part_world_aabb(barrel)
    if clip_aabb is not None and barrel_aabb is not None:
        clip_z_span = clip_aabb[1][2] - clip_aabb[0][2]
        ctx.check(
            "clip is a long blade with curl",
            clip_z_span > 0.040,
            details=f"clip z-length={clip_z_span}",
        )
        # Clip sits on the +X side, on the upper portion of the barrel.
        ctx.check(
            "clip stands off on +X side",
            clip_aabb[1][0] > barrel_aabb[1][0] - 0.001,
            details=f"clip max x={clip_aabb[1][0]}, barrel max x={barrel_aabb[1][0]}",
        )
        # The curled ball-foot extends the clip well outward from the barrel
        # (the ball is further from barrel axis than the blade).
        ctx.check(
            "ball-foot extends clip outward",
            clip_aabb[1][0] > R_COLLAR + 0.006,
            details=f"clip max x={clip_aabb[1][0]}",
        )
        # The curl+ball extends the clip below the straight blade portion.
        # The ball-foot tip should be significantly below the blade bottom.
        ctx.check(
            "curled ball-foot reaches below blade",
            clip_aabb[0][2] < COLLAR_TOP_Z - 0.055,
            details=f"clip min z={clip_aabb[0][2]}",
        )

    # Clip is mounted to the upper barrel (its top is near the collar top).
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

    # --- Click mechanism actuates correctly ---
    rest_pos = ctx.part_world_position(plunger)
    rest_aabb = ctx.part_world_aabb(plunger)
    with ctx.pose({plunge_joint: PLUNGER_TRAVEL}):
        pressed_pos = ctx.part_world_position(plunger)
        pressed_aabb = ctx.part_world_aabb(plunger)
        # Plunger shaft stays captured in the barrel (overlap on Z) when pressed.
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
