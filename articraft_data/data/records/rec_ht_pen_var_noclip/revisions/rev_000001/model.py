from __future__ import annotations

# Realistic articulated retractable (click-action) ballpoint pen (clipless variant).
#
# Articraft brief:
# - Object: a silver metallic click-action ballpoint pen, ~0.146 m long, barrel
#   diameter ~0.011 m. Modeled upright along +Z: writing tip at the bottom
#   (-Z), push button at the top (+Z). This is a clipless fork variant with a
#   clean barrel collar (no pocket clip).
# - Root/support: the barrel (body + nose cone + tip + collar) is the fixed root
#   that carries everything.
# - Parts:
#     * barrel  - root: tapered body shell, conical nose cone, writing ball-tip,
#                  the clean upper collar (no clip), and decorative grip rings
#                  inlined as parent visuals via a shared geometry helper.
#     * plunger - the push-button knob + plunger shaft that slides into the top
#                  of the barrel. This is the click mechanism.
# - Articulations:
#     * barrel_to_plunger, PRISMATIC, axis -Z (button travels DOWN into the
#       barrel when pressed). Positive q presses the button down. Travel ~7 mm.
#       The plunger shaft is long enough to stay captured inside the barrel
#       throughout the click stroke.
# - Visible geometry: lathed tapered metal barrel, conical nose, tiny round
#   ball tip, knurled push-button cap, decorative grip rings on the lower barrel,
#   restrained brushed-aluminium / chrome materials.
# - Support/fit: plunger shaft is seated inside the barrel bore (intentional
#   nested fit -> scoped allow_overlap).
# - Intentional overlaps: plunger shaft inside the barrel-top bore proxy.
# - Tests: barrel/plunger present; no clip part exists; collar is clean;
#   plunger is PRISMATIC along Z; pressing the button moves it DOWN; the
#   plunger stays captured in the barrel at both stroke ends; nose tip is the
#   lowest geometry.

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


def _build_grip_ring(barrel_r: float, center_z: float) -> cq.Workplane:
    """A thin decorative torus-like ring that wraps around the barrel at a given
    height. Built as a revolved rectangle to read as a raised band on the body.

    Args:
        barrel_r: the barrel outer radius at the ring location.
        center_z: world Z of the ring center.
    """
    ring_h = 0.0008       # ring axial thickness
    ring_proud = 0.0003   # how far the ring stands proud of the barrel surface
    # Inset the inner radius so the ring physically intersects/overlaps with the
    # barrel body, ensuring geometric connectivity (not just surface contact).
    inner_r = barrel_r - 0.0005
    outer_r = barrel_r + ring_proud
    z_lo = center_z - ring_h / 2.0
    z_hi = center_z + ring_h / 2.0
    # For cq.Workplane("XZ"): local x → world X, local y → world Z.
    # To revolve around world Z, use 2D axis along local Y: (0,0) to (0,1).
    ring = (
        cq.Workplane("XZ")
        .moveTo(inner_r, z_lo)
        .lineTo(outer_r, z_lo)
        .lineTo(outer_r, z_hi)
        .lineTo(inner_r, z_hi)
        .close()
        .revolve(360, (0, 0), (0, 1))
    )
    return ring


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
    model = ArticulatedObject(name="retractable_ballpoint_pen_clipless")
    model.material("pen_alum", rgba=ALUM)
    model.material("pen_chrome", rgba=CHROME)
    model.material("pen_tip", rgba=TIP_METAL)
    model.material("pen_ring", rgba=(0.68, 0.69, 0.72, 1.0))

    # --- Barrel (root) ---
    barrel = model.part("barrel")
    barrel.visual(
        mesh_from_geometry(_build_barrel_geometry(), "barrel_body"),
        material="pen_alum",
        name="barrel_body",
    )

    # Decorative grip rings on the lower barrel body, inlined as parent visuals.
    # A shared geometry helper + regular spacing via a for-i-in-range loop.
    grip_ring_positions = [0.040 + i * 0.006 for i in range(4)]
    for i, z in enumerate(grip_ring_positions):
        barrel.visual(
            mesh_from_cadquery(_build_grip_ring(R_BODY_MID, z), f"grip_ring_{i}"),
            material="pen_ring",
            name=f"grip_ring_{i}",
        )

    # --- Plunger / push button (the click mechanism) ---
    plunger = model.part("plunger")
    plunger.visual(
        mesh_from_geometry(_build_plunger_geometry(), "plunger_button"),
        material="pen_chrome",
        name="plunger_button",
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
    plunger = object_model.get_part("plunger")
    plunge_joint = object_model.get_articulation("barrel_to_plunger")

    # The plunger shaft is intentionally nested inside the barrel bore proxy.
    ctx.allow_overlap(
        barrel,
        plunger,
        reason="The plunger shaft is captured inside the barrel bore; this is the click mechanism's sliding fit.",
    )

    # --- Clipless variant: no clip part exists ---
    all_part_names = [p.name for p in object_model.parts]
    ctx.check(
        "no clip part exists",
        "clip" not in all_part_names,
        details=f"parts={all_part_names}",
    )

    # Only barrel and plunger should be present (exactly 2 parts).
    ctx.check(
        "exactly two parts: barrel and plunger",
        len(object_model.parts) == 2,
        details=f"got {len(object_model.parts)} parts: {all_part_names}",
    )

    # Only one articulation (the prismatic plunger joint).
    all_joint_names = [a.name for a in object_model.articulations]
    ctx.check(
        "only one articulation: barrel_to_plunger",
        len(object_model.articulations) == 1
        and "barrel_to_plunger" in all_joint_names,
        details=f"articulations={all_joint_names}",
    )

    # No FIXED joints (the clip fixed joint is gone).
    fixed_joints = [
        a for a in object_model.articulations
        if a.articulation_type == ArticulationType.FIXED
    ]
    ctx.check(
        "no fixed joints remain",
        len(fixed_joints) == 0,
        details=f"fixed joints={[j.name for j in fixed_joints]}",
    )

    # --- Clean collar: barrel has no protruding clip geometry on the collar ---
    barrel_aabb = ctx.part_world_aabb(barrel)
    if barrel_aabb is not None:
        # The collar is the widest part at R_COLLAR=0.0062. Without a clip,
        # the barrel max X should be close to R_COLLAR (no clip standing off).
        collar_max_x = barrel_aabb[1][0]
        ctx.check(
            "collar is clean (no clip protrusion)",
            collar_max_x < R_COLLAR + 0.002,
            details=f"barrel max x={collar_max_x}, expected < {R_COLLAR + 0.002}",
        )

    # --- Barrel has grip ring visuals ---
    barrel_visual_names = [v.name for v in barrel.visuals]
    grip_rings = [n for n in barrel_visual_names if n.startswith("grip_ring_")]
    ctx.check(
        "barrel has grip ring visuals",
        len(grip_rings) >= 3,
        details=f"grip rings found: {grip_rings}",
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
