from __future__ import annotations

# Twist-action ballpoint pen (fork variant of the click-action pen).
#
# Articraft brief:
# - Object: a silver metallic twist-action ballpoint pen, ~0.126 m long,
#   barrel diameter ~0.011 m. Upright along +Z: writing tip at bottom (-Z),
#   smooth dome cap at top (+Z).
# - Root/support: lower barrel body (tapered body + nose cone + writing tip
#   + seam band) is the fixed root that carries everything.
# - Parts:
#     * barrel  - root: tapered body shell, conical nose, writing ball-tip,
#                  and a decorative seam band at the twist joint.
#     * sleeve  - rotating upper section: collar body + smooth dome cap +
#                  grip ridges. This is the twist-advance mechanism.
#     * clip    - the spring-steel pocket clip, fixed to the sleeve so it
#                  rotates with the twist action.
# - Articulations:
#     * barrel_to_sleeve, REVOLUTE, axis +Z (barrel long axis), origin at the
#       visible seam surface (z=SEAM_Z). Positive q twists the sleeve.
#       Limits: 0 to π/2 (quarter turn to advance the nib).
#     * sleeve_to_clip, FIXED. Clip is structural trim on the sleeve.
# - Visible geometry: lathed tapered metal barrel, lathed sleeve with dome
#   cap, CadQuery grip ridges (repeated via for-loop), CadQuery seam band,
#   CadQuery spring clip with oval window. Restrained brushed-aluminium /
#   chrome materials.
# - Support/fit: sleeve flat-seats on barrel at the seam face; clip mounts
#   on the sleeve collar via its bridge piece.
# - Intentional overlaps: clip bridge embeds slightly into the sleeve collar
#   surface (seated trim -> scoped allow_overlap).
# - Tests: sleeve is REVOLUTE about Z; twist moves clip; no plunger; barrel
#   tip is lowest; sleeve dome is topmost; grip rings present; clip present.

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
SEAM_Z = 0.100           # barrel/sleeve twist joint seam (= body top)
COLLAR_TOP_Z = 0.118     # collar top in world coordinates
BODY_TOP_Z = 0.100       # main tapered body upper edge (= SEAM_Z)
NOSE_TOP_Z = 0.030       # where the body meets the nose cone
NOSE_TIP_Z = 0.006       # base of the metal writing ball seat
TIP_END_Z = 0.000        # ball tip lowest point

R_COLLAR = 0.0062        # radius of the upper collar
R_BODY_TOP = 0.0058      # upper body radius
R_BODY_MID = 0.0056
R_NOSE_TOP = 0.0050      # body radius where nose begins
R_TIP_SEAT = 0.0011      # tip seat radius
R_BALL = 0.0006          # writing ball radius

TWIST_UPPER = math.pi / 2.0  # quarter-turn advance

CHROME = (0.83, 0.84, 0.86, 1.0)
ALUM = (0.74, 0.75, 0.77, 1.0)
TIP_METAL = (0.62, 0.63, 0.66, 1.0)
GRIP_METAL = (0.55, 0.56, 0.58, 1.0)
BAND_METAL = (0.78, 0.79, 0.82, 1.0)


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _build_barrel_geometry() -> LatheGeometry:
    """Lower barrel body: lathed tapered profile from writing tip to the seam.

    Identical outer silhouette to the parent click-action pen up to the seam.
    Flat top at SEAM_Z provides the visible mating face for the twist joint.
    """
    profile = [
        (0.0, TIP_END_Z),                 # tip apex on axis (lowest point)
        (R_BALL, NOSE_TIP_Z * 0.45),      # rounded ball
        (R_TIP_SEAT, NOSE_TIP_Z),         # tip seat shoulder
        (R_NOSE_TOP, NOSE_TOP_Z),         # top of the conical nose cone
        (R_BODY_MID, 0.060),              # mid body
        (R_BODY_TOP, BODY_TOP_Z),         # upper body / seam level
        (R_BODY_TOP, SEAM_Z),             # outer wall up to seam face
        (0.0, SEAM_Z),                    # flat top close to axis
    ]
    return LatheGeometry(profile, segments=64, closed=True)


def _build_sleeve_geometry() -> LatheGeometry:
    """Upper rotating sleeve: collar body + dome cap, in sleeve local frame.

    Built with origin at z=0 (the bottom face = joint seam surface).
    The dome replaces the exposed push-button; smooth rounded top.
    """
    local_collar_top = COLLAR_TOP_Z - SEAM_Z  # 0.018 in local coords
    dome_h = 0.008                             # dome height above collar

    profile = [
        (0.0, 0.0),                                  # center bottom (flat face)
        (R_BODY_TOP, 0.0),                           # outer bottom edge
        (R_BODY_TOP, 0.001),                         # short straight outer wall
        (R_COLLAR, 0.002),                           # step out to collar
        (R_COLLAR, local_collar_top),                 # collar top
        (R_COLLAR * 0.93, local_collar_top + 0.002), # dome transition
        (R_COLLAR * 0.74, local_collar_top + 0.004), # dome curve
        (R_COLLAR * 0.44, local_collar_top + 0.006), # approaching apex
        (0.0, local_collar_top + dome_h),             # dome apex on axis
    ]
    return LatheGeometry(profile, segments=64, closed=True)


def _build_seam_band() -> cq.Workplane:
    """Decorative metal band at the barrel/sleeve twist seam.

    Thin annular ring centred on the seam, slightly proud of the barrel
    surface and embedding marginally for a seated-trim connection.
    """
    r_outer = R_BODY_TOP + 0.0004
    r_inner = R_BODY_TOP - 0.0003
    h = 0.003
    return (
        cq.Workplane("XY")
        .workplane(offset=SEAM_Z - h / 2.0)
        .circle(r_outer)
        .circle(r_inner)
        .extrude(h)
    )


def _build_grip_ring(z_pos: float) -> cq.Workplane:
    """Thin annular grip ridge for the sleeve collar surface.

    Inner radius embeds slightly into the sleeve body for connectivity;
    outer radius is proud of the collar surface to read as a raised ridge.
    """
    r_outer = R_COLLAR + 0.0004
    r_inner = R_COLLAR - 0.0003
    h = 0.0008
    return (
        cq.Workplane("XY")
        .workplane(offset=z_pos)
        .circle(r_outer)
        .circle(r_inner)
        .extrude(h)
    )


def _build_clip_shape() -> cq.Workplane:
    """Spring-steel pocket clip built in sleeve-local coordinates.

    Identical shape to the parent click-action pen clip, shifted so its
    bridge anchors to the sleeve collar and the blade extends down the
    barrel. When the sleeve twists, the clip rotates with it.
    """
    clip_len = 0.052
    clip_w = 0.0072
    clip_t = 0.0009
    local_collar_top = COLLAR_TOP_Z - SEAM_Z
    top_z = local_collar_top - 0.002

    # Thin blade on the +X side of the collar, extruded in X (thickness).
    blade = (
        cq.Workplane("YZ")
        .workplane(offset=R_COLLAR + clip_t / 2.0 + 0.0006)
        .center(0.0, top_z - clip_len / 2.0)
        .rect(clip_w, clip_len)
        .extrude(clip_t)
    )
    # Oval cut-out window in the upper part of the blade.
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

    # Bridge connecting the blade top to the collar surface (anchor).
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

    # Rounded foot/bump near the clip lower tip (pocket-grip pad).
    foot = (
        cq.Workplane("YZ")
        .workplane(offset=R_COLLAR + 0.0006)
        .center(0.0, top_z - clip_len + 0.004)
        .circle(0.0028)
        .extrude(0.0016)
    )

    clip = blade.union(bridge).union(foot)
    return clip


# ---------------------------------------------------------------------------
# Build
# ---------------------------------------------------------------------------

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="twist_action_ballpoint_pen")
    model.material("pen_alum", rgba=ALUM)
    model.material("pen_chrome", rgba=CHROME)
    model.material("pen_tip", rgba=TIP_METAL)
    model.material("pen_grip", rgba=GRIP_METAL)
    model.material("pen_band", rgba=BAND_METAL)

    # --- Barrel (root): lower body + nose + tip + seam band ---
    barrel = model.part("barrel")
    barrel.visual(
        mesh_from_geometry(_build_barrel_geometry(), "barrel_body"),
        material="pen_alum",
        name="barrel_body",
    )
    barrel.visual(
        mesh_from_cadquery(_build_seam_band(), "seam_band"),
        material="pen_band",
        name="seam_band",
    )

    # --- Sleeve (rotating upper section): collar + dome + grip rings ---
    sleeve = model.part("sleeve")
    sleeve.visual(
        mesh_from_geometry(_build_sleeve_geometry(), "sleeve_body"),
        material="pen_alum",
        name="sleeve_body",
    )

    # Grip ridges: repeated surface detail via shared helper + for-loop
    n_grips = 3
    grip_spacing = 0.004
    grip_start_z = 0.005
    for i in range(n_grips):
        z = grip_start_z + i * grip_spacing
        sleeve.visual(
            mesh_from_cadquery(_build_grip_ring(z), f"grip_ring_{i}"),
            material="pen_grip",
            name=f"grip_ring_{i}",
        )

    # --- Clip (fixed to sleeve, rotates with twist action) ---
    clip = model.part("clip")
    clip.visual(
        mesh_from_cadquery(_build_clip_shape(), "clip_blade"),
        material="pen_chrome",
        name="clip_blade",
    )

    # --- Articulations ---

    # Twist joint: sleeve rotates about barrel long axis at the seam.
    # Origin at the visible contact surface (flat mating face).
    model.articulation(
        "barrel_to_sleeve",
        ArticulationType.REVOLUTE,
        parent=barrel,
        child=sleeve,
        origin=Origin(xyz=(0.0, 0.0, SEAM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=2.0,
            lower=0.0,
            upper=TWIST_UPPER,
        ),
    )

    # Clip is structural trim; fixed mount on the sleeve collar.
    model.articulation(
        "sleeve_to_clip",
        ArticulationType.FIXED,
        parent=sleeve,
        child=clip,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    return model


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    barrel = object_model.get_part("barrel")
    sleeve = object_model.get_part("sleeve")
    clip = object_model.get_part("clip")
    twist_joint = object_model.get_articulation("barrel_to_sleeve")
    clip_joint = object_model.get_articulation("sleeve_to_clip")

    # --- Intentional overlap: clip bridge embeds into sleeve collar ---
    ctx.allow_overlap(
        sleeve,
        clip,
        elem_a="sleeve_body",
        elem_b="clip_blade",
        reason=(
            "The clip bridge piece embeds slightly into the sleeve collar "
            "surface for a seated-trim mounting connection."
        ),
    )

    # --- Joint type / axis claims ---
    ctx.check(
        "sleeve joint is revolute",
        twist_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"got {twist_joint.articulation_type}",
    )
    ax = tuple(round(a, 6) for a in twist_joint.axis)
    ctx.check(
        "twist axis is along Z (barrel long axis)",
        abs(ax[0]) < 1e-6 and abs(ax[1]) < 1e-6 and abs(ax[2]) > 0.5,
        details=f"axis={ax}",
    )
    ctx.check(
        "clip joint is fixed",
        clip_joint.articulation_type == ArticulationType.FIXED,
        details=f"got {clip_joint.articulation_type}",
    )

    # --- Twist joint origin is at the seam surface ---
    j_origin = twist_joint.origin
    ctx.check(
        "twist joint origin at seam level",
        j_origin is not None and abs(j_origin.xyz[2] - SEAM_Z) < 0.001,
        details=f"origin_z={j_origin.xyz[2] if j_origin else None}",
    )

    # --- Hero parts present and correctly shaped ---
    barrel_aabb = ctx.part_world_aabb(barrel)
    sleeve_aabb = ctx.part_world_aabb(sleeve)
    clip_aabb = ctx.part_world_aabb(clip)

    # Writing tip is the lowest geometry
    if barrel_aabb is not None:
        ctx.check(
            "writing tip reaches the bottom",
            barrel_aabb[0][2] <= 0.001,
            details=f"barrel min z={barrel_aabb[0][2]}",
        )

    # Sleeve dome is the topmost feature (no exposed push-button)
    if sleeve_aabb is not None and barrel_aabb is not None:
        ctx.check(
            "sleeve dome is topmost feature (no button)",
            sleeve_aabb[1][2] > barrel_aabb[1][2],
            details=f"sleeve top={sleeve_aabb[1][2]}, barrel top={barrel_aabb[1][2]}",
        )

    # Sleeve body sits above the seam
    if sleeve_aabb is not None:
        ctx.check(
            "sleeve sits above seam level",
            sleeve_aabb[0][2] >= SEAM_Z - 0.002,
            details=f"sleeve min z={sleeve_aabb[0][2]}",
        )

    # Clip present and has the long-blade shape
    if clip_aabb is not None:
        clip_len = clip_aabb[1][2] - clip_aabb[0][2]
        ctx.check(
            "clip is a long blade",
            clip_len > 0.035,
            details=f"clip z-length={clip_len}",
        )
        if barrel_aabb is not None:
            ctx.check(
                "clip stands off on +X side",
                clip_aabb[1][0] > barrel_aabb[1][0] - 0.001,
                details=f"clip max x={clip_aabb[1][0]}, barrel max x={barrel_aabb[1][0]}",
            )

    # Clip is anchored to the sleeve collar
    ctx.expect_contact(
        clip,
        sleeve,
        contact_tol=0.0015,
        name="clip is anchored to the sleeve",
    )

    # --- Sleeve and barrel contact at seam (support) ---
    ctx.expect_contact(
        sleeve,
        barrel,
        contact_tol=0.002,
        name="sleeve seated on barrel at rest",
    )

    # --- Twist mechanism actuates correctly ---
    rest_clip_aabb = ctx.part_world_aabb(clip)

    with ctx.pose({twist_joint: TWIST_UPPER}):
        twisted_clip_aabb = ctx.part_world_aabb(clip)

        # Sleeve stays on barrel when twisted
        ctx.expect_contact(
            sleeve,
            barrel,
            contact_tol=0.003,
            name="sleeve stays on barrel when twisted",
        )

    # Clip geometry is offset from the Z axis (on the +X side), so twisting
    # the sleeve must shift the clip AABB centre in XY.
    if rest_clip_aabb is not None and twisted_clip_aabb is not None:
        def _xy_center(aabb):
            cx = (aabb[0][0] + aabb[1][0]) / 2.0
            cy = (aabb[0][1] + aabb[1][1]) / 2.0
            return cx, cy

        rcx, rcy = _xy_center(rest_clip_aabb)
        tcx, tcy = _xy_center(twisted_clip_aabb)
        ctx.check(
            "twisting the sleeve moves the clip",
            abs(tcx - rcx) > 0.002 or abs(tcy - rcy) > 0.002,
            details=f"rest_xy=({rcx:.5f},{rcy:.5f}), twisted_xy=({tcx:.5f},{tcy:.5f})",
        )

    # --- Seam band is present on the barrel ---
    barrel_visuals = [v.name for v in barrel.visuals]
    ctx.check(
        "seam band visual present on barrel",
        "seam_band" in barrel_visuals,
        details=f"barrel visuals={barrel_visuals}",
    )

    # --- Grip rings are present on the sleeve ---
    sleeve_visuals = [v.name for v in sleeve.visuals]
    expected_grips = [f"grip_ring_{i}" for i in range(n_grips)]
    ctx.check(
        "grip ring visuals present on sleeve",
        all(g in sleeve_visuals for g in expected_grips),
        details=f"sleeve visuals={sleeve_visuals}, expected={expected_grips}",
    )

    return ctx.report()


# Reference to n_grips used in tests (must match build)
n_grips = 3

object_model = build_object_model()
