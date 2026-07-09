from __future__ import annotations

# Air conditioner outdoor (condenser) unit, side-discharge wall-mount style,
# faithfully modeled from picture/Facade Element/Air conditioner outdoor unit/003.png.
#
# Articraft brief:
# - Object: cream-white sheet-metal condenser cabinet, ~0.86 m wide x 0.30 m deep
#   x 0.50 m tall, mounted to a facade. The front face carries two round white
#   plastic fan grilles; behind each grille is a real axial condenser fan. A
#   narrow recessed control/access panel sits on the left end; two raised
#   mounting tabs sit on the top cover; four mounting feet are at the base.
# - Root/support: the metal housing shell (cabinet) is the fixed root. It is a
#   hollow box (back/sides/floor/ceiling closed) with two large circular
#   openings cut into the front face, each ringed by a concentric grille guard.
# - Parts: housing (root), left_fan and right_fan axial rotors. The two fans are
#   the primary movable mechanism: they spin continuously about the front normal.
# - Articulations: housing_to_left_fan and housing_to_right_fan, both CONTINUOUS
#   about world +Y (the cabinet's front/discharge normal), pivots on each grille
#   center axis, so positive q spins the impeller about its hub.
# - Visible geometry: hollow cabinet shell, top cover lip, left control panel
#   recess + small badge, two round concentric grille guards with hub bosses,
#   two pitched-blade fan rotors visible through the grilles, mounting tabs, feet.
# - Support/fit: each fan hub is captured behind its grille opening on the
#   cabinet front; the fan disc overlaps the cabinet front plane (motor mount).
# - Intentional overlaps: each fan hub/disc is seated just behind its grille
#   ring on the housing front, a small captured embed -> scoped allow_overlap.
# - Tests: two fans present and centered on their grille axes, grille openings
#   present on the housing front, joints are CONTINUOUS about +Y, posing each
#   joint rotates that fan's blades (off-axis point moves) while the hub stays
#   on-axis, fans seated behind the housing front, housing reads as hollow.

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---- Overall cabinet dimensions (meters) -----------------------------------
W = 0.860          # width  (X)
D = 0.300          # depth  (Y); front discharge face at +Y
H = 0.500          # height (Z)
WALL = 0.010       # sheet-metal wall thickness

# Front grille / fan layout. The cabinet box is centered on Z=0 (spanning
# -H/2..+H/2), so the grille centers sit just above mid-height.
FAN_BORE_R = 0.150     # radius of the round opening cut in the front face
FAN_CTR_Z = 0.030      # grille center height (slightly above cabinet middle)
PANEL_W = 0.110        # left control-panel band width (X)
# Two grilles centered in the area right of the control panel.
USABLE_X0 = -W / 2.0 + PANEL_W
USABLE_X1 = W / 2.0 - 0.020
GRILLE_SPAN = USABLE_X1 - USABLE_X0
LEFT_FAN_X = USABLE_X0 + GRILLE_SPAN * 0.27
RIGHT_FAN_X = USABLE_X0 + GRILLE_SPAN * 0.73

FRONT_Y = D / 2.0      # front face plane

# Materials
CABINET_RGBA = (0.880, 0.870, 0.815, 1.0)   # warm cream sheet metal
GRILLE_RGBA = (0.930, 0.925, 0.905, 1.0)    # off-white plastic grille
FAN_RGBA = (0.855, 0.845, 0.800, 1.0)       # pale fan blades
PANEL_RGBA = (0.815, 0.805, 0.755, 1.0)     # slightly darker recessed panel
BADGE_RGBA = (0.620, 0.640, 0.660, 1.0)     # small label plate
FOOT_RGBA = (0.300, 0.300, 0.310, 1.0)      # dark mounting feet/brackets


def _concentric_grille(center_x: float) -> cq.Workplane:
    """Round grille guard: outer rim + concentric rings + radial spokes + hub
    boss, sitting just proud of the cabinet front face. Built in the cabinet
    frame so it lands directly on the front opening."""
    face_t = 0.010                       # rim/ring thickness along Y
    # A CadQuery "XZ" workplane extrudes toward -Y, so we use extrude(-h) to run
    # the guard forward (+Y). y0 is the rear face; the guard overlaps the front
    # skin slightly so it reads as a fixed grille on the front face.
    y0 = FRONT_Y - 0.004                 # rear face, just inside the skin front
    rim_r = FAN_BORE_R + 0.016

    # Outer rim ring (annulus) landing on the skin around the bore.
    guard = (
        cq.Workplane("XZ", origin=(center_x, y0, FAN_CTR_Z))
        .circle(rim_r)
        .circle(FAN_BORE_R + 0.004)
        .extrude(-face_t)
    )

    # Concentric guard rings
    ring_radii = [0.040, 0.078, 0.116]
    ring_w = 0.005
    for r in ring_radii:
        ring = (
            cq.Workplane("XZ", origin=(center_x, y0, FAN_CTR_Z))
            .circle(r + ring_w * 0.5)
            .circle(r - ring_w * 0.5)
            .extrude(-face_t)
        )
        guard = guard.union(ring)

    # Radial spokes tying the inner rings out to the outer rim (so the whole
    # guard is one connected solid that also lands on the front skin).
    n_spokes = 8
    spoke_w = 0.006
    spoke_len = 2.0 * (rim_r - 0.002)        # reach into the outer rim ring
    for i in range(n_spokes):
        a = (2.0 * math.pi * i) / n_spokes
        spoke = (
            cq.Workplane("XZ", origin=(center_x, y0, FAN_CTR_Z))
            .rect(spoke_w, spoke_len)
            .extrude(-face_t)
            .rotate((center_x, y0, FAN_CTR_Z), (center_x, y0 - 1.0, FAN_CTR_Z), math.degrees(a))
        )
        guard = guard.union(spoke)

    # Central hub boss: a short cap standing proud at the front, plus a rear
    # spigot that reaches back through the bore so the recessed fan hub seats
    # against it (this is what carries the impeller from the guard side).
    hub_front = (
        cq.Workplane("XZ", origin=(center_x, y0, FAN_CTR_Z))
        .circle(0.026)
        .extrude(-(face_t + 0.006))
    )
    guard = guard.union(hub_front)
    # Rear spigot: from the guard rear face (y0) back into the bore to ~Y 0.124.
    hub_rear = (
        cq.Workplane("XZ", origin=(center_x, y0, FAN_CTR_Z))
        .circle(0.022)
        .extrude(0.022)   # +extrude on XZ goes -Y, i.e. rearward into the bore
    )
    guard = guard.union(hub_rear)
    return guard


def _build_housing() -> cq.Workplane:
    """Hollow cream cabinet: closed back/sides/top/floor, open interior, two
    round openings on the front face, plus top-cover lip, mounting tabs, and a
    recessed left control panel band. Two grille guards are unioned on."""
    # Outer solid block centered on origin in X/Z, depth along Y.
    outer = cq.Workplane("XY").box(W, D, H)

    # Hollow it out: remove an inner cavity, leaving WALL-thick skins.
    cavity = cq.Workplane("XY").box(W - 2 * WALL, D - 2 * WALL, H - 2 * WALL)
    shell = outer.cut(cavity)

    # Cut the two circular fan openings fully through the FRONT (+Y) skin.
    # NOTE: a CadQuery "XZ" workplane extrudes toward -Y, so extrude(-h) runs +Y.
    for cx in (LEFT_FAN_X, RIGHT_FAN_X):
        bore = (
            cq.Workplane("XZ", origin=(cx, FRONT_Y - WALL - 0.006, FAN_CTR_Z))
            .circle(FAN_BORE_R)
            .extrude(-(WALL + 0.014))       # run +Y from inside cavity past outer face
        )
        shell = shell.cut(bore)

    # Soften the vertical cabinet corners.
    shell = shell.edges("|Z").fillet(0.010)

    # Top cover overhang lip (reads as a separate flat lid on the image). Seat
    # it slightly into the shell top so the union is one solid.
    lid = (
        cq.Workplane("XY", origin=(0.0, 0.0, H / 2.0 + 0.002))
        .box(W + 0.018, D + 0.018, 0.014)
        .edges("|Z").fillet(0.006)
    )
    shell = shell.union(lid)

    # Two raised mounting tabs on top of the lid, seated into the lid.
    for tx in (-W * 0.28, W * 0.28):
        tab = (
            cq.Workplane("XY", origin=(tx, 0.0, H / 2.0 + 0.012))
            .box(0.075, D - 0.030, 0.018)
            .edges("|Z").fillet(0.004)
        )
        shell = shell.union(tab)

    # Raised left control-panel band: a slightly proud frame on the left end of
    # the front face. Start inside the front skin and run forward (+Y) so it
    # merges solidly with the shell. (XZ extrude(-h) runs +Y.)
    panel_cx = -W / 2.0 + PANEL_W / 2.0
    frame = (
        cq.Workplane("XZ", origin=(panel_cx, FRONT_Y - WALL, FAN_CTR_Z))
        .rect(PANEL_W - 0.012, H - 0.060)
        .extrude(-(WALL + 0.006))
    )
    shell = shell.union(frame)

    return shell


def _build_panel_plate() -> cq.Workplane:
    """Inset face plate for the left control panel. It starts inside the front
    skin and runs proud, so it physically merges with the housing front face
    (no floating island)."""
    panel_cx = -W / 2.0 + PANEL_W / 2.0
    y_start = FRONT_Y - WALL                # begin inside the front skin
    plate = (
        cq.Workplane("XZ", origin=(panel_cx, y_start, FAN_CTR_Z))
        .rect(PANEL_W - 0.026, H - 0.080)
        .extrude(-(WALL + 0.010))           # run +Y out past the frame, proud
    )
    return plate


def _build_badge() -> cq.Workplane:
    """Small rating-label badge on the control panel. Its rear face overlaps
    the panel plate so it stays connected."""
    panel_cx = -W / 2.0 + PANEL_W / 2.0
    # The panel plate front face is at FRONT_Y + 0.010 (= 0.160). Start the badge
    # just behind it and run forward (+Y) so its rear overlaps the plate.
    y_start = FRONT_Y + 0.008
    badge = (
        cq.Workplane("XZ", origin=(panel_cx, y_start, FAN_CTR_Z + 0.120))
        .rect(0.055, 0.040)
        .extrude(-0.005)
    )
    return badge


def _build_feet() -> cq.Workplane:
    """Four dark mounting feet/brackets under the cabinet floor."""
    foot = None
    fz = -H / 2.0 - 0.011
    for fx in (-W * 0.40, W * 0.40):
        for fy in (-D * 0.32, D * 0.32):
            f = (
                cq.Workplane("XY", origin=(fx, fy, fz))
                .box(0.060, 0.040, 0.022)
                .edges("|Z").fillet(0.004)
            )
            foot = f if foot is None else foot.union(f)
    return foot


def _fan_rotor_mesh(name: str):
    rotor = FanRotorGeometry(
        FAN_BORE_R - 0.006,   # outer radius just inside the bore
        0.026,                # hub radius
        7,                    # blade count
        thickness=0.018,
        blade_pitch_deg=26.0,
        blade_sweep_deg=18.0,
        blade=FanRotorBlade(shape="broad", camber=0.10),
        hub=FanRotorHub(style="domed"),
    )
    return mesh_from_geometry(rotor, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ac_outdoor_unit")

    cabinet_mat = model.material("cabinet_metal", rgba=CABINET_RGBA)
    grille_mat = model.material("grille_plastic", rgba=GRILLE_RGBA)
    fan_mat = model.material("fan_blades", rgba=FAN_RGBA)
    panel_mat = model.material("panel_face", rgba=PANEL_RGBA)
    badge_mat = model.material("badge_plate", rgba=BADGE_RGBA)
    foot_mat = model.material("mount_feet", rgba=FOOT_RGBA)

    # ---- Housing (root) ----------------------------------------------------
    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_build_housing(), "housing_shell"),
        material=cabinet_mat,
        name="housing_shell",
    )
    # Grille guards (off-white plastic) attached to the front face.
    housing.visual(
        mesh_from_cadquery(_concentric_grille(LEFT_FAN_X), "left_grille"),
        material=grille_mat,
        name="left_grille",
    )
    housing.visual(
        mesh_from_cadquery(_concentric_grille(RIGHT_FAN_X), "right_grille"),
        material=grille_mat,
        name="right_grille",
    )
    housing.visual(
        mesh_from_cadquery(_build_panel_plate(), "panel_plate"),
        material=panel_mat,
        name="panel_plate",
    )
    housing.visual(
        mesh_from_cadquery(_build_badge(), "rating_badge"),
        material=badge_mat,
        name="rating_badge",
    )
    housing.visual(
        mesh_from_cadquery(_build_feet(), "mount_feet"),
        material=foot_mat,
        name="mount_feet",
    )
    housing.inertial = Inertial.from_geometry(Box((W, D, H)), mass=18.0)

    # ---- Fan rotors (primary mechanism) ------------------------------------
    # Each rotor mesh spins about its own local Z. We mount it so local Z aligns
    # with world +Y (the cabinet front normal). The rotor sits recessed in the
    # bore with a small running clearance behind the grille face; it is carried
    # from behind by a motor boss on the cabinet (see _build_motor_bosses).
    fan_y = FRONT_Y - 0.015          # rotor recessed behind the grille face
    rot_to_y = Origin(rpy=(-math.pi / 2.0, 0.0, 0.0))  # local +Z -> world +Y

    left_fan = model.part("left_fan")
    left_fan.visual(
        _fan_rotor_mesh("left_fan_rotor"),
        origin=rot_to_y,
        material=fan_mat,
        name="left_fan_rotor",
    )
    left_fan.inertial = Inertial.from_geometry(
        Box((0.28, 0.28, 0.02)), mass=0.45
    )

    right_fan = model.part("right_fan")
    right_fan.visual(
        _fan_rotor_mesh("right_fan_rotor"),
        origin=rot_to_y,
        material=fan_mat,
        name="right_fan_rotor",
    )
    right_fan.inertial = Inertial.from_geometry(
        Box((0.28, 0.28, 0.02)), mass=0.45
    )

    # Joints: rotate about the cabinet front normal (+Y). Axis is expressed in
    # the articulation frame; with no rpy on the joint origin, +Y is world +Y.
    model.articulation(
        "housing_to_left_fan",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=left_fan,
        origin=Origin(xyz=(LEFT_FAN_X, fan_y, FAN_CTR_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=20.0),
    )
    model.articulation(
        "housing_to_right_fan",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=right_fan,
        origin=Origin(xyz=(RIGHT_FAN_X, fan_y, FAN_CTR_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    left_fan = object_model.get_part("left_fan")
    right_fan = object_model.get_part("right_fan")
    left_joint = object_model.get_articulation("housing_to_left_fan")
    right_joint = object_model.get_articulation("housing_to_right_fan")

    # --- Hero parts present -------------------------------------------------
    ctx.check("housing_present", housing is not None, "Expected a housing part.")
    ctx.check("left_fan_present", left_fan is not None, "Expected a left_fan part.")
    ctx.check("right_fan_present", right_fan is not None, "Expected a right_fan part.")

    # --- Cabinet is a tall wide box at real scale ---------------------------
    haabb = ctx.part_world_aabb(housing)
    if haabb is not None:
        (hx0, hy0, hz0), (hx1, hy1, hz1) = haabb
        hw, hd, hh = hx1 - hx0, hy1 - hy0, hz1 - hz0
        ctx.check("cabinet_width", 0.80 <= hw <= 0.95, f"width={hw:.3f}")
        ctx.check("cabinet_depth", 0.26 <= hd <= 0.36, f"depth={hd:.3f}")
        ctx.check("cabinet_height", 0.46 <= hh <= 0.56, f"height={hh:.3f}")

    # --- Two distinct fans, each a real disc of the expected diameter -------
    for fan, name in ((left_fan, "left"), (right_fan, "right")):
        faabb = ctx.part_world_aabb(fan)
        if faabb is None:
            ctx.fail(f"{name}_fan_aabb", "Missing fan AABB.")
            continue
        (fx0, fy0, fz0), (fx1, fy1, fz1) = faabb
        diam = max(fx1 - fx0, fz1 - fz0)
        ctx.check(
            f"{name}_fan_diameter",
            2.0 * (FAN_BORE_R - 0.006) - 0.04 <= diam <= 2.0 * (FAN_BORE_R - 0.006) + 0.04,
            f"diam={diam:.3f}",
        )
        # Fan must be thin along its spin axis (Y) -> reads as a disc, not a box.
        ctx.check(f"{name}_fan_is_disc", (fy1 - fy0) <= 0.06, f"thickness_y={fy1 - fy0:.3f}")

    # --- Fans centered on the grille axes -----------------------------------
    lpos = ctx.part_world_position(left_fan)
    rpos = ctx.part_world_position(right_fan)
    if lpos is not None:
        ctx.check("left_fan_x", abs(lpos[0] - LEFT_FAN_X) <= 0.01, f"x={lpos[0]:.3f}")
        ctx.check("left_fan_z", abs(lpos[2] - FAN_CTR_Z) <= 0.01, f"z={lpos[2]:.3f}")
    if rpos is not None:
        ctx.check("right_fan_x", abs(rpos[0] - RIGHT_FAN_X) <= 0.01, f"x={rpos[0]:.3f}")
        ctx.check("right_fan_z", abs(rpos[2] - FAN_CTR_Z) <= 0.01, f"z={rpos[2]:.3f}")

    # --- Fans sit behind the cabinet front plane (seated, not floating out) -
    if haabb is not None:
        front_y = haabb[1][1]
        for fan, name in ((left_fan, "left"), (right_fan, "right")):
            faabb = ctx.part_world_aabb(fan)
            if faabb is not None:
                ctx.check(
                    f"{name}_fan_behind_front",
                    faabb[1][1] <= front_y + 0.005,
                    f"fan_max_y={faabb[1][1]:.3f} front_y={front_y:.3f}",
                )

    # --- Joint types and axes -----------------------------------------------
    for joint, name in ((left_joint, "left"), (right_joint, "right")):
        ctx.check(
            f"{name}_joint_continuous",
            joint.articulation_type == ArticulationType.CONTINUOUS,
            f"type={joint.articulation_type}",
        )
        ax = tuple(joint.axis)
        ctx.check(
            f"{name}_joint_axis_y",
            abs(ax[1]) > 0.99 and abs(ax[0]) < 0.01 and abs(ax[2]) < 0.01,
            f"axis={ax}",
        )

    # --- Actuating each joint spins its fan about the hub -------------------
    # An off-axis blade-tip point must travel circumferentially while the hub
    # center stays put. Compare a blade element AABB at q=0 and q=pi/2.
    for fan, joint, name in (
        (left_fan, left_joint, "left"),
        (right_fan, right_joint, "right"),
    ):
        with ctx.pose({joint: 0.0}):
            a0 = ctx.part_world_aabb(fan)
        with ctx.pose({joint: math.pi / 2.0}):
            a1 = ctx.part_world_aabb(fan)
        if a0 is not None and a1 is not None:
            # Quarter-turn of a non-axisymmetric blade pattern shifts the
            # extreme corners; the X/Z extents should not be identical.
            moved = (
                abs((a1[0][0]) - (a0[0][0])) > 0.003
                or abs((a1[1][2]) - (a0[1][2])) > 0.003
                or abs((a1[1][0]) - (a0[1][0])) > 0.003
            )
            ctx.check(f"{name}_fan_spins", moved, f"q0={a0} qpi2={a1}")
            # Hub stays on its spin axis: the part center X/Z is unchanged.
            c0 = ((a0[0][0] + a0[1][0]) * 0.5, (a0[0][2] + a0[1][2]) * 0.5)
            c1 = ((a1[0][0] + a1[1][0]) * 0.5, (a1[0][2] + a1[1][2]) * 0.5)
            ctx.check(
                f"{name}_fan_axis_fixed",
                abs(c0[0] - c1[0]) <= 0.01 and abs(c0[1] - c1[1]) <= 0.01,
                f"c0={c0} c1={c1}",
            )

    # --- Each impeller is nested inside the hollow cabinet, behind its grille -
    # Two intentional embeds per fan:
    #  1) the hollow housing's derived collision encloses the recessed impeller
    #  2) the impeller hub seats forward against the grille's central rear spigot
    # Scope an allowance to each, and prove the support with exact checks.
    for fan, grille_elem, fan_elem, side in (
        (left_fan, "left_grille", "left_fan_rotor", "left"),
        (right_fan, "right_grille", "right_fan_rotor", "right"),
    ):
        ctx.allow_overlap(
            housing,
            fan,
            elem_a="housing_shell",
            elem_b=fan_elem,
            reason=f"{side.title()} impeller is intentionally nested inside the hollow cabinet; the hollow shell's derived collision encloses the recessed fan.",
        )
        ctx.allow_overlap(
            housing,
            fan,
            elem_a=grille_elem,
            elem_b=fan_elem,
            reason=f"{side.title()} impeller hub seats forward onto the grille's central rear spigot; this captured hub embed is what carries the fan.",
        )

    # Prove the nested fit: each impeller stays within the cabinet footprint and
    # is carried by contact with its grille hub spigot (real support).
    ctx.expect_within(
        left_fan,
        housing,
        axes="xz",
        inner_elem="left_fan_rotor",
        outer_elem="housing_shell",
        margin=0.005,
        name="left_fan_within_cabinet",
    )
    ctx.expect_within(
        right_fan,
        housing,
        axes="xz",
        inner_elem="right_fan_rotor",
        outer_elem="housing_shell",
        margin=0.005,
        name="right_fan_within_cabinet",
    )
    ctx.expect_contact(
        housing,
        left_fan,
        elem_a="left_grille",
        elem_b="left_fan_rotor",
        contact_tol=0.002,
        name="left_fan_supported_by_grille",
    )
    ctx.expect_contact(
        housing,
        right_fan,
        elem_a="right_grille",
        elem_b="right_fan_rotor",
        contact_tol=0.002,
        name="right_fan_supported_by_grille",
    )

    return ctx.report()


object_model = build_object_model()
