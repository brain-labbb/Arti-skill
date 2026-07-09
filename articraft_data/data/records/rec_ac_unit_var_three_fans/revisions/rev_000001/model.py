from __future__ import annotations

# Air conditioner outdoor (condenser) unit, side-discharge wall-mount style,
# triple-fan variant: three front fan rotors in a horizontal row.
#
# Articraft brief:
# - Object: cream-white sheet-metal condenser cabinet, ~1.20 m wide x 0.30 m deep
#   x 0.50 m tall, mounted to a facade. The front face carries three round white
#   plastic fan grilles; behind each grille is a real axial condenser fan. A
#   narrow recessed control/access panel sits on the left end; two raised
#   mounting tabs sit on the top cover; four mounting feet are at the base.
# - Root/support: the metal housing shell (cabinet) is the fixed root. It is a
#   hollow box (back/sides/floor/ceiling closed) with three large circular
#   openings cut into the front face, each ringed by a concentric grille guard.
# - Parts: housing (root), fan_0, fan_1, fan_2 axial rotors in a horizontal row.
# - Articulations: housing_to_fan_i, all CONTINUOUS about world +Y (the
#   cabinet's front/discharge normal), pivots on each grille center axis.
# - Visible geometry: hollow cabinet shell, top cover lip, left control panel
#   recess + small badge, three round concentric grille guards with hub bosses,
#   three pitched-blade fan rotors visible through the grilles, mounting tabs, feet.
# - Support/fit: each fan hub is captured behind its grille opening on the
#   cabinet front; the fan disc overlaps the cabinet front plane (motor mount).
# - Intentional overlaps: each fan hub/disc is seated just behind its grille
#   ring on the housing front, a small captured embed -> scoped allow_overlap.
# - Tests: three fans present and centered on their grille axes, grille openings
#   present on the housing front, joints are CONTINUOUS about +Y, posing each
#   joint rotates that fan's blades while the hub stays on-axis.

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
N_FANS = 3
W = 1.200          # width  (X) — wider to fit 3 fans
D = 0.300          # depth  (Y); front discharge face at +Y
H = 0.500          # height (Z)
WALL = 0.010       # sheet-metal wall thickness

# Front grille / fan layout. The cabinet box is centered on Z=0 (spanning
# -H/2..+H/2), so the grille centers sit just above mid-height.
FAN_BORE_R = 0.150     # radius of the round opening cut in the front face
FAN_CTR_Z = 0.030      # grille center height (slightly above cabinet middle)
PANEL_W = 0.110        # left control-panel band width (X)

# Three grilles evenly spaced in the area right of the control panel.
USABLE_X0 = -W / 2.0 + PANEL_W
USABLE_X1 = W / 2.0 - 0.020
GRILLE_SPAN = USABLE_X1 - USABLE_X0
FAN_X_POSITIONS = [
    USABLE_X0 + GRILLE_SPAN * (i + 0.5) / N_FANS for i in range(N_FANS)
]

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

    # Radial spokes tying the inner rings out to the outer rim.
    n_spokes = 8
    spoke_w = 0.006
    spoke_len = 2.0 * (rim_r - 0.002)
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
    hub_rear = (
        cq.Workplane("XZ", origin=(center_x, y0, FAN_CTR_Z))
        .circle(0.022)
        .extrude(0.022)   # +extrude on XZ goes -Y, i.e. rearward into the bore
    )
    guard = guard.union(hub_rear)
    return guard


def _build_housing() -> cq.Workplane:
    """Hollow cream cabinet: closed back/sides/top/floor, open interior, three
    round openings on the front face, plus top-cover lip, mounting tabs, and a
    recessed left control panel band."""
    outer = cq.Workplane("XY").box(W, D, H)

    # Hollow it out: remove an inner cavity, leaving WALL-thick skins.
    cavity = cq.Workplane("XY").box(W - 2 * WALL, D - 2 * WALL, H - 2 * WALL)
    shell = outer.cut(cavity)

    # Cut the three circular fan openings fully through the FRONT (+Y) skin.
    for cx in FAN_X_POSITIONS:
        bore = (
            cq.Workplane("XZ", origin=(cx, FRONT_Y - WALL - 0.006, FAN_CTR_Z))
            .circle(FAN_BORE_R)
            .extrude(-(WALL + 0.014))
        )
        shell = shell.cut(bore)

    # Soften the vertical cabinet corners.
    shell = shell.edges("|Z").fillet(0.010)

    # Top cover overhang lip.
    lid = (
        cq.Workplane("XY", origin=(0.0, 0.0, H / 2.0 + 0.002))
        .box(W + 0.018, D + 0.018, 0.014)
        .edges("|Z").fillet(0.006)
    )
    shell = shell.union(lid)

    # Two raised mounting tabs on top of the lid.
    for tx in (-W * 0.28, W * 0.28):
        tab = (
            cq.Workplane("XY", origin=(tx, 0.0, H / 2.0 + 0.012))
            .box(0.075, D - 0.030, 0.018)
            .edges("|Z").fillet(0.004)
        )
        shell = shell.union(tab)

    # Raised left control-panel band.
    panel_cx = -W / 2.0 + PANEL_W / 2.0
    frame = (
        cq.Workplane("XZ", origin=(panel_cx, FRONT_Y - WALL, FAN_CTR_Z))
        .rect(PANEL_W - 0.012, H - 0.060)
        .extrude(-(WALL + 0.006))
    )
    shell = shell.union(frame)

    return shell


def _build_panel_plate() -> cq.Workplane:
    """Inset face plate for the left control panel."""
    panel_cx = -W / 2.0 + PANEL_W / 2.0
    y_start = FRONT_Y - WALL
    plate = (
        cq.Workplane("XZ", origin=(panel_cx, y_start, FAN_CTR_Z))
        .rect(PANEL_W - 0.026, H - 0.080)
        .extrude(-(WALL + 0.010))
    )
    return plate


def _build_badge() -> cq.Workplane:
    """Small rating-label badge on the control panel."""
    panel_cx = -W / 2.0 + PANEL_W / 2.0
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
    """Shared geometry helper: build a fan rotor mesh with consistent params."""
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

    # Three grille guards (off-white plastic) attached to the front face.
    for i in range(N_FANS):
        housing.visual(
            mesh_from_cadquery(_concentric_grille(FAN_X_POSITIONS[i]), f"grille_{i}"),
            material=grille_mat,
            name=f"grille_{i}",
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
    housing.inertial = Inertial.from_geometry(Box((W, D, H)), mass=22.0)

    # ---- Fan rotors (primary mechanism) via loop ---------------------------
    fan_y = FRONT_Y - 0.015          # rotor recessed behind the grille face
    rot_to_y = Origin(rpy=(-math.pi / 2.0, 0.0, 0.0))  # local +Z -> world +Y

    fan_parts = []
    for i in range(N_FANS):
        fan = model.part(f"fan_{i}")
        fan.visual(
            _fan_rotor_mesh(f"fan_{i}_rotor"),
            origin=rot_to_y,
            material=fan_mat,
            name=f"fan_{i}_rotor",
        )
        fan.inertial = Inertial.from_geometry(
            Box((0.28, 0.28, 0.02)), mass=0.45
        )
        fan_parts.append(fan)

    # Joints: each fan spins about the cabinet front normal (+Y).
    for i in range(N_FANS):
        model.articulation(
            f"housing_to_fan_{i}",
            ArticulationType.CONTINUOUS,
            parent=housing,
            child=fan_parts[i],
            origin=Origin(xyz=(FAN_X_POSITIONS[i], fan_y, FAN_CTR_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=20.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    fans = [object_model.get_part(f"fan_{i}") for i in range(N_FANS)]
    joints = [object_model.get_articulation(f"housing_to_fan_{i}") for i in range(N_FANS)]

    # --- Hero parts present -------------------------------------------------
    ctx.check("housing_present", housing is not None, "Expected a housing part.")
    for i in range(N_FANS):
        ctx.check(f"fan_{i}_present", fans[i] is not None, f"Expected fan_{i} part.")

    # --- Cabinet is a tall wide box at real scale ---------------------------
    haabb = ctx.part_world_aabb(housing)
    if haabb is not None:
        (hx0, hy0, hz0), (hx1, hy1, hz1) = haabb
        hw, hd, hh = hx1 - hx0, hy1 - hy0, hz1 - hz0
        ctx.check("cabinet_width", 1.10 <= hw <= 1.30, f"width={hw:.3f}")
        ctx.check("cabinet_depth", 0.26 <= hd <= 0.36, f"depth={hd:.3f}")
        ctx.check("cabinet_height", 0.46 <= hh <= 0.56, f"height={hh:.3f}")

    # --- Three distinct fans, each a real disc of the expected diameter -----
    for i in range(N_FANS):
        fan = fans[i]
        faabb = ctx.part_world_aabb(fan)
        if faabb is None:
            ctx.fail(f"fan_{i}_aabb", "Missing fan AABB.")
            continue
        (fx0, fy0, fz0), (fx1, fy1, fz1) = faabb
        diam = max(fx1 - fx0, fz1 - fz0)
        ctx.check(
            f"fan_{i}_diameter",
            2.0 * (FAN_BORE_R - 0.006) - 0.04 <= diam <= 2.0 * (FAN_BORE_R - 0.006) + 0.04,
            f"diam={diam:.3f}",
        )
        ctx.check(f"fan_{i}_is_disc", (fy1 - fy0) <= 0.06, f"thickness_y={fy1 - fy0:.3f}")

    # --- Fans centered on the grille axes -----------------------------------
    for i in range(N_FANS):
        pos = ctx.part_world_position(fans[i])
        if pos is not None:
            ctx.check(f"fan_{i}_x", abs(pos[0] - FAN_X_POSITIONS[i]) <= 0.01, f"x={pos[0]:.3f}")
            ctx.check(f"fan_{i}_z", abs(pos[2] - FAN_CTR_Z) <= 0.01, f"z={pos[2]:.3f}")

    # --- Fans sit behind the cabinet front plane ----------------------------
    if haabb is not None:
        front_y = haabb[1][1]
        for i in range(N_FANS):
            faabb = ctx.part_world_aabb(fans[i])
            if faabb is not None:
                ctx.check(
                    f"fan_{i}_behind_front",
                    faabb[1][1] <= front_y + 0.005,
                    f"fan_max_y={faabb[1][1]:.3f} front_y={front_y:.3f}",
                )

    # --- Joint types and axes -----------------------------------------------
    for i in range(N_FANS):
        joint = joints[i]
        ctx.check(
            f"fan_{i}_joint_continuous",
            joint.articulation_type == ArticulationType.CONTINUOUS,
            f"type={joint.articulation_type}",
        )
        ax = tuple(joint.axis)
        ctx.check(
            f"fan_{i}_joint_axis_y",
            abs(ax[1]) > 0.99 and abs(ax[0]) < 0.01 and abs(ax[2]) < 0.01,
            f"axis={ax}",
        )

    # --- Actuating each joint spins its fan about the hub -------------------
    for i in range(N_FANS):
        fan = fans[i]
        joint = joints[i]
        with ctx.pose({joint: 0.0}):
            a0 = ctx.part_world_aabb(fan)
        with ctx.pose({joint: math.pi / 2.0}):
            a1 = ctx.part_world_aabb(fan)
        if a0 is not None and a1 is not None:
            moved = (
                abs((a1[0][0]) - (a0[0][0])) > 0.003
                or abs((a1[1][2]) - (a0[1][2])) > 0.003
                or abs((a1[1][0]) - (a0[1][0])) > 0.003
            )
            ctx.check(f"fan_{i}_spins", moved, f"q0={a0} qpi2={a1}")
            c0 = ((a0[0][0] + a0[1][0]) * 0.5, (a0[0][2] + a0[1][2]) * 0.5)
            c1 = ((a1[0][0] + a1[1][0]) * 0.5, (a1[0][2] + a1[1][2]) * 0.5)
            ctx.check(
                f"fan_{i}_axis_fixed",
                abs(c0[0] - c1[0]) <= 0.01 and abs(c0[1] - c1[1]) <= 0.01,
                f"c0={c0} c1={c1}",
            )

    # --- Each impeller is nested inside the hollow cabinet, behind its grille
    for i in range(N_FANS):
        fan = fans[i]
        grille_elem = f"grille_{i}"
        fan_elem = f"fan_{i}_rotor"

        ctx.allow_overlap(
            housing,
            fan,
            elem_a="housing_shell",
            elem_b=fan_elem,
            reason=f"Fan {i} impeller is intentionally nested inside the hollow cabinet; the hollow shell's derived collision encloses the recessed fan.",
        )
        ctx.allow_overlap(
            housing,
            fan,
            elem_a=grille_elem,
            elem_b=fan_elem,
            reason=f"Fan {i} impeller hub seats forward onto the grille's central rear spigot; this captured hub embed is what carries the fan.",
        )

        ctx.expect_within(
            fan,
            housing,
            axes="xz",
            inner_elem=fan_elem,
            outer_elem="housing_shell",
            margin=0.005,
            name=f"fan_{i}_within_cabinet",
        )
        ctx.expect_contact(
            housing,
            fan,
            elem_a=grille_elem,
            elem_b=fan_elem,
            contact_tol=0.002,
            name=f"fan_{i}_supported_by_grille",
        )

    return ctx.report()


object_model = build_object_model()
