from __future__ import annotations

# Red three-arm fidget spinner with domed metal weight caps.
# Frame: spinner lies flat in the XY plane, thickness along +Z, centered on the
# origin. The central vertical axis is +Z.
# Construction:
#   - center_cap (ROOT / held part): the central bearing cap you pinch between
#     two fingers. A short silver hub barrel through the middle plus a red CAP
#     button disc on each face (top +Z and bottom -Z).
#   - spinner_body: flat glossy red tri-lobe plate (3 round lobes at 120deg
#     fused to a central hub), with a circular pocket bored through each lobe.
#     CONTINUOUS spin about the central +Z axis relative to the cap.
#   - weight_0/1/2: each lobe holds a press-fit domed metal weight cap: a
#     polished biconvex metal lens that fills and caps the bearing pocket,
#     replacing the parent's open skateboard bearing. Each spins CONTINUOUSLY
#     about its own lobe axis (+Z). A flat facet cut through one side of each
#     weight breaks rotational symmetry and makes spin detectable by AABB tests.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

# ---- key dimensions (meters) ----
BODY_THICK = 0.011            # tri-lobe plate thickness
HALF_T = BODY_THICK / 2.0

LOBE_R = 0.0150               # radius of each round lobe
LOBE_DIST = 0.0250            # center-to-lobe-center distance
HUB_R = 0.0098                # central hub radius of the body

BEARING_POCKET_R = 0.0116     # bored pocket radius in each lobe

# Dome weight cap dimensions
DOME_R = 0.0119               # dome/plug base radius (press-fit, slight interference with pocket)
DOME_SPHERE_R = 0.0160        # sphere radius for dome curvature

# central bearing / cap
CAP_HUB_R = 0.0072            # silver hub barrel radius at the very center
CAP_DISC_R = 0.0090           # red cap button radius (each face)
CAP_DISC_T = 0.0020           # red cap button thickness (protrudes past face)
CAP_HUB_LEN = 0.014           # silver hub length (slightly taller than body)

LOBE_ANGLES = (math.pi / 2.0, math.pi / 2.0 + 2.0 * math.pi / 3.0,
               math.pi / 2.0 + 4.0 * math.pi / 3.0)


def _tri_lobe_body() -> cq.Workplane:
    """Flat red tri-lobe plate: central hub disc fused with 3 lobe discs and
    the connecting webs, then a bearing pocket bored through each lobe and a
    hub bore through the center for the cap."""
    # Central hub disc.
    body = cq.Workplane("XY").circle(HUB_R).extrude(BODY_THICK)

    for ang in LOBE_ANGLES:
        cx = LOBE_DIST * math.cos(ang)
        cy = LOBE_DIST * math.sin(ang)
        # Lobe disc.
        lobe = cq.Workplane("XY").center(cx, cy).circle(LOBE_R).extrude(BODY_THICK)
        body = body.union(lobe)
        # Connecting web (tapered bridge) between hub and lobe.
        web_len = LOBE_DIST
        web = (
            cq.Workplane("XY")
            .center(cx / 2.0, cy / 2.0)
            .transformed(rotate=(0.0, 0.0, math.degrees(ang)))
            .rect(web_len, 2.0 * (HUB_R * 0.92))
            .extrude(BODY_THICK)
        )
        body = body.union(web)

    # Bore a bearing pocket through each lobe.
    for ang in LOBE_ANGLES:
        cx = LOBE_DIST * math.cos(ang)
        cy = LOBE_DIST * math.sin(ang)
        pocket = (
            cq.Workplane("XY")
            .center(cx, cy)
            .circle(BEARING_POCKET_R)
            .extrude(BODY_THICK + 0.004)
            .translate((0.0, 0.0, -0.002))
        )
        body = body.cut(pocket)

    # Bore the central hub hole for the cap barrel.
    center_bore = (
        cq.Workplane("XY")
        .circle(CAP_HUB_R + 0.0004)
        .extrude(BODY_THICK + 0.004)
        .translate((0.0, 0.0, -0.002))
    )
    body = body.cut(center_bore)

    # Round the flat top/bottom edges of the lobes for a glossy molded look.
    body = body.edges("|Z").fillet(0.0012)
    return body


def _dome_weight_mesh(name: str):
    """Polished biconvex metal dome weight cap for one lobe pocket.

    A solid biconvex lens shape: cylindrical midsection filling the pocket,
    with spherical dome caps protruding above and below the body. A flat
    facet is cut through one side of the entire weight to break rotational
    symmetry and make spin detectable by AABB tests.
    """
    h_offset = math.sqrt(DOME_SPHERE_R**2 - DOME_R**2)
    dome_h = DOME_SPHERE_R - h_offset

    # Cylindrical plug fills the pocket from z=-HALF_T to z=+HALF_T.
    plug = (
        cq.Workplane("XY")
        .circle(DOME_R)
        .extrude(BODY_THICK)
        .translate((0.0, 0.0, -HALF_T))
    )

    # Top dome: spherical cap above z = +HALF_T.
    # Sphere center is at z = HALF_T - h_offset so the cross-section at
    # z = HALF_T has radius exactly DOME_R.
    sphere_cz_top = HALF_T - h_offset
    sphere_top = cq.Workplane("XY").sphere(DOME_SPHERE_R).translate(
        (0.0, 0.0, sphere_cz_top)
    )
    # Cut everything below z = HALF_T to isolate the dome cap.
    cut_top = (
        cq.Workplane("XY")
        .box(DOME_SPHERE_R * 4, DOME_SPHERE_R * 4, DOME_SPHERE_R * 2)
        .translate((0.0, 0.0, HALF_T - DOME_SPHERE_R))
    )
    dome_top = sphere_top.cut(cut_top)

    # Bottom dome: spherical cap below z = -HALF_T (mirrored).
    sphere_cz_bot = -HALF_T + h_offset
    sphere_bot = cq.Workplane("XY").sphere(DOME_SPHERE_R).translate(
        (0.0, 0.0, sphere_cz_bot)
    )
    cut_bot = (
        cq.Workplane("XY")
        .box(DOME_SPHERE_R * 4, DOME_SPHERE_R * 4, DOME_SPHERE_R * 2)
        .translate((0.0, 0.0, -HALF_T + DOME_SPHERE_R))
    )
    dome_bot = sphere_bot.cut(cut_bot)

    result = plug.union(dome_top).union(dome_bot)

    # Off-axis flat facet: cut a D-shaped flat through the entire weight
    # on one side. This breaks rotational symmetry so the AABB changes
    # when the weight spins, making rotation detectable.
    facet_x = DOME_R - 0.003  # flat surface at this x coordinate
    cut_w = DOME_R + 0.006 - facet_x  # wide enough to clear everything
    total_z = BODY_THICK + 2.0 * dome_h + 0.01
    facet_cutter = (
        cq.Workplane("XY")
        .box(cut_w, DOME_SPHERE_R * 4, total_z)
        .translate((facet_x + cut_w / 2.0, 0.0, 0.0))
    )
    result = result.cut(facet_cutter)

    return mesh_from_cadquery(result, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fidget_spinner")

    red = model.material("glossy_red", rgba=(0.82, 0.07, 0.09, 1.0))
    silver = model.material("silver_steel", rgba=(0.78, 0.80, 0.83, 1.0))
    chrome = model.material("polished_chrome", rgba=(0.85, 0.87, 0.90, 1.0))

    # ---- center_cap (ROOT / held part) ----
    # Silver hub barrel down the central axis + a red cap button on each face.
    cap = model.part("center_cap")

    hub_barrel = CylinderGeometry(CAP_HUB_R, CAP_HUB_LEN, radial_segments=48)
    cap.visual(
        mesh_from_geometry(hub_barrel, "cap_hub"),
        origin=Origin(xyz=(0.0, 0.0, HALF_T)),
        material=silver,
        name="cap_hub",
    )
    # Red cap buttons proud of each face (the part you pinch).
    cap.visual(
        Cylinder(radius=CAP_DISC_R, length=CAP_DISC_T),
        origin=Origin(xyz=(0.0, 0.0, BODY_THICK + CAP_DISC_T / 2.0)),
        material=red,
        name="cap_button_top",
    )
    cap.visual(
        Cylinder(radius=CAP_DISC_R, length=CAP_DISC_T),
        origin=Origin(xyz=(0.0, 0.0, -CAP_DISC_T / 2.0)),
        material=red,
        name="cap_button_bottom",
    )
    cap.inertial = Inertial.from_geometry(
        Cylinder(radius=CAP_DISC_R, length=CAP_HUB_LEN + 2.0 * CAP_DISC_T),
        mass=0.004,
        origin=Origin(xyz=(0.0, 0.0, HALF_T)),
    )

    # ---- spinner_body: tri-lobe red plate, spins about +Z relative to cap ----
    body = model.part("spinner_body")
    body.visual(
        mesh_from_cadquery(_tri_lobe_body(), "tri_lobe_body"),
        material=red,
        name="tri_lobe_body",
    )
    body.inertial = Inertial.from_geometry(
        Cylinder(radius=LOBE_DIST + LOBE_R, length=BODY_THICK),
        mass=0.045,
        origin=Origin(xyz=(0.0, 0.0, HALF_T)),
    )
    model.articulation(
        "cap_to_body",
        ArticulationType.CONTINUOUS,
        parent=cap,
        child=body,
        origin=Origin(xyz=(0.0, 0.0, HALF_T)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.2, velocity=80.0),
    )

    # ---- 3 domed weight caps, one per lobe, each spinning about its lobe axis ----
    weight_z = HALF_T  # joint at plate mid-thickness (lobe pocket center)
    for i, ang in enumerate(LOBE_ANGLES):
        cx = LOBE_DIST * math.cos(ang)
        cy = LOBE_DIST * math.sin(ang)
        name = f"weight_{i}"
        w = model.part(name)
        # Domed weight visual: biconvex polished metal lens with flat facet.
        w.visual(
            _dome_weight_mesh(f"{name}_dome"),
            material=chrome,
            name=f"{name}_dome",
        )
        w.inertial = Inertial.from_geometry(
            Cylinder(radius=DOME_R, length=BODY_THICK + 0.008),
            mass=0.008,
        )
        # Weight spins about the lobe's own vertical axis, relative to the body.
        model.articulation(
            f"body_to_{name}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=w,
            origin=Origin(xyz=(cx, cy, weight_z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.1, velocity=80.0),
        )

    return model


def _ext(aabb):
    mn, mx = aabb
    return (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cap = object_model.get_part("center_cap")
    body = object_model.get_part("spinner_body")
    weights = [object_model.get_part(f"weight_{i}") for i in range(3)]
    spin = object_model.get_articulation("cap_to_body")

    # --- Cap is on the central axis and is the held root. ---
    cap_pos = ctx.part_world_position(cap)
    ctx.check(
        "center cap on the central axis",
        cap_pos is not None and abs(cap_pos[0]) < 1e-4 and abs(cap_pos[1]) < 1e-4,
        details=f"cap origin={cap_pos}",
    )

    # --- Three weights are arranged at ~120deg around the center. ---
    angs = []
    for i, w in enumerate(weights):
        p = ctx.part_world_position(w)
        r = math.hypot(p[0], p[1])
        ctx.check(
            f"weight_{i} sits out on a lobe",
            abs(r - LOBE_DIST) < 0.002,
            details=f"radius={r}",
        )
        angs.append(math.atan2(p[1], p[0]))

    def sep(a, b):
        d = abs((a - b) % (2.0 * math.pi))
        return min(d, 2.0 * math.pi - d)

    ctx.check(
        "three lobes spaced ~120deg apart",
        all(abs(sep(angs[i], angs[j]) - 2.0 * math.pi / 3.0) < 0.10
            for i, j in ((0, 1), (1, 2), (2, 0))),
        details=f"angles={[math.degrees(a) for a in angs]}",
    )

    # --- The tri-lobe body spins about the center relative to the cap. ---
    rest = ctx.part_world_position(weights[0])
    with ctx.pose({spin: math.pi / 3.0}):
        turned = ctx.part_world_position(weights[0])
    moved = math.hypot(turned[0] - rest[0], turned[1] - rest[1])
    ctx.check(
        "spinning the body swings a lobe around the center",
        moved > 0.010,
        details=f"rest={rest}, turned={turned}, moved={moved}",
    )
    # Cap must NOT move when the body spins (it is the held root).
    with ctx.pose({spin: math.pi / 3.0}):
        cap_turned = ctx.part_world_position(cap)
    ctx.check(
        "center cap stays put while body spins",
        cap_turned is not None and math.hypot(cap_turned[0], cap_turned[1]) < 1e-4,
        details=f"cap_turned={cap_turned}",
    )

    # --- Each domed weight spins about its own lobe axis (facet reorients). ---
    h_offset = math.sqrt(DOME_SPHERE_R**2 - DOME_R**2)
    dome_h = DOME_SPHERE_R - h_offset
    for i in range(3):
        joint = object_model.get_articulation(f"body_to_weight_{i}")
        ext0 = _ext(ctx.part_element_world_aabb(weights[i], elem=f"weight_{i}_dome"))
        with ctx.pose({joint: math.pi / 2.0}):
            ext90 = _ext(ctx.part_element_world_aabb(weights[i], elem=f"weight_{i}_dome"))
        # The off-axis flat facet makes the weight AABB swap its x/y extents
        # on a quarter turn, proving the weight actually spins.
        ctx.check(
            f"weight_{i} dome spins about its lobe axis",
            abs(ext0[0] - ext90[1]) < 1e-3 and abs(ext0[0] - ext0[1]) > 1e-4,
            details=f"ext0={ext0}, ext90={ext90}",
        )

    # --- Each dome weight protrudes above the body (convex dome visible). ---
    for i in range(3):
        weight_aabb = ctx.part_world_aabb(weights[i])
        body_aabb = ctx.part_world_aabb(body)
        ctx.check(
            f"weight_{i} dome protrudes above body surface",
            weight_aabb is not None and body_aabb is not None
            and weight_aabb[1][2] > body_aabb[1][2] + 0.001,
            details=f"weight_top={weight_aabb[1][2] if weight_aabb else None}, "
                    f"body_top={body_aabb[1][2] if body_aabb else None}",
        )

    # --- Each dome weight is seated in its lobe pocket (intentional press-fit). ---
    for i in range(3):
        ctx.allow_overlap(
            weights[i],
            body,
            elem_a=f"weight_{i}_dome",
            elem_b="tri_lobe_body",
            reason="Domed weight cap is press-fit into the lobe pocket.",
        )
        ctx.expect_overlap(
            weights[i], body, axes="z", min_overlap=0.004,
            name=f"weight_{i} seated in lobe pocket (thickness)",
        )

    # --- Cap hub seats through the central body bore (intentional). ---
    ctx.allow_overlap(
        cap,
        body,
        elem_a="cap_hub",
        elem_b="tri_lobe_body",
        reason="Silver cap hub passes through the central bore of the spinner body.",
    )
    ctx.expect_overlap(
        cap, body, axes="z", min_overlap=0.006,
        name="cap hub passes through body center",
    )

    return ctx.report()


object_model = build_object_model()
