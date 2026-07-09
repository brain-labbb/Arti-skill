from __future__ import annotations

# Red three-arm fidget spinner.
# Frame: spinner lies flat in the XY plane, thickness along +Z, centered on the
# origin. The central vertical axis is +Z.
# Construction:
#   - center_cap (ROOT / held part): the central bearing cap you pinch between
#     two fingers. A short silver hub barrel through the middle plus a red
#     convex mushroom pinch DOME on each face (top +Z and bottom -Z), built as
#     true revolved lathe surfaces. This is the part that stays still while the
#     body spins.
#   - spinner_body: flat glossy red tri-lobe plate (3 round lobes at 120deg
#     fused to a central hub), with a circular bearing pocket bored through each
#     lobe. CONTINUOUS spin about the central +Z axis relative to the cap. Its
#     3-lobe silhouette is off-axis, so the spin is detectable by AABB tests.
#   - bearing_0/1/2: each lobe holds a press-fit skateboard-style bearing: a
#     black rubber ring (outer race), a silver inner race ring, and the open
#     center hole. Each spins CONTINUOUSLY about its own lobe axis (+Z). A tiny
#     off-axis silver marker tab on the inner race makes each ring's spin
#     detectable by AABB spin tests.

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
    sample_catmull_rom_spline_2d,
)

# ---- key dimensions (meters) ----
BODY_THICK = 0.011            # tri-lobe plate thickness
HALF_T = BODY_THICK / 2.0

LOBE_R = 0.0150               # radius of each round lobe
LOBE_DIST = 0.0250           # center-to-lobe-center distance
HUB_R = 0.0098               # central hub radius of the body

BEARING_POCKET_R = 0.0116    # bored pocket radius in each lobe
BEARING_OUTER_R = 0.0119     # black rubber ring outer radius (press fit, slight interference)
BEARING_INNER_R = 0.0072     # boundary between black ring and silver race
RACE_INNER_R = 0.0042        # silver inner race inner radius (the hole)
BEARING_THICK = 0.010        # bearing total thickness

# central bearing / cap
CAP_HUB_R = 0.0072           # silver hub barrel radius at the very center
CAP_HUB_LEN = BODY_THICK     # silver hub length matches body thickness (flush at faces)

# Convex mushroom pinch domes (replace flat disc buttons).
DOME_MAX_R = 0.0095           # widest radius of the mushroom dome
DOME_HEIGHT = 0.0050          # dome peak height above its base (convex bulge)
DOME_FLANGE_R = 0.0086        # base flange radius (wider than bore, contacts body face)
DOME_FLANGE_T = 0.0004        # thin flange thickness at the dome base

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
        # Connecting web (tapered bridge) between hub and lobe, built as a hull
        # of two circles via a convex loft of stacked profiles is overkill; use
        # a simple swept rectangle aligned to the arm direction.
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


def _bearing_ring_mesh(name: str):
    """Black rubber outer ring (a thick torus-like ring) for one bearing."""
    # Outer black ring: annulus between BEARING_OUTER_R and BEARING_INNER_R.
    outer = cq.Workplane("XY").circle(BEARING_OUTER_R).extrude(BEARING_THICK)
    hole = cq.Workplane("XY").circle(BEARING_INNER_R).extrude(BEARING_THICK + 0.004).translate(
        (0.0, 0.0, -0.002)
    )
    ring = outer.cut(hole)
    return mesh_from_cadquery(ring, name)


def _bearing_race_mesh(name: str):
    """Silver inner race ring + a tiny off-axis marker tab so spin is visible."""
    race = cq.Workplane("XY").circle(BEARING_INNER_R).extrude(BEARING_THICK)
    hole = cq.Workplane("XY").circle(RACE_INNER_R).extrude(BEARING_THICK + 0.004).translate(
        (0.0, 0.0, -0.002)
    )
    race = race.cut(hole)
    # Tiny off-axis notch/marker on the race rim so an AABB spin test can detect
    # rotation of the otherwise rotationally-symmetric ring.
    marker = (
        cq.Workplane("XY")
        .center(BEARING_INNER_R - 0.0008, 0.0)
        .box(0.0026, 0.0014, BEARING_THICK + 0.0010, centered=(True, True, True))
        .translate((0.0, 0.0, BEARING_THICK / 2.0))
    )
    race = race.union(marker)
    return mesh_from_cadquery(race, name)


def _pinch_dome_profile() -> list[tuple[float, float]]:
    """(radius, z) lathe profile for one mushroom pinch dome.

    A thin base flange wider than the body bore sits on the body face, then the
    dome flares outward from the flange to DOME_MAX_R and curves smoothly back
    to the axis at DOME_HEIGHT.  The flange provides geometric contact with the
    body while the dome overhangs for the mushroom pinch shape.
    """
    # Smooth dome surface via Catmull-Rom spline through control points that
    # trace from the flange top outward to the widest point and back to the peak.
    dome_ctrl: list[tuple[float, float]] = [
        (DOME_FLANGE_R, DOME_FLANGE_T),
        (DOME_MAX_R * 0.92, DOME_FLANGE_T + (DOME_HEIGHT - DOME_FLANGE_T) * 0.15),
        (DOME_MAX_R, DOME_FLANGE_T + (DOME_HEIGHT - DOME_FLANGE_T) * 0.35),
        (DOME_MAX_R * 0.80, DOME_FLANGE_T + (DOME_HEIGHT - DOME_FLANGE_T) * 0.62),
        (DOME_MAX_R * 0.38, DOME_FLANGE_T + (DOME_HEIGHT - DOME_FLANGE_T) * 0.87),
        (0.0, DOME_HEIGHT),
    ]
    dome_curve = sample_catmull_rom_spline_2d(dome_ctrl, samples_per_segment=10)

    profile: list[tuple[float, float]] = [
        (0.0, 0.0),                   # center of flat base (on axis)
        (DOME_FLANGE_R, 0.0),         # outer edge of base flange (wider than bore)
        (DOME_FLANGE_R, DOME_FLANGE_T),  # flange top face
    ]
    profile.extend(dome_curve)
    return profile


def _pinch_dome_mesh(name: str) -> object:
    """Build a lathed mushroom pinch dome mesh. Base at z=0, peak at +Z."""
    profile = _pinch_dome_profile()
    dome = LatheGeometry(profile, segments=48, closed=True)
    return mesh_from_geometry(dome, name)


def _pinch_dome_mesh_flipped(name: str) -> object:
    """Build a lathed mushroom pinch dome mesh flipped so peak points -Z.

    Base at z=0, peak at -DOME_HEIGHT.
    """
    profile = _pinch_dome_profile()
    dome = LatheGeometry(profile, segments=48, closed=True)
    dome.rotate_x(math.pi)
    return mesh_from_geometry(dome, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fidget_spinner")

    red = model.material("glossy_red", rgba=(0.82, 0.07, 0.09, 1.0))
    black = model.material("rubber_black", rgba=(0.06, 0.06, 0.07, 1.0))
    silver = model.material("silver_steel", rgba=(0.78, 0.80, 0.83, 1.0))

    # ---- center_cap (ROOT / held part) ----
    # Silver hub barrel down the central axis + a red cap button on each face.
    cap = model.part("center_cap")

    hub_barrel = CylinderGeometry(CAP_HUB_R, CAP_HUB_LEN, radial_segments=48)
    cap.visual(
        mesh_from_geometry(hub_barrel, "cap_hub"),
        # Center hub on the body so it protrudes past both body faces.
        # Body frame is at HALF_T (joint origin); body extends HALF_T above that.
        origin=Origin(xyz=(0.0, 0.0, BODY_THICK)),
        material=silver,
        name="cap_hub",
    )
    # Convex mushroom pinch domes on each face (the part you pinch).
    # Body frame is at HALF_T (joint origin); body top face is at HALF_T + BODY_THICK,
    # body bottom face is at HALF_T.  Domes sit on these faces with flanges that
    # contact the body surface outside the bore.
    body_top_z = HALF_T + BODY_THICK
    body_bot_z = HALF_T
    cap.visual(
        _pinch_dome_mesh("cap_dome_top"),
        origin=Origin(xyz=(0.0, 0.0, body_top_z)),
        material=red,
        name="cap_dome_top",
    )
    cap.visual(
        _pinch_dome_mesh_flipped("cap_dome_bottom"),
        origin=Origin(xyz=(0.0, 0.0, body_bot_z)),
        material=red,
        name="cap_dome_bottom",
    )
    cap.inertial = Inertial.from_geometry(
        Cylinder(radius=DOME_MAX_R, length=CAP_HUB_LEN + 2.0 * DOME_HEIGHT),
        mass=0.004,
        origin=Origin(xyz=(0.0, 0.0, BODY_THICK)),
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

    # ---- 3 ring bearings, one per lobe, each spinning about its lobe axis ----
    bearing_z = HALF_T  # center the bearing in the plate thickness
    for i, ang in enumerate(LOBE_ANGLES):
        cx = LOBE_DIST * math.cos(ang)
        cy = LOBE_DIST * math.sin(ang)
        name = f"bearing_{i}"
        b = model.part(name)
        # Black rubber outer ring.
        b.visual(
            _bearing_ring_mesh(f"{name}_ring"),
            origin=Origin(xyz=(0.0, 0.0, -BEARING_THICK / 2.0)),
            material=black,
            name=f"{name}_ring",
        )
        # Silver inner race + marker.
        b.visual(
            _bearing_race_mesh(f"{name}_race"),
            origin=Origin(xyz=(0.0, 0.0, -BEARING_THICK / 2.0)),
            material=silver,
            name=f"{name}_race",
        )
        b.inertial = Inertial.from_geometry(
            Cylinder(radius=BEARING_OUTER_R, length=BEARING_THICK),
            mass=0.006,
        )
        # Bearing spins about the lobe's own vertical axis, relative to the body.
        model.articulation(
            f"body_to_{name}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=b,
            origin=Origin(xyz=(cx, cy, bearing_z)),
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
    bearings = [object_model.get_part(f"bearing_{i}") for i in range(3)]
    spin = object_model.get_articulation("cap_to_body")

    # --- Cap is on the central axis and is the held root. ---
    cap_pos = ctx.part_world_position(cap)
    ctx.check(
        "center cap on the central axis",
        cap_pos is not None and abs(cap_pos[0]) < 1e-4 and abs(cap_pos[1]) < 1e-4,
        details=f"cap origin={cap_pos}",
    )

    # --- Three lobes are arranged at ~120deg around the center. ---
    angs = []
    for i, b in enumerate(bearings):
        p = ctx.part_world_position(b)
        r = math.hypot(p[0], p[1])
        ctx.check(
            f"bearing_{i} sits out on a lobe",
            abs(r - LOBE_DIST) < 0.002,
            details=f"radius={r}",
        )
        angs.append(math.atan2(p[1], p[0]))
    # Pairwise angular separations should be ~120deg.
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
    # A lobe swings around: track a single lobe (bearing_0) world position.
    rest = ctx.part_world_position(bearings[0])
    with ctx.pose({spin: math.pi / 3.0}):
        turned = ctx.part_world_position(bearings[0])
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

    # --- Each ring bearing spins about its own lobe axis (marker reorients). ---
    for i in range(3):
        joint = object_model.get_articulation(f"body_to_bearing_{i}")
        ext0 = _ext(ctx.part_element_world_aabb(bearings[i], elem=f"bearing_{i}_race"))
        with ctx.pose({joint: math.pi / 2.0}):
            ext90 = _ext(ctx.part_element_world_aabb(bearings[i], elem=f"bearing_{i}_race"))
        # The off-axis marker makes the race AABB swap its x/y extents on a
        # quarter turn, proving the ring actually spins.
        ctx.check(
            f"bearing_{i} ring spins about its lobe axis",
            abs(ext0[0] - ext90[1]) < 1e-3 and abs(ext0[0] - ext0[1]) > 1e-4,
            details=f"ext0={ext0}, ext90={ext90}",
        )

    # --- Each bearing is seated in its lobe pocket (intentional press-fit). ---
    for i in range(3):
        ctx.allow_overlap(
            bearings[i],
            body,
            elem_a=f"bearing_{i}_ring",
            elem_b="tri_lobe_body",
            reason="Black rubber bearing ring is press-fit into the lobe pocket.",
        )
        ctx.expect_overlap(
            bearings[i], body, axes="z", min_overlap=0.004,
            name=f"bearing_{i} seated in lobe pocket (thickness)",
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

    # --- Mushroom pinch domes: convex lathe domes on both faces. ---
    body_top_z = HALF_T + BODY_THICK   # body top face in world
    body_bot_z = HALF_T                # body bottom face in world
    dome_top_aabb = ctx.part_element_world_aabb(cap, elem="cap_dome_top")
    dome_bot_aabb = ctx.part_element_world_aabb(cap, elem="cap_dome_bottom")
    ctx.check(
        "top pinch dome exists and has 3D extent",
        dome_top_aabb is not None,
        details=f"aabb={dome_top_aabb}",
    )
    ctx.check(
        "bottom pinch dome exists and has 3D extent",
        dome_bot_aabb is not None,
        details=f"aabb={dome_bot_aabb}",
    )
    if dome_top_aabb is not None and dome_bot_aabb is not None:
        top_min, top_max = dome_top_aabb
        bot_min, bot_max = dome_bot_aabb
        # Top dome peak is above the body top face.
        ctx.check(
            "top dome peak protrudes above body top face",
            top_max[2] > body_top_z + 0.002,
            details=f"dome peak z={top_max[2]:.5f}, body top={body_top_z:.5f}",
        )
        # Top dome base is above the body top face (no embedment).
        ctx.check(
            "top dome base sits above body top face",
            top_min[2] >= body_top_z - 0.0005,
            details=f"dome base z={top_min[2]:.5f}, body top={body_top_z:.5f}",
        )
        # Bottom dome peak is below the body bottom face.
        ctx.check(
            "bottom dome peak protrudes below body bottom face",
            bot_min[2] < body_bot_z - 0.002,
            details=f"dome peak z={bot_min[2]:.5f}, body bot={body_bot_z:.5f}",
        )
        # Bottom dome base is below the body bottom face (no embedment).
        ctx.check(
            "bottom dome base sits below body bottom face",
            bot_max[2] <= body_bot_z + 0.0005,
            details=f"dome base z={bot_max[2]:.5f}, body bot={body_bot_z:.5f}",
        )
        # Both domes are centered on the spin axis (XY origin).
        top_cx = (top_min[0] + top_max[0]) / 2.0
        top_cy = (top_min[1] + top_max[1]) / 2.0
        bot_cx = (bot_min[0] + bot_max[0]) / 2.0
        bot_cy = (bot_min[1] + bot_max[1]) / 2.0
        ctx.check(
            "pinch domes are centered on the spin axis",
            abs(top_cx) < 0.001 and abs(top_cy) < 0.001
            and abs(bot_cx) < 0.001 and abs(bot_cy) < 0.001,
            details=f"top_center=({top_cx:.5f},{top_cy:.5f}) "
                    f"bot_center=({bot_cx:.5f},{bot_cy:.5f})",
        )
        # Dome has meaningful XY width (convex lathe, not a flat disc or needle).
        top_dx = top_max[0] - top_min[0]
        ctx.check(
            "top dome has convex lathe width",
            top_dx > 2.0 * DOME_FLANGE_R and top_dx <= 2.0 * DOME_MAX_R + 0.001,
            details=f"top dome XY width={top_dx:.5f}",
        )

    return ctx.report()


object_model = build_object_model()
