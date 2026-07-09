from __future__ import annotations

# Red three-arm fidget spinner with hex brass nut weights.
# Frame: spinner lies flat in the XY plane, thickness along +Z, centered on the
# origin. The central vertical axis is +Z.
# Construction:
#   - center_cap (ROOT / held part): the central bearing cap you pinch between
#     two fingers. A short silver hub barrel through the middle plus a red CAP
#     button disc on each face (top +Z and bottom -Z). This is the part that
#     stays still while the body spins.
#   - spinner_body: flat glossy red tri-lobe plate (3 round lobes at 120deg
#     fused to a central hub), with a circular pocket bored through each lobe.
#     CONTINUOUS spin about the central +Z axis relative to the cap. Its 3-lobe
#     silhouette is off-axis, so the spin is detectable by AABB tests.
#   - hex_weight_0/1/2: each lobe holds a machined brass hex nut weight — a
#     regular hexagonal prism, flat top and bottom, with a small round center
#     bore. Each spins CONTINUOUSLY about its own lobe axis (+Z) relative to the
#     body. The six flat faces make the hex AABB asymmetric, so spin is
#     detectable by AABB tests.

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

BEARING_POCKET_R = 0.0116     # bored pocket radius in each lobe (circular through-hole)

# Hex nut weight dimensions
HEX_R = 0.0110                # circumscribed radius of regular hex (fits inside pocket)
HEX_THICK = 0.010             # hex prism thickness (slightly less than body)
HEX_HOLE_R = 0.003            # small center bore radius

# Lobe axle pin (bolt through hex center hole, integrated into body mesh)
PIN_R = 0.0032                # pin radius (slight interference with hex hole for contact)

# central bearing / cap
CAP_HUB_R = 0.0072            # silver hub barrel radius at the very center
CAP_DISC_R = 0.0090           # red cap button radius (each face)
CAP_DISC_T = 0.0020           # red cap button thickness (protrudes past face)
CAP_HUB_LEN = 0.014           # silver hub length (slightly taller than body)

LOBE_ANGLES = (math.pi / 2.0, math.pi / 2.0 + 2.0 * math.pi / 3.0,
               math.pi / 2.0 + 4.0 * math.pi / 3.0)


def _tri_lobe_body() -> cq.Workplane:
    """Flat red tri-lobe plate: central hub disc fused with 3 lobe discs and
    the connecting webs, then a circular pocket bored through each lobe and a
    hub bore through the center for the cap. Each lobe pocket retains a thin
    cross-spider bridge that connects the central axle pin to the pocket wall."""
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

    # Bore a circular pocket through each lobe, leaving a thin cross-spider
    # bridge (pin + 2 perpendicular ribs connecting pin to pocket wall).
    spoke_w = 0.0012  # spoke width (thin cage bridge)
    for ang in LOBE_ANGLES:
        cx = LOBE_DIST * math.cos(ang)
        cy = LOBE_DIST * math.sin(ang)
        # Cut the pocket (slightly oversized to clear the body top/bottom).
        pocket = (
            cq.Workplane("XY")
            .center(cx, cy)
            .circle(BEARING_POCKET_R)
            .extrude(BODY_THICK + 0.004)
            .translate((0.0, 0.0, -0.002))
        )
        body = body.cut(pocket)
        # Add central axle pin (cylinder at pocket center).
        pin = (
            cq.Workplane("XY")
            .center(cx, cy)
            .circle(PIN_R)
            .extrude(BODY_THICK)
        )
        body = body.union(pin)
        # Add two perpendicular cross-spokes from center to pocket wall + slight
        # overlap into body wall for a connected union.
        spoke_half_len = (BEARING_POCKET_R + 0.0008) / 2.0  # extends slightly past pocket wall
        for sa in (0.0, math.pi / 2.0):
            mid_r = spoke_half_len
            sx = cx + mid_r * math.cos(sa)
            sy = cy + mid_r * math.sin(sa)
            spoke = (
                cq.Workplane("XY")
                .center(sx, sy)
                .transformed(rotate=(0.0, 0.0, math.degrees(sa)))
                .rect(spoke_half_len * 2.0, spoke_w)
                .extrude(BODY_THICK)
            )
            body = body.union(spoke)

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


def _hex_nut_mesh(name: str):
    """Regular hexagonal brass prism with a small center bore.
    Six flat machined faces, flat top/bottom, round through-hole at center."""
    # Build hex vertices (first vertex along +X for consistent orientation).
    pts = [
        (HEX_R * math.cos(i * 2.0 * math.pi / 6.0),
         HEX_R * math.sin(i * 2.0 * math.pi / 6.0))
        for i in range(6)
    ]
    hex_prism = (
        cq.Workplane("XY")
        .moveTo(pts[0][0], pts[0][1])
        .polyline(pts[1:])
        .close()
        .extrude(HEX_THICK)
    )
    # Bore the center hole through the hex.
    hole = (
        cq.Workplane("XY")
        .circle(HEX_HOLE_R)
        .extrude(HEX_THICK + 0.004)
        .translate((0.0, 0.0, -0.002))
    )
    nut = hex_prism.cut(hole)
    # Small chamfer on top and bottom edges for a machined finish.
    nut = nut.edges("|Z").chamfer(0.0006)
    return mesh_from_cadquery(nut, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fidget_spinner")

    red = model.material("glossy_red", rgba=(0.82, 0.07, 0.09, 1.0))
    brass = model.material("machined_brass", rgba=(0.76, 0.60, 0.12, 1.0))
    silver = model.material("silver_steel", rgba=(0.78, 0.80, 0.83, 1.0))

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

    # ---- 3 hex brass nut weights, one per lobe, each spinning about its lobe axis ----
    for i, ang in enumerate(LOBE_ANGLES):
        cx = LOBE_DIST * math.cos(ang)
        cy = LOBE_DIST * math.sin(ang)
        name = f"hex_weight_{i}"
        hw = model.part(name)
        # Brass hex nut visual (centered vertically via origin offset).
        hw.visual(
            _hex_nut_mesh(f"{name}_nut"),
            origin=Origin(xyz=(0.0, 0.0, -HEX_THICK / 2.0)),
            material=brass,
            name=f"{name}_nut",
        )
        hw.inertial = Inertial.from_geometry(
            Cylinder(radius=HEX_R, length=HEX_THICK),
            mass=0.008,
        )
        # Hex weight spins about the lobe's own vertical axis, relative to the body.
        model.articulation(
            f"body_to_{name}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=hw,
            origin=Origin(xyz=(cx, cy, HALF_T)),
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
    hex_weights = [object_model.get_part(f"hex_weight_{i}") for i in range(3)]
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
    for i, hw in enumerate(hex_weights):
        p = ctx.part_world_position(hw)
        r = math.hypot(p[0], p[1])
        ctx.check(
            f"hex_weight_{i} sits out on a lobe",
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
    rest = ctx.part_world_position(hex_weights[0])
    with ctx.pose({spin: math.pi / 3.0}):
        turned = ctx.part_world_position(hex_weights[0])
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

    # --- Each hex weight spins about its own lobe axis (AABB changes due to hex shape). ---
    for i in range(3):
        joint = object_model.get_articulation(f"body_to_hex_weight_{i}")
        ext0 = _ext(ctx.part_element_world_aabb(hex_weights[i], elem=f"hex_weight_{i}_nut"))
        with ctx.pose({joint: math.pi / 2.0}):
            ext90 = _ext(ctx.part_element_world_aabb(hex_weights[i], elem=f"hex_weight_{i}_nut"))
        # The hex shape has different X and Y extents at rest (vertex-aligned).
        ctx.check(
            f"hex_weight_{i} has asymmetric AABB (flat faces visible)",
            abs(ext0[0] - ext0[1]) > 1e-4,
            details=f"ext0={ext0}",
        )
        # After 90deg rotation, X and Y extents swap (hex symmetry).
        ctx.check(
            f"hex_weight_{i} spin swaps X/Y extents (detectable rotation)",
            abs(ext0[0] - ext90[1]) < 1e-3,
            details=f"ext0={ext0}, ext90={ext90}",
        )

    # --- Each hex weight is seated inside its lobe pocket, spinning on the axle pin. ---
    for i in range(3):
        # The integrated axle pin (part of tri_lobe_body mesh) passes through the
        # hex center bore with slight interference — intentional local overlap.
        ctx.allow_overlap(
            body,
            hex_weights[i],
            elem_a="tri_lobe_body",
            elem_b=f"hex_weight_{i}_nut",
            reason=f"Integrated axle pin and spider bridge pass through hex_weight_{i} center bore.",
        )
        ctx.expect_overlap(
            hex_weights[i], body, axes="z", min_overlap=0.004,
            name=f"hex_weight_{i} seated in lobe pocket (thickness)",
        )
        ctx.expect_within(
            hex_weights[i], body, axes="xy",
            inner_elem=f"hex_weight_{i}_nut",
            outer_elem="tri_lobe_body",
            margin=0.002,
            name=f"hex_weight_{i} fits within lobe pocket footprint",
        )
        # Prove the pin/spider contacts the hex bore (they overlap in XY footprint).
        ctx.expect_overlap(
            body, hex_weights[i], axes="xy",
            elem_a="tri_lobe_body",
            elem_b=f"hex_weight_{i}_nut",
            min_overlap=0.002,
            name=f"lobe spider bridge contacts hex_weight_{i} bore",
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
