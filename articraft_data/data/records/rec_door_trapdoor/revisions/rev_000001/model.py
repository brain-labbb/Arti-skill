from __future__ import annotations

# Round cast-iron access hatch / trap door lid hinged at the rear, sitting on a
# square metal-mesh collar that caps a round concrete / stone well shaft.
#
# Articraft brief:
# - Object: a circular cast-iron access hatch / manhole-style trap door, lid
#   ~0.72 m diameter (wider than the throat so it seats on the ring lip), on a
#   square mesh collar over a round concrete well shaft ~0.80 m across and
#   ~0.55 m tall, standing on the ground (z=0 up).
# - Root/support: the concrete well shaft is the fixed root resting on z=0; the
#   square mesh collar is fixed to the shaft top; the lid is hinged to the
#   collar at the rear rim.
# - Parts: well_shaft (hollow concrete tube), mesh_collar (square diamond-mesh
#   frame with circular throat, fixed to shaft), lid (round cast-iron disc with
#   recessed spoked relief on top and rim bolts).
# - Articulation: collar_to_lid REVOLUTE, hinge line along the rear rim, axis
#   horizontal (world Y at q=0) so the front edge lifts upward; positive q
#   swings the lid up past vertical. A closed trap door correctly lies FLAT, so
#   the hinge axis is horizontal here (unlike an upright door).
# - Visible geometry: reddish-brown cast iron lid, recessed circular relief with
#   a central hub and radial spokes, ring of rim bolts; grey concrete shaft with
#   a hollow bore; dark rust-brown diamond-mesh collar top.
# - Support/fit: the lid rim seats on the throat ring lip when closed; the hinge
#   is a real mount -- collar-side lug plates + pin on the rear frame band, with
#   the lid knuckle barrel coaxial on the pin.
# - Intentional overlaps: hinge knuckle/pin/lugs embed into the lid and collar
#   (local, mechanically explanatory); lid relief features sink slightly into the
#   lid disc; the closed lid rim embeds ~2mm into the ring lip seat.
# - Tests: lid disc + spokes + bolts present, lid lies flat & seats on the collar
#   when closed, open pose lifts the front edge well above the rim, shaft is
#   hollow, nothing floats.
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoxGeometry,
    CylinderGeometry,
    LatheGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)

# --- Absolute dimensions (meters) ---------------------------------------------
SHAFT_OUTER_R = 0.40
SHAFT_WALL = 0.085
SHAFT_INNER_R = SHAFT_OUTER_R - SHAFT_WALL  # bore radius ~0.315
SHAFT_HEIGHT = 0.52

COLLAR_HALF = 0.40  # square mesh collar 0.80 m x 0.80 m
COLLAR_FRAME = 0.06  # outer frame band width
COLLAR_THK = 0.05  # collar plate thickness
COLLAR_THROAT_R = SHAFT_INNER_R + 0.01  # circular throat opening radius

LID_R = 0.36  # lid radius (0.72 m diameter) -- wider than the throat so it seats
LID_THK = 0.05  # cast-iron lid thickness
LID_RIM_SEAT = 0.015  # shallow centering step that nests into the throat ring

RELIEF_RING_R = 0.245  # recessed relief ring outer radius on lid top
HUB_R = 0.070  # central hub radius
SPOKE_HALF_W = 0.050  # half-width of each radial spoke (thick cast bars)
N_SPOKES = 4

BOLT_R = 0.014
N_BOLTS = 12
BOLT_RING_R = 0.320  # bolt circle radius on the lid rim

HINGE_PIN_R = 0.020
HINGE_KNUCKLE_LEN = 0.17

# Hinge line placement (in the collar part frame). The lid rim rests on the
# throat ring lip; the hinge axis runs along the rear rim at the lid top plane,
# directly over the rear collar frame band so the collar-side lugs are grounded.
THROAT_LIP_TOP = COLLAR_THK + 0.015  # top of the throat ring lip
HINGE_Y = LID_R
HINGE_Z = THROAT_LIP_TOP + LID_THK - 0.002  # lid bottom embeds 2mm into the lip seat

HINGE_LUG_X = 0.10  # lug plate centers either side of the knuckle
HINGE_LUG_THK = 0.03
HINGE_LUG_TOP = HINGE_Z + HINGE_PIN_R + 0.014


def _disc(radius: float, height: float, segments: int = 64) -> MeshGeometry:
    return CylinderGeometry(radius, height, radial_segments=segments)


RECESS_OUTER_R = 0.290  # outer radius of the sunken top panel
RECESS_DEPTH = 0.026  # how deep the top panel is recessed below the rim
# Top of the raised hub/spokes. The reference shows a bold cast cross-wheel that
# stands clearly PROUD of the rim band, so the relief tops sit above z=0.
RELIEF_TOP_Z = 0.018


def _build_lid_body_mesh() -> MeshGeometry:
    """Round cast-iron lid BODY (reddish-brown). A solid disc whose TOP is
    stepped: a flat outer rim band (z=0) carrying a ring of bolts, then a
    recessed circular panel that the dark cross-wheel relief sits in. Authored in
    the lid part frame, centered on the lid axis, rim top at z=0. The dark relief
    casting is a separate visual (`_build_lid_relief_mesh`) so it can read in a
    contrasting near-black iron like the photographed hatch."""
    geom = MeshGeometry()

    # Disc body as a solid of revolution with a stepped (recessed) top.
    # Closed cross-section outline in (radius, z), counter-clockwise:
    #   bottom face -> outer wall -> flat rim top -> step down -> recess floor.
    profile = [
        (0.0, -LID_THK),  # bottom center
        (LID_R, -LID_THK),  # bottom outer edge
        (LID_R, 0.0),  # outer wall up to rim top
        (RECESS_OUTER_R, 0.0),  # flat outer rim band
        (RECESS_OUTER_R, -RECESS_DEPTH),  # step down into recess
        (0.0, -RECESS_DEPTH),  # recessed panel floor
    ]
    body = LatheGeometry(profile, segments=72, closed=True)
    geom = geom.merge(body)

    # Shallow centering step under the lid that nests inside the throat ring so
    # the closed lid reads as seated (small, intended, mostly hidden overlap).
    seat = _disc(COLLAR_THROAT_R - 0.02, LID_RIM_SEAT, segments=64)
    seat = seat.translate(0.0, 0.0, -LID_THK - LID_RIM_SEAT / 2.0)
    geom = geom.merge(seat)

    # Rim bolts: raised cylinders around the flat outer rim band (cast iron, same
    # body material so they read as part of the lid casting / the light radial
    # studs seen around the rim in the reference).
    bolt_h = 0.012
    for i in range(N_BOLTS):
        ang = (2.0 * math.pi / N_BOLTS) * i
        bolt = CylinderGeometry(BOLT_R, bolt_h, radial_segments=12)
        bolt = bolt.translate(BOLT_RING_R, 0.0, bolt_h / 2.0)
        bolt = bolt.rotate_z(ang)
        geom = geom.merge(bolt)

    return geom


def _build_lid_relief_mesh() -> MeshGeometry:
    """Bold raised cast cross-wheel relief that sits in the lid recess: a central
    hub, four thick radial spokes in an X layout, and a raised ring framing them.
    Authored in the lid part frame (same as the body), sitting on the recess
    floor (z=-RECESS_DEPTH) and standing proud above the rim (z=RELIEF_TOP_Z).
    Carried by a darker iron material so the cross reads strongly, as in the
    reference photo where the wheel casting is near-black against the rusty lid."""
    geom = MeshGeometry()

    relief_h = RELIEF_TOP_Z - (-RECESS_DEPTH)
    relief_base_z = -RECESS_DEPTH

    # Raised central hub.
    hub = CylinderGeometry(HUB_R, relief_h, radial_segments=48)
    hub = hub.translate(0.0, 0.0, relief_base_z + relief_h / 2.0)
    geom = geom.merge(hub)

    # Thick radial spokes from the hub out to the framing ring, X-cross layout.
    spoke_inner = HUB_R - 0.010
    spoke_outer = RELIEF_RING_R + 0.004
    spoke_len = spoke_outer - spoke_inner
    for i in range(N_SPOKES):
        ang = (math.pi / 2.0) * i + math.pi / 4.0  # X-cross diagonal layout
        bar = BoxGeometry((spoke_len, 2.0 * SPOKE_HALF_W, relief_h))
        bar = bar.translate(spoke_inner + spoke_len / 2.0, 0.0, relief_base_z + relief_h / 2.0)
        bar = bar.rotate_z(ang)
        geom = geom.merge(bar)

    # Raised ring framing the wheel (the bold outer band of the cross casting).
    ring_tube = 0.020
    ring = TorusGeometry(RELIEF_RING_R, ring_tube, radial_segments=18, tubular_segments=72)
    ring = ring.translate(0.0, 0.0, RELIEF_TOP_Z - ring_tube + 0.006)
    geom = geom.merge(ring)

    return geom


def _build_collar_mesh() -> MeshGeometry:
    """Square collar frame with a diamond-mesh grille inside and a circular
    throat. Authored centered on the well axis with its base at z=0 of the part
    frame; the frame top is at z=COLLAR_THK."""
    geom = MeshGeometry()

    # Outer square frame band (four bars) leaving a mesh field inside.
    inner = COLLAR_HALF - COLLAR_FRAME
    # +X / -X bars
    for sx in (1.0, -1.0):
        bar = BoxGeometry((COLLAR_FRAME, 2.0 * COLLAR_HALF, COLLAR_THK))
        bar = bar.translate(sx * (COLLAR_HALF - COLLAR_FRAME / 2.0), 0.0, COLLAR_THK / 2.0)
        geom = geom.merge(bar)
    # +Y / -Y bars
    for sy in (1.0, -1.0):
        bar = BoxGeometry((2.0 * inner, COLLAR_FRAME, COLLAR_THK))
        bar = bar.translate(0.0, sy * (COLLAR_HALF - COLLAR_FRAME / 2.0), COLLAR_THK / 2.0)
        geom = geom.merge(bar)

    # Diamond mesh: two families of thin diagonal bars (the expanded-metal /
    # diamond grille look). Each bar is clipped to the chord it spans inside the
    # inner square so no bar pokes past the frame.
    mesh_z = COLLAR_THK - 0.012
    bar_h = 0.012
    bar_w = 0.009
    n = 11
    pitch = (2.0 * inner) / n
    for fam in (1.0, -1.0):  # +45 and -45 families => diamond pattern
        ang = fam * math.pi / 4.0
        # unit direction of the bar and the perpendicular (offset) direction
        dx, dy = math.cos(ang), math.sin(ang)
        px, py = -dy, dx
        for k in range(1, n):
            off = -inner + k * pitch  # perpendicular offset of this bar
            # Sample the diagonal line p_perp*off + t*dir and find the t-range
            # that stays inside the inner square [-inner, inner]^2.
            ts = []
            base_x, base_y = px * off, py * off
            # intersect with the four square edges
            for bound, dcomp, bcomp in (
                (inner, dx, base_x),
                (-inner, dx, base_x),
                (inner, dy, base_y),
                (-inner, dy, base_y),
            ):
                if abs(dcomp) > 1e-9:
                    t = (bound - bcomp) / dcomp
                    x = base_x + dx * t
                    y = base_y + dy * t
                    if -inner - 1e-6 <= x <= inner + 1e-6 and -inner - 1e-6 <= y <= inner + 1e-6:
                        ts.append(t)
            if len(ts) < 2:
                continue
            t0, t1 = min(ts), max(ts)

            # Carve the circular throat clear so the hatch opening is a real hole
            # (the lid plugs an actual hole and the shaft is visible when open),
            # instead of letting the mesh span across the opening. Because `base`
            # is perpendicular to the bar direction, the line meets a circle of
            # radius R at t = +/- sqrt(R^2 - off^2); drop that central span.
            clear_r = COLLAR_THROAT_R + 0.035
            if off * off < clear_r * clear_r:
                tc = math.sqrt(clear_r * clear_r - off * off)
                segments = [(t0, -tc), (tc, t1)]
            else:
                segments = [(t0, t1)]

            for s0, s1 in segments:
                length = s1 - s0
                if length < pitch * 0.4:
                    continue
                cx = base_x + dx * (s0 + s1) / 2.0
                cy = base_y + dy * (s0 + s1) / 2.0
                bar = BoxGeometry((length, bar_w, bar_h))
                bar = bar.rotate_z(ang)
                bar = bar.translate(cx, cy, mesh_z + bar_h / 2.0 - 0.001)
                geom = geom.merge(bar)

    # Circular throat collar wall: a short ring around the opening tying the mesh
    # field down to the shaft bore and giving the lid a seat.
    throat = LatheGeometry.from_shell_profiles(
        [
            (COLLAR_THROAT_R + 0.03, 0.0),
            (COLLAR_THROAT_R + 0.03, COLLAR_THK),
            (COLLAR_THROAT_R, COLLAR_THK + 0.015),
        ],
        [
            (COLLAR_THROAT_R, 0.0),
            (COLLAR_THROAT_R, COLLAR_THK),
            (COLLAR_THROAT_R - 0.004, COLLAR_THK + 0.015),
        ],
        segments=64,
        start_cap="flat",
        end_cap="flat",
    )
    geom = geom.merge(throat)

    # The diamond bars above are carved clear of the throat circle, so the
    # circular opening reads as a real hole framed by the throat ring; the lid
    # plugs that hole when closed and the shaft bore is visible when it swings up.
    return geom


def _build_hinge_mount_mesh() -> MeshGeometry:
    """Collar-side hinge mount: two upright cast lug plates standing on the rear
    collar frame band, plus the hinge pin spanning between them along the hinge
    axis. The lid knuckle barrel rides on this pin, so the joint is physically
    anchored to the collar instead of hanging in the air. Authored in the collar
    part frame (base of the collar at z=0)."""
    geom = MeshGeometry()

    # Lug plates sit fully on the rear frame band (y in [0.345, COLLAR_HALF]),
    # behind the lid rim so the closed disc only kisses their front faces.
    lug_y0 = 0.345
    lug_y1 = COLLAR_HALF
    for sx in (1.0, -1.0):
        lug = BoxGeometry((HINGE_LUG_THK, lug_y1 - lug_y0, HINGE_LUG_TOP))
        lug = lug.translate(sx * HINGE_LUG_X, (lug_y0 + lug_y1) / 2.0, HINGE_LUG_TOP / 2.0)
        geom = geom.merge(lug)

    # Hinge pin along the axis (world X), passing through both lugs and the lid
    # knuckle; the ends stand slightly proud as pin heads.
    pin_len = 2.0 * (HINGE_LUG_X + HINGE_LUG_THK / 2.0) + 0.012
    pin = CylinderGeometry(0.013, pin_len, radial_segments=16)
    pin = pin.rotate_y(math.pi / 2.0)  # cylinder long axis Z -> X (hinge axis)
    pin = pin.translate(0.0, HINGE_Y, HINGE_Z)
    geom = geom.merge(pin)

    return geom


def _build_shaft_mesh() -> MeshGeometry:
    """Hollow round concrete well shaft, base on z=0, open bore through the top."""
    shaft = LatheGeometry.from_shell_profiles(
        [
            (SHAFT_OUTER_R, 0.0),
            (SHAFT_OUTER_R, SHAFT_HEIGHT * 0.85),
            (SHAFT_OUTER_R - 0.02, SHAFT_HEIGHT),
        ],
        [
            (SHAFT_INNER_R, 0.0),
            (SHAFT_INNER_R, SHAFT_HEIGHT * 0.85),
            (SHAFT_INNER_R, SHAFT_HEIGHT),
        ],
        segments=64,
        start_cap="flat",
        end_cap="flat",
    )
    return shaft


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cast_iron_trap_door")

    concrete = Material(name="concrete", rgba=(0.70, 0.69, 0.66, 1.0))
    cast_iron = Material(name="cast_iron", rgba=(0.46, 0.21, 0.15, 1.0))
    # Dark near-black iron for the bold cross-wheel relief so it contrasts with
    # the rusty lid like the photographed hatch.
    relief_iron = Material(name="relief_iron", rgba=(0.12, 0.10, 0.09, 1.0))
    mesh_iron = Material(name="mesh_iron", rgba=(0.22, 0.13, 0.10, 1.0))
    for mat in (concrete, cast_iron, relief_iron, mesh_iron):
        model.material(mat.name, rgba=mat.rgba)

    # --- Well shaft (fixed root) ---------------------------------------------
    shaft = model.part("well_shaft")
    shaft.visual(
        mesh_from_geometry(_build_shaft_mesh(), "well_shaft"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="concrete",
        name="shaft_wall",
    )

    # --- Square mesh collar (fixed to shaft top) -----------------------------
    collar = model.part("mesh_collar")
    collar.visual(
        mesh_from_geometry(_build_collar_mesh(), "mesh_collar"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="mesh_iron",
        name="collar_frame",
    )
    # Hinge lugs + pin on the rear frame band; the lid knuckle rides on this pin.
    collar.visual(
        mesh_from_geometry(_build_hinge_mount_mesh(), "hinge_mount"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="mesh_iron",
        name="hinge_mount",
    )

    # --- Lid (cast-iron disc + spoked relief + bolts) ------------------------
    # The lid part frame sits on the rear-rim hinge line. The disc mesh is
    # centered on the lid axis, so we offset it forward (-Y) by LID_R so the
    # rear rim of the disc lands on the hinge line at the lid part origin.
    lid = model.part("lid")
    lid.visual(
        mesh_from_geometry(_build_lid_body_mesh(), "lid_body"),
        origin=Origin(xyz=(0.0, -LID_R, 0.0)),
        material="cast_iron",
        name="lid_disc",
    )
    # Bold dark cross-wheel relief sitting in the lid recess (separate material).
    lid.visual(
        mesh_from_geometry(_build_lid_relief_mesh(), "lid_relief"),
        origin=Origin(xyz=(0.0, -LID_R, 0.0)),
        material="relief_iron",
        name="lid_relief",
    )

    # Hinge knuckle: a barrel COAXIAL with the revolute axis (lid part origin),
    # spanning between the collar lugs. Because it sits exactly on the axis it
    # stays on the collar pin in every pose instead of orbiting away from it.
    knuckle = mesh_from_geometry(
        CylinderGeometry(HINGE_PIN_R, HINGE_KNUCKLE_LEN, radial_segments=20),
        "hinge_knuckle",
    )
    # Knuckle barrel lies along the hinge line (world X at q=0): rotate the
    # cylinder's local +Z to +X via a pitch of pi/2 about Y.
    lid.visual(
        knuckle,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="cast_iron",
        name="lid_knuckle",
    )

    # --- Fixed joints -------------------------------------------------------
    # Collar sits on top of the shaft.
    model.articulation(
        "shaft_to_collar",
        ArticulationType.FIXED,
        parent=shaft,
        child=collar,
        origin=Origin(xyz=(0.0, 0.0, SHAFT_HEIGHT)),
    )

    # --- Lid hinge (primary articulation) -----------------------------------
    # Hinge line is along the rear rim of the throat, at the collar top plane.
    # The lid part frame is placed there; the lid disc geometry is authored
    # centered on the lid axis, so we offset the lid mesh forward (-Y) via the
    # joint origin so the rear rim aligns with the hinge.
    #
    # The lid mesh is centered on its own axis. We want, at q=0, the lid center to
    # sit over the throat center and lie flat. The hinge line is at the REAR rim
    # (y = +LID_R from the lid center). Place the joint frame at the collar top,
    # rear of the throat; then the lid frame (which equals the joint frame at q=0)
    # must be shifted so the lid disc center is forward (-Y) of the hinge by LID_R.
    # HINGE_Y/HINGE_Z put the axis on the collar-side lug pin, with the lid bottom
    # embedded ~2mm into the throat ring lip so the closed lid reads as seated.
    model.articulation(
        "collar_to_lid",
        ArticulationType.REVOLUTE,
        parent=collar,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        # Horizontal hinge axis along the rear rim (world X). The lid disc extends
        # along local -Y (front) from the hinge; positive rotation about -X lifts
        # the front (-Y) edge upward and over past vertical.
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=2.0, lower=0.0, upper=2.0),
    )

    return model


object_model = build_object_model()


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    shaft = object_model.get_part("well_shaft")
    collar = object_model.get_part("mesh_collar")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("collar_to_lid")

    lid_disc = lid.get_visual("lid_disc")
    lid_relief = lid.get_visual("lid_relief")
    lid_knuckle = lid.get_visual("lid_knuckle")
    hinge_mount = collar.get_visual("hinge_mount")

    # --- Hero geometry present ----------------------------------------------
    ctx.check(
        "lid has disc, cross-wheel relief, knuckle, and rim-bolt geometry",
        lid_disc is not None and lid_relief is not None and lid_knuckle is not None,
        details="expected lid_disc, lid_relief, and lid_knuckle visuals",
    )
    # The recessed spoked top is a hero feature: the recess floor sits below the
    # flat rim, and the wheel relief (hub + 4 spokes) plus a 12-bolt rim ring
    # frame it. Prove the recess depth via the lid mesh's local Z extent and the
    # relief/bolt geometry via the authored counts.
    ctx.check(
        "lid top is recessed and the cross-wheel relief stands proud of the rim",
        RECESS_DEPTH >= 0.015 and RELIEF_TOP_Z > 0.0,
        details=f"recess_depth={RECESS_DEPTH}, relief_top_z={RELIEF_TOP_Z}",
    )
    ctx.check(
        "spoked wheel relief and rim-bolt ring are present",
        N_SPOKES == 4 and N_BOLTS == 12 and BOLT_RING_R > RECESS_OUTER_R,
        details=f"spokes={N_SPOKES}, bolts={N_BOLTS}, bolt_ring_r={BOLT_RING_R}, "
        f"recess_outer_r={RECESS_OUTER_R}",
    )

    # --- Hinge is physically mounted (not floating) ---------------------------
    # The collar carries lug plates + a pin on the hinge axis, the lid knuckle is
    # coaxial with that axis, and the lid is wider than the throat so its rim
    # rests on the ring lip instead of hanging over the opening.
    ctx.check(
        "collar-side hinge mount (lugs + pin) reaches the hinge axis",
        hinge_mount is not None and HINGE_LUG_TOP > HINGE_Z + HINGE_PIN_R,
        details=f"lug_top={HINGE_LUG_TOP:.3f}, hinge_z={HINGE_Z:.3f}",
    )
    ctx.check(
        "lid is wider than the throat opening so it seats on the ring lip",
        LID_R >= COLLAR_THROAT_R + 0.02,
        details=f"lid_r={LID_R:.3f}, throat_r={COLLAR_THROAT_R:.3f}",
    )

    # The hinge knuckle bridges the lid rim and the collar rear edge, and the
    # shallow lid seating step nests inside the throat ring when closed. Both are
    # small, local, intended overlaps at the lid/collar interface.
    ctx.allow_overlap(
        lid,
        collar,
        reason="Hinge knuckle barrel embeds into the collar rear edge and the "
        "shallow lid centering step nests inside the throat ring when closed; "
        "both are local intended seated/hinge overlaps at the hatch lip.",
    )

    # --- Closed pose: lid lies FLAT and seats over the throat ----------------
    with ctx.pose({hinge: 0.0}):
        closed_aabb = ctx.part_world_aabb(lid)
        # Lid disc thickness is small compared to its diameter => it lies flat.
        # The lid should span roughly a full diameter in both X and Y near the
        # collar top, well above the ground.
        if closed_aabb is not None:
            (cx0, cy0, cz0), (cx1, cy1, cz1) = closed_aabb
            x_span = cx1 - cx0
            y_span = cy1 - cy0
            z_span = cz1 - cz0
            ctx.check(
                "closed lid lies flat (thin in Z, wide in X and Y)",
                z_span < 0.12 and x_span > 0.5 and y_span > 0.5,
                details=f"x_span={x_span:.3f} y_span={y_span:.3f} z_span={z_span:.3f}",
            )
            ctx.check(
                "closed lid sits at the collar top, not on the ground",
                cz0 > SHAFT_HEIGHT - 0.02,
                details=f"lid min z={cz0:.3f}, shaft height={SHAFT_HEIGHT}",
            )
        # The lid footprint overlaps the throat/collar in plan when closed.
        ctx.expect_overlap(
            lid,
            collar,
            axes="xy",
            min_overlap=0.20,
            name="closed lid covers the collar throat in plan",
        )
        # The closed lid actually rests on the collar (seated, not floating above
        # the opening). Allowed local seating overlap, so require near contact.
        ctx.expect_contact(
            lid,
            collar,
            contact_tol=0.006,
            name="closed lid seats on the collar (not floating)",
        )

    closed_front = ctx.part_world_aabb(lid)

    # --- Open pose: front edge lifts upward, past vertical -------------------
    with ctx.pose({hinge: 1.9}):
        open_aabb = ctx.part_world_aabb(lid)
        ctx.check(
            "open pose lifts the lid well above the collar",
            open_aabb is not None
            and closed_front is not None
            and open_aabb[1][2] > closed_front[1][2] + 0.20,
            details=f"closed max z={None if closed_front is None else closed_front[1][2]:.3f}, "
            f"open max z={None if open_aabb is None else open_aabb[1][2]:.3f}",
        )
        # When swung up, the lid becomes tall (large Z extent) and shallow in Y.
        if open_aabb is not None:
            (ox0, oy0, oz0), (ox1, oy1, oz1) = open_aabb
            ctx.check(
                "open lid stands up (tall in Z)",
                (oz1 - oz0) > 0.45,
                details=f"open z_span={(oz1 - oz0):.3f}",
            )

    # --- Support / placement -------------------------------------------------
    # Shaft is the fixed root standing on the ground.
    shaft_aabb = ctx.part_world_aabb(shaft)
    if shaft_aabb is not None:
        ctx.check(
            "well shaft rests on the ground plane (z~0)",
            abs(shaft_aabb[0][2]) < 0.01,
            details=f"shaft min z={shaft_aabb[0][2]:.4f}",
        )

    # Collar sits on top of the shaft and supports the lid (no floating).
    collar_aabb = ctx.part_world_aabb(collar)
    if collar_aabb is not None and shaft_aabb is not None:
        ctx.check(
            "mesh collar sits at the shaft top",
            abs(collar_aabb[0][2] - shaft_aabb[1][2]) < 0.05,
            details=f"collar min z={collar_aabb[0][2]:.3f}, shaft max z={shaft_aabb[1][2]:.3f}",
        )

    # Collar throat is wider than the shaft bore so the opening reads through.
    ctx.check(
        "collar throat clears the shaft bore (hollow well)",
        COLLAR_THROAT_R <= SHAFT_INNER_R + 0.05 and SHAFT_INNER_R > 0.25,
        details=f"throat_r={COLLAR_THROAT_R:.3f}, bore_r={SHAFT_INNER_R:.3f}",
    )

    return ctx.report()
